// SPDX-License-Identifier: AGPL-3.0-only
//! The Go2 default Rerun BLUEPRINT — a designed layout sent once per
//! recording, so the viewer opens on a good default instead of an auto-generated
//! grid of raw views.
//!
//! **Decision**: the default is now Scene-only — a single full-bleed 3D
//! scene (see [`BlueprintPlan::go2_default`]). The old telemetry-plot + status /
//! field-dump panels are DROPPED: on a robot-free desk they were empty (an empty
//! `time_series` plot renders a nonsense 1970-era epoch axis, and the status panel
//! showed nothing). Panels now exist only when there is data — the viewer's
//! `with_auto_views(true)` heuristic auto-spawns the right view (a plot for a scalar
//! stream, a text view for a `TextLog`) when a NON-spatial topic is attached, WITH
//! data (so no 1970 axis), and the [`compose_layout`] compiler builds richer
//! dashboards on demand. `auto_views` on the default is load-bearing (not
//! decorative): it is what keeps a scalar/status attach visible. So nothing logged
//! is invisible.
//!
//! Rerun 0.34's Rust blueprint API (`rerun::blueprint`) assembles a view tree from
//! container + view builders; it does NOT expose a `TextLogView` (only
//! `TextDocumentView`).
//!
//! The blueprint is sent ONCE per process behind a resettable `AtomicBool`
//! guard (a graph may run several sink instances; the layout must not be
//! re-sent per frame or per instance), mirroring
//! [`crate::tf::log_viz_statics_once`].

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;

use rerun::blueprint::{
    Blueprint, BlueprintActivation, ContainerLike, Horizontal, Spatial3DView, TextDocumentView,
    TimeSeriesView, Vertical,
};
use rerun::{RecordingStream, RecordingStreamBuilder, RecordingStreamResult};

// The hand-rolled blueprint emission reaches per-view blueprint
// PROPERTIES (a live-scope trailing time window on time_series views; a solid stage
// background on spatial views) that rerun 0.34.1's high-level `Blueprint` / `View`
// builders CANNOT set — their `add_property` is `pub(crate)` and the view UUIDs are
// generated internally by `Blueprint::send`, so per-view overrides are unreachable
// through the SDK. We therefore replicate its emission (viewport + containers +
// views) directly on a blueprint `RecordingStream`, additionally logging the
// `Background` / `VisibleTimeRanges` / `TimeAxis` archetypes onto each view's PROPERTY
// path — the SAME path the viewer reads them from: `re_viewport_blueprint`'s
// `entity_path_for_view_property` resolves to `view/<uuid>/<ArchetypeShortName>`,
// and the read seams are `ViewBlueprint::query_range` (the query window, matched by
// timeline name), `re_view_time_series`'s `TimeAxis:view_range` read (the DISPLAY
// x-axis extents — the empty-epoch-axis fix), and
// `re_view_spatial::configure_background` (the background). See [`build_blueprint_msgs`].
use rerun::external::re_log_types::{BlueprintActivationCommand, LogMsg};
use rerun::external::re_sdk_types::blueprint::archetypes::{
    Background, ContainerBlueprint, TimeAxis, TimePanelBlueprint, ViewBlueprint, ViewContents,
    ViewportBlueprint, VisibleTimeRanges,
};
use rerun::external::re_sdk_types::blueprint::components::{
    AutoViews, BackgroundKind, ColumnShare, ContainerKind as RrContainerKind, GridColumns,
    IncludedContent, QueryExpression, RootContainer, RowShare, TimelineName as RrTimelineName,
    ViewClass,
};
use rerun::external::re_sdk_types::components::Name;
use rerun::external::re_sdk_types::datatypes::{
    Bool, Float32, TimeInt, TimeRange, TimeRangeBoundary, UInt32, Uuid as RrUuid, VisibleTimeRange,
};

use crate::archetype::ROBOT_TIME;
use crate::representation::{resolve_render_plan, Representation};
use crate::sink::{ArchetypeKind, RenderProof};
use crate::skeleton::ROBOT_ROOT;

/// The viz root all entities live under (see [`crate::tf::WORLD_ROOT`]). The
/// Go2 default; overridable via [`BlueprintConfig`].
const WORLD_ORIGIN: &str = "/world";

/// Constant group 4 (blueprint layout): the dashboard layout parameters.
/// `Default` reproduces the exact Go2 dashboard (a hero 3D view ~3× the width
/// beside a plots-over-status sidebar, rooted at `/world`, with auto-views on)
/// so the demo stays byte-identical; the `cerulion viz` verb (through
/// the runtime layout below) can lay out a per-robot dashboard.
#[derive(Debug, Clone, PartialEq)]
pub struct BlueprintConfig {
    /// The entity-path origin all three views are rooted at.
    pub world_origin: String,
    /// The 3D hero view title.
    pub scene_name: String,
    /// The time-series (telemetry) view title.
    pub plots_name: String,
    /// The text-document (status / field-dump) view title.
    pub status_name: String,
    /// The 3D hero view's column share (relative width).
    pub scene_share: f32,
    /// The sidebar column's share (relative width).
    pub sidebar_share: f32,
    /// Whether the viewer additionally auto-creates views for archetypes the
    /// three explicit views do not cover (e.g. a `TextLog` stream).
    pub auto_views: bool,
}

impl Default for BlueprintConfig {
    fn default() -> Self {
        Self {
            world_origin: WORLD_ORIGIN.to_string(),
            scene_name: "Scene".to_string(),
            plots_name: "Telemetry".to_string(),
            status_name: "Status & Field Dumps".to_string(),
            scene_share: 3.0,
            sidebar_share: 1.0,
            auto_views: true,
        }
    }
}

/// Build the LEGACY 3-panel SDK-builder blueprint (a hero `Spatial3DView` beside a
/// `Vertical[ TimeSeriesView / TextDocumentView ]` sidebar).
///
/// **NOT the production default.** The runtime send path
/// ([`send_blueprint_once`] → `send_plan`) emits [`BlueprintPlan::go2_default`] — by
/// design a Scene-only plan (the empty telemetry/status panels are
/// dropped). This SDK-`Blueprint` builder is retained only to exercise the
/// [`BlueprintConfig`] plumbing (`config_test`); nothing in production sends it. Pure
/// (no I/O).
pub fn go2_blueprint() -> Blueprint {
    blueprint_from(&BlueprintConfig::default())
}

/// Build a 3-panel SDK-`Blueprint` from a [`BlueprintConfig`] (the [`go2_blueprint`]
/// legacy builder generalized). NOT the production default — see [`go2_blueprint`].
pub fn blueprint_from(cfg: &BlueprintConfig) -> Blueprint {
    let scene = Spatial3DView::new(cfg.scene_name.as_str()).with_origin(cfg.world_origin.as_str());
    let plots = TimeSeriesView::new(cfg.plots_name.as_str()).with_origin(cfg.world_origin.as_str());
    let status =
        TextDocumentView::new(cfg.status_name.as_str()).with_origin(cfg.world_origin.as_str());

    // Right-hand column: plots over status.
    let sidebar = Vertical::new([ContainerLike::from(plots), ContainerLike::from(status)]);
    // Root: the hero 3D view (~scene_share width) beside the sidebar (~sidebar_share).
    let root = Horizontal::new([ContainerLike::from(scene), ContainerLike::from(sidebar)])
        .with_column_shares([cfg.scene_share, cfg.sidebar_share]);

    Blueprint::new(root).with_auto_views(cfg.auto_views)
}

/// Structurally-once guard for the blueprint send. An `AtomicBool` (not
/// `std::sync::Once`) so [`rearm_blueprint`] can re-arm it across test
/// runs / re-inits — the same reasoning as [`crate::tf::log_viz_statics_once`].
static BLUEPRINT_SENT: AtomicBool = AtomicBool::new(false);

/// Send the dashboard blueprint to `rec` exactly ONCE per process (per
/// [`rearm_blueprint`] epoch). `swap(true)` returns `false` to exactly
/// one caller even if several sink instances race their first tick — the same
/// guarantee a `Once` gives, but resettable for tests. Best-effort: a send
/// error warns and is swallowed — a missing blueprint never fails the graph,
/// and the recording still renders under the viewer's default auto-layout.
///
/// If a RUNTIME layout has been installed (via [`apply_runtime_blueprint`] — the
/// `set_blueprint` verb), THAT layout is sent here instead of the Go2
/// default. So after a live [`crate::stream::reconnect`] re-arms this guard, the
/// bounced (empty) server re-receives the AGENT'S current layout, not a revert to
/// the default (the reset/default behavior is documented on the verb).
pub fn send_blueprint_once(rec: &RecordingStream) {
    if BLUEPRINT_SENT.swap(true, Ordering::SeqCst) {
        return;
    }
    // make_active + make_default: open the viewer ON this layout and remember
    // it as the default for the application.
    let activation = BlueprintActivation {
        make_active: true,
        make_default: true,
    };
    // A runtime layout SUPERSEDES the Go2 default (so a reconnect re-applies the
    // agent's chosen dashboard); with none set, send the Go2 default.
    let runtime = current_runtime_blueprint_plan();
    let result = match &runtime {
        Some(plan) => {
            // Record-only reconnect-reapply grounding check (the plan
            // may have lost all its topics to detaches since it was set).
            warn_if_plan_grounds_nothing(plan);
            // Send via the hand-rolled emitter so a compose plan's per-view
            // trailing windows + backgrounds re-apply on reconnect (they travel in the
            // remembered plan's `decorate` flag). This is the boot/reconnect
            // send (the first per epoch — see `ensure_setup`), so it PINS the default
            // timeline (so a fresh/bounced viewer opens on `log_time`, not robot uptime).
            send_plan(rec, plan, activation, true)
        }
        None => send_plan(rec, &BlueprintPlan::go2_default(), activation, true),
    };
    match result {
        Ok(()) => tracing::info!(
            runtime = runtime.is_some(),
            "Rerun viz: sent the dashboard blueprint"
        ),
        Err(e) => tracing::warn!(
            error = %e,
            "Rerun viz: blueprint send failed — falling back to the viewer's auto-layout"
        ),
    }
}

/// Process-static slot holding the current RUNTIME layout (the last blueprint set
/// via the `set_blueprint` verb), or `None` when the Go2 default is in
/// effect. Read by [`send_blueprint_once`] (so a reconnect re-applies it) and
/// written by [`apply_runtime_blueprint`]. Both run on the viz worker thread in
/// production, but the `Mutex` keeps it sound regardless.
static RUNTIME_BLUEPRINT: Mutex<Option<BlueprintPlan>> = Mutex::new(None);

/// A clone of the current runtime layout plan, or `None` when the Go2 default is
/// in effect. The queryable seam (Principle #3) + the test observation point for
/// the reconnect-reapply contract.
pub fn current_runtime_blueprint_plan() -> Option<BlueprintPlan> {
    RUNTIME_BLUEPRINT.lock().unwrap().clone()
}

/// Install `plan` as the current runtime layout AND send it immediately,
/// SUPERSEDING whatever blueprint is active (rerun's `send` with `make_active` +
/// `make_default` replaces the active blueprint). Remembering it makes a later
/// [`crate::stream::reconnect`] re-apply THIS layout (via [`send_blueprint_once`])
/// rather than reverting to the Go2 default. Runs on the viz worker thread (the
/// one thread that owns + may block on the `RecordingStream`); a send
/// failure warns and keeps the previous layout — never a panic.
pub fn apply_runtime_blueprint(rec: &RecordingStream, plan: BlueprintPlan) {
    *RUNTIME_BLUEPRINT.lock().unwrap() = Some(plan.clone());
    // Record-only grounding check at the apply/reapply site (never a
    // refusal — the daemon's set_blueprint verb already runs the hard guardrails;
    // this catches a stored plan whose topics detached before a direct re-apply).
    warn_if_plan_grounds_nothing(&plan);
    let activation = BlueprintActivation {
        make_active: true,
        make_default: true,
    };
    // A runtime RE-APPLY does NOT re-pin the default timeline (the boot
    // send already pinned it), so a topic toggle never snaps a user's manual timeline
    // choice back to `log_time`.
    match send_plan(rec, &plan, activation, false) {
        Ok(()) => tracing::info!(
            views = plan.view_count(),
            "Rerun viz: applied a runtime blueprint (layout)"
        ),
        Err(e) => tracing::warn!(
            error = %e,
            "Rerun viz: runtime blueprint send failed — keeping the previous layout"
        ),
    }
}

/// Clear the runtime layout back to "unset" (the Go2 default takes over on the
/// next [`send_blueprint_once`]). Test hygiene ONLY — production resets to the
/// default by installing [`BlueprintPlan::go2_default`] via
/// [`apply_runtime_blueprint`] (which sends it immediately). Wired into
/// [`crate::stream::reset_for_test`] so a later same-process test starts on a
/// clean slate.
pub fn clear_runtime_blueprint() {
    *RUNTIME_BLUEPRINT.lock().unwrap() = None;
    ATTACHED_ENTITIES_SNAPSHOT.lock().unwrap().clear();
}

/// A process-static snapshot of the daemon's currently-attached
/// render entity paths. The daemon refreshes it on each attach/detach (low-freq
/// control ops, never the poll hot path); the worker-side reapply sites
/// ([`apply_runtime_blueprint`] / [`send_blueprint_once`]) read it to emit a
/// record-only warn when the layout being (re-)sent grounds NONE of them (a
/// reconnect after every topic detached would silently re-apply an empty
/// dashboard). Production runs ONE daemon per process, so the single static is
/// correct; the warn is record-only, so a same-process test sharing it is benign.
static ATTACHED_ENTITIES_SNAPSHOT: Mutex<Vec<String>> = Mutex::new(Vec::new());

/// Refresh the attached-entities snapshot the reapply sites warn against (called by
/// the daemon on attach/detach). See the `ATTACHED_ENTITIES_SNAPSHOT` static.
pub fn set_attached_entities_snapshot(entities: Vec<String>) {
    *ATTACHED_ENTITIES_SNAPSHOT.lock().unwrap() = entities;
}

/// Record-only: warn (never refuse) when `plan` grounds NONE of the currently-
/// attached entities — the reconnect-reapply safety net (reconnect must not brick,
/// so this only OBSERVES). No-op when nothing is attached (a lay-before-attach or
/// pre-attach reconnect can't be proven empty).
fn warn_if_plan_grounds_nothing(plan: &BlueprintPlan) {
    let entities = ATTACHED_ENTITIES_SNAPSHOT.lock().unwrap().clone();
    if !entities.is_empty() && !plan_grounds_any_entity(plan, &entities) {
        tracing::warn!(
            views = plan.view_count(),
            attached = entities.len(),
            "Rerun viz: the runtime blueprint grounds NONE of the attached topics — the dashboard \
             will render empty (topics may have detached since it was set); re-issue set_blueprint \
             with current entity paths, or reset to the default"
        );
    }
}

/// Re-arm the blueprint-send guard so the dashboard blueprint re-sends on the
/// next sink fire. Callers: [`crate::stream::rearm_after_reconnect`] (so
/// a bounced server re-receives the layout) and [`crate::stream::reset_for_test`]
/// (test hygiene — a later same-process graph test re-sends the blueprint on its
/// first sink fire).
pub fn rearm_blueprint() {
    BLUEPRINT_SENT.store(false, Ordering::SeqCst);
}

// ────────────────────────────────────────────────────────────────────────────
// Runtime layout: the rerun-agnostic BLUEPRINT PLAN + its translation.
//
// The `set_blueprint` verb carries a rerun-AGNOSTIC layout spec (the wire types
// live in `cerulion_vizd::protocol`, rerun-free). The daemon validates that spec
// into the inspectable [`BlueprintPlan`] here (a pure, `PartialEq` tree), then
// [`build_blueprint_msgs`] assembles the blueprint `LogMsg`s (hand-rolled, so the
// per-view properties the SDK cannot set are reachable). The split is deliberate:
// everything up to the plan is unit-testable against hand oracles; the emitted
// chunks are decodable ([`blueprint_property_paths`] / [`blueprint_decorations`]).
// ────────────────────────────────────────────────────────────────────────────

/// A rerun VIEW kind the layout verb can place — the closed set of `rerun` 0.34
/// SDK views that a Cerulion sink archetype actually renders into (Points3D /
/// Transforms / Odometry / skeleton → 3D; Image → 2D; Scalars / telemetry →
/// time-series; TextLog / field-dump → text document). rerun 0.34 ALSO ships
/// `MapView` + `GraphView`, deliberately NOT exposed: no Cerulion archetype
/// populates them, so offering them would only let an agent build empty panes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViewKind {
    /// A 3D spatial scene (`rerun::blueprint::Spatial3DView`).
    Spatial3d,
    /// A 2D spatial scene / image view (`rerun::blueprint::Spatial2DView`).
    Spatial2d,
    /// A scalar time-series plot (`rerun::blueprint::TimeSeriesView`).
    TimeSeries,
    /// A text-document / field-dump panel (`rerun::blueprint::TextDocumentView`).
    TextDocument,
}

impl ViewKind {
    /// The wire kind strings, in the order the error message lists them.
    pub const WIRE_KINDS: [&'static str; 4] =
        ["spatial3d", "spatial2d", "time_series", "text_document"];

    /// The stable wire name (matches the accepted `kind` string).
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Spatial3d => "spatial3d",
            Self::Spatial2d => "spatial2d",
            Self::TimeSeries => "time_series",
            Self::TextDocument => "text_document",
        }
    }

    /// Resolve a wire `kind` string, or a loud [`LayoutError::UnknownViewKind`]
    /// naming the supported set (never a silent drop).
    pub fn from_wire(kind: &str) -> Result<Self, LayoutError> {
        match kind {
            "spatial3d" => Ok(Self::Spatial3d),
            "spatial2d" => Ok(Self::Spatial2d),
            "time_series" => Ok(Self::TimeSeries),
            "text_document" => Ok(Self::TextDocument),
            other => Err(LayoutError::UnknownViewKind(other.to_string())),
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// The DETERMINISTIC archetype → layout mapping. The Studio agent drove
// broken layouts because it hand-authored rerun blueprint internals (guessing
// entity paths, letting `auto_views` explode a Twist's 6 scalar components into
// 6 panels). These two PURE functions formalize "given a topic's resolved sink
// archetype, which rerun view(s) render it AND what rerun families it produces"
// so the daemon can hand the agent the answer instead of letting it guess. Both
// are exhaustive over [`ArchetypeKind`] with NO wildcard arm — a future archetype
// variant is a compile error until it is placed here on purpose.
// ────────────────────────────────────────────────────────────────────────────

/// The rerun view kind(s) an [`ArchetypeKind`] renders into — the decision
/// table. Most archetypes resolve to ONE view; the pose-plus-scalars families
/// ([`ArchetypeKind::Odometry`] / [`ArchetypeKind::Imu`]) return BOTH a 3D scene
/// (the moving pose) AND a time-series plot (the twist / accel-gyro scalars),
/// matching their two-builder render arms (a geometry builder plus a curated
/// series — `log_odometry_pose_in_frame` / `log_imu_geometry_in_frame` beside
/// `odometry_twist_scalars` / `imu_scalars`).
///
/// `LaserScan → spatial3d` is a rule AT THE MAPPING LEVEL: a LaserScan
/// projects to a [`rerun::Points3D`] scan ring, which renders EMPTY in a 2D view —
/// it belongs in the 3D scene, never a `spatial2d`.
///
/// **Nothing may render nothing.** Seven render arms can DEGRADE at
/// frame time to the AnyValues field dump (an image encoding this viewer does not decode, a
/// grid/box/element array that will not extract, an inert skeleton, a video rescan
/// disagreement — see [`ArchetypeKind::can_degrade_to_dump`], which is the single
/// source of truth). A layout is derived ONCE from the CLASSIFIED archetype, so
/// with spatial-only views that dump landed in an entity nothing displays: an
/// empty pane, no diagnostic, on exactly the never-seen-robot path this ladder
/// exists for. Every degradable kind therefore carries a `text_document` view IN
/// ADDITION to its primary views, and the `VIEWS_MATCH_TEXT_DOCUMENT_CONTRACT`
/// const assertion below makes the agreement a COMPILE-TIME fact in BOTH
/// directions — a degradable kind cannot lose its dump view, and a kind that
/// never dumps cannot be handed a permanently empty pane.
///
/// The cost is deliberate and bounded: a healthy map / marker topic carries one
/// extra (empty until it degrades) status pane. The alternative — adding the view
/// when a degradation is OBSERVED — would make the layout a function of frame
/// content, which is the frame-dependent-layout failure mode (a once-resolved layout
/// must not depend on which frame arrived first).
///
/// The video kind refuses that cost, and the refusal is generalized to
/// every kind that can PROVE it is rendering, but both live in
/// [`views_for_render`] rather than here: an operator reads the empty tile beside
/// a live `/go2/camera/h264` as the topic being duplicated. This table is about
/// the KIND and stays true of it — "this topic is provably rendering" is a
/// TOPIC-level refinement, made on live evidence
/// ([`AttachedRender::video_renditions`] for video, [`RenderProof`] for the rest).
/// See `dump_companion_is_provably_unused` (beside [`views_for_render`]) for the
/// conjuncts, and [`RenderProof`] for why the standing "layout must not be a
/// function of which frame arrived first" rule survives it: both flags are
/// STICKY, so the pane moves at most twice per attach and never flickers.
///
/// Pure (allocation-free `&'static` slices), `const`, and oracle-tested, so the
/// layout compiler's placement decisions are pinned without touching rerun.
pub const fn views_for_archetype(kind: ArchetypeKind) -> &'static [ViewKind] {
    use ArchetypeKind as A;
    use ViewKind as V;
    match kind {
        // 3D scene: clouds, the projected LaserScan ring (NOT spatial2d),
        // rigid transforms (single Transform3D, the per-child TF tree, single
        // points), and the URDF skeleton.
        // An oriented detection box (`rerun::Boxes3D`) is 3D geometry —
        // it belongs beside the cloud in the Scene, never in a 2D pane.
        // A decoded element array's polyline / point set / detection set
        // is 3D geometry for the same reason a single box is — it belongs beside
        // the cloud in the Scene, never in a 2D pane (the lesson).
        A::Points3D | A::LaserScan | A::Transform3D | A::Transforms | A::Point3D => &[V::Spatial3d],
        // The 3D geometry kinds that CAN degrade to the field dump, so
        // they carry the status pane too: a box / grid-less detection, an absent or
        // undecodable element array, and the inert skeleton — which is now
        // EVERY skeleton (`install_skeleton` has no caller), so this arm is what
        // turns a `/lowstate`-class topic from a blank pane into a live field dump.
        // A MarkerArray's markers are 3D primitives (boxes, ellipsoids,
        // cylinders, arrows, polylines, meshes) on child entities of the topic
        // entity. A spatial3d view rooted at an origin includes its SUBTREE, so
        // the one view covers every marker with no extra blueprint work.
        A::Skeleton | A::Boxes3D | A::Path3D | A::PoseArray3D | A::MarkerArray => {
            &[V::Spatial3d, V::TextDocument]
        }
        // Pose AND scalars: the moving pose lands in the 3D scene, the twist /
        // accel-gyro series in a plot (both builders fire — see
        // `log_odometry_pose_in_frame` and the curated twist series beside it).
        // An inferred spatial shape that ALSO carries sibling
        // telemetry joins them — its render arm logs the transform / point AND
        // those series, so without the plot view the numbers would have no home
        // (the silent-drop regression). ONE `time_series` view per topic, per the
        // standing Twist-6-panel-explosion decision.
        A::Odometry | A::Imu | A::Transform3DWithScalars | A::Point3DWithScalars => {
            &[V::Spatial3d, V::TimeSeries]
        }
        // A raw or JPEG image → a 2D image view. Note that an occupancy grid is
        // rendered as a grayscale `rerun::Image`, so it lands in the SAME 2D
        // family (the map is a picture, not 3D geometry).
        // A decoded H.264 stream is a picture too — since
        // THIS DESK decodes each access unit into a frame that renders
        // exactly where a JPEG would, in the same 2D family.
        // All three can degrade (an undecoded raw encoding, a grid whose
        // cell buffer is short, a video rescan disagreement), so each also carries
        // the status pane its dump lands in. The desk-side decode does NOT narrow that for
        // video: the `VideoRescanDisagreed` fallback still routes to the dump, and
        // the desk-side decode adds no dump path of its own (an access unit the
        // decoder refuses renders NOTHING and is reported through counters plus a
        // flood-latched log — see `crate::video_decode`).
        A::Image | A::OccupancyGrid | A::VideoStream => &[V::Spatial2d, V::TextDocument],
        // Numeric telemetry → ONE time-series view (the Twist-6-panel-explosion
        // fix: one plot per topic, not one panel per scalar component).
        A::Scalars | A::SportModeState => &[V::TimeSeries],
        // Numeric telemetry that ALSO carries text renders the plots AND
        // the field dump, so it needs BOTH views — the `Odometry`/`Imu`
        // dual-view precedent. `[TimeSeries]` alone is what silently dropped a
        // vendor status message's payload while plotting its envelope.
        A::ScalarsWithText => &[V::TimeSeries, V::TextDocument],
        // A single string line or the field-dump fallback → a text document.
        // Rerun 0.34's blueprint builder API ships NO `TextLogView`
        // (only `TextDocumentView`, which displays only `TextDocument` entities),
        // so a TextLog topic's `text_document` view would render EMPTY on the
        // `TextLog` component alone. The sink's TextLog arm therefore ALSO logs a
        // rolling-latest `TextDocument` mirror at `<entity>/text` (see
        // [`crate::archetype::log_text`]), making this mapping genuinely
        // displayable — NOT a provably-empty view.
        A::TextLog | A::AnyValues => &[V::TextDocument],
    }
}

/// Whether [`views_for_archetype`] gives `kind` a `text_document` view — a `const`
/// slice scan so the contract below can be checked at COMPILE time.
const fn views_carry_text_document(kind: ArchetypeKind) -> bool {
    let views = views_for_archetype(kind);
    let mut i = 0;
    while i < views.len() {
        if matches!(views[i], ViewKind::TextDocument) {
            return true;
        }
        i += 1;
    }
    false
}

/// The `text_document` view is present EXACTLY when the render path can
/// put a [`rerun::TextDocument`] at the topic entity
/// ([`ArchetypeKind::renders_text_document`]) — checked at COMPILE time over the
/// closed [`ArchetypeKind::ALL`] set, so the two halves cannot drift.
///
/// An IFF, because both directions are real defects:
///
/// - MISSING the view on a degradable kind is the bug itself — the frame
///   logs a dump into an entity whose views are spatial-only and the user sees an
///   empty pane with no diagnostic;
/// - an EXTRA view on a kind that never dumps hands every one of its topics a
///   permanently empty status pane, which is the layout-cleanliness half.
///
/// Deliberately a `const` assertion rather than a `#[cfg(test)]` oracle: the
/// enforcement then lives in the LIBRARY build (the `declare_archetype_kinds!`
/// precedent), so the mistake cannot be made even in a branch whose tests are not
/// run. The test suite still pins the two lists against HAND oracles — this
/// assertion proves they AGREE, the oracles prove they are RIGHT.
const VIEWS_MATCH_TEXT_DOCUMENT_CONTRACT: () = {
    let mut i = 0;
    while i < ArchetypeKind::ALL.len() {
        let kind = ArchetypeKind::ALL[i];
        assert!(
            views_carry_text_document(kind) == kind.renders_text_document(),
            "an archetype whose render path can log a TextDocument (a degradation \
             via ArchetypeKind::can_degrade_to_dump, or a primary text render) must have a \
             text_document view in views_for_archetype, and one that cannot must not — \
             otherwise the dump renders nowhere, or the topic gets a permanently empty pane"
        );
        i += 1;
    }
};

/// Force the const-evaluation of [`VIEWS_MATCH_TEXT_DOCUMENT_CONTRACT`] (an unused
/// `const` item is still evaluated, but naming it here keeps the link explicit and
/// survives a future `dead_code` sweep).
const _: () = VIEWS_MATCH_TEXT_DOCUMENT_CONTRACT;

/// The rerun archetype/component families an [`ArchetypeKind`] produces on the
/// wire — grounded in what each render arm actually logs (see
/// [`crate::archetype`] / [`crate::sink`]). This tells the agent WHAT a topic's
/// view will contain (e.g. an [`ArchetypeKind::Odometry`] topic logs BOTH a
/// `Transform3D` and `Scalars`), so it can reason about a topic without decoding
/// a frame. It is intentionally a function of the ARCHETYPE, not of the concrete
/// message fields — a per-field scalar list would need a decoded frame and is a
/// data-plane concern, not this deterministic mapping.
///
/// [`ArchetypeKind::Image`] lists BOTH `EncodedImage` (a JPEG
/// `CompressedImage`) and `Image` (a raw rgb8/bgr8/mono8 buffer) because the
/// archetype covers both and the concrete one is fixed by the topic's schema, not
/// derivable from the archetype alone.
///
/// Pure (allocation-free `&'static` slices) and oracle-tested.
pub fn archetype_components(kind: ArchetypeKind) -> &'static [&'static str] {
    use ArchetypeKind as A;
    match kind {
        // An unordered element array (`PoseArray` / `GridCells`) logs ONE
        // multi-instance Points3D — the same family, N positions instead of 1.
        A::Points3D | A::Point3D | A::LaserScan | A::PoseArray3D => &["Points3D"],
        A::Image => &["EncodedImage", "Image"],
        // An `Image` per decoded picture, logged at each
        // rendition's `viz-video/<WxH>` child rather than at the topic entity — a
        // `spatial2d` view rooted at the topic includes the subtree, so every
        // rendition shows. The family is `Image` and NOT `VideoStream` because
        // the DECODE was moved to this desk: rerun's viewer-side H.264 backend
        // spawns an `ffmpeg` whose frame threading cost a measured 567 ms of lag,
        // so what now goes on the wire is a decoded picture, replaced on arrival.
        A::VideoStream => &["Image"],
        // One oriented box instance per frame, OR N instances
        // for a detection SET — the same family either way.
        A::Boxes3D => &["Boxes3D"],
        // An ordered element array logs the polyline AND its vertices (see
        // `crate::archetype::log_path3d` — the vertices ride the
        // `<entity>/viz-vertices` child, so a spatial3d view rooted at the topic shows
        // both). Understating this as `["LineStrips3D"]` would tell the agent the
        // waypoints are not rendered.
        A::Path3D => &["LineStrips3D", "Points3D"],
        // The occupancy grid is logged as a grayscale raw `Image` (never
        // an `EncodedImage` — nothing is compressed).
        A::OccupancyGrid => &["Image"],
        // A plain TF topic (single Transform3D or the per-child TF tree) logs
        // ONLY Transform3D — see `log_pose` / `crate::tf::log_transforms`.
        A::Transforms | A::Transform3D => &["Transform3D"],
        // The URDF skeleton render arm (see `crate::skeleton`) logs FOUR families,
        // NOT one: per-joint origin/angle `Transform3D`s, joint marker `Points3D`,
        // the bone `LineStrips3D`, and per-link visual mesh `Asset3D`s. Understating
        // this as `["Transform3D"]` (the earlier shared-arm bug) told the agent
        // a skeleton topic renders only transforms.
        A::Skeleton => &["Transform3D", "Points3D", "LineStrips3D", "Asset3D"],
        A::Scalars | A::SportModeState => &["Scalars"],
        // The plots PLUS the structured dump its text lands in — both
        // logged on every frame, so understating this as `["Scalars"]` would tell
        // the agent the text half is not rendered.
        A::ScalarsWithText => &["Scalars", "TextDocument"],
        // Pose + scalars: a moving transform AND the plotted series.
        // An inferred transform-shape carrying sibling telemetry
        // logs exactly the same two families.
        A::Imu | A::Odometry | A::Transform3DWithScalars => &["Transform3D", "Scalars"],
        // The point twin: a single-point `Points3D` PLUS its sibling series.
        A::Point3DWithScalars => &["Points3D", "Scalars"],
        // The TextLog arm logs a `TextLog` line AND a rolling-latest
        // `TextDocument` mirror (see [`crate::archetype::log_text`]), so the
        // `text_document` view [`views_for_archetype`] places is genuinely
        // displayable.
        A::TextLog => &["TextLog", "TextDocument"],
        // A MarkerArray's render arm is a KIND SWITCH, so a topic's
        // markers can produce any of these — every marker logs a `Transform3D`
        // (its pose) plus exactly one geometry family, and `Clear` is logged on
        // DELETE / DELETEALL. Listing the union is the correct answer for an
        // archetype-level (not frame-level) table: which subset a given topic
        // uses depends on its data, which this mapping deliberately does not read.
        A::MarkerArray => &[
            "Transform3D",
            "Arrows3D",
            "Boxes3D",
            "Ellipsoids3D",
            "Cylinders3D",
            "LineStrips3D",
            "Points3D",
            "Mesh3D",
            "Clear",
        ],
        A::AnyValues => &["TextDocument"],
    }
}

/// A rerun CONTAINER kind the layout verb can arrange children into — the full
/// `rerun` 0.34 SDK set (all four are meaningful for a dashboard).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContainerKind {
    /// A left-to-right split (`Horizontal`; `shares` size the columns).
    Horizontal,
    /// A top-to-bottom split (`Vertical`; `shares` size the rows).
    Vertical,
    /// A wrapping grid (`Grid`; `columns` fixes the column count, `shares` size
    /// the COLUMNS — one per column, applied via `with_column_shares`).
    Grid,
    /// Stacked tabs (`Tabs`; no sizing — one child visible at a time).
    Tabs,
}

impl ContainerKind {
    /// The wire kind strings, in the order the error message lists them.
    pub const WIRE_KINDS: [&'static str; 4] = ["horizontal", "vertical", "grid", "tabs"];

    /// The stable wire name (matches the accepted `kind` string).
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Horizontal => "horizontal",
            Self::Vertical => "vertical",
            Self::Grid => "grid",
            Self::Tabs => "tabs",
        }
    }

    /// Resolve a wire `kind` string, or a loud
    /// [`LayoutError::UnknownContainerKind`] naming the supported set.
    pub fn from_wire(kind: &str) -> Result<Self, LayoutError> {
        match kind {
            "horizontal" => Ok(Self::Horizontal),
            "vertical" => Ok(Self::Vertical),
            "grid" => Ok(Self::Grid),
            "tabs" => Ok(Self::Tabs),
            other => Err(LayoutError::UnknownContainerKind(other.to_string())),
        }
    }
}

/// A layout-spec validation failure (the precise, actionable errors the
/// `set_blueprint` verb surfaces VERBATIM to the controller). Every arm names
/// the fixable problem; nothing is a silent drop.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum LayoutError {
    /// A view `kind` string that is not one of [`ViewKind::WIRE_KINDS`].
    #[error(
        "unknown view kind '{0}' — supported view kinds: spatial3d, spatial2d, time_series, \
         text_document"
    )]
    UnknownViewKind(String),
    /// A container `kind` string that is not one of [`ContainerKind::WIRE_KINDS`].
    #[error(
        "unknown container kind '{0}' — supported container kinds: horizontal, vertical, grid, tabs"
    )]
    UnknownContainerKind(String),
    /// A container with no children (a viewport pane must hold something).
    #[error(
        "the '{0}' container is empty — every container must hold at least one view or nested \
         container"
    )]
    EmptyContainer(&'static str),
    /// `horizontal`/`vertical` `shares` length ≠ child count (each child IS a
    /// sized column/row, so shares are one-per-CHILD there — grids differ, see
    /// [`Self::GridSharesLenMismatch`]).
    #[error(
        "the '{container}' container has {shares} share(s) but {children} child(ren) — a \
         horizontal/vertical container's 'shares' must have exactly one entry per child"
    )]
    SharesLenMismatch {
        /// The offending container kind's wire name.
        container: &'static str,
        /// The declared `shares` length.
        shares: usize,
        /// The actual child count.
        children: usize,
    },
    /// A `shares` entry that is not a finite value strictly greater than zero.
    #[error(
        "the '{0}' container has a non-positive or non-finite share — every share must be a finite \
         value greater than 0"
    )]
    NonPositiveShare(&'static str),
    /// `shares` set on a `tabs` container (tabs stack, they are not sized).
    #[error("a 'tabs' container does not take 'shares' (tabs stack, one child shows at a time)")]
    SharesOnTabs,
    /// `columns` set on a non-grid container (grid-only).
    #[error("'columns' is only valid on a 'grid' container, not on '{0}'")]
    ColumnsOnNonGrid(&'static str),
    /// A `grid` container's `columns` is zero (the viewer would silently clamp it
    /// to one column — a wrong layout with no signal).
    #[error("a 'grid' container's 'columns' must be greater than 0")]
    ZeroGridColumns,
    /// A `grid` container's `columns` exceeds its child count (extra columns are
    /// pointless, and an unbounded value would OOM the viewer's per-column sizing).
    #[error(
        "the 'grid' container has 'columns' = {columns} but only {children} child(ren) — 'columns' \
         must be between 1 and the child count"
    )]
    TooManyGridColumns {
        /// The declared column count.
        columns: u32,
        /// The actual child count.
        children: usize,
    },
    /// A `grid` container declares `shares` without `columns`. A grid's shares are
    /// COLUMN-scoped (one per column), so the column count must be known to
    /// validate + apply them.
    #[error(
        "a 'grid' container with 'shares' must also set 'columns' — a grid's shares are per-column, \
         so the column count must be known"
    )]
    GridSharesRequireColumns,
    /// A `grid` container's `shares` length ≠ its `columns` (grid shares are
    /// per-COLUMN, not per-child).
    #[error(
        "the 'grid' container has {shares} share(s) but {columns} column(s) — a grid's 'shares' \
         are per-column (exactly one entry per column)"
    )]
    GridSharesLenMismatch {
        /// The declared `shares` length.
        shares: usize,
        /// The declared column count.
        columns: u32,
    },
    /// Guardrail 3 (the all-guessed-origins failure): EVERY leaf
    /// view in the layout is ungrounded — not one of its origins matches an attached
    /// topic — so the whole dashboard would render EMPTY. One aggregate refusal
    /// naming the guessed origins AND the concrete attached entity paths to use
    /// instead (only raised when topics ARE attached; a lay-before-attach layout
    /// with nothing attached is never refused).
    #[error(
        "every view in the layout is ungrounded — none of its origins ({}) matches any attached \
         topic, so the whole dashboard would render empty; use one of the attached entity paths \
         ({}) from discover/attach/status/list (never guessed origins)",
        .origins.join(", "), .available.join(", ")
    )]
    AllViewsUngrounded {
        /// The distinct ungrounded view origins, in declaration order.
        origins: Vec<String>,
        /// The concrete attached entity paths the agent should use instead
        /// (distinct, sorted) — the "did you mean" targets, mirroring G1.
        available: Vec<String>,
    },
    /// Guardrail 2 (the LaserScan shape): a `spatial2d` view would render
    /// NOTHING — every attached topic it grounds resolves (via
    /// [`views_for_render`], so an override counts) to a view family that
    /// excludes `spatial2d` (a
    /// LaserScan / point cloud projects to `Points3D` → a 3D scene; a scalar bag to
    /// a time-series plot; …). Only raised when NO grounded topic supports 2D (the
    /// pane is provably empty); a spatial2d view that grounds at least one 2D-capable
    /// topic APPLIES and its 3D-only siblings become a soft warning instead. Each
    /// offender phrase names the topic, its archetype, and the exact view kinds it
    /// DOES render in (derived from the archetype's own table — never hard-coded).
    #[error(
        "the spatial2d view at origin '{origin}' would render nothing — none of its grounded \
         topics can display in 2D: {}",
        .offenders.join("; ")
    )]
    Spatial2dRendersNothing {
        /// The offending view's origin (or the world root when it had none).
        origin: String,
        /// One `'topic' (Archetype → renders only in a, b)` phrase per grounded
        /// topic that cannot display in 2D.
        offenders: Vec<String>,
    },
    /// Guardrail 1: a named view's origin grounds NO attached topic AND
    /// the layout's `auto_views` is OFF, so nothing will populate the view — it
    /// would render EMPTY. Names the view kind + origin, suggests the nearest
    /// attached entities, and states the fix. (With `auto_views` ON an ungrounded
    /// view stays a soft, non-fatal hint — the agent may lay out ahead + auto_views
    /// fills the rest.)
    #[error(
        "the {view_kind} view at origin '{origin}' matches no attached topic and auto_views is off, \
         so it would render empty — use one of the attached entity paths (nearest: {}) from \
         discover/attach/status/list, or enable auto_views",
        .suggestions.join(", ")
    )]
    UngroundedView {
        /// The view kind's wire name.
        view_kind: &'static str,
        /// The ungrounded origin.
        origin: String,
        /// The nearest attached entity paths (the "did you mean" suggestions).
        suggestions: Vec<String>,
    },
    /// Guardrail 4: a view carries EXPLICIT `contents`
    /// globs (only the [`compose_layout`] compiler emits these — `set_blueprint`
    /// views carry none) that ground NO attached topic, so the pane would render
    /// empty even though its origin trivially grounds. The origin-only guardrails
    /// (1/3) cannot catch a contents glob that diverged from its (grounded) origin;
    /// this one does. The robot-skeleton glob grounds the URDF (not an attached
    /// topic), so a contents list that is ONLY the robot glob is exempt.
    #[error(
        "the {view_kind} view at origin '{origin}' has contents ({}) that match no attached topic, \
         so it would render empty — use one of the attached entity paths ({}) from \
         discover/attach/status/list",
        .contents.join(", "), .available.join(", ")
    )]
    ViewContentsUngrounded {
        /// The view kind's wire name.
        view_kind: &'static str,
        /// The view's origin (or the world root when it had none).
        origin: String,
        /// The emitted contents globs that ground nothing.
        contents: Vec<String>,
        /// The concrete attached entity paths to target instead (distinct, sorted).
        available: Vec<String>,
    },
}

/// A single leaf VIEW in a [`BlueprintPlan`]: its kind, an optional display name,
/// an optional entity-path origin (defaults to the world root when absent — the
/// same `WORLD_ORIGIN` the Go2 default uses), and optional explicit CONTENTS.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanView {
    /// The view kind.
    pub kind: ViewKind,
    /// The display name (defaults to the kind's title when absent).
    pub name: Option<String>,
    /// The entity-path origin the view is rooted at (defaults to the world root).
    /// For a spatial view this is ALSO the coordinate-frame anchor, so the
    /// [`compose_layout`] hero 3D view roots at the world origin even while its
    /// [`contents`](Self::contents) explicitly list the assigned topic subtrees.
    pub origin: Option<String>,
    /// Explicit rerun view-contents query expressions (e.g.
    /// `["/world/utlidar/cloud/**", "/world/tf-tree/robot/**"]`), or `None` to keep
    /// rerun's default `$origin/**` (the pre-PR-D behavior — `set_blueprint` +
    /// the Go2 default set `None`). The [`compose_layout`] compiler sets this to
    /// the UNION of the assigned topics' `<entity>/**` globs so ONE view grounds
    /// EXACTLY the topics the agent placed in it — the "one plot per topic, no
    /// origin guessing" linchpin. `Some(vec![])` would ground nothing, so the
    /// compiler never emits an empty list (it only builds a view for a non-empty
    /// bucket); the emit path treats an empty list as "keep the default".
    pub contents: Option<Vec<String>>,
}

/// A CONTAINER node in a [`BlueprintPlan`]: its kind, its child nodes, and the
/// validated sizing knobs (`shares` / `columns`) already checked against the kind.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanContainer {
    /// The container kind.
    pub kind: ContainerKind,
    /// The child nodes (views and/or nested containers), in layout order.
    pub children: Vec<PlanNode>,
    /// The optional display name.
    pub name: Option<String>,
    /// Relative sizes along the sized axis, all finite > 0, never on `tabs`.
    /// Validated per kind: `horizontal` (column) and `vertical` (row) take one
    /// share per CHILD (each child is a column/row); `grid` takes one share per
    /// COLUMN (applied via `with_column_shares`) and so requires `columns`.
    pub shares: Option<Vec<f32>>,
    /// Grid column count (grid only — validated `1..=children.len()`).
    pub columns: Option<u32>,
}

/// A node in a validated layout tree: a leaf [`PlanView`] or a [`PlanContainer`].
#[derive(Debug, Clone, PartialEq)]
pub enum PlanNode {
    /// A leaf view.
    View(PlanView),
    /// A container of child nodes.
    Container(PlanContainer),
}

/// A validated, rerun-agnostic viewport layout — the inspectable intermediate
/// between the wire spec and the emitted blueprint `LogMsg`s. Produced by the
/// daemon's spec validation; consumed by [`build_blueprint_msgs`]. `PartialEq`
/// so unit tests assert exact structure against hand oracles.
#[derive(Debug, Clone, PartialEq)]
pub struct BlueprintPlan {
    /// The root node (a container tree, or a single view).
    pub root: PlanNode,
    /// Whether the viewer may auto-create views for archetypes the explicit views
    /// do not cover (the Go2 default is `true`).
    pub auto_views: bool,
    /// Whether this plan's views are AUTO-DECORATED at emit time — every
    /// `time_series` view gets the trailing live-scope window (`TRAILING_WINDOW_SECS`)
    /// as BOTH a query-side `VisibleTimeRanges` and a display-side `TimeAxis`
    /// x-axis window (so an empty plot shows a tight `[-30s, 0]` axis, not the epoch
    /// axis), and every spatial (`spatial2d`/`spatial3d`) view the Studio stage
    /// background (`STAGE_BACKGROUND_RGB`). `true` for [`compose_layout`] compiler products AND
    /// the built-in [`BlueprintPlan::go2_default`] — a robot-free desk must
    /// show the Studio stage background by default, not rerun's off-brand green.
    /// (The default is Scene-ONLY, so it carries only the spatial background; the
    /// time_series window/axis decorations ride the compose path's plots, which have
    /// data.) `false` ONLY for hand-authored `set_blueprint` plans — a power user's explicit
    /// blueprint is theirs and is NEVER auto-decorated (see `build_plan` in the vizd
    /// daemon). The flag travels with the plan into `RUNTIME_BLUEPRINT`, so a
    /// reconnect-reapply preserves the decoration (the decoration is derived at emit,
    /// so it never drifts from the flag). [`build_blueprint_msgs`] is the ONLY reader.
    pub decorate: bool,
}

impl BlueprintPlan {
    /// The Go2 default scene as a plan: a SINGLE full-bleed `Spatial3D` "Scene" view
    /// rooted at `WORLD_ORIGIN`, decorated with the Studio stage background, auto-views
    /// ON. A `set_blueprint` RESET reproduces this built-in default.
    ///
    /// **Decision** ("if a plot with no data is gonna default to 1970,
    /// either fix it or drop the panel"): the default scene DROPS the Telemetry
    /// (`TimeSeries`) and Status & Field Dumps (`TextDocument`) panels — a robot-free
    /// desk has no scalar/status data, so those panels were BOTH empty (the Telemetry
    /// axis rendered the nonsense 1970-era epoch span, and the Status panel showed
    /// nothing). Panels now exist ONLY when there is data to fill them:
    ///
    /// - `auto_views: true` is LOAD-BEARING here (not decorative): when a user
    ///   checkbox-attaches a NON-spatial topic (scalars/status), the frames log under
    ///   `/world/...` — inside this 3D view's `$origin/**` contents — but the 3D view
    ///   cannot VISUALIZE them. rerun's `spawn_heuristic_views` then auto-spawns the
    ///   right view (a `TimeSeries` plot for scalars, a text view for a `TextLog`),
    ///   because its redundancy filter is scoped PER VIEW CLASS
    ///   (`re_viewport_blueprint::ViewportBlueprint::spawn_heuristic_views`, lines
    ///   ~388-398: only existing views OF THE SAME CLASS suppress a recommendation).
    ///   With no `TimeSeries`/`TextDocument` view in the default, the scalar/status
    ///   recommendation survives → an auto-view spawns WITH data → no 1970 axis.
    ///   Dropping `auto_views` would make a scalar attach render NOWHERE.
    /// - `decorate: true` keeps the stage `Background` (`STAGE_BACKGROUND_RGB`,
    ///   #10161f) on the spatial scene so a fresh desk is on-brand by default (an
    ///   UNDECORATED spatial view falls back to rerun's off-brand `GradientDark`
    ///   skybox). With no `TimeSeries` view here, the default emits no
    ///   `VisibleTimeRanges`/`TimeAxis` — those decorations ride ONLY the
    ///   [`compose_layout`] path's time_series views (the live-scope fix), where
    ///   the plots carry real data.
    pub fn go2_default() -> Self {
        let cfg = BlueprintConfig::default();
        Self {
            auto_views: cfg.auto_views,
            decorate: true,
            // Scene-only, full-bleed: a lone Spatial3D view (the emitter wraps a
            // single-view root in a synthetic Tabs container — see build_blueprint_msgs).
            root: PlanNode::View(PlanView {
                kind: ViewKind::Spatial3d,
                name: Some(cfg.scene_name),
                origin: Some(cfg.world_origin),
                contents: None,
            }),
        }
    }

    /// The number of leaf VIEWS in the plan (reported in the verb's response).
    pub fn view_count(&self) -> u32 {
        count_views(&self.root)
    }
}

fn count_views(node: &PlanNode) -> u32 {
    match node {
        PlanNode::View(_) => 1,
        PlanNode::Container(c) => c.children.iter().map(count_views).sum(),
    }
}

/// The default display name for a view kind when the spec names none.
fn default_view_name(kind: ViewKind) -> &'static str {
    match kind {
        ViewKind::Spatial3d => "3D",
        ViewKind::Spatial2d => "2D",
        ViewKind::TimeSeries => "Plots",
        ViewKind::TextDocument => "Text",
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Time-range + Background folds: HAND-ROLLED blueprint emission with
// per-view PROPERTIES the SDK cannot set.
//
// rerun 0.34.1's high-level `Blueprint`/`View` builders expose NO reachable way to
// set a per-view visible-time-range or a `Background` on a time_series / spatial
// view (`View::add_property` is `pub(crate)`, `with_defaults` writes the wrong —
// entity-defaults — path, and the view UUIDs are generated inside
// `Blueprint::send`). So [`build_blueprint_msgs`] replicates the SDK's emission
// (see re_sdk `blueprint::{api,container,view}`) on a blueprint `RecordingStream`,
// and ADDITIONALLY logs `VisibleTimeRanges` / `TimeAxis` / `Background` onto each
// view's PROPERTY path `view/<uuid>/<ArchetypeShortName>` — exactly where the viewer
// reads them (`re_viewport_blueprint::entity_path_for_view_property`; consumed by
// `ViewBlueprint::query_range` for the query window, `re_view_time_series`'s
// `TimeAxis:view_range` read for the DISPLAY x-axis extents, and
// `configure_background` for the background). ONLY a `decorate = true`
// (compose-compiled) plan is decorated.
// ────────────────────────────────────────────────────────────────────────────

/// The default trailing PLOT window, in whole seconds. Every compose-emitted
/// `time_series` view carries a `[-secs, 0]` CURSOR-RELATIVE window, so a live plot
/// scrolls like a scope (the line rides the RIGHT edge, no shrink-to-fit, no empty
/// future half) instead of accumulating the full history width. 30 s balances
/// "enough recent context to read a trend" against "tight enough to read as live" on
/// control-rate telemetry. Cursor-relative ⇒ deterministic (no wall-clock read); the
/// viewer resolves the cursor.
const TRAILING_WINDOW_SECS: i64 = 30;

/// The Studio stage background — `--cer-bg-stage` in the studio
/// `palette.toml` (the cool blue-black `#10161f`), replacing rerun's `GradientDark`
/// 3D skybox / 2D solid fallback so scene backgrounds match the Studio palette
/// without forking `re_renderer` (CANNOT-MATCH rows 2+3). A plain hex constant here
/// (no cross-repo read); keep in sync with the palette if it ever changes.
const STAGE_BACKGROUND_RGB: [u8; 3] = [0x10, 0x16, 0x1f];

/// Decision: the timeline the viewer opens on by default — rerun's
/// built-in `log_time`, which is the DESK's wall-clock RECEIVE time (every logged
/// row carries it automatically). vizd ALSO stamps a custom [`ROBOT_TIME`] timeline
/// from each frame's wire `timestamp_ns` (see [`crate::archetype::set_robot_time`]),
/// but a robot commonly stamps CLOCK_MONOTONIC (time-since-boot — the Go2 does), so
/// with NO default pin rerun's own heuristic (`Timeline::pick_best_timeline`) prefers
/// the user-defined `robot_time` over `log_time` and the viewer opens showing a
/// nonsense "+6h10m" uptime axis. Pinning the panel timeline to `log_time` in EVERY
/// blueprint vizd emits makes a fresh attach open on wall-clock receive time; the
/// `robot_time` timeline stays fully available in the viewer's timeline dropdown for
/// cross-sensor correlation (it is still logged — this only changes which timeline is
/// ACTIVE, via [`TimePanelBlueprint`]'s `timeline` component read by the viewer's
/// `TimeControl::update_from_blueprint`). See [`build_blueprint_msgs`].
const DEFAULT_ACTIVE_TIMELINE: &str = "log_time";

/// The timelines a trailing window is emitted for. Scalars ride the
/// `robot_time` timeline ([`crate::archetype::ROBOT_TIME`]); `log_time` is rerun's
/// built-in wall timeline every logged row ALSO carries. The viewer applies a window
/// only when its `timeline` matches the ACTIVE timeline
/// (`re_viewport_blueprint::ViewBlueprint::query_range`), so emitting one range per
/// timeline makes the live scope apply whichever temporal timeline the user views.
/// Both are nanosecond time timelines, so a cursor-relative `[-secs, 0]` in ns is
/// correct on each (a sequence timeline like `log_tick` is deliberately excluded — a
/// `-30e9`-tick window is meaningless there).
const TRAILING_WINDOW_TIMELINES: [&str; 2] = [ROBOT_TIME, "log_time"];

/// One nanosecond-per-second factor for the cursor-relative window boundary.
const NANOS_PER_SEC: i64 = 1_000_000_000;

/// The single cursor-relative `[-secs, 0]` window (in ns) — the ONE range
/// shared by the query-side `VisibleTimeRanges` (per timeline) and the display-side
/// `TimeAxis:view_range` (timeline-agnostic). Pure + deterministic (no clock read).
fn trailing_window_range(secs: i64) -> TimeRange {
    TimeRange {
        start: TimeRangeBoundary::CursorRelative(TimeInt(-secs * NANOS_PER_SEC)),
        end: TimeRangeBoundary::CursorRelative(TimeInt(0)),
    }
}

/// Build the rerun `VisibleTimeRange` datatype values for a `secs`-second
/// trailing window — `[-secs, 0]` CURSOR-RELATIVE (in ns) on each of
/// [`TRAILING_WINDOW_TIMELINES`]. Pure + deterministic (no clock read); the exact
/// serialized values are oracle-pinned (`trailing_window_ranges_are_cursor_relative`).
fn trailing_window_ranges(secs: i64) -> Vec<VisibleTimeRange> {
    let range = trailing_window_range(secs);
    TRAILING_WINDOW_TIMELINES
        .iter()
        .map(|tl| VisibleTimeRange {
            timeline: (*tl).into(),
            range,
        })
        .collect()
}

/// The `VisibleTimeRanges` blueprint archetype for the default trailing
/// window — logged onto a time_series view's `view/<uuid>/VisibleTimeRanges` property
/// path (the path `ViewBlueprint::query_range` reads). This bounds only the QUERY
/// (which rows are fetched), NOT the rendered x-axis — see [`trailing_window_time_axis`].
fn trailing_window_archetype() -> VisibleTimeRanges {
    VisibleTimeRanges::new(trailing_window_ranges(TRAILING_WINDOW_SECS))
}

/// Completes the time-range work: the `TimeAxis` blueprint archetype pinning a plot's
/// DISPLAY x-axis to the same `[-30s, 0]` cursor-relative window — logged onto a
/// time_series view's `view/<uuid>/TimeAxis` property path. This is the seam
/// `re_view_time_series` consults for the RENDERED x-axis extents (its
/// `TimeAxis:view_range` read, `view_class.rs::resolve_time_range`) AND the query
/// range (`util::determine_query_range`). Stamping only `VisibleTimeRanges`
/// (the query seam) leaves the DISPLAY axis falling back to `TimeRange::EVERYTHING`
/// on an EMPTY plot — which resolves to the full-i64 timeline bounds and renders the
/// nonsense epoch axis (≈1696→2243). Stamping `view_range` makes an empty plot show a
/// tight `[-30s, 0]` window and a live plot scroll like a scope (the cursor rides the
/// right edge — the intent `VisibleTimeRanges` alone could not deliver).
/// Timeline-AGNOSTIC: one `TimeRange` applies to whichever timeline is active, so no
/// per-timeline expansion. `link`/`zoom_lock` are left unset (fallbacks: Independent,
/// unlocked). Oracle-pinned by `decorated_plan_emits_*` / `go2_default_emits_*`.
fn trailing_window_time_axis() -> TimeAxis {
    TimeAxis::new().with_view_range(trailing_window_range(TRAILING_WINDOW_SECS))
}

/// The solid `Background` blueprint archetype for the Studio stage color —
/// logged onto a spatial view's `view/<uuid>/Background` property path (the path
/// `re_view_spatial::configure_background` reads). `SolidColor` kind replaces the
/// GradientDark skybox / 2D fallback.
fn stage_background_archetype() -> Background {
    let [r, g, b] = STAGE_BACKGROUND_RGB;
    Background::new(BackgroundKind::SolidColor).with_color(rerun::Color::from_rgb(r, g, b))
}

/// The rerun view-class identifier string for a [`ViewKind`] (mirrors the SDK's
/// `*View::new` `class_identifier`, so the hand-rolled `ViewBlueprint` is byte-shaped
/// exactly like the SDK's).
fn view_class_id(kind: ViewKind) -> &'static str {
    match kind {
        ViewKind::Spatial3d => "3D",
        ViewKind::Spatial2d => "2D",
        ViewKind::TimeSeries => "TimeSeries",
        ViewKind::TextDocument => "TextDocument",
    }
}

/// Render a 16-byte id as the canonical lowercase `8-4-4-4-12` UUID string — the
/// `container/<id>` / `view/<id>` path segment the viewer parses via
/// `uuid::Uuid::parse_str` (`re_viewer_context::BlueprintId::from_entity_path`). A
/// non-UUID segment would fail that parse and the node would be dropped, so the
/// segment MUST be UUID-shaped.
fn format_uuid(b: &[u8; 16]) -> String {
    format!(
        "{:02x}{:02x}{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
        b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], b[10], b[11], b[12], b[13],
        b[14], b[15],
    )
}

/// The FNV-1a 128 offset basis + prime (the standard constants). Hand-rolled, and
/// deliberately NOT `std::collections::hash_map::DefaultHasher`: SipHash-1-3's output
/// is explicitly not guaranteed stable across Rust releases, and a toolchain bump that
/// silently re-minted every blueprint node id would reset every view's retained UI
/// state on upgrade. FNV-1a is fixed by its constants, so the ids are stable forever.
const FNV_OFFSET_128: u128 = 0x6c62_272e_07bb_0142_62b8_2175_6295_c58d;
/// See [`FNV_OFFSET_128`].
const FNV_PRIME_128: u128 = 0x0000_0000_0100_0000_0000_0000_0000_013b;

/// FNV-1a 128 over `bytes`.
fn fnv1a_128(bytes: &[u8]) -> u128 {
    let mut h = FNV_OFFSET_128;
    for b in bytes {
        h ^= u128::from(*b);
        h = h.wrapping_mul(FNV_PRIME_128);
    }
    h
}

/// The 16 raw id bytes for one IDENTITY string: the fixed `0xce30` ("cerulion")
/// namespace prefix + 14 bytes (112 bits) of [`fnv1a_128`] over the identity.
fn id_bytes_for(identity: &str) -> [u8; 16] {
    let h = fnv1a_128(identity.as_bytes()).to_be_bytes();
    let mut bytes = [0u8; 16];
    bytes[0] = 0xce;
    bytes[1] = 0x30;
    bytes[2..16].copy_from_slice(&h[2..16]);
    bytes
}

/// The field separator inside an identity key. `\u{1}` (SOH) cannot appear in a rerun
/// entity path or a view/container kind name, so the joined key is unambiguous. (Even
/// if two distinct tuples somehow rendered the same key they would merely SHARE an
/// occurrence counter and take indices 0 and 1 — distinct ids, never an alias.)
const ID_KEY_SEP: char = '\u{1}';

/// Mints DETERMINISTIC blueprint node ids from a node's IDENTITY, never from
/// its POSITION in the emitted plan.
///
/// The id used to be a bare monotone counter, so it was a function of emission order:
/// ANY plan change — a topic checked or unchecked, a second video rendition appearing
/// mid-run, a re-scan — shifted every node after the insertion point by one, and an id
/// that had carried a `Spatial2d` view was re-declared as a `TextDocument` (or a view
/// id became a container id, since containers were minted from the same counter). The
/// viewer keys its per-view UI state by view id, so it then read the retained state
/// through the wrong class and failed the downcast — one
/// `Failed to downcast view's … to SpatialViewState` toast per shifted view, per
/// frame. The dump-companion work made mixed-class view pairs at one origin routine
/// (`[Spatial2d, TextDocument]` for images/video, `[TimeSeries, TextDocument]` for
/// scalars-with-text), which is why a multi-rendition camera was the loudest trigger.
///
/// Hashing the identity instead means a plan change ADDS and REMOVES ids rather than
/// RE-LABELLING them: retained state either matches or is absent, and both are fine.
///
/// The identity keys, and why they are what they are:
///
/// * **view** — `(class, origin)`. The CLASS is the load-bearing term: it is exactly
///   what the failed downcast disagreed about, so with the class in the hash a given
///   id can never change class, whatever else moves.
/// * **container** — `(kind, name, tree path)`. A container has no content of its own;
///   it IS its kind at its position. The kind is in the hash for the same reason a
///   view's class is, and the `container` domain tag keeps a container id and a view id
///   in disjoint spaces (under the counter a plan change could turn one into the other).
///
/// Deliberately NOT in a view's key:
///
/// * **contents** — folding the globs in would be more content-addressed, but the
///   default layout's hero Scene view grounds the UNION of every attached spatial
///   topic, so its contents change on EVERY spatial attach. Its id would then be
///   re-minted each time and the user's dragged 3D camera would reset on every
///   checkbox — a NEW regression, in exchange for precision the occurrence
///   disambiguator below already supplies.
/// * **name** — `build_image_views` renames a lone image view from `Images` to
///   `Images: /cam1/image_raw` the moment a SECOND camera attaches, so a name in the
///   key would re-mint the first camera's view on an unrelated attach.
///
/// Two nodes may legitimately share an identity key (two same-class views at one
/// origin differing only in contents). The layout producers here do not emit that
/// shape — `default_layout` gives each topic its own origin, and a `compose_layout`
/// bucket roots at its first topic entity — so in practice it arrives from a
/// hand-authored `set_blueprint` plan, which is the user's to shape. Such views are
/// separated by an OCCURRENCE INDEX within that exact key, assigned in emission order.
/// That index shifts only when a SAME-KEY sibling is added, removed, or REORDERED
/// relative to it — an enormously narrower blast radius than global position — and
/// because the class is in the key, a shift can only ever hand one view another
/// SAME-CLASS view's state, never trip the downcast.
///
/// A hash COLLISION between distinct keys would alias two nodes onto one id, so it is
/// resolved rather than assumed away: the minter tracks the ids it has issued and, on a
/// collision, re-hashes with an escalating salt, saying so loudly.
///
/// SCOPE of that arm, because it is the ONE path whose id is not a pure function
/// of the node's identity: the salt is resolved in EMISSION order, so for a given plan
/// it is deterministic, but which of a colliding pair takes the unsalted digest can
/// FLIP across a plan change. The loser is merely re-minted (empty viewer state), but
/// the new WINNER inherits the digest the other key held before — and the two keys may
/// name different CLASSES, so on that flip the same-class guarantee above does NOT
/// hold and a downcast could fail exactly once, until the state is re-created. Nothing
/// cheaper fixes it (a collision is only knowable once both keys are in hand), and at
/// 112 bits over a few dozen nodes it is a defensive arm rather than a running cost —
/// which is why it is loud, and why the guarantee is stated with its exception rather
/// than as an absolute.
#[derive(Default)]
struct NodeIdMinter {
    /// How many ids have been minted for each identity key so far.
    occurrences: std::collections::HashMap<String, u32>,
    /// Every id issued this emit, so a hash collision is detected rather than aliased.
    minted: std::collections::HashSet<[u8; 16]>,
}

impl NodeIdMinter {
    /// The identity key of a VIEW: its rerun class identifier + its resolved origin
    /// (resolved exactly as [`emit_view`] resolves the origin it logs).
    fn view_key(v: &PlanView) -> String {
        let class = view_class_id(v.kind);
        let origin = v.origin.as_deref().unwrap_or(WORLD_ORIGIN);
        format!("view{ID_KEY_SEP}{class}{ID_KEY_SEP}{origin}")
    }

    /// The identity key of a CONTAINER: its kind + display name + its path from the
    /// plan root (the child indices walked to reach it, `.`-joined; empty at the root).
    fn container_key(kind: ContainerKind, name: Option<&str>, path: &[usize]) -> String {
        let path: Vec<String> = path.iter().map(|i| i.to_string()).collect();
        format!(
            "container{ID_KEY_SEP}{}{ID_KEY_SEP}{}{ID_KEY_SEP}{}",
            kind.as_str(),
            name.unwrap_or(""),
            path.join(".")
        )
    }

    /// Mint the id for one identity `key`. Returns BOTH the raw bytes (the
    /// `RootContainer` reference needs a `datatypes::Uuid`) AND the canonical UUID
    /// string (the path segment) — the same 16 bytes feed both, so they always resolve
    /// to the same viewer-side id.
    fn mint(&mut self, key: &str) -> ([u8; 16], String) {
        let occurrence = self.occurrences.entry(key.to_string()).or_insert(0);
        let n = *occurrence;
        *occurrence += 1;
        let identity = format!("{key}{ID_KEY_SEP}#{n}");
        let mut salt: u32 = 0;
        loop {
            let bytes = if salt == 0 {
                id_bytes_for(&identity)
            } else {
                id_bytes_for(&format!("{identity}{ID_KEY_SEP}~{salt}"))
            };
            if self.minted.insert(bytes) {
                if salt > 0 {
                    tracing::warn!(
                        identity = %identity,
                        salt,
                        id = %format_uuid(&bytes),
                        "blueprint node id hash collision — resolved with a salt (deterministic); \
                         this node's retained viewer state starts empty"
                    );
                }
                return (bytes, format_uuid(&bytes));
            }
            salt += 1;
        }
    }
}

/// Emit ONE view + its per-view properties to the blueprint stream, returning its
/// `view/<uuid>` blueprint path + the raw id bytes. Mirrors re_sdk `View::log_to_stream`
/// (ViewContents + ViewBlueprint), plus the per-view property logs when `decorate`.
fn emit_view(
    bp: &RecordingStream,
    v: &PlanView,
    decorate: bool,
    ids: &mut NodeIdMinter,
) -> RecordingStreamResult<(String, [u8; 16])> {
    let (bytes, uuid) = ids.mint(&NodeIdMinter::view_key(v));
    let base = format!("view/{uuid}");

    // ViewContents (the placed globs, or rerun's default `$origin/**`).
    let contents: Vec<String> = match v.contents.as_deref().filter(|c| !c.is_empty()) {
        Some(c) => c.to_vec(),
        None => vec!["$origin/**".to_string()],
    };
    let view_contents = ViewContents::new(contents.into_iter().map(QueryExpression::from));
    bp.log(format!("{base}/ViewContents"), &view_contents)?;

    // ViewBlueprint (class + name + origin).
    let name = v
        .name
        .clone()
        .unwrap_or_else(|| default_view_name(v.kind).to_string());
    let arch = ViewBlueprint::new(ViewClass::from(view_class_id(v.kind)))
        .with_display_name(Name::from(name))
        .with_space_origin(v.origin.as_deref().unwrap_or(WORLD_ORIGIN).to_string());
    bp.log(base.clone(), &arch)?;

    // Per-view PROPERTIES, only on a compose-decorated plan, by view kind.
    if decorate {
        match v.kind {
            ViewKind::TimeSeries => {
                // VisibleTimeRanges bounds the QUERY; TimeAxis bounds the rendered
                // x-axis (the empty-epoch-axis fix + the live scope).
                bp.log(
                    format!("{base}/VisibleTimeRanges"),
                    &trailing_window_archetype(),
                )?;
                bp.log(format!("{base}/TimeAxis"), &trailing_window_time_axis())?;
            }
            ViewKind::Spatial2d | ViewKind::Spatial3d => {
                bp.log(format!("{base}/Background"), &stage_background_archetype())?;
            }
            ViewKind::TextDocument => {}
        }
    }

    Ok((base, bytes))
}

/// Emit ONE container + its subtree to the blueprint stream (children FIRST, then the
/// container), returning its `container/<uuid>` path + the raw id bytes. Mirrors
/// re_sdk `container::log_container` (contents = the children's paths; horizontal →
/// column shares, vertical → row shares, grid → columns + column shares; tabs stack).
fn emit_container(
    bp: &RecordingStream,
    c: &PlanContainer,
    decorate: bool,
    ids: &mut NodeIdMinter,
    tree_path: &mut Vec<usize>,
) -> RecordingStreamResult<(String, [u8; 16])> {
    let mut child_paths: Vec<String> = Vec::with_capacity(c.children.len());
    for (i, child) in c.children.iter().enumerate() {
        tree_path.push(i);
        let emitted = emit_node(bp, child, decorate, ids, tree_path);
        tree_path.pop();
        child_paths.push(emitted?.0);
    }

    let (bytes, uuid) = ids.mint(&NodeIdMinter::container_key(
        c.kind,
        c.name.as_deref(),
        tree_path,
    ));
    let path = format!("container/{uuid}");

    let kind = match c.kind {
        ContainerKind::Horizontal => RrContainerKind::Horizontal,
        ContainerKind::Vertical => RrContainerKind::Vertical,
        ContainerKind::Grid => RrContainerKind::Grid,
        ContainerKind::Tabs => RrContainerKind::Tabs,
    };
    let mut arch = ContainerBlueprint::new(kind);
    if let Some(name) = &c.name {
        arch = arch.with_display_name(Name::from(name.clone()));
    }
    if !child_paths.is_empty() {
        arch = arch.with_contents(child_paths.iter().map(|p| IncludedContent::from(p.clone())));
    }
    // Shares map exactly as the rerun SDK builder did: horizontal/grid → COLUMN
    // shares, vertical → ROW shares; tabs take none.
    match c.kind {
        ContainerKind::Horizontal | ContainerKind::Grid => {
            if let Some(shares) = &c.shares {
                arch = arch.with_col_shares(shares.iter().map(|&s| ColumnShare(Float32(s))));
            }
        }
        ContainerKind::Vertical => {
            if let Some(shares) = &c.shares {
                arch = arch.with_row_shares(shares.iter().map(|&s| RowShare(Float32(s))));
            }
        }
        ContainerKind::Tabs => {}
    }
    if c.kind == ContainerKind::Grid {
        if let Some(columns) = c.columns {
            arch = arch.with_grid_columns(GridColumns(UInt32(columns)));
        }
    }
    bp.log(path.clone(), &arch)?;
    Ok((path, bytes))
}

/// Emit any plan node (view or container). `tree_path` is the child-index path walked
/// from the plan root to `node` — a CONTAINER's identity (see [`NodeIdMinter`]); a VIEW
/// ignores it entirely, which is exactly what makes a view id survive an unrelated
/// topic being inserted somewhere else in the tree.
fn emit_node(
    bp: &RecordingStream,
    node: &PlanNode,
    decorate: bool,
    ids: &mut NodeIdMinter,
    tree_path: &mut Vec<usize>,
) -> RecordingStreamResult<(String, [u8; 16])> {
    match node {
        PlanNode::View(v) => emit_view(bp, v, decorate, ids),
        PlanNode::Container(c) => emit_container(bp, c, decorate, ids, tree_path),
    }
}

/// Build the blueprint `LogMsg`s for `plan` — the hand-rolled replica of
/// re_sdk `Blueprint::to_log_msgs`, with per-view decoration when `plan.decorate`.
/// Emits, on a fresh in-memory blueprint recording: each view's ViewContents +
/// ViewBlueprint (+ the VisibleTimeRanges / TimeAxis / Background properties), each
/// container's ContainerBlueprint, and the root ViewportBlueprint. A single-VIEW root
/// is wrapped in a `Tabs` container (rerun's `Blueprint::new` behavior — the viewport
/// root must reference a container). Exposed (not `send`-coupled) so tests can decode
/// the emitted chunks and assert the property paths + serialized values.
pub fn build_blueprint_msgs(
    app_id: &str,
    plan: &BlueprintPlan,
) -> RecordingStreamResult<Vec<LogMsg>> {
    // The PUBLIC emit always PINS the default timeline — it is the boot/default
    // emission (the first send of a daemon run, which must beat rerun's
    // `pick_best_timeline` custom-timeline preference) AND the tests' observation
    // point. The runtime RE-APPLY path ([`apply_runtime_blueprint`]) emits with
    // `pin_timeline = false`, so a topic toggle never re-pins + snaps a
    // user's manual timeline choice back to `log_time`.
    build_blueprint_msgs_inner(app_id, plan, true)
}

/// [`build_blueprint_msgs`] with an explicit `pin_timeline` gate. When
/// `false`, the `time_panel` timeline pin is OMITTED — so a runtime re-apply never
/// resets the viewer's active timeline (which would snap a user who switched to
/// `robot_time` back to `log_time` on every attach/detach). Only [`send_blueprint_once`]
/// (the boot/reconnect send — the FIRST send per epoch, `ensure_setup` runs it before
/// the worker loop) pins; [`apply_runtime_blueprint`] does not.
fn build_blueprint_msgs_inner(
    app_id: &str,
    plan: &BlueprintPlan,
    pin_timeline: bool,
) -> RecordingStreamResult<Vec<LogMsg>> {
    let (bp, storage) = RecordingStreamBuilder::new(app_id).blueprint().memory()?;
    // Required so the viewer identifies the blueprint data (mirrors re_sdk).
    bp.set_time_sequence("blueprint", 0);

    let mut ids = NodeIdMinter::default();
    let mut tree_path: Vec<usize> = Vec::new();
    // The viewport's root_container must be a CONTAINER id. A single-view root is
    // wrapped in a synthetic Tabs container, exactly like `Blueprint::new`.
    let (_root_path, root_bytes) = match &plan.root {
        PlanNode::Container(c) => emit_container(&bp, c, plan.decorate, &mut ids, &mut tree_path)?,
        PlanNode::View(v) => {
            let (view_path, _) = emit_view(&bp, v, plan.decorate, &mut ids)?;
            // The synthetic wrapper IS the root container, so it takes the identity a
            // root `Tabs` container would have taken (kind `tabs`, no name, empty path).
            let (bytes, uuid) = ids.mint(&NodeIdMinter::container_key(
                ContainerKind::Tabs,
                None,
                &tree_path,
            ));
            let path = format!("container/{uuid}");
            let arch = ContainerBlueprint::new(RrContainerKind::Tabs)
                .with_contents([IncludedContent::from(view_path)]);
            bp.log(path, &arch)?;
            (String::new(), bytes)
        }
    };
    // `_root_path` is unused for a wrapped single-view root (the viewport references
    // the container by its id bytes, not its path); silence the binding uniformly.
    let _ = _root_path;

    let viewport = ViewportBlueprint::new()
        .with_root_container(RootContainer(RrUuid::from(root_bytes)))
        .with_auto_views(AutoViews(Bool(plan.auto_views)));
    bp.log("viewport", &viewport)?;

    // Pin the viewer's DEFAULT active timeline to `log_time` (desk wall-clock
    // receive time). The viewer reads `TimePanelBlueprint:timeline` at the `time_panel`
    // entity in `TimeControl::update_from_blueprint`; without it, rerun's
    // `pick_best_timeline` heuristic prefers the custom user-defined `robot_time` over
    // `log_time`, opening on a boot-relative "+6h10m" uptime axis. Emitted ONLY on the
    // first/boot send (`pin_timeline` is false on runtime re-applies), so a
    // topic toggle never snaps a user's manual timeline choice back. `robot_time` stays
    // selectable in the dropdown. See [`DEFAULT_ACTIVE_TIMELINE`].
    if pin_timeline {
        let time_panel = TimePanelBlueprint::new().with_timeline(DEFAULT_ACTIVE_TIMELINE);
        bp.log("time_panel", &time_panel)?;
    }

    Ok(storage.take())
}

/// Send `plan` to `rec` as a fully hand-rolled blueprint (see
/// [`build_blueprint_msgs`]) with the given activation. The replacement for the SDK
/// `Blueprint::send`-based emission at every runtime emission site (so the
/// per-view decoration is applied AND survives a reconnect-reapply).
fn send_plan(
    rec: &RecordingStream,
    plan: &BlueprintPlan,
    activation: BlueprintActivation,
    pin_timeline: bool,
) -> RecordingStreamResult<()> {
    let app_id = rec
        .store_info()
        .map(|info| info.application_id().to_string())
        .unwrap_or_else(|| "rerun_example_app".to_owned());
    let msgs = build_blueprint_msgs_inner(&app_id, plan, pin_timeline)?;
    let Some(first) = msgs.first() else {
        return Ok(());
    };
    let blueprint_id = first.store_id().clone();
    let activation_cmd = BlueprintActivationCommand {
        blueprint_id,
        make_active: activation.make_active,
        make_default: activation.make_default,
    };
    rec.send_blueprint(msgs, activation_cmd);
    Ok(())
}

/// Inspection helper: the ENTITY PATH of every chunk in a blueprint
/// `LogMsg` stream (as from [`build_blueprint_msgs`]), decoding each `ArrowMsg` into
/// a `Chunk`. Lets a test prove the per-view PROPERTY paths are emitted where the
/// viewer reads them — `view/<uuid>/VisibleTimeRanges` (the query-side trailing
/// window; consumed by `re_viewport_blueprint::ViewBlueprint::query_range`),
/// `view/<uuid>/TimeAxis` (the DISPLAY-side x-axis window; consumed by
/// `re_view_time_series::view_class::resolve_time_range`), and
/// `view/<uuid>/Background` (the stage background; consumed by
/// `re_view_spatial::configure_background`). Non-`ArrowMsg` messages (SetStoreInfo /
/// activation) carry no entity and are skipped; an undecodable chunk is skipped
/// (never a panic — this is an observation seam).
pub fn blueprint_property_paths(msgs: &[LogMsg]) -> Vec<String> {
    msgs.iter()
        .filter_map(|msg| match msg {
            LogMsg::ArrowMsg(_, arrow) => rerun::log::Chunk::from_arrow_msg(arrow)
                .ok()
                .map(|chunk| chunk.entity_path().to_string()),
            _ => None,
        })
        .collect()
}

/// The viewer's pinned DEFAULT active-timeline name decoded from a blueprint
/// `LogMsg` stream (as from [`build_blueprint_msgs`]) — the actual serialized
/// `TimePanelBlueprint:timeline` VALUE at the `time_panel` entity the viewer reads in
/// `TimeControl::update_from_blueprint`, NOT just its path. `None` when no
/// `time_panel` timeline chunk was emitted (so a test can pin the wall-clock default
/// by string without naming rerun types). Undecodable chunks are skipped (an
/// observation seam, never a panic).
pub fn blueprint_panel_timeline(msgs: &[LogMsg]) -> Option<String> {
    for msg in msgs {
        let LogMsg::ArrowMsg(_, arrow) = msg else {
            continue;
        };
        let Ok(chunk) = rerun::log::Chunk::from_arrow_msg(arrow) else {
            continue;
        };
        // The blueprint entity path renders with a leading slash (`/time_panel`).
        if chunk.entity_path().to_string().trim_start_matches('/') != "time_panel" {
            continue;
        }
        let names: Vec<String> = chunk
            .iter_component::<RrTimelineName>(TimePanelBlueprint::descriptor_timeline().component)
            .flat_map(|item| {
                item.iter()
                    .map(|n| n.as_str().to_string())
                    .collect::<Vec<_>>()
            })
            .collect();
        if let Some(name) = names.into_iter().next() {
            return Some(name);
        }
    }
    None
}

/// One decoded `/Background` view-property chunk from a blueprint `LogMsg` stream —
/// the actual serialized component VALUES `re_view_spatial::configure_background`
/// reads (NOT just its entity path).
#[derive(Debug, Clone, PartialEq)]
pub struct DecodedBackground {
    /// The `view/<uuid>/Background` entity path the chunk was logged onto.
    pub path: String,
    /// The decoded [`BackgroundKind`] values (one per row; a decorated stage view
    /// carries exactly `[SolidColor]`).
    pub kinds: Vec<BackgroundKind>,
    /// The decoded solid `Color`s as gamma sRGB `[r, g, b, a]` (one per row; the
    /// stage color is `#10161f` → `[0x10, 0x16, 0x1f, 0xff]`).
    pub colors: Vec<[u8; 4]>,
}

/// One decoded `/VisibleTimeRanges` view-property chunk from a blueprint `LogMsg`
/// stream — the actual serialized `VisibleTimeRange` VALUES
/// `re_viewport_blueprint::ViewBlueprint::query_range` reads (NOT just its path).
#[derive(Debug, Clone, PartialEq)]
pub struct DecodedWindow {
    /// The `view/<uuid>/VisibleTimeRanges` entity path the chunk was logged onto.
    pub path: String,
    /// The decoded trailing-window ranges (one [`VisibleTimeRange`] per timeline;
    /// the default is cursor-relative `[-30s, 0]` on `robot_time` + `log_time`).
    pub ranges: Vec<VisibleTimeRange>,
}

impl DecodedBackground {
    /// The decoded kind(s) rendered via rerun's stable `Display` (e.g. `"SolidColor"`)
    /// — a rerun-type-free handle so a downstream test can pin the kind by string
    /// without naming the rerun component type.
    pub fn kind_names(&self) -> Vec<String> {
        self.kinds.iter().map(|k| k.to_string()).collect()
    }
}

impl DecodedWindow {
    /// The timeline name(s) the ranges apply to, in order (rerun-type-free).
    pub fn timelines(&self) -> Vec<String> {
        self.ranges
            .iter()
            .map(|r| r.timeline.0.as_str().to_string())
            .collect()
    }

    /// Each range's `[start, end]` bounds as plain CURSOR-RELATIVE nanoseconds, or
    /// `None` for a bound that is not cursor-relative (rerun-type-free) — so a
    /// downstream test can pin the exact window as `(-30e9, 0)` without rerun types.
    pub fn cursor_relative_ns(&self) -> Vec<(Option<i64>, Option<i64>)> {
        use rerun::external::re_sdk_types::datatypes::{TimeInt, TimeRangeBoundary};
        let ns = |b: &TimeRangeBoundary| match b {
            TimeRangeBoundary::CursorRelative(TimeInt(n)) => Some(*n),
            _ => None,
        };
        self.ranges
            .iter()
            .map(|r| (ns(&r.range.start), ns(&r.range.end)))
            .collect()
    }
}

/// One decoded `/TimeAxis` view-property chunk from a blueprint `LogMsg` stream — the
/// actual serialized `TimeAxis:view_range` VALUE(s) `re_view_time_series` reads for the
/// rendered x-axis extents (`view_class.rs::resolve_time_range`) and query range
/// (`util::determine_query_range`). This is the DISPLAY-side lever the empty-axis
/// fix stamps (the `/VisibleTimeRanges` sibling bounds only the query).
#[derive(Debug, Clone, PartialEq)]
pub struct DecodedTimeAxis {
    /// The `view/<uuid>/TimeAxis` entity path the chunk was logged onto.
    pub path: String,
    /// The decoded `view_range`(s) (one [`TimeRange`] per row; a decorated time_series
    /// view carries exactly ONE — cursor-relative `[-30s, 0]`, timeline-agnostic).
    pub view_ranges: Vec<TimeRange>,
}

impl DecodedTimeAxis {
    /// Each `view_range`'s `[start, end]` bounds as plain CURSOR-RELATIVE nanoseconds,
    /// or `None` for a bound that is not cursor-relative (rerun-type-free) — so a test
    /// can pin the display window as `(-30e9, 0)` without naming rerun types.
    pub fn cursor_relative_ns(&self) -> Vec<(Option<i64>, Option<i64>)> {
        use rerun::external::re_sdk_types::datatypes::{TimeInt, TimeRangeBoundary};
        let ns = |b: &TimeRangeBoundary| match b {
            TimeRangeBoundary::CursorRelative(TimeInt(n)) => Some(*n),
            _ => None,
        };
        self.view_ranges
            .iter()
            .map(|r| (ns(&r.start), ns(&r.end)))
            .collect()
    }
}

/// The decoded per-view decoration COMPONENT VALUES of a blueprint `LogMsg` stream.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct DecodedDecorations {
    /// Every `/Background` view-property chunk, in emit order.
    pub backgrounds: Vec<DecodedBackground>,
    /// Every `/VisibleTimeRanges` view-property chunk, in emit order.
    pub windows: Vec<DecodedWindow>,
    /// Every `/TimeAxis` view-property chunk (the display-side x-axis window), in emit
    /// order — the empty-axis / live-scope lever.
    pub time_axes: Vec<DecodedTimeAxis>,
}

/// The value-level sibling of [`blueprint_property_paths`].
/// Decodes the emitted `Background` (kind + color) and `VisibleTimeRanges` (ranges)
/// component batches out of a blueprint `LogMsg` stream (as from
/// [`build_blueprint_msgs`]) so a test can assert the emitter's OUTPUT against hand
/// oracles — the exact bytes the viewer reads: `Background:kind`/`Background:color`
/// (consumed by `re_view_spatial::configure_background`), `VisibleTimeRanges:ranges`
/// (consumed by `ViewBlueprint::query_range`), and `TimeAxis:view_range` (consumed by
/// `re_view_time_series::view_class::resolve_time_range` for the DISPLAY x-axis). The
/// deserialization uses the SAME archetype component descriptors the emit path
/// writes with, so a value drift (e.g. `SolidColor` → `GradientDark`) is caught,
/// not just its entity path. Undecodable chunks are skipped (an observation seam,
/// never a panic).
pub fn blueprint_decorations(msgs: &[LogMsg]) -> DecodedDecorations {
    // The blueprint `VisibleTimeRange` COMPONENT (distinct from the same-named
    // `datatypes::VisibleTimeRange` imported at module scope — it Derefs to it).
    use rerun::external::re_sdk_types::blueprint::components::VisibleTimeRange as VtrComponent;
    // The blueprint `TimeRange` COMPONENT wrapping `datatypes::TimeRange` (imported at
    // module scope) — the `TimeAxis:view_range` type.
    use rerun::external::re_sdk_types::blueprint::components::TimeRange as TimeRangeComponent;

    let mut out = DecodedDecorations::default();
    for msg in msgs {
        let LogMsg::ArrowMsg(_, arrow) = msg else {
            continue;
        };
        let Ok(chunk) = rerun::log::Chunk::from_arrow_msg(arrow) else {
            continue;
        };
        let path = chunk.entity_path().to_string();
        if path.ends_with("/Background") {
            let kinds: Vec<BackgroundKind> = chunk
                .iter_component::<BackgroundKind>(Background::descriptor_kind().component)
                .flat_map(|item| item.to_vec())
                .collect();
            let colors: Vec<[u8; 4]> = chunk
                .iter_component::<rerun::Color>(Background::descriptor_color().component)
                .flat_map(|item| item.iter().map(|c| c.to_array()).collect::<Vec<_>>())
                .collect();
            out.backgrounds.push(DecodedBackground {
                path,
                kinds,
                colors,
            });
        } else if path.ends_with("/VisibleTimeRanges") {
            let ranges: Vec<VisibleTimeRange> = chunk
                .iter_component::<VtrComponent>(VisibleTimeRanges::descriptor_ranges().component)
                .flat_map(|item| item.iter().map(|r| r.0.clone()).collect::<Vec<_>>())
                .collect();
            out.windows.push(DecodedWindow { path, ranges });
        } else if path.ends_with("/TimeAxis") {
            let view_ranges: Vec<TimeRange> = chunk
                .iter_component::<TimeRangeComponent>(TimeAxis::descriptor_view_range().component)
                .flat_map(|item| item.iter().map(|r| r.0).collect::<Vec<_>>())
                .collect();
            out.time_axes.push(DecodedTimeAxis { path, view_ranges });
        }
    }
    out
}

/// SOFT check for the `set_blueprint` verb: the view origins in `plan` that match
/// NONE of the currently-attached render `entities`. A HINT, never an error — an
/// agent may lay out a dashboard BEFORE attaching the topics that populate it.
///
/// An origin is "grounded" when it is the world root (`WORLD_ORIGIN` — which
/// prefixes every tap entity) or shares a path-prefix relationship with some
/// attached entity (leading/trailing slashes normalized away). Returns the
/// UNGROUNDED origins, de-duplicated, in first-seen (declaration) order. Pure —
/// oracle-tested.
pub fn plan_origins_unmatched(plan: &BlueprintPlan, entities: &[String]) -> Vec<String> {
    let mut out = Vec::new();
    collect_unmatched_origins(&plan.root, entities, &mut out);
    out
}

fn collect_unmatched_origins(node: &PlanNode, entities: &[String], out: &mut Vec<String>) {
    match node {
        PlanNode::Container(c) => {
            for child in &c.children {
                collect_unmatched_origins(child, entities, out);
            }
        }
        PlanNode::View(v) => {
            if let Some(origin) = &v.origin {
                if !origin_is_grounded(origin, entities) && !out.iter().any(|o| o == origin) {
                    out.push(origin.clone());
                }
            }
        }
    }
}

/// Normalize an entity path for prefix comparison: drop leading/trailing slashes.
fn normalize_entity(path: &str) -> &str {
    path.trim_matches('/')
}

fn origin_is_grounded(origin: &str, entities: &[String]) -> bool {
    let o = normalize_entity(origin);
    // The world root prefixes every tap entity, so it is always grounded (an
    // empty origin defaults to the world root too).
    if o.is_empty() || o == normalize_entity(WORLD_ORIGIN) {
        return true;
    }
    entities.iter().any(|e| {
        let e = normalize_entity(e);
        e == o || e.starts_with(&format!("{o}/")) || o.starts_with(&format!("{e}/"))
    })
}

// ────────────────────────────────────────────────────────────────────────────
// The set_blueprint GUARDRAILS. `set_blueprint` stays the manual /
// power escape hatch, so these reject ONLY layouts that are PROVABLY broken
// against the currently-attached topics — never merely unusual-but-valid shapes.
// The all-guessed-origins failure: a hand-authored blueprint used GUESSED entity
// origins (`/cloud`, `/cmd_vel` — real paths are `world/<topic path>`), so every
// named view rendered EMPTY, and a LaserScan sat in a spatial2d view where it
// shows nothing. These three guardrails turn that silent
// apply-a-broken-layout into a loud, actionable refusal. All pure over
// [`ArchetypeKind`] + the merged [`views_for_archetype`] table (never duplicated).
// ────────────────────────────────────────────────────────────────────────────

/// The stable wire name for an [`ArchetypeKind`] (the protocol string the vizd
/// responses + guardrail-2 error carry). Exhaustive — a variant rename is a compile
/// error here, never a silent protocol drift.
pub fn archetype_wire_name(kind: ArchetypeKind) -> &'static str {
    use ArchetypeKind as A;
    match kind {
        A::Points3D => "Points3D",
        A::Image => "Image",
        A::Transforms => "Transforms",
        A::Scalars => "Scalars",
        A::ScalarsWithText => "ScalarsWithText",
        A::Transform3D => "Transform3D",
        A::Point3D => "Point3D",
        A::Transform3DWithScalars => "Transform3DWithScalars",
        A::Point3DWithScalars => "Point3DWithScalars",
        A::Imu => "Imu",
        A::Odometry => "Odometry",
        A::LaserScan => "LaserScan",
        A::SportModeState => "SportModeState",
        A::TextLog => "TextLog",
        A::Skeleton => "Skeleton",
        A::Boxes3D => "Boxes3D",
        A::OccupancyGrid => "OccupancyGrid",
        A::Path3D => "Path3D",
        A::PoseArray3D => "PoseArray3D",
        A::VideoStream => "VideoStream",
        A::MarkerArray => "MarkerArray",
        A::AnyValues => "AnyValues",
    }
}

/// One attached topic's render placement — the guardrails' input: the absolute
/// `topic`, the render `entity` its frames log under, and its resolved `archetype`
/// (`None` for a still-silent topic). The daemon builds this from its live tap
/// stats; keeping it rerun-free keeps the guardrails oracle-testable.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AttachedRender {
    /// The absolute topic.
    pub topic: String,
    /// The full render entity path the topic's frames log under.
    pub entity: String,
    /// The resolved archetype, or `None` when the topic is still silent.
    pub archetype: Option<ArchetypeKind>,
    /// The topic's LIVE producer count (from the desk's SHM probe, or a
    /// robot's catalog for a remote mirror), or `None` when it could not be probed.
    /// The compose compiler ([`compose_layout`]) reads this to PREFER live topics:
    /// `Some(0)` is a registered-but-DEAD route (nothing publishing, like the live
    /// `/uslam/cloud_map`), which the AUTOMAGIC (role-less) path SKIPS from placement
    /// (surfaced in [`ComposedLayout::skipped_dead`]) while an EXPLICIT (role-tagged)
    /// placement is honored but flagged [`ComposedPlacement::dead`]; `Some(n)` (n>0)
    /// and `None` (UNKNOWN) are both treated as live — unknown liveness is NEVER
    /// penalized. The `set_blueprint` guardrails and [`default_layout`] IGNORE this
    /// field (liveness gates only the compose compiler); the daemon populates it via
    /// `TransportManager::topic_publisher_count_checked`.
    pub producer_count: Option<u32>,
    /// This topic's H.264 rendition SEGMENTS (`["640x360", "1280x720"]`),
    /// in resolution order — empty for every non-video topic.
    ///
    /// A video topic renders each rendition under its OWN child entity
    /// (`<entity>/viz-video/<segment>`). Placed in ONE topic-rooted view they
    /// OVERLAY: both decode correctly and one is invisible — the exact failure
    /// per-topic views exist to prevent. The video path renders ONE view rooted
    /// at the DEFAULT rendition (`default_rendition`, the highest pixel count)
    /// rather than a view per rendition, because a user reads two
    /// tiles of one camera as the topic being checked twice. With 0 or 1
    /// renditions the topic keeps its single subtree-rooted view, so the common
    /// case is untouched.
    ///
    /// This field has a SECOND reader: a non-empty set is positive
    /// evidence that the topic's video is DECODING, which is what lets
    /// [`views_for_render`] drop the dump companion that would otherwise
    /// sit beside the video as a permanently empty pane. It is kept as
    /// VIDEO's signal — see `render_is_proven` (beside [`views_for_render`]) for why
    /// video reads this while every other archetype reads [`RenderProof`].
    ///
    /// Populated by the daemon from [`crate::worker::VizWorkerCounters::video_renditions`],
    /// which the worker mirrors out of the demux — the demux lives on the render
    /// thread and the layout is built on the control thread, so this is the only
    /// path between them.
    pub video_renditions: Vec<String>,
    /// The operator's REPRESENTATION choice for this topic —
    /// [`Representation::Auto`] for every topic nobody overrode.
    ///
    /// The layout must apply the SAME choice the render will
    /// ([`views_for_render`]), or a forced dump lands in a topic whose views
    /// carry no `text_document` and the operator gets a document nothing
    /// displays — the rule, one layer up.
    ///
    /// Deliberately the CHOICE and not the resolved plan: `archetype: None` has
    /// to keep meaning "still silent" (the layout treats it as un-placeable), so
    /// a suppressed visual half cannot be encoded by blanking that field.
    pub representation: Representation,
    /// What this topic's render arm has been OBSERVED to do — the live
    /// "provably rendering" signal [`views_for_render`] gates the default
    /// `text_document` companion on, for every archetype that has one.
    ///
    /// The companion is refused for VIDEO on the evidence of
    /// [`AttachedRender::video_renditions`]; here that refusal generalizes,
    /// on evidence that exists for every degradable kind rather than for one.
    /// Its DEFAULT (nothing observed) keeps the companion, so a caller that cannot
    /// supply the signal gets exactly the earlier behaviour.
    ///
    /// Populated by the daemon from [`crate::worker::VizWorkerCounters::render_proofs`],
    /// which the worker mirrors out of its [`crate::sink::SinkState`] once per
    /// batch — the same demux→counters→daemon path the video work opened for the
    /// rendition set, and for the same reason: the render arms run on the worker
    /// thread and the layout is built on the control thread.
    pub render_proof: RenderProof,
}

/// The view kinds `render` needs, with the operator's representation choice applied.
///
/// [`views_for_archetype`] answers for a KIND; this answers for a TOPIC, which
/// is the question the layout is actually asking once a choice can suppress the
/// visual half or add a document to it. A still-silent topic (`archetype: None`)
/// yields nothing, exactly as before — it is un-TYPED, not un-rendered.
///
/// The `text_document` a forced dump needs is added HERE rather than inside
/// [`views_for_archetype`], whose table is const-asserted to carry that view IFF
/// the KIND [`ArchetypeKind::renders_text_document`] — an IFF that is about the
/// archetype and must stay true of it.
pub fn views_for_render(render: &AttachedRender) -> Vec<ViewKind> {
    let Some(kind) = render.archetype else {
        return Vec::new();
    };
    let plan = resolve_render_plan(kind, render.representation);
    let mut views: Vec<ViewKind> = plan.visual.map(views_for_archetype).unwrap_or(&[]).to_vec();
    // The operator's representation choice is the DISCRIMINATOR, taken once: a FORCED
    // dump always gets its pane, and only an un-forced one can have the default
    // companion refused (first for video, then for every other archetype
    // that can prove it is rendering). Written as one `if`/`else` rather than a drop followed by
    // a re-add — that ordering made the "was it forced?" test a property of which
    // statement ran last, so a swap silently turned `representation both` into a
    // no-op on exactly the topic class the video refusal touches.
    if plan.dump {
        if !views.contains(&ViewKind::TextDocument) {
            views.push(ViewKind::TextDocument);
        }
    } else if dump_companion_is_provably_unused(kind, render) {
        views.retain(|v| *v != ViewKind::TextDocument);
    }
    views
}

/// Whether [`views_for_archetype`]'s `text_document` companion is
/// provably an EMPTY pane on this topic, the generalization of the
/// video-only refusal to every archetype that carries a live rendering signal.
///
/// The standing rule is that nothing may render nothing: a kind that can degrade to a
/// field dump carries a `text_document` view so the dump has somewhere to land,
/// and it accepted the cost in its own words — "a healthy camera / map / marker
/// topic carries one extra (empty until it degrades) status pane". A user
/// reads that pane, on the flagship camera, as the topic being checked twice
/// (video refused it first); the same decision then covers every archetype that can prove it is
/// rendering. One checked topic is one pane.
///
/// **Signal-gated, never blanket.** Four conjuncts, each closing one way this
/// could become the silent-dump regression that rule closed:
///
/// * **The pane must be earned by DEGRADING** ([`ArchetypeKind::can_degrade_to_dump`]).
///   [`ArchetypeKind::TextLog`] and [`ArchetypeKind::ScalarsWithText`] earn it
///   through [`ArchetypeKind::renders_text_document`]'s OTHER clause — a PRIMARY
///   text render — so refusing it would delete the view their own data lands in.
///   That is the text-topic silent drop, re-created from the opposite side.
/// * **The kind must have a VISUAL half** ([`views_carry_a_visual`]). The check is
///   structural rather than a named exception so a future degradable-but-text-only
///   kind is protected by construction: [`ArchetypeKind::AnyValues`]'s dump IS its
///   render, and dropping its only view would leave the topic with NO pane —
///   strictly worse than the empty one this is removing.
/// * **Nothing may have degraded yet.** Sticky ([`RenderProof::degraded`] — see
///   there for its true scope, which outlives one attach), which is what makes
///   the core guarantee reachable: the pane APPEARS when a degradation fires
///   and STAYS, rather than vanishing on the next healthy frame before anyone can
///   read it.
/// * **The render must be PROVEN** ([`render_is_proven`]). A topic nobody has seen
///   render keeps its pane — that is the never-seen-robot case the whole ladder
///   exists for, and it is the DEFAULT rather than an afterthought.
///
/// A FORCED dump never reaches here — [`views_for_render`] takes
/// `plan.dump` as its discriminator first, so the operator's own choice is
/// answered before this refusal is consulted.
fn dump_companion_is_provably_unused(kind: ArchetypeKind, render: &AttachedRender) -> bool {
    kind.can_degrade_to_dump()
        && views_carry_a_visual(kind)
        && !render.render_proof.degraded
        && render_is_proven(kind, render)
}

/// The per-archetype "provably rendering" signal — positive evidence
/// that this topic's visual half is working, so its dump companion would be empty.
///
/// TWO signals, because video's is both STRONGER and older:
///
/// * [`ArchetypeKind::VideoStream`] reads [`AttachedRender::video_renditions`],
///   the video refusal's shipped template, left byte-for-byte as it shipped. A rendition
///   segment exists because the demux opened a sub-stream from a REAL SPS, so it
///   is evidence about the decoder rather than about one arm's return path.
/// * Every other kind reads [`RenderProof::rendered_without_dumping`] — a frame
///   reached the arm and the arm did not dump. See that field for the four paths
///   on which it claims slightly more than "it drew", and why none of them can
///   hide a dump.
///
/// [`ArchetypeKind::Skeleton`] needs no arm here and gets none: it is INERT on
/// every live run since the last `install_skeleton` caller was removed, so its
/// arm ALWAYS dumps and the general signal can never turn true for it. Its
/// companion is kept by the evidence itself rather than by an exception — which is
/// the shape the whole predicate is built for: where no live signal exists, the
/// pane stays.
fn render_is_proven(kind: ArchetypeKind, render: &AttachedRender) -> bool {
    match kind {
        ArchetypeKind::VideoStream => !render.video_renditions.is_empty(),
        _ => render.render_proof.rendered_without_dumping,
    }
}

/// Whether [`views_for_archetype`] gives `kind` any view OTHER than the
/// `text_document` one — a `const` slice scan, the mirror of
/// [`views_carry_text_document`].
///
/// It is what makes "never leave a topic with zero views" a STRUCTURAL property of
/// the refusal rather than a variant name someone has to remember to exclude.
const fn views_carry_a_visual(kind: ArchetypeKind) -> bool {
    let views = views_for_archetype(kind);
    let mut i = 0;
    while i < views.len() {
        if !matches!(views[i], ViewKind::TextDocument) {
            return true;
        }
        i += 1;
    }
    false
}

/// Collect every leaf [`PlanView`] in the plan, in declaration order.
fn collect_leaf_views<'a>(node: &'a PlanNode, out: &mut Vec<&'a PlanView>) {
    match node {
        PlanNode::View(v) => out.push(v),
        PlanNode::Container(c) => {
            for child in &c.children {
                collect_leaf_views(child, out);
            }
        }
    }
}

/// Whether a leaf view is GROUNDED against the attached `entities`: a view with no
/// origin defaults to the world root (always grounded); an origin is grounded iff
/// it shares a path-prefix relationship with some attached entity (see
/// [`origin_is_grounded`]).
fn leaf_view_grounded(v: &PlanView, entities: &[String]) -> bool {
    match &v.origin {
        None => true,
        Some(o) => origin_is_grounded(o, entities),
    }
}

/// Whether a view rooted at `view_origin` WOULD RENDER a topic at `topic_entity` —
/// the strict subtree relation (a rerun view shows entities AT OR BELOW its
/// origin). A `None` origin (or an origin at the world root) renders every attached
/// entity. This is deliberately stricter than [`origin_is_grounded`] (which also
/// grounds when the origin is BELOW the entity): guardrail 2 asks "does this 2D
/// view actually try to draw that topic", not "are they related".
fn view_renders_entity(view_origin: Option<&str>, topic_entity: &str) -> bool {
    let e = normalize_entity(topic_entity);
    match view_origin {
        None => true,
        Some(o) => {
            let o = normalize_entity(o);
            if o.is_empty() || o == normalize_entity(WORLD_ORIGIN) {
                return true;
            }
            e == o || e.starts_with(&format!("{o}/"))
        }
    }
}

/// Rank attached `entities` by descending shared leading path-segment count with
/// `origin` (ties broken alphabetically), returning up to `max` distinct nearest
/// entities — the "did you mean" suggestions for an ungrounded view. Pure.
fn nearest_entities(origin: &str, entities: &[String], max: usize) -> Vec<String> {
    let o_segs: Vec<&str> = normalize_entity(origin)
        .split('/')
        .filter(|s| !s.is_empty())
        .collect();
    let mut scored: Vec<(usize, &String)> = entities
        .iter()
        .map(|e| {
            let e_segs = normalize_entity(e).split('/').filter(|s| !s.is_empty());
            let shared = o_segs
                .iter()
                .zip(e_segs)
                .take_while(|(a, b)| **a == *b)
                .count();
            (shared, e)
        })
        .collect();
    // Most-shared first; equal scores fall to alphabetical for determinism.
    scored.sort_by(|a, b| b.0.cmp(&a.0).then_with(|| a.1.cmp(b.1)));
    let mut out: Vec<String> = Vec::new();
    for (_, e) in scored {
        if !out.contains(e) {
            out.push(e.clone());
        }
        if out.len() >= max {
            break;
        }
    }
    out
}

/// The view list a guardrail DIAGNOSTIC must quote for one attached topic, plus
/// the clause naming an override when one is in force.
///
/// The guardrail DECISIONS ask `views_for_render` (what this topic
/// renders), so their messages have to answer the same question or they
/// contradict themselves in a single sentence. Measured on this module's own
/// fixture, a camera under `Text` was refused with "Image → renders only in
/// spatial2d, text_document" as the reason it cannot display in 2D, and the soft
/// warning told the operator to move it into the view it was already in. Neither
/// named the override — the only fact that would let anyone fix it.
///
/// Un-overridden topics render EXACTLY as before (empty suffix, and
/// `views_for_render` reduces to `views_for_archetype`), which is what keeps the
/// pre-existing `guardrail_g2_*` message oracles meaningful rather than merely green.
fn renders_only_in_for(render: &AttachedRender) -> (String, String) {
    let views = views_for_render(render)
        .iter()
        .map(|k| k.as_str())
        .collect::<Vec<_>>()
        .join(", ");
    let suffix = if render.representation.is_auto() {
        String::new()
    } else {
        format!(
            " because its representation is set to '{}' (change it back, or move the view)",
            render.representation.as_wire()
        )
    };
    (views, suffix)
}

/// One `'topic' (Archetype → renders only in a, b)` offender phrase for the
/// guardrail-2 "renders nothing" refusal.
fn offender_phrase(render: &AttachedRender, kind: ArchetypeKind) -> String {
    let (views, why) = renders_only_in_for(render);
    format!(
        "'{}' ({} → renders only in {views}{why})",
        render.topic,
        archetype_wire_name(kind),
    )
}

/// The soft warning for a 3D-only sibling that is INVISIBLE inside a spatial2d view
/// that DID ground at least one 2D-capable topic (the mixed case — applied, not
/// refused).
fn invisible_sibling_warning(origin: &str, render: &AttachedRender, kind: ArchetypeKind) -> String {
    let (views, why) = renders_only_in_for(render);
    format!(
        "the spatial2d view at origin '{origin}' also grounds '{}' ({}), which renders only \
         in {views}{why} and is invisible in 2D — move it to a {views} view",
        render.topic,
        archetype_wire_name(kind),
    )
}

/// Whether ANY leaf view in `plan` grounds at least one of `entities`. Pure — used
/// by the reconnect-reapply record-only grounding warn (`warn_if_plan_grounds_nothing`).
pub fn plan_grounds_any_entity(plan: &BlueprintPlan, entities: &[String]) -> bool {
    let mut leaves = Vec::new();
    collect_leaf_views(&plan.root, &mut leaves);
    leaves.iter().any(|v| leaf_view_grounded(v, entities))
}

/// Validate a [`BlueprintPlan`] against the currently-attached render
/// placements, REFUSING a provably-broken layout. `set_blueprint` stays the manual
/// / power escape hatch, so an unusual-but-VALID layout still applies (returning any
/// soft warnings the caller appends to its hint list). The guardrails, in the
/// deterministic PRECEDENCE order they are checked (which is the tie-break when a
/// layout violates more than one):
///
/// 1. **All views ungrounded** ([`LayoutError::AllViewsUngrounded`]) — every leaf
///    view's origin matches no attached topic, so the whole dashboard renders empty
///    (the all-guessed-origins failure). Checked FIRST, regardless of
///    `auto_views` (the most severe: nothing named renders).
/// 2. **`spatial2d` view renders nothing** ([`LayoutError::Spatial2dRendersNothing`])
///    — a `spatial2d` view whose grounded topics ALL resolve (via
///    [`views_for_render`], so an override counts) to non-`spatial2d`
///    families, so the pane is provably
///    empty (the LaserScan shape). A spatial2d view that grounds at least
///    one 2D-capable topic APPLIES; its 3D-only siblings become a SOFT warning
///    (returned in the `Ok` vec), never a refusal.
/// 3. **Ungrounded named view with `auto_views` off** ([`LayoutError::UngroundedView`])
///    — an ungrounded view that nothing (neither a match nor auto_views) will fill.
/// 4. **View contents ground nothing** ([`LayoutError::ViewContentsUngrounded`],
///    PR-D) — a view with EXPLICIT contents globs (only the [`compose_layout`]
///    compiler emits these; `set_blueprint` views carry none) whose globs match no
///    attached entity, so the pane renders empty even though its ORIGIN grounds. The
///    origin guardrails (1/3) cannot catch a contents glob that diverged from a
///    grounded origin; this one does (the robot-skeleton glob is exempt). Checked
///    between guardrail 2 and guardrail 1, unconditional (auto_views cannot fill a
///    view that already declares contents matching nothing).
///
/// The numbers are the scope labels (1 = ungrounded-when-auto-off,
/// 2 = spatial2d-renders-nothing, 3 = all-ungrounded, 4 = contents-ground-nothing);
/// the CHECK order is 3 → 2 → 4 → 1, most-severe-first.
///
/// `Ok(soft_warnings)` for a valid layout (the `spatial2d` invisible-sibling hints).
/// When NOTHING is attached it is always `Ok(vec![])`: a lay-before-attach dashboard
/// cannot be proven broken (the soft [`plan_origins_unmatched`] hint still fires at
/// the caller). The grounding relation matches the soft hint's, so the two never
/// disagree on what is "grounded".
pub fn validate_plan_against_attached(
    plan: &BlueprintPlan,
    attached: &[AttachedRender],
) -> Result<Vec<String>, LayoutError> {
    // Nothing attached → cannot prove anything broken (lay-before-attach is valid).
    if attached.is_empty() {
        return Ok(Vec::new());
    }
    let entities: Vec<String> = attached.iter().map(|a| a.entity.clone()).collect();
    let mut leaves = Vec::new();
    collect_leaf_views(&plan.root, &mut leaves);
    if leaves.is_empty() {
        return Ok(Vec::new());
    }

    // Guardrail 3 (checked first — most severe): EVERY leaf view is ungrounded →
    // the whole dashboard is empty. Names the guessed origins + the concrete
    // attached entity paths to use instead.
    let ungrounded: Vec<&PlanView> = leaves
        .iter()
        .copied()
        .filter(|v| !leaf_view_grounded(v, &entities))
        .collect();
    if ungrounded.len() == leaves.len() {
        let mut origins: Vec<String> = Vec::new();
        for v in &ungrounded {
            // A None-origin view is grounded (world root), so every ungrounded view
            // here carries an explicit Some(origin).
            if let Some(o) = &v.origin {
                if !origins.contains(o) {
                    origins.push(o.clone());
                }
            }
        }
        return Err(LayoutError::AllViewsUngrounded {
            origins,
            available: distinct_sorted(&entities),
        });
    }

    // Guardrail 2 (mixed boundary): a spatial2d view renders in 2D ONLY topics whose
    // archetype supports spatial2d. For each spatial2d view, partition the topics it
    // GROUNDS into 2D-capable and 3D-only. Refuse ONLY when the pane is provably
    // empty (≥1 grounded topic, NONE 2D-capable); when at least one 2D-capable topic
    // grounds, APPLY + soft-warn each invisible 3D-only sibling.
    let mut soft_warnings: Vec<String> = Vec::new();
    for v in &leaves {
        if v.kind != ViewKind::Spatial2d {
            continue;
        }
        let origin_disp = v.origin.as_deref().unwrap_or(WORLD_ORIGIN);
        // The grounded, archetype-RESOLVED topics this 2D view would render.
        let grounded: Vec<(&AttachedRender, ArchetypeKind)> = attached
            .iter()
            .filter_map(|a| {
                let arch = a.archetype?; // a silent topic can't be proven 3D-only
                view_renders_entity(v.origin.as_deref(), &a.entity).then_some((a, arch))
            })
            .collect();
        if grounded.is_empty() {
            continue; // grounding of this view is a G1/G3 concern, not G2
        }
        // Through `views_for_render`, like the two composers — the
        // question is whether THIS TOPIC renders in 2D, and a `Text` override
        // suppresses the visual half entirely (`render_classified` returns before
        // any geometry). On the kind alone an overridden camera validates as
        // 2D-capable and an explicit `set_blueprint` grounded solely on it passes,
        // producing exactly the empty pane this guardrail exists to refuse.
        let two_d_capable = grounded
            .iter()
            .filter(|(a, _)| views_for_render(a).contains(&ViewKind::Spatial2d))
            .count();
        if two_d_capable == 0 {
            // The whole 2D view renders nothing → refuse, naming every offender.
            return Err(LayoutError::Spatial2dRendersNothing {
                origin: origin_disp.to_string(),
                offenders: grounded
                    .iter()
                    .map(|(a, arch)| offender_phrase(a, *arch))
                    .collect(),
            });
        }
        // Mixed: applies. Soft-warn each 3D-only sibling that is invisible here.
        for (a, arch) in &grounded {
            if !views_for_render(a).contains(&ViewKind::Spatial2d) {
                soft_warnings.push(invisible_sibling_warning(origin_disp, a, *arch));
            }
        }
    }

    // Guardrail 4: a view with EXPLICIT contents globs
    // (only the compose_layout compiler emits these; set_blueprint views carry
    // none) must have at least one glob that grounds an attached/placed entity — the
    // origin check alone passes a view whose contents diverged from its (grounded)
    // origin, rendering an empty pane. Unconditional (auto_views cannot fill a view
    // that already declares contents matching nothing). The robot-skeleton glob
    // grounds the URDF (not an attached topic), so a contents list that is ONLY the
    // robot glob is exempt (an include_robot-only hero is legitimately non-topic-
    // grounded).
    for v in &leaves {
        let Some(contents) = v.contents.as_ref() else {
            continue;
        };
        let real: Vec<&String> = contents.iter().filter(|g| !is_robot_glob(g)).collect();
        if real.is_empty() {
            continue; // robot-only (or empty) contents: legitimately non-topic-grounded
        }
        if !real.iter().any(|g| glob_grounds_any(g, &entities)) {
            return Err(LayoutError::ViewContentsUngrounded {
                view_kind: v.kind.as_str(),
                origin: v.origin.clone().unwrap_or_else(|| WORLD_ORIGIN.to_string()),
                contents: contents.clone(),
                available: distinct_sorted(&entities),
            });
        }
    }

    // Guardrail 1 (checked last): with auto_views OFF, an ungrounded named view
    // renders empty and nothing fills it → refuse the first one (declaration order),
    // with the nearest attached entities as suggestions.
    if !plan.auto_views {
        if let Some(v) = ungrounded.first() {
            let origin = v.origin.clone().unwrap_or_default();
            return Err(LayoutError::UngroundedView {
                view_kind: v.kind.as_str(),
                suggestions: nearest_entities(&origin, &entities, 3),
                origin,
            });
        }
    }

    Ok(soft_warnings)
}

/// Distinct, sorted copy of `values` (the deterministic "did you mean" target set).
fn distinct_sorted(values: &[String]) -> Vec<String> {
    let mut out: Vec<String> = values.to_vec();
    out.sort();
    out.dedup();
    out
}

// ────────────────────────────────────────────────────────────────────────────
// The DETERMINISTIC LAYOUT COMPILER. The Studio agent expresses
// SEMANTIC INTENT in TOPIC terms (a set of topics, or role-tagged groups of
// topics) and vizd COMPILES a correct blueprint from the mapping logic it already
// owns — instead of the agent hand-authoring rerun internals (guessing entity
// origins → empty views; auto_views exploding a Twist into 6 panels).
//
// The compiler is PURE over already-RESOLVED topics ([`AttachedRender`]) so the
// placement decisions are oracle-testable without touching the transport or
// rerun; the daemon does the attach-if-missing + per-topic resolution + the emit.
// The pipeline (mirrors the house style — pure core + one emit):
//   resolve  → views_for_archetype (REUSED, never re-derived)
//            → GROUND every view via `with_contents` = the union of `<entity>/**`
//              globs (one per topic — no origin guessing)
//            → auto_views = FALSE always
//            → the caller validates HARD through `validate_plan_against_attached`.
// ────────────────────────────────────────────────────────────────────────────

/// The relative column share of the hero 3D view in the `Horizontal[ hero |
/// sidebar ]` arrangement (the go2_default `3.0` generalized).
const HERO_SHARE: f32 = 3.0;
/// The relative column share of the plots/images/status sidebar (the decided
/// generalization of the go2_default's `1.0` → `1.5`, a slightly wider sidebar
/// now that it can hold three stacked panels instead of two).
const SIDEBAR_SHARE: f32 = 1.5;

/// The default display name for each canonical bucket's view.
const HERO_VIEW_NAME: &str = "3D Scene";
const PLOTS_VIEW_NAME: &str = "Plots";
const IMAGES_VIEW_NAME: &str = "Images";
const STATUS_VIEW_NAME: &str = "Status";

/// The always-present primary "Scene" view name in the [`default_layout`] — the
/// Go2 default's scene name (`BlueprintConfig::default().scene_name`), so a fresh
/// attach keeps a familiar hero title.
const DEFAULT_SCENE_NAME: &str = "Scene";

/// The requested top-level ARRANGEMENT strategy — a coarse hint over the four
/// canonical buckets. `auto` picks by shape; `focus_3d` forces the
/// hero-beside-sidebar shape; `grid` lays every view out in an even grid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComposeStrategy {
    /// Pick by shape: a hero-beside-sidebar split when a 3D scene exists, else a
    /// grid.
    Auto,
    /// Force `Horizontal[ hero-3d (3) | Vertical(plots, images, status) (1.5) ]`
    /// (the go2_default generalized). Falls back to a grid when there is no 3D
    /// content to focus.
    Focus3d,
    /// An even grid of every view.
    Grid,
}

impl ComposeStrategy {
    /// Every variant, in the order an error lists them — the ONE place this set is
    /// enumerated. [`Self::WIRE_STRATEGIES`] is derived from it, [`Self::from_wire`]
    /// searches it, and [`ComposeError::UnknownStrategy`] lists it, so the compose
    /// vocabulary cannot drift between the parser and the message that teaches it.
    pub const VARIANTS: [Self; 3] = [Self::Auto, Self::Focus3d, Self::Grid];

    /// The accepted wire strings, in the order an error lists them.
    ///
    /// DERIVED from [`Self::as_str`], so the strings are spelled exactly once. These
    /// are user-facing vizd compose protocol and must not change; pinned byte-for-byte
    /// by `the_compose_wire_vocabulary_is_spelled_once_and_frozen`.
    pub const WIRE_STRATEGIES: [&'static str; 3] = [
        Self::VARIANTS[0].as_str(),
        Self::VARIANTS[1].as_str(),
        Self::VARIANTS[2].as_str(),
    ];

    /// The stable wire name (matches the accepted `strategy` string).
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::Focus3d => "focus_3d",
            Self::Grid => "grid",
        }
    }

    /// Resolve a wire `strategy` string, or a loud [`ComposeError::UnknownStrategy`]
    /// naming the supported set (never a silent drop / default).
    pub fn from_wire(strategy: &str) -> Result<Self, ComposeError> {
        Self::VARIANTS
            .into_iter()
            .find(|v| v.as_str() == strategy)
            .ok_or_else(|| ComposeError::UnknownStrategy(strategy.to_string()))
    }
}

/// A coarse ROLE hint on a group of topics — it names ONE of the four canonical
/// buckets (and thus one [`ViewKind`]). A group's role is OPTIONAL: a role-less
/// group falls to per-topic automagic (each topic lands in every view its
/// archetype renders in).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GroupRole {
    /// The hero 3D scene (`spatial3d`) — clouds, scans, transforms, the robot.
    Hero,
    /// The telemetry plots (`time_series`) — scalars / twists / odom+imu series.
    Plots,
    /// The image panel (`spatial2d`) — camera frames.
    Images,
    /// The status / text panel (`text_document`) — text logs + the field dump.
    Status,
}

impl GroupRole {
    /// Every variant, in the order an error lists them — the ONE place this set is
    /// enumerated. [`Self::WIRE_ROLES`] is derived from it, [`Self::from_wire`]
    /// searches it, and [`ComposeError::UnknownRole`] lists it.
    pub const VARIANTS: [Self; 4] = [Self::Hero, Self::Plots, Self::Images, Self::Status];

    /// The accepted wire strings, in the order an error lists them.
    ///
    /// DERIVED from [`Self::as_str`], so the strings are spelled exactly once. These
    /// are user-facing vizd compose protocol and must not change; pinned byte-for-byte
    /// by `the_compose_wire_vocabulary_is_spelled_once_and_frozen`.
    pub const WIRE_ROLES: [&'static str; 4] = [
        Self::VARIANTS[0].as_str(),
        Self::VARIANTS[1].as_str(),
        Self::VARIANTS[2].as_str(),
        Self::VARIANTS[3].as_str(),
    ];

    /// The stable wire name (matches the accepted `role` string).
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Hero => "hero",
            Self::Plots => "plots",
            Self::Images => "images",
            Self::Status => "status",
        }
    }

    /// The single [`ViewKind`] this role targets (the bucket its topics land in).
    pub fn view_kind(self) -> ViewKind {
        match self {
            Self::Hero => ViewKind::Spatial3d,
            Self::Plots => ViewKind::TimeSeries,
            Self::Images => ViewKind::Spatial2d,
            Self::Status => ViewKind::TextDocument,
        }
    }

    /// Resolve a wire `role` string, or a loud [`ComposeError::UnknownRole`]
    /// naming the supported set.
    pub fn from_wire(role: &str) -> Result<Self, ComposeError> {
        Self::VARIANTS
            .into_iter()
            .find(|v| v.as_str() == role)
            .ok_or_else(|| ComposeError::UnknownRole(role.to_string()))
    }
}

/// One GROUP of resolved topics + an optional role hint + an optional title (the
/// display name of the group's view). Bare-topics automagic is expressed as ONE
/// group with `role: None`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ComposeGroup {
    /// The resolved topics in this group (topic + render entity + archetype).
    pub topics: Vec<AttachedRender>,
    /// The coarse role hint (`None` = per-topic automagic within the group).
    pub role: Option<GroupRole>,
    /// The display name for the group's view (only honored on a ROLE group, whose
    /// bucket is unambiguous; a role-less group spans buckets, so its title is
    /// ignored).
    pub title: Option<String>,
}

/// The compiler INPUT: the groups (already resolved), the arrangement strategy,
/// and whether the robot model / tf skeleton rides in the hero 3D view.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ComposeInput {
    /// The groups, in declaration order.
    pub groups: Vec<ComposeGroup>,
    /// The requested arrangement strategy.
    pub strategy: ComposeStrategy,
    /// When true, the robot skeleton subtree (`ROBOT_ROOT`) is added to the hero
    /// 3D view's contents (so the robot always renders in the scene).
    pub include_robot: bool,
}

/// One PLACEMENT the compiler produced — the topic, its render entity, its
/// resolved archetype, and the exact view it was placed in. A dual-view topic
/// (Odometry / Imu) yields ONE placement PER view it lands in (spatial3d AND
/// time_series), so a placement is always `(topic, view)`-unique.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ComposedPlacement {
    /// The absolute topic.
    pub topic: String,
    /// The render entity path the topic's frames log under.
    pub entity: String,
    /// The resolved archetype (`None` for a still-silent topic placed by role).
    pub archetype: Option<ArchetypeKind>,
    /// The view kind this placement lands in.
    pub view: ViewKind,
    /// `true` iff this is an EXPLICIT (role-tagged) placement of a topic
    /// whose `producer_count` is `Some(0)` — a registered-but-DEAD route the user
    /// (agent) named directly, so user intent WINS and it IS placed, but the response
    /// flags it so the caller can tell the user the pane will stay empty until the
    /// topic publishes. Always `false` in the AUTOMAGIC path (dead topics are skipped
    /// there, never placed) and for live/unknown topics.
    pub dead: bool,
}

/// The applied top-level ARRANGEMENT shape — reported back so the agent sees what
/// the strategy resolved to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Arrangement {
    /// `Horizontal[ hero-3d (3) | Vertical(plots, images, status) (1.5) ]`.
    HeroSidebar,
    /// An even grid of every view.
    Grid,
    /// A single view (only one bucket was non-empty).
    Single,
}

impl Arrangement {
    /// The stable wire name reported in the response's `arrangement` field.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::HeroSidebar => "hero_sidebar",
            Self::Grid => "grid",
            Self::Single => "single",
        }
    }
}

/// One topic the AUTOMAGIC (role-less) compose path SKIPPED because it is a
/// registered-but-DEAD route (`producer_count == Some(0)` — nothing is publishing).
/// Surfaced EXPLICITLY (never a silent drop) so the caller can tell the user which
/// topics were left out and why.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SkippedTopic {
    /// The absolute topic that was skipped.
    pub topic: String,
    /// A human-readable reason (e.g. "never published — 0 producers").
    pub reason: String,
}

/// The compiler OUTPUT: the emit-ready plan, the placement echo, the applied
/// arrangement, and any soft warnings (silent-topic + not-placed hints).
#[derive(Debug, Clone, PartialEq)]
pub struct ComposedLayout {
    /// The validated, emit-ready blueprint plan (`auto_views` always FALSE).
    pub plan: BlueprintPlan,
    /// One entry per `(topic, view)` placement, in placement order.
    pub placements: Vec<ComposedPlacement>,
    /// The applied top-level arrangement shape.
    pub arrangement: Arrangement,
    /// Soft, non-fatal hints (a silent topic placed by role; a silent role-less
    /// topic that could not be placed). Never an error.
    pub warnings: Vec<String>,
    /// Topics the AUTOMAGIC (role-less) path SKIPPED because they are
    /// registered-but-DEAD routes (`producer_count == Some(0)`), each with a human
    /// reason. Empty when nothing was skipped. An EXPLICIT (role-tagged) dead topic
    /// is NOT here — it is placed and flagged [`ComposedPlacement::dead`] instead.
    pub skipped_dead: Vec<SkippedTopic>,
}

/// A layout-compilation failure — the loud, actionable errors the `compose_layout`
/// verb surfaces VERBATIM. Every arm names the fixable problem; nothing is a
/// silent drop.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ComposeError {
    /// A `strategy` string that is not one of [`ComposeStrategy::WIRE_STRATEGIES`].
    #[error(
        "unknown strategy '{0}' — supported strategies: {supported}",
        supported = ComposeStrategy::WIRE_STRATEGIES.join(", ")
    )]
    UnknownStrategy(String),
    /// A `role` string that is not one of [`GroupRole::WIRE_ROLES`].
    #[error(
        "unknown group role '{0}' — supported roles: {supported}",
        supported = GroupRole::WIRE_ROLES.join(", ")
    )]
    UnknownRole(String),
    /// A topic whose resolved view kinds are INCOMPATIBLE with its group's role —
    /// the role targets a view kind the topic never renders in, so placing it
    /// there would render an empty pane. Names the topic, its real view kinds, and
    /// the fix.
    #[error(
        "topic '{topic}' ({archetype} → renders only in {real_views}) is incompatible with the \
         '{role}' group (a {role_view} view) — move it to a group whose role matches (one of: \
         {real_views}), or drop the role so it is placed automatically"
    )]
    RoleIncompatible {
        /// The offending topic.
        topic: String,
        /// The topic's resolved archetype wire name.
        archetype: String,
        /// The group's role wire name.
        role: &'static str,
        /// The view kind the role targets.
        role_view: &'static str,
        /// The view kinds the topic ACTUALLY renders in (comma-joined).
        real_views: String,
    },
    /// The compile produced NO placements — none of the requested topics resolved
    /// to a renderable archetype (all silent / undecodable, or the topic set was
    /// empty). Names the count + the still-silent topics so the agent can wait for
    /// data or fix the names.
    #[error(
        "compose_layout produced no views — none of the {requested} requested topic(s) resolved to \
         a renderable archetype (still-silent: {}); attach topics with data flowing first, or \
         check the topic names",
        if .silent.is_empty() { "none".to_string() } else { .silent.join(", ") }
    )]
    NoRenderableTopics {
        /// The number of requested topics.
        requested: usize,
        /// The topics that were attached but still silent (no resolved archetype).
        silent: Vec<String>,
    },
    /// An AUTOMAGIC (role-less) compose whose EVERY non-placed topic was a
    /// registered-but-DEAD route (`producer_count == Some(0)`), leaving nothing to
    /// place and no explicitly-named (role-tagged) topic to fall back on. Names the
    /// dead topics + the fix, so the agent never silently returns an empty layout.
    /// Used ONLY when there are no still-silent topics (a mixed set is the accurate
    /// [`ComposeError::DeadAndSilentTopics`] instead — the "all dead" claim must be true).
    #[error(
        "compose_layout produced no views — every requested topic is a registered-but-DEAD route \
         (nothing is publishing): {}; wait for data to flow, check the topic names, or place a \
         topic in a role-tagged group to lay it out anyway",
        .dead.join(", ")
    )]
    AllTopicsDead {
        /// The dead topics (each `producer_count == Some(0)`), in request order.
        dead: Vec<String>,
    },
    /// An AUTOMAGIC (role-less) compose that placed NOTHING
    /// because its non-placed topics are a MIX of registered-but-DEAD routes AND
    /// attached-but-still-SILENT topics (LIVE or unknown liveness, but not yet typed —
    /// e.g. a just-started publisher whose first frame the bounded resolve peek has not
    /// caught yet). Names BOTH sets ACCURATELY — a live-but-silent topic is NOT called
    /// dead (its producer count is > 0 / unknown), and it carries the "renders once it
    /// publishes" hint the pure-silent path gives. The precise cousin of
    /// [`ComposeError::AllTopicsDead`] (all dead) and [`ComposeError::NoRenderableTopics`]
    /// (all silent).
    #[error(
        "compose_layout produced no views — the requested topics are all either DEAD routes \
         (nothing is publishing: {}) or attached-but-still-silent (no data yet: {}); wait for the \
         silent topic(s) to publish then re-run, check the topic names, or place a topic in a \
         role-tagged group to lay it out anyway",
        .dead.join(", "),
        .silent.join(", ")
    )]
    DeadAndSilentTopics {
        /// The registered-but-DEAD topics (each `producer_count == Some(0)`), in order.
        dead: Vec<String>,
        /// The attached-but-still-silent topics (live/unknown liveness, no archetype yet).
        silent: Vec<String>,
    },
}

/// The four canonical buckets, in the fixed arrangement order.
const HERO: usize = 0;
const PLOTS: usize = 1;
const IMAGES: usize = 2;
const STATUS: usize = 3;

/// The rerun view-contents query expression for an entity subtree: a leading-slash
/// `/<entity>/**` glob (rerun's canonical form — `/foo/**` matches `foo` AND its
/// descendants, so the cloud's `.../lidar/viz-sweep/k` sub-entities are included).
fn content_glob(entity: &str) -> String {
    format!("/{}/**", entity.trim_matches('/'))
}

/// The EXCLUSION form of [`content_glob`] — rerun's `QueryExpression` treats a
/// leading `-` as "remove this subtree from the view's contents".
fn excluded_glob(entity: &str) -> String {
    format!("-/{}/**", entity.trim_matches('/'))
}

/// The contents for a view that should show `entity`'s subtree and
/// NOTHING BELONGING TO ANOTHER TOPIC — the include glob, minus one exclusion per
/// `attached` entity that is a STRICT DESCENDANT of it.
///
/// Under the earlier flat scheme no topic entity was ever an ancestor of
/// another (they were all siblings under `world/odom/base`), so a subtree glob was
/// safe. Topic-shaped paths change that: the canonical ROS `image_transport`
/// family gives `world/camera/image_raw` a child `world/camera/image_raw/compressed`,
/// and `/camera/image_raw`'s own view would silently ALSO draw the compressed
/// stream. (The Go2 has zero such pairs; the family is a real class, not a
/// hypothetical.)
///
/// Exclusion rather than an exact-path expression, deliberately: an archetype's
/// OWN sub-entities must stay in — a cloud's geometry lives at
/// `<entity>/viz-sweep/{k}` and a path's waypoints at `<entity>/viz-vertices`, so an
/// exact-path view would render empty. Only paths that are themselves ATTACHED
/// TOPICS are removed.
fn subtree_contents_excluding_topics(entity: &str, attached: &[String]) -> Vec<String> {
    let mut out = vec![content_glob(entity)];
    let e = normalize_entity(entity);
    let prefix = format!("{e}/");
    for other in attached {
        let o = normalize_entity(other);
        if o.starts_with(&prefix) {
            out.push(excluded_glob(o));
        }
    }
    out.dedup();
    out
}

/// The display TITLE for a per-topic view: the TOPIC, never the entity's leaf
/// segment.
///
/// `entity_leaf("world/api/audiohub/response")` is `response` — and so is every
/// other `/api/*/response` topic's, so all 15 views were titled "response" in the
/// viewport. Fixing the entity paths alone would have left "check two topics, see
/// one" alive in the view tabs. The topic is what the user checked in the sidebar,
/// so it is what the view is called.
fn topic_view_title(topic: &str, default_name: &str) -> String {
    let t = topic.trim();
    if t.is_empty() || t == "/" {
        default_name.to_string()
    } else {
        t.to_string()
    }
}

/// Whether a content glob is the robot-skeleton subtree glob ([`ROBOT_ROOT`]) —
/// which grounds the URDF, not an attached topic, so guardrail 4 exempts
/// a contents list that is only the robot glob (an include_robot-only hero).
fn is_robot_glob(glob: &str) -> bool {
    glob_prefix(glob) == normalize_entity(ROBOT_ROOT)
}

/// Strip a content glob's `/<prefix>/**` wrapper back to `<prefix>` (the entity
/// subtree it targets). Tolerant: a glob without the trailing `/**` is returned
/// trimmed, so a non-standard contents entry still grounds by exact path.
fn glob_prefix(glob: &str) -> String {
    let g = glob.trim();
    let g = g.strip_suffix("/**").unwrap_or(g);
    g.trim_matches('/').to_string()
}

/// Whether a content glob (`/<prefix>/**`) grounds ANY of `entities` — an entity AT
/// or BELOW the glob's prefix subtree (the same strict-subtree relation
/// [`view_renders_entity`] uses for origins, keyed off a glob string). A world-root
/// or bare `**` glob grounds everything. Pure — the guardrail's oracle.
fn glob_grounds_any(glob: &str, entities: &[String]) -> bool {
    let prefix = glob_prefix(glob);
    let p = normalize_entity(&prefix);
    if p.is_empty() || p == normalize_entity(WORLD_ORIGIN) {
        return true;
    }
    entities.iter().any(|e| {
        let e = normalize_entity(e);
        e == p || e.starts_with(&format!("{p}/"))
    })
}

/// A per-bucket accumulator: the canonical view kind, the deduped topic content
/// globs + their entities (for the scoped sidebar origin), any extra globs (the
/// robot skeleton on the hero), and an optional title override.
#[derive(Default)]
struct Bucket {
    topic_globs: Vec<String>,
    topic_entities: Vec<String>,
    /// The TOPIC beside each entity in [`Self::topic_entities`] (same
    /// index), so a per-entity view can be TITLED by its topic rather than by the
    /// entity's leaf segment — which is `image_raw` for every camera on a robot
    /// that publishes `/cam1/image_raw` and `/cam2/image_raw`.
    topic_names: Vec<String>,
    extra_globs: Vec<String>,
    name: Option<String>,
}

impl Bucket {
    /// Add a topic's `<entity>/**` glob (deduped) + its entity (deduped) to this
    /// bucket.
    fn add(&mut self, topic: &AttachedRender) {
        let g = content_glob(&topic.entity);
        if !self.topic_globs.contains(&g) {
            self.topic_globs.push(g);
        }
        // An INTERLEAVED video topic contributes ONE ENTITY — its DEFAULT
        // rendition's child — not one per rendition and not the topic's own.
        // `build_image_views` builds a view per entity (N sibling cameras
        // must not share a 2D view), so feeding it the
        // chosen child yields exactly one pane through the existing mechanism.
        //
        // The topic entity would NOT do: its subtree holds every rendition, so a
        // topic-rooted view overlays them and one is invisible — the overlay
        // failure. One checked topic is one pane; the renditions the pane
        // does not show keep their entities and stay reachable in the entity tree.
        //
        // Below two renditions there is nothing to separate, so the topic entity is
        // used as before and the common case is byte-identical.
        if topic.video_renditions.len() >= 2 {
            if let Some(segment) = default_rendition(&topic.video_renditions) {
                let child = format!("{}/{}/{segment}", topic.entity, crate::video::VIDEO_CHILD);
                if !self.topic_entities.contains(&child) {
                    self.topic_entities.push(child);
                    // The rendition rides the NAME: the pane shows one of several
                    // streams the topic carries, so a bare topic title would claim
                    // it is the whole of it.
                    self.topic_names.push(format!("{} {segment}", topic.topic));
                }
                return;
            }
        }
        if !self.topic_entities.contains(&topic.entity) {
            self.topic_entities.push(topic.entity.clone());
            self.topic_names.push(topic.topic.clone());
        }
    }

    /// A bucket is empty (→ no view) when it grounds nothing (no topic AND no
    /// extra glob).
    fn is_empty(&self) -> bool {
        self.topic_globs.is_empty() && self.extra_globs.is_empty()
    }

    /// Build the bucket's [`PlanView`]. The hero roots at the world origin (the 3D
    /// coordinate frame); a sidebar view roots at its FIRST topic entity so the
    /// reused grounding check ([`validate_plan_against_attached`]) is precise (a
    /// world-root sidebar origin would ground — and G2-soft-warn — every attached
    /// topic). Contents are the union of the bucket's globs (+ extras).
    fn build_view(&self, kind: ViewKind, default_name: &str, is_hero: bool) -> PlanView {
        let mut contents = self.topic_globs.clone();
        contents.extend(self.extra_globs.iter().cloned());
        let origin = if is_hero {
            WORLD_ORIGIN.to_string()
        } else {
            self.topic_entities
                .first()
                .cloned()
                .unwrap_or_else(|| WORLD_ORIGIN.to_string())
        };
        PlanView {
            kind,
            name: Some(
                self.name
                    .clone()
                    .unwrap_or_else(|| default_name.to_string()),
            ),
            origin: Some(origin),
            contents: Some(contents),
        }
    }

    /// Build ONE `spatial2d` view PER image ENTITY. A rerun
    /// 0.34 spatial2d view anchors to a SINGLE `space_origin` coordinate space, so N
    /// sibling cameras must NOT share one 2D view (the wrist feed would overlay the
    /// front feed in the origin's space, or not project at all). Each image entity
    /// gets its own view rooted at — and grounding only — that entity's subtree
    /// (`<entity>/**`). A single image keeps the plain name (or the role title); with
    /// two or more, each name is suffixed with the entity's leaf segment so the panes
    /// are distinguishable in the viewport. `extra_globs` never apply here (only the
    /// hero carries the robot skeleton), so the images bucket is purely per-entity.
    fn build_image_views(&self, default_name: &str, attached: &[String]) -> Vec<PlanView> {
        let base = self
            .name
            .clone()
            .unwrap_or_else(|| default_name.to_string());
        let multi = self.topic_entities.len() > 1;
        self.topic_entities
            .iter()
            .zip(&self.topic_names)
            .map(|(entity, topic)| {
                // Disambiguate by TOPIC, not by the entity's leaf — two
                // cameras at `/cam1/image_raw` and `/cam2/image_raw` share the leaf
                // `image_raw`, so both panes were titled "Images: image_raw".
                let name = if multi {
                    format!("{base}: {topic}")
                } else {
                    base.clone()
                };
                PlanView {
                    kind: ViewKind::Spatial2d,
                    name: Some(name),
                    origin: Some(entity.clone()),
                    contents: Some(subtree_contents_excluding_topics(entity, attached)),
                }
            })
            .collect()
    }
}

/// The human reason recorded for a topic the automagic path skips as dead.
const DEAD_ROUTE_REASON: &str = "never published — 0 producers";

/// Whether a resolved topic is a registered-but-DEAD route — its probed
/// `producer_count` is EXACTLY `Some(0)` (a service exists / is known but nothing is
/// publishing). `None` (UNKNOWN — probe failed / not carried) and `Some(n)` (n > 0)
/// are BOTH live-equivalent: unknown liveness is never penalized (by design).
fn is_dead_route(topic: &AttachedRender) -> bool {
    topic.producer_count == Some(0)
}

/// Compile a semantic [`ComposeInput`] into an emit-ready
/// [`ComposedLayout`]. PURE over already-resolved topics — the caller
/// (`cerulion-vizd`) does the attach-if-missing + resolution + the emit, then
/// validates the plan through [`validate_plan_against_attached`] (reuse, never
/// reimplement). Grounding is BY CONSTRUCTION (a view is built only for a bucket
/// that grounds real topics), so that validation is a defense-in-depth safety net.
///
/// Each topic lands in the bucket(s) its archetype renders in ([`views_for_archetype`],
/// REUSED). A ROLE group pins its topics to the role's ONE bucket and REJECTS any
/// topic whose archetype cannot render there ([`ComposeError::RoleIncompatible`]);
/// a role-less group is per-topic automagic. `include_robot` adds the robot
/// skeleton subtree to the hero. `auto_views` is FALSE (so a Twist never explodes
/// into panels). An empty result is [`ComposeError::NoRenderableTopics`].
///
/// PREFER LIVE: a registered-but-DEAD route (probed `producer_count ==
/// Some(0)` — see `is_dead_route`) is handled by intent origin:
/// - In the AUTOMAGIC (role-less) path it is SKIPPED from placement and recorded in
///   [`ComposedLayout::skipped_dead`] with a human reason (never a silent drop), so a
///   dead route never consumes a hero/sidebar slot a live topic could use.
/// - In an EXPLICIT (role-tagged) group it is PLACED anyway (user intent wins) but
///   flagged [`ComposedPlacement::dead`] so the caller can warn the user.
/// - `None` (UNKNOWN) and `Some(n>0)` (LIVE) are never penalized.
///
/// When the automagic path places NOTHING, the empty result is classified ACCURATELY by
/// what the non-placed topics were: [`ComposeError::AllTopicsDead`] when EVERY one was a
/// dead route, [`ComposeError::DeadAndSilentTopics`] when the set MIXES dead routes and
/// live-but-still-silent topics (naming both, never a false "all dead" claim), or
/// [`ComposeError::NoRenderableTopics`] when all were silent — never a silent empty
/// layout, never a misleading universal-dead claim.
pub fn compose_layout(input: &ComposeInput) -> Result<ComposedLayout, ComposeError> {
    let mut buckets: [Bucket; 4] = Default::default();
    let mut placements: Vec<ComposedPlacement> = Vec::new();
    let mut warnings: Vec<String> = Vec::new();
    let mut silent: Vec<String> = Vec::new();
    let mut skipped_dead: Vec<SkippedTopic> = Vec::new();
    let requested: usize = input.groups.iter().map(|g| g.topics.len()).sum();

    for group in &input.groups {
        match group.role {
            // A ROLE group: every topic is pinned to the role's ONE bucket.
            Some(role) => {
                let target = role.view_kind();
                for topic in &group.topics {
                    match topic.archetype {
                        Some(arch) => {
                            // The views this TOPIC needs, choice applied —
                            // a role a forced dump satisfies must not be refused
                            // because the kind alone would not render there.
                            let views = views_for_render(topic);
                            if !views.contains(&target) {
                                return Err(ComposeError::RoleIncompatible {
                                    topic: topic.topic.clone(),
                                    archetype: archetype_wire_name(arch).to_string(),
                                    role: role.as_str(),
                                    role_view: target.as_str(),
                                    // The CHECK above asks
                                    // `views_for_render`, so the report must too —
                                    // an overridden topic refused for a role its
                                    // archetype could satisfy has to say WHY.
                                    real_views: {
                                        let (views, why) = renders_only_in_for(topic);
                                        format!("{views}{why}")
                                    },
                                });
                            }
                        }
                        // A still-silent topic can't be proven incompatible — place
                        // it by its declared role (lay-before-data), soft-warn it.
                        None => warnings.push(format!(
                            "topic '{}' is attached but still silent — placed in the '{}' view by \
                             its declared role; it renders once it publishes",
                            topic.topic,
                            role.as_str()
                        )),
                    }
                    // An EXPLICIT (role-tagged) topic is placed even when
                    // dead — user intent wins — but the placement is FLAGGED so the
                    // caller can tell the user the pane stays empty until it publishes.
                    let dead = is_dead_route(topic);
                    if dead {
                        warnings.push(format!(
                            "topic '{}' is placed in the '{}' view by its declared role but is a \
                             registered-but-DEAD route ({}) — the pane stays empty until it \
                             publishes",
                            topic.topic,
                            role.as_str(),
                            DEAD_ROUTE_REASON
                        ));
                    }
                    buckets[bucket_index(target)].add(topic);
                    placements.push(ComposedPlacement {
                        topic: topic.topic.clone(),
                        entity: topic.entity.clone(),
                        archetype: topic.archetype,
                        view: target,
                        dead,
                    });
                }
                // A role group's title names its (unambiguous) bucket's view.
                if let Some(title) = &group.title {
                    let b = &mut buckets[bucket_index(target)];
                    if b.name.is_none() {
                        b.name = Some(title.clone());
                    }
                }
            }
            // A role-LESS group: per-topic automagic (each topic → every view it
            // renders in).
            None => {
                for topic in &group.topics {
                    // A DEAD route (Some(0)) is SKIPPED from automagic
                    // placement — it never consumes a slot a live topic could use —
                    // and recorded (never a silent drop). The dead check precedes the
                    // archetype match so a topic that is dead AND still-silent reads as
                    // dead (the more actionable cause: it never published).
                    if is_dead_route(topic) {
                        skipped_dead.push(SkippedTopic {
                            topic: topic.topic.clone(),
                            reason: DEAD_ROUTE_REASON.to_string(),
                        });
                        continue;
                    }
                    match topic.archetype {
                        Some(arch) => {
                            for view in views_for_render(topic) {
                                buckets[bucket_index(view)].add(topic);
                                placements.push(ComposedPlacement {
                                    topic: topic.topic.clone(),
                                    entity: topic.entity.clone(),
                                    archetype: Some(arch),
                                    view,
                                    dead: false,
                                });
                            }
                        }
                        // A silent topic with NO role hint can't be typed → cannot
                        // be placed. Skip it with a loud hint (never fabricate a view).
                        None => {
                            silent.push(topic.topic.clone());
                            warnings.push(format!(
                                "topic '{}' is attached but still silent (no resolved archetype) — \
                                 not placed; re-run compose_layout once it publishes, or put it in a \
                                 group with an explicit role",
                                topic.topic
                            ));
                        }
                    }
                }
            }
        }
    }

    // include_robot: the robot model / tf skeleton rides in the hero 3D view.
    if input.include_robot {
        buckets[HERO].extra_globs.push(content_glob(ROBOT_ROOT));
    }

    // Nothing to render → a loud refusal (compose over zero renderable topics).
    // Classify the empty result ACCURATELY by what the
    // non-placed topics actually were — the "all dead" claim must be TRUE. The dead
    // and silent sets are independent (a live but not-yet-typed topic — the bounded
    // resolve peek has not caught its first frame — is SILENT, never dead), so:
    //   - dead-only (silent empty)  → AllTopicsDead (a true "everything is dead")
    //   - mixed (dead AND silent)   → DeadAndSilentTopics (names BOTH, never a false
    //                                 universal-dead claim, keeps the re-run hint)
    //   - silent-only (dead empty)  → NoRenderableTopics (the original all-silent path)
    if placements.is_empty() {
        let dead: Vec<String> = skipped_dead.into_iter().map(|s| s.topic).collect();
        return match (dead.is_empty(), silent.is_empty()) {
            (false, true) => Err(ComposeError::AllTopicsDead { dead }),
            (false, false) => Err(ComposeError::DeadAndSilentTopics { dead, silent }),
            (true, _) => Err(ComposeError::NoRenderableTopics { requested, silent }),
        };
    }

    // Every entity this compose knows about, so a per-entity image view
    // can EXCLUDE another attached topic living below it.
    let all_entities: Vec<String> = input
        .groups
        .iter()
        .flat_map(|g| g.topics.iter().map(|t| t.entity.clone()))
        .collect();

    // Build one view per non-empty bucket.
    let hero_view = (!buckets[HERO].is_empty())
        .then(|| buckets[HERO].build_view(ViewKind::Spatial3d, HERO_VIEW_NAME, true));
    let mut sidebar_views: Vec<PlanView> = Vec::new();
    if !buckets[PLOTS].is_empty() {
        sidebar_views.push(buckets[PLOTS].build_view(ViewKind::TimeSeries, PLOTS_VIEW_NAME, false));
    }
    if !buckets[IMAGES].is_empty() {
        // ONE spatial2d view PER image entity (never a
        // single origin-anchored view unioning every camera — they would overlay).
        sidebar_views.extend(buckets[IMAGES].build_image_views(IMAGES_VIEW_NAME, &all_entities));
    }
    if !buckets[STATUS].is_empty() {
        sidebar_views.push(buckets[STATUS].build_view(
            ViewKind::TextDocument,
            STATUS_VIEW_NAME,
            false,
        ));
    }

    // The hero-beside-sidebar shape is used when a hero exists AND the strategy
    // asks for it (focus_3d always; auto by shape). grid never uses it.
    let use_hero_sidebar = hero_view.is_some()
        && matches!(
            input.strategy,
            ComposeStrategy::Auto | ComposeStrategy::Focus3d
        );

    let (root, arrangement) = if use_hero_sidebar {
        let hero = hero_view.expect("hero present under use_hero_sidebar");
        if sidebar_views.is_empty() {
            // Only a 3D scene → a single view (rerun tabs-wraps it).
            (PlanNode::View(hero), Arrangement::Single)
        } else {
            let sidebar = collapse_or_vertical(sidebar_views);
            (
                PlanNode::Container(PlanContainer {
                    kind: ContainerKind::Horizontal,
                    children: vec![PlanNode::View(hero), sidebar],
                    name: None,
                    shares: Some(vec![HERO_SHARE, SIDEBAR_SHARE]),
                    columns: None,
                }),
                Arrangement::HeroSidebar,
            )
        }
    } else {
        // A grid (or, for a single view, that view directly) of every view.
        let mut all: Vec<PlanView> = hero_view.into_iter().collect();
        all.extend(sidebar_views);
        if all.len() == 1 {
            (
                PlanNode::View(all.into_iter().next().unwrap()),
                Arrangement::Single,
            )
        } else {
            let columns = grid_columns(all.len());
            (
                PlanNode::Container(PlanContainer {
                    kind: ContainerKind::Grid,
                    children: all.into_iter().map(PlanNode::View).collect(),
                    name: None,
                    shares: None,
                    columns: Some(columns),
                }),
                Arrangement::Grid,
            )
        }
    };

    Ok(ComposedLayout {
        plan: BlueprintPlan {
            root,
            // The Twist-6-panel-explosion fix: never let the viewer auto-create
            // views for the components the explicit views already cover.
            auto_views: false,
            // A compose product — its time_series views get the trailing
            // live-scope window, its spatial views the Studio stage background (emit
            // decorates by view kind; see `build_blueprint_msgs`).
            decorate: true,
        },
        placements,
        arrangement,
        warnings,
        skipped_dead,
    })
}

/// Decision: the dynamic consolidated default viewport layout vizd
/// RE-DERIVES + re-applies on EVERY attach/detach when NO explicit
/// `set_blueprint`/`compose_layout` is active — "click a thing → see it visualized
/// (in the Scene or as a plot on the side), uncheck → it goes away, no more tabs".
///
/// The viewer showed "Scene + 6 tabs" because the Go2 default ([`BlueprintPlan::go2_default`])
/// is Scene-ONLY with `auto_views: true`, so a plot-shaped attach (e.g. an Odometry
/// with 6 twist scalars) let rerun's `spawn_heuristic_views` explode ONE VIEW PER
/// SCALAR component (`.../angular/z`, `.../linear/z`, …), rendered as unarranged TABS.
/// This composes a GROUNDED plan instead, with `auto_views` FALSE (so the viewer never
/// re-explodes the components the explicit views cover) and `decorate` TRUE (plots get
/// the trailing live-scope window, the scene the Studio stage background):
///
/// - The primary "Scene" 3D hero, rooted at the world origin, grounds EXACTLY the
///   attached SPATIAL topics (Points3D / LaserScan / tf / poses …) via an EXPLICIT
///   union of their `<entity>/**` globs PLUS the robot skeleton glob (so the robot
///   model always renders). Because the scene grounds an explicit set, a DETACHED
///   spatial topic drops out of the union — it "goes away" from the Scene on the next
///   reflow (the underlying frames stay in the rerun store, but the view no longer
///   queries them). Attaching a spatial topic adds its glob → it appears immediately.
/// - One `time_series` view PER plot topic (all its scalar series as lines in ONE plot
///   with a legend, NEVER one-per-component), one `spatial2d` view per image topic,
///   one `text_document` view per status topic — each a per-topic view that appears on
///   attach and disappears on detach.
///
/// REFLOW RULE: `Horizontal[ Scene (HERO_SHARE=3) | sidebar (SIDEBAR_SHARE=1.5) ]` —
/// the Scene keeps ~2/3 of the width; the sidebar is a `Grid` of the per-topic views
/// (`ceil(sqrt(n))` columns) that SHRINKS its cells as more topics attach (rerun's Grid
/// tiles + shrinks — it NEVER tab-ifies overflow). A single sidebar view is placed
/// directly (no grid nesting). A Scene with no sidebar views is a lone Scene view.
///
/// Each topic lands in the view(s) its archetype renders in ([`views_for_archetype`],
/// REUSED) — a dual-view Odometry/Imu contributes its pose to the Scene AND gets its
/// own plot. Silent (unresolved) topics are skipped (never a fabricated view). When
/// NOTHING is renderable — the empty attach set (detach-all) OR only-silent topics —
/// this returns the exact [`BlueprintPlan::go2_default`] (the Scene-only
/// default state chosen as the detach-all target).
///
/// Grounded BY CONSTRUCTION, so the daemon applies it directly (never through the hard
/// `set_blueprint` guardrails). An explicit agent/user layout SUPERSEDES this until the
/// next attach/detach resets to the dynamic default (the daemon stops auto-applying it
/// once a `set_blueprint`/`compose_layout` verb runs — see the daemon's `LayoutMode`).
pub fn default_layout(attached: &[AttachedRender]) -> BlueprintPlan {
    default_layout_excluding(attached, &[])
}

/// [`default_layout`] plus the entities that were LOGGED EARLIER THIS RUN and are
/// no longer attached — the zombie class, closed for detach.
///
/// Exclusions used to be computed from the CURRENTLY-attached set only, and nothing
/// in vizd ever clears a detached topic's entity from the rerun store (the uncheck
/// tombstone is the Studio shell's, and there are shell-free detach paths — the CLI
/// detaches only the taps it opened, so a foregrounded
/// `cerulion viz /camera/image_raw/compressed` that is Ctrl+C'd leaves a
/// still-attached `/camera/image_raw` whose view has just LOST its
/// exclusion). Recomposing then swept the detached topic's last-logged frame back
/// into its ancestor topic's view: a topic the user just removed, drawn under a
/// DIFFERENT topic's title — "check two topics, see one", now triggered by a detach.
///
/// The relation is NEW with per-topic entities: under the earlier flat scheme every topic
/// entity was a sibling under `world/odom/base`, so no topic could be an ancestor of
/// another. `world/<topic>` makes the canonical `image_transport` pair
/// (`/camera/image_raw` + `/camera/image_raw/compressed`, both renderable) exactly
/// that. `ever_logged` is additive — passing it can only ever REMOVE another topic's
/// data from a view, never hide the view's own subtree (only STRICT descendants are
/// excluded), so a stale entry is harmless.
/// The DEFAULT rendition of a multi-rendition video topic — the one its
/// single pane shows.
///
/// The pixel count decides, so the answer does not depend on the order the
/// segments arrive in. That independence is the point rather than a refinement:
/// the segments reach the layout as OPAQUE `WxH` strings across a thread
/// boundary ([`AttachedRender::video_renditions`]), and the Go2's own pair is
/// exactly where the cheap proxies break — `"1280x720" < "640x360"`
/// lexicographically, so "take the last" is correct today only because
/// [`crate::video::StreamKey`]'s derived `Ord` happens to sort a `BTreeMap` by
/// `(width, height)` numerically. Reading the numbers here means a producer-side
/// re-ordering can never silently demote the operator's camera to its thumbnail.
///
/// Ties (equal pixel counts, e.g. `1920x1080` vs `1080x1920`) and unparseable
/// segments both fall back to the segment STRING, so the choice is total and
/// deterministic on any input — a layout that flickered between renditions run to
/// run would be worse than either answer.
fn default_rendition(segments: &[String]) -> Option<&String> {
    segments.iter().max_by_key(|segment| {
        let pixels = segment
            .split_once('x')
            .and_then(|(w, h)| Some((w.parse::<u64>().ok()?, h.parse::<u64>().ok()?)))
            .map(|(w, h)| w * h);
        // An unparseable segment sorts BELOW every real resolution rather than
        // above it: it is not evidence of a bigger picture, and letting one win
        // would hand the pane to the segment we understand least.
        (pixels.unwrap_or(0), segment.as_str())
    })
}

/// The single `spatial2d` view of a multi-rendition video topic, shared
/// by BOTH layout producers.
///
/// `Some(view)` — rooted at the DEFAULT rendition's child entity — when `topic`
/// is a video topic that has shown TWO OR MORE renditions and this is its
/// `spatial2d` slot. `None` in every other case, which keeps the caller's
/// existing single-view path intact: a non-video topic, a video topic with one
/// rendition (the common case — its lone child is inside the topic-rooted view's
/// subtree already), or a non-2D slot.
///
/// The video path first gave each rendition its OWN view, because a topic-rooted view draws
/// sibling child entities into one pane and one of them is invisible. That is
/// still true, and it is still why the ENTITIES stay split — but a user reads
/// the result as the same camera checked twice, which is the more
/// expensive mistake: one checked topic is one pane. So the other renditions keep
/// their entities and stay reachable in the entity tree; they are simply not
/// auto-tiled.
///
/// Rooting at the child rather than at the topic is what makes ONE pane correct:
/// a topic-rooted view would overlay both renditions again, which is the exact
/// failure per-rendition views exist to prevent.
fn default_rendition_view(
    topic: &AttachedRender,
    view: ViewKind,
    default_name: &str,
) -> Option<PlanView> {
    if view != ViewKind::Spatial2d || topic.video_renditions.len() < 2 {
        return None;
    }
    let segment = default_rendition(&topic.video_renditions)?;
    Some(PlanView {
        kind: view,
        // The rendition rides the TITLE even though there is now one pane: the
        // operator is looking at one of several streams the topic carries, and a
        // bare topic title would claim it is the whole of it.
        name: Some(format!(
            "{} {segment}",
            topic_view_title(&topic.topic, default_name)
        )),
        origin: Some(format!(
            "{}/{}/{segment}",
            topic.entity,
            crate::video::VIDEO_CHILD
        )),
        contents: None,
    })
}

pub fn default_layout_excluding(
    attached: &[AttachedRender],
    ever_logged: &[String],
) -> BlueprintPlan {
    // Per-topic sidebar views (plots, images, status) in attach order, and the EXPLICIT
    // union of attached spatial3d entity globs for the Scene hero (so a detach reflows
    // the spatial topic out of the Scene). A dual-view topic feeds BOTH.
    let mut sidebar: Vec<PlanView> = Vec::new();
    let mut scene_globs: Vec<String> = Vec::new();
    // Every attached entity, so a per-topic view can EXCLUDE any other
    // attached topic living below it (see `subtree_contents_excluding_topics`).
    // Includes still-silent topics: they get no view of their own, but their data
    // must not be swept into an ancestor topic's view either.
    let attached_entities: Vec<String> = attached.iter().map(|t| t.entity.clone()).collect();
    // …plus every entity logged EARLIER this run whose data is still in the store.
    let mut excludable: Vec<String> = attached_entities.clone();
    for e in ever_logged {
        if !excludable.contains(e) {
            excludable.push(e.clone());
        }
    }
    for topic in attached {
        if topic.archetype.is_none() {
            continue; // still-silent → skip (never fabricate a view)
        }
        for view in views_for_render(topic) {
            let default_name = match view {
                ViewKind::Spatial3d => {
                    // Fold into the Scene hero's explicit contents (deduped).
                    let g = content_glob(&topic.entity);
                    if !scene_globs.contains(&g) {
                        scene_globs.push(g);
                    }
                    continue;
                }
                ViewKind::TimeSeries => PLOTS_VIEW_NAME,
                ViewKind::Spatial2d => IMAGES_VIEW_NAME,
                ViewKind::TextDocument => STATUS_VIEW_NAME,
            };
            // An INTERLEAVED video topic gets ONE view, rooted at its
            // DEFAULT rendition's child entity. The renditions live at sibling
            // child entities, so a topic-rooted view would draw both into one pane
            // and one would be invisible — rooting at the chosen child
            // shows exactly one, and the others stay reachable in the entity tree.
            if let Some(view) = default_rendition_view(topic, view, default_name) {
                sidebar.push(view);
                continue;
            }
            sidebar.push(PlanView {
                kind: view,
                // Name each per-topic view by its TOPIC, so the plots and
                // images are distinguishable in the viewport (the "which
                // topic?" bar). It used to be the entity's LEAF segment, which is
                // `response` for all 15 `/api/*/response` topics — "check two
                // topics, see one" survived in the view tabs even after the entity
                // paths were fixed.
                name: Some(topic_view_title(&topic.topic, default_name)),
                origin: Some(topic.entity.clone()),
                // A topic whose entity is an ANCESTOR of another attached
                // topic's must not silently draw that topic's data too.
                contents: Some(subtree_contents_excluding_topics(
                    &topic.entity,
                    &excludable,
                )),
            });
        }
    }

    // Nothing renderable (detach-all, or only-silent topics) → the exact
    // Scene-only default state (the chosen detach-all target).
    if scene_globs.is_empty() && sidebar.is_empty() {
        return BlueprintPlan::go2_default();
    }

    // The primary Scene: the attached spatial topics + the robot skeleton (always, so
    // the robot model persists across every reflow). An explicit-union grounding is
    // what lets a detached spatial topic reflow OUT of the Scene.
    // …and the Scene needs the SAME treatment: its union of per-topic subtree globs
    // keeps drawing a DETACHED descendant of a still-attached spatial topic, which
    // contradicts this module's own contract that "a DETACHED spatial topic drops out
    // of the union — it 'goes away' from the Scene on the next reflow".
    for e in &excludable {
        let e = normalize_entity(e);
        let already_included = scene_globs.iter().any(|g| glob_prefix(g) == e);
        if already_included {
            continue;
        }
        if scene_globs
            .iter()
            .any(|g| e.starts_with(&format!("{}/", glob_prefix(g))))
        {
            scene_globs.push(excluded_glob(e));
        }
    }
    scene_globs.push(content_glob(ROBOT_ROOT));
    let hero = PlanView {
        kind: ViewKind::Spatial3d,
        name: Some(DEFAULT_SCENE_NAME.to_string()),
        origin: Some(WORLD_ORIGIN.to_string()),
        contents: Some(scene_globs),
    };

    let root = if sidebar.is_empty() {
        // Only spatial topics → a lone Scene view (no plots to tile).
        PlanNode::View(hero)
    } else {
        // Scene primary (large) beside a GRID of the per-topic views (tiled + shrinking,
        // NEVER tabs). HERO_SHARE:SIDEBAR_SHARE keeps the Scene the dominant column.
        PlanNode::Container(PlanContainer {
            kind: ContainerKind::Horizontal,
            children: vec![PlanNode::View(hero), grid_or_single(sidebar)],
            name: None,
            shares: Some(vec![HERO_SHARE, SIDEBAR_SHARE]),
            columns: None,
        })
    };

    BlueprintPlan {
        root,
        auto_views: false,
        decorate: true,
    }
}

/// Wrap `views` in an even `Grid` container (tiled — NEVER tabs), OR return the single
/// view directly when there is only one. The grid's column count is [`grid_columns`]
/// (`ceil(sqrt(n))`), the same even tiling the `grid` compose strategy uses.
fn grid_or_single(views: Vec<PlanView>) -> PlanNode {
    if views.len() == 1 {
        PlanNode::View(views.into_iter().next().unwrap())
    } else {
        let columns = grid_columns(views.len());
        PlanNode::Container(PlanContainer {
            kind: ContainerKind::Grid,
            children: views.into_iter().map(PlanNode::View).collect(),
            name: None,
            shares: None,
            columns: Some(columns),
        })
    }
}

/// Map a canonical view kind to its bucket index (the four buckets are one per
/// distinct view kind).
fn bucket_index(view: ViewKind) -> usize {
    match view {
        ViewKind::Spatial3d => HERO,
        ViewKind::TimeSeries => PLOTS,
        ViewKind::Spatial2d => IMAGES,
        ViewKind::TextDocument => STATUS,
    }
}

/// Wrap `views` in a `Vertical` container, OR return the single view directly when
/// there is only one (a Vertical of one child is valid but needlessly nested).
fn collapse_or_vertical(views: Vec<PlanView>) -> PlanNode {
    if views.len() == 1 {
        PlanNode::View(views.into_iter().next().unwrap())
    } else {
        PlanNode::Container(PlanContainer {
            kind: ContainerKind::Vertical,
            children: views.into_iter().map(PlanNode::View).collect(),
            name: None,
            shares: None,
            columns: None,
        })
    }
}

/// An even grid's column count for `n` views: `ceil(sqrt(n))`, clamped to
/// `1..=max(n,1)` (so `PlanContainer`'s grid validation — `columns <= children` —
/// always passes). Total over `n` (n=0→1, guarding the `0.clamp(1,0)` panic — the
/// callers only pass `n >= 2`, but the clamp stays sound for any input). n=1→1, 2→2,
/// 3→2, 4→2, 5→3, 9→3.
fn grid_columns(n: usize) -> u32 {
    let cols = (n as f64).sqrt().ceil() as usize;
    cols.clamp(1, n.max(1)) as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blueprint_constructs() {
        // Smoke: the container + view tree assembles without panicking (the
        // rendered layout is not assertable without a viewer).
        let _blueprint = go2_blueprint();
    }

    #[test]
    fn send_once_guard_fires_once_then_rearms() {
        // This is the only test touching BLUEPRINT_SENT *directly*, but
        // NOT the only one that reaches it — `ensure_setup` swaps it on the
        // PRODUCTION path (:159), `worker.rs` calls that at :679/:782/:836, and
        // `worker.rs`'s test module spawns a `VizLogWorker` at 8 sites in THIS
        // same lib test binary. This comment used to claim "no intra-binary
        // race"; that was false, and only libtest's dispatch ORDER was keeping
        // it true in practice.
        //
        // Now ENFORCED: the crate-level lock serializes this test against those
        // eight. Taken FIRST so it is dropped LAST (see the lock's docs — drop
        // order is load-bearing).
        let _statics = crate::test_support::blueprint_statics_guard();
        rearm_blueprint();
        assert!(
            !BLUEPRINT_SENT.swap(true, Ordering::SeqCst),
            "a fresh guard yields false to the first caller"
        );
        assert!(
            BLUEPRINT_SENT.swap(true, Ordering::SeqCst),
            "and true thereafter (send happens once)"
        );
        rearm_blueprint();
        assert!(
            !BLUEPRINT_SENT.swap(true, Ordering::SeqCst),
            "reset re-arms the guard"
        );
        rearm_blueprint();
    }

    // ── Runtime layout: kinds + validation-error resolution ─────────────────

    #[test]
    fn view_kind_from_wire_resolves_and_names_the_unknown() {
        assert_eq!(ViewKind::from_wire("spatial3d"), Ok(ViewKind::Spatial3d));
        assert_eq!(ViewKind::from_wire("spatial2d"), Ok(ViewKind::Spatial2d));
        assert_eq!(ViewKind::from_wire("time_series"), Ok(ViewKind::TimeSeries));
        assert_eq!(
            ViewKind::from_wire("text_document"),
            Ok(ViewKind::TextDocument)
        );
        // Round-trip: as_str is the exact accepted wire string.
        for k in [
            ViewKind::Spatial3d,
            ViewKind::Spatial2d,
            ViewKind::TimeSeries,
            ViewKind::TextDocument,
        ] {
            assert_eq!(ViewKind::from_wire(k.as_str()), Ok(k));
        }
        // Unknown kind → a loud error NAMING the offender + the supported set.
        let err = ViewKind::from_wire("map").expect_err("map is not exposed");
        assert_eq!(err, LayoutError::UnknownViewKind("map".to_string()));
        let msg = err.to_string();
        assert!(msg.contains("map"), "names the offender: {msg}");
        assert!(msg.contains("spatial3d"), "lists the supported set: {msg}");
    }

    /// The compose vocabulary is spelled ONCE and the wire strings are FROZEN.
    ///
    /// `WIRE_STRATEGIES` / `WIRE_ROLES` used to be a third, unread copy of a set that
    /// was also written out in `from_wire`'s match arms and again in the `ComposeError`
    /// text — three spellings, one of which nothing checked. They are now derived from
    /// `as_str`, and both `from_wire` and the error message read them, so a drift is
    /// no longer expressible.
    ///
    /// That makes it the whole vocabulary's single point of failure, so this arm pins
    /// it from the OUTSIDE: the expected literals below are written out by hand, NOT
    /// read from the consts, because these strings are user-facing vizd compose
    /// protocol and a rename would silently break every client that sends them.
    #[test]
    fn the_compose_wire_vocabulary_is_spelled_once_and_frozen() {
        // 1. The wire strings, byte for byte against hand-written literals.
        assert_eq!(
            ComposeStrategy::WIRE_STRATEGIES,
            ["auto", "focus_3d", "grid"],
            "compose `strategy` wire strings are protocol — changing one breaks clients"
        );
        assert_eq!(
            GroupRole::WIRE_ROLES,
            ["hero", "plots", "images", "status"],
            "compose `role` wire strings are protocol — changing one breaks clients"
        );

        // 2. The const and the variant list agree, index for index (the pairing the
        //    derivation rests on — a reordered VARIANTS would silently re-map names).
        assert_eq!(
            ComposeStrategy::VARIANTS.map(ComposeStrategy::as_str),
            ComposeStrategy::WIRE_STRATEGIES
        );
        assert_eq!(
            GroupRole::VARIANTS.map(GroupRole::as_str),
            GroupRole::WIRE_ROLES
        );

        // 3. Round-trip: every variant resolves from its own wire string, and the
        //    accepted set is EXACTLY the const (no extra spelling still accepted).
        for s in ComposeStrategy::VARIANTS {
            assert_eq!(ComposeStrategy::from_wire(s.as_str()), Ok(s));
        }
        for r in GroupRole::VARIANTS {
            assert_eq!(GroupRole::from_wire(r.as_str()), Ok(r));
        }

        // 4. The error message LISTS the const — pinned against the exact text the
        //    `compose_layout` verb surfaces verbatim, so driving it from the const did
        //    not change one byte of what an operator reads.
        let err = ComposeStrategy::from_wire("focus3d").expect_err("not a strategy");
        assert_eq!(
            err.to_string(),
            "unknown strategy 'focus3d' — supported strategies: auto, focus_3d, grid"
        );
        let err = GroupRole::from_wire("heroes").expect_err("not a role");
        assert_eq!(
            err.to_string(),
            "unknown group role 'heroes' — supported roles: hero, plots, images, status"
        );
    }

    #[test]
    fn container_kind_from_wire_resolves_and_names_the_unknown() {
        for k in [
            ContainerKind::Horizontal,
            ContainerKind::Vertical,
            ContainerKind::Grid,
            ContainerKind::Tabs,
        ] {
            assert_eq!(ContainerKind::from_wire(k.as_str()), Ok(k));
        }
        let err = ContainerKind::from_wire("stack").expect_err("stack is not a kind");
        assert_eq!(err, LayoutError::UnknownContainerKind("stack".to_string()));
        assert!(err.to_string().contains("horizontal"), "lists supported");
    }

    // ── The Go2 default as a plan (exact hand oracle) ───────────────────────

    #[test]
    fn go2_default_plan_matches_the_hand_oracle() {
        // Decision: the default scene is Scene-only (a single full-bleed
        // Spatial3D view) — the empty Telemetry + Status panels are DROPPED. Decoration
        // + auto_views stay on (the stage background rides the spatial view; auto_views
        // spawns the right view for a non-spatial attach, WITH data).
        let plan = BlueprintPlan::go2_default();
        assert!(
            plan.auto_views,
            "auto_views must stay ON — it renders non-spatial attaches"
        );
        assert_eq!(
            plan.view_count(),
            1,
            "Scene-only: a single 3D view, no empty panels"
        );
        let expected = BlueprintPlan {
            auto_views: true,
            decorate: true,
            root: PlanNode::View(PlanView {
                kind: ViewKind::Spatial3d,
                name: Some("Scene".to_string()),
                origin: Some("/world".to_string()),
                contents: None,
            }),
        };
        assert_eq!(plan, expected);
    }

    // ── Trailing plot window + stage background emission ──────────

    /// A compose-shaped plan (`decorate = true`) with ONE view of each kind under a
    /// grid — the emit fixture. NOT a real compose output (that path is covered in
    /// `layout_compose_test`); this isolates the emit's kind→property mapping.
    fn decorated_one_of_each_kind() -> BlueprintPlan {
        BlueprintPlan {
            auto_views: false,
            decorate: true,
            root: PlanNode::Container(PlanContainer {
                kind: ContainerKind::Grid,
                shares: None,
                columns: Some(2),
                name: None,
                children: vec![
                    vk(ViewKind::Spatial3d, "world"),
                    vk(ViewKind::TimeSeries, "world/plots"),
                    vk(ViewKind::Spatial2d, "world/cam"),
                    vk(ViewKind::TextDocument, "world/status"),
                ],
            }),
        }
    }

    /// The `decorate = false` control — structurally identical to
    /// [`decorated_one_of_each_kind`] (spatial + time_series views that WOULD be
    /// decorated under `decorate = true`) but hand-authored provenance, so the emit's
    /// `decorate` gate is exercised on a plan that is NOT the (now-decorated) go2
    /// default. Models a `set_blueprint` power-user plan (`build_plan` → `decorate:
    /// false`); the undecorated-emit tests use it now that the built-in default carries
    /// the stage background.
    fn undecorated_one_of_each_kind() -> BlueprintPlan {
        BlueprintPlan {
            decorate: false,
            ..decorated_one_of_each_kind()
        }
    }

    #[test]
    fn trailing_window_ranges_are_cursor_relative_minus_30s_to_0_on_both_timelines() {
        // Hand oracle for the SERIALIZED window values: [-30s, 0] CURSOR-RELATIVE (ns)
        // on robot_time + log_time. The whole VisibleTimeRange datatype is Eq, so this
        // pins the exact emitted values (not a self-compare).
        let expected_range = TimeRange {
            start: TimeRangeBoundary::CursorRelative(TimeInt(-30_000_000_000)),
            end: TimeRangeBoundary::CursorRelative(TimeInt(0)),
        };
        let expected = vec![
            VisibleTimeRange {
                timeline: "robot_time".into(),
                range: expected_range,
            },
            VisibleTimeRange {
                timeline: "log_time".into(),
                range: expected_range,
            },
        ];
        assert_eq!(trailing_window_ranges(TRAILING_WINDOW_SECS), expected);
        // The SINGLE shared range (used by the display-side TimeAxis view_range)
        // is the same cursor-relative [-30s, 0] window — one source, two archetypes.
        assert_eq!(trailing_window_range(TRAILING_WINDOW_SECS), expected_range);
        // The default window is 30 s; the stage color is #10161f (--cer-bg-stage).
        assert_eq!(TRAILING_WINDOW_SECS, 30);
        assert_eq!(STAGE_BACKGROUND_RGB, [0x10, 0x16, 0x1f]);
    }

    #[test]
    fn decorated_plan_emits_window_on_time_series_and_background_on_spatial_only() {
        let msgs = build_blueprint_msgs("f", &decorated_one_of_each_kind()).expect("emit");
        let paths = blueprint_property_paths(&msgs);

        let windows: Vec<&String> = paths
            .iter()
            .filter(|p| p.ends_with("/VisibleTimeRanges"))
            .collect();
        // `/TimeAxis` must not be swallowed by the `/VisibleTimeRanges` filter —
        // the two are distinct property paths on the same time_series view.
        let time_axes: Vec<&String> = paths.iter().filter(|p| p.ends_with("/TimeAxis")).collect();
        let backgrounds: Vec<&String> = paths
            .iter()
            .filter(|p| p.ends_with("/Background"))
            .collect();

        // Exactly the ONE time_series view carries a query window AND a display x-axis;
        // both spatial views (3D + 2D) carry a background; the text_document view carries
        // none of the three.
        assert_eq!(
            windows.len(),
            1,
            "one query window (the time_series view): {paths:?}"
        );
        assert_eq!(
            time_axes.len(),
            1,
            "one display x-axis window (the time_series view): {paths:?}"
        );
        assert_eq!(backgrounds.len(), 2, "two backgrounds (3D + 2D): {paths:?}");
        // Each property is logged onto the VIEW's own blueprint property path — the
        // EXACT path the viewer reads it from (view/<uuid>/<ArchetypeShortName>; the
        // EntityPath Display renders the canonical leading slash).
        let under_view = |p: &str| p.trim_start_matches('/').starts_with("view/");
        assert!(
            under_view(windows[0]),
            "window on a view property path: {}",
            windows[0]
        );
        assert!(
            under_view(time_axes[0]),
            "time axis on a view property path: {}",
            time_axes[0]
        );
        // The query window + display axis ride the SAME view (same view/<uuid> prefix).
        assert_eq!(
            windows[0].trim_end_matches("/VisibleTimeRanges"),
            time_axes[0].trim_end_matches("/TimeAxis"),
            "the query window and the display x-axis are on the same time_series view"
        );
        assert!(
            backgrounds.iter().all(|p| under_view(p)),
            "backgrounds on view property paths: {backgrounds:?}"
        );
    }

    #[test]
    fn decorated_plan_emits_solid_stage_background_and_cursor_relative_window_values() {
        // Decode the emitted COMPONENT VALUES (not just paths) and
        // pin them against hand oracles. This is the mutation-kill for the emitter's
        // output: flipping `BackgroundKind::SolidColor` → any other variant, or the
        // stage color / window bounds, fails HERE (path-presence asserts do not).
        let dec = blueprint_decorations(
            &build_blueprint_msgs("f", &decorated_one_of_each_kind()).expect("emit"),
        );

        // Both spatial views (3D + 2D) carry a SolidColor #10161f background.
        assert_eq!(
            dec.backgrounds.len(),
            2,
            "two backgrounds (3D + 2D): {dec:?}"
        );
        for bg in &dec.backgrounds {
            assert_eq!(
                bg.kinds,
                vec![BackgroundKind::SolidColor],
                "the stage background kind is SolidColor (mutation-kill): {bg:?}"
            );
            assert_eq!(
                bg.colors,
                vec![[0x10, 0x16, 0x1f, 0xff]],
                "the stage color is #10161f, opaque: {bg:?}"
            );
        }

        // Exactly the ONE time_series view carries the [-30s, 0] cursor-relative window
        // on both nanosecond timelines.
        assert_eq!(dec.windows.len(), 1, "one window: {dec:?}");
        let expected_range = TimeRange {
            start: TimeRangeBoundary::CursorRelative(TimeInt(-30_000_000_000)),
            end: TimeRangeBoundary::CursorRelative(TimeInt(0)),
        };
        assert_eq!(
            dec.windows[0].ranges,
            vec![
                VisibleTimeRange {
                    timeline: "robot_time".into(),
                    range: expected_range,
                },
                VisibleTimeRange {
                    timeline: "log_time".into(),
                    range: expected_range,
                },
            ],
            "cursor-relative [-30s, 0] on robot_time + log_time: {:?}",
            dec.windows[0]
        );

        // The empty-epoch-axis fix: exactly the ONE
        // time_series view carries a `TimeAxis:view_range` = the SAME [-30s, 0]
        // cursor-relative window, timeline-AGNOSTIC (exactly one range, no per-timeline
        // expansion — this is what the viewer resolves for the DISPLAY x-axis, taming
        // the empty plot's epoch axis). Flipping the bounds or dropping the emit fails
        // HERE (path presence alone would not catch a wrong value).
        assert_eq!(dec.time_axes.len(), 1, "one display x-axis: {dec:?}");
        assert_eq!(
            dec.time_axes[0].view_ranges,
            vec![expected_range],
            "TimeAxis:view_range is the single cursor-relative [-30s, 0] window: {:?}",
            dec.time_axes[0]
        );
        assert_eq!(
            dec.time_axes[0].cursor_relative_ns(),
            vec![(Some(-30_000_000_000), Some(0))],
            "the display window is cursor-relative [-30e9, 0] ns: {:?}",
            dec.time_axes[0]
        );
    }

    #[test]
    fn undecorated_plan_decodes_no_decoration_component_values() {
        // A hand-authored (decorate = false) plan emits NO decoration components at all
        // (the value-level twin of the path-presence `undecorated_*` test). The built-in
        // default now rides the decorated path, so the `decorate = false`
        // branch is now exercised via a set_blueprint-shaped control.
        let dec = blueprint_decorations(
            &build_blueprint_msgs("f", &undecorated_one_of_each_kind()).expect("emit"),
        );
        assert_eq!(dec, DecodedDecorations::default(), "no decoration: {dec:?}");
    }

    #[test]
    fn undecorated_plan_emits_no_window_and_no_background() {
        // A `set_blueprint` power-user plan is hand-authored provenance (decorate =
        // false) — NEVER auto-decorated (the "set_blueprint is theirs" contract, still
        // upheld by `build_plan` in the vizd daemon). The built-in default is NO LONGER
        // in this class (it is decorated now); the control is a set_blueprint-shaped
        // plan.
        let msgs = build_blueprint_msgs("f", &undecorated_one_of_each_kind()).expect("emit");
        let paths = blueprint_property_paths(&msgs);
        assert!(
            !paths.iter().any(|p| p.ends_with("/VisibleTimeRanges")),
            "no trailing window on a hand-authored plan: {paths:?}"
        );
        assert!(
            !paths.iter().any(|p| p.ends_with("/TimeAxis")),
            "no display x-axis window on a hand-authored plan: {paths:?}"
        );
        assert!(
            !paths.iter().any(|p| p.ends_with("/Background")),
            "no stage background on a hand-authored plan: {paths:?}"
        );
    }

    #[test]
    fn go2_default_emits_the_solid_stage_background_on_its_spatial_scene() {
        // By design, the built-in default/empty scene must show the
        // Studio stage background, not rerun's off-brand `GradientDark` green. Decode
        // the emitted COMPONENT VALUES (the mutation-kill — path presence alone would
        // pass a wrong-color regression) and pin the go2 default's ONE spatial (3D)
        // scene to the SolidColor #10161f stage background, matching the composed path's
        // `decorated_plan_emits_solid_stage_background_*` oracle.
        let dec = blueprint_decorations(
            &build_blueprint_msgs("go2", &BlueprintPlan::go2_default()).expect("emit"),
        );
        assert_eq!(
            dec.backgrounds.len(),
            1,
            "the go2 default has exactly one spatial (3D) scene → one background: {dec:?}"
        );
        let bg = &dec.backgrounds[0];
        assert_eq!(
            bg.kinds,
            vec![BackgroundKind::SolidColor],
            "the default stage background kind is SolidColor (mutation-kill): {bg:?}"
        );
        assert_eq!(
            bg.colors,
            vec![[0x10, 0x16, 0x1f, 0xff]],
            "the default stage color is #10161f, opaque — the shared STAGE_BACKGROUND_RGB: {bg:?}"
        );
    }

    #[test]
    fn go2_default_is_scene_only_and_emits_no_telemetry_decorations() {
        // Decision ("if a plot with no data is gonna default to 1970, either
        // fix it or drop the panel"): the default scene DROPS the empty Telemetry +
        // Status panels, so it emits NO `TimeSeries`/`TextDocument` views — the empty
        // 1970-era epoch axis is now STRUCTURALLY IMPOSSIBLE (there is no empty plot).
        // Decode the emitted chunks and assert the Scene-only shape: exactly one spatial
        // Background (pinned by value in the sibling test) and ZERO telemetry decorations
        // (no `/VisibleTimeRanges`, no `/TimeAxis`). Those live-scope decorations ride
        // ONLY the compose path's time_series views (pinned by the layout_compose +
        // `decorated_plan_emits_*` tests), where the plots carry real data.
        let msgs = build_blueprint_msgs("scene_only", &BlueprintPlan::go2_default()).expect("emit");
        let dec = blueprint_decorations(&msgs);
        assert_eq!(
            dec.backgrounds.len(),
            1,
            "Scene-only: exactly one spatial (3D) scene background: {dec:?}"
        );
        assert!(
            dec.windows.is_empty(),
            "Scene-only: no telemetry query window (no time_series view): {dec:?}"
        );
        assert!(
            dec.time_axes.is_empty(),
            "Scene-only: no display x-axis window (no time_series view → no empty 1970 axis): {dec:?}"
        );
        // And structurally: exactly ONE leaf view, and it is the 3D scene.
        let paths = blueprint_property_paths(&msgs);
        assert!(
            !paths.iter().any(|p| p.ends_with("/VisibleTimeRanges")),
            "no telemetry window path on the Scene-only default: {paths:?}"
        );
        assert!(
            !paths.iter().any(|p| p.ends_with("/TimeAxis")),
            "no telemetry axis path on the Scene-only default: {paths:?}"
        );
        assert_eq!(
            BlueprintPlan::go2_default().view_count(),
            1,
            "Scene-only: a single 3D view"
        );
    }

    #[test]
    fn timeline_pin_is_first_send_only_pin_true_emits_it_false_omits_it() {
        // The `time_panel` timeline pin (log_time) rides ONLY the pinning
        // emit (the boot/first send, `pin_timeline = true` — the public
        // `build_blueprint_msgs`), so a runtime RE-APPLY (`pin_timeline = false`, the
        // `apply_runtime_blueprint` path) never re-pins + snaps a user's manual timeline
        // choice back. Decode the emitted paths for BOTH gates over the SAME plan.
        let plan = BlueprintPlan::go2_default();
        let pinned = blueprint_property_paths(
            &build_blueprint_msgs_inner("cer47_pin", &plan, true).expect("emit"),
        );
        assert_eq!(
            pinned
                .iter()
                .filter(|p| p.trim_start_matches('/') == "time_panel")
                .count(),
            1,
            "pin_timeline=true emits EXACTLY one time_panel chunk (the log_time pin): {pinned:?}"
        );
        // And it pins log_time, not some other timeline (the public path agrees).
        assert_eq!(
            blueprint_panel_timeline(&build_blueprint_msgs("cer47_pin", &plan).expect("emit"))
                .as_deref(),
            Some(DEFAULT_ACTIVE_TIMELINE),
            "the pinned timeline is the wall-clock default"
        );
        let unpinned = blueprint_property_paths(
            &build_blueprint_msgs_inner("cer47_nopin", &plan, false).expect("emit"),
        );
        assert!(
            !unpinned
                .iter()
                .any(|p| p.trim_start_matches('/') == "time_panel"),
            "pin_timeline=false emits NO time_panel chunk (a re-apply never re-pins): {unpinned:?}"
        );
    }

    #[test]
    fn single_view_root_emits_the_window_and_wraps_in_a_tabs_container() {
        // A decorated single-time_series-view plan: the emit wraps the lone view in a
        // synthetic Tabs container (rerun's Blueprint::new behavior) AND still stamps
        // the window on the view's property path.
        let plan = BlueprintPlan {
            auto_views: false,
            decorate: true,
            root: vk(ViewKind::TimeSeries, "world/plots"),
        };
        let paths = blueprint_property_paths(&build_blueprint_msgs("f", &plan).expect("emit"));
        assert_eq!(
            paths
                .iter()
                .filter(|p| p.ends_with("/VisibleTimeRanges"))
                .count(),
            1,
            "the single view still carries a window: {paths:?}"
        );
        assert!(
            paths
                .iter()
                .any(|p| p.trim_start_matches('/').starts_with("container/")),
            "the lone view is wrapped in a synthetic container: {paths:?}"
        );
    }

    #[test]
    fn build_blueprint_msgs_entity_paths_are_deterministic() {
        // Deterministic node ids ⇒ two emits of the same plan yield the SAME entity
        // paths in the SAME order (the raw LogMsg bytes differ only in rerun's inherent
        // per-row RowIds / store id, which no oracle depends on).
        let plan = decorated_one_of_each_kind();
        let a = blueprint_property_paths(&build_blueprint_msgs("f", &plan).expect("emit"));
        let b = blueprint_property_paths(&build_blueprint_msgs("f", &plan).expect("emit"));
        assert_eq!(a, b, "identical inputs ⇒ identical emitted entity paths");
    }

    // ── build_blueprint_msgs STRUCTURAL tests (decode of the LIVE emit path) ─
    //
    // The production emission path is `build_blueprint_msgs` (every send site calls
    // it via `send_plan`); the old SDK `plan_to_blueprint` assembler was DELETED
    // (it had ZERO production callers, so its "assembles + sends"
    // smoke tests guarded dead code while the LIVE hand-rolled container/viewport
    // mapping went unguarded). These tests decode the LIVE emitted chunks and assert
    // the container / viewport / view mapping against hand oracles.

    fn memory() -> rerun::RecordingStream {
        rerun::RecordingStreamBuilder::new("blueprint_test")
            .recording_id("layout")
            .memory()
            .expect("memory sink")
            .0
    }

    /// A decoded `ContainerBlueprint` chunk — the exact fields the live emit path
    /// writes (kind, name, per-axis shares, grid columns, child count + own path).
    #[derive(Debug, Clone, PartialEq)]
    struct DecodedContainer {
        kind: RrContainerKind,
        name: Option<String>,
        col_shares: Vec<f32>,
        row_shares: Vec<f32>,
        grid_columns: Option<u32>,
        children: usize,
        /// The container's own `container/<uuid>` path (leading slash trimmed).
        path: String,
    }

    /// A decoded `ViewBlueprint` (+ its sibling `ViewContents`) — class identifier,
    /// display name, origin, and the resolved query globs.
    #[derive(Debug, Clone, PartialEq)]
    struct DecodedView {
        class: String,
        name: Option<String>,
        origin: Option<String>,
        contents: Vec<String>,
        path: String,
    }

    fn chunks(msgs: &[LogMsg]) -> Vec<rerun::log::Chunk> {
        msgs.iter()
            .filter_map(|msg| match msg {
                LogMsg::ArrowMsg(_, arrow) => rerun::log::Chunk::from_arrow_msg(arrow).ok(),
                _ => None,
            })
            .collect()
    }

    fn norm(path: &str) -> String {
        path.trim_start_matches('/').to_string()
    }

    fn decode_containers(msgs: &[LogMsg]) -> Vec<DecodedContainer> {
        chunks(msgs)
            .iter()
            .filter_map(|chunk| {
                let path = norm(&chunk.entity_path().to_string());
                if !path.starts_with("container/") {
                    return None;
                }
                let kind = chunk
                    .iter_component::<RrContainerKind>(
                        ContainerBlueprint::descriptor_container_kind().component,
                    )
                    .flat_map(|item| item.to_vec())
                    .next()?;
                let name = chunk
                    .iter_component::<Name>(ContainerBlueprint::descriptor_display_name().component)
                    .flat_map(|item| {
                        item.iter()
                            .map(|n| n.0 .0.as_str().to_string())
                            .collect::<Vec<_>>()
                    })
                    .next();
                let col_shares = chunk
                    .iter_component::<ColumnShare>(
                        ContainerBlueprint::descriptor_col_shares().component,
                    )
                    .flat_map(|item| item.iter().map(|s| s.0 .0).collect::<Vec<f32>>())
                    .collect();
                let row_shares = chunk
                    .iter_component::<RowShare>(
                        ContainerBlueprint::descriptor_row_shares().component,
                    )
                    .flat_map(|item| item.iter().map(|s| s.0 .0).collect::<Vec<f32>>())
                    .collect();
                let grid_columns = chunk
                    .iter_component::<GridColumns>(
                        ContainerBlueprint::descriptor_grid_columns().component,
                    )
                    .flat_map(|item| item.iter().map(|g| g.0 .0).collect::<Vec<u32>>())
                    .next();
                let children = chunk
                    .iter_component::<IncludedContent>(
                        ContainerBlueprint::descriptor_contents().component,
                    )
                    .flat_map(|item| item.iter().map(|_| ()).collect::<Vec<_>>())
                    .count();
                Some(DecodedContainer {
                    kind,
                    name,
                    col_shares,
                    row_shares,
                    grid_columns,
                    children,
                    path,
                })
            })
            .collect()
    }

    fn decode_views(msgs: &[LogMsg]) -> Vec<DecodedView> {
        use rerun::external::re_sdk_types::blueprint::components::ViewOrigin;
        let cs = chunks(msgs);
        // First index each view's ViewContents query globs by the parent view path.
        let contents_of = |view_path: &str| -> Vec<String> {
            let want = format!("{view_path}/ViewContents");
            cs.iter()
                .find(|c| norm(&c.entity_path().to_string()) == want)
                .map(|c| {
                    c.iter_component::<QueryExpression>(ViewContents::descriptor_query().component)
                        .flat_map(|item| {
                            item.iter()
                                .map(|q| q.0 .0.as_str().to_string())
                                .collect::<Vec<_>>()
                        })
                        .collect()
                })
                .unwrap_or_default()
        };
        cs.iter()
            .filter_map(|chunk| {
                let path = norm(&chunk.entity_path().to_string());
                // The ViewBlueprint chunk is at `view/<uuid>` EXACTLY (one '/').
                if !path.starts_with("view/") || path.matches('/').count() != 1 {
                    return None;
                }
                let class = chunk
                    .iter_component::<ViewClass>(
                        ViewBlueprint::descriptor_class_identifier().component,
                    )
                    .flat_map(|item| {
                        item.iter()
                            .map(|c| c.0 .0.as_str().to_string())
                            .collect::<Vec<_>>()
                    })
                    .next()?;
                let name = chunk
                    .iter_component::<Name>(ViewBlueprint::descriptor_display_name().component)
                    .flat_map(|item| {
                        item.iter()
                            .map(|n| n.0 .0.as_str().to_string())
                            .collect::<Vec<_>>()
                    })
                    .next();
                let origin = chunk
                    .iter_component::<ViewOrigin>(
                        ViewBlueprint::descriptor_space_origin().component,
                    )
                    .flat_map(|item| {
                        item.iter()
                            .map(|o| o.0 .0.as_str().to_string())
                            .collect::<Vec<_>>()
                    })
                    .next();
                let contents = contents_of(&path);
                Some(DecodedView {
                    class,
                    name,
                    origin,
                    contents,
                    path,
                })
            })
            .collect()
    }

    /// The viewport root container id (the 16 raw bytes) + auto_views flag.
    fn decode_viewport(msgs: &[LogMsg]) -> (Option<[u8; 16]>, Option<bool>) {
        for chunk in &chunks(msgs) {
            if norm(&chunk.entity_path().to_string()) != "viewport" {
                continue;
            }
            let root = chunk
                .iter_component::<RootContainer>(
                    ViewportBlueprint::descriptor_root_container().component,
                )
                .flat_map(|item| item.iter().map(|r| r.0.bytes).collect::<Vec<[u8; 16]>>())
                .next();
            let auto = chunk
                .iter_component::<AutoViews>(ViewportBlueprint::descriptor_auto_views().component)
                .flat_map(|item| item.iter().map(|a| a.0 .0).collect::<Vec<bool>>())
                .next();
            return (root, auto);
        }
        (None, None)
    }

    /// The multi-kind plan the deleted `plan_to_blueprint_assembles_and_sends_every_kind`
    /// smoke test used — every container AND view kind, nested.
    fn every_kind_plan() -> BlueprintPlan {
        BlueprintPlan {
            auto_views: false,
            decorate: false,
            root: PlanNode::Container(PlanContainer {
                kind: ContainerKind::Horizontal,
                shares: Some(vec![2.0, 1.0]),
                columns: None,
                name: Some("root".to_string()),
                children: vec![
                    PlanNode::View(PlanView {
                        kind: ViewKind::Spatial3d,
                        name: Some("Scene".to_string()),
                        origin: Some("world".to_string()),
                        contents: None,
                    }),
                    PlanNode::Container(PlanContainer {
                        kind: ContainerKind::Grid,
                        shares: None,
                        columns: Some(2),
                        name: None,
                        children: vec![
                            PlanNode::View(PlanView {
                                kind: ViewKind::Spatial2d,
                                name: None,
                                origin: None,
                                contents: None,
                            }),
                            PlanNode::Container(PlanContainer {
                                kind: ContainerKind::Vertical,
                                shares: Some(vec![1.0, 1.0]),
                                columns: None,
                                name: None,
                                children: vec![
                                    PlanNode::View(PlanView {
                                        kind: ViewKind::TimeSeries,
                                        name: Some("Plots".to_string()),
                                        origin: Some("world/odom".to_string()),
                                        contents: None,
                                    }),
                                    PlanNode::Container(PlanContainer {
                                        kind: ContainerKind::Tabs,
                                        shares: None,
                                        columns: None,
                                        name: Some("tabs".to_string()),
                                        children: vec![PlanNode::View(PlanView {
                                            kind: ViewKind::TextDocument,
                                            name: None,
                                            origin: None,
                                            contents: None,
                                        })],
                                    }),
                                ],
                            }),
                        ],
                    }),
                ],
            }),
        }
    }

    #[test]
    fn build_blueprint_msgs_maps_every_container_kind_to_the_right_blueprint() {
        // The LIVE emit path (`build_blueprint_msgs`) maps EVERY container kind + view
        // kind to the right ContainerBlueprint / ViewportBlueprint chunks. Decoded +
        // pinned against hand oracles (the old smoke test only proved it "sent").
        let msgs = build_blueprint_msgs("smoke", &every_kind_plan()).expect("emit");
        let containers = decode_containers(&msgs);
        assert_eq!(containers.len(), 4, "4 containers: {containers:?}");

        let by = |k: RrContainerKind| {
            containers
                .iter()
                .find(|c| c.kind == k)
                .unwrap_or_else(|| panic!("no {k:?} container in {containers:?}"))
        };
        let root = by(RrContainerKind::Horizontal);
        assert_eq!(
            root.col_shares,
            vec![2.0, 1.0],
            "horizontal → COLUMN shares"
        );
        assert!(root.row_shares.is_empty(), "horizontal has no row shares");
        assert_eq!(root.name.as_deref(), Some("root"));
        assert_eq!(root.children, 2);

        let grid = by(RrContainerKind::Grid);
        assert_eq!(grid.grid_columns, Some(2), "grid columns");
        assert_eq!(grid.children, 2);

        let vert = by(RrContainerKind::Vertical);
        assert_eq!(vert.row_shares, vec![1.0, 1.0], "vertical → ROW shares");
        assert!(vert.col_shares.is_empty(), "vertical has no column shares");

        let tabs = by(RrContainerKind::Tabs);
        assert_eq!(tabs.name.as_deref(), Some("tabs"));
        assert_eq!(tabs.children, 1);
        assert!(
            tabs.col_shares.is_empty() && tabs.row_shares.is_empty(),
            "tabs take no shares: {tabs:?}"
        );

        // Every view kind is present with its rerun class identifier.
        let mut classes: Vec<String> = decode_views(&msgs).into_iter().map(|v| v.class).collect();
        classes.sort();
        assert_eq!(
            classes,
            vec!["2D", "3D", "TextDocument", "TimeSeries"],
            "one view per kind, mapped to its rerun class"
        );

        // The viewport root references the ROOT (Horizontal) container by its id.
        let (root_bytes, auto) = decode_viewport(&msgs);
        let root_bytes = root_bytes.expect("viewport has a root container");
        assert_eq!(
            format!("container/{}", format_uuid(&root_bytes)),
            root.path,
            "the viewport root_container is the top-level Horizontal container"
        );
        assert_eq!(auto, Some(false), "auto_views mirrors the plan");
    }

    #[test]
    fn build_blueprint_msgs_wraps_a_single_view_root_in_a_tabs_container() {
        // A single-VIEW root must be wrapped in a synthetic Tabs container the viewport
        // references (rerun's `Blueprint::new` behavior — the viewport root MUST be a
        // container). Decoded structurally (the old smoke only proved it "sent").
        let plan = BlueprintPlan {
            auto_views: true,
            decorate: false,
            root: PlanNode::View(PlanView {
                kind: ViewKind::Spatial3d,
                name: Some("Only".to_string()),
                origin: Some("world".to_string()),
                contents: None,
            }),
        };
        let msgs = build_blueprint_msgs("smoke", &plan).expect("emit");

        let containers = decode_containers(&msgs);
        assert_eq!(
            containers.len(),
            1,
            "exactly the synthetic wrapper: {containers:?}"
        );
        let wrapper = &containers[0];
        assert_eq!(wrapper.kind, RrContainerKind::Tabs, "wrapped in Tabs");
        assert_eq!(wrapper.children, 1, "the wrapper holds the one view");

        let views = decode_views(&msgs);
        assert_eq!(views.len(), 1, "one leaf view: {views:?}");
        assert_eq!(views[0].class, "3D");
        assert_eq!(views[0].name.as_deref(), Some("Only"));

        // The viewport references the synthetic wrapper, and auto_views is honored.
        let (root_bytes, auto) = decode_viewport(&msgs);
        assert_eq!(
            format!("container/{}", format_uuid(&root_bytes.expect("root"))),
            wrapper.path,
            "the viewport root is the synthetic Tabs wrapper"
        );
        assert_eq!(auto, Some(true));
    }

    #[test]
    fn build_blueprint_msgs_multirow_grid_carries_per_column_shares() {
        // A MULTI-ROW grid (4 views, 2 columns → 2 rows) with per-COLUMN shares
        // (exactly `columns` entries) emits `col_shares` + `grid_columns` exactly —
        // the value-level pin for the f0 grid-axis fix (the old smoke only "sent").
        let plan = BlueprintPlan {
            auto_views: false,
            decorate: false,
            root: PlanNode::Container(PlanContainer {
                kind: ContainerKind::Grid,
                shares: Some(vec![3.0, 1.0]), // one per column
                columns: Some(2),
                name: None,
                children: vec![
                    vk(ViewKind::Spatial2d, "world"),
                    vk(ViewKind::TimeSeries, "world"),
                    vk(ViewKind::TextDocument, "world"),
                    vk(ViewKind::Spatial3d, "world"),
                ],
            }),
        };
        let msgs = build_blueprint_msgs("smoke", &plan).expect("emit");
        let containers = decode_containers(&msgs);
        assert_eq!(containers.len(), 1, "one grid: {containers:?}");
        let grid = &containers[0];
        assert_eq!(grid.kind, RrContainerKind::Grid);
        assert_eq!(grid.grid_columns, Some(2), "grid_columns emitted");
        assert_eq!(
            grid.col_shares,
            vec![3.0, 1.0],
            "per-COLUMN shares, in order"
        );
        assert!(grid.row_shares.is_empty(), "a grid never emits row shares");
        assert_eq!(grid.children, 4, "all 4 views are grid children");
        assert_eq!(decode_views(&msgs).len(), 4);
    }

    #[test]
    fn build_blueprint_msgs_go2_default_maps_views_and_containers_structurally() {
        // Decision: the go2 default is Scene-only. The LIVE emit path maps it
        // to a SINGLE `3D` ViewBlueprint ("Scene", rooted at /world, default `$origin/**`
        // ViewContents), wrapped in the synthetic Tabs container the emitter uses for a
        // single-view root — NOT the old Horizontal[3:1]/Vertical tree (dropped with the
        // empty Telemetry + Status panels). auto_views stays on; the viewport roots at
        // the synthetic Tabs.
        let plan = BlueprintPlan::go2_default();
        let msgs = build_blueprint_msgs("go2", &plan).expect("emit");

        let views = decode_views(&msgs);
        assert_eq!(views.len(), 1, "Scene-only: a single 3D view: {views:?}");
        assert_eq!(views[0].class, "3D");
        assert_eq!(views[0].name.as_deref(), Some("Scene"));
        assert_eq!(
            views[0].origin.as_deref(),
            Some("/world"),
            "rooted at /world: {:?}",
            views[0]
        );
        assert_eq!(
            views[0].contents,
            vec!["$origin/**".to_string()],
            "default ViewContents glob (no explicit contents): {:?}",
            views[0]
        );

        // The single-view root is wrapped in exactly ONE synthetic Tabs container
        // (rerun's Blueprint::new behavior — the viewport root must be a container).
        let containers = decode_containers(&msgs);
        assert_eq!(
            containers.len(),
            1,
            "Scene-only: one synthetic Tabs wrapper, no Horizontal/Vertical tree: {containers:?}"
        );
        let root = &containers[0];
        assert_eq!(
            root.kind,
            RrContainerKind::Tabs,
            "single-view root wraps in a Tabs container: {root:?}"
        );
        assert!(
            root.col_shares.is_empty() && root.row_shares.is_empty(),
            "a Tabs container takes no shares: {root:?}"
        );
        assert_eq!(root.children, 1, "the lone 3D view is the sole child");

        let (root_bytes, auto) = decode_viewport(&msgs);
        assert_eq!(auto, Some(true), "the go2 default keeps auto_views on");
        assert_eq!(
            format!("container/{}", format_uuid(&root_bytes.expect("root"))),
            root.path,
            "the viewport root is the synthetic Tabs wrapper"
        );
    }

    // ── Soft origin-grounding hint (oracle) ─────────────────────────────────

    fn view(origin: &str) -> PlanNode {
        PlanNode::View(PlanView {
            kind: ViewKind::Spatial3d,
            name: None,
            origin: Some(origin.to_string()),
            contents: None,
        })
    }

    #[test]
    fn plan_origins_unmatched_flags_only_ungrounded_views() {
        let plan = BlueprintPlan {
            auto_views: true,
            decorate: false,
            root: PlanNode::Container(PlanContainer {
                kind: ContainerKind::Vertical,
                shares: None,
                columns: None,
                name: None,
                children: vec![
                    view("/world"),                    // root → always grounded
                    view("world/odom/base/lidar"),     // exact match to an attached
                    view("world/odom"),                // prefix of an attached
                    view("world/odom/base/lidar/xyz"), // prefixed BY an attached
                    view("world/ghost"),               // matches nothing → flagged
                ],
            }),
        };
        let attached = vec!["world/odom/base/lidar".to_string()];
        assert_eq!(
            plan_origins_unmatched(&plan, &attached),
            vec!["world/ghost".to_string()],
            "only the origin matching no attached entity is flagged"
        );
    }

    #[test]
    fn plan_origins_unmatched_dedups_and_roots_are_grounded_even_with_nothing_attached() {
        // Nothing attached: a non-root origin is ungrounded (deduped), the world
        // root stays grounded (so a reset-to-default never noise-warns).
        let plan = BlueprintPlan {
            auto_views: true,
            decorate: false,
            root: PlanNode::Container(PlanContainer {
                kind: ContainerKind::Horizontal,
                shares: None,
                columns: None,
                name: None,
                children: vec![view("world"), view("cam/wrist"), view("cam/wrist")],
            }),
        };
        assert_eq!(
            plan_origins_unmatched(&plan, &[]),
            vec!["cam/wrist".to_string()],
            "the root is grounded; the ungrounded origin is flagged once (deduped)"
        );
    }

    // ── Guardrails: validate_plan_against_attached (hand oracles) ──

    fn att(topic: &str, entity: &str, archetype: Option<ArchetypeKind>) -> AttachedRender {
        AttachedRender {
            topic: topic.to_string(),
            entity: entity.to_string(),
            archetype,
            producer_count: None,
            video_renditions: Vec::new(),
            representation: Representation::Auto,
            render_proof: RenderProof::default(),
        }
    }

    /// A leaf view of `kind` at `origin`.
    fn vk(kind: ViewKind, origin: &str) -> PlanNode {
        PlanNode::View(PlanView {
            kind,
            name: None,
            origin: Some(origin.to_string()),
            contents: None,
        })
    }

    /// A `horizontal` container of `children`, `auto_views` as given (the shape
    /// most guardrail cases use).
    fn plan_of(children: Vec<PlanNode>, auto_views: bool) -> BlueprintPlan {
        BlueprintPlan {
            auto_views,
            decorate: false,
            root: PlanNode::Container(PlanContainer {
                kind: ContainerKind::Horizontal,
                shares: None,
                columns: None,
                name: None,
                children,
            }),
        }
    }

    #[test]
    fn guardrail_all_views_ungrounded_reconstructs_the_pt42_failure() {
        // The all-guessed-origins shape: a hand-authored plan GUESSED entity origins (`/cloud`, `/cmd_vel`)
        // that don't exist — real paths are `world/<topic path>`. Every named view
        // is ungrounded, so the whole dashboard renders empty → one aggregate
        // refusal naming the guessed origins AND the concrete attached entity paths
        // (regardless of auto_views).
        let plan = plan_of(
            vec![
                vk(ViewKind::Spatial3d, "/cloud"),
                vk(ViewKind::TimeSeries, "/cmd_vel"),
            ],
            true, // auto_views implicitly on (the default)
        );
        let attached = vec![att(
            "/go2/vel",
            "world/odom/base/vel",
            Some(ArchetypeKind::Scalars),
        )];
        let err = validate_plan_against_attached(&plan, &attached).expect_err("plan is refused");
        assert_eq!(
            err,
            LayoutError::AllViewsUngrounded {
                origins: vec!["/cloud".to_string(), "/cmd_vel".to_string()],
                available: vec!["world/odom/base/vel".to_string()],
            }
        );
        let msg = err.to_string();
        assert!(msg.contains("/cloud") && msg.contains("/cmd_vel"), "{msg}");
        assert!(
            msg.contains("world/odom/base/vel"),
            "the refusal names a concrete attached entity path (G3 symmetry with G1): {msg}"
        );
        assert!(
            msg.contains("render empty") && msg.contains("discover/attach"),
            "the refusal names the symptom + the fix: {msg}"
        );
    }

    #[test]
    fn guardrail_nothing_attached_is_never_refused_lay_before_attach() {
        // Nothing attached: an agent may lay out BEFORE attaching — a layout with
        // any origin is unprovable-broken, so it is accepted (the soft hint fires
        // separately).
        let plan = plan_of(
            vec![
                vk(ViewKind::Spatial3d, "/cloud"),
                vk(ViewKind::TimeSeries, "/cmd_vel"),
            ],
            false,
        );
        assert_eq!(validate_plan_against_attached(&plan, &[]), Ok(Vec::new()));
    }

    #[test]
    fn guardrail_all_ungrounded_wins_over_ungrounded_with_auto_off() {
        // Precedence: all-ungrounded (the whole-dashboard refusal) beats the
        // per-view auto_views-off refusal even when auto_views is off.
        let plan = plan_of(vec![vk(ViewKind::Spatial3d, "/cloud")], false);
        let attached = vec![att("/go2/vel", "world/odom/base/vel", None)];
        assert!(matches!(
            validate_plan_against_attached(&plan, &attached),
            Err(LayoutError::AllViewsUngrounded { .. })
        ));
    }

    #[test]
    fn guardrail_spatial2d_all_3d_only_renders_nothing_is_refused() {
        // A LaserScan projects to Points3D → 3D-only. A spatial2d view whose
        // ONLY grounded topic is 3D-only renders nothing → refuse, naming the origin,
        // the topic, and (derived from views_for_archetype) the view kinds it CAN
        // render in (spatial3d).
        let plan = plan_of(vec![vk(ViewKind::Spatial2d, "world/odom/base/scan")], true);
        let attached = vec![att(
            "/go2/scan",
            "world/odom/base/scan",
            Some(ArchetypeKind::LaserScan),
        )];
        let err = validate_plan_against_attached(&plan, &attached).expect_err("laserscan in 2d");
        assert_eq!(
            err,
            LayoutError::Spatial2dRendersNothing {
                origin: "world/odom/base/scan".to_string(),
                offenders: vec!["'/go2/scan' (LaserScan → renders only in spatial3d)".to_string()],
            }
        );
        let msg = err.to_string();
        assert!(
            msg.contains("/go2/scan")
                && msg.contains("LaserScan")
                && msg.contains("spatial3d")
                && msg.contains("render nothing"),
            "{msg}"
        );
    }

    #[test]
    fn guardrail_g2_error_text_is_derived_for_a_non_3d_archetype() {
        // Fix #3: the "renders only in …" clause is derived from the topic's OWN
        // views_for_archetype list — NOT a hard-coded "3D scene". A Scalars topic in a
        // spatial2d view renders only in time_series, and the offender phrase says
        // exactly that (never "3D scene").
        let plan = plan_of(vec![vk(ViewKind::Spatial2d, "world/odom/base/vel")], true);
        let attached = vec![att(
            "/go2/vel",
            "world/odom/base/vel",
            Some(ArchetypeKind::Scalars),
        )];
        let err = validate_plan_against_attached(&plan, &attached).expect_err("scalars in 2d");
        assert_eq!(
            err,
            LayoutError::Spatial2dRendersNothing {
                origin: "world/odom/base/vel".to_string(),
                offenders: vec!["'/go2/vel' (Scalars → renders only in time_series)".to_string()],
            }
        );
        assert!(
            err.to_string().contains("time_series"),
            "the clause names the archetype's actual view kind: {err}"
        );
    }

    /// The same clause, on a topic the OPERATOR overrode.
    ///
    /// The decision asks `views_for_render`, so the message must too — and it must
    /// name the override, which is the only fact that makes the refusal
    /// actionable. Asking the KIND instead prints "Image → renders only in spatial2d,
    /// text_document" as the reason the topic cannot display in 2D
    /// (self-contradictory in one sentence) and the soft warning tells the operator
    /// to move it into the view it is already in.
    #[test]
    fn guardrail_g2_error_text_names_the_representation_override() {
        let plan = plan_of(vec![vk(ViewKind::Spatial2d, "world/cam")], true);
        let mut cam = att("/cam", "world/cam", Some(ArchetypeKind::Image));
        cam.representation = Representation::Text;
        let err = validate_plan_against_attached(&plan, &[cam]).expect_err("text-only in 2d");
        assert_eq!(
            err,
            LayoutError::Spatial2dRendersNothing {
                origin: "world/cam".to_string(),
                offenders: vec![
                    "'/cam' (Image → renders only in text_document because its representation is \
                     set to 'text' (change it back, or move the view))"
                        .to_string()
                ],
            }
        );
        // The un-overridden control, in the same body: the clause is UNCHANGED for
        // every topic nobody touched — which is what keeps the sibling oracles
        // above meaningful rather than merely green.
        let plan = plan_of(vec![vk(ViewKind::Spatial2d, "world/vel")], true);
        let err = validate_plan_against_attached(
            &plan,
            &[att("/vel", "world/vel", Some(ArchetypeKind::Scalars))],
        )
        .expect_err("scalars in 2d");
        assert_eq!(
            err,
            LayoutError::Spatial2dRendersNothing {
                origin: "world/vel".to_string(),
                offenders: vec!["'/vel' (Scalars → renders only in time_series)".to_string()],
            }
        );
    }

    /// The SOFT-WARNING half of the same rule — the arm that told the
    /// operator to move a topic into the view it is already in.
    #[test]
    fn the_invisible_sibling_warning_names_the_override_and_a_reachable_remedy() {
        let plan = plan_of(vec![vk(ViewKind::Spatial2d, "world")], true);
        let mut cam = att("/cam", "world/cam", Some(ArchetypeKind::Image));
        cam.representation = Representation::Text;
        // A second, un-overridden image keeps the view APPLYING, so the overridden
        // one lands on the soft-warning arm rather than the refusal.
        let ok_cam = att("/cam2", "world/cam2", Some(ArchetypeKind::Image));
        let warnings =
            validate_plan_against_attached(&plan, &[ok_cam, cam]).expect("the view applies");
        let warn = warnings
            .iter()
            .find(|w| w.contains("/cam'"))
            .unwrap_or_else(|| panic!("no warning for the overridden camera: {warnings:?}"));
        assert!(
            warn.contains("representation is set to 'text'"),
            "the warning must name the override — the only actionable fact: {warn}"
        );
        assert!(
            !warn.contains("move it to a spatial2d"),
            "…and must not tell the operator to move it INTO the view it is in: {warn}"
        );
    }

    #[test]
    fn guardrail_spatial2d_mixed_applies_with_a_soft_warn_naming_the_scan() {
        // Fix #2 (the mixed boundary): a broadly-rooted spatial2d view grounds a VALID
        // image AND an invisible LaserScan under the same root → it APPLIES (the pane
        // renders the image) and the LaserScan becomes a SOFT WARNING, not a refusal.
        // This is the near-universal real-robot case the old boundary wrongly refused.
        let plan = plan_of(vec![vk(ViewKind::Spatial2d, "world/odom/base")], true);
        let attached = vec![
            att(
                "/go2/cam",
                "world/odom/base/cam",
                Some(ArchetypeKind::Image),
            ),
            att(
                "/go2/scan",
                "world/odom/base/scan",
                Some(ArchetypeKind::LaserScan),
            ),
        ];
        let warnings =
            validate_plan_against_attached(&plan, &attached).expect("mixed 2d view applies");
        assert_eq!(
            warnings.len(),
            1,
            "one invisible-sibling warning: {warnings:?}"
        );
        assert!(
            warnings[0].contains("/go2/scan")
                && warnings[0].contains("LaserScan")
                && warnings[0].contains("spatial3d")
                && !warnings[0].contains("/go2/cam"),
            "the warning names the invisible LaserScan (not the valid image): {warnings:?}"
        );
    }

    #[test]
    fn guardrail_spatial2d_image_only_applies_no_warn() {
        // An Image DOES render in spatial2d — a grounded image-only 2d layout is a
        // valid, common shape with NO warning (guardrails don't over-reject).
        let plan = plan_of(vec![vk(ViewKind::Spatial2d, "world/odom/base/cam")], true);
        let attached = vec![att(
            "/go2/cam",
            "world/odom/base/cam",
            Some(ArchetypeKind::Image),
        )];
        assert_eq!(
            validate_plan_against_attached(&plan, &attached),
            Ok(Vec::new())
        );
    }

    #[test]
    fn guardrail_laserscan_in_spatial3d_is_accepted() {
        // The CORRECT placement (LaserScan → spatial3d) is NOT over-rejected.
        let plan = plan_of(vec![vk(ViewKind::Spatial3d, "world/odom/base/scan")], true);
        let attached = vec![att(
            "/go2/scan",
            "world/odom/base/scan",
            Some(ArchetypeKind::LaserScan),
        )];
        assert_eq!(
            validate_plan_against_attached(&plan, &attached),
            Ok(Vec::new())
        );
    }

    #[test]
    fn guardrail_ungrounded_view_with_auto_off_is_refused_with_suggestions() {
        // auto_views OFF + a mix (one grounded, one ungrounded): guardrail 3 does
        // not fire (not ALL ungrounded), but the ungrounded view will render empty
        // and nothing fills it → refuse, naming the nearest attached entity.
        let plan = plan_of(
            vec![
                vk(ViewKind::Spatial3d, "world/odom/base/vel"),
                vk(ViewKind::TimeSeries, "/ghost"),
            ],
            false,
        );
        let attached = vec![att(
            "/go2/vel",
            "world/odom/base/vel",
            Some(ArchetypeKind::Scalars),
        )];
        let err =
            validate_plan_against_attached(&plan, &attached).expect_err("ungrounded + auto off");
        assert_eq!(
            err,
            LayoutError::UngroundedView {
                view_kind: "time_series",
                origin: "/ghost".to_string(),
                suggestions: vec!["world/odom/base/vel".to_string()],
            }
        );
        assert!(
            err.to_string().contains("world/odom/base/vel")
                && err.to_string().contains("auto_views"),
            "the refusal suggests the nearest entity + names auto_views: {err}"
        );
    }

    #[test]
    fn guardrail_g2_wins_over_g1_when_both_violated() {
        // Fix #7 multi-violation precedence: a layout that violates BOTH G2 (a
        // spatial2d view rendering nothing) AND G1 (auto_views off + an ungrounded
        // view) is refused by G2 (checked before G1). Deterministic tie-break.
        let plan = plan_of(
            vec![
                vk(ViewKind::Spatial2d, "world/odom/base/scan"), // grounds only a LaserScan → G2
                vk(ViewKind::TimeSeries, "/ghost"),              // ungrounded + auto off → G1
            ],
            false,
        );
        let attached = vec![att(
            "/go2/scan",
            "world/odom/base/scan",
            Some(ArchetypeKind::LaserScan),
        )];
        assert!(
            matches!(
                validate_plan_against_attached(&plan, &attached),
                Err(LayoutError::Spatial2dRendersNothing { .. })
            ),
            "G2 (renders-nothing) is checked before G1 (ungrounded-auto-off)"
        );
    }

    #[test]
    fn guardrail_ungrounded_view_with_auto_on_is_accepted() {
        // The SAME mix with auto_views ON is accepted — the agent may lay out ahead
        // and auto_views fills the rest (the soft hint still fires at the caller).
        let plan = plan_of(
            vec![
                vk(ViewKind::Spatial3d, "world/odom/base/vel"),
                vk(ViewKind::TimeSeries, "/ghost"),
            ],
            true,
        );
        let attached = vec![att(
            "/go2/vel",
            "world/odom/base/vel",
            Some(ArchetypeKind::Scalars),
        )];
        assert_eq!(
            validate_plan_against_attached(&plan, &attached),
            Ok(Vec::new())
        );
    }

    #[test]
    fn guardrail_grounded_single_view_layout_applies() {
        // A grounded single-view layout (no container) is valid.
        let plan = BlueprintPlan {
            auto_views: false,
            decorate: false,
            root: vk(ViewKind::TimeSeries, "world/odom/base/vel"),
        };
        let attached = vec![att(
            "/go2/vel",
            "world/odom/base/vel",
            Some(ArchetypeKind::Scalars),
        )];
        assert_eq!(
            validate_plan_against_attached(&plan, &attached),
            Ok(Vec::new())
        );
    }

    /// A leaf view of `kind`, `origin`, and EXPLICIT `contents` globs (the compose
    /// output shape guardrail 4 checks).
    fn vk_contents(kind: ViewKind, origin: &str, contents: Vec<&str>) -> PlanNode {
        PlanNode::View(PlanView {
            kind,
            name: None,
            origin: Some(origin.to_string()),
            contents: Some(contents.into_iter().map(String::from).collect()),
        })
    }

    #[test]
    fn guardrail4_contents_that_ground_nothing_are_refused_even_with_grounded_origin() {
        // A view whose ORIGIN grounds but whose contents glob targets a
        // typo'd/divergent entity would render empty. Guardrail 4 catches it (origin
        // checks 1/3 would trivially pass).
        let plan = BlueprintPlan {
            auto_views: false,
            decorate: false,
            // origin grounds `.../vel`, but contents point at a non-existent subtree.
            root: vk_contents(
                ViewKind::TimeSeries,
                "world/odom/base/vel",
                vec!["/world/odom/base/TYPO/**"],
            ),
        };
        let attached = vec![att(
            "/go2/vel",
            "world/odom/base/vel",
            Some(ArchetypeKind::Scalars),
        )];
        let err = validate_plan_against_attached(&plan, &attached)
            .expect_err("contents ground nothing → refuse");
        assert!(
            matches!(err, LayoutError::ViewContentsUngrounded { .. }),
            "got {err:?}"
        );
        let msg = err.to_string();
        assert!(
            msg.contains("TYPO") && msg.contains("world/odom/base/vel"),
            "the refusal names the bad contents + the attached entity: {msg}"
        );
    }

    #[test]
    fn guardrail4_contents_matching_an_attached_entity_apply() {
        // The compose happy path: contents = the placed entity's glob → grounds →
        // applies (no over-rejection).
        let plan = BlueprintPlan {
            auto_views: false,
            decorate: false,
            root: vk_contents(
                ViewKind::TimeSeries,
                "world/odom/base/vel",
                vec!["/world/odom/base/vel/**"],
            ),
        };
        let attached = vec![att(
            "/go2/vel",
            "world/odom/base/vel",
            Some(ArchetypeKind::Scalars),
        )];
        assert_eq!(
            validate_plan_against_attached(&plan, &attached),
            Ok(Vec::new())
        );
    }

    #[test]
    fn guardrail4_robot_only_contents_are_exempt() {
        // An include_robot-only hero grounds the URDF skeleton, not an attached
        // topic — its lone robot glob must NOT trip guardrail 4.
        let plan = BlueprintPlan {
            auto_views: false,
            decorate: false,
            // a DIFFERENT topic is attached, but the hero's only content is the robot.
            root: vk_contents(
                ViewKind::Spatial3d,
                "world",
                vec!["/world/tf-tree/robot/**"],
            ),
        };
        let attached = vec![att(
            "/go2/vel",
            "world/odom/base/vel",
            Some(ArchetypeKind::Scalars),
        )];
        assert_eq!(
            validate_plan_against_attached(&plan, &attached),
            Ok(Vec::new()),
            "the robot-only hero is exempt from the contents-grounding check"
        );
    }

    #[test]
    fn guardrail4_mixed_real_plus_robot_contents_ground_via_the_real_glob() {
        // A hero with BOTH a real 3D topic glob AND the robot glob grounds via the
        // real one (the common include_robot-with-a-cloud shape).
        let plan = BlueprintPlan {
            auto_views: false,
            decorate: false,
            root: vk_contents(
                ViewKind::Spatial3d,
                "world",
                vec!["/world/odom/base/lidar/**", "/world/tf-tree/robot/**"],
            ),
        };
        let attached = vec![att(
            "/utlidar/cloud",
            "world/odom/base/lidar",
            Some(ArchetypeKind::Points3D),
        )];
        assert_eq!(
            validate_plan_against_attached(&plan, &attached),
            Ok(Vec::new())
        );
    }

    #[test]
    fn guardrail4_contents_none_views_are_never_checked() {
        // set_blueprint views carry NO contents → guardrail 4 never applies to them
        // (only the origin guardrails do), so a `contents: None` view at a grounded
        // origin still applies.
        let plan = plan_of(vec![vk(ViewKind::TimeSeries, "world/odom/base/vel")], false);
        let attached = vec![att(
            "/go2/vel",
            "world/odom/base/vel",
            Some(ArchetypeKind::Scalars),
        )];
        assert_eq!(
            validate_plan_against_attached(&plan, &attached),
            Ok(Vec::new())
        );
    }

    #[test]
    fn glob_grounds_and_is_robot_glob_oracles() {
        let ents = vec!["world/odom/base/cam".to_string()];
        // Subtree relation: exact + descendant grounds; a sibling/typo does not.
        assert!(glob_grounds_any("/world/odom/base/cam/**", &ents));
        assert!(glob_grounds_any("/world/odom/base/**", &ents)); // ancestor covers it
        assert!(!glob_grounds_any("/world/odom/base/TYPO/**", &ents));
        assert!(!glob_grounds_any("/world/odom/base/camX/**", &ents)); // not a prefix boundary
                                                                       // World-root / bare globs ground everything.
        assert!(glob_grounds_any("/world/**", &ents));
        // Robot-glob recognition (exempt in guardrail 4).
        assert!(is_robot_glob(&content_glob(ROBOT_ROOT)));
        assert!(is_robot_glob("/world/tf-tree/robot/**"));
        assert!(!is_robot_glob("/world/odom/base/cam/**"));
        // The exclusion form + the subtree-minus-topics contents.
        assert_eq!(excluded_glob("world/a/b"), "-/world/a/b/**");
        assert_eq!(excluded_glob("/world/a/b/"), "-/world/a/b/**");
        let family = vec![
            "world/camera/image_raw".to_string(),
            "world/camera/image_raw/compressed".to_string(),
            "world/camera/image_rawX".to_string(), // NOT a descendant (prefix boundary)
        ];
        assert_eq!(
            subtree_contents_excluding_topics("world/camera/image_raw", &family),
            vec![
                "/world/camera/image_raw/**".to_string(),
                "-/world/camera/image_raw/compressed/**".to_string(),
            ],
            "only genuine strict descendants are excluded"
        );
        // A leaf topic excludes nothing, and an entity is never excluded from itself.
        assert_eq!(
            subtree_contents_excluding_topics("world/camera/image_raw/compressed", &family),
            vec!["/world/camera/image_raw/compressed/**".to_string()]
        );
        // The per-topic view TITLE is the topic, with a default fallback.
        assert_eq!(
            topic_view_title("/api/vui/response", "Plots"),
            "/api/vui/response"
        );
        assert_eq!(topic_view_title("/", "Plots"), "Plots");
        assert_eq!(topic_view_title("", "Plots"), "Plots");
    }

    #[test]
    fn guardrail_text_document_of_a_textlog_topic_applies() {
        // A TextLog topic renders in a text_document view — a grounded
        // text_document layout is valid, not refused.
        let plan = plan_of(
            vec![vk(ViewKind::TextDocument, "world/odom/base/log")],
            true,
        );
        let attached = vec![att(
            "/go2/log",
            "world/odom/base/log",
            Some(ArchetypeKind::TextLog),
        )];
        assert_eq!(
            validate_plan_against_attached(&plan, &attached),
            Ok(Vec::new())
        );
    }

    #[test]
    fn guardrail_silent_topic_archetype_is_not_checked_in_2d() {
        // A still-silent attached topic (archetype None) cannot be proven 3D-only,
        // so a spatial2d view over it is NOT refused (no fabricated archetype).
        let plan = plan_of(vec![vk(ViewKind::Spatial2d, "world/odom/base/scan")], true);
        let attached = vec![att("/go2/scan", "world/odom/base/scan", None)];
        assert_eq!(
            validate_plan_against_attached(&plan, &attached),
            Ok(Vec::new())
        );
    }

    #[test]
    fn plan_grounds_any_entity_oracle() {
        // The reconnect-reapply grounding predicate (fix #5): true iff ANY leaf view
        // grounds an attached entity.
        let entities = vec!["world/odom/base/vel".to_string()];
        let grounded = plan_of(vec![vk(ViewKind::TimeSeries, "world/odom/base/vel")], true);
        let ungrounded = plan_of(vec![vk(ViewKind::TimeSeries, "/ghost")], true);
        assert!(plan_grounds_any_entity(&grounded, &entities));
        assert!(!plan_grounds_any_entity(&ungrounded, &entities));
        // A world-root default-origin view always grounds (nothing to warn about).
        let default_origin = plan_of(
            vec![PlanNode::View(PlanView {
                kind: ViewKind::TimeSeries,
                name: None,
                origin: None,
                contents: None,
            })],
            true,
        );
        assert!(plan_grounds_any_entity(&default_origin, &entities));
    }

    #[test]
    fn nearest_entities_ranks_by_shared_prefix_then_alpha() {
        let entities = vec![
            "world/odom/base/lidar".to_string(),
            "world/odom/base/cam".to_string(),
            "world/arm/wrist".to_string(),
        ];
        // Origin shares `world/odom/base` with the first two (3 segments) and only
        // `world` with the third (1). The two 3-segment matches tie → alphabetical
        // (cam before lidar); the 1-segment match is last.
        assert_eq!(
            nearest_entities("world/odom/base/ghost", &entities, 3),
            vec![
                "world/odom/base/cam".to_string(),
                "world/odom/base/lidar".to_string(),
                "world/arm/wrist".to_string(),
            ]
        );
        // `max` caps the list.
        assert_eq!(
            nearest_entities("world/odom/base/ghost", &entities, 1),
            vec!["world/odom/base/cam".to_string()]
        );
    }

    #[test]
    fn archetype_wire_name_is_stable() {
        assert_eq!(archetype_wire_name(ArchetypeKind::LaserScan), "LaserScan");
        assert_eq!(archetype_wire_name(ArchetypeKind::Points3D), "Points3D");
        assert_eq!(archetype_wire_name(ArchetypeKind::AnyValues), "AnyValues");
    }

    // ── Runtime slot: the reconnect-reapply contract (SOLE static toucher) ──

    #[test]
    fn apply_runtime_blueprint_remembers_the_plan_for_reconnect() {
        // The ONLY test touching RUNTIME_BLUEPRINT (mirrors the BLUEPRINT_SENT
        // guard-test convention — no intra-binary race). Deliberately touches
        // ONLY RUNTIME_BLUEPRINT (never BLUEPRINT_SENT — apply bypasses the
        // once-guard, and this test never calls rearm), so it cannot race the
        // `send_once_guard...` test. The "rearm preserves the runtime slot"
        // half of the reconnect-reapply contract holds BY CONSTRUCTION:
        // rearm_blueprint touches only BLUEPRINT_SENT, never RUNTIME_BLUEPRINT.
        clear_runtime_blueprint();
        assert_eq!(current_runtime_blueprint_plan(), None, "starts unset");

        let rec = memory();
        // A DISTINCT, clearly-non-default plan (a 2-view horizontal with custom
        // names/origins + auto_views off) — NOT go2_default. This is the
        // tautology fix: with the default as the subject, an apply that ignored
        // its argument and hardcoded go2_default would still pass the equality;
        // a distinct plan (asserted `!= default` below) makes the pin
        // mutation-tight against an argument-ignoring apply.
        let plan = BlueprintPlan {
            auto_views: false,
            decorate: false,
            root: PlanNode::Container(PlanContainer {
                kind: ContainerKind::Horizontal,
                shares: Some(vec![1.0, 2.0]),
                columns: None,
                name: Some("custom".to_string()),
                children: vec![
                    PlanNode::View(PlanView {
                        kind: ViewKind::Spatial2d,
                        name: Some("Cam".to_string()),
                        origin: Some("world/odom/base/cam".to_string()),
                        contents: None,
                    }),
                    PlanNode::View(PlanView {
                        kind: ViewKind::TimeSeries,
                        name: Some("Vel".to_string()),
                        origin: Some("world/odom/base/vel".to_string()),
                        contents: None,
                    }),
                ],
            }),
        };
        assert_ne!(
            plan,
            BlueprintPlan::go2_default(),
            "the subject must differ from the default (anti-tautology)"
        );
        apply_runtime_blueprint(&rec, plan.clone());

        // Remembered VERBATIM (the INPUT plan, not the default) → a later
        // reconnect's send_blueprint_once re-applies THIS custom layout instead
        // of the Go2 default (the reconnect-reapply contract). An apply that
        // ignored its argument would store go2_default here and FAIL this.
        assert_eq!(
            current_runtime_blueprint_plan(),
            Some(plan),
            "apply remembers the ACTUAL plan so a reconnect re-applies it"
        );
        clear_runtime_blueprint();
        assert_eq!(
            current_runtime_blueprint_plan(),
            None,
            "cleared for hygiene"
        );
    }

    // ── Compose compiler pure helpers (oracle vectors) ────────────

    #[test]
    fn content_glob_is_a_leading_slash_recursive_query() {
        // rerun's canonical query form: `/path/**` matches the anchor AND its
        // subtree (the cloud's `.../lidar/viz-sweep/k`), and leading/trailing slashes
        // on the input entity are normalized away.
        assert_eq!(
            content_glob("world/odom/base/lidar"),
            "/world/odom/base/lidar/**"
        );
        assert_eq!(
            content_glob("/world/tf-tree/robot"),
            "/world/tf-tree/robot/**"
        );
        assert_eq!(
            content_glob("world/tf-tree/robot/"),
            "/world/tf-tree/robot/**"
        );
    }

    #[test]
    fn grid_columns_is_ceil_sqrt_clamped() {
        // ceil(sqrt(n)), clamped to 1..=n so PlanContainer's grid bound holds.
        assert_eq!(grid_columns(1), 1);
        assert_eq!(grid_columns(2), 2);
        assert_eq!(grid_columns(3), 2);
        assert_eq!(grid_columns(4), 2);
        assert_eq!(grid_columns(5), 3);
        assert_eq!(grid_columns(9), 3);
    }

    #[test]
    fn strategy_and_role_wire_round_trip_and_reject_unknowns() {
        for s in [
            ComposeStrategy::Auto,
            ComposeStrategy::Focus3d,
            ComposeStrategy::Grid,
        ] {
            assert_eq!(ComposeStrategy::from_wire(s.as_str()), Ok(s));
        }
        assert_eq!(
            ComposeStrategy::from_wire("spiral"),
            Err(ComposeError::UnknownStrategy("spiral".to_string()))
        );
        assert!(ComposeStrategy::from_wire("spiral")
            .unwrap_err()
            .to_string()
            .contains("focus_3d"));
        for r in [
            GroupRole::Hero,
            GroupRole::Plots,
            GroupRole::Images,
            GroupRole::Status,
        ] {
            assert_eq!(GroupRole::from_wire(r.as_str()), Ok(r));
        }
        assert_eq!(
            GroupRole::from_wire("villain"),
            Err(ComposeError::UnknownRole("villain".to_string()))
        );
        // Roles map to their one canonical view kind.
        assert_eq!(GroupRole::Hero.view_kind(), ViewKind::Spatial3d);
        assert_eq!(GroupRole::Plots.view_kind(), ViewKind::TimeSeries);
        assert_eq!(GroupRole::Images.view_kind(), ViewKind::Spatial2d);
        assert_eq!(GroupRole::Status.view_kind(), ViewKind::TextDocument);
    }

    #[test]
    fn compose_over_zero_renderable_topics_is_a_loud_refusal() {
        // A single silent topic in bare-topics mode (no archetype) → not placed →
        // NoRenderableTopics naming the count + the silent topic (even with the
        // robot included, a robot-only scene is not a placement).
        let input = ComposeInput {
            groups: vec![ComposeGroup {
                topics: vec![AttachedRender {
                    topic: "/silent".to_string(),
                    entity: "world/odom/base/silent".to_string(),
                    archetype: None,
                    producer_count: None,
                    video_renditions: Vec::new(),
                    representation: Representation::Auto,
                    render_proof: RenderProof::default(),
                }],
                role: None,
                title: None,
            }],
            strategy: ComposeStrategy::Auto,
            include_robot: true,
        };
        let err = compose_layout(&input).expect_err("zero renderable → refuse");
        assert_eq!(
            err,
            ComposeError::NoRenderableTopics {
                requested: 1,
                silent: vec!["/silent".to_string()],
            }
        );
        assert!(err.to_string().contains("/silent") && err.to_string().contains("no views"));
    }

    #[test]
    fn role_incompatible_topic_is_rejected_with_the_real_view_kinds() {
        // An Image topic (never spatial3d) dropped into a `hero` (spatial3d) group →
        // REJECT naming the topic, its real view kinds, and the fix. Images now carry
        // the degradation status pane, so "the real view kinds" is now
        // `spatial2d, text_document` — the refusal is unchanged (an image renders in
        // neither of those as 3D geometry) and the message stays accurate by deriving
        // the list from `views_for_archetype` rather than hardcoding one kind.
        let input = ComposeInput {
            groups: vec![ComposeGroup {
                topics: vec![AttachedRender {
                    topic: "/cam".to_string(),
                    entity: "world/odom/base/camera".to_string(),
                    archetype: Some(ArchetypeKind::Image),
                    producer_count: None,
                    video_renditions: Vec::new(),
                    representation: Representation::Auto,
                    render_proof: RenderProof::default(),
                }],
                role: Some(GroupRole::Hero),
                title: None,
            }],
            strategy: ComposeStrategy::Auto,
            include_robot: false,
        };
        let err = compose_layout(&input).expect_err("image in hero → refuse");
        assert_eq!(
            err,
            ComposeError::RoleIncompatible {
                topic: "/cam".to_string(),
                archetype: "Image".to_string(),
                role: "hero",
                role_view: "spatial3d",
                real_views: "spatial2d, text_document".to_string(),
            }
        );
        let msg = err.to_string();
        assert!(
            msg.contains("/cam") && msg.contains("spatial2d") && msg.contains("hero"),
            "{msg}"
        );
    }

    // ── Node ids are IDENTITY-hashed, never positional ──────────────
    //
    // The bug: `next_node_id` minted every view/container id from a bare monotone
    // counter, so an id was a function of a node's POSITION in the emitted plan. Any
    // plan change — a topic checked or unchecked, a second video rendition appearing,
    // a re-scan — shifted every node after the insertion point, and an id that had
    // carried a `Spatial2d` view was re-declared as a `TextDocument`. The viewer keys
    // per-view UI state by view id, so it read the retained state through the new
    // class and failed the downcast: one
    // "Failed to downcast view's … to SpatialViewState" toast per shifted view, per
    // frame — the shape seen on checking `/go2/camera/h264`.
    //
    // Oracles are HAND-WRITTEN: the pinned ids below are literals, and the
    // class-stability arms compare emission-to-emission id→class maps built here,
    // never a value the emitter also produced for the same purpose.

    /// Every emitted node's id → what CLASS that id was declared as: a view's rerun
    /// class identifier (`"2D"`, `"TextDocument"`, …) or `"container:<kind>"`. The
    /// re-labelling bug is exactly one id mapping to two different values of this
    /// across two emissions, so it is the natural probe — and it spans views AND
    /// containers, which shared the one counter and so could swap roles too.
    fn id_classes(msgs: &[LogMsg]) -> std::collections::BTreeMap<String, String> {
        let mut out = std::collections::BTreeMap::new();
        for v in decode_views(msgs) {
            out.insert(
                v.path.trim_start_matches("view/").to_string(),
                v.class.clone(),
            );
        }
        for c in decode_containers(msgs) {
            out.insert(
                c.path.trim_start_matches("container/").to_string(),
                format!("container:{:?}", c.kind),
            );
        }
        out
    }

    /// The `view/<uuid>` id of the FIRST view with this class + origin.
    fn view_id(msgs: &[LogMsg], class: &str, origin: &str) -> String {
        let views = decode_views(msgs);
        views
            .iter()
            .find(|v| v.class == class && v.origin.as_deref() == Some(origin))
            .unwrap_or_else(|| panic!("no {class} view at '{origin}' in {views:?}"))
            .path
            .trim_start_matches("view/")
            .to_string()
    }

    /// Each view's `(class, origin)` → its id. The IDENTITY probe: a view that is
    /// still in the plan is the SAME view, so its id must be the same id. (A mere
    /// count of shared ids is not this property — wholesale re-minting renumbers into
    /// the same low id space and overlaps by coincidence, which is how a class-only
    /// occurrence scoping can pass a shared-id count.)
    fn ids_by_identity(msgs: &[LogMsg]) -> std::collections::BTreeMap<(String, String), String> {
        decode_views(msgs)
            .into_iter()
            .map(|v| {
                (
                    (v.class, v.origin.unwrap_or_default()),
                    v.path.trim_start_matches("view/").to_string(),
                )
            })
            .collect()
    }

    /// Every view that exists in BOTH emissions must carry the SAME id — the
    /// property "a plan change adds and removes ids, it never re-labels them", stated
    /// over the views themselves rather than over a count. `min_shared` guards
    /// vacuity: the two plans really do have that many views in common.
    fn assert_surviving_views_keep_their_ids(
        before: &[LogMsg],
        after: &[LogMsg],
        min_shared: usize,
    ) {
        let (a, b) = (ids_by_identity(before), ids_by_identity(after));
        let shared: Vec<_> = a.keys().filter(|k| b.contains_key(*k)).cloned().collect();
        assert!(
            shared.len() >= min_shared,
            "expected >= {min_shared} views to survive the change, got {}: {:?} vs {:?}",
            shared.len(),
            a.keys(),
            b.keys()
        );
        let moved: Vec<_> = shared
            .iter()
            .filter(|k| a[*k] != b[*k])
            .map(|k| (k.clone(), a[k].clone(), b[k].clone()))
            .collect();
        assert!(
            moved.is_empty(),
            "a surviving view was RE-MINTED — its retained viewer state is orphaned: {moved:?}"
        );
    }

    /// Two views deliberately SHARING an identity key — same class, same origin,
    /// differing only in contents (the hand-authored `set_blueprint` shape) — with
    /// `extra` views appended after them.
    fn same_key_pair(extra: Vec<PlanNode>) -> BlueprintPlan {
        let mut children = vec![
            PlanNode::View(PlanView {
                kind: ViewKind::TimeSeries,
                name: Some("Battery".to_string()),
                origin: Some("world/robot/state".to_string()),
                contents: Some(vec!["world/robot/state/battery/**".to_string()]),
            }),
            PlanNode::View(PlanView {
                kind: ViewKind::TimeSeries,
                name: Some("Motors".to_string()),
                origin: Some("world/robot/state".to_string()),
                contents: Some(vec!["world/robot/state/motors/**".to_string()]),
            }),
        ];
        children.extend(extra);
        plan_of(children, false)
    }

    /// [`same_key_pair`] with `extra` PREPENDED instead. Prepending is what makes an
    /// unrelated-insertion probe discriminating: appending leaves even a positional
    /// counter's ids alone, so an appended probe proves nothing.
    fn same_key_pair_prefixed(extra: Vec<PlanNode>) -> BlueprintPlan {
        let mut plan = same_key_pair(vec![]);
        if let PlanNode::Container(c) = &mut plan.root {
            let mut children = extra;
            children.append(&mut c.children);
            c.children = children;
        }
        plan
    }

    /// The ids of the `world/robot/state` pair, SORTED.
    ///
    /// Sorted, not in decode order, deliberately: the decoded chunk order is NOT
    /// stable across emissions (measured — two emissions of these same two views
    /// returned the identical pair of ids in opposite order), and the property under
    /// test is "these views keep their ids", not "the decoder yields them in a
    /// particular sequence". Anything that needs to tell the two APART uses
    /// [`same_key_id_named`], which keys on the view's display name.
    fn same_key_ids(msgs: &[LogMsg]) -> Vec<String> {
        let mut ids: Vec<String> = decode_views(msgs)
            .into_iter()
            .filter(|v| v.origin.as_deref() == Some("world/robot/state"))
            .map(|v| v.path.trim_start_matches("view/").to_string())
            .collect();
        ids.sort();
        ids
    }

    /// The id of the `world/robot/state` view DISPLAY-NAMED `name` — the
    /// order-independent way to name one half of the same-key pair.
    fn same_key_id_named(msgs: &[LogMsg], name: &str) -> String {
        let views = decode_views(msgs);
        views
            .iter()
            .find(|v| {
                v.origin.as_deref() == Some("world/robot/state") && v.name.as_deref() == Some(name)
            })
            .unwrap_or_else(|| panic!("no '{name}' view in {views:?}"))
            .path
            .trim_start_matches("view/")
            .to_string()
    }

    /// The realistic shape: an image topic (→ `[Spatial2d, TextDocument]`,
    /// the mixed-class adjacent pair that made this loud) beside a scalars
    /// topic, laid out by the SHIPPING default-layout compiler.
    fn attach_plan(topics: &[(&str, ArchetypeKind)]) -> BlueprintPlan {
        let attached: Vec<AttachedRender> = topics
            .iter()
            .map(|(t, a)| att(t, &format!("world{t}"), Some(*a)))
            .collect();
        default_layout(&attached)
    }

    #[test]
    fn a_plan_change_never_re_labels_an_existing_view_id_with_a_new_class() {
        // The regression guard, in the reported flow: check a topic, re-emit.
        // The inserted topic sorts FIRST, so under the counter every id after it
        // shifted by two (an image contributes two views) and the classes rotated.
        let before = attach_plan(&[
            ("/cam/image_raw", ArchetypeKind::Image),
            ("/imu/data", ArchetypeKind::Scalars),
            ("/diag/status", ArchetypeKind::TextLog),
        ]);
        let after = attach_plan(&[
            ("/aaa/new_video", ArchetypeKind::VideoStream), // checked mid-session
            ("/cam/image_raw", ArchetypeKind::Image),
            ("/imu/data", ArchetypeKind::Scalars),
            ("/diag/status", ArchetypeKind::TextLog),
        ]);

        let before = build_blueprint_msgs("blueprint", &before).expect("emit");
        let after = build_blueprint_msgs("blueprint", &after).expect("emit");
        let a = id_classes(&before);
        let b = id_classes(&after);

        // Anti-vacuity, stated over the VIEWS THEMSELVES. Asserting only
        // that >= 4 ids are SHARED is satisfied by the positional counter itself —
        // it renumbers into the same low id space, so ids coincide while zero views
        // semantically survive. That lets a minter scoping the occurrence index on
        // CLASS alone re-mint 3 of 5 surviving views with the whole suite green.
        assert_surviving_views_keep_their_ids(&before, &after, 4);
        assert!(
            b.len() > a.len(),
            "the new topic added nodes: {a:?} → {b:?}"
        );

        let relabelled: Vec<(&String, &String, &String)> = a
            .iter()
            .filter_map(|(id, class)| b.get(id).map(|after| (id, class, after)))
            .filter(|(_, before, after)| before != after)
            .collect();
        assert!(
            relabelled.is_empty(),
            "an id was RE-LABELLED with a different class across a plan change — this is \
             exactly the rerun downcast-toast flood: {relabelled:?}"
        );
    }

    #[test]
    fn a_same_class_view_at_a_new_origin_does_not_move_an_incumbents_id() {
        // The occurrence index is scoped to the FULL identity key, not to the class.
        // Every other arm inserts a view of a DIFFERENT class, so all of them are
        // blind to that distinction: a minter counting occurrences per CLASS passes
        // them while re-minting on every same-class attach — the retained-state reset
        // the key's exclusions exist to prevent. Here the inserted topic is the SAME
        // class (`Scalars` → TimeSeries) at a DIFFERENT origin, and sorts AHEAD of the
        // incumbent, so a class-scoped index would hand the incumbent index 1.
        let before = attach_plan(&[
            ("/imu/data", ArchetypeKind::Scalars),
            ("/motor/temps", ArchetypeKind::Scalars),
        ]);
        let after = attach_plan(&[
            ("/aaa/battery", ArchetypeKind::Scalars), // same class, sorts FIRST
            ("/imu/data", ArchetypeKind::Scalars),
            ("/motor/temps", ArchetypeKind::Scalars),
        ]);
        let before = build_blueprint_msgs("blueprint", &before).expect("emit");
        let after = build_blueprint_msgs("blueprint", &after).expect("emit");

        // Anti-vacuity: the inserted view really is the same class as the incumbents.
        let inserted = view_id(&after, "TimeSeries", "world/aaa/battery");
        let incumbent = view_id(&before, "TimeSeries", "world/imu/data");
        assert_ne!(inserted, incumbent, "the inserted view is its own view");
        assert_eq!(
            view_id(&after, "TimeSeries", "world/imu/data"),
            incumbent,
            "a SAME-CLASS view inserted ahead of it must not move the incumbent's id"
        );
        assert_surviving_views_keep_their_ids(&before, &after, 2);
    }

    #[test]
    fn the_hero_scene_keeps_its_id_when_a_spatial_attach_changes_its_contents() {
        // THE behavioural arm for the `contents` exclusion — the design's load-bearing
        // decision, which until now was argued only in a doc comment. The default
        // layout's hero Scene grounds the UNION of every attached spatial topic, so a
        // spatial attach genuinely rewrites its contents; folding contents into the
        // identity key would therefore re-mint the Scene on every checkbox and reset
        // the user's dragged 3D camera. A LaserScan is used deliberately: a
        // VideoStream (what the other arms attach) does NOT move the Scene's contents,
        // so it cannot discriminate this.
        let before = attach_plan(&[("/imu/data", ArchetypeKind::Scalars)]);
        let after = attach_plan(&[
            ("/aaa/scan", ArchetypeKind::LaserScan), // spatial → folds into the Scene
            ("/imu/data", ArchetypeKind::Scalars),
        ]);
        let before = build_blueprint_msgs("blueprint", &before).expect("emit");
        let after = build_blueprint_msgs("blueprint", &after).expect("emit");

        let scene = |msgs: &[LogMsg]| {
            decode_views(msgs)
                .into_iter()
                .find(|v| v.class == "3D")
                .expect("the hero Scene view")
        };
        let (a, b) = (scene(&before), scene(&after));

        // Anti-vacuity: the attach really DID rewrite the Scene's contents, so a
        // contents-keyed minter really would have re-minted it here.
        assert_ne!(
            a.contents, b.contents,
            "precondition: a spatial attach must change the Scene's contents"
        );
        assert_eq!(
            a.path, b.path,
            "the hero Scene keeps its id across a spatial attach — its contents are \
             NOT part of its identity, so the user's 3D camera survives a checkbox"
        );
    }

    #[test]
    fn a_view_id_is_a_pinned_function_of_its_class_and_origin() {
        // The ANTI-POSITIONAL pin, against hand-written literal oracles. Emitting the
        // SAME view from two structurally different plans — one where it is the first
        // node emitted, one where two unrelated views are emitted before it — must
        // yield the SAME id, and specifically THESE ids. Under the counter the second
        // plan hands it a different id (and hands its old id to a different class).
        let target = || {
            PlanNode::View(PlanView {
                kind: ViewKind::TextDocument,
                name: Some("Status".to_string()),
                origin: Some("world/diag/status".to_string()),
                contents: None,
            })
        };
        let alone = plan_of(vec![target()], false);
        let preceded = plan_of(
            vec![
                vk(ViewKind::Spatial2d, "world/cam/image_raw"),
                vk(ViewKind::TimeSeries, "world/imu/data"),
                target(),
            ],
            false,
        );

        let alone = build_blueprint_msgs("blueprint", &alone).expect("emit");
        let preceded = build_blueprint_msgs("blueprint", &preceded).expect("emit");

        // Hand oracle: the frozen id of `(TextDocument, world/diag/status, #0)`. A
        // literal, so it also pins determinism across runs AND across toolchains
        // (the hash is FNV-1a, not `DefaultHasher`).
        const STATUS_ID: &str = "ce30f968-8f8d-7a20-cee1-2e1c38b2187f";
        assert_eq!(
            view_id(&alone, "TextDocument", "world/diag/status"),
            STATUS_ID,
            "the id is the hash of (class, origin, occurrence) — nothing else"
        );
        assert_eq!(
            view_id(&preceded, "TextDocument", "world/diag/status"),
            STATUS_ID,
            "inserting two unrelated views BEFORE it must not move its id"
        );
        // …and the id it used to be handed under the counter now belongs to nobody
        // else: the two preceding views have their own identity-derived ids.
        let cam = view_id(&preceded, "2D", "world/cam/image_raw");
        let imu = view_id(&preceded, "TimeSeries", "world/imu/data");
        assert!(
            cam != STATUS_ID && imu != STATUS_ID && cam != imu,
            "distinct identities → distinct ids: cam={cam} imu={imu} status={STATUS_ID}"
        );
    }

    #[test]
    fn the_same_plan_re_emitted_twice_yields_byte_identical_ids() {
        // Determinism (the property a positional counter has and the minter must keep):
        // re-emitting an unchanged plan must reproduce every id exactly, or a mere
        // reconnect-reapply would orphan every view's retained state.
        let plan = attach_plan(&[
            ("/cam/image_raw", ArchetypeKind::Image),
            ("/imu/data", ArchetypeKind::Scalars),
        ]);
        let first = id_classes(&build_blueprint_msgs("blueprint", &plan).expect("emit"));
        let second = id_classes(&build_blueprint_msgs("blueprint", &plan).expect("emit"));
        assert!(!first.is_empty(), "the plan emitted nodes");
        assert_eq!(first, second, "an unchanged plan re-emits identical ids");
    }

    #[test]
    fn views_sharing_a_class_and_origin_get_distinct_stable_ids() {
        // Two views that legitimately share `(class, origin)` — a hand-authored
        // `set_blueprint` plan with two same-class panes rooted at one entity,
        // differing only in contents — must NOT alias onto one id. They are separated
        // by an occurrence index within that exact key, stable while the pair is.
        let msgs = build_blueprint_msgs("blueprint", &same_key_pair(vec![])).expect("emit");
        let ids = same_key_ids(&msgs);
        assert_eq!(ids.len(), 2, "both views emitted: {ids:?}");
        assert_ne!(ids[0], ids[1], "a shared identity key still mints two ids");

        // …and a view of a DIFFERENT identity does not perturb either of them (only a
        // SAME-KEY sibling can shift an occurrence index). It is PREPENDED, not
        // appended: appending is the one position at which even a positional counter
        // leaves the pair alone, so an appended probe cannot discriminate at all.
        let grown = build_blueprint_msgs(
            "blueprint",
            &same_key_pair_prefixed(vec![vk(ViewKind::Spatial3d, "world")]),
        )
        .expect("emit");
        assert_eq!(
            same_key_ids(&grown),
            ids,
            "an unrelated view inserted AHEAD of the pair leaves their ids alone"
        );
    }

    #[test]
    fn removing_a_same_key_sibling_shifts_the_survivor_within_its_own_class() {
        // The POSITIVE half of the disambiguator's safety argument — "a shift can only
        // hand one view another SAME-CLASS view's state, never trip the downcast" —
        // which the other arms assert only negatively (no cross-class re-labelling).
        // Removing the FIRST of two same-key views shifts the survivor from occurrence
        // 1 to occurrence 0, so it inherits the removed sibling's id. That is the
        // designed, bounded cost, and this arm states it out loud: the inheritance is
        // real, and the class on both sides of it is identical.
        let both = build_blueprint_msgs("blueprint", &same_key_pair(vec![])).expect("emit");
        // By NAME, never by decode position — the decoded chunk order is not stable
        // across emissions, so `ids[0]` would not reliably be the occurrence-0 view.
        let first = same_key_id_named(&both, "Battery"); // occurrence 0, emitted first
        let second = same_key_id_named(&both, "Motors"); // occurrence 1

        let mut survivor_only = same_key_pair(vec![]);
        if let PlanNode::Container(c) = &mut survivor_only.root {
            c.children.remove(0); // drop the occurrence-0 sibling
        }
        let after = build_blueprint_msgs("blueprint", &survivor_only).expect("emit");
        let after_views = decode_views(&after);
        assert_eq!(after_views.len(), 1, "one survivor: {after_views:?}");

        let survivor = &after_views[0];
        assert_eq!(
            survivor.name.as_deref(),
            Some("Motors"),
            "precondition: the occurrence-0 sibling is the one that was removed"
        );
        let survivor_id = survivor.path.trim_start_matches("view/").to_string();
        assert_eq!(
            survivor_id, first,
            "the survivor shifts to occurrence 0 and inherits the removed sibling's id"
        );
        assert_ne!(survivor_id, second, "…so its own former id is retired");
        // THE POINT: the id it inherited was a TimeSeries id and it is a TimeSeries
        // view. A shift moves state WITHIN a class; it can never present one class's
        // retained state through another's, which is what the downcast rejects.
        let before_class = decode_views(&both)
            .into_iter()
            .find(|v| v.path.trim_start_matches("view/") == first)
            .expect("the removed sibling")
            .class;
        assert_eq!(
            survivor.class, before_class,
            "an inherited id carries the SAME class on both sides of the shift"
        );
    }

    #[test]
    fn detaching_a_topic_never_re_labels_a_surviving_view() {
        // The DELETE direction through the shipping layout compiler: every other arm
        // drives an ATTACH. A detach removes views from the middle of the plan, which
        // under the counter shifted everything after it just as an attach did.
        let before = attach_plan(&[
            ("/cam/image_raw", ArchetypeKind::Image),
            ("/imu/data", ArchetypeKind::Scalars),
            ("/diag/status", ArchetypeKind::TextLog),
        ]);
        let after = attach_plan(&[
            ("/cam/image_raw", ArchetypeKind::Image),
            // /imu/data unchecked — its TimeSeries view leaves from the MIDDLE
            ("/diag/status", ArchetypeKind::TextLog),
        ]);
        let before = build_blueprint_msgs("blueprint", &before).expect("emit");
        let after = build_blueprint_msgs("blueprint", &after).expect("emit");

        // The detached view really is gone (anti-vacuity), and every survivor kept
        // its id and its class.
        assert!(
            !ids_by_identity(&after)
                .contains_key(&("TimeSeries".to_string(), "world/imu/data".to_string())),
            "precondition: the detached topic's view is gone"
        );
        assert_surviving_views_keep_their_ids(&before, &after, 3);
        let (a, b) = (id_classes(&before), id_classes(&after));
        let relabelled: Vec<_> = a
            .iter()
            .filter_map(|(id, class)| b.get(id).map(|now| (id, class, now)))
            .filter(|(_, was, now)| was != now)
            .collect();
        assert!(
            relabelled.is_empty(),
            "a DETACH re-labelled an id with a different class: {relabelled:?}"
        );
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_digest_already_issued_is_re_salted_loudly_rather_than_aliased() {
        // The collision arm. A 112-bit FNV-1a collision cannot be produced on demand,
        // so the CONDITION is staged instead of the cause: the digest the next mint
        // would return is pre-seeded into the issued set, which is exactly the state a
        // real collision creates. Without this, the salt loop, the `~{salt}` identity
        // format and the warn were executed by no test at all.
        let key = NodeIdMinter::view_key(&PlanView {
            kind: ViewKind::Spatial2d,
            name: None,
            origin: Some("world/cam".to_string()),
            contents: None,
        });
        let natural = id_bytes_for(&format!("{key}{ID_KEY_SEP}#0"));

        let mut minter = NodeIdMinter::default();
        minter.minted.insert(natural);
        let (bytes, uuid) = minter.mint(&key);

        assert_ne!(
            bytes, natural,
            "a taken digest must be re-salted, never aliased onto the incumbent"
        );
        assert_eq!(uuid, format_uuid(&bytes), "the string mirrors the bytes");
        assert_eq!(
            bytes,
            id_bytes_for(&format!("{key}{ID_KEY_SEP}#0{ID_KEY_SEP}~1")),
            "the resolution is the documented salt-1 identity, not an arbitrary value"
        );
        assert!(
            logs_contain("blueprint node id hash collision"),
            "a collision is LOUD — it is the one path whose id is not a pure function \
             of the node's identity"
        );

        // Deterministic: the same staged state resolves the same way every time.
        let mut again = NodeIdMinter::default();
        again.minted.insert(natural);
        assert_eq!(
            again.mint(&key).0,
            bytes,
            "salt resolution is deterministic"
        );
    }

    #[test]
    fn a_container_id_carries_its_kind_so_it_cannot_become_a_view() {
        // Containers were minted from the SAME counter as views, so a plan change
        // could hand a container's old id to a view (and back). The `container`
        // domain tag + the kind in the hash separate the two id spaces — up to a hash
        // COLLISION, which the minter detects and re-salts (see `NodeIdMinter`); it is
        // "no shared derivation", not a mathematical impossibility. The tree path
        // keeps a container that did not move from being re-minted.
        let one = attach_plan(&[
            ("/cam/image_raw", ArchetypeKind::Image),
            ("/imu/data", ArchetypeKind::Scalars),
        ]);
        let two = attach_plan(&[
            ("/cam/image_raw", ArchetypeKind::Image),
            ("/imu/data", ArchetypeKind::Scalars),
            ("/diag/status", ArchetypeKind::TextLog),
        ]);
        let a = build_blueprint_msgs("blueprint", &one).expect("emit");
        let b = build_blueprint_msgs("blueprint", &two).expect("emit");

        let container_ids = |msgs: &[LogMsg]| -> std::collections::BTreeSet<String> {
            decode_containers(msgs)
                .into_iter()
                .map(|c| c.path.trim_start_matches("container/").to_string())
                .collect()
        };
        let view_ids = |msgs: &[LogMsg]| -> std::collections::BTreeSet<String> {
            decode_views(msgs)
                .into_iter()
                .map(|v| v.path.trim_start_matches("view/").to_string())
                .collect()
        };

        for (label, msgs) in [("before", &a), ("after", &b)] {
            let overlap: Vec<String> = container_ids(msgs)
                .intersection(&view_ids(msgs))
                .cloned()
                .collect();
            assert!(
                overlap.is_empty(),
                "{label}: a container id and a view id must never coincide: {overlap:?}"
            );
        }
        // Across the change, no id that was a container is now a view or vice versa
        // (covered by the class map, asserted here on the cross pair specifically).
        let cross: Vec<String> = container_ids(&a)
            .intersection(&view_ids(&b))
            .cloned()
            .collect();
        assert!(
            cross.is_empty(),
            "a container id became a view id: {cross:?}"
        );
        let cross: Vec<String> = view_ids(&a)
            .intersection(&container_ids(&b))
            .cloned()
            .collect();
        assert!(
            cross.is_empty(),
            "a view id became a container id: {cross:?}"
        );
    }
}
