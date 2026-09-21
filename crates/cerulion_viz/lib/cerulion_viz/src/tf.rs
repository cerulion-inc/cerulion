// SPDX-License-Identifier: AGPL-3.0-only
//! TFMessage → Rerun transform-tree mapping.
//!
//! Rerun resolves the transform tree ITSELF: you log a [`rerun::Transform3D`]
//! at each frame's entity and Rerun composes the chain by entity hierarchy
//! (parent entity ∘ child entity). Nothing here does a tf2-style
//! `lookupTransform` (a transform-lookup library is deliberately out of scope).
//! This module FORWARDS transforms — the pure decode lives in `go2_tf`, this
//! is the Rerun-logging half.
//!
//! # Entity tree (the frame_id → entity-path mapping)
//!
//! The tree encodes the ASSUMED Go2 topology `odom → base → {lidar, camera}`.
//! Each transform (`parent → child`) is logged at the CHILD frame's entity
//! path; Rerun applies it relative to the child's PARENT ENTITY, so the chain
//! composes as long as the frame topology matches the tree:
//!
//! | frame id (child) | entity path | logged transform |
//! |---|---|---|
//! | `odom` | [`ODOM_ENTITY`] (`world/tf-tree/odom`) | `map`/`world → odom` (if present) |
//! | `base` / `base_link` | [`BASE_ENTITY`] (`world/tf-tree/odom/base`) | `odom → base` (robot pose) |
//! | `lidar` / … | [`LIDAR_ENTITY`] (`world/tf-tree/odom/base/lidar`) | `base → lidar` (mount) |
//! | `camera` / … | [`CAMERA_ENTITY`] (`world/tf-tree/odom/base/camera`) | `base → camera` (mount) |
//! | anything else | `world/tf-tree/odom/base/<sanitized>` (fallback) | + once-per-frame WARN |
//!
//! `world` is the fixed viz root; [`log_world_view_coordinates_static`] stamps
//! it `RIGHT_HAND_Z_UP` (the ROS convention) so the tree renders correctly.
//! Every FRAME entity hangs off the RESERVED [`FRAME_ROOT`] (`world/tf-tree`)
//! segment, which no topic can ever produce — see [`FRAME_ROOT`] for the
//! uniqueness proof.
//!
//! **This tree is the FRAME tree only; it is no longer where topics
//! live.** Previously a topic's entity path was `world/odom/base/<last
//! topic segment>`, and the media names (`cloud`/`lidar`/`points`/… and
//! `image`/`camera`/…) FOLDED a topic onto [`LIDAR_ENTITY`] /
//! [`CAMERA_ENTITY`] so its geometry rendered POSED under this tree. That
//! collapsed distinct topics onto one entity (measured on a Go2: 6
//! entity paths covered 38 of 75 topics — all 15 `/api/*/response` topics
//! rendered to the then-`world/odom/base/response`). Topic entities are now
//! `world/<full sanitized topic>` — unique by construction (see
//! [`crate::sink::route_for_input`]) — and posing is a SEPARATE assignment:
//! a topic whose message carries a resolvable `frame_id` gets a
//! [`rerun::CoordinateFrame`] naming `tf#/<this tree's entity for that
//! frame>`, which Rerun composes through exactly the same nested
//! `Transform3D`s logged below. Path and frame are decoupled, so nothing here
//! needs a tf2-style `lookupTransform`.
//!
//! # Static vs temporal
//!
//! `/tf` transforms are logged TEMPORAL (on the `robot_time` timeline via
//! [`log_transforms`] with `is_static = false`); `/tf_static` transforms are
//! logged with `rerun::RecordingStream::log_static` (`is_static = true`) so
//! they exist on all timelines (Rerun 0.34's static/timeless log API).
//!
//! # Drain-all read
//!
//! `/tf` is an ACCUMULATE-ALL topic (like ROS tf2): sibling broadcasters push
//! different subtrees on the same topic, so a latest-only read would DROP
//! siblings. A drain-all consumer reads EVERY queued frame per tick via
//! `ctx.subscriber("tf").try_receive(...)` + `.with_unified_drain(false)` and
//! calls [`transforms_from_frame`] on each (see the memory-sink e2e). The
//! production macro cdylib sinks read latest-only per fire — see
//! [`log_tf_bytes`] — which is LOSSLESS for the native single-frame producer
//! (`go2_tf_source` publishes the whole subtree per frame); the multi-writer
//! ROS 2 case needs the first-class accumulate-all input.

use std::sync::Mutex;

use go2_tf::{decode_tf_transforms, TfDecodeError, TfTransform};
use rerun::RecordingStream;

use cerulion_core::codegen::{FrameValue, FrameValueKind, FrameWalker, WalkError};
use cerulion_core::wire::WireHeader;

use crate::archetype::set_robot_time;
use crate::pointcloud::{FieldsLogAction, FieldsWarnLatch};

/// Flood latch for the unexpected-`transforms`-field-kind warning in
/// [`transforms_from_frame`] — the function is stateless, so the process
/// shares one latch (the `archetype::GENERIC_FIELDS_LATCH` precedent). See
/// [`FieldsWarnLatch`] for the once-per-regime contract: the first unexpected
/// kind of a regime WARNs, repeats log at debug with a running count, an
/// expected kind re-arms.
static UNEXPECTED_KIND_LATCH: Mutex<FieldsWarnLatch> = Mutex::new(FieldsWarnLatch::new());

// ---- Canonical Go2 entity tree ------------------------------------------

/// The fixed visualization root (ROS Z-up world). See
/// [`log_world_view_coordinates_static`].
pub const WORLD_ROOT: &str = "world";

/// The RESERVED path segment the whole FRAME tree lives under, directly below
/// [`WORLD_ROOT`] — see [`FRAME_ROOT`].
pub const FRAME_SUBTREE_SEGMENT: &str = "tf-tree";

/// The RESERVED root of the FRAME tree (`world/tf-tree`). Every frame entity —
/// the `/tf` child-frame entities below AND the URDF skeleton's
/// [`crate::skeleton::ROBOT_ROOT`] — hangs off this one segment.
///
/// # Why a reserved segment: the topic/frame uniqueness proof
///
/// A topic's entity is now a MECHANICAL function of its name:
/// `world/` + one [`sanitize_segment`] per topic segment (see
/// [`crate::sink::route_for_input`]) — which is unique across TOPICS by
/// construction but says nothing about the FRAME entities the transform tree
/// writes to. Those used to sit directly under `world` (`world/odom`,
/// `world/odom/base`, `world/robot`), so the most canonical ROS topic name of
/// all — `/odom` — resolved to `world/odom`, which is EXACTLY the entity `/tf`
/// logs its `odom` child transform at. Two unrelated `Transform3D`s on one
/// entity under latest-at: the same silent-wrong-answer class this decoupling exists
/// to remove, on `/odom`, `/odom/base` and `/robot`.
///
/// The guard is structural rather than a name blacklist. [`sanitize_segment`]
/// maps every character outside `[A-Za-z0-9_]` to `_` and its disambiguating
/// suffix is `_` + lowercase hex, so **every segment it can ever emit is drawn
/// from `[A-Za-z0-9_]`** (pinned by
/// `sanitize_segment_output_alphabet_is_closed_under_the_reserved_segment`, in
/// this module's test suite).
/// `tf-tree` contains `-`, so no topic name — and no `entity` override, which
/// is routed through the same sanitizer — can produce it. A topic entity and a
/// frame entity therefore cannot collide, for ANY topic name, without a code
/// change to the sanitizer that the alphabet test fails.
///
/// `-` specifically (rather than, say, `#`) because rerun 0.34 prints `-`
/// as-is in an entity path — `EntityPathPart::escaped_string` keeps
/// alphanumerics plus `_ - .` unescaped — so the reserved subtree renders
/// cleanly in the viewer's entity tree and in the blueprint content globs
/// (`/world/tf-tree/robot/**`).
pub const FRAME_ROOT: &str = "world/tf-tree";

/// The odom frame entity (`world/tf-tree/odom`).
pub const ODOM_ENTITY: &str = "world/tf-tree/odom";
/// The robot base entity (`world/tf-tree/odom/base`).
pub const BASE_ENTITY: &str = "world/tf-tree/odom/base";
/// The lidar entity (`world/tf-tree/odom/base/lidar`) — the cloud sink logs
/// here too.
pub const LIDAR_ENTITY: &str = "world/tf-tree/odom/base/lidar";
/// The camera entity (`world/tf-tree/odom/base/camera`) — the image sink + the
/// `Pinhole` intrinsics log here too.
pub const CAMERA_ENTITY: &str = "world/tf-tree/odom/base/camera";

// ---- Go2 720p camera intrinsics --------------------------------

/// Go2 front-camera width (px).
pub const GO2_CAMERA_WIDTH: u32 = 1280;
/// Go2 front-camera height (px).
pub const GO2_CAMERA_HEIGHT: u32 = 720;
/// Representative focal length (px) for the Go2 720p front camera.
///
/// Placeholder intrinsics: replace them with your camera's calibration (its
/// `camera_info`). ~650 px is a typical focal for a
/// wide-FOV 720p sensor and gives a plausible frustum until then.
pub const GO2_CAMERA_FOCAL_PX: f32 = 650.0;

/// Constant 2 (camera pinhole): the camera intrinsics logged as the
/// static [`rerun::Pinhole`]. `Default` is the Go2 720p front camera
/// ([`GO2_CAMERA_WIDTH`] × [`GO2_CAMERA_HEIGHT`], [`GO2_CAMERA_FOCAL_PX`]) so
/// the demo stays byte-identical; the `cerulion viz` verb (in later
/// work) supplies a per-camera intrinsic.
#[derive(Debug, Clone, PartialEq)]
pub struct CameraConfig {
    /// Image width (px).
    pub width: u32,
    /// Image height (px).
    pub height: u32,
    /// Focal length (px), used for both fx and fy.
    pub focal_px: f32,
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            width: GO2_CAMERA_WIDTH,
            height: GO2_CAMERA_HEIGHT,
            focal_px: GO2_CAMERA_FOCAL_PX,
        }
    }
}

impl CameraConfig {
    /// The Rerun `Pinhole` archetype these intrinsics resolve to.
    pub fn pinhole(&self) -> rerun::Pinhole {
        rerun::Pinhole::from_focal_length_and_resolution(self.focal(), self.resolution())
    }

    /// The `[fx, fy]` focal pair the pinhole is built from — the assertable
    /// flow-through observable (`rerun::Pinhole` itself is opaque).
    pub fn focal(&self) -> [f32; 2] {
        [self.focal_px, self.focal_px]
    }

    /// The `[width, height]` resolution the pinhole is built from — the
    /// assertable flow-through observable.
    pub fn resolution(&self) -> [f32; 2] {
        [self.width as f32, self.height as f32]
    }
}

// ---- frame_id → entity path ----------------------------------------------

/// The resolved entity path for a frame plus whether it was a KNOWN frame
/// (an unknown frame took the fallback path — the caller warns once).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FramePath {
    /// The Rerun entity path the transform is logged at.
    pub path: String,
    /// `true` if `frame` matched the known-frame table; `false` = fallback.
    pub known: bool,
}

/// Constant 5 (entity topology): the frame_id → entity-path tree.
/// `Default` reproduces the exact Go2 topology (`world → odom → base →
/// {lidar, camera}` with the Go2 frame-id aliases) so the demo stays
/// byte-identical; the `cerulion viz` verb (in later work) supplies a
/// per-robot tree. Each `*_frames` list holds the child frame-id aliases that
/// map to the corresponding `*_entity`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EntityTopology {
    /// The viz root entity (Z-up world).
    pub world_root: String,
    /// The odom-frame entity.
    pub odom_entity: String,
    /// The robot-base entity (also the fallback parent for unknown frames).
    pub base_entity: String,
    /// The lidar entity (the cloud sink logs here too).
    pub lidar_entity: String,
    /// The camera entity (the image sink + the `Pinhole` intrinsics log here).
    pub camera_entity: String,
    /// Child frame-ids that map to [`Self::odom_entity`].
    pub odom_frames: Vec<String>,
    /// Child frame-ids that map to [`Self::base_entity`].
    pub base_frames: Vec<String>,
    /// Child frame-ids that map to [`Self::lidar_entity`].
    pub lidar_frames: Vec<String>,
    /// Child frame-ids that map to [`Self::camera_entity`].
    pub camera_frames: Vec<String>,
}

impl Default for EntityTopology {
    fn default() -> Self {
        let s = |v: &[&str]| v.iter().map(|x| x.to_string()).collect::<Vec<_>>();
        Self {
            world_root: WORLD_ROOT.to_string(),
            odom_entity: ODOM_ENTITY.to_string(),
            base_entity: BASE_ENTITY.to_string(),
            lidar_entity: LIDAR_ENTITY.to_string(),
            camera_entity: CAMERA_ENTITY.to_string(),
            odom_frames: s(&["odom", "odom_frame"]),
            base_frames: s(&["base", "base_link", "base_footprint"]),
            lidar_frames: s(&["lidar", "livox_frame", "utlidar_lidar", "laser", "velodyne"]),
            camera_frames: s(&[
                "camera",
                "camera_link",
                "front_camera",
                "camera_optical_frame",
            ]),
        }
    }
}

impl EntityTopology {
    /// Map a child `frame_id` to its Rerun entity path. Known frames (per the
    /// `*_frames` alias lists) map to the corresponding entity; anything else
    /// falls back under [`Self::base_entity`] with the segment sanitized (Rerun
    /// entity segments can't contain `/`) — and, when sanitization changed the
    /// id, a 4-hex-char FNV suffix disambiguating raw ids that would otherwise
    /// collapse onto one segment (see `sanitize_segment`) — and `known = false`
    /// so the caller can warn once.
    pub fn entity_path(&self, frame: &str) -> FramePath {
        let f = frame.trim_start_matches('/');
        let known = if self.odom_frames.iter().any(|s| s == f) {
            Some(&self.odom_entity)
        } else if self.base_frames.iter().any(|s| s == f) {
            Some(&self.base_entity)
        } else if self.lidar_frames.iter().any(|s| s == f) {
            Some(&self.lidar_entity)
        } else if self.camera_frames.iter().any(|s| s == f) {
            Some(&self.camera_entity)
        } else {
            None
        };
        match known {
            Some(p) => FramePath {
                path: p.clone(),
                known: true,
            },
            None => FramePath {
                path: format!("{}/{}", self.base_entity, sanitize_segment(f)),
                known: false,
            },
        }
    }
}

/// The process-shared default (Go2) entity topology — built once so the
/// per-transform [`entity_path_for_frame`] pays no allocation per call.
fn default_topology() -> &'static EntityTopology {
    static TOPOLOGY: std::sync::OnceLock<EntityTopology> = std::sync::OnceLock::new();
    TOPOLOGY.get_or_init(EntityTopology::default)
}

/// Map a child `frame_id` to its Rerun entity path under the default (Go2)
/// topology. Thin wrapper over [`EntityTopology::entity_path`] on the shared
/// `default_topology` — byte-identical to the earlier static match, and the
/// production per-transform path.
pub fn entity_path_for_frame(frame: &str) -> FramePath {
    default_topology().entity_path(frame)
}

/// Sanitize a frame id into a single Rerun entity-path segment: ASCII
/// alphanumerics and `_` survive; everything else becomes `_`. An empty
/// result degrades to `unknown` (never an empty segment).
///
/// **Aliasing guard**: whenever sanitization CHANGED the string (or emptied
/// it), a short stable hash of the RAW id — 4 hex chars of FNV-1a 64 — is
/// appended (`weird.frame` → `weird_frame_<hhhh>`). Without it, distinct raw
/// ids differing only in separators (`a/b` vs `a.b`) would collapse onto ONE
/// entity while the unknown-frame warn latch keys on the raw string — two
/// frames silently overwriting each other's transform. Unchanged ids get NO
/// suffix (`imu_link` stays `imu_link`).
///
/// **Scope of that guard** — it separates the SEPARATOR-VARIANT family it
/// was written for, not every conceivable pair:
///
/// - The suffixed form shares one namespace with untouched identifiers, because
///   an already-valid name is returned verbatim. So a raw name that already LOOKS
///   suffixed aliases the suffixed form of another: `a.b` → `a_b_522c`, and a raw
///   `a_b_522c` → `a_b_522c`. Contrived on real names (it requires a name ending
///   in `_` + four hex that is also the FNV of a sibling), and the cost is the
///   pre-guard behaviour for that one pair.
/// - The hash is masked to 16 bits, so two raw ids sharing a sanitized BASE
///   collide 1-in-65536. Real same-base groups are size 1–2, so it is a stable
///   short discriminator, not a collision-free identifier.
///
/// Neither weakens the TOPIC/FRAME uniqueness proof on [`FRAME_ROOT`], which
/// rests only on the output ALPHABET (`[A-Za-z0-9_]`), never on the hash.
///
/// **This is the ONE entity-segment sanitizer for the whole viz
/// stack.** `sink.rs` (topic → entity path) and `skeleton.rs` (URDF link →
/// entity path) previously each carried a private copy that kept the SAME
/// character map but DIFFERENT fallbacks and, crucially, NO aliasing suffix —
/// so `a/b` and `a.b` collapsed onto one segment on exactly the path that
/// builds topic entities. Those copies are deleted; every entity segment in
/// the crate now goes through here, pinned by
/// `sanitize_segment_matches_the_hand_vector_table`. (Until recently there was a
/// second, deliberately DIVERGENT sanitizer — `cerulion_cli_engine`'s
/// `sanitize_input_name`, which named `ros2 attach`'s generated viz-node inputs.
/// Deleting robot-side viz generation took that function with it, so this
/// is now the only one anywhere.)
pub fn sanitize_segment(s: &str) -> String {
    let out: String = s
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect();
    let base = if out.is_empty() {
        "unknown".to_string()
    } else {
        out
    };
    if base == s {
        base
    } else {
        format!("{base}_{:04x}", fnv1a64(s.as_bytes()) & 0xFFFF)
    }
}

/// FNV-1a 64 (the house schema-hash primitive's algorithm) over raw bytes —
/// used only to disambiguate sanitized entity segments. Oracle-pinned below
/// against the published FNV-1a 64 test vectors.
fn fnv1a64(bytes: &[u8]) -> u64 {
    const OFFSET_BASIS: u64 = 0xcbf2_9ce4_8422_2325;
    const PRIME: u64 = 0x0000_0100_0000_01b3;
    let mut hash = OFFSET_BASIS;
    for &b in bytes {
        hash ^= b as u64;
        hash = hash.wrapping_mul(PRIME);
    }
    hash
}

/// The rerun 0.34 transform-frame name for an entity's IMPLICIT frame. Every
/// entity has one (`tf#/<entity path>` — note the `/`, which the code emits and
/// which is part of the name — identity-connected to its path parent),
/// and [`rerun::CoordinateFrame`] re-points an entity at a DIFFERENT frame by
/// name — which is how the design decouples "where a topic lives in the tree" from
/// "where its geometry is posed".
///
/// Naming an entity's OWN implicit frame is how a re-pointed entity is put BACK
/// (`crate::sink`'s clear path): a `CoordinateFrame` is temporal + latest-at, so
/// an assignment is never forgotten, only overwritten.
pub fn implicit_frame_of(entity_path: &str) -> String {
    format!("tf#/{}", entity_path.trim_matches('/'))
}

/// The implicit frame of `entity_path`'s PATH PARENT — the frame a
/// `Transform3D` at `entity_path` is expressed in when its `parent_frame`
/// component is null (rerun 0.34's default, `re_tf`'s
/// `transform_forest::implicit_transform_parent`).
///
/// Naming it EXPLICITLY is how a transform payload is put back after having been
/// posed. Note this is equivalent to OMITTING the component, not a correction for
/// it: rerun 0.34 resolves a `Transform3D` ATOMICALLY PER ROW —
/// `re_tf`'s `query_and_resolve_tree_transform_at_entity` finds the last CHANGED
/// row and reads every component off THAT row id ("we don't have to do latest-at
/// for individual components"), and `get_parent_frame` falls back to the entity's
/// PATH parent whenever the row carries none. So a row without `parent_frame` is
/// RESET to the path parent, not left at the previous row's value: the frame must
/// be written on EVERY row, and the change-triggered dedup `emit_frame_at` uses for
/// `CoordinateFrame` would un-pose every message after the first.
///
/// The explicit form is a legibility choice — the un-posed case becomes an
/// asserted value rather than an absence. `None` when the entity has no parent
/// segment (the root).
pub fn implicit_parent_frame_of(entity_path: &str) -> Option<String> {
    let trimmed = entity_path.trim_matches('/');
    let (parent, _) = trimmed.rsplit_once('/')?;
    if parent.is_empty() {
        return None;
    }
    Some(implicit_frame_of(parent))
}

/// Read a message's `frame_id` — the ROS convention for "which coordinate frame
/// is this data in" — from EITHER shape it takes on the wire.
///
/// Almost every stamped ROS type nests it in a `std_msgs/Header`
/// (`header.frame_id`: PointCloud2, Odometry, Imu, PoseStamped, …), but some
/// vendor types carry it TOP-LEVEL with no `Header` at all — verified against
/// the Go2's own served schema for `unitree_go/HeightMap`, which is a bare
/// `float64 stamp` + `string frame_id`. A `header`-only extractor MISSES those
/// silently, which is why both shapes are read here. The nested form is tried
/// first (it is the convention; a type carrying both would mean the header).
///
/// An empty `frame_id` reads as ABSENT: ROS treats `""` as "no frame stated",
/// and resolving it would fabricate a pose.
pub fn frame_id_of<'a>(fv: &FrameValue<'a>) -> Option<&'a str> {
    let nested = match fv.field("header") {
        Some(FrameValueKind::Nested(inner)) => match inner.field("frame_id") {
            Some(FrameValueKind::Str(s)) => Some(*s),
            _ => None,
        },
        _ => None,
    };
    let raw = match nested {
        Some(s) => Some(s),
        None => match fv.field("frame_id") {
            Some(FrameValueKind::Str(s)) => Some(*s),
            _ => None,
        },
    };
    raw.map(str::trim).filter(|s| !s.is_empty())
}

/// The frames this run has OBSERVED as `/tf` / `/tf_static` child
/// frames, and the rule that turns a message's `frame_id` into the
/// [`rerun::CoordinateFrame`] name that poses its entity.
///
/// A topic's entity path is now its own (`world/<topic>`), so nothing about the
/// path says where the data sits in space. [`Self::resolve`] answers that
/// separately by naming the frame the `/tf` tree already logs its transforms at
/// — so Rerun composes the chain and nothing here does a tf2-style
/// `lookupTransform` (still out of scope).
///
/// # Cost and bound
///
/// [`Self::observe_child`] runs once per transform per `/tf` frame (~100 Hz ×
/// ~15 transforms on a Go2), so it takes a `contains` fast path and allocates
/// NOTHING once the set reaches steady state — which is the first message on a
/// robot with a fixed frame topology.
///
/// The set is CAPPED at [`MAX_OBSERVED_FRAMES`]. A robot's frame cardinality is a
/// handful, but a publisher that mints a frame PER OBJECT (an AR-tag tracker
/// emitting `tag_36h11_<id>`, per-detection frames) would otherwise grow it
/// without limit in a desk daemon that runs for hours. At the cap the registry
/// STOPS LEARNING rather than evicting: every frame already observed keeps
/// resolving (eviction would silently un-pose a live topic), and a new one is
/// simply unresolvable — the same explicit "cannot place this" answer an unknown
/// frame already gets, which the caller warns about once per input. Known
/// aliases are a static table and are never affected.
#[derive(Debug, Default, Clone)]
pub struct FrameRegistry {
    seen: std::collections::BTreeSet<String>,
    /// True once the cap was hit — so the "no longer learning" warn fires once.
    saturated: bool,
}

/// The cap on [`FrameRegistry`]'s observed-frame set. Two orders of magnitude
/// above any real robot's frame cardinality (a Go2 publishes 4), so it is only
/// ever reached by a publisher minting frames per object.
pub const MAX_OBSERVED_FRAMES: usize = 4096;

impl FrameRegistry {
    /// A fresh registry that has observed nothing.
    pub fn new() -> Self {
        Self::default()
    }

    /// Record that `frame` appeared as a transform's CHILD frame — i.e. that the
    /// `/tf` tree carries a transform positioning it, logged at
    /// [`entity_path_for_frame`]'s entity for it.
    pub fn observe_child(&mut self, frame: &str) {
        let f = frame.trim().trim_start_matches('/');
        // Hot path (once per transform per /tf frame): at steady state this is a
        // lookup and NOTHING is allocated.
        if f.is_empty() || self.seen.contains(f) {
            return;
        }
        if self.seen.len() >= MAX_OBSERVED_FRAMES {
            if !self.saturated {
                self.saturated = true;
                tracing::warn!(
                    observed = self.seen.len(),
                    frame = f,
                    "cerulion_viz: /tf has published more than {MAX_OBSERVED_FRAMES} distinct child \
                     frames, so no FURTHER frame will be learned this run (a publisher minting a \
                     frame per object?); frames already observed keep resolving, and a topic \
                     stamped with a new one renders unposed rather than at a fabricated mount"
                );
            }
            return;
        }
        self.seen.insert(f.to_string());
    }

    /// The number of `/tf` child frames observed this run — bounded by
    /// [`MAX_OBSERVED_FRAMES`].
    pub fn observed_len(&self) -> usize {
        self.seen.len()
    }

    /// True if `frame` has been observed on `/tf` / `/tf_static` this run.
    pub fn has_observed(&self, frame: &str) -> bool {
        self.seen.contains(frame.trim().trim_start_matches('/'))
    }

    /// The `CoordinateFrame` name that poses a message stamped `frame_id`, or
    /// `None` when the frame cannot be resolved.
    ///
    /// Resolvable means one of two things: the id is in the [`EntityTopology`]
    /// alias table (a KNOWN frame — the tree states where it belongs), or the
    /// `/tf` tree has actually published a transform for it this run
    /// ([`Self::observe_child`]). Either way the answer is the implicit frame of
    /// the entity `/tf` logs that transform at, so the pose composes through the
    /// SAME nested `Transform3D`s already in the recording.
    ///
    /// **An UNRESOLVABLE frame returns `None`, and the caller must log
    /// nothing.** The tempting alternative — naming
    /// `entity_path_for_frame`'s fallback entity (`world/tf-tree/odom/base/<sanitized>`)
    /// — is a silent WRONG ANSWER: that frame is connected to `world` by
    /// IDENTITY through its path parents, so the viewer would render the data as
    /// if the sensor sat exactly at the robot base. A fabricated mount is worse
    /// than an unposed entity, which at least renders at the world origin and is
    /// visibly not localized. (The design expected the viewer to DROP an
    /// unconnected frame instead; that is not what happens for a `tf#/world/...`
    /// name, so the reason to withhold is fabrication, not invisibility.)
    pub fn resolve(&self, frame_id: &str) -> Option<String> {
        let fp = entity_path_for_frame(frame_id);
        if fp.known || self.has_observed(frame_id) {
            Some(implicit_frame_of(&fp.path))
        } else {
            None
        }
    }
}

/// Once-per-distinct-frame flood latch for the unknown-frame warning — the
/// [`FieldsWarnLatch`] house pattern,
/// specialized to frames: the FIRST time a given unknown frame id is seen it
/// WARNs; repeats of the SAME frame stay silent (so a steady stream of an
/// unknown frame at lidar rate does not flood). Bounded by the robot's frame
/// cardinality (a handful). Pure — the latch never logs itself.
#[derive(Debug, Default, Clone)]
pub struct UnknownFrameLog {
    seen: std::collections::BTreeSet<String>,
}

impl UnknownFrameLog {
    /// A fresh latch that has seen nothing.
    pub fn new() -> Self {
        Self::default()
    }

    /// Record one sighting of an unknown `frame`. Returns `true` on the FIRST
    /// sighting (the caller warns), `false` for a repeat (silent).
    pub fn observe(&mut self, frame: &str) -> bool {
        self.seen.insert(frame.to_string())
    }
}

// ---- Rerun archetype builders + logging ----------------------------------

/// Build a [`rerun::Transform3D`] from a decoded transform. Translation and
/// the quaternion are narrowed `f64 → f32` (Rerun is f32 — a viz-precision
/// narrowing, documented like the pointcloud codec's).
pub fn tf_transform3d(t: &TfTransform) -> rerun::Transform3D {
    let translation = [
        t.translation[0] as f32,
        t.translation[1] as f32,
        t.translation[2] as f32,
    ];
    // `rerun::Quaternion` is `[x, y, z, w]` (same order as ROS + our codec).
    let rotation = rerun::Quaternion::from_xyzw([
        t.rotation[0] as f32,
        t.rotation[1] as f32,
        t.rotation[2] as f32,
        t.rotation[3] as f32,
    ]);
    rerun::Transform3D::from_translation_rotation(translation, rotation)
}

/// Log one transform at its child-frame entity. `is_static` routes to
/// `RecordingStream::log_static` (for `/tf_static`) vs a temporal
/// `RecordingStream::log` on the `robot_time` timeline (for `/tf`).
fn log_one(
    rec: &RecordingStream,
    timestamp_ns: u64,
    t: &TfTransform,
    is_static: bool,
    unknown: &mut UnknownFrameLog,
) {
    let fp = entity_path_for_frame(&t.child_frame_id);
    if !fp.known && unknown.observe(&t.child_frame_id) {
        tracing::warn!(
            child_frame_id = %t.child_frame_id,
            parent = %t.frame_id,
            entity = %fp.path,
            "TF: unknown child frame — using the fallback entity path under base \
             (segment sanitized + hash-suffixed when the raw id needed \
             sanitizing; add it to entity_path_for_frame's table if it should \
             map elsewhere)"
        );
    }
    let archetype = tf_transform3d(t);
    let result = if is_static {
        rec.log_static(fp.path.clone(), &archetype)
    } else {
        set_robot_time(rec, timestamp_ns);
        rec.log(fp.path.clone(), &archetype)
    };
    if let Err(e) = result {
        tracing::warn!(error = %e, entity = %fp.path, is_static, "Rerun: Transform3D log failed");
    }
}

/// Log every transform in a decoded set. `is_static = false` logs them on the
/// `robot_time` timeline (the `timestamp_ns` wire stamp); `is_static = true`
/// logs them via `log_static` (the `/tf_static` path — `timestamp_ns`
/// ignored).
pub fn log_transforms(
    rec: &RecordingStream,
    timestamp_ns: u64,
    transforms: &[TfTransform],
    is_static: bool,
    unknown: &mut UnknownFrameLog,
) {
    for t in transforms {
        log_one(rec, timestamp_ns, t, is_static, unknown);
    }
}

/// Decode an opaque `transforms` blob (`TFMessage.transforms_bytes()`) and log
/// every transform. An undecodable blob is a loud warning + skipped frame
/// (never a tick failure — visualization is best-effort). The production macro
/// sinks call this with the typed accessor's bytes + `wire_timestamp_ns()`.
pub fn log_tf_bytes(
    rec: &RecordingStream,
    transforms_bytes: &[u8],
    timestamp_ns: u64,
    is_static: bool,
    unknown: &mut UnknownFrameLog,
) {
    match decode_tf_transforms(transforms_bytes) {
        Ok(transforms) => log_transforms(rec, timestamp_ns, &transforms, is_static, unknown),
        Err(e) => {
            tracing::warn!(error = %e, "TF: undecodable transforms blob — skipping frame");
        }
    }
}

/// A decode failure on a raw TFMessage wire frame (the walker path).
#[derive(Debug, thiserror::Error)]
pub enum TfFrameError {
    /// The frame is shorter than a `WireHeader`.
    #[error("frame too short for a WireHeader ({0} bytes)")]
    ShortFrame(usize),
    /// The generic frame walker rejected the frame.
    #[error("frame walk failed: {0}")]
    Walk(#[from] WalkError),
    /// The `transforms` element blob was undecodable.
    #[error("transforms decode failed: {0}")]
    Decode(#[from] TfDecodeError),
}

/// Decode a FULL TFMessage wire frame (WireHeader included) into its wire
/// timestamp + transforms, via the generic [`FrameWalker`] — the cross-check
/// that the walker surfaces exactly the opaque `transforms` bytes
/// [`decode_tf_transforms`] consumes. This is the drain-all consumer's path:
/// it reconstructs each queued frame and calls this to get the accumulate-all
/// transforms.
pub fn transforms_from_frame(
    walker: &FrameWalker,
    frame: &[u8],
) -> Result<(u64, Vec<TfTransform>), TfFrameError> {
    let header = WireHeader::read_from_buf(frame).ok_or(TfFrameError::ShortFrame(frame.len()))?;
    let fv = walker.walk_by_hash(frame)?;
    // The walker surfaces `TFMessage.transforms` (a DynamicArray<Nested>) as
    // the field's raw bytes — exactly what our element codec decodes. Any
    // OTHER kind is unexpected (a walker/schema drift signal): decode as zero
    // transforms but say so LOUDLY — once-per-regime via the shared latch,
    // never silently.
    //
    // The walker has a canonical element framing, so a
    // canonically-framed `/tf` now arrives as a DECODED `NestedArray` — which
    // still carries the identical field slice in `raw`. Reading `raw` keeps
    // this decoder's input byte-for-byte what it was before element framing: an empty
    // array decodes to zero transforms, and a canonically-framed one (which
    // this module's bespoke convention cannot read) surfaces the TRUTHFUL
    // `TfDecodeError` from `decode_tf_transforms` rather than being reported
    // as "the field is missing" or as walker drift. Consuming the walker's
    // decoded `elements` — i.e. unifying the two conventions — is not
    // done here.
    let (bytes, expected): (&[u8], bool) = match fv.field("transforms") {
        Some(FrameValueKind::NestedArrayOpaque(b)) => (*b, true),
        Some(FrameValueKind::Bytes(b)) => (*b, true),
        Some(FrameValueKind::NestedArray { raw, .. }) => (*raw, true),
        other => {
            let kind = kind_name(other);
            let mut latch = UNEXPECTED_KIND_LATCH
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            match latch.on_inferred() {
                FieldsLogAction::WarnFirst => tracing::warn!(
                    kind,
                    schema = %fv.schema_name,
                    "TF: walker surfaced an unexpected `transforms` field kind — \
                     decoding as ZERO transforms (expected NestedArrayOpaque, \
                     NestedArray or Bytes; walker/schema drift?). Repeats log at \
                     debug"
                ),
                FieldsLogAction::DebugSuppressed { suppressed } => tracing::debug!(
                    kind,
                    suppressed,
                    "TF: unexpected `transforms` field kind sustained (warn suppressed)"
                ),
            }
            (&[], false)
        }
    };
    if expected {
        // An expected kind re-arms the latch; recovery from a suppressed
        // regime logs once (the FieldsWarnLatch contract).
        let mut latch = UNEXPECTED_KIND_LATCH
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        if let Some(suppressed) = latch.on_decoded() {
            tracing::info!(
                suppressed_count = suppressed,
                "TF: `transforms` field kind expected again — unexpected-kind \
                 regime healed"
            );
        }
    }
    let transforms = decode_tf_transforms(bytes)?;
    Ok((header.timestamp_ns, transforms))
}

/// Stable name of a walker field-kind for diagnostics (never dumps payload
/// contents — a `Debug` render of `Bytes`/`PrimArray` could be huge).
fn kind_name(v: Option<&FrameValueKind<'_>>) -> &'static str {
    match v {
        None => "missing",
        Some(FrameValueKind::Bool(_)) => "Bool",
        Some(FrameValueKind::I8(_)) => "I8",
        Some(FrameValueKind::U8(_)) => "U8",
        Some(FrameValueKind::I16(_)) => "I16",
        Some(FrameValueKind::U16(_)) => "U16",
        Some(FrameValueKind::I32(_)) => "I32",
        Some(FrameValueKind::U32(_)) => "U32",
        Some(FrameValueKind::I64(_)) => "I64",
        Some(FrameValueKind::U64(_)) => "U64",
        Some(FrameValueKind::F32(_)) => "F32",
        Some(FrameValueKind::F64(_)) => "F64",
        Some(FrameValueKind::Str(_)) => "Str",
        Some(FrameValueKind::Bytes(_)) => "Bytes",
        Some(FrameValueKind::PrimArray(_)) => "PrimArray",
        Some(FrameValueKind::Nested(_)) => "Nested",
        Some(FrameValueKind::Array(_)) => "Array",
        Some(FrameValueKind::NestedArray { .. }) => "NestedArray",
        Some(FrameValueKind::NestedArrayOpaque(_)) => "NestedArrayOpaque",
    }
}

/// Guard for the scene statics (see [`log_viz_statics_once`]). An
/// `AtomicBool` rather than `std::sync::Once` DELIBERATELY: a `Once` is
/// unresettable, so [`crate::stream::reset_for_test`] could reset the stream
/// but never the statics guard — a second same-process graph test would
/// silently lose the statics (and shift every exact chunk-count oracle). The
/// `swap` below keeps the same structurally-exactly-once guarantee.
static VIZ_STATICS_LOGGED: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

/// Log the scene-level statics — the world `ViewCoordinates` (ROS Z-up) and
/// the camera `Pinhole` intrinsics — exactly ONCE per process (per
/// [`rearm_viz_statics`] epoch in tests), from whichever sink fires
/// FIRST (every Go2 sink calls this at the top of its tick; the `stream`
/// module's shared-identity pattern class).
///
/// Why not tf-sink-local: on an incremental bring-up day the camera/lidar
/// (or /tf_static alone) can flow BEFORE any /tf broadcaster exists — a
/// tf-sink-owned statics block would leave the whole scene without Z-up view
/// coordinates and the camera view unselectable until /tf first fired.
/// Exactly-once is structural even when
/// several sinks race their first fires: `swap(true, SeqCst)` returns `false`
/// to exactly ONE caller — the same guarantee a `Once` gives, but resettable
/// for the test seam (see [`rearm_viz_statics`]).
pub fn log_viz_statics_once(rec: &RecordingStream) {
    if !VIZ_STATICS_LOGGED.swap(true, std::sync::atomic::Ordering::SeqCst) {
        log_world_view_coordinates_static(rec);
        log_camera_pinhole_static(rec);
    }
}

/// Re-arm the scene-statics guard so the world statics re-log on the next sink
/// fire. Two production/test callers: [`crate::stream::rearm_after_reconnect`]
/// (a bounced server re-receives the statics) and
/// [`crate::stream::reset_for_test`] (test hygiene — a later same-process graph
/// test re-receives the statics on its first sink fire, keeping its exact
/// chunk-count oracles valid regardless of test ordering).
pub fn rearm_viz_statics() {
    VIZ_STATICS_LOGGED.store(false, std::sync::atomic::Ordering::SeqCst);
}

/// Log the Go2 camera `Pinhole` intrinsics under [`CAMERA_ENTITY`] as STATIC
/// (they don't change). This makes the camera view selectable in the viewer
/// (view-from-camera + the cloud projected into the image plane).
pub fn log_camera_pinhole_static(rec: &RecordingStream) {
    let pinhole = CameraConfig::default().pinhole();
    if let Err(e) = rec.log_static(CAMERA_ENTITY, &pinhole) {
        tracing::warn!(error = %e, entity = CAMERA_ENTITY, "Rerun: camera Pinhole log failed");
    }
}

/// Stamp the world root with the ROS right-handed Z-up view coordinates
/// (STATIC), so the whole transform tree renders in the robot's convention.
pub fn log_world_view_coordinates_static(rec: &RecordingStream) {
    if let Err(e) = rec.log_static(WORLD_ROOT, &rerun::ViewCoordinates::RIGHT_HAND_Z_UP()) {
        tracing::warn!(error = %e, entity = WORLD_ROOT, "Rerun: world ViewCoordinates log failed");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_frames_map_to_the_canonical_tree() {
        // Hand oracles for the entity paths (the authoritative path assertion).
        assert_eq!(
            entity_path_for_frame("odom"),
            FramePath {
                path: "world/tf-tree/odom".to_string(),
                known: true
            }
        );
        assert_eq!(
            entity_path_for_frame("base_link"),
            FramePath {
                path: "world/tf-tree/odom/base".to_string(),
                known: true
            }
        );
        assert_eq!(
            entity_path_for_frame("lidar"),
            FramePath {
                path: "world/tf-tree/odom/base/lidar".to_string(),
                known: true
            }
        );
        assert_eq!(
            entity_path_for_frame("camera").path,
            "world/tf-tree/odom/base/camera"
        );
        // Leading '/' is stripped before matching (absolute frame ids).
        assert!(entity_path_for_frame("/lidar").known);
    }

    #[test]
    fn unknown_frame_takes_sanitized_fallback_under_base() {
        // An already-clean id gets NO hash suffix (sanitization unchanged).
        let fp = entity_path_for_frame("imu_link");
        assert_eq!(fp.path, "world/tf-tree/odom/base/imu_link");
        assert!(!fp.known);
        // Non-segment characters are sanitized to '_' + a 4-hex-char FNV
        // suffix of the raw id (sanitization changed the string).
        let weird = entity_path_for_frame("weird frame/name").path;
        assert!(
            weird.starts_with("world/tf-tree/odom/base/weird_frame_name_"),
            "sanitized-and-changed ids carry a hash suffix: {weird}"
        );
        let suffix = weird.rsplit('_').next().expect("suffix segment");
        assert_eq!(suffix.len(), 4, "suffix is 4 hex chars: {weird}");
        assert!(suffix.chars().all(|c| c.is_ascii_hexdigit()));
        // An all-symbol frame degrades to a non-empty segment; it trims to ""
        // and hashes it — fnv1a64("") is the offset basis 0xcbf29ce484222325,
        // low 16 bits 0x2325 (a hand literal from the published FNV vectors).
        assert_eq!(
            entity_path_for_frame("///").path,
            "world/tf-tree/odom/base/unknown_2325"
        );
    }

    /// The aliasing guard: distinct raw ids that
    /// sanitize to the SAME base segment must land on DISTINCT entity paths
    /// (without the suffix, `a/b` and `a.b` both map to `world/odom/base/a_b` while
    /// the unknown-frame warn latch keys on the raw strings).
    #[test]
    fn sanitized_aliases_get_distinct_entity_paths() {
        let slash = entity_path_for_frame("a/b").path;
        let dot = entity_path_for_frame("a.b").path;
        assert!(slash.starts_with("world/tf-tree/odom/base/a_b_"), "{slash}");
        assert!(dot.starts_with("world/tf-tree/odom/base/a_b_"), "{dot}");
        assert_ne!(
            slash, dot,
            "raw ids differing only in separators must not collapse"
        );
        // Deterministic: the same raw id always maps to the same path.
        assert_eq!(entity_path_for_frame("a/b").path, slash);
        // And neither collides with a genuinely clean `a_b` (no suffix).
        assert_eq!(
            entity_path_for_frame("a_b").path,
            "world/tf-tree/odom/base/a_b"
        );
    }

    /// `sanitize_segment` pinned against a HAND-WRITTEN vector table.
    ///
    /// This is the ONE entity-segment sanitizer in the crate — `sink.rs` (topic →
    /// entity path) and `skeleton.rs` (URDF link → entity path) each carried a
    /// private copy that kept the SAME character map but DIFFERENT fallbacks and,
    /// crucially, NO aliasing suffix, so `a/b` and `a.b` collapsed onto one segment
    /// on exactly the path that builds topic entities. Those copies are deleted.
    ///
    /// The first version of this test compared a second column against
    /// `cerulion_cli_engine::ros_cmd::sanitize_input_name`, whose deliberate
    /// divergence (different fallbacks, no suffix) had to be declared because a
    /// graph input name fed this sanitizer. The deletion of `ros2 attach`'s viz-node
    /// generation took that whole function with it, so there is no second sanitizer
    /// left to diverge from and no shared fixture to keep in step — the table is
    /// self-contained again.
    #[test]
    fn sanitize_segment_matches_the_hand_vector_table() {
        // (raw, expected)
        let table: &[(&str, &str)] = &[
            // Already-valid identifiers pass through untouched — no suffix.
            ("cloud", "cloud"),
            ("imu_data", "imu_data"),
            ("tf_static", "tf_static"),
            ("MiXeD9", "MiXeD9"),
            // Anything else is mapped to `_` AND carries the 4-hex FNV of the RAW
            // string, which is what keeps separator variants apart.
            ("wrist cam!", "wrist_cam__38ca"),
            ("naïve", "na_ve_32ab"),
            ("a/b", "a_b_cf61"),
            ("a.b", "a_b_522c"),
            ("a-b", "a_b_4883"),
            // Empty / all-invalid degrade to a NON-EMPTY segment, still suffixed.
            ("", "unknown_2325"),
            ("///", "____3e64"),
            ("...", "____1219"),
        ];
        for (raw, expected) in table {
            assert_eq!(
                &sanitize_segment(raw) as &str,
                *expected,
                "sanitize({raw:?})"
            );
        }
        // The three `a_b` rows prove the suffix is load-bearing, not decorative.
        assert_ne!(sanitize_segment("a/b"), sanitize_segment("a.b"));
        assert_ne!(sanitize_segment("a/b"), sanitize_segment("a-b"));
        // …and that a genuinely clean `a_b` does not collide with any of them.
        assert_eq!(sanitize_segment("a_b"), "a_b");
    }

    /// THE [`FRAME_ROOT`] UNIQUENESS PROOF.
    ///
    /// A topic's entity is `world/` + one [`sanitize_segment`] per topic segment, so
    /// a topic can NEVER reach the frame tree iff no `sanitize_segment` output can
    /// equal [`FRAME_SUBTREE_SEGMENT`]. This asserts the two halves of that: the
    /// sanitizer's output alphabet is closed under `[A-Za-z0-9_]`, and the reserved
    /// segment is outside it.
    ///
    /// Deliberately property-shaped rather than a name list: a blacklist of "topics
    /// that must not collide" reproduces the very failure this fixes (nothing
    /// enumerated `/odom`). Widening the sanitizer's alphabet — the one change that
    /// could re-open the hole — fails HERE.
    #[test]
    fn sanitize_segment_output_alphabet_is_closed_under_the_reserved_segment() {
        // Adversarial corpus: separators, punctuation, unicode, empties, the
        // reserved segment itself, and the frame-tree names a topic might share.
        let corpus = [
            "",
            "/",
            "///",
            "...",
            "a/b",
            "a.b",
            "a-b",
            "odom",
            "robot",
            "tf-tree",
            "world/tf-tree/odom",
            "wrist cam!",
            "naïve",
            "-",
            "--",
            "__",
            "_-_",
            "tf-tree-2",
            "\u{262E}",
            "a\tb",
            "9lives",
            "MiXeD",
        ];
        for raw in corpus {
            let out = sanitize_segment(raw);
            assert!(
                !out.is_empty(),
                "sanitize_segment({raw:?}) must never be empty"
            );
            assert!(
                out.chars().all(|c| c.is_ascii_alphanumeric() || c == '_'),
                "sanitize_segment({raw:?}) = {out:?} escaped the [A-Za-z0-9_] alphabet — \
                 the FRAME_ROOT uniqueness proof rests on that closure"
            );
            assert_ne!(
                out, FRAME_SUBTREE_SEGMENT,
                "sanitize_segment({raw:?}) produced the RESERVED frame segment — a topic \
                 can now collide with the transform tree"
            );
        }
        // The other half: the reserved segment is outside the alphabet, so the
        // closure above is a proof and not a coincidence of this corpus.
        assert!(
            FRAME_SUBTREE_SEGMENT
                .chars()
                .any(|c| !(c.is_ascii_alphanumeric() || c == '_')),
            "FRAME_SUBTREE_SEGMENT must contain a character sanitize_segment cannot emit"
        );
        // …and the constants agree on where the frame tree is rooted.
        assert_eq!(FRAME_ROOT, format!("{WORLD_ROOT}/{FRAME_SUBTREE_SEGMENT}"));
        for entity in [ODOM_ENTITY, BASE_ENTITY, LIDAR_ENTITY, CAMERA_ENTITY] {
            assert!(
                entity.starts_with(&format!("{FRAME_ROOT}/")),
                "{entity} must live under the reserved FRAME_ROOT"
            );
        }
        // The UNKNOWN-frame fallback is under it too (it hangs off base_entity).
        assert!(entity_path_for_frame("velodyne_link")
            .path
            .starts_with(&format!("{FRAME_ROOT}/")));
    }

    /// `fnv1a64` pinned against the PUBLISHED FNV-1a 64 test vectors (hand
    /// literals — never recomputed).
    #[test]
    fn fnv1a64_matches_published_vectors() {
        assert_eq!(fnv1a64(b""), 0xcbf2_9ce4_8422_2325);
        assert_eq!(fnv1a64(b"a"), 0xaf63_dc4c_8601_ec8c);
        assert_eq!(fnv1a64(b"foobar"), 0x8594_4171_f739_67e8);
    }

    #[test]
    fn observe_child_is_bounded_and_stops_learning_at_the_cap() {
        let mut reg = FrameRegistry::new();
        // Steady state: a repeat is a lookup, and the set does not grow.
        reg.observe_child("velodyne_link");
        reg.observe_child("/velodyne_link");
        reg.observe_child(" velodyne_link ");
        assert_eq!(reg.observed_len(), 1, "normalized, deduped");
        // An empty id is never learned (it is "no frame stated", not a frame).
        reg.observe_child("   ");
        assert_eq!(reg.observed_len(), 1);

        // A per-object publisher cannot grow it without limit.
        for i in 0..MAX_OBSERVED_FRAMES + 500 {
            reg.observe_child(&format!("tag_36h11_{i}"));
        }
        assert_eq!(reg.observed_len(), MAX_OBSERVED_FRAMES);
        // At the cap it STOPS LEARNING rather than evicting: the first frame
        // observed still resolves (an eviction policy would silently un-pose a
        // live topic), and a new one is reported unresolvable.
        assert!(reg.has_observed("velodyne_link"));
        assert!(reg.resolve("velodyne_link").is_some());
        assert!(!reg.has_observed("a_brand_new_frame"));
        assert_eq!(reg.resolve("a_brand_new_frame"), None);
        // Known ALIASES are a static table — unaffected by saturation.
        assert!(reg.resolve("odom").is_some());
    }

    #[test]
    fn unknown_frame_log_warns_once_per_distinct_frame() {
        let mut log = UnknownFrameLog::new();
        assert!(log.observe("imu_link"), "first sighting warns");
        assert!(!log.observe("imu_link"), "repeat is silent");
        assert!(log.observe("gps_link"), "a different frame warns");
        assert!(!log.observe("gps_link"));
    }

    #[test]
    fn camera_intrinsics_constants() {
        assert_eq!((GO2_CAMERA_WIDTH, GO2_CAMERA_HEIGHT), (1280, 720));
        assert_eq!(GO2_CAMERA_FOCAL_PX, 650.0);
    }
}
