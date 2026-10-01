// SPDX-License-Identifier: AGPL-3.0-only
//! The URDF-derived stick-figure SKELETON archetype for the Go2 demo.
//!
//! Renders a MOVING stick figure of the robot in Rerun: the URDF kinematic
//! tree mirrored onto Rerun's entity hierarchy (which IS the forward-kinematics
//! chain — Rerun composes `parent ∘ child` transforms automatically), animated
//! by the robot's 12 real leg-joint angles from `unitree_go/LowState`. The
//! Skeleton is a COALESCING archetype (see [`crate::sink::coalesces`]):
//! the `/lowstate` firehose (~500 Hz) is drained accumulate-all each poll tick
//! and only the NEWEST frame of that tick's batch is rendered (the 12 joint
//! `Transform3D`s overwrite visual state under Rerun's latest-at semantics), so
//! the figure animates at the poll cadence — NOT once per wire frame.
//!
//! # Meshes
//!
//! Each URDF `<link>`'s first `<visual>` that carries a `<geometry><mesh>` is
//! logged as a STATIC [`rerun::Asset3D`] on a `<link entity>/mesh` child, so the
//! mesh rides its link's (moving) joint transform for free. Rerun reads glTF, not
//! the URDF's mesh source (commonly Collada `.dae`, sometimes `.stl`/`.obj`), so
//! the runtime NEVER converts: it swaps the resolved mesh path's extension to
//! `.glb` and logs the sibling if it EXISTS (converted once, offline). A missing
//! sibling degrades to the stick figure alone plus one loud once-per-load info
//! naming the conversion recipe (extension-agnostic — it derives from the ACTUAL
//! missing pair, and reports the resolved/missing COUNTS so one gap reads
//! differently from a systemic `package://` break) — meshes are ADDITIVE; the
//! articulating sticks are always the fallback truth.
//!
//! The `/mesh` child path segment is RESERVED by this renderer: a URDF link named
//! literally `mesh` that hangs under a mesh-bearing parent link would collide with
//! that parent's `{entity}/mesh` Asset3D child. This is un-guarded (a contrived
//! name for a real robot link) and documented here rather than defended in code.
//!
//! # Pipeline seam (how this plugs into the generic sink)
//!
//! The generic sink ([`crate::sink::dispatch_frame`]) walks each raw frame by
//! its `WireHeader.schema_hash`, then keys the archetype on the decoded schema
//! NAME. `unitree_go/LowState` used to key
//! [`crate::sink::ArchetypeKind::Skeleton`], whose dispatch arm calls
//! [`Skeleton::log_statics_once`] (the static tree, once per recording) +
//! [`Skeleton::log_joint_angles`] (the 12 revolute Transform3Ds, per frame).
//!
//! **That row has been REMOVED, so NO schema reaches this archetype today.** Since
//! the last `install_skeleton` caller was deleted the skeleton is inert on
//! every live run, and the mapping therefore DEMOTED the topic: its degraded
//! field dump was logged into an entity whose only view is 3D (invisible), where
//! an unmapped schema of the same shape rides the shape ladder and plots its
//! whole joint bank. Everything below is unchanged and reachable from
//! [`crate::sink::SinkState::install_skeleton`] + tests; wiring that installer
//! into `cerulion-vizd` means restoring the schema row alongside
//! it. The walker would then need
//! `unitree_go/LowState` + its nested `unitree_go/MotorState` — seeded from the
//! bridge config's `msg_dirs` store
//! (`crate::schema_registry::walker_from_bridge_config_env`), which the parallel
//! `ros2 attach` / harvest populates.
//!
//! # Entity tree
//!
//! Base link → [`ROBOT_ROOT`] (`world/tf-tree/robot`), identity (body pose in
//! world is not applied; the cloud shares this robot-relative frame so superposition
//! is correct regardless). Every other link hangs off its parent's entity, so
//! e.g. `FL_calf` lands at `world/tf-tree/robot/FL_hip/FL_thigh/FL_calf`. At each joint's
//! CHILD entity a `Transform3D` carries the joint origin (∘ the axis rotation for
//! a revolute joint). Because the geometry (markers/bones) at each link is drawn
//! in that link's OWN (rotating) frame, static geometry + a per-frame temporal
//! transform on the 12 revolute joints = an animated skeleton, with zero
//! per-frame geometry re-logging.
//!
//! # Cloud re-parenting (the extrinsic)
//!
//! The URDF's fixed `base → radar` chain (`radar` is the L1 lidar mount) IS the
//! lidar extrinsic. When the skeleton is active, [`Skeleton::reparent_cloud_route`]
//! moves the lidar cloud under the URDF `radar` link entity so the fixed chain
//! superposes cloud + skeleton automatically. When inert (env unset), the route
//! is returned UNCHANGED — every existing archetype behaves exactly as before.
//! (General per-topic frame attachment is not supported.)
//!
//! # Determinism / best-effort
//!
//! Nothing here reads wall-clock: the timeline is the frame's wire
//! `timestamp_ns` (via [`crate::archetype::set_robot_time`]), matching every
//! other archetype (Principle #7). All Rerun logging is best-effort — a log
//! error warns, never propagates.

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};

use cerulion_core::codegen::{FrameValue, FrameValueKind};
use rerun::RecordingStream;

use crate::archetype::set_robot_time;
use crate::pointcloud::{FieldsLogAction, FieldsWarnLatch};
use crate::sink::InputRoute;
// ONE entity-segment sanitizer for the crate (this module carried a
// private copy with a different fallback and NO aliasing suffix).
use crate::tf::sanitize_segment;

/// Env var naming the Go2 URDF file. Absent ⇒ the skeleton archetype is INERT.
///
/// Nothing breaks either way: today no schema classifies to the skeleton
/// archetype at all, so a joint-state frame rides the shape ladder and PLOTS
/// (the earlier sentence here promised a generic field dump, which was true
/// of the routing but was invisible in practice — see the module docs).
pub const GO2_URDF_PATH_ENV: &str = "GO2_URDF_PATH";

/// The URDF path found on the robot tonight — named in the absent-env warn so an
/// operator knows exactly what to set `GO2_URDF_PATH` to.
pub const GO2_URDF_DEFAULT_PATH: &str =
    "/home/unitree/unitree_ros/robots/go2_description/urdf/go2_description.urdf";

/// The skeleton root entity: the URDF base link maps HERE (the robot root).
/// Identity: the odom→base world pose is not inserted above it.
///
/// Lives under the RESERVED [`crate::tf::FRAME_ROOT`] (`world/tf-tree`) segment
/// like every other FRAME entity: a URDF link tree IS a transform tree, and the
/// reserved segment is what keeps a topic literally named `/robot` from landing
/// on the skeleton root. See [`crate::tf::FRAME_ROOT`] for the uniqueness proof.
pub const ROBOT_ROOT: &str = "world/tf-tree/robot";

/// **Go2 demo-lore seam.** Maps `LowState.motor_state[i]` → the URDF revolute
/// joint it drives. The Unitree Go2 SDK packs the motor array **FR leg first**
/// (`LegID` FR=0, FL=1, RR=2, RL=3; within a leg hip=0, thigh=1, calf=2, so
/// `motor_index = leg*3 + joint`) — this is the well-established SDK convention
/// (the `LowState.msg` carries no order comments, and the URDF *lists* joints
/// FL-first, so neither file states this; it is verified against the Unitree SDK
/// `LegID`/`JointIndex` enums). **The general Cerulion product does NOT hardcode
/// this** — it keys joints by `/joint_states` name. This table is the
/// ONE place the Go2-specific mapping lives; a wrong-legs demo means flipping
/// entries here.
pub const GO2_MOTOR_JOINTS: [&str; 12] = [
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
];

/// Number of leg motors we animate (the first 12 of `MotorState[20]`).
pub const LEG_MOTOR_COUNT: usize = 12;

/// Constant 3 (URDF / joint config): the URDF skeleton parameters.
/// `Default` reproduces the exact Go2 values ([`GO2_URDF_DEFAULT_PATH`],
/// [`ROBOT_ROOT`], [`GO2_MOTOR_JOINTS`]) so the demo stays byte-identical; the
/// `cerulion viz` verb (in later work) supplies a per-robot URDF path,
/// root entity, and motor→joint mapping.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UrdfConfig {
    /// The default URDF file path — named in the absent-env warn.
    pub default_path: String,
    /// The skeleton root entity the URDF base link maps to.
    pub robot_root: String,
    /// The `LowState.motor_state[i]` → URDF revolute-joint-name mapping (the SDK
    /// motor-array order). Length should match the robot's leg-motor count
    /// ([`LEG_MOTOR_COUNT`] for the Go2).
    pub motor_joints: Vec<String>,
}

impl Default for UrdfConfig {
    fn default() -> Self {
        Self {
            default_path: GO2_URDF_DEFAULT_PATH.to_string(),
            robot_root: ROBOT_ROOT.to_string(),
            motor_joints: GO2_MOTOR_JOINTS.iter().map(|s| s.to_string()).collect(),
        }
    }
}

/// Joint-marker point radius (m) — a small dot at each link origin.
const JOINT_MARKER_RADIUS: f32 = 0.012;
/// Bone line radius (m).
const BONE_RADIUS: f32 = 0.006;

// ===========================================================================
// Quaternion math (hand-rolled — glam is NOT in this isolated workspace's tree)
// ===========================================================================

/// A unit quaternion `(x, y, z, w)` for URDF joint forward kinematics. `f64`
/// internally (URDF is f64); narrowed to `f32` only at the Rerun boundary.
/// Hand-rolled so the demo pulls NO math dependency (glam is absent from the
/// `examples/go2` lockfile) and the FK is exactly oracle-testable.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Quat {
    x: f64,
    y: f64,
    z: f64,
    w: f64,
}

impl Quat {
    const IDENTITY: Quat = Quat {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };

    /// Rotation of `angle` radians about `axis`. A zero-length axis (a fixed
    /// joint's `<axis xyz="0 0 0"/>`) yields the identity — never a NaN.
    fn from_axis_angle(axis: [f64; 3], angle: f64) -> Quat {
        let n = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        if n < 1e-9 {
            return Quat::IDENTITY;
        }
        let (s, c) = (angle * 0.5).sin_cos();
        let k = s / n;
        Quat {
            x: axis[0] * k,
            y: axis[1] * k,
            z: axis[2] * k,
            w: c,
        }
    }

    /// A URDF `<origin rpy="r p y">` rotation. URDF convention is the fixed-axis
    /// composition `R = Rz(y) · Ry(p) · Rx(r)` (roll about X, then pitch about Y,
    /// then yaw about Z), i.e. quaternion `qz ∘ qy ∘ qx`.
    fn from_rpy(r: f64, p: f64, y: f64) -> Quat {
        Quat::from_axis_angle([0.0, 0.0, 1.0], y)
            .mul(Quat::from_axis_angle([0.0, 1.0, 0.0], p))
            .mul(Quat::from_axis_angle([1.0, 0.0, 0.0], r))
    }

    /// Hamilton product `self ∘ o` (apply `o` first, then `self`).
    fn mul(self, o: Quat) -> Quat {
        Quat {
            w: self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z,
            x: self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y,
            y: self.w * o.y - self.x * o.z + self.y * o.w + self.z * o.x,
            z: self.w * o.z + self.x * o.y - self.y * o.x + self.z * o.w,
        }
    }

    fn to_xyzw_f32(self) -> [f32; 4] {
        [self.x as f32, self.y as f32, self.z as f32, self.w as f32]
    }
}

/// The joint rotation at angle `q`: the origin orientation composed with a
/// rotation about the joint `axis` (which is expressed in the joint/origin
/// frame, so it POST-multiplies): `origin_rot ∘ Rot(axis, q)`. A fixed joint
/// (`q = 0`, zero axis) reduces to `origin_rot`.
fn joint_rotation(rpy: [f64; 3], axis: [f64; 3], q: f64) -> Quat {
    Quat::from_rpy(rpy[0], rpy[1], rpy[2]).mul(Quat::from_axis_angle(axis, q))
}

/// Build the Rerun `Transform3D` for a joint at angle `q`: translation =
/// the origin `xyz` (revolute joints do not translate), rotation =
/// [`joint_rotation`].
fn joint_transform3d(xyz: [f64; 3], rpy: [f64; 3], axis: [f64; 3], q: f64) -> rerun::Transform3D {
    let rot = joint_rotation(rpy, axis, q);
    rerun::Transform3D::from_translation_rotation(
        [xyz[0] as f32, xyz[1] as f32, xyz[2] as f32],
        rerun::Quaternion::from_xyzw(rot.to_xyzw_f32()),
    )
}

// ===========================================================================
// URDF model
// ===========================================================================

/// The joint classes we distinguish. Everything that is neither `revolute`/
/// `continuous` nor `fixed` (prismatic/floating/planar — the Go2 has none) is
/// `Other`, logged as a static origin like a fixed joint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum JointKind {
    Revolute,
    Fixed,
    Other,
}

/// One parsed URDF joint (the subset the skeleton needs — no limits/dynamics).
#[derive(Debug, Clone)]
struct UrdfJoint {
    name: String,
    kind: JointKind,
    parent: String,
    child: String,
    xyz: [f64; 3],
    rpy: [f64; 3],
    axis: [f64; 3],
}

/// One link's first `<visual>` mesh, as PARSED from the URDF but NOT yet
/// resolved to a filesystem path (resolution needs the URDF file's directory,
/// which only [`Skeleton::load`] has — [`Skeleton::from_urdf_str`] leaves meshes
/// unresolved). A link with no mesh visual carries no entry.
#[derive(Debug, Clone, PartialEq)]
struct RawVisual {
    /// The raw `<mesh filename="...">` string (`package://` / relative /
    /// absolute), resolved by [`resolve_mesh_path`].
    mesh_filename: String,
    /// The `<visual><origin xyz>` (default `[0,0,0]`).
    origin_xyz: [f64; 3],
    /// The `<visual><origin rpy>` (default `[0,0,0]`).
    origin_rpy: [f64; 3],
    /// The `<mesh scale>` (default `[1,1,1]`).
    scale: [f64; 3],
}

/// A resolved per-link mesh asset ready to log as a static [`rerun::Asset3D`]:
/// the `.glb` sibling that EXISTS (checked at resolve time — see
/// `UrdfModel::resolve_mesh_assets`) plus the visual origin + scale for its
/// static [`rerun::Transform3D`]. Public so tests can oracle the resolution.
#[derive(Debug, Clone, PartialEq)]
pub struct LinkMeshAsset {
    /// The Rerun entity the mesh logs to: `<link entity>/mesh` (a child of the
    /// link's own — moving — entity, so the mesh inherits the joint FK). The
    /// `/mesh` segment is RESERVED: a URDF link named literally `mesh` under a
    /// mesh-bearing parent would collide with this child (un-guarded, contrived).
    pub entity: String,
    /// The existing `.glb` sibling (the resolved mesh path with its extension
    /// swapped to `.glb`). The original mesh (commonly `.dae`, sometimes
    /// `.stl`/`.obj`) is never read by the runtime.
    pub glb_path: PathBuf,
    /// The visual `<origin xyz>` translation.
    pub origin_xyz: [f64; 3],
    /// The visual `<origin rpy>` rotation.
    pub origin_rpy: [f64; 3],
    /// The mesh `scale` (default `[1,1,1]`).
    pub scale: [f64; 3],
}

impl LinkMeshAsset {
    /// True when the visual origin + scale are the identity (no translation, no
    /// rotation, unit scale) — the mesh then inherits its link's transform
    /// verbatim and needs NO own `Transform3D` (see [`Self::origin_transform`]).
    fn is_identity(&self) -> bool {
        self.origin_xyz == [0.0; 3] && self.origin_rpy == [0.0; 3] && self.scale == [1.0; 3]
    }

    /// The mesh's static child transform COMPONENTS — `(translation, quaternion
    /// xyzw, scale)`, each narrowed to `f32` at the Rerun boundary — or `None`
    /// when identity+unit (skipped). Split out of [`Self::origin_transform`] so
    /// the exact produced values are hand-oracle-testable: a built
    /// `rerun::Transform3D` stores Arrow-serialized components that cannot be
    /// read back for assertion, but this tuple can.
    fn origin_transform_components(&self) -> Option<([f32; 3], [f32; 4], [f32; 3])> {
        if self.is_identity() {
            return None;
        }
        let rot = Quat::from_rpy(self.origin_rpy[0], self.origin_rpy[1], self.origin_rpy[2]);
        Some((
            [
                self.origin_xyz[0] as f32,
                self.origin_xyz[1] as f32,
                self.origin_xyz[2] as f32,
            ],
            rot.to_xyzw_f32(),
            [
                self.scale[0] as f32,
                self.scale[1] as f32,
                self.scale[2] as f32,
            ],
        ))
    }

    /// The mesh's static child `Transform3D` (visual origin ∘ scale), or `None`
    /// when identity+unit (skipped — the mesh rides its link's transform as-is).
    fn origin_transform(&self) -> Option<rerun::Transform3D> {
        let (translation, quaternion, scale) = self.origin_transform_components()?;
        Some(rerun::Transform3D::from_translation_rotation_scale(
            translation,
            rerun::Quaternion::from_xyzw(quaternion),
            scale,
        ))
    }
}

/// One link's absent `.glb` sibling, for the once-per-load recipe info:
/// the `.glb` the runtime looked for and the resolved ORIGINAL mesh path it
/// should sit next to. Extension-agnostic — the original is commonly `.dae` but
/// may be `.stl`/`.obj`, so the recipe never hardcodes an extension.
#[derive(Debug, Clone, PartialEq)]
struct MissingMesh {
    /// The absent `.glb` sibling (the resolved mesh path with its extension
    /// swapped to `.glb`).
    glb: PathBuf,
    /// The resolved original mesh path the `.glb` should sit beside.
    original: PathBuf,
}

/// The outcome of [`UrdfModel::resolve_mesh_assets`]: the assets whose `.glb`
/// sibling was found, plus a MISS summary — how many link meshes lacked a
/// sibling (`missing_count`) and the FIRST such miss in link-name order
/// (`first_missing`), which together let [`Skeleton::activate`] tell a single
/// conversion gap apart from a systemic `package://` break.
#[derive(Debug, Clone, PartialEq)]
struct MeshResolution {
    /// The links whose `.glb` sibling exists (ready to log).
    assets: Vec<LinkMeshAsset>,
    /// The first link mesh with no `.glb` sibling (`None` ⇒ every mesh resolved).
    first_missing: Option<MissingMesh>,
    /// Total count of link meshes with no `.glb` sibling.
    missing_count: usize,
}

/// A resolved binding from a `LowState` motor index to the revolute joint it
/// drives, carrying everything [`Skeleton::log_joint_angles`] needs without a
/// per-frame lookup.
#[derive(Debug, Clone)]
struct MotorBinding {
    joint_name: String,
    child_entity: String,
    xyz: [f64; 3],
    rpy: [f64; 3],
    axis: [f64; 3],
}

/// The parsed URDF kinematic model, plus the precomputed entity paths and
/// motor→joint bindings.
#[derive(Debug, Clone)]
struct UrdfModel {
    joints: Vec<UrdfJoint>,
    links: Vec<String>,
    root_link: String,
    /// Link name → Rerun entity path (`world/tf-tree/robot/...`).
    link_entity: BTreeMap<String, String>,
    /// `motor_bindings[i]` = the revolute joint `LowState.motor_state[i]` drives
    /// ([`GO2_MOTOR_JOINTS`]), or `None` if that joint name is absent from the
    /// URDF (it then simply never animates).
    motor_bindings: Vec<Option<MotorBinding>>,
    /// Link name → its first `<visual>` mesh (UNRESOLVED — see [`RawVisual`]).
    /// Links without a mesh visual carry no entry. Ordered (`BTreeMap`) so
    /// [`Self::resolve_mesh_assets`]'s first-missing pick is deterministic.
    link_visuals: BTreeMap<String, RawVisual>,
    /// The RESOLVED per-link `.glb` mesh assets whose sibling was found on disk
    /// (populated by [`Self::resolve_mesh_assets`] from [`Skeleton::load`]; empty
    /// for the dir-less [`Skeleton::from_urdf_str`] path). Logged by
    /// [`Self::log_statics`].
    mesh_assets: Vec<LinkMeshAsset>,
}

impl UrdfModel {
    /// The lidar mount link's entity path (the `radar` link on the Go2), for
    /// [`Skeleton::reparent_cloud_route`]. Tries the Go2 `radar` link first,
    /// then common lidar link aliases.
    fn lidar_link_entity(&self) -> Option<&str> {
        for name in ["radar", "lidar", "livox_frame", "utlidar_lidar", "laser"] {
            if let Some(e) = self.link_entity.get(name) {
                return Some(e.as_str());
            }
        }
        None
    }

    /// Log the STATIC tree once: every joint's origin (revolute joints at angle
    /// 0 — a rest pose that the per-frame temporal transforms then override),
    /// plus each link's marker point and the bones to its child joints.
    fn log_statics(&self, rec: &RecordingStream) {
        // (1) One static origin transform per joint, at the child entity.
        for j in &self.joints {
            let Some(entity) = self.link_entity.get(&j.child) else {
                continue;
            };
            let axis = if matches!(j.kind, JointKind::Revolute) {
                j.axis
            } else {
                [0.0; 3] // fixed/other joints have no motion axis
            };
            let tf = joint_transform3d(j.xyz, j.rpy, axis, 0.0);
            if let Err(e) = rec.log_static(entity.clone(), &tf) {
                tracing::warn!(error = %e, entity = %entity, "cerulion_viz skeleton: static joint transform log failed");
            }
        }

        // (2) Markers + bones. A link's bone set is a segment from its own
        // origin (`[0,0,0]` in its frame) to EACH child joint's origin
        // translation — a rigid, per-link-frame quantity, so it is static and
        // still animates (it is drawn in the link's rotating frame).
        let mut children_of: BTreeMap<&str, Vec<&UrdfJoint>> = BTreeMap::new();
        for j in &self.joints {
            children_of.entry(j.parent.as_str()).or_default().push(j);
        }
        for (link, entity) in &self.link_entity {
            let marker = rerun::Points3D::new([[0.0f32, 0.0, 0.0]])
                .with_radii([JOINT_MARKER_RADIUS])
                .with_colors([rerun::Color::from_rgb(90, 200, 255)]);
            if let Err(e) = rec.log_static(entity.clone(), &marker) {
                tracing::warn!(error = %e, entity = %entity, "cerulion_viz skeleton: static joint marker log failed");
            }
            if let Some(kids) = children_of.get(link.as_str()) {
                let strips: Vec<Vec<[f32; 3]>> = kids
                    .iter()
                    .map(|j| {
                        vec![
                            [0.0f32, 0.0, 0.0],
                            [j.xyz[0] as f32, j.xyz[1] as f32, j.xyz[2] as f32],
                        ]
                    })
                    .collect();
                let bones = rerun::LineStrips3D::new(strips)
                    .with_radii([BONE_RADIUS])
                    .with_colors([rerun::Color::from_rgb(235, 235, 235)]);
                if let Err(e) = rec.log_static(entity.clone(), &bones) {
                    tracing::warn!(error = %e, entity = %entity, "cerulion_viz skeleton: static bone log failed");
                }
            }
        }

        // (3) Per-link visual meshes: a static Asset3D on the
        // `<link>/mesh` child, plus the visual origin+scale as that child's
        // static Transform3D (skipped when identity+unit). Rerun reads the
        // `.glb` bytes at log time (`from_file_path` → an io error, not a
        // panic); every failure is a best-effort warn, never propagated.
        for asset in &self.mesh_assets {
            match rerun::Asset3D::from_file_path(&asset.glb_path) {
                Ok(mesh) => {
                    if let Err(e) = rec.log_static(asset.entity.clone(), &mesh) {
                        tracing::warn!(error = %e, entity = %asset.entity, "cerulion_viz skeleton: static mesh Asset3D log failed");
                    }
                }
                Err(e) => tracing::warn!(
                    error = %e,
                    entity = %asset.entity,
                    glb = %asset.glb_path.display(),
                    "cerulion_viz skeleton: could not read the .glb mesh — skipping it (stick figure unaffected)"
                ),
            }
            if let Some(tf) = asset.origin_transform() {
                if let Err(e) = rec.log_static(asset.entity.clone(), &tf) {
                    tracing::warn!(error = %e, entity = %asset.entity, "cerulion_viz skeleton: static mesh transform log failed");
                }
            }
        }
    }

    /// Resolve every parsed [`RawVisual`] to its `.glb` sibling relative to
    /// `urdf_dir` (the URDF file's directory), keeping only links whose sibling
    /// EXISTS. Returns a [`MeshResolution`]: the resolved assets, the COUNT of
    /// link meshes with no `.glb` sibling (`missing_count`), and the FIRST such
    /// miss (`first_missing`, in link-name / `BTreeMap` order) for the systemic-
    /// vs-single once-per-load recipe info. A link with no resolvable entity
    /// path is skipped (it never renders).
    fn resolve_mesh_assets(&self, urdf_dir: &Path) -> MeshResolution {
        let mut assets = Vec::new();
        let mut first_missing: Option<MissingMesh> = None;
        let mut missing_count = 0usize;
        for (link, vis) in &self.link_visuals {
            let Some(entity) = self.link_entity.get(link) else {
                continue;
            };
            let original = resolve_mesh_path(urdf_dir, &vis.mesh_filename);
            let glb = original.with_extension("glb");
            if glb.is_file() {
                assets.push(LinkMeshAsset {
                    entity: format!("{entity}/mesh"),
                    glb_path: glb,
                    origin_xyz: vis.origin_xyz,
                    origin_rpy: vis.origin_rpy,
                    scale: vis.scale,
                });
            } else {
                missing_count += 1;
                if first_missing.is_none() {
                    first_missing = Some(MissingMesh { glb, original });
                }
            }
        }
        MeshResolution {
            assets,
            first_missing,
            missing_count,
        }
    }
}

/// Resolve a URDF `<mesh filename>` to a filesystem path relative to the URDF
/// file's directory `urdf_dir`. THREE input forms, ONE rule + ONE fallback:
///
/// - `package://<pkg>/<rest>`: the FIRST ancestor of `urdf_dir` (inclusive) that
///   either IS named `<pkg>` or CONTAINS a child directory `<pkg>` roots `<rest>`
///   (covers the standard `<pkg>/urdf/robot.urdf` + `<pkg>/meshes/*` layout and a
///   sibling-package checkout). Fallback (no such ancestor): `urdf_dir/../<rest>`.
/// - an ABSOLUTE path: passed through unchanged.
/// - anything else (a RELATIVE path): joined onto `urdf_dir`.
///
/// The extension-to-`.glb` swap is applied by the caller
/// ([`UrdfModel::resolve_mesh_assets`]), not here.
fn resolve_mesh_path(urdf_dir: &Path, filename: &str) -> PathBuf {
    if let Some(rest) = filename.strip_prefix("package://") {
        let mut it = rest.splitn(2, '/');
        let pkg = it.next().unwrap_or("");
        let subpath = it.next().unwrap_or("");
        for anc in urdf_dir.ancestors() {
            if anc.file_name().is_some_and(|n| n == pkg) {
                return anc.join(subpath);
            }
            let sibling = anc.join(pkg);
            if sibling.is_dir() {
                return sibling.join(subpath);
            }
        }
        // Fallback: the common `<pkg>/urdf/` layout puts meshes one dir up.
        return urdf_dir.parent().unwrap_or(urdf_dir).join(subpath);
    }
    let p = Path::new(filename);
    if p.is_absolute() {
        return p.to_path_buf();
    }
    urdf_dir.join(p)
}

/// A structural URDF parse failure. The skeleton degrades to INERT on any of
/// these (a loud warn from [`Skeleton::load`]); it never panics the sink.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum UrdfError {
    /// The document is not well-formed XML.
    #[error("URDF XML parse failed: {0}")]
    Xml(String),
    /// The document root is not `<robot>`.
    #[error("URDF root element is not <robot> (found <{0}>)")]
    NotRobot(String),
    /// The document declared no `<link>`s.
    #[error("URDF has no <link> elements")]
    NoLinks,
    /// An explicitly supplied geometry vector is malformed or cannot reach the renderer.
    #[error("URDF <{element}> {attribute} at line {line}: expected three finite numbers representable as f32, got {value:?}")]
    InvalidVector {
        /// XML element carrying the attribute.
        element: String,
        /// Attribute name within that element.
        attribute: String,
        /// One-based XML source line.
        line: u32,
        /// The rejected attribute value.
        value: String,
    },
}

/// Read an optional vector attribute. Defaults apply only when the attribute is
/// absent; malformed values never become plausible geometry at the origin.
fn vector_attribute(
    node: Option<roxmltree::Node<'_, '_>>,
    attribute: &str,
    default: [f64; 3],
) -> Result<[f64; 3], UrdfError> {
    let Some((node, value)) = node.and_then(|n| n.attribute(attribute).map(|v| (n, v))) else {
        return Ok(default);
    };
    let invalid = || UrdfError::InvalidVector {
        element: node.tag_name().name().to_string(),
        attribute: attribute.to_string(),
        line: node.document().text_pos_at(node.range().start).row,
        value: value.to_string(),
    };
    let mut components = value.split_whitespace();
    let mut vector = [0.0; 3];
    for component in &mut vector {
        *component = components
            .next()
            .and_then(|v| v.parse::<f64>().ok())
            .filter(|v| (*v as f32).is_finite())
            .ok_or_else(invalid)?;
    }
    if components.next().is_some() {
        return Err(invalid());
    }
    Ok(vector)
}

/// The `link` attribute of a joint's `<parent>` / `<child>` child element.
fn joint_link_ref(joint: roxmltree::Node<'_, '_>, tag: &str) -> String {
    joint
        .children()
        .find(|n| n.is_element() && n.tag_name().name() == tag)
        .and_then(|n| n.attribute("link"))
        .unwrap_or("")
        .to_string()
}

/// Read a joint or visual's origin, with identity defaults for absent attributes.
fn parse_origin(element: roxmltree::Node<'_, '_>) -> Result<([f64; 3], [f64; 3]), UrdfError> {
    let origin = element
        .children()
        .find(|n| n.is_element() && n.tag_name().name() == "origin");
    Ok((
        vector_attribute(origin, "xyz", [0.0; 3])?,
        vector_attribute(origin, "rpy", [0.0; 3])?,
    ))
}

/// Parse a `<link>`'s first MESH `<visual>` into a [`RawVisual`], or `None` when
/// the link has no mesh visual. Scans ALL `<visual>` elements and takes the FIRST
/// that carries a `<geometry><mesh filename="...">`: a real URDF often lists
/// a primitive visual — a `<box>`/`<cylinder>` collision-style shape — BEFORE the
/// mesh visual, and taking the first `<visual>` unconditionally would silently
/// drop the mesh. Tolerant: a link with zero mesh visuals (no `<visual>`, only
/// non-`<mesh>` geometries, or only empty `filename`s) degrades to `None` — it
/// missing mesh does not fail the URDF load. The `<origin>` (default identity)
/// and mesh `scale` (default `[1,1,1]`) are optional; malformed supplied vectors
/// return an error.
fn parse_link_visual(link: roxmltree::Node<'_, '_>) -> Result<Option<RawVisual>, UrdfError> {
    for visual in link.children().filter(|n| n.has_tag_name("visual")) {
        if let Some(mesh) = parse_mesh_visual(visual)? {
            return Ok(Some(mesh));
        }
    }
    Ok(None)
}

/// Read the first mesh in one visual. A missing mesh remains a supported
/// stick-figure fallback; malformed transforms on a mesh are an import error.
fn parse_mesh_visual(visual: roxmltree::Node<'_, '_>) -> Result<Option<RawVisual>, UrdfError> {
    let mesh = visual
        .children()
        .find(|n| n.has_tag_name("geometry"))
        .and_then(|n| n.children().find(|n| n.has_tag_name("mesh")));
    let Some(mesh) = mesh else { return Ok(None) };
    let Some(filename) = mesh.attribute("filename").filter(|f| !f.is_empty()) else {
        return Ok(None);
    };
    let (origin_xyz, origin_rpy) = parse_origin(visual)?;
    let scale = vector_attribute(Some(mesh), "scale", [1.0; 3])?;
    Ok(Some(RawVisual {
        mesh_filename: filename.to_string(),
        origin_xyz,
        origin_rpy,
        scale,
    }))
}

/// URDF defaults an omitted motion axis to X; fixed joints ignore their axis.
fn joint_axis(joint: roxmltree::Node<'_, '_>) -> Result<[f64; 3], UrdfError> {
    let axis = joint.children().find(|n| n.has_tag_name("axis"));
    vector_attribute(axis, "xyz", [1.0, 0.0, 0.0])
}

/// Parse a URDF XML string into a resolved [`UrdfModel`] using the default
/// (Go2) [`UrdfConfig`]. Invalid numeric geometry returns an explicit error.
fn parse_urdf(xml: &str) -> Result<UrdfModel, UrdfError> {
    parse_urdf_with_config(xml, &UrdfConfig::default())
}

/// Parse a URDF XML string into a resolved [`UrdfModel`], rooting link entities
/// at `cfg.robot_root` and binding `LowState` motors by `cfg.motor_joints`
/// (the per-robot config). [`parse_urdf`] is the default-config entry.
fn parse_urdf_with_config(xml: &str, cfg: &UrdfConfig) -> Result<UrdfModel, UrdfError> {
    let doc = roxmltree::Document::parse(xml).map_err(|e| UrdfError::Xml(e.to_string()))?;
    let robot = doc.root_element();
    if robot.tag_name().name() != "robot" {
        return Err(UrdfError::NotRobot(robot.tag_name().name().to_string()));
    }

    let mut links: Vec<String> = Vec::new();
    let mut joints: Vec<UrdfJoint> = Vec::new();
    let mut link_visuals: BTreeMap<String, RawVisual> = BTreeMap::new();
    for node in robot.children().filter(|n| n.is_element()) {
        match node.tag_name().name() {
            "link" => {
                if let Some(name) = node.attribute("name") {
                    links.push(name.to_string());
                    if let Some(vis) = parse_link_visual(node)? {
                        link_visuals.insert(name.to_string(), vis);
                    }
                }
            }
            "joint" => {
                let name = node.attribute("name").unwrap_or("").to_string();
                let kind = match node.attribute("type") {
                    Some("revolute") | Some("continuous") => JointKind::Revolute,
                    Some("fixed") => JointKind::Fixed,
                    _ => JointKind::Other,
                };
                let parent = joint_link_ref(node, "parent");
                let child = joint_link_ref(node, "child");
                let (xyz, rpy) = parse_origin(node)?;
                let axis = joint_axis(node)?;
                if !name.is_empty() && !parent.is_empty() && !child.is_empty() {
                    joints.push(UrdfJoint {
                        name,
                        kind,
                        parent,
                        child,
                        xyz,
                        rpy,
                        axis,
                    });
                }
            }
            _ => {}
        }
    }
    if links.is_empty() {
        return Err(UrdfError::NoLinks);
    }

    // Root = the link that is never a joint's child. Prefer base/base_link; fall
    // back to the first such link, else the first link (a malformed non-tree
    // still renders something rather than failing the demo).
    let child_links: BTreeSet<&str> = joints.iter().map(|j| j.child.as_str()).collect();
    let roots: Vec<String> = links
        .iter()
        .filter(|l| !child_links.contains(l.as_str()))
        .cloned()
        .collect();
    let root_link = roots
        .iter()
        .find(|l| l.as_str() == "base" || l.as_str() == "base_link")
        .or_else(|| roots.first())
        .or_else(|| links.first())
        .cloned()
        .ok_or(UrdfError::NoLinks)?;

    // Entity paths: root → ROBOT_ROOT, then repeated relaxation passes assign a
    // child once its parent has a path (tree depth iterations; cycle-safe — a
    // cyclic link never resolves and falls to the orphan default below).
    let mut link_entity: BTreeMap<String, String> = BTreeMap::new();
    link_entity.insert(root_link.clone(), cfg.robot_root.clone());
    loop {
        let mut changed = false;
        for j in &joints {
            if !link_entity.contains_key(&j.child) {
                // Clone the parent path so no immutable borrow of `link_entity`
                // survives into the mutable `insert` below.
                if let Some(parent_entity) = link_entity.get(&j.parent).cloned() {
                    let entity = format!("{parent_entity}/{}", sanitize_segment(&j.child));
                    link_entity.insert(j.child.clone(), entity);
                    changed = true;
                }
            }
        }
        if !changed {
            break;
        }
    }
    // Any link not reachable from the root (orphan or cycle) hangs off the root
    // so it is at least visible.
    for l in &links {
        link_entity
            .entry(l.clone())
            .or_insert_with(|| format!("{}/{}", cfg.robot_root, sanitize_segment(l)));
    }

    // Bind each LowState motor index to its revolute joint (by name).
    let motor_bindings: Vec<Option<MotorBinding>> = cfg
        .motor_joints
        .iter()
        .map(|jname| {
            joints
                .iter()
                .find(|j| j.name == *jname && matches!(j.kind, JointKind::Revolute))
                .map(|j| MotorBinding {
                    joint_name: j.name.clone(),
                    child_entity: link_entity.get(&j.child).cloned().unwrap_or_else(|| {
                        format!("{}/{}", cfg.robot_root, sanitize_segment(&j.child))
                    }),
                    xyz: j.xyz,
                    rpy: j.rpy,
                    axis: j.axis,
                })
        })
        .collect();

    Ok(UrdfModel {
        joints,
        links,
        root_link,
        link_entity,
        motor_bindings,
        link_visuals,
        // Resolved lazily by `resolve_mesh_assets` in `Skeleton::load` (needs the
        // URDF's directory); the dir-less `from_urdf_str` leaves this empty.
        mesh_assets: Vec::new(),
    })
}

// ===========================================================================
// Skeleton (the public archetype)
// ===========================================================================

/// The URDF skeleton archetype state. Owned by [`crate::sink::SinkState`],
/// which the render loop holds across polls. `Default` = INERT (no URDF) so
/// every existing test / code path that constructs a `SinkState` is unaffected.
///
/// # NO PRODUCTION INSTALLER
///
/// Nothing in production calls
/// [`Skeleton::load`] / [`crate::sink::SinkState::install_skeleton`]:
/// `cerulion-vizd` does not install one. Every caller is a
/// test, so on a live run this state is INERT and the stick figure does not
/// render. Do not read
/// the docs below as describing a live path: they describe what an installed
/// skeleton does.
#[derive(Debug, Default)]
pub struct Skeleton {
    /// `None` = inert (env unset / URDF unreadable / unparseable). `Some` =
    /// active, rendering the stick figure.
    model: Option<UrdfModel>,
    /// Once-per-regime latch for the frozen-skeleton
    /// warning. An ACTIVE skeleton whose `LowState` frames resolve ZERO joint
    /// angles (wrong `motor_state` field name, empty motor array, or no bound
    /// joint) renders a FROZEN stick figure with no signal — the
    /// [`FieldsWarnLatch`] house pattern surfaces it loudly once, then debug
    /// with a running count, healing with one recovery `info!` when a frame
    /// resolves joints again. `Default` = armed (fresh).
    zero_joint_latch: FieldsWarnLatch,
}

impl Skeleton {
    /// An explicitly inert skeleton (renders nothing; `reparent_cloud_route` is
    /// the identity).
    pub fn inert() -> Self {
        Self::default()
    }

    /// `true` once a URDF is loaded (the demo is active). Also the discriminator
    /// the cloud re-parent gates on.
    pub fn is_active(&self) -> bool {
        self.model.is_some()
    }

    /// Load from an explicit path option. `None` (env unset) ⇒ INERT + ONE loud
    /// warn naming [`GO2_URDF_PATH_ENV`] and the on-robot default path. A read /
    /// parse failure ⇒ INERT + a loud warn. NEVER panics.
    ///
    /// Takes the path as an ARGUMENT rather than reading the env itself so the
    /// caller can hand it a FROZEN env snapshot — the choice then stays
    /// deterministic and replay-safe (Principle #7). The `rerun_sink` node used
    /// to do exactly that at `init()`; it was deleted and nothing has
    /// replaced it, so today only tests call this (see the type-level note).
    pub fn load(path: Option<&str>) -> Self {
        let Some(path) = path.filter(|p| !p.is_empty()) else {
            tracing::warn!(
                env = GO2_URDF_PATH_ENV,
                default_path = GO2_URDF_DEFAULT_PATH,
                "cerulion_viz skeleton: {GO2_URDF_PATH_ENV} unset — URDF stick-figure archetype is \
                 INERT (joint-state topics still render, as plots, via the shape ladder). Set \
                 {GO2_URDF_PATH_ENV} to the Go2 URDF (on the robot: {GO2_URDF_DEFAULT_PATH})"
            );
            return Self::inert();
        };
        match std::fs::read_to_string(path) {
            // The URDF's directory roots mesh resolution (relative + `package://`
            // paths). A bare filename's `Path::parent()` is the EMPTY path `""`
            // (NOT `None`, and NOT `"."`); joining a relative mesh path onto `""`
            // resolves against the current working directory exactly as `"."`
            // would, so lookups still work. The `unwrap_or` guards only the true
            // `None` case — the filesystem root `/`, whose parent is `None`.
            Ok(xml) => match parse_urdf(&xml) {
                Ok(model) => {
                    let urdf_dir = Path::new(path).parent().unwrap_or_else(|| Path::new("."));
                    Self::activate(model, Some(urdf_dir))
                }
                Err(e) => {
                    tracing::warn!(
                        path,
                        error = %e,
                        "cerulion_viz skeleton: URDF parse failed — stick-figure archetype INERT"
                    );
                    Self::inert()
                }
            },
            Err(e) => {
                tracing::warn!(
                    path,
                    error = %e,
                    "cerulion_viz skeleton: could not read the URDF — stick-figure archetype INERT"
                );
                Self::inert()
            }
        }
    }

    /// Load from the LIVE process env — the convenience path for tests and for
    /// an embedding host that has no frozen snapshot to offer. A caller that
    /// does have one must use [`Skeleton::load`] instead, or the loaded model
    /// depends on process env at run time and replay stops matching live
    /// (Principle #7). Currently uncalled: see the type-level note.
    pub fn load_from_env() -> Self {
        Self::load(std::env::var(GO2_URDF_PATH_ENV).ok().as_deref())
    }

    /// Parse a URDF XML string into an ACTIVE skeleton. Used by hermetic tests
    /// (no file / no robot needed). Meshes are NOT resolved on this path — that
    /// needs the URDF file's directory, which only [`Skeleton::load`] has — so a
    /// skeleton built here renders the stick figure but no `Asset3D` meshes.
    pub fn from_urdf_str(xml: &str) -> Result<Self, UrdfError> {
        Ok(Self::activate(parse_urdf(xml)?, None))
    }

    /// [`Skeleton::from_urdf_str`] parametrized by a [`UrdfConfig`] —
    /// roots link entities at `cfg.robot_root` and binds motors by
    /// `cfg.motor_joints`. The default-config path is [`Skeleton::from_urdf_str`]
    /// (byte-identical to the pre-config parse); this is the entry the
    /// `cerulion viz` verb / an embedding host uses for a non-Go2 robot.
    pub fn from_urdf_str_with_config(xml: &str, cfg: &UrdfConfig) -> Result<Self, UrdfError> {
        Ok(Self::activate(parse_urdf_with_config(xml, cfg)?, None))
    }

    /// Finish an ACTIVE skeleton from a parsed model: emit the load-info /
    /// diagnostics, and — when `mesh_dir` is `Some` (the [`Skeleton::load`] file
    /// path) — resolve each link's `.glb` mesh sibling relative to it. `None`
    /// (the `from_urdf_str` path) skips mesh resolution (no directory to root
    /// relative / `package://` paths against).
    fn activate(mut model: UrdfModel, mesh_dir: Option<&Path>) -> Self {
        let resolved = model.motor_bindings.iter().filter(|b| b.is_some()).count();
        tracing::info!(
            joints = model.joints.len(),
            links = model.links.len(),
            motors_resolved = resolved,
            root = %model.root_link,
            "cerulion_viz skeleton: URDF loaded — stick-figure archetype ACTIVE"
        );
        // The expected count is THIS config's motor set (`motor_bindings` is
        // built 1:1 from `UrdfConfig::motor_joints`), NOT the Go2's
        // LEG_MOTOR_COUNT — a non-Go2 robot with fewer motors must not warn
        // when fully resolved.
        let expected = model.motor_bindings.len();
        if resolved < expected {
            tracing::warn!(
                motors_resolved = resolved,
                expected,
                "cerulion_viz skeleton: only {resolved}/{expected} motor joints resolved \
                 against the URDF — unresolved joints will not animate (check \
                 UrdfConfig::motor_joints vs the URDF joint names)"
            );
        }
        // If the URDF carries no lidar-mount link, the
        // cloud CANNOT be re-parented onto the skeleton (see
        // [`Skeleton::reparent_cloud_route`]) — warn once at load so the
        // superposition-missing case is not silent.
        if model.lidar_link_entity().is_none() {
            tracing::warn!(
                aliases = "radar/lidar/livox_frame/utlidar_lidar/laser",
                "cerulion_viz skeleton: no lidar-mount link found in the URDF (tried \
                 radar/lidar/livox_frame/utlidar_lidar/laser) — the lidar cloud will NOT be \
                 posed in the mount's frame; it renders wherever its own message frame_id \
                 places it (or at the world origin) instead of superposed on the stick \
                 figure (add a `radar` mount link to the URDF to fix the extrinsic)"
            );
        }
        // Resolve the per-link `.glb` mesh siblings (the automagic). A
        // present sibling → a static Asset3D rides the link's transform; a
        // missing one → the stick figure alone plus ONE once-per-load recipe
        // info naming the resolved/missing COUNTS + the first missing pair (load
        // runs once, so a bare `tracing::info!` here IS once-per-load — no latch
        // needed). The recipe is extension-agnostic and, when NO mesh has a
        // sibling, reads as the SYSTEMIC case, not a single file.
        if let Some(dir) = mesh_dir {
            let resolution = model.resolve_mesh_assets(dir);
            let resolved = resolution.assets.len();
            let missing_count = resolution.missing_count;
            if resolved > 0 {
                tracing::info!(
                    resolved,
                    missing = missing_count,
                    "cerulion_viz skeleton: resolved {resolved} link mesh(es) (.glb siblings) — \
                     rendering Asset3D meshes on the stick figure"
                );
            }
            if let Some(first_missing) = &resolution.first_missing {
                let glb = first_missing.glb.display().to_string();
                let original = first_missing.original.display().to_string();
                let recipe = format!(
                    "no {glb} next to {original} — convert it once (e.g. Blender / assimp / \
                     trimesh) and it renders automatically"
                );
                if resolved == 0 {
                    tracing::info!(
                        resolved,
                        missing = missing_count,
                        first_missing_glb = %glb,
                        first_missing_original = %original,
                        "cerulion_viz skeleton: NONE of the {missing_count} URDF link mesh(es) have a \
                         .glb sibling — rendering the stick figure alone (this reads as a systemic \
                         package:// / conversion gap, not one file). {recipe}"
                    );
                } else {
                    tracing::info!(
                        resolved,
                        missing = missing_count,
                        first_missing_glb = %glb,
                        first_missing_original = %original,
                        "cerulion_viz skeleton: {missing_count} of {total} URDF link mesh(es) have no \
                         .glb sibling — rendering the stick figure alone for them. {recipe}",
                        total = resolved + missing_count
                    );
                }
            }
            model.mesh_assets = resolution.assets;
        }
        Self {
            model: Some(model),
            ..Default::default()
        }
    }

    /// The resolved per-link `.glb` mesh assets — empty when inert or
    /// when built via the dir-less [`Skeleton::from_urdf_str`]. A pure accessor
    /// for tests + operator introspection (Principle #3).
    pub fn link_mesh_assets(&self) -> &[LinkMeshAsset] {
        self.model.as_ref().map_or(&[], |m| &m.mesh_assets)
    }

    /// Log the STATIC skeleton tree exactly ONCE per process (per
    /// [`rearm_skeleton_statics`] epoch): the joint origins + markers +
    /// bones. Inert ⇒ no-op. Mirrors [`crate::tf::log_viz_statics_once`]'s
    /// resettable `AtomicBool` guard (a `Once` could not be re-armed for the
    /// same-process memory-sink tests).
    pub fn log_statics_once(&self, rec: &RecordingStream) {
        let Some(model) = &self.model else {
            return;
        };
        if SKELETON_STATICS_LOGGED.swap(true, Ordering::SeqCst) {
            return;
        }
        model.log_statics(rec);
    }

    /// Log the 12 revolute joint transforms from one `LowState` frame on the
    /// `robot_time` timeline. Reads `motor_state[0..12].q` (radians) via the
    /// generic frame walker (an `Array` of nested-fixed `MotorState` values —
    /// see `read_leg_motor_qs`). Inert ⇒ no-op.
    pub fn log_joint_angles(&mut self, rec: &RecordingStream, timestamp_ns: u64, fv: &FrameValue) {
        let Some(model) = &self.model else {
            return;
        };
        let qs = read_leg_motor_qs(fv);
        set_robot_time(rec, timestamp_ns);
        // Count the joint transforms this frame actually applied (a bound joint
        // whose motor delivered a `q`). Zero applied ⇒ a FROZEN stick figure.
        let mut applied = 0usize;
        for (i, binding) in model.motor_bindings.iter().enumerate() {
            let Some(b) = binding else { continue };
            let Some(q) = qs[i] else { continue };
            applied += 1;
            let tf = joint_transform3d(b.xyz, b.rpy, b.axis, q);
            if let Err(e) = rec.log(b.child_entity.clone(), &tf) {
                tracing::warn!(
                    error = %e,
                    entity = %b.child_entity,
                    joint = %b.joint_name,
                    "cerulion_viz skeleton: joint transform log failed"
                );
            }
        }
        // A frame that resolves ZERO joint angles renders
        // a frozen figure with no signal (wrong `motor_state` field name, empty
        // motor array, or no bound joint). Surface it via the once-per-regime
        // house latch — loud first, debug-with-count sustained, one recovery
        // `info!` when joints resolve again.
        if applied == 0 {
            match self.zero_joint_latch.on_inferred() {
                FieldsLogAction::WarnFirst => tracing::warn!(
                    expected_field = "motor_state[0..12].q",
                    "cerulion_viz skeleton: LowState frame resolved ZERO joint angles — the stick \
                     figure is FROZEN (no `motor_state[i].q` reads; check the LowState field \
                     name / motor array vs GO2_MOTOR_JOINTS). Repeats log at debug until a \
                     frame resolves joints"
                ),
                FieldsLogAction::DebugSuppressed { suppressed } => tracing::debug!(
                    suppressed,
                    "cerulion_viz skeleton: LowState still resolving zero joint angles (warn suppressed)"
                ),
            }
        } else if let Some(suppressed) = self.zero_joint_latch.on_decoded() {
            tracing::info!(
                suppressed_count = suppressed,
                applied,
                "cerulion_viz skeleton: LowState joint angles resolving again — skeleton animating \
                 (frozen-figure regime healed)"
            );
        }
    }

    /// When active, pose the LIDAR cloud in the URDF `radar` link's coordinate
    /// FRAME so the fixed `base → radar` extrinsic superposes cloud + skeleton.
    /// Only lidar/cloud inputs are re-pointed; every other input (and the inert
    /// case) is returned UNCHANGED — the identity that keeps existing behavior
    /// intact.
    ///
    /// **The coordinate-frame change altered the mechanism, not the effect.** This used to REWRITE
    /// `route.entity` to the radar link's own entity path, which re-created the
    /// very collision that change fixes: with a URDF loaded, EVERY cloud topic landed
    /// on that one path, so two lidar topics overwrote each other again. It now
    /// sets [`InputRoute::frame`] and leaves the entity alone, so each cloud keeps
    /// its own unique `world/<topic>` path while its geometry is posed in the
    /// radar frame — the same `CoordinateFrame` mechanism the data-derived path
    /// uses. Composition is unchanged: the radar link entity carries the URDF's
    /// real `Transform3D` chain up to `tf#/world`, so its implicit frame is
    /// genuinely placed.
    ///
    /// This special-cases the demo's radar-framed cloud. The general
    /// per-topic frame-attach story (a `frame:` on any input) is a separate
    /// product feature.
    pub fn reparent_cloud_route(&self, input_name: &str, route: InputRoute) -> InputRoute {
        let Some(model) = &self.model else {
            return route;
        };
        // Match the cloud-name set on the LAST path segment, case-insensitively,
        // in lockstep with `crate::sink::route_for_input`'s knob matching.
        // The daemon's route key is now the WHOLE topic, so a bare
        // `input_name` comparison would silently stop recognising
        // `/utlidar/cloud` and the demo's cloud would leave the skeleton.
        // The knob reads the TOPIC half of the key only (an `entity` override
        // names an entity and nothing else — `crate::sink::route_key_for_topic`).
        let trimmed = crate::sink::route_key_topic(input_name).trim_matches('/');
        let leaf = trimmed
            .rsplit('/')
            .next()
            .unwrap_or("")
            .to_ascii_lowercase();
        if !matches!(leaf.as_str(), "cloud" | "lidar" | "points" | "pointcloud") {
            return route;
        }
        match model.lidar_link_entity() {
            Some(link_entity) => InputRoute {
                // The topic KEEPS its own entity path.
                entity: route.entity,
                is_static: false,
                // A cloud input never poses the robot root — only odom-named
                // inputs do (see `crate::sink::route_for_input`).
                drives_robot_root: false,
                frame: Some(format!("tf#/{}", link_entity.trim_matches('/'))),
            },
            None => route,
        }
    }
}

/// Read `LowState.motor_state[0..12].q` (radians) from a decoded frame value.
///
/// The walker surfaces the fixed `MotorState[20]` array of an all-primitive
/// nested message as [`FrameValueKind::Array`] of [`FrameValueKind::Nested`]
/// elements (the fixed-array-of-nested-fixed decode path — see
/// `cerulion_core::codegen::frame_walker`). Each element's `q` field is an
/// `F32`. A missing / opaque / short array degrades to all-`None` (the skeleton
/// then holds its rest pose) — never a panic.
fn read_leg_motor_qs(fv: &FrameValue) -> [Option<f64>; LEG_MOTOR_COUNT] {
    let mut out = [None; LEG_MOTOR_COUNT];
    let Some(FrameValueKind::Array(elems)) = fv.field("motor_state") else {
        return out;
    };
    for (i, slot) in out.iter_mut().enumerate() {
        if i >= elems.len() {
            break;
        }
        if let FrameValueKind::Nested(inner) = &elems[i] {
            *slot = scalar_f64(inner.field("q"));
        }
    }
    out
}

/// Read a numeric field as `f64` (the joint angle `q` is `float32` ⇒ `F32`;
/// `F64` tolerated defensively).
fn scalar_f64(v: Option<&FrameValueKind>) -> Option<f64> {
    match v {
        Some(FrameValueKind::F32(x)) => Some(*x as f64),
        Some(FrameValueKind::F64(x)) => Some(*x),
        _ => None,
    }
}

/// Structurally-once guard for the static-tree send (an `AtomicBool`, not a
/// `Once`, so [`rearm_skeleton_statics`] can re-arm it — the same
/// reasoning as [`crate::tf::log_viz_statics_once`]).
static SKELETON_STATICS_LOGGED: AtomicBool = AtomicBool::new(false);

/// Re-arm the static-tree guard so the skeleton tree re-logs on the next
/// `LowState` frame. Callers: [`crate::stream::rearm_after_reconnect`] (where
/// a bounced server re-receives the URDF tree) and
/// [`crate::stream::reset_for_test`] (test hygiene — one reset restores the full
/// clean slate).
///
/// The guard is PROCESS-global: tests in the SAME binary that reset or trip it
/// must serialize against each other (the integration binary uses a file-local
/// mutex; in this lib's unit tests exactly ONE test touches it today — a second
/// one must add the same mutex discipline). Separate test binaries are separate
/// processes and cannot race.
pub fn rearm_skeleton_statics() {
    SKELETON_STATICS_LOGGED.store(false, Ordering::SeqCst);
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::codegen::NamedValue;
    use std::f64::consts::{FRAC_PI_2, PI};

    /// A compact hermetic URDF fixture: base → {FR_hip → FR_thigh, radar}. Two
    /// revolute leg joints (distinct axes) + one fixed radar mount (the real Go2
    /// radar rpy). Enough for parse + entity-path + motor-mapping + reparent
    /// oracles without touching the robot.
    const FIXTURE_URDF: &str = r#"<?xml version="1.0"?>
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

    /// A non-default [`UrdfConfig`] flows through `parse_urdf` into
    /// the resolved model — link entities root at `cfg.robot_root` and motor
    /// bindings follow `cfg.motor_joints` (order + membership). The default path
    /// is byte-identical (pinned by the sibling `motor_bindings_*` tests, which
    /// still assert the `world/tf-tree/robot` Go2 root). This is the URDF half of the
    /// config test suite (the model is private, so it lives in-module).
    #[test]
    fn urdf_config_flows_through_to_entity_paths_and_bindings() {
        // Custom root + a REORDERED SUBSET of the motor→joint mapping.
        let cfg = UrdfConfig {
            default_path: "/custom/robot.urdf".to_string(),
            robot_root: "sim/quad".to_string(),
            motor_joints: vec!["FR_thigh_joint".to_string(), "FR_hip_joint".to_string()],
        };
        let model = parse_urdf_with_config(FIXTURE_URDF, &cfg).expect("fixture parses");

        // Link entities are rooted at the CUSTOM root (not `world/tf-tree/robot`).
        assert_eq!(
            model.link_entity.get("base").map(String::as_str),
            Some("sim/quad")
        );
        assert_eq!(
            model.link_entity.get("FR_hip").map(String::as_str),
            Some("sim/quad/FR_hip")
        );
        assert_eq!(
            model.link_entity.get("FR_thigh").map(String::as_str),
            Some("sim/quad/FR_hip/FR_thigh")
        );

        // Motor bindings follow cfg.motor_joints (its length, order, membership).
        assert_eq!(model.motor_bindings.len(), 2);
        let b0 = model.motor_bindings[0]
            .as_ref()
            .expect("index 0 → FR_thigh_joint");
        assert_eq!(b0.joint_name, "FR_thigh_joint");
        assert_eq!(b0.child_entity, "sim/quad/FR_hip/FR_thigh");
        let b1 = model.motor_bindings[1]
            .as_ref()
            .expect("index 1 → FR_hip_joint");
        assert_eq!(b1.joint_name, "FR_hip_joint");
        assert_eq!(b1.child_entity, "sim/quad/FR_hip");

        // The DEFAULT config keeps the Go2 root (anti-tautology: the override,
        // not a hardcode, produced the custom paths above).
        let default_model =
            parse_urdf_with_config(FIXTURE_URDF, &UrdfConfig::default()).expect("parses");
        assert_eq!(
            default_model.link_entity.get("FR_hip").map(String::as_str),
            Some("world/tf-tree/robot/FR_hip")
        );
        assert_eq!(default_model.motor_bindings.len(), LEG_MOTOR_COUNT);
    }

    /// `activate`'s under-resolved
    /// warn measures against THIS config's motor set (`motor_bindings.len()`),
    /// never the Go2 `LEG_MOTOR_COUNT`. A fully-resolved non-Go2 robot (2/2
    /// configured motors) must NOT warn — a Go2-fixed count always warns "2/12"; an
    /// under-resolved one warns with the CONFIG's expected count (1/2, not /12).
    #[tracing_test::traced_test]
    #[test]
    fn activate_warn_expected_is_config_motor_count_not_go2() {
        let full = UrdfConfig {
            default_path: "/custom/robot.urdf".to_string(),
            robot_root: "sim/quad".to_string(),
            motor_joints: vec!["FR_thigh_joint".to_string(), "FR_hip_joint".to_string()],
        };
        let _sk = Skeleton::from_urdf_str_with_config(FIXTURE_URDF, &full).expect("parses");
        assert!(
            !logs_contain("motor joints resolved"),
            "a fully-resolved non-Go2 config (2/2) must not fire the under-resolved warn"
        );

        // Under-resolved control: one configured joint absent from the URDF →
        // the warn fires with the CONFIG's expected (1/2), never the Go2 12.
        let under = UrdfConfig {
            motor_joints: vec!["FR_hip_joint".to_string(), "absent_joint".to_string()],
            ..full
        };
        let _sk2 = Skeleton::from_urdf_str_with_config(FIXTURE_URDF, &under).expect("parses");
        assert!(
            logs_contain("only 1/2 motor joints resolved"),
            "the under-resolved warn must name the config's expected count (1/2)"
        );
        assert!(
            !logs_contain("/12 "),
            "the warn must never reference the Go2 LEG_MOTOR_COUNT for a custom config"
        );
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    fn quat_close(q: Quat, x: f64, y: f64, z: f64, w: f64) -> bool {
        close(q.x, x) && close(q.y, y) && close(q.z, z) && close(q.w, w)
    }

    // ---- quaternion / FK math oracles ------------------------------------

    #[test]
    fn axis_angle_oracle() {
        let s = FRAC_PI_2 / 2.0; // = π/4
        let (sin, cos) = s.sin_cos();
        // Rot(Y, π/2) = (0, sin(π/4), 0, cos(π/4)).
        assert!(quat_close(
            Quat::from_axis_angle([0.0, 1.0, 0.0], FRAC_PI_2),
            0.0,
            sin,
            0.0,
            cos
        ));
        // Rot(X, π) = (1, 0, 0, ~0).
        let qx = Quat::from_axis_angle([1.0, 0.0, 0.0], PI);
        assert!(close(qx.x, 1.0) && close(qx.y, 0.0) && close(qx.z, 0.0) && qx.w.abs() < 1e-9);
        // A zero-length axis (fixed joint) is the identity, never a NaN.
        assert_eq!(
            Quat::from_axis_angle([0.0, 0.0, 0.0], 1.234),
            Quat::IDENTITY
        );
        // A non-unit axis is normalized: Rot([0,2,0], π/2) == Rot([0,1,0], π/2).
        assert!(quat_close(
            Quat::from_axis_angle([0.0, 2.0, 0.0], FRAC_PI_2),
            0.0,
            sin,
            0.0,
            cos
        ));
    }

    #[test]
    fn rpy_matches_axis_composition() {
        // A pure pitch rpy == Rot(Y, p).
        assert!(quat_close(
            Quat::from_rpy(0.0, FRAC_PI_2, 0.0),
            0.0,
            (FRAC_PI_2 / 2.0).sin(),
            0.0,
            (FRAC_PI_2 / 2.0).cos()
        ));
        // A pure roll rpy == Rot(X, r).
        assert!(quat_close(
            Quat::from_rpy(FRAC_PI_2, 0.0, 0.0),
            (FRAC_PI_2 / 2.0).sin(),
            0.0,
            0.0,
            (FRAC_PI_2 / 2.0).cos()
        ));
    }

    #[test]
    fn from_rpy_fixed_axis_composition_hand_oracle() {
        // Pin the URDF rpy convention as the FIXED-axis
        // composition R = Rz(yaw)·Ry(pitch)·Rx(roll) (quaternion Rz ∘ Ry ∘ Rx)
        // with a NON-degenerate triple roll=π/2, pitch=0, yaw=π/2.
        //   s = c = √2/2. Rx(π/2)=(s,0,0,c), Ry(0)=(0,0,0,1), Rz(π/2)=(0,0,s,c).
        //   Rz ∘ Rx  (Hamilton, self=Rz o=Rx):
        //     w = c·c − 0 − 0 − s·0 = c² = 0.5
        //     x = c·s + 0 + 0 − s·0 = c·s = 0.5
        //     y = 0 − 0 + 0 + s·s   = s² = 0.5
        //     z = 0 + 0 − 0 + s·c   = s·c = 0.5
        //   ⇒ (0.5, 0.5, 0.5, 0.5). The REVERSED order Rx ∘ Ry ∘ Rz yields
        //   (0.5, −0.5, 0.5, 0.5) — a DIFFERENT quaternion, so this pins the
        //   compose ORDER, not merely the component magnitudes.
        let q = Quat::from_rpy(FRAC_PI_2, 0.0, FRAC_PI_2);
        assert!(
            quat_close(q, 0.5, 0.5, 0.5, 0.5),
            "got ({}, {}, {}, {})",
            q.x,
            q.y,
            q.z,
            q.w
        );
    }

    #[test]
    fn joint_rotation_compose_order_hand_oracle() {
        // origin rpy = (π/2, 0, 0) ⇒ Rx(π/2) = (0.7071.., 0, 0, 0.7071..).
        // axis = Z, q = π/2 ⇒ Rz(π/2) = (0, 0, 0.7071.., 0.7071..).
        // origin_rot ∘ axis_rot (Rx then Rz) = (0.5, -0.5, 0.5, 0.5) — a clean
        // hand-computed Hamilton product that pins BOTH the compose order and
        // the product formula (swapping either breaks it).
        let q = joint_rotation([FRAC_PI_2, 0.0, 0.0], [0.0, 0.0, 1.0], FRAC_PI_2);
        assert!(
            quat_close(q, 0.5, -0.5, 0.5, 0.5),
            "got ({}, {}, {}, {})",
            q.x,
            q.y,
            q.z,
            q.w
        );
    }

    #[test]
    fn revolute_joint_rotation_is_origin_then_axis() {
        // Zero origin rpy ⇒ the joint rotation is just Rot(axis, q).
        let q = joint_rotation([0.0, 0.0, 0.0], [0.0, 1.0, 0.0], FRAC_PI_2);
        assert!(quat_close(
            q,
            0.0,
            (FRAC_PI_2 / 2.0).sin(),
            0.0,
            (FRAC_PI_2 / 2.0).cos()
        ));
        // A fixed joint (q = 0, zero axis) reduces to the origin rotation.
        let fixed = joint_rotation([0.0, 2.8782, 0.0], [0.0, 0.0, 0.0], 0.0);
        let origin = Quat::from_rpy(0.0, 2.8782, 0.0);
        assert!(quat_close(fixed, origin.x, origin.y, origin.z, origin.w));
    }

    // ---- URDF parse + entity tree + motor mapping ------------------------

    #[test]
    fn parse_builds_tree_and_entity_paths() {
        let model = parse_urdf(FIXTURE_URDF).expect("fixture parses");
        assert_eq!(model.root_link, "base");
        assert_eq!(model.joints.len(), 3);
        // Base is the robot root; children hang off it; chained children nest.
        assert_eq!(model.link_entity["base"], "world/tf-tree/robot");
        assert_eq!(model.link_entity["FR_hip"], "world/tf-tree/robot/FR_hip");
        assert_eq!(
            model.link_entity["FR_thigh"],
            "world/tf-tree/robot/FR_hip/FR_thigh"
        );
        assert_eq!(model.link_entity["radar"], "world/tf-tree/robot/radar");
    }

    /// A valid URDF with legs but NO lidar-alias mount link (radar / lidar /
    /// livox_frame / utlidar_lidar / laser): loads ACTIVE (the skeleton still
    /// renders), but `lidar_link_entity()` is `None`, so the cloud is NOT
    /// re-parented — the observable consequence of the F-C0 load-time warn
    /// (this workspace has no tracing-test tooling to capture the warn itself,
    /// so the triggering condition + its consequence are pinned instead).
    #[test]
    fn urdf_without_lidar_link_loads_active_with_no_cloud_reparent() {
        const NO_LIDAR_URDF: &str = r#"<?xml version="1.0"?>
<robot name="no_lidar">
  <link name="base"/>
  <link name="FR_hip"/>
  <joint name="FR_hip_joint" type="revolute">
    <origin xyz="0.1934 -0.0465 0" rpy="0 0 0"/>
    <parent link="base"/>
    <child link="FR_hip"/>
    <axis xyz="1 0 0"/>
  </joint>
</robot>"#;
        let model = parse_urdf(NO_LIDAR_URDF).expect("legs-only URDF parses");
        assert!(
            model.lidar_link_entity().is_none(),
            "no lidar-alias link present ⇒ the F-C0 warn condition is met"
        );
        let sk = Skeleton::from_urdf_str(NO_LIDAR_URDF).expect("still active");
        assert!(
            sk.is_active(),
            "a lidar-less URDF is still a valid skeleton"
        );
        // Consequence: a cloud input is returned UNCHANGED (no radar link to
        // move it onto) — the cloud renders un-reparented, not superposed.
        let route = InputRoute {
            entity: "world/utlidar/cloud".to_string(),
            is_static: false,
            drives_robot_root: false,
            frame: None,
        };
        assert_eq!(
            sk.reparent_cloud_route("cloud", route.clone()),
            route,
            "no lidar link ⇒ the cloud route is the identity (the F-C0 consequence)"
        );
    }

    /// Each structural URDF parse-failure branch maps to
    /// its own [`UrdfError`] variant (hand oracles, never a self-compare).
    #[test]
    fn urdf_parse_errors_are_classified_per_variant() {
        // (1) Malformed XML (unclosed root) → Xml.
        assert!(
            matches!(parse_urdf("<robot>").unwrap_err(), UrdfError::Xml(_)),
            "unclosed XML is a Xml error"
        );
        // (2) Well-formed XML whose root is not <robot> → NotRobot(found).
        assert_eq!(
            parse_urdf(r#"<?xml version="1.0"?><scene><link name="a"/></scene>"#).unwrap_err(),
            UrdfError::NotRobot("scene".to_string())
        );
        // (3) A <robot> with zero <link>s → NoLinks.
        assert_eq!(
            parse_urdf(r#"<?xml version="1.0"?><robot name="empty"></robot>"#).unwrap_err(),
            UrdfError::NoLinks
        );
    }

    #[test]
    fn motor_table_is_fr_first_and_binds_by_name() {
        // The demo-lore mapping table, pinned exactly (FR, FL, RR, RL × hip,
        // thigh, calf).
        assert_eq!(
            GO2_MOTOR_JOINTS,
            [
                "FR_hip_joint",
                "FR_thigh_joint",
                "FR_calf_joint",
                "FL_hip_joint",
                "FL_thigh_joint",
                "FL_calf_joint",
                "RR_hip_joint",
                "RR_thigh_joint",
                "RR_calf_joint",
                "RL_hip_joint",
                "RL_thigh_joint",
                "RL_calf_joint",
            ]
        );
        let model = parse_urdf(FIXTURE_URDF).expect("fixture parses");
        // Index 0 = FR_hip_joint → the FR_hip child entity, axis X.
        let b0 = model.motor_bindings[0].as_ref().expect("FR_hip bound");
        assert_eq!(b0.joint_name, "FR_hip_joint");
        assert_eq!(b0.child_entity, "world/tf-tree/robot/FR_hip");
        assert_eq!(b0.axis, [1.0, 0.0, 0.0]);
        // Index 1 = FR_thigh_joint → the nested FR_thigh entity, axis Y.
        let b1 = model.motor_bindings[1].as_ref().expect("FR_thigh bound");
        assert_eq!(b1.child_entity, "world/tf-tree/robot/FR_hip/FR_thigh");
        assert_eq!(b1.axis, [0.0, 1.0, 0.0]);
        // Joints absent from the fixture are unbound (they simply never animate).
        assert!(model.motor_bindings[2].is_none()); // FR_calf_joint
        assert!(model.motor_bindings[11].is_none()); // RL_calf_joint
        assert_eq!(model.motor_bindings.len(), LEG_MOTOR_COUNT);
    }

    // ---- LowState motor read (the array-of-nested walker path) ------------

    fn motor_state_value(q: f32) -> FrameValueKind<'static> {
        FrameValueKind::Nested(Box::new(FrameValue {
            schema_name: "unitree_go/MotorState".to_string(),
            fields: vec![NamedValue {
                name: "q".to_string(),
                value: FrameValueKind::F32(q),
            }],
        }))
    }

    #[test]
    fn motor_angles_only_animate_revolute_and_continuous_joints() {
        for kind in [
            "fixed",
            "prismatic",
            "floating",
            "planar",
            "revolute",
            "continuous",
        ] {
            for axis in ["", r#"<axis xyz="1 0 0"/>"#] {
                let xml = format!(
                    r#"<robot name="test"><link name="base"/><link name="tip"/>
                    <joint name="joint" type="{kind}"><parent link="base"/><child link="tip"/>{axis}</joint></robot>"#
                );
                let config = UrdfConfig {
                    motor_joints: vec!["joint".into()],
                    ..UrdfConfig::default()
                };
                let mut skeleton = Skeleton::from_urdf_str_with_config(&xml, &config).unwrap();
                let (rec, storage) = rerun::RecordingStreamBuilder::new("motion_kinds")
                    .memory()
                    .unwrap();
                rec.flush_blocking().unwrap();
                let before = storage.num_msgs();
                let frame = FrameValue {
                    schema_name: "unitree_go/LowState".into(),
                    fields: vec![NamedValue {
                        name: "motor_state".into(),
                        value: FrameValueKind::Array(vec![motor_state_value(1.0)]),
                    }],
                };
                skeleton.log_joint_angles(&rec, 1_000, &frame);
                rec.flush_blocking().unwrap();
                assert_eq!(
                    storage.num_msgs() > before,
                    matches!(kind, "revolute" | "continuous"),
                    "unexpected angular transform for {kind} with axis {axis:?}"
                );
            }
        }
    }

    #[test]
    fn reads_leg_motor_qs_from_array_of_nested() {
        // Build a LowState-shaped value: motor_state = Array of 20 MotorState
        // nested values, q = index as radians (a distinct oracle per motor).
        let elems: Vec<FrameValueKind<'static>> =
            (0..20).map(|i| motor_state_value(i as f32 * 0.1)).collect();
        let fv = FrameValue {
            schema_name: "unitree_go/LowState".to_string(),
            fields: vec![NamedValue {
                name: "motor_state".to_string(),
                value: FrameValueKind::Array(elems),
            }],
        };
        let qs = read_leg_motor_qs(&fv);
        // Exactly the first 12 are read, each == its index * 0.1. The stored q
        // is an `f32` (`i as f32 * 0.1`) read back through `scalar_f64` as `f64`,
        // so the oracle is computed at the SAME f32 precision — 0.1 is not exact
        // in f32 (a ~1.5e-9 gap vs the f64 literal), which is faithful readback,
        // not a bug.
        for (i, q) in qs.into_iter().enumerate() {
            assert!(
                close(q.expect("motor read"), (i as f32 * 0.1) as f64),
                "motor {i} q mismatch"
            );
        }
        // A frame with no motor_state field degrades to all-None (no panic).
        let empty = FrameValue {
            schema_name: "unitree_go/LowState".to_string(),
            fields: vec![],
        };
        assert!(read_leg_motor_qs(&empty).iter().all(Option::is_none));
        // A short array reads what is there, None for the rest.
        let short = FrameValue {
            schema_name: "unitree_go/LowState".to_string(),
            fields: vec![NamedValue {
                name: "motor_state".to_string(),
                value: FrameValueKind::Array(vec![motor_state_value(1.5)]),
            }],
        };
        let qs = read_leg_motor_qs(&short);
        assert!(close(qs[0].unwrap(), 1.5));
        assert!(qs[1..].iter().all(Option::is_none));
    }

    // ---- inert-without-env + reparent identity ---------------------------

    #[test]
    fn inert_without_env_is_safe_noop() {
        // load(None) is inert (env absent path — emits one warn, asserted here
        // only for its behavioral effects: no panic, not active).
        let sk = Skeleton::load(None);
        assert!(!sk.is_active());
        // An empty path string is treated as absent, too.
        assert!(!Skeleton::load(Some("")).is_active());
        // A missing file degrades to inert (never a panic).
        assert!(!Skeleton::load(Some("/no/such/go2.urdf")).is_active());
        // Inert reparent is the identity — existing cloud routing is untouched.
        let route = InputRoute {
            entity: "world/utlidar/cloud".to_string(),
            is_static: false,
            drives_robot_root: false,
            frame: None,
        };
        assert_eq!(sk.reparent_cloud_route("cloud", route.clone()), route);
        // Inert dynamic/static logging is a no-op even with no recording stream
        // context (the None guard returns before any Rerun call).
        let dummy = FrameValue {
            schema_name: "unitree_go/LowState".to_string(),
            fields: vec![],
        };
        let _ = read_leg_motor_qs(&dummy); // exercises the read path
    }

    #[test]
    fn active_reparents_cloud_to_radar_link() {
        let sk = Skeleton::from_urdf_str(FIXTURE_URDF).expect("fixture active");
        assert!(sk.is_active());
        let original = InputRoute {
            entity: "world/utlidar/cloud".to_string(),
            is_static: false,
            drives_robot_root: false,
            frame: None,
        };
        // A cloud/lidar input is posed IN the URDF radar link's frame and
        // KEEPS its own entity path (previously the entity was rewritten to the radar
        // link, so with a URDF loaded every cloud topic collapsed onto one path).
        let moved = sk.reparent_cloud_route("cloud", original.clone());
        assert_eq!(
            moved.entity, "world/utlidar/cloud",
            "the topic keeps its own unique entity"
        );
        assert_eq!(
            moved.frame.as_deref(),
            Some("tf#/world/tf-tree/robot/radar")
        );
        assert!(!moved.is_static);
        // The name match is on the LAST segment, so an absolute topic key works.
        assert_eq!(
            sk.reparent_cloud_route("/lidar", original.clone())
                .frame
                .as_deref(),
            Some("tf#/world/tf-tree/robot/radar")
        );
        assert_eq!(
            sk.reparent_cloud_route("utlidar/cloud", original.clone())
                .frame
                .as_deref(),
            Some("tf#/world/tf-tree/robot/radar"),
            "a full-topic route key still matches on its last segment"
        );
        // TWO topics that BOTH match the cloud-name set keep DISTINCT entities
        // while sharing the radar frame; the collision the old entity
        // rewrite re-created (with a URDF loaded, both landed on the radar path).
        let other = InputRoute {
            entity: "world/velodyne/points".to_string(),
            ..original.clone()
        };
        let a = sk.reparent_cloud_route("utlidar/cloud", original.clone());
        let b = sk.reparent_cloud_route("velodyne/points", other);
        assert_ne!(a.entity, b.entity, "two cloud topics must not collapse");
        assert_eq!(a.frame, b.frame, "…while sharing the radar frame");
        assert!(a.frame.is_some(), "both really were re-pointed");
        // A NON-cloud input is untouched (skeleton only moves the lidar cloud).
        assert_eq!(sk.reparent_cloud_route("image", original.clone()), original);
        assert_eq!(sk.reparent_cloud_route("tf", original.clone()), original);
    }

    // ---- hermetic end-to-end log path (memory sink, no viewer) ------------

    #[test]
    fn active_skeleton_logs_statics_and_joint_angles_to_memory() {
        rearm_skeleton_statics();
        let (rec, storage) = rerun::RecordingStreamBuilder::new("go2_skeleton_test")
            .memory()
            .expect("memory recording");
        let mut sk = Skeleton::from_urdf_str(FIXTURE_URDF).expect("fixture active");

        // Static tree logs once and grows the store.
        let before = storage.num_msgs();
        sk.log_statics_once(&rec);
        rec.flush_blocking().expect("flush");
        let after_statics = storage.num_msgs();
        assert!(
            after_statics > before,
            "static skeleton tree logged (store grew from {before} to {after_statics})"
        );

        // A LowState frame's joint angles log per-joint transforms.
        let elems: Vec<FrameValueKind<'static>> = (0..20)
            .map(|i| motor_state_value(i as f32 * 0.05))
            .collect();
        let fv = FrameValue {
            schema_name: "unitree_go/LowState".to_string(),
            fields: vec![NamedValue {
                name: "motor_state".to_string(),
                value: FrameValueKind::Array(elems),
            }],
        };
        sk.log_joint_angles(&rec, 1_000, &fv);
        rec.flush_blocking().expect("flush");
        assert!(
            storage.num_msgs() > after_statics,
            "joint-angle transforms logged (store grew past {after_statics})"
        );
        rearm_skeleton_statics();
    }

    /// The frozen-skeleton latch WIRING — a `LowState`
    /// frame that resolves ZERO joint angles routes to the latch's `on_inferred`
    /// (loud-once-then-debug), and a frame that resolves joints routes to
    /// `on_decoded` (heal). Observed through the pub, unconditional
    /// `inferred_total` counter (the level→log mapping itself is the oracle-
    /// pinned `FieldsWarnLatch` contract in `pointcloud.rs`).
    #[test]
    fn frozen_skeleton_zero_joint_reads_drive_the_warn_latch() {
        let (rec, _storage) = rerun::RecordingStreamBuilder::new("go2_skeleton_frozen_test")
            .memory()
            .expect("memory recording");
        let mut sk = Skeleton::from_urdf_str(FIXTURE_URDF).expect("fixture active");

        // A frame with NO `motor_state` field resolves zero joints → each
        // frozen frame routes to on_inferred, bumping the counter (1, 2, 3).
        let frozen = FrameValue {
            schema_name: "unitree_go/LowState".to_string(),
            fields: vec![],
        };
        for expected in 1..=3u64 {
            sk.log_joint_angles(&rec, expected * 1_000, &frozen);
            assert_eq!(
                sk.zero_joint_latch.inferred_total, expected,
                "each zero-joint frame routes to on_inferred (frame {expected})"
            );
        }

        // A GOOD frame (motor_state present; FIXTURE_URDF binds FR_hip/FR_thigh)
        // resolves joints → routes to on_decoded, so the inference counter is
        // UNTOUCHED (the heal path, not an inference).
        let elems: Vec<FrameValueKind<'static>> =
            (0..20).map(|i| motor_state_value(i as f32 * 0.1)).collect();
        let good = FrameValue {
            schema_name: "unitree_go/LowState".to_string(),
            fields: vec![NamedValue {
                name: "motor_state".to_string(),
                value: FrameValueKind::Array(elems),
            }],
        };
        sk.log_joint_angles(&rec, 4_000, &good);
        assert_eq!(
            sk.zero_joint_latch.inferred_total, 3,
            "a resolving frame heals via on_decoded — never bumps the inference counter"
        );

        // The next frozen frame re-enters the zero-read path (counter → 4),
        // proving the wiring routes correctly across a heal.
        sk.log_joint_angles(&rec, 5_000, &frozen);
        assert_eq!(
            sk.zero_joint_latch.inferred_total, 4,
            "after healing, a zero-joint frame routes to on_inferred again"
        );
    }

    // ---- URDF visual meshes ----------------------------------------------

    /// Write a minimal valid GLB (12-byte header + one JSON chunk) at `path` —
    /// enough for the `.is_file()` sibling check AND for `Asset3D::from_file_path`
    /// (which only reads bytes + guesses the media type at log time; GLB
    /// structure is validated by the viewer at render, never here). Binary
    /// fixtures are NEVER committed — built in a tempdir at test time.
    fn write_min_glb(path: &Path) {
        let mut json = br#"{"asset":{"version":"2.0"}}"#.to_vec();
        while !json.len().is_multiple_of(4) {
            json.push(b' '); // pad the JSON chunk to a 4-byte boundary
        }
        let total = 12 + 8 + json.len();
        let mut glb = Vec::with_capacity(total);
        glb.extend_from_slice(b"glTF"); // magic
        glb.extend_from_slice(&2u32.to_le_bytes()); // version
        glb.extend_from_slice(&(total as u32).to_le_bytes()); // total length
        glb.extend_from_slice(&(json.len() as u32).to_le_bytes()); // chunk length
        glb.extend_from_slice(&0x4E4F_534Au32.to_le_bytes()); // "JSON"
        glb.extend_from_slice(&json);
        std::fs::write(path, glb).expect("write glb");
    }

    /// A URDF whose `base` link carries a mesh `<visual>` (`filename` +
    /// `origin` + `scale` injected), plus the two revolute leg joints so it also
    /// animates. Parameterized so tests vary the filename form.
    fn mesh_urdf(filename: &str, origin: &str, scale: &str) -> String {
        format!(
            r#"<?xml version="1.0"?>
<robot name="mesh_fixture">
  <link name="base">
    <visual>
      <origin xyz="{origin}"/>
      <geometry>
        <mesh filename="{filename}" scale="{scale}"/>
      </geometry>
    </visual>
  </link>
  <link name="FR_hip"/>
  <link name="FR_thigh"/>
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
</robot>"#
        )
    }

    #[test]
    fn malformed_geometry_vectors_are_rejected_instead_of_replaced_with_zero() {
        for vector in [
            "",
            "1 2",
            "1 2 3 4",
            "1 nope 3",
            "NaN 0 0",
            "inf 0 0",
            "1e100 0 0",
        ] {
            for attribute in [
                format!("<origin xyz=\"{vector}\"/>"),
                format!("<origin rpy=\"{vector}\"/>"),
                format!("<axis xyz=\"{vector}\"/>"),
            ] {
                let xml = format!(
                    r#"<robot name="test"><link name="a"/><link name="b"/>
                    <joint name="joint" type="revolute"><parent link="a"/><child link="b"/>{attribute}</joint></robot>"#
                );
                assert!(
                    parse_urdf(&xml).is_err(),
                    "accepted joint vector: {attribute}"
                );
            }
            let visual_rpy = mesh_urdf("body.glb", "0 0 0", "1 1 1").replace(
                "<origin xyz=\"0 0 0\"/>",
                &format!("<origin rpy=\"{vector}\"/>"),
            );
            assert!(
                parse_urdf(&visual_rpy).is_err(),
                "accepted visual rpy: {vector:?}"
            );
            for (origin, scale) in [(vector, "1 1 1"), ("0 0 0", vector)] {
                let xml = mesh_urdf("body.glb", origin, scale);
                assert!(
                    parse_urdf(&xml).is_err(),
                    "accepted visual: origin={origin:?}, scale={scale:?}"
                );
            }
        }
    }

    #[test]
    fn geometry_errors_locate_the_attribute_and_valid_numbers_keep_their_values() {
        let xml = r#"<robot name="test"><link name="base"><visual>
<origin xyz="1 wrong 3"/><geometry><mesh filename="body.glb"/></geometry>
</visual></link></robot>"#;
        assert_eq!(
            parse_urdf(xml).unwrap_err(),
            UrdfError::InvalidVector {
                element: "origin".into(),
                attribute: "xyz".into(),
                line: 2,
                value: "1 wrong 3".into(),
            }
        );
        let xml = mesh_urdf("body.glb", " +1e-1  -2.5  3 ", "1 2 0.5");
        let model = parse_urdf(&xml).unwrap();
        assert_eq!(model.link_visuals["base"].origin_xyz, [0.1, -2.5, 3.0]);
        assert_eq!(model.link_visuals["base"].scale, [1.0, 2.0, 0.5]);
    }

    #[test]
    fn omitted_revolute_axis_uses_urdf_x_axis_default() {
        let xml = r#"<robot name="test"><link name="a"/><link name="b"/>
            <joint name="joint" type="revolute"><parent link="a"/><child link="b"/></joint></robot>"#;
        let model = parse_urdf(xml).unwrap();
        assert_eq!(model.joints[0].axis, [1.0, 0.0, 0.0]);
        assert_eq!(model.joints[0].xyz, [0.0; 3]);
        assert_eq!(model.joints[0].rpy, [0.0; 3]);
    }

    #[test]
    fn parse_link_visual_extracts_mesh_origin_and_scale() {
        // A full mesh visual → the exact hand-parsed values.
        let urdf = mesh_urdf(
            "package://go2_description/meshes/base.dae",
            "0.1 0.2 0.3",
            "2 2 2",
        );
        let model = parse_urdf(&urdf).expect("mesh URDF parses");
        let vis = model.link_visuals.get("base").expect("base has a visual");
        assert_eq!(
            *vis,
            RawVisual {
                mesh_filename: "package://go2_description/meshes/base.dae".to_string(),
                origin_xyz: [0.1, 0.2, 0.3],
                origin_rpy: [0.0, 0.0, 0.0],
                scale: [2.0, 2.0, 2.0],
            }
        );
        // The non-visual links carry no entry.
        assert!(!model.link_visuals.contains_key("FR_hip"));

        // An absent scale defaults to unit, independently of origin defaults.
        let no_scale = r#"<?xml version="1.0"?>
<robot name="ns">
  <link name="base">
    <visual>
      <geometry><mesh filename="a.dae"/></geometry>
    </visual>
  </link>
</robot>"#;
        let m = parse_urdf(no_scale).expect("parses");
        assert_eq!(
            m.link_visuals["base"].scale,
            [1.0, 1.0, 1.0],
            "an absent scale is unit, never zero"
        );
        assert_eq!(m.link_visuals["base"].origin_xyz, [0.0; 3]);
    }

    #[test]
    fn malformed_or_absent_visual_degrades_to_none_and_load_still_succeeds() {
        // A link with a <visual> but a non-<mesh> geometry (a box) → no raw
        // visual, and the URDF still parses fully (tolerant per-link degrade).
        let box_urdf = r#"<?xml version="1.0"?>
<robot name="ns">
  <link name="base">
    <visual><geometry><box size="1 1 1"/></geometry></visual>
  </link>
  <link name="child"/>
  <joint name="j" type="fixed"><parent link="base"/><child link="child"/></joint>
</robot>"#;
        let m = parse_urdf(box_urdf).expect("box-geometry URDF still parses");
        assert!(
            m.link_visuals.is_empty(),
            "a non-mesh geometry yields no mesh visual"
        );
        assert_eq!(m.links.len(), 2, "the whole URDF still loaded");

        // A <mesh> with an EMPTY filename → None for that link (never a load
        // failure).
        let empty_fn = r#"<?xml version="1.0"?>
<robot name="ns">
  <link name="base">
    <visual><geometry><mesh filename=""/></geometry></visual>
  </link>
</robot>"#;
        assert!(parse_urdf(empty_fn)
            .expect("parses")
            .link_visuals
            .is_empty());
    }

    /// A link whose FIRST `<visual>` is a primitive (box)
    /// and whose SECOND `<visual>` carries the mesh — the mesh must still parse
    /// (taking the first `<visual>` unconditionally silently
    /// drops the mesh for this common URDF shape).
    #[test]
    fn first_mesh_visual_wins_past_a_leading_primitive_visual() {
        let primitive_then_mesh = r#"<?xml version="1.0"?>
<robot name="ns">
  <link name="base">
    <visual><geometry><box size="1 1 1"/></geometry></visual>
    <visual>
      <origin xyz="0.4 0 0"/>
      <geometry><mesh filename="package://pkg/meshes/base.dae" scale="3 3 3"/></geometry>
    </visual>
  </link>
</robot>"#;
        let m = parse_urdf(primitive_then_mesh).expect("parses");
        assert_eq!(
            m.link_visuals.get("base"),
            Some(&RawVisual {
                mesh_filename: "package://pkg/meshes/base.dae".to_string(),
                origin_xyz: [0.4, 0.0, 0.0],
                origin_rpy: [0.0, 0.0, 0.0],
                scale: [3.0, 3.0, 3.0],
            }),
            "the mesh visual is taken even though a box visual precedes it"
        );
    }

    #[test]
    fn resolve_mesh_path_package_relative_absolute_oracles() {
        let root = tempfile::tempdir().expect("tempdir");
        let rootp = root.path();
        let urdf_dir = rootp.join("go2_description/urdf");
        std::fs::create_dir_all(&urdf_dir).unwrap();
        std::fs::create_dir_all(rootp.join("go2_description/meshes")).unwrap();
        std::fs::create_dir_all(rootp.join("other_pkg/meshes")).unwrap();

        // package:// → the ancestor dir NAMED <pkg> roots <rest>.
        assert_eq!(
            resolve_mesh_path(&urdf_dir, "package://go2_description/meshes/base.dae"),
            rootp.join("go2_description/meshes/base.dae")
        );
        // package:// → a SIBLING <pkg> dir (found by walking to the common root).
        assert_eq!(
            resolve_mesh_path(&urdf_dir, "package://other_pkg/meshes/y.dae"),
            rootp.join("other_pkg/meshes/y.dae")
        );
        // package:// with NO matching dir → the one fallback: <urdf_dir>/../<rest>.
        assert_eq!(
            resolve_mesh_path(&urdf_dir, "package://nope/meshes/z.dae"),
            urdf_dir.parent().unwrap().join("meshes/z.dae")
        );
        // A relative path resolves against the URDF's directory (lexical join).
        assert_eq!(
            resolve_mesh_path(&urdf_dir, "../meshes/base.dae"),
            urdf_dir.join("../meshes/base.dae")
        );
        // An absolute path passes through unchanged.
        assert_eq!(
            resolve_mesh_path(&urdf_dir, "/abs/mesh.dae"),
            PathBuf::from("/abs/mesh.dae")
        );
    }

    #[test]
    fn dae_to_glb_extension_swap() {
        // The automagic swap: `.dae` → `.glb` (final extension replaced).
        assert_eq!(
            PathBuf::from("/x/base.dae").with_extension("glb"),
            PathBuf::from("/x/base.glb")
        );
        // A dotted stem keeps everything up to the final extension.
        assert_eq!(
            PathBuf::from("/x/a.b.dae").with_extension("glb"),
            PathBuf::from("/x/a.b.glb")
        );
    }

    #[test]
    fn present_glb_sibling_is_reported_as_a_mesh_asset() {
        let dir = tempfile::tempdir().expect("tempdir");
        let pkg = dir.path().join("go2_description");
        std::fs::create_dir_all(pkg.join("urdf")).unwrap();
        std::fs::create_dir_all(pkg.join("meshes")).unwrap();
        // The REAL sibling the automagic looks for (only the .glb needs to exist;
        // the .dae is never read by the runtime).
        write_min_glb(&pkg.join("meshes/base.glb"));

        let urdf_path = pkg.join("urdf/go2.urdf");
        std::fs::write(
            &urdf_path,
            mesh_urdf(
                "package://go2_description/meshes/base.dae",
                "0 0 0",
                "1 1 1",
            ),
        )
        .unwrap();

        let sk = Skeleton::load(Some(urdf_path.to_str().unwrap()));
        assert!(sk.is_active());
        let assets = sk.link_mesh_assets();
        assert_eq!(assets.len(), 1, "the one base mesh with a present .glb");
        assert_eq!(
            assets[0].entity, "world/tf-tree/robot/mesh",
            "the mesh logs on the link entity's /mesh child"
        );
        assert_eq!(assets[0].glb_path, pkg.join("meshes/base.glb"));
        assert_eq!(assets[0].scale, [1.0, 1.0, 1.0]);
        // Identity origin + unit scale ⇒ no own Transform3D (the mesh rides the
        // link transform as-is).
        assert!(assets[0].origin_transform().is_none());
    }

    #[test]
    fn absent_glb_sibling_yields_no_asset_and_reports_first_missing() {
        let dir = tempfile::tempdir().expect("tempdir");
        let pkg = dir.path().join("go2_description");
        std::fs::create_dir_all(pkg.join("urdf")).unwrap();
        std::fs::create_dir_all(pkg.join("meshes")).unwrap();
        // NO .glb sibling is written — only the .dae exists (which the runtime
        // never reads), so nothing resolves.
        std::fs::write(pkg.join("meshes/base.dae"), b"not read").unwrap();

        let urdf_path = pkg.join("urdf/go2.urdf");
        std::fs::write(
            &urdf_path,
            mesh_urdf(
                "package://go2_description/meshes/base.dae",
                "0 0 0",
                "1 1 1",
            ),
        )
        .unwrap();

        let sk = Skeleton::load(Some(urdf_path.to_str().unwrap()));
        assert!(sk.is_active(), "the stick figure still loads");
        assert!(
            sk.link_mesh_assets().is_empty(),
            "no .glb sibling ⇒ no mesh asset (sticks-only fallback)"
        );

        // The pure condition behind the once-per-load recipe info: resolve_mesh_assets
        // returns NO assets, counts the one miss, and names BOTH the first missing
        // .glb (the swapped path) and the original mesh it should sit next to.
        let model = parse_urdf(&std::fs::read_to_string(&urdf_path).unwrap()).unwrap();
        let res = model.resolve_mesh_assets(urdf_path.parent().unwrap());
        assert!(res.assets.is_empty());
        assert_eq!(
            res.missing_count, 1,
            "the single unresolved mesh is counted"
        );
        let miss = res.first_missing.expect("names the missing sibling");
        assert_eq!(
            miss.glb,
            pkg.join("meshes/base.glb"),
            "the recipe info names exactly the missing .glb sibling"
        );
        assert_eq!(
            miss.original,
            pkg.join("meshes/base.dae"),
            "and the original mesh path the .glb should sit next to (extension-agnostic)"
        );
    }

    /// Three mesh-bearing links, two with a present `.glb`
    /// sibling and one WITHOUT — `link_mesh_assets()` reports exactly the two
    /// (right entities), and `resolve_mesh_assets` counts the one miss and names
    /// its file. `link_visuals` is a `BTreeMap`, so iteration — and thus the
    /// first-missing pick — is by link NAME order (base, calf, thigh): with only
    /// `calf` missing, it is unambiguously the first (and only) miss.
    #[test]
    fn multi_link_meshes_report_resolved_and_missing_counts() {
        let dir = tempfile::tempdir().expect("tempdir");
        let pkg = dir.path().join("go2_description");
        std::fs::create_dir_all(pkg.join("urdf")).unwrap();
        std::fs::create_dir_all(pkg.join("meshes")).unwrap();
        // base + thigh have their .glb siblings; calf does NOT (only its .dae,
        // which the runtime never reads).
        write_min_glb(&pkg.join("meshes/base.glb"));
        write_min_glb(&pkg.join("meshes/thigh.glb"));
        std::fs::write(pkg.join("meshes/calf.dae"), b"not read").unwrap();

        // base → thigh → calf, each carrying a mesh visual.
        let urdf = r#"<?xml version="1.0"?>
<robot name="multi">
  <link name="base">
    <visual><geometry><mesh filename="package://go2_description/meshes/base.dae"/></geometry></visual>
  </link>
  <link name="thigh">
    <visual><geometry><mesh filename="package://go2_description/meshes/thigh.dae"/></geometry></visual>
  </link>
  <link name="calf">
    <visual><geometry><mesh filename="package://go2_description/meshes/calf.dae"/></geometry></visual>
  </link>
  <joint name="j1" type="fixed"><parent link="base"/><child link="thigh"/></joint>
  <joint name="j2" type="fixed"><parent link="thigh"/><child link="calf"/></joint>
</robot>"#;
        let urdf_path = pkg.join("urdf/go2.urdf");
        std::fs::write(&urdf_path, urdf).unwrap();

        // Public accessor: exactly the two resolved links, in BTreeMap (name)
        // order — base (world/tf-tree/robot/mesh) then thigh (world/tf-tree/robot/thigh/mesh).
        let sk = Skeleton::load(Some(urdf_path.to_str().unwrap()));
        assert!(sk.is_active());
        let entities: Vec<&str> = sk
            .link_mesh_assets()
            .iter()
            .map(|a| a.entity.as_str())
            .collect();
        assert_eq!(
            entities,
            ["world/tf-tree/robot/mesh", "world/tf-tree/robot/thigh/mesh"],
            "the two present-.glb links resolve to their /mesh child entities"
        );

        // Pure resolution: 2 assets, 1 miss, first-missing = calf (the only miss,
        // and lexicographically first among the missing set).
        let model = parse_urdf(urdf).unwrap();
        let res = model.resolve_mesh_assets(&pkg.join("urdf"));
        assert_eq!(res.assets.len(), 2);
        assert_eq!(res.missing_count, 1, "exactly the one absent calf sibling");
        let miss = res.first_missing.expect("the one miss is named");
        assert_eq!(miss.glb, pkg.join("meshes/calf.glb"));
        assert_eq!(miss.original, pkg.join("meshes/calf.dae"));
    }

    #[test]
    fn non_identity_visual_logs_a_transform_identity_skips() {
        // Non-unit scale ⇒ the mesh gets its own static Transform3D.
        let scaled = LinkMeshAsset {
            entity: "world/tf-tree/robot/mesh".to_string(),
            glb_path: PathBuf::from("/x/base.glb"),
            origin_xyz: [0.0; 3],
            origin_rpy: [0.0; 3],
            scale: [0.5, 0.5, 0.5],
        };
        assert!(
            scaled.origin_transform().is_some(),
            "a non-unit scale is logged even at an identity origin"
        );
        // A translated/rotated origin at unit scale ⇒ also a transform.
        let posed = LinkMeshAsset {
            origin_xyz: [1.0, 0.0, 0.0],
            scale: [1.0; 3],
            ..scaled.clone()
        };
        assert!(posed.origin_transform().is_some());
        // Fully identity ⇒ skipped.
        let ident = LinkMeshAsset {
            origin_xyz: [0.0; 3],
            origin_rpy: [0.0; 3],
            scale: [1.0; 3],
            ..scaled
        };
        assert!(ident.origin_transform().is_none());
    }

    /// Pin `origin_transform`'s PRODUCED component values —
    /// translation, quaternion `xyzw`, and scale — against a HAND-computed oracle
    /// for a non-trivial visual origin (a built `rerun::Transform3D` stores
    /// Arrow-serialized components that cannot be read back, so the split-out
    /// `origin_transform_components` is the oracle seam).
    #[test]
    fn mesh_origin_transform_components_hand_oracle() {
        // origin xyz = (0.5, 0.25, -0.75) (all exactly f32-representable),
        // rpy = (π/2, 0, π/2), scale = (2, 3, 4). The rotation is the URDF
        // fixed-axis compose Rz(π/2)·Ry(0)·Rx(π/2): with s = c = √2/2 this is the
        // clean quaternion (0.5, 0.5, 0.5, 0.5) — the SAME value pinned by
        // `from_rpy_fixed_axis_composition_hand_oracle`; the reversed order would
        // give (0.5, −0.5, 0.5, 0.5), so this also pins the compose order.
        let asset = LinkMeshAsset {
            entity: "world/tf-tree/robot/mesh".to_string(),
            glb_path: PathBuf::from("/x/base.glb"),
            origin_xyz: [0.5, 0.25, -0.75],
            origin_rpy: [FRAC_PI_2, 0.0, FRAC_PI_2],
            scale: [2.0, 3.0, 4.0],
        };
        let (translation, quat, scale) = asset
            .origin_transform_components()
            .expect("a non-identity visual ⇒ Some");
        // Translation = the origin xyz, narrowed to f32 (values are exact).
        assert_eq!(translation, [0.5f32, 0.25, -0.75]);
        // Scale = the mesh scale, narrowed to f32 (exact integers).
        assert_eq!(scale, [2.0f32, 3.0, 4.0]);
        // Quaternion xyzw = (0.5, 0.5, 0.5, 0.5) (hand-derived above).
        let f32_close = |a: f32, b: f32| (a - b).abs() < 1e-6;
        assert!(
            quat.iter()
                .zip([0.5f32, 0.5, 0.5, 0.5])
                .all(|(a, b)| f32_close(*a, b)),
            "quaternion xyzw = {quat:?}, expected (0.5, 0.5, 0.5, 0.5)"
        );
        // The public builder still yields Some for this non-identity visual.
        assert!(asset.origin_transform().is_some());
    }

    #[test]
    fn from_urdf_str_does_not_resolve_meshes() {
        // The dir-less path parses the visual but resolves NO assets (it has no
        // base directory to root relative / package:// paths against) — meshes
        // are additive; the stick figure is unaffected.
        let sk = Skeleton::from_urdf_str(&mesh_urdf(
            "package://go2_description/meshes/base.dae",
            "0 0 0",
            "1 1 1",
        ))
        .expect("active");
        assert!(sk.is_active());
        assert!(
            sk.link_mesh_assets().is_empty(),
            "from_urdf_str never resolves meshes (no URDF directory)"
        );
    }
}
