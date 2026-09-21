// SPDX-License-Identifier: AGPL-3.0-only
//! The tier-audit FOLLOW-ON: the slice tiers for the six vendored packages that were
//! taking the 128 MiB user-defined catch-all SILENTLY, plus the registration
//! gate that stops the next vendored package from doing the same.
//!
//! # The defect this file closes
//!
//! `variable_schema_max_slice_len` ends in a `_` catch-all documented as
//! "reserved for user-defined schemas", guarded by a panic: a variable schema
//! whose package is in `IN_REPO_PACKAGES` and has no explicit arm fails the
//! BUILD rather than silently reserving 128 MiB per topic.
//!
//! That guard cannot see an UNREGISTERED package — the panic is gated on
//! `IN_REPO_PACKAGES.contains(&pkg)`, so a vendored package missing from the
//! list is precisely the case it is blind to. Six were missing
//! (`autoware_perception_msgs`, `autoware_planning_msgs`, `grid_map_msgs`,
//! `radar_msgs`, `unique_identifier_msgs`, `vision_msgs` — all vendored for
//! the robotics benchmarks), and their 24 variable schemas took the
//! catch-all. `vision_msgs/ObjectHypothesis` is a string and a float64; it was
//! reserving the same 128 MiB as an 8K camera frame, and deriving the same
//! bridged ingress depth (64) as a point cloud.
//!
//! The maintenance instruction to register a new package was already written
//! in the table. It was not followed, and nothing noticed for a year. So this
//! file replaces the instruction with a GATE.
//!
//! # What each test is for
//!
//! * [`every_vendored_package_is_registered_so_none_takes_the_catch_all_silently`]
//!   — the gate. Set EQUALITY between `IN_REPO_PACKAGES` and the directories
//!   under `msg/`, checked in both directions.
//! * [`the_catch_all_packages_land_on_their_audited_tiers`] — the 24 VALUES,
//!   each pinned to the literal byte count of its tier (the `max_slice_len_test`
//!   convention: a literal cannot be satisfied by a coordinated re-bless of the
//!   `TIER_*` constants).
//! * [`the_fixed_types_in_the_catch_all_packages_never_consult_the_tier_table`]
//!   — the ten types in those packages that resolve FIXED, and therefore never
//!   had a tier problem at all. Recorded because the premise "16 fixed bytes
//!   reserving 128 MiB" is a natural thing to assume about
//!   `unique_identifier_msgs/UUID` and it is FALSE.
//! * [`joint_trajectory_controller_state_lands_on_small`] — the other
//!   follow-on, pinned together with the two members whose own move unblocked
//!   it.
//!
//! Totality (no in-repo variable schema without an arm) and the container rule
//! are `tier_container_rule_test.rs`'s job; the CAPACITY arithmetic behind each
//! placement is `tier_capacity_test.rs`'s.

use cerulion_core::codegen::IN_REPO_PACKAGES;
use cerulion_core::message::ShmMessage;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use std::collections::BTreeSet;
use std::path::PathBuf;

/// Every package directory under `native_ros2_messages/msg/`, with `build.rs`'s
/// own name sanitisation applied so the two sides are comparable.
fn vendored_package_dirs() -> BTreeSet<String> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("msg");
    std::fs::read_dir(&root)
        .unwrap_or_else(|e| panic!("read {}: {e}", root.display()))
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.is_dir())
        .map(|p| {
            p.file_name()
                .expect("package dir name")
                .to_string_lossy()
                .replace('-', "_")
        })
        .collect()
}

#[test]
fn every_vendored_package_is_registered_so_none_takes_the_catch_all_silently() {
    let vendored = vendored_package_dirs();
    let registered: BTreeSet<String> = IN_REPO_PACKAGES.iter().map(|s| s.to_string()).collect();

    // Anti-tautology: a walk that read an empty directory would pass every
    // assertion below vacuously. The floor is deliberately far under the real
    // count so it pins "the tree was read", not a census.
    assert!(
        vendored.len() >= 20,
        "only {} package directories found under msg/ — the walk is not reaching the \
         vendored tree, so the comparisons below are vacuous",
        vendored.len()
    );

    let unregistered: Vec<_> = vendored.difference(&registered).collect();
    assert!(
        unregistered.is_empty(),
        "TIER FOLLOW-ON: {} vendored package(s) are NOT in `IN_REPO_PACKAGES`: {unregistered:?}\n\n\
         Every variable schema in an unregistered package takes the 128 MiB user-defined \
         catch-all in `variable_schema_max_slice_len` — SILENTLY, because that function's panic \
         guard is itself gated on `IN_REPO_PACKAGES.contains(&pkg)` and so cannot fire for a \
         package it does not know about. That is exactly how six packages' 24 variable schemas \
         came to reserve 128 MiB apiece.\n\n\
         THE FIX: add the package to `IN_REPO_PACKAGES` in \
         `cerulion_core/src/codegen/generator/wire_impl.rs`, then build. The build will now FAIL \
         for each of its variable schemas until you give each one an explicit tier arm — which is \
         the point, and is what makes that build failure the totality proof.",
        unregistered.len()
    );

    let phantom: Vec<_> = registered.difference(&vendored).collect();
    assert!(
        phantom.is_empty(),
        "TIER FOLLOW-ON: {} package(s) in `IN_REPO_PACKAGES` are not vendored under msg/: \
         {phantom:?}\n\n\
         Checked in this direction too so the list stays a description of the tree rather than a \
         wish-list. A name here that no longer exists reads as coverage the table does not have; \
         delete it (and its now-dead arms).",
        phantom.len()
    );
}

/// The tier byte counts, written out. Every assertion below compares against
/// one of these LITERALS rather than against a re-exported `TIER_*` constant,
/// so a re-bless that edited the table and the constants in lockstep cannot
/// satisfy them.
const HUGE: u32 = 134_217_728;
const MEDIUM: u32 = 4_194_304;
const SMALL: u32 = 262_144;
const TINY: u32 = 16_384;

macro_rules! pin {
    ($($t:ty => $bytes:expr, $why:literal;)*) => {
        $(assert_eq!(
            <$t as ShmMessage>::MAX_SLICE_LEN,
            MaxSliceLen::try_new($bytes),
            concat!("tier follow-on ", stringify!($t), ": ", $why),
        );)*
    };
}

/// All 24 variable schemas from the six formerly-unregistered packages.
///
/// The count is not asserted here — it is proved by construction elsewhere:
/// registering the six packages makes codegen PANIC for any variable schema
/// among them still lacking an arm, so `cargo build -p native_ros2_messages`
/// succeeding is the statement that the set below is complete.
///
/// Read off the generated trait const rather than
/// `variable_schema_max_slice_len`, so this also proves codegen carried the
/// number into `native_ros2_messages` instead of pinning the table to itself.
#[test]
fn the_catch_all_packages_land_on_their_audited_tiers() {
    use native_ros2_messages::{
        autoware_perception_msgs as ap, autoware_planning_msgs as al, grid_map_msgs as gm,
        radar_msgs as rm, vision_msgs as vm,
    };

    // ── KEEP 128 MiB, now deliberately ──
    pin! {
        gm::GridMap => HUGE, "the multi-layer float generalisation of nav_msgs/OccupancyGrid \
            (itself HUGE): an 8-layer 1000x1000 elevation map measures 32,001,693 B, whose \
            x4 headroom is 128,006,772 — this tier's own ceiling";
        ap::PredictedObjects => HUGE, "a realistic 100-object frame is 1,851,693 B (x4 would say \
            LARGE), but LARGE hard-fails at 29 objects filled to the IDL's own per-object bound, \
            inside the count a dense urban scene reaches";
    }

    // ── MEDIUM: a conforming producer can fill these past SMALL ──
    pin! {
        ap::PredictedObjectKinematics => MEDIUM, "IDL PredictedPath[<=100] x Pose[<=100] = \
            563,460 B, which SMALL cannot hold at all";
        ap::PredictedObject => MEDIUM, "564,816 B at the IDL maximum; floor is the kinematics \
            member anyway";
        al::LaneletRoute => MEDIUM, "500 segments x 4 primitives = 122,729 B (x4 = 490,916, over \
            SMALL); one-shot latched, so SMALL's depth buys nothing to weigh against the risk";
        al::Trajectory => MEDIUM, "IDL TrajectoryPoint[<=10000] over an 88 B fixed element = \
            880,089 B; SMALL holds only 2,977 points, under the declared bound";
        rm::RadarScan => MEDIUM, "a 4D imaging radar frame reaches ~30,000 returns = 600,089 B, \
            which SMALL cannot hold";
        vm::Detection2DArray => MEDIUM, "300 detections x 1 hypothesis (YOLO max_det default) = \
            166,900 B; x4 = 667,600";
        vm::Detection3DArray => MEDIUM, "500 detections x 1 hypothesis (the nuScenes 3D cap) = \
            298,100 B; x4 = 1,192,400";
        vm::LabelInfo => MEDIUM, "a 1,000-class map with 64-char names is 78,136 B; x4 = 312,544, \
            over SMALL. Latched, so again no depth to trade";
    }

    // ── SMALL: 4x bridged ingress depth (64 -> 256) on per-frame outputs ──
    pin! {
        ap::PredictedPath => SMALL, "IDL Pose[<=100] over a 56 B fixed element = 5,652 B; \
            x4 = 22,608 clears TINY";
        ap::Shape => SMALL, "FLOOR-placed: 1,280 B at a 100-vertex footprint is TINY's class, but \
            it embeds geometry_msgs/Polygon (SMALL)";
        rm::RadarTracks => SMALL, "255 tracks x a 216 B fixed RadarTrack = 55,169 B; x4 = 220,676";
        vm::BoundingBox2DArray => SMALL, "1,000 boxes x a 40 B fixed element = 40,089 B";
        vm::BoundingBox3DArray => SMALL, "500 boxes x an 80 B fixed element = 40,089 B";
        vm::Classification => SMALL, "a full 1,000-class softmax with 32-char ids = 52,100 B";
        vm::Detection2D => SMALL, "100 hypotheses = 40,580 B; standard detectors emit 1";
        vm::Detection3D => SMALL, "100 hypotheses = 40,620 B";
    }

    // ── TINY: the small records that were reserving 128 MiB apiece ──
    pin! {
        al::LaneletPrimitive => TINY, "int64 id + a short type string = 73 B";
        al::LaneletSegment => TINY, "20 candidate primitives = 993 B; floor holds at equality";
        vm::ObjectHypothesis => TINY, "a class id string + a score = 80 B — the headline of this \
            sweep: it was reserving 128 MiB, the same as an 8K image";
        vm::ObjectHypothesisWithPose => TINY, "that plus a fixed 344 B PoseWithCovariance = 432 B";
        vm::VisionClass => TINY, "uint16 + a class name = 106 B";
        vm::VisionInfo => TINY, "Header + method + database location + version = 1,612 B even at \
            512-char paths for both strings";
    }
}

/// The ten types in those six packages that resolve FIXED.
///
/// They are recorded because the obvious summary of this sweep —
/// "`unique_identifier_msgs/UUID` is 16 bytes and was reserving 128 MiB" — is
/// FALSE, and it is the kind of false that survives review. `uint8[16]` is a
/// FixedArray, so `resolve_fixed_nested` classifies UUID FIXED, and a fixed
/// schema never reaches the tier table: its `MAX_SLICE_LEN` is the exact,
/// provably-correct `WireHeader::SIZE + WIRE_FIXED_SIZE`. `unique_identifier_msgs`
/// therefore contributes ZERO arms while still being registered — the
/// registration is what makes a future variable field on it a build failure.
///
/// Asserting the exact auto-computed bound (not merely "small") is what would
/// catch the real regression here: a change that made one of these VARIABLE
/// again would send it to the tier table, where it has no arm, and the build
/// would fail — but if it were given a lazy arm instead, this test is what
/// notices the type stopped being exactly-sized.
#[test]
fn the_fixed_types_in_the_catch_all_packages_never_consult_the_tier_table() {
    use native_ros2_messages::{
        autoware_perception_msgs as ap, autoware_planning_msgs as al, grid_map_msgs as gm,
        radar_msgs as rm, unique_identifier_msgs as ui, vision_msgs as vm,
    };

    macro_rules! assert_exact {
        ($($t:ty),* $(,)?) => {$(
            assert_eq!(
                <$t as ShmMessage>::VARIABLE_FIELD_COUNT, 0,
                concat!(stringify!($t), " is expected to resolve FIXED"),
            );
            assert_eq!(
                <$t as ShmMessage>::MAX_SLICE_LEN,
                MaxSliceLen::try_new(
                    (WireHeader::SIZE + <$t as ShmMessage>::WIRE_FIXED_SIZE) as u32
                ),
                concat!(
                    stringify!($t),
                    ": a fixed schema's slice is the exact frame size, never a tier",
                ),
            );
        )*};
    }

    assert_exact!(
        ui::UUID,
        ap::ObjectClassification,
        al::TrajectoryPoint,
        gm::GridMapInfo,
        rm::RadarReturn,
        rm::RadarTrack,
        vm::BoundingBox2D,
        vm::BoundingBox3D,
        vm::Point2D,
        vm::Pose2D,
    );

    // The headline number, spelled out: UUID's whole frame is 48 bytes, and the
    // 16 of them that are payload are `uint8[16]`.
    assert_eq!(<ui::UUID as ShmMessage>::WIRE_FIXED_SIZE, 16);
    assert_eq!(
        <ui::UUID as ShmMessage>::MAX_SLICE_LEN,
        MaxSliceLen::try_new(48),
        "unique_identifier_msgs/UUID is a 48-byte frame — 32 B of WireHeader plus 16 B of \
         payload — and reaches that number without the tier table's involvement",
    );
}

/// The tier-audit follow-on, the other half: `JointTrajectoryControllerState`
/// MEDIUM -> SMALL.
///
/// The audit recorded this one as "checked_ok at MEDIUM only because it was
/// pinned by the trajectory-point members" and opened it as a follow-on. Both
/// of those members moved to SMALL in the audit's own batch, which lowered this
/// container's floor, and its own worst case × 4 had always fitted SMALL.
///
/// The two members are pinned HERE as well as in `max_slice_len_test.rs`,
/// because they are the PRECONDITION: if either ever moves back up, this arm
/// becomes a container-rule violation, and a reader of this test should be able
/// to see why from this test.
#[test]
fn joint_trajectory_controller_state_lands_on_small() {
    use native_ros2_messages::control_msgs::JointTrajectoryControllerState;
    use native_ros2_messages::std_msgs::Header;
    use native_ros2_messages::trajectory_msgs::{
        JointTrajectoryPoint, MultiDOFJointTrajectoryPoint,
    };

    pin! {
        JointTrajectoryControllerState => SMALL, "MEDIUM -> SMALL. A 60-DOF humanoid \
            whole-body group measures 9,885 B (four JointTrajectoryPoints over the group's \
            joints, plus the names); x4 = 39,540 fits SMALL and does NOT fit TINY. MEASURED \
            boundary: SMALL holds 1,666 joints, 28x the widest real controller group";
    }

    // The floor, and the precondition that unblocked the move.
    pin! {
        Header => TINY, "the container's smallest variable member";
        JointTrajectoryPoint => SMALL, "PRECONDITION (the tier-audit batch): this member's own \
            MEDIUM -> SMALL move is what lowered JTCS's floor to SMALL";
        MultiDOFJointTrajectoryPoint => SMALL, "PRECONDITION (the tier-audit batch), same";
    }
}
