// SPDX-License-Identifier: AGPL-3.0-only
//! **every** generated ROS 2 message type carries serde.
//!
//! # Why a corpus walk rather than a list
//!
//! `serde`'s array impls stop at 32 elements, so three vendored covariance
//! messages (`geometry_msgs/{Pose,Twist,Accel}WithCovariance`, each carrying a
//! `float64[36]`) cannot derive it with a plain derive, and because nested
//! resolution is transitive, that absence propagates to every schema embedding
//! one. `nav_msgs/Odometry` is in that set, which is what makes this a
//! shipping problem rather than a curiosity: a localization or SLAM node
//! caching the latest pose estimate holds an `Odometry`, and a node
//! state type without serde cannot be serialized.
//!
//! The gate is a count over the corpus, and it is a WALK for the same
//! reason CI's viz job names packages rather than test files: a hand-written
//! list of poisoned types goes stale the moment somebody vendors a message,
//! and the failure mode is silent. The count is over
//! GENERATED OUTPUT, so it must be re-measured after a rebuild — reading
//! `OUT_DIR` at compile time is what makes that automatic.
//!
//! **Without the generator's `big_array` emission, 9 of the 254 snapshot
//! types have no serde.** Two of them are
//! `autoware_perception_msgs/PredictedObjectKinematics` and
//! `vision_msgs/ObjectHypothesisWithPose`, both transitively poisoned through
//! the same three covariance types. The number moves whenever a message is
//! vendored, which is precisely the argument for measuring it here instead of
//! quoting it.
//!
//! # What each arm is for
//!
//! - [`every_generated_snapshot_type_derives_serde`] is the GATE. Its
//!   companion oracle is an INDEPENDENT enumeration — the vendored `.msg`
//!   tree — because "zero types are missing serde" is trivially satisfied by a
//!   walk that found no types at all.
//! - The round-trip arms are COMPILE-TIME proofs for one type of each shape
//!   (fixed, variable, direct-large-array, transitively-poisoned): the walk
//!   reads text, so only a real `to_string`/`from_str` proves the emitted
//!   attributes actually work.

use std::path::{Path, PathBuf};

use native_ros2_messages::{geometry_msgs, nav_msgs, sensor_msgs, std_msgs};

/// The directory `build.rs` generated this crate's types into.
///
/// `OUT_DIR` is baked in at COMPILE time, so the walk can never read a stale
/// tree from an unrelated build.
const GENERATED_DIR: &str = env!("OUT_DIR");

/// A `<Name>Snapshot` struct found in the generated corpus.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord)]
struct SnapshotType {
    file: String,
    name: String,
    /// Every `#[...]` attribute line directly above the `pub struct` line,
    /// joined. Collected as a BLOCK rather than "the previous line" because
    /// the emission is two lines (`#[derive(..)]` + `#[serde(crate = ..)]`)
    /// and a one-line lookback would silently read the wrong one.
    attributes: String,
}

impl SnapshotType {
    fn derives_serde(&self) -> bool {
        self.attributes.contains("Serialize") && self.attributes.contains("Deserialize")
    }
}

/// Every `<Name>Snapshot` struct in the generated corpus, with the attribute
/// block that applies to it.
fn walk_generated_snapshots() -> Vec<SnapshotType> {
    let dir = Path::new(GENERATED_DIR);
    let entries = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("cannot read the generated corpus at {}: {e}", dir.display()));

    let mut found = Vec::new();
    for entry in entries {
        let path = entry.expect("read_dir entry").path();
        if path.extension().and_then(|e| e.to_str()) != Some("rs") {
            continue;
        }
        let file_name = path
            .file_name()
            .and_then(|n| n.to_str())
            .expect("generated file name")
            .to_string();
        let source = std::fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));

        let mut attributes = String::new();
        for line in source.lines() {
            if line.starts_with("#[") {
                attributes.push_str(line);
                attributes.push('\n');
                continue;
            }
            if let Some(name) = snapshot_struct_name(line) {
                found.push(SnapshotType {
                    file: file_name.clone(),
                    name,
                    attributes: std::mem::take(&mut attributes),
                });
            }
            // Any other line — a doc comment, a blank, a field — ends the
            // attribute block. Doc comments precede the derives in the emitted
            // order, so this cannot drop a real attribute.
            attributes.clear();
        }
    }
    found.sort();
    found
}

/// `Some("FooSnapshot")` for a top-level `pub struct FooSnapshot {` line.
fn snapshot_struct_name(line: &str) -> Option<String> {
    let rest = line.strip_prefix("pub struct ")?;
    let name = rest.split([' ', '{', '<', ';', '(']).next()?;
    if name.ends_with("Snapshot") && !name.is_empty() {
        Some(name.to_string())
    } else {
        None
    }
}

/// Count the vendored `.msg` files — the INDEPENDENT enumeration.
///
/// Codegen emits exactly one `<Name>Snapshot` per schema, so this number and
/// the walk's total must agree. Disagreement means the walk missed a generated
/// file (e.g. a package's `.rs` skipped), which no "zero missing" assertion
/// could catch.
fn count_vendored_msg_files() -> usize {
    fn recurse(dir: &Path, count: &mut usize) {
        for entry in std::fs::read_dir(dir).unwrap_or_else(|e| {
            panic!(
                "cannot read the vendored msg tree at {}: {e}",
                dir.display()
            )
        }) {
            let path = entry.expect("read_dir entry").path();
            if path.is_dir() {
                recurse(&path, count);
            } else if path.extension().and_then(|e| e.to_str()) == Some("msg") {
                *count += 1;
            }
        }
    }
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("msg");
    let mut count = 0;
    recurse(&root, &mut count);
    count
}

/// THE GATE: zero types without serde.
#[test]
fn every_generated_snapshot_type_derives_serde() {
    let snapshots = walk_generated_snapshots();

    // Anti-vacuity, part 1: the walk found types at all.
    assert!(
        !snapshots.is_empty(),
        "the walk over {GENERATED_DIR} found no `<Name>Snapshot` types — \
         the corpus moved or the parser broke, so a zero-missing verdict \
         would be meaningless"
    );

    // Anti-vacuity, part 2 (this fails a walk that skips a
    // package): an INDEPENDENT enumeration of the same corpus must agree.
    let vendored = count_vendored_msg_files();
    assert_eq!(
        snapshots.len(),
        vendored,
        "codegen emits one `<Name>Snapshot` per vendored `.msg`, so the walk \
         ({} types) and the `.msg` tree ({vendored} files) must agree; a \
         shortfall means the walk skipped generated output",
        snapshots.len()
    );

    let missing: Vec<&SnapshotType> = snapshots.iter().filter(|s| !s.derives_serde()).collect();
    assert!(
        missing.is_empty(),
        "{} of {} generated snapshot types do not derive serde (the \
         gate: ZERO):\n{}",
        missing.len(),
        snapshots.len(),
        missing
            .iter()
            .map(|s| format!("  - {} ({})", s.name, s.file))
            .collect::<Vec<_>>()
            .join("\n")
    );
}

/// The derive names cerulion_core's `serde` RE-EXPORT, not a bare `::serde`.
///
/// A scaffolded node crate depends on `cerulion_core` and
/// `native_ros2_messages` only (`cerulion_cli_engine/src/templates.rs`), so a
/// bare path does not resolve there. This crate CAN NOT catch that by
/// compiling — it would just need its own serde dep — which is exactly why
/// this crate has no serde dep and this assertion exists.
#[test]
fn the_generated_derives_name_the_cerulion_core_re_export() {
    for snapshot in walk_generated_snapshots() {
        assert!(
            snapshot
                .attributes
                .contains("::cerulion_core::serde::Serialize"),
            "{} ({}) must derive through the re-export; got:\n{}",
            snapshot.name,
            snapshot.file,
            snapshot.attributes
        );
        // REQUIRED: without it serde's derive emits `extern crate serde as
        // _serde;`, which needs serde in the extern prelude of whatever crate
        // the generated code is compiled into.
        assert!(
            snapshot
                .attributes
                .contains("#[serde(crate = \"::cerulion_core::serde\")]"),
            "{} ({}) must carry the serde `crate` attribute; got:\n{}",
            snapshot.name,
            snapshot.file,
            snapshot.attributes
        );
        // TOTAL, in every derive position.
        // A `contains("(::serde::")` only sees a bare path as the FIRST derive,
        // so `#[derive(Debug, ::serde::Serialize, …)]` — or the re-exported
        // path AND a bare one on the same line — passed while a scaffolded
        // node crate still failed to compile. Blank the legitimate prefix,
        // then reject any residual.
        let residual = snapshot
            .attributes
            .replace("::cerulion_core::serde::", "«ok»");
        assert!(
            !residual.contains("::serde::"),
            "{} ({}) names a bare `::serde` path; got:\n{}",
            snapshot.name,
            snapshot.file,
            snapshot.attributes
        );
    }
}

/// A FIXED schema round-trips against a hand-built value.
#[test]
fn a_fixed_snapshot_round_trips_to_a_hand_built_value() {
    let original = geometry_msgs::Vector3Snapshot {
        x: -1.25,
        y: 0.5,
        z: 1024.75,
    };

    let json = serde_json::to_string(&original).expect("serialize");
    let recovered: geometry_msgs::Vector3Snapshot =
        serde_json::from_str(&json).expect("deserialize");

    // Hand oracle, field by field — never `assert_eq!(recovered, original)`
    // alone, which a swapped-field bug would satisfy if the values matched.
    assert_eq!(recovered.x, -1.25);
    assert_eq!(recovered.y, 0.5);
    assert_eq!(recovered.z, 1024.75);
}

/// A VARIABLE schema round-trips (its variable fields are `Vec`/`String`).
#[test]
fn a_variable_snapshot_round_trips_to_a_hand_built_value() {
    let original = sensor_msgs::ImageSnapshot {
        header: vec![7, 8, 9],
        height: 480,
        width: 640,
        is_bigendian: 1,
        step: 1920,
        encoding: "rgb8".to_string(),
        data: vec![1, 2, 3, 4, 5],
    };

    let json = serde_json::to_string(&original).expect("serialize");
    let recovered: sensor_msgs::ImageSnapshot = serde_json::from_str(&json).expect("deserialize");

    assert_eq!(recovered.header, vec![7, 8, 9]);
    assert_eq!(recovered.height, 480);
    assert_eq!(recovered.width, 640);
    assert_eq!(recovered.is_bigendian, 1);
    assert_eq!(recovered.step, 1920);
    assert_eq!(recovered.encoding, "rgb8");
    assert_eq!(recovered.data, vec![1, 2, 3, 4, 5]);
}

/// A `std_msgs/String` round-trips — the smallest variable shape, and the one
/// whose schema hash the CLI pins.
#[test]
fn a_string_snapshot_round_trips_to_a_hand_built_value() {
    let original = std_msgs::StringSnapshot {
        data: "hand-built".to_string(),
    };
    let json = serde_json::to_string(&original).expect("serialize");
    let recovered: std_msgs::StringSnapshot = serde_json::from_str(&json).expect("deserialize");
    assert_eq!(recovered.data, "hand-built");
}

/// THE poisoned shape: a DIRECT `float64[36]`, through the big-array helper.
#[test]
fn a_direct_large_array_snapshot_round_trips_to_a_hand_built_value() {
    let mut covariance = [0.0f64; 36];
    // Every element distinct, so a reversal, a truncation, or an off-by-one
    // shows up as a value mismatch rather than passing on a zero-filled array.
    for (index, slot) in covariance.iter_mut().enumerate() {
        *slot = (index as f64) * 0.125 - 3.0;
    }
    let original = geometry_msgs::PoseWithCovarianceSnapshot {
        pose: geometry_msgs::PoseSnapshot {
            position: geometry_msgs::PointSnapshot {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            orientation: geometry_msgs::QuaternionSnapshot {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        },
        covariance,
    };

    let json = serde_json::to_string(&original).expect("serialize");
    let recovered: geometry_msgs::PoseWithCovarianceSnapshot =
        serde_json::from_str(&json).expect("deserialize");

    assert_eq!(recovered.pose.position.x, 1.0);
    assert_eq!(recovered.pose.position.y, 2.0);
    assert_eq!(recovered.pose.position.z, 3.0);
    assert_eq!(recovered.pose.orientation.w, 1.0);
    for index in 0..36 {
        assert_eq!(
            recovered.covariance[index],
            (index as f64) * 0.125 - 3.0,
            "covariance element {index} did not survive the round trip"
        );
    }
}

/// THE headline type: `nav_msgs/Odometry`, poisoned TRANSITIVELY
/// through two covariance members, neither of which it declares itself.
#[test]
fn a_transitively_poisoned_snapshot_round_trips_to_a_hand_built_value() {
    let mut pose_cov = [0.0f64; 36];
    let mut twist_cov = [0.0f64; 36];
    // DIFFERENT fills per member, so a bug that serializes one member twice
    // (or swaps them) is visible.
    for index in 0..36 {
        pose_cov[index] = index as f64;
        twist_cov[index] = -(index as f64);
    }

    let original = nav_msgs::OdometrySnapshot {
        header: vec![1, 2],
        child_frame_id: "base_link".to_string(),
        pose: geometry_msgs::PoseWithCovarianceSnapshot {
            pose: geometry_msgs::PoseSnapshot::default(),
            covariance: pose_cov,
        },
        twist: geometry_msgs::TwistWithCovarianceSnapshot {
            twist: geometry_msgs::TwistSnapshot::default(),
            covariance: twist_cov,
        },
    };

    let json = serde_json::to_string(&original).expect("serialize");
    let recovered: nav_msgs::OdometrySnapshot = serde_json::from_str(&json).expect("deserialize");

    assert_eq!(recovered.header, vec![1, 2]);
    assert_eq!(recovered.child_frame_id, "base_link");
    for index in 0..36 {
        assert_eq!(recovered.pose.covariance[index], index as f64);
        assert_eq!(recovered.twist.covariance[index], -(index as f64));
    }
}

/// The parser the gate depends on, tested against hand-written input rather
/// than trusted for self-agreement.
///
/// Without this, a `snapshot_struct_name` that silently matched nothing would
/// make the gate's "zero missing" verdict vacuous — the `.msg` count oracle
/// catches a total shortfall, but not a per-line parse that is subtly wrong.
#[test]
fn the_struct_name_parser_matches_only_top_level_snapshot_structs() {
    assert_eq!(
        snapshot_struct_name("pub struct Vector3Snapshot {"),
        Some("Vector3Snapshot".to_string())
    );
    // Non-snapshot generated types must not be counted.
    assert_eq!(snapshot_struct_name("pub struct Vector3Shm {"), None);
    assert_eq!(snapshot_struct_name("pub struct Vector3;"), None);
    assert_eq!(snapshot_struct_name("pub struct ImageShm<'a> {"), None);
    // Indented (nested) declarations are not top-level corpus types.
    assert_eq!(snapshot_struct_name("    pub struct XSnapshot {"), None);
    // Prose that merely mentions the shape is not a declaration.
    assert_eq!(snapshot_struct_name("/// pub struct XSnapshot {"), None);
}
