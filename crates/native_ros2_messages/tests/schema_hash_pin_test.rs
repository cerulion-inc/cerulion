// SPDX-License-Identifier: AGPL-3.0-only
//! Pin tests for generated `SCHEMA_HASH` /
//! `WIRE_FIXED_SIZE` consistency on representative ROS2 messages.
//!
//! Each test re-parses the FULL `.msg` corpus at test time and runs
//! `resolve_fixed_nested` over it — the SAME parse → resolve pipeline
//! `build.rs` uses — then compares the IR-computed values for one schema
//! against the constants baked into the generated code.
//!
//! Resolving the whole corpus (not a single `.msg` in isolation) is
//! REQUIRED under recipe 3: a message with a recursively-fixed nested
//! field (e.g. `sensor_msgs/Imu` embedding `geometry_msgs/Quaternion` /
//! `Vector3`) only inlines those targets into its fixed section AFTER
//! resolution. Parsing `Imu.msg` alone would leave them classified
//! variable, under-counting `wire_fixed_size()` and dropping the nested
//! layout from `schema_hash()` — diverging from the generated (resolved)
//! constants.
//!
//! # What each half genuinely proves
//!
//! - **`WIRE_FIXED_SIZE` half** — a genuine rustc oracle.
//!   `<T as ShmMessage>::WIRE_FIXED_SIZE` is `size_of::<FixedSection>()`,
//!   computed by *rustc* from the emitted `#[repr(C)]` struct (which, post
//!   resolve, inlines `<Target>Shm` fixed-nested members).
//!   `MessageSchema::wire_fixed_size()` re-derives the same number from
//!   the resolved IR with hand-written layout rules. Agreement proves the
//!   IR layout algorithm matches the compiler's `#[repr(C)]` semantics —
//!   two independent implementations.
//!
//! - **`SCHEMA_HASH` half** — this catches *pipeline drift
//!   only*: build.rs and this test both call the same
//!   `MessageSchema::schema_hash()` recipe function, so the assertion
//!   cannot detect a bug inside the recipe itself. What it does catch:
//!   the generated constant going stale (build.rs not re-run), a `.msg`
//!   edit not propagating, or the parse → resolve → generate pipeline
//!   diverging from the parse → resolve → test pipeline.

use cerulion_core::codegen::{parse_rosmsg, resolve_fixed_nested, MessageSchema};
use cerulion_core::message::ShmMessage;
use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::OnceLock;

/// Parse + resolve the entire vendored `.msg` corpus ONCE, exactly as
/// `build.rs` does, and return a `(package, name) -> resolved schema` map.
fn resolved_corpus() -> &'static BTreeMap<(String, String), MessageSchema> {
    static CORPUS: OnceLock<BTreeMap<(String, String), MessageSchema>> = OnceLock::new();
    CORPUS.get_or_init(|| {
        let msg_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("msg");
        let mut flat: Vec<MessageSchema> = Vec::new();
        // msg/<package>/<Name>.msg
        let mut pkg_dirs: Vec<PathBuf> = std::fs::read_dir(&msg_dir)
            .unwrap_or_else(|e| panic!("read {}: {e}", msg_dir.display()))
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.is_dir())
            .collect();
        pkg_dirs.sort();
        for pkg_dir in pkg_dirs {
            // Mirror build.rs's `sanitize_package_name` (`-` → `_`) so the
            // package string we hash over matches the generated constant for
            // any hyphenated package dir (all dirs use
            // underscores, so this is a no-op — kept for fidelity).
            let package = pkg_dir
                .file_name()
                .unwrap()
                .to_string_lossy()
                .replace('-', "_");
            let mut msg_files: Vec<PathBuf> = std::fs::read_dir(&pkg_dir)
                .unwrap_or_else(|e| panic!("read {}: {e}", pkg_dir.display()))
                .flatten()
                .map(|e| e.path())
                .filter(|p| p.extension().is_some_and(|x| x == "msg"))
                .collect();
            msg_files.sort();
            for msg_path in msg_files {
                let msg_name = msg_path.file_stem().unwrap().to_string_lossy().to_string();
                let content = std::fs::read_to_string(&msg_path)
                    .unwrap_or_else(|e| panic!("read {}: {e}", msg_path.display()));
                let schema = parse_rosmsg(&content, &msg_name, Some(&package))
                    .unwrap_or_else(|e| panic!("parse {package}/{msg_name}.msg: {e}"));
                flat.push(schema);
            }
        }
        // build.rs is the authority (it PANICS on any resolution warning for
        // the vendored set), but assert clean here too: cheap belt-and-
        // suspenders that converts a future build-vs-test corpus drift from a
        // silently-swallowed warning into a loud test failure.
        let warnings = resolve_fixed_nested(&mut flat);
        assert!(
            warnings.is_empty(),
            "vendored corpus must resolve with zero warnings; got: {warnings:?}"
        );
        flat.into_iter()
            .map(|s| ((s.package.clone().unwrap_or_default(), s.name.clone()), s))
            .collect()
    })
}

fn schema_for(package: &str, msg_name: &str) -> &'static MessageSchema {
    resolved_corpus()
        .get(&(package.to_string(), msg_name.to_string()))
        .unwrap_or_else(|| panic!("{package}/{msg_name} not found in resolved corpus"))
}

/// Pin one message type: IR-computed hash + fixed size vs generated
/// constants.
macro_rules! pin_schema {
    ($test_name:ident, $package:literal, $msg:literal, $ty:ty) => {
        #[test]
        fn $test_name() {
            let schema = schema_for($package, $msg);
            assert_eq!(
                schema.schema_hash(),
                <$ty as ShmMessage>::SCHEMA_HASH,
                "IR schema_hash() for {}/{} must match the generated \
                 SCHEMA_HASH constant (pipeline drift)",
                $package,
                $msg,
            );
            assert_eq!(
                schema.wire_fixed_size(),
                <$ty as ShmMessage>::WIRE_FIXED_SIZE,
                "IR wire_fixed_size() for {}/{} must match \
                 size_of::<FixedSection>() (rustc #[repr(C)] oracle)",
                $package,
                $msg,
            );
        }
    };
}

// Variable schema, empty fixed section (string-only).
pin_schema!(
    pin_std_msgs_string,
    "std_msgs",
    "String",
    native_ros2_messages::std_msgs::String
);

// Variable schema with a mixed fixed section (u32s + u8 + variable data).
pin_schema!(
    pin_sensor_msgs_image,
    "sensor_msgs",
    "Image",
    native_ros2_messages::sensor_msgs::Image
);

// Fixed-only schema (3 × f64).
pin_schema!(
    pin_geometry_msgs_vector3,
    "geometry_msgs",
    "Vector3",
    native_ros2_messages::geometry_msgs::Vector3
);

// Variable schema (Header → string frame_id) embedding RECURSIVELY-FIXED
// nested fields (Quaternion + Vector3 ×2, inlined as <Target>Shm) plus
// FixedArray covariances (float64[9] ×3). This is the recipe-3 oracle that
// exercises BOTH the FixedArray layout arithmetic AND the inlined
// fixed-nested size/alignment against rustc — it only passes if the corpus
// was resolved first (see module docs).
pin_schema!(
    pin_sensor_msgs_imu,
    "sensor_msgs",
    "Imu",
    native_ros2_messages::sensor_msgs::Imu
);

// Variable schema with i8[] data + nested layout.
pin_schema!(
    pin_std_msgs_int8_multi_array,
    "std_msgs",
    "Int8MultiArray",
    native_ros2_messages::std_msgs::Int8MultiArray
);

// ─────────── Hashes MEASURED off a real rmw publisher ───────────

/// Schema hashes read off the WIRE from a live `rmw_cerulion` publisher,
/// captured 2026-07-27 in a ROS 2 Jazzy container
/// (`ros:jazzy@sha256:31daab66`, aarch64).
///
/// Every other assertion in this file is a SELF-check, as the module docs
/// say plainly: build.rs and the pin tests both call the same
/// `MessageSchema::schema_hash()` recipe, so they catch pipeline drift and
/// nothing else. These seven numbers are different in kind. They were not
/// derived from the vendored corpus at all: stock `rclpy` created a publisher for
/// each type under `RMW_IMPLEMENTATION=rmw_cerulion`, so the schema reached
/// the rmw through the same rosidl introspection typesupport a real robot
/// uses, and a native subscriber read `WireHeader::schema_hash` off the
/// first frame.
///
/// So this is the one place in the repo where a vendored schema's hash is
/// checked against an EXTERNAL, measured truth — "does a stock ROS 2 robot
/// agree with this corpus about what this message is?" — rather than against the
/// same recipe run twice.
///
/// The first five are types whose upstream `.msg` text writes a nested ref
/// BARE. A bare ref hashes differently and matches NOTHING on the wire (an
/// rmw publisher of any of them hits `UnknownSchemaHash` and the topic
/// renders nothing, silently), so the corpus qualifies them. The last
/// two have no bare refs upstream and are the anti-tautology
/// CONTROLS: without them "everything agrees" would be equally
/// consistent with a harness that measured the corpus's own hash back.
///
/// A FAILURE HERE IS A WIRE-COMPATIBILITY BREAK, not a stale constant. Do
/// not re-bless it from the actual value — that is exactly the reflex this
/// pin exists to interrupt.
///
/// SCOPE: five of the 48 types the corpus qualifies were measured, and
/// `control_msgs` was captured against Jazzy 5.9.0 rather than the Lyrical
/// pin (`DynamicJointState` agrees across both eras; that does not
/// generalise to the package). The other 43 are covered by inference from
/// the same three structural shapes, not by measurement.
///
/// That understates the coverage, though, because only the hash-RECIPE
/// agreement is sampled at five. The other half of the question — whether
/// each of the 80 refs is qualified to the RIGHT package — is checked for
/// all 48, by two mechanisms neither of which is this test:
///
/// * If the named package does not define the type, `build.rs` PANICS at
///   nested-schema resolution ("every reference in the vendored .msg set
///   must resolve cleanly"). Only `Pose2D` is defined in two packages, so
///   in this corpus essentially every possible mis-qualification lands
///   here and never reaches a test at all.
/// * If it does define one (the `Pose2D` case), `upstream_drift_test.rs`
///   catches it: `normalize_type` renders an EXPLICIT ref verbatim while an
///   upstream bare ref renders from the parent package, so the field line
///   diverges from the recorded upstream signature. Retargeting
///   `vision_msgs/Pose2D center` to `geometry_msgs/Pose2D`
///   fails it with both sides named.
///
/// The upstream manifest matching the qualified corpus is therefore evidence that
/// every one of the 80 targets is the package upstream implies.
const LIVE_RMW_MEASURED_HASHES: &[(&str, &str, u64)] = &[
    // (package, Name, hash observed on the wire)
    (
        "vision_msgs",
        "ObjectHypothesisWithPose",
        0x350989a02ff4d84d,
    ),
    ("vision_msgs", "Detection2DArray", 0x3f876344bbf2108b),
    ("control_msgs", "DynamicJointState", 0x53a44d7b015b7f75),
    ("grid_map_msgs", "GridMap", 0x24807b3a0e858581),
    ("object_recognition_msgs", "Table", 0xe2ff281b689fb0f8),
    // CONTROLS — no bare refs; unchanged by the qualification.
    ("geometry_msgs", "PoseStamped", 0x7bae8da2598b0993),
    ("sensor_msgs", "Imu", 0xa99349e4f040ba93),
];

#[test]
fn vendored_hashes_match_the_hashes_a_live_rmw_publisher_puts_on_the_wire() {
    let mut mismatches: Vec<String> = Vec::new();
    for (package, msg, measured) in LIVE_RMW_MEASURED_HASHES {
        let ours = schema_for(package, msg).schema_hash();
        if ours != *measured {
            mismatches.push(format!(
                "  {package}/{msg}: ours 0x{ours:016x}, measured on the wire 0x{measured:016x}"
            ));
        }
    }
    assert!(
        mismatches.is_empty(),
        "{} vendored schema hash(es) no longer match what a live \
         `rmw_cerulion` publisher was MEASURED to put on the wire.\n\n{}\n\n\
         A frame from a stock ROS 2 robot carrying one of these types will hit \
         `UnknownSchemaHash` in `walk_by_hash` — refused at the hash gate before framing is \
         consulted — so the topic renders nothing and the user is told nothing.\n\n\
         DO NOT re-bless these constants from the actual values. They were captured off the \
         wire, not computed here. Either the `.msg` drifted \
         from upstream (fix the corpus — `upstream_drift_test.rs` should also be failing), or \
         a bare nested ref crept back in (`the_vendored_corpus_declares_no_bare_nested_refs`), \
         or `HASH_RECIPE` changed — which is wire-affecting for EVERY type and needs its own \
         capture and paired rollout.",
        mismatches.len(),
        mismatches.join("\n"),
    );
}
