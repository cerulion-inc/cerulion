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
//! REQUIRED: a message with a recursively-fixed nested field (e.g.
//! `sensor_msgs/Imu` embedding `geometry_msgs/Quaternion` / `Vector3`) only
//! inlines those targets into its fixed section AFTER resolution, and every
//! nested reference only carries its target's hash after resolution.
//! Parsing `Imu.msg` alone would leave them classified variable,
//! under-counting `wire_fixed_size()` and dropping the nested layout from
//! `schema_hash()`, diverging from the generated (resolved) constants.
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

use cerulion_core::codegen::{parse_rosmsg, resolve_fixed_nested, FieldType, MessageSchema};
use cerulion_core::message::ShmMessage;
use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;
use std::sync::OnceLock;

/// Parse the entire vendored `.msg` corpus, in the order `build.rs` reads it.
fn parsed_corpus() -> Vec<MessageSchema> {
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
    flat
}

/// Resolve `flat` as `build.rs` does and return a `(package, name) ->
/// resolved schema` map.
fn resolve_corpus(mut flat: Vec<MessageSchema>) -> BTreeMap<(String, String), MessageSchema> {
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
}

/// Parse + resolve the entire vendored `.msg` corpus ONCE, exactly as
/// `build.rs` does.
fn resolved_corpus() -> &'static BTreeMap<(String, String), MessageSchema> {
    static CORPUS: OnceLock<BTreeMap<(String, String), MessageSchema>> = OnceLock::new();
    CORPUS.get_or_init(|| resolve_corpus(parsed_corpus()))
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

/// Schema hashes read off the WIRE from a live `rmw_cerulion` publisher
/// under hash recipe 4, captured 2026-11-27 by the Jazzy row of the rmw
/// distro lane's schema hash capture (`rclpy_xproc_test`,
/// `ros:jazzy-ros-base`, x86_64).
///
/// Every other assertion in this file is a SELF-check, as the module docs
/// say plainly: build.rs and the pin tests both call the same
/// `MessageSchema::schema_hash()` recipe, so they catch pipeline drift and
/// nothing else. These eight numbers are different in kind. They were not
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
/// `geometry_msgs/Pose` holds fixed nested fields only, so no change to how
/// a variable or array reference is hashed can move it: the lane measured
/// the same value under recipe 3 and recipe 4.
///
/// The first five are types whose upstream `.msg` text writes a nested ref
/// BARE. A bare ref hashes differently and matches NOTHING on the wire (an
/// rmw publisher of any of them hits `UnknownSchemaHash` and the topic
/// renders nothing, silently), so the corpus qualifies them.
/// `PoseStamped` and `Imu` have no bare refs upstream and are the
/// anti-tautology CONTROLS: without them "everything agrees" would be
/// equally consistent with a harness that measured the corpus's own hash
/// back.
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
        0x8fe08825d5e5479d,
    ),
    ("vision_msgs", "Detection2DArray", 0x4ffee1ff26318ecb),
    ("control_msgs", "DynamicJointState", 0x9e82b0d388b9513e),
    ("grid_map_msgs", "GridMap", 0xb79957a837c606aa),
    ("object_recognition_msgs", "Table", 0x597f0717e0c69300),
    // CONTROLS: no bare refs, unchanged by the qualification.
    ("geometry_msgs", "PoseStamped", 0x3fc2d60fd8310b34),
    ("sensor_msgs", "Imu", 0x9601801853cd9e4a),
    // Fixed-only control: the same value under recipe 3 and recipe 4.
    ("geometry_msgs", "Pose", POSE_WIRE_HASH),
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

/// `geometry_msgs/Pose` as a stock ROS 2 Jazzy `rclpy` publisher puts it on
/// the wire (read by the Jazzy lane's schema hash capture). `Pose` holds
/// only fixed fields, so no change to how nested references are hashed can
/// move it.
const POSE_WIRE_HASH: u64 = 0x8db061f3f9baff7d;

/// Every nested reference of the resolved corpus, at any array depth.
fn nested_refs(ft: &FieldType, out: &mut Vec<FieldType>) {
    match ft {
        FieldType::FixedArray { element_type, .. } | FieldType::DynamicArray { element_type } => {
            nested_refs(element_type, out)
        }
        FieldType::Nested { .. } => out.push(ft.clone()),
        _ => {}
    }
}

#[test]
fn every_corpus_reference_carries_its_target_hash_in_exactly_one_stamp() {
    let corpus = resolved_corpus();
    let mut problems: Vec<String> = Vec::new();
    let mut unlined = 0usize;
    for ((package, name), schema) in corpus {
        let mut refs = Vec::new();
        for field in &schema.fields {
            nested_refs(&field.field_type, &mut refs);
        }
        for r in refs {
            let FieldType::Nested {
                schema_name,
                package: Some(target_package),
                fixed,
                target_hash,
            } = &r
            else {
                problems.push(format!("{package}/{name}: unqualified reference {r:?}"));
                continue;
            };
            let target = schema_for(target_package, schema_name).schema_hash();
            let stamp = match (fixed, target_hash) {
                (Some(info), None) => info.target_hash,
                (None, Some(hash)) => {
                    unlined += 1;
                    *hash
                }
                _ => {
                    problems.push(format!(
                        "{package}/{name} -> {target_package}/{schema_name}: \
                         fixed {fixed:?}, target_hash {target_hash:?}"
                    ));
                    continue;
                }
            };
            if stamp != target {
                problems.push(format!(
                    "{package}/{name} -> {target_package}/{schema_name}: stamp 0x{stamp:016x}, \
                     target hash 0x{target:016x}"
                ));
            }
        }
    }
    assert!(problems.is_empty(), "{}", problems.join("\n"));
    assert!(unlined > 0, "the corpus has variable references");

    let pose_array = schema_for("geometry_msgs", "PoseArray");
    let FieldType::DynamicArray { element_type } = &pose_array.fields[1].field_type else {
        panic!("geometry_msgs/PoseArray.poses must be a dynamic array");
    };
    assert!(matches!(
        element_type.as_ref(),
        FieldType::Nested {
            fixed: None,
            target_hash: Some(POSE_WIRE_HASH),
            ..
        }
    ));
}

// ─────────── The previous recipe, against its own wire captures ───────────

/// Schema hashes read off the wire under hash recipe 3, before recipe 4
/// folded the target hash of every variable and array reference. The first
/// seven are the 2026-07-27 capture `LIVE_RMW_MEASURED_HASHES` held then
/// (stock `rclpy` publishers under `RMW_IMPLEMENTATION=rmw_cerulion` in a
/// ROS 2 Jazzy container, `ros:jazzy@sha256:31daab66`, aarch64); the
/// Jazzy row of the rmw distro lane measured the same seven values again on
/// 2026-11-27, the last run before recipe 4. `sensor_msgs/Image` is the hash a recorder learned off the wire from the
/// latency benchmark's ping node on 2026-09-17 (kept in the committed run
/// logs under `docs/benchmarks/results/`).
///
/// Recipe 4 moved every one of them, but they stay true of recipe 3, so
/// they pin [`MessageSchema::schema_hash_recipe3`]: the function that
/// recognizes a recording made before recipe 4. Do not re-bless them.
const RECIPE3_WIRE_HASHES: &[(&str, &str, u64)] = &[
    (
        "vision_msgs",
        "ObjectHypothesisWithPose",
        0x350989a02ff4d84d,
    ),
    ("vision_msgs", "Detection2DArray", 0x3f876344bbf2108b),
    ("control_msgs", "DynamicJointState", 0x53a44d7b015b7f75),
    ("grid_map_msgs", "GridMap", 0x24807b3a0e858581),
    ("object_recognition_msgs", "Table", 0xe2ff281b689fb0f8),
    ("geometry_msgs", "PoseStamped", 0x7bae8da2598b0993),
    ("sensor_msgs", "Imu", 0xa99349e4f040ba93),
    ("sensor_msgs", "Image", 0xbd08dafceccfa89e),
];

#[test]
fn the_previous_recipe_reproduces_every_hash_measured_on_the_wire_under_it() {
    let mismatches: Vec<String> = RECIPE3_WIRE_HASHES
        .iter()
        .filter_map(|(package, msg, measured)| {
            let ours = schema_for(package, msg).schema_hash_recipe3();
            (ours != *measured).then(|| {
                format!("  {package}/{msg}: recipe 3 0x{ours:016x}, measured 0x{measured:016x}")
            })
        })
        .collect();
    assert!(mismatches.is_empty(), "{}", mismatches.join("\n"));
}

#[test]
fn every_type_measured_under_recipe_3_has_a_new_hash_under_recipe_4() {
    for (package, msg, measured) in RECIPE3_WIRE_HASHES {
        assert_ne!(
            schema_for(package, msg).schema_hash(),
            *measured,
            "{package}/{msg} carries a variable or array reference, so recipe 4 moves its hash"
        );
    }
}

// ─────────── A nested layout change reaches every embedder ───────────

/// The qualified name of every schema in `corpus` that references `target`
/// at any depth (any array depth, through any chain of references), read
/// off the parsed `.msg` text with a plain name walk: no resolver, no hash.
fn schemas_reaching(corpus: &[MessageSchema], target: &str) -> BTreeSet<String> {
    fn referenced(ft: &FieldType) -> Option<String> {
        match ft {
            FieldType::FixedArray { element_type, .. }
            | FieldType::DynamicArray { element_type } => referenced(element_type),
            FieldType::Nested {
                schema_name,
                package: Some(package),
                ..
            } => Some(format!("{package}/{schema_name}")),
            _ => None,
        }
    }
    let edges: BTreeMap<String, Vec<String>> = corpus
        .iter()
        .map(|s| {
            let targets = s
                .fields
                .iter()
                .filter_map(|f| referenced(&f.field_type))
                .collect();
            (s.qualified_name(), targets)
        })
        .collect();
    let mut reaching = BTreeSet::new();
    let mut changed = true;
    while changed {
        changed = false;
        for (name, targets) in &edges {
            if !reaching.contains(name)
                && targets.iter().any(|t| t == target || reaching.contains(t))
            {
                reaching.insert(name.clone());
                changed = true;
            }
        }
    }
    reaching
}

/// `(recipe 4, recipe 3)` hash of every schema, after resolving the corpus
/// with `mutate` applied to the schema named `qualified`.
fn hashes_with(
    qualified: &str,
    mutate: impl Fn(&mut MessageSchema),
) -> BTreeMap<String, (u64, u64)> {
    let mut flat = parsed_corpus();
    let target = flat
        .iter_mut()
        .find(|s| s.qualified_name() == qualified)
        .unwrap_or_else(|| panic!("{qualified} not in the corpus"));
    mutate(target);
    resolve_corpus(flat)
        .into_values()
        .map(|s| {
            (
                s.qualified_name(),
                (s.schema_hash(), s.schema_hash_recipe3()),
            )
        })
        .collect()
}

/// Swap a schema's first two fields: same names, types and fixed-section
/// size, a different wire layout.
fn swap_first_two_fields(s: &mut MessageSchema) {
    s.fields.swap(0, 1);
}

/// Assert that the `mutated` hashes differ from `before` for `target` and
/// every schema that reaches it, and equal it for every other schema.
fn assert_change_reaches_every_embedder(
    target: &str,
    before: &BTreeMap<String, (u64, u64)>,
    mutated: &BTreeMap<String, (u64, u64)>,
) -> BTreeSet<String> {
    let reaching = schemas_reaching(&parsed_corpus(), target);
    let mut problems = Vec::new();
    for (name, (hash, _)) in before {
        let moved = mutated[name].0 != *hash;
        let should_move = name == target || reaching.contains(name);
        if moved != should_move {
            problems.push(format!(
                "  {name}: moved {moved}, reaches {target} {should_move}"
            ));
        }
    }
    assert!(problems.is_empty(), "{}", problems.join("\n"));
    reaching
}

fn unmutated_hashes() -> BTreeMap<String, (u64, u64)> {
    resolved_corpus()
        .values()
        .map(|s| {
            (
                s.qualified_name(),
                (s.schema_hash(), s.schema_hash_recipe3()),
            )
        })
        .collect()
}

#[test]
fn a_header_layout_change_moves_the_hash_of_every_type_that_carries_a_header() {
    let before = unmutated_hashes();
    let mutated = hashes_with("std_msgs/Header", swap_first_two_fields);
    let reaching = assert_change_reaches_every_embedder("std_msgs/Header", &before, &mutated);

    // A variable target, a dynamic array of a type that holds one, and a
    // dynamic array two references away.
    for bearer in [
        "geometry_msgs/PoseStamped",
        "sensor_msgs/Image",
        "nav_msgs/Path",
        "tf2_msgs/TFMessage",
    ] {
        assert!(reaching.contains(bearer), "{bearer} carries a Header");
        // Recipe 3 folds the name of a variable target only: it does not
        // see this change at all.
        assert_eq!(
            mutated[bearer].1, before[bearer].1,
            "{bearer} under recipe 3"
        );
    }
    assert!(reaching.len() > 100, "{} Header bearers", reaching.len());
}

#[test]
fn a_time_layout_change_reaches_through_the_variable_header() {
    let before = unmutated_hashes();
    let mutated = hashes_with("builtin_interfaces/Time", swap_first_two_fields);
    let reaching =
        assert_change_reaches_every_embedder("builtin_interfaces/Time", &before, &mutated);
    // Time is inlined into Header, so recipe 3 saw this change in Header,
    // and stopped there.
    assert!(reaching.contains("std_msgs/Header"));
    assert_ne!(mutated["std_msgs/Header"].1, before["std_msgs/Header"].1);
    for bearer in ["geometry_msgs/PoseStamped", "nav_msgs/Path"] {
        assert_eq!(
            mutated[bearer].1, before[bearer].1,
            "{bearer} under recipe 3"
        );
    }
}

#[test]
fn a_type_with_no_variable_or_array_reference_keeps_its_recipe_3_hash() {
    let mut kept = 0usize;
    for schema in resolved_corpus().values() {
        // A fixed-only schema inlines every reference; recipe 3 and 4 fold
        // the same stamps.
        if schema
            .fields
            .iter()
            .all(|f| f.field_type.is_definitely_fixed())
        {
            assert_eq!(
                schema.schema_hash(),
                schema.schema_hash_recipe3(),
                "{}",
                schema.qualified_name()
            );
            kept += 1;
        }
    }
    assert!(kept > 50, "{kept} fixed-only schemas");
    for (package, name) in [
        ("geometry_msgs", "Point"),
        ("geometry_msgs", "Quaternion"),
        ("geometry_msgs", "Vector3"),
        ("builtin_interfaces", "Time"),
        ("std_msgs", "String"),
    ] {
        let schema = schema_for(package, name);
        assert_eq!(
            schema.schema_hash(),
            schema.schema_hash_recipe3(),
            "{package}/{name}"
        );
    }
    assert_eq!(
        schema_for("geometry_msgs", "Pose").schema_hash(),
        POSE_WIRE_HASH
    );
}

#[test]
fn resolving_the_corpus_twice_gives_identical_hashes() {
    let first = resolve_corpus(parsed_corpus());
    let mut reversed = parsed_corpus();
    reversed.reverse();
    let second = resolve_corpus(reversed);
    for (key, schema) in &first {
        assert_eq!(schema.schema_hash(), second[key].schema_hash(), "{key:?}");
    }
}
