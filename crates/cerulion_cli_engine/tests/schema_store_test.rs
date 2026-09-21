// SPDX-License-Identifier: AGPL-3.0-only
//! Oracle-vector tests for the workspace `.msg` schema store
//! reader (`schema_store::SchemaStore`) and its surfacing through
//! `schema list` / `schema info`.
//!
//! Hermetic — tempdir workspaces with hand-written `.msg` fixtures; NO DDS,
//! NO transport; parallel-safe. Oracles are the hand-written `.msg` text
//! (field names + canonical type strings) and hand-built precedence
//! expectations — never a self-compare. The IMUState fixture is the REAL Go2
//! `unitree_go/IMUState` shape (harvested 2026-07-09).

use std::path::Path;

use cerulion_cli_engine::schema_cmd::{schema_info_unified, schema_list, SchemaSource};
use cerulion_cli_engine::schema_store::SchemaStore;
use cerulion_core::codegen::parse_rosmsg;

/// The REAL Go2 `unitree_go/IMUState` message (float32[4] quaternion /
/// float32[3] gyroscope / float32[3] accelerometer / float32[3] rpy /
/// int8 temperature).
const IMU_STATE_MSG: &str = "float32[4] quaternion\nfloat32[3] gyroscope\n\
     float32[3] accelerometer\nfloat32[3] rpy\nint8 temperature\n";

/// Write a `.msg` into `<schemas_dir>/<pkg>/msg/<Type>.msg` (the ament mirror).
fn write_msg(schemas_dir: &Path, pkg: &str, type_name: &str, text: &str) {
    let dir = schemas_dir.join(pkg).join("msg");
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join(format!("{type_name}.msg")), text).unwrap();
}

/// The `(name, canonical_type)` shape of a store entry's fields — the oracle
/// target (compared against the hand-written `.msg` text below).
fn field_shape(store: &SchemaStore, qualified: &str) -> Vec<(String, String)> {
    store
        .get(qualified)
        .unwrap_or_else(|| panic!("store missing {qualified}"))
        .schema
        .fields
        .iter()
        .map(|f| (f.name.clone(), f.field_type.canonical_str()))
        .collect()
}

// ───────────────────────────── (a) store reader ─────────────────────────────

/// A Vector3-like (all-fixed), a nested type, a variable-length-array type,
/// and the real IMUState parse to the exact hand-written field shapes — and
/// the package comes FROM THE PATH (`.msg` text carries no package).
#[test]
fn store_reader_parses_expected_field_shapes() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas = tmp.path().join("schemas");
    write_msg(
        &schemas,
        "demo_msgs",
        "Vec3",
        "float64 x\nfloat64 y\nfloat64 z\n",
    );
    write_msg(
        &schemas,
        "demo_msgs",
        "Stamped",
        "demo_msgs/Vec3 position\nint32 seq\n",
    );
    write_msg(
        &schemas,
        "demo_msgs",
        "Cloud",
        "uint8[] data\nstring label\n",
    );
    write_msg(&schemas, "unitree_go", "IMUState", IMU_STATE_MSG);

    let store = SchemaStore::load(&schemas);
    assert_eq!(store.len(), 4);

    // Vector3-like — three f64s.
    assert_eq!(
        field_shape(&store, "demo_msgs/Vec3"),
        vec![
            ("x".into(), "float64".into()),
            ("y".into(), "float64".into()),
            ("z".into(), "float64".into()),
        ]
    );
    // Nested (qualified reference) + a primitive.
    assert_eq!(
        field_shape(&store, "demo_msgs/Stamped"),
        vec![
            ("position".into(), "demo_msgs/Vec3".into()),
            ("seq".into(), "int32".into()),
        ]
    );
    // Variable-length array (uint8[]) + a string — both variable.
    assert_eq!(
        field_shape(&store, "demo_msgs/Cloud"),
        vec![
            ("data".into(), "uint8[]".into()),
            ("label".into(), "string".into()),
        ]
    );
    // The real Go2 IMUState.
    assert_eq!(
        field_shape(&store, "unitree_go/IMUState"),
        vec![
            ("quaternion".into(), "float32[4]".into()),
            ("gyroscope".into(), "float32[3]".into()),
            ("accelerometer".into(), "float32[3]".into()),
            ("rpy".into(), "float32[3]".into()),
            ("temperature".into(), "int8".into()),
        ]
    );

    // Package comes from the PATH; provenance path is the ament mirror.
    let imu = store.get("unitree_go/IMUState").unwrap();
    assert_eq!(imu.schema.qualified_name(), "unitree_go/IMUState");
    assert_eq!(imu.relative_path, "unitree_go/msg/IMUState.msg");
    assert_eq!(imu.schema.package.as_deref(), Some("unitree_go"));
}

/// A syntactically bad `.msg` warns LOUDLY (naming the file) and skips — the
/// rest of the store still loads (never a hard abort, never a silent skip).
#[tracing_test::traced_test]
#[test]
fn store_reader_bad_msg_warns_and_rest_still_load() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas = tmp.path().join("schemas");
    write_msg(&schemas, "demo_msgs", "Good", "float64 x\n");
    // A single-token field line has no field name — the parser's InvalidLine
    // hard error ("expected 'type name'"). NOTE: an unknown capitalized type
    // like `String oops` is NOT a parse error (it parses as a nested-type
    // reference; resolution happens later), so it cannot serve as the bad
    // fixture here.
    write_msg(&schemas, "demo_msgs", "Bad", "float64\n");

    let store = SchemaStore::load(&schemas);
    assert!(
        store.resolves("demo_msgs/Good"),
        "the good file still loaded"
    );
    assert!(!store.resolves("demo_msgs/Bad"), "the bad file was skipped");
    assert_eq!(store.len(), 1);

    assert!(
        logs_contain("failed to parse .msg"),
        "the parse failure must be loud"
    );
    assert!(
        logs_contain("Bad.msg"),
        "the warn must name the offending file"
    );
}

/// A missing store dir yields an empty store with NO loud warn (only a debug
/// breadcrumb); a `schemas/` dir holding only flat `*.yaml` files (no package
/// subdirs) is also an empty store — the two layouts never collide.
#[tracing_test::traced_test]
#[test]
fn store_reader_missing_or_yaml_only_is_empty_and_quiet() {
    let tmp = tempfile::tempdir().unwrap();

    // No `schemas/` directory at all.
    let store = SchemaStore::load(&tmp.path().join("schemas"));
    assert!(store.is_empty());
    assert_eq!(store.len(), 0);
    assert!(
        !logs_contain("skipped") && !logs_contain("failed to parse"),
        "a missing store must not emit a loud warn"
    );

    // A `schemas/` with only a flat YAML file — the store ignores it (the
    // YAML lives directly under schemas/, the store in per-package subdirs).
    let schemas = tmp.path().join("schemas");
    std::fs::create_dir_all(&schemas).unwrap();
    std::fs::write(schemas.join("flat.yaml"), "schemas: {}\n").unwrap();
    let store2 = SchemaStore::load(&schemas);
    assert!(store2.is_empty(), "flat *.yaml files are not store entries");
}

/// The store enumeration is byte-deterministic across loads (Principle #7).
#[test]
fn store_reader_is_deterministic() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas = tmp.path().join("schemas");
    write_msg(&schemas, "b_pkg", "Two", "int32 a\n");
    write_msg(&schemas, "a_pkg", "One", "int32 a\n");
    write_msg(&schemas, "a_pkg", "Zeta", "int32 a\n");

    let names =
        |store: &SchemaStore| -> Vec<String> { store.iter().map(|(q, _)| q.to_string()).collect() };
    let first = names(&SchemaStore::load(&schemas));
    let second = names(&SchemaStore::load(&schemas));
    assert_eq!(first, second);
    // Sorted, package-grouped enumeration.
    assert_eq!(first, vec!["a_pkg/One", "a_pkg/Zeta", "b_pkg/Two"]);
}

// ──────────────────────── (c) schema list / info variant ────────────────────

/// `schema list` shows store entries grouped by package and marked
/// `(msg store)`, with the ament-mirror provenance path.
#[test]
fn schema_list_shows_store_grouped_by_package() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas = tmp.path().join("schemas");
    write_msg(
        &schemas,
        "demo_msgs",
        "Vec3",
        "float64 x\nfloat64 y\nfloat64 z\n",
    );
    write_msg(&schemas, "unitree_go", "IMUState", IMU_STATE_MSG);

    let listing = schema_list(&schemas);
    let names: Vec<&str> = listing.store.iter().map(|e| e.name.as_str()).collect();
    assert_eq!(names, vec!["demo_msgs/Vec3", "unitree_go/IMUState"]);
    let pkgs: Vec<&str> = listing.store.iter().map(|e| e.package.as_str()).collect();
    assert_eq!(pkgs, vec!["demo_msgs", "unitree_go"]);
    assert_eq!(listing.store[0].relative_path, "demo_msgs/msg/Vec3.msg");

    let rendered = format!("{listing}");
    assert!(
        rendered.contains("Schemas from the .msg store (schemas/<pkg>/msg): 2 in 2 packages"),
        "{rendered}"
    );
    assert!(rendered.contains("demo_msgs (msg store):"), "{rendered}");
    assert!(rendered.contains("unitree_go (msg store):"), "{rendered}");
    assert!(rendered.contains("(demo_msgs/msg/Vec3.msg)"), "{rendered}");
    // Two independent runs render byte-identically (Principle #7).
    assert_eq!(rendered, format!("{}", schema_list(&schemas)));
}

/// `schema info` resolves a store type by qualified `pkg/Type`, `pkg::Type`,
/// and a unique bare stem; a nested-free all-fixed type reports a RESOLVED
/// layout whose hash equals an independent single-file parse.
#[test]
fn schema_info_resolves_store_by_qualified_and_bare() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas = tmp.path().join("schemas");
    write_msg(&schemas, "unitree_go", "IMUState", IMU_STATE_MSG);

    let by_q = schema_info_unified(&schemas, "unitree_go/IMUState").unwrap();
    assert_eq!(by_q.source, SchemaSource::MsgStore);
    let entry = &by_q.result.entries[0];
    assert_eq!(entry.name, "unitree_go/IMUState");
    assert_eq!(entry.field_count, 5);
    assert!(
        entry.wire_fixed_size_resolved,
        "all-fixed → resolved layout"
    );
    assert!(
        entry.fields.iter().all(|f| !f.is_variable),
        "IMUState has no variable fields"
    );
    // Oracle: a nested-free type's resolved hash == its single-file hash
    // (independent raw-text parse — not a store self-compare).
    let independent = parse_rosmsg(IMU_STATE_MSG, "IMUState", Some("unitree_go")).unwrap();
    assert_eq!(entry.schema_hash, Some(independent.schema_hash()));

    // Bare unique stem resolves (the convention).
    let by_bare = schema_info_unified(&schemas, "IMUState").unwrap();
    assert_eq!(by_bare.source, SchemaSource::MsgStore);
    assert_eq!(by_bare.result.entries[0].name, "unitree_go/IMUState");

    // `pkg::Type` normalizes to the same store entry.
    let by_colon = schema_info_unified(&schemas, "unitree_go::IMUState").unwrap();
    assert_eq!(by_colon.source, SchemaSource::MsgStore);

    // The rendered source line marks the store, naming the exact file the
    // entry was materialized into (CER — schema-info unified renderer).
    assert!(
        format!("{by_q}").contains("source: msg store (schemas/unitree_go/msg/IMUState.msg)"),
        "{by_q}"
    );
}

/// A nested store type resolves its layout against the FULL store (the
/// qualified nested reference), reporting a resolved fixed size.
#[test]
fn schema_info_store_resolves_nested_over_the_store_set() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas = tmp.path().join("schemas");
    write_msg(
        &schemas,
        "demo_msgs",
        "Vec3",
        "float64 x\nfloat64 y\nfloat64 z\n",
    );
    write_msg(
        &schemas,
        "demo_msgs",
        "Stamped",
        "demo_msgs/Vec3 position\nint32 seq\n",
    );

    let info = schema_info_unified(&schemas, "demo_msgs/Stamped").unwrap();
    assert_eq!(info.source, SchemaSource::MsgStore);
    let entry = &info.result.entries[0];
    // Vec3 (24 B, recursively fixed) inlines → Stamped is fully fixed.
    assert!(
        entry.wire_fixed_size_resolved,
        "nested resolved over the store"
    );
    assert!(
        entry.fields.iter().all(|f| !f.is_variable),
        "position (nested fixed) + seq (i32) are both fixed"
    );
}

/// A bare store name defined by MORE THAN ONE package is AMBIGUOUS — a loud
/// error listing the candidates (the house rule), never a silent pick.
#[test]
fn schema_info_ambiguous_bare_store_name_errors() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas = tmp.path().join("schemas");
    write_msg(&schemas, "pkg_a", "Widget", "int32 a\n");
    write_msg(&schemas, "pkg_b", "Widget", "int32 b\n");

    let err = schema_info_unified(&schemas, "Widget").unwrap_err();
    let msg = format!("{err}");
    assert!(msg.contains("ambiguous"), "{msg}");
    assert!(
        msg.contains("pkg_a/Widget") && msg.contains("pkg_b/Widget"),
        "the error must list both candidates: {msg}"
    );
}

/// Workspace over built-in, store over built-in — and the workspace/store
/// TWIN refused. `sensor_msgs/Imu` is a built-in; a store copy shadows it
/// under the qualified spelling and a YAML entry `Image` shadows the built-in
/// `sensor_msgs/Image` under the bare one, each with a distinct single field
/// so the winner is observable. The bare spelling `Imu`, which BOTH the YAML
/// entry and the store define, is no precedence question at all:
/// a spelling that names more than one workspace definition is
/// REFUSED naming every source (never "YAML wins").
#[test]
fn schema_info_workspace_and_store_shadow_builtins_but_their_twin_is_refused() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas = tmp.path().join("schemas");
    std::fs::create_dir_all(&schemas).unwrap();
    write_msg(&schemas, "sensor_msgs", "Imu", "int32 store_marker\n");
    std::fs::write(
        schemas.join("imu.yaml"),
        "schemas:\n  Imu:\n    fields:\n      uint8 yaml_marker:\n",
    )
    .unwrap();
    std::fs::write(
        schemas.join("image.yaml"),
        "schemas:\n  Image:\n    fields:\n      uint8 yaml_marker:\n",
    )
    .unwrap();

    // Bare `Image`: a workspace entry shadows the built-in (no store twin).
    let bare = schema_info_unified(&schemas, "Image").unwrap();
    assert_eq!(bare.source, SchemaSource::Workspace);
    assert_eq!(bare.result.entries[0].fields[0].name, "yaml_marker");
    assert_eq!(bare.shadowed_builtin.as_deref(), Some("sensor_msgs/Image"));

    // Bare `Imu`: the YAML entry AND the store define it — refused, both named.
    let twin = schema_info_unified(&schemas, "Imu")
        .expect_err("a YAML/store twin is refused, never resolved by precedence")
        .to_string();
    assert!(
        twin.contains("'Imu' is ambiguous in this workspace")
            && twin.contains("schemas/imu.yaml (entry Imu)")
            && twin.contains("schemas/sensor_msgs/msg/Imu.msg (store)"),
        "{twin}"
    );

    // Qualified `sensor_msgs/Imu`: no YAML entry by that name → the STORE
    // wins over the built-in of the same qualified name.
    let qualified = schema_info_unified(&schemas, "sensor_msgs/Imu").unwrap();
    assert_eq!(qualified.source, SchemaSource::MsgStore);
    assert_eq!(qualified.result.entries[0].fields[0].name, "store_marker");
    assert_eq!(
        qualified.shadowed_builtin.as_deref(),
        Some("sensor_msgs/Imu")
    );

    // Control: with NO workspace overrides, the built-in is reached.
    let clean = tempfile::tempdir().unwrap();
    let builtin = schema_info_unified(&clean.path().join("schemas"), "sensor_msgs/Imu").unwrap();
    assert_eq!(builtin.source, SchemaSource::Builtin);
}

/// `schema list` marks a store entry that shadows a built-in on BOTH sides:
/// the store entry flags `shadows_builtin`, and the built-in row renders the
/// `shadowed by msg store` marker (yaml-shadow takes render precedence, but
/// there is no yaml here).
#[test]
fn schema_list_marks_store_over_builtin_shadow() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas = tmp.path().join("schemas");
    write_msg(&schemas, "sensor_msgs", "Imu", "int32 store_marker\n");

    let listing = schema_list(&schemas);
    let imu = listing
        .store
        .iter()
        .find(|e| e.name == "sensor_msgs/Imu")
        .unwrap();
    assert!(imu.shadows_builtin);

    let group = listing
        .builtin_packages
        .iter()
        .find(|g| g.package == "sensor_msgs")
        .unwrap();
    let bt = group.types.iter().find(|t| t.name == "Imu").unwrap();
    assert_eq!(bt.shadowed_by_store.as_deref(), Some("sensor_msgs/Imu"));
    assert!(bt.shadowed_by.is_none(), "no YAML shadow in this fixture");

    let rendered = format!("{listing}");
    assert!(
        rendered.contains("Imu (shadowed by msg store 'sensor_msgs/Imu')"),
        "{rendered}"
    );
    assert!(rendered.contains("(shadows built-in)"), "{rendered}");
}

/// The unified schema-info renderer: when a
/// `.msg`-store schema SHADOWS a built-in (same qualified name), a nested
/// expansion of that type must render the STORE's field NAMES with the STORE's
/// classification. The field LIST and the fixed/variable CLASS both derive
/// from ONE winner map, so they can never disagree. Were the list to come from
/// a first-wins map (the built-in) while the class came from a last-wins
/// resolver (the store), a divergent shadow would render the WRONG schema's field
/// names, all shown `(fixed)`. The oracle: the store's names
/// appear + the built-in's distinctive fields do NOT.
#[test]
fn store_shadowed_builtin_expands_with_shadow_field_names() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas = tmp.path().join("schemas");
    // A store `sensor_msgs/PointField` that DIVERGES from the built-in (whose
    // fields are name/offset/datatype/count): distinctive names, all-fixed.
    write_msg(
        &schemas,
        "sensor_msgs",
        "PointField",
        "float64 shadowed_marker\nint32 shadow_seq\n",
    );

    // Built-in `sensor_msgs/PointCloud2` has `sensor_msgs/PointField[] fields`,
    // so its render expands PointField inline — resolving to the SHADOW.
    let info = schema_info_unified(&schemas, "sensor_msgs/PointCloud2").unwrap();
    let rendered = format!("{info}");

    // The STORE's field names + correct classification appear inline (depth 2).
    assert!(
        rendered.contains("    shadowed_marker: float64 (fixed)\n"),
        "store shadow's field must expand with the store's names:\n{rendered}"
    );
    assert!(
        rendered.contains("    shadow_seq: int32 (fixed)\n"),
        "store shadow's field must expand:\n{rendered}"
    );
    // The built-in PointField's distinctive fields must NOT leak (a first-wins
    // field list would leak them).
    assert!(
        !rendered.contains("offset: uint32"),
        "shadowed-out built-in field leaked into the tree:\n{rendered}"
    );
    assert!(
        !rendered.contains("datatype: uint8"),
        "shadowed-out built-in field leaked into the tree:\n{rendered}"
    );

    // The loud shadow FACT still fires where it does today: a DIRECT lookup of
    // the shadowed name resolves to the store + carries `shadowed_builtin`
    // (which main.rs turns into the stderr shadow warning).
    let direct = schema_info_unified(&schemas, "sensor_msgs/PointField").unwrap();
    assert_eq!(direct.source, SchemaSource::MsgStore);
    assert_eq!(
        direct.shadowed_builtin.as_deref(),
        Some("sensor_msgs/PointField")
    );
}

// ──────────────────── msg-store parity: the shared fold seam ────────────────────

/// `SchemaStore::schemas()` — the ONE enumeration seam the combined-set
/// folds (hash map, wire sizes, replay registry) consume — clones every
/// entry's IR in sorted qualified-name order, package attached.
#[test]
fn store_schemas_seam_enumerates_sorted_packaged_ir() {
    let tmp = tempfile::tempdir().unwrap();
    // Written in reverse-sorted order so sortedness cannot be a read_dir
    // accident.
    write_msg(tmp.path(), "zeta", "B", "uint32 a\n");
    write_msg(tmp.path(), "alpha", "A", "float64 x\n");
    let store = SchemaStore::load(tmp.path());
    let schemas = store.schemas();
    let names: Vec<String> = schemas.iter().map(|s| s.qualified_name()).collect();
    assert_eq!(names, vec!["alpha/A".to_string(), "zeta/B".to_string()]);
    assert_eq!(schemas[0].package.as_deref(), Some("alpha"));
}
