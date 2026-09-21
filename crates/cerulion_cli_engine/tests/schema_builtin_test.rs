// SPDX-License-Identifier: AGPL-3.0-only
//! 1-3: the embedded built-in message registry
//! (`native_ros2_messages::BUILTIN_MSGS`), the canonical resolver seam
//! (`schema_cmd::builtin_layout_resolver`), the unified `schema info`
//! lookup (`schema_cmd::schema_info_unified` — workspace YAML first,
//! then the built-in registry), and the grouped `schema list`
//! (`schema_cmd::schema_list`).
//!
//! Oracle-vector tests — NEVER self-compares. The oracle is the
//! GENERATED `ShmMessage` constants that `native_ros2_messages` compiled
//! from the same vendored `.msg` tree the registry freezes: a resolved
//! `WireLayout` must equal `<Type as ShmMessage>::{SCHEMA_HASH,
//! WIRE_FIXED_SIZE, VARIABLE_FIELD_COUNT}`. This is the CLI-side twin of
//! `native_ros2_messages/tests/layout_equivalence_test.rs`, which pins
//! the same equivalence against the `msg/` tree; here the input is the
//! embedded registry instead, proving CLI introspection sees the exact
//! layouts the linked-in types were built with.
//!
//! Pure parse/compare — no iceoryx2, no shared memory, parallel-safe.

use cerulion_cli_engine::error::CliError;
use cerulion_cli_engine::schema_cmd::{
    builtin_layout_resolver, schema_info_unified, schema_list, SchemaSource,
};
use cerulion_core::codegen::{parse_rosmsg, FieldDef, FieldType, MessageSchema};
use cerulion_core::message::ShmMessage;
use native_ros2_messages::geometry_msgs::{Pose, Vector3};
use native_ros2_messages::sensor_msgs::Image;
use native_ros2_messages::BUILTIN_MSGS;

/// The resolver built over the whole registry reproduces the generated
/// constants for a representative all-fixed schema (`Vector3`), a
/// variable-field schema (`Image`), and a nested-fixed schema (`Pose`).
///
/// The nested case is the point of building the resolver over the FULL
/// set before any lookup: `Pose` only lays out correctly because `Point`
/// and `Quaternion` resolve against their siblings in the same set.
#[test]
fn builtin_resolver_layouts_match_generated_constants() {
    let (mut resolver, _) = builtin_layout_resolver();

    // Vector3 — all-fixed (3 × f64), zero variable fields.
    let v3 = resolver
        .layout_of("geometry_msgs/Vector3")
        .expect("Vector3 layout");
    assert_eq!(v3.schema_hash, <Vector3 as ShmMessage>::SCHEMA_HASH);
    assert_eq!(v3.fixed_size, <Vector3 as ShmMessage>::WIRE_FIXED_SIZE);
    assert_eq!(
        v3.variable_fields.len(),
        <Vector3 as ShmMessage>::VARIABLE_FIELD_COUNT
    );

    // Image — variable fields (header bytes + encoding + data).
    let img = resolver
        .layout_of("sensor_msgs/Image")
        .expect("Image layout");
    assert_eq!(img.schema_hash, <Image as ShmMessage>::SCHEMA_HASH);
    assert_eq!(img.fixed_size, <Image as ShmMessage>::WIRE_FIXED_SIZE);
    assert_eq!(
        img.variable_fields.len(),
        <Image as ShmMessage>::VARIABLE_FIELD_COUNT
    );

    // Pose — nested-fixed (Point + Quaternion inlined; zero variable).
    let pose = resolver
        .layout_of("geometry_msgs/Pose")
        .expect("Pose layout");
    assert_eq!(pose.schema_hash, <Pose as ShmMessage>::SCHEMA_HASH);
    assert_eq!(pose.fixed_size, <Pose as ShmMessage>::WIRE_FIXED_SIZE);
    assert_eq!(
        pose.variable_fields.len(),
        <Pose as ShmMessage>::VARIABLE_FIELD_COUNT
    );
}

/// The registry entry count equals the canonical vendored-schema count.
///
/// 254 is pinned as the single source of truth by
/// `native_ros2_messages/tests/layout_equivalence_test.rs`
/// (`build_resolver`'s `assert_eq!(schemas.len(), 254, ...)`). It is
/// mirrored here rather than imported (that assert lives in a different
/// crate's test binary, unreachable from here); vendoring a new `.msg`
/// must move BOTH asserts together.
#[test]
fn builtin_msgs_count_matches_canonical_anchor() {
    const CANONICAL_BUILTIN_COUNT: usize = 254;
    assert_eq!(
        BUILTIN_MSGS.len(),
        CANONICAL_BUILTIN_COUNT,
        "BUILTIN_MSGS count drifted from the vendored .msg count pinned by \
         native_ros2_messages/tests/layout_equivalence_test.rs — extend both asserts"
    );
}

/// Registry entries are STRICTLY ascending by `(package, message_name)`.
///
/// Determinism (Principle #7): the build script emits from a `BTreeMap`,
/// so the order is fixed across builds. Strict `<` also proves there are
/// no duplicate `(package, message_name)` entries. `windows(2)` is used
/// rather than `<[_]>::is_sorted_by_key` to stay obviously MSRV-safe
/// (1.88) and explicit about the composite key.
#[test]
fn builtin_msgs_are_strictly_sorted_by_package_then_name() {
    let sorted = BUILTIN_MSGS
        .windows(2)
        .all(|w| (w[0].0, w[0].1) < (w[1].0, w[1].1));
    assert!(
        sorted,
        "BUILTIN_MSGS must be strictly ascending by (package, message_name) — \
         deterministic BTreeMap emission with no duplicate entries"
    );
}

/// Every vendored schema in the registry is resolvable: each entry
/// round-trips through the canonical `parse_rosmsg` without error.
///
/// The "every embedded schema parses" sweep — the frozen text was
/// parseable at build time, and this asserts it stays so through the
/// same parser the seam and codegen use.
#[test]
fn every_builtin_entry_parses() {
    for &(package, name, text) in BUILTIN_MSGS {
        if let Err(e) = parse_rosmsg(text, name, Some(package)) {
            panic!("built-in {package}/{name} failed to parse: {e}");
        }
    }
}

/// Happy path: `sensor_msgs/Image` resolves from the BUILT-IN
/// registry when the workspace schemas dir doesn't exist. Hash and fixed
/// size are asserted against the generated constants; the field list
/// against a hand-written declaration-order oracle from `Image.msg`
/// (header/encoding/data ride the offset table, the rest are fixed —
/// `header` is variable only because the resolver KNOWS `std_msgs/Header`
/// contains a string, which the deleted approximate parser could not).
#[test]
fn unified_builtin_image_matches_generated_constants() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas"); // never created
    let info = schema_info_unified(&schemas_dir, "sensor_msgs/Image").unwrap();

    assert_eq!(info.source, SchemaSource::Builtin);
    assert!(info.shadowed_builtin.is_none());
    assert_eq!(info.result.entries.len(), 1);

    let entry = &info.result.entries[0];
    assert_eq!(entry.name, "sensor_msgs/Image");
    assert_eq!(entry.schema_hash, Some(<Image as ShmMessage>::SCHEMA_HASH));
    assert_eq!(
        entry.wire_fixed_size,
        <Image as ShmMessage>::WIRE_FIXED_SIZE
    );

    let got: Vec<(&str, bool)> = entry
        .fields
        .iter()
        .map(|f| (f.name.as_str(), f.is_variable))
        .collect();
    let expected = vec![
        ("header", true),
        ("height", false),
        ("width", false),
        ("encoding", true),
        ("is_bigendian", false),
        ("step", false),
        ("data", true),
    ];
    assert_eq!(
        got, expected,
        "Image.msg declaration order + variable split"
    );
    let variable_count = entry.fields.iter().filter(|f| f.is_variable).count();
    assert_eq!(variable_count, <Image as ShmMessage>::VARIABLE_FIELD_COUNT);
}

/// The `::` alias resolves to the identical rendered result as the `/`
/// form — one schema, one renderer, two accepted spellings.
#[test]
fn unified_double_colon_alias_renders_identically_to_slash_form() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    let slash = schema_info_unified(&schemas_dir, "sensor_msgs/Image").unwrap();
    let alias = schema_info_unified(&schemas_dir, "sensor_msgs::Image").unwrap();
    let rendered = format!("{}", slash);
    assert_eq!(rendered, format!("{}", alias));
    // Anchor against the generated constant (not a pure cross-compare).
    let hash_line = format!("hash: 0x{:016x}", <Image as ShmMessage>::SCHEMA_HASH);
    assert!(
        rendered.contains(&hash_line),
        "missing {hash_line} in:\n{rendered}"
    );
}

/// Edge: a well-formed qualified name matching no built-in is the
/// `SchemaNotFound` VARIANT (not just any error), carrying the requested
/// name verbatim.
#[test]
fn unified_unknown_qualified_name_is_schema_not_found() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    let err = schema_info_unified(&schemas_dir, "fake_msgs/Nope").unwrap_err();
    match err {
        CliError::SchemaNotFound { name, remedy } => {
            assert_eq!(name, "fake_msgs/Nope");
            // The info-context remedy points at the discovery command.
            assert!(
                remedy.contains("cerulion schema list"),
                "info remedy must name the discovery command: {remedy}"
            );
        }
        other => panic!("expected SchemaNotFound, got: {other:?}"),
    }
}

/// Adversarial: a bare name matching no workspace file is the
/// `Validation` VARIANT naming the expected qualified form (there is no
/// bare-name-to-built-in guessing — built-ins are addressed 'pkg/Type').
#[test]
fn unified_bare_unknown_name_is_validation_error() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    let err = schema_info_unified(&schemas_dir, "JustAName").unwrap_err();
    match err {
        CliError::Validation(msg) => {
            assert!(
                msg.contains("package/Type"),
                "must state the expected qualified form: {msg}"
            );
        }
        other => panic!("expected Validation, got: {other:?}"),
    }
}

/// Determinism: two lookups render byte-identically (BTreeMap-
/// ordered registry + deterministic parse/resolve — Principle #7), and
/// the output carries the generated-constant anchors so the pin is not a
/// pure self-compare.
#[test]
fn unified_rendered_output_is_deterministic() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    let a = format!(
        "{}",
        schema_info_unified(&schemas_dir, "sensor_msgs/Image").unwrap()
    );
    let b = format!(
        "{}",
        schema_info_unified(&schemas_dir, "sensor_msgs/Image").unwrap()
    );
    assert_eq!(a, b, "two renders must be byte-identical");
    assert!(a.contains(&format!(
        "hash: 0x{:016x}",
        <Image as ShmMessage>::SCHEMA_HASH
    )));
    // The trailing newline also pins the ABSENCE of the ` (unresolved)`
    // suffix — built-in entries are always resolved.
    assert!(a.contains(&format!(
        "wire fixed size: {} bytes\n",
        <Image as ShmMessage>::WIRE_FIXED_SIZE
    )));
    assert!(a.contains("source: built-in (ROS 2)"));
    // The unified tree expands the nested `std_msgs/Header` (variable) inline
    // beneath the `header` field — `stamp` (builtin_interfaces/Time, fixed)
    // recurses to its `sec`/`nanosec` leaves; `frame_id` (string) is variable.
    assert!(
        a.contains("  header: std_msgs/Header (variable)\n"),
        "header must render its nested type + class:\n{a}"
    );
    assert!(
        a.contains("    stamp: builtin_interfaces/Time (fixed)\n      sec: int32 (fixed)\n"),
        "Header's nested Time must expand to its int32 `sec` leaf:\n{a}"
    );
}

/// Workspace-wins: a workspace-local schema shadowing a built-in
/// bare name is returned INSTEAD of the built-in, and the collision fact
/// is carried in `shadowed_builtin` (the CLI emits the loud warn; the engine
/// carries the fact).
#[test]
fn workspace_schema_wins_and_reports_shadowed_builtin() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    std::fs::create_dir_all(&schemas_dir).unwrap();
    let content = "schemas:\n  Image:\n    description: \"local override\"\n    fields:\n      \
                   uint32 height:\n";
    std::fs::write(schemas_dir.join("Image.yaml"), content).unwrap();

    let info = schema_info_unified(&schemas_dir, "Image").unwrap();
    assert_eq!(info.source, SchemaSource::Workspace);
    let entry = &info.result.entries[0];
    assert_eq!(entry.name, "Image");
    assert_eq!(
        entry.field_count, 1,
        "the 1-field workspace schema must win, not the 7-field built-in"
    );
    let shadowed = info
        .shadowed_builtin
        .as_deref()
        .expect("the shadow fact must be carried");
    assert!(
        shadowed.contains("sensor_msgs/Image"),
        "must name the shadowed built-in: {shadowed}"
    );
}

/// Negative control for the shadow carrier: a workspace schema colliding
/// with no built-in carries `shadowed_builtin == None` (the fact is not
/// fabricated).
#[test]
fn workspace_schema_without_collision_has_no_shadow() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    std::fs::create_dir_all(&schemas_dir).unwrap();
    let content = "schemas:\n  TotallyCustom:\n    fields:\n      uint32 a:\n";
    std::fs::write(schemas_dir.join("TotallyCustom.yaml"), content).unwrap();

    let info = schema_info_unified(&schemas_dir, "TotallyCustom").unwrap();
    assert_eq!(info.source, SchemaSource::Workspace);
    assert!(info.shadowed_builtin.is_none());
}

/// List (a): a missing workspace schemas dir yields the built-in
/// groups only — totals and package count equal the registry (the 254
/// anchor is pinned by `builtin_msgs_count_matches_canonical_anchor`),
/// no shadow markers, and two renders are byte-identical (Principle #7).
#[test]
fn list_on_missing_workspace_is_builtin_only_and_deterministic() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas"); // never created
    let listing = schema_list(&schemas_dir);

    assert!(listing.workspace.is_empty());
    assert_eq!(listing.builtin_total, BUILTIN_MSGS.len());
    let type_total: usize = listing.builtin_packages.iter().map(|g| g.types.len()).sum();
    assert_eq!(
        type_total,
        BUILTIN_MSGS.len(),
        "grouping must not drop or dup types"
    );

    // Distinct-package oracle straight from the (sorted) registry.
    let mut pkgs: Vec<&str> = BUILTIN_MSGS.iter().map(|&(p, _, _)| p).collect();
    pkgs.dedup();
    assert_eq!(listing.builtin_packages.len(), pkgs.len());

    // No workspace schemas → no shadow markers anywhere.
    assert!(listing
        .builtin_packages
        .iter()
        .all(|g| g.types.iter().all(|t| t.shadowed_by.is_none())));

    assert_eq!(
        format!("{}", listing),
        format!("{}", schema_list(&schemas_dir)),
        "two renders must be byte-identical"
    );
}

/// List (b): a workspace schema appears in the workspace group with
/// its declaring file and its `schema info` hash.
#[test]
fn list_includes_workspace_schema_in_workspace_group() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    std::fs::create_dir_all(&schemas_dir).unwrap();
    let content = "schemas:\n  TotallyCustom:\n    fields:\n      uint32 a:\n";
    std::fs::write(schemas_dir.join("TotallyCustom.yaml"), content).unwrap();

    let listing = schema_list(&schemas_dir);
    assert_eq!(listing.workspace.len(), 1);
    let entry = &listing.workspace[0];
    assert_eq!(entry.name, "TotallyCustom");
    assert_eq!(entry.file, "TotallyCustom.yaml");
    let hash = entry
        .schema_hash
        .expect("a representable workspace entry lists with its hash");
    assert_ne!(hash, 0);

    // No collision with any built-in → no inline shadow markers.
    assert!(listing
        .builtin_packages
        .iter()
        .all(|g| g.types.iter().all(|t| t.shadowed_by.is_none())));

    let rendered = format!("{}", listing);
    assert!(rendered.contains("Workspace schemas (schemas/*.yaml): 1"));
    assert!(rendered.contains(&format!(
        "  TotallyCustom  0x{hash:016x}  (TotallyCustom.yaml)"
    )));
}

/// List (c): a workspace schema shadowing a built-in bare name marks
/// the BUILT-IN entry inline (same collision semantics as
/// `schema_info_unified`'s `shadowed_builtin`), while unrelated types in
/// the same package stay unmarked.
#[test]
fn list_marks_shadowed_builtin_inline() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    std::fs::create_dir_all(&schemas_dir).unwrap();
    let content = "schemas:\n  Image:\n    description: \"local override\"\n    fields:\n      \
                   uint32 height:\n";
    std::fs::write(schemas_dir.join("Image.yaml"), content).unwrap();

    let listing = schema_list(&schemas_dir);
    let sensor = listing
        .builtin_packages
        .iter()
        .find(|g| g.package == "sensor_msgs")
        .expect("sensor_msgs group present");
    let image = sensor
        .types
        .iter()
        .find(|t| t.name == "Image")
        .expect("Image entry present");
    assert_eq!(image.shadowed_by.as_deref(), Some("Image"));
    let imu = sensor
        .types
        .iter()
        .find(|t| t.name == "Imu")
        .expect("Imu entry present");
    assert!(
        imu.shadowed_by.is_none(),
        "unrelated types must stay unmarked"
    );

    let rendered = format!("{}", listing);
    assert!(
        rendered.contains("Image (shadowed by workspace 'Image')"),
        "the built-in entry must carry the inline shadow marker:\n{rendered}"
    );
}

/// List (d): exact-substring rendering pins with HAND-WRITTEN oracle
/// strings (never derived from the renderer): the built-in section
/// header (counts move with the vendored set, in lockstep with the 254
/// anchor), a known package header, and its full type line —
/// builtin_interfaces vendors exactly Duration.msg + Time.msg, sorted
/// ascending by the registry.
#[test]
fn list_rendering_pins_known_package_header_and_type_lines() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    let rendered = format!("{}", schema_list(&schemas_dir));

    assert!(rendered.contains("Workspace schemas (schemas/*.yaml): 0"));
    assert!(
        rendered.contains("Built-in ROS 2 messages: 254 in 22 packages"),
        "section header (counts track the vendored set):\n{rendered}"
    );
    assert!(
        rendered.contains("  builtin_interfaces (2):"),
        "package header oracle:\n{rendered}"
    );
    assert!(
        rendered.contains("    Duration, Time"),
        "type-line oracle:\n{rendered}"
    );
}

/// The two layout columns are asserted on the
/// WORKSPACE source — a fixed field renders `(fixed)`, a variable field
/// `(variable)`, and `wire_fixed_size` equals the hand-derived value
/// (one u32 in the fixed section = 4 bytes; the string rides the offset
/// table and contributes nothing).
#[test]
fn workspace_info_reports_fixed_and_variable_columns() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    std::fs::create_dir_all(&schemas_dir).unwrap();
    let content = "schemas:\n  Telemetry:\n    fields:\n      uint32 seq:\n      string label:\n";
    std::fs::write(schemas_dir.join("Telemetry.yaml"), content).unwrap();

    let info = schema_info_unified(&schemas_dir, "Telemetry").unwrap();
    assert_eq!(info.source, SchemaSource::Workspace);
    let entry = &info.result.entries[0];
    assert_eq!(
        entry.wire_fixed_size, 4,
        "u32 only — the string is offset-table payload"
    );
    let got: Vec<(&str, bool)> = entry
        .fields
        .iter()
        .map(|f| (f.name.as_str(), f.is_variable))
        .collect();
    assert_eq!(got, vec![("seq", false), ("label", true)]);

    let rendered = format!("{}", info);
    assert!(rendered.contains("  seq: uint32 (fixed)"), "{rendered}");
    assert!(
        rendered.contains("  label: string (variable)"),
        "{rendered}"
    );
    // The workspace source now renders `source: workspace (schemas/<file>.yaml)`.
    assert!(
        rendered.contains("source: workspace (schemas/Telemetry.yaml)"),
        "{rendered}"
    );
    // Trailing newline pins the ABSENCE of the ` (unresolved)` suffix —
    // a plain-fixed workspace schema resolves, so it renders unsuffixed.
    assert!(
        rendered.contains("wire fixed size: 4 bytes\n"),
        "{rendered}"
    );
}

/// A workspace schema referencing a BUILT-IN nested
/// type resolves canonically — `geometry_msgs/Point` is recursively
/// fixed (24 B, align 8), so `origin` inlines into the fixed section
/// exactly as codegen would lay it out: origin@0 (24) + kind@24 (1),
/// padded to align 8 → 32 bytes, `origin` NOT variable. The loud
/// workspace nested marker stays. The displayed HASH stays the
/// single-file UNRESOLVED-parse value, pinned against a hand-built IR
/// oracle (nested field contributes only its qualified name; the
/// unresolved `wire_fixed_size` input is kind-only = 1).
///
/// Provenance note: this test therefore pins a pair of
/// INTENTIONALLY different provenance on ONE entry — the RESOLVED wire
/// fixed size (32, the true inlined SHM layout) next to the UNRESOLVED
/// recipe hash (whose own `wire_fixed_size` input was 1). Both are
/// correct; neither may be "fixed" to match the other. See
/// `enrich_entries_with_resolved_layout`'s doc in `schema_cmd.rs`.
#[test]
fn workspace_info_resolves_nested_ref_against_builtin_registry() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    std::fs::create_dir_all(&schemas_dir).unwrap();
    let content =
        "schemas:\n  Marker:\n    fields:\n      geometry_msgs/Point origin:\n      uint8 kind:\n";
    std::fs::write(schemas_dir.join("Marker.yaml"), content).unwrap();

    let info = schema_info_unified(&schemas_dir, "Marker").unwrap();
    assert_eq!(info.source, SchemaSource::Workspace);
    let entry = &info.result.entries[0];

    // Resolved layout columns (hand-derived, matches #[repr(C)]).
    assert_eq!(
        entry.wire_fixed_size, 32,
        "24 (Point) + 1 (u8) padded to align 8"
    );
    assert!(
        !entry.fields[0].is_variable,
        "resolved fixed nested is NOT variable"
    );
    assert!(!entry.fields[1].is_variable);

    // Negative control for the ` (unresolved)` suffix: a RESOLVED size renders
    // WITHOUT ` (unresolved)` (the trailing newline pins the absence).
    assert!(entry.wire_fixed_size_resolved);
    let rendered = format!("{}", info);
    assert!(
        rendered.contains("wire fixed size: 32 bytes\n"),
        "resolved size must render unsuffixed:\n{rendered}"
    );

    // `type_display` is now the CLEAN canonical string (the old loud
    // `(treated as nested schema reference)` marker is superseded by the
    // renderer, which resolves + expands the built-in nested type inline).
    assert_eq!(entry.fields[0].type_display, "geometry_msgs/Point");
    assert!(entry.fields[0].is_nested);

    // The unified tree resolves `geometry_msgs/Point` against the built-in
    // registry and expands its x/y/z leaves indented directly beneath the
    // `origin` field — the workspace-references-a-built-in case.
    assert!(
        rendered.contains(
            "  origin: geometry_msgs/Point (fixed)\n    x: float64 (fixed)\n    \
             y: float64 (fixed)\n    z: float64 (fixed)\n"
        ),
        "workspace nested built-in ref must expand inline:\n{rendered}"
    );

    // Hash oracle: the single-file UNRESOLVED parse (hand-built IR —
    // never a rerun of the CLI's own YAML parse).
    let mut oracle = MessageSchema::new("Marker");
    oracle.add_field(FieldDef::new(
        "origin",
        FieldType::Nested {
            schema_name: "Point".to_string(),
            package: Some("geometry_msgs".to_string()),
            fixed: None,
        },
    ));
    oracle.add_field(FieldDef::new("kind", FieldType::U8));
    assert_eq!(oracle.wire_fixed_size(), 1, "unresolved IR: kind only");
    assert_eq!(
        entry.schema_hash,
        Some(oracle.schema_hash()),
        "hash must stay the single-file unresolved-parse value"
    );
}

/// A file whose YAML declares MULTIPLE schemas
/// — every entry is listed, every listed ENTRY NAME is introspectable
/// (tier-2 entry-name lookup, filtered to the match), and the
/// FILE-STEM lookup still returns the whole file.
#[test]
fn multi_schema_file_lists_all_and_info_finds_each_by_entry_name() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    std::fs::create_dir_all(&schemas_dir).unwrap();
    let content =
        "schemas:\n  Alpha:\n    fields:\n      uint32 a:\n  Beta:\n    fields:\n      string b:\n";
    std::fs::write(schemas_dir.join("pair.yaml"), content).unwrap();

    // `schema list` shows BOTH entries, keyed by schema name.
    let listing = schema_list(&schemas_dir);
    let names: Vec<(&str, &str)> = listing
        .workspace
        .iter()
        .map(|w| (w.name.as_str(), w.file.as_str()))
        .collect();
    assert_eq!(names, vec![("Alpha", "pair.yaml"), ("Beta", "pair.yaml")]);

    // Tier-2 entry-name lookup: each LISTED name resolves, filtered to
    // the matching entry.
    let alpha = schema_info_unified(&schemas_dir, "Alpha").unwrap();
    assert_eq!(alpha.source, SchemaSource::Workspace);
    assert_eq!(alpha.result.entries.len(), 1);
    assert_eq!(alpha.result.entries[0].name, "Alpha");

    let beta = schema_info_unified(&schemas_dir, "Beta").unwrap();
    assert_eq!(beta.result.entries.len(), 1);
    assert_eq!(beta.result.entries[0].name, "Beta");
    assert!(
        beta.result.entries[0].fields[0].is_variable,
        "string b is variable"
    );

    // Tier-1 file-stem lookup contract is preserved: the stem returns
    // the WHOLE file.
    let by_stem = schema_info_unified(&schemas_dir, "pair").unwrap();
    assert_eq!(by_stem.result.entries.len(), 2);
}

/// A workspace row whose frame crosses the wire
/// ceiling only AFTER fixed-nested inlining must list UNHASHED. `Big` (a
/// store `.msg`, exactly at the ceiling) resolves; `Outer` inlines it and
/// adds a `uint64`, so its RESOLVED frame is over while its DECLARED one
/// (the reference counts no bytes) is tiny — a declared-moment check
/// alone would print a hash for it. `Small` is the control row.
#[test]
fn list_workspace_row_composed_over_the_ceiling_lists_unhashed() {
    const AT: usize = u32::MAX as usize - 32;
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    let msg_dir = schemas_dir.join("big_pkg").join("msg");
    std::fs::create_dir_all(&msg_dir).unwrap();
    std::fs::write(msg_dir.join("Big.msg"), format!("uint8[{AT}] a\n")).unwrap();
    std::fs::write(
        schemas_dir.join("outer.yaml"),
        "schemas:\n  Outer:\n    fields:\n      big_pkg/Big b:\n      uint64 x:\n",
    )
    .unwrap();
    std::fs::write(
        schemas_dir.join("small.yaml"),
        "schemas:\n  Small:\n    fields:\n      uint32 a:\n",
    )
    .unwrap();

    let listing = schema_list(&schemas_dir);
    let row = |name: &str| {
        listing
            .workspace
            .iter()
            .find(|w| w.name == name)
            .unwrap_or_else(|| panic!("{name} row listed: {:?}", listing.workspace))
    };
    assert!(
        row("Outer").schema_hash.is_none(),
        "a composition past the ceiling lists UNHASHED, never a hash no frame can carry"
    );
    assert!(row("Small").schema_hash.is_some(), "the control row hashes");
    let big = listing
        .store
        .iter()
        .find(|s| s.name == "big_pkg/Big")
        .expect("store row listed");
    assert!(
        big.schema_hash.is_some(),
        "AT the ceiling is representable — the store row hashes"
    );
}

/// An over-ceiling YAML definition whose NAME a
/// lower tier also bears must not borrow that tier's layout. With the
/// resolved-frame gate alone, the preflight drops the hostile YAML
/// `geometry_msgs/Vector3` and `layout_of` then finds the BUILT-IN, so the
/// row would print a hash for a declaration no frame can carry; the same hole
/// falls through to a sane STORE twin. Both rows list UNHASHED; the store
/// twin's own row still hashes, and a sane shadowing YAML twin still lists
/// with its hash (the precedence itself is untouched).
#[test]
fn list_over_ceiling_yaml_row_never_borrows_a_lower_tiers_layout() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    let msg_dir = schemas_dir.join("acme_pkg").join("msg");
    std::fs::create_dir_all(&msg_dir).unwrap();
    std::fs::write(msg_dir.join("Twin.msg"), "uint32 a\n").unwrap();
    // 520 × float64[1_048_576] parses (each dimension at the YAML cap) and
    // declares a fixed section past `u32::MAX − 32`.
    let over = |name: &str| {
        let mut out = format!("schemas:\n  {name}:\n    fields:\n");
        for i in 0..520 {
            out.push_str(&format!("      float64[1048576] f{i}:\n"));
        }
        out
    };
    std::fs::write(schemas_dir.join("vec.yaml"), over("geometry_msgs/Vector3")).unwrap();
    std::fs::write(schemas_dir.join("twin.yaml"), over("acme_pkg/Twin")).unwrap();
    std::fs::write(
        schemas_dir.join("sane.yaml"),
        "schemas:\n  std_msgs/Bool:\n    fields:\n      bool data:\n",
    )
    .unwrap();

    let listing = schema_list(&schemas_dir);
    let row = |name: &str| {
        listing
            .workspace
            .iter()
            .find(|w| w.name == name)
            .unwrap_or_else(|| panic!("{name} row listed: {:?}", listing.workspace))
    };
    assert!(
        row("geometry_msgs/Vector3").schema_hash.is_none(),
        "the over-ceiling YAML twin of a built-in lists UNHASHED, never with the \
         built-in's representability"
    );
    assert!(
        row("acme_pkg/Twin").schema_hash.is_none(),
        "the over-ceiling YAML twin of a store type lists UNHASHED"
    );
    assert!(
        row("std_msgs/Bool").schema_hash.is_some(),
        "a sane shadowing YAML twin still lists with its hash"
    );
    let store_twin = listing
        .store
        .iter()
        .find(|s| s.name == "acme_pkg/Twin")
        .expect("store row listed");
    assert!(
        store_twin.schema_hash.is_some(),
        "the store twin's own row hashes"
    );
}

/// At the resolver seam: a store PARENT that inlines
/// an over-ceiling target must be refused WITH it. A set preflight that
/// dropped `Huge` at the declared moment would let `Parent` resolve with `h`
/// left variable — a small "representable" layout — and both `schema list`
/// and `schema info` would serve a hash and size no producer can emit. So
/// `Huge` stays in the set, `Parent` composes over, and both list unhashed
/// / are refused; the sane sibling is untouched.
#[test]
fn store_parent_of_an_over_ceiling_target_is_refused_with_it() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    let msg_dir = schemas_dir.join("nav32").join("msg");
    std::fs::create_dir_all(&msg_dir).unwrap();
    std::fs::write(msg_dir.join("Huge.msg"), format!("uint8[{}] a\n", u32::MAX)).unwrap();
    std::fs::write(msg_dir.join("Parent.msg"), "Huge h\nfloat64 x\n").unwrap();
    std::fs::write(msg_dir.join("Ok.msg"), "uint32 a\n").unwrap();

    let listing = schema_list(&schemas_dir);
    let store_row = |name: &str| {
        listing
            .store
            .iter()
            .find(|s| s.name == name)
            .unwrap_or_else(|| panic!("{name} row listed: {:?}", listing.store))
    };
    assert!(
        store_row("nav32/Huge").schema_hash.is_none(),
        "the target lists unhashed"
    );
    assert!(
        store_row("nav32/Parent").schema_hash.is_none(),
        "the parent composes past the ceiling through its target and lists UNHASHED — \
         never a hash of a layout with the reference left variable"
    );
    assert!(
        store_row("nav32/Ok").schema_hash.is_some(),
        "the sane sibling hashes"
    );

    let parent = schema_info_unified(&schemas_dir, "nav32/Parent")
        .expect_err("a parent whose resolved frame crosses the ceiling is refused");
    assert!(
        parent.to_string().contains("nav32/Parent"),
        "the refusal names the parent: {parent}"
    );
    schema_info_unified(&schemas_dir, "nav32/Huge")
        .expect_err("the over-ceiling target itself is refused");
    let ok = schema_info_unified(&schemas_dir, "nav32/Ok").expect("the sane sibling resolves");
    assert_eq!(ok.result.entries[0].wire_fixed_size, 4);
}

/// Decision: `schema info` agrees with `schema list` on an
/// entry whose frame crosses the ceiling only AFTER fixed-nested inlining —
/// the parse-time hash is WITHDRAWN (it would name a contract no producer
/// can stamp) and the rendering carries the same unhashable marker the
/// listing row gets; the layout stays unresolved beside it. `Small` is the
/// control: hashed and resolved.
#[test]
fn info_withholds_the_hash_of_an_entry_composed_over_the_ceiling() {
    const AT: usize = u32::MAX as usize - 32;
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    let msg_dir = schemas_dir.join("big_pkg").join("msg");
    std::fs::create_dir_all(&msg_dir).unwrap();
    std::fs::write(msg_dir.join("Big.msg"), format!("uint8[{AT}] a\n")).unwrap();
    std::fs::write(
        schemas_dir.join("outer.yaml"),
        "schemas:\n  Outer:\n    fields:\n      big_pkg/Big b:\n      uint64 x:\n",
    )
    .unwrap();
    std::fs::write(
        schemas_dir.join("small.yaml"),
        "schemas:\n  Small:\n    fields:\n      uint32 a:\n",
    )
    .unwrap();

    let outer = schema_info_unified(&schemas_dir, "Outer").expect("the entry still lists");
    let entry = &outer.result.entries[0];
    assert_eq!(
        entry.schema_hash, None,
        "no frame can carry Outer — no hash"
    );
    assert!(
        !entry.wire_fixed_size_resolved,
        "the layout stays unresolved beside the withdrawn hash"
    );
    let rendered = format!("{outer}");
    assert!(
        rendered.contains("hash: (unhashable"),
        "the info render carries the listing's unhashable marker:\n{rendered}"
    );
    let listing = schema_list(&schemas_dir);
    assert!(
        listing
            .workspace
            .iter()
            .find(|w| w.name == "Outer")
            .expect("row listed")
            .schema_hash
            .is_none(),
        "list and info agree"
    );

    let small = schema_info_unified(&schemas_dir, "Small").unwrap();
    assert!(
        small.result.entries[0].schema_hash.is_some(),
        "the control hashes"
    );
    assert!(small.result.entries[0].wire_fixed_size_resolved);
}

/// This test pins two facts: `pkg::Type` and `pkg/Type` are ONE identity
/// for CLAIMS — two files spelling one name both ways are a duplicate entry,
/// REFUSED on both listing rows and on `schema info` by either spelling,
/// naming both files — while each entry keeps its DECLARED spelling as its
/// hash identity: a workspace node's build script reads the same YAML key
/// into `MessageSchema::new` ("one yaml, one hash"), so a `::` entry's hash
/// is exactly that raw-key hash.
#[test]
fn double_colon_and_slash_spellings_are_one_identity_and_collide() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    std::fs::create_dir_all(&schemas_dir).unwrap();
    std::fs::write(
        schemas_dir.join("a_twin.yaml"),
        "schemas:\n  acme::Twin:\n    fields:\n      uint32 a:\n",
    )
    .unwrap();
    std::fs::write(
        schemas_dir.join("b_twin.yaml"),
        "schemas:\n  acme/Twin:\n    fields:\n      uint32 b:\n",
    )
    .unwrap();
    std::fs::write(
        schemas_dir.join("solo.yaml"),
        "schemas:\n  acme::Solo:\n    fields:\n      uint32 s:\n",
    )
    .unwrap();

    let listing = schema_list(&schemas_dir);
    let twins: Vec<_> = listing
        .workspace
        .iter()
        .filter(|w| w.name == "acme::Twin" || w.name == "acme/Twin")
        .collect();
    // EACH declared spelling lists exactly once (an
    // OR-filter count of 2 would also pass two canonical rows).
    for spelling in ["acme::Twin", "acme/Twin"] {
        assert_eq!(
            listing
                .workspace
                .iter()
                .filter(|w| w.name == spelling)
                .count(),
            1,
            "{spelling} lists under its declared spelling exactly once: {:?}",
            listing.workspace
        );
    }
    for row in &twins {
        let refusal = row.refusal.as_deref().expect("a twin row is REFUSED");
        assert!(
            refusal.contains("a_twin.yaml") && refusal.contains("b_twin.yaml"),
            "the refusal names both files: {refusal}"
        );
        assert!(row.schema_hash.is_none(), "a refused row is unhashed");
    }
    for spelling in ["acme/Twin", "acme::Twin"] {
        let err = schema_info_unified(&schemas_dir, spelling)
            .expect_err("an ambiguous identity is refused by either spelling");
        let msg = err.to_string();
        assert!(
            msg.contains("a_twin.yaml") && msg.contains("b_twin.yaml"),
            "{spelling}: the refusal names both sources: {msg}"
        );
    }
    // The control: one `::` file is one entry, resolved by either spelling
    // under its declared name — and its hash is the RAW-key hash a node's
    // build script computes from the same YAML.
    let mut raw = MessageSchema::new("acme::Solo");
    raw.add_field(FieldDef::new("s", FieldType::U32));
    let solo = listing
        .workspace
        .iter()
        .find(|w| w.name == "acme::Solo")
        .expect("the :: entry lists under its declared spelling");
    assert!(solo.refusal.is_none());
    assert_eq!(
        solo.schema_hash,
        Some(raw.schema_hash()),
        "the wire hash is the declared key's"
    );
    for spelling in ["acme/Solo", "acme::Solo"] {
        let info =
            schema_info_unified(&schemas_dir, spelling).expect("resolves by either spelling");
        assert_eq!(info.result.entries[0].name, "acme::Solo");
        assert_eq!(info.result.entries[0].schema_hash, Some(raw.schema_hash()));
    }
}

/// A `Goal.yaml` whose sole entry is
/// spelled `pkg::Other` binds the stem `Goal` — and the binding must reach
/// the graph-hash map (and the recorded size map, which shares the fold
/// shape and is pinned in-module), which look the parsed
/// definition up by its DECLARED name. Handing the binding the
/// canonical `pkg/Other` instead makes `Goal` vanish from both.
#[test]
fn a_stem_bound_to_a_double_colon_entry_reaches_the_folds() {
    use cerulion_cli_engine::graph_cmd::build_workspace_schema_hashes;
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    std::fs::create_dir_all(&schemas_dir).unwrap();
    std::fs::write(
        schemas_dir.join("Goal.yaml"),
        "schemas:\n  pkg::Other:\n    fields:\n      uint32 a:\n      float64 b:\n",
    )
    .unwrap();
    let hashes = build_workspace_schema_hashes(tmp.path());
    assert!(
        hashes.contains_key("Goal"),
        "the stem is hashed: {hashes:?}"
    );
    // The entry itself is keyed CANONICALLY (what a graph's normalized
    // `schema:` looks up) and carries the DECLARED name's hash.
    let mut raw = MessageSchema::new("pkg::Other");
    raw.add_field(FieldDef::new("a", FieldType::U32));
    raw.add_field(FieldDef::new("b", FieldType::F64));
    assert_eq!(
        hashes.get("pkg/Other"),
        Some(&raw.schema_hash()),
        "{hashes:?}"
    );
    assert!(
        !hashes.contains_key("pkg::Other"),
        "never keyed raw: {hashes:?}"
    );
    assert_eq!(
        hashes.get("Goal"),
        Some(&raw.schema_hash()),
        "the stem carries its sole entry's hash"
    );
    let info = schema_info_unified(&schemas_dir, "Goal").expect("the stem resolves");
    assert_eq!(info.result.entries[0].name, "pkg::Other");
}

/// Workspace entries list in SORTED file order regardless
/// of creation order — exercises the `files.sort()` that closes the
/// OS-dependent `read_dir` ordering hole — and two renders are
/// byte-identical.
#[test]
fn list_workspace_entries_sorted_across_files() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    std::fs::create_dir_all(&schemas_dir).unwrap();
    // Written in REVERSE of the expected listing order.
    let bravo = "schemas:\n  Bravo:\n    fields:\n      uint32 b:\n";
    std::fs::write(schemas_dir.join("b_second.yaml"), bravo).unwrap();
    let alpha = "schemas:\n  Alpha:\n    fields:\n      uint32 a:\n";
    std::fs::write(schemas_dir.join("a_first.yaml"), alpha).unwrap();

    let listing = schema_list(&schemas_dir);
    let order: Vec<(&str, &str)> = listing
        .workspace
        .iter()
        .map(|w| (w.name.as_str(), w.file.as_str()))
        .collect();
    assert_eq!(
        order,
        vec![("Alpha", "a_first.yaml"), ("Bravo", "b_second.yaml")],
        "sorted by file name, not creation order"
    );
    assert_eq!(
        format!("{}", listing),
        format!("{}", schema_list(&schemas_dir)),
        "two renders must be byte-identical"
    );
}

/// The same entry NAME declared in TWO workspace files is
/// a workspace inconsistency — `schema list` still renders BOTH
/// entries, but layout enrichment is SKIPPED for that name (resolving
/// through whichever definition the resolver kept would be arbitrary),
/// so the parse-time columns stand, and a `warn!` names the duplicate
/// and its declaring files. The skip is OBSERVABLE: `one.yaml`'s `Dup`
/// has a `geometry_msgs/Point` field that enrichment WOULD resolve to
/// fixed/24 B — parse-time keeps it variable with `wire_fixed_size` 0.
#[tracing_test::traced_test]
#[test]
fn duplicate_workspace_entry_names_skip_enrichment_and_warn() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    std::fs::create_dir_all(&schemas_dir).unwrap();
    let one = "schemas:\n  Dup:\n    fields:\n      geometry_msgs/Point origin:\n";
    std::fs::write(schemas_dir.join("one.yaml"), one).unwrap();
    let two = "schemas:\n  Dup:\n    fields:\n      uint32 a:\n";
    std::fs::write(schemas_dir.join("two.yaml"), two).unwrap();

    // `schema list` renders BOTH duplicate entries (keyed by file).
    let listing = schema_list(&schemas_dir);
    let names: Vec<(&str, &str)> = listing
        .workspace
        .iter()
        .map(|w| (w.name.as_str(), w.file.as_str()))
        .collect();
    assert_eq!(names, vec![("Dup", "one.yaml"), ("Dup", "two.yaml")]);

    // Enrichment skipped for the duplicated name: parse-time columns
    // stand (unresolved nested = variable, contributing nothing).
    // Enrichment through EITHER duplicate definition would flip origin
    // to non-variable and the size to a non-zero value (4 via two.yaml's
    // last-wins layout, 24 via one.yaml's) — 0/variable proves the skip.
    let info = schema_info_unified(&schemas_dir, "one").unwrap();
    let entry = &info.result.entries[0];
    assert_eq!(entry.name, "Dup");
    assert_eq!(
        entry.wire_fixed_size, 0,
        "parse-time size — enrichment must be skipped"
    );
    assert!(
        !entry.wire_fixed_size_resolved,
        "the skip must be carried on the entry"
    );
    assert!(
        entry.fields[0].is_variable,
        "parse-time classification — enrichment skipped"
    );

    // The renderer marks the parse-time size loudly:
    // an unresolved size must never print identically to a resolved one.
    let rendered = format!("{}", info);
    assert!(
        rendered.contains("wire fixed size: 0 bytes (unresolved)"),
        "unresolved size must carry the loud suffix:\n{rendered}"
    );

    // The loud inconsistency warn fires, naming the duplicate + files.
    assert!(
        logs_contain("duplicate workspace schema name"),
        "the duplicate-name warn must fire"
    );
    assert!(
        logs_contain("one.yaml"),
        "warn must name the first declaring file"
    );
    assert!(
        logs_contain("two.yaml"),
        "warn must name the second declaring file"
    );

    // The duplicated NAME is refused wherever a
    // name is resolved — `schema info Dup` prints the refusal (every file
    // named), both port surfaces refuse `Dup` AND the stem `one` (its sole
    // entry is the duplicate), and `schema list` prints the refusal on both
    // rows. The file view above (`schema info one`) still shows the file,
    // each entry judged on its own.
    let err = schema_info_unified(&schemas_dir, "Dup")
        .expect_err("a duplicated entry name prints the refusal")
        .to_string();
    assert!(
        err.contains("'Dup' is ambiguous in this workspace — defined by:")
            && err.contains("schemas/one.yaml (entry Dup), schemas/two.yaml (entry Dup)"),
        "got: {err}"
    );
    assert_eq!(
        cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "Dup")
            .expect_err("the gate refuses the duplicate")
            .to_string(),
        err
    );
    // The STEM `one` names exactly ONE file, so the spelling surfaces
    // resolve it; what no MAP can do is bind a document for it,
    // because its sole entry is the duplicated name — that is the FOLD's
    // refusal, and `schema list` carries it on both `Dup` rows below.
    assert!(cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "one").unwrap());
    assert_eq!(
        cerulion_cli_engine::graph_cmd::build_workspace_schema_hashes(tmp.path()).get("one"),
        None,
        "the fold binds nothing for a stem whose sole entry is refused"
    );
    let listing = schema_list(&schemas_dir);
    for row in listing.workspace.iter().filter(|w| w.name == "Dup") {
        assert_eq!(
            row.schema_hash, None,
            "{}: no hash for a refused name",
            row.file
        );
        assert_eq!(
            row.refusal.as_deref(),
            Some(err.as_str()),
            "{}: the refusal",
            row.file
        );
    }
}

/// `schema list`'s shadow marker and `schema info`'s
/// `shadowed_builtin_names` must normalize the spelling the SAME way. If the
/// list compared the DECLARED spelling while info normalized it — a workspace
/// entry declared `std_msgs::String` would be reported by `schema info` as
/// shadowing the built-in while `schema list` still showed the built-in as
/// reachable. One identity, one answer, on both surfaces.
#[test]
fn a_double_colon_workspace_entry_shadows_the_builtin_on_list_as_well_as_info() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    std::fs::create_dir_all(&schemas_dir).unwrap();
    std::fs::write(
        schemas_dir.join("mystring.yaml"),
        "schemas:\n  std_msgs::String:\n    fields:\n      uint32 a:\n",
    )
    .unwrap();

    // `schema info` says it shadows (the normalized comparison).
    let info = schema_info_unified(&schemas_dir, "std_msgs/String").unwrap();
    assert_eq!(
        info.shadowed_builtin.as_deref(),
        Some("std_msgs/String"),
        "premise: schema info reports the shadow for the `::` spelling"
    );

    // THE PIN: `schema list` marks the same built-in as shadowed, under the
    // entry's DECLARED spelling (the row renders what the file says).
    let listing = schema_list(&schemas_dir);
    let marked = listing
        .builtin_packages
        .iter()
        .find(|g| g.package == "std_msgs")
        .expect("the std_msgs group is listed")
        .types
        .iter()
        .find(|t| t.name == "String")
        .expect("the String built-in is listed");
    assert_eq!(
        marked.shadowed_by.as_deref(),
        Some("std_msgs::String"),
        "schema list must not say the built-in is reachable when schema info \
         resolves the workspace copy"
    );
}

/// UNIFIED TREE (built-in, deep nesting): `nav_msgs/Odometry` renders as the
/// header block + a recursive field tree — every non-primitive field expands
/// inline, indented directly beneath it. Full hand-pasted oracle; the wire
/// identity (`hash`/`wire fixed size`) is interpolated from the GENERATED twin
/// so a recipe change fails here, not silently. Pins: `header`→`std_msgs/Header`
/// (variable) → `stamp`→`builtin_interfaces/Time` (fixed) → `sec`/`nanosec`
/// leaves + `frame_id` (string, variable); the `covariance` fixed array of a
/// PRIMITIVE (`float64[36]`) does NOT expand; and the repeated-type policy —
/// the 2nd `geometry_msgs/Vector3` (`angular`) shows `(fixed, expanded above)`
/// and does NOT re-expand.
#[test]
fn unified_builtin_odometry_renders_full_recursive_tree() {
    use native_ros2_messages::nav_msgs::Odometry;
    let tmp = tempfile::tempdir().unwrap();
    let dir = tmp.path().join("schemas"); // never created
    let rendered = format!(
        "{}",
        schema_info_unified(&dir, "nav_msgs/Odometry").unwrap()
    );
    let expected = format!(
        "schema: nav_msgs/Odometry\n\
         source: built-in (ROS 2)\n\
         wire fixed size: {} bytes\n\
         hash: 0x{:016x}\n\
         fields: 4\n\
         \x20 header: std_msgs/Header (variable)\n\
         \x20   stamp: builtin_interfaces/Time (fixed)\n\
         \x20     sec: int32 (fixed)\n\
         \x20     nanosec: uint32 (fixed)\n\
         \x20   frame_id: string (variable)\n\
         \x20 child_frame_id: string (variable)\n\
         \x20 pose: geometry_msgs/PoseWithCovariance (fixed)\n\
         \x20   pose: geometry_msgs/Pose (fixed)\n\
         \x20     position: geometry_msgs/Point (fixed)\n\
         \x20       x: float64 (fixed)\n\
         \x20       y: float64 (fixed)\n\
         \x20       z: float64 (fixed)\n\
         \x20     orientation: geometry_msgs/Quaternion (fixed)\n\
         \x20       x: float64 (fixed)\n\
         \x20       y: float64 (fixed)\n\
         \x20       z: float64 (fixed)\n\
         \x20       w: float64 (fixed)\n\
         \x20   covariance: float64[36] (fixed)\n\
         \x20 twist: geometry_msgs/TwistWithCovariance (fixed)\n\
         \x20   twist: geometry_msgs/Twist (fixed)\n\
         \x20     linear: geometry_msgs/Vector3 (fixed)\n\
         \x20       x: float64 (fixed)\n\
         \x20       y: float64 (fixed)\n\
         \x20       z: float64 (fixed)\n\
         \x20     angular: geometry_msgs/Vector3 (fixed, expanded above)\n\
         \x20   covariance: float64[36] (fixed)\n",
        <Odometry as ShmMessage>::WIRE_FIXED_SIZE,
        <Odometry as ShmMessage>::SCHEMA_HASH,
    );
    assert_eq!(rendered, expected, "GOT:\n{rendered}");
}

/// UNIFIED TREE (workspace custom w/ nested custom + array-of-nested-custom):
/// a LowState-shaped workspace schema referencing a nested custom `IMUState`
/// and an array-of-nested-custom `MotorState[3]` — BOTH expand inline exactly
/// like built-ins, proving the renderer is source-agnostic. The `source:` line
/// names the declaring file. Full hand-pasted oracle (the workspace hash is a
/// single-file parse value; anchored via the CLI's own `schema list` hash for
/// this file to avoid a magic literal).
#[test]
fn unified_workspace_custom_with_nested_custom_renders_full_tree() {
    use cerulion_cli_engine::schema_cmd::schema_list;
    let tmp = tempfile::tempdir().unwrap();
    let dir = tmp.path().join("schemas");
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(
        dir.join("imu_state.yaml"),
        "schemas:\n  IMUState:\n    fields:\n      float32[4] quaternion:\n      \
         int8 temperature:\n",
    )
    .unwrap();
    std::fs::write(
        dir.join("motor_state.yaml"),
        "schemas:\n  MotorState:\n    fields:\n      uint8 mode:\n      float32 q:\n",
    )
    .unwrap();
    std::fs::write(
        dir.join("low_state.yaml"),
        "schemas:\n  LowState:\n    description: \"low-level robot state\"\n    fields:\n      \
         uint8[2] head:\n      IMUState imu_state:\n      MotorState[3] motor_state:\n      \
         uint32 tick:\n",
    )
    .unwrap();

    // Anchor the displayed hash to the CLI's own listing (not a magic literal).
    let listing = schema_list(&dir);
    let ls_hash = listing
        .workspace
        .iter()
        .find(|w| w.name == "LowState")
        .expect("LowState listed")
        .schema_hash
        .expect("a representable workspace entry lists with its hash");

    let rendered = format!("{}", schema_info_unified(&dir, "LowState").unwrap());
    let expected = format!(
        "schema: LowState\n\
         source: workspace (schemas/low_state.yaml)\n\
         description: low-level robot state\n\
         wire fixed size: 52 bytes\n\
         hash: 0x{ls_hash:016x}\n\
         fields: 4\n\
         \x20 head: uint8[2] (fixed)\n\
         \x20 imu_state: IMUState (fixed)\n\
         \x20   quaternion: float32[4] (fixed)\n\
         \x20   temperature: int8 (fixed)\n\
         \x20 motor_state: MotorState[3] (fixed)\n\
         \x20   mode: uint8 (fixed)\n\
         \x20   q: float32 (fixed)\n\
         \x20 tick: uint32 (fixed)\n",
    );
    assert_eq!(rendered, expected, "GOT:\n{rendered}");
}

/// UNIFIED TREE (unknown/typo'd nested ref): a workspace field whose type is
/// neither a primitive nor a known schema is flagged LOUDLY on its field line
/// as `(… unknown type — no schema found)` — the loud-over-silent successor to
/// the old `(treated as nested schema reference)` marker. A real primitive
/// sibling stays clean.
#[test]
fn unified_unknown_nested_ref_is_flagged_loudly_on_the_field_line() {
    let tmp = tempfile::tempdir().unwrap();
    let dir = tmp.path().join("schemas");
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(
        dir.join("pose.yaml"),
        "schemas:\n  Pose:\n    fields:\n      Position3D position:\n      uint32 seq:\n",
    )
    .unwrap();
    let rendered = format!("{}", schema_info_unified(&dir, "Pose").unwrap());
    assert!(
        rendered.contains("  position: Position3D (variable, unknown type — no schema found)\n"),
        "unknown nested type must be flagged loudly:\n{rendered}"
    );
    assert!(
        rendered.contains("  seq: uint32 (fixed)\n"),
        "a real primitive sibling stays clean:\n{rendered}"
    );
}

/// A REFUSED workspace name binds
/// nothing, so it shadows nothing — `schema list` must not mark the built-in
/// `sensor_msgs/Image` as shadowed by a workspace `Image` that two files
/// declare (a name no lookup can resolve to either file).
#[test]
fn a_refused_workspace_name_shadows_no_builtin_in_the_listing() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    std::fs::create_dir_all(&schemas_dir).unwrap();
    for file in ["a.yaml", "b.yaml"] {
        std::fs::write(
            schemas_dir.join(file),
            "schemas:\n  Image:\n    fields:\n      uint32 a:\n",
        )
        .unwrap();
    }
    let listing = schema_list(&schemas_dir);
    let image = listing
        .builtin_packages
        .iter()
        .find(|g| g.package == "sensor_msgs")
        .expect("sensor_msgs group present")
        .types
        .iter()
        .find(|t| t.name == "Image")
        .expect("Image entry present");
    assert_eq!(
        image.shadowed_by, None,
        "a refused name shadows nothing; got {:?}",
        image.shadowed_by
    );
    // `all` over an EMPTY filter is vacuously true —
    // count the rows first, so a listing that dropped both duplicates fails.
    let image_rows: Vec<_> = listing
        .workspace
        .iter()
        .filter(|w| w.name == "Image")
        .collect();
    assert_eq!(
        image_rows.len(),
        2,
        "both duplicate rows are listed; got {image_rows:?}"
    );
    assert!(
        image_rows.iter().all(|w| w.refusal.is_some()),
        "both rows carry the refusal"
    );
}

/// A fixed-nested YAML root the
/// preflight REMOVES (its composed layout overflows through a store
/// `Filler[8589934592]`) resolves to NO layout at all — `schema info` must
/// report it unhashable exactly like a resolved frame past the ceiling
/// (keeping the parse-time hash would disagree with `schema list`).
#[test]
fn r39_info_withholds_the_hash_of_a_root_the_preflight_removed() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    let msg_dir = schemas_dir.join("big_pkg").join("msg");
    std::fs::create_dir_all(&msg_dir).unwrap();
    std::fs::write(msg_dir.join("Filler.msg"), "float64[536870907] v\n").unwrap();
    // The YAML parser caps a single fixed array, so the composed overflow
    // rides 4096 capped `Filler[1048576]` fields (the composed-overflow shape): each
    // fits usize alone; their sum does not — the preflight REMOVES Outer.
    let mut outer = String::from("schemas:\n  Outer:\n    fields:\n");
    for i in 0..4096 {
        outer.push_str(&format!("      big_pkg/Filler[1048576] f{i}:\n"));
    }
    std::fs::write(schemas_dir.join("outer.yaml"), outer).unwrap();
    std::fs::write(
        schemas_dir.join("small.yaml"),
        "schemas:\n  Small:\n    fields:\n      uint32 a:\n",
    )
    .unwrap();

    let outer = schema_info_unified(&schemas_dir, "Outer").expect("the entry still lists");
    let entry = &outer.result.entries[0];
    assert_eq!(
        entry.schema_hash, None,
        "a removed root has no frame — no hash"
    );
    assert!(
        !entry.wire_fixed_size_resolved,
        "the layout stays unresolved beside the withdrawn hash"
    );
    let rendered = format!("{outer}");
    assert!(
        rendered.contains("hash: (unhashable"),
        "the info render carries the listing's unhashable marker:\n{rendered}"
    );
    // The control: the sane sibling still hashes.
    let small = schema_info_unified(&schemas_dir, "Small").unwrap();
    assert!(small.result.entries[0].schema_hash.is_some());
}
