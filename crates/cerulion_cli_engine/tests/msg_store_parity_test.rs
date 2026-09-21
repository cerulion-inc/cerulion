// SPDX-License-Identifier: AGPL-3.0-only
//! msg-store parity: the workspace `.msg` store (`schemas/<pkg>/msg/<Type>.msg`)
//! must be treated no worse than workspace YAML on the combined-set surfaces.
//!
//! Two of the four parity surfaces are pinned here (both `pub`, both pure
//! file-parse — no transport, per-test tempdirs, parallel-safe):
//!
//! - **Surface 1** — `graph_cmd::build_workspace_schema_hashes` (the
//!   hash-divergence map at `graph run`). Without the fold, a store type
//!   produces NO map entry, so the runtime silently skips the divergence
//!   check for every store-typed output.
//! - **Surface 3** — `replay_field_registry::FieldRegistry::from_graph` (the
//!   replay tolerance registry). Without the fold, a store-typed topic
//!   degrades to schema-unavailable, so its field-level validation is
//!   silently skipped.
//!
//! (Surface 2 — the recorded wire-size map — is a private `#[cfg(unix)]` fn,
//! pinned in `graph_cmd`'s in-module tests; surface 4 —
//! `resolve_port_schema` — is pinned in `schema_cmd`'s in-module tests.)
//!
//! Oracles are HAND-BUILT schema IR (`MessageSchema::new_in_package` +
//! `FieldDef`s), resolved + hashed directly — never the file-parse path under
//! test, so no assertion is a self-compare.

use std::path::Path;

use cerulion_cli_engine::graph_cmd;
use cerulion_cli_engine::replay_field_registry::{FieldRegistry, TopicClass};
use cerulion_cli_engine::tolerance::FieldResolver;
use cerulion_core::codegen::{resolve_fixed_nested, FieldDef, FieldType, MessageSchema};

// ---------------------------------------------------------------------------
// Fixtures + oracles
// ---------------------------------------------------------------------------

/// Write one workspace YAML schema file under `<root>/schemas/`.
fn write_yaml(root: &Path, file: &str, content: &str) {
    let dir = root.join("schemas");
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join(file), content).unwrap();
}

/// A PARSEABLE workspace declaration whose fixed section exceeds the wire
/// ceiling. A `float64[u64::MAX]` fixture would be
/// refused by the YAML parser's per-dimension array cap before either
/// fold saw it, so the omission asserts would hold with the folds' ceiling retain
/// deleted. 520 × `float64[1_048_576]` (each dimension AT the cap) declares
/// 4,362,076,160 fixed bytes — past `u32::MAX − 32` — and parses.
fn hostile_yaml() -> String {
    let mut out = String::from("schemas:\n  HostileY:\n    fields:\n");
    for i in 0..520 {
        out.push_str(&format!("      float64[1048576] f{i}:\n"));
    }
    out
}

/// Write one `.msg` store file under `<root>/schemas/<pkg>/msg/`.
fn write_store_msg(root: &Path, pkg: &str, type_name: &str, text: &str) {
    let dir = root.join("schemas").join(pkg).join("msg");
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join(format!("{type_name}.msg")), text).unwrap();
}

/// Hand-built IR for the store type used throughout: `nav_pkg/Goal`
/// (`float64 x`, `float64 y`).
fn goal_store_ir() -> MessageSchema {
    let mut s = MessageSchema::new_in_package("Goal", "nav_pkg");
    s.add_field(FieldDef::new("x", FieldType::F64));
    s.add_field(FieldDef::new("y", FieldType::F64));
    s
}

/// The `.msg` text matching [`goal_store_ir`].
const GOAL_MSG: &str = "float64 x\nfloat64 y\n";

/// Resolve a hand-built set and return the recipe-3 hash of the schema whose
/// QUALIFIED name is `target` (bypasses every file-parse path under test).
/// A built-in's IR from the vendored `.msg` text: the oracle
/// side of "the fold resolves against the compiled corpus", parsed here by
/// the recipe's own parser rather than borrowed from the fold under test.
fn builtin_ir(pkg: &str, name: &str) -> MessageSchema {
    let (_, _, text) = native_ros2_messages::BUILTIN_MSGS
        .iter()
        .find(|&&(p, n, _)| p == pkg && n == name)
        .unwrap_or_else(|| panic!("built-in {pkg}/{name} is vendored"));
    cerulion_core::codegen::parse_rosmsg(text, name, Some(pkg)).expect("vendored text parses")
}

fn oracle_hash(set: &mut [MessageSchema], target: &str) -> u64 {
    let _ = resolve_fixed_nested(set);
    set.iter()
        .find(|s| s.qualified_name() == target)
        .unwrap_or_else(|| panic!("oracle: schema '{target}' not in set"))
        .schema_hash()
}

// ===========================================================================
// Surface 1 — build_workspace_schema_hashes
// ===========================================================================

/// THE surface-1 parity pin (fails without the store fold: the map has NO entry for
/// a store type, so `graph run`'s divergence check silently skips it).
#[test]
fn s1_a_store_type_appears_in_the_schema_hash_map_with_its_codegen_hash() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "nav_pkg", "Goal", GOAL_MSG);

    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());

    let expected = oracle_hash(&mut [goal_store_ir()], "nav_pkg/Goal");
    assert_eq!(
        map.get("nav_pkg/Goal").copied(),
        Some(expected),
        "a .msg store type must be in the hash map under its qualified name; got {map:?}"
    );
}

/// A YAML type and a store type sharing a STEM coexist under distinct keys,
/// each carrying its own hash — the bare key belongs to the YAML type
/// (workspace-YAML-first precedence), the qualified key to the store type.
#[test]
fn s1_yaml_and_store_types_with_one_stem_coexist_under_distinct_keys() {
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(
        tmp.path(),
        "goal.yaml",
        "schemas:\n  Goal:\n    fields:\n      uint32 a:\n",
    );
    write_store_msg(tmp.path(), "nav_pkg", "Goal", GOAL_MSG);

    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());

    let mut yaml_ir = MessageSchema::new("Goal");
    yaml_ir.add_field(FieldDef::new("a", FieldType::U32));
    let yaml_expected = oracle_hash(&mut [yaml_ir], "Goal");
    let store_expected = oracle_hash(&mut [goal_store_ir()], "nav_pkg/Goal");
    assert_ne!(
        yaml_expected, store_expected,
        "the two layouts differ, so the oracle hashes must too (else this pins nothing)"
    );
    assert_eq!(map.get("Goal").copied(), Some(yaml_expected));
    assert_eq!(map.get("nav_pkg/Goal").copied(), Some(store_expected));
}

/// A store type nesting a
/// BUILT-IN (`std_msgs/Header`) HASHES — the fold resolves against the
/// compiled corpus exactly as a node's build script does, so its hash is
/// codegen's and the divergence check covers it (a closure gate that
/// omitted it would make `graph run` silently skip the check). The oracle is
/// the vendored built-in text resolved by hand; the flat sibling still
/// hashes; the built-in itself is never served.
#[test]
fn s1_a_store_type_nesting_a_builtin_hashes_like_codegen() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(
        tmp.path(),
        "nav_pkg",
        "Stamped",
        "std_msgs/Header header\nfloat64 v\n",
    );
    write_store_msg(tmp.path(), "nav_pkg", "Goal", GOAL_MSG);

    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());

    let expected = {
        let mut stamped = MessageSchema::new_in_package("Stamped", "nav_pkg");
        stamped.add_field(FieldDef::new(
            "header",
            FieldType::Nested {
                schema_name: "Header".to_string(),
                package: Some("std_msgs".to_string()),
                fixed: None,
            },
        ));
        stamped.add_field(FieldDef::new("v", FieldType::F64));
        oracle_hash(
            &mut [
                builtin_ir("builtin_interfaces", "Time"),
                builtin_ir("std_msgs", "Header"),
                stamped,
            ],
            "nav_pkg/Stamped",
        )
    };
    assert_eq!(
        map.get("nav_pkg/Stamped").copied(),
        Some(expected),
        "a store type nesting a built-in hashes like codegen; got {map:?}"
    );
    assert!(
        map.contains_key("nav_pkg/Goal"),
        "the flat sibling still hashes"
    );
    assert!(
        !map.contains_key("std_msgs/Header") && !map.contains_key("builtin_interfaces/Time"),
        "built-ins are resolved against, never served: {map:?}"
    );
}

/// A store type with a SAME-PACKAGE bare nested ref (`Inner i` inside
/// `nav_pkg/Outer` — the same-package rule) resolves like codegen:
/// the map hash equals the RESOLVED oracle and differs from the unresolved
/// hash (proving resolution is load-bearing, not just parsed-in-isolation).
#[test]
fn s1_a_store_type_with_a_same_package_bare_ref_resolves_like_codegen() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "nav_pkg", "Inner", "float64 v\n");
    write_store_msg(tmp.path(), "nav_pkg", "Outer", "Inner i\nfloat64 w\n");

    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());

    let inner = {
        let mut s = MessageSchema::new_in_package("Inner", "nav_pkg");
        s.add_field(FieldDef::new("v", FieldType::F64));
        s
    };
    let outer = {
        let mut s = MessageSchema::new_in_package("Outer", "nav_pkg");
        s.add_field(FieldDef::new(
            "i",
            FieldType::Nested {
                schema_name: "Inner".to_string(),
                package: None,
                fixed: None,
            },
        ));
        s.add_field(FieldDef::new("w", FieldType::F64));
        s
    };
    let resolved_expected = oracle_hash(&mut [inner, outer.clone()], "nav_pkg/Outer");
    assert_eq!(map.get("nav_pkg/Outer").copied(), Some(resolved_expected));
    assert_ne!(
        resolved_expected,
        outer.schema_hash(),
        "resolution must change the hash — otherwise this test cannot prove the \
         builder resolved the same-package ref"
    );
}

/// A hostile-but-parseable declaration must not PANIC the map builder — for
/// a YAML file this test FAILS (panics) without the hostile guard, and the store
/// twin guards the store path the same way. The sane sibling
/// still hashes (one hostile file must not sink the rest).
///
/// Also covered: the COMPOSED-overflow-as-nested-TARGET shape (the
/// registry's nested-target class): `ComposedInner` is individually sizable,
/// `ComposedOuter = ComposedInner[4]` overflows only after inlining, and `ComposedWrapper`
/// REFERENCES `ComposedOuter` — resolving the wrapper materializes the
/// overflow target inside `resolve_fixed_nested` itself, which PANICKED
/// this builder before the composed preflight.
#[test]
fn s1_hostile_declarations_are_skipped_loudly_never_a_panic() {
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(tmp.path(), "hostile.yaml", &hostile_yaml());
    write_store_msg(
        tmp.path(),
        "nav_pkg",
        "HostileM",
        "float64[2305843009213693952] a\n",
    );
    write_store_msg(
        tmp.path(),
        "nav_pkg",
        "ComposedInner",
        "float64[2305843009213693951] v\n",
    );
    write_store_msg(
        tmp.path(),
        "nav_pkg",
        "ComposedOuter",
        "ComposedInner[4] arr\n",
    );
    write_store_msg(
        tmp.path(),
        "nav_pkg",
        "ComposedWrapper",
        "ComposedOuter o\n",
    );
    write_store_msg(tmp.path(), "nav_pkg", "Goal", GOAL_MSG);

    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());

    assert!(
        !map.contains_key("HostileY"),
        "hostile YAML omitted: {map:?}"
    );
    assert!(
        !map.contains_key("nav_pkg/HostileM"),
        "hostile store type omitted: {map:?}"
    );
    assert!(
        !map.contains_key("nav_pkg/ComposedOuter"),
        "the composed overflow is preflighted out: {map:?}"
    );
    assert!(
        !map.contains_key("nav_pkg/ComposedWrapper"),
        "the wrapper's ref to the dropped target now escapes the set — \
         omitted, never hashed against a resolution codegen would refuse: {map:?}"
    );
    assert!(
        map.contains_key("nav_pkg/Goal"),
        "the sane sibling survives"
    );
}

/// Two builds over one workspace are byte-identical (BTreeMap-backed store
/// enumeration + sorted YAML walk — Principle #7).
#[test]
fn s1_the_hash_map_is_deterministic_across_builds() {
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(
        tmp.path(),
        "goal.yaml",
        "schemas:\n  Goal:\n    fields:\n      uint32 a:\n",
    );
    write_store_msg(tmp.path(), "nav_pkg", "Goal", GOAL_MSG);
    write_store_msg(tmp.path(), "nav_pkg", "Inner", "float64 v\n");
    write_store_msg(tmp.path(), "nav_pkg", "Outer", "Inner i\nfloat64 w\n");

    let a = graph_cmd::build_workspace_schema_hashes(tmp.path());
    let b = graph_cmd::build_workspace_schema_hashes(tmp.path());
    assert_eq!(a, b, "two builds over one workspace must agree exactly");
    assert!(a.len() >= 4, "sanity: all four entries present, got {a:?}");
}

// ===========================================================================
// Surface 3 — FieldRegistry::from_graph (replay tolerance resolution)
// ===========================================================================

/// A one-node graph producing `/replaytest/cam/out` with the given output
/// `schema:` string.
fn one_output_graph(schema: &str) -> cerulion_core::graph::GraphConfig {
    let yaml = format!(
        "name: replaytest\nprefix: replaytest\nnodes:\n  - id: cam\n    type: camera\n    \
         outputs:\n      - name: out\n        schema: {schema}\n"
    );
    cerulion_core::graph::parse_graph(&yaml).unwrap()
}

const TOPIC: &str = "/replaytest/cam/out";

/// THE surface-3 parity pin (fails without the store fold: a store-typed topic
/// resolves NO schema, so `expected_schema_hash` is `None` and field-level
/// tolerance validation silently skips the topic).
#[test]
fn s3_a_store_typed_topic_resolves_for_replay_field_validation() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "nav_pkg", "Goal", GOAL_MSG);

    let reg = FieldRegistry::from_graph(&one_output_graph("nav_pkg/Goal"), tmp.path());

    let expected = oracle_hash(&mut [goal_store_ir()], "nav_pkg/Goal");
    assert_eq!(
        reg.expected_schema_hash(TOPIC),
        Some(expected),
        "a store-typed produced topic must resolve its expected wire hash"
    );
    // Field-level resolution works against the store schema's real fields.
    assert!(reg.resolve_field(TOPIC, "x").is_ok(), "field `x` resolves");
    assert!(
        reg.resolve_field(TOPIC, "nope").is_err(),
        "a field the store schema does not declare still errors"
    );
}

/// A BARE graph `schema:` resolves to the store when only the store defines
/// that name (the store outranks built-ins for a bare claim it uniquely
/// makes — here nothing else defines `Goal` at all).
#[test]
fn s3_a_bare_schema_string_resolves_to_a_unique_store_type() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "nav_pkg", "Goal", GOAL_MSG);

    let reg = FieldRegistry::from_graph(&one_output_graph("Goal"), tmp.path());

    let expected = oracle_hash(&mut [goal_store_ir()], "nav_pkg/Goal");
    assert_eq!(reg.expected_schema_hash(TOPIC), Some(expected));
}

/// YAML-vs-store precedence for a shared stem: the graph's bare `Goal`
/// resolves to the WORKSPACE YAML copy, not the store's.
#[test]
fn s3_yaml_wins_over_the_store_for_a_shared_bare_name() {
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(
        tmp.path(),
        "goal.yaml",
        "schemas:\n  Goal:\n    fields:\n      uint32 a:\n",
    );
    write_store_msg(tmp.path(), "nav_pkg", "Goal", GOAL_MSG);

    let reg = FieldRegistry::from_graph(&one_output_graph("Goal"), tmp.path());

    let mut yaml_ir = MessageSchema::new("Goal");
    yaml_ir.add_field(FieldDef::new("a", FieldType::U32));
    let yaml_expected = oracle_hash(&mut [yaml_ir], "Goal");
    let store_expected = oracle_hash(&mut [goal_store_ir()], "nav_pkg/Goal");
    assert_ne!(yaml_expected, store_expected, "oracle sanity");
    assert_eq!(
        reg.expected_schema_hash(TOPIC),
        Some(yaml_expected),
        "the bare name must resolve to the YAML copy (workspace-YAML-first)"
    );
}

/// A bare name defined by TWO store packages is never guessed: the topic
/// degrades to schema-unavailable rather than silently picking a package.
#[test]
fn s3_an_ambiguous_bare_store_name_never_guesses() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "pkga", "Goal", "float64 x\n");
    write_store_msg(tmp.path(), "pkgb", "Goal", "uint32 a\n");

    let reg = FieldRegistry::from_graph(&one_output_graph("Goal"), tmp.path());

    // PREMISE (the vacuous-guard class): the output IS a
    // registered Produced topic, so the `None` below pins schema-unavailable,
    // not an output `from_graph` dropped.
    assert_eq!(reg.topic_class(TOPIC), Some(&TopicClass::Produced));
    assert_eq!(
        reg.expected_schema_hash(TOPIC),
        None,
        "an ambiguous bare name must resolve NO schema (never guess)"
    );
}

/// A bare name claimed by ONE store package AND a same-short-named built-in
/// resolves to the STORE (the ladder: workspace YAML → store → built-ins).
#[test]
fn s3_the_store_outranks_a_same_named_builtin_for_a_bare_claim() {
    let tmp = tempfile::tempdir().unwrap();
    // `Vector3` is uniquely geometry_msgs among built-ins; the store adds
    // its own differently-shaped Vector3 in another package.
    write_store_msg(tmp.path(), "my_pkg", "Vector3", "float64 x\nfloat64 y\n");

    let reg = FieldRegistry::from_graph(&one_output_graph("Vector3"), tmp.path());

    let store_expected = {
        let mut s = MessageSchema::new_in_package("Vector3", "my_pkg");
        s.add_field(FieldDef::new("x", FieldType::F64));
        s.add_field(FieldDef::new("y", FieldType::F64));
        oracle_hash(&mut [s], "my_pkg/Vector3")
    };
    assert_eq!(
        reg.expected_schema_hash(TOPIC),
        Some(store_expected),
        "the store tier outranks the built-in tier for a bare claim"
    );
}

/// A store schema REDEFINING a built-in's qualified name wins (the
/// shadow semantics): the expected hash for `geometry_msgs/Vector3` is the
/// STORE copy's, not the built-in's. (Without the shadow rule the built-in resolves.)
#[test]
fn s3_a_store_shadow_of_a_builtin_qualified_name_wins() {
    let tmp = tempfile::tempdir().unwrap();
    // Deliberately a DIFFERENT layout from the real geometry_msgs/Vector3
    // (x/y/z f64), so the two hashes cannot coincide.
    write_store_msg(
        tmp.path(),
        "geometry_msgs",
        "Vector3",
        "float64 x\nfloat64 y\nfloat64 z\nfloat64 w\n",
    );

    let reg = FieldRegistry::from_graph(&one_output_graph("geometry_msgs/Vector3"), tmp.path());

    let store_expected = {
        let mut s = MessageSchema::new_in_package("Vector3", "geometry_msgs");
        for f in ["x", "y", "z", "w"] {
            s.add_field(FieldDef::new(f, FieldType::F64));
        }
        oracle_hash(&mut [s], "geometry_msgs/Vector3")
    };
    assert_eq!(
        reg.expected_schema_hash(TOPIC),
        Some(store_expected),
        "the store copy must shadow the built-in under its qualified name"
    );
    // And the built-in copy is genuinely different (else this pins nothing).
    let builtin_expected = {
        let mut s = MessageSchema::new_in_package("Vector3", "geometry_msgs");
        for f in ["x", "y", "z"] {
            s.add_field(FieldDef::new(f, FieldType::F64));
        }
        oracle_hash(&mut [s], "geometry_msgs/Vector3")
    };
    assert_ne!(store_expected, builtin_expected, "oracle sanity");
}

/// Hostile declarations degrade the affected schema only — never a panic,
/// and a sane store-typed sibling topic still resolves.
///
/// The test oracle: asserting ONLY the sane sibling would let a
/// registry that left `HostileM`/`HostileY` reachable pass here and panic
/// later, when a tolerance run materializes the hostile layout. The
/// hostile roots are therefore exercised THEMSELVES: each degrades to
/// schema-unavailable — no expected hash, no decoder, and field
/// resolution errs CLEANLY (`schema_unavailable`, never a panic).
#[test]
fn s3_hostile_declarations_never_panic_the_registry() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(
        tmp.path(),
        "nav_pkg",
        "HostileM",
        "float64[2305843009213693952] a\n",
    );
    write_yaml(tmp.path(), "hostile.yaml", &hostile_yaml());
    write_store_msg(tmp.path(), "nav_pkg", "Goal", GOAL_MSG);

    let reg = FieldRegistry::from_graph(&one_output_graph("nav_pkg/Goal"), tmp.path());

    let expected = oracle_hash(&mut [goal_store_ir()], "nav_pkg/Goal");
    assert_eq!(reg.expected_schema_hash(TOPIC), Some(expected));

    // The hostile roots themselves — both the store and the YAML spelling —
    // degrade instead of staying reachable-and-armed.
    for hostile in ["nav_pkg/HostileM", "HostileY"] {
        let hreg = FieldRegistry::from_graph(&one_output_graph(hostile), tmp.path());
        // PREMISE (the vacuous-guard class): every
        // degradation assert below is ALSO satisfied by a topic the
        // registry never registered at all, so first prove the hostile
        // graph output IS a known Produced topic — the asserts then pin
        // degradation, not disappearance.
        assert_eq!(
            hreg.topic_class(TOPIC),
            Some(&TopicClass::Produced),
            "'{hostile}': the hostile output stays a REGISTERED Produced topic"
        );
        assert_eq!(
            hreg.expected_schema_hash(TOPIC),
            None,
            "'{hostile}': a filtered hostile root serves NO expected hash"
        );
        assert!(
            hreg.decoder_for(TOPIC).is_none(),
            "'{hostile}': no decoder — nothing left to materialize its layout"
        );
        let err = hreg.resolve_field(TOPIC, "a").unwrap_err();
        assert!(
            err.schema_unavailable,
            "'{hostile}': field resolution is a clean schema-unavailable \
             refusal, never a panic"
        );
    }
}

/// Two registries over one workspace + graph agree on every topic's expected
/// hash (Principle #7).
#[test]
fn s3_resolution_is_deterministic_across_builds() {
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(
        tmp.path(),
        "goal.yaml",
        "schemas:\n  Goal:\n    fields:\n      uint32 a:\n",
    );
    write_store_msg(tmp.path(), "nav_pkg", "Goal", GOAL_MSG);

    let config = one_output_graph("nav_pkg/Goal");
    let a = FieldRegistry::from_graph(&config, tmp.path());
    let b = FieldRegistry::from_graph(&config, tmp.path());
    assert_eq!(a.expected_schema_hash(TOPIC), b.expected_schema_hash(TOPIC));
    assert!(a.expected_schema_hash(TOPIC).is_some(), "and it resolves");
}

// ===========================================================================
// The composed-overflow scrub, the scaffold gate, YAML shadows
// ===========================================================================

/// The replay field registry: a composed overflow — every
/// schema individually sizable, the overflow appearing only after
/// fixed-nested inlining (Inner's 8 x 536870907 bytes fit the u32 wire
/// ceiling; Outer = Inner[2^33] does not fit usize) — must be scrubbed from
/// EVERY registry
/// structure, not just the local hash clone. If the schema stays in
/// `FieldRegistry.schemas`, materializing its layout here PANICS
/// instead of degrading the topic to schema-unavailable.
#[test]
fn s3_a_composed_overflow_schema_is_scrubbed_everywhere_never_a_panic() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "nav_pkg", "Inner", "float64[536870907] v\n");
    write_store_msg(tmp.path(), "nav_pkg", "Outer", "Inner[8589934592] arr\n");
    write_store_msg(tmp.path(), "nav_pkg", "Goal", GOAL_MSG);

    let reg = FieldRegistry::from_graph(&one_output_graph("nav_pkg/Outer"), tmp.path());

    // The composed-overflow topic degrades to schema-unavailable...
    // (premise first: registered, not merely unknown)
    assert_eq!(reg.topic_class(TOPIC), Some(&TopicClass::Produced));
    assert_eq!(reg.expected_schema_hash(TOPIC), None, "no expected hash");
    assert!(reg.decoder_for(TOPIC).is_none(), "no decoder");
    // ...and field resolution ERRS instead of panicking (the key one: this
    // call materializes the layout, which otherwise blows up in layout math).
    assert!(reg.resolve_field(TOPIC, "arr").is_err(), "clean refusal");

    // The sane sibling still resolves fully — one bomb never sinks the set.
    let sib = FieldRegistry::from_graph(&one_output_graph("nav_pkg/Goal"), tmp.path());
    let expected = oracle_hash(&mut [goal_store_ir()], "nav_pkg/Goal");
    assert_eq!(sib.expected_schema_hash(TOPIC), Some(expected));
    assert!(sib.resolve_field(TOPIC, "x").is_ok());
}

/// The scaffold gate: `node create` must refuse a `.msg`
/// store-typed port — bare and qualified spellings — BEFORE any filesystem
/// mutation. A store type has no generated Rust type (`schema_to_import`
/// maps every qualified name into `native_ros2_messages`, which holds only
/// the built-in corpus), so the scaffolded crate could never compile.
/// Without the gate the bare spelling resolves via the store tier and mints
/// exactly that broken crate.
#[test]
fn s4_node_create_refuses_a_store_typed_port_before_any_write() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = cerulion_cli_engine::workspace::workspace_create(tmp.path(), "ws").unwrap();
    let cargo_toml = ws.root.join("Cargo.toml");
    let msg_dir = ws.schemas_dir.join("acme").join("msg");
    std::fs::create_dir_all(&msg_dir).unwrap();
    std::fs::write(msg_dir.join("WidgetState.msg"), "uint32 a\n").unwrap();

    for spelling in ["WidgetState", "acme/WidgetState", "acme::WidgetState"] {
        let options = cerulion_cli_engine::node_cmd::NodeCreateOptions {
            outputs: vec![(spelling.to_string(), "out".to_string())],
            ..Default::default()
        };
        let err = cerulion_cli_engine::node_cmd::node_create_with_options(
            &ws.nodes_dir,
            &cargo_toml,
            "camera",
            Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }),
            &options,
        )
        .expect_err("a store-typed port must be refused");
        let msg = err.to_string();
        assert!(
            msg.contains("acme/WidgetState")
                && msg.contains("no generated Rust type")
                && msg.contains("cerulion schema create"),
            "'{spelling}': the refusal names the store type + the remedy: {msg}"
        );
        assert!(
            !ws.nodes_dir.join("camera").exists(),
            "'{spelling}': no node directory may be created"
        );
    }
}

/// The `node modify` twin: a store-typed port is refused with the node's
/// `lib.rs` BYTE-untouched, for the bare and qualified spellings — and the
/// control half proves a built-in port still scaffolds (the refusal is
/// scoped to store types, not the verb).
#[test]
fn s4_node_modify_refuses_a_store_typed_port_lib_rs_untouched() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = cerulion_cli_engine::workspace::workspace_create(tmp.path(), "ws").unwrap();
    let cargo_toml = ws.root.join("Cargo.toml");
    cerulion_cli_engine::node_cmd::node_create(&ws.nodes_dir, &cargo_toml, "camera", None).unwrap();
    let msg_dir = ws.schemas_dir.join("acme").join("msg");
    std::fs::create_dir_all(&msg_dir).unwrap();
    std::fs::write(msg_dir.join("WidgetState.msg"), "uint32 a\n").unwrap();
    let lib_rs = ws.nodes_dir.join("camera").join("src").join("lib.rs");
    let before = std::fs::read(&lib_rs).unwrap();

    for spelling in ["WidgetState", "acme/WidgetState"] {
        let err = cerulion_cli_engine::node_cmd::node_modify_add_port(
            &ws.nodes_dir,
            "camera",
            "st",
            Some(spelling),
            true,
            false,
        )
        .expect_err("a store-typed port must be refused");
        assert!(
            err.to_string().contains("acme/WidgetState"),
            "the refusal names the store type: {err}"
        );
        assert_eq!(
            std::fs::read(&lib_rs).unwrap(),
            before,
            "'{spelling}': lib.rs must stay byte-untouched"
        );
    }

    // CONTROL: a built-in port still scaffolds — the refusal is scoped to
    // store types, not the verb.
    cerulion_cli_engine::node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "camera",
        "vec",
        Some("geometry_msgs/Vector3"),
        true,
        false,
    )
    .expect("a built-in port must still scaffold");
    let after = std::fs::read_to_string(&lib_rs).unwrap();
    assert!(
        after.contains("geometry_msgs::Vector3"),
        "the built-in import landed: {after}"
    );
}

/// The registry's bare index beside a composed-overflow `p1/Type` (dropped
/// by the post-resolve scrub, unservable) and a valid `p2/Type`: the
/// scrubbed definition still exists ON DISK, so port resolution sees two
/// bearers of bare `Type` and REFUSES the name — replay reaches the same
/// verdict rather than picking `p2/Type` (a sibling pick validation forbids).
/// The sibling stays untouched under its own QUALIFIED name, which is what
/// "not poisoned" means.
#[test]
fn s3_a_scrubbed_composed_overflow_does_not_poison_a_siblings_bare_claim() {
    let tmp = tempfile::tempdir().unwrap();
    // p1/Type is the composed bomb (Inner representable at ~4 GiB; Type =
    // Inner[2^33] overflows only after inlining); p2/Type is valid.
    write_store_msg(tmp.path(), "p1", "Inner", "float64[536870907] v\n");
    write_store_msg(tmp.path(), "p1", "Type", "Inner[8589934592] arr\n");
    write_store_msg(tmp.path(), "p2", "Type", "uint32 a\n");

    // The scrubbed `p1/Type` still exists ON DISK, so port resolution sees two
    // bearers of bare `Type` and REFUSES the name — a graph declaring it
    // never validates, and replay picking `p2/Type` for it would be a sibling
    // pick validation forbids (the exact hostile-pair fixture). Replay
    // reaches validation's verdict; the sibling is untouched under its own
    // QUALIFIED name, which is what "not poisoned" actually means.
    let err =
        cerulion_cli_engine::schema_cmd::port_schema_exists(&tmp.path().join("schemas"), "Type")
            .expect_err("PREMISE: bare `Type` is ambiguous at validation");
    assert!(err.to_string().contains("ambiguous"), "got: {err}");
    let reg = FieldRegistry::from_graph(&one_output_graph("Type"), tmp.path());
    assert_eq!(
        reg.topic_class(TOPIC),
        Some(&TopicClass::Produced),
        "registered"
    );
    assert_eq!(
        reg.expected_schema_hash(TOPIC),
        None,
        "replay never picks the surviving sibling for a name validation refuses"
    );
    assert!(reg.decoder_for(TOPIC).is_none());

    let expected = {
        let mut s = MessageSchema::new_in_package("Type", "p2");
        s.add_field(FieldDef::new("a", FieldType::U32));
        oracle_hash(&mut [s], "p2/Type")
    };
    let by_q = FieldRegistry::from_graph(&one_output_graph("p2/Type"), tmp.path());
    assert_eq!(
        by_q.expected_schema_hash(TOPIC),
        Some(expected),
        "the sibling is not poisoned under its qualified name"
    );
    assert!(by_q.resolve_field(TOPIC, "a").is_ok());
}

/// A slash-named workspace YAML entry
/// (`schemas:` key `geometry_msgs/Vector3` — a supported shape: `schema
/// info` and `port_schema_exists` both resolve them) must WIN its own
/// qualified name over the same-named BUILT-IN at replay. A
/// combined vec keeping both definitions would let both winner rules
/// (`compute_schema_hashes`' collect and `LayoutResolver`'s
/// `by_qualified`) pick the LATER one — the built-in — inverting the
/// documented workspace-first ladder for both the expected hash AND the
/// materialized layout.
#[test]
fn s3_a_yaml_entry_shadowing_a_builtin_qualified_name_wins_at_replay() {
    let tmp = tempfile::tempdir().unwrap();
    // Deliberately a DIFFERENT layout from the real geometry_msgs/Vector3
    // (x/y/z f64) so neither the hash nor the field set can coincide.
    write_yaml(
        tmp.path(),
        "vec.yaml",
        "schemas:\n  geometry_msgs/Vector3:\n    fields:\n      float64 x:\n",
    );

    let reg = FieldRegistry::from_graph(&one_output_graph("geometry_msgs/Vector3"), tmp.path());

    // Hash half: the YAML copy's hash (package-less IR whose NAME carries
    // the slash — exactly what `parse_message_schemas` produces).
    let yaml_expected = {
        let mut s = MessageSchema::new("geometry_msgs/Vector3");
        s.add_field(FieldDef::new("x", FieldType::F64));
        oracle_hash(&mut [s], "geometry_msgs/Vector3")
    };
    let builtin_expected = {
        let mut s = MessageSchema::new_in_package("Vector3", "geometry_msgs");
        for f in ["x", "y", "z"] {
            s.add_field(FieldDef::new(f, FieldType::F64));
        }
        oracle_hash(&mut [s], "geometry_msgs/Vector3")
    };
    assert_ne!(yaml_expected, builtin_expected, "oracle sanity");
    assert_eq!(
        reg.expected_schema_hash(TOPIC),
        Some(yaml_expected),
        "the workspace YAML entry wins its qualified name over the built-in"
    );
    // Layout half: the MATERIALIZED layout is the YAML one — `y` (a real
    // built-in Vector3 field) must NOT resolve. A built-in-last winner rule
    // would serve the built-in layout and `y` would resolve.
    assert!(reg.resolve_field(TOPIC, "x").is_ok());
    assert!(
        reg.resolve_field(TOPIC, "y").is_err(),
        "the built-in's `y` must not resolve through the YAML shadow"
    );
}

/// The YAML-over-STORE half of the same rule: a slash-named YAML
/// entry claiming a `.msg` store type's qualified name wins it (the
/// documented workspace-YAML-first rule). A STORE copy that came
/// later in the combined vec would take both the hash and the layout.
#[test]
fn s3_a_yaml_entry_shadowing_a_store_qualified_name_wins_at_replay() {
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(
        tmp.path(),
        "widget.yaml",
        "schemas:\n  acme/Widget:\n    fields:\n      uint32 a:\n",
    );
    write_store_msg(tmp.path(), "acme", "Widget", "float64 x\nfloat64 y\n");

    let reg = FieldRegistry::from_graph(&one_output_graph("acme/Widget"), tmp.path());

    let yaml_expected = {
        let mut s = MessageSchema::new("acme/Widget");
        s.add_field(FieldDef::new("a", FieldType::U32));
        oracle_hash(&mut [s], "acme/Widget")
    };
    let store_expected = {
        let mut s = MessageSchema::new_in_package("Widget", "acme");
        s.add_field(FieldDef::new("x", FieldType::F64));
        s.add_field(FieldDef::new("y", FieldType::F64));
        oracle_hash(&mut [s], "acme/Widget")
    };
    assert_ne!(yaml_expected, store_expected, "oracle sanity");
    assert_eq!(
        reg.expected_schema_hash(TOPIC),
        Some(yaml_expected),
        "the workspace YAML entry wins its qualified name over the store copy"
    );
    assert!(reg.resolve_field(TOPIC, "a").is_ok());
    assert!(
        reg.resolve_field(TOPIC, "x").is_err(),
        "the store copy's `x` must not resolve through the YAML shadow"
    );
}

/// The BARE shadow regression alongside the
/// qualified ones: a bare-named YAML entry sharing a built-in's short name
/// resolves a bare graph `schema:` to the YAML copy (distinct qualified
/// names — `Image` vs `sensor_msgs/Image` — so this rides the exact-match
/// tier; pinned so the tier order can never regress it).
#[test]
fn s3_a_bare_yaml_name_sharing_a_builtin_short_name_resolves_to_yaml() {
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(
        tmp.path(),
        "image.yaml",
        "schemas:\n  Image:\n    fields:\n      uint32 a:\n",
    );

    let reg = FieldRegistry::from_graph(&one_output_graph("Image"), tmp.path());

    let yaml_expected = {
        let mut s = MessageSchema::new("Image");
        s.add_field(FieldDef::new("a", FieldType::U32));
        oracle_hash(&mut [s], "Image")
    };
    assert_eq!(reg.expected_schema_hash(TOPIC), Some(yaml_expected));
    assert!(reg.resolve_field(TOPIC, "a").is_ok());
    assert!(
        reg.resolve_field(TOPIC, "height").is_err(),
        "the built-in Image's `height` must not resolve on the YAML schema"
    );
}

// ===========================================================================
// Qualified twins, the post-scrub set, nested targets, the closure walk
// ===========================================================================

/// The precedence rule, pinned here: a
/// QUALIFIED name BOTH workspace YAML and the `.msg` store define is
/// REFUSED — on all three resolution surfaces, in ONE message naming both
/// sources — never won by a precedence. Giving the collision to the
/// YAML tier would spare the scaffold gate refusing a port the workspace
/// owns, but the ambiguity-refusal rule says no tier outranks another for
/// a spelling you NAME, and the one workspace lookup applies it here.
///
/// The three surfaces are asserted in ONE test so they can never diverge:
/// `resolve_port_schema`, `schema_info_unified`, `port_schema_exists`.
/// Both workspace tiers are exercised: a slash-named ENTRY key in a flat
/// file (`acme/Widget`) and a nested FILE STEM (`schemas/acme2/Widget2.yaml`).
///
/// The two CONTROLS are what keep the refusal scoped, and both
/// pin the unclaimed case: a store type NO YAML tier claims
/// still classifies `QualifiedStore` (and the scaffold gate refuses it,
/// with the store's own remedy), and a YAML type NO store copies still
/// classifies `Qualified` (and scaffolds).
#[test]
fn r4_a_qualified_name_both_tiers_define_is_refused_on_all_three_resolution_surfaces() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");

    // Tier A: slash-named entry key in a flat YAML file + colliding store type.
    write_yaml(
        tmp.path(),
        "widget.yaml",
        "schemas:\n  acme/Widget:\n    fields:\n      uint32 a:\n",
    );
    write_store_msg(tmp.path(), "acme", "Widget", "float64 x\nfloat64 y\n");

    // Tier B: nested file stem + colliding store type.
    let nested_dir = schemas_dir.join("acme2");
    std::fs::create_dir_all(&nested_dir).unwrap();
    std::fs::write(
        nested_dir.join("Widget2.yaml"),
        "schemas:\n  acme2/Widget2:\n    fields:\n      uint32 a:\n",
    )
    .unwrap();
    write_store_msg(tmp.path(), "acme2", "Widget2", "float64 x\n");

    for (raw, qualified, yaml_source, store_source) in [
        (
            "acme/Widget",
            "acme/Widget",
            "schemas/widget.yaml (entry acme/Widget)",
            "schemas/acme/msg/Widget.msg (store)",
        ),
        (
            "acme::Widget",
            "acme/Widget",
            "schemas/widget.yaml (entry acme/Widget)",
            "schemas/acme/msg/Widget.msg (store)",
        ),
        (
            "acme2/Widget2",
            "acme2/Widget2",
            "schemas/acme2/Widget2.yaml (file stem; entries: acme2/Widget2)",
            "schemas/acme2/msg/Widget2.msg (store)",
        ),
    ] {
        // Surface 1: resolve_port_schema — the refusal, naming both sources.
        let err = cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, raw)
            .expect_err("a qualified twin is refused")
            .to_string();
        for needle in [
            &format!("'{qualified}' is ambiguous in this workspace — defined by:"),
            yaml_source,
            store_source,
        ] {
            assert!(
                err.contains(needle),
                "'{raw}': carries {needle:?}; got: {err}"
            );
        }
        // Surface 2: schema_info_unified — the SAME message, verbatim.
        assert_eq!(
            cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, raw)
                .expect_err("schema info refuses it too")
                .to_string(),
            err,
            "'{raw}': one condition, one message"
        );
        // Surface 3: port_schema_exists — identical, never a silent `false`.
        assert_eq!(
            cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, raw)
                .expect_err("the graph gate refuses it identically")
                .to_string(),
            err,
            "'{raw}': the gate agrees with the verb"
        );
    }

    // CONTROL 1: a store type NO YAML tier claims still classifies
    // `QualifiedStore` (the scaffold gate refuses it with the STORE's
    // remedy — a store type has no generated Rust type).
    write_store_msg(tmp.path(), "acme3", "Gadget", "uint32 a\n");
    let store_only =
        cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "acme3/Gadget").unwrap();
    assert_eq!(
        store_only.provenance,
        cerulion_cli_engine::schema_cmd::PortSchemaProvenance::QualifiedStore,
        "no collision: the store hit still classifies"
    );
    assert!(
        cerulion_cli_engine::schema_cmd::refuse_store_port_scaffold(&store_only, "acme3/Gadget")
            .is_err(),
        "and the scaffold gate still refuses a store-typed port"
    );

    // CONTROL 2: a YAML type NO store copies still classifies `Qualified`
    // and scaffolds — the refusal is scoped to genuine collisions.
    write_yaml(
        tmp.path(),
        "solo.yaml",
        "schemas:\n  acme4/Solo:\n    fields:\n      uint32 a:\n",
    );
    let yaml_only =
        cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "acme4/Solo").unwrap();
    assert_eq!(
        yaml_only.provenance,
        cerulion_cli_engine::schema_cmd::PortSchemaProvenance::Qualified,
        "no collision: the workspace definition classifies and scaffolds"
    );
    cerulion_cli_engine::schema_cmd::refuse_store_port_scaffold(&yaml_only, "acme4/Solo")
        .expect("the scaffold gate accepts a workspace-owned qualified name");
    assert!(
        cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "acme4/Solo").unwrap()
    );
}

/// Hash/layout divergence: dropping a composed-overflow
/// schema can FLIP a bare reference from ambiguous (two global candidates)
/// to resolved (one survivor) — so a survivor hashed BEFORE the scrub
/// carries an expected hash computed with the ref variable while
/// `decoder_for`'s `LayoutResolver` sees ONE candidate and INLINES it:
/// the expected hash passes while replay decodes different offsets.
/// The registry recomputes hashes over the post-scrub set until
/// nothing more drops, so the hash map and every materialized layout
/// derive from the SAME set.
///
/// Shape: `p1/Inner` (composed past the u32 wire ceiling — `p1/Filler` is
/// representable at 8·536870907 bytes, seven under the ceiling `u32::MAX − 32`; four
/// inlined are not) and `p2/Inner`
/// (sane) are the two global bare candidates for `p3/User`'s bare `Inner`
/// ref (same-package `p3/Inner` does not exist, so the resolver's
/// global-bare arm decides).
///
/// `p1/Inner` carries a trailing `string s`, DELIBERATELY: that makes it
/// VARIABLE, so the composed preflight (which drops only recursively-FIXED
/// overflows — the resolver never sizes a variable schema) keeps it, and
/// the drop happens in `compute_schema_hashes`' POST-RESOLVE retain — the
/// arm whose ambiguity flip only the outer recompute loop can heal. (The
/// FIXED flavor of the same shape is the nested-target test's Wrapper pin: there the
/// preflight itself drops the bomb before anything is hashed.)
#[test]
fn r4_the_hash_map_and_decoder_layouts_derive_from_the_same_post_scrub_set() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "p1", "Filler", "float64[536870907] v\n");
    write_store_msg(tmp.path(), "p1", "Inner", "Filler[4] a\nstring s\n");
    write_store_msg(tmp.path(), "p2", "Inner", "float64 x\n");
    write_store_msg(tmp.path(), "p3", "User", "Inner val\n");

    let reg = FieldRegistry::from_graph(&one_output_graph("p3/User"), tmp.path());

    let user_ir = || {
        let mut s = MessageSchema::new_in_package("User", "p3");
        s.add_field(FieldDef::new(
            "val",
            FieldType::Nested {
                schema_name: "Inner".to_string(),
                package: None,
                fixed: None,
            },
        ));
        s
    };
    let p2_inner_ir = || {
        let mut s = MessageSchema::new_in_package("Inner", "p2");
        s.add_field(FieldDef::new("x", FieldType::F64));
        s
    };
    // POST-scrub oracle: `p1/Inner` gone, bare `Inner` unique → the
    // resolver INLINES `p2/Inner` into `User`.
    let post_scrub_expected = oracle_hash(&mut [p2_inner_ir(), user_ir()], "p3/User");
    // PRE-scrub oracle: both candidates present, bare `Inner` ambiguous →
    // the ref stays variable. (`p1/Inner` is in the set but never
    // materialized — nothing resolves TO it — so hashing `User` is safe.)
    let pre_scrub_stale = {
        let p1_filler = {
            let mut s = MessageSchema::new_in_package("Filler", "p1");
            s.add_field(FieldDef::new(
                "v",
                FieldType::FixedArray {
                    element_type: Box::new(FieldType::F64),
                    length: 536870907,
                },
            ));
            s
        };
        let p1_inner = {
            let mut s = MessageSchema::new_in_package("Inner", "p1");
            s.add_field(FieldDef::new(
                "a",
                FieldType::FixedArray {
                    element_type: Box::new(FieldType::Nested {
                        schema_name: "Filler".to_string(),
                        package: None,
                        fixed: None,
                    }),
                    length: 4,
                },
            ));
            s.add_field(FieldDef::new("s", FieldType::String));
            s
        };
        oracle_hash(
            &mut [p1_filler, p1_inner, p2_inner_ir(), user_ir()],
            "p3/User",
        )
    };
    assert_ne!(
        post_scrub_expected, pre_scrub_stale,
        "the scrub must change the survivor's resolution — else this pins nothing"
    );
    assert_eq!(
        reg.expected_schema_hash(TOPIC),
        Some(post_scrub_expected),
        "the expected hash derives from the SAME post-scrub set the decoder's \
         LayoutResolver materializes layouts from (never the stale pre-scrub hash)"
    );
    // And the materialized layout agrees: the inlined p2/Inner's field
    // resolves through the ref.
    assert!(reg.resolve_field(TOPIC, "val.x").is_ok());
}

/// The panic: a composed-overflow schema used as a NESTED
/// TARGET panics inside `resolve_fixed_nested` ITSELF (`resolved_fixed_of`
/// sizes each fixed target via the panicking `wire_fixed_size`) — BEFORE
/// the post-resolve retain can drop anything, so the composed-overflow
/// scrub never runs. The Inner/Outer pair from that scrub's test arms the panic the moment a
/// third schema references `Outer`. The registry PREFLIGHTS
/// composed fixed-array sizes (`retain_composed_sizable_schemas`) before
/// resolution: the hostile root degrades to schema-unavailable, the
/// referencing schema survives with the ref conservatively variable, and
/// the sane sibling is untouched.
#[test]
fn r4_a_composed_overflow_nested_target_degrades_instead_of_panicking_resolution() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "nav_pkg", "Inner", "float64[536870907] v\n");
    write_store_msg(tmp.path(), "nav_pkg", "Outer", "Inner[8589934592] arr\n");
    // The arming reference: resolving Wrapper materializes Outer as a
    // fixed nested target — the exact site that panics without the preflight.
    write_store_msg(tmp.path(), "nav_pkg", "Wrapper", "Outer o\nfloat64 w\n");
    write_store_msg(tmp.path(), "nav_pkg", "Goal", GOAL_MSG);

    // Without the preflight this call PANICS inside resolve_fixed_nested.
    let reg = FieldRegistry::from_graph(&one_output_graph("nav_pkg/Outer"), tmp.path());

    // The hostile root degrades to schema-unavailable — PREMISE first
    // (the vacuous-guard class): it stays a REGISTERED Produced topic, so the Nones
    // below pin degradation, not an unknown topic.
    assert_eq!(reg.topic_class(TOPIC), Some(&TopicClass::Produced));
    assert_eq!(reg.expected_schema_hash(TOPIC), None, "no expected hash");
    assert!(reg.decoder_for(TOPIC).is_none(), "no decoder");
    let err = reg.resolve_field(TOPIC, "arr").unwrap_err();
    assert!(err.schema_unavailable, "clean schema-unavailable refusal");

    // The REFERENCING schema survives — its ref to the dropped Outer is
    // conservatively variable, exactly what the resolver produces over the
    // post-preflight set.
    let wrapper = FieldRegistry::from_graph(&one_output_graph("nav_pkg/Wrapper"), tmp.path());
    let wrapper_expected = {
        let mut s = MessageSchema::new_in_package("Wrapper", "nav_pkg");
        s.add_field(FieldDef::new(
            "o",
            FieldType::Nested {
                schema_name: "Outer".to_string(),
                package: None,
                fixed: None,
            },
        ));
        s.add_field(FieldDef::new("w", FieldType::F64));
        // Resolved over a set WITHOUT Outer: the ref stays variable.
        oracle_hash(&mut [s], "nav_pkg/Wrapper")
    };
    assert_eq!(
        wrapper.expected_schema_hash(TOPIC),
        Some(wrapper_expected),
        "the referencing schema survives with the ref variable"
    );

    // The sane sibling still resolves fully.
    let sib = FieldRegistry::from_graph(&one_output_graph("nav_pkg/Goal"), tmp.path());
    let expected = oracle_hash(&mut [goal_store_ir()], "nav_pkg/Goal");
    assert_eq!(sib.expected_schema_hash(TOPIC), Some(expected));
    assert!(sib.resolve_field(TOPIC, "x").is_ok());
}

/// The closure walk: with TWO packages defining `Inner`, a bare
/// ref inside `p1/Outer` binds SAME-PACKAGE-first (`p1/Inner`) at layout
/// time — so the divergence-map closure walk must recurse into `p1/Inner`
/// too. Keying bare refs on a flat last-wins alias (here
/// `p2/Inner`, whose closure escapes the workspace via `std_msgs/Header`)
/// would silently omit the perfectly hashable `p1/Outer` from the map
/// and skip its divergence check.
#[test]
fn r4_the_closure_walk_binds_a_bare_ref_same_package_first() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "p1", "Inner", "float64 v\n");
    write_store_msg(tmp.path(), "p1", "Outer", "Inner i\n");
    // Sorts LAST among the three (p1/Inner < p1/Outer < p2/Inner), so the
    // flat alias would hand `Inner` to this escaping copy.
    write_store_msg(tmp.path(), "p2", "Inner", "std_msgs/Header h\nfloat64 z\n");

    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());

    let expected = {
        let inner = {
            let mut s = MessageSchema::new_in_package("Inner", "p1");
            s.add_field(FieldDef::new("v", FieldType::F64));
            s
        };
        let outer = {
            let mut s = MessageSchema::new_in_package("Outer", "p1");
            s.add_field(FieldDef::new(
                "i",
                FieldType::Nested {
                    schema_name: "Inner".to_string(),
                    package: None,
                    fixed: None,
                },
            ));
            s
        };
        oracle_hash(&mut [inner, outer], "p1/Outer")
    };
    assert_eq!(
        map.get("p1/Outer").copied(),
        Some(expected),
        "the closure walk binds `Inner` same-package-first (p1/Inner), so \
         p1/Outer is hashable — a flat bare alias would walk p2/Inner \
         and omit it; got {map:?}"
    );
    assert!(
        map.contains_key("p1/Inner"),
        "the clean same-package target hashes too"
    );
    // `p2/Inner` does not "escape" — built-ins are in the
    // resolution set, so it hashes too; what this test pins is that
    // `p1/Outer` bound the SAME-PACKAGE `p1/Inner` (the oracle above), not
    // that the other copy vanished.
    assert!(
        map.contains_key("p2/Inner"),
        "the other package's copy hashes on its own; got {map:?}"
    );
}

// ===========================================================================
// Shadowed-duplicate eviction and its inverse control
// ===========================================================================

/// Shadowed-duplicate eviction: one qualified name, TWO
/// definitions — a composed-overflowing `.msg` store `nav_pkg/X` and a
/// VALID slash-named workspace YAML `nav_pkg/X` entry (distinct resolver
/// keys, both live; the YAML copy is the later-wins winner every map
/// consumer serves). A preflight that classifies the store loser and
/// then removes BY NAME throws the valid YAML winner away with it
/// and the graph hash map silently loses the schema. The removal
/// is INDEX-precise: the overflowing store definition goes, the YAML
/// winner survives and hashes.
#[test]
fn r5_a_shadowed_composed_overflow_never_evicts_its_valid_yaml_winner() {
    let tmp = tempfile::tempdir().unwrap();
    // The ingredient must be REPRESENTABLE (seven bytes under the
    // wire ceiling) — the hash map applies the wire ceiling at the
    // declared moment, before the composed probe, so a 2^64 − 8 byte
    // `Filler` would be refused outright and never reach the probe this
    // test pins; the composed ARITHMETIC overflow rides the multiplier.
    write_store_msg(tmp.path(), "nav_pkg", "Filler", "float64[536870907] v\n");
    write_store_msg(tmp.path(), "nav_pkg", "X", "Filler[8589934592] arr\n");
    write_yaml(
        tmp.path(),
        "x_yaml.yaml",
        "schemas:\n  nav_pkg/X:\n    fields:\n      uint32 a:\n",
    );

    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());

    let yaml_expected = {
        let mut s = MessageSchema::new("nav_pkg/X");
        s.add_field(FieldDef::new("a", FieldType::U32));
        oracle_hash(&mut [s], "nav_pkg/X")
    };
    assert_eq!(
        map.get("nav_pkg/X").copied(),
        Some(yaml_expected),
        "the valid YAML winner survives its shadowed overflowing store \
         loser and serves the name (a name-keyed removal would evict \
         both and the map would lose the schema); got {map:?}"
    );
    assert!(
        map.contains_key("nav_pkg/Filler"),
        "the individually-sizable Filler is untouched"
    );
}

/// The INVERSE control (no silent resurrection): when the name's WINNING
/// definition is the overflowing one — here the later-wins YAML
/// `nav_pkg/Y`, composed-overflowing through two nested refs to the huge
/// store `Filler` — the name yields NO entry: the valid shadowed STORE
/// copy must not silently become the served definition of a name whose
/// winner was dropped as hostile.
#[test]
fn r5_a_hostile_winner_yields_no_entry_never_a_resurrected_loser() {
    let tmp = tempfile::tempdir().unwrap();
    // A REPRESENTABLE ingredient (see the sibling test); the YAML
    // winner's composed ARITHMETIC overflow — the composed probe's verdict,
    // which this test pins — is 4096 capped `Filler[1048576]` fields
    // (4096 × 2^20 × ~2^32 bytes ≈ 2^64: the checked recipe overflows,
    // exactly the shape this test needs under the wire ceiling).
    write_store_msg(tmp.path(), "nav_pkg", "Filler", "float64[536870907] v\n");
    // The shadowed-but-valid store copy of the same qualified name.
    write_store_msg(tmp.path(), "nav_pkg", "Y", "uint32 a\n");
    // The later-wins YAML winner: the inlined Fillers overflow the composed
    // fixed section's arithmetic (each fits usize alone; their sum does not).
    let mut y = String::from("schemas:\n  nav_pkg/Y:\n    fields:\n");
    for i in 0..4096 {
        y.push_str(&format!("      Filler[1048576] f{i}:\n"));
    }
    write_yaml(tmp.path(), "y_yaml.yaml", &y);
    // A sane YAML sibling proves the YAML tier was
    // PROCESSED — without it a fold that skipped every YAML schema passed
    // the omission below.
    write_yaml(
        tmp.path(),
        "ok_yaml.yaml",
        "schemas:\n  nav_pkg/Ok:\n    fields:\n      uint32 a:\n",
    );

    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());

    assert!(
        map.contains_key("nav_pkg/Ok"),
        "the sane YAML sibling is hashed — the YAML tier ran: {map:?}"
    );
    assert!(
        !map.contains_key("nav_pkg/Y"),
        "a hostile WINNER takes its name with it — the valid shadowed \
         store copy must not be silently served in its place; got {map:?}"
    );
    assert!(
        map.contains_key("nav_pkg/Filler"),
        "the individually-sizable Filler is untouched"
    );
}

// ===========================================================================
// A failing later-wins winner at the post-resolve leg
// ===========================================================================

/// The mirror image of the hostile-winner case above, at the
/// POST-RESOLVE leg: the name's later-wins WINNER (`z_dup.yaml`'s `Dup` —
/// VARIABLE, so the composed preflight rightly keeps it, with a fixed
/// section that lands past the u32 wire ceiling only after resolution
/// inlines two ~4 GiB store `Filler`s) is dropped by the post-resolve
/// sizing pass while an EARLIER
/// valid bearer (`a_dup.yaml`'s `Dup`) survives. A
/// fully-removed-only filter would then suppress the name from the returned
/// scrub list — correct for the tier scrub, but it would MASK the partial
/// removal from the registry's own state: `self.schemas` would keep the rejected
/// winner, and `decoder_for`'s `layout_of` would materialize it and PANIC.
///
/// The invariant enforced (shared winner-sweep policy at BOTH legs): a
/// qualified name is either fully present with a sizable later-wins winner
/// or fully gone — so the topic degrades to schema-unavailable, and the
/// earlier bearer is NOT resurrected under the name either.
#[test]
fn r6_a_failing_later_wins_winner_never_stays_behind_to_panic_the_decoder() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "nav_pkg", "Filler", "float64[536870907] v\n");
    // Earlier bearer (sorted file walk: a_dup < z_dup): valid.
    write_yaml(
        tmp.path(),
        "a_dup.yaml",
        "schemas:\n  Dup:\n    fields:\n      uint32 a:\n",
    );
    // Later-wins winner: VARIABLE (string) with a fixed section that lands
    // past the u32 wire ceiling only after resolution (two inlined ~4 GiB
    // Fillers — each representable alone, their sum is not).
    write_yaml(
        tmp.path(),
        "z_dup.yaml",
        "schemas:\n  Dup:\n    fields:\n      Filler c:\n      Filler d:\n      string s:\n",
    );

    let reg = FieldRegistry::from_graph(&one_output_graph("Dup"), tmp.path());

    // PREMISE: the topic stays a registered Produced topic.
    assert_eq!(reg.topic_class(TOPIC), Some(&TopicClass::Produced));
    // Without the winner sweep this PANICS — layout_of materializes the rejected winner
    // left behind in the registry's schema set.
    assert!(
        reg.decoder_for(TOPIC).is_none(),
        "the topic degrades — no decoder over a name whose winning \
         definition sizing rejected"
    );
    assert_eq!(
        reg.expected_schema_hash(TOPIC),
        None,
        "and the earlier valid bearer is NOT resurrected under the name \
         (the hash map must not serve the shadowed loser's hash)"
    );
    let err = reg.resolve_field(TOPIC, "a").unwrap_err();
    assert!(err.schema_unavailable, "clean schema-unavailable refusal");

    // The sane store sibling is untouched by the sweep.
    let sib = FieldRegistry::from_graph(&one_output_graph("nav_pkg/Filler"), tmp.path());
    assert!(sib.expected_schema_hash(TOPIC).is_some());
}

// ===========================================================================
// The unified renderer
// ===========================================================================

/// The FOURTH resolution surface — the unified renderer's
/// NESTED-expansion context. Were `resolution_schema_set` to seed the
/// `.msg` store AFTER workspace YAML into a LAST-wins winner map, a store
/// definition of a same-qualified name would replace the YAML one during nested
/// expansion: `schema info` would render the STORE child's fields under a header
/// claiming YAML provenance, disagreeing with the replay registry.
///
/// Pinned with the twin rule: precedence decides only a nested reference
/// NOBODY SPELLED, and it decides it the way the verb selects — the
/// workspace's own definition. A spelling that NAMES the collision is
/// refused outright, on the other three surfaces, which this test asserts in
/// the same body so the two halves cannot drift apart. The store-only
/// control proves the ordering loses nothing.
#[test]
fn r7_a_nested_reference_nobody_spelled_expands_the_yaml_definition() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");

    // The collision pair, with MARKER field names unique to each side.
    write_yaml(
        tmp.path(),
        "widget4.yaml",
        "schemas:\n  acme4/Widget:\n    fields:\n      uint32 yaml_only_marker:\n",
    );
    write_store_msg(tmp.path(), "acme4", "Widget", "float64 store_only_marker\n");
    // A host schema whose nested field references the collided name.
    write_yaml(
        tmp.path(),
        "holder.yaml",
        "schemas:\n  Holder:\n    fields:\n      acme4/Widget child:\n",
    );

    // Surfaces 1-3 (the port-resolution trio) on the collided name: SPELLING it is refused,
    // one message naming both sources, identically on all three.
    let err = cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "acme4/Widget")
        .expect_err("surface 1: a spelling that names the collision is refused")
        .to_string();
    assert!(
        err.contains("'acme4/Widget' is ambiguous in this workspace — defined by:")
            && err.contains("schemas/widget4.yaml (entry acme4/Widget)")
            && err.contains("schemas/acme4/msg/Widget.msg (store)"),
        "got: {err}"
    );
    assert_eq!(
        cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "acme4/Widget")
            .expect_err("surface 2: schema info refuses it too")
            .to_string(),
        err
    );
    assert_eq!(
        cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "acme4/Widget")
            .expect_err("surface 3: the graph gate refuses it identically")
            .to_string(),
        err
    );

    // Surface 4: the RENDERED nested tree expands the YAML definition.
    let rendered = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "Holder")
        .unwrap()
        .to_string();
    assert!(
        rendered.contains("yaml_only_marker"),
        "surface 4: the nested expansion renders the YAML winner's field; got:\n{rendered}"
    );
    assert!(
        !rendered.contains("store_only_marker"),
        "surface 4: the shadowed store copy's field must NOT render under a \
         header claiming YAML provenance; got:\n{rendered}"
    );

    // CONTROL: a store-only name still expands from the store — the
    // precedence reorder lost nothing.
    write_store_msg(tmp.path(), "acme4", "Gadget", "float64 gadget_marker\n");
    write_yaml(
        tmp.path(),
        "gholder.yaml",
        "schemas:\n  GHolder:\n    fields:\n      acme4/Gadget child:\n",
    );
    let control = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "GHolder")
        .unwrap()
        .to_string();
    assert!(
        control.contains("gadget_marker"),
        "control: a store-only nested target still expands; got:\n{control}"
    );
}

// ===========================================================================
// The resolution-identity class
// ===========================================================================

/// Resolver identity: a store `acme5/Widget` is resolver
/// key `(Some("acme5"), "Widget")` while a slash-named YAML entry is
/// `(None, "acme5/Widget")` — two definitions core resolution keeps APART.
/// `Holder`'s qualified ref `acme5/Widget` binds the STORE in
/// `resolve_fixed_nested`, but a closure walk over a flattened qualified-name
/// map (YAML last-wins) would follow the YAML definition — here one that
/// ESCAPES the workspace set via `std_msgs/Header` — so the perfectly
/// hashable `Holder` would be silently omitted from the divergence map. The
/// walk keys candidates by the resolver's FULL identity and must
/// follow the store definition core binds.
#[test]
fn r8_the_closure_walk_binds_the_resolver_identity_not_the_qualified_string() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "acme5", "Widget", "float64 store_ok\n");
    write_yaml(
        tmp.path(),
        "widget5.yaml",
        "schemas:\n  acme5/Widget:\n    fields:\n      std_msgs/Header h:\n",
    );
    write_store_msg(tmp.path(), "acme5", "Holder", "acme5/Widget child\n");

    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());

    let expected = {
        let widget = {
            let mut s = MessageSchema::new_in_package("Widget", "acme5");
            s.add_field(FieldDef::new("store_ok", FieldType::F64));
            s
        };
        let holder = {
            let mut s = MessageSchema::new_in_package("Holder", "acme5");
            s.add_field(FieldDef::new(
                "child",
                FieldType::Nested {
                    schema_name: "Widget".to_string(),
                    package: Some("acme5".to_string()),
                    fixed: None,
                },
            ));
            s
        };
        oracle_hash(&mut [widget, holder], "acme5/Holder")
    };
    assert_eq!(
        map.get("acme5/Holder").copied(),
        Some(expected),
        "the walk must follow the STORE identity core binds for the \
         qualified ref (a flattened map would walk the escaping \
         YAML twin and omit Holder); got {map:?}"
    );
}

/// Determinism: duplicate schema names across YAML files —
/// a graph-hash fold in raw `read_dir` order beside a replay registry that
/// sorts would make which definition "later-wins" filesystem-dependent, and
/// the two surfaces could DISAGREE. All YAML-set builders ride the ONE
/// sorted walk (`workspace_yaml_files`), so both surfaces pick the SAME
/// winner — the lexically-last file's — regardless of creation order
/// (files are deliberately created in non-lexical order).
#[test]
fn r8_r27_duplicate_yaml_entry_names_are_refused_on_every_surface_naming_every_file() {
    let tmp = tempfile::tempdir().unwrap();
    // Created in NON-lexical order; each defines Dup3 with a distinct
    // layout. The original pin was that both surfaces picked the SAME sorted
    // last-wins winner; the ambiguity-refusal rule refuses a duplicate
    // entry name outright — and the refusal names every file, in the sorted
    // walk order, which is the determinism the original pin protected.
    write_yaml(
        tmp.path(),
        "e_dupe.yaml",
        "schemas:\n  Dup3:\n    fields:\n      float64 e_field:\n      float64 e2:\n",
    );
    write_yaml(
        tmp.path(),
        "b_dupe.yaml",
        "schemas:\n  Dup3:\n    fields:\n      uint32 b_field:\n",
    );
    write_yaml(
        tmp.path(),
        "a_dupe.yaml",
        "schemas:\n  Dup3:\n    fields:\n      uint32 a_field:\n",
    );
    write_yaml(
        tmp.path(),
        "c_dupe.yaml",
        "schemas:\n  Dup3:\n    fields:\n      uint32 c_field:\n",
    );
    // CONTROL: an entry one file declares binds everywhere.
    write_yaml(
        tmp.path(),
        "solo.yaml",
        "schemas:\n  Solo:\n    fields:\n      uint32 s:\n",
    );
    let schemas_dir = tmp.path().join("schemas");
    let solo_expected = {
        let mut s = MessageSchema::new("Solo");
        s.add_field(FieldDef::new("s", FieldType::U32));
        s.schema_hash()
    };

    // Validation — both port surfaces, one message, every file named in
    // sorted order.
    let err = cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Dup3")
        .expect_err("a duplicate entry name is refused")
        .to_string();
    for needle in [
        "'Dup3' is ambiguous in this workspace — defined by:",
        "schemas/a_dupe.yaml (entry Dup3), schemas/b_dupe.yaml (entry Dup3), \
         schemas/c_dupe.yaml (entry Dup3), schemas/e_dupe.yaml (entry Dup3)",
        "Declare it once, or name the one you mean",
    ] {
        assert!(
            err.contains(needle),
            "refusal carries {needle:?}; got: {err}"
        );
    }
    let gate = cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "Dup3")
        .expect_err("the existence gate refuses it identically")
        .to_string();
    assert_eq!(gate, err);
    // `schema info`: the refusal, never a definition one precedence picked.
    let info = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "Dup3")
        .expect_err("schema info prints the refusal")
        .to_string();
    assert_eq!(info, err);
    // `schema list`: every row of the name carries the refusal, no hash.
    let listing = cerulion_cli_engine::schema_cmd::schema_list(&schemas_dir);
    let dup_rows: Vec<_> = listing
        .workspace
        .iter()
        .filter(|e| e.name == "Dup3")
        .collect();
    assert_eq!(dup_rows.len(), 4, "every declaring file lists its row");
    for row in &dup_rows {
        assert_eq!(row.schema_hash, None, "{}: no hash", row.file);
        assert_eq!(
            row.refusal.as_deref(),
            Some(err.as_str()),
            "{}: the refusal",
            row.file
        );
    }
    assert!(listing
        .to_string()
        .contains("Dup3  (refused: 'Dup3' is ambiguous"));
    let solo_row = listing.workspace.iter().find(|e| e.name == "Solo").unwrap();
    assert_eq!(solo_row.schema_hash, Some(solo_expected));
    assert_eq!(solo_row.refusal, None);
    // Hash map: no entry under the refused name; the control binds.
    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());
    assert_eq!(
        map.get("Dup3"),
        None,
        "no entry for a refused name; got {map:?}"
    );
    assert_eq!(map.get("Solo"), Some(&solo_expected));
    // Serving: served by nothing — no document, pass-through binding.
    let config = one_output_graph("Dup3");
    let serving =
        cerulion_cli_engine::schema_serve::build_schema_serving(&config, &schemas_dir, None);
    assert_eq!(serving.topic_schemas[0].schema_name, "Dup3", "pass-through");
    assert!(!serving.schema_docs.iter().any(|d| d.qualified == "Dup3"));
    assert!(serving.schema_docs.iter().any(|d| d.qualified == "Solo"));
    // Replay: schema-unavailable on a registered topic — never a winner.
    let reg = FieldRegistry::from_graph(&config, tmp.path());
    assert_eq!(reg.topic_class(TOPIC), Some(&TopicClass::Produced));
    assert_eq!(reg.expected_schema_hash(TOPIC), None);
    assert!(reg.decoder_for(TOPIC).is_none());
    let solo = FieldRegistry::from_graph(&one_output_graph("Solo"), tmp.path());
    assert_eq!(solo.expected_schema_hash(TOPIC), Some(solo_expected));
    let ctl = cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Solo").unwrap();
    assert!(matches!(
        ctl.provenance,
        cerulion_cli_engine::schema_cmd::PortSchemaProvenance::Workspace { .. }
    ));
}

/// Nested bare ambiguity: a store type sharing a BUILTIN's
/// bare name makes an unqualified nested ref (no same-package target)
/// AMBIGUOUS to codegen — `resolve_fixed_nested`'s global-bare arm needs
/// exactly ONE candidate, tier-free. The registry's tiered ladder answered
/// that fallback before, silently store-binding the ref, so
/// `resolve_field` accepted paths under a layout codegen never resolved
/// (while the expected hash was computed with the ref UNRESOLVED). The
/// tiered ladder stays the rule for the graph's TOP-LEVEL `schema:`
/// strings only (pinned by `s3_the_store_outranks_a_same_named_builtin_
/// for_a_bare_claim`, which keeps passing).
#[test]
fn r8_a_bare_nested_ref_colliding_with_a_builtin_stays_ambiguous_for_replay() {
    let tmp = tempfile::tempdir().unwrap();
    // Bare name "Vector3" now has TWO bearers: the built-in
    // geometry_msgs/Vector3 and this store copy.
    write_store_msg(tmp.path(), "my_pkg", "Vector3", "float64 store_field\n");
    write_yaml(
        tmp.path(),
        "host.yaml",
        "schemas:\n  Host:\n    fields:\n      Vector3 v:\n",
    );

    let reg = FieldRegistry::from_graph(&one_output_graph("Host"), tmp.path());

    // PREMISE + hash-side agreement: Host resolves, its ref UNRESOLVED —
    // exactly what codegen computes (the ambiguous ref stays variable), so
    // the oracle over Host ALONE matches.
    let host_expected = {
        let mut s = MessageSchema::new("Host");
        s.add_field(FieldDef::new(
            "v",
            FieldType::Nested {
                schema_name: "Vector3".to_string(),
                package: None,
                fixed: None,
            },
        ));
        oracle_hash(&mut [s], "Host")
    };
    assert_eq!(reg.expected_schema_hash(TOPIC), Some(host_expected));

    // THE PIN: paths under the ambiguous ref refuse — neither silently
    // store-bound nor builtin-bound.
    assert!(
        reg.resolve_field(TOPIC, "v.store_field").is_err(),
        "an ambiguous bare ref must not be silently STORE-bound"
    );
    assert!(
        reg.resolve_field(TOPIC, "v.x").is_err(),
        "…nor silently BUILTIN-bound"
    );

    // CONTROL: the store copy stays fully reachable when NAMED.
    write_yaml(
        tmp.path(),
        "qhost.yaml",
        "schemas:\n  QHost:\n    fields:\n      my_pkg/Vector3 w:\n",
    );
    let qreg = FieldRegistry::from_graph(&one_output_graph("QHost"), tmp.path());
    assert!(
        qreg.resolve_field(TOPIC, "w.store_field").is_ok(),
        "a QUALIFIED ref still descends into the store layout"
    );
}

// ===========================================================================
// The registry's tier filter
// ===========================================================================

/// The resolver-identity rule's mirror, in the REGISTRY's own
/// tier filter: deleting every store schema whose qualified-name
/// STRING a slash-named YAML entry claims would be wrong, because core keeps
/// `(Some("acme6"), "Widget")` and `(None, "acme6/Widget")` APART, and a
/// QUALIFIED nested ref `acme6/Widget` binds the STORE identity. With the
/// store copy deleted, `Holder`'s ref goes UNRESOLVED at replay while
/// codegen, the robot's producer, and the graph_cmd folds all inline
/// the store definition — the expected hash diverges from the wire.
///
/// The rule: precedence by ORDER, never deletion — every identity stays in
/// the set (ascending built-ins → store → YAML, so last-wins string maps
/// still express the ladder), and core binds references by identity. The
/// oracle is built from what `resolve_fixed_nested` ACTUALLY does over the
/// full three-definition set, with the unresolved-ref hash as the
/// anti-vacuity control.
#[test]
fn r9_a_qualified_nested_ref_binds_the_store_identity_at_replay() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "acme6", "Widget", "float64 wx\nfloat64 wy\n");
    write_yaml(
        tmp.path(),
        "widget6.yaml",
        "schemas:\n  acme6/Widget:\n    fields:\n      uint32 a:\n",
    );
    write_store_msg(
        tmp.path(),
        "acme6",
        "Holder",
        "acme6/Widget child\nfloat64 tail\n",
    );

    let yaml_w = || {
        let mut s = MessageSchema::new("acme6/Widget");
        s.add_field(FieldDef::new("a", FieldType::U32));
        s
    };
    let store_w = || {
        let mut s = MessageSchema::new_in_package("Widget", "acme6");
        s.add_field(FieldDef::new("wx", FieldType::F64));
        s.add_field(FieldDef::new("wy", FieldType::F64));
        s
    };
    let holder = || {
        let mut s = MessageSchema::new_in_package("Holder", "acme6");
        s.add_field(FieldDef::new(
            "child",
            FieldType::Nested {
                schema_name: "Widget".to_string(),
                package: Some("acme6".to_string()),
                fixed: None,
            },
        ));
        s.add_field(FieldDef::new("tail", FieldType::F64));
        s
    };
    // What core ACTUALLY does over the full three-definition set: the
    // qualified ref binds the store identity and inlines it.
    let core_expected = oracle_hash(&mut [yaml_w(), store_w(), holder()], "acme6/Holder");
    // Anti-vacuity: a registry with the store copy deleted resolves the
    // ref against a set WITHOUT the store identity — provably a different
    // hash.
    let unresolved_stale = oracle_hash(&mut [yaml_w(), holder()], "acme6/Holder");
    assert_ne!(
        core_expected, unresolved_stale,
        "the store binding must be hash-visible — else this pins nothing"
    );

    let reg = FieldRegistry::from_graph(&one_output_graph("acme6/Holder"), tmp.path());
    assert_eq!(reg.topic_class(TOPIC), Some(&TopicClass::Produced));
    assert_eq!(
        reg.expected_schema_hash(TOPIC),
        Some(core_expected),
        "Holder's expected hash must bind the STORE identity for the \
         qualified ref, exactly as core does (a deleted store \
         copy would leave the ref unresolved)"
    );

    // The LADDER still governs the claimed NAME: a topic whose graph
    // `schema:` is the collided string resolves the YAML winner.
    let claimed = FieldRegistry::from_graph(&one_output_graph("acme6/Widget"), tmp.path());
    let yaml_expected = oracle_hash(&mut [yaml_w()], "acme6/Widget");
    assert_eq!(
        claimed.expected_schema_hash(TOPIC),
        Some(yaml_expected),
        "a user-claimed name still resolves the workspace-YAML winner"
    );
    assert!(claimed.resolve_field(TOPIC, "a").is_ok());
    assert!(claimed.resolve_field(TOPIC, "wx").is_err());

    // String-addressed descent under the identity-AMBIGUOUS string refuses
    // BOTH ways — a path walk keyed by name cannot know which identity the
    // ref bound, so it never guesses (the documented residual);
    // sibling fields are untouched.
    assert!(reg.resolve_field(TOPIC, "child.wx").is_err());
    assert!(reg.resolve_field(TOPIC, "child.a").is_err());
    assert!(reg.resolve_field(TOPIC, "tail").is_ok());
}

// ===========================================================================
// The identity model
// ===========================================================================

/// Nested bare alias vs a same-key shadow: precedence by order keeps
/// a store definition that SHADOWS a built-in at the SAME `(package,
/// name)` key in the set beside the built-in — one resolver identity,
/// two vec entries. A flat nested-bare index that counts both reads the bare
/// name as AMBIGUOUS, and `resolve_field` then refuses dotted paths core
/// resolves happily (core's `by_key` dedups the key later-wins and binds
/// the store copy). The index dedups by resolver key first; TRUE
/// ambiguity (distinct keys sharing a bare name) still refuses (pinned by
/// `r8_a_bare_nested_ref_colliding_with_a_builtin_stays_ambiguous_for_replay`).
#[test]
fn r10_a_store_shadowing_a_builtin_at_the_same_key_keeps_its_bare_alias() {
    let tmp = tempfile::tempdir().unwrap();
    // SAME key as the built-in geometry_msgs/Vector3, different layout +
    // a marker field name.
    write_store_msg(
        tmp.path(),
        "geometry_msgs",
        "Vector3",
        "float64 store_field\nfloat64 store_field2\n",
    );
    write_yaml(
        tmp.path(),
        "hosta.yaml",
        "schemas:\n  HostA:\n    fields:\n      Vector3 v:\n",
    );

    let reg = FieldRegistry::from_graph(&one_output_graph("HostA"), tmp.path());

    // Hash side: core dedups the key later-wins → the STORE copy binds the
    // bare ref and inlines.
    let expected = {
        let store_v3 = {
            let mut s = MessageSchema::new_in_package("Vector3", "geometry_msgs");
            s.add_field(FieldDef::new("store_field", FieldType::F64));
            s.add_field(FieldDef::new("store_field2", FieldType::F64));
            s
        };
        let host = {
            let mut s = MessageSchema::new("HostA");
            s.add_field(FieldDef::new(
                "v",
                FieldType::Nested {
                    schema_name: "Vector3".to_string(),
                    package: None,
                    fixed: None,
                },
            ));
            s
        };
        oracle_hash(&mut [store_v3, host], "HostA")
    };
    assert_eq!(reg.expected_schema_hash(TOPIC), Some(expected));

    // THE PIN: the dotted path under the store winner resolves — one
    // identity is ONE candidate, not two (never refused as ambiguous).
    assert!(
        reg.resolve_field(TOPIC, "v.store_field").is_ok(),
        "a same-key shadow is one resolver identity — the bare ref binds it"
    );
    // The shadowed built-in's layout does not serve.
    assert!(reg.resolve_field(TOPIC, "v.x").is_err());
}

/// Partial-removal propagation: an OVERFLOWING store
/// definition `(Some("acme7"), "Widget")` beside a later VALID slash-named
/// YAML `(None, "acme7/Widget")` — same flattened string, distinct
/// identities (they coexist under precedence by order). The preflight removes only the
/// store definition and the name legitimately SURVIVES via the YAML
/// winner, so a names-only return would propagate NOTHING: the tiers
/// would keep the rejected store definition, and `Holder`'s QUALIFIED ref —
/// which binds the STORE identity — would make `LayoutResolver::new` PANIC
/// materializing it at `decoder_for`/`resolve_field` time. The removal
/// contract carries removed IDENTITIES too, and the tier scrub
/// applies both halves.
#[test]
fn r10_a_partially_removed_store_definition_propagates_to_the_registry_tiers() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(
        tmp.path(),
        "acme7",
        "Filler",
        "float64[2305843009213693951] v\n",
    );
    // Fixed composed overflow — the preflight's removal arm.
    write_store_msg(tmp.path(), "acme7", "Widget", "Filler[4] arr\n");
    // The valid same-flattened-name YAML winner.
    write_yaml(
        tmp.path(),
        "widget7.yaml",
        "schemas:\n  acme7/Widget:\n    fields:\n      uint32 a:\n",
    );
    write_store_msg(
        tmp.path(),
        "acme7",
        "Holder",
        "acme7/Widget child\nfloat64 tail\n",
    );

    // If the removal did not propagate, decoder_for/resolve_field PANIC (the retained store Widget
    // is bound by Holder's qualified ref and sized by resolution).
    let reg = FieldRegistry::from_graph(&one_output_graph("acme7/Holder"), tmp.path());
    assert_eq!(reg.topic_class(TOPIC), Some(&TopicClass::Produced));
    assert!(reg.decoder_for(TOPIC).is_some(), "no panic, real decoder");
    // With the store identity scrubbed, the qualified ref is UNRESOLVED
    // (the YAML twin is a different identity core cannot bind for it):
    // the expected hash matches exactly what core computes over that set.
    let holder_expected = {
        let yaml_w = {
            let mut s = MessageSchema::new("acme7/Widget");
            s.add_field(FieldDef::new("a", FieldType::U32));
            s
        };
        let holder = {
            let mut s = MessageSchema::new_in_package("Holder", "acme7");
            s.add_field(FieldDef::new(
                "child",
                FieldType::Nested {
                    schema_name: "Widget".to_string(),
                    package: Some("acme7".to_string()),
                    fixed: None,
                },
            ));
            s.add_field(FieldDef::new("tail", FieldType::F64));
            s
        };
        oracle_hash(&mut [yaml_w, holder], "acme7/Holder")
    };
    assert_eq!(reg.expected_schema_hash(TOPIC), Some(holder_expected));
    assert!(reg.resolve_field(TOPIC, "tail").is_ok());
    // The rejected store layout must not serve anywhere — and (the
    // vacuity-guard-needs-a-premise class) the surviving YAML
    // twin must not satisfy the removed STORE identity either: a
    // string-fallback regression would resolve `child.a` through the
    // same-string YAML layout while `child.arr` still failed.
    assert!(
        reg.resolve_field(TOPIC, "child.a").is_err(),
        "the surviving YAML twin must not satisfy the removed store identity"
    );
    assert!(
        reg.resolve_field(TOPIC, "child.arr").is_err(),
        "the removed store definition's field must not resolve"
    );

    // The claimed NAME still serves the YAML winner on both string-keyed
    // surfaces — the name survives the partial removal.
    let claimed = FieldRegistry::from_graph(&one_output_graph("acme7/Widget"), tmp.path());
    let yaml_expected = {
        let mut s = MessageSchema::new("acme7/Widget");
        s.add_field(FieldDef::new("a", FieldType::U32));
        oracle_hash(&mut [s], "acme7/Widget")
    };
    assert_eq!(claimed.expected_schema_hash(TOPIC), Some(yaml_expected));
    assert!(claimed.resolve_field(TOPIC, "a").is_ok());
    assert!(claimed.resolve_field(TOPIC, "arr").is_err());

    // And the graph-hash fold (which removes in place) agrees: the name
    // serves the YAML definition there too.
    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());
    assert_eq!(map.get("acme7/Widget").copied(), Some(yaml_expected));
}

// ===========================================================================
// Closing the identity-collapse class
// ===========================================================================

/// The identity rule in the registry's path walk: a walk that binds
/// a QUALIFIED nested ref by comparing flattened STRINGS lets
/// a ref core leaves UNRESOLVED — `p9/T9` where only a slash-named
/// YAML `(None, "p9/T9")` exists, an identity a qualified ref can never
/// bind — happily descend that same-string different-identity layout
/// during tolerance path walking, accepting paths against bytes the
/// producer never wrote. The walk binds by core's `(package, name)`
/// identity rule (qualified = exact lookup, NO fallthrough) and
/// materializes through the string-winner gate; a ref core binds walks
/// exactly core's layout, a ref core leaves unresolved walks NOTHING.
#[test]
fn r11_a_qualified_ref_core_leaves_unresolved_walks_nothing() {
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(
        tmp.path(),
        "t9.yaml",
        "schemas:\n  p9/T9:\n    fields:\n      uint32 yaml_field:\n",
    );
    write_store_msg(tmp.path(), "p9", "Host9", "p9/T9 child\nfloat64 tail\n");
    // The positive control pair: a ref core BINDS (and whose identity is
    // its own string's winner) must keep walking.
    write_store_msg(tmp.path(), "p9", "Inner", "float64 inner_f\n");
    write_store_msg(tmp.path(), "p9", "Outer2", "p9/Inner child\nfloat64 tail\n");

    let reg = FieldRegistry::from_graph(&one_output_graph("p9/Host9"), tmp.path());
    assert_eq!(reg.topic_class(TOPIC), Some(&TopicClass::Produced));
    // Hash side: core leaves the qualified ref UNRESOLVED (the YAML twin
    // is a different resolver identity), and the expected hash agrees.
    let host_expected = {
        let yaml_t9 = {
            let mut s = MessageSchema::new("p9/T9");
            s.add_field(FieldDef::new("yaml_field", FieldType::U32));
            s
        };
        let host = {
            let mut s = MessageSchema::new_in_package("Host9", "p9");
            s.add_field(FieldDef::new(
                "child",
                FieldType::Nested {
                    schema_name: "T9".to_string(),
                    package: Some("p9".to_string()),
                    fixed: None,
                },
            ));
            s.add_field(FieldDef::new("tail", FieldType::F64));
            s
        };
        oracle_hash(&mut [yaml_t9, host], "p9/Host9")
    };
    assert_eq!(reg.expected_schema_hash(TOPIC), Some(host_expected));
    assert!(reg.decoder_for(TOPIC).is_some());
    assert!(reg.resolve_field(TOPIC, "tail").is_ok());
    // THE PIN: the unresolved ref walks NOTHING — a string
    // match would descend the YAML twin's layout and accept this path.
    assert!(
        reg.resolve_field(TOPIC, "child.yaml_field").is_err(),
        "a ref core leaves unresolved must not walk a same-string impostor"
    );

    // Positive control: a core-bound ref (identity == its string's
    // winner) accepts exactly core's layout's paths.
    let ctl = FieldRegistry::from_graph(&one_output_graph("p9/Outer2"), tmp.path());
    assert!(ctl.resolve_field(TOPIC, "child.inner_f").is_ok());
    assert!(ctl.resolve_field(TOPIC, "child.nope").is_err());
}

/// Cross-surface layout parity: if store
/// `schema info` resolved layout columns against built-ins + the store
/// ONLY, while the graph-hash surface and the unified renderer resolve
/// the SAME store schema with workspace YAML included, a store field
/// referencing a workspace YAML type (bound through the resolver's
/// global-bare arm) would report a DIFFERENT fixedness/size/hash than every
/// other surface. All three consume the one precedence-ordered
/// resolution set (`resolution_schema_set`).
#[test]
fn r11_store_schema_info_resolves_workspace_yaml_refs_like_the_hash_surface() {
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(
        tmp.path(),
        "inner11.yaml",
        "schemas:\n  Inner11:\n    fields:\n      float64 ya:\n      float64 yb:\n",
    );
    write_store_msg(tmp.path(), "p11", "Wrap", "Inner11 w\nfloat64 t\n");

    // The graph-hash surface's value (workspace YAML in the set → the
    // bare ref binds + inlines the YAML type).
    // The expected value is an INDEPENDENT resolved
    // oracle — hand-built IR for both definitions, resolved and hashed by
    // the codegen recipe — so two surfaces agreeing on a WRONG value
    // cannot pass.
    let expected = {
        let mut inner = MessageSchema::new("Inner11");
        inner.add_field(FieldDef::new("ya", FieldType::F64));
        inner.add_field(FieldDef::new("yb", FieldType::F64));
        let mut wrap = MessageSchema::new_in_package("Wrap", "p11");
        wrap.add_field(FieldDef::new(
            "w",
            FieldType::Nested {
                schema_name: "Inner11".to_string(),
                package: None,
                fixed: None,
            },
        ));
        wrap.add_field(FieldDef::new("t", FieldType::F64));
        oracle_hash(&mut [inner, wrap], "p11/Wrap")
    };
    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());
    let hash_surface = map
        .get("p11/Wrap")
        .copied()
        .expect("the wrap schema hashes on the graph surface");
    assert_eq!(
        hash_surface, expected,
        "the graph-hash surface matches the oracle"
    );

    // schema info's store arm must report the SAME resolved hash.
    let info = cerulion_cli_engine::schema_cmd::schema_info_unified(
        &tmp.path().join("schemas"),
        "p11/Wrap",
    )
    .unwrap();
    assert_eq!(
        info.source,
        cerulion_cli_engine::schema_cmd::SchemaSource::MsgStore,
        "premise: the store arm answers"
    );
    assert_eq!(
        info.result.entries[0].schema_hash,
        Some(hash_surface),
        "schema info resolves the YAML-referencing store schema exactly \
         as the graph-hash surface does (built-ins + store only would \
         leave the ref unresolved here)"
    );

    // Anti-vacuity: the YAML binding is hash-visible — the unresolved-ref
    // twin provably differs.
    let unresolved = {
        let mut s = MessageSchema::new_in_package("Wrap", "p11");
        s.add_field(FieldDef::new(
            "w",
            FieldType::Nested {
                schema_name: "Inner11".to_string(),
                package: None,
                fixed: None,
            },
        ));
        s.add_field(FieldDef::new("t", FieldType::F64));
        oracle_hash(&mut [s], "p11/Wrap")
    };
    assert_ne!(hash_surface, unresolved, "the ref must be load-bearing");
}

// ===========================================================================
// The shared resolution set
// ===========================================================================

/// The preflight class at the resolution-set seam: `schema list` and the
/// store-info arm read the ONE shared
/// `resolution_schema_set`, which must not hand the untrusted workspace tiers to
/// `LayoutResolver::new` UNFILTERED. TWO panic shapes, each pinned:
///
/// - COMPOSED overflow: individually-valid `Inner` (~4 GiB, under the
///   u32 wire ceiling), `Outer = Inner[2^33]` (the product ~3.7e19
///   overflows usize only after inlining), and a `Wrapper` referencing `Outer` — resolving
///   `Wrapper` materializes `Outer` and would panic inside
///   `LayoutResolver::new`. The composed preflight drops `Outer`.
/// - LAYOUT-arithmetic overflow: `Huge = float64[2^61 - 1]` passes the
///   CHECKED sizing recipe (2^64 − 8 fits usize) yet overflows the layout
///   walker's unchecked header/table arithmetic in `layout_of`. The wire's
///   `total_size` is u32, so a fixed section beyond `u32::MAX` is
///   physically unrepresentable — the ceiling drops it from the
///   resolver set, loudly.
///
/// Every store FILE still lists (the preflights thin the resolver set,
/// never the store), and the renderer over the same shared set degrades
/// the same way.
#[test]
fn r12_schema_list_degrades_a_hostile_composition_instead_of_panicking() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    write_store_msg(
        tmp.path(),
        "nav12",
        "Huge",
        "float64[2305843009213693951] v\n",
    );
    write_store_msg(tmp.path(), "nav12", "Inner", "float64[536870907] v\n");
    write_store_msg(tmp.path(), "nav12", "Outer", "Inner[8589934592] arr\n");
    write_store_msg(tmp.path(), "nav12", "Wrapper", "Outer o\nfloat64 w\n");

    // Unfiltered, this PANICS — the composed arm inside LayoutResolver::new
    // (resolve_fixed_nested materializes Outer as Wrapper's fixed nested
    // target), the layout arm inside layout_of("nav12/Huge").
    let listing = cerulion_cli_engine::schema_cmd::schema_list(&schemas_dir);
    assert_eq!(
        listing.store.len(),
        4,
        "every store file still LISTS (the preflight thins the resolver \
         set, never the store itself)"
    );

    // The renderer path over the same shared set degrades too.
    let rendered =
        cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "nav12/Wrapper")
            .unwrap()
            .to_string();
    assert!(
        rendered.contains("nav12/Wrapper"),
        "schema info renders the wrapper without panicking; got:\n{rendered}"
    );

    // The premise-assert class:
    // `Huge`'s REJECTION is asserted explicitly, not implied by the listing
    // count — the u32 wire ceiling dropped it from the resolver set, and a
    // rejected store definition is REFUSED by `schema info` (serving
    // its parse-time classification would be wrong; no number is served at all). In a
    // release build a removed ceiling would WRAP the layout arithmetic
    // silently and resolve it; in debug it panics — either way this pin fails.
    let err = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "nav12/Huge")
        .expect_err("the wire-unrepresentable Huge is refused");
    let msg = err.to_string();
    assert!(
        msg.contains("nav12/Huge") && msg.contains("exceeds the u32 wire ceiling"),
        "the refusal names the schema and the ceiling; got: {msg}"
    );
    // The listing rows agree: rejected definitions list UNHASHED, the
    // referencing `Wrapper` survives with its ref variable (oracle: the
    // definition hashed alone), `Inner` (nested-free) hashes as itself.
    let row = |name: &str| {
        listing
            .store
            .iter()
            .find(|e| e.name == name)
            .unwrap()
            .schema_hash
    };
    assert_eq!(row("nav12/Huge"), None, "Huge lists unhashed");
    assert_eq!(
        row("nav12/Outer"),
        None,
        "the composed overflow lists unhashed"
    );
    let wrapper_alone =
        cerulion_core::codegen::parse_rosmsg("Outer o\nfloat64 w\n", "Wrapper", Some("nav12"))
            .unwrap();
    assert_eq!(row("nav12/Wrapper"), Some(wrapper_alone.schema_hash()));
    let inner_alone =
        cerulion_core::codegen::parse_rosmsg("float64[536870907] v\n", "Inner", Some("nav12"))
            .unwrap();
    assert_eq!(row("nav12/Inner"), Some(inner_alone.schema_hash()));
}

// ===========================================================================
// The resolution-set seam at EVERY size-fold consumer
// ===========================================================================

/// The resolution-set class at another size-fold site: if `schema
/// info`'s YAML-entry layout enrichment seeded built-ins + YAML but NOT
/// the `.msg` store, a YAML entry nesting a store type would report a
/// `wire fixed size` WITHOUT the store type's bytes — while the recording
/// fold and replay both inline it. HAND ORACLE: `acme/Widget` = 2 ×
/// `float64` = 16, then `float64 v` = 24, never 8 (the ref left
/// variable). Replay's layout over the same files is the cross-check.
#[test]
fn r14_a_yaml_entry_nesting_a_store_type_reports_the_store_size_on_schema_info() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "acme", "Widget", "float64 x\nfloat64 y\n");
    write_yaml(
        tmp.path(),
        "stamped.yaml",
        "schemas:\n  Stamped:\n    fields:\n      acme/Widget w:\n      float64 v:\n",
    );
    let schemas_dir = tmp.path().join("schemas");

    let info =
        cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "Stamped").unwrap();
    let entry = &info.result.entries[0];
    assert!(
        entry.wire_fixed_size_resolved,
        "the YAML entry resolves against the combined set"
    );
    assert_eq!(
        entry.wire_fixed_size, 24,
        "the store-typed nested field INLINES into the reported fixed size (16 + 8)"
    );
    let w = entry
        .fields
        .iter()
        .find(|f| f.name == "w")
        .expect("the nested field is listed");
    assert!(
        !w.is_variable,
        "a fixed-resolved store-typed field is classified fixed, not variable"
    );

    // Replay resolves the SAME layout for a topic of that type.
    let reg = FieldRegistry::from_graph(&one_output_graph("Stamped"), tmp.path());
    let replay_size = reg
        .decoder_for(TOPIC)
        .expect("replay resolves the YAML type")
        .root_layout()
        .expect("replay materializes its layout")
        .fixed_size;
    assert_eq!(replay_size, 24, "replay inlines the store type too");
}

/// The wire-ceiling class at the replay registry: a registry that
/// preflights the declared-overflow retain but never the u32 wire
/// ceiling lets a produced topic typed `Huge` (2^64 − 8 bytes — passes the
/// checked recipe) reach `layout_of`'s UNCHECKED header/offset-table
/// arithmetic through `decoder_for` + `root_layout` — a panic in debug, a
/// silent wrap in release — instead of degrading to schema-unavailable.
/// Both the declared shape and the composed one (`Outer = Inner[2]` of a
/// ~4 GiB `Inner`) degrade; the premise assert proves the topic is still a
/// REGISTERED Produced topic (degradation, not disappearance); the sane
/// sibling still resolves its hash.
#[test]
fn r14_the_registry_rejects_an_unrepresentable_fixed_section_instead_of_panicking() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(
        tmp.path(),
        "nav14",
        "Huge",
        "float64[2305843009213693951] v\n",
    );
    write_store_msg(tmp.path(), "nav14", "Inner", "float64[536870907] v\n");
    write_store_msg(tmp.path(), "nav14", "Outer", "Inner[2] arr\n");
    write_store_msg(tmp.path(), "nav_pkg", "Goal", GOAL_MSG);

    for hostile in ["nav14/Huge", "nav14/Outer"] {
        let reg = FieldRegistry::from_graph(&one_output_graph(hostile), tmp.path());
        assert_eq!(
            reg.topic_class(TOPIC),
            Some(&TopicClass::Produced),
            "'{hostile}': the output stays a REGISTERED Produced topic"
        );
        assert_eq!(
            reg.expected_schema_hash(TOPIC),
            None,
            "'{hostile}': an unrepresentable root serves NO expected hash"
        );
        assert!(
            reg.decoder_for(TOPIC).is_none(),
            "'{hostile}': no decoder — nothing left to materialize its layout"
        );
        let err = reg.resolve_field(TOPIC, "v").unwrap_err();
        assert!(
            err.schema_unavailable,
            "'{hostile}': a clean schema-unavailable refusal, never a panic"
        );
    }

    // The representable member and the sane sibling still resolve.
    let reg = FieldRegistry::from_graph(&one_output_graph("nav14/Inner"), tmp.path());
    let inner_size = reg
        .decoder_for(TOPIC)
        .expect("a fixed section UNDER the ceiling resolves")
        .root_layout()
        .expect("layout")
        .fixed_size;
    assert_eq!(inner_size, 536_870_907 * 8);
    let reg = FieldRegistry::from_graph(&one_output_graph("nav_pkg/Goal"), tmp.path());
    let expected = oracle_hash(&mut [goal_store_ir()], "nav_pkg/Goal");
    assert_eq!(reg.expected_schema_hash(TOPIC), Some(expected));
}

// ===========================================================================
// The listing/info fallbacks and the store row's own identity
// ===========================================================================

/// A `schema list` STORE row names a store
/// FILE, so its hash must be that definition's. Under the canonical
/// YAML-last order a slash-named workspace YAML entry `pkg/Type` is the
/// resolver's string winner, so a row hashed by string would print the YAML hash beside
/// the store path. Hand oracles: the store definition hashed alone (it has
/// no nested refs) — and it must differ from the YAML twin's hash, or the
/// pin would be vacuous. A nesting arm proves nested BINDING is untouched
/// by the reordering: `pkg/Wrap` (store) nests `pkg/Type` by qualified ref,
/// which binds the STORE identity on every surface — its resolved hash
/// folds the store `Type`, never the YAML one.
#[test]
fn r15_a_store_row_lists_the_store_definitions_own_hash_under_a_yaml_slash_twin() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "pkg", "Type", "uint32 a\n");
    write_store_msg(tmp.path(), "pkg", "Wrap", "pkg/Type t\nfloat64 w\n");
    write_yaml(
        tmp.path(),
        "ptype.yaml",
        "schemas:\n  pkg/Type:\n    fields:\n      uint32 b:\n      uint32 c:\n",
    );
    let schemas_dir = tmp.path().join("schemas");

    let store_type =
        cerulion_core::codegen::parse_rosmsg("uint32 a\n", "Type", Some("pkg")).unwrap();
    let store_type_hash = store_type.schema_hash();
    let yaml_twin_hash = {
        let mut s = MessageSchema::new("pkg/Type");
        s.add_field(FieldDef::new("b", FieldType::U32));
        s.add_field(FieldDef::new("c", FieldType::U32));
        s.schema_hash()
    };
    assert_ne!(
        store_type_hash, yaml_twin_hash,
        "the twins must differ or this pins nothing"
    );
    let wrap_expected = {
        let wrap =
            cerulion_core::codegen::parse_rosmsg("pkg/Type t\nfloat64 w\n", "Wrap", Some("pkg"))
                .unwrap();
        oracle_hash(&mut [store_type.clone(), wrap], "pkg/Wrap")
    };

    let listing = cerulion_cli_engine::schema_cmd::schema_list(&schemas_dir);
    let row = |name: &str| {
        listing
            .store
            .iter()
            .find(|e| e.name == name)
            .unwrap_or_else(|| panic!("store row {name} listed"))
    };
    assert_eq!(
        row("pkg/Type").schema_hash,
        Some(store_type_hash),
        "the store row carries the STORE definition's hash, not the YAML twin's"
    );
    assert_eq!(row("pkg/Type").relative_path, "pkg/msg/Type.msg");
    assert_eq!(
        row("pkg/Wrap").schema_hash,
        Some(wrap_expected),
        "nested binding is by identity — the store Type folds into Wrap on this surface too"
    );
    // SPELLING the collided string is REFUSED (the twin rule) — the
    // listing pin is about the store ROW, which is reached by no spelling and
    // must still show the store definition's own hash. The YAML twin's hash
    // is computed above purely to prove the two differ.
    let err = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "pkg/Type")
        .expect_err("the qualified twin is refused")
        .to_string();
    assert!(
        err.contains("'pkg/Type' is ambiguous in this workspace — defined by:")
            && err.contains("schemas/ptype.yaml (entry pkg/Type)")
            && err.contains("schemas/pkg/msg/Type.msg (store)"),
        "got: {err}"
    );
    assert_ne!(
        yaml_twin_hash, store_type_hash,
        "the row's hash is the store's, and the twins differ — asserted above"
    );
}

/// `schema info` over a declaration whose
/// fixed section OVERFLOWS the size arithmetic (`HostileM`, 2^61 × 8 bytes)
/// must not PANIC — a store-tier unresolved fallback that called the
/// panicking `schema_hash()`/`wire_fixed_size()` on the very definition the
/// shared preflight had rejected would. It REFUSES loudly, naming the
/// schema, its file and the offending field; a sane sibling is untouched.
/// (`Huge`, 2^64 − 8, is the OTHER class — sizable but past the u32
/// ceiling — and is refused too: a preflight-rejected
/// definition is refused at every consumer.)
/// The workspace-YAML tier needs no checked twin, and the test pins WHY:
/// its parser refuses any `FixedArray` past `MAX_FIXED_ARRAY_LEN` at parse
/// time — the same hostile length is a loud Validation refusal there
/// before any size is ever computed.
#[test]
fn r15_schema_info_refuses_a_hostile_declaration_loudly_instead_of_panicking() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(
        tmp.path(),
        "nav15",
        "HostileM",
        "float64[2305843009213693952] a\n",
    );
    write_store_msg(tmp.path(), "nav15", "Goal", GOAL_MSG);
    // The file is named by the entry's EXACT stem (a CI Linux failure):
    // `schema info` reaches a present-but-BROKEN YAML file only through the
    // file-STEM tier, an exact file-name lookup — the entry-name tier cannot
    // see inside a file that fails to parse (it warns and skips it). A stem
    // spelled `hostiley.yaml` matches `HostileY.yaml` on case-insensitive
    // APFS and NOT on ext4, where the lookup falls through to the built-in
    // "not a qualified name" refusal (a case-sensitive APFS volume as
    // TMPDIR behaves the same). The engine tests run on both OSes: mixed-case names
    // must be exact.
    write_yaml(
        tmp.path(),
        "HostileY.yaml",
        "schemas:\n  HostileY:\n    fields:\n      float64[18446744073709551615] a:\n",
    );
    let schemas_dir = tmp.path().join("schemas");

    // Without the refusal this PANICS (schema.rs: FixedArray size overflows usize).
    let err = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "nav15/HostileM")
        .expect_err("a hostile store declaration is a loud refusal");
    let msg = err.to_string();
    for needle in [
        "nav15/HostileM",
        "nav15/msg/HostileM.msg",
        "overflows",
        "field `a`",
    ] {
        assert!(msg.contains(needle), "refusal names {needle:?}; got: {msg}");
    }
    // PREMISE for the YAML tier: the parser's own cap refuses the length up
    // front (the reason the YAML fallbacks keep the direct recipe).
    let err = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "HostileY")
        .expect_err("a hostile YAML length is refused at parse");
    let msg = err.to_string();
    for needle in ["HostileY", "maximum supported length"] {
        assert!(
            msg.contains(needle),
            "parse refusal names {needle:?}; got: {msg}"
        );
    }
    // The sane sibling is untouched.
    let goal =
        cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "nav15/Goal").unwrap();
    assert_eq!(goal.result.entries[0].wire_fixed_size, 16);
}

/// The `schema list` half: a store loop whose
/// resolution fallback hashes through the panicking recipe lets one hostile
/// `.msg` crash the whole listing. Every store file still LISTS: the
/// hostile row carries `schema_hash: None` (rendered as the loud unhashable
/// marker, never `0x0`), the sane rows their real hashes, and the rendered
/// text says so. A hostile YAML FILE is refused by the parser's cap and
/// skipped with a warn (pre-existing, pinned here as the premise) — the
/// YAML loop never sizes one.
#[test]
fn r15_schema_list_lists_a_hostile_declaration_unhashed_instead_of_panicking() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(
        tmp.path(),
        "nav15",
        "HostileM",
        "float64[2305843009213693952] a\n",
    );
    write_store_msg(tmp.path(), "nav_pkg", "Goal", GOAL_MSG);
    write_yaml(
        tmp.path(),
        "hostiley.yaml",
        "schemas:\n  HostileY:\n    fields:\n      float64[18446744073709551615] a:\n",
    );
    write_yaml(
        tmp.path(),
        "plain.yaml",
        "schemas:\n  Plain:\n    fields:\n      uint32 a:\n",
    );
    let schemas_dir = tmp.path().join("schemas");

    // Without the unhashed fallback this PANICS in the store loop.
    let listing = cerulion_cli_engine::schema_cmd::schema_list(&schemas_dir);
    let store_row = |name: &str| listing.store.iter().find(|e| e.name == name).unwrap();
    let yaml_row = |name: &str| listing.workspace.iter().find(|e| e.name == name).unwrap();
    assert_eq!(
        store_row("nav15/HostileM").schema_hash,
        None,
        "hostile store row lists unhashed"
    );
    assert!(
        !listing.workspace.iter().any(|e| e.name == "HostileY"),
        "the hostile YAML file is refused at parse and skipped (premise)"
    );
    assert_eq!(
        store_row("nav_pkg/Goal").schema_hash,
        Some(oracle_hash(&mut [goal_store_ir()], "nav_pkg/Goal")),
        "the sane store row keeps its real hash"
    );
    let plain_expected = {
        let mut s = MessageSchema::new("Plain");
        s.add_field(FieldDef::new("a", FieldType::U32));
        s.schema_hash()
    };
    assert_eq!(yaml_row("Plain").schema_hash, Some(plain_expected));
    let rendered = listing.to_string();
    assert!(
        rendered.contains("nav15/HostileM  (unhashable"),
        "the marker renders beside the hostile store row; got:\n{rendered}"
    );
    assert!(
        !rendered.contains("0x0000000000000000"),
        "never a fabricated zero hash"
    );
}

// ===========================================================================
// A preflight-rejected definition is rejected at EVERY consumer
// ===========================================================================

/// After the shared preflight drops a
/// composed-overflow store definition, a fallback that hashes
/// the RAW declaration through `checked_schema_hash` reports success —
/// `Outer = Inner[2^33]` has an unresolved nested ref, so its raw fixed
/// section is 0 bytes and the hash "succeeds" — advertising an identity no
/// producer could stamp, on `schema info` AND beside the store path on
/// `schema list`. Instead: `schema info` REFUSES it naming the schema, its file
/// and the composed reason; `schema list` lists the row UNHASHED (the
/// marker, never `0x…`); the referencing `Wrapper` and the representable
/// `Inner` keep their real hashes (hand oracles — never a self-compare).
#[test]
fn r17_a_preflight_rejected_store_definition_is_refused_never_hashed_from_its_raw_declaration() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "nav17", "Inner", "float64[536870907] v\n");
    write_store_msg(tmp.path(), "nav17", "Outer", "Inner[8589934592] arr\n");
    write_store_msg(tmp.path(), "nav17", "Wrapper", "Outer o\nfloat64 w\n");
    write_store_msg(tmp.path(), "nav17", "Goal", GOAL_MSG);
    let schemas_dir = tmp.path().join("schemas");

    // `schema info`: the exact fallback — never `Ok` with the raw hash and
    // `wire_fixed_size: 0`.
    let err = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "nav17/Outer")
        .expect_err("a composed-overflow store definition is refused");
    let msg = err.to_string();
    for needle in [
        "nav17/Outer",
        "nav17/msg/Outer.msg",
        "resolution preflight rejected",
        "composed through fixed-nested inlining",
    ] {
        assert!(msg.contains(needle), "refusal names {needle:?}; got: {msg}");
    }

    // `schema list`: the same definition lists unhashed; its neighbours keep
    // their real hashes.
    let listing = cerulion_cli_engine::schema_cmd::schema_list(&schemas_dir);
    let row = |name: &str| {
        listing
            .store
            .iter()
            .find(|e| e.name == name)
            .unwrap()
            .schema_hash
    };
    assert_eq!(row("nav17/Outer"), None, "the rejected row lists unhashed");
    let wrapper_alone =
        cerulion_core::codegen::parse_rosmsg("Outer o\nfloat64 w\n", "Wrapper", Some("nav17"))
            .unwrap();
    assert_eq!(
        row("nav17/Wrapper"),
        Some(wrapper_alone.schema_hash()),
        "the referencing definition survives with its ref variable"
    );
    let inner_alone =
        cerulion_core::codegen::parse_rosmsg("float64[536870907] v\n", "Inner", Some("nav17"))
            .unwrap();
    assert_eq!(row("nav17/Inner"), Some(inner_alone.schema_hash()));
    let goal_alone = cerulion_core::codegen::parse_rosmsg(GOAL_MSG, "Goal", Some("nav17")).unwrap();
    assert_eq!(
        row("nav17/Goal"),
        Some(goal_alone.schema_hash()),
        "the sane sibling keeps its real hash"
    );
    let rendered = listing.to_string();
    assert!(
        rendered.contains("nav17/Outer  (unhashable: rejected by the size preflight"),
        "the marker renders beside the rejected row; got:\n{rendered}"
    );
    assert!(
        !rendered.contains("0x0000000000000000"),
        "never a fabricated zero hash"
    );
}

// ===========================================================================
// The `schema info` bare store tier beside a YAML slash twin
// ===========================================================================

/// A bare `Type` selecting the
/// STORE definition `pkg/Type` beside a slash-named workspace YAML entry
/// `pkg/Type` must never produce a mixed render. The refusal rule governs the
/// bare half: `schema info`'s store tier is port resolution's ONE verdict
/// (`bare_store_port_binding`), so the collision is REFUSED here exactly as
/// `resolve_port_schema` and `port_schema_exists` refuse it —
/// with the same message — and no mixed render is reachable. The twin rule makes
/// the QUALIFIED spelling the same refusal (the YAML/store twin), so the
/// collision is unnameable from either side. Still pinned, and the reason
/// this test matters: a store `Wrap` nesting `pkg/Type` by qualified ref
/// folds the STORE `Type` by IDENTITY on this surface (the
/// store-last set at work — hand oracle over the two store definitions,
/// 16 bytes), which is a reference nobody spelled and therefore still
/// renders.
#[test]
fn r27_schema_info_refuses_a_bare_store_name_beside_its_yaml_slash_twin_like_the_port_surfaces() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "pkg", "Type", "uint32 a\n");
    write_store_msg(tmp.path(), "pkg", "Wrap", "pkg/Type t\nfloat64 w\n");
    write_yaml(
        tmp.path(),
        "ptype.yaml",
        "schemas:\n  pkg/Type:\n    fields:\n      uint32 b:\n      uint32 c:\n",
    );
    let schemas_dir = tmp.path().join("schemas");

    let store_type =
        cerulion_core::codegen::parse_rosmsg("uint32 a\n", "Type", Some("pkg")).unwrap();
    let yaml_hash = {
        let mut s = MessageSchema::new("pkg/Type");
        s.add_field(FieldDef::new("b", FieldType::U32));
        s.add_field(FieldDef::new("c", FieldType::U32));
        s.schema_hash()
    };
    assert_ne!(
        store_type.schema_hash(),
        yaml_hash,
        "the twins must differ or this pins nothing"
    );

    // The BARE spelling is REFUSED — the port surfaces' verdict, verbatim.
    let info_err = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "Type")
        .expect_err("schema info refuses the bare name whose store string the YAML tier claims")
        .to_string();
    let port_err = cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Type")
        .expect_err("premise: the port surfaces refuse it")
        .to_string();
    assert_eq!(info_err, port_err, "one verdict on every resolving surface");
    assert!(
        info_err.contains("'pkg/Type' is ambiguous in this workspace — defined by:")
            && info_err.contains("schemas/ptype.yaml (entry pkg/Type)")
            && info_err.contains("schemas/pkg/msg/Type.msg (store)"),
        "the refusal names both definitions; got: {info_err}"
    );

    // The QUALIFIED spelling is the same collision named from the other
    // side, so it carries the SAME refusal — the twin is unnameable.
    assert_eq!(
        cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "pkg/Type")
            .expect_err("the qualified twin is refused too")
            .to_string(),
        info_err
    );
    assert_ne!(
        yaml_hash,
        store_type.schema_hash(),
        "the twins differ — asserted above, restated here because neither is nameable"
    );

    // Nested binding is by identity: the store `Wrap` folds the STORE `Type`
    // (the store-last set — its own definition, never the twin).
    let wrap_expected = {
        let wrap =
            cerulion_core::codegen::parse_rosmsg("pkg/Type t\nfloat64 w\n", "Wrap", Some("pkg"))
                .unwrap();
        oracle_hash(&mut [store_type.clone(), wrap], "pkg/Wrap")
    };
    let wrap = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "Wrap").unwrap();
    assert_eq!(
        wrap.source,
        cerulion_cli_engine::schema_cmd::SchemaSource::MsgStore
    );
    assert_eq!(wrap.result.entries[0].schema_hash, Some(wrap_expected));
    assert_eq!(
        wrap.result.entries[0].wire_fixed_size, 16,
        "Type (4, padded to 8) + f64 (8)"
    );
    // ANTI-TAUTOLOGY: without the twin the bare name answers with the store.
    let ctl = tempfile::tempdir().unwrap();
    write_store_msg(ctl.path(), "pkg", "Type", "uint32 a\n");
    let info =
        cerulion_cli_engine::schema_cmd::schema_info_unified(&ctl.path().join("schemas"), "Type")
            .unwrap();
    assert_eq!(
        info.source,
        cerulion_cli_engine::schema_cmd::SchemaSource::MsgStore
    );
    assert_eq!(
        info.result.entries[0].schema_hash,
        Some(store_type.schema_hash())
    );
}

// ===========================================================================
// A store root's nested tree; unique-bare hash aliases
// ===========================================================================

/// A store root laid out from the store-last
/// set must not render its NESTED tree over the canonical YAML-last
/// `schema_set`: with store `pkg/Root -> Child` and workspace YAML slash
/// entries `pkg/Root` + `pkg/Child`, the bare `Root` query would show the store
/// header and layout (core binds `Child` by identity — same-package → the
/// store) over the YAML `Child`'s fields. The tree renders over the SAME
/// set the root was selected from. Two halves, each catching its own broken variant:
/// the HEADER (store root's own hash + size — the store tier; oracle
/// over the two store definitions resolved together, asserted `!=` the YAML
/// twin's hash) and the TREE (the store child's field, never the YAML
/// twin's).
#[test]
fn r20_a_store_roots_nested_tree_renders_from_the_set_it_was_selected_from() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "pkg", "Child", "uint32 a\n");
    write_store_msg(tmp.path(), "pkg", "Root", "Child c\nfloat64 x\n");
    // Only the CHILD has a slash-named YAML twin — a twin of the
    // ROOT itself would make bare `Root` the refused collision (the store
    // tier is port resolution's verdict), so the render never happens.
    // The mechanism under test is the child twin: under the canonical
    // YAML-last set `pkg/Child`'s string winner is the YAML entry, so a
    // store root's tree printed the twin's fields beside a header that had
    // inlined the store child.
    write_yaml(
        tmp.path(),
        "ptwins.yaml",
        "schemas:\n  pkg/Child:\n    fields:\n      uint32 b:\n      uint32 c:\n",
    );
    let schemas_dir = tmp.path().join("schemas");

    let store_child =
        cerulion_core::codegen::parse_rosmsg("uint32 a\n", "Child", Some("pkg")).unwrap();
    let store_root =
        cerulion_core::codegen::parse_rosmsg("Child c\nfloat64 x\n", "Root", Some("pkg")).unwrap();
    let root_expected = oracle_hash(&mut [store_child.clone(), store_root.clone()], "pkg/Root");
    let yaml_child_root_hash = {
        let mut yaml_child = MessageSchema::new("pkg/Child");
        yaml_child.add_field(FieldDef::new("b", FieldType::U32));
        yaml_child.add_field(FieldDef::new("c", FieldType::U32));
        oracle_hash(&mut [yaml_child, store_root], "pkg/Root")
    };
    assert_ne!(
        root_expected, yaml_child_root_hash,
        "the child twins must differ or the header pin is vacuous"
    );

    let info = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "Root").unwrap();
    assert_eq!(
        info.source,
        cerulion_cli_engine::schema_cmd::SchemaSource::MsgStore
    );
    let entry = &info.result.entries[0];
    // HEADER half: the store root's own identity.
    assert_eq!(
        entry.schema_hash,
        Some(root_expected),
        "the header hash is the store root's (Child inlined)"
    );
    assert_eq!(
        entry.wire_fixed_size, 16,
        "Child (4, padded to 8) + f64 (8)"
    );
    let rendered = info.to_string();
    assert!(
        rendered.contains(&format!("hash: 0x{root_expected:016x}")),
        "got:\n{rendered}"
    );
    assert!(
        rendered.contains("wire fixed size: 16 bytes"),
        "got:\n{rendered}"
    );
    // TREE half: the store child's field under the store root — never the
    // YAML twin's.
    assert!(
        rendered.contains("  c: Child (fixed)"),
        "the nested field line; got:\n{rendered}"
    );
    assert!(
        rendered.contains("    a: uint32 (fixed)"),
        "the tree expands the STORE child (the definition the layout inlined); got:\n{rendered}"
    );
    assert!(
        !rendered.contains("b: uint32"),
        "no YAML twin field under a store header; got:\n{rendered}"
    );
}

/// If `build_workspace_schema_hashes` indexed a
/// store schema under `pkg/Type` only, while port resolution binds a UNIQUE
/// bare store name, a graph output declared as `Goal` would validate and run
/// yet have no map entry, and the runtime's divergence check (keyed on the
/// graph's literal `schema:` string) would silently skip it. The map carries
/// the bare alias under the ONE rule (`unique_bare_store_names`, shared with
/// the recording fold): unique ⇒ alias with the codegen hash (the string the
/// runtime looks up, so a divergent cdylib hash is compared and CAUGHT — the
/// warn's emission is pinned in core's `max_slice_len_warn_emission_test`);
/// ambiguous ⇒ no alias AND validation REFUSES the name loudly (never a
/// silent skip); YAML-claimed ⇒ the YAML tier's hash (workspace-first).
#[test]
fn r20_a_unique_bare_store_name_gets_a_hash_entry_the_ambiguous_twin_refuses() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "nav20", "Goal", GOAL_MSG);
    write_store_msg(tmp.path(), "a20", "Type", "uint32 a\n");
    write_store_msg(tmp.path(), "b20", "Type", "uint32 a\n");
    write_store_msg(tmp.path(), "nav20", "Pose", "float64 x\n");
    write_yaml(
        tmp.path(),
        "pose.yaml",
        "schemas:\n  Pose:\n    fields:\n      uint32 a:\n",
    );
    let schemas_dir = tmp.path().join("schemas");

    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());

    // UNIQUE: the graph form port resolution accepts gets the codegen hash.
    let goal = cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Goal").unwrap();
    assert_eq!(
        goal.schema, "nav20/Goal",
        "PREMISE: bare `Goal` binds the unique store definition"
    );
    let goal_ir = cerulion_core::codegen::parse_rosmsg(GOAL_MSG, "Goal", Some("nav20")).unwrap();
    let goal_expected = oracle_hash(&mut [goal_ir], "nav20/Goal");
    assert_eq!(
        map.get("Goal").copied(),
        Some(goal_expected),
        "the bare alias; got {map:?}"
    );
    assert_eq!(map.get("nav20/Goal").copied(), Some(goal_expected));

    // AMBIGUOUS: no alias — and validation refuses the name loudly.
    let err = cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "Type")
        .expect_err("an ambiguous bare store name is REFUSED at validation");
    assert!(err.to_string().contains("ambiguous"), "got: {err}");
    assert_eq!(
        map.get("Type"),
        None,
        "no alias for a name validation refuses; got {map:?}"
    );
    assert!(map.contains_key("a20/Type") && map.contains_key("b20/Type"));

    // YAML-CLAIMED: the bare key is the YAML tier's (workspace-first).
    let yaml_pose = {
        let mut s = MessageSchema::new("Pose");
        s.add_field(FieldDef::new("a", FieldType::U32));
        s.schema_hash()
    };
    let store_pose = {
        let ir =
            cerulion_core::codegen::parse_rosmsg("float64 x\n", "Pose", Some("nav20")).unwrap();
        oracle_hash(&mut [ir], "nav20/Pose")
    };
    assert_ne!(yaml_pose, store_pose);
    assert_eq!(
        map.get("Pose").copied(),
        Some(yaml_pose),
        "YAML claims the bare key"
    );
    assert_eq!(map.get("nav20/Pose").copied(), Some(store_pose));
}

// ===========================================================================
// A YAML file STEM claims a bare name for the YAML tier
// ===========================================================================

/// `schemas/Goal.yaml` may parse while declaring
/// `Other`; port resolution still binds bare `Goal` to the YAML tier from the
/// FILE STEM, so a guard that records only entry keys adds the store
/// alias `Goal` → `nav21/Goal` and `graph run` compares the
/// YAML-typed output against the STORE hash — a false divergence warn. The
/// claim set is port resolution's own (`workspace_yaml_bare_claims`:
/// parseable stems + entry names, shared with the fold and the gateway
/// bindings): no store alias under `Goal`; `Other` and `nav21/Goal` keep
/// their own entries.
#[test]
fn r21_a_yaml_file_stem_claim_suppresses_the_store_alias_no_false_divergence() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "nav21", "Goal", GOAL_MSG);
    write_yaml(
        tmp.path(),
        "Goal.yaml",
        "schemas:\n  Other:\n    fields:\n      uint32 a:\n",
    );
    let schemas_dir = tmp.path().join("schemas");

    // PREMISE: bare `Goal` is YAML-claimed by the FILE STEM — and the `.msg`
    // store spells it too, so the one workspace lookup refuses the spelling
    // as the TWIN. Either way the store's `nav21/Goal` is not the
    // stem's to alias, which is what the hash map below must withhold.
    let refusal = cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Goal")
        .expect_err("a stem file the store also spells is the refused twin")
        .to_string();
    assert!(
        refusal.contains("'Goal' is ambiguous in this workspace — defined by:")
            && refusal.contains("schemas/Goal.yaml (file stem; entries: Other)")
            && refusal.contains("schemas/nav21/msg/Goal.msg (store)"),
        "got: {refusal}"
    );

    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());
    let other_expected = {
        let mut s = MessageSchema::new("Other");
        s.add_field(FieldDef::new("a", FieldType::U32));
        s.schema_hash()
    };
    assert_eq!(map.get("Other").copied(), Some(other_expected));
    let goal_ir = cerulion_core::codegen::parse_rosmsg(GOAL_MSG, "Goal", Some("nav21")).unwrap();
    let store_goal = oracle_hash(&mut [goal_ir], "nav21/Goal");
    assert_eq!(map.get("nav21/Goal").copied(), Some(store_goal));
    // The stem binds the file's sole entry — `Goal` carries OTHER's
    // hash (the YAML-typed output is checked against its own document), and
    // NEVER the store's (the false-divergence hole this arm was written for).
    assert_ne!(other_expected, store_goal);
    assert_eq!(
        map.get("Goal").copied(),
        Some(other_expected),
        "the YAML-claimed bare name binds the stem's sole entry, never the store; got {map:?}"
    );
}

// ===========================================================================
// The stem→sole-entry binding on every surface
// ===========================================================================

/// The stem rule (a bare port name a workspace YAML file claims
/// by its file STEM binds the file's SOLE entry) lived in gateway serving
/// only, so the same port had no divergence check in the graph-hash map and
/// replayed schema-unavailable — three surfaces disagreeing about one port,
/// the same class as bare store names. HASH MAP half: a port
/// labelled `Goal` beside `Goal.yaml` declaring only `Other` gets `Other`'s
/// codegen hash under `Goal` — the literal string the runtime looks up, so a
/// divergent cdylib hash is compared and CAUGHT (the warn's emission is
/// pinned in core's `max_slice_len_warn_emission_test`); the multi-entry
/// `Multi.yaml` stem gets no entry. The store `nav23/Goal` keeps its qualified
/// key and no store alias (the stem-claim rule).
#[test]
fn r23_a_stem_claimed_port_is_hash_checked_against_the_sole_entry() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "nav23", "Goal", GOAL_MSG);
    write_yaml(
        tmp.path(),
        "Goal.yaml",
        "schemas:\n  Other:\n    fields:\n      uint32 a:\n",
    );
    write_yaml(
        tmp.path(),
        "Multi.yaml",
        "schemas:\n  A:\n    fields:\n      uint32 a:\n  B:\n    fields:\n      uint32 b:\n",
    );
    let schemas_dir = tmp.path().join("schemas");
    // PREMISE: `Goal` is claimed by the YAML stem tier (the file's sole
    // entry) — and refused as the `.msg` store TWIN by the one workspace
    // lookup, which is what makes the hash below the store's
    // problem to stay out of; `Multi`, a multi-entry stem, RESOLVES on the
    // spelling surfaces (it names one file) while no MAP can pick which of
    // its entries a channel carries, so it gets no entry below either.
    assert!(cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Goal").is_err());
    assert!(matches!(
        cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Multi")
            .expect("a multi-entry stem names ONE file")
            .provenance,
        cerulion_cli_engine::schema_cmd::PortSchemaProvenance::Workspace { .. }
    ));

    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());
    let other_expected = {
        let mut s = MessageSchema::new("Other");
        s.add_field(FieldDef::new("a", FieldType::U32));
        s.schema_hash()
    };
    assert_eq!(
        map.get("Goal").copied(),
        Some(other_expected),
        "the stem is checked against the sole entry's hash; got {map:?}"
    );
    assert_eq!(map.get("Other").copied(), Some(other_expected));
    let goal_ir = cerulion_core::codegen::parse_rosmsg(GOAL_MSG, "Goal", Some("nav23")).unwrap();
    assert_eq!(
        map.get("nav23/Goal").copied(),
        Some(oracle_hash(&mut [goal_ir], "nav23/Goal"))
    );
    assert_eq!(map.get("Multi"), None, "a multi-entry stem gets no entry");
    assert!(map.contains_key("A") && map.contains_key("B"));
}

/// The REPLAY half: a bag channel labelled `Goal` beside a
/// `Goal.yaml` declaring only `Other` replays with `Other`'s schema (its
/// expected hash, a decoder, field resolution) instead of schema-unavailable.
/// (The multi-entry twin is its own test below: sharing one
/// workspace would let the `Goal` construction emit the multi-entry warning too, so
/// the log assertion would not be attributable to the `Multi` construction.)
#[test]
fn r23_a_stem_claimed_channel_replays_with_the_sole_entrys_schema() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "nav23", "Goal", GOAL_MSG);
    write_yaml(
        tmp.path(),
        "Goal.yaml",
        "schemas:\n  Other:\n    fields:\n      uint32 a:\n",
    );

    let reg = FieldRegistry::from_graph(&one_output_graph("Goal"), tmp.path());
    let other_expected = {
        let mut s = MessageSchema::new("Other");
        s.add_field(FieldDef::new("a", FieldType::U32));
        s.schema_hash()
    };
    assert_eq!(reg.topic_class(TOPIC), Some(&TopicClass::Produced));
    assert_eq!(
        reg.expected_schema_hash(TOPIC),
        Some(other_expected),
        "replays with the sole entry's schema"
    );
    assert!(
        reg.decoder_for(TOPIC).is_some(),
        "a decoder over that schema"
    );
    assert!(
        reg.resolve_field(TOPIC, "a").is_ok(),
        "the entry's field resolves"
    );
    assert!(
        reg.resolve_field(TOPIC, "x").is_err(),
        "the store twin's field does not (never the store)"
    );
}

/// A multi-entry stem file (`Multi.yaml`
/// declaring `A` and `B`) beside a channel labelled `Multi` stays a
/// REGISTERED Produced topic (premise) with no schema — and the registry says
/// why. The workspace holds ONLY this file, so the captured warning can come
/// from no other stem claim. The SPELLING surfaces take the
/// `workspace_lookup` verdict instead: a multi-entry stem names one
/// FILE, so `schema info` renders it and the port surfaces accept it; what the
/// registry cannot do is pick WHICH of its entries a channel labelled `Multi`
/// carries, and that is the refusal pinned below.
#[test]
#[tracing_test::traced_test]
fn r23_a_multi_entry_stem_is_refused_loudly_at_replay() {
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(
        tmp.path(),
        "Multi.yaml",
        "schemas:\n  A:\n    fields:\n      uint32 a:\n  B:\n    fields:\n      uint32 b:\n",
    );
    let schemas_dir = tmp.path().join("schemas");

    // The SPELLING tier is `workspace_lookup`: a stem file
    // that declares several entries names exactly ONE file, so the stem
    // RESOLVES — the refusal for such a spelling lives at the graph
    // identity layer (`WorkspaceSchemaClaim::AmbiguousStem`), not here.
    assert!(cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "Multi").unwrap());
    assert!(matches!(
        cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Multi")
            .expect("the stem names ONE file")
            .provenance,
        cerulion_cli_engine::schema_cmd::PortSchemaProvenance::Workspace { .. }
    ));
    // CONTROL: the file's own entries are fine.
    assert!(cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "A").unwrap());
    // `schema info Multi` names ONE file: the file view renders it whole,
    // each entry judged on its own.
    let view = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "Multi")
        .expect("the stem names ONE file");
    assert_eq!(view.source_detail.as_deref(), Some("schemas/Multi.yaml"));
    assert_eq!(
        view.result
            .entries
            .iter()
            .map(|e| e.name.as_str())
            .collect::<Vec<_>>(),
        vec!["A", "B"]
    );

    let multi = FieldRegistry::from_graph(&one_output_graph("Multi"), tmp.path());
    assert_eq!(
        multi.topic_class(TOPIC),
        Some(&TopicClass::Produced),
        "premise: registered, then degraded"
    );
    assert_eq!(
        multi.expected_schema_hash(TOPIC),
        None,
        "a multi-entry stem binds nothing — never a guessed entry"
    );
    assert!(multi.decoder_for(TOPIC).is_none());
    assert!(
        logs_contain("claims this name ambiguously"),
        "the registry says so with the ONE message every surface uses"
    );
}

/// The stem binding on the HASH-MAP surface, under the
/// ambiguity-refusal rule: with `Goal.yaml` declaring only `Other` beside
/// `other.yaml` declaring an entry `Goal`, a bare stem rule would bind the validated port
/// `Goal` to `Goal.yaml`'s document. `Goal` is REFUSED, both sources
/// named: validation errs, the hash map carries no `Goal` (said in its own
/// context), `schema list` marks `other.yaml`'s `Goal` row with the refusal,
/// and the file's own `Other`, declared once, still binds (the control).
#[test]
#[tracing_test::traced_test]
fn r27_a_stem_beside_another_files_entry_of_that_name_is_refused_on_the_hash_map() {
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(
        tmp.path(),
        "Goal.yaml",
        "schemas:\n  Other:\n    fields:\n      uint32 a:\n",
    );
    write_yaml(
        tmp.path(),
        "other.yaml",
        "schemas:\n  Goal:\n    fields:\n      uint32 z:\n      uint32 y:\n",
    );
    let schemas_dir = tmp.path().join("schemas");
    let other_expected = {
        let mut s = MessageSchema::new("Other");
        s.add_field(FieldDef::new("a", FieldType::U32));
        s.schema_hash()
    };
    // `Goal` names both a schema ENTRY
    // (other.yaml) and a FILE of that stem (Goal.yaml) — REFUSED, both named;
    // the stem-authoritative pick is gone.
    let err = cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Goal")
        .expect_err("a stem beside another file's entry of that name is refused")
        .to_string();
    for needle in [
        "'Goal' is ambiguous in this workspace — defined by:",
        "schemas/other.yaml (entry Goal), schemas/Goal.yaml (file stem; entries: Other)",
        "Declare it once, or name the one you mean",
    ] {
        assert!(
            err.contains(needle),
            "refusal carries {needle:?}; got: {err}"
        );
    }
    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());
    assert_eq!(
        map.get("Goal"),
        None,
        "no entry for a refused name; got {map:?}"
    );
    assert_eq!(
        map.get("Other"),
        Some(&other_expected),
        "the file's own entry, declared once, still binds"
    );
    assert!(
        logs_contain("claims this name ambiguously") && logs_contain("schema-hash map"),
        "the hash map says the refusal in its own context"
    );
    // `schema list`: the `other.yaml` row of the refused entry name carries
    // the refusal instead of a hash; `Goal.yaml`'s `Other` row is hashed.
    let listing = cerulion_cli_engine::schema_cmd::schema_list(&schemas_dir);
    let goal_row = listing
        .workspace
        .iter()
        .find(|w| w.name == "Goal")
        .expect("other.yaml's Goal row is listed");
    assert_eq!(goal_row.schema_hash, None);
    assert_eq!(goal_row.refusal.as_deref(), Some(err.as_str()));
    let other_row = listing
        .workspace
        .iter()
        .find(|w| w.name == "Other")
        .expect("Goal.yaml's Other row is listed");
    assert_eq!(other_row.schema_hash, Some(other_expected));
    assert_eq!(other_row.refusal, None);
}

/// Replay deriving bare names from
/// post-filtered schema tiers while validation applies the port-resolution
/// rules would disagree with it: with a hostile `p1/Type` beside a valid `p2/Type` the graph
/// REFUSES bare `Type` as ambiguous but replay would pick `p2/Type`, and with a
/// `Goal.yaml` declaring only `Other` beside store `p/Goal` the graph
/// accepts `Goal` through the file stem but replay would validate it as
/// `p/Goal`. ONE resolver: the claim map (stem-authoritative YAML tier)
/// and `unique_bare_store_names` over the PARSED store feed validation, the
/// hash-divergence map, gateway serving, the recording fold and the replay
/// registry alike. The three fixtures, each asserted on
/// graph / hash map / serving / replay in one body so they cannot drift
/// again (the fold is pinned in `graph_cmd`'s own in-module arms — the same
/// code path): ambiguous ⇒ refused / no entry / bare pass-through /
/// schema-unavailable, NEVER a sibling pick; the stem ⇒ the sole entry on
/// every surface.
#[test]
fn r23_graph_hash_serve_and_replay_agree_on_the_three_bare_name_fixtures() {
    // Fixture 1 — the hostile pair.
    let f1 = tempfile::tempdir().unwrap();
    write_store_msg(f1.path(), "p1", "Type", "float64[18446744073709551615] a\n");
    write_store_msg(f1.path(), "p2", "Type", "uint32 a\n");
    // Fixture 2 — the qualified-shadow triple.
    let f2 = tempfile::tempdir().unwrap();
    write_store_msg(f2.path(), "p", "Type", "uint32 a\n");
    write_store_msg(f2.path(), "other", "Type", "uint32 a\n");
    write_yaml(
        f2.path(),
        "ptype.yaml",
        "schemas:\n  p/Type:\n    fields:\n      uint32 b:\n      uint32 c:\n",
    );
    for (label, fx) in [("hostile pair", &f1), ("qualified-shadow triple", &f2)] {
        let schemas_dir = fx.path().join("schemas");
        // graph: refused as ambiguous.
        let err = cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "Type")
            .expect_err("bare `Type` is refused at validation");
        assert!(err.to_string().contains("ambiguous"), "{label}: got {err}");
        // hash map: no entry under the refused name.
        let map = graph_cmd::build_workspace_schema_hashes(fx.path());
        assert_eq!(
            map.get("Type"),
            None,
            "{label}: no hash entry for a refused name; got {map:?}"
        );
        // serving: the declared name passes through — never a sibling.
        let config = one_output_graph("Type");
        let serving =
            cerulion_cli_engine::schema_serve::build_schema_serving(&config, &schemas_dir, None);
        assert_eq!(
            serving.topic_schemas[0].schema_name, "Type",
            "{label}: serving never picks a sibling"
        );
        // replay: schema-unavailable on a REGISTERED topic — never a sibling.
        let reg = FieldRegistry::from_graph(&config, fx.path());
        assert_eq!(
            reg.topic_class(TOPIC),
            Some(&TopicClass::Produced),
            "{label}: premise"
        );
        assert_eq!(
            reg.expected_schema_hash(TOPIC),
            None,
            "{label}: replay must not pick a sibling"
        );
        assert!(
            reg.decoder_for(TOPIC).is_none(),
            "{label}: no decoder over a refused name"
        );
    }

    // Fixture 3 — the file-stem claim beside a same-named store type.
    let f3 = tempfile::tempdir().unwrap();
    write_yaml(
        f3.path(),
        "Goal.yaml",
        "schemas:\n  Other:\n    fields:\n      uint32 a:\n",
    );
    write_store_msg(f3.path(), "p", "Goal", "float64 x\nfloat64 y\n");
    let schemas_dir = f3.path().join("schemas");
    let other_expected = {
        let mut s = MessageSchema::new("Other");
        s.add_field(FieldDef::new("a", FieldType::U32));
        s.schema_hash()
    };
    // The stem file and the store both spell `Goal`, so the SPELLING is the
    // twin the one workspace lookup refuses — while the maps below
    // must still bind the stem's sole entry, never the store's document.
    assert!(
        cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Goal")
            .expect_err("stem beside a same-named store type")
            .to_string()
            .contains("schemas/p/msg/Goal.msg (store)"),
        "the refusal names the store twin"
    );
    let map = graph_cmd::build_workspace_schema_hashes(f3.path());
    assert_eq!(
        map.get("Goal").copied(),
        Some(other_expected),
        "hash map: the stem's sole entry"
    );
    let config = one_output_graph("Goal");
    let serving =
        cerulion_cli_engine::schema_serve::build_schema_serving(&config, &schemas_dir, None);
    assert_eq!(
        serving.topic_schemas[0].schema_name, "Other",
        "serving: the stem's sole entry"
    );
    let reg = FieldRegistry::from_graph(&config, f3.path());
    assert_eq!(
        reg.expected_schema_hash(TOPIC),
        Some(other_expected),
        "replay: the stem's sole entry, never `p/Goal`"
    );
    assert!(reg.resolve_field(TOPIC, "a").is_ok());
    assert!(
        reg.resolve_field(TOPIC, "x").is_err(),
        "the store's field must not resolve"
    );
}

// ---------------------------------------------------------------------------
// Owner-qualified stem bindings, scrubbed-claim masking, the
// schema-info parse gate
// ---------------------------------------------------------------------------

/// Schema provenance across the surfaces,
/// under the ambiguity-refusal rule: `Goal.yaml` declares
/// only `Other` (4 bytes) and a lexically-LATER file declares its own
/// `Other` (8 bytes). A stem rule that hands the port `Goal` the later file's
/// document is wrong, and so is resolving it through the owner. `Other` is a
/// refused duplicate and `Goal` — a stem bound to an ambiguous name — is
/// refused with it: validation errs on both (every declarer named), the
/// hash map carries neither, serving binds the declared name unserved and
/// mints nothing, replay degrades to schema-unavailable — never the owner's
/// hash, never the sibling's. `schema info Goal` keeps its FILE view (one
/// file) and carries the port refusal; `schema info Other` prints the
/// refusal.
#[test]
fn r27_a_stem_whose_sole_entry_another_file_also_declares_is_refused_everywhere() {
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(
        tmp.path(),
        "Goal.yaml",
        "schemas:\n  Other:\n    fields:\n      uint32 a:\n",
    );
    write_yaml(
        tmp.path(),
        "zz_other.yaml",
        "schemas:\n  Other:\n    fields:\n      uint32 z:\n      uint32 y:\n",
    );
    let schemas_dir = tmp.path().join("schemas");

    // The SPELLING tier is `workspace_lookup`: `Other` is
    // declared by TWO files, so it is REFUSED with every source named; `Goal`
    // names exactly ONE file, so the stem resolves — the refusal for
    // a stem whose sole entry is ambiguous elsewhere lives at the graph
    // identity layer, not on the spelling surfaces.
    let other_err = cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Other")
        .expect_err("the duplicate entry is refused")
        .to_string();
    for needle in [
        "'Other' is ambiguous in this workspace — defined by:",
        "schemas/Goal.yaml (entry Other)",
        "schemas/zz_other.yaml (entry Other)",
    ] {
        assert!(
            other_err.contains(needle),
            "refusal carries {needle:?}; got: {other_err}"
        );
    }
    assert_eq!(
        cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "Other")
            .expect_err("the gate refuses the duplicate identically")
            .to_string(),
        other_err
    );
    // `schema info Goal` names ONE file and renders its file view; `Other` —
    // a name meaning two definitions — prints the refusal.
    let view = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "Goal").unwrap();
    assert_eq!(view.source_detail.as_deref(), Some("schemas/Goal.yaml"));
    assert_eq!(view.result.entries[0].name, "Other");
    assert_eq!(
        cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "Other")
            .expect_err("a duplicated entry name prints the refusal")
            .to_string(),
        other_err
    );
    assert!(cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "Goal").unwrap());
    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());
    assert_eq!(map.get("Goal"), None, "no entry; got {map:?}");
    assert_eq!(
        map.get("Other"),
        None,
        "the duplicate entry has no entry either"
    );
    let config = one_output_graph("Goal");
    let serving =
        cerulion_cli_engine::schema_serve::build_schema_serving(&config, &schemas_dir, None);
    assert_eq!(serving.topic_schemas[0].schema_name, "Goal", "pass-through");
    assert!(!serving.schema_docs.iter().any(|d| d.qualified == "Other"));
    let reg = FieldRegistry::from_graph(&config, tmp.path());
    assert_eq!(reg.topic_class(TOPIC), Some(&TopicClass::Produced));
    assert_eq!(reg.expected_schema_hash(TOPIC), None);
    assert!(reg.decoder_for(TOPIC).is_none());
}

/// The second schema-provenance shape, on the hash map: the stem's
/// sole entry is a slash-named YAML `p/Other` whose closure escapes the
/// workspace set (it nests a built-in `std_msgs/Header`), so the hash map
/// OMITS it — beside a VALID store `p/Other`. A stem rule that finds the store's
/// hash under the surviving string would check the YAML-typed port `Goal`
/// against it. The owner-qualified lookup yields nothing for `Goal`, and
/// the string `p/Other` — which validation binds to the YAML tier — is masked
/// rather than served from the store: both checks are SKIPPED, never run
/// against a document the resolver never bound.
#[test]
fn r24_an_omitted_stem_owner_beside_a_surviving_store_key_is_skipped_in_the_hash_map() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "p", "Other", "uint32 a\n");
    write_yaml(
        tmp.path(),
        "Goal.yaml",
        "schemas:\n  p/Other:\n    fields:\n      std_msgs/Header h:\n      uint32 a:\n",
    );
    let schemas_dir = tmp.path().join("schemas");
    // PREMISE: the STEM `Goal` validates to the YAML tier; the QUALIFIED
    // spelling `p/Other` is the YAML/store TWIN, refused by the one
    // workspace lookup — so the hash map below is the only surface
    // that still has to say what it bound for either name.
    assert!(matches!(
        cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Goal")
            .unwrap()
            .provenance,
        cerulion_cli_engine::schema_cmd::PortSchemaProvenance::Workspace { .. }
    ));
    let twin = cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "p/Other")
        .expect_err("a YAML definition the .msg store also spells is refused")
        .to_string();
    assert!(
        twin.contains("'p/Other' is ambiguous in this workspace — defined by:")
            && twin.contains("schemas/p/msg/Other.msg (store)"),
        "got: {twin}"
    );

    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());
    // The owner's closure does not escape (built-ins are in the
    // resolution set), so the stem and the string both serve the YAML
    // owner's OWN hash — and the masking property this test exists for
    // still holds: the store twin's hash is NEVER what they serve.
    let yaml_expected = {
        let mut other = MessageSchema::new("p/Other");
        other.add_field(FieldDef::new(
            "h",
            FieldType::Nested {
                schema_name: "Header".to_string(),
                package: Some("std_msgs".to_string()),
                fixed: None,
            },
        ));
        other.add_field(FieldDef::new("a", FieldType::U32));
        oracle_hash(
            &mut [
                builtin_ir("builtin_interfaces", "Time"),
                builtin_ir("std_msgs", "Header"),
                other,
            ],
            "p/Other",
        )
    };
    let store_twin = {
        let mut s = MessageSchema::new_in_package("Other", "p");
        s.add_field(FieldDef::new("a", FieldType::U32));
        oracle_hash(&mut [s], "p/Other")
    };
    assert_ne!(
        yaml_expected, store_twin,
        "the twins differ, else this pins nothing"
    );
    assert_eq!(
        map.get("Goal").copied(),
        Some(yaml_expected),
        "the stem serves its OWNER's hash — never the store twin's; got {map:?}"
    );
    assert_eq!(
        map.get("p/Other").copied(),
        Some(yaml_expected),
        "a string the YAML tier claims is served from the YAML tier, never the store"
    );
}

/// The registry, the ENTRY half: a YAML
/// entry `Goal` whose RESOLVED fixed section composes past the u32 wire
/// ceiling is scrubbed from the registry's set, beside a valid store
/// `nav/Goal` bearing the same bare name. Validation binds `Goal` to the YAML
/// tier (the file parses — only its sizing failed; premise), so replay must
/// degrade to schema-unavailable — a YAML tier counted over the
/// SURVIVORS would lose the scrubbed claim, and the store alias would validate the
/// bag against a document the resolver never bound. The store definition
/// stays reachable under its own qualified name (not poisoned).
#[test]
fn r24_a_scrubbed_yaml_entry_claim_masks_the_store_tier_at_replay() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "nav", "Goal", GOAL_MSG);
    write_yaml(
        tmp.path(),
        "x.yaml",
        "schemas:\n  Inner:\n    fields:\n      float64[1048576] v:\n  Goal:\n    fields:\n      Inner[1048576] arr:\n",
    );
    let schemas_dir = tmp.path().join("schemas");
    // PREMISE: the YAML entry and the store definition spell the SAME name,
    // which the one workspace lookup refuses as the twin — so the
    // FOLD below is the surface that still has to answer for the name, and
    // what it must not do is validate a bag against the store's document.
    assert!(
        cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Goal")
            .expect_err("the spelling is the YAML/store twin")
            .to_string()
            .contains("schemas/nav/msg/Goal.msg (store)"),
        "PREMISE: the twin is refused, naming the store"
    );

    let reg = FieldRegistry::from_graph(&one_output_graph("Goal"), tmp.path());
    assert_eq!(
        reg.topic_class(TOPIC),
        Some(&TopicClass::Produced),
        "registered"
    );
    assert_eq!(
        reg.expected_schema_hash(TOPIC),
        None,
        "a scrubbed YAML claim masks the store tier — never `nav/Goal`"
    );
    assert!(reg.decoder_for(TOPIC).is_none());
    assert!(
        reg.resolve_field(TOPIC, "x").is_err(),
        "the store's field never resolves"
    );

    let by_q = FieldRegistry::from_graph(&one_output_graph("nav/Goal"), tmp.path());
    let store_expected = {
        let mut s = MessageSchema::new_in_package("Goal", "nav");
        s.add_field(FieldDef::new("x", FieldType::F64));
        s.add_field(FieldDef::new("y", FieldType::F64));
        oracle_hash(&mut [s], "nav/Goal")
    };
    assert_eq!(
        by_q.expected_schema_hash(TOPIC),
        Some(store_expected),
        "the store definition is not poisoned under its qualified name"
    );
}

/// The registry, the QUALIFIED-string half of the same masking —
/// a PARITY pin: a slash-named YAML entry `p/Other` composes past
/// the u32 ceiling and is scrubbed, beside a valid store `p/Other`.
/// Validation binds the string to the YAML tier (premise). The registry
/// already answers schema-unavailable: the YAML definition is the string's
/// LAST bearer, so its failure trips the shared winner sweep, which removes
/// the store twin with it (so this shape needs no masking set of
/// its own). Pinned so the qualified half stays
/// as masked as the bare half (`r24_a_scrubbed_yaml_entry_claim_masks_the_
/// store_tier_at_replay`, which IS load-bearing) — and beside the hash map's
/// and the fold's own masking, which have no winner sweep to lean on.
#[test]
fn r24_a_scrubbed_qualified_yaml_claim_masks_the_store_twin_at_replay() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "p", "Other", "uint32 a\n");
    write_yaml(
        tmp.path(),
        "x.yaml",
        "schemas:\n  Inner:\n    fields:\n      float64[1048576] v:\n  p/Other:\n    fields:\n      Inner[1048576] arr:\n",
    );
    let schemas_dir = tmp.path().join("schemas");
    // PREMISE: the qualified spelling is the YAML/store twin, refused by the
    // one workspace lookup; the FOLD below still has to answer for
    // it, and must not fall through to the store definition.
    assert!(
        cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "p/Other")
            .expect_err("the spelling is the YAML/store twin")
            .to_string()
            .contains("schemas/p/msg/Other.msg (store)"),
        "PREMISE: the twin is refused, naming the store"
    );

    let reg = FieldRegistry::from_graph(&one_output_graph("p/Other"), tmp.path());
    assert_eq!(
        reg.topic_class(TOPIC),
        Some(&TopicClass::Produced),
        "registered"
    );
    assert_eq!(
        reg.expected_schema_hash(TOPIC),
        None,
        "a scrubbed qualified YAML claim masks the store twin — never its hash"
    );
    assert!(reg.decoder_for(TOPIC).is_none());
}

/// The registry, the STEM half: `Goal.yaml`'s sole entry `Other`
/// composes past the u32 ceiling and is scrubbed, beside a valid store
/// `nav/Goal`. A stem loop whose `Some(_) => {}` arm leaves the store
/// alias `Goal` → `nav/Goal` in the bare index replays a YAML-typed port
/// against the store; the scrubbed owner masks it instead — schema-unavailable.
#[test]
fn r24_a_scrubbed_stem_owner_masks_the_store_tier_at_replay() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "nav", "Goal", GOAL_MSG);
    write_yaml(
        tmp.path(),
        "Goal.yaml",
        "schemas:\n  Other:\n    fields:\n      Inner[1048576] arr:\n",
    );
    write_yaml(
        tmp.path(),
        "chain.yaml",
        "schemas:\n  Inner:\n    fields:\n      float64[1048576] v:\n",
    );
    let schemas_dir = tmp.path().join("schemas");
    // PREMISE: the stem claims `Goal` for the YAML tier, and the `.msg` store
    // spells it too — so the spelling is refused as the TWIN while
    // the FOLD below still has to say what it bound for the name.
    assert!(
        cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Goal").is_err(),
        "PREMISE: the stem/store twin is refused at the spelling surfaces"
    );

    let reg = FieldRegistry::from_graph(&one_output_graph("Goal"), tmp.path());
    assert_eq!(
        reg.topic_class(TOPIC),
        Some(&TopicClass::Produced),
        "registered"
    );
    assert_eq!(
        reg.expected_schema_hash(TOPIC),
        None,
        "a scrubbed stem owner masks the store tier — never `nav/Goal`"
    );
    assert!(reg.decoder_for(TOPIC).is_none());
    assert!(reg.resolve_field(TOPIC, "x").is_err());
}

/// Schema provenance at the registry, under the
/// ambiguity-refusal rule: with `Goal.yaml`
/// declaring only `Other` beside `other.yaml` declaring an entry `Goal`,
/// resolving through the owner would replay `Goal` with the owner's `Other`. The name is
/// REFUSED at validation (premise, the refusal's own text), so replay
/// resolves NOTHING for it — never the owner's `Other`, never `other.yaml`'s
/// `Goal` — while the file's own entry, declared once, replays (control).
#[test]
fn r27_a_stem_beside_another_files_entry_is_refused_at_replay() {
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(
        tmp.path(),
        "Goal.yaml",
        "schemas:\n  Other:\n    fields:\n      uint32 a:\n",
    );
    write_yaml(
        tmp.path(),
        "other.yaml",
        "schemas:\n  Goal:\n    fields:\n      uint32 z:\n      uint32 y:\n",
    );
    // Refused at validation (a stem beside
    // another file's entry of that name), so replay resolves NOTHING for it —
    // never the owner's `Other`, never `other.yaml`'s `Goal`.
    let premise =
        cerulion_cli_engine::schema_cmd::port_schema_exists(&tmp.path().join("schemas"), "Goal")
            .expect_err("PREMISE: refused at validation")
            .to_string();
    assert!(
        premise.contains("'Goal' is ambiguous in this workspace — defined by:")
            && premise.contains(
                "schemas/other.yaml (entry Goal), schemas/Goal.yaml (file stem; entries: Other)"
            ),
        "the ambiguity refusal, not some other error; got: {premise}"
    );
    let reg = FieldRegistry::from_graph(&one_output_graph("Goal"), tmp.path());
    assert_eq!(reg.topic_class(TOPIC), Some(&TopicClass::Produced));
    assert_eq!(reg.expected_schema_hash(TOPIC), None, "schema-unavailable");
    let err = reg.resolve_field(TOPIC, "a").unwrap_err();
    assert!(err.schema_unavailable, "never the owner's field");
    assert!(
        reg.resolve_field(TOPIC, "z").is_err(),
        "never the other file's field"
    );
    // CONTROL: the file's own entry, declared once, replays.
    let other_expected = {
        let mut s = MessageSchema::new("Other");
        s.add_field(FieldDef::new("a", FieldType::U32));
        s.schema_hash()
    };
    let other = FieldRegistry::from_graph(&one_output_graph("Other"), tmp.path());
    assert_eq!(other.expected_schema_hash(TOPIC), Some(other_expected));
}

/// Validation bypass: `schema_info` — reached
/// DIRECTLY by `schema_info_unified` for a present `<name>.yaml` — must not accept an
/// empty entry key (or coerce a non-string key to `"unknown"`), or `schema
/// info Foo` displays a blank schema from a file validation, listing and
/// serving all refuse. It routes through the ONE parse gate first: both
/// shapes are refused with the gate's own message, on both entry points.
#[test]
fn r24_schema_info_refuses_what_the_parse_gate_refuses() {
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(
        tmp.path(),
        "Foo.yaml",
        "schemas:\n  \"\":\n    fields:\n      uint32 a:\n",
    );
    write_yaml(
        tmp.path(),
        "Bar.yaml",
        "schemas:\n  123:\n    fields:\n      uint32 a:\n",
    );
    let schemas_dir = tmp.path().join("schemas");
    let err = cerulion_cli_engine::schema_cmd::schema_info(&schemas_dir, "Foo")
        .map(|_| ())
        .expect_err("an empty entry key is refused by `schema_info`");
    assert!(err.to_string().contains("non-empty"), "got: {err}");
    let err = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "Foo")
        .map(|_| ())
        .expect_err("…and by `schema_info_unified`'s stem tier");
    assert!(err.to_string().contains("non-empty"), "got: {err}");
    let err = cerulion_cli_engine::schema_cmd::schema_info(&schemas_dir, "Bar")
        .map(|_| ())
        .expect_err("a non-string entry key is refused, never coerced to `unknown`");
    assert!(err.to_string().contains("not a string"), "got: {err}");
    // Control: a well-formed file still displays.
    write_yaml(
        tmp.path(),
        "Ok.yaml",
        "schemas:\n  Ok:\n    fields:\n      uint32 a:\n",
    );
    let info = cerulion_cli_engine::schema_cmd::schema_info(&schemas_dir, "Ok").unwrap();
    assert_eq!(info.entries[0].name, "Ok");
}

// ===========================================================================
// The wire ceiling counts the frame; a YAML-claimed store string
// ===========================================================================

/// Wire-size validation: a size surface that
/// judges the fixed section alone (`<= u32::MAX`) is wrong, because the frame a
/// publisher writes is the 32-byte `WireHeader`, the fixed section, then one
/// 8-byte offset entry per variable field — so a fixed section of `u32::MAX`
/// bytes would pass every preflight and reach the recording map as a
/// descriptor no publisher can emit. The rule is core's
/// `frame_prefix_exceeds_wire` (header + fixed section + offset table must
/// fit the `u32` `total_size`), consulted at the DECLARED moment with the
/// offset-table FLOOR (a nested reference may still inline) and at the
/// RESOLVED moment with the exact count. Pinned as a BOTH-SIDES boundary on
/// the `schema info` reader, the served hash bindings (declared retain +
/// resolved filter) and the replay registry (the recording fold's twin
/// lives beside the fold): exactly at the ceiling is served with its exact
/// size, one byte past is refused — for a zero-variable schema (`At` /
/// `Over` around `u32::MAX − 32`), a one-variable schema (`VarAt` /
/// `VarOver` around `u32::MAX − 40`, the offset entry), the fixed-section-only extreme
/// (`Max`, exactly `u32::MAX`, refused on every surface), a composition
/// that crosses the ceiling only through resolution (`WrapAt` fits,
/// `WrapOver` lands one byte past through padding + an inlined 8-byte
/// `Small`), `Tight`, which pins the DECLARED floor (counting its
/// not-yet-resolved `Tiny t` as an offset entry would refuse it at
/// preflight, yet resolution inlines the 4-byte target and the frame fits),
/// and `VarNestAt` / `VarNestOver`, which pin the RESOLVED moment's EXACT
/// count (`Blob` has a string, so `Blob b` stays a real offset entry after
/// resolution — the floor would not count it — and one byte past
/// `u32::MAX − 40` is refused where the floor would accept). A `uint8` run
/// places every boundary byte-exactly (align 1, no padding of its own).
#[test]
fn r26_the_wire_ceiling_counts_the_header_and_the_offset_table_on_every_surface() {
    const AT: usize = u32::MAX as usize - 32; // 4_294_967_263
    const VAR_AT: usize = AT - 8; // one offset entry
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "p26", "At", &format!("uint8[{AT}] a\n"));
    write_store_msg(tmp.path(), "p26", "Over", &format!("uint8[{}] a\n", AT + 1));
    write_store_msg(
        tmp.path(),
        "p26",
        "Max",
        &format!("uint8[{}] a\n", u32::MAX),
    );
    write_store_msg(
        tmp.path(),
        "p26",
        "VarAt",
        &format!("uint8[{VAR_AT}] a\nstring s\n"),
    );
    write_store_msg(
        tmp.path(),
        "p26",
        "VarOver",
        &format!("uint8[{}] a\nstring s\n", VAR_AT + 1),
    );
    write_store_msg(tmp.path(), "p26", "Small", "uint64 v\n");
    write_store_msg(
        tmp.path(),
        "p26",
        "WrapAt",
        &format!("uint8[{}] a\nSmall s\n", AT - 16),
    );
    write_store_msg(
        tmp.path(),
        "p26",
        "WrapOver",
        &format!("uint8[{}] a\nSmall s\n", AT - 8),
    );
    write_store_msg(tmp.path(), "p26", "Tiny", "uint32 v\n");
    write_store_msg(
        tmp.path(),
        "p26",
        "Tight",
        &format!("uint8[{}] a\nTiny t\n", AT - 7),
    );
    write_store_msg(tmp.path(), "p26", "Blob", "string s\n");
    write_store_msg(
        tmp.path(),
        "p26",
        "VarNestAt",
        &format!("uint8[{VAR_AT}] a\nBlob b\n"),
    );
    write_store_msg(
        tmp.path(),
        "p26",
        "VarNestOver",
        &format!("uint8[{}] a\nBlob b\n", VAR_AT + 1),
    );
    let schemas_dir = tmp.path().join("schemas");

    // Hand oracles: each accepted definition's resolved size and hash.
    let ir = |name: &str, text: &str| {
        cerulion_core::codegen::parse_rosmsg(text, name, Some("p26")).unwrap()
    };
    let accepted: Vec<(&str, usize, u64)> = vec![
        (
            "p26/At",
            AT,
            ir("At", &format!("uint8[{AT}] a\n")).schema_hash(),
        ),
        (
            "p26/VarAt",
            VAR_AT,
            ir("VarAt", &format!("uint8[{VAR_AT}] a\nstring s\n")).schema_hash(),
        ),
        (
            // AT − 16 = …247; the u64 lands at …248 (align 8) and ends …256.
            "p26/WrapAt",
            AT - 7,
            oracle_hash(
                &mut [
                    ir("Small", "uint64 v\n"),
                    ir("WrapAt", &format!("uint8[{}] a\nSmall s\n", AT - 16)),
                ],
                "p26/WrapAt",
            ),
        ),
        (
            // AT − 7 = …256; the u32 lands there (align 4) and ends …260.
            "p26/Tight",
            AT - 3,
            oracle_hash(
                &mut [
                    ir("Tiny", "uint32 v\n"),
                    ir("Tight", &format!("uint8[{}] a\nTiny t\n", AT - 7)),
                ],
                "p26/Tight",
            ),
        ),
        (
            // `Blob b` stays variable: the run is the whole fixed section and
            // the reference is one offset entry — exactly at the ceiling.
            "p26/VarNestAt",
            VAR_AT,
            oracle_hash(
                &mut [
                    ir("Blob", "string s\n"),
                    ir("VarNestAt", &format!("uint8[{VAR_AT}] a\nBlob b\n")),
                ],
                "p26/VarNestAt",
            ),
        ),
    ];
    let refused = [
        "p26/Over",
        "p26/VarOver",
        "p26/Max",
        "p26/WrapOver",
        "p26/VarNestOver",
    ];

    // `schema info`: the resolved-layout reader.
    for (name, size, hash) in &accepted {
        let info = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, name)
            .unwrap_or_else(|e| panic!("'{name}': at or under the ceiling resolves; got {e}"));
        let entry = &info.result.entries[0];
        assert!(entry.wire_fixed_size_resolved, "'{name}': resolved");
        assert_eq!(entry.wire_fixed_size, *size, "'{name}': exact size");
        assert_eq!(entry.schema_hash, Some(*hash), "'{name}': exact hash");
    }
    for name in refused {
        let err = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, name)
            .expect_err("a frame prefix past the u32 total_size is refused");
        let msg = err.to_string();
        assert!(
            msg.contains("resolution preflight rejected"),
            "'{name}': refused, never served a size no frame can carry; got {msg}"
        );
    }
    // The declared-moment refusal names the header (the reason a `Max`
    // declaration — under a fixed-section-only ceiling — is unrepresentable).
    let max_err = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "p26/Max")
        .unwrap_err()
        .to_string();
    assert!(
        max_err.contains("32-byte header"),
        "the refusal explains the frame prefix; got {max_err}"
    );

    // Gateway serving: the served hash bindings apply the DECLARED ceiling
    // to the served set AND the RESOLVED ceiling to the bindings — every
    // refused definition binds no hash (`WrapOver` / `VarNestOver` cross the
    // wire only after resolution); every definition that fits binds its
    // exact hash.
    let serving = cerulion_cli_engine::schema_serve::build_schema_serving(
        &one_output_graph("p26/At"),
        &schemas_dir,
        None,
    );
    let binding = |name: &str| {
        serving
            .schema_hashes
            .iter()
            .find(|h| h.qualified == name)
            .map(|h| h.schema_hash)
    };
    for (name, _, hash) in &accepted {
        assert_eq!(binding(name), Some(*hash), "'{name}': served hash binding");
    }
    for name in refused {
        assert_eq!(binding(name), None, "'{name}': no served hash binding");
    }

    // Replay: the registry's resolved leg — the expected hash and the
    // decoder's root layout for what fits; schema-unavailable past it.
    for (name, size, hash) in &accepted {
        let reg = FieldRegistry::from_graph(&one_output_graph(name), tmp.path());
        assert_eq!(
            reg.expected_schema_hash(TOPIC),
            Some(*hash),
            "'{name}': replay expects the exact hash"
        );
        let fixed = reg
            .decoder_for(TOPIC)
            .unwrap_or_else(|| panic!("'{name}': a decoder"))
            .root_layout()
            .expect("layout")
            .fixed_size;
        assert_eq!(fixed, *size, "'{name}': the decoder's root layout");
    }
    for name in refused {
        let reg = FieldRegistry::from_graph(&one_output_graph(name), tmp.path());
        assert_eq!(
            reg.topic_class(TOPIC),
            Some(&TopicClass::Produced),
            "'{name}': registered"
        );
        assert_eq!(
            reg.expected_schema_hash(TOPIC),
            None,
            "'{name}': schema-unavailable, never a hash no frame can carry"
        );
        assert!(reg.decoder_for(TOPIC).is_none(), "'{name}': no decoder");
    }
}

/// Schema identity (the collision class that
/// identity-addressed lookup, not implemented, would resolve): a bare `Type` beside the
/// SOLE store bearer `p/Type` AND a slash-named workspace YAML entry
/// `p/Type`. Validation bound the bare name to the STORE definition while
/// every string-addressed surface serves the YAML twin for the string
/// `p/Type`: the replay registry expected the YAML hash for a bag whose
/// channel carries the store hash (a FALSE schema-drift refusal, exit 2),
/// and the gateway served the YAML document for frames stamped with the
/// store hash (the desk's hash gate refuses; nothing decodes). Never wrong
/// bytes — the hash gates run first — but a valid bag unreplayable and a
/// valid topic unviewable, and both "working" only while the twins happen
/// to share a layout. The collision is REFUSED at
/// validation: the sole bearer's qualified spelling belongs to the workspace
/// tier, so the bare name binds nothing on every surface, loudly, and the
/// refusal names the remedy (the qualified spelling, which selects the
/// workspace definition everywhere). `schema info` refuses it too
/// — the three resolving surfaces share the verdict — while `schema list`
/// still shows both twins under their own files.
#[test]
fn r26_a_bare_store_name_whose_qualified_string_a_yaml_entry_claims_is_refused_on_every_surface() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "p", "Type", "uint32 a\n");
    write_yaml(
        tmp.path(),
        "ptype.yaml",
        "schemas:\n  p/Type:\n    fields:\n      uint32 b:\n      uint32 c:\n",
    );
    let schemas_dir = tmp.path().join("schemas");
    let store_hash = cerulion_core::codegen::parse_rosmsg("uint32 a\n", "Type", Some("p"))
        .unwrap()
        .schema_hash();
    let yaml_hash = {
        let mut s = MessageSchema::new("p/Type");
        s.add_field(FieldDef::new("b", FieldType::U32));
        s.add_field(FieldDef::new("c", FieldType::U32));
        s.schema_hash()
    };
    assert_ne!(
        store_hash, yaml_hash,
        "the twins must differ or this pins nothing"
    );

    // Validation: refused on BOTH port surfaces, with one message naming
    // both definitions and the remedy.
    let err = cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Type")
        .expect_err("a bare name whose store bearer's string the YAML tier claims is refused");
    let msg = err.to_string();
    for needle in [
        "'p/Type' is ambiguous in this workspace — defined by:",
        "schemas/ptype.yaml (entry p/Type)",
        "schemas/p/msg/Type.msg (store)",
    ] {
        assert!(
            msg.contains(needle),
            "the refusal carries {needle:?}; got: {msg}"
        );
    }
    let gate = cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "Type")
        .expect_err("the existence gate refuses identically");
    assert_eq!(gate.to_string(), msg, "one verdict on both port surfaces");
    // The QUALIFIED spelling is the same collision reached by its own name,
    // so it carries the SAME refusal — never a remedy pointing at a spelling
    // that is itself refused (identity-addressed
    // lookup, which would let one of them be named, is not implemented).
    assert_eq!(
        cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "p/Type")
            .expect_err("the qualified twin is refused too")
            .to_string(),
        msg
    );
    assert_eq!(
        cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "p/Type")
            .expect_err("the gate agrees")
            .to_string(),
        msg
    );

    // Hash map: no entry under the refused bare name; the string is the
    // YAML definition's.
    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());
    assert_eq!(map.get("Type"), None, "no bare alias; got {map:?}");
    assert_eq!(
        map.get("p/Type"),
        Some(&yaml_hash),
        "the YAML twin owns the string"
    );

    // Serving: the declared bare name passes through unserved — never the
    // store twin's key, which the YAML document is served under.
    let config = one_output_graph("Type");
    let serving =
        cerulion_cli_engine::schema_serve::build_schema_serving(&config, &schemas_dir, None);
    assert_eq!(serving.topic_schemas[0].schema_name, "Type");
    let served_hash = serving
        .schema_hashes
        .iter()
        .find(|h| h.qualified == "p/Type")
        .map(|h| h.schema_hash);
    assert_eq!(
        served_hash,
        Some(yaml_hash),
        "the served `p/Type` IS the YAML twin"
    );

    // Replay: schema-unavailable on a registered topic — never the store,
    // never the YAML twin.
    let reg = FieldRegistry::from_graph(&config, tmp.path());
    assert_eq!(reg.topic_class(TOPIC), Some(&TopicClass::Produced));
    assert_eq!(reg.expected_schema_hash(TOPIC), None, "no expected hash");
    assert!(reg.decoder_for(TOPIC).is_none(), "no decoder");

    // `schema info Type` refuses too, with the same message (the
    // three resolving surfaces share ONE verdict; this one must not be left
    // selecting the store while the docs name it among the refusers).
    let info_err = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "Type")
        .expect_err("schema info refuses the collision")
        .to_string();
    assert_eq!(info_err, msg, "one verdict on every resolving surface");
    // `schema list` still shows BOTH twins with their own hashes — the
    // inspection surface that names each definition by its own file.
    let listing = cerulion_cli_engine::schema_cmd::schema_list(&schemas_dir);
    assert_eq!(
        listing
            .store
            .iter()
            .find(|e| e.name == "p/Type")
            .unwrap()
            .schema_hash,
        Some(store_hash)
    );
    assert_eq!(
        listing
            .workspace
            .iter()
            .find(|e| e.name == "p/Type")
            .unwrap()
            .schema_hash,
        Some(yaml_hash)
    );

    // CONTROL (anti-tautology): without the YAML twin the same bare name
    // binds the store definition on every surface.
    let ctl = tempfile::tempdir().unwrap();
    write_store_msg(ctl.path(), "p", "Type", "uint32 a\n");
    let ctl_dir = ctl.path().join("schemas");
    let bound = cerulion_cli_engine::schema_cmd::resolve_port_schema(&ctl_dir, "Type").unwrap();
    assert_eq!(bound.schema, "p/Type");
    assert!(cerulion_cli_engine::schema_cmd::port_schema_exists(&ctl_dir, "Type").unwrap());
    assert_eq!(
        graph_cmd::build_workspace_schema_hashes(ctl.path()).get("Type"),
        Some(&store_hash),
        "the bare alias binds the store when nothing claims its string"
    );
    let serving = cerulion_cli_engine::schema_serve::build_schema_serving(&config, &ctl_dir, None);
    assert_eq!(serving.topic_schemas[0].schema_name, "p/Type");
    let reg = FieldRegistry::from_graph(&config, ctl.path());
    assert_eq!(reg.expected_schema_hash(TOPIC), Some(store_hash));
}

/// If `port_schema_exists`'s bare arm
/// consulted only the workspace file-STEM tier before the store probe — the
/// ENTRY tier reached only through the resolver fallback BELOW it — a
/// bare name a workspace YAML entry owns outright would get DIFFERENT answers from
/// the two port surfaces. Both take the ONE workspace lookup, which is
/// the property this test exists for and pins on both shapes.
///
/// WHAT that lookup answers: a bare workspace entry beside
/// ANY `<pkg>/msg/<Name>.msg` of the same name is the YAML/store TWIN, and a
/// spelling you NAME is refused rather than won by a tier. So
/// both shapes are refused, and the pin is that BOTH surfaces refuse
/// them IDENTICALLY, naming every source — the disagreement, not the
/// direction, is what this test exists to catch. The ADVISORY maps
/// still bind the workspace entry (they index top-level files only), which
/// is unreachable through a validated run and pinned as the other half.
#[test]
fn r26_both_port_surfaces_take_one_verdict_for_a_bare_workspace_entry() {
    // Shape 1: the store's sole bearer's string is a slash entry of the SAME
    // file that also declares the bare entry.
    let one = tempfile::tempdir().unwrap();
    write_store_msg(one.path(), "p", "Type", "uint32 c\n");
    write_yaml(
        one.path(),
        "both.yaml",
        "schemas:\n  Type:\n    fields:\n      uint32 a:\n  p/Type:\n    fields:\n      uint32 b:\n",
    );
    // Shape 2: the bare entry beside an AMBIGUOUS store name.
    let two = tempfile::tempdir().unwrap();
    write_store_msg(two.path(), "p", "Type", "uint32 c\n");
    write_store_msg(two.path(), "other", "Type", "uint32 d\n");
    write_yaml(
        two.path(),
        "type.yaml",
        "schemas:\n  Type:\n    fields:\n      uint32 a:\n",
    );
    for (label, fx, yaml_source, store_sources) in [
        (
            "slash twin",
            &one,
            "schemas/both.yaml (entry Type)",
            vec!["schemas/p/msg/Type.msg (store)"],
        ),
        (
            "ambiguous store",
            &two,
            "schemas/type.yaml (entry Type)",
            vec![
                "schemas/other/msg/Type.msg (store)",
                "schemas/p/msg/Type.msg (store)",
            ],
        ),
    ] {
        let schemas_dir = fx.path().join("schemas");
        let err = cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Type")
            .expect_err("a bare workspace entry the store also spells is the twin")
            .to_string();
        assert!(
            err.contains("'Type' is ambiguous in this workspace — defined by:")
                && err.contains(yaml_source),
            "{label}: the refusal names the workspace source; got: {err}"
        );
        for store_source in &store_sources {
            assert!(
                err.contains(store_source),
                "{label}: the refusal names {store_source:?}; got: {err}"
            );
        }
        // THE PIN: the existence gate gives the SAME verdict, verbatim — the
        // two surfaces reached different answers before this test existed.
        assert_eq!(
            cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "Type")
                .expect_err("the gate refuses identically")
                .to_string(),
            err,
            "{label}: one verdict on both port surfaces"
        );

        // The ADVISORY maps still bind the WORKSPACE entry (`uint32 a`) —
        // unreachable through a validated run, pinned as the other half.
        let yaml_hash = {
            let mut s = MessageSchema::new("Type");
            s.add_field(FieldDef::new("a", FieldType::U32));
            s.schema_hash()
        };
        let map = graph_cmd::build_workspace_schema_hashes(fx.path());
        assert_eq!(
            map.get("Type"),
            Some(&yaml_hash),
            "{label}: the bare name is the workspace entry's on the hash map; got {map:?}"
        );
        let reg = FieldRegistry::from_graph(&one_output_graph("Type"), fx.path());
        assert_eq!(
            reg.expected_schema_hash(TOPIC),
            Some(yaml_hash),
            "{label}: replay expects the workspace entry"
        );
    }
}

/// Predicate consistency for a nested stem twin.
///
/// Judging every store claim over the ENTRY claims of the TOP-LEVEL
/// workspace YAML files alone leaves a nested `schemas/<pkg>/<Type>.yaml` in no
/// surface's claims: the bare port binds the store everywhere alike, and the
/// QUALIFIED spelling drifts — the resolver's qualified probe finds the
/// nested file while string maps that never walked it serve the
/// STORE.
///
/// The workspace lookup walks nested stems and refuses a YAML definition
/// the store also spells, so BOTH spellings of this shape are refused
/// and no graph can declare either name. The
/// ADVISORY maps still alias the store (they index top-level files only and
/// cannot pay a per-spelling walk), which is unreachable through a
/// validated run rather than a disagreement a user can hit; the test pins
/// both halves so a future map that does walk nested files re-pins here.
#[test]
fn r26_a_nested_stem_yaml_twin_is_refused_on_both_spellings() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "p", "Type", "uint32 a\n");
    let nested = tmp.path().join("schemas").join("p");
    std::fs::create_dir_all(&nested).unwrap();
    std::fs::write(
        nested.join("Type.yaml"),
        "schemas:\n  p/Type:\n    fields:\n      uint32 b:\n      uint32 c:\n",
    )
    .unwrap();
    let schemas_dir = tmp.path().join("schemas");
    let store_hash = cerulion_core::codegen::parse_rosmsg("uint32 a\n", "Type", Some("p"))
        .unwrap()
        .schema_hash();

    // BOTH spellings are the same collision, so both carry the same refusal.
    let err = cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "p/Type")
        .expect_err("the nested stem file is a twin of the store definition")
        .to_string();
    assert!(
        err.contains("'p/Type' is ambiguous in this workspace — defined by:")
            && err.contains("schemas/p/Type.yaml (file stem; entries: p/Type)")
            && err.contains("schemas/p/msg/Type.msg (store)"),
        "got: {err}"
    );
    for spelling in ["Type", "p/Type"] {
        assert_eq!(
            cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, spelling)
                .expect_err("refused")
                .to_string(),
            err,
            "'{spelling}': one condition, one message"
        );
        assert_eq!(
            cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, spelling)
                .expect_err("the gate refuses it identically")
                .to_string(),
            err,
            "'{spelling}': the gate agrees with the resolver"
        );
    }

    // The ADVISORY maps still alias the store under both spellings — the
    // claims index walks top-level files only. Unreachable through a
    // validated run; pinned so a nested-aware map re-pins here.
    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());
    assert_eq!(
        map.get("Type"),
        Some(&store_hash),
        "the bare alias is still the store's"
    );
    assert_eq!(
        map.get("p/Type"),
        Some(&store_hash),
        "and so is the string — the maps never walk nested files"
    );
    let config = one_output_graph("Type");
    let serving =
        cerulion_cli_engine::schema_serve::build_schema_serving(&config, &schemas_dir, None);
    assert_eq!(serving.topic_schemas[0].schema_name, "p/Type");
    let reg = FieldRegistry::from_graph(&config, tmp.path());
    assert_eq!(reg.expected_schema_hash(TOPIC), Some(store_hash));
}

// ===========================================================================
// Every hash/size surface behind the ONE wire preflight
// ===========================================================================

/// One workspace YAML entry text whose declared fixed section is exactly
/// `bytes` bytes — `full` fields of `uint8[MAX_FIXED_ARRAY_LEN]` (the YAML
/// parser's per-dimension cap) plus one `uint8[rest]` — so a boundary can be
/// placed byte-exactly on the YAML tier too.
fn yaml_entry_of_fixed_bytes(name: &str, bytes: usize) -> (String, MessageSchema) {
    const CAP: usize = cerulion_cli_engine::schema_cmd::MAX_FIXED_ARRAY_LEN;
    let full = bytes / CAP;
    let rest = bytes % CAP;
    let mut text = format!("  {name}:\n    fields:\n");
    let mut ir = MessageSchema::new(name);
    for i in 0..full {
        text.push_str(&format!("      uint8[{CAP}] f{i}:\n"));
        ir.add_field(FieldDef::new(
            format!("f{i}"),
            FieldType::FixedArray {
                element_type: Box::new(FieldType::U8),
                length: CAP,
            },
        ));
    }
    text.push_str(&format!("      uint8[{rest}] tail:\n"));
    ir.add_field(FieldDef::new(
        "tail",
        FieldType::FixedArray {
            element_type: Box::new(FieldType::U8),
            length: rest,
        },
    ));
    (text, ir)
}

/// Wire limit: a `schema info` that
/// hashes and sizes a workspace YAML entry BEFORE the shared preflight, and a
/// `schema list` that hashes every YAML entry without the frame-prefix rule, let
/// a declared-over-ceiling YAML entry (declarable: the parser caps one
/// dimension at 2^20, and 4096 such `uint8` fields sit past `u32::MAX − 32`)
/// render its parse-time hash and size while every other size surface
/// refuses it. Both run the ONE preflight first: `schema info` REFUSES
/// the entry with the store tier's own rejection message, `schema list`
/// lists it UNHASHED (the store row's marker), the graph-run hash map omits
/// it, and BOTH port surfaces refuse a port typed after it — while the
/// entry exactly AT the ceiling keeps its hash, its resolved size and its
/// port on every surface (the both-sides boundary, on the YAML tier).
#[test]
fn r27_schema_info_and_list_refuse_an_over_ceiling_yaml_entry_before_hashing_it() {
    const AT: usize = u32::MAX as usize - 32;
    let tmp = tempfile::tempdir().unwrap();
    let (at_text, at_ir) = yaml_entry_of_fixed_bytes("AtCeiling", AT);
    let (over_text, _) = yaml_entry_of_fixed_bytes("Over", AT + 1);
    write_yaml(tmp.path(), "at.yaml", &format!("schemas:\n{at_text}"));
    write_yaml(tmp.path(), "over.yaml", &format!("schemas:\n{over_text}"));
    let schemas_dir = tmp.path().join("schemas");
    let at_hash = at_ir.schema_hash();

    // `schema info`: at the ceiling resolves with its exact size and hash.
    let at = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "AtCeiling")
        .expect("exactly at the ceiling resolves");
    assert_eq!(at.result.entries[0].schema_hash, Some(at_hash));
    assert!(at.result.entries[0].wire_fixed_size_resolved);
    assert_eq!(at.result.entries[0].wire_fixed_size, AT);
    // One byte past: REFUSED, never a parse-time hash — by entry name AND
    // by file stem (the two workspace tiers reach the same loop). The file
    // is named in the refusal, spelled as the tier that found it spells it
    // (`Over.yaml` on a case-insensitive filesystem, where the stem tier
    // matches the entry-name lookup; `over.yaml` on Linux) — the needle
    // does not encode the case.
    for lookup in ["Over", "over"] {
        let err = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, lookup)
            .expect_err("an over-ceiling YAML entry is refused")
            .to_string();
        for needle in [
            "schema 'Over' (schemas/",
            ".yaml) has no wire layout",
            "resolution preflight rejected",
            "32-byte header",
        ] {
            assert!(
                err.contains(needle),
                "'{lookup}': refusal carries {needle:?}; got: {err}"
            );
        }
    }

    // `schema list`: the row lists unhashed, like a rejected store row.
    let listing = cerulion_cli_engine::schema_cmd::schema_list(&schemas_dir);
    let row = |name: &str| listing.workspace.iter().find(|e| e.name == name).unwrap();
    assert_eq!(row("AtCeiling").schema_hash, Some(at_hash));
    assert_eq!(row("Over").schema_hash, None, "no hash no frame can carry");
    let rendered = listing.to_string();
    assert!(
        rendered.contains("Over  (unhashable: rejected by the size preflight"),
        "the workspace row renders the unhashable marker; got:\n{rendered}"
    );
    assert!(
        rendered.contains(&format!("AtCeiling  0x{at_hash:016x}")),
        "the at-ceiling row keeps its hash; got:\n{rendered}"
    );

    // The graph-run hash map: the same verdict (declared moment).
    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());
    assert_eq!(map.get("AtCeiling"), Some(&at_hash));
    assert_eq!(
        map.get("Over"),
        None,
        "no map entry — the check is skipped, loudly"
    );

    // Both port surfaces: a port typed after the refused entry is refused
    // with the SAME message (both bind through the ONE `workspace_lookup`
    // and size what it bound through the ONE `workspace_binding_preflight`).
    let resolver_err = cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Over")
        .expect_err("the resolver refuses a port typed after a refused entry")
        .to_string();
    let gate_err = cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "Over")
        .expect_err("the gate refuses it identically, never accepts a name it never sized")
        .to_string();
    assert_eq!(gate_err, resolver_err);
    assert!(resolver_err.contains("resolution preflight rejected"));
    let ok =
        cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "AtCeiling").unwrap();
    assert!(matches!(
        ok.provenance,
        cerulion_cli_engine::schema_cmd::PortSchemaProvenance::Workspace { .. }
    ));
    assert!(
        cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "AtCeiling").unwrap()
    );
}

/// The graph-run hash map (`build_workspace_schema_hashes`) must not
/// hash every definition that survives the arithmetic preflight, wire
/// ceiling or not — a `.msg` `Over` / `Max`, or a `WrapOver` that crosses
/// only after inlining, would get a map entry for a contract no cdylib can stamp.
/// It applies the ONE ceiling at the RESOLVED moment, after the
/// composed scrub (a declared-over definition is resolved-over, so both
/// shapes are caught there; a Declared retain in a fold is redundant and,
/// before the scrub, changed what the scrub saw); the port's divergence
/// check is skipped (loudly) instead. `At` / `WrapAt` keep their exact
/// hashes (hand oracles).
#[test]
fn r27_the_hash_map_omits_a_definition_the_wire_ceiling_refuses_at_either_moment() {
    const AT: usize = u32::MAX as usize - 32;
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "p27", "At", &format!("uint8[{AT}] a\n"));
    write_store_msg(tmp.path(), "p27", "Over", &format!("uint8[{}] a\n", AT + 1));
    write_store_msg(
        tmp.path(),
        "p27",
        "Max",
        &format!("uint8[{}] a\n", u32::MAX),
    );
    write_store_msg(tmp.path(), "p27", "Small", "uint64 v\n");
    write_store_msg(
        tmp.path(),
        "p27",
        "WrapAt",
        &format!("uint8[{}] a\nSmall s\n", AT - 16),
    );
    write_store_msg(
        tmp.path(),
        "p27",
        "WrapOver",
        &format!("uint8[{}] a\nSmall s\n", AT - 8),
    );
    let ir = |name: &str, text: &str| {
        cerulion_core::codegen::parse_rosmsg(text, name, Some("p27")).unwrap()
    };
    let at_hash = ir("At", &format!("uint8[{AT}] a\n")).schema_hash();
    let wrap_at_hash = oracle_hash(
        &mut [
            ir("Small", "uint64 v\n"),
            ir("WrapAt", &format!("uint8[{}] a\nSmall s\n", AT - 16)),
        ],
        "p27/WrapAt",
    );

    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());
    assert_eq!(map.get("p27/At"), Some(&at_hash), "at the ceiling: hashed");
    assert_eq!(
        map.get("p27/WrapAt"),
        Some(&wrap_at_hash),
        "fits after inlining: hashed"
    );
    for name in ["p27/Over", "p27/Max", "p27/WrapOver"] {
        assert_eq!(
            map.get(name),
            None,
            "'{name}': no entry for a contract no frame can carry; got {map:?}"
        );
    }
}

/// The QUALIFIED spelling: a slash-named entry
/// `p/Type` declared by two workspace files is a duplicate like any other —
/// the qualified port is refused on both surfaces naming both files, the
/// string maps carry nothing under it, replay resolves nothing for it, and
/// the bare store twin `Type` stays refused (the string is YAML-claimed,
/// ambiguously — the collision refusal names the claiming files).
#[test]
fn r27_a_duplicate_slash_named_entry_refuses_the_qualified_port_and_its_bare_store_twin() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "p", "Type", "uint32 a\n");
    write_yaml(
        tmp.path(),
        "a.yaml",
        "schemas:\n  p/Type:\n    fields:\n      uint32 b:\n",
    );
    write_yaml(
        tmp.path(),
        "b.yaml",
        "schemas:\n  p/Type:\n    fields:\n      uint32 c:\n",
    );
    let schemas_dir = tmp.path().join("schemas");

    let err = cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "p/Type")
        .expect_err("a duplicate slash-named entry is refused")
        .to_string();
    assert!(
        err.contains("'p/Type' is ambiguous in this workspace — defined by:")
            && err.contains("schemas/a.yaml (entry p/Type), schemas/b.yaml (entry p/Type)")
            && err.contains("schemas/p/msg/Type.msg (store)"),
        "the spelling surfaces name every source, the store twin included; got: {err}"
    );
    assert_eq!(
        cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "p/Type")
            .expect_err("the gate refuses it identically")
            .to_string(),
        err
    );
    // The bare twin takes the QUALIFIED string's own refusal, verbatim: the
    // store tier asks `workspace_lookup` who owns `p/Type`, so a bare port
    // is never given a remedy pointing at a spelling that is itself refused.
    let bare = cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Type")
        .expect_err("the bare store twin stays refused")
        .to_string();
    assert_eq!(bare, err, "one condition, one message");
    // `schema info` on the qualified name prints the refusal; `schema list`
    // marks both rows.
    assert_eq!(
        cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "p/Type")
            .expect_err("a duplicated qualified entry name prints the refusal")
            .to_string(),
        err
    );
    let listing = cerulion_cli_engine::schema_cmd::schema_list(&schemas_dir);
    let rows: Vec<_> = listing
        .workspace
        .iter()
        .filter(|w| w.name == "p/Type")
        .collect();
    assert_eq!(rows.len(), 2, "both files' rows are listed; got {rows:?}");
    for row in rows {
        assert_eq!(row.schema_hash, None, "{}", row.file);
        // The listing's refusal is the FOLD index's, which reads the
        // workspace-YAML tier only: same shape, same two YAML sources, and
        // WITHOUT the `.msg` store twin the spelling surfaces also name (the
        // index never loads the store — a stated divergence, one direction).
        let refusal = row.refusal.as_deref().expect("the row carries a refusal");
        assert!(
            refusal.contains("'p/Type' is ambiguous in this workspace — defined by:")
                && refusal.contains("schemas/a.yaml (entry p/Type), schemas/b.yaml (entry p/Type)"),
            "{}: {refusal}",
            row.file
        );
    }
    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());
    assert_eq!(map.get("p/Type"), None, "no string entry; got {map:?}");
    assert_eq!(map.get("Type"), None, "no bare alias; got {map:?}");
    let config = one_output_graph("p/Type");
    let serving =
        cerulion_cli_engine::schema_serve::build_schema_serving(&config, &schemas_dir, None);
    // The YAML documents of the refused string are withheld; the `.msg` store
    // twin stays served (the served set is also the dependency closure other
    // types nest through), though no port can bind it.
    let served: Vec<_> = serving
        .schema_docs
        .iter()
        .filter(|d| d.qualified == "p/Type")
        .collect();
    assert_eq!(served.len(), 1, "exactly the store twin; got {served:?}");
    assert_eq!(served[0].encoding, cerulion_core::SchemaEncoding::Msg);
    let reg = FieldRegistry::from_graph(&config, tmp.path());
    assert_eq!(reg.topic_class(TOPIC), Some(&TopicClass::Produced));
    assert_eq!(
        reg.expected_schema_hash(TOPIC),
        None,
        "never a last-wins winner"
    );
}

/// A stem file whose SOLE entry's own name is another file's STEM —
/// `Goal.yaml` declaring only `Other`, `Other.yaml` declaring only `Thing` —
/// makes `Other` a stem-vs-entry refusal; leaving `Goal` a BOUND stem to
/// the name the same map refuses would let validation accept `Goal`, the folds
/// drop its document (no hash entry, recorded 0), serving bind an
/// unserved name and replay resolve it. The claim map refuses a stem
/// bound to any refused name, to a fixpoint (a chain `A.yaml`→`B`,
/// `B.yaml`→`C`, `C.yaml`→`D` refuses `C`, then `B`, then `A`), naming the
/// entry's own sources — and every surface agrees.
#[test]
fn r27_a_stem_bound_to_an_ambiguous_entry_is_refused_to_a_fixpoint() {
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(
        tmp.path(),
        "Goal.yaml",
        "schemas:\n  Other:\n    fields:\n      uint32 a:\n",
    );
    write_yaml(
        tmp.path(),
        "Other.yaml",
        "schemas:\n  Thing:\n    fields:\n      uint32 t:\n",
    );
    let schemas_dir = tmp.path().join("schemas");
    // `Other`: an ENTRY (Goal.yaml) beside a FILE of that stem.
    let other_err = cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Other")
        .expect_err("refused")
        .to_string();
    assert!(
        other_err.contains("'Other' is ambiguous in this workspace — defined by:")
            && other_err.contains(
                "schemas/Goal.yaml (entry Other), schemas/Other.yaml (file stem; entries: Thing)"
            ),
        "got: {other_err}"
    );
    // `Goal` names exactly ONE file, so the SPELLING surfaces resolve it
    // — the fixpoint below is what no MAP can do: pick a document
    // for a stem whose sole entry is a name nothing binds.
    assert!(cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "Goal").unwrap());
    assert!(matches!(
        cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Goal")
            .expect("a stem naming one file resolves")
            .provenance,
        cerulion_cli_engine::schema_cmd::PortSchemaProvenance::Workspace { .. }
    ));
    // `Thing` (Other.yaml's own entry, declared once) binds everywhere.
    let thing_expected = {
        let mut s = MessageSchema::new("Thing");
        s.add_field(FieldDef::new("t", FieldType::U32));
        s.schema_hash()
    };
    assert!(cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "Thing").unwrap());
    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());
    assert_eq!(map.get("Goal"), None, "got {map:?}");
    assert_eq!(map.get("Other"), None, "got {map:?}");
    assert_eq!(map.get("Thing"), Some(&thing_expected));
    let config = one_output_graph("Goal");
    let serving =
        cerulion_cli_engine::schema_serve::build_schema_serving(&config, &schemas_dir, None);
    assert_eq!(
        serving.topic_schemas[0].schema_name, "Goal",
        "pass-through, never `Other`"
    );
    assert!(!serving.schema_docs.iter().any(|d| d.qualified == "Other"));
    assert!(serving.schema_docs.iter().any(|d| d.qualified == "Thing"));
    let reg = FieldRegistry::from_graph(&config, tmp.path());
    assert_eq!(reg.topic_class(TOPIC), Some(&TopicClass::Produced));
    assert_eq!(
        reg.expected_schema_hash(TOPIC),
        None,
        "never `Other`'s hash"
    );

    // The chain: refusing `C` refuses `B`, refusing `B` refuses `A`.
    let chain = tempfile::tempdir().unwrap();
    write_yaml(
        chain.path(),
        "A.yaml",
        "schemas:\n  B:\n    fields:\n      uint32 b:\n",
    );
    write_yaml(
        chain.path(),
        "B.yaml",
        "schemas:\n  C:\n    fields:\n      uint32 c:\n",
    );
    write_yaml(
        chain.path(),
        "C.yaml",
        "schemas:\n  D:\n    fields:\n      uint32 d:\n",
    );
    let chain_dir = chain.path().join("schemas");
    // The SPELLING surfaces refuse the two names that mean two definitions
    // (`B`: A.yaml's entry beside B.yaml; `C`: likewise). `A` names exactly
    // ONE file and resolves there — the FIXPOINT is the FOLD's: A binds
    // nothing, because the entry it would bind (`B`) is itself refused.
    for name in ["B", "C"] {
        assert!(
            cerulion_cli_engine::schema_cmd::port_schema_exists(&chain_dir, name).is_err(),
            "{name}: an entry beside a file of that stem is refused"
        );
    }
    assert!(cerulion_cli_engine::schema_cmd::port_schema_exists(&chain_dir, "A").unwrap());
    assert!(cerulion_cli_engine::schema_cmd::port_schema_exists(&chain_dir, "D").unwrap());
    let chain_map = graph_cmd::build_workspace_schema_hashes(chain.path());
    for name in ["A", "B", "C"] {
        assert_eq!(
            chain_map.get(name),
            None,
            "{name}: the fold binds nothing along the chain; got {chain_map:?}"
        );
    }
    assert!(chain_map.contains_key("D"), "got {chain_map:?}");

    // A file declaring no entries is no ambiguity SOURCE — it defines
    // nothing to collide with — but the EXISTENCE probe still answers with
    // the file, exactly as it does with no store present (that
    // back-compat is deliberate). The fold binds nothing under the name,
    // which is the half that matters to a recording. The stem is chosen to
    // match no built-in short name (`Empty` is `std_msgs/Empty`).
    let empty = tempfile::tempdir().unwrap();
    write_yaml(empty.path(), "Vacant.yaml", "schemas: {}\n");
    let empty_dir = empty.path().join("schemas");
    assert!(
        cerulion_cli_engine::schema_cmd::port_schema_exists(&empty_dir, "Vacant").unwrap(),
        "the stem file itself is the existence answer"
    );
    assert!(cerulion_cli_engine::schema_cmd::schema_list(&empty_dir)
        .workspace
        .is_empty());
    assert_eq!(
        graph_cmd::build_workspace_schema_hashes(empty.path()).get("Vacant"),
        None,
        "nothing to bind"
    );
}

/// The `schema info` ENTRY view and
/// the port surfaces judge the SAME thing — the one entry asked for — while
/// the FILE view refuses a file holding any over-ceiling entry. An
/// entry view that rendered the whole file would refuse a sane entry for its
/// hostile sibling, so `schema info Sane` would error while
/// `port_schema_exists("Sane")` accepts.
#[test]
fn r27_a_hostile_sibling_refuses_the_file_view_never_a_sane_entry_by_name() {
    const AT: usize = u32::MAX as usize - 32;
    let tmp = tempfile::tempdir().unwrap();
    let (over_text, _) = yaml_entry_of_fixed_bytes("Hostile", AT + 1);
    write_yaml(
        tmp.path(),
        "pair.yaml",
        &format!("schemas:\n  Sane:\n    fields:\n      uint32 a:\n{over_text}"),
    );
    let schemas_dir = tmp.path().join("schemas");
    // Entry view + both port surfaces: `Sane` is judged alone.
    let sane = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "Sane")
        .expect("a sane entry is shown by name whatever its sibling declares");
    assert_eq!(sane.result.entries.len(), 1);
    assert_eq!(sane.result.entries[0].name, "Sane");
    assert!(sane.result.entries[0].wire_fixed_size_resolved);
    assert!(cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "Sane").unwrap());
    assert!(cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Sane").is_ok());
    // The hostile entry: refused by name on every surface, one message.
    let by_name = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "Hostile")
        .expect_err("refused")
        .to_string();
    assert!(by_name.contains("schema 'Hostile' (schemas/pair.yaml) has no wire layout"));
    assert_eq!(
        cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "Hostile")
            .expect_err("the gate refuses it")
            .to_string(),
        by_name
    );
    // The FILE view: the whole file is refused ("fix the file").
    let by_stem = cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "pair")
        .expect_err("the file view refuses a file holding an over-ceiling entry")
        .to_string();
    assert_eq!(by_stem, by_name);
}

/// The two claim-map branches where a
/// stem FILE meets a DUPLICATED entry name, neither reachable from any other
/// fixture. (i) `Goal.yaml` declares `Goal` and `other.yaml` declares `Goal`
/// too: the stem file's like-named entry is one of the duplicates, so the
/// DuplicateEntry refusal stands (both files named) — a stem-authoritative
/// revert (`Goal.yaml`'s own `Goal` binds, silently) fails only
/// here. (ii) `a.yaml` and `b.yaml` declare `Goal` and `Goal.yaml`
/// declares only `Other`: the stem collides with an already-refused
/// duplicate, and the refusal names ALL THREE sources and says the entry is
/// itself declared twice. Every surface agrees on both.
#[test]
fn r27_a_stem_file_meeting_a_duplicated_entry_name_is_refused_naming_every_file() {
    // (i) the stem file declares its own name, and so does another file.
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(
        tmp.path(),
        "Goal.yaml",
        "schemas:\n  Goal:\n    fields:\n      uint32 a:\n",
    );
    write_yaml(
        tmp.path(),
        "other.yaml",
        "schemas:\n  Goal:\n    fields:\n      uint32 z:\n",
    );
    let schemas_dir = tmp.path().join("schemas");
    let err = cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Goal")
        .expect_err("the duplicate stands; the stem file's own entry is one of the duplicates")
        .to_string();
    assert!(
        err.contains("'Goal' is ambiguous in this workspace — defined by:")
            && err.contains("schemas/Goal.yaml (entry Goal), schemas/other.yaml (entry Goal)")
            && !err.contains("(file stem"),
        "the stem file that IS a declarer contributes ONCE, as that declaration; got: {err}"
    );
    assert_eq!(
        cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "Goal")
            .expect_err("the gate agrees")
            .to_string(),
        err
    );
    assert_eq!(
        cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "Goal")
            .expect_err("two definitions: the refusal, never the stem file's own")
            .to_string(),
        err
    );
    assert_eq!(
        graph_cmd::build_workspace_schema_hashes(tmp.path()).get("Goal"),
        None
    );
    let reg = FieldRegistry::from_graph(&one_output_graph("Goal"), tmp.path());
    assert_eq!(reg.expected_schema_hash(TOPIC), None);
    let serving = cerulion_cli_engine::schema_serve::build_schema_serving(
        &one_output_graph("Goal"),
        &schemas_dir,
        None,
    );
    assert_eq!(serving.topic_schemas[0].schema_name, "Goal");
    assert!(!serving.schema_docs.iter().any(|d| d.qualified == "Goal"));

    // (ii) the stem file declares another name; two other files declare the
    // stem's name.
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(
        tmp.path(),
        "a.yaml",
        "schemas:\n  Goal:\n    fields:\n      uint32 a:\n",
    );
    write_yaml(
        tmp.path(),
        "b.yaml",
        "schemas:\n  Goal:\n    fields:\n      uint32 b:\n",
    );
    write_yaml(
        tmp.path(),
        "Goal.yaml",
        "schemas:\n  Other:\n    fields:\n      uint32 o:\n",
    );
    let schemas_dir = tmp.path().join("schemas");
    let err = cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Goal")
        .expect_err("refused, every file named")
        .to_string();
    for needle in [
        "'Goal' is ambiguous in this workspace — defined by:",
        "schemas/a.yaml (entry Goal), schemas/b.yaml (entry Goal), \
         schemas/Goal.yaml (file stem; entries: Other)",
    ] {
        assert!(
            err.contains(needle),
            "refusal carries {needle:?}; got: {err}"
        );
    }
    assert_eq!(
        cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "Goal")
            .expect_err("the gate agrees")
            .to_string(),
        err
    );
    assert_eq!(
        cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "Goal")
            .expect_err("several definitions: the refusal")
            .to_string(),
        err
    );
    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());
    assert_eq!(map.get("Goal"), None, "got {map:?}");
    // The stem file's own entry, declared once, binds (control).
    assert!(cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "Other").unwrap());
    assert!(map.contains_key("Other"));
    let reg = FieldRegistry::from_graph(&one_output_graph("Goal"), tmp.path());
    assert_eq!(reg.expected_schema_hash(TOPIC), None);
}

/// Two silent failures closed: a workspace YAML file that
/// cannot be parsed CLAIMS NOTHING, said loudly by the ONE walk the port
/// surfaces derive their verdict from (a claim map that skips it silently
/// while only the entry probe warns makes `graph validate` report
/// "no such schema" for a workspace that does declare it). And a
/// present-but-broken `<name>.yaml` is a PARSE error before the refusal
/// gate: judged after it, two other files declaring an entry `<name>` made
/// the broken file read as "ambiguous" — the wrong cause, the file named by
/// nothing.
#[test]
#[tracing_test::traced_test]
fn r27_a_broken_workspace_file_claims_nothing_loudly_and_a_broken_stem_file_errs_first() {
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(
        tmp.path(),
        "broken.yaml",
        "schemas:\n  Lonely:\n    fields: [not a map\n",
    );
    write_yaml(
        tmp.path(),
        "a.yaml",
        "schemas:\n  Goal:\n    fields:\n      uint32 a:\n",
    );
    write_yaml(
        tmp.path(),
        "b.yaml",
        "schemas:\n  Goal:\n    fields:\n      uint32 b:\n",
    );
    write_yaml(
        tmp.path(),
        "Goal.yaml",
        "schemas:\n  Goal:\n    fields: [not a map\n",
    );
    let schemas_dir = tmp.path().join("schemas");
    // A name only the broken file declares: not found — and the walk says
    // why.
    let err = cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Lonely")
        .expect_err("the broken file claims nothing")
        .to_string();
    assert!(!err.contains("ambiguous"), "got: {err}");
    logs_assert(|lines: &[&str]| {
        // At WARN — a `debug!` here is the silent skip this arm exists to
        // catch (the capture is level-agnostic, so the level is asserted).
        lines
            .iter()
            .any(|l| {
                l.contains("WARN")
                    && l.contains("could not parse schema file for the workspace claim scan")
                    && l.contains("broken.yaml")
            })
            .then_some(())
            .ok_or_else(|| "the walk names the broken file at WARN".to_string())
    });
    // The broken STEM file: its parse error, never the sibling duplicates'
    // refusal, on both port surfaces.
    let stem_err = cerulion_cli_engine::schema_cmd::resolve_port_schema(&schemas_dir, "Goal")
        .expect_err("a broken stem file is a loud parse error")
        .to_string();
    // The lookup NAMES the unreadable stem file inside the refusal
    // instead of raising its parse error first (a file it cannot read may
    // define anything, so it is listed as a source rather than assumed
    // empty). Either way the property this arm exists for holds: the broken
    // file is NAMED, never silently skipped.
    assert!(
        stem_err.contains("schemas/Goal.yaml (file stem; unreadable)"),
        "the refusal names the broken stem file; got: {stem_err}"
    );
    assert_eq!(
        cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "Goal")
            .expect_err("the gate errs the same way")
            .to_string(),
        stem_err
    );
}

/// The replay registry's name
/// resolution NORMALIZES first — the same rule serving
/// applies. A recorded `pkg::Type` (the documented alternate spelling; reachable
/// only through a programmatically-built config, since `parse_graph`
/// canonicalizes `schema:`) would otherwise resolve NOTHING — detected as bare, looked
/// up in the bare index under a string no tier keys — and, for a refused
/// `pkg/Type`, dodge the refused set the same way. Both pinned against a
/// hand-built expectation.
#[test]
fn r27_the_registry_normalizes_a_recorded_double_colon_spelling_before_resolving_it() {
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(
        tmp.path(),
        "one.yaml",
        "schemas:\n  p/Type:\n    fields:\n      uint32 a:\n",
    );
    let expected = {
        let mut s = MessageSchema::new_in_package("Type", "p");
        s.add_field(FieldDef::new("a", FieldType::U32));
        s.schema_hash()
    };
    let mut config = one_output_graph("p/Type");
    config.nodes[0].outputs[0].schema = "p::Type".to_string();
    let reg = FieldRegistry::from_graph(&config, tmp.path());
    assert_eq!(
        reg.expected_schema_hash(TOPIC),
        Some(expected),
        "`p::Type` resolves the served `p/Type` definition"
    );
    // The refused twin: a second declarer makes `p/Type` a refused name, and
    // the `::` spelling must not slip past the refused set.
    write_yaml(
        tmp.path(),
        "two.yaml",
        "schemas:\n  p/Type:\n    fields:\n      uint32 b:\n",
    );
    let reg = FieldRegistry::from_graph(&config, tmp.path());
    assert_eq!(reg.topic_class(TOPIC), Some(&TopicClass::Produced));
    assert_eq!(
        reg.expected_schema_hash(TOPIC),
        None,
        "refused, whichever spelling"
    );
}

/// The claim map keys a bare stem by its
/// EXACT spelling, so `schema info` probing the file with `Path::exists` —
/// case-insensitive on APFS — would let `schema info over` render
/// `Over.yaml` on a Mac while both port surfaces refuse `over` (the ladder
/// disagreement, sign flipped). The file view is gated on the exact
/// dirent; the assertion is platform-independent (on a case-sensitive
/// filesystem it was already true) and kills the `.exists()` revert where the
/// bug lived.
#[test]
fn r27_schema_info_matches_the_port_surfaces_on_the_stems_exact_case() {
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(
        tmp.path(),
        "Over.yaml",
        "schemas:\n  Zed:\n    fields:\n      uint32 a:\n",
    );
    let schemas_dir = tmp.path().join("schemas");
    assert!(cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "Over").is_ok());
    assert!(cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "Over").unwrap());
    assert!(
        !cerulion_cli_engine::schema_cmd::port_schema_exists(&schemas_dir, "over").unwrap(),
        "the port surfaces know no `over`"
    );
    assert!(
        cerulion_cli_engine::schema_cmd::schema_info_unified(&schemas_dir, "over").is_err(),
        "so neither does `schema info`"
    );
}

/// A `pkg::Other` YAML entry is reachable from a graph — whose
/// `schema:` is `/`-normalized — under `pkg/Other`, and from its file stem,
/// and replays with the DECLARED name's hash (the one a node's build script
/// computes from the same YAML). Pre-PR the raw-keyed map never matched.
#[test]
fn r37_a_double_colon_yaml_entry_replays_under_its_canonical_key_with_its_declared_hash() {
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(
        tmp.path(),
        "Goal.yaml",
        "schemas:\n  pkg::Other:\n    fields:\n      uint32 a:\n      float64 b:\n",
    );
    let mut raw = MessageSchema::new("pkg::Other");
    raw.add_field(FieldDef::new("a", FieldType::U32));
    raw.add_field(FieldDef::new("b", FieldType::F64));
    let want = raw.schema_hash();
    for declared in ["pkg/Other", "pkg::Other", "Goal"] {
        let reg = FieldRegistry::from_graph(&one_output_graph(declared), tmp.path());
        assert_eq!(
            reg.expected_schema_hash(TOPIC),
            Some(want),
            "`schema: {declared}` resolves the entry and expects its declared-name hash"
        );
    }
}

/// A `pkg::Other` entry resolves its HASH under the canonical `pkg/Other`, but
/// the `LayoutResolver` is keyed by the DECLARED spelling, so a decoder
/// built for that root without translation reports `SchemaUnavailable` and field-level tolerance
/// validation is silently skipped. Every layout lookup translates the
/// canonical root to its declared key: the root layout materializes, a real
/// field resolves, and a wrong field fails with CANDIDATES (a schema that is
/// present), never `schema_unavailable`.
#[test]
fn r38_a_double_colon_yaml_entry_materializes_its_layout_under_its_canonical_root() {
    let tmp = tempfile::tempdir().unwrap();
    write_yaml(
        tmp.path(),
        "Goal.yaml",
        "schemas:\n  pkg::Other:\n    fields:\n      uint32 a:\n      float64 b:\n",
    );
    for declared in ["pkg/Other", "pkg::Other", "Goal"] {
        let reg = FieldRegistry::from_graph(&one_output_graph(declared), tmp.path());
        let mut decoder = reg
            .decoder_for(TOPIC)
            .unwrap_or_else(|| panic!("`schema: {declared}` yields a decoder"));
        let layout = decoder
            .root_layout()
            .unwrap_or_else(|| panic!("`schema: {declared}`: the root layout materializes"));
        let names: Vec<&str> = layout
            .fixed_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect();
        assert_eq!(names, ["a", "b"], "`schema: {declared}`");
        reg.resolve_field(TOPIC, "a")
            .unwrap_or_else(|e| panic!("`schema: {declared}`: `a` resolves: {e:?}"));
        let err = reg
            .resolve_field(TOPIC, "nope")
            .expect_err("a field the schema lacks is refused");
        assert!(
            !err.schema_unavailable,
            "`schema: {declared}`: the schema is PRESENT — the refusal names candidates, \
             never schema-unavailable: {err:?}"
        );
        assert!(
            err.candidates.iter().any(|c| c == "a"),
            "`schema: {declared}`: candidates come from the real layout: {err:?}"
        );
    }
}

/// The post-resolve
/// removal report must not record each removed definition's hash from the RESOLVED
/// clone while `from_graph`'s fixpoint applies `RemovedDefinitions::removes`
/// to its UNRESOLVED tiers — two states of one schema, and a definition
/// carrying a fixed-nested reference hashes differently across them. With
/// the other two arms also missing (the shape below), such a scrub removes
/// NOTHING, the next pass rebuilds an identical set, and the loop SPINS
/// FOREVER.
///
/// The shape needs a definition that PASSES the unresolved composed
/// preflight and FAILS the RESOLVED one — the shadowing-store fixture below cannot reach it
/// (its usize overflow is caught unresolved, where the identity source was
/// already right). `Filler` is 3.2 GB, representable; a store
/// `geometry_msgs/Vector3` of TWO of them composes to 6.4 GB — inside usize,
/// past the u32 wire ceiling — so it survives to the resolved leg. It
/// shadows the BUILT-IN `geometry_msgs/Vector3` by `(package, name)` key
/// (so `removed_keys` has a survivor) while a slash-named YAML entry holds
/// the same qualified STRING (so `fully_removed_names` has one too).
///
/// The no-progress guard means a regression DEGRADES
/// instead of hanging, so this test can pin it at all: the warn is the
/// regression's signature, and its absence is what the tier scrub buys.
#[test]
#[tracing_test::traced_test]
fn r41_a_resolved_moment_removal_scrubs_the_tiers_instead_of_spinning_the_fixpoint() {
    let tmp = tempfile::tempdir().unwrap();
    // 400_000_000 f64 = 3.2e9 bytes: representable, under the u32 ceiling.
    write_store_msg(
        tmp.path(),
        "geometry_msgs",
        "Filler",
        "float64[400000000] v\n",
    );
    // TWO of them = 6.4e9: inside usize, PAST the u32 wire ceiling — caught
    // only at the RESOLVED moment, which is the leg this test exists for.
    write_store_msg(tmp.path(), "geometry_msgs", "Vector3", "Filler[2] arr\n");
    // The slash-named YAML entry bearing the same STRING — the string winner.
    write_yaml(
        tmp.path(),
        "vec_yaml.yaml",
        "schemas:\n  geometry_msgs/Vector3:\n    fields:\n      uint32 a:\n",
    );

    let yaml_expected = {
        let mut s = MessageSchema::new("geometry_msgs/Vector3");
        s.add_field(FieldDef::new("a", FieldType::U32));
        oracle_hash(&mut [s], "geometry_msgs/Vector3")
    };

    // THE PIN: the registry terminates AND scrubs — the YAML winner serves
    // the name and its layout materializes.
    let reg = FieldRegistry::from_graph(&one_output_graph("geometry_msgs/Vector3"), tmp.path());
    assert_eq!(
        reg.expected_schema_hash(TOPIC),
        Some(yaml_expected),
        "the valid YAML string winner serves the name"
    );
    assert!(
        reg.decoder_for(TOPIC)
            .expect("a decoder for the YAML winner")
            .root_layout()
            .is_some(),
        "the layout materializes — the hostile store copy is gone"
    );

    // PREMISE, positively asserted: the fixture is caught at the RESOLVED
    // moment, which is the whole reason it can reach the bug. Without this,
    // a future change that applied the u32 ceiling in the UNRESOLVED
    // preflight too would leave the test green while covering nothing — the
    // absence oracle below would still hold, for the wrong reason.
    assert!(
        logs_contain("replay field registry (resolved)"),
        "premise: the fixture must be refused at the RESOLVED moment — the leg \
         whose identity source this test pins"
    );

    // THE REGRESSION SIGNATURE: recording the identity from the resolved
    // clone makes the scrub match nothing, which the force-scrub fallback
    // reports (verified by reverting the identity source and re-running).
    assert!(
        !logs_contain("force-scrubbing by identity alone"),
        "the scrub must MATCH the removed definition by hash, not fall through \
         to the identity-only force-scrub (that fallback is the backstop, not \
         the mechanism — it can take a healthy sibling with the offender)"
    );
}

/// The removal report must not key
/// suppression on the `(package, name)` key while the winner-sweep
/// keys on the qualified-name STRING: a definition could be the
/// `by_key` winner (a store `.msg` shadowing a BUILT-IN by package + name)
/// while a slash-named YAML entry (`package = None`) is the string winner.
/// The failing store copy then has a same-key SURVIVOR (the shadowed
/// built-in) and a surviving string bearer, the report comes back EMPTY,
/// both consumers read "nothing to scrub", and the hostile definition
/// stays in the working set for `LayoutResolver::new` to panic on. The
/// report names every removed DEFINITION (identity + hash): the hash
/// map serves the valid YAML winner, the built-in keeps its own hash, and
/// the registry materializes the layout without panicking.
#[test]
fn r39_a_store_copy_shadowing_a_builtin_that_composed_overflows_is_scrubbed_not_masked() {
    let tmp = tempfile::tempdir().unwrap();
    // The hostile store definition: `geometry_msgs/Vector3` by KEY (it
    // shadows the built-in's `(Some("geometry_msgs"), "Vector3")`), whose
    // composed size overflows through a nested fixed-array reference.
    write_store_msg(
        tmp.path(),
        "geometry_msgs",
        "Filler",
        "float64[536870907] v\n",
    );
    write_store_msg(
        tmp.path(),
        "geometry_msgs",
        "Vector3",
        "Filler[8589934592] arr\n",
    );
    // The slash-named YAML entry bearing the same STRING — the string winner.
    write_yaml(
        tmp.path(),
        "vec_yaml.yaml",
        "schemas:\n  geometry_msgs/Vector3:\n    fields:\n      uint32 a:\n",
    );

    // Surface 1: the graph hash map — must not panic, must serve the YAML winner.
    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());
    let yaml_expected = {
        let mut s = MessageSchema::new("geometry_msgs/Vector3");
        s.add_field(FieldDef::new("a", FieldType::U32));
        oracle_hash(&mut [s], "geometry_msgs/Vector3")
    };
    assert_eq!(
        map.get("geometry_msgs/Vector3").copied(),
        Some(yaml_expected),
        "the valid YAML string winner serves the name: {map:?}"
    );
    assert!(
        map.contains_key("geometry_msgs/Filler"),
        "Filler alone is sizable"
    );

    // Surface 2: the replay registry — the same scrub, then a real layout.
    let reg = FieldRegistry::from_graph(&one_output_graph("geometry_msgs/Vector3"), tmp.path());
    assert_eq!(reg.expected_schema_hash(TOPIC), Some(yaml_expected));
    let mut decoder = reg
        .decoder_for(TOPIC)
        .expect("a decoder for the YAML winner");
    let layout = decoder
        .root_layout()
        .expect("the root layout materializes — the hostile store copy is gone");
    let names: Vec<&str> = layout
        .fixed_fields
        .iter()
        .map(|f| f.name.as_str())
        .collect();
    assert_eq!(names, ["a"]);
}

/// A store type nesting a BUILT-IN
/// (`builtin_interfaces/Time`) gets a graph-hash entry — the fold resolves
/// against the compiled corpus exactly as a node's build script does — so
/// `graph run`'s divergence check covers it instead of being silently
/// skipped; the built-in itself is resolved against, never served.
#[test]
fn r39_a_store_type_nesting_a_builtin_hashes_on_the_graph_surface() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(
        tmp.path(),
        "nav39",
        "Stamped",
        "builtin_interfaces/Time stamp\nfloat64 v\n",
    );
    let expected = {
        let mut time = MessageSchema::new_in_package("Time", "builtin_interfaces");
        time.add_field(FieldDef::new("sec", FieldType::I32));
        time.add_field(FieldDef::new("nanosec", FieldType::U32));
        let mut stamped = MessageSchema::new_in_package("Stamped", "nav39");
        stamped.add_field(FieldDef::new(
            "stamp",
            FieldType::Nested {
                schema_name: "Time".to_string(),
                package: Some("builtin_interfaces".to_string()),
                fixed: None,
            },
        ));
        stamped.add_field(FieldDef::new("v", FieldType::F64));
        oracle_hash(&mut [time, stamped], "nav39/Stamped")
    };
    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());
    assert_eq!(
        map.get("nav39/Stamped").copied(),
        Some(expected),
        "the built-in nested target inlines into the recorded hash; got {map:?}"
    );
    assert_eq!(
        map.get("Stamped").copied(),
        Some(expected),
        "the unique bare alias too"
    );
    assert!(
        !map.contains_key("builtin_interfaces/Time"),
        "built-ins resolve references only — never served from the map: {map:?}"
    );
}
