// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end integration test for the CLI engine scaffolding workflow.
//!
//! Exercises the full lifecycle without iceoryx2 (no shared memory):
//! workspace → node create → modify → graph create → stage → validate.

use cerulion_cli_engine::{graph_cmd, node_cmd, schema_cmd, workspace};

#[test]
fn test_full_scaffolding_workflow() {
    let tmp = tempfile::tempdir().unwrap();

    // 1. Create workspace
    let ws = workspace::workspace_create(tmp.path(), "my_robot").unwrap();
    assert!(ws.root.join("Cargo.toml").exists());
    assert!(ws.nodes_dir.is_dir());
    assert!(ws.graphs_dir.is_dir());
    assert!(ws.schemas_dir.is_dir());

    let cargo_toml = ws.root.join("Cargo.toml");

    // 2. Create nodes: camera and detector
    node_cmd::node_create(&ws.nodes_dir, &cargo_toml, "camera", None).unwrap();
    node_cmd::node_create(&ws.nodes_dir, &cargo_toml, "detector", None).unwrap();

    assert!(ws.nodes_dir.join("camera/Cargo.toml").exists());
    assert!(ws.nodes_dir.join("camera/src/lib.rs").exists());
    assert!(ws.nodes_dir.join("detector/Cargo.toml").exists());
    assert!(ws.nodes_dir.join("detector/src/lib.rs").exists());

    // 3. Modify nodes: add output on camera, input+output on detector
    node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "camera",
        "image",
        Some("sensor_msgs::Image"),
        true,
        false,
    )
    .unwrap();

    node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "detector",
        "image",
        Some("sensor_msgs::Image"),
        false,
        false,
    )
    .unwrap();
    node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "detector",
        "detections",
        Some("vision_msgs::Detection2DArray"),
        true,
        false,
    )
    .unwrap();

    // 4. Verify node info — port counts
    let camera_info = node_cmd::node_info(&ws.nodes_dir, "camera").unwrap();
    assert_eq!(camera_info.outputs.len(), 1);
    assert_eq!(camera_info.inputs.len(), 0);
    assert_eq!(camera_info.outputs[0].name, "image");

    let detector_info = node_cmd::node_info(&ws.nodes_dir, "detector").unwrap();
    assert_eq!(detector_info.inputs.len(), 1);
    assert_eq!(detector_info.outputs.len(), 1);
    assert_eq!(detector_info.inputs[0].name, "image");
    assert_eq!(detector_info.outputs[0].name, "detections");

    // 5. Create graph
    graph_cmd::graph_create(&ws.graphs_dir, "perception", Some("robot1")).unwrap();

    // 6. Stage camera node into graph
    let camera_outputs: Vec<(String, Option<String>)> = camera_info
        .outputs
        .iter()
        .map(|p| (p.name.clone(), p.schema.clone()))
        .collect();
    let camera_def = graph_cmd::build_node_def("camera", None, &camera_outputs, &[]);
    graph_cmd::node_stage(&ws.graphs_dir, "perception", camera_def).unwrap();

    // Stage detector with input binding [camera,image]
    let detector_outputs: Vec<(String, Option<String>)> = detector_info
        .outputs
        .iter()
        .map(|p| (p.name.clone(), p.schema.clone()))
        .collect();
    let detector_def = graph_cmd::build_node_def(
        "detector",
        None,
        &detector_outputs,
        &[("image".to_string(), "[camera,image]".to_string())],
    );
    let config = graph_cmd::node_stage(&ws.graphs_dir, "perception", detector_def).unwrap();

    // 7. Validate graph: 2 nodes, correct input source resolution
    assert_eq!(config.nodes.len(), 2);
    assert_eq!(config.nodes[0].id, "camera");
    assert_eq!(config.nodes[1].id, "detector");
    assert_eq!(config.nodes[1].inputs[0].source, "camera/image");

    // 8. Graph read roundtrip
    let reloaded = graph_cmd::graph_read(&ws.graphs_dir, "perception").unwrap();
    assert_eq!(reloaded.identity(), "perception");
    assert!(
        reloaded.name.is_none(),
        "project rule: `graph create` writes no `name:` key"
    );
    assert_eq!(reloaded.prefix, "robot1");
    assert_eq!(reloaded.nodes.len(), 2);

    // 9. Node list — verify 2 nodes
    let nodes = node_cmd::node_list(&ws.nodes_dir).unwrap();
    assert_eq!(nodes.len(), 2);
    let types: Vec<&str> = nodes.iter().map(|n| n.node_type.as_str()).collect();
    assert!(types.contains(&"camera"));
    assert!(types.contains(&"detector"));

    // 10. Schema create — verify file exists
    schema_cmd::schema_create(&ws.schemas_dir, "laser_scan").unwrap();
    assert!(ws.schemas_dir.join("laser_scan.yaml").exists());

    let schema_info = schema_cmd::schema_info(&ws.schemas_dir, "laser_scan").unwrap();
    assert_eq!(schema_info.entries.len(), 1);
    assert_eq!(schema_info.entries[0].name, "LaserScan");

    // 11. Graph list — verify 1 graph
    let graphs = graph_cmd::graph_list(&ws.graphs_dir).unwrap();
    assert_eq!(graphs.len(), 1);
    assert_eq!(graphs[0], "perception");
}

/// `cerulion node modify --policy data_trigger=<NAME>` for an
/// EXISTING input must promote that input to `#[input(trigger)]`
/// and clear any prior `period_ms` / `external` macro args.
#[test]
fn test_node_modify_promote_existing_input_to_trigger() {
    use cerulion_cli_engine::node_metadata::parse_node_metadata;
    use cerulion_core::MacroPolicy;
    let tmp = tempfile::tempdir().unwrap();
    let ws = workspace::workspace_create(tmp.path(), "ws").unwrap();
    let cargo_toml = ws.root.join("Cargo.toml");

    // Default-create a node with `period_ms = 100` (no inputs yet).
    node_cmd::node_create(&ws.nodes_dir, &cargo_toml, "consumer", None).unwrap();
    // Add a regular input.
    node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "consumer",
        "data",
        Some("std_msgs::Int32"),
        false,
        false,
    )
    .unwrap();
    let before = parse_node_metadata(&ws.nodes_dir.join("consumer")).unwrap();
    assert_eq!(
        before.policy,
        Some(MacroPolicy::Period { period_ms: 100 }),
        "fresh node has period_ms=100"
    );
    // Promote the existing input to trigger via the engine helper.
    node_cmd::node_modify_promote_input_to_trigger(&ws.nodes_dir, "consumer", "data").unwrap();
    let after = parse_node_metadata(&ws.nodes_dir.join("consumer")).unwrap();
    assert_eq!(
        after.policy,
        Some(MacroPolicy::DataTrigger {
            input_name: "data".to_string(),
        }),
        "policy must flip to DataTrigger after promotion"
    );
    // Source must NOT carry both `period_ms` and `#[input(trigger)]`.
    let src = std::fs::read_to_string(ws.nodes_dir.join("consumer/src/lib.rs")).unwrap();
    assert!(
        !src.contains("period_ms"),
        "promotion must strip the conflicting `period_ms` macro arg"
    );
    assert!(
        src.contains("#[input(trigger)]"),
        "promotion must mark the input field"
    );
}

/// `node_modify_promote_input_to_trigger` errors descriptively when
/// the named input doesn't exist on the node.
#[test]
fn test_node_modify_promote_missing_input_errors() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = workspace::workspace_create(tmp.path(), "ws").unwrap();
    let cargo_toml = ws.root.join("Cargo.toml");
    node_cmd::node_create(&ws.nodes_dir, &cargo_toml, "consumer", None).unwrap();
    let err = node_cmd::node_modify_promote_input_to_trigger(&ws.nodes_dir, "consumer", "ghost")
        .unwrap_err();
    let msg = err.to_string();
    assert!(
        msg.contains("ghost"),
        "error must name the missing input; got: {msg}"
    );
    assert!(
        msg.contains("does not exist"),
        "error must explain why it failed; got: {msg}"
    );
}

/// The generated workspace `Cargo.toml` must NOT carry its own
/// `iceoryx2` pin. The user inherits `cerulion_core`'s EXACT `=0.9.1`
/// transitive pin — Cargo honours an exact (`=`) requirement through the
/// dependency graph, so any consumer of `cerulion_core` already resolves
/// iceoryx2 to exactly the version `cerulion_core` chose, today (path dep)
/// and post-publish (crates.io dep). Adding an independent iceoryx2 line
/// to the scaffold would create a SECOND source of truth that could drift
/// from `cerulion_core`'s choice and would block the planned
/// `cerulion_core` auto-updater (which bumps the pin in one place).
///
/// This is the cheap, always-on regression guard: a future accidental
/// stray pin in `workspace.rs` / `templates.rs` is caught here in CI
/// without needing the toolchain or a full dependency resolution.
#[test]
fn generated_workspace_has_no_iceoryx2_pin_and_depends_on_cerulion_core() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = workspace::workspace_create(tmp.path(), "no_pin_ws").unwrap();
    let content = std::fs::read_to_string(ws.root.join("Cargo.toml")).unwrap();

    // The transitive-pin SOURCE must be present: the scaffold depends on
    // cerulion_core, which is where the `=0.9.1` iceoryx2 pin lives.
    assert!(
        content.contains("cerulion_core"),
        "scaffold must depend on cerulion_core (the transitive iceoryx2-pin source); \
         got:\n{content}"
    );

    // The regression guard: no `iceoryx2` DEPENDENCY entry. We match on a
    // line whose first non-whitespace token is `iceoryx2` followed by `=`
    // (a TOML dependency key like `iceoryx2 = "..."` or `iceoryx2 = {...}`),
    // NOT a bare substring — the word may one day appear legitimately inside
    // a path string or a comment (e.g. `# pinned via iceoryx2`) and that
    // must not trip the guard. There are no such occurrences today; this is
    // robustness for the future.
    for line in content.lines() {
        let trimmed = line.trim_start();
        let is_iceoryx2_dep = trimmed
            .strip_prefix("iceoryx2")
            .map(|rest| {
                // Next non-whitespace char must be `=` (the TOML key/value
                // separator) to count as a dependency entry. This rejects
                // e.g. `iceoryx2-log = ...` (the char after the prefix is
                // `-`, not whitespace-then-`=`).
                let rest = rest.trim_start();
                rest.starts_with('=')
            })
            .unwrap_or(false);
        assert!(
            !is_iceoryx2_dep,
            "generated workspace Cargo.toml must NOT pin iceoryx2 directly — \
             the user inherits cerulion_core's transitive `=0.9.1` pin. \
             A stray pin here would be a second source of truth and would \
             block the cerulion_core auto-updater. Offending line:\n  {line}\n\
             Full Cargo.toml:\n{content}"
        );
    }
}

/// The gated, RESOLUTION-true companion to the cheap guard above.
///
/// Scaffolds a workspace + a node, then runs `cargo metadata` over the
/// generated manifest and asserts that EXACTLY ONE `iceoryx2` package
/// resolves, at the version `cerulion_core` pins. The expected version is
/// parsed out of `cerulion_core/Cargo.toml` (`iceoryx2 = "=X.Y.Z"`) rather
/// than hardcoded, so this test tracks the pin automatically — when the
/// cerulion_core auto-updater bumps the pin, this stays green without edits.
///
/// `#[ignore]` because it needs the full cargo toolchain AND must run from
/// inside the Cerulion source checkout (the scaffold's `cerulion_core` /
/// `native_ros2_messages` path deps must resolve — `cargo metadata` reads
/// the real on-disk crates). It is a manual check, not a
/// CI gate. Run with:
///   `cargo test -p cerulion_cli_engine --test integration_test \
///        generated_workspace_resolves_single_iceoryx2 -- --ignored`
#[test]
#[ignore = "needs cargo toolchain + must run inside the Cerulion source checkout (path deps must resolve)"]
fn generated_workspace_resolves_single_iceoryx2_at_cerulion_core_pin() {
    use std::path::{Path, PathBuf};
    use std::process::Command;

    let tmp = tempfile::tempdir().unwrap();
    let ws = workspace::workspace_create(tmp.path(), "resolve_ws").unwrap();
    let cargo_toml = ws.root.join("Cargo.toml");

    // A workspace with no members resolves nothing useful — scaffold a node
    // so the `nodes/*` glob has a real crate that pulls in cerulion_core.
    node_cmd::node_create(&ws.nodes_dir, &cargo_toml, "probe", None).unwrap();

    // Locate cerulion_core/Cargo.toml to read its iceoryx2 pin. The scaffold
    // embeds the absolute path to cerulion_core when a source checkout was found
    // from the CLI binary (the common case under `cargo test`); fall back to
    // walking up from this test's manifest dir otherwise.
    let core_manifest = cerulion_core_manifest_from_scaffold(&cargo_toml)
        .or_else(cerulion_core_manifest_from_test_dir)
        .expect(
            "could not locate cerulion_core/Cargo.toml — run this test from inside \
             the Cerulion source checkout",
        );
    let expected_version = parse_iceoryx2_exact_pin(&core_manifest).unwrap_or_else(|| {
        panic!(
            "could not parse `iceoryx2 = \"=X.Y.Z\"` from {}",
            core_manifest.display()
        )
    });

    // Resolve the scaffold's dependency graph.
    let output = Command::new(env!("CARGO"))
        .args([
            "metadata",
            "--format-version",
            "1",
            "--manifest-path",
            cargo_toml.to_str().unwrap(),
        ])
        .output()
        .expect("failed to spawn `cargo metadata`");
    assert!(
        output.status.success(),
        "`cargo metadata` failed:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let stdout = String::from_utf8(output.stdout).unwrap();

    // Pull every resolved `iceoryx2` (exact-name) package + its version out
    // of the metadata JSON. We parse the well-known `packages[]` array shape
    // textually (no serde_json dep in this crate's dev-deps) by scanning for
    // the `"name":"iceoryx2"` / `"version":"..."` pairs.
    let versions = iceoryx2_versions_in_metadata(&stdout);
    assert_eq!(
        versions.len(),
        1,
        "expected EXACTLY one iceoryx2 package to resolve (single source of truth \
         via cerulion_core's transitive pin); resolved: {versions:?}"
    );
    assert_eq!(
        versions[0], expected_version,
        "resolved iceoryx2 version must equal cerulion_core's pin ({expected_version}); \
         got {}",
        versions[0]
    );

    // --- gated-test local helpers -------------------------------------

    /// Recover the absolute `cerulion_core/Cargo.toml` path from the
    /// scaffold's generated Cargo.toml (`cerulion_core = { path = "..." }`).
    fn cerulion_core_manifest_from_scaffold(scaffold_cargo_toml: &Path) -> Option<PathBuf> {
        let content = std::fs::read_to_string(scaffold_cargo_toml).ok()?;
        for line in content.lines() {
            let trimmed = line.trim_start();
            // Find the `cerulion_core = { path = "<dir>" }` line; skip others.
            let rest = match trimmed.strip_prefix("cerulion_core") {
                Some(r) => r,
                None => continue,
            };
            let path_marker = "path = \"";
            let idx = match rest.find(path_marker) {
                Some(i) => i,
                None => continue,
            };
            let after = &rest[idx + path_marker.len()..];
            let end = match after.find('"') {
                Some(e) => e,
                None => continue,
            };
            let dir = &after[..end];
            let manifest = Path::new(dir).join("Cargo.toml");
            return manifest.exists().then_some(manifest);
        }
        None
    }

    /// Fall back: walk up from this test crate's manifest dir to find a
    /// sibling `cerulion_core/Cargo.toml`.
    fn cerulion_core_manifest_from_test_dir() -> Option<PathBuf> {
        let mut dir: PathBuf = env!("CARGO_MANIFEST_DIR").into();
        for _ in 0..5 {
            let candidate = dir.join("cerulion_core").join("Cargo.toml");
            if candidate.exists() {
                return Some(candidate);
            }
            if !dir.pop() {
                break;
            }
        }
        None
    }

    /// Parse the exact (`=X.Y.Z`) iceoryx2 pin out of cerulion_core's
    /// Cargo.toml. Matches `iceoryx2 = "=X.Y.Z"` (NOT `iceoryx2-log`), and
    /// returns the bare `X.Y.Z` (stripping the leading `=`) — that's the
    /// resolved version `cargo metadata` reports.
    fn parse_iceoryx2_exact_pin(manifest: &Path) -> Option<String> {
        let content = std::fs::read_to_string(manifest).ok()?;
        for line in content.lines() {
            let trimmed = line.trim_start();
            let rest = match trimmed.strip_prefix("iceoryx2") {
                Some(r) => r,
                None => continue,
            };
            // Reject `iceoryx2-log` etc.: the next char must be whitespace
            // or `=`, not `-`.
            if rest.starts_with('-') {
                continue;
            }
            let rest = rest.trim_start();
            let rest = match rest.strip_prefix('=') {
                Some(r) => r.trim_start(),
                None => continue,
            };
            // `rest` is now the requirement string, e.g. `"=0.9.1"`.
            let req = rest.trim_matches('"');
            return Some(req.trim_start_matches('=').to_string());
        }
        None
    }

    /// Extract the versions of every resolved package named exactly
    /// `iceoryx2` from `cargo metadata --format-version 1` JSON. Scans for
    /// the `"name":"iceoryx2"` marker and reads the `"version"` that
    /// accompanies it in the same package object.
    fn iceoryx2_versions_in_metadata(json: &str) -> Vec<String> {
        let mut out = Vec::new();
        // Package objects appear as `{"name":"iceoryx2","version":"X.Y.Z",...`
        // (field order from cargo is name-before-version). Match the exact
        // name token to avoid `iceoryx2-bb-*` / `iceoryx2-log` false hits.
        let needle = "\"name\":\"iceoryx2\",";
        let mut search = json;
        while let Some(pos) = search.find(needle) {
            let after = &search[pos + needle.len()..];
            if let Some(v) = extract_quoted_field(after, "version") {
                out.push(v);
            }
            search = after;
        }
        out
    }

    /// Read the value of `"<field>":"<value>"` from the start of `s`
    /// (within the next ~120 chars, i.e. the same JSON object).
    fn extract_quoted_field(s: &str, field: &str) -> Option<String> {
        let key = format!("\"{field}\":\"");
        let window = &s[..s.len().min(200)];
        let idx = window.find(&key)?;
        let after = &window[idx + key.len()..];
        let end = after.find('"')?;
        Some(after[..end].to_string())
    }
}

/// The pretty-print pins in `topic_cmd.rs` (and the TUI, which
/// re-uses these consts) must track the generated layout-sensitive
/// SCHEMA_HASH constants. After an intentional hash-recipe change, run
/// this test, copy the actual values from the failure message into the
/// pinned literals, and re-run to green — never hand-compute them.
#[test]
fn pinned_hashes_match_generated_constants() {
    use cerulion_core::message::ShmMessage;

    assert_eq!(
        cerulion_cli_engine::topic_cmd::STD_MSGS_STRING_SCHEMA_HASH,
        <native_ros2_messages::std_msgs::String as ShmMessage>::SCHEMA_HASH,
        "std_msgs/String pin out of sync — update \
         STD_MSGS_STRING_SCHEMA_HASH in topic_cmd.rs to the generated value \
         (right-hand side of this assertion)"
    );
    assert_eq!(
        cerulion_cli_engine::topic_cmd::SENSOR_MSGS_IMAGE_SCHEMA_HASH,
        <native_ros2_messages::sensor_msgs::Image as ShmMessage>::SCHEMA_HASH,
        "sensor_msgs/Image pin out of sync — update \
         SENSOR_MSGS_IMAGE_SCHEMA_HASH in topic_cmd.rs to the generated value \
         (right-hand side of this assertion)"
    );
}

// ============================================================
// `build_workspace_schema_hashes` producer-side coverage.
//
// The runtime's YAML-`schema:`-vs-macro-output divergence warn compares a
// workspace schema's recipe-3 `schema_hash` against the macro's generated
// `ShmMessage::SCHEMA_HASH`. The CLI builds the workspace side via
// `graph_cmd::build_workspace_schema_hashes`. These tests pin that the
// produced hashes MATCH what codegen would compute — crucially for a schema
// carrying a FIXED-NESTED field, which codegen resolves (inlines the nested
// target into the parent's fixed section) BEFORE hashing. A builder that
// skipped resolution would diverge for such schemas and false-positive the
// warn on every build.
// ============================================================

use cerulion_core::codegen::{resolve_fixed_nested, FieldDef, FieldType, MessageSchema};

/// Independent oracle: hand-build the same workspace schema SET as IR,
/// resolve fixed-nested fields over the FULL set (exactly as codegen does
/// before computing `SCHEMA_HASH`), and return the recipe-3 hash of the
/// schema named `target`. Built from the IR directly (not via the
/// YAML-parse path under test), so it is NOT a self-compare.
fn oracle_hash(set: &mut [MessageSchema], target: &str) -> u64 {
    let _ = resolve_fixed_nested(set);
    set.iter()
        .find(|s| s.name == target)
        .unwrap_or_else(|| panic!("oracle: schema '{target}' not in set"))
        .schema_hash()
}

/// A FLAT workspace schema (no nested fields): the produced hash must equal
/// the recipe-3 hash of the same schema. The resolution step is a no-op for
/// a flat schema, so this is the baseline equality.
#[test]
fn build_workspace_schema_hashes_flat_schema_matches_message_schema_hash() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    std::fs::create_dir_all(&schemas_dir).unwrap();
    std::fs::write(
        schemas_dir.join("foo.yaml"),
        "schemas:\n  Foo:\n    fields:\n      uint32 a:\n      float64 b:\n",
    )
    .unwrap();

    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());

    // Oracle: the same flat schema as IR.
    let mut foo = MessageSchema::new("Foo");
    foo.add_field(FieldDef::new("a", FieldType::U32));
    foo.add_field(FieldDef::new("b", FieldType::F64));
    let expected = oracle_hash(&mut [foo], "Foo");

    assert_eq!(
        map.get("Foo").copied(),
        Some(expected),
        "flat workspace schema hash must equal MessageSchema::schema_hash()"
    );
}

/// THE regression guard for fix #1: a workspace schema with a FIXED-NESTED
/// field (`Point3D position`, where `Point3D` is an all-`f64` fixed schema
/// also in the workspace). Codegen RESOLVES the nested field (inlines
/// `Point3DShm` into the parent's fixed section, folds the target hash)
/// before computing `SCHEMA_HASH`. The produced map hash MUST match the
/// RESOLVED oracle — and MUST NOT match the UNRESOLVED hash (proving the
/// builder actually resolved, not just parsed in isolation).
#[test]
fn build_workspace_schema_hashes_fixed_nested_resolves_like_codegen() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    std::fs::create_dir_all(&schemas_dir).unwrap();
    // Point3D: recursively-fixed (3 × f64). Pose embeds it as a fixed
    // nested field plus a primitive.
    std::fs::write(
        schemas_dir.join("point3d.yaml"),
        "schemas:\n  Point3D:\n    fields:\n      float64 x:\n      float64 y:\n      float64 z:\n",
    )
    .unwrap();
    std::fs::write(
        schemas_dir.join("pose.yaml"),
        "schemas:\n  Pose:\n    fields:\n      Point3D position:\n      uint32 frame:\n",
    )
    .unwrap();

    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());

    // RESOLVED oracle: hand-build BOTH schemas, resolve the set, hash Pose.
    let mut point3d = MessageSchema::new("Point3D");
    for f in ["x", "y", "z"] {
        point3d.add_field(FieldDef::new(f, FieldType::F64));
    }
    let mut pose = MessageSchema::new("Pose");
    pose.add_field(FieldDef::new(
        "position",
        FieldType::parse("Point3D").unwrap(),
    ));
    pose.add_field(FieldDef::new("frame", FieldType::U32));
    let resolved_expected = oracle_hash(&mut [point3d.clone(), pose.clone()], "Pose");

    assert_eq!(
        map.get("Pose").copied(),
        Some(resolved_expected),
        "fixed-nested workspace schema hash must match the RESOLVED codegen hash"
    );

    // The UNRESOLVED hash (Pose hashed without resolving the nested field —
    // the buggy behavior) must DIFFER, proving resolution is load-bearing.
    let unresolved = {
        let mut p = MessageSchema::new("Pose");
        p.add_field(FieldDef::new(
            "position",
            FieldType::parse("Point3D").unwrap(),
        ));
        p.add_field(FieldDef::new("frame", FieldType::U32));
        p.schema_hash()
    };
    assert_ne!(
        resolved_expected, unresolved,
        "the fixed-nested resolution must actually change the hash — otherwise this \
         test would not prove the builder resolves (the field is fixed-size, so \
         resolution inlines it and folds the target hash)"
    );

    // Point3D itself (flat, fixed) is present and matches.
    assert_eq!(
        map.get("Point3D").copied(),
        Some(point3d.schema_hash()),
        "the nested target schema must also be in the map"
    );
}

/// The omit-if-unresolvable guard, RE-PINNED for the resolved contract: the
/// fold resolves against the BUILT-IN ROS 2 corpus as well as the workspace
/// so "not in the workspace `schemas/` set" is not the
/// same thing as "codegen cannot see it". A nested `std_msgs/Header` is a
/// type codegen resolves too, so its bearer IS hashed — omitting it leaves
/// `graph run`'s divergence check with no entry for a port codegen hashes
/// perfectly well. What still omits is a ref to a type NOTHING can resolve.
///
/// The oracle is independent of the fold: `std_msgs/Header` carries a `string frame_id`, so it is VARIABLE:
/// `resolve_fixed_nested` cannot inline it and recipe 3 folds only its
/// qualified NAME, so the bearer's hash is its own IR's and the oracle needs
/// no set. That makes the VARIABLE arm a presence pin — the value cannot
/// discriminate, because nothing the fold does can change it — so a FIXED
/// built-in target (`geometry_msgs/Point`) is asserted alongside it, where
/// resolution really does fold the target's hash in. That arm carries the
/// `assert_ne!` differential its sibling above carries: without it, a test
/// asserting a hash the fold could not have altered proves only that the
/// entry exists.
#[test]
fn build_workspace_schema_hashes_hashes_a_builtin_nested_ref_and_omits_an_unresolvable_one() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    std::fs::create_dir_all(&schemas_dir).unwrap();
    // Stamped references std_msgs/Header — a BUILT-IN, resolvable by codegen.
    std::fs::write(
        schemas_dir.join("stamped.yaml"),
        "schemas:\n  Stamped:\n    fields:\n      std_msgs/Header header:\n      float64 value:\n",
    )
    .unwrap();
    // Ghost references a type NO tier defines — neither workspace nor corpus.
    std::fs::write(
        schemas_dir.join("ghost.yaml"),
        "schemas:\n  Ghost:\n    fields:\n      no_such_pkg/NoSuchType t:\n      float64 value:\n",
    )
    .unwrap();
    // A sibling FLAT schema in the same set is unaffected.
    std::fs::write(
        schemas_dir.join("plain.yaml"),
        "schemas:\n  Plain:\n    fields:\n      uint32 a:\n",
    )
    .unwrap();

    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());

    // HAND ORACLE: Header is variable, so the bearer's hash is its own IR's.
    let mut stamped = MessageSchema::new("Stamped");
    stamped.add_field(FieldDef::new(
        "header",
        FieldType::parse("std_msgs/Header").unwrap(),
    ));
    stamped.add_field(FieldDef::new("value", FieldType::F64));
    assert_eq!(
        map.get("Stamped").copied(),
        Some(stamped.schema_hash()),
        "a nested BUILT-IN is resolvable by codegen, so its bearer is hashed \
         (the fold resolves against the built-in corpus)"
    );

    assert!(
        !map.contains_key("Ghost"),
        "a nested ref NO tier can resolve is still OMITTED — emitting a hash \
         codegen never computes is the false positive this guard exists for"
    );
    assert!(
        map.contains_key("Plain"),
        "a sibling flat schema must still be hashed"
    );

    // THE DISCRIMINATING ARM: a FIXED built-in target, where resolution folds
    // the target's own hash into the bearer — so the value, not just the
    // presence, depends on the fold having resolved against the corpus.
    std::fs::write(
        schemas_dir.join("posed.yaml"),
        "schemas:\n  Posed:\n    fields:\n      geometry_msgs/Point at:\n      uint32 n:\n",
    )
    .unwrap();
    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());
    let mut posed = MessageSchema::new("Posed");
    posed.add_field(FieldDef::new(
        "at",
        FieldType::parse("geometry_msgs/Point").unwrap(),
    ));
    posed.add_field(FieldDef::new("n", FieldType::U32));
    // The oracle resolves the SAME pair codegen does: the hand-built bearer
    // beside the built-in corpus's own `geometry_msgs/Point`.
    let mut point = MessageSchema::new("Point");
    point.package = Some("geometry_msgs".to_string());
    for f in ["x", "y", "z"] {
        point.add_field(FieldDef::new(f, FieldType::F64));
    }
    let resolved_expected = oracle_hash(&mut [point, posed.clone()], "Posed");
    assert_eq!(
        map.get("Posed").copied(),
        Some(resolved_expected),
        "a FIXED built-in target is INLINED, so the fold must serve the \
         resolved hash — the value, not merely an entry"
    );
    assert_ne!(
        resolved_expected,
        posed.schema_hash(),
        "the fixed-nested resolution must actually change the hash — otherwise \
         this arm would not prove the fold resolves against the corpus"
    );
}

/// Best-effort path: a missing `schemas/` directory yields an empty map and
/// never panics.
#[test]
fn build_workspace_schema_hashes_missing_dir_is_empty_no_panic() {
    let tmp = tempfile::tempdir().unwrap();
    // No `schemas/` subdir created.
    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());
    assert!(
        map.is_empty(),
        "a missing schemas/ dir must yield an empty map, not panic"
    );
}

/// Best-effort path: a malformed schema YAML is SKIPPED (its file produces
/// no entries) but does NOT abort the whole map — sibling well-formed
/// schemas still get hashed.
#[test]
fn build_workspace_schema_hashes_malformed_yaml_is_skipped_not_fatal() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    std::fs::create_dir_all(&schemas_dir).unwrap();
    // Malformed: not a YAML mapping with a `schemas:` key.
    std::fs::write(
        schemas_dir.join("broken.yaml"),
        "this: is: not valid: schema",
    )
    .unwrap();
    // A well-formed sibling.
    std::fs::write(
        schemas_dir.join("good.yaml"),
        "schemas:\n  Good:\n    fields:\n      uint32 a:\n",
    )
    .unwrap();

    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());

    assert!(
        map.contains_key("Good"),
        "a malformed sibling must not stop the well-formed schema from being hashed"
    );
    assert!(
        !map.contains_key("broken"),
        "the malformed file must contribute no entry"
    );
}

/// Builder `Err`-arm downgrade: a schema file with a NON-STRING schema-name
/// key (a YAML int `123:`) makes `parse_message_schemas` return
/// `Err(Validation)` (the name must be a string — see `schema_cmd`). The
/// builder DOWNGRADES that to `warn` + skip rather than aborting the whole
/// run, so a sibling VALID schema file is still hashed.
/// `parse_message_schemas` is unit-tested for the non-string-key rejection;
/// this pins the builder's warn+skip path end-to-end. Asserts no panic /
/// returned map (the fn is infallible) + the valid sibling survives.
#[test]
fn build_workspace_schema_hashes_non_string_key_is_warned_and_skipped_not_fatal() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    std::fs::create_dir_all(&schemas_dir).unwrap();
    // Non-string schema-name key (`123` parses as a YAML integer key) →
    // `parse_message_schemas` returns `Err(Validation)`. The whole file is
    // skipped (the builder's `Err` arm warns), contributing no entry.
    std::fs::write(
        schemas_dir.join("bad_key.yaml"),
        "schemas:\n  123:\n    fields:\n      uint32 a:\n",
    )
    .unwrap();
    // A sibling VALID schema in the same dir must be unaffected.
    std::fs::write(
        schemas_dir.join("valid.yaml"),
        "schemas:\n  Valid:\n    fields:\n      uint32 a:\n      float64 b:\n",
    )
    .unwrap();

    // Infallible by contract — must not panic.
    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());

    // The valid sibling IS present and matches its recipe-3 hash.
    let mut valid = MessageSchema::new("Valid");
    valid.add_field(FieldDef::new("a", FieldType::U32));
    valid.add_field(FieldDef::new("b", FieldType::F64));
    assert_eq!(
        map.get("Valid").copied(),
        Some(valid.schema_hash()),
        "a sibling valid schema must still be hashed when a non-string-key \
         file is downgraded to warn+skip"
    );

    // The coerced int key (serde renders `123` as the string "123") must NOT
    // appear — the whole bad file is skipped, never partially hashed.
    assert!(
        !map.contains_key("123"),
        "a non-string-key file must contribute no entry (warn + skip, not coerce)"
    );
    // `valid.yaml`'s file STEM is a bare name port resolution
    // accepts (the stem tier) and binds to the file's sole entry, so the map
    // carries `valid` beside `Valid` — the same hash. The bad-key file adds
    // nothing.
    let mut keys: Vec<&str> = map.keys().map(String::as_str).collect();
    keys.sort_unstable();
    assert_eq!(
        keys,
        vec!["Valid", "valid"],
        "the valid sibling and its stem alias only; the bad-key file adds nothing"
    );
    assert_eq!(map.get("valid"), map.get("Valid"));
}

/// The regression guard for transitive membership, pinned for the
/// resolved contract (the
/// fold resolves against the BUILT-IN ROS 2 corpus as well as the
/// workspace).
///
/// Setup — three independent chains:
///
/// - BUILT-IN chain `Outer -> Middle -> std_msgs/Header`. `Header` is a type
///   codegen resolves, so the whole closure resolves and BOTH bearers are
///   hashed. Without transitive membership both are omitted, which leaves `graph run`'s
///   divergence check blind to every port typed after a schema that nests a
///   ROS 2 message — the commonest shape in the corpus.
/// - UNRESOLVABLE chain `Ghost -> Phantom -> no_such_pkg/NoSuchType`. Nothing
///   defines the leaf, so the transitive check must OMIT BOTH — `Ghost` at
///   depth 2 is the regression guard a DIRECT-only check fails.
/// - FULLY-INTERNAL chain `Top -> Mid -> Leaf` (all in-set, `Leaf` all
///   primitives): entire closure in the workspace, all three PRESENT and
///   matching their resolved recipe-3 hashes.
#[test]
fn build_workspace_schema_hashes_transitive_unresolvable_ref_is_omitted() {
    let tmp = tempfile::tempdir().unwrap();
    let schemas_dir = tmp.path().join("schemas");
    std::fs::create_dir_all(&schemas_dir).unwrap();

    // BUILT-IN chain: Outer -> Middle -> std_msgs/Header (resolvable).
    std::fs::write(
        schemas_dir.join("middle.yaml"),
        "schemas:\n  Middle:\n    fields:\n      std_msgs/Header header:\n      float64 m:\n",
    )
    .unwrap();
    std::fs::write(
        schemas_dir.join("outer.yaml"),
        "schemas:\n  Outer:\n    fields:\n      Middle middle:\n      uint32 o:\n",
    )
    .unwrap();

    // UNRESOLVABLE chain: Ghost -> Phantom -> no_such_pkg/NoSuchType.
    std::fs::write(
        schemas_dir.join("phantom.yaml"),
        "schemas:\n  Phantom:\n    fields:\n      no_such_pkg/NoSuchType t:\n      float64 m:\n",
    )
    .unwrap();
    std::fs::write(
        schemas_dir.join("ghost.yaml"),
        "schemas:\n  Ghost:\n    fields:\n      Phantom phantom:\n      uint32 g:\n",
    )
    .unwrap();

    // FULLY-INTERNAL chain: Top -> Mid -> Leaf (all in-set).
    std::fs::write(
        schemas_dir.join("leaf.yaml"),
        "schemas:\n  Leaf:\n    fields:\n      float64 x:\n      float64 y:\n",
    )
    .unwrap();
    std::fs::write(
        schemas_dir.join("mid.yaml"),
        "schemas:\n  Mid:\n    fields:\n      Leaf leaf:\n      uint32 k:\n",
    )
    .unwrap();
    std::fs::write(
        schemas_dir.join("top.yaml"),
        "schemas:\n  Top:\n    fields:\n      Mid mid:\n      uint32 t:\n",
    )
    .unwrap();

    let map = graph_cmd::build_workspace_schema_hashes(tmp.path());

    // BUILT-IN chain: both bearers hashed. HAND ORACLES, no set needed —
    // `std_msgs/Header` is VARIABLE (its `frame_id` is a string), so
    // `resolve_fixed_nested` cannot inline it; a variable nested field
    // contributes only its qualified NAME to recipe 3, and `Middle` is
    // therefore variable too, so `Outer`'s nested `Middle` stays variable.
    let mut middle = MessageSchema::new("Middle");
    middle.add_field(FieldDef::new(
        "header",
        FieldType::parse("std_msgs/Header").unwrap(),
    ));
    middle.add_field(FieldDef::new("m", FieldType::F64));
    assert_eq!(
        map.get("Middle").copied(),
        Some(middle.schema_hash()),
        "Middle nests a BUILT-IN, which codegen resolves — it is hashed"
    );
    let mut outer = MessageSchema::new("Outer");
    outer.add_field(FieldDef::new("middle", FieldType::parse("Middle").unwrap()));
    outer.add_field(FieldDef::new("o", FieldType::U32));
    assert_eq!(
        map.get("Outer").copied(),
        Some(outer.schema_hash()),
        "and so is its depth-2 bearer"
    );

    // UNRESOLVABLE chain: both omitted. The `Ghost` omission is the
    // TRANSITIVE guard — a direct-only check would hash it.
    assert!(
        !map.contains_key("Phantom"),
        "Phantom directly nests a type NO tier defines — must be omitted"
    );
    assert!(
        !map.contains_key("Ghost"),
        "Ghost's nested closure reaches an unresolvable type THROUGH Phantom — \
         the membership check must be TRANSITIVE, not direct-only (this kills \
         the non-transitive regression)"
    );

    // FULLY-INTERNAL chain: all three present and matching the RESOLVED
    // codegen hash (oracle hand-builds + resolves the same IR set).
    let mut leaf = MessageSchema::new("Leaf");
    leaf.add_field(FieldDef::new("x", FieldType::F64));
    leaf.add_field(FieldDef::new("y", FieldType::F64));
    let mut mid = MessageSchema::new("Mid");
    mid.add_field(FieldDef::new("leaf", FieldType::parse("Leaf").unwrap()));
    mid.add_field(FieldDef::new("k", FieldType::U32));
    let mut top = MessageSchema::new("Top");
    top.add_field(FieldDef::new("mid", FieldType::parse("Mid").unwrap()));
    top.add_field(FieldDef::new("t", FieldType::U32));

    let internal_set = [leaf.clone(), mid.clone(), top.clone()];
    for name in ["Leaf", "Mid", "Top"] {
        let expected = oracle_hash(&mut internal_set.clone(), name);
        assert_eq!(
            map.get(name).copied(),
            Some(expected),
            "fully-internal chain member '{name}' must be present and match the \
             RESOLVED codegen hash"
        );
    }
}
