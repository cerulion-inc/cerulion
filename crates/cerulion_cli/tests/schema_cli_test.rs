// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion schema info` / `schema list` subprocess e2e over
//! the REAL binary (`CARGO_BIN_EXE_cerulion`).
//!
//! Closes the original tutorial gap at the binary level: a fresh
//! workspace with no local schemas can introspect built-in ROS 2 types
//! (the embedded-registry path), a workspace schema that shadows a
//! built-in name WINS and warns loudly on stderr, and the not-found
//! remedy is accurate. Hash oracles are hand-derived (the generated
//! `SCHEMA_HASH` constant for the built-in; a hand-built `MessageSchema`
//! IR for the workspace schema) — never a rerun-compare.
//!
//! Each test scaffolds its own tempdir workspace (the minimal
//! `node_run_e2e_test.rs` pattern: a `[workspace]` Cargo.toml + the
//! `graphs/` dir that `CerulionWorkspace::discover` requires) and every
//! subprocess is a plain, fast, self-terminating CLI call driven through
//! `Output` (no graph runtime, no iceoryx2, no shared state) — so no
//! `#[serial]`, matching `replay_cli_test.rs`'s non-transport tests.

use std::path::Path;
use std::process::{Command, Output};

use cerulion_core::codegen::{FieldDef, FieldType, MessageSchema};
use cerulion_core::message::ShmMessage;
use native_ros2_messages::sensor_msgs::Image;

/// Hand-build a minimal workspace in `root`: a `[workspace]` Cargo.toml
/// plus the `graphs/` dir (`CerulionWorkspace::discover` checks
/// `graphs_dir.is_dir()` before matching the Cargo.toml) and an empty
/// `schemas/` dir for the collision arm to seed.
fn build_workspace(root: &Path) {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("schemas")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
}

/// Run `cerulion schema <args..>` from `root` and wait for exit.
///
/// LOCAL-ONLY: every arm in this file asserts a purely LOCAL
/// resolution (workspace YAML / the built-in registry / the not-found remedy), so
/// the child runs with the `CERULION_NETWORK=off` kill switch. Without it, a
/// qualified name that misses locally (`schema info fake_msgs/Nope`) falls through
/// to the REMOTE fetch, which — being on the automagic scouting-ON path —
/// `NetdClient::connect_or_spawn()`s the WELL-KNOWN `cerulion-netd` socket
/// (`$XDG_RUNTIME_DIR/cerulion/netd.sock` / `$HOME/.cerulion/netd.sock`): the test
/// SPAWNS a real machine-wide network daemon on the developer's desk, which then
/// scouts the LAN. It is also why this "plain fast CLI call" took **31.5 s** —
/// the detached daemon INHERITS the child's stderr, so `output()` (which reads to
/// EOF) blocked for the daemon's full 30 s idle self-exit. The kill switch is
/// honored inside `topic_cmd::fetch_remote_schema` BEFORE any session/spawn, and
/// the caller re-raises the same local `SchemaNotFound`, so every assertion here
/// (exit code + stderr text) is byte-unchanged.
fn run_schema(root: &Path, args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .arg("schema")
        .args(args)
        .current_dir(root)
        .env("CERULION_NETWORK", "off")
        .output()
        .expect("failed to spawn cerulion binary")
}

fn stdout_of(out: &Output) -> String {
    String::from_utf8_lossy(&out.stdout).into_owned()
}

fn stderr_of(out: &Output) -> String {
    String::from_utf8_lossy(&out.stderr).into_owned()
}

/// Collision arm: a workspace `Image` schema shadows the built-in
/// `sensor_msgs/Image`. The binary must exit 0, warn LOUDLY on stderr
/// (naming both sides + the precedence), and render the WORKSPACE
/// schema's hash — pinned against a HAND-BUILT `MessageSchema` IR oracle
/// (independent of the CLI's YAML parsing) — and NOT the built-in's
/// generated `SCHEMA_HASH`.
#[test]
fn schema_info_collision_warns_on_stderr_and_workspace_wins() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path());
    let content = "schemas:\n  Image:\n    description: \"local override\"\n    fields:\n      \
                   uint32 height:\n";
    std::fs::write(tmp.path().join("schemas/Image.yaml"), content).unwrap();

    // Hand-derived oracle: the same IR `schema info` hashes for this
    // YAML (package-less `Image` with one `uint32 height` field).
    let mut oracle = MessageSchema::new("Image");
    oracle.add_field(FieldDef::new("height", FieldType::U32));
    let workspace_hash = oracle.schema_hash();

    let out = run_schema(tmp.path(), &["info", "Image"]);
    assert_eq!(out.status.code(), Some(0), "stderr: {}", stderr_of(&out));

    let stderr = stderr_of(&out);
    assert!(
        stderr.contains(
            "WARNING: workspace schema 'Image' shadows built-in 'sensor_msgs/Image' \
             — workspace wins for schema info"
        ),
        "stderr must carry the loud shadow warning: {stderr}"
    );

    let stdout = stdout_of(&out);
    assert!(stdout.contains("source: workspace"), "stdout: {stdout}");
    assert!(
        stdout.contains(&format!("hash: 0x{:016x}", workspace_hash)),
        "stdout must render the WORKSPACE schema's hash: {stdout}"
    );
    assert!(
        !stdout.contains(&format!("{:016x}", <Image as ShmMessage>::SCHEMA_HASH)),
        "stdout must NOT render the shadowed built-in's hash: {stdout}"
    );
}

/// The `::`-SPELLED collision, at the BINARY.
///
/// `cerulion_cli/src/main.rs`'s stderr shadow loop is normalized so an
/// entry declared `pkg::Type` gets the warning its `pkg/Type` twin gets:
/// the loop compares `normalize_schema(&entry.name)` against the built-in
/// the ENGINE flagged, instead of the raw declared string. Compared raw,
/// `"std_msgs::String"` matches neither `"std_msgs/String"` nor the bare
/// `"String"`, so the loud line goes missing on exactly the spelling the
/// engine had just flagged as shadowing — silently, with the workspace copy
/// still winning.
///
/// The engine half is pinned
/// (`schema_builtin_test::a_double_colon_workspace_entry_shadows_the_builtin_on_list_as_well_as_info`),
/// but nothing reached the BINARY: every existing arm of this loop uses a
/// BARE name, and no `cerulion_cli` test wrote a `::`-spelled YAML entry at
/// all. This is user-facing stderr, which this repo treats as HIGH surface.
///
/// The warning must name the entry by its DECLARED spelling (what the user
/// wrote and must go edit) and the built-in by its canonical one.
#[test]
fn schema_info_double_colon_entry_warns_at_the_binary() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path());
    // A `::`-spelled entry, in a file whose STEM is neither spelling — so
    // the entry NAME is the only thing that can bind, and the file stem
    // cannot accidentally supply the match.
    std::fs::write(
        tmp.path().join("schemas/mystring.yaml"),
        "schemas:\n  std_msgs::String:\n    fields:\n      uint32 a:\n",
    )
    .unwrap();

    // Hand-derived oracle: the IR this YAML declares. The entry
    // keeps its DECLARED spelling as the wire-hash identity
    // (`parse_message_schemas` does `MessageSchema::new(name_str)` on the raw
    // key), because canonicalizing it would make the CLI hash `pkg/Type`
    // while the workspace node's build script hashes `pkg::Type`. So the
    // oracle is spelled exactly as the file writes it — NOT normalized.
    // Normalization applies to CLAIMS and lookups, never to the hashed name.
    let mut oracle = MessageSchema::new("std_msgs::String");
    oracle.add_field(FieldDef::new("a", FieldType::U32));
    let workspace_hash = oracle.schema_hash();

    let out = run_schema(tmp.path(), &["info", "std_msgs/String"]);
    assert_eq!(out.status.code(), Some(0), "stderr: {}", stderr_of(&out));

    let stderr = stderr_of(&out);
    assert!(
        stderr.contains(
            "WARNING: workspace schema 'std_msgs::String' shadows built-in \
             'std_msgs/String' — workspace wins for schema info"
        ),
        "the `::` spelling must get the warning its `/` twin gets, naming the \
         entry as DECLARED: {stderr}"
    );

    // ANTI-TAUTOLOGY / premise: the workspace copy really did win, so the
    // warning is describing a shadow that happened — a build where the
    // built-in answered would make the missing warning correct.
    let stdout = stdout_of(&out);
    assert!(stdout.contains("source: workspace"), "stdout: {stdout}");
    assert!(
        stdout.contains(&format!("hash: 0x{:016x}", workspace_hash)),
        "stdout must render the WORKSPACE schema's hash: {stdout}"
    );
}

/// Built-in happy arm — the original tutorial gap closed at the
/// binary level: `schema info sensor_msgs::Image` from a fresh workspace
/// with no local schemas resolves from the embedded registry, rendering
/// the generated `SCHEMA_HASH` and the built-in source line.
#[test]
fn schema_info_builtin_qualified_resolves_from_binary() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path());

    let out = run_schema(tmp.path(), &["info", "sensor_msgs::Image"]);
    assert_eq!(out.status.code(), Some(0), "stderr: {}", stderr_of(&out));
    let stderr = stderr_of(&out);
    assert!(
        !stderr.contains("WARNING"),
        "no shadow → no warning: {stderr}"
    );

    let stdout = stdout_of(&out);
    assert!(
        stdout.contains("schema: sensor_msgs/Image"),
        "stdout: {stdout}"
    );
    assert!(
        stdout.contains("source: built-in (ROS 2)"),
        "stdout: {stdout}"
    );
    assert!(
        stdout.contains(&format!(
            "hash: 0x{:016x}",
            <Image as ShmMessage>::SCHEMA_HASH
        )),
        "stdout must render the generated SCHEMA_HASH: {stdout}"
    );
}

/// `schema list` subprocess: the built-in section header (counts move
/// with the vendored set, in lockstep with the engine-test 254 anchor)
/// and a known package header render from the real binary.
#[test]
fn schema_list_renders_builtin_groups_from_binary() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path());

    let out = run_schema(tmp.path(), &["list"]);
    assert_eq!(out.status.code(), Some(0), "stderr: {}", stderr_of(&out));

    let stdout = stdout_of(&out);
    assert!(
        stdout.contains("Built-in ROS 2 messages: 254 in 22 packages"),
        "stdout: {stdout}"
    );
    assert!(
        stdout.contains("  builtin_interfaces (2):"),
        "stdout: {stdout}"
    );
}

/// Not-found arm: an unknown qualified name exits NONZERO and stderr
/// carries the accurate remedy (naming both lookup sources — it
/// must never claim only workspace schemas are
/// introspectable).
#[test]
fn schema_info_unknown_qualified_exits_nonzero_with_remedy() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path());

    let out = run_schema(tmp.path(), &["info", "fake_msgs/Nope"]);
    assert_ne!(out.status.code(), Some(0), "must exit nonzero");

    let stderr = stderr_of(&out);
    assert!(
        stderr.contains("Schema 'fake_msgs/Nope' not found"),
        "stderr: {stderr}"
    );
    assert!(
        stderr.contains("built-in ROS 2 messages use qualified 'pkg/Type' names"),
        "stderr must carry the accurate two-source remedy: {stderr}"
    );
    assert!(
        stderr.contains("cerulion schema list"),
        "stderr must point at the discovery command: {stderr}"
    );
}

/// A workspace schema whose bare name is defined in TWO
/// built-in packages (`Pose2D` lives in geometry_msgs AND vision_msgs)
/// warns ONCE PER (workspace schema, shadowed built-in) pair —
/// exercising the comma-split warn loop in the Info arm.
#[test]
fn schema_info_multi_package_collision_warns_per_pair() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path());
    let content = "schemas:\n  Pose2D:\n    fields:\n      uint32 x:\n";
    std::fs::write(tmp.path().join("schemas/Pose2D.yaml"), content).unwrap();

    let out = run_schema(tmp.path(), &["info", "Pose2D"]);
    assert_eq!(out.status.code(), Some(0), "stderr: {}", stderr_of(&out));

    let stderr = stderr_of(&out);
    assert!(
        stderr.contains(
            "WARNING: workspace schema 'Pose2D' shadows built-in 'geometry_msgs/Pose2D' \
             — workspace wins for schema info"
        ),
        "stderr must warn for the geometry_msgs pair: {stderr}"
    );
    assert!(
        stderr.contains(
            "WARNING: workspace schema 'Pose2D' shadows built-in 'vision_msgs/Pose2D' \
             — workspace wins for schema info"
        ),
        "stderr must warn for the vision_msgs pair: {stderr}"
    );

    let stdout = stdout_of(&out);
    assert!(stdout.contains("source: workspace"), "stdout: {stdout}");
}
