// SPDX-License-Identifier: AGPL-3.0-only
//! Bare port-schema resolution subprocess e2e over the REAL
//! binary (`CARGO_BIN_EXE_cerulion`).
//!
//! The bug: `node modify <type> -o Vector3 <name>` (a bare built-in
//! short name) silently scaffolded `use Vector3;` → E0432 at the
//! user's next `node build`. The fix resolves bare names against the
//! embedded built-in registry (workspace schemas win; ambiguous and
//! unknown names are loud errors that mutate nothing) and reports the
//! resolution on stderr — a `note:` for a bare→built-in qualification
//! (every invocation; qualifying silences it) and a `schema info`-style
//! `WARNING:` when a workspace schema shadows a built-in.
//!
//! THIS file pins the LITERAL stderr strings: the in-crate unit tests
//! deliberately cover only `resolve_and_report`'s return values (no
//! stderr-capture machinery in-process), so the exact `note:` /
//! `WARNING:` lines are only observable — and only pinned — here, at
//! the binary boundary. All literals are hand-pasted from
//! `cerulion_cli/src/main.rs::resolve_and_report`.
//!
//! Each test scaffolds its own tempdir workspace (the minimal
//! `schema_cli_test.rs` pattern: a `[workspace]` Cargo.toml + the
//! `graphs/` dir that `CerulionWorkspace::discover` requires) and
//! every subprocess is a plain, fast, self-terminating CLI call driven
//! through `Output` (no graph runtime, no iceoryx2, no shared state) —
//! so no `#[serial]`, matching `schema_cli_test.rs`.

use std::path::Path;
use std::process::{Command, Output};

/// Hand-build a minimal workspace in `root`: a `[workspace]` Cargo.toml
/// plus the `graphs/` dir (`CerulionWorkspace::discover` checks
/// `graphs_dir.is_dir()` before matching the Cargo.toml) and an empty
/// `schemas/` dir for the shadow arm to seed. `node create` makes
/// `nodes/<type>/src` itself via `create_dir_all`.
fn build_workspace(root: &Path) {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("schemas")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
}

/// Run `cerulion node <args..>` from `root` and wait for exit (plain
/// fast CLI calls — `output()` blocks until the process terminates).
fn run_node(root: &Path, args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .arg("node")
        .args(args)
        .current_dir(root)
        .output()
        .expect("failed to spawn cerulion binary")
}

fn stderr_of(out: &Output) -> String {
    String::from_utf8_lossy(&out.stderr).into_owned()
}

/// Read `nodes/prod/src/lib.rs` — every arm uses the node type `prod`.
fn read_prod_lib(root: &Path) -> String {
    std::fs::read_to_string(root.join("nodes/prod/src/lib.rs")).unwrap()
}

/// Create the `prod` node every arm modifies (a plain 10ms-period
/// source — the exact repro's first command).
fn create_prod(root: &Path) {
    let out = run_node(root, &["create", "prod", "--policy", "period_ms=10"]);
    assert_eq!(
        out.status.code(),
        Some(0),
        "node create must succeed; stderr: {}",
        stderr_of(&out)
    );
}

/// The sequence end-to-end: `node create prod
/// --policy period_ms=10` then `node modify prod -o Vector3 out`. The
/// bare name resolves to the unique built-in (qualified import spliced,
/// never a bare `use Vector3;` that fails E0432) and the EXACT stderr note
/// is printed.
#[test]
fn bare_builtin_modify_resolves_end_to_end_with_note() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path());
    create_prod(tmp.path());

    let out = run_node(tmp.path(), &["modify", "prod", "-o", "Vector3", "out"]);
    assert_eq!(out.status.code(), Some(0), "stderr: {}", stderr_of(&out));

    let stderr = stderr_of(&out);
    assert!(
        stderr.contains(
            "note: resolved bare schema 'Vector3' to built-in 'geometry_msgs/Vector3' \
             (qualify it to silence this note)"
        ),
        "stderr must carry the exact resolution note: {stderr}"
    );

    let src = read_prod_lib(tmp.path());
    assert!(
        src.contains("use native_ros2_messages::geometry_msgs::Vector3;"),
        "lib.rs must carry the qualified import: {src}"
    );
    assert!(
        !src.contains("use Vector3;"),
        "the E0432 bare import must NOT be emitted: {src}"
    );
}

/// Create-path arm: the bare name supplied at `node create` time (via
/// `-o`) resolves the same way — qualified import + the exact note.
#[test]
fn bare_builtin_create_resolves_end_to_end_with_note() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path());

    let out = run_node(
        tmp.path(),
        &[
            "create",
            "prod",
            "-o",
            "Vector3",
            "out",
            "--policy",
            "period_ms=10",
        ],
    );
    assert_eq!(out.status.code(), Some(0), "stderr: {}", stderr_of(&out));

    let stderr = stderr_of(&out);
    assert!(
        stderr.contains(
            "note: resolved bare schema 'Vector3' to built-in 'geometry_msgs/Vector3' \
             (qualify it to silence this note)"
        ),
        "stderr must carry the exact resolution note: {stderr}"
    );

    let src = read_prod_lib(tmp.path());
    assert!(
        src.contains("use native_ros2_messages::geometry_msgs::Vector3;"),
        "lib.rs must carry the qualified import: {src}"
    );
    assert!(
        !src.contains("use Vector3;"),
        "the E0432 bare import must NOT be emitted: {src}"
    );
}

/// `-i` arm: the bare name through `node modify`'s INPUT flag resolves
/// the same way — the CLI wiring (not just the engine backstop) must
/// route `-i` through `resolve_and_report`, or its stderr note would
/// silently go missing. Qualified import + a regular `#[input]` field
/// (`#[input]` is NOT a substring of `#[input(trigger)]`, so the two
/// checks are mutually distinguishing) + the exact note.
#[test]
fn bare_builtin_modify_input_resolves_with_note() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path());
    create_prod(tmp.path());

    let out = run_node(tmp.path(), &["modify", "prod", "-i", "Vector3", "vin"]);
    assert_eq!(out.status.code(), Some(0), "stderr: {}", stderr_of(&out));

    let stderr = stderr_of(&out);
    assert!(
        stderr.contains(
            "note: resolved bare schema 'Vector3' to built-in 'geometry_msgs/Vector3' \
             (qualify it to silence this note)"
        ),
        "stderr must carry the exact resolution note: {stderr}"
    );

    let src = read_prod_lib(tmp.path());
    assert!(
        src.contains("use native_ros2_messages::geometry_msgs::Vector3;"),
        "lib.rs must carry the qualified import: {src}"
    );
    assert!(
        src.contains("#[input]"),
        "lib.rs must carry a regular #[input] field: {src}"
    );
    assert!(
        src.contains("vin: Vector3,"),
        "lib.rs must carry the typed input field: {src}"
    );
    assert!(
        !src.contains("use Vector3;"),
        "the E0432 bare import must NOT be emitted: {src}"
    );
}

/// `-T` arm: the bare name through `node modify`'s TRIGGER-input flag
/// resolves the same way. `-T` also rewrites the node's trigger policy
/// (clearing `period_ms` and marking the field `#[input(trigger)]`) —
/// the pin here is the note + the qualified import + the trigger attr,
/// nothing about the resulting policy beyond success.
#[test]
fn bare_builtin_trigger_input_resolves_with_note() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path());
    create_prod(tmp.path());

    let out = run_node(tmp.path(), &["modify", "prod", "-T", "Vector3", "trig"]);
    assert_eq!(out.status.code(), Some(0), "stderr: {}", stderr_of(&out));

    let stderr = stderr_of(&out);
    assert!(
        stderr.contains(
            "note: resolved bare schema 'Vector3' to built-in 'geometry_msgs/Vector3' \
             (qualify it to silence this note)"
        ),
        "stderr must carry the exact resolution note: {stderr}"
    );

    let src = read_prod_lib(tmp.path());
    assert!(
        src.contains("use native_ros2_messages::geometry_msgs::Vector3;"),
        "lib.rs must carry the qualified import: {src}"
    );
    assert!(
        src.contains("#[input(trigger)]"),
        "lib.rs must mark the new field as the trigger: {src}"
    );
    assert!(
        src.contains("trig: Vector3,"),
        "lib.rs must carry the typed trigger field: {src}"
    );
    assert!(
        !src.contains("use Vector3;"),
        "the E0432 bare import must NOT be emitted: {src}"
    );
}

/// Ambiguous arm: `Pose2D` lives in geometry_msgs AND vision_msgs —
/// the modify exits NONZERO, stderr names BOTH candidates, and lib.rs
/// is byte-unchanged (fail-before-mutation).
#[test]
fn ambiguous_bare_name_exits_nonzero_and_leaves_source_untouched() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path());
    create_prod(tmp.path());
    let before = read_prod_lib(tmp.path());

    let out = run_node(tmp.path(), &["modify", "prod", "-o", "Pose2D", "p"]);
    assert_ne!(out.status.code(), Some(0), "must exit nonzero");

    let stderr = stderr_of(&out);
    assert!(
        stderr.contains("ambiguous"),
        "stderr must state the ambiguity: {stderr}"
    );
    assert!(
        stderr.contains("geometry_msgs/Pose2D") && stderr.contains("vision_msgs/Pose2D"),
        "stderr must list BOTH candidate packages: {stderr}"
    );

    assert_eq!(
        before,
        read_prod_lib(tmp.path()),
        "lib.rs must be byte-unchanged after the failed modify"
    );
}

/// Unknown arm: a typo'd bare name exits nonzero and stderr carries
/// the schema-list remedy (hand-pasted from `PORT_SCHEMA_REMEDY` in
/// `cerulion_cli_engine/src/schema_cmd.rs`).
#[test]
fn unknown_bare_name_exits_nonzero_with_schema_list_remedy() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path());
    create_prod(tmp.path());

    let out = run_node(tmp.path(), &["modify", "prod", "-o", "Vectorr3", "v"]);
    assert_ne!(out.status.code(), Some(0), "must exit nonzero");

    let stderr = stderr_of(&out);
    assert!(
        stderr.contains("Schema 'Vectorr3' not found"),
        "stderr must name the missing schema: {stderr}"
    );
    assert!(
        stderr.contains(
            "Run 'cerulion schema list' to see every available schema, or qualify as 'pkg/Type'"
        ),
        "stderr must carry the schema-list remedy: {stderr}"
    );
}

/// Workspace-shadow arm: a workspace `schemas/Vector3.yaml` WINS over
/// the built-in — the bare name is preserved (`use Vector3;`, never
/// hijacked to the built-in path) and the EXACT shadow WARNING prints.
#[test]
fn workspace_schema_wins_bare_preserved_with_shadow_warning() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path());
    create_prod(tmp.path());
    std::fs::write(
        tmp.path().join("schemas/Vector3.yaml"),
        "schemas:\n  Vector3:\n    fields:\n      float64 x:\n",
    )
    .unwrap();

    let out = run_node(tmp.path(), &["modify", "prod", "-o", "Vector3", "w"]);
    assert_eq!(out.status.code(), Some(0), "stderr: {}", stderr_of(&out));

    let stderr = stderr_of(&out);
    assert!(
        stderr.contains(
            "WARNING: workspace schema 'Vector3' shadows built-in 'geometry_msgs/Vector3' \
             — workspace wins for node ports"
        ),
        "stderr must carry the exact shadow warning: {stderr}"
    );

    let src = read_prod_lib(tmp.path());
    assert!(
        src.contains("use Vector3;"),
        "workspace bare name must pass through unchanged: {src}"
    );
    assert!(
        !src.contains("use native_ros2_messages::geometry_msgs::Vector3;"),
        "workspace name must NOT be hijacked to the built-in path: {src}"
    );
}

/// Silence pin: an already-qualified name resolves silently — stderr
/// carries NEITHER the note prefix nor the shadow-WARNING prefix, so
/// the breadcrumb can never regress into spam on qualified names.
#[test]
fn qualified_name_is_silent_on_stderr() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path());
    create_prod(tmp.path());

    let out = run_node(
        tmp.path(),
        &["modify", "prod", "-o", "geometry_msgs::Vector3", "q"],
    );
    assert_eq!(out.status.code(), Some(0), "stderr: {}", stderr_of(&out));

    let stderr = stderr_of(&out);
    assert!(
        !stderr.contains("note: resolved bare schema"),
        "a qualified name must not print the resolution note: {stderr}"
    );
    assert!(
        !stderr.contains("WARNING: workspace schema"),
        "a qualified name must not print the shadow warning: {stderr}"
    );

    let src = read_prod_lib(tmp.path());
    assert!(
        src.contains("use native_ros2_messages::geometry_msgs::Vector3;"),
        "the qualified import must be spliced as before: {src}"
    );
}
