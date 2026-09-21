// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion node stage` argv contracts at the REAL binary boundary.
//!
//! The flag this file exists for: `node stage -p/--prefix` was DECLARED,
//! help-texted, accepted by clap, and then destructured into `_` in `main`'s
//! arm — no warn, no error, no effect — while the sibling `node run -p` honours
//! its own. A user who typed it got a silent success on a verb whose whole job
//! is to write a graph file, and the file came out exactly as if they had not.
//!
//! It could not be honoured instead of removed. A staged instance is a
//! `NodeDef`, which carries no prefix field at all, and `prefix:` is a
//! GRAPH-level key (`graph create -n PREFIX`) shared by every instance in the
//! file — so the only two behaviours available were the no-op it already was, or
//! a silent rewrite of every other instance's topic resolution.
//!
//! The parse-level half of this contract is a unit test
//! (`cli.rs::removed_verb_tests::removed_node_stage_prefix_flag_is_an_unexpected_argument`,
//! which also pins that `node run -p` and `graph create -n` still parse). THIS
//! file adds the half a parser test cannot see: the shipped process exits
//! NONZERO and the graph file on disk is BYTE-IDENTICAL, so the refusal lands
//! before `stage_declared_node` touches the artifact.
//!
//! Harness shape cribbed from `node_schema_resolve_cli_test.rs`: a hand-built
//! minimal workspace in a tempdir and plain, fast, self-terminating CLI calls
//! through `Output`. No graph runtime, no iceoryx2, no shared state — so no
//! `#[serial]`.

use std::path::Path;
use std::process::{Command, Output};

/// Hand-build a minimal workspace in `root`: a `[workspace]` Cargo.toml plus
/// the `graphs/` dir (`CerulionWorkspace::discover` checks `graphs_dir.is_dir()`
/// before matching the Cargo.toml). `node create` makes `nodes/<type>/src`
/// itself.
fn build_workspace(root: &Path) {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
}

/// Run `cerulion <args..>` from `root` and wait for exit.
fn run_cli(root: &Path, args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(args)
        .current_dir(root)
        // The login gate is on by default. This file is not about the gate, so
        // it pins the value this repository's own runs carry: a device-code
        // prompt would otherwise replace every exit code below.
        .env("CERULION_LOGIN_GATE", "off")
        .output()
        .expect("failed to spawn cerulion binary")
}

fn stderr_of(out: &Output) -> String {
    String::from_utf8_lossy(&out.stderr).into_owned()
}

/// Run `args` and require exit 0 — used for the scaffolding steps, so a broken
/// fixture fails as a fixture rather than as the contract under test.
fn run_ok(root: &Path, args: &[&str]) {
    let out = run_cli(root, args);
    assert_eq!(
        out.status.code(),
        Some(0),
        "scaffolding step {args:?} must succeed; stderr: {}",
        stderr_of(&out)
    );
}

/// **The dead `node stage -p/--prefix` is refused, and the graph file is
/// untouched.**
///
/// Three assertions per spelling, each catching a different half-fix:
///
/// * NONZERO exit — a script still passing the flag must fail, not appear to
///   work. (Clap's usage-error code is 2; asserted exactly, since that is the
///   code every other refused argv in this CLI returns.)
/// * `unexpected argument` on stderr — the operator is told WHICH token is
///   wrong. A bare nonzero exit would also be satisfied by an unrelated
///   failure, e.g. a workspace that stopped resolving.
/// * BYTE-IDENTICAL graph file — the refusal happens before the engine writes.
///   This is the assertion that catches the inverse defect: a binary that
///   exits 0 here AND mutates the file, so that neither the exit code nor the
///   bytes match what the flag claimed to do.
///
/// And the ANTI-TAUTOLOGY half, in the same body and against the same
/// workspace: the identical invocation WITHOUT the flag exits 0 and DOES change
/// the graph file. Without it, a binary that had stopped staging anything at all
/// would satisfy every byte-identity assertion above.
#[test]
fn node_stage_rejects_the_dead_prefix_flag_and_leaves_the_graph_byte_identical() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    build_workspace(root);
    // A 10 ms-period source with one output — the ordinary shape a user stages
    // (`stage_declared_node` reads the node's DECLARED ports, so a portless node
    // would make the staged entry degenerate).
    run_ok(
        root,
        &[
            "node",
            "create",
            "prod",
            "--policy",
            "period_ms=10",
            "-o",
            "geometry_msgs/Vector3",
            "out",
        ],
    );
    run_ok(root, &["graph", "create", "g", "-n", "pfx"]);

    let graph_path = root.join("graphs/g.yaml");
    let before = std::fs::read(&graph_path).expect("graph create must write graphs/g.yaml");

    for flag in [vec!["-p", "other"], vec!["--prefix", "other"]] {
        let mut argv = vec!["node", "stage", "prod", "-g", "g"];
        argv.extend_from_slice(&flag);
        let out = run_cli(root, &argv);
        let stderr = stderr_of(&out);
        assert_eq!(
            out.status.code(),
            Some(2),
            "{argv:?} must be refused with clap's usage-error code, not silently accepted; \
             stderr: {stderr}"
        );
        assert!(
            stderr.contains("unexpected argument"),
            "{argv:?} must name the offending token so a stale script fails loudly; \
             stderr: {stderr}"
        );
        let after = std::fs::read(&graph_path).expect("read graph");
        assert_eq!(
            after, before,
            "a refused `node stage` must not touch the graph file ({argv:?})"
        );
    }

    // ANTI-TAUTOLOGY: the same staging call WITHOUT the dead flag works and
    // really does write. (`node stage` succeeding here is also the proof that
    // the byte-identity assertions above are about the refusal, not about a
    // verb that writes nothing.)
    run_ok(root, &["node", "stage", "prod", "-g", "g"]);
    let staged = std::fs::read(&graph_path).expect("read graph");
    assert_ne!(
        staged, before,
        "staging must change the graph file — otherwise the byte-identity assertions above \
         prove nothing about the refusal"
    );
    let staged_text = String::from_utf8_lossy(&staged);
    assert!(
        staged_text.contains("prod"),
        "the staged instance must appear in the graph file; file was:\n{staged_text}"
    );
    // The graph-level prefix is the ONLY prefix in play, and it is the one
    // `graph create -n` wrote — the substitute the removed flag never was.
    assert!(
        staged_text.contains("prefix: pfx"),
        "topic resolution comes from the GRAPH-level `prefix:`, which must still be the one \
         `graph create -n pfx` wrote; file was:\n{staged_text}"
    );
}
