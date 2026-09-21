// SPDX-License-Identifier: AGPL-3.0-only
//! Structural pin (by design, bags embed + skip): the replay/resim path
//! must never SPAWN a `ros2:` entry — a resim re-executes native nodes
//! against recorded frames, and a recorded graph's ROS 2 entries are
//! classified-and-skipped (their topics are recorded inputs).
//!
//! A behavioral test can only show one bag did not respawn; this walk makes
//! the property structural — no replay-path module references the spawner
//! (`Ros2Children` / `spawn_argv` / `stage_ros2_child_env`) or builds a
//! `ros2` command at all — over a COMMENT-STRIPPED view, so prose naming the
//! forbidden tokens (this file's own reason-comments included) never
//! false-positives, and a call commented out never satisfies anything.
//! Paired with the behavioral half in
//! `cerulion_cli/tests/ros2_graph_e2e_test.rs` (a recorded mixed graph
//! resims to exit 0 with the fake `ros2`'s spawn report NOT reappearing).

use std::path::Path;

/// Strip `//` line comments and (nesting) `/* … */` block comments. String
/// literals are deliberately not modelled — none of the walked files carries
/// the forbidden tokens in a literal (asserted below by the anti-tautology
/// arm finding the ALLOWED tokens). An unterminated block fails CLOSED
/// (everything after the opener is stripped, so a forbidden call inside is
/// still not matchable — and the allowed-token arm would then fail loudly).
fn code_only(src: &str) -> String {
    let mut out = String::with_capacity(src.len());
    let bytes = src.as_bytes();
    let mut i = 0;
    let mut block_depth = 0usize;
    while i < bytes.len() {
        if block_depth > 0 {
            if bytes[i..].starts_with(b"/*") {
                block_depth += 1;
                i += 2;
            } else if bytes[i..].starts_with(b"*/") {
                block_depth -= 1;
                i += 2;
            } else {
                i += 1;
            }
            continue;
        }
        if bytes[i..].starts_with(b"//") {
            while i < bytes.len() && bytes[i] != b'\n' {
                i += 1;
            }
        } else if bytes[i..].starts_with(b"/*") {
            block_depth += 1;
            i += 2;
        } else {
            out.push(bytes[i] as char);
            i += 1;
        }
    }
    out
}

/// The replay path's modules (the preflight, the engine, the resim planner
/// and its helpers). A new replay module belongs in this list.
const REPLAY_PATH_FILES: &[&str] = &[
    "src/replay_cmd.rs",
    "src/replay_engine.rs",
    "src/resim_cmd.rs",
    "src/replay_inject.rs",
    "src/replay_rederive.rs",
    "src/replay_state.rs",
    "src/replay_rank.rs",
];

#[test]
fn the_replay_path_never_references_the_ros2_spawner() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    // Token spellings assembled at runtime so THIS file's own code section
    // cannot satisfy or trip the walk it performs.
    let spawner = format!("Ros2{}", "Children");
    let spawn_argv = format!("spawn{}", "_argv");
    let stage_env = format!("stage_ros2{}", "_child_env");
    let ros2_command = format!("Command::new({:?})", "ros2");
    let mut walked = 0usize;
    for file in REPLAY_PATH_FILES {
        let path = root.join(file);
        let src = std::fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("walk must read {}: {e}", path.display()));
        let code = code_only(&src);
        for (token, why) in [
            (spawner.as_str(), "the supervised-children spawner"),
            (spawn_argv.as_str(), "the entry argv builder"),
            (stage_env.as_str(), "the ros2 child env staging"),
            (ros2_command.as_str(), "a literal ros2 process launch"),
        ] {
            assert!(
                !code.contains(token),
                "{file}: the replay path references {why} (`{token}`) — a resim must \
                 classify-and-skip `ros2:` entries, never respawn them"
            );
        }
        walked += 1;
    }
    assert_eq!(walked, REPLAY_PATH_FILES.len());

    // Anti-tautology: the stripper really serves CODE — the preflight's
    // skip arm (the take + the warn) must be visible in the stripped view,
    // or every absence assertion above is vacuous.
    let preflight = code_only(
        &std::fs::read_to_string(root.join("src/replay_cmd.rs")).expect("read replay_cmd"),
    );
    assert!(
        preflight.contains("take_ros2_nodes"),
        "the skip arm itself must be visible to the walk"
    );
    assert!(
        preflight.contains("ros2_entries_skipped"),
        "the skip list threading must be visible to the walk"
    );
}
