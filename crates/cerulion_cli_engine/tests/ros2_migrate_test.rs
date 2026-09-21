// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion ros2 migrate`: engine-orchestration integration tests.
//!
//! The clang prover itself is container-gated (tools/ros2_migrate/
//! run_matrix.sh); THESE tests inject a fixture [`MigrateEngine`] returning
//! hand-written analysis JSON over a synthetic colcon workspace inside a
//! REAL temp git repository, and pin everything the Rust half owns:
//!
//! * dry-run: report shape, byte-determinism across two runs, manifest
//!   refresh (schema, counts, candidate rows, decision-slot preservation),
//!   and that NOTHING else is written;
//! * `--write`: consent ladder (non-TTY refusal naming `--yes`, interactive
//!   decline, `--yes` applies), the dirty-tree refusal, the ONE commit, the
//!   patch file being byte-identical to the dry-run diff (same code path),
//!   `git apply -R` really restoring the original bytes (REAL git as the
//!   oracle), the automatic build-runner invocation with the derived
//!   affected packages, and the loud build-failure text naming the revert
//!   path;
//! * safety: a file changed since analysis refuses before anything is
//!   written; a missing compile database names the exact colcon command.
//!
//! Parallel-safe: every test builds its own tempdir workspace and spawns
//! `git` only inside it.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::mpsc;
use std::time::Duration;

use cerulion_cli_engine::error::CliResult;
use cerulion_cli_engine::ros2_migrate::{
    run_migrate, MigrateDeps, MigrateEngine, MigrateOptions, MigrateOutcome, MANIFEST_REL_PATH,
    PATCH_FILENAME, PRE_WRITE_INTERRUPT_REFUSAL,
};
#[cfg(unix)]
use cerulion_cli_engine::workspace_lock::CONTENTION_POLL;
use cerulion_cli_engine::workspace_lock::{WorkspaceLock, LOCK_DIR, LOCK_FILE};
#[cfg(unix)]
use tracing_test::traced_test;

// ---------------------------------------------------------------------------
// Harness.
// ---------------------------------------------------------------------------

const TALKER_ORIGINAL: &str = "#include \"demo.hpp\"\n\
\n\
void Talker::tick() {\n\
\x20 auto msg = std::make_unique<std_msgs::msg::String>();\n\
\x20 msg->data = \"hello\";\n\
\x20 pub_->publish(std::move(msg));\n\
}\n";

/// The hand-written oracle for the migrated file — the decided 3-line shape,
/// every fill line untouched.
const TALKER_MIGRATED: &str = "#include \"demo.hpp\"\n\
\n\
void Talker::tick() {\n\
\x20 auto loaned = pub_->borrow_loaned_message();\n\
\x20 auto msg = &loaned.get();\n\
\x20 msg->data = \"hello\";\n\
\x20 pub_->publish(std::move(loaned));\n\
}\n";

const DECL_OLD: &str = "auto msg = std::make_unique<std_msgs::msg::String>();";
const DECL_NEW: &str = "auto loaned = pub_->borrow_loaned_message();\n  auto msg = &loaned.get();";
const CALL_OLD: &str = "pub_->publish(std::move(msg))";
const CALL_NEW: &str = "pub_->publish(std::move(loaned))";

struct Fixture {
    _tmp: tempfile::TempDir,
    ws: PathBuf,
    talker: PathBuf,
}

fn git(ws: &Path, args: &[&str]) -> String {
    let out = Command::new("git")
        .arg("-C")
        .arg(ws)
        .args(args)
        .output()
        .expect("git runs");
    assert!(
        out.status.success(),
        "git {:?} failed: {}",
        args,
        String::from_utf8_lossy(&out.stderr)
    );
    String::from_utf8_lossy(&out.stdout).trim_end().to_string()
}

fn git_commit_all(ws: &Path, msg: &str) {
    git(ws, &["add", "-A"]);
    git(
        ws,
        &[
            "-c",
            "user.email=test@example.invalid",
            "-c",
            "user.name=test",
            "commit",
            "-qm",
            msg,
        ],
    );
}

/// Build a synthetic colcon workspace (one package, one TU, a compile
/// database) inside a fresh git repo, committed clean.
fn make_workspace() -> Fixture {
    let tmp = tempfile::tempdir().expect("tempdir");
    // Canonicalize up front: on macOS the tempdir sits behind /var -> /private/var,
    // and run_migrate canonicalizes the workspace before comparing paths.
    let ws = tmp.path().canonicalize().expect("canonical tempdir");
    let pkg = ws.join("src/demo_pkg");
    std::fs::create_dir_all(pkg.join("src")).unwrap();
    std::fs::write(
        pkg.join("package.xml"),
        "<?xml version=\"1.0\"?>\n<package format=\"3\">\n  \
         <name>demo_pkg</name>\n</package>\n",
    )
    .unwrap();
    let talker = pkg.join("src/talker.cpp");
    std::fs::write(&talker, TALKER_ORIGINAL).unwrap();
    let db_dir = ws.join("build/demo_pkg");
    std::fs::create_dir_all(&db_dir).unwrap();
    std::fs::write(
        db_dir.join("compile_commands.json"),
        format!(
            "[{{\"directory\": \"{}\", \"command\": \"clang++ -c {}\", \
             \"file\": \"{}\"}}]",
            db_dir.display(),
            talker.display(),
            talker.display()
        ),
    )
    .unwrap();
    git(&ws, &["init", "-q"]);
    // The build tree stays untracked, like a real colcon workspace.
    std::fs::write(ws.join(".gitignore"), "/build/\n").unwrap();
    git_commit_all(&ws, "baseline");
    Fixture {
        _tmp: tmp,
        ws,
        talker,
    }
}

/// The injected engine: per-TU canned JSON.
struct FixtureEngine {
    by_tu: HashMap<PathBuf, String>,
}

impl MigrateEngine for FixtureEngine {
    fn describe(&self) -> String {
        "fixture-engine".to_string()
    }
    fn analyze_tu(
        &mut self,
        _compile_db_dir: &Path,
        _src_root: &Path,
        tu: &Path,
    ) -> CliResult<String> {
        Ok(self
            .by_tu
            .get(tu)
            .unwrap_or_else(|| panic!("no fixture analysis for {}", tu.display()))
            .clone())
    }
}

/// The canned analysis for the talker fixture: the two edits of the decided
/// rewrite shape, offsets computed from the file text (the REPLACEMENTS and
/// the resulting bytes are the hand oracles).
fn talker_analysis(fx: &Fixture) -> String {
    talker_analysis_for(&fx.talker, &fx.talker)
}

/// The canned analysis with the TU and the REWRITTEN file parameterized —
/// paths go through `serde_json` (JSON-safe on every platform; a raw
/// string-replace over serialized JSON would corrupt Windows backslash
/// paths). The containment test points `rewrite_file` OUTSIDE the workspace.
fn talker_analysis_for(tu: &Path, rewrite_file: &Path) -> String {
    let decl_off = TALKER_ORIGINAL.find(DECL_OLD).expect("decl in fixture");
    let call_off = TALKER_ORIGINAL.find(CALL_OLD).expect("call in fixture");
    serde_json::json!({
        "format": 1,
        "tool_version": "0.1.0",
        "file": tu.display().to_string(),
        "rewrites": [{
            "file": rewrite_file.display().to_string(),
            "function": "Talker::tick",
            "kind": "unique_ptr",
            "message_type": "std_msgs::msg::String_<std::allocator<void>>",
            "publisher": "pub_",
            "line": 6,
            "edits": [
                {"offset": decl_off, "length": DECL_OLD.len(),
                 "original": DECL_OLD, "replacement": DECL_NEW},
                {"offset": call_off, "length": CALL_OLD.len(),
                 "original": CALL_OLD, "replacement": CALL_NEW}
            ]
        }],
        "candidates": [{
            "file": tu.display().to_string(),
            "function": "Talker::helper",
            "line": 42,
            "reason": "pointer-escapes",
            "detail": "the message variable is used other than as a field fill or the publish"
        }]
    })
    .to_string()
}

fn engine_for(fx: &Fixture) -> FixtureEngine {
    let mut by_tu = HashMap::new();
    by_tu.insert(fx.talker.clone(), talker_analysis(fx));
    FixtureEngine { by_tu }
}

fn empty_engine_for(fx: &Fixture) -> FixtureEngine {
    let json = serde_json::json!({
        "format": 1,
        "tool_version": "0.1.0",
        "file": fx.talker.display().to_string(),
        "rewrites": [],
        "candidates": []
    })
    .to_string();
    let mut by_tu = HashMap::new();
    by_tu.insert(fx.talker.clone(), json);
    FixtureEngine { by_tu }
}

#[derive(Debug)]
struct RunResult {
    outcome: MigrateOutcome,
    stdout: Vec<u8>,
    build_calls: Vec<(PathBuf, Vec<String>)>,
}

#[allow(clippy::too_many_arguments)]
fn drive(
    fx: &Fixture,
    engine: &mut FixtureEngine,
    write: bool,
    assume_yes: bool,
    is_tty: bool,
    confirm_answer: bool,
    build_ok: bool,
) -> cerulion_cli_engine::error::CliResult<RunResult> {
    let mut stdout: Vec<u8> = Vec::new();
    let mut build_calls: Vec<(PathBuf, Vec<String>)> = Vec::new();
    let mut confirm = |_prompt: &str| Ok(confirm_answer);
    let mut build_runner = |ws: &Path, pkgs: &[String]| {
        build_calls.push((ws.to_path_buf(), pkgs.to_vec()));
        Ok(build_ok)
    };
    let outcome = run_migrate(
        &MigrateOptions {
            workspace: fx.ws.clone(),
            write,
            assume_yes,
        },
        &mut MigrateDeps {
            engine,
            is_tty,
            confirm: &mut confirm,
            build_runner: &mut build_runner,
            out: &mut stdout,
            interrupted: &|| false,
        },
    )?;
    Ok(RunResult {
        outcome,
        stdout,
        build_calls,
    })
}

fn manifest_value(fx: &Fixture) -> serde_json::Value {
    let text = std::fs::read_to_string(fx.ws.join(MANIFEST_REL_PATH)).expect("manifest exists");
    serde_json::from_str(&text).expect("manifest parses")
}

// ---------------------------------------------------------------------------
// Dry-run.
// ---------------------------------------------------------------------------

#[test]
fn dry_run_reports_diff_candidates_and_writes_only_the_manifest() {
    let fx = make_workspace();
    let mut engine = engine_for(&fx);
    let r = drive(&fx, &mut engine, false, false, true, true, true).expect("runs");
    assert!(matches!(r.outcome, MigrateOutcome::Reported));
    let out = String::from_utf8(r.stdout).unwrap();

    // The diff carries the decided 3-line shape and nothing touches the fill.
    assert!(out.contains("-  auto msg = std::make_unique<std_msgs::msg::String>();"));
    assert!(out.contains("+  auto loaned = pub_->borrow_loaned_message();"));
    assert!(out.contains("+  auto msg = &loaned.get();"));
    assert!(out.contains("+  pub_->publish(std::move(loaned));"));
    assert!(
        !out.contains("-  msg->data"),
        "fill lines must be untouched:\n{out}"
    );
    // The candidates section names the reason.
    assert!(out.contains("pointer-escapes"));
    // Dry-run footer names the write path.
    assert!(out.contains("dry-run: nothing was modified"));

    // Nothing but the manifest was written.
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL,
        "dry-run must not touch sources"
    );
    assert!(!fx.ws.join(PATCH_FILENAME).exists());
    assert!(r.build_calls.is_empty(), "dry-run must not build");
    // The WHOLE workspace is untouched apart from the manifest (spot-checking
    // talker/patch would miss a regression writing
    // package.xml or any other file): no tracked change, and the only
    // untracked addition is the manifest's directory.
    // The DEFAULT porcelain collapses an entirely
    // untracked directory to `?? .cerulion/`, so a regression that wrote
    // extra files beside the manifest would hide behind the directory entry.
    // `--untracked-files=all` enumerates them, and the manifest is named
    // exactly.
    let porcelain = git(&fx.ws, &["status", "--porcelain", "--untracked-files=all"]);
    for line in porcelain.lines() {
        assert_eq!(
            line,
            format!("?? {MANIFEST_REL_PATH}"),
            "dry-run may add ONLY the manifest; workspace state: {porcelain}"
        );
    }

    let m = manifest_value(&fx);
    assert_eq!(m["schema"], "cerulion-ros2-migrate-manifest");
    assert_eq!(m["version"], 1);
    assert_eq!(m["generated_by"], "dry-run");
    assert_eq!(m["counts"]["call_sites"], 1);
    assert_eq!(m["counts"]["packages"], 1);
    assert_eq!(m["counts"]["manual_candidates"], 1);
    assert_eq!(m["candidates"][0]["package"], "demo_pkg");
    assert_eq!(m["candidates"][0]["kind"], "unique_ptr");
    assert_eq!(m["candidates"][0]["file"], "src/demo_pkg/src/talker.cpp");
    assert_eq!(m["manual_candidates"][0]["reason"], "pointer-escapes");
    assert_eq!(m["decision"], serde_json::Value::Null);
    // The workspace key names the real HEAD.
    let head = git(&fx.ws, &["rev-parse", "HEAD"]);
    assert_eq!(m["workspace_key"]["head"], head.as_str());
    assert_eq!(m["workspace_key"]["dirty"], false);
}

#[test]
fn dry_run_is_byte_deterministic_across_two_runs() {
    let fx = make_workspace();
    let mut engine = engine_for(&fx);
    let a = drive(&fx, &mut engine, false, false, true, true, true).expect("runs");
    let manifest_a = std::fs::read(fx.ws.join(MANIFEST_REL_PATH)).unwrap();
    let mut engine = engine_for(&fx);
    let b = drive(&fx, &mut engine, false, false, true, true, true).expect("runs");
    let manifest_b = std::fs::read(fx.ws.join(MANIFEST_REL_PATH)).unwrap();
    assert_eq!(a.stdout, b.stdout, "dry-run stdout must be byte-identical");
    assert_eq!(manifest_a, manifest_b, "manifest must be byte-identical");
}

#[test]
fn dry_run_preserves_a_consumer_written_decision_slot() {
    let fx = make_workspace();
    let mut engine = engine_for(&fx);
    drive(&fx, &mut engine, false, false, true, true, true).expect("runs");
    // Simulate the launcher recording a declined offer.
    let path = fx.ws.join(MANIFEST_REL_PATH);
    let mut m: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    m["decision"] = serde_json::json!({"declined_at_key": "abc123"});
    std::fs::write(&path, serde_json::to_string_pretty(&m).unwrap()).unwrap();

    let mut engine = engine_for(&fx);
    drive(&fx, &mut engine, false, false, true, true, true).expect("runs");
    let m = manifest_value(&fx);
    assert_eq!(
        m["decision"]["declined_at_key"], "abc123",
        "a dry-run refresh must not forget the consumer's decision"
    );
}

#[test]
fn dry_run_reports_rclpy_nodes_report_only() {
    let fx = make_workspace();
    let py_pkg = fx.ws.join("src/py_pkg");
    std::fs::create_dir_all(&py_pkg).unwrap();
    std::fs::write(
        py_pkg.join("package.xml"),
        "<package format=\"3\"><name>py_pkg</name></package>",
    )
    .unwrap();
    std::fs::write(
        py_pkg.join("talker.py"),
        "import rclpy\np = n.create_publisher(String, 't', 10)\n\
         q = n.create_publisher(String, 'u', 10)\n",
    )
    .unwrap();
    git_commit_all(&fx.ws, "add py node");
    let py_before = std::fs::read(py_pkg.join("talker.py")).unwrap();
    let mut engine = engine_for(&fx);
    let r = drive(&fx, &mut engine, false, false, true, true, true).expect("runs");
    let out = String::from_utf8(r.stdout).unwrap();
    assert!(out.contains("py_pkg  src/py_pkg/talker.py  (2 create_publisher call site(s))"));
    assert!(out.contains("No Python file was or will be modified"));
    // "Report-only" is asserted on the BYTES, not just the
    // report text — the Python source is byte-identical after the run,
    // and the workspace carries no other modification or new file (the
    // manifest's directory is the sole untracked addition).
    assert_eq!(
        std::fs::read(py_pkg.join("talker.py")).unwrap(),
        py_before,
        "report-only must leave the Python source byte-identical"
    );
    // `--untracked-files=all` for the same reason as the twin
    // above — the collapsed `?? .cerulion/` hides anything written beside the
    // manifest.
    let porcelain = git(&fx.ws, &["status", "--porcelain", "--untracked-files=all"]);
    for line in porcelain.lines() {
        assert_eq!(
            line,
            format!("?? {MANIFEST_REL_PATH}"),
            "report-only side effects beyond the manifest: {porcelain}"
        );
    }
    let m = manifest_value(&fx);
    assert_eq!(m["counts"]["rclpy_nodes"], 1);
    assert_eq!(m["rclpy"][0]["package"], "py_pkg");
    assert_eq!(m["rclpy"][0]["publishers"], 2);
}

#[test]
fn missing_compile_db_names_the_exact_colcon_command() {
    let fx = make_workspace();
    std::fs::remove_dir_all(fx.ws.join("build")).unwrap();
    let mut engine = engine_for(&fx);
    let err = drive(&fx, &mut engine, false, false, true, true, true)
        .unwrap_err()
        .to_string();
    assert!(
        err.contains("colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"),
        "remediation must name the exact command, got: {err}"
    );
}

#[test]
fn a_file_changed_since_analysis_refuses_loudly() {
    let fx = make_workspace();
    let mut engine = engine_for(&fx); // offsets against the ORIGINAL bytes
    std::fs::write(&fx.talker, TALKER_ORIGINAL.replace("hello", "helllo")).unwrap();
    git_commit_all(&fx.ws, "drift");
    let err = drive(&fx, &mut engine, false, false, true, true, true)
        .unwrap_err()
        .to_string();
    assert!(
        err.contains("no longer match") || err.contains("out of bounds"),
        "got: {err}"
    );
}

// ---------------------------------------------------------------------------
// --write.
// ---------------------------------------------------------------------------

#[test]
fn write_applies_commits_emits_patch_and_builds_affected_packages() {
    let fx = make_workspace();
    let commits_before = git(&fx.ws, &["rev-list", "--count", "HEAD"]);

    // The dry-run diff is the byte oracle the patch must equal.
    let mut engine = engine_for(&fx);
    let dry = drive(&fx, &mut engine, false, false, true, true, true).expect("runs");
    let dry_out = String::from_utf8(dry.stdout).unwrap();

    let mut engine = engine_for(&fx);
    let r = drive(&fx, &mut engine, true, true, false, true, true).expect("applies");
    let MigrateOutcome::Applied {
        commit,
        affected_packages,
        build_error,
    } = r.outcome
    else {
        panic!("expected Applied, got {:?}", r.outcome);
    };
    assert!(build_error.is_none());
    assert_eq!(affected_packages, vec!["demo_pkg".to_string()]);

    // The file now carries the hand-written migrated oracle.
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_MIGRATED
    );

    // ONE commit, by the verb, at HEAD.
    let commits_after = git(&fx.ws, &["rev-list", "--count", "HEAD"]);
    assert_eq!(
        commits_after.parse::<u64>().unwrap(),
        commits_before.parse::<u64>().unwrap() + 1
    );
    assert_eq!(git(&fx.ws, &["rev-parse", "HEAD"]), commit);
    let subject = git(&fx.ws, &["log", "-1", "--format=%s"]);
    assert_eq!(
        subject, "ros2 migrate: rewrite 1 publish call site(s) to the loaned-message API",
        "commit subject: {subject}"
    );
    // The tree is clean apart from untracked artifacts (patch + manifest).
    let porcelain = git(&fx.ws, &["status", "--porcelain"]);
    assert!(
        porcelain.lines().all(|l| l.starts_with("??")),
        "tracked changes remain: {porcelain}"
    );

    // The patch file is BYTE-IDENTICAL to the diff the dry-run printed —
    // same code path, asserted by EXACT equality, not containment
    // (`dry_out.contains(&patch)` passes on a TRUNCATED patch, since a
    // prefix is a substring). The report frames the diff between the
    // PROPOSED-REWRITES ruler and the MANUAL CANDIDATES header; extract
    // exactly that region and compare bytes both ways.
    let patch = std::fs::read_to_string(fx.ws.join(PATCH_FILENAME)).unwrap();
    assert!(!patch.is_empty());
    let ruler = "=================================================\n";
    let diff_start = dry_out
        .find(ruler)
        .expect("dry-run report must carry the PROPOSED REWRITES ruler")
        + ruler.len();
    let after_ruler = &dry_out[diff_start..];
    let diff_end = after_ruler
        .find("\nMANUAL CANDIDATES (")
        .expect("dry-run report must carry the MANUAL CANDIDATES header");
    assert_eq!(
        &after_ruler[..diff_end],
        patch,
        "the dry-run's printed diff and the written patch must be \
         byte-identical — same plan, same bytes"
    );

    // REAL git is the reversibility oracle: apply -R restores the original.
    git(&fx.ws, &["apply", "-R", PATCH_FILENAME]);
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL
    );
    git(&fx.ws, &["apply", PATCH_FILENAME]); // leave the tree consistent

    // The build runner ran once, in the workspace, on the affected set.
    assert_eq!(r.build_calls.len(), 1);
    assert_eq!(r.build_calls[0].0, fx.ws);
    assert_eq!(r.build_calls[0].1, vec!["demo_pkg".to_string()]);

    // The manifest consumed the applied candidates and re-keyed.
    let m = manifest_value(&fx);
    assert_eq!(m["generated_by"], "write");
    assert_eq!(m["counts"]["call_sites"], 0);
    assert_eq!(m["candidates"].as_array().unwrap().len(), 0);
    assert_eq!(m["counts"]["manual_candidates"], 1);
    assert_eq!(m["workspace_key"]["head"], commit.as_str());
}

#[test]
fn write_refuses_without_tty_and_without_yes_naming_both() {
    let fx = make_workspace();
    let mut engine = engine_for(&fx);
    let err = drive(&fx, &mut engine, true, false, false, true, true)
        .unwrap_err()
        .to_string();
    assert!(err.contains("--yes"), "must name --yes, got: {err}");
    assert!(err.contains("nothing was written"));
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL
    );
    assert!(!fx.ws.join(PATCH_FILENAME).exists());
}

#[test]
fn write_decline_leaves_everything_untouched() {
    let fx = make_workspace();
    let mut engine = engine_for(&fx);
    let r = drive(&fx, &mut engine, true, false, true, false, true).expect("runs");
    assert!(matches!(r.outcome, MigrateOutcome::Declined));
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL
    );
    assert!(!fx.ws.join(PATCH_FILENAME).exists());
    assert!(r.build_calls.is_empty());
}

/// Build the same workspace with the package ENTIRELY UNTRACKED: the
/// baseline commit carries only `.gitignore`, so `src/` never entered the
/// index (UNTRACKED_BASELINE).
fn make_workspace_with_untracked_package() -> Fixture {
    let fx = make_workspace();
    // Rewind the baseline to `.gitignore` alone: the package files leave the
    // index and become untracked, exactly as a package copied into a
    // workspace after its last commit.
    git(&fx.ws, &["rm", "-r", "-q", "--cached", "src"]);
    git(
        &fx.ws,
        &[
            "-c",
            "user.email=test@example.invalid",
            "-c",
            "user.name=test",
            "commit",
            "-qm",
            "baseline without the package",
        ],
    );
    assert!(
        git(&fx.ws, &["ls-files", "src"]).is_empty(),
        "the package must be entirely untracked"
    );
    fx
}

/// UNTRACKED_BASELINE: git's default
/// status collapses an entirely untracked directory to ONE `?? src/` entry,
/// which matches no planned path, so under it a planned file inside an untracked
/// package would sail past the write gate — the source would be committed and a
/// `git revert` of the migration would then DELETE the user's original
/// file. With `--untracked-files=all` the planned file is enumerated and
/// the gate refuses it, naming it, with nothing staged or committed.
#[test]
fn write_refuses_a_planned_file_inside_an_entirely_untracked_directory() {
    let fx = make_workspace_with_untracked_package();
    let commits_before = git(&fx.ws, &["rev-list", "--count", "HEAD"]);
    let mut engine = engine_for(&fx);
    let err = drive(&fx, &mut engine, true, true, false, true, true)
        .unwrap_err()
        .to_string();
    assert!(err.contains("dirty git tree"), "got: {err}");
    assert!(
        err.contains("src/demo_pkg/src/talker.cpp"),
        "must name the untracked planned file: {err}"
    );
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL,
        "the untracked source must be untouched"
    );
    assert_eq!(
        git(&fx.ws, &["rev-list", "--count", "HEAD"]),
        commits_before,
        "nothing may be committed"
    );
    assert_eq!(
        git(&fx.ws, &["diff", "--cached", "--name-only"]),
        "",
        "nothing may be staged"
    );
    assert!(
        git(&fx.ws, &["ls-files", "src"]).is_empty(),
        "the package must still be untracked"
    );
}

/// A failed `git status` must never be
/// `unwrap_or_default()`ed into an empty — clean — status, or `--write`
/// could commit over uncommitted tracked changes. A corrupt index makes
/// `git status` fail (rc 128, "index file smaller than expected") while
/// both `rev-parse` probes still succeed — exactly the shape the gate must
/// not mistake for clean. The write refuses naming the failure; the
/// dry-run still reports but records `dirty: true`.
#[test]
fn write_refuses_when_git_status_fails_instead_of_treating_the_tree_as_clean() {
    let fx = make_workspace();
    std::fs::write(fx.ws.join(".git/index"), b"garbage").unwrap();
    let mut engine = engine_for(&fx);
    let err = drive(&fx, &mut engine, true, true, false, true, true)
        .unwrap_err()
        .to_string();
    assert!(err.contains("could not be verified clean"), "got: {err}");
    assert!(
        err.contains("git status"),
        "must name the failed command: {err}"
    );
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL,
        "nothing may be written"
    );
    let mut engine = engine_for(&fx);
    drive(&fx, &mut engine, false, false, true, true, true).expect("the dry-run still reports");
    assert_eq!(
        manifest_value(&fx)["workspace_key"]["dirty"],
        serde_json::json!(true),
        "an unverifiable tree is recorded dirty, never clean"
    );
}

#[test]
fn write_refuses_a_dirty_tree_naming_the_offender() {
    let fx = make_workspace();
    std::fs::write(
        fx.ws.join("src/demo_pkg/package.xml"),
        "<package format=\"3\"><name>demo_pkg</name><!-- dirty --></package>",
    )
    .unwrap();
    let mut engine = engine_for(&fx);
    let err = drive(&fx, &mut engine, true, true, false, true, true)
        .unwrap_err()
        .to_string();
    assert!(err.contains("dirty git tree"), "got: {err}");
    assert!(err.contains("package.xml"), "must name the offender: {err}");
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL
    );
}

#[test]
fn write_build_failure_is_loud_and_names_the_revert_path() {
    let fx = make_workspace();
    let mut engine = engine_for(&fx);
    let r = drive(&fx, &mut engine, true, true, false, true, false).expect("applies");
    let MigrateOutcome::Applied {
        commit,
        build_error,
        ..
    } = r.outcome
    else {
        panic!("expected Applied");
    };
    let msg = build_error.expect("build failure must be reported");
    assert!(msg.contains("FAILED"), "got: {msg}");
    assert!(
        msg.contains(&format!("git revert {commit}")),
        "must name the revert commit: {msg}"
    );
    assert!(msg.contains(PATCH_FILENAME));
    // The commit stays — the message says so and the log agrees.
    assert_eq!(git(&fx.ws, &["rev-parse", "HEAD"]), commit);
}

#[test]
fn write_with_nothing_provable_applies_nothing() {
    let fx = make_workspace();
    let mut engine = empty_engine_for(&fx);
    let r = drive(&fx, &mut engine, true, true, false, true, true).expect("runs");
    assert!(matches!(r.outcome, MigrateOutcome::NothingToApply));
    assert!(!fx.ws.join(PATCH_FILENAME).exists());
    assert!(r.build_calls.is_empty());
    let porcelain = git(&fx.ws, &["status", "--porcelain"]);
    assert!(porcelain.lines().all(|l| l.starts_with("??")));
}

#[test]
fn second_dry_run_after_migration_reports_no_rewrites() {
    // Desk-side idempotence pin at the seam: the real tool proposes nothing
    // for migrated code (its already-loaned skip is container-pinned); an
    // empty analysis must render the explicit "none" report and an empty
    // diff.
    let fx = make_workspace();
    let mut engine = engine_for(&fx);
    drive(&fx, &mut engine, true, true, false, true, true).expect("applies");
    let mut engine = empty_engine_for(&fx);
    let r = drive(&fx, &mut engine, false, false, true, true, true).expect("runs");
    let out = String::from_utf8(r.stdout).unwrap();
    assert!(out.contains("none — no publish call site was proven safe to rewrite."));
    assert!(
        !out.contains("--- a/"),
        "no diff on the second dry run:\n{out}"
    );
}

/// A workspace OUTSIDE any git repository: the dry-run still reports (the
/// manifest's key is `head: null` — a consumer treats that as
/// never-matching), and `--write` refuses naming the undo contract.
#[test]
fn outside_a_git_repo_dry_run_reports_and_write_refuses() {
    let fx = make_workspace();
    std::fs::remove_dir_all(fx.ws.join(".git")).unwrap();

    let mut engine = engine_for(&fx);
    let r = drive(&fx, &mut engine, false, false, true, true, true).expect("dry-run runs");
    assert!(matches!(r.outcome, MigrateOutcome::Reported));
    let m = manifest_value(&fx);
    assert_eq!(m["workspace_key"]["head"], serde_json::Value::Null);

    let mut engine = engine_for(&fx);
    let err = drive(&fx, &mut engine, true, true, false, true, true)
        .unwrap_err()
        .to_string();
    assert!(
        err.contains("not inside a git repository"),
        "must refuse without a repo: {err}"
    );
    assert!(
        err.contains("git revert"),
        "must name the undo contract: {err}"
    );
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL,
        "the refusal must not write"
    );
}

/// The byte-level oracle over the APPLY PATH and the edit
/// CONTRACT: applying this file's canned edit JSON yields exactly ONE `->`
/// per rewritten call — never a doubled operator. SCOPE: the JSON
/// here is hand-written, so this test pins the contract spec + the Rust
/// apply path, NOT the clang prover's emission — a prover regression back
/// to `pub_->->…` is caught in-container by `assert_matrix.py`'s
/// exactly-one-arrow pins over REAL tool output and by `run_matrix.sh` compiling
/// the applied bytes.
#[test]
fn applied_bytes_carry_exactly_one_arrow_per_rewritten_call() {
    let fx = make_workspace();
    let analysis = cerulion_cli_engine::ros2_migrate::parse_tool_output(&talker_analysis(&fx))
        .expect("fixture analysis parses");
    let edits: Vec<_> = analysis
        .rewrites
        .iter()
        .flat_map(|rw| rw.edits.iter().cloned())
        .collect();
    let applied =
        cerulion_cli_engine::ros2_migrate::apply_edits(TALKER_ORIGINAL.as_bytes(), &edits)
            .expect("applies");
    let applied = String::from_utf8(applied).expect("utf8");
    assert!(
        !applied.contains("->->"),
        "doubled arrow in applied bytes:\n{applied}"
    );
    assert_eq!(
        applied.matches("pub_->borrow_loaned_message()").count(),
        1,
        "exactly one arrow-borrow in:\n{applied}"
    );
    assert_eq!(
        applied.matches("pub_->publish(std::move(loaned))").count(),
        1,
        "exactly one arrow-publish in:\n{applied}"
    );
    // And the whole file is the hand-written migrated oracle.
    assert_eq!(applied, TALKER_MIGRATED);
}

/// A file edited WHILE the user reviews the diff (between plan
/// and consent) must refuse — never be overwritten with stale planned
/// bytes. The consent seam is exactly that window, so the confirm closure
/// mutates the file and then answers yes.
#[test]
fn write_refuses_when_a_file_changes_during_the_consent_prompt() {
    let fx = make_workspace();
    let mutated = format!("{TALKER_ORIGINAL}// reviewed and tweaked\n");
    let mut engine = engine_for(&fx);
    let mut stdout: Vec<u8> = Vec::new();
    let talker = fx.talker.clone();
    let mutated_for_confirm = mutated.clone();
    let mut confirm = move |_prompt: &str| {
        std::fs::write(&talker, &mutated_for_confirm).unwrap();
        Ok(true)
    };
    let mut build_runner = |_ws: &Path, _pkgs: &[String]| Ok(true);
    let err = run_migrate(
        &MigrateOptions {
            workspace: fx.ws.clone(),
            write: true,
            assume_yes: false,
        },
        &mut MigrateDeps {
            engine: &mut engine,
            is_tty: true,
            confirm: &mut confirm,
            build_runner: &mut build_runner,
            out: &mut stdout,
            interrupted: &|| false,
        },
    )
    .unwrap_err()
    .to_string();
    assert!(
        err.contains("changed while the migration was being reviewed"),
        "got: {err}"
    );
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        mutated,
        "the user's mid-review edit must survive untouched"
    );
    assert!(!fx.ws.join(PATCH_FILENAME).exists(), "nothing written");
}

/// Source-swap TOCTOU (the
/// identical-bytes-foreign-occupant pattern, at the SOURCE gate): a
/// different regular file whose contents MATCH the reviewed source,
/// atomically renamed over the planned path from the consent window,
/// passes a bytes-only pre-write check — the migration would return Applied
/// and write generated output into a file consent never covered. The gate
/// therefore also requires the plan-time `OccupantIdentity` (dev, ino, mtime,
/// captured off the same fd as the bytes): the swap must REFUSE with the
/// identity message and the replacement must survive byte-intact.
///
/// Unix-only like the identity itself (the non-unix arm is the declared
/// weaker bytes-only tier). Reverting the gate
/// to bytes-only fails this test with `Applied` + migrated bytes in the
/// replacement.
#[cfg(unix)]
#[test]
fn write_refuses_an_identical_bytes_source_swapped_during_the_consent_prompt() {
    let fx = make_workspace();
    let mut engine = engine_for(&fx);
    let mut stdout: Vec<u8> = Vec::new();
    let talker = fx.talker.clone();
    let mut confirm = move |_prompt: &str| {
        // Same bytes, different inode: build the replacement beside the
        // source, then atomically rename it over the path — exactly the
        // shape a byte-only gate accepts.
        let bytes = std::fs::read(&talker).unwrap();
        let swap = talker.with_extension("cpp.swap");
        std::fs::write(&swap, &bytes).unwrap();
        std::fs::rename(&swap, &talker).unwrap();
        Ok(true)
    };
    let mut build_runner = |_ws: &Path, _pkgs: &[String]| Ok(true);
    let err = run_migrate(
        &MigrateOptions {
            workspace: fx.ws.clone(),
            write: true,
            assume_yes: false,
        },
        &mut MigrateDeps {
            engine: &mut engine,
            is_tty: true,
            confirm: &mut confirm,
            build_runner: &mut build_runner,
            out: &mut stdout,
            interrupted: &|| false,
        },
    )
    .unwrap_err()
    .to_string();
    assert!(
        err.contains("changed identity since the migration was reviewed"),
        "the swap must refuse on IDENTITY, not pass the byte check; got: {err}"
    );
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL,
        "the replacement (identical bytes) must survive untouched — no \
         generated output may land in a file consent never covered"
    );
    assert!(!fx.ws.join(PATCH_FILENAME).exists(), "nothing written");
}

/// The same identical-bytes swap, landed
/// AFTER the pre-write identity gate has already passed — the
/// gate→write gap. The injection point is the first interrupt safepoint
/// (polled at the head of the WRITE PHASE, strictly after the gate): the
/// closure performs the atomic rename ONCE and always answers "not
/// interrupted". A `write_phased` that opened the replacement
/// `O_TRUNC` would truncate it at open, write generated output and return
/// Applied. The write seam is identity-BOUND (`rewrite_verified`
/// opens without `O_TRUNC`, fstats the OPENED fd against the plan-time
/// identity, and truncates only on a match — identity and write on one
/// descriptor), so the swap must refuse with the replacement unmodified.
///
/// Neutralizing the write-seam identity compare
/// fails THIS test (Applied + migrated bytes in the replacement) while the
/// consent-window twin above stays green — the two seams are independently
/// load-bearing.
#[cfg(unix)]
#[test]
fn write_refuses_an_identical_bytes_source_swapped_after_the_pre_write_gate() {
    let fx = make_workspace();
    let mut engine = engine_for(&fx);
    let mut stdout: Vec<u8> = Vec::new();
    let talker = fx.talker.clone();
    // The swap must land INSIDE the write batch — after the TOCTOU gate,
    // before the first write — which is the first poll that sees the
    // workspace lock FILE (the acquire creates it; this fixture starts
    // without it). Anchored on that observable rather than on the poll's
    // ordinal: the consent boundary polls the same seam BEFORE the lock,
    // and a count would move the swap in front of the gate.
    let lock_file = fx.ws.join(LOCK_DIR).join(LOCK_FILE);
    assert!(
        !lock_file.exists(),
        "this fixture must start without the lock file — the anchor below is the pre-write \
         safepoint only on an UNCONTENDED acquire that creates it"
    );
    let swapped = std::sync::atomic::AtomicBool::new(false);
    let interrupted = move || {
        if lock_file.exists() && !swapped.swap(true, std::sync::atomic::Ordering::SeqCst) {
            // Same bytes, different inode, atomically renamed over the
            // path — after the TOCTOU gate, before the first write.
            let bytes = std::fs::read(&talker).unwrap();
            let swap = talker.with_extension("cpp.swap");
            std::fs::write(&swap, &bytes).unwrap();
            std::fs::rename(&swap, &talker).unwrap();
        }
        false
    };
    let mut confirm = |_prompt: &str| Ok(true);
    let mut build_runner = |_ws: &Path, _pkgs: &[String]| Ok(true);
    let err = run_migrate(
        &MigrateOptions {
            workspace: fx.ws.clone(),
            write: true,
            assume_yes: false,
        },
        &mut MigrateDeps {
            engine: &mut engine,
            is_tty: true,
            confirm: &mut confirm,
            build_runner: &mut build_runner,
            out: &mut stdout,
            interrupted: &interrupted,
        },
    )
    .unwrap_err()
    .to_string();
    assert!(
        err.contains("changed identity since the migration was reviewed")
            && err.contains("moment of write"),
        "the write seam must refuse on IDENTITY, at the write; got: {err}"
    );
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL,
        "the post-gate replacement must survive unmodified — never \
         truncated, never rewritten"
    );
    assert!(
        !fx.ws.join(PATCH_FILENAME).exists(),
        "the patch written ahead of the refused file must be rolled back"
    );
}

/// A forge that defeats the stat identity — the
/// SAME inode rewritten in place with same-LENGTH different bytes and its
/// mtime RESTORED (`File::set_times`), landed post-gate at the first
/// interrupt safepoint. (dev, ino, mtime, mtime_nsec) all match the
/// reviewed identity, so only the write seam's CONTENT binding — the
/// occupant's bytes read back through the same descriptor — can refuse
/// it. Without that binding this reaches `ftruncate` + `write_all` and returns
/// Applied with generated output in a file nobody reviewed.
///
/// Dropping the byte compare fails THIS test
/// (Applied + migrated bytes over the forge) while the identity arms and
/// the byte-identical control stay green.
#[cfg(unix)]
#[test]
fn write_refuses_a_same_inode_content_forge_with_a_restored_mtime() {
    let fx = make_workspace();
    let mut engine = engine_for(&fx);
    let mut stdout: Vec<u8> = Vec::new();
    let talker = fx.talker.clone();
    let forged = TALKER_ORIGINAL.replace("demo.hpp", "demo.hpq");
    assert_eq!(
        forged.len(),
        TALKER_ORIGINAL.len(),
        "the forge must keep the length or the len half of the identity \
         would catch it first"
    );
    let forged_for_confirm = forged.clone();
    // The swap must land INSIDE the write batch — after the TOCTOU gate,
    // before the first write — which is the first poll that sees the
    // workspace lock FILE (the acquire creates it; this fixture starts
    // without it). Anchored on that observable rather than on the poll's
    // ordinal: the consent boundary polls the same seam BEFORE the lock,
    // and a count would move the swap in front of the gate.
    let lock_file = fx.ws.join(LOCK_DIR).join(LOCK_FILE);
    assert!(
        !lock_file.exists(),
        "this fixture must start without the lock file — the anchor below is the pre-write \
         safepoint only on an UNCONTENDED acquire that creates it"
    );
    let swapped = std::sync::atomic::AtomicBool::new(false);
    let interrupted = move || {
        if lock_file.exists() && !swapped.swap(true, std::sync::atomic::Ordering::SeqCst) {
            use std::io::Write as _;
            let reviewed_mtime = std::fs::metadata(&talker).unwrap().modified().unwrap();
            let mut f = std::fs::OpenOptions::new()
                .write(true)
                .open(&talker)
                .unwrap();
            // Same length at offset 0 = an exact in-place overwrite —
            // same inode, no rename.
            f.write_all(forged_for_confirm.as_bytes()).unwrap();
            f.set_times(std::fs::FileTimes::new().set_modified(reviewed_mtime))
                .unwrap();
        }
        false
    };
    let mut confirm = |_prompt: &str| Ok(true);
    let mut build_runner = |_ws: &Path, _pkgs: &[String]| Ok(true);
    let err = run_migrate(
        &MigrateOptions {
            workspace: fx.ws.clone(),
            write: true,
            assume_yes: false,
        },
        &mut MigrateDeps {
            engine: &mut engine,
            is_tty: true,
            confirm: &mut confirm,
            build_runner: &mut build_runner,
            out: &mut stdout,
            interrupted: &interrupted,
        },
    )
    .unwrap_err()
    .to_string();
    assert!(
        err.contains("changed identity since the migration was reviewed")
            && err.contains("moment of write"),
        "the content binding must refuse at the write; got: {err}"
    );
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        forged,
        "the forged content must survive — never truncated, never rewritten"
    );
    assert!(!fx.ws.join(PATCH_FILENAME).exists(), "patch rolled back");
}

/// The OVERSIZED twin of the content forge above — a same-inode
/// APPEND with a restored mtime, landed post-gate. The unix identity
/// carries no length, so the stat identity passes; the write seam's
/// BOUNDED verification read (reviewed len + 1) sees the divergence at
/// the first byte past the reviewed size and refuses — a prefix-matching
/// LONGER occupant must never be accepted on its prefix. (That the read
/// is take-capped — never a full read of the tail — is pinned
/// structurally in the lib tests; this arm pins the refusal semantics
/// over the real --write path.)
#[cfg(unix)]
#[test]
fn write_refuses_an_appended_tail_with_a_restored_mtime() {
    let fx = make_workspace();
    let mut engine = engine_for(&fx);
    let mut stdout: Vec<u8> = Vec::new();
    let talker = fx.talker.clone();
    // The swap must land INSIDE the write batch — after the TOCTOU gate,
    // before the first write — which is the first poll that sees the
    // workspace lock FILE (the acquire creates it; this fixture starts
    // without it). Anchored on that observable rather than on the poll's
    // ordinal: the consent boundary polls the same seam BEFORE the lock,
    // and a count would move the swap in front of the gate.
    let lock_file = fx.ws.join(LOCK_DIR).join(LOCK_FILE);
    assert!(
        !lock_file.exists(),
        "this fixture must start without the lock file — the anchor below is the pre-write \
         safepoint only on an UNCONTENDED acquire that creates it"
    );
    let swapped = std::sync::atomic::AtomicBool::new(false);
    let interrupted = move || {
        if lock_file.exists() && !swapped.swap(true, std::sync::atomic::Ordering::SeqCst) {
            use std::io::Write as _;
            let reviewed_mtime = std::fs::metadata(&talker).unwrap().modified().unwrap();
            let mut f = std::fs::OpenOptions::new()
                .append(true)
                .open(&talker)
                .unwrap();
            f.write_all(b"// appended tail\n").unwrap();
            f.set_times(std::fs::FileTimes::new().set_modified(reviewed_mtime))
                .unwrap();
        }
        false
    };
    let mut confirm = |_prompt: &str| Ok(true);
    let mut build_runner = |_ws: &Path, _pkgs: &[String]| Ok(true);
    let err = run_migrate(
        &MigrateOptions {
            workspace: fx.ws.clone(),
            write: true,
            assume_yes: false,
        },
        &mut MigrateDeps {
            engine: &mut engine,
            is_tty: true,
            confirm: &mut confirm,
            build_runner: &mut build_runner,
            out: &mut stdout,
            interrupted: &interrupted,
        },
    )
    .unwrap_err()
    .to_string();
    assert!(
        err.contains("changed identity since the migration was reviewed")
            && err.contains("moment of write"),
        "the bounded content read must refuse the longer occupant; got: {err}"
    );
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        format!("{TALKER_ORIGINAL}// appended tail\n"),
        "the appended occupant must survive — never truncated"
    );
    assert!(!fx.ws.join(PATCH_FILENAME).exists(), "patch rolled back");
}

/// The content-forge control — the harmless arm, and the reasoning that collapses
/// the residual: an occupant that is BYTE-IDENTICAL to the reviewed
/// source (here the same inode rewritten in place with the same bytes,
/// mtime restored) passes identity AND content, and the rewrite it then
/// receives is exactly the consented output bytes — so the strongest
/// forge that can still pass produces precisely the migration the user
/// approved.
#[cfg(unix)]
#[test]
fn a_byte_identical_occupant_proceeds_because_the_rewrite_is_the_consented_output() {
    let fx = make_workspace();
    let mut engine = engine_for(&fx);
    let mut stdout: Vec<u8> = Vec::new();
    let talker = fx.talker.clone();
    // The swap must land INSIDE the write batch — after the TOCTOU gate,
    // before the first write — which is the first poll that sees the
    // workspace lock FILE (the acquire creates it; this fixture starts
    // without it). Anchored on that observable rather than on the poll's
    // ordinal: the consent boundary polls the same seam BEFORE the lock,
    // and a count would move the swap in front of the gate.
    let lock_file = fx.ws.join(LOCK_DIR).join(LOCK_FILE);
    assert!(
        !lock_file.exists(),
        "this fixture must start without the lock file — the anchor below is the pre-write \
         safepoint only on an UNCONTENDED acquire that creates it"
    );
    // Shared with the assertion below: this control expects SUCCESS, and a
    // successful run with no swap at all produces exactly the same bytes —
    // so the arm must prove the forge really landed or it proves nothing.
    let swapped = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let swapped_in_closure = swapped.clone();
    let interrupted = move || {
        if lock_file.exists() && !swapped_in_closure.swap(true, std::sync::atomic::Ordering::SeqCst)
        {
            use std::io::Write as _;
            let reviewed_mtime = std::fs::metadata(&talker).unwrap().modified().unwrap();
            let mut f = std::fs::OpenOptions::new()
                .write(true)
                .open(&talker)
                .unwrap();
            f.write_all(TALKER_ORIGINAL.as_bytes()).unwrap();
            f.set_times(std::fs::FileTimes::new().set_modified(reviewed_mtime))
                .unwrap();
        }
        false
    };
    let mut confirm = |_prompt: &str| Ok(true);
    let mut build_runner = |_ws: &Path, _pkgs: &[String]| Ok(true);
    let outcome = run_migrate(
        &MigrateOptions {
            workspace: fx.ws.clone(),
            write: true,
            assume_yes: false,
        },
        &mut MigrateDeps {
            engine: &mut engine,
            is_tty: true,
            confirm: &mut confirm,
            build_runner: &mut build_runner,
            out: &mut stdout,
            interrupted: &interrupted,
        },
    )
    .expect("byte-identical occupant must proceed");
    assert!(
        swapped.load(std::sync::atomic::Ordering::SeqCst),
        "the byte-identical occupant was never swapped in — the lock-file anchor never \
         fired, so this control arm proved nothing"
    );
    assert!(
        matches!(outcome, MigrateOutcome::Applied { .. }),
        "got: {outcome:?}"
    );
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_MIGRATED,
        "the occupant received exactly the consented migrated bytes"
    );
}

/// A FAILED commit (here: a pre-commit hook rejecting it) must
/// roll the working tree back — sources restored, nothing staged, patch
/// removed — because HEAD is unchanged, so the documented `git revert`
/// recovery does not exist and stranding rewritten staged sources would be
/// worse than the starting state.
#[cfg(unix)]
#[test]
fn write_rolls_back_when_the_commit_fails() {
    use std::os::unix::fs::PermissionsExt as _;
    let fx = make_workspace();
    let hook = fx.ws.join(".git/hooks/pre-commit");
    std::fs::write(&hook, "#!/bin/sh\nexit 1\n").unwrap();
    std::fs::set_permissions(&hook, std::fs::Permissions::from_mode(0o755)).unwrap();

    let mut engine = engine_for(&fx);
    let err = drive(&fx, &mut engine, true, true, false, true, true)
        .unwrap_err()
        .to_string();
    assert!(err.contains("migration commit FAILED"), "got: {err}");
    assert!(err.contains("rolled back"), "got: {err}");
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL,
        "sources must be restored"
    );
    assert!(
        !fx.ws.join(PATCH_FILENAME).exists(),
        "patch must be removed"
    );
    let porcelain = git(&fx.ws, &["status", "--porcelain"]);
    assert!(
        porcelain.lines().all(|l| l.starts_with("??")),
        "nothing may remain staged/modified: {porcelain}"
    );
}

/// The engine binary CONTROLS the edit paths — an analysis
/// naming a file outside the workspace source tree must refuse loudly,
/// never rewrite an arbitrary readable file.
#[test]
fn analysis_naming_a_file_outside_the_workspace_refuses() {
    let fx = make_workspace();
    let outside_dir = tempfile::tempdir().expect("outside dir");
    let outside = outside_dir
        .path()
        .canonicalize()
        .unwrap()
        .join("innocent.cpp");
    std::fs::write(&outside, TALKER_ORIGINAL).unwrap();
    // Built through the parameterized helper (serde_json escaping), never a
    // string-replace over serialized JSON — a raw replace corrupts Windows
    // backslash paths and would silently leave the analysis pointing at the
    // in-workspace file.
    let analysis = talker_analysis_for(&fx.talker, &outside);
    let mut by_tu = HashMap::new();
    by_tu.insert(fx.talker.clone(), analysis);
    let mut engine = FixtureEngine { by_tu };
    let err = drive(&fx, &mut engine, false, false, true, true, true)
        .unwrap_err()
        .to_string();
    assert!(
        err.contains("OUTSIDE the workspace source tree"),
        "got: {err}"
    );
    assert_eq!(
        std::fs::read_to_string(&outside).unwrap(),
        TALKER_ORIGINAL,
        "the outside file must be untouched"
    );
}

/// A symlink cycle under src/ must not hang or overflow the
/// rclpy scan.
#[cfg(unix)]
#[test]
fn rclpy_scan_survives_a_symlinked_directory_cycle() {
    let fx = make_workspace();
    let py_pkg = fx.ws.join("src/py_pkg");
    std::fs::create_dir_all(&py_pkg).unwrap();
    std::fs::write(
        py_pkg.join("package.xml"),
        "<package format=\"3\"><name>py_pkg</name></package>",
    )
    .unwrap();
    std::fs::write(py_pkg.join("talker.py"), "import rclpy\n").unwrap();
    std::os::unix::fs::symlink(fx.ws.join("src"), py_pkg.join("loop")).unwrap();
    let nodes = cerulion_cli_engine::ros2_migrate::scan_rclpy(&fx.ws, &fx.ws.join("src"));
    assert_eq!(nodes.len(), 1, "exactly the real node, no cycle: {nodes:?}");
    assert_eq!(nodes[0].file, "src/py_pkg/talker.py");
}

/// The manifest is the dry-run's one machine-readable product
/// — a failed refresh is a FAILED dry-run (exit nonzero), though the
/// report itself must still have printed in full first.
#[test]
fn dry_run_fails_loudly_when_the_manifest_cannot_be_written() {
    let fx = make_workspace();
    // A FILE named `.cerulion` makes create_dir_all fail.
    std::fs::write(fx.ws.join(".cerulion"), b"not a directory").unwrap();
    let mut engine = engine_for(&fx);
    let mut stdout: Vec<u8> = Vec::new();
    let mut confirm = |_p: &str| Ok(true);
    let mut build_runner = |_w: &Path, _p: &[String]| Ok(true);
    let err = run_migrate(
        &MigrateOptions {
            workspace: fx.ws.clone(),
            write: false,
            assume_yes: false,
        },
        &mut MigrateDeps {
            engine: &mut engine,
            is_tty: true,
            confirm: &mut confirm,
            build_runner: &mut build_runner,
            out: &mut stdout,
            interrupted: &|| false,
        },
    )
    .unwrap_err()
    .to_string();
    assert!(err.contains("manifest refresh"), "got: {err}");
    let out = String::from_utf8(stdout).unwrap();
    // "The report printed in full" is asserted on representative
    // markers from EVERY section reached before the manifest write — the
    // header alone would pass a run truncated after it. (The dry-run footer is
    // deliberately absent here: it prints only on a SUCCESSFUL refresh.)
    assert!(
        out.contains("PROPOSED REWRITES"),
        "the report must print before the failure:\n{out}"
    );
    assert!(
        out.contains("+  auto loaned = pub_->borrow_loaned_message();"),
        "the diff body must print in full:\n{out}"
    );
    assert!(
        out.contains("MANUAL CANDIDATES ("),
        "the candidates section must print:\n{out}"
    );
    assert!(
        out.contains("RCLPY NODES ("),
        "the rclpy section must print:\n{out}"
    );
}

/// A PRE-EXISTING patch file (an earlier run's artifact) must be
/// RESTORED by a failure rollback, never deleted — a rollback that
/// removes the patch path unconditionally destroys the user's own
/// file.
#[cfg(unix)]
#[test]
fn a_failed_commit_restores_a_pre_existing_patch() {
    use std::os::unix::fs::PermissionsExt as _;
    let fx = make_workspace();
    const SENTINEL: &[u8] = b"--- an earlier migration's patch, user-owned\n";
    std::fs::write(fx.ws.join(PATCH_FILENAME), SENTINEL).unwrap();
    let hook = fx.ws.join(".git/hooks/pre-commit");
    std::fs::write(&hook, "#!/bin/sh\nexit 1\n").unwrap();
    std::fs::set_permissions(&hook, std::fs::Permissions::from_mode(0o755)).unwrap();

    let mut engine = engine_for(&fx);
    let err = drive(&fx, &mut engine, true, true, false, true, true)
        .unwrap_err()
        .to_string();
    assert!(err.contains("migration commit FAILED"), "got: {err}");
    assert_eq!(
        std::fs::read(fx.ws.join(PATCH_FILENAME)).unwrap(),
        SENTINEL,
        "the earlier run's patch must be restored byte-for-byte"
    );
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL
    );
}

/// A rejecting commit hook replaces the generated patch
/// with a WRITERLESS FIFO — a teardown that quarantines it and calls
/// `fs::read` blocks forever in the open, waiting for a writer: the
/// migration would never return and never reach the index cleanup. The
/// quarantine verify is type-gated and non-blocking: the FIFO
/// classifies FOREIGN without a byte read, is given back by `hard_link`
/// (which never opens it), and the rollback COMPLETES. Bounded harness
/// deliberately — a regression wedges rather than fails.
#[cfg(unix)]
#[test]
fn a_fifo_planted_at_the_patch_path_never_wedges_the_rollback() {
    use std::os::unix::fs::FileTypeExt as _;
    use std::os::unix::fs::PermissionsExt as _;
    let fx = make_workspace();
    let hook = fx.ws.join(".git/hooks/pre-commit");
    std::fs::write(
        &hook,
        format!(
            "#!/bin/sh\n\
             rm -f {p}\n\
             mkfifo {p}\n\
             exit 1\n",
            p = PATCH_FILENAME
        ),
    )
    .unwrap();
    std::fs::set_permissions(&hook, std::fs::Permissions::from_mode(0o755)).unwrap();

    // DETACHED thread + recv_timeout (not thread::scope, which would JOIN
    // the wedged thread and defeat the bound): the Arc keeps the tempdir
    // alive for the post-asserts either way.
    let fx = std::sync::Arc::new(fx);
    let (tx, rx) = std::sync::mpsc::channel();
    let fx2 = std::sync::Arc::clone(&fx);
    std::thread::spawn(move || {
        let mut engine = engine_for(&fx2);
        let err = drive(&fx2, &mut engine, true, true, false, true, true)
            .unwrap_err()
            .to_string();
        tx.send(err).ok();
    });
    let err = rx
        .recv_timeout(std::time::Duration::from_secs(60))
        .expect("the rollback must COMPLETE with a FIFO at the patch path — never wedge");

    assert!(err.contains("migration commit FAILED"), "got: {err}");
    // The FIFO survives AT THE PATH (quarantined, classified foreign
    // without being opened, returned by hard_link) and is reported.
    let meta = fx.ws.join(PATCH_FILENAME).symlink_metadata().unwrap();
    assert!(
        meta.file_type().is_fifo(),
        "the planted FIFO must be preserved at the patch path"
    );
    assert!(
        err.contains("not the migration's bytes"),
        "the preservation must be reported: {err}"
    );
    // No quarantine leftover.
    let leftovers: Vec<_> = std::fs::read_dir(&fx.ws)
        .unwrap()
        .filter_map(|e| e.ok())
        .filter(|e| e.file_name().to_string_lossy().contains(".rollback."))
        .collect();
    assert!(
        leftovers.is_empty(),
        "no quarantine leftover: {leftovers:?}"
    );
    // The rollback RAN TO COMPLETION: sources restored, index clean.
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL
    );
    let cached = git(&fx.ws, &["diff", "--cached", "--name-only"]);
    assert!(
        cached.trim().is_empty(),
        "index cleanup must be reached: {cached}"
    );
}

/// The source-side arm of the FIFO case: a planned SOURCE swapped for a FIFO
/// after consent is refused at the pre-write TOCTOU gate — loudly and
/// WITHOUT wedging (the gate's read is bounded + type-gated) — before
/// anything is written.
#[cfg(unix)]
#[test]
fn a_source_swapped_for_a_fifo_after_consent_refuses_without_wedging() {
    use std::os::unix::fs::FileTypeExt as _;
    let fx = std::sync::Arc::new(make_workspace());
    // DETACHED thread + recv_timeout — a wedged gate must fail the test,
    // not hang the suite (thread::scope would join the wedged thread).
    let (tx, rx) = std::sync::mpsc::channel();
    let fx2 = std::sync::Arc::clone(&fx);
    std::thread::spawn(move || {
        let mut engine = engine_for(&fx2);
        let mut stdout: Vec<u8> = Vec::new();
        let talker = fx2.talker.clone();
        let mut confirm = move |_prompt: &str| {
            // The swap: consent granted, then the source becomes a FIFO.
            std::fs::remove_file(&talker).unwrap();
            let st = std::process::Command::new("mkfifo")
                .arg(&talker)
                .status()
                .expect("mkfifo runs");
            assert!(st.success(), "mkfifo");
            Ok(true)
        };
        let mut build_calls = 0u32;
        let mut build_runner = |_ws: &Path, _pkgs: &[String]| {
            build_calls += 1;
            Ok(true)
        };
        let r = run_migrate(
            &MigrateOptions {
                workspace: fx2.ws.clone(),
                write: true,
                assume_yes: false,
            },
            &mut MigrateDeps {
                engine: &mut engine,
                is_tty: true,
                confirm: &mut confirm,
                build_runner: &mut build_runner,
                out: &mut stdout,
                interrupted: &|| false,
            },
        );
        tx.send((r.err().map(|e| e.to_string()), build_calls)).ok();
    });
    let (err, build_calls) = rx
        .recv_timeout(std::time::Duration::from_secs(60))
        .expect("the TOCTOU gate must refuse a FIFO — never wedge");
    let err = err.expect("a FIFO at a planned source must refuse");
    assert!(
        err.contains("no longer a regular file"),
        "the refusal names the shape: {err}"
    );
    // Nothing was written: the FIFO survives, no patch, no commit.
    assert!(fx.talker.symlink_metadata().unwrap().file_type().is_fifo());
    assert!(!fx.ws.join(PATCH_FILENAME).exists());
    let log = git(&fx.ws, &["log", "--oneline"]);
    assert!(!log.contains("ros2 migrate"), "no commit: {log}");
    assert_eq!(build_calls, 0);
}

/// PATCH_ROLLBACK: with a PRIOR patch snapshotted, a planted
/// SYMLINK at the patch path must not route into the prior-patch restore —
/// whose first operation is `remove_file`, unlinking the planted link.
/// An unverifiable patch path (ELOOP, any non-NotFound read error)
/// WITHHOLDS the restore: the link survives, and the withheld prior patch
/// is reported.
#[cfg(unix)]
#[test]
fn a_link_planted_at_the_patch_path_withholds_the_prior_patch_restore() {
    use std::os::unix::fs::PermissionsExt as _;
    let fx = make_workspace();
    const SENTINEL: &[u8] = b"--- an earlier migration's patch, user-owned\n";
    std::fs::write(fx.ws.join(PATCH_FILENAME), SENTINEL).unwrap();
    // The hook swaps the run's patch for a symlink and rejects the commit.
    std::fs::write(fx.ws.join("link-target.txt"), b"external target").unwrap();
    let hook = fx.ws.join(".git/hooks/pre-commit");
    std::fs::write(
        &hook,
        format!(
            "#!/bin/sh\n\
             rm -f {p}\n\
             ln -s link-target.txt {p}\n\
             exit 1\n",
            p = PATCH_FILENAME
        ),
    )
    .unwrap();
    std::fs::set_permissions(&hook, std::fs::Permissions::from_mode(0o755)).unwrap();

    let mut engine = engine_for(&fx);
    let err = drive(&fx, &mut engine, true, true, false, true, true)
        .unwrap_err()
        .to_string();
    assert!(err.contains("migration commit FAILED"), "got: {err}");
    // The planted link SURVIVES — never unlinked by the prior-patch
    // restore — and its target is byte-untouched.
    let meta = fx.ws.join(PATCH_FILENAME).symlink_metadata().unwrap();
    assert!(
        meta.file_type().is_symlink(),
        "the planted link must be preserved, never unlinked"
    );
    assert_eq!(
        std::fs::read(fx.ws.join("link-target.txt")).unwrap(),
        b"external target".to_vec()
    );
    // The withholding is REPORTED, never silent (the link is
    // captured by the atomic quarantine rename, classified foreign, and
    // returned in place — never read through, never unlinked).
    assert!(err.contains("not the migration's bytes"), "got: {err}");
    assert!(err.contains("NOT restored"), "got: {err}");
    // The sources still roll back.
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL
    );
}

/// The WRITE phase is all-or-nothing — a failure writing a LATER
/// planned file must restore the files already written (and the patch)
/// before returning, never leave a partial migration with no commit to
/// revert. Driven with a two-file plan whose second file (sorts after the
/// first) is read-only.
#[cfg(unix)]
#[test]
fn a_failing_source_write_rolls_back_files_already_written() {
    use std::os::unix::fs::PermissionsExt as _;
    let fx = make_workspace();
    // A second rewritten file in the SAME analysis (a header-like sibling).
    let extra = fx.ws.join("src/demo_pkg/src/zz_extra.cpp");
    std::fs::write(&extra, TALKER_ORIGINAL).unwrap();
    git_commit_all(&fx.ws, "add extra");
    let mut analysis: serde_json::Value = serde_json::from_str(&talker_analysis(&fx)).unwrap();
    let mut second = analysis["rewrites"][0].clone();
    second["file"] = serde_json::json!(extra.display().to_string());
    analysis["rewrites"].as_array_mut().unwrap().push(second);
    let mut by_tu = HashMap::new();
    by_tu.insert(fx.talker.clone(), analysis.to_string());
    let mut engine = FixtureEngine { by_tu };

    // Make the SECOND write fail (BTreeMap order: talker.cpp < zz_extra.cpp,
    // so talker is written first and must be rolled back).
    std::fs::set_permissions(&extra, std::fs::Permissions::from_mode(0o444)).unwrap();

    let err = drive(&fx, &mut engine, true, true, false, true, true)
        .unwrap_err()
        .to_string();
    assert!(err.contains("failed to write"), "got: {err}");
    assert!(err.contains("rolled back"), "got: {err}");
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL,
        "the already-written first file must be restored"
    );
    assert!(
        !fx.ws.join(PATCH_FILENAME).exists(),
        "this run's patch must not survive its own failed write phase"
    );
    // Restore permissions so the tempdir can be cleaned up.
    std::fs::set_permissions(&extra, std::fs::Permissions::from_mode(0o644)).unwrap();
    let porcelain = git(&fx.ws, &["status", "--porcelain"]);
    assert!(
        porcelain.lines().all(|l| l.starts_with("??")),
        "no tracked changes may remain: {porcelain}"
    );
}

/// A security shape: a
/// pre-existing patch PATH that is a symlink is refused up front — a VALID
/// link would make the patch write overwrite whatever it points at (a
/// contributor can plant `cerulion-ros2-migration.patch -> anywhere` in a
/// shared workspace), and a DANGLING link, which a following read misreads
/// as "absent", would be DELETED by rollback. Neither the link nor its
/// target may be touched. (Note: the SOURCE-side link handling is
/// preserve-and-report — see the swapped-source test below.)
#[cfg(unix)]
#[test]
fn a_patch_path_symlink_is_refused_and_never_followed_or_deleted() {
    // Arm 1: VALID symlink pointing at a file OUTSIDE the workspace.
    let fx = make_workspace();
    let outside_dir = tempfile::tempdir().expect("outside dir");
    let target = outside_dir.path().join("precious.txt");
    std::fs::write(&target, b"precious external bytes").unwrap();
    std::os::unix::fs::symlink(&target, fx.ws.join(PATCH_FILENAME)).unwrap();
    let mut engine = engine_for(&fx);
    let err = drive(&fx, &mut engine, true, true, false, true, true)
        .unwrap_err()
        .to_string();
    assert!(err.contains("SYMLINK"), "got: {err}");
    assert_eq!(
        std::fs::read(&target).unwrap(),
        b"precious external bytes".to_vec(),
        "the link target must be untouched"
    );
    assert!(
        fx.ws
            .join(PATCH_FILENAME)
            .symlink_metadata()
            .unwrap()
            .file_type()
            .is_symlink(),
        "the link itself must survive"
    );
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL
    );

    // Arm 2: DANGLING symlink — a following read calls this "absent" and
    // rollback's remove arm would delete the user's link; the refusal must
    // fire and the link must survive.
    let fx = make_workspace();
    std::os::unix::fs::symlink("/nonexistent/nowhere", fx.ws.join(PATCH_FILENAME)).unwrap();
    let mut engine = engine_for(&fx);
    let err = drive(&fx, &mut engine, true, true, false, true, true)
        .unwrap_err()
        .to_string();
    assert!(err.contains("SYMLINK"), "got: {err}");
    assert!(
        fx.ws
            .join(PATCH_FILENAME)
            .symlink_metadata()
            .unwrap()
            .file_type()
            .is_symlink(),
        "the dangling link must survive"
    );
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL
    );
}

/// The symlink class one layer over, refused
/// at the identity gate: a SOURCE file swapped for a symlink
/// AFTER the review-time byte check — the swap window is the consent
/// prompt, and the link's target mirrors the original bytes so the byte
/// re-check passes THROUGH it. The write seam would catch this too
/// (`O_NOFOLLOW` refuses, rollback preserves the link), but the identity
/// gate refuses BEFORE any write is attempted — the TOCTOU re-read
/// follows the link and fstats the TARGET, whose (dev, ino, mtime) cannot
/// match the reviewed file's. Strictly stronger: the link and its target
/// are untouched because nothing was ever opened for write. The write-seam
/// `O_NOFOLLOW` defense is still load-bearing — it owns a swap landing in
/// the gate→write gap, pinned at the `apply_plan_in_place` unit seam
/// (which drives the anchored walker with no TOCTOU gate in front).
#[cfg(unix)]
#[test]
fn a_source_swapped_for_a_symlink_after_consent_is_refused_and_preserved() {
    let fx = make_workspace();
    let outside_dir = tempfile::tempdir().expect("outside dir");
    let target = outside_dir.path().join("mirror.cpp");
    let mut engine = engine_for(&fx);
    let mut stdout: Vec<u8> = Vec::new();
    let talker = fx.talker.clone();
    let target_for_confirm = target.clone();
    let mut confirm = move |_prompt: &str| {
        // The swap: the target MIRRORS the original bytes, so the TOCTOU
        // byte re-check (which reads through the link) passes — only the
        // identity compare and the write seam can catch it.
        std::fs::write(&target_for_confirm, TALKER_ORIGINAL).unwrap();
        std::fs::remove_file(&talker).unwrap();
        std::os::unix::fs::symlink(&target_for_confirm, &talker).unwrap();
        Ok(true)
    };
    let mut build_runner = |_ws: &Path, _pkgs: &[String]| Ok(true);
    let err = run_migrate(
        &MigrateOptions {
            workspace: fx.ws.clone(),
            write: true,
            assume_yes: false,
        },
        &mut MigrateDeps {
            engine: &mut engine,
            is_tty: true,
            confirm: &mut confirm,
            build_runner: &mut build_runner,
            out: &mut stdout,
            interrupted: &|| false,
        },
    )
    .unwrap_err()
    .to_string();
    assert!(
        err.contains("changed identity since the migration was reviewed"),
        "the r28 identity gate must refuse before any write; got: {err}"
    );
    assert_eq!(
        std::fs::read_to_string(&target).unwrap(),
        TALKER_ORIGINAL,
        "the link's external target must be byte-identical"
    );
    // Ownership, upheld at the gate: the planted link is
    // another actor's entry — PRESERVED, never unlinked-and-recreated (and
    // with the pre-write refusal, never even opened for write).
    let meta = fx.talker.symlink_metadata().unwrap();
    assert!(
        meta.file_type().is_symlink(),
        "the planted link must be preserved, never destroyed"
    );
    assert!(!fx.ws.join(PATCH_FILENAME).exists());
}

/// The symlink class one MORE layer up, also refused
/// at the identity gate: an intermediate DIRECTORY of a planned source path
/// swapped for a symlink after the review-time byte check. The decoy file
/// byte-MIRRORS the original, so the byte re-check (which reads through
/// the redirected path) provably passes — behind the gate the anchored
/// per-component walker is the catch (a final-only `O_NOFOLLOW` open
/// would follow the swapped PARENT into the decoy tree); the
/// identity gate refuses BEFORE any write, because the redirected
/// re-read fstats the DECOY, whose (dev, ino, mtime) cannot match the
/// reviewed file's. The kill oracle stays the decoy's CONTENT plus its
/// MTIME (a read bumps neither) — with a bytes-only gate alone the
/// walker still refuses, but with both checks weakened together the decoy is
/// corrupted, which is what the mtime pin catches. The per-component
/// walker itself stays pinned at the `apply_plan_in_place` unit seam
/// (no TOCTOU gate in front — the gate→write gap's defense).
#[cfg(unix)]
#[test]
fn a_parent_directory_swapped_for_a_symlink_never_redirects_the_write() {
    let fx = make_workspace();
    let outside_dir = tempfile::tempdir().expect("outside dir");
    let decoy_dir = outside_dir.path().join("decoy_src");
    std::fs::create_dir_all(&decoy_dir).unwrap();
    let decoy_file = decoy_dir.join("talker.cpp");
    std::fs::write(&decoy_file, TALKER_ORIGINAL).unwrap();
    let decoy_mtime_before = std::fs::metadata(&decoy_file).unwrap().modified().unwrap();

    let real_dir = fx.ws.join("src/demo_pkg/src");
    let aside = fx.ws.join("src/demo_pkg/src_real_aside");
    let mut engine = engine_for(&fx);
    let mut stdout: Vec<u8> = Vec::new();
    let real_dir_for_confirm = real_dir.clone();
    let aside_for_confirm = aside.clone();
    let decoy_dir_for_confirm = decoy_dir.clone();
    let mut confirm = move |_prompt: &str| {
        // The swap, one level up: the PARENT directory of the planned file
        // becomes a symlink into the external decoy tree.
        std::fs::rename(&real_dir_for_confirm, &aside_for_confirm).unwrap();
        std::os::unix::fs::symlink(&decoy_dir_for_confirm, &real_dir_for_confirm).unwrap();
        Ok(true)
    };
    let mut build_runner = |_ws: &Path, _pkgs: &[String]| Ok(true);
    let err = run_migrate(
        &MigrateOptions {
            workspace: fx.ws.clone(),
            write: true,
            assume_yes: false,
        },
        &mut MigrateDeps {
            engine: &mut engine,
            is_tty: true,
            confirm: &mut confirm,
            build_runner: &mut build_runner,
            out: &mut stdout,
            interrupted: &|| false,
        },
    )
    .unwrap_err()
    .to_string();
    assert!(
        err.contains("changed identity since the migration was reviewed"),
        "the r28 identity gate must refuse before any write; got: {err}"
    );
    // THE kill oracle: the external decoy was never opened for write —
    // bytes AND mtime identical (a write through the swapped parent bumps
    // the mtime even if a later rollback rewrites the content; the gate's
    // re-read bumps neither).
    assert_eq!(
        std::fs::read_to_string(&decoy_file).unwrap(),
        TALKER_ORIGINAL,
        "the external decoy must be byte-identical"
    );
    assert_eq!(
        std::fs::metadata(&decoy_file).unwrap().modified().unwrap(),
        decoy_mtime_before,
        "the external decoy must never have been opened for write"
    );
    // The user's real file, moved aside by the attacker, is untouched.
    assert_eq!(
        std::fs::read_to_string(aside.join("talker.cpp")).unwrap(),
        TALKER_ORIGINAL
    );
    assert!(!fx.ws.join(PATCH_FILENAME).exists());
}

/// A workspace reached through SYMLINKED SPELLINGS — the
/// workspace path itself a link AND `src/` a link into the real tree (the
/// colcon-overlay layout) — must round-trip (report + apply, edits landing
/// in the REAL tree). The clang engine canonicalizes `--src-root` and
/// emits CANONICAL edit paths, so every Rust-side comparison must happen
/// in canonical space; the compile database here deliberately carries the
/// SPELLED path (the other direction of the same mismatch).
#[cfg(unix)]
#[test]
fn a_workspace_reached_through_symlinked_spellings_round_trips() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let root = tmp.path().canonicalize().unwrap();
    let ws = root.join("ws");
    let pkg = ws.join("actual_src/demo_pkg");
    std::fs::create_dir_all(pkg.join("src")).unwrap();
    std::fs::write(
        pkg.join("package.xml"),
        "<package format=\"3\"><name>demo_pkg</name></package>",
    )
    .unwrap();
    let talker = pkg.join("src/talker.cpp");
    std::fs::write(&talker, TALKER_ORIGINAL).unwrap();
    std::os::unix::fs::symlink(ws.join("actual_src"), ws.join("src")).unwrap();
    let linkws = root.join("linkws");
    std::os::unix::fs::symlink(&ws, &linkws).unwrap();
    let db_dir = ws.join("build/demo_pkg");
    std::fs::create_dir_all(&db_dir).unwrap();
    let spelled = ws.join("src/demo_pkg/src/talker.cpp");
    std::fs::write(
        db_dir.join("compile_commands.json"),
        format!(
            "[{{\"directory\": \"{}\", \"command\": \"clang++ -c {}\", \"file\": \"{}\"}}]",
            db_dir.display(),
            spelled.display(),
            spelled.display()
        ),
    )
    .unwrap();
    git(&ws, &["init", "-q"]);
    std::fs::write(ws.join(".gitignore"), "/build/\n").unwrap();
    git_commit_all(&ws, "baseline");

    // The engine sees the CANONICAL TU and emits CANONICAL edit paths.
    let make_engine = || {
        let mut by_tu = HashMap::new();
        by_tu.insert(talker.clone(), talker_analysis_for(&talker, &talker));
        FixtureEngine { by_tu }
    };
    let run = |engine: &mut FixtureEngine, write: bool| {
        let mut stdout: Vec<u8> = Vec::new();
        let mut affected: Vec<String> = Vec::new();
        let outcome = {
            let mut confirm = |_p: &str| Ok(true);
            let mut build_runner = |_w: &Path, pkgs: &[String]| {
                affected = pkgs.to_vec();
                Ok(true)
            };
            run_migrate(
                &MigrateOptions {
                    workspace: linkws.clone(),
                    write,
                    assume_yes: write,
                },
                &mut MigrateDeps {
                    engine,
                    is_tty: false,
                    confirm: &mut confirm,
                    build_runner: &mut build_runner,
                    out: &mut stdout,
                    interrupted: &|| false,
                },
            )
            .expect("a symlink-spelled workspace is legitimate")
        };
        (outcome, String::from_utf8(stdout).unwrap(), affected)
    };

    let (outcome, out, _) = run(&mut make_engine(), false);
    assert!(matches!(outcome, MigrateOutcome::Reported));
    assert!(
        out.contains("PROPOSED REWRITES (1 call site(s)"),
        "the rewrite must be found through the symlinked spellings:\n{out}"
    );

    let (outcome, _, affected) = run(&mut make_engine(), true);
    assert!(matches!(outcome, MigrateOutcome::Applied { .. }));
    assert_eq!(affected, vec!["demo_pkg".to_string()]);
    // The edit landed in the REAL tree — visible through both spellings.
    assert_eq!(std::fs::read_to_string(&talker).unwrap(), TALKER_MIGRATED);
    assert_eq!(std::fs::read_to_string(&spelled).unwrap(), TALKER_MIGRATED);
    let subject = git(&ws, &["log", "-1", "--format=%s"]);
    assert_eq!(
        subject, "ros2 migrate: rewrite 1 publish call site(s) to the loaned-message API",
        "subject: {subject}"
    );
}

/// A pre-commit hook (or any actor)
/// REPLACES `cerulion-ros2-migration.patch` and rejects the commit — a
/// rollback that `remove_file`s the patch path unconditionally deletes
/// the actor's file. The rollback verifies the bytes at the path are
/// the migration's own diff before any destructive act: a replacement is
/// preserved and reported.
#[cfg(unix)]
#[test]
fn a_patch_replaced_by_a_hook_survives_the_rollback() {
    use std::os::unix::fs::PermissionsExt as _;
    let fx = make_workspace();
    const FOREIGN: &str = "foreign patch content from a hook\n";
    let hook = fx.ws.join(".git/hooks/pre-commit");
    std::fs::write(
        &hook,
        format!(
            "#!/bin/sh\n\
             printf '{}' > {}\n\
             exit 1\n",
            "foreign patch content from a hook\\n", PATCH_FILENAME
        ),
    )
    .unwrap();
    std::fs::set_permissions(&hook, std::fs::Permissions::from_mode(0o755)).unwrap();

    let mut engine = engine_for(&fx);
    let err = drive(&fx, &mut engine, true, true, false, true, true)
        .unwrap_err()
        .to_string();
    assert!(err.contains("migration commit FAILED"), "got: {err}");
    // The replacement SURVIVES byte-identical — never deleted, never
    // overwritten by a prior-patch restore.
    assert_eq!(
        std::fs::read_to_string(fx.ws.join(PATCH_FILENAME)).unwrap(),
        FOREIGN,
        "the actor's patch replacement must be preserved"
    );
    // And the preservation is REPORTED, never silent.
    assert!(
        err.contains("replaced mid-run (not the migration's bytes)"),
        "got: {err}"
    );
    // The sources still roll back to the originals.
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL
    );
}

/// INTERRUPT_SAFETY: the write window polls the
/// injected `interrupted` seam at its safepoints — a trip rolls EVERYTHING
/// back and refuses, so a Ctrl-C (whose CLI handler flips the flag instead
/// of terminating) can never strand a partial uncommitted migration. Three
/// arms: a trip at the pre-write safepoint (nothing written), at the first
/// between-writes safepoint (patch written, no sources yet) and at the
/// pre-commit safepoint (sources written, then restored).
#[test]
fn an_interrupt_mid_write_rolls_back_everything_and_refuses() {
    for trip_at_call in [1u32, 2u32, 3u32] {
        let fx = make_workspace();
        let mut engine = engine_for(&fx);
        let mut stdout: Vec<u8> = Vec::new();
        let mut build_calls = 0u32;
        let mut confirm = |_prompt: &str| Ok(true);
        let mut build_runner = |_ws: &Path, _pkgs: &[String]| {
            build_calls += 1;
            Ok(true)
        };
        // Counted from the first poll INSIDE the write batch — the first that
        // sees the workspace lock FILE, which the acquire creates — so the
        // consent-boundary polls before the lock do not
        // shift which safepoint trips: 1 = pre-write, 2 = between writes,
        // 3 = pre-commit.
        let lock_file = fx.ws.join(LOCK_DIR).join(LOCK_FILE);
        assert!(
            !lock_file.exists(),
            "this fixture must start without the lock file — the in-batch count below is \
             exact only on an UNCONTENDED acquire that creates it"
        );
        let calls = std::cell::Cell::new(0u32);
        let interrupted = || {
            if !lock_file.exists() {
                return false;
            }
            calls.set(calls.get() + 1);
            calls.get() >= trip_at_call
        };
        let err = run_migrate(
            &MigrateOptions {
                workspace: fx.ws.clone(),
                write: true,
                assume_yes: true,
            },
            &mut MigrateDeps {
                engine: &mut engine,
                is_tty: false,
                confirm: &mut confirm,
                build_runner: &mut build_runner,
                out: &mut stdout,
                interrupted: &interrupted,
            },
        )
        .expect_err("an interrupted write must refuse")
        .to_string();
        // The WINDOW, not just the word: the consent boundary refuses with
        // "interrupted" too, and a trip that never reached a write safepoint
        // would leave this arm passing for the wrong reason. Trip 1 is the
        // pre-write safepoint and is the one POSITIVE pin of its string —
        // every consent-boundary arm asserts that string's absence, and an
        // absence guard alone disarms silently on a reword.
        // (At trip 1 the shared rollback oracles below hold trivially — nothing
        // was written — so trip 1's whole content is this string.)
        let expected = if trip_at_call == 1 {
            PRE_WRITE_INTERRUPT_REFUSAL
        } else {
            "interrupted — every write was rolled back"
        };
        assert!(
            err.contains(expected),
            "trip {trip_at_call} must refuse from its own WRITE-WINDOW safepoint; got: {err}"
        );
        // Everything rolled back: sources original, patch gone, no commit,
        // no build.
        assert_eq!(
            std::fs::read_to_string(&fx.talker).unwrap(),
            TALKER_ORIGINAL,
            "trip_at_call={trip_at_call}"
        );
        assert!(
            !fx.ws.join(PATCH_FILENAME).exists(),
            "trip_at_call={trip_at_call}: the migration's own patch is removed"
        );
        let log = git(&fx.ws, &["log", "--oneline"]);
        assert!(
            !log.contains("ros2 migrate"),
            "trip_at_call={trip_at_call}: no migration commit: {log}"
        );
        assert_eq!(build_calls, 0, "trip_at_call={trip_at_call}");
    }
}

/// The migration REFUSES to commit when the
/// index carries changes it did not stage.
///
/// The clean-tree gate runs ONCE, early — before the report is rendered and
/// before the consent prompt, which is an unbounded blocking read — while a
/// bare `git commit -m` commits the whole INDEX as of commit time. So
/// anything another actor staged while the operator read the diff (a second
/// terminal, an IDE, a `pre-commit` hook re-staging with `git add -A`) would be
/// swept into the migration commit, and the undo this verb documents,
/// `git revert <sha>`, would then revert that work too.
///
/// The racer is driven at the exact documented window — inside the consent
/// callback — so this is the real sequence, not an approximation of it.
///
/// Why a refusal and not a pathspec: `git commit -- <paths>` commits from a
/// TEMPORARY index, which on the failure path discards the entry a
/// `pre-commit` hook staged — precisely what the sibling
/// `a_hook_staged_edit_on_a_planned_path_survives_the_rollback` promises to
/// preserve, and that test fails under a pathspec commit. Refusing leaves git's commit
/// semantics untouched and matches what this verb already does with a dirty
/// tree at entry.
#[test]
fn a_foreign_staged_change_refuses_the_migration_commit() {
    let fx = make_workspace();
    let unrelated = fx.ws.join("unrelated.txt");
    let head_before = git(&fx.ws, &["rev-parse", "HEAD"]);
    let mut engine = engine_for(&fx);

    let mut stdout: Vec<u8> = Vec::new();
    let mut build_calls: Vec<(PathBuf, Vec<String>)> = Vec::new();
    // THE WINDOW: a third party stages unrelated work while the operator is
    // reading the diff. The tree was clean when the gate ran, so nothing
    // earlier can refuse this.
    let ws_for_confirm = fx.ws.clone();
    let unrelated_for_confirm = unrelated.clone();
    let mut confirm = |_prompt: &str| {
        std::fs::write(&unrelated_for_confirm, b"someone else's work\n").unwrap();
        git(&ws_for_confirm, &["add", "--", "unrelated.txt"]);
        Ok(true)
    };
    let mut build_runner = |ws: &Path, pkgs: &[String]| {
        build_calls.push((ws.to_path_buf(), pkgs.to_vec()));
        Ok(true)
    };
    let err = run_migrate(
        &MigrateOptions {
            workspace: fx.ws.clone(),
            write: true,
            assume_yes: false,
        },
        &mut MigrateDeps {
            engine: &mut engine,
            is_tty: true,
            confirm: &mut confirm,
            build_runner: &mut build_runner,
            out: &mut stdout,
            interrupted: &|| false,
        },
    )
    .expect_err("a foreign staged change must refuse the commit");

    // The refusal NAMES the offender and says how to proceed.
    let err = err.to_string();
    assert!(
        err.contains("unrelated.txt"),
        "the refusal must name the foreign staged path: {err}"
    );
    assert!(
        err.contains("did not stage") && err.contains("re-run"),
        "the refusal must say what happened and what to do: {err}"
    );

    // NOTHING was committed — so `git revert` can never take the third
    // party's work with it.
    assert_eq!(
        git(&fx.ws, &["rev-parse", "HEAD"]),
        head_before,
        "no commit may be created when the index is not ours alone"
    );
    // The third party's work is untouched: still staged, bytes intact.
    let staged = git(&fx.ws, &["diff", "--cached", "--name-only"]);
    assert!(
        staged.lines().any(|l| l == "unrelated.txt"),
        "the third party's staged entry must survive; index: {staged:?}"
    );
    assert_eq!(
        std::fs::read_to_string(&unrelated).unwrap(),
        "someone else's work\n"
    );
    // And the build never ran — the run stopped at the commit.
    assert!(build_calls.is_empty(), "no build on a refused commit");
}

/// A SUCCESSFUL pre-commit hook stages INSIDE `git commit` — after this verb's
/// own index check and before the commit object is written — so its additions
/// land in the migration commit.
///
/// The hook is the user's own configuration, so the verb commits and warns. The
/// commit STANDS; what is pinned here is that the operator is TOLD, by name,
/// and told the undo that is actually safe. `git revert <sha>` — the undo this
/// verb advertises — would take the hook's paths with it, so the warning must
/// name `git apply -R <patch>`, which reverses only the migration's edits.
///
/// Mutation kill: deleting the post-commit read-back leaves the run silent and
/// fails the naming assertions.
#[cfg(unix)]
#[test]
fn a_hook_staged_path_in_the_commit_is_reported_with_the_safe_undo() {
    use std::os::unix::fs::PermissionsExt as _;
    let fx = make_workspace();
    let hook = fx.ws.join(".git/hooks/pre-commit");
    std::fs::write(
        &hook,
        "#!/bin/sh\n\
         echo hook-added > hook-unrelated.txt\n\
         git add -- hook-unrelated.txt\n\
         exit 0\n",
    )
    .unwrap();
    std::fs::set_permissions(&hook, std::fs::Permissions::from_mode(0o755)).unwrap();
    let head_before = git(&fx.ws, &["rev-parse", "HEAD"]);

    let mut engine = engine_for(&fx);
    let r = drive(&fx, &mut engine, true, true, false, true, true).expect("applies");
    let MigrateOutcome::Applied { .. } = r.outcome else {
        panic!("expected Applied, got {:?}", r.outcome);
    };
    let out = String::from_utf8(r.stdout).unwrap();

    // The commit stands.
    assert_ne!(
        git(&fx.ws, &["rev-parse", "HEAD"]),
        head_before,
        "the migration commit must stand — hooks are the user's problem"
    );
    // The condition is real, not hypothetical: the hook's file IS in it.
    let names = git(&fx.ws, &["show", "--name-only", "--format=", "HEAD"]);
    assert!(
        names.lines().any(|l| l.trim() == "hook-unrelated.txt"),
        "the hook's staging must actually reach the commit for this arm to \
         mean anything; commit carried: {names:?}"
    );

    // THE PIN: named, and pointed at the undo that is actually safe.
    assert!(
        out.contains("hook-unrelated.txt"),
        "the unplanned path must be named: {out}"
    );
    assert!(
        out.contains("did not plan"),
        "the warning must say what happened: {out}"
    );
    assert!(
        out.contains("git apply -R") && out.contains(PATCH_FILENAME),
        "the warning must name the SAFE undo (the patch), not leave the \
         operator with `git revert`: {out}"
    );
    assert!(
        out.contains("`git revert <sha>` is NOT equivalent"),
        "the warning must say plainly that the advertised undo is not safe \
         here: {out}"
    );

    // ANTI-TAUTOLOGY: a run with no hook says none of this.
    std::fs::remove_file(&hook).unwrap();
    let fx2 = make_workspace();
    let mut engine2 = engine_for(&fx2);
    let r2 = drive(&fx2, &mut engine2, true, true, false, true, true).expect("applies");
    let out2 = String::from_utf8(r2.stdout).unwrap();
    assert!(
        !out2.contains("did not plan"),
        "a clean run must not warn about unplanned paths: {out2}"
    );
}

/// A hook that BOTH edits a planned source AND stages an unplanned path breaks
/// the undo the warning recommends: the patch was generated before the hook
/// touched that source, so `git apply -R` no longer applies to it, while
/// `git revert` would delete the hook's own file. A warning that names a
/// recovery which fails in exactly the state it warns about is worse than no
/// warning, so the recommendation is VALIDATED before it is made.
///
/// Mutation kill: dropping the `git apply -R --check` validation makes the run
/// recommend the patch again and fails the "must not recommend" assertion.
#[cfg(unix)]
#[test]
fn a_hook_that_edits_a_planned_source_is_not_told_to_reverse_the_patch() {
    use std::os::unix::fs::PermissionsExt as _;
    let fx = make_workspace();
    let hook = fx.ws.join(".git/hooks/pre-commit");
    std::fs::write(
        &hook,
        // The hook edits the very lines the migration just wrote (an
        // auto-formatter touching the rewritten region is the realistic
        // shape), which is what makes the pre-generated patch stop
        // reversing — an append elsewhere in the file does not.
        "#!/bin/sh\n\
         perl -pi -e 's/borrow_loaned_message\\(\\);/borrow_loaned_message();  \\/\\/ hook/' \
         src/demo_pkg/src/talker.cpp\n\
         git add -- src/demo_pkg/src/talker.cpp\n\
         echo hook-added > hook-unrelated.txt\n\
         git add -- hook-unrelated.txt\n\
         exit 0\n",
    )
    .unwrap();
    std::fs::set_permissions(&hook, std::fs::Permissions::from_mode(0o755)).unwrap();

    let mut engine = engine_for(&fx);
    let r = drive(&fx, &mut engine, true, true, false, true, true).expect("applies");
    let MigrateOutcome::Applied { .. } = r.outcome else {
        panic!("expected Applied, got {:?}", r.outcome);
    };
    let out = String::from_utf8(r.stdout).unwrap();

    // The condition is real: the patch genuinely no longer reverses.
    let check = Command::new("git")
        .arg("-C")
        .arg(&fx.ws)
        .args(["apply", "-R", "--check"])
        .arg(fx.ws.join(PATCH_FILENAME))
        .output()
        .expect("git runs");
    assert!(
        !check.status.success(),
        "this arm needs the patch to be un-reversible for it to mean anything"
    );

    // THE PIN: the run must NOT hand the operator that command.
    assert!(
        !out.contains("SAFE UNDO"),
        "the patch must not be recommended when it no longer applies: {out}"
    );
    assert!(
        out.contains("no single-command undo"),
        "the operator must be told plainly that neither undo is safe: {out}"
    );
    assert!(
        out.contains("git show"),
        "the operator must be pointed at the commit to split by hand: {out}"
    );
    assert!(
        out.contains("hook-unrelated.txt"),
        "the unplanned path is still named: {out}"
    );

    // CONTROL: a hook that only ADDS a path leaves the patch reversible, and
    // there the recommendation is still made.
    let fx2 = make_workspace();
    let hook2 = fx2.ws.join(".git/hooks/pre-commit");
    std::fs::write(
        &hook2,
        "#!/bin/sh\n\
         echo hook-added > hook-unrelated.txt\n\
         git add -- hook-unrelated.txt\n\
         exit 0\n",
    )
    .unwrap();
    std::fs::set_permissions(&hook2, std::fs::Permissions::from_mode(0o755)).unwrap();
    let mut engine2 = engine_for(&fx2);
    let r2 = drive(&fx2, &mut engine2, true, true, false, true, true).expect("applies");
    let out2 = String::from_utf8(r2.stdout).unwrap();
    assert!(
        out2.contains("SAFE UNDO") && out2.contains(PATCH_FILENAME),
        "a reversible patch must still be recommended: {out2}"
    );
}

/// A planned file whose name git DISPLAY-QUOTES
/// must not be mistaken for a hook's addition.
///
/// `git show --name-only` without `-z` honours `core.quotePath`, git's
/// default, so `café.cpp` comes back as `"src/demo_pkg/src/caf\303\251.cpp"`
/// while `plan.files[].rel` holds the raw path. Comparing the two calls the
/// operator's own migrated file "unplanned", fires the pre-commit-hook
/// warning with NO hook installed, and tells them `git revert` is unsafe when
/// it is not.
///
/// Mutation kill: dropping `-z` from the read-back fires the warning here.
#[cfg(unix)]
#[test]
fn a_display_quoted_planned_path_is_not_mistaken_for_a_hook_addition() {
    let mut fx = make_workspace();
    // Rename the planned source to a name git quotes, and keep the workspace
    // coherent: the compile database names the TU, and the tree must be clean
    // for `--write` to proceed.
    let accented = fx.ws.join("src/demo_pkg/src/café.cpp");
    git(
        &fx.ws,
        &[
            "mv",
            "src/demo_pkg/src/talker.cpp",
            "src/demo_pkg/src/café.cpp",
        ],
    );
    let db_dir = fx.ws.join("build/demo_pkg");
    std::fs::write(
        db_dir.join("compile_commands.json"),
        format!(
            "[{{\"directory\": \"{}\", \"command\": \"clang++ -c {}\", \"file\": \"{}\"}}]",
            db_dir.display(),
            accented.display(),
            accented.display()
        ),
    )
    .unwrap();
    // Through `git_commit_all`, which injects the test identity: a bare
    // `git commit` exits 128 ("Author identity unknown") on a host or
    // container with no configured git identity, failing this arm for a
    // reason that has nothing to do with what it pins — the shape the
    // bare-identity suite exists to catch.
    git_commit_all(&fx.ws, "rename to a quoted path");
    fx.talker = accented.clone();

    // The premise: git really does quote this path in the display form, so
    // the arm cannot pass because the name happened to be plain ASCII.
    let shown = git(&fx.ws, &["show", "--name-only", "--format=", "HEAD"]);
    assert!(
        shown.contains('\\') || shown.contains('"'),
        "this arm needs git to DISPLAY-QUOTE the path; got {shown:?}"
    );

    let mut engine = engine_for(&fx);
    let r = drive(&fx, &mut engine, true, true, false, true, true).expect("applies");
    let MigrateOutcome::Applied { .. } = r.outcome else {
        panic!("expected Applied, got {:?}", r.outcome);
    };
    let out = String::from_utf8(r.stdout).unwrap();

    // THE PIN: no hook is installed, so nothing may claim one staged anything.
    assert!(
        !out.contains("did not plan"),
        "a planned file with a quoted name must not be reported as unplanned: {out}"
    );
    assert!(
        !out.contains("NOT equivalent"),
        "the operator must not be told `git revert` is unsafe when no hook ran: {out}"
    );
}

/// A pre-commit hook that REWRITES a planned file, re-stages it, and rejects
/// the commit (the mainstream lint-staged/formatter pattern) must not have its
/// work erased by the rollback. Ownership rule: the rollback resets only an index entry
/// holding the MIGRATION'S OWN staged bytes and restores only worktree
/// content that is provably the migration's write; anything else on a
/// planned path is preserved and reported.
#[cfg(unix)]
#[test]
fn a_hook_staged_edit_on_a_planned_path_survives_the_rollback() {
    use std::os::unix::fs::PermissionsExt as _;
    let fx = make_workspace();
    const HOOK_EDIT: &str = "// formatted by the pre-commit hook\n";
    let hook = fx.ws.join(".git/hooks/pre-commit");
    std::fs::write(
        &hook,
        "#!/bin/sh\n\
         echo '// formatted by the pre-commit hook' > src/demo_pkg/src/talker.cpp\n\
         git add src/demo_pkg/src/talker.cpp\n\
         exit 1\n",
    )
    .unwrap();
    std::fs::set_permissions(&hook, std::fs::Permissions::from_mode(0o755)).unwrap();

    let mut engine = engine_for(&fx);
    let err = drive(&fx, &mut engine, true, true, false, true, true)
        .unwrap_err()
        .to_string();
    assert!(err.contains("migration commit FAILED"), "got: {err}");
    // The hook's WORKTREE edit survives — not clobbered to the snapshot,
    // not left as the migration's bytes.
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        HOOK_EDIT,
        "the hook's worktree edit must be preserved"
    );
    // The hook's STAGED entry survives byte-identical.
    let staged = git(&fx.ws, &["show", ":src/demo_pkg/src/talker.cpp"]);
    assert_eq!(
        staged,
        HOOK_EDIT.trim_end(),
        "the hook's staged blob must be preserved"
    );
    let cached = git(&fx.ws, &["diff", "--cached", "--name-only"]);
    assert!(
        cached.lines().any(|l| l == "src/demo_pkg/src/talker.cpp"),
        "the staged marker must survive: {cached}"
    );
    // The preservation is REPORTED, never silent.
    assert!(err.contains("left as-is"), "got: {err}");
    // The migration's own artifacts still roll back.
    assert!(!fx.ws.join(PATCH_FILENAME).exists());
}

// ---------------------------------------------------------------------------
// The workspace lock: `--write` is a workspace WRITER and must
// serialize against every other one, over exactly its write batch, with a wait
// the user's Ctrl-C can end and without the tracked-file side effect
// `WorkspaceLock::acquire_and_track_gitignore` carries.
// ---------------------------------------------------------------------------

/// Take the INTERRUPTIBLE lock, asserting it was actually taken.
#[cfg(unix)]
fn acquire_quiet(root: &Path) -> WorkspaceLock {
    WorkspaceLock::acquire_interruptibly(root, &|| false)
        .expect("an uninterrupted acquire always yields the lock")
}

/// Open `<ws>/.cerulion/workspace.lock` as a FOREIGN open file description.
///
/// `flock(2)` locks belong to a description, so this observes the lock exactly
/// as another PROCESS would. It is the only probe that works here: a second
/// `WorkspaceLock` on the test's own thread is REENTRANT and returns `Ok`
/// whether or not the lock is held, so a test built on one proves nothing
/// about the lock — and it needs no timeout, so it adds no wall.
#[cfg(unix)]
fn foreign_description(ws: &Path) -> std::fs::File {
    std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .open(ws.join(".cerulion/workspace.lock"))
        .expect("the lock file exists")
}

/// Whether a foreign description can take the exclusive lock right now. The
/// probe releases immediately, so it never changes what is held.
#[cfg(unix)]
fn lock_is_free(f: &std::fs::File) -> bool {
    use std::os::fd::AsRawFd;
    // SAFETY: `f` is a live open file owned by the caller across this call.
    if unsafe { libc::flock(f.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) } == 0 {
        // SAFETY: as above.
        unsafe { libc::flock(f.as_raw_fd(), libc::LOCK_UN) };
        true
    } else {
        false
    }
}

/// A `Write` that samples the workspace lock on every chunk the migration
/// prints, so the span oracle is not limited to the three interrupt safepoints.
/// It is armed by the first safepoint (which is inside the batch), so the
/// pre-consent report is never sampled.
#[cfg(unix)]
struct LockProbingOut<'a> {
    probe: &'a std::fs::File,
    armed: &'a std::cell::Cell<bool>,
    seen: &'a std::cell::RefCell<Vec<(String, bool)>>,
}

#[cfg(unix)]
impl std::io::Write for LockProbingOut<'_> {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        if self.armed.get() {
            let held = !lock_is_free(self.probe);
            self.seen
                .borrow_mut()
                .push((String::from_utf8_lossy(buf).trim_end().to_string(), held));
        }
        Ok(buf.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

/// Turn the fixture into the shape a workspace has once `cerulion` has mutated
/// it once: `.cerulion/` exists and is IGNORED, committed clean. That is what
/// makes the during-the-block oracle exact — `git status` is empty, so ANY
/// interleaved write shows up.
#[cfg(unix)]
fn adopt_the_lock_directory(fx: &Fixture) {
    drop(
        WorkspaceLock::acquire_and_track_gitignore(&fx.ws).expect("acquire scaffolds the lock dir"),
    );
    assert_eq!(
        std::fs::read_to_string(fx.ws.join(".gitignore")).unwrap(),
        "/build/\n.cerulion/\n",
        "the tracking constructor tops up an existing .gitignore (unchanged behaviour)"
    );
    git_commit_all(&fx.ws, "adopt the cerulion lock directory");
    assert!(
        git(&fx.ws, &["status", "--porcelain", "--untracked-files=all"]).is_empty(),
        "the fixture must start from an empty git status"
    );
}

/// What a `--write` run needs beyond the fixture engine: a consent seam that
/// announces itself, an interrupt predicate, and a build runner that can probe.
#[cfg(unix)]
struct WriteRun<'a> {
    at_consent: mpsc::Sender<()>,
    interrupted: &'a dyn Fn() -> bool,
    build_runner: &'a mut dyn FnMut(&Path, &[String]) -> CliResult<bool>,
    assume_yes: bool,
}

/// Drive `run_migrate` in `--write` mode with the seams `WriteRun` supplies.
#[cfg(unix)]
fn drive_write(fx: &Fixture, engine: &mut FixtureEngine, run: WriteRun<'_>) -> CliResult<()> {
    let mut stdout: Vec<u8> = Vec::new();
    let at_consent = run.at_consent;
    let mut confirm = |_prompt: &str| {
        let _ = at_consent.send(());
        Ok(true)
    };
    run_migrate(
        &MigrateOptions {
            workspace: fx.ws.clone(),
            write: true,
            assume_yes: run.assume_yes,
        },
        &mut MigrateDeps {
            engine,
            is_tty: !run.assume_yes,
            confirm: &mut confirm,
            build_runner: run.build_runner,
            out: &mut stdout,
            interrupted: run.interrupted,
        },
    )
    .map(|_| ())
}

/// THE headline pin: while another Cerulion writer holds the
/// workspace lock, a `--write` migration BLOCKS instead of interleaving its
/// writes — and the oracle for "did not interleave" is the whole tree, not a
/// spot check: `git status` stays empty and HEAD is unmoved for as long as the
/// other writer holds it.
///
/// It also carries the only proof that the lock is taken AFTER consent, and it
/// is implicit, so name it: `at_consent` fires WHILE the other writer still
/// holds the lock. If the acquire had been placed before the prompt, the
/// consent seam could not have run at all until the holder released.
///
/// Two oracles carry it, one wall-free. The contender releases only after
/// the main thread has observed the tree; the SECOND contender then acquires
/// and samples HEAD — it can only get the lock once the write batch has ended,
/// so the HEAD it sees must be the migration's NEW commit. An acquire that was
/// released early (or never taken across the batch) would let it sample the
/// old one, which no wall-clock assertion here can see.
#[cfg(unix)]
#[test]
fn write_blocks_while_another_writer_holds_the_workspace_lock() {
    let fx = make_workspace();
    adopt_the_lock_directory(&fx);
    let head_before = git(&fx.ws, &["rev-parse", "HEAD"]);
    let gitignore_before = std::fs::read(fx.ws.join(".gitignore")).unwrap();

    let mut engine = engine_for(&fx);
    let (holding_tx, holding_rx) = mpsc::channel();
    let (release_tx, release_rx) = mpsc::channel::<()>();
    let (at_consent_tx, at_consent_rx) = mpsc::channel();
    let (done_tx, done_rx) = mpsc::channel();
    let (sampled_tx, sampled_rx) = mpsc::channel();
    let (in_batch_tx, in_batch_rx) = mpsc::channel();

    std::thread::scope(|scope| {
        // OWNED by this closure, not borrowed from the enclosing frame: a
        // panicking assertion below must drop these senders during unwind, or
        // a waiting thread outlives the scope and the test HANGS instead of
        // failing. The holder's wait is bounded for the same reason.
        let release_tx = release_tx;
        let done_tx = done_tx;
        let ws = fx.ws.clone();
        scope.spawn(move || {
            let guard = WorkspaceLock::acquire(&ws).expect("the other writer takes the lock");
            holding_tx.send(()).unwrap();
            release_rx
                .recv_timeout(Duration::from_secs(120))
                .expect("the main thread never released the holder — its premise was violated");
            drop(guard);
        });
        holding_rx.recv().unwrap();

        let done_tx_worker = done_tx.clone();
        let fx_ref = &fx;
        let engine_ref = &mut engine;
        scope.spawn(move || {
            // ARRIVAL ORACLE. Signalling on any predicate call that
            // sees the lock held is NOT sound: the registry park
            // consults the predicate, so during the WAIT the
            // predicate fires with the lock held — by the OTHER writer —
            // and such a rule would spawn the contender before the batch
            // started and race it.
            //
            // The oracle keys on WORK DONE, not on lock state: the source
            // file differs from its original only after the migration has
            // rewritten it, which cannot happen before the lock is taken. The
            // first call observing a changed file is therefore inside the
            // write batch on EITHER contention path, and it is reached BEFORE
            // the commit — which is what keeps the HEAD oracle below
            // discriminating, since a lock released early lets the contender
            // in while HEAD is still the old one.
            //
            // It cannot pass vacuously: if the migration never writes, the
            // signal never arrives and the `recv_timeout` below fails naming
            // exactly that, instead of the test quietly proceeding.
            let talker = fx_ref.talker.clone();
            let in_batch = move || {
                if std::fs::read_to_string(&talker).is_ok_and(|now| now != TALKER_ORIGINAL) {
                    let _ = in_batch_tx.send(());
                }
                false
            };
            let mut build_runner = |_ws: &Path, _pkgs: &[String]| Ok(true);
            let outcome = drive_write(
                fx_ref,
                engine_ref,
                WriteRun {
                    at_consent: at_consent_tx,
                    interrupted: &in_batch,
                    build_runner: &mut build_runner,
                    assume_yes: false,
                },
            );
            done_tx_worker.send(outcome).unwrap();
        });
        // The scope frame's own handle goes, so a panicking worker DISCONNECTS
        // the channel instead of leaving the recv below to run its full budget.
        drop(done_tx);

        // Consent is given: the migration is one poll and one statement away
        // from the lock, and everything it would write is still unwritten.
        at_consent_rx
            .recv_timeout(Duration::from_secs(60))
            .expect("the migration reached its consent prompt");
        assert!(
            matches!(
                done_rx.recv_timeout(Duration::from_secs(2)),
                Err(mpsc::RecvTimeoutError::Timeout)
            ),
            "the migration finished (or died) while another writer held the workspace lock"
        );
        // The no-interleaving oracle, over the WHOLE tree.
        assert_eq!(
            git(&fx.ws, &["status", "--porcelain", "--untracked-files=all"]),
            "",
            "the blocked migration wrote into the workspace anyway"
        );
        assert_eq!(
            std::fs::read_to_string(&fx.talker).unwrap(),
            TALKER_ORIGINAL,
            "the blocked migration rewrote the source anyway"
        );
        assert_eq!(
            git(&fx.ws, &["rev-parse", "HEAD"]),
            head_before,
            "the blocked migration committed anyway"
        );

        release_tx.send(()).unwrap();

        // The timing-free half. The contender is spawned only once the
        // migration is provably INSIDE its write batch (its first interrupt
        // safepoint has run), so it cannot win the wake race against the
        // migration when the holder releases — it must queue behind the
        // migration and can only acquire once the batch has ended. It reports
        // the HEAD it sees when it gets in: that must be the NEW commit. An
        // acquire released early, or never held across the batch, lets it
        // sample the old one, which no wall-clock assertion here can see.
        in_batch_rx
            .recv_timeout(Duration::from_secs(120))
            .expect("the migration never rewrote the source — the arrival oracle never fired");
        let ws = fx.ws.clone();
        scope.spawn(move || {
            let guard = WorkspaceLock::acquire(&ws).expect("the contender acquires");
            sampled_tx
                .send(
                    Command::new("git")
                        .arg("-C")
                        .arg(&ws)
                        .args(["rev-parse", "HEAD"])
                        .output()
                        .map(|o| String::from_utf8_lossy(&o.stdout).trim_end().to_string())
                        .unwrap_or_default(),
                )
                .unwrap();
            drop(guard);
        });

        done_rx
            .recv_timeout(Duration::from_secs(120))
            .expect("the migration never completed after the lock was released")
            .expect("the migration applies once it holds the lock");
    });

    // And the whole migration really happened, against the hand oracle.
    let head_after = git(&fx.ws, &["rev-parse", "HEAD"]);
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_MIGRATED
    );
    assert_ne!(head_after, head_before);
    let sampled = sampled_rx
        .recv_timeout(Duration::from_secs(120))
        .expect("the contender never acquired");
    assert_eq!(
        sampled, head_after,
        "a writer that took the lock saw a tree mid-migration — the batch did not hold it \
         throughout (it sampled {sampled}, the pre-migration HEAD was {head_before})"
    );
    assert_eq!(
        std::fs::read(fx.ws.join(".gitignore")).unwrap(),
        gitignore_before,
        "--write takes the lock variant that never writes a tracked file"
    );
    assert_eq!(
        git(&fx.ws, &["status", "--porcelain", "--untracked-files=all"]),
        format!("?? {PATCH_FILENAME}")
    );
}

/// A SHARED reader is not good enough: taking `acquire_read` at the call site
/// would let two `--write` runs into the batch at once while every other arm in
/// this file stayed green. The holder here takes
/// only `LOCK_SH`; the migration must still wait for it.
///
/// Runs WITH the prompt (`assume_yes: false`): the arrival proof anchors on
/// the consent signal, which only the prompt path emits.
/// The lock FILE cannot anchor it — the fixture adopts it up
/// front — and a raw poll count would fire in the analysis phase. The
/// `--yes` path into a CONTENDED wait is therefore not driven by this
/// arm; the acquire it reaches is the same code, one poll earlier.
#[cfg(unix)]
#[test]
fn a_shared_reader_still_blocks_the_write_batch() {
    let fx = make_workspace();
    adopt_the_lock_directory(&fx);
    let head_before = git(&fx.ws, &["rev-parse", "HEAD"]);

    let mut engine = engine_for(&fx);
    // The wait loop consults the predicate exactly once per poll, so the call
    // count is the anti-spin oracle: a `CONTENTION_POLL` of zero busy-loops
    // into the millions. It is bounded as a RATE against the measured hold, not
    // as a bare count — the hold LENGTHENS under load (it spans a 2 s window,
    // a file read and two `git` subprocesses), so a fixed ceiling would fail
    // on exactly the loaded runner it is meant to survive.
    let polls = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let (in_wait_tx, in_wait_rx) = mpsc::channel();
    // Measured inside the scope, read after it: the hold is what the poll
    // ceiling below is a rate against.
    let mut held_for = Duration::ZERO;
    let (release_tx, release_rx) = mpsc::channel::<()>();
    let (holding_tx, holding_rx) = mpsc::channel();
    let (done_tx, done_rx) = mpsc::channel();
    let (at_consent_tx, at_consent_rx) = mpsc::channel();

    std::thread::scope(|scope| {
        let release_tx = release_tx;
        let done_tx = done_tx;
        let ws = fx.ws.clone();
        scope.spawn(move || {
            let guard = WorkspaceLock::acquire_read(&ws).expect("a shared read lock");
            assert!(guard.is_locked(), "the fixture's lock file must exist");
            holding_tx.send(()).unwrap();
            release_rx
                .recv_timeout(Duration::from_secs(120))
                .expect("the main thread never released the reader");
            drop(guard);
        });
        holding_rx.recv().unwrap();

        let done_tx_worker = done_tx.clone();
        let fx_ref = &fx;
        let engine_ref = &mut engine;
        let polls_worker = polls.clone();
        scope.spawn(move || {
            // ARRIVAL PROOF, anchored on CONSENT. The consent boundary polls
            // this predicate BEFORE the lock, so "any
            // poll" is not proof of the wait: after the yes the post-answer
            // poll comes first, and only the poll AFTER it can be the wait
            // loop — the reader holds LOCK_SH, so the acquire cannot succeed
            // and every write safepoint is unreachable. Without the proof the
            // 2 s window below is satisfied by a migration still in its
            // analysis phase, and the whole arm passes for the wrong reason.
            let consented = std::cell::Cell::new(false);
            let after_consent = std::cell::Cell::new(0usize);
            let count = move || {
                polls_worker.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
                if !consented.get() && at_consent_rx.try_recv().is_ok() {
                    consented.set(true);
                }
                if consented.get() {
                    after_consent.set(after_consent.get() + 1);
                    // ORDINAL, deliberately and narrowly: after consent the
                    // post-answer poll comes first, the wait loop second. A
                    // fourth pre-lock poll added later would fire this early;
                    // `>= 2` keeps the send alive on every later poll so the
                    // proof degrades to "reached the wait eventually" rather
                    // than to nothing. This is the ANTI-VACUITY half — the
                    // kill of a lock that ignores a shared reader is the
                    // 2 s no-progress window below, which this signal only
                    // starts.
                    if after_consent.get() >= 2 {
                        let _ = in_wait_tx.send(());
                    }
                }
                false
            };
            let mut build_runner = |_ws: &Path, _pkgs: &[String]| Ok(true);
            let outcome = drive_write(
                fx_ref,
                engine_ref,
                WriteRun {
                    at_consent: at_consent_tx,
                    interrupted: &count,
                    build_runner: &mut build_runner,
                    assume_yes: false,
                },
            );
            done_tx_worker.send(outcome).unwrap();
        });
        drop(done_tx);

        in_wait_rx
            .recv_timeout(Duration::from_secs(60))
            .expect("the migration never reached the lock wait");
        let held_from = std::time::Instant::now();
        assert!(
            matches!(
                done_rx.recv_timeout(Duration::from_secs(2)),
                Err(mpsc::RecvTimeoutError::Timeout)
            ),
            "the migration finished (or died) while a SHARED reader held the lock"
        );
        assert_eq!(
            std::fs::read_to_string(&fx.talker).unwrap(),
            TALKER_ORIGINAL,
            "a shared reader did not keep the write batch out"
        );
        assert_eq!(git(&fx.ws, &["rev-parse", "HEAD"]), head_before);

        release_tx.send(()).unwrap();
        held_for = held_from.elapsed();
        done_rx
            .recv_timeout(Duration::from_secs(120))
            .expect("the migration never completed after the reader released")
            .expect("the migration applies once the reader is gone");
    });

    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_MIGRATED
    );
    // The reader was held for the ~2 s observation window above, so at a ~100 ms
    // cadence the loop polls on the order of twenty times, plus the three write
    // safepoints. The floor proves it waited by POLLING rather than blocking;
    // the ceiling (200 = ~20 s of polling) would be blown by orders of magnitude
    // by a zero-sleep spin.
    let polls = polls.load(std::sync::atomic::Ordering::SeqCst);
    // Twice the real rate plus generous slack: at a ~100 ms cadence a hold of
    // `held_for` admits ~held_for/100 polls, and a zero-sleep spin exceeds this
    // by three orders of magnitude. The three write safepoints and the three
    // consent-boundary polls ride the same predicate, hence the +10 rather
    // than +6.
    let ceiling = (held_for.as_millis() as usize) / 50 + 10;
    assert!(
        polls <= ceiling,
        "poll count {polls} exceeds {ceiling} for a {held_for:?} hold — the wait spun \
         instead of sleeping its cadence"
    );
}

/// WHERE the lock is held, pinned with no wall in it: the interrupt safepoints
/// run INSIDE the write batch and the build runner runs AFTER it, so a foreign
/// `flock(LOCK_EX|LOCK_NB)` must FAIL at the former and SUCCEED at the latter.
/// A guard dropped straight after the acquire, or one left to fall at the end
/// of the function, each fails exactly one of these.
#[cfg(unix)]
#[test]
fn the_lock_is_held_across_the_batch_and_released_before_the_build() {
    let fx = make_workspace();
    adopt_the_lock_directory(&fx);
    // The lock file must exist before the probe opens it; adopting it did that.
    let probe = foreign_description(&fx.ws);
    assert!(
        lock_is_free(&probe),
        "nothing holds the lock before the run"
    );

    let mut engine = engine_for(&fx);
    // One observation per safepoint, in order — an aggregate flag would hide
    // WHICH call saw the lock free.
    let at_safepoints: std::cell::RefCell<Vec<bool>> = std::cell::RefCell::new(Vec::new());
    // Armed by the first poll that finds the lock HELD — the first write
    // safepoint — so everything the verb prints before the batch is not
    // sampled. The consent boundary polls this predicate before the lock
    // and sees it FREE; arming on "any poll" would start
    // sampling in the analysis phase.
    let armed = std::cell::Cell::new(false);
    let printed: std::cell::RefCell<Vec<(String, bool)>> = std::cell::RefCell::new(Vec::new());
    let interrupted = || {
        let held = !lock_is_free(&probe);
        at_safepoints.borrow_mut().push(held);
        if held {
            armed.set(true);
        }
        false
    };
    let free_at_build = std::cell::Cell::new(false);
    let build_calls = std::cell::Cell::new(0usize);
    let mut build_runner = |_ws: &Path, _pkgs: &[String]| {
        build_calls.set(build_calls.get() + 1);
        if lock_is_free(&probe) {
            free_at_build.set(true);
        }
        Ok(true)
    };
    // `run_migrate` directly, not `drive_write`: the printed-output seam is the
    // only one that reaches INSIDE the batch past the last safepoint (the
    // "applied: …" line is written AFTER the commit), and the safepoints stop
    // at pre-commit.
    let mut confirm = |_prompt: &str| Ok(true);
    let mut out = LockProbingOut {
        probe: &probe,
        armed: &armed,
        seen: &printed,
    };
    run_migrate(
        &MigrateOptions {
            workspace: fx.ws.clone(),
            write: true,
            assume_yes: true,
        },
        &mut MigrateDeps {
            engine: &mut engine,
            is_tty: false,
            confirm: &mut confirm,
            build_runner: &mut build_runner,
            out: &mut out,
            interrupted: &interrupted,
        },
    )
    .expect("the migration applies");

    // ANTI-VACUITY: both probes must actually have run.
    let at_safepoints = at_safepoints.into_inner();
    // The EXACT poll vector, which pins totality:
    // under `--yes` on a one-TU fixture the consent boundary polls
    // twice (the per-TU poll and the write head) and sees the lock FREE,
    // then the three write-window safepoints (pre-write, between file
    // writes, pre-commit) must ALL see it HELD. A safepoint outside the
    // hold, a mid-batch release, an extra poll anywhere, or a mutation that
    // returns early after the first safepoint all change the vector.
    assert_eq!(
        at_safepoints,
        vec![false, false, true, true, true],
        "expected two free consent-boundary polls then the 3 write-window safepoints \
         (pre-write, between writes, pre-commit) with the lock held; got {at_safepoints:?}"
    );
    assert_eq!(
        build_calls.get(),
        1,
        "the build runner must run exactly once"
    );
    assert!(
        free_at_build.get(),
        "the workspace lock was still held while the colcon build ran"
    );

    // The gap the safepoints cannot see. They stop at PRE-COMMIT, so the commit,
    // the manifest refresh and the release itself are unsampled by them — a lock
    // released and re-acquired in there would still read `[true, true, true]`.
    // Every line the verb prints while armed is a probe point, and the two that
    // matter bracket the release exactly: "applied: …" is written after the
    // commit (must be HELD) and "building affected package(s): …" after the drop
    // (must be FREE).
    //
    // SCOPE: this samples at the points the verb happens to print, not at
    // every write. A release-and-reacquire entirely between two adjacent file
    // writes is still unobserved here; the contender arm in
    // `write_blocks_while_another_writer_holds_the_workspace_lock` is what
    // catches an interleave at any boundary a release would expose; there is
    // no finer-grained per-write probe.
    let printed = printed.into_inner();
    let build_line = printed
        .iter()
        .position(|(text, _)| text.starts_with("building affected package(s)"))
        .expect("the verb must announce the build");
    assert!(
        build_line > 0,
        "nothing was printed inside the write batch — the post-commit window went unprobed: \
         {printed:?}"
    );
    assert!(
        printed[..build_line].iter().all(|(_, held)| *held),
        "the workspace lock was NOT held for a line printed inside the batch (the commit and \
         manifest-refresh window the safepoints cannot reach): {printed:?}"
    );
    assert!(
        !printed[build_line].1,
        "the lock was still held when the build was announced — it is released too late: \
         {printed:?}"
    );
}

/// The regression the whole variant exists for, on the fixture where it is
/// LOAD-BEARING: a fresh workspace, where the write batch's acquire is the
/// first thing to create `.cerulion/` and so the one that would top up
/// `.gitignore`. (On a workspace that already has `.cerulion/` — the headline
/// test's shape — `acquire` tops nothing up and the same assertion is
/// vacuous.) The porcelain oracle is spelled out in full so a regression that
/// writes anything else is caught too.
#[cfg(unix)]
#[test]
fn write_adds_no_gitignore_entry_where_the_gitignore_lacks_it() {
    let fx = make_workspace();
    let gitignore_before = std::fs::read(fx.ws.join(".gitignore")).unwrap();
    assert!(
        !fx.ws.join(".cerulion").exists(),
        "the point of this fixture is that the acquire is the CREATOR"
    );

    let mut engine = engine_for(&fx);
    let r = drive(&fx, &mut engine, true, true, false, true, true).expect("runs");
    assert!(matches!(r.outcome, MigrateOutcome::Applied { .. }));

    assert_eq!(
        std::fs::read(fx.ws.join(".gitignore")).unwrap(),
        gitignore_before,
        "--write topped up .gitignore — a TRACKED write this verb must never make"
    );
    assert_eq!(
        git(&fx.ws, &["status", "--porcelain", "--untracked-files=all"]),
        format!("?? {MANIFEST_REL_PATH}\n?? .cerulion/workspace.lock\n?? {PATCH_FILENAME}"),
        "--write left something other than its manifest, its lock and its patch. This oracle \
         is EXACT on purpose: a set comparison would absorb a new `.cerulion/` artifact \
         silently, and what this arm exists to catch is exactly a write nobody declared. If \
         a new artifact is intentional, add it to this list in the same change that \
         introduces it."
    );
}

/// The acquire lands AFTER consent, so its failure has its own message,
/// exercised here. `.cerulion` planted as a regular FILE makes
/// `create_dir_all` fail deterministically (it is untracked, so the dirty gate
/// still lets the run reach the acquire).
#[cfg(unix)]
#[test]
fn a_workspace_lock_that_cannot_be_taken_refuses_after_consent_without_writing() {
    let fx = make_workspace();
    std::fs::write(fx.ws.join(".cerulion"), b"not a directory").unwrap();
    let head_before = git(&fx.ws, &["rev-parse", "HEAD"]);

    let mut engine = engine_for(&fx);
    let err = drive(&fx, &mut engine, true, true, false, true, true)
        .expect_err("the acquire cannot succeed")
        .to_string();
    assert!(
        err.contains("could not take the workspace lock"),
        "got: {err}"
    );
    assert!(err.contains(".cerulion/workspace.lock"), "got: {err}");
    assert!(err.contains("nothing was committed"), "got: {err}");
    // And the claim is TRUE.
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL
    );
    assert_eq!(git(&fx.ws, &["rev-parse", "HEAD"]), head_before);
    assert!(!fx.ws.join(PATCH_FILENAME).exists());
}

/// A wait entered after the user consented must be endable. The holder keeps
/// the lock; the migration's interrupt predicate trips; the run must refuse in
/// migrate's own interrupt vocabulary having written nothing — not park in an
/// `SA_RESTART`-proof `flock` that only SIGKILL ends.
/// `wait_announced` is the calling test's `logs_contain` closure over the
/// lock wait's own one-shot warning — the deterministic rendezvous the
/// predicate trips on (see the comment at the predicate).
#[cfg(unix)]
fn lock_wait_interrupt_arm(assume_yes: bool, wait_announced: &dyn Fn() -> bool) {
    let fx = make_workspace();
    adopt_the_lock_directory(&fx);
    let head_before = git(&fx.ws, &["rev-parse", "HEAD"]);
    let mut engine = engine_for(&fx);

    // A FOREIGN description, not a `WorkspaceLock` on this thread: the lock is
    // same-thread REENTRANT, so a guard taken here would not block the
    // migration at all and the test would pass without ever entering a wait.
    use std::os::fd::AsRawFd;
    let held = foreign_description(&fx.ws);
    // SAFETY: `held` is open for the whole test.
    assert_eq!(0, unsafe { libc::flock(held.as_raw_fd(), libc::LOCK_EX) });

    // WATCHDOG. Without it, a build that stops honouring the predicate does not
    // FAIL this arm — it BLOCKS it, and a blocked test is a wedged CI job with
    // no attribution. The
    // watchdog removes the blocking condition so the call returns, and the flag
    // turns "it blocked" into an assertion. Expected wall here is ~100 ms, so
    // the 30 s budget is a ~300x margin and load can only make it safer: a slow
    // machine still returns long before the watchdog fires.
    //
    // It OWNS a duplicate descriptor rather than capturing the raw fd NUMBER.
    // A number is not a handle: this test returns in ~0.1 s, `held` drops, the
    // number is immediately reusable, and a detached watchdog waking 30 s later
    // would `LOCK_UN` whatever a PARALLEL test had since opened — silently
    // invalidating that test's verdict. The dup keeps the number reserved for
    // as long as the watchdog can act, and shares the same open file
    // description, so unlocking through it releases the intended lock. It is
    // also cancelled and joined, so nothing outlives the test at all.
    let watchdog_fd = held.try_clone().expect("dup the lock descriptor");
    let watchdog_fired = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let (cancel_tx, cancel_rx) = mpsc::channel::<()>();
    let watchdog = {
        let watchdog_fired = watchdog_fired.clone();
        std::thread::spawn(move || {
            if cancel_rx.recv_timeout(Duration::from_secs(30)).is_ok() {
                return;
            }
            watchdog_fired.store(true, std::sync::atomic::Ordering::SeqCst);
            // SAFETY: the watchdog owns `watchdog_fd` for the whole call.
            unsafe { libc::flock(watchdog_fd.as_raw_fd(), libc::LOCK_UN) };
        })
    };
    // The predicate trips on a DETERMINISTIC RENDEZVOUS, not on an ordinal
    // and not on a wall. The lock wait announces itself exactly once — the
    // "waiting for it to finish" warning — AFTER its first check returned
    // false and BEFORE its first sleep, on this thread, so the first poll
    // that sees the announcement is the post-sleep check and no other poll
    // can be. `wait_announced` is the test's `logs_contain` over that line
    // (`#[traced_test]` on the caller captures it; the prompted path also
    // gates on the consent signal so polls before the yes are not counted).
    // Tripping on a count, or on a timing gap between
    // the last two polls, is weaker: a count cannot tell an extra pre-lock poll
    // from the wait's own first check, and a gap is something scheduler
    // delay can supply without any sleep (the load-sensitive class). With the
    // rendezvous the count below is EXACT and load-independent — an extra
    // pre-lock poll reads as 5, not 4 — and the gap assertion is only a
    // corroboration that the sleep was real.
    let (at_consent_tx, at_consent_rx) = mpsc::channel();
    let consented = std::cell::Cell::new(false);
    let polls = std::cell::Cell::new(0usize);
    let wait_seen = std::cell::Cell::new(false);
    let poll_instants = std::cell::RefCell::new(Vec::<std::time::Instant>::new());
    let interrupted = || {
        if !assume_yes {
            if !consented.get() && at_consent_rx.try_recv().is_ok() {
                consented.set(true);
            }
            if !consented.get() {
                return false;
            }
        }
        polls.set(polls.get() + 1);
        poll_instants.borrow_mut().push(std::time::Instant::now());
        if !wait_seen.get() && wait_announced() {
            wait_seen.set(true);
        }
        wait_seen.get()
    };
    let mut build_runner = |_ws: &Path, _pkgs: &[String]| Ok(true);
    let started = std::time::Instant::now();
    let err = drive_write(
        &fx,
        &mut engine,
        WriteRun {
            at_consent: at_consent_tx,
            interrupted: &interrupted,
            build_runner: &mut build_runner,
            assume_yes,
        },
    )
    .expect_err("an interrupted wait must refuse")
    .to_string();
    let elapsed = started.elapsed();
    let _ = cancel_tx.send(());
    watchdog.join().expect("the watchdog thread panicked");
    // SAFETY: as above.
    unsafe { libc::flock(held.as_raw_fd(), libc::LOCK_UN) };

    assert!(
        !watchdog_fired.load(std::sync::atomic::Ordering::SeqCst),
        "the wait never honoured the interrupt predicate — it blocked past 30 s \
         (the watchdog had to release the lock to get the call back). Error: {err}"
    );
    // THE discriminating oracle. The write safepoints refuse with a bare
    // "interrupted — nothing was written; nothing was migrated.", so asserting
    // only that would also pass on a call site that dropped the refusal and
    // proceeded UNLOCKED — the next safepoint produces a byte-identical string.
    // Only the lock wait names the lock.
    assert!(
        err.contains("interrupted while waiting for the workspace lock"),
        "the refusal must come from the LOCK WAIT, not a later write safepoint; got: {err}"
    );
    assert!(err.contains(".cerulion/workspace.lock"), "got: {err}");
    assert!(err.contains("nothing was committed"), "got: {err}");
    // The residue clause: the wait itself may have created `.cerulion/`, and the
    // sibling failed-acquire arm is scrupulous about saying so. This one must be
    // too: a flat "nothing was written" would be false.
    assert!(err.contains("may have created the untracked"), "got: {err}");
    // The rendezvous fired: the wait announced itself and the predicate saw
    // it. Without this the count below could be satisfied by a predicate
    // that never tripped at all (the watchdog would then have ended the
    // wait, which the assertion above already rules out — this names the
    // mechanism rather than its absence).
    assert!(
        wait_seen.get(),
        "the predicate never saw the lock wait's announcement: {err}"
    );
    // The count is EXACT and deterministic: the tripping poll is the first
    // one after the announcement, which is the wait's post-sleep check —
    // poll 3 after consent on the prompted path, poll 4 raw under `--yes`
    // (the per-TU poll, the write-head poll, the wait's first check, then
    // the post-sleep check). A poll added before the lock reads as one more,
    // loudly, on every runner.
    let tripping_poll = if assume_yes { 4 } else { 3 };
    assert_eq!(
        polls.get(),
        tripping_poll,
        "the interrupt tripped at poll {} rather than at the wait's post-sleep check: {err}",
        polls.get()
    );
    // Corroboration only: the loop sleeps `CONTENTION_POLL` between the
    // check that announced and the check that tripped. A LOWER bound on a
    // sleep is load-safe (contention only lengthens it), and it is not the
    // discriminator — the rendezvous is — so scheduler delay cannot turn a
    // missing sleep into a pass here without also faking the announcement.
    let gap = {
        let instants = poll_instants.borrow();
        let n = instants.len();
        assert!(n >= 2, "fewer than two polls were timestamped: {err}");
        instants[n - 1].duration_since(instants[n - 2])
    };
    assert!(
        gap >= CONTENTION_POLL,
        "the tripping poll came {gap:?} after the poll before it, less than the wait's \
         {CONTENTION_POLL:?} cadence — the loop did not sleep between them: {err}"
    );
    // The whole-run wall is the coarse, independent half: a gap that lied
    // would still have to explain it.
    assert!(
        elapsed >= CONTENTION_POLL,
        "the refusal came back in {elapsed:?} — the wait never slept a poll, so it \
         did not come from the polling loop: {err}"
    );
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL
    );
    assert_eq!(git(&fx.ws, &["rev-parse", "HEAD"]), head_before);
    assert_eq!(
        git(&fx.ws, &["status", "--porcelain", "--untracked-files=all"]),
        "",
        "an interrupted run left something behind"
    );
}

/// The prompted path into a contended wait.
#[cfg(unix)]
#[test]
#[traced_test]
fn an_interrupt_during_the_lock_wait_refuses_without_writing() {
    lock_wait_interrupt_arm(false, &|| logs_contain("waiting for it to finish"));
}

/// The non-interactive path into a contended wait — a CI `--yes` run parked
/// behind another writer. The acquire is the same code one poll earlier, and
/// this arm is what keeps it driven.
#[cfg(unix)]
#[test]
#[traced_test]
fn an_interrupt_during_the_lock_wait_under_yes_refuses_without_writing() {
    lock_wait_interrupt_arm(true, &|| logs_contain("waiting for it to finish"));
}

/// The primitive claim `--write` rests on, stated in git's own terms and with
/// the control that makes it non-vacuous: on a workspace whose `.gitignore`
/// does NOT yet ignore `.cerulion/`, `acquire_and_track_gitignore` leaves a
/// MODIFIED TRACKED FILE — the exact side effect that fails migrate's
/// rollback arms — while the constructor
/// migrate takes leaves only untracked state; and on a workspace that does
/// ignore it (what `cerulion workspace create` writes), it leaves `git status`
/// entirely EMPTY.
#[cfg(unix)]
#[test]
fn the_constructor_migrate_takes_adds_no_tracked_change_where_the_tracking_one_does() {
    let topping = make_workspace();
    drop(WorkspaceLock::acquire_and_track_gitignore(&topping.ws).unwrap());
    assert_eq!(
        git(
            &topping.ws,
            &["status", "--porcelain", "--untracked-files=all"]
        ),
        // Porcelain's two status columns: an UNSTAGED modification is
        // `<space>M`, so the leading space is part of the oracle.
        " M .gitignore",
        "the control: the constructor named for the tracked write does modify a tracked file"
    );

    let quiet = make_workspace();
    drop(acquire_quiet(&quiet.ws));
    assert_eq!(
        git(
            &quiet.ws,
            &["status", "--porcelain", "--untracked-files=all"]
        ),
        "?? .cerulion/workspace.lock",
        "acquire_interruptibly leaves the lock file and nothing else"
    );

    let ignored = make_workspace();
    std::fs::write(ignored.ws.join(".gitignore"), "/build/\n.cerulion/\n").unwrap();
    git_commit_all(&ignored.ws, "ignore the cerulion directory");
    drop(acquire_quiet(&ignored.ws));
    assert_eq!(
        git(
            &ignored.ws,
            &["status", "--porcelain", "--untracked-files=all"]
        ),
        "",
        "on a workspace that ignores `.cerulion/`, the acquire is invisible"
    );
}

/// A refusal INSIDE the write batch must give the lock back — both halves of it.
///
/// Every other arm either never acquires or runs to completion, so only this
/// one pins the unwind path. It matters beyond this verb: `cerulion-wsd` is a
/// LONG-LIVED process sharing this primitive, and a guard held past an early
/// return (or a registry reservation left behind) wedges every later writer in
/// that process — a leak a short-lived CLI would never reveal.
#[cfg(unix)]
#[test]
fn a_refusal_inside_the_write_batch_releases_the_workspace_lock() {
    let fx = make_workspace();
    adopt_the_lock_directory(&fx);
    let head_before = git(&fx.ws, &["rev-parse", "HEAD"]);
    let probe = foreign_description(&fx.ws);
    let mut engine = engine_for(&fx);

    // Trip at the SECOND safepoint: the batch has started and files are on
    // disk, so the refusal unwinds through the rollback rather than returning
    // before anything happened. Counted from the first poll that finds the
    // lock HELD — the consent-boundary polls see it free
    // and must not shift which safepoint trips. The lock FILE cannot anchor
    // this arm: the fixture adopts it up front.
    let calls = std::cell::Cell::new(0usize);
    let interrupted = || {
        if lock_is_free(&probe) {
            return false;
        }
        calls.set(calls.get() + 1);
        calls.get() >= 2
    };
    let mut build_runner = |_ws: &Path, _pkgs: &[String]| Ok(true);
    let (at_consent_tx, _rx) = mpsc::channel();
    let err = drive_write(
        &fx,
        &mut engine,
        WriteRun {
            at_consent: at_consent_tx,
            interrupted: &interrupted,
            build_runner: &mut build_runner,
            assume_yes: true,
        },
    )
    .expect_err("an interrupted write batch must refuse")
    .to_string();
    // The WINDOW, not the word: the consent boundary refuses with
    // "interrupted" too, and a trip there never takes the lock — which would
    // leave every "the lock was given back" oracle below passing vacuously.
    assert!(
        err.contains("interrupted — every write was rolled back"),
        "the trip must come from a WRITE-WINDOW safepoint; got: {err}"
    );
    assert!(calls.get() >= 2, "the run never reached a safepoint: {err}");
    assert_eq!(git(&fx.ws, &["rev-parse", "HEAD"]), head_before);

    // Half one: the KERNEL lock is back.
    assert!(
        lock_is_free(&probe),
        "a refusal inside the write batch leaked the kernel lock"
    );
    // Half two: the in-process REGISTRY reservation is back. The probe cannot
    // see that — a leaked reservation parks the next acquire on the registry
    // condvar, never on `flock` — so take the lock for real, on a thread with a
    // deadline so the failure is a FAIL and not a hung binary.
    let (tx, rx) = mpsc::channel();
    let ws = fx.ws.clone();
    std::thread::spawn(move || {
        let got = WorkspaceLock::acquire_interruptibly(&ws, &|| false);
        tx.send(got.is_ok()).unwrap();
    });
    assert!(
        rx.recv_timeout(Duration::from_secs(10))
            .expect("a refusal inside the batch leaked the registry reservation"),
        "the workspace must be lockable again after a refusal"
    );
}

/// A structural gate on migrate's choice of constructor.
///
/// The wrong choice is also killed behaviourally by
/// three arms, which is a stronger gate than a source walk. The walk is warranted
/// because of the constructor shape: both blocking constructors
/// return a `CliResult<Self>` that `?` accepts, so a future edit could reach
/// for one here and compile, where the interruptible constructor forces an
/// exhaustive match on `AcquireError`. The behavioural arms still run; this one
/// names the mistake at the point a reader makes it.
///
/// TWO forbidden forms (the tracked write lives outside the
/// default constructor), and they are wrong for DIFFERENT reasons — which is
/// why the messages are separate rather than merged:
///
/// * `acquire` does not touch `.gitignore`, so its defect here is the WAIT:
///   it blocks in the kernel, `ctrlc` installs its handler with `SA_RESTART`,
///   and this verb promises the user can Ctrl-C out of the window it opens
///   after consent;
/// * `acquire_and_track_gitignore` carries BOTH that wait and the tracked-file
///   write that fails migrate's rollback arms.
///
/// The walk reads a COMMENT-STRIPPED view, because the file's own prose
/// discusses these constructors at length in order to explain which one it uses
/// — a naive grep would fail on the explanation rather than the code.
#[test]
fn migrate_takes_the_interruptible_constructor_and_neither_blocking_one() {
    let source = std::fs::read_to_string(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/ros2_migrate.rs"),
    )
    .expect("ros2_migrate.rs is readable");
    let code = code_only(&source);

    // ANTI-TAUTOLOGY: without this, a stripper that ate the whole file would
    // make every assertion below vacuous.
    assert!(
        code.contains("WorkspaceLock::acquire_interruptibly("),
        "the comment-stripped view lost the call site — the walk proves nothing"
    );
    assert!(
        !code.contains("WorkspaceLock::acquire("),
        "`ros2_migrate.rs` calls `WorkspaceLock::acquire(`, whose contended wait blocks in \
         the kernel — `ctrlc` installs its handler with `SA_RESTART`, so the user cannot \
         Ctrl-C out of it, and this verb promises an interruptible window after consent. \
         Use `acquire_interruptibly`, whose `AcquireError` must be matched."
    );
    assert!(
        !code.contains("WorkspaceLock::acquire_and_track_gitignore("),
        "`ros2_migrate.rs` calls `WorkspaceLock::acquire_and_track_gitignore(`, which tops \
         up a TRACKED `.gitignore` the first time it creates `.cerulion/` — this verb \
         promises a dry-run writes only its manifest and a rolled-back `--write` leaves a \
         pristine tree — AND blocks uninterruptibly while it waits. Use \
         `acquire_interruptibly`."
    );
    // NO third assertion here, deliberately. Asserting that
    // `ros2_migrate.rs`'s PROSE still names the forbidden form, to show the
    // strip is load-bearing, fails the arm on any rewrite that deletes
    // that sentence, a change that can make the code more
    // correct. Asserting "the module header still mentions some
    // `WorkspaceLock::` path" is WORSE: two independent whole-file
    // substring checks, neither requiring a COMMENT to contain the forbidden
    // needle, so it survives exactly the removal it is meant to notice.
    //
    // What the strip buys is stated, not asserted: the module header carries
    // intra-doc links naming `WorkspaceLock::` paths, so a future doc line
    // explaining why the default constructor is wrong HERE would otherwise
    // fail this walk on the explanation instead of on the code. The stripper
    // itself is proven on inputs of its own by
    // `code_only_strips_both_comment_syntaxes`, which is where a stripper
    // belongs — against this file an IDENTITY stripper yields a passing walk,
    // so no assertion here can tell a working stripper from a no-op one. The
    // anti-tautology assertion above catches only the opposite failure, a
    // stripper that eats the call site.
}

/// Oracle for the walk's comment stripper, on inputs of its own.
///
/// The walk above cannot test this: against a file that happens not to mention
/// the forbidden form in prose, a stripper that returns the input UNCHANGED
/// produces a passing walk. (A stripper that ate the whole file is the one
/// shape the walk does catch, at its anti-tautology assertion.)
#[test]
fn code_only_strips_both_comment_syntaxes() {
    assert_eq!(
        code_only("a // b\nc"),
        "a \nc",
        "line comment to end of line"
    );
    assert_eq!(code_only("a /* b */ c"), "a  c", "block comment");
    assert_eq!(
        code_only("a /* b /* c */ d */ e"),
        "a  e",
        "Rust block comments NEST — a depth-blind stripper leaves ` d */ e`"
    );
    assert_eq!(
        code_only("a /* b\nc */ d"),
        "a  d",
        "a block comment spans lines"
    );
    assert_eq!(
        code_only("// /* a\nb"),
        "\nb",
        "a block opener inside a line comment opens nothing"
    );
    assert_eq!(
        code_only("a /* b"),
        "a ",
        "an unterminated block fails CLOSED — it eats the tail rather than \
         leaking it into the walk"
    );
    assert_eq!(
        code_only("//! names WorkspaceLock::acquire(\nWorkspaceLock::acquire_interruptibly(&ws"),
        "\nWorkspaceLock::acquire_interruptibly(&ws",
        "the exact shape the walk depends on: prose naming the forbidden form is \
         stripped while the real call site survives"
    );
    assert_eq!(
        code_only("/* // */ a"),
        " a",
        "a line comment INSIDE a block opens nothing — dropping the `depth == 0` guard \
         eats the `*/`, strands the depth and swallows the rest of the file"
    );
    assert_eq!(
        code_only("*/ a"),
        "*/ a",
        "a close with no open at depth 0 is ordinary code, not a strip"
    );
    assert_eq!(
        code_only("é /* ø */ é"),
        "é  é",
        "multi-byte characters survive BYTE-for-byte — a `bytes[i] as char` \
         cast renders each continuation byte as its own Latin-1 codepoint"
    );
    assert_eq!(
        code_only("/* é */x"),
        "x",
        "a multi-byte character INSIDE a stripped span is dropped whole"
    );
}

/// Comment-stripped view of Rust source: `//` to end of line and `/* */`
/// blocks, depth-tracked because Rust block comments NEST. String literals are
/// deliberately not modelled — the walk above asserts on code shapes that no
/// literal in that file contains.
///
/// Byte-preserving: the kept bytes are copied VERBATIM, so a multi-byte
/// character survives. (Pushing `bytes[i] as char` instead is a
/// Latin-1 cast that turns every multi-byte character into several codepoints. It
/// cannot affect an ASCII needle, but it is a real bug the moment the
/// helper is copied — and `cerulion_core/tests/cdylib_iox2_log_level_test.rs`'s
/// `code_only` is oracle-tested against exactly that case.)
fn code_only(source: &str) -> String {
    let bytes = source.as_bytes();
    let mut kept: Vec<u8> = Vec::with_capacity(source.len());
    let (mut i, mut depth) = (0usize, 0usize);
    while i < bytes.len() {
        if depth == 0 && bytes[i..].starts_with(b"//") {
            while i < bytes.len() && bytes[i] != b'\n' {
                i += 1;
            }
            continue;
        }
        if bytes[i..].starts_with(b"/*") {
            depth += 1;
            i += 2;
            continue;
        }
        if depth > 0 && bytes[i..].starts_with(b"*/") {
            depth -= 1;
            i += 2;
            continue;
        }
        if depth == 0 {
            kept.push(bytes[i]);
        }
        i += 1;
    }
    String::from_utf8(kept).expect("dropping whole comment spans keeps the rest valid UTF-8")
}

// ---------------------------------------------------------------------------
// The interrupt latch is polled at the consent boundary.
//
// The CLI installs its Ctrl-C handler before the analysis and the flag it
// flips is one-shot. If the engine consumed it only inside the
// write window (the lock wait and the three write safepoints), a Ctrl-C
// during the analysis or at the `[y/N]` prompt would do nothing visible — and
// then poison the run the user went on to approve, which would report
// "interrupted" from a write safepoint after taking the lock. Four polls
// prevent that: at the top of every per-TU analysis iteration, when the engine
// FAILS with the latch set (a terminal Ctrl-C kills the engine child too),
// at the head of the `--write` section ahead of every gate, and after a
// `yes`. Each is pinned by its own arm through the injected seams
// (deterministic — no signal, no thread), with a negative control for the
// attribution; the REAL-binary shape (a real handler, a real terminal read,
// a real process-group signal) is `cerulion_cli/tests/ros2_migrate_cli_test.rs`.
// ---------------------------------------------------------------------------

/// Every arm below asserts the ABSENCE of the pre-write safepoint's string
/// (`PRE_WRITE_INTERRUPT_REFUSAL`, exported by the engine so a reword cannot
/// disarm the guard silently — its PRESENCE is pinned by the trip-1 arm of
/// `an_interrupt_mid_write_rolls_back_everything_and_refuses`), and that
/// the workspace lock was never taken (`LOCK_DIR` is what taking it
/// creates — the FILE, since the manifest shares the directory): a
/// pre-consent interrupt is answered at the consent boundary, never from
/// inside the write window.
fn assert_refused_at_the_consent_boundary(fx: &Fixture, head_before: &str, err: &str) {
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL,
        "a source file was rewritten; err: {err}"
    );
    assert_eq!(
        git(&fx.ws, &["rev-parse", "HEAD"]),
        head_before,
        "HEAD moved; err: {err}"
    );
    // The lock FILE, not its directory: the manifest lives in the same
    // directory, so the directory alone would misattribute a manifest write.
    assert!(
        !fx.ws.join(LOCK_DIR).join(LOCK_FILE).exists(),
        "the workspace lock file exists, so the lock was TAKEN — the refusal came from a \
         write-window safepoint after consent, not from the consent boundary; err: {err}"
    );
    assert!(
        !err.contains(PRE_WRITE_INTERRUPT_REFUSAL),
        "the refusal came from the write safepoint, not the consent boundary; err: {err}"
    );
    assert!(err.contains("nothing was committed"), "err: {err}");
    // The tree, not just the one file and HEAD: "nothing was written" is a
    // claim about every path under the workspace, untracked ones included —
    // a patch file, a lock directory, a stray backup. `--untracked-files=all`
    // lists a file inside an untracked directory by name, so a directory the
    // refusal created does not hide what it holds.
    assert_eq!(
        git(&fx.ws, &["status", "--porcelain", "--untracked-files=all"]),
        "",
        "the refusal left the tree dirty; err: {err}"
    );
}

/// An engine that flips the interrupt latch WHILE analysing a TU and counts
/// its invocations — the shape of a Ctrl-C landing mid-clang.
struct InterruptingEngine<'a> {
    inner: FixtureEngine,
    latch: &'a std::cell::Cell<bool>,
    calls: &'a std::cell::Cell<usize>,
}

impl MigrateEngine for InterruptingEngine<'_> {
    fn describe(&self) -> String {
        self.inner.describe()
    }
    fn analyze_tu(&mut self, db: &Path, src: &Path, tu: &Path) -> CliResult<String> {
        self.calls.set(self.calls.get() + 1);
        self.latch.set(true);
        self.inner.analyze_tu(db, src, tu)
    }
}

/// Run `--write` with the consent seam and the latch as the arm's own; the
/// build runner must never be reached. `is_tty` mirrors `!assume_yes`, as
/// `drive_write` does — a `--yes` run is the non-TTY shape and a prompted
/// run is the TTY shape; the non-TTY-without-`--yes` refusal is pinned by
/// its own arms elsewhere.
fn drive_write_with(
    fx: &Fixture,
    engine: &mut dyn MigrateEngine,
    assume_yes: bool,
    confirm: &mut dyn FnMut(&str) -> CliResult<bool>,
    interrupted: &dyn Fn() -> bool,
) -> String {
    let mut build_runner = |_ws: &Path, _pkgs: &[String]| -> CliResult<bool> {
        panic!("the build runner must not be reached by a refused run")
    };
    let mut stdout: Vec<u8> = Vec::new();
    run_migrate(
        &MigrateOptions {
            workspace: fx.ws.clone(),
            write: true,
            assume_yes,
        },
        &mut MigrateDeps {
            engine,
            is_tty: !assume_yes,
            confirm,
            build_runner: &mut build_runner,
            out: &mut stdout,
            interrupted,
        },
    )
    .expect_err("a run interrupted before consent must refuse")
    .to_string()
}

/// The interrupt lands while TU 1 of 2 is being analysed: the per-TU poll
/// stops the loop before TU 2 is handed to the engine, and the refusal
/// names the analysis window and the count.
#[test]
fn an_interrupt_during_analysis_stops_the_per_tu_loop_before_the_next_tu() {
    let fx = make_workspace();
    let talker2 = fx.talker.with_file_name("talker2.cpp");
    std::fs::write(&talker2, TALKER_ORIGINAL).unwrap();
    let db_dir = fx.ws.join("build/demo_pkg");
    std::fs::write(
        db_dir.join("compile_commands.json"),
        format!(
            "[{{\"directory\": \"{d}\", \"command\": \"clang++ -c {a}\", \"file\": \"{a}\"}}, \
             {{\"directory\": \"{d}\", \"command\": \"clang++ -c {b}\", \"file\": \"{b}\"}}]",
            d = db_dir.display(),
            a = fx.talker.display(),
            b = talker2.display()
        ),
    )
    .unwrap();
    git_commit_all(&fx.ws, "a second translation unit");
    let head_before = git(&fx.ws, &["rev-parse", "HEAD"]);

    let mut by_tu = HashMap::new();
    by_tu.insert(fx.talker.clone(), talker_analysis(&fx));
    by_tu.insert(talker2.clone(), talker_analysis_for(&talker2, &talker2));
    let latch = std::cell::Cell::new(false);
    let calls = std::cell::Cell::new(0usize);
    let mut engine = InterruptingEngine {
        inner: FixtureEngine { by_tu },
        latch: &latch,
        calls: &calls,
    };
    let interrupted = || latch.get();
    let mut confirm = |_prompt: &str| -> CliResult<bool> {
        panic!("the prompt must not be reached by a run interrupted during analysis")
    };

    let err = drive_write_with(&fx, &mut engine, true, &mut confirm, &interrupted);
    assert_eq!(
        calls.get(),
        1,
        "TU 2 was handed to the engine after the interrupt — the per-TU poll is missing; \
         err: {err}"
    );
    assert!(
        err.contains("interrupted — analysis stopped after 1 translation unit(s)"),
        "the refusal must name the analysis window and the count; got: {err}"
    );
    assert_refused_at_the_consent_boundary(&fx, &head_before, &err);
}

/// The interrupt lands during the LAST TU (so the per-TU poll never sees
/// it): the run must refuse before the prompt is asked — a question whose
/// "yes" cannot be honoured is not asked.
#[test]
fn an_interrupt_during_the_last_tu_refuses_before_the_prompt_is_asked() {
    let fx = make_workspace();
    let head_before = git(&fx.ws, &["rev-parse", "HEAD"]);
    let latch = std::cell::Cell::new(false);
    let calls = std::cell::Cell::new(0usize);
    let mut engine = InterruptingEngine {
        inner: engine_for(&fx),
        latch: &latch,
        calls: &calls,
    };
    let interrupted = || latch.get();
    let prompted = std::cell::Cell::new(false);
    let mut confirm = |_prompt: &str| -> CliResult<bool> {
        prompted.set(true);
        Ok(true)
    };

    let err = drive_write_with(&fx, &mut engine, false, &mut confirm, &interrupted);
    assert!(
        !prompted.get(),
        "the prompt was ASKED after an interrupt had already been requested; err: {err}"
    );
    assert_eq!(calls.get(), 1, "the single TU is analysed exactly once");
    assert!(
        err.contains("interrupted before consent"),
        "the refusal must name the pre-consent window; got: {err}"
    );
    assert_refused_at_the_consent_boundary(&fx, &head_before, &err);
}

/// THE headline: the interrupt lands while the prompt is waiting and the
/// user then answers `yes`. The run must refuse in the prompt-window
/// vocabulary having taken no lock and written nothing — never accept the
/// yes and report "interrupted" from inside the write window.
#[test]
fn an_interrupt_while_the_prompt_waits_refuses_the_yes_that_follows_it() {
    let fx = make_workspace();
    let head_before = git(&fx.ws, &["rev-parse", "HEAD"]);
    let mut engine = engine_for(&fx);
    let latch = std::cell::Cell::new(false);
    let interrupted = || latch.get();
    let mut confirm = |prompt: &str| -> CliResult<bool> {
        assert!(prompt.contains("[y/N]"), "prompt: {prompt}");
        latch.set(true);
        Ok(true)
    };

    let err = drive_write_with(&fx, &mut engine, false, &mut confirm, &interrupted);
    assert!(
        err.contains("interrupted while the consent prompt was waiting"),
        "the refusal must name the PROMPT window; got: {err}"
    );
    assert_refused_at_the_consent_boundary(&fx, &head_before, &err);
}

/// A `no` after a Ctrl-C is a plain decline: the post-answer poll sits AFTER
/// the decline branch, so the safe answer is not turned into an error. This
/// is the PLACEMENT control — it passes with or without the post-answer poll and
/// exists to kill "the poll moved above the decline branch". It also fixes the one
/// deliberate exit-code asymmetry: every other pre-consent window exits 1,
/// while a `no` exits 0, because here the user answered affirmatively.
#[test]
fn a_decline_after_an_interrupt_is_a_plain_decline() {
    let fx = make_workspace();
    let head_before = git(&fx.ws, &["rev-parse", "HEAD"]);
    let mut engine = engine_for(&fx);
    let latch = std::cell::Cell::new(false);
    let interrupted = || latch.get();
    let mut confirm = |_prompt: &str| -> CliResult<bool> {
        latch.set(true);
        Ok(false)
    };
    let mut build_runner = |_ws: &Path, _pkgs: &[String]| -> CliResult<bool> {
        panic!("the build runner must not be reached by a declined run")
    };
    let mut stdout: Vec<u8> = Vec::new();
    let outcome = run_migrate(
        &MigrateOptions {
            workspace: fx.ws.clone(),
            write: true,
            assume_yes: false,
        },
        &mut MigrateDeps {
            engine: &mut engine,
            is_tty: true,
            confirm: &mut confirm,
            build_runner: &mut build_runner,
            out: &mut stdout,
            interrupted: &interrupted,
        },
    )
    .expect("a decline is not an error");
    assert!(
        matches!(outcome, MigrateOutcome::Declined),
        "outcome: {outcome:?}"
    );
    let stdout = String::from_utf8_lossy(&stdout);
    assert!(
        stdout.contains("declined — nothing written."),
        "stdout: {stdout}"
    );
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL
    );
    assert_eq!(git(&fx.ws, &["rev-parse", "HEAD"]), head_before);
    assert!(!fx.ws.join(LOCK_DIR).join(LOCK_FILE).exists());
    assert_eq!(
        git(&fx.ws, &["status", "--porcelain", "--untracked-files=all"]),
        "",
        "a decline left the tree dirty"
    );
}

/// Under `--yes` there is no prompt and no post-answer poll, so the poll at
/// the head of the write section is the ONE consent-boundary read on the
/// non-interactive path. An interrupt during the last TU must refuse there
/// — a narrowing of that poll to the prompted path would pass every other
/// arm.
#[test]
fn an_interrupt_during_the_last_tu_under_yes_refuses_before_the_write_batch() {
    let fx = make_workspace();
    let head_before = git(&fx.ws, &["rev-parse", "HEAD"]);
    let latch = std::cell::Cell::new(false);
    let calls = std::cell::Cell::new(0usize);
    let mut engine = InterruptingEngine {
        inner: engine_for(&fx),
        latch: &latch,
        calls: &calls,
    };
    let interrupted = || latch.get();
    let mut confirm = |_prompt: &str| -> CliResult<bool> {
        panic!("--yes never asks; the prompt must not be reached")
    };

    let err = drive_write_with(&fx, &mut engine, true, &mut confirm, &interrupted);
    assert_eq!(calls.get(), 1, "the single TU is analysed exactly once");
    // The `--yes` window names the write batch, not a prompt: the command
    // line WAS the consent.
    assert!(
        err.contains("interrupted before the write batch"),
        "the refusal must name the write-batch window, never a prompt a --yes run has no \
         part in; got: {err}"
    );
    assert_refused_at_the_consent_boundary(&fx, &head_before, &err);
}

/// An engine that fails WHILE flipping the latch — the shape a terminal
/// Ctrl-C produces, since the engine child shares the process group and dies
/// at its default disposition.
struct SignalledEngine<'a> {
    latch: &'a std::cell::Cell<bool>,
}

impl MigrateEngine for SignalledEngine<'_> {
    fn describe(&self) -> String {
        "signalled-engine".to_string()
    }
    fn analyze_tu(&mut self, _db: &Path, _src: &Path, _tu: &Path) -> CliResult<String> {
        self.latch.set(true);
        Err(cerulion_cli_engine::error::CliError::Validation(
            "migration engine failed on the TU (signal: 2)".to_string(),
        ))
    }
}

/// A terminal Ctrl-C kills the engine child too, so the engine's failure IS
/// the interrupt: it must be reported as the interrupt, not as a prover
/// crash the user is told to investigate.
#[test]
fn an_engine_failure_with_the_latch_tripped_is_reported_as_the_interrupt() {
    let fx = make_workspace();
    let head_before = git(&fx.ws, &["rev-parse", "HEAD"]);
    let latch = std::cell::Cell::new(false);
    let mut engine = SignalledEngine { latch: &latch };
    let interrupted = || latch.get();
    let mut confirm = |_prompt: &str| -> CliResult<bool> {
        panic!("the prompt must not be reached after an interrupted analysis")
    };

    let err = drive_write_with(&fx, &mut engine, true, &mut confirm, &interrupted);
    assert!(
        err.contains("interrupted — analysis stopped after 0 translation unit(s)"),
        "the engine's death under an interrupt must be attributed to the interrupt; got: {err}"
    );
    assert!(
        !err.contains("migration engine failed"),
        "the user who pressed Ctrl-C was told the engine crashed; got: {err}"
    );
    assert_refused_at_the_consent_boundary(&fx, &head_before, &err);
}

/// An engine that fails with the latch CLEAR — a genuine prover crash, the
/// shape the attribution must NOT touch.
struct CrashingEngine;

impl MigrateEngine for CrashingEngine {
    fn describe(&self) -> String {
        "crashing-engine".to_string()
    }
    fn analyze_tu(&mut self, _db: &Path, _src: &Path, _tu: &Path) -> CliResult<String> {
        Err(cerulion_cli_engine::error::CliError::Validation(
            "migration engine failed on the TU (exit status: 1). Engine stderr tail:\n\
             clang: fatal error: no such file"
                .to_string(),
        ))
    }
}

/// The NEGATIVE control for the attribution: a genuine engine failure with
/// no interrupt pending must surface the engine's own error, stderr tail
/// included, and must never be dressed up as an interrupt the user did not
/// request — an unconditional attribution passes every other arm. This is
/// also the arm that drives the grace to its deadline.
#[test]
fn a_genuine_engine_failure_is_reported_as_the_engines_not_as_an_interrupt() {
    let fx = make_workspace();
    let head_before = git(&fx.ws, &["rev-parse", "HEAD"]);
    let mut engine = CrashingEngine;
    let interrupted = || false;
    let mut confirm = |_prompt: &str| -> CliResult<bool> {
        panic!("the prompt must not be reached after a failed analysis")
    };

    let err = drive_write_with(&fx, &mut engine, true, &mut confirm, &interrupted);
    assert!(
        err.contains("migration engine failed"),
        "a genuine engine failure must be reported as the engine's; got: {err}"
    );
    assert!(
        err.contains("clang: fatal error: no such file"),
        "the engine's stderr tail must survive; got: {err}"
    );
    assert!(
        !err.contains("interrupted"),
        "an engine failure with no interrupt pending was attributed to one; got: {err}"
    );
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL
    );
    assert_eq!(git(&fx.ws, &["rev-parse", "HEAD"]), head_before);
    assert!(!fx.ws.join(LOCK_DIR).join(LOCK_FILE).exists());
}

/// The grace itself, without a wall: the flag flips only on the THIRD read
/// — after the per-TU poll (1) and the first read following the engine's
/// death (2) — which is the first re-read inside the grace. A single read
/// after the failure would report the engine's crash to a user who pressed
/// Ctrl-C a scheduling quantum earlier.
#[test]
fn an_interrupt_observed_only_within_the_grace_is_still_attributed() {
    let fx = make_workspace();
    let head_before = git(&fx.ws, &["rev-parse", "HEAD"]);
    let mut engine = CrashingEngine;
    let reads = std::cell::Cell::new(0usize);
    let interrupted = || {
        reads.set(reads.get() + 1);
        reads.get() >= 3
    };
    let mut confirm = |_prompt: &str| -> CliResult<bool> {
        panic!("the prompt must not be reached after an interrupted analysis")
    };

    let err = drive_write_with(&fx, &mut engine, true, &mut confirm, &interrupted);
    assert!(
        reads.get() >= 3,
        "the latch was read {} time(s) — the engine-failure path never re-read it",
        reads.get()
    );
    assert!(
        err.contains("interrupted — analysis stopped after 0 translation unit(s)"),
        "a flip that landed inside the grace must still be attributed; got: {err}"
    );
    assert!(
        !err.contains("migration engine failed"),
        "the user who pressed Ctrl-C was told the engine crashed; got: {err}"
    );
    assert_refused_at_the_consent_boundary(&fx, &head_before, &err);
}

/// Why the poll sits above the empty-plan exit, pinned: an interrupt during the analysis of a
/// plan that turns out EMPTY must refuse — never exit 0 through
/// `NothingToApply` as if nothing had been asked. A poll placed below that
/// exit passes every other arm.
#[test]
fn an_interrupt_with_an_empty_plan_refuses_instead_of_exiting_clean() {
    let fx = make_workspace();
    let head_before = git(&fx.ws, &["rev-parse", "HEAD"]);
    let latch = std::cell::Cell::new(false);
    let calls = std::cell::Cell::new(0usize);
    let mut engine = InterruptingEngine {
        inner: empty_engine_for(&fx),
        latch: &latch,
        calls: &calls,
    };
    let interrupted = || latch.get();
    let mut confirm = |_prompt: &str| -> CliResult<bool> {
        panic!("--yes never asks; the prompt must not be reached")
    };

    let err = drive_write_with(&fx, &mut engine, true, &mut confirm, &interrupted);
    assert!(
        err.contains("interrupted before the write batch"),
        "an interrupted run with nothing to apply must REFUSE, not exit clean; got: {err}"
    );
    assert_refused_at_the_consent_boundary(&fx, &head_before, &err);
}

/// ...and a `git status` the Ctrl-C killed must read as the interrupt, not
/// as "repair the repository": the index is corrupted the way the
/// status-failure arm does it, and the latch trips during the analysis.
#[test]
fn an_interrupt_with_a_failed_git_status_is_the_interrupt_not_a_broken_repo() {
    let fx = make_workspace();
    let head_before = git(&fx.ws, &["rev-parse", "HEAD"]);
    std::fs::write(fx.ws.join(".git/index"), b"garbage").unwrap();
    let latch = std::cell::Cell::new(false);
    let calls = std::cell::Cell::new(0usize);
    let mut engine = InterruptingEngine {
        inner: engine_for(&fx),
        latch: &latch,
        calls: &calls,
    };
    let interrupted = || latch.get();
    let mut confirm = |_prompt: &str| -> CliResult<bool> {
        panic!("--yes never asks; the prompt must not be reached")
    };

    let err = drive_write_with(&fx, &mut engine, true, &mut confirm, &interrupted);
    assert!(
        err.contains("interrupted before the write batch"),
        "the interrupt must be named ahead of the git gates; got: {err}"
    );
    assert!(
        !err.contains("could not be verified clean"),
        "a Ctrl-C'd `git status` was reported as a broken repository; got: {err}"
    );
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL
    );
    assert_eq!(git(&fx.ws, &["rev-parse", "HEAD"]), head_before);
    assert!(!fx.ws.join(LOCK_DIR).join(LOCK_FILE).exists());
}
