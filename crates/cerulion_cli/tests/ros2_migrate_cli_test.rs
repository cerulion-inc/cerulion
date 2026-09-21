// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion ros2 migrate` — REAL-binary e2e over a stub engine.
//!
//! The engine-level tests (`cerulion_cli_engine/tests/ros2_migrate_test.rs`)
//! inject the analysis through the `MigrateEngine` trait; this file covers
//! the seams THEY structurally cannot: the production `ClangToolEngine`
//! spawn path (`CERULION_ROS2_MIGRATE_TOOL` → a real subprocess whose
//! stdout is parsed), the clap surface + exit codes of the real binary, and
//! the production colcon runner (a stub `colcon` executable planted on the
//! child's PATH — success and failure arms). The clang prover itself is
//! container-gated (tools/ros2_migrate/run_matrix.sh).
//!
//! Unix-only (shell-script stubs); parallel-safe — every test owns a
//! tempdir and mutates only the CHILD's environment.
#![cfg(unix)]

use std::os::unix::fs::PermissionsExt;
use std::path::{Path, PathBuf};
use std::process::Command;

const TALKER_ORIGINAL: &str = "#include \"demo.hpp\"\n\
\n\
void Talker::tick() {\n\
\x20 auto msg = std::make_unique<std_msgs::msg::String>();\n\
\x20 msg->data = \"hello\";\n\
\x20 pub_->publish(std::move(msg));\n\
}\n";

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
    /// The stub engine script (set as CERULION_ROS2_MIGRATE_TOOL).
    tool: PathBuf,
    /// Directory holding the stub `colcon` (prepended to the child PATH).
    bin_dir: PathBuf,
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

fn write_executable(path: &Path, body: &str) {
    std::fs::write(path, body).unwrap();
    std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o755)).unwrap();
}

/// A synthetic colcon ws + git repo + a stub engine emitting the canned
/// analysis + a stub `colcon` recording its argv (exit code per
/// `colcon_exit`).
fn make_fixture(colcon_exit: i32) -> Fixture {
    let tmp = tempfile::tempdir().expect("tempdir");
    let root = tmp.path().canonicalize().expect("canonical tempdir");
    let ws = root.join("ws");
    let pkg = ws.join("src/demo_pkg");
    std::fs::create_dir_all(pkg.join("src")).unwrap();
    std::fs::write(
        pkg.join("package.xml"),
        "<package format=\"3\"><name>demo_pkg</name></package>",
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
    std::fs::write(ws.join(".gitignore"), "/build/\n").unwrap();
    git(&ws, &["add", "-A"]);
    git(
        &ws,
        &[
            "-c",
            "user.email=test@example.invalid",
            "-c",
            "user.name=test",
            "commit",
            "-qm",
            "baseline",
        ],
    );

    let decl_off = TALKER_ORIGINAL.find(DECL_OLD).unwrap();
    let call_off = TALKER_ORIGINAL.find(CALL_OLD).unwrap();
    let analysis = serde_json::json!({
        "format": 1,
        "tool_version": "0.1.0",
        "file": talker.display().to_string(),
        "rewrites": [{
            "file": talker.display().to_string(),
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
        "candidates": []
    })
    .to_string();
    let analysis_file = root.join("analysis.json");
    std::fs::write(&analysis_file, analysis).unwrap();

    let bin_dir = root.join("bin");
    std::fs::create_dir_all(&bin_dir).unwrap();
    let tool = bin_dir.join("stub-migrate-engine");
    write_executable(
        &tool,
        &format!("#!/bin/sh\ncat '{}'\n", analysis_file.display()),
    );
    let colcon_log = root.join("colcon.argv");
    write_executable(
        &bin_dir.join("colcon"),
        &format!(
            "#!/bin/sh\necho \"$@\" > '{}'\nexit {}\n",
            colcon_log.display(),
            colcon_exit
        ),
    );
    Fixture {
        _tmp: tmp,
        ws,
        talker,
        tool,
        bin_dir,
    }
}

fn cerulion(fx: &Fixture, args: &[&str]) -> std::process::Output {
    let path = format!(
        "{}:{}",
        fx.bin_dir.display(),
        std::env::var("PATH").unwrap_or_default()
    );
    Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(args)
        .env("CERULION_ROS2_MIGRATE_TOOL", &fx.tool)
        .env("PATH", path)
        .output()
        .expect("cerulion runs")
}

#[test]
fn dry_run_over_the_real_spawn_path_prints_the_diff_and_writes_the_manifest() {
    let fx = make_fixture(0);
    let ws = fx.ws.display().to_string();
    let out = cerulion(&fx, &["ros2", "migrate", "--workspace", &ws]);
    assert!(
        out.status.success(),
        "stderr: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(stdout.contains("+  auto loaned = pub_->borrow_loaned_message();"));
    assert!(stdout.contains("dry-run: nothing was modified"));
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL
    );
    // The manifest CONTRACT, not just its presence — an empty or
    // malformed file must fail here.
    let manifest: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(fx.ws.join(".cerulion/ros2-migrate-manifest.json")).unwrap(),
    )
    .expect("the manifest must parse as JSON");
    assert_eq!(manifest["schema"], "cerulion-ros2-migrate-manifest");
    assert_eq!(manifest["version"], 1);
    assert_eq!(manifest["generated_by"], "dry-run");
    assert_eq!(manifest["counts"]["call_sites"], 1, "manifest: {manifest}");
    assert_eq!(manifest["counts"]["packages"], 1, "manifest: {manifest}");
    // `.get()`, not indexing: indexing a missing key ALSO
    // yields Null, so the indexed form would pass a manifest omitting the
    // field entirely — the contract is "present AND null".
    assert_eq!(manifest.get("decision"), Some(&serde_json::Value::Null));
}

#[test]
fn write_yes_applies_commits_and_runs_the_stub_colcon() {
    let fx = make_fixture(0);
    let ws = fx.ws.display().to_string();
    let out = cerulion(
        &fx,
        &["ros2", "migrate", "--workspace", &ws, "--write", "--yes"],
    );
    assert!(
        out.status.success(),
        "stderr: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_MIGRATED
    );
    assert!(fx.ws.join("cerulion-ros2-migration.patch").is_file());
    let subject = git(&fx.ws, &["log", "-1", "--format=%s"]);
    // Exact subject: a substring check would pass a wrong subject
    // and assert neither the rewrite count nor the API suffix.
    assert_eq!(
        subject, "ros2 migrate: rewrite 1 publish call site(s) to the loaned-message API",
        "subject: {subject}"
    );
    // The production colcon runner really spawned our stub with the
    // affected set.
    let argv = std::fs::read_to_string(fx.ws.parent().unwrap().join("colcon.argv"))
        .expect("stub colcon ran");
    assert_eq!(argv.trim(), "build --packages-select demo_pkg");
}

#[test]
fn a_failing_colcon_build_exits_nonzero_naming_the_revert_path() {
    let fx = make_fixture(3);
    let ws = fx.ws.display().to_string();
    let out = cerulion(
        &fx,
        &["ros2", "migrate", "--workspace", &ws, "--write", "--yes"],
    );
    assert_eq!(
        out.status.code(),
        Some(1),
        "a post-commit colcon failure must exit EXACTLY 1 (2 is a usage error, 69 a missing engine)"
    );
    let stderr = String::from_utf8_lossy(&out.stderr);
    let commit = git(&fx.ws, &["rev-parse", "HEAD"]);
    assert!(
        stderr.contains(&format!("git revert {commit}")),
        "stderr must name the revert path: {stderr}"
    );
    // The migration commit stays in place.
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_MIGRATED
    );
    // The failing verdict came FROM the stub colcon actually running — an
    // implementation that skipped the build but fabricated a matching
    // error would otherwise pass.
    let argv = std::fs::read_to_string(fx.ws.parent().unwrap().join("colcon.argv"))
        .expect("stub colcon ran");
    assert_eq!(argv.trim(), "build --packages-select demo_pkg");
}

#[test]
fn a_missing_engine_exits_69_with_build_instructions() {
    let fx = make_fixture(0);
    let ws = fx.ws.display().to_string();
    let path = format!(
        "{}:{}",
        fx.bin_dir.display(),
        std::env::var("PATH").unwrap_or_default()
    );
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["ros2", "migrate", "--workspace", &ws])
        .env_remove("CERULION_ROS2_MIGRATE_TOOL")
        .env("PATH", path)
        .output()
        .expect("cerulion runs");
    assert_eq!(out.status.code(), Some(69), "engine-not-built is exit 69");
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        stderr.contains("migration engine not built"),
        "stderr: {stderr}"
    );
    assert!(stderr.contains("tools/ros2_migrate/README.md"));
}

#[test]
fn yes_without_write_is_a_clap_usage_error() {
    let fx = make_fixture(0);
    let out = cerulion(&fx, &["ros2", "migrate", "--yes"]);
    assert_eq!(out.status.code(), Some(2), "clap usage error is exit 2");
}

#[test]
fn an_explicit_tool_env_pointing_at_a_missing_file_is_a_loud_69_never_a_fallback() {
    let fx = make_fixture(0);
    let ws = fx.ws.display().to_string();
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["ros2", "migrate", "--workspace", &ws])
        .env(
            "CERULION_ROS2_MIGRATE_TOOL",
            fx.bin_dir.join("no-such-engine"),
        )
        .output()
        .expect("cerulion runs");
    assert_eq!(out.status.code(), Some(69));
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        stderr.contains("no such file exists"),
        "an explicit path must fail loudly, not fall back: {stderr}"
    );
}

/// Run the real binary with the CHILD's git identity environment scrubbed —
/// the container condition (root, no global/system gitconfig, no
/// identity env), where a bare `git commit` refuses the auto-detected
/// `user@host.(none)` address. Child-env only: parallel-safe.
fn cerulion_bare_git_identity(fx: &Fixture, home: &Path, args: &[&str]) -> std::process::Output {
    let path = format!(
        "{}:{}",
        fx.bin_dir.display(),
        std::env::var("PATH").unwrap_or_default()
    );
    Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(args)
        .env("CERULION_ROS2_MIGRATE_TOOL", &fx.tool)
        .env("PATH", path)
        .env("HOME", home)
        .env("GIT_CONFIG_GLOBAL", "/dev/null")
        .env("GIT_CONFIG_SYSTEM", "/dev/null")
        .env_remove("EMAIL")
        .env_remove("GIT_AUTHOR_NAME")
        .env_remove("GIT_AUTHOR_EMAIL")
        .env_remove("GIT_COMMITTER_NAME")
        .env_remove("GIT_COMMITTER_EMAIL")
        .output()
        .expect("cerulion runs")
}

/// `cerulion_bare_git_identity` plus extra environment variables the
/// spawned verb inherits (the git-redirect and identity-fallback arms).
fn cerulion_bare_git_identity_with_env(
    fx: &Fixture,
    home: &Path,
    extra_env: &[(&str, String)],
    args: &[&str],
) -> std::process::Output {
    let path = format!(
        "{}:{}",
        fx.bin_dir.display(),
        std::env::var("PATH").unwrap_or_default()
    );
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    cmd.args(args)
        .env("CERULION_ROS2_MIGRATE_TOOL", &fx.tool)
        .env("PATH", path)
        .env("HOME", home)
        .env("GIT_CONFIG_GLOBAL", "/dev/null")
        .env("GIT_CONFIG_SYSTEM", "/dev/null")
        .env_remove("EMAIL")
        .env_remove("GIT_AUTHOR_NAME")
        .env_remove("GIT_AUTHOR_EMAIL")
        .env_remove("GIT_COMMITTER_NAME")
        .env_remove("GIT_COMMITTER_EMAIL");
    for (k, v) in extra_env {
        cmd.env(k, v);
    }
    cmd.output().expect("cerulion runs")
}

/// Git repository redirect: the verb's git
/// children inherited the caller's environment verbatim, and `-C` does not
/// neutralize `GIT_DIR` / `GIT_WORK_TREE` / `GIT_INDEX_FILE` — with those
/// aimed at a DECOY repository, status/add/commit targeted the decoy while
/// the source writes landed in the requested workspace: a "successful"
/// migration whose commit lived somewhere else. The verb now runs every
/// git child under an explicit keep-list policy; the redirection trio is
/// stripped, so the commit lands in the workspace and the decoy is
/// untouched.
#[test]
fn hostile_git_redirection_env_cannot_move_the_migration_commit() {
    let fx = make_fixture(0);
    let root = fx.ws.parent().unwrap();
    let home = root.join("empty-home");
    std::fs::create_dir_all(&home).unwrap();
    let decoy = root.join("decoy");
    std::fs::create_dir_all(&decoy).unwrap();
    git(&decoy, &["init", "-q"]);
    std::fs::write(decoy.join("d.txt"), "decoy").unwrap();
    git(&decoy, &["add", "-A"]);
    git(
        &decoy,
        &[
            "-c",
            "user.email=decoy@example.invalid",
            "-c",
            "user.name=decoy",
            "commit",
            "-qm",
            "decoy",
        ],
    );
    let decoy_head = git(&decoy, &["rev-parse", "HEAD"]);
    let ws_commits_before = git(&fx.ws, &["rev-list", "--count", "HEAD"]);
    let ws = fx.ws.display().to_string();
    let out = cerulion_bare_git_identity_with_env(
        &fx,
        &home,
        &[
            ("GIT_DIR", decoy.join(".git").display().to_string()),
            ("GIT_WORK_TREE", decoy.display().to_string()),
            (
                "GIT_INDEX_FILE",
                decoy.join(".git/index").display().to_string(),
            ),
        ],
        &["ros2", "migrate", "--workspace", &ws, "--write", "--yes"],
    );
    assert!(
        out.status.success(),
        "the migration must succeed under hostile redirection.\nstderr: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    assert_eq!(
        git(&fx.ws, &["rev-list", "--count", "HEAD"]),
        format!("{}", ws_commits_before.parse::<u32>().unwrap() + 1),
        "the migration commit must land in the WORKSPACE"
    );
    assert!(
        git(&fx.ws, &["status", "--porcelain", "--untracked-files=no"]).is_empty(),
        "no modified tracked files may remain in the workspace (the commit went elsewhere)"
    );
    assert_eq!(
        git(&decoy, &["rev-parse", "HEAD"]),
        decoy_head,
        "the decoy's HEAD must not move"
    );
    assert!(
        git(&decoy, &["status", "--porcelain"]).is_empty(),
        "the decoy's tree must stay clean"
    );
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_MIGRATED
    );
}

/// Identity fallback: a repository with a
/// configured `user.email` but NO `user.name`. Skipping the identity
/// fallback there (counting any configured email as a complete identity)
/// would make git refuse the commit — `user.useConfigOnly` makes that refusal
/// deterministic on every host — and roll the migration back for
/// nothing. The fallback is PER SIDE: the tool's NAME fills the missing
/// side while the configured email is kept.
#[test]
fn a_configured_email_without_a_name_commits_with_the_tool_name_and_the_configured_email() {
    let fx = make_fixture(0);
    let home = fx.ws.parent().unwrap().join("empty-home");
    std::fs::create_dir_all(&home).unwrap();
    git(&fx.ws, &["config", "user.email", "owner@example.invalid"]);
    git(&fx.ws, &["config", "user.useConfigOnly", "true"]);
    let ws = fx.ws.display().to_string();
    let out = cerulion_bare_git_identity(
        &fx,
        &home,
        &["ros2", "migrate", "--workspace", &ws, "--write", "--yes"],
    );
    assert!(
        out.status.success(),
        "a configured email without a name must not fail the commit.\nstderr: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    let ident = git(&fx.ws, &["log", "-1", "--format=%an <%ae> / %cn <%ce>"]);
    assert_eq!(
        ident,
        "cerulion ros2 migrate <owner@example.invalid> / \
         cerulion ros2 migrate <owner@example.invalid>",
        "the tool NAME fills the missing side; the configured EMAIL is kept"
    );
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_MIGRATED
    );
}

/// The container regression pin: with NO resolvable git identity
/// anywhere, `--write` must still commit — under the tool identity, which
/// names the generator. (Without the tool identity this fails in a container:
/// `git commit` dies with "Author identity unknown … unable to auto-detect
/// email address".)
#[test]
fn bare_identity_environment_commits_with_the_tool_identity() {
    let fx = make_fixture(0);
    let empty_home = fx.ws.parent().unwrap().join("empty-home");
    std::fs::create_dir_all(&empty_home).unwrap();
    let ws = fx.ws.display().to_string();
    let out = cerulion_bare_git_identity(
        &fx,
        &empty_home,
        &["ros2", "migrate", "--workspace", &ws, "--write", "--yes"],
    );
    assert!(
        out.status.success(),
        "a bare-identity environment must not fail the commit.\nstderr: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    let ident = git(&fx.ws, &["log", "-1", "--format=%an <%ae> / %cn <%ce>"]);
    assert_eq!(
        ident,
        "cerulion ros2 migrate <ros2-migrate@cerulion.invalid> / \
         cerulion ros2 migrate <ros2-migrate@cerulion.invalid>",
        "author AND committer must both carry the tool identity"
    );
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_MIGRATED
    );
}

/// Identity env vars that are SET but BLANK would outrank the
/// `-c` fallback with an EMPTY identity — the verb strips them from its git
/// children, so the commit still lands under the tool identity.
#[test]
fn blank_identity_env_vars_do_not_hollow_out_the_fallback() {
    let fx = make_fixture(0);
    let empty_home = fx.ws.parent().unwrap().join("empty-home3");
    std::fs::create_dir_all(&empty_home).unwrap();
    let ws = fx.ws.display().to_string();
    let path = format!(
        "{}:{}",
        fx.bin_dir.display(),
        std::env::var("PATH").unwrap_or_default()
    );
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["ros2", "migrate", "--workspace", &ws, "--write", "--yes"])
        .env("CERULION_ROS2_MIGRATE_TOOL", &fx.tool)
        .env("PATH", path)
        .env("HOME", &empty_home)
        .env("GIT_CONFIG_GLOBAL", "/dev/null")
        .env("GIT_CONFIG_SYSTEM", "/dev/null")
        .env_remove("EMAIL")
        // SET but blank — the blank-identity class.
        .env("GIT_AUTHOR_EMAIL", "")
        .env("GIT_COMMITTER_EMAIL", "   ")
        .env_remove("GIT_AUTHOR_NAME")
        .env_remove("GIT_COMMITTER_NAME")
        .output()
        .expect("cerulion runs");
    assert!(
        out.status.success(),
        "blank identity env vars must not fail or hollow the commit.\nstderr: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    let ident = git(&fx.ws, &["log", "-1", "--format=%ae / %ce"]);
    assert_eq!(
        ident,
        "ros2-migrate@cerulion.invalid / ros2-migrate@cerulion.invalid"
    );
}

/// The other half of the contract: a REPO-configured identity always wins —
/// the fallback is only for environments with none.
#[test]
fn a_configured_repo_identity_wins_over_the_fallback() {
    let fx = make_fixture(0);
    git(&fx.ws, &["config", "user.name", "Repo Owner"]);
    git(&fx.ws, &["config", "user.email", "owner@example.invalid"]);
    let empty_home = fx.ws.parent().unwrap().join("empty-home2");
    std::fs::create_dir_all(&empty_home).unwrap();
    let ws = fx.ws.display().to_string();
    let out = cerulion_bare_git_identity(
        &fx,
        &empty_home,
        &["ros2", "migrate", "--workspace", &ws, "--write", "--yes"],
    );
    assert!(
        out.status.success(),
        "stderr: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    let ident = git(&fx.ws, &["log", "-1", "--format=%ae / %ce"]);
    assert_eq!(
        ident, "owner@example.invalid / owner@example.invalid",
        "a configured identity must be used, never the tool fallback"
    );
}

// ---------------------------------------------------------------------------
// A Ctrl-C pressed BEFORE consent must not silently poison
// the run the user then approves.
//
// The CLI installs its `ctrlc` handler for `--write` before the analysis; the
// handler flips a one-shot flag on ctrlc's own thread, and it is installed
// with `SA_RESTART`, so a Ctrl-C landing while the `[y/N]` read waits does
// NOT end the read. The user types `y`; if the first
// consumer of the flag were the pre-write safepoint AFTER the workspace lock,
// it would tell a user who had just said yes "interrupted — nothing was written;
// nothing was migrated." and leave `.cerulion/` behind. The engine polls
// the latch at the consent boundary (per TU, at the head of the write
// section, after a `yes`) and refuses in a vocabulary that names the window
// it was in — and attributes an engine death under a TERMINAL Ctrl-C (which
// reaches the engine child too) to the interrupt rather than to a crash.
//
// These arms drive the REAL binary: the prompt path needs
// `stdin.is_terminal()`, so the two consent arms hand the child a
// pseudo-terminal SLAVE as stdin and type into the MASTER. The engine-level
// twins (`cerulion_cli_engine/tests/ros2_migrate_test.rs`) pin the same polls
// deterministically through the injected seams; these prove the wiring
// through `main.rs`, the real handler, a real terminal read, and — for the
// group arm — the real signal shape a terminal delivers.
// ---------------------------------------------------------------------------

/// The write-window refusal — the safepoint's string, imported from
/// the engine so a reword cannot disarm the guard silently. Every arm below
/// asserts its ABSENCE: an interrupt requested before consent must be
/// answered before the write window is entered, never by it.
use cerulion_cli_engine::ros2_migrate::{PATCH_FILENAME, PRE_WRITE_INTERRUPT_REFUSAL};
use cerulion_cli_engine::workspace_lock::{LOCK_DIR, LOCK_FILE};

/// The `ctrlc` closure that flips the flag runs on ctrlc's OWN thread, so
/// the flip lands some scheduling latency after `kill(2)` returns. This gap
/// is a STIMULUS margin, not an oracle: a flip that somehow landed after the
/// answer fails the arm LOUDLY (the write-safepoint string, plus `.cerulion/`
/// on disk), never passes it. If it ever flakes under load, raise it — the
/// direction is always red.
const LATCH_SETTLE: std::time::Duration = std::time::Duration::from_secs(1);

const ARM_DEADLINE: std::time::Duration = std::time::Duration::from_secs(60);

/// SIGKILL + reap on drop so a panicking arm never leaks a `cerulion` child
/// parked on a prompt. It reaches ONLY that child: the stub engine is its
/// grandchild (spawned by the engine seam with no process group), which is
/// why the pausing stub bounds its own wait instead of relying on this.
struct ChildGuard(std::process::Child);
impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

fn wait_bounded(
    child: &mut std::process::Child,
    timeout: std::time::Duration,
) -> Option<std::process::ExitStatus> {
    let start = std::time::Instant::now();
    loop {
        match child.try_wait().expect("try_wait") {
            Some(status) => return Some(status),
            None if start.elapsed() > timeout => return None,
            None => std::thread::sleep(std::time::Duration::from_millis(20)),
        }
    }
}

/// Poll `cond` until it holds, panicking with BOTH captured streams if the
/// child exits first or the deadline passes — a child that refused at a gate
/// says why on stderr, and a bare timeout would discard it.
fn wait_until(
    what: &str,
    timeout: std::time::Duration,
    child: &mut std::process::Child,
    out: &Capture,
    err: &Capture,
    mut cond: impl FnMut() -> bool,
) {
    let start = std::time::Instant::now();
    while !cond() {
        if let Some(status) = child.try_wait().expect("try_wait") {
            panic!(
                "the child exited ({status}) before {what};\nstdout:\n{}\nstderr:\n{}",
                out.text(),
                err.text()
            );
        }
        assert!(
            start.elapsed() < timeout,
            "{what} was not observed within {timeout:?};\nstdout:\n{}\nstderr:\n{}",
            out.text(),
            err.text()
        );
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
}

fn send_sigint(pid: u32) {
    // SAFETY: kill(2) with a live child's pid and a signal number; no memory
    // is touched.
    unsafe {
        libc::kill(pid as libc::pid_t, libc::SIGINT);
    }
}

/// SIGINT to a whole process GROUP — what a terminal's Ctrl-C does. `pgid`
/// is the group leader's pid (the child was spawned with `process_group(0)`).
fn send_sigint_to_group(pgid: u32) {
    // SAFETY: kill(2) with a negated live group id and a signal number; the
    // test process is not in that group.
    unsafe {
        libc::kill(-(pgid as libc::pid_t), libc::SIGINT);
    }
}

/// A pseudo-terminal pair. The SLAVE becomes the child's stdin — the real
/// `[y/N]` path runs only when `stdin.is_terminal()` — and the MASTER is what
/// the test types into.
struct Pty {
    master: std::fs::File,
    slave: std::fs::File,
}

/// `ptsname(3)` returns a pointer into a process-wide static buffer, and this
/// binary runs its arms in parallel: two `open_pty` calls interleaving
/// between `ptsname` and the copy-out would hand one arm the OTHER's slave.
/// Minting is serialised here; the arms themselves stay parallel.
static PTY_MINT: std::sync::Mutex<()> = std::sync::Mutex::new(());

fn open_pty() -> Pty {
    use std::ffi::CStr;
    use std::os::unix::io::{AsRawFd, FromRawFd};
    let _minting = PTY_MINT
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    // SAFETY: plain POSIX pty calls. The master is owned by a `File` the
    // instant `posix_openpt` returns it, so no later assertion can leak it;
    // `ptsname`'s static buffer is copied out under `PTY_MINT` before any
    // other pty call. `O_NOCTTY` everywhere: the test process must not
    // acquire the pty as ITS controlling terminal.
    unsafe {
        let master_fd = libc::posix_openpt(libc::O_RDWR | libc::O_NOCTTY);
        assert!(
            master_fd >= 0,
            "posix_openpt: {}",
            std::io::Error::last_os_error()
        );
        let master = std::fs::File::from_raw_fd(master_fd);
        assert_eq!(
            libc::grantpt(master.as_raw_fd()),
            0,
            "grantpt: {}",
            std::io::Error::last_os_error()
        );
        assert_eq!(
            libc::unlockpt(master.as_raw_fd()),
            0,
            "unlockpt: {}",
            std::io::Error::last_os_error()
        );
        let name = libc::ptsname(master.as_raw_fd());
        assert!(
            !name.is_null(),
            "ptsname: {}",
            std::io::Error::last_os_error()
        );
        let name = CStr::from_ptr(name).to_owned();
        let slave_fd = libc::open(name.as_ptr(), libc::O_RDWR | libc::O_NOCTTY);
        assert!(
            slave_fd >= 0,
            "open {name:?}: {}",
            std::io::Error::last_os_error()
        );
        Pty {
            master,
            slave: std::fs::File::from_raw_fd(slave_fd),
        }
    }
}

/// The seams a PAUSING stub engine exposes: one line appended to `count`
/// per invocation, `started` created when an invocation begins, and every
/// invocation blocks until `go` exists — the window a Ctrl-C during analysis
/// lands in. The block is BOUNDED — 400 polls of at least 50 ms each, i.e.
/// 20 s idle and a few minutes on a loaded box (each poll forks a `sleep`),
/// well inside `ARM_DEADLINE` — because the stub is `cerulion`'s grandchild,
/// out of `ChildGuard`'s reach, and an arm that panicked before releasing it
/// would otherwise leave a shell spinning on a marker in a tempdir that no
/// longer exists. The arms release it within `LATCH_SETTLE`.
struct EngineSeams {
    count: PathBuf,
    started: PathBuf,
    go: PathBuf,
}

impl EngineSeams {
    fn invocations(&self) -> usize {
        std::fs::read_to_string(&self.count)
            .map(|s| s.lines().count())
            .unwrap_or(0)
    }
}

/// `make_fixture(0)` with `tus` translation units in the compile database
/// (extra copies of the talker, committed so the tree stays clean) and the
/// stub engine replaced by a pausing one. The stub traps `INT` to an exit so
/// that a process-group SIGINT kills it DETERMINISTICALLY: a shell waiting on
/// its `sleep` defers the signal, and when it lands inside that window the
/// shell sometimes keeps running (measured on bash 3.2: 9 survivals in 80
/// group SIGINTs). A surviving engine turns the terminal-Ctrl-C arm into the
/// directed-SIGINT shape, where the write head rather than the engine's
/// death reports the interrupt. The arms that signal `cerulion` alone never
/// reach the stub, so the trap changes nothing for them.
fn make_pausing_fixture(tus: usize) -> (Fixture, EngineSeams) {
    let fx = make_fixture(0);
    let root = fx.ws.parent().unwrap().to_path_buf();
    let db_dir = fx.ws.join("build/demo_pkg");
    let mut entries = Vec::new();
    for i in 0..tus {
        let file = if i == 0 {
            fx.talker.clone()
        } else {
            let extra = fx.talker.with_file_name(format!("talker{i}.cpp"));
            std::fs::write(&extra, TALKER_ORIGINAL).unwrap();
            extra
        };
        entries.push(format!(
            "{{\"directory\": \"{}\", \"command\": \"clang++ -c {}\", \"file\": \"{}\"}}",
            db_dir.display(),
            file.display(),
            file.display()
        ));
    }
    std::fs::write(
        db_dir.join("compile_commands.json"),
        format!("[{}]", entries.join(", ")),
    )
    .unwrap();
    if tus > 1 {
        git(&fx.ws, &["add", "-A"]);
        git(
            &fx.ws,
            &[
                "-c",
                "user.email=test@example.invalid",
                "-c",
                "user.name=test",
                "commit",
                "-qm",
                "more translation units",
            ],
        );
    }
    let seams = EngineSeams {
        count: root.join("engine.count"),
        started: root.join("engine.started"),
        go: root.join("engine.go"),
    };
    write_executable(
        &fx.tool,
        &format!(
            "#!/bin/sh\ntrap 'exit 130' INT\necho x >> '{}'\n: > '{}'\ni=0\nwhile [ ! -e '{}' ] && [ \"$i\" -lt 400 ]; do sleep 0.05; i=$((i+1)); done\ncat '{}'\n",
            seams.count.display(),
            seams.started.display(),
            seams.go.display(),
            root.join("analysis.json").display()
        ),
    );
    (fx, seams)
}

/// A child stream drained on its own thread, incrementally, so an arm can
/// watch for the prompt while the child is still blocked on it. `text()` is
/// a live snapshot for polling; `finish()` waits (bounded) for the reader to
/// reach EOF first, so the text an assertion trusts is the WHOLE stream and
/// not whatever had been copied when the child exited — and a read error is
/// a loud failure, never a silently shortened stream.
struct Capture {
    buf: std::sync::Arc<std::sync::Mutex<Vec<u8>>>,
    eof: std::sync::mpsc::Receiver<std::io::Result<()>>,
}

impl Capture {
    fn of(mut reader: impl std::io::Read + Send + 'static) -> Capture {
        let buf = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
        let sink = buf.clone();
        let (eof_tx, eof) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let mut chunk = [0u8; 4096];
            let outcome = loop {
                match reader.read(&mut chunk) {
                    Ok(0) => break Ok(()),
                    Err(e) => break Err(e),
                    Ok(n) => sink.lock().unwrap().extend_from_slice(&chunk[..n]),
                }
            };
            let _ = eof_tx.send(outcome);
        });
        Capture { buf, eof }
    }

    fn text(&self) -> String {
        String::from_utf8_lossy(&self.buf.lock().unwrap()).into_owned()
    }

    /// Bounded by `ARM_DEADLINE`: the pipe's write ends belong to the
    /// `cerulion` child alone (its engine seam pipes the stub's streams
    /// separately), so EOF follows the child's exit — a wait past the
    /// deadline means something else holds the pipe, and that is a failure
    /// with a message rather than a hung binary.
    fn finish(&mut self) -> String {
        match self.eof.recv_timeout(ARM_DEADLINE) {
            Ok(Ok(())) => {}
            Ok(Err(e)) => panic!("the capture reader failed before EOF: {e}"),
            Err(_) => panic!(
                "the capture reader did not reach EOF within {ARM_DEADLINE:?} — something \
                 other than the exited child still holds the pipe"
            ),
        }
        self.text()
    }
}

/// Spawn `cerulion ros2 migrate --workspace <ws> --write <extra..>` with the
/// given stdin, stdout + stderr captured incrementally. `own_group` puts the
/// child at the head of a NEW process group — the shape a terminal gives a
/// foreground job — so a group-wide SIGINT reaches its engine child too.
fn spawn_write(
    fx: &Fixture,
    stdin: std::process::Stdio,
    extra: &[&str],
    own_group: bool,
) -> (ChildGuard, Capture, Capture) {
    use std::os::unix::process::CommandExt;
    let path = format!(
        "{}:{}",
        fx.bin_dir.display(),
        std::env::var("PATH").unwrap_or_default()
    );
    let ws = fx.ws.display().to_string();
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    cmd.args(["ros2", "migrate", "--workspace", &ws, "--write"])
        .args(extra)
        .env("CERULION_ROS2_MIGRATE_TOOL", &fx.tool)
        .env("PATH", path)
        .stdin(stdin)
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped());
    if own_group {
        cmd.process_group(0);
    }
    let mut child = cmd.spawn().expect("cerulion spawns");
    let out = Capture::of(child.stdout.take().expect("piped stdout"));
    let err = Capture::of(child.stderr.take().expect("piped stderr"));
    (ChildGuard(child), out, err)
}

/// What every pre-consent refusal must leave behind: nothing. The last two
/// checks discriminate WHERE the refusal came from — the workspace
/// lock is taken only after consent and its lock FILE is what taking it
/// creates, so the file's absence proves the refusal came BEFORE the write
/// batch was entered rather than from a safepoint inside it.
fn assert_refused_before_the_write_window(fx: &Fixture, head_before: &str, stderr: &str) {
    assert_eq!(
        std::fs::read_to_string(&fx.talker).unwrap(),
        TALKER_ORIGINAL,
        "a source file was rewritten; stderr: {stderr}"
    );
    assert_eq!(
        git(&fx.ws, &["rev-parse", "HEAD"]),
        head_before,
        "HEAD moved; stderr: {stderr}"
    );
    assert!(
        !fx.ws.join(PATCH_FILENAME).exists(),
        "the patch file was written; stderr: {stderr}"
    );
    // The lock FILE, not its directory: the manifest lives in the same
    // directory, so the directory alone would misattribute a manifest write.
    assert!(
        !fx.ws.join(LOCK_DIR).join(LOCK_FILE).exists(),
        "the workspace lock file exists, so the lock was TAKEN — the refusal came from a \
         write-window safepoint after consent, not from the consent boundary; stderr: {stderr}"
    );
    assert!(
        !stderr.contains(PRE_WRITE_INTERRUPT_REFUSAL),
        "the refusal is the write-safepoint string — the interrupt was consumed \
         AFTER the user's yes instead of at the consent boundary; stderr: {stderr}"
    );
}

/// THE headline: the interrupt lands while the `[y/N]` read is waiting, the
/// user then types `y`, and the run must refuse in the prompt-window
/// vocabulary — having taken no lock and written nothing — instead of
/// accepting the yes and reporting "interrupted" from inside the write
/// window.
#[test]
fn a_ctrl_c_while_the_consent_prompt_waits_refuses_the_yes_that_follows_it() {
    let fx = make_fixture(0);
    let head_before = git(&fx.ws, &["rev-parse", "HEAD"]);
    let mut pty = open_pty();
    let slave = pty.slave.try_clone().expect("dup the slave for the child");
    let (mut child, mut out, mut err) =
        spawn_write(&fx, std::process::Stdio::from(slave), &[], false);

    wait_until(
        "the [y/N] prompt",
        ARM_DEADLINE,
        &mut child.0,
        &out,
        &err,
        || out.text().contains("[y/N]"),
    );
    send_sigint(child.0.id());
    std::thread::sleep(LATCH_SETTLE);
    use std::io::Write as _;
    pty.master.write_all(b"y\n").expect("type the answer");

    let status = wait_bounded(&mut child.0, ARM_DEADLINE).unwrap_or_else(|| {
        panic!(
            "the run did not exit after the answer; stdout:\n{}\nstderr:\n{}",
            out.text(),
            err.text()
        )
    });
    let _ = out.finish();
    let stderr = err.finish();
    assert_eq!(status.code(), Some(1), "stderr: {stderr}");
    assert!(
        stderr.contains("interrupted while the consent prompt was waiting"),
        "the refusal must name the PROMPT window; stderr: {stderr}"
    );
    assert!(stderr.contains("nothing was committed"), "stderr: {stderr}");
    assert_refused_before_the_write_window(&fx, &head_before, &stderr);
}

/// The interrupt lands during the ANALYSIS (the engine is parked mid-TU);
/// the run must refuse before the prompt is ever asked — a question whose
/// "yes" cannot be honoured is not asked.
#[test]
fn a_ctrl_c_during_analysis_refuses_before_the_prompt_is_asked() {
    let (fx, seams) = make_pausing_fixture(1);
    let head_before = git(&fx.ws, &["rev-parse", "HEAD"]);
    let mut pty = open_pty();
    let slave = pty.slave.try_clone().expect("dup the slave for the child");
    let (mut child, mut out, mut err) =
        spawn_write(&fx, std::process::Stdio::from(slave), &[], false);

    wait_until(
        "the engine to start",
        ARM_DEADLINE,
        &mut child.0,
        &out,
        &err,
        || seams.started.exists(),
    );
    send_sigint(child.0.id());
    std::thread::sleep(LATCH_SETTLE);
    // Pre-type the answer BEFORE releasing the engine: if a prompt does appear
    // (the regression), the child reads this instead of wedging on an empty
    // terminal, and the arm fails on the prompt's presence rather than on a
    // timeout.
    use std::io::Write as _;
    pty.master.write_all(b"y\n").expect("pre-type the answer");
    std::fs::write(&seams.go, b"").expect("release the engine");

    let status = wait_bounded(&mut child.0, ARM_DEADLINE).unwrap_or_else(|| {
        panic!(
            "the run did not exit; stdout:\n{}\nstderr:\n{}",
            out.text(),
            err.text()
        )
    });
    let stdout = out.finish();
    let stderr = err.finish();
    assert_eq!(status.code(), Some(1), "stderr: {stderr}");
    // The positive stdout anchor that makes the negative one below meaningful:
    // the report is printed before the write section's poll refuses, so a
    // stdout that lacks the header was not read at all.
    assert!(
        stdout.contains("cerulion ros2 migrate — loaned-message API migration"),
        "the report header never reached stdout; stdout:\n{stdout}"
    );
    assert!(
        !stdout.contains("[y/N]"),
        "the prompt was ASKED after an interrupt had already been requested; stdout:\n{stdout}"
    );
    assert!(
        stderr.contains("interrupted before consent"),
        "the refusal must name the pre-consent window; stderr: {stderr}"
    );
    assert_eq!(
        seams.invocations(),
        1,
        "the single TU must have been analysed exactly once"
    );
    assert_refused_before_the_write_window(&fx, &head_before, &stderr);
}

/// The interrupt lands while TU 1 of 2 is being analysed; the per-TU poll
/// must stop the loop before TU 2 is handed to the engine — a real clang run
/// over a workspace is minutes, and an interrupt that is only honoured at the
/// end of it is not honoured.
#[test]
fn a_ctrl_c_during_analysis_stops_the_per_tu_loop_before_the_next_tu() {
    let (fx, seams) = make_pausing_fixture(2);
    let head_before = git(&fx.ws, &["rev-parse", "HEAD"]);
    let (mut child, mut out, mut err) =
        spawn_write(&fx, std::process::Stdio::null(), &["--yes"], false);

    wait_until(
        "the engine to start on TU 1",
        ARM_DEADLINE,
        &mut child.0,
        &out,
        &err,
        || seams.started.exists(),
    );
    send_sigint(child.0.id());
    std::thread::sleep(LATCH_SETTLE);
    std::fs::write(&seams.go, b"").expect("release the engine");

    let status = wait_bounded(&mut child.0, ARM_DEADLINE).unwrap_or_else(|| {
        panic!(
            "the run did not exit; stdout:\n{}\nstderr:\n{}",
            out.text(),
            err.text()
        )
    });
    let _ = out.finish();
    let stderr = err.finish();
    assert_eq!(status.code(), Some(1), "stderr: {stderr}");
    assert_eq!(
        seams.invocations(),
        1,
        "the engine was handed TU 2 after the interrupt — the per-TU poll is missing; \
         stderr: {stderr}"
    );
    assert!(
        stderr.contains("interrupted — analysis stopped after 1 translation unit(s)"),
        "the refusal must name the analysis window and the count; stderr: {stderr}"
    );
    assert_refused_before_the_write_window(&fx, &head_before, &stderr);
}

/// THE shape a user actually produces: a terminal Ctrl-C goes to the whole
/// foreground process group, so the engine child dies too (it has no
/// handler). Its death must be reported as the interrupt — not as a prover
/// crash the user is told to investigate — and nothing is written.
#[test]
fn a_terminal_ctrl_c_that_kills_the_engine_is_reported_as_the_interrupt() {
    let (fx, seams) = make_pausing_fixture(1);
    let head_before = git(&fx.ws, &["rev-parse", "HEAD"]);
    let (mut child, mut out, mut err) =
        spawn_write(&fx, std::process::Stdio::null(), &["--yes"], true);

    wait_until(
        "the engine to start",
        ARM_DEADLINE,
        &mut child.0,
        &out,
        &err,
        || seams.started.exists(),
    );
    // The child leads its own group, so this is exactly what the terminal
    // delivers: cerulion's handler flips its flag; the stub engine dies. No
    // settle is possible here — the race is INSIDE cerulion, between its
    // engine seam observing the death and its ctrlc thread flipping the flag
    // — which is what the engine's own bounded grace exists for; a loss of
    // that race fails this arm loudly, never passes it.
    send_sigint_to_group(child.0.id());

    let status = wait_bounded(&mut child.0, ARM_DEADLINE).unwrap_or_else(|| {
        panic!(
            "the run did not exit after the group SIGINT; stdout:\n{}\nstderr:\n{}",
            out.text(),
            err.text()
        )
    });
    let _ = out.finish();
    let stderr = err.finish();
    assert_eq!(status.code(), Some(1), "stderr: {stderr}");
    assert!(
        stderr.contains("interrupted — analysis stopped after 0 translation unit(s)"),
        "the engine's death under a terminal Ctrl-C must be attributed to the interrupt; \
         stderr: {stderr}"
    );
    assert!(
        !stderr.contains("migration engine failed"),
        "the user who pressed Ctrl-C was told the engine crashed; stderr: {stderr}"
    );
    assert_refused_before_the_write_window(&fx, &head_before, &stderr);
}
