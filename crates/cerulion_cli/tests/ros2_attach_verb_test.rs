// SPDX-License-Identifier: AGPL-3.0-only
//! The `ros2` verb family: `cerulion ros
//! attach` is `cerulion ros2 attach`; there is no `ros` family (`ros2` carries
//! run / launch / attach) and no alias for the old spelling.
//! HARD BREAK: the old spelling is rejected, never silently accepted.
//!
//! Pins, over the REAL binary (`CARGO_BIN_EXE_cerulion`):
//! 1. the NEW spelling works — `ros2 attach --help` serves the attach flag
//!    surface, `ros2 --help` lists the verb, and a non-workspace invocation
//!    reaches the ENGINE path (a write from outside a workspace fails on
//!    workspace discovery, not on verb resolution; `--dry-run` does not);
//! 2. the OLD spelling fails LOUDLY — exit 2 (usage, the "your command line
//!    is stale" code) with stderr EXACTLY the migration message naming
//!    `cerulion ros2 attach`, byte-pinned against `main.rs`'s
//!    `ROS_VERB_MOVED_MSG` (the duplicate literal here is the pin: an edit to
//!    either side fails). Help flags are pinned too: `ros --help`, `ros -h`
//!    and `ros attach --help` all reach the stub, never clap's help page;
//! 3. the top-level help no longer lists a `ros` family.
//!
//! Restoring the old `ros attach` arm as an alias makes the old
//! spelling run attach instead of printing the migration message —
//! `the_old_spelling_fails_loudly_with_the_migration_message` fails on both
//! the exit code and the stderr text.
//!
//! Every subprocess is a plain, self-terminating call under an ISOLATED
//! `HOME` and a temp (non-workspace) cwd. The write-path arm stops before
//! discovery. The dry-run arm reaches discovery and is bounded by a short
//! `--timeout`. No `#[serial]`.

use std::path::Path;
use std::process::{Command, Output};

/// The exact migration message `main.rs` prints (its `ROS_VERB_MOVED_MSG`).
/// Kept as a duplicate literal ON PURPOSE: `cerulion_cli` has no lib target,
/// so the byte-for-byte duplication is what pins the shipped text.
const MIGRATION_MSG: &str = "`cerulion ros attach` has moved: the verb is now `cerulion ros2 attach` (the `ros` family was folded into `ros2`). Re-run the same invocation with `ros2` in place of `ros`.";

fn run(args: &[&str], home: &Path, cwd: &Path) -> Output {
    Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(args)
        .env("HOME", home)
        // Pin the iceoryx2 level to a parseable value: an unparseable ambient
        // `IOX2_LOG_LEVEL` makes `init_iceoryx_log_level_from_env` print a
        // fallback line, which would break the EXACT stderr oracle below for
        // an env reason rather than a verb reason.
        .env("IOX2_LOG_LEVEL", "error")
        .current_dir(cwd)
        .output()
        .expect("run cerulion")
}

/// `run` with the child's `TMPDIR` pinned INSIDE this test's tempdir, so
/// (a) the workspace-less `--dry-run` creates its exclusive root under a
/// directory the test owns (the leftover scan below then asserts about this
/// test's own files, never the machine's shared `/tmp` — the same hermetic
/// rule the `cerulion clean` tests follow), and (b) the child is isolated
/// from an ambient `TMPDIR` a CI host might set.
fn run_with_pinned_tmp(args: &[&str], home: &Path, cwd: &Path, tmp_root: &Path) -> Output {
    Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(args)
        .env("HOME", home)
        .env("TMPDIR", tmp_root)
        .env("IOX2_LOG_LEVEL", "error")
        .current_dir(cwd)
        .output()
        .expect("run cerulion")
}

fn isolated() -> (tempfile::TempDir, std::path::PathBuf) {
    let tmp = tempfile::tempdir().expect("tempdir");
    let home = tmp.path().join("home");
    std::fs::create_dir_all(&home).expect("home");
    (tmp, home)
}

/// The subcommand ENTRY names in a clap family help's `Commands:` section.
///
/// Line-aware on purpose: the family's doc
/// comment above the section names every verb WORD, so a substring check
/// over the whole help is a vacuous oracle — a deleted subcommand stays
/// green. An entry line in clap 4's help is the name at EXACTLY two-space
/// indent (`  attach  Discover…`); wrapped about text is indented deeper,
/// and the section ends at the next unindented header (`Options:`). Only
/// those first tokens count as entries here.
fn command_entries(help: &str) -> Vec<String> {
    let mut entries = Vec::new();
    let mut in_commands = false;
    for line in help.lines() {
        if line.trim_end() == "Commands:" {
            in_commands = true;
            continue;
        }
        if !in_commands {
            continue;
        }
        if !line.is_empty() && !line.starts_with(' ') {
            break; // the next section header ends the Commands section
        }
        if let Some(rest) = line.strip_prefix("  ") {
            if !rest.starts_with(' ') {
                if let Some(name) = rest.split_whitespace().next() {
                    entries.push(name.to_string());
                }
            }
        }
    }
    entries
}

#[test]
fn the_old_spelling_fails_loudly_with_the_migration_message() {
    let (tmp, home) = isolated();
    // The full old invocation, flags and all: the stub swallows the argv so
    // the user sees the migration error, never a clap complaint about the
    // old verb's own flags. The HELP shapes are pinned explicitly
    // because clap answers `--help` before returning a
    // parsed command, so without `disable_help_flag` on the stub,
    // `cerulion ros --help` would serve clap's help page (exit 0) instead of the
    // migration message.
    for argv in [
        vec!["ros", "attach", "--iface", "10.0.0.7"],
        vec!["ros", "attach"],
        vec!["ros"],
        vec!["ros", "--help"],
        vec!["ros", "-h"],
        vec!["ros", "attach", "--help"],
    ] {
        let out = run(&argv, &home, tmp.path());
        let stderr = String::from_utf8_lossy(&out.stderr);
        assert_eq!(
            out.status.code(),
            Some(2),
            "`cerulion {}` must exit 2 (usage — the stale-spelling refusal); \
             got {:?}\nstderr: {stderr}",
            argv.join(" "),
            out.status.code()
        );
        // Exact-match oracle, not `contains`: the migration refusal is the
        // WHOLE of stderr — nothing runs before the intercept (it sits above
        // `init_logging` and the login gate), so extra bytes here would mean
        // something leaked ahead of the refusal or the shipped text drifted.
        assert_eq!(
            stderr,
            format!("Error: {MIGRATION_MSG}\n"),
            "`cerulion {}` must print EXACTLY the migration message naming \
             `cerulion ros2 attach`",
            argv.join(" ")
        );
    }
}

#[test]
fn the_new_spelling_serves_help_and_reaches_the_engine_path() {
    let (tmp, home) = isolated();

    // `ros2 attach --help` works and carries the attach flag surface.
    let help = run(&["ros2", "attach", "--help"], &home, tmp.path());
    assert_eq!(
        help.status.code(),
        Some(0),
        "`ros2 attach --help` must exit 0"
    );
    let stdout = String::from_utf8_lossy(&help.stdout);
    for flag in ["--iface", "--domain", "--timeout", "--dry-run", "--yes"] {
        assert!(
            stdout.contains(flag),
            "`ros2 attach --help` must document `{flag}`; stdout:\n{stdout}"
        );
    }

    // The family help lists EXACTLY the four verbs plus
    // clap's auto `help`. A bare substring check
    // over the whole help would be a vacuous oracle — the family doc comment
    // above the `Commands:` section itself names all four verb words, so
    // deleting a subcommand would leave it green. The pin is SET EQUALITY over
    // the actual subcommand ENTRY names parsed line-aware from the
    // `Commands:` section (this file drives the real binary, so the
    // programmatic `Cli::command()` walk the completion wiring tests use is
    // out of reach — the bin crate has no lib target to import). Equality
    // also carries the absence control for free: a phantom entry (`ros`,
    // say) fails as loudly as a missing one.
    let fam = run(&["ros2", "--help"], &home, tmp.path());
    assert_eq!(fam.status.code(), Some(0), "`ros2 --help` must exit 0");
    let fam_out = String::from_utf8_lossy(&fam.stdout);
    let mut entries = command_entries(&fam_out);
    entries.sort();
    assert_eq!(
        entries,
        ["attach", "help", "launch", "migrate", "run"],
        "`ros2 --help` must list exactly the family's subcommand entries; \
         stdout:\n{fam_out}"
    );

    // `--iface` is still REQUIRED — the verb resolves and clap enforces the
    // moved arg surface (a usage error, not the migration stub).
    let noiface = run(&["ros2", "attach"], &home, tmp.path());
    assert_eq!(
        noiface.status.code(),
        Some(2),
        "missing --iface is usage (2)"
    );
    let noiface_err = String::from_utf8_lossy(&noiface.stderr);
    assert!(
        noiface_err.contains("--iface") && !noiface_err.contains(MIGRATION_MSG),
        "`ros2 attach` without --iface must be clap's required-arg error, \
         not the migration stub; stderr:\n{noiface_err}"
    );

    // The WRITE path from a non-workspace cwd reaches workspace discovery
    // (exit 1) and never the old-spelling migration text. `--yes` is what
    // would write; it still needs a workspace, and the check runs before
    // DDS discovery.
    let engine = run(
        &["ros2", "attach", "--iface", "10.0.0.7", "--yes"],
        &home,
        tmp.path(),
    );
    assert_eq!(
        engine.status.code(),
        Some(1),
        "a non-workspace `ros2 attach` that would write must fail on \
         workspace discovery, not at verb resolution"
    );
    let engine_err = String::from_utf8_lossy(&engine.stderr);
    assert!(
        engine_err.contains("Workspace not found") && !engine_err.contains(MIGRATION_MSG),
        "the failure must be workspace discovery reached THROUGH the verb, \
         with no migration text; stderr:\n{engine_err}"
    );

    // `--dry-run` writes nothing, so the same directory is not an error.
    // `10.0.0.7` is not a local interface, so discovery itself refuses. That
    // refusal is what shows the command got past the workspace gate. The
    // window is short so the arm stays bounded if a bind does wait.
    // `TMPDIR` is pinned inside this test's tempdir, so the exclusive root
    // the verb creates lands where the leftover scan below can see it (and
    // only it).
    let pinned_tmp = tmp.path().join("pinned-tmp");
    std::fs::create_dir_all(&pinned_tmp).expect("pinned tmpdir");
    let dry = run_with_pinned_tmp(
        &[
            "ros2",
            "attach",
            "--iface",
            "10.0.0.7",
            "--timeout",
            "0.05",
            "--dry-run",
        ],
        &home,
        tmp.path(),
        &pinned_tmp,
    );
    let dry_err = String::from_utf8_lossy(&dry.stderr);
    let dry_out = String::from_utf8_lossy(&dry.stdout);
    assert!(
        !dry_err.contains("Workspace not found") && !dry_err.contains(MIGRATION_MSG),
        "`ros2 attach --dry-run` outside a workspace must not be a workspace \
         error or the old spelling; stderr:\n{dry_err}"
    );
    // A COMPLETE-OUTCOME oracle, not the old disjunction with "any stdout":
    // the original `|| !dry_out.is_empty()` accepted ANY non-empty stdout, so
    // an unrelated early exit (one that prints something and never reaches
    // discovery) passed as the dry-run working. Instead, exactly two
    // completions are legitimate, each pinned with its own evidence —
    // (a) discovery REFUSED: exit 1, the refusal names DDS, no report; or
    // (b) discovery COMPLETED (e.g. an empty window): exit 0, the report
    // printed, ending with the automatic MIGRATION section every attach
    // report carries. An early exit (workspace error, clap usage exit 2, a
    // crash) matches neither arm.
    let code = dry.status.code();
    let refused = code == Some(1) && dry_err.contains("DDS discovery failed") && dry_out.is_empty();
    // The completed arm requires the report's HEADLINE too, not just its
    // MIGRATION tail: an implementation that printed only the tail with no
    // discovery report must not pass as "completed".
    let completed = code == Some(0)
        && dry_out.contains("DISCOVERED DDS TOPICS")
        && dry_out.contains("MIGRATION — what could run natively");
    assert!(
        refused || completed,
        "`ros2 attach --dry-run` outside a workspace must either refuse AT discovery \
         (exit 1, DDS named, no report) or complete and print the report (exit 0, with \
         its MIGRATION tail); got code={code:?}\nstderr:\n{dry_err}\nstdout:\n{dry_out}"
    );
    // The exclusively created temp root is dropped again on the failure
    // path: no `cerulion-attach-dry-run-*` entry may remain under the
    // PINNED temp dir (this test's own directory, never the machine's
    // shared temp) for a PID-reusing successor to read.
    let leftovers: Vec<_> = std::fs::read_dir(&pinned_tmp)
        .expect("pinned tempdir must still be listable")
        .filter_map(|e| e.ok())
        .filter(|e| {
            e.file_name()
                .to_str()
                .is_some_and(|n| n.starts_with("cerulion-attach-dry-run-"))
        })
        .collect();
    assert!(
        leftovers.is_empty(),
        "the workspace-less dry-run must remove its temporary root on every \
         exit path; found: {:?}",
        leftovers.iter().map(|e| e.file_name()).collect::<Vec<_>>()
    );
}

#[test]
fn the_top_level_help_no_longer_lists_a_ros_family() {
    let (tmp, home) = isolated();
    let out = run(&["--help"], &home, tmp.path());
    assert_eq!(out.status.code(), Some(0), "`cerulion --help` must exit 0");
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(
        stdout.lines().any(|l| l.trim().starts_with("ros2")),
        "`cerulion --help` must list `ros2`; stdout:\n{stdout}"
    );
    // Exact-token check: `ros2` contains `ros` as a substring, so the pin is
    // that no help line's FIRST word is exactly `ros`.
    assert!(
        !stdout
            .lines()
            .any(|l| l.split_whitespace().next() == Some("ros")),
        "the removed `ros` family must be hidden from `cerulion --help`; \
         stdout:\n{stdout}"
    );
}
