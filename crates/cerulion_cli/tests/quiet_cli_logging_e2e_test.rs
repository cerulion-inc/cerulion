// SPDX-License-Identifier: AGPL-3.0-only
//! Quiet-by-default CLI logging over the REAL binary
//! (`CARGO_BIN_EXE_cerulion`).
//!
//! Decision: a plain one-shot invocation (`cerulion topic list`,
//! `topic hz`, `schema list`, …) must print ONLY the command's output — the
//! lifecycle `tracing::info!` breadcrumbs (the discovery-ladder result, the
//! netd first-use spawn line) must NOT interleave with it. `-v` shows the
//! breadcrumbs; an explicit `RUST_LOG` always wins over the default. (The
//! transport-init breadcrumbs, "transport manager initialized" and the
//! startup dead-node sweep, ride `debug!` in every process and are not the
//! oracle here.)
//!
//! This file pins that contract behaviorally, through the real binary, using
//! the decision's own example command (`topic list`) and its
//! `cerulion_cli_engine::discovery_ladder` INFO breadcrumb. The ladder runs
//! UNCONDITIONALLY on the default (scouting-on) path and logs an INFO line at
//! the end of the gather in EITHER of two branches — "discovery ladder
//! gathered peers=N" when a peer is found, or "discovery ladder found no
//! robots …" when none is (the offline / clean-sandbox case). Both share the
//! **`discovery ladder` prefix**, so matching that prefix is environment- and
//! branch-independent (an oracle of the longer
//! `"discovery ladder gathered"` would be the found-a-peer branch only, and so
//! vacuous in a peerless CI). Each subprocess also runs with an ISOLATED
//! `HOME` + no `CERULION_PEERS`, so the host's `~/.cerulion/peers.json` peer
//! cache can never steer which branch fires. `topic list` needs no live topic,
//! no workspace, and no reachable robot, and self-exits after the bounded
//! gather.
//!
//! The classification of every verb (one-shot vs long-running) is exhaustively
//! pinned by the unit tests in `cerulion_cli::cli::verb_log_class_tests`, and
//! the (verbose, quiet) to level mapping — plus the default-path global ERROR
//! floor for non-`cerulion` targets — by `cerulion_core::init_logging_tests`.
//! This file pins the PLUMBING: that the chosen default actually reaches (or is
//! suppressed from) stderr. SCOPE: the long-running-keeps-`info`-by
//! default arm is pinned by composition of those two unit tests (a runtime verb
//! needs a built workspace and cdylib to emit a breadcrumb, which is out of
//! proportion for a plumbing pin); the existing `topic_introspect_cli_e2e_test`
//! and `graph_start_order_e2e_test` additionally exercise `graph run`'s info
//! breadcrumbs live.
//!
//! Each subprocess is a plain, self-terminating CLI call driven through
//! `Output` (`topic list` self-exits after the bounded discovery gather), so
//! there is no `#[serial]` and no iceoryx2 SHM mutation. The remote half is
//! best-effort and never hangs (a session/query failure is a loud note, then
//! exit 0), so the assertions hold offline.

use std::path::Path;
use std::process::{Command, Output};

/// The branch-independent prefix of the `cerulion_cli_engine::discovery_ladder`
/// INFO breadcrumb — matches BOTH "discovery ladder gathered peers=N" (peer
/// found) and "discovery ladder found no robots …" (peerless). One of the two
/// ALWAYS fires on the default `topic list` path, so this substring is a
/// hermetic one-shot INFO oracle regardless of what is on the LAN.
const LADDER_BREADCRUMB: &str = "discovery ladder";

/// Run `cerulion [-v] topic list` with the given `RUST_LOG` override (or the
/// inherited-then-cleared env when `rust_log` is `None`) under an ISOLATED
/// `HOME` (so the host peer cache can't influence the run), and wait for exit.
///
/// `topic list` self-terminates after its bounded discovery gather, so
/// `Output` (which blocks until the process exits) is safe — unlike the
/// streaming `topic echo`/`hz` verbs.
fn run_topic_list(home: &Path, verbose: bool, rust_log: Option<&str>) -> Output {
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    if verbose {
        cmd.arg("-v");
    }
    cmd.args(["topic", "list"]);
    // Hermetic peer-discovery env: an isolated HOME (empties the
    // `~/.cerulion/peers.json` cache) + no CERULION_PEERS override, so neither
    // the host's cache nor the runner's env can steer which ladder branch fires.
    cmd.env("HOME", home);
    cmd.env_remove("CERULION_PEERS");
    // Isolate from the ambient RUST_LOG the test runner may set.
    match rust_log {
        Some(spec) => {
            cmd.env("RUST_LOG", spec);
        }
        None => {
            cmd.env_remove("RUST_LOG");
        }
    }
    cmd.output().expect("failed to spawn cerulion binary")
}

fn stdout_of(out: &Output) -> String {
    String::from_utf8_lossy(&out.stdout).into_owned()
}

fn stderr_of(out: &Output) -> String {
    String::from_utf8_lossy(&out.stderr).into_owned()
}

/// A fresh isolated HOME for one subprocess (empty peer cache).
fn isolated_home() -> tempfile::TempDir {
    tempfile::tempdir().expect("create isolated HOME tempdir")
}

/// DEFAULT one-shot invocation: the INFO breadcrumb is SUPPRESSED (the quiet
/// `cerulion=warn` default), while the command's output still prints to stdout.
/// Diagnostics never leak into stdout either.
#[test]
fn default_one_shot_topic_list_is_quiet() {
    let home = isolated_home();
    let out = run_topic_list(home.path(), false, None);
    assert!(
        out.status.success(),
        "`cerulion topic list` must exit 0; stderr:\n{}",
        stderr_of(&out)
    );
    let stderr = stderr_of(&out);
    assert!(
        !stderr.contains(LADDER_BREADCRUMB),
        "the '{LADDER_BREADCRUMB}' INFO breadcrumb must be SUPPRESSED by default \
         (quiet one-shot verb); stderr was:\n{stderr}"
    );
    let stdout = stdout_of(&out);
    // The command's real output still prints (the "ONLY the command
    // output" requirement — a header, a topic list, or the empty-state note).
    assert!(
        !stdout.trim().is_empty(),
        "the command's output must still print to stdout; stdout was:\n{stdout}"
    );
    // Logs go to stderr, never stdout — the breadcrumb must not appear in the
    // data stream under any level.
    assert!(
        !stdout.contains(LADDER_BREADCRUMB),
        "the breadcrumb must NEVER pollute stdout; stdout was:\n{stdout}"
    );
}

/// `-v/--verbose` raises the default to `debug`, so the INFO breadcrumb SHOWS.
#[test]
fn verbose_flag_shows_breadcrumbs() {
    let home = isolated_home();
    let out = run_topic_list(home.path(), true, None);
    assert!(
        out.status.success(),
        "`cerulion -v topic list` must exit 0; stderr:\n{}",
        stderr_of(&out)
    );
    let stderr = stderr_of(&out);
    assert!(
        stderr.contains(LADDER_BREADCRUMB),
        "`-v` must SHOW the '{LADDER_BREADCRUMB}' breadcrumb; stderr was:\n{stderr}"
    );
}

/// An explicit global `RUST_LOG=info` RESTORES the breadcrumbs a one-shot
/// verb's quiet default suppresses — the "RUST_LOG always wins" contract.
#[test]
fn rust_log_info_restores_breadcrumbs() {
    let home = isolated_home();
    let out = run_topic_list(home.path(), false, Some("info"));
    assert!(
        out.status.success(),
        "must exit 0; stderr:\n{}",
        stderr_of(&out)
    );
    let stderr = stderr_of(&out);
    assert!(
        stderr.contains(LADDER_BREADCRUMB),
        "`RUST_LOG=info` must RESTORE the '{LADDER_BREADCRUMB}' breadcrumb the \
         quiet default suppresses; stderr was:\n{stderr}"
    );
}

/// A TARGET-SCOPED `RUST_LOG=cerulion_cli_engine=info` also restores the
/// breadcrumb — proving `RUST_LOG` wins over the more-specific `cerulion=warn`
/// default for the `cerulion` target (the reason the default is installed only
/// when `RUST_LOG` is unset, never layered on top).
#[test]
fn rust_log_target_scoped_info_restores_breadcrumbs() {
    let home = isolated_home();
    let out = run_topic_list(home.path(), false, Some("cerulion_cli_engine=info"));
    assert!(
        out.status.success(),
        "must exit 0; stderr:\n{}",
        stderr_of(&out)
    );
    let stderr = stderr_of(&out);
    assert!(
        stderr.contains(LADDER_BREADCRUMB),
        "a target-scoped `RUST_LOG=cerulion_cli_engine=info` must RESTORE the \
         '{LADDER_BREADCRUMB}' breadcrumb (RUST_LOG governs the cerulion target, \
         not the warn default); stderr was:\n{stderr}"
    );
}

/// An explicit `RUST_LOG=warn` is HONORED (not silently force-raised to
/// `info`): the INFO breadcrumb stays suppressed. The anti-tautology twin of
/// the restore arms — proves `RUST_LOG` genuinely GOVERNS the level rather than
/// the default always winning.
#[test]
fn rust_log_warn_stays_quiet() {
    let home = isolated_home();
    let out = run_topic_list(home.path(), false, Some("warn"));
    assert!(
        out.status.success(),
        "must exit 0; stderr:\n{}",
        stderr_of(&out)
    );
    let stderr = stderr_of(&out);
    assert!(
        !stderr.contains(LADDER_BREADCRUMB),
        "`RUST_LOG=warn` must be HONORED — the INFO breadcrumb stays suppressed; \
         stderr was:\n{stderr}"
    );
}
