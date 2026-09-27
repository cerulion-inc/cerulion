// SPDX-License-Identifier: AGPL-3.0-only
//! The binary login-gate HOOK end to end over the REAL `cerulion` binary: the
//! only coverage of `main`'s `command_needs_identity` + `ensure_login_gate`
//! wiring firing against a real command. The engine-level e2e
//! (`cerulion_cli_engine/tests/login_flow_e2e_test.rs`) drives
//! `ensure_login_gate` directly against a real account service and never spawns
//! the binary.
//!
//! Self-contained: each test isolates `CERULION_HOME` to its own tempdir (never
//! reads the real `~/.cerulion`) and runs from a non-workspace cwd, so it needs
//! no fixtures and touches no shared state (parallel-safe, no `#[serial]`).
//!
//! ONE arm is not purely local, and cannot be. `clean` is exempt precisely
//! because it sweeps THIS machine's iceoryx2 bookkeeping, so the only way to
//! prove the exemption at the boundary a user feels is to let that sweep run.
//! It runs in the `--report-only` form, which removes no state file and no
//! orphan port tag; the dead-node sweep it still performs is the same one
//! `graph run` performs at its own startup and reaches provably dead nodes
//! only, so nothing a concurrent test owns is reachable by it and the file
//! stays parallel-safe.
//!
//! ## Which arm runs here, and why
//!
//! A spawned process gets pipes for stdin and stderr, so every test in this file
//! takes the NOT-AT-A-TERMINAL arm: the machine has never signed in, nobody can
//! read a device code, and the gate refuses at once instead of starting a ten
//! minute poll. That is the arm every script, CI job and service hits, and it is
//! the one worth pinning over the real binary.
//!
//! The at-a-terminal arm needs a pty, which would mean a new dependency; it is
//! pinned instead at the engine level, where the terminal answer is passed in
//! (`ensure_login_gate_with`) and the device flow runs against a real local
//! account service.

use std::path::Path;
use std::process::Command;
use std::time::{Duration, Instant};

use cerulion_cli_engine::auth;
use cerulion_cli_engine::login_cmd::EXIT_AUTH_REQUIRED;

/// A spawned `cerulion <args>`. `seeded` provisions the isolated
/// `CERULION_HOME` with the logged-in-ever marker (the sanctioned way a test or
/// a CI job supplies the state the gate reads); otherwise the home is empty and
/// the machine has never signed in.
///
/// `extra_env` sets variables the test wants observed. The account service is
/// always a reserved dead port: a run that dialled it would either fail on the
/// connection or sit in the poll loop, so no test here can pass by accident
/// against a service that answered.
///
/// `CERULION_LOGIN_GATE` is removed on every spawn. The workspace cargo
/// configuration sets it to `off` for this repository's own runs, so a test
/// binary inherits it and would hand it to every child. This file is the one
/// place that must see the gate as a user's machine sees it, so each case puts
/// back exactly the value it wants to prove something about.
fn run_cerulion(seeded: bool, extra_env: &[(&str, &str)], args: &[&str]) -> (Option<i32>, String) {
    let home = tempfile::tempdir().unwrap();
    if seeded {
        auth::seed_logged_in_at(home.path(), "acct-login-gate-e2e").unwrap();
    }
    let (code, _stdout, stderr) = run_cerulion_at(home.path(), extra_env, args);
    (code, stderr)
}

/// The same spawn against a home the CALLER owns, handing back stdout as well.
///
/// Two arms need more than the refusal on stderr: one has to read the report an
/// exempt verb prints, and both have to look at the home afterwards to show the
/// run left no local identity behind. Everything else about the spawn — the
/// non-workspace cwd, the removed `CERULION_LOGIN_GATE`, the dead account
/// service — is [`run_cerulion`]'s, because this is that function's body.
fn run_cerulion_at(
    home: &Path,
    extra_env: &[(&str, &str)],
    args: &[&str],
) -> (Option<i32>, String, String) {
    let cwd = tempfile::tempdir().unwrap(); // NOT a workspace
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    cmd.args(args)
        .current_dir(cwd.path())
        .env_remove("CERULION_LOGIN_GATE")
        .env("CERULION_HOME", home)
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1");
    for (k, v) in extra_env {
        cmd.env(k, v);
    }
    let out = cmd.output().expect("spawn cerulion");
    (
        out.status.code(),
        String::from_utf8_lossy(&out.stdout).to_string(),
        String::from_utf8_lossy(&out.stderr).to_string(),
    )
}

/// Hand oracle for the refusal, kept as literals rather than read back off the
/// constant the binary prints: a test that compared the output with the string
/// the code holds would agree with any wording, including none.
fn assert_is_the_refusal(stderr: &str) {
    assert!(
        stderr.contains("never signed in to a Cerulion account"),
        "the refusal must state the condition; stderr={stderr}"
    );
    assert!(
        stderr.contains("cerulion login"),
        "the refusal must name the fix for a person; stderr={stderr}"
    );
    assert!(
        stderr.contains("CERULION_HOME"),
        "the refusal must name the fix for automation; stderr={stderr}"
    );
}

/// The refusal a user reads, WHOLE, transcribed by hand from the two lines the
/// binary prints (the `Error: ` frame `main` puts in front of every `CliError`
/// included). Written out here rather than composed from the constant the code
/// holds: a comparison against that constant would agree with any wording,
/// which is the failure this oracle exists to catch. Whoever rewords the
/// refusal reworded what a user reads, and retypes it here.
const REFUSAL_A_USER_READS: &str = "Error: This machine has never signed in to a Cerulion account, and every command needs one.\nRun `cerulion login` once in a terminal, or for automation point CERULION_HOME at a directory holding the state of a machine that did.";

/// The file a real sign-in writes into `CERULION_HOME`. A run that neither
/// signed in nor was asked to leaves the home exactly as it found it, and an
/// exemption that worked by quietly minting one would be a different change
/// than the one under test.
fn assert_no_identity_was_written(home: &Path) {
    let auth = home.join("auth.json");
    assert!(
        !auth.exists(),
        "the run wrote {} — neither an exempt verb nor a refused one may leave \
         this machine signed in",
        auth.display()
    );
}

#[test]
fn a_machine_that_never_signed_in_is_refused_before_the_command_runs() {
    let started = Instant::now();
    let (code, stderr) = run_cerulion(false, &[], &["node", "list"]);
    assert_is_the_refusal(&stderr);
    assert_eq!(
        code,
        Some(i32::from(EXIT_AUTH_REQUIRED)),
        "an auth refusal exits 7, distinct from a runtime failure; stderr={stderr}"
    );
    // The gate fired before the verb's own logic: outside a workspace `node
    // list` would have said this, and it never got the chance.
    assert!(
        !stderr.contains("Workspace not found"),
        "the gate must short-circuit before the command runs; stderr={stderr}"
    );
    // Not at a terminal means refuse now, not poll for ten minutes.
    assert!(
        started.elapsed() < Duration::from_secs(30),
        "the refusal must be immediate, not a device-code poll"
    );
}

#[test]
fn a_machine_that_signed_in_runs_the_command() {
    // The anti-tautology control. Without it every assertion above would still
    // hold for a binary that refused unconditionally.
    let (code, stderr) = run_cerulion(true, &[], &["node", "list"]);
    assert!(
        !stderr.contains("never signed in"),
        "a signed-in machine must not be refused; stderr={stderr}"
    );
    assert!(
        stderr.contains("Workspace not found"),
        "the command ran its normal path; stderr={stderr}"
    );
    assert_ne!(
        code,
        Some(i32::from(EXIT_AUTH_REQUIRED)),
        "a signed-in machine never exits with the auth code; stderr={stderr}"
    );
}

#[test]
fn a_guess_at_a_switch_does_not_switch_the_gate_off() {
    // The gate is on by default and stays on for every plausible guess. The
    // one value that works is not documented for users and is not tried here
    // by name: the property under test is that everything else fails.
    for (k, v) in [
        ("CERULION_LOGIN_GATE", "0"),
        ("CERULION_LOGIN_GATE", "1"),
        ("CERULION_LOGIN_GATE", "false"),
        ("CERULION_LOGIN_GATE", "OFF"),
        ("CERULION_LOGIN_GATE", "Off"),
        ("CERULION_LOGIN_GATE", " off"),
        ("CERULION_LOGIN_GATE", "off "),
        ("CERULION_LOGIN", "0"),
        ("CERULION_NO_LOGIN", "1"),
        ("CERULION_SKIP_LOGIN", "1"),
        ("CERULION_LOGIN_REQUIRED", "0"),
        ("CI", "true"),
    ] {
        let (code, stderr) = run_cerulion(false, &[(k, v)], &["node", "list"]);
        assert_is_the_refusal(&stderr);
        assert_eq!(
            code,
            Some(i32::from(EXIT_AUTH_REQUIRED)),
            "{k}={v} must not switch the gate off; stderr={stderr}"
        );
    }
}

#[test]
fn this_repositorys_own_runs_proceed_without_an_account() {
    // The other half, and the reason the suite can run at all on a machine that
    // has never signed in: the internal value this repository sets for its own
    // runs lets an unseeded home through. The home is the same empty one the
    // refusal arms use, so this can only pass on the switch.
    let (code, stderr) = run_cerulion(false, &[("CERULION_LOGIN_GATE", "off")], &["node", "list"]);
    assert!(
        !stderr.contains("never signed in"),
        "our own runs are not refused; stderr={stderr}"
    );
    assert!(
        stderr.contains("Workspace not found"),
        "the command ran its normal path; stderr={stderr}"
    );
    assert_ne!(
        code,
        Some(i32::from(EXIT_AUTH_REQUIRED)),
        "our own runs never exit with the auth code; stderr={stderr}"
    );
}

#[test]
fn the_exempt_verbs_run_without_an_identity() {
    // `completions` renders local text and is read by a shell rc file at
    // startup, where a device-code prompt nobody is watching would block the
    // shell. `login` is the login itself. Neither may be gated. (`clean` is
    // exempt too and has its own arm below, because proving it means reading
    // the report it prints rather than only its exit code.)
    for args in [
        vec!["completions", "zsh"],
        vec!["--help"],
        vec!["--version"],
        vec!["login", "--help"],
    ] {
        let (code, stderr) = run_cerulion(false, &[], &args);
        assert!(
            !stderr.contains("never signed in"),
            "{args:?} must not be gated; stderr={stderr}"
        );
        assert_eq!(
            code,
            Some(0),
            "{args:?} must succeed on a machine that never signed in; stderr={stderr}"
        );
    }
}

#[test]
fn clean_runs_on_a_machine_that_has_never_signed_in() {
    // `clean` sweeps the shared-memory bookkeeping that dead processes left on
    // THIS machine. It reads no account, sends nothing anywhere and reaches no
    // network, and the desks that need it most have never signed in: a CI
    // runner, a fresh install a `kill -9` left wedged. Gating it would leave
    // such a machine no way to clear its own state.
    let home = tempfile::tempdir().unwrap();
    let (code, stdout, stderr) = run_cerulion_at(home.path(), &[], &["clean", "--report-only"]);
    assert!(
        !stderr.contains(REFUSAL_A_USER_READS),
        "`clean` must not meet the login gate; stderr={stderr}"
    );
    assert_eq!(
        code,
        Some(0),
        "`clean --report-only` must succeed on a machine that never signed in; \
         stdout={stdout} stderr={stderr}"
    );
    // It RAN, rather than exiting 0 having printed nothing. The report names
    // the node registry on every path it can take (a population, an absence,
    // or a directory it could not read), so this one line is present whatever
    // the machine's state is; the literal is transcribed from the renderer, not
    // read back off it.
    assert!(
        stdout.contains("iceoryx2 node registry:"),
        "`clean --report-only` must print its report; stdout={stdout}"
    );
    assert_no_identity_was_written(home.path());
}

#[test]
fn a_gated_verb_under_the_same_never_signed_in_home_still_refuses() {
    // The anti-tautology control for the arm above: the same empty home, a verb
    // that is NOT exempt, and the whole message a user reads. Without it the
    // arm above would pass just as well on a build that had switched the gate
    // off altogether. `--no-network` bounds the blast radius if it ever does:
    // a `topic list` that reached its own body would otherwise go scouting.
    let home = tempfile::tempdir().unwrap();
    let (code, stdout, stderr) =
        run_cerulion_at(home.path(), &[], &["topic", "list", "--no-network"]);
    assert!(
        stderr.contains(REFUSAL_A_USER_READS),
        "a gated verb must print the whole refusal a user reads; stderr={stderr}"
    );
    assert_eq!(
        code,
        Some(i32::from(EXIT_AUTH_REQUIRED)),
        "a gated verb exits 7 on a machine that never signed in; stderr={stderr}"
    );
    // The command did not run: `topic list` prints a LOCAL section on every
    // path of its own, and the refusal came instead of all of it.
    assert!(
        stdout.is_empty(),
        "the gate must short-circuit before the verb writes anything; stdout={stdout}"
    );
    assert_no_identity_was_written(home.path());
}

#[test]
fn a_wrong_command_line_is_answered_without_proving_who_you_are() {
    // The usage refusals sit above the gate on purpose. A stale invocation is
    // told it is stale, with clap's usage code (2), rather than being sent to a
    // browser first and then told nothing about what it got wrong.
    let (code, stderr) = run_cerulion(false, &[], &["definitely-not-a-verb"]);
    assert!(
        !stderr.contains("never signed in"),
        "an unknown verb is a usage error, not an auth refusal; stderr={stderr}"
    );
    assert_eq!(code, Some(2), "clap's usage code; stderr={stderr}");
}
