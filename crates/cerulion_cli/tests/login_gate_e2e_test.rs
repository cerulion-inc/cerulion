// SPDX-License-Identifier: AGPL-3.0-only
//! The binary login-gate HOOK end to end over the REAL `cerulion` binary: the
//! only coverage of `main`'s `command_needs_identity` + `ensure_login_gate`
//! wiring firing against a real command. The engine-level e2e
//! (`cerulion_cli_engine/tests/login_flow_e2e_test.rs`) drives
//! `ensure_login_gate` directly against a real account service and never spawns
//! the binary.
//!
//! Each test isolates `CERULION_HOME` to its own tempdir (never reads the real
//! `~/.cerulion`) and runs from a non-workspace cwd, so the file needs no
//! fixtures. Every arm but one touches nothing outside that tempdir, and no arm
//! needs `#[serial]`.
//!
//! ## The `clean` arm, and the registry it runs against
//!
//! `clean` is exempt precisely BECAUSE it sweeps iceoryx2 bookkeeping, so
//! proving the exemption at the boundary a user feels means letting a real
//! sweep run. It runs against a registry this test OWNS, never the machine's:
//! [`write_isolated_iceoryx2_config`] drops the project-local config file
//! iceoryx2 reads first into the spawn's own temporary cwd, so the sweep, the
//! registry count and the report all resolve to a directory under that cwd.
//! The arm proves the isolation took rather than assuming it, by requiring the
//! report to name that directory back.
//!
//! Two consequences worth stating. A sibling process cannot perturb the arm,
//! because it cannot see the registry the arm reads, and the arm cannot perturb
//! a sibling, because it reaches nothing outside its own temporary directories.
//! And the arm is constant time in the machine's dead-node population, which a
//! sweep of the default registry would not be. The one thing it still READS off
//! the machine is the `/tmp/*.shm_state` population on macOS and FreeBSD, whose
//! directory is a compile-time constant; `--report-only` removes nothing there,
//! and the walk is bounded by the verb's own report budget, which the arm's
//! elapsed-time assertion is a second check on.
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
    let cwd = tempfile::tempdir().unwrap(); // NOT a workspace
    let (code, _stdout, stderr) = run_cerulion_at(home.path(), cwd.path(), extra_env, args);
    (code, stderr)
}

/// The same spawn against a home and a cwd the CALLER owns, handing back stdout
/// as well.
///
/// Three arms need more than the refusal on stderr: one has to read the report
/// an exempt verb prints and to furnish the cwd it reads its iceoryx2 config
/// from, and two have to look at the home afterwards to show the run left no
/// local identity behind. Everything else about the spawn — the removed
/// `CERULION_LOGIN_GATE`, the dead account service — is [`run_cerulion`]'s,
/// because this is that function's body. The cwd must not be a workspace.
fn run_cerulion_at(
    home: &Path,
    cwd: &Path,
    extra_env: &[(&str, &str)],
    args: &[&str],
) -> (Option<i32>, String, String) {
    run_binary_at(
        Path::new(env!("CARGO_BIN_EXE_cerulion")),
        home,
        cwd,
        extra_env,
        args,
    )
}

/// [`run_cerulion_at`] with the `cerulion` binary at `bin`.
fn run_binary_at(
    bin: &Path,
    home: &Path,
    cwd: &Path,
    extra_env: &[(&str, &str)],
    args: &[&str],
) -> (Option<i32>, String, String) {
    let mut cmd = Command::new(bin);
    cmd.args(args)
        .current_dir(cwd)
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
        stderr.contains("Cerulion account service (http://127.0.0.1:1)"),
        "the refusal must name the resolved account service; stderr={stderr}"
    );
    assert!(
        stderr.contains("never signed in"),
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
/// included) when `CERULION_ACCOUNT_SERVICE=http://127.0.0.1:1`.
const REFUSAL_A_USER_READS: &str = "Error: This command requires the Cerulion account service (http://127.0.0.1:1), and this machine has never signed in.\nRun `cerulion login` once in a terminal, or for automation point CERULION_HOME at a directory holding the state of a machine that did.";

/// Point a spawned `cerulion` at a private iceoryx2 registry under `root`, by
/// writing the PROJECT-LOCAL config file into `cwd`.
///
/// `iceoryx2::config::Config` resolves its file in three places and takes the
/// first that loads: `config/iceoryx2.toml` relative to the process cwd, then
/// the user config directory, then the compiled-in global path. Only the first
/// is reachable from a test, and only because the spawn already runs from a
/// temporary directory. `Global` is `#[serde(default)]`, so `root-path` is the
/// one key that has to be written and everything else keeps its default.
///
/// The seam has no in-process equivalent here: `Config::set_root_path` acts on
/// the caller's own singleton, and the binary reads no environment variable for
/// the root, so a file in the child's cwd is the only way to reach the child's
/// singleton without adding a production seam.
///
/// `root` is written as a TOML basic string with `\` and `"` escaped. A
/// temporary directory is unlikely to carry either, and a path that broke the
/// file would be caught rather than silently ignored (see below), but the cost
/// of getting it right is two `replace` calls and the cost of getting it wrong
/// is the arm's whole point: a child reading no config falls back to the
/// machine's registry. The residual is a control character in a path, which no
/// escaping of these two characters covers and which the loud failure below
/// still catches.
///
/// Nothing asserts here. A config that failed to load would leave the child on
/// `/tmp/iceoryx2`, which the caller catches by requiring the report to name
/// `root` back.
fn write_isolated_iceoryx2_config(cwd: &Path, root: &Path) {
    let dir = cwd.join("config");
    std::fs::create_dir_all(&dir).expect("create the project-local config dir");
    let escaped = root
        .display()
        .to_string()
        .replace('\\', "\\\\")
        .replace('"', "\\\"");
    std::fs::write(
        dir.join("iceoryx2.toml"),
        format!("[global]\nroot-path = \"{escaped}\"\n"),
    )
    .expect("write the project-local iceoryx2 config");
}

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
    let (code, stderr) = run_cerulion(false, &[], &["account", "devices", "list"]);
    assert_is_the_refusal(&stderr);
    assert!(
        stderr.contains(REFUSAL_A_USER_READS),
        "must print the exact refusal naming the account service; stderr={stderr}"
    );
    assert_eq!(
        code,
        Some(i32::from(EXIT_AUTH_REQUIRED)),
        "an auth refusal exits 7, distinct from a runtime failure; stderr={stderr}"
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
    let (code, stderr) = run_cerulion(true, &[], &["account", "devices", "list"]);
    assert!(
        !stderr.contains("never signed in"),
        "a signed-in machine must not be refused; stderr={stderr}"
    );
    assert_ne!(
        code,
        Some(i32::from(EXIT_AUTH_REQUIRED)),
        "a signed-in machine never exits with the auth code; stderr={stderr}"
    );
}

#[test]
fn a_guess_at_a_switch_does_not_switch_the_gate_off() {
    // The gate is on by default for gated account/remote verbs and stays on for
    // every plausible guess.
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
        let (code, stderr) = run_cerulion(false, &[(k, v)], &["account", "devices", "list"]);
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
    // The internal value this repository sets for its own runs lets an unseeded
    // home through gated verbs.
    let (code, stderr) = run_cerulion(
        false,
        &[("CERULION_LOGIN_GATE", "off")],
        &["account", "devices", "list"],
    );
    assert!(
        !stderr.contains("never signed in"),
        "our own runs are not refused; stderr={stderr}"
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
        vec!["logout"],
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
    let cwd = tempfile::tempdir().unwrap(); // NOT a workspace
    let root = cwd.path().join("iox2-root");
    std::fs::create_dir_all(&root).expect("create the private iceoryx2 root");
    write_isolated_iceoryx2_config(cwd.path(), &root);

    let started = Instant::now();
    let (code, stdout, stderr) =
        run_cerulion_at(home.path(), cwd.path(), &[], &["clean", "--report-only"]);
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
    // It RAN, rather than exiting 0 having printed nothing: the sweep reports
    // its outcome on every path, and an empty registry has exactly this one.
    // Hand-written from `report_sweep`, not read back off it.
    assert!(
        stdout.contains("No dead iceoryx2 nodes found"),
        "`clean --report-only` must run its dead-node sweep and report it; stdout={stdout}"
    );
    // And it ran against the registry THIS TEST owns. The renderer prints the
    // configured node directory back, so naming `root` here is what proves the
    // project-local config took: a child that had fallen back to the machine's
    // default would print `/tmp/iceoryx2/nodes` and fail this line.
    let registry_line = format!("iceoryx2 node registry: none at {}/nodes", root.display());
    assert!(
        stdout.contains(&registry_line),
        "the report must name the private registry ({registry_line}), which is the proof \
         the sweep never reached the machine's own; stdout={stdout}"
    );
    // A private empty registry makes the sweep constant time, so the only walk
    // left that grows with the machine is the budget-bounded `/tmp` read.
    assert!(
        started.elapsed() < Duration::from_secs(30),
        "`clean --report-only` must stay inside its own report budget"
    );
    assert_no_identity_was_written(home.path());
}

#[test]
fn a_gated_verb_under_the_same_never_signed_in_home_still_refuses() {
    // The anti-tautology control for the arm above: the same empty home, a
    // gated account verb, and the whole message a user reads.
    let home = tempfile::tempdir().unwrap();
    let cwd = tempfile::tempdir().unwrap(); // NOT a workspace
    let (code, stdout, stderr) = run_cerulion_at(
        home.path(),
        cwd.path(),
        &[],
        &["account", "devices", "list"],
    );
    assert!(
        stderr.contains(REFUSAL_A_USER_READS),
        "a gated verb must print the whole refusal a user reads; stderr={stderr}"
    );
    assert_eq!(
        code,
        Some(i32::from(EXIT_AUTH_REQUIRED)),
        "a gated verb exits 7 on a machine that never signed in; stderr={stderr}"
    );
    assert!(
        stdout.is_empty(),
        "the gate must short-circuit before the verb writes anything; stdout={stdout}"
    );
    assert_no_identity_was_written(home.path());
}

#[test]
fn local_topic_and_bag_verbs_run_without_sign_in() {
    for args in [
        &["topic", "list", "--local"][..],
        &["bag", "info", "missing.mcap"],
        &["schema", "list"],
        &["node", "list"],
        &["graph", "list"],
        &["graph", "validate", "demo"],
    ] {
        let home = tempfile::tempdir().unwrap();
        let cwd = tempfile::tempdir().unwrap();
        let (code, _stdout, stderr) = run_cerulion_at(
            home.path(),
            cwd.path(),
            &[("CERULION_NETWORK", "off")],
            args,
        );
        assert_ne!(
            code,
            Some(i32::from(EXIT_AUTH_REQUIRED)),
            "{args:?} must bypass the login gate; stderr={stderr}"
        );
        assert!(
            !stderr.contains("never signed in"),
            "{args:?} must not print the login refusal; stderr={stderr}"
        );
        assert_no_identity_was_written(home.path());
    }
}

#[test]
fn robot_flag_flips_dual_mode_verbs_behind_the_login_gate() {
    // Without `--robot`, local verbs bypass the login gate.
    for args in [
        &["topic", "info", "/no_such_topic"][..],
        &[
            "topic",
            "pub",
            "/offline_gate_test",
            "std_msgs/String",
            "{data: hi}",
            "--wait-subscribers",
            "0",
        ],
        &["topic", "snapshot", "/no_such_cam", "--timeout", "0"],
        &[
            "service",
            "call",
            "/no_such_srv",
            "std_srvs/srv/Trigger",
            "--timeout",
            "0",
        ],
    ] {
        let home = tempfile::tempdir().unwrap();
        let cwd = tempfile::tempdir().unwrap();
        let root = cwd.path().join("iox2-root");
        std::fs::create_dir_all(&root).unwrap();
        write_isolated_iceoryx2_config(cwd.path(), &root);
        let (code, _stdout, stderr) = run_cerulion_at(
            home.path(),
            cwd.path(),
            &[("CERULION_NETWORK", "off")],
            args,
        );
        assert_ne!(
            code,
            Some(i32::from(EXIT_AUTH_REQUIRED)),
            "{args:?} without --robot must bypass the login gate; stderr={stderr}"
        );
        assert!(
            !stderr.contains("never signed in"),
            "{args:?} without --robot must not print the login refusal; stderr={stderr}"
        );
        assert_no_identity_was_written(home.path());
    }

    // With `--robot` (and for `graph live` / `graph fires` always), the same
    // verbs enter the login gate and exit 7 naming the Cerulion account service.
    for args in [
        &["viz", "/camera", "--robot", "bench-arm"][..],
        &["topic", "echo", "/camera", "--robot", "bench-arm"],
        &["topic", "hz", "/camera", "--robot", "bench-arm"],
        &["topic", "info", "/camera", "--robot", "bench-arm"],
        &[
            "topic",
            "pub",
            "/cmd_vel",
            "std_msgs/String",
            "{data: hi}",
            "--robot",
            "bench-arm",
        ],
        &["topic", "snapshot", "/camera", "--robot", "bench-arm"],
        &[
            "service",
            "call",
            "/reset",
            "std_srvs/srv/Trigger",
            "--robot",
            "bench-arm",
        ],
        &["graph", "live"],
        &["graph", "fires"],
    ] {
        let home = tempfile::tempdir().unwrap();
        let cwd = tempfile::tempdir().unwrap();
        let (code, stdout, stderr) =
            run_cerulion_at(home.path(), cwd.path(), &[("CERULION_NETWORK", "")], args);
        assert_eq!(
            code,
            Some(i32::from(EXIT_AUTH_REQUIRED)),
            "{args:?} must exit 7 without sign-in; stderr={stderr}"
        );
        assert!(
            stderr.contains(REFUSAL_A_USER_READS),
            "{args:?} must print the exact refusal naming the account service; stderr={stderr}"
        );
        assert!(stdout.is_empty(), "{args:?}: stdout={stdout}");
        assert_no_identity_was_written(home.path());
    }
}

#[test]
fn account_service_refusal_names_override_and_default_url() {
    let cwd = tempfile::tempdir().unwrap();
    for (override_url, expected_url) in [
        (Some("http://127.0.0.1:1"), "http://127.0.0.1:1"),
        (None, "https://app.cerulion.com"),
    ] {
        let home = tempfile::tempdir().unwrap();
        let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
        cmd.args(["account", "devices", "list"])
            .current_dir(cwd.path())
            .env_remove("CERULION_LOGIN_GATE")
            .env_remove("CERULION_ACCOUNT_SERVICE")
            .env("CERULION_HOME", home.path());
        if let Some(url) = override_url {
            cmd.env("CERULION_ACCOUNT_SERVICE", url);
        }
        let out = cmd.output().expect("spawn cerulion");
        let stderr = String::from_utf8_lossy(&out.stderr);
        assert_eq!(
            out.status.code(),
            Some(i32::from(EXIT_AUTH_REQUIRED)),
            "stderr={stderr}"
        );
        let want = format!(
            "Error: This command requires the Cerulion account service ({expected_url}), and this machine has never signed in.\nRun `cerulion login` once in a terminal, or for automation point CERULION_HOME at a directory holding the state of a machine that did."
        );
        assert!(stderr.contains(&want), "want:\n{want}\ngot:\n{stderr}");
        assert_no_identity_was_written(home.path());
    }
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

/// `cerulion logout` over the real binary: the offline sign-out still removes
/// the session (exit 1 names the unconfirmed revoke), and the next command is
/// refused with the signed-out message, not the never-signed-in one.
#[test]
fn logout_signs_the_machine_out_and_the_next_command_is_refused() {
    let home = tempfile::tempdir().unwrap();
    auth::seed_logged_in_at(home.path(), "acct-logout-e2e").unwrap();
    let cwd = tempfile::tempdir().unwrap();
    let run = |args: &[&str]| {
        let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
            .args(args)
            .current_dir(cwd.path())
            .env_remove("CERULION_LOGIN_GATE")
            .env("CERULION_HOME", home.path())
            .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1")
            .output()
            .expect("spawn cerulion");
        (
            out.status.code(),
            String::from_utf8_lossy(&out.stdout).to_string(),
            String::from_utf8_lossy(&out.stderr).to_string(),
        )
    };

    let (code, stdout, stderr) = run(&["logout"]);
    assert_eq!(stdout.trim(), "signed_out: account=acct-logout-e2e");
    assert_eq!(
        code,
        Some(1),
        "an unconfirmed revoke exits 1; stderr={stderr}"
    );
    assert!(
        stderr.contains("signed out on this machine"),
        "stderr={stderr}"
    );

    // Local verbs still run on a signed-out machine without entering the gate.
    let (code, _, stderr) = run(&["node", "list"]);
    assert_ne!(code, Some(i32::from(EXIT_AUTH_REQUIRED)), "stderr={stderr}");
    assert!(stderr.contains("Workspace not found"), "stderr={stderr}");

    // Account verbs exit 7 with the signed-out refusal naming the account service.
    let (code, _, stderr) = run(&["account", "devices", "list"]);
    assert_eq!(code, Some(i32::from(EXIT_AUTH_REQUIRED)), "stderr={stderr}");
    assert!(
        stderr.contains("Cerulion account service (http://127.0.0.1:1)")
            && stderr.contains("this machine is signed out")
            && stderr.contains("cerulion login"),
        "stderr={stderr}"
    );
    assert!(!stderr.contains("never signed in"), "stderr={stderr}");

    let (code, stdout, stderr) = run(&["logout"]);
    assert_eq!(code, Some(0), "stderr={stderr}");
    assert_eq!(stdout.trim(), "not_signed_in");
}

/// Signing out a machine that never signed in writes nothing, not even the
/// store's lock file.
#[test]
fn logout_on_a_machine_that_never_signed_in_leaves_no_trace() {
    let root = tempfile::tempdir().unwrap();
    let home = root.path().join("cerulion-home");
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .arg("logout")
        .current_dir(root.path())
        .env_remove("CERULION_LOGIN_GATE")
        .env("CERULION_HOME", &home)
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1")
        .output()
        .expect("spawn cerulion");
    assert_eq!(out.status.code(), Some(0));
    assert_eq!(String::from_utf8_lossy(&out.stdout).trim(), "not_signed_in");
    assert!(!home.exists(), "logout created {}", home.display());
}

#[test]
fn a_local_verb_family_runs_without_sign_in_and_a_remote_family_is_gated() {
    // `schema list` is a local verb family and runs on a never-signed-in machine
    // without leaving any identity behind.
    let home = tempfile::tempdir().unwrap();
    let cwd = tempfile::tempdir().unwrap(); // NOT a workspace
    let (code, stdout, stderr) = run_cerulion_at(home.path(), cwd.path(), &[], &["schema", "list"]);
    assert!(
        !stderr.contains("never signed in"),
        "local family verb must not meet the login gate; stderr={stderr}"
    );
    assert_eq!(code, Some(0), "stdout={stdout} stderr={stderr}");
    assert!(
        stdout.contains("Built-in ROS 2 messages: "),
        "the local family ran its normal path; stdout={stdout}"
    );
    assert_no_identity_was_written(home.path());

    // A remote-robot verb family (`service call --robot`) stays gated like a
    // built-in account/remote verb.
    let (code, stdout, stderr) = run_cerulion_at(
        home.path(),
        cwd.path(),
        &[("CERULION_NETWORK", "")],
        &[
            "service",
            "call",
            "/reset",
            "std_srvs/srv/Trigger",
            "--robot",
            "bench-arm",
        ],
    );
    assert!(
        stderr.contains(REFUSAL_A_USER_READS),
        "a remote family verb must print the whole refusal a user reads; stderr={stderr}"
    );
    assert_eq!(
        code,
        Some(i32::from(EXIT_AUTH_REQUIRED)),
        "a remote family verb exits 7 on a machine that never signed in; stderr={stderr}"
    );
    assert!(
        stdout.is_empty(),
        "the gate must short-circuit before the family runs; stdout={stdout}"
    );
    assert_no_identity_was_written(home.path());
}

#[test]
fn rcp_under_the_network_switch_exits_2_before_the_login_gate() {
    // `cerulion rcp` under the kill-switch reaches nothing: on a machine that
    // never signed in it exits 2 with its own refusal, and the login gate
    // never runs, so there is no sign-in prompt and no account-service
    // contact. The account service here is a listener this test owns; a run
    // that dialled it leaves a connection in its backlog.
    for (extra, network) in [(&["--network", "off"][..], ""), (&[][..], "off")] {
        let home = tempfile::tempdir().unwrap();
        let cwd = tempfile::tempdir().unwrap(); // NOT a workspace
        let account = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let service = format!("http://{}", account.local_addr().unwrap());
        let mut args = vec!["rcp", "bench-arm"];
        args.extend_from_slice(extra);
        let (code, stdout, stderr) = run_cerulion_at(
            home.path(),
            cwd.path(),
            &[
                ("CERULION_NETWORK", network),
                ("CERULION_ACCOUNT_SERVICE", service.as_str()),
            ],
            &args,
        );
        assert_eq!(
            code,
            Some(2),
            "{args:?} network={network:?}: stderr={stderr}"
        );
        // Hand-written from the refusal `cerulion rcp` prints, not read back
        // off the code that prints it.
        assert!(
            stderr.contains("cerulion rcp reaches the robot over the network, which"),
            "{args:?} network={network:?}: the rcp refusal; stderr={stderr}"
        );
        assert!(
            !stderr.contains("never signed in") && !stderr.contains("cerulion login"),
            "{args:?} network={network:?}: no sign-in prompt; stderr={stderr}"
        );
        assert!(
            stdout.is_empty(),
            "{args:?} network={network:?}: stdout={stdout}"
        );
        account.set_nonblocking(true).unwrap();
        assert!(
            matches!(account.accept(), Err(e) if e.kind() == std::io::ErrorKind::WouldBlock),
            "{args:?} network={network:?}: the run contacted the account service"
        );
        assert_no_identity_was_written(home.path());
    }
}

#[test]
fn rcp_without_the_network_switch_stays_behind_the_login_gate() {
    // The control for the arm above: with the switch off, `cerulion rcp` on a
    // machine that never signed in is refused by the gate like every gated
    // verb, so the arm above cannot pass on a verb that skipped the gate.
    let home = tempfile::tempdir().unwrap();
    let cwd = tempfile::tempdir().unwrap(); // NOT a workspace
    let (code, stdout, stderr) = run_cerulion_at(
        home.path(),
        cwd.path(),
        &[("CERULION_NETWORK", "")],
        &["rcp", "bench-arm"],
    );
    assert!(
        stderr.contains(REFUSAL_A_USER_READS),
        "rcp prints the whole refusal a user reads; stderr={stderr}"
    );
    assert_eq!(code, Some(i32::from(EXIT_AUTH_REQUIRED)), "stderr={stderr}");
    assert!(stdout.is_empty(), "stdout={stdout}");
    assert_no_identity_was_written(home.path());
}

/// A harness folder whose launcher writes the words it was run with to
/// `<dir>/ran.txt`, prints `MCP-STREAM` on stdout and exits 3: a run that
/// reached the launcher leaves the file, and the exit code and stdout show the
/// CLI handed both back untouched.
fn recording_harness(dir: &Path) {
    std::fs::write(
        dir.join("rcp.sh"),
        "printf '%s\\n' \"$@\" > \"$(dirname \"$0\")/ran.txt\"\nprintf 'MCP-STREAM\\n'\nexit 3\n",
    )
    .unwrap();
    std::fs::write(dir.join("install.sh"), "exit 9\n").unwrap();
}

/// Every form of the simulated robot and the benchmark.
const HARNESS_FORMS: &[&[&str]] = &[
    &["rcp", "--sim"],
    &["rcp", "--sim", "--http", "127.0.0.1:8765"],
    &["rcp", "bench", "--model", "m"],
    &["rcp", "bench", "--verify"],
];

#[test]
fn the_simulated_robot_and_the_bench_run_without_sign_in() {
    // `--sim` and `bench` reach no robot and no Cerulion service, so a machine
    // that never signed in runs the harness launcher directly with or without
    // the network switch and writes no auth.json.
    let version = env!("CARGO_PKG_VERSION");
    for (args, words) in [
        (HARNESS_FORMS[0], &["sim"][..]),
        (HARNESS_FORMS[1], &["sim", "--http", "127.0.0.1:8765"][..]),
        (HARNESS_FORMS[2], &["bench", "--model", "m"][..]),
        (HARNESS_FORMS[3], &["verify"][..]),
    ] {
        for network in ["", "off"] {
            let home = tempfile::tempdir().unwrap();
            let harness = tempfile::tempdir().unwrap();
            recording_harness(harness.path());
            let cwd = tempfile::tempdir().unwrap(); // NOT a workspace
            let dir = harness.path().to_str().unwrap();
            let (code, stdout, stderr) = run_cerulion_at(
                home.path(),
                cwd.path(),
                &[("CERULION_NETWORK", network), ("CERULION_HARNESS_DIR", dir)],
                args,
            );
            assert_eq!(
                code,
                Some(3),
                "{args:?} network={network:?}: stderr={stderr}"
            );
            assert_eq!(
                stdout, "MCP-STREAM\n",
                "{args:?} network={network:?}: stderr={stderr}"
            );
            let ran = std::fs::read_to_string(harness.path().join("ran.txt")).unwrap();
            let mut want = vec![
                "--home".to_string(),
                home.path()
                    .join("harness")
                    .join(version)
                    .display()
                    .to_string(),
            ];
            want.extend(words.iter().map(|w| w.to_string()));
            assert_eq!(ran.lines().collect::<Vec<_>>(), want, "{args:?}");
            assert_no_identity_was_written(home.path());
        }
    }
}

#[test]
fn a_signed_in_machine_runs_the_harness_launcher_and_hands_back_its_stream_and_code() {
    // The control for the arm above: on a signed-in home every form runs the
    // launcher of the folder CERULION_HARNESS_DIR names, under bash, with the
    // words hand-written below, and the CLI adds nothing to its stdout and
    // returns its exit code.
    let version = env!("CARGO_PKG_VERSION");
    for (args, words) in [
        (HARNESS_FORMS[0], &["sim"][..]),
        (HARNESS_FORMS[1], &["sim", "--http", "127.0.0.1:8765"][..]),
        (HARNESS_FORMS[2], &["bench", "--model", "m"][..]),
        (HARNESS_FORMS[3], &["verify"][..]),
    ] {
        let home = tempfile::tempdir().unwrap();
        auth::seed_logged_in_at(home.path(), "acct-login-gate-e2e").unwrap();
        let harness = tempfile::tempdir().unwrap();
        recording_harness(harness.path());
        let cwd = tempfile::tempdir().unwrap(); // NOT a workspace
        let dir = harness.path().to_str().unwrap();
        let (code, stdout, stderr) = run_cerulion_at(
            home.path(),
            cwd.path(),
            &[("CERULION_NETWORK", "off"), ("CERULION_HARNESS_DIR", dir)],
            args,
        );
        assert_eq!(code, Some(3), "{args:?}: stderr={stderr}");
        assert_eq!(stdout, "MCP-STREAM\n", "{args:?}: stderr={stderr}");
        let ran = std::fs::read_to_string(harness.path().join("ran.txt")).unwrap();
        let mut want = vec![
            "--home".to_string(),
            home.path()
                .join("harness")
                .join(version)
                .display()
                .to_string(),
        ];
        want.extend(words.iter().map(|w| w.to_string()));
        assert_eq!(ran.lines().collect::<Vec<_>>(), want, "{args:?}");
    }
}

#[test]
fn a_harness_folder_that_does_not_exist_is_named_and_nothing_runs() {
    let home = tempfile::tempdir().unwrap();
    auth::seed_logged_in_at(home.path(), "acct-login-gate-e2e").unwrap();
    let cwd = tempfile::tempdir().unwrap(); // NOT a workspace
    let missing = cwd.path().join("no-harness-here");
    let (code, stdout, stderr) = run_cerulion_at(
        home.path(),
        cwd.path(),
        &[("CERULION_HARNESS_DIR", missing.to_str().unwrap())],
        &["rcp", "--sim"],
    );
    assert_eq!(code, Some(1), "stderr={stderr}");
    assert!(stdout.is_empty(), "stdout={stdout}");
    assert!(
        stderr.contains(&format!(
            "Error: CERULION_HARNESS_DIR names '{}', which does not exist",
            missing.display()
        )),
        "stderr={stderr}"
    );
}

/// Set `path`'s permission bits to `mode`.
#[cfg(unix)]
fn set_mode(path: &Path, mode: u32) {
    use std::os::unix::fs::PermissionsExt;
    std::fs::set_permissions(path, std::fs::Permissions::from_mode(mode)).unwrap();
}

/// A link to the built `cerulion` (a copy where the file system refuses a
/// hard link) at `bin`, so the running binary's own folder is `bin`'s.
#[cfg(unix)]
fn link_cerulion_at(bin: &Path) {
    let built = env!("CARGO_BIN_EXE_cerulion");
    if std::fs::hard_link(built, bin).is_err() {
        std::fs::copy(built, bin).unwrap();
    }
}

/// Every form run with `bin` from `cwd`, signed in, with
/// `CERULION_HARNESS_DIR` unset: none may run `planted`'s launcher, each exits
/// 1 with nothing on stdout, and stderr holds `said`.
#[cfg(unix)]
fn assert_no_form_runs(bin: &Path, cwd: &Path, planted: &Path, said: &str) {
    for args in HARNESS_FORMS {
        let home = tempfile::tempdir().unwrap();
        auth::seed_logged_in_at(home.path(), "acct-login-gate-e2e").unwrap();
        let (code, stdout, stderr) = run_binary_at(
            bin,
            home.path(),
            cwd,
            &[("CERULION_NETWORK", "off"), ("CERULION_HARNESS_DIR", "")],
            args,
        );
        assert!(
            !planted.join("ran.txt").exists(),
            "{args:?}: the planted launcher ran; stderr={stderr}"
        );
        assert_eq!(code, Some(1), "{args:?}: stderr={stderr}");
        assert!(stdout.is_empty(), "{args:?}: stdout={stdout}");
        assert!(stderr.contains(said), "{args:?}: stderr={stderr}");
    }
}

#[cfg(unix)]
#[test]
fn a_harness_in_the_directory_cerulion_runs_in_never_runs() {
    // A downloaded folder dressed as a Cerulion checkout (the checkout marker
    // and a `harness/` with both file names) whose launcher records that it
    // ran. Every form run inside it, signed in, with CERULION_HARNESS_DIR
    // unset, runs nothing from it. The binary runs from a link outside any
    // checkout, in a folder of its own inside a fence of mode 1777, so the
    // folder beside it holds no harness and the walk up from it stops at the
    // fence on every machine.
    let fence = tempfile::tempdir().unwrap();
    set_mode(fence.path(), 0o1777);
    let bin_dir = fence.path().join("bin");
    std::fs::create_dir(&bin_dir).unwrap();
    set_mode(&bin_dir, 0o755);
    let bin_dir_real = std::fs::canonicalize(&bin_dir).unwrap();
    let fence_real = std::fs::canonicalize(fence.path()).unwrap();
    assert_eq!(
        cerulion_cli_engine::harness_cmd::checkout_root(&bin_dir_real),
        cerulion_cli_engine::harness_cmd::CheckoutRoot::StoppedAt(&fence_real),
    );
    let bin = bin_dir.join("cerulion");
    link_cerulion_at(&bin);
    let download = tempfile::tempdir().unwrap();
    std::fs::create_dir_all(download.path().join("crates/cerulion_cli")).unwrap();
    std::fs::write(download.path().join("crates/cerulion_cli/Cargo.toml"), "").unwrap();
    let harness = download.path().join("harness");
    std::fs::create_dir_all(&harness).unwrap();
    recording_harness(&harness);
    let cwd = download.path().join("docs");
    std::fs::create_dir_all(&cwd).unwrap();
    assert_no_form_runs(
        &bin,
        &cwd,
        &harness,
        &format!(
            "(3) a Cerulion source checkout around the binary, but the search up from '{}' \
             stopped at '{}', which every user can write to, and no folder on the way there \
             holds crates/cerulion_cli/Cargo.toml. The current directory is never searched.",
            bin_dir_real.display(),
            fence_real.display()
        ),
    );
}

#[cfg(unix)]
#[test]
fn a_harness_planted_in_a_folder_other_users_can_write_never_runs() {
    // A folder of mode 1777 stands in for /tmp, where any user of the
    // machine can plant the checkout marker and a `harness/` whose launcher
    // records that it ran. The user's own cerulion sits in a folder of its own
    // inside it, then in the shared folder itself, beside the planted harness.
    // Each form runs from another directory and runs nothing planted.
    let shared = tempfile::tempdir().unwrap();
    let shared_real = std::fs::canonicalize(shared.path()).unwrap();
    std::fs::create_dir_all(shared_real.join("crates/cerulion_cli")).unwrap();
    std::fs::write(shared_real.join("crates/cerulion_cli/Cargo.toml"), "").unwrap();
    let planted = shared_real.join("harness");
    std::fs::create_dir_all(&planted).unwrap();
    recording_harness(&planted);
    set_mode(&shared_real, 0o1777);
    let own = shared_real.join("rel");
    std::fs::create_dir(&own).unwrap();
    set_mode(&own, 0o700);
    let cwd = tempfile::tempdir().unwrap(); // NOT a workspace
    link_cerulion_at(&own.join("cerulion"));
    assert_no_form_runs(
        &own.join("cerulion"),
        cwd.path(),
        &planted,
        &format!(
            "(2) '{}', and (3) a Cerulion source checkout around the binary, but the search up \
             from '{}' stopped at '{}', which every user can write to, and no folder on the \
             way there holds crates/cerulion_cli/Cargo.toml.",
            own.join("harness").display(),
            own.display(),
            shared_real.display()
        ),
    );
    link_cerulion_at(&shared_real.join("cerulion"));
    assert_no_form_runs(
        &shared_real.join("cerulion"),
        cwd.path(),
        &planted,
        &format!(
            "(2) '{}', which is not taken because every user can write to '{}', and (3)",
            planted.display(),
            shared_real.display()
        ),
    );
}
