// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion telemetry status|on|off` and the first-run notice over the real
//! binary. Each test isolates `CERULION_HOME` in its own tempdir and clears
//! every variable that could decide consent, so the machine's own settings
//! never leak in. The key, where one is set, points at a reserved dead port:
//! nothing can be delivered anywhere.
#![cfg(feature = "telemetry")]

use std::path::Path;
use std::process::Command;

use cerulion_cli_engine::auth;

struct Out {
    code: Option<i32>,
    stdout: String,
    stderr: String,
}

fn cerulion(home: &Path, env: &[(&str, &str)], args: &[&str]) -> Out {
    let cwd = tempfile::tempdir().unwrap();
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    cmd.args(args)
        .current_dir(cwd.path())
        .env("CERULION_HOME", home)
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1")
        .env_remove("DO_NOT_TRACK")
        .env_remove("CERULION_TELEMETRY")
        .env_remove("POSTHOG_API_KEY")
        .env_remove("POSTHOG_HOST");
    for (k, v) in env {
        cmd.env(k, v);
    }
    let out = cmd.output().expect("spawn cerulion");
    Out {
        code: out.status.code(),
        stdout: String::from_utf8_lossy(&out.stdout).into_owned(),
        stderr: String::from_utf8_lossy(&out.stderr).into_owned(),
    }
}

#[test]
fn status_needs_no_login_and_writes_nothing() {
    let home = tempfile::tempdir().unwrap();
    let out = cerulion(
        home.path(),
        &[("CERULION_LOGIN_GATE", "on")],
        &["telemetry", "status"],
    );
    assert_eq!(out.code, Some(0), "stderr={}", out.stderr);
    assert!(
        out.stdout.contains("telemetry: on (default)"),
        "{}",
        out.stdout
    );
    assert!(!home.path().join("telemetry.json").exists());
}

#[test]
fn off_then_on_persist_in_the_consent_file() {
    let home = tempfile::tempdir().unwrap();
    let off = cerulion(home.path(), &[], &["telemetry", "off"]);
    assert_eq!(off.code, Some(0), "stderr={}", off.stderr);
    assert!(
        off.stdout.contains("telemetry: off (set by"),
        "{}",
        off.stdout
    );
    let file = std::fs::read_to_string(home.path().join("telemetry.json")).unwrap();
    assert!(
        file.contains("\"enabled\":false") || file.contains("\"enabled\": false"),
        "{file}"
    );
    let status = cerulion(home.path(), &[], &["telemetry", "status"]);
    assert!(
        status.stdout.contains("telemetry: off"),
        "{}",
        status.stdout
    );
    let on = cerulion(home.path(), &[], &["telemetry", "on"]);
    assert!(on.stdout.contains("telemetry: on"), "{}", on.stdout);
}

#[test]
fn do_not_track_wins_over_an_opt_in_and_says_so() {
    let home = tempfile::tempdir().unwrap();
    let out = cerulion(home.path(), &[("DO_NOT_TRACK", "1")], &["telemetry", "on"]);
    assert_eq!(out.code, Some(0), "stderr={}", out.stderr);
    assert!(
        out.stdout.contains("telemetry: off (set by DO_NOT_TRACK)"),
        "{}",
        out.stdout
    );
    assert!(
        out.stdout.contains("overrides the consent file"),
        "{}",
        out.stdout
    );
}

#[test]
fn the_notice_prints_once_and_only_when_a_key_could_send() {
    let home = tempfile::tempdir().unwrap();
    auth::seed_logged_in_at(home.path(), "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc").unwrap();
    let args = ["graph", "list"];
    let keyless = cerulion(home.path(), &[], &args);
    assert!(
        !keyless.stderr.contains("usage events"),
        "{}",
        keyless.stderr
    );
    let key = [
        ("POSTHOG_API_KEY", "k"),
        ("POSTHOG_HOST", "http://127.0.0.1:1"),
    ];
    let first = cerulion(home.path(), &key, &args);
    assert!(
        first.stderr.contains("cerulion telemetry off"),
        "{}",
        first.stderr
    );
    let second = cerulion(home.path(), &key, &args);
    assert!(!second.stderr.contains("usage events"), "{}", second.stderr);
    assert_eq!(
        first.code, second.code,
        "the notice never changes the exit code"
    );
}

#[test]
fn an_opted_out_machine_never_sees_the_notice() {
    let home = tempfile::tempdir().unwrap();
    auth::seed_logged_in_at(home.path(), "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc").unwrap();
    cerulion(home.path(), &[], &["telemetry", "off"]);
    let key = [
        ("POSTHOG_API_KEY", "k"),
        ("POSTHOG_HOST", "http://127.0.0.1:1"),
    ];
    let out = cerulion(home.path(), &key, &["graph", "list"]);
    assert!(!out.stderr.contains("usage events"), "{}", out.stderr);
}
