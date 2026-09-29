// SPDX-License-Identifier: AGPL-3.0-only
//! Real-binary local-scope refusals and precedence over network locators.
//! Each spawn reads a private shared-memory registry and Cerulion home.

use std::path::Path;
use std::process::{Command, Output};

fn run(root: &Path, args: &[&str], network: Option<&str>) -> Output {
    let home = root.join("home");
    cerulion_cli_engine::auth::seed_logged_in_at(&home, "local-scope-test").unwrap();
    let config = root.join("config");
    std::fs::create_dir_all(&config).unwrap();
    let registry = root.join("shm");
    let path = registry
        .to_string_lossy()
        .replace('\\', "\\\\")
        .replace('"', "\\\"");
    std::fs::write(
        config.join("iceoryx2.toml"),
        format!("[global]\nroot-path = \"{path}\"\n"),
    )
    .unwrap();
    let mut command = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    command
        .args(args)
        .current_dir(root)
        .env("CERULION_HOME", &home)
        .env("CERULION_NETD_SOCK", root.join("netd.sock"))
        .env("CERULION_NETD_BIN", root.join("absent-netd"))
        .env_remove("CERULION_NETWORK")
        .env_remove("CERULION_LOGIN_GATE")
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1");
    if let Some(value) = network {
        command.env("CERULION_NETWORK", value);
    }
    let output = command.output().expect("run the real CLI");
    assert!(
        !root.join("netd.sock").exists(),
        "a local command created a network-daemon socket"
    );
    output
}

#[test]
fn local_observers_refuse_missing_topics_without_remote_search_or_daemon() {
    for verb in ["echo", "hz", "info"] {
        let root = tempfile::tempdir().unwrap();
        let output = run(
            root.path(),
            &["topic", verb, "/local-scope-test/missing", "--local"],
            None,
        );
        let error = String::from_utf8_lossy(&output.stderr);
        assert_eq!(output.status.code(), Some(1), "{error}");
        assert!(error.contains("not found locally"), "{error}");
        assert!(error.contains("disabled by --local"), "{error}");
        assert!(!error.contains("on any discovered robot"), "{error}");
        assert!(output.stdout.is_empty());
    }
}

#[test]
fn local_list_and_legacy_alias_override_scan_and_explicit_locators() {
    for flag in ["--local", "--no-network"] {
        let root = tempfile::tempdir().unwrap();
        let output = run(
            root.path(),
            &[
                "topic",
                "list",
                flag,
                "--scan",
                "--connect",
                "tcp/127.0.0.1:1",
                "--listen",
                "tcp/127.0.0.1:0",
            ],
            None,
        );
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        let listing = String::from_utf8_lossy(&output.stdout);
        assert!(listing == "No active local topics.\n", "{listing}");
        assert!(!listing.contains("remote:"), "{listing}");
        assert!(
            output.stderr.is_empty(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
    }
}

#[test]
fn environment_kill_switch_also_suppresses_topic_list() {
    for value in ["off", " OFF ", "typo"] {
        let root = tempfile::tempdir().unwrap();
        let output = run(
            root.path(),
            &["topic", "list", "--scan", "--connect", "tcp/127.0.0.1:1"],
            Some(value),
        );
        let error = String::from_utf8_lossy(&output.stderr);
        assert!(output.status.success(), "{error}");
        let listing = String::from_utf8_lossy(&output.stdout);
        assert_eq!(listing, "No active local topics.\n", "{listing}");
        assert!(
            !error.contains("remote: discovery unavailable"),
            "the environment kill-switch must skip the query, not fail it: {error}"
        );
        if value == "typo" {
            assert!(error.contains("failing CLOSED"), "{error}");
        } else {
            assert!(error.is_empty(), "{error}");
        }
    }
}
