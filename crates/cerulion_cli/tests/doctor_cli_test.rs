// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion doctor` over the REAL binary: the `--json` report's shape, the
//! exit code, the human table's fix lines, the absence of a login gate in
//! front of the verb, and a usage error for an unknown check.
//!
//! Each case runs under its own empty `HOME` and `CERULION_HOME` and a
//! private cwd whose `config/iceoryx2.toml` points iceoryx2 at a private root,
//! so the registry walk of `shm.stale-nodes` reads a directory no other
//! process uses and the account check always reads "never signed in".

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

struct Sandbox {
    root: PathBuf,
}

impl Sandbox {
    fn new(name: &str) -> Self {
        let root = std::env::temp_dir().join(format!("doctor-cli-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        for dir in ["home", "cerulion-home", "cwd/config", "shm"] {
            std::fs::create_dir_all(root.join(dir)).unwrap();
        }
        let shm = root.join("shm").to_string_lossy().replace('\\', "\\\\");
        std::fs::write(
            root.join("cwd/config/iceoryx2.toml"),
            format!(
                "[global]\nroot-path = \"{shm}/\"\nprefix = \"doctor_{}_\"\n",
                std::process::id()
            ),
        )
        .unwrap();
        Self { root }
    }

    fn cwd(&self) -> PathBuf {
        self.root.join("cwd")
    }

    fn run(&self, args: &[&str]) -> Output {
        run_in(&self.root, &self.cwd(), args)
    }
}

impl Drop for Sandbox {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.root);
    }
}

fn run_in(root: &Path, cwd: &Path, args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(args)
        .current_dir(cwd)
        .env("HOME", root.join("home"))
        .env("CERULION_HOME", root.join("cerulion-home"))
        .env("DO_NOT_TRACK", "1")
        .env_remove("CERULION_LOGIN_GATE")
        .env_remove("CERULION_NETWORK")
        .env_remove("RUST_LOG")
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1")
        .output()
        .expect("run cerulion")
}

fn json_of(out: &Output) -> serde_json::Value {
    serde_json::from_slice(&out.stdout).unwrap_or_else(|e| {
        panic!(
            "stdout is not one JSON object ({e}):\n{}\nstderr:\n{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        )
    })
}

/// The catalogue as the verb lists it, in order.
const CHECK_IDS: [&str; 7] = [
    "limits.open-files",
    "shm.dev-shm",
    "shm.stale-nodes",
    "iceoryx2.cwd-config",
    "disk.recordings",
    "account.login",
    "cpu.governor",
];

#[test]
fn a_full_json_report_lists_every_check_in_order_and_exits_1_on_the_failed_login() {
    let sb = Sandbox::new("full");
    let out = sb.run(&["doctor", "--json"]);
    let json = json_of(&out);
    assert_eq!(json["report_version"], 1);
    let ids: Vec<&str> = json["checks"]
        .as_array()
        .unwrap()
        .iter()
        .map(|c| c["id"].as_str().unwrap())
        .collect();
    assert_eq!(ids, CHECK_IDS);
    for check in json["checks"].as_array().unwrap() {
        let status = check["status"].as_str().unwrap();
        assert!(
            ["pass", "info", "warn", "fail", "skip"].contains(&status),
            "{check}"
        );
        assert!(!check["why"].as_str().unwrap().is_empty(), "{check}");
        if status == "warn" || status == "fail" {
            assert!(
                check["fix"].is_string(),
                "a {status} without a fix: {check}"
            );
        }
    }
    let total: u64 = json["counts"]
        .as_object()
        .unwrap()
        .values()
        .map(|v| v.as_u64().unwrap())
        .sum();
    assert_eq!(total, CHECK_IDS.len() as u64);
    // The empty CERULION_HOME has never signed in: account.login fails, so the
    // run exits 1, and the login gate did not stop the verb first (exit 7).
    assert_eq!(out.status.code(), Some(1), "{json}");
    assert_eq!(json["exit_code"], 1);
}

#[test]
fn an_unsigned_machine_gets_the_login_fix_in_json_and_in_the_table() {
    let sb = Sandbox::new("login");
    let out = sb.run(&["doctor", "--json", "--check", "account.login"]);
    assert_eq!(out.status.code(), Some(1));
    let json = json_of(&out);
    assert_eq!(json["checks"].as_array().unwrap().len(), 1);
    assert_eq!(json["checks"][0]["status"], "fail");
    assert_eq!(json["checks"][0]["fix"], "run `cerulion login`");
    assert_eq!(json["counts"]["fail"], 1);

    let table = sb.run(&["doctor", "--check", "account.login"]);
    assert_eq!(table.status.code(), Some(1));
    let text = String::from_utf8(table.stdout).unwrap();
    assert!(
        text.starts_with("FAIL  account.login  this machine has never signed in"),
        "{text}"
    );
    assert!(
        text.contains("\n                     fix: run `cerulion login`\n"),
        "{text}"
    );
    assert!(
        text.ends_with("\n0 pass, 0 info, 0 warn, 1 fail, 0 skip\n"),
        "{text}"
    );
}

#[test]
fn a_project_iceoryx2_config_that_moves_the_root_warns_and_exits_0() {
    let sb = Sandbox::new("cwdcfg");
    let out = sb.run(&["doctor", "--json", "--check", "iceoryx2.cwd-config"]);
    assert_eq!(out.status.code(), Some(0), "a warning alone exits 0");
    let json = json_of(&out);
    let check = &json["checks"][0];
    assert_eq!(check["status"], "warn", "{check}");
    assert!(
        check["fix"]
            .as_str()
            .unwrap()
            .contains("~/.config/iceoryx2/iceoryx2.toml"),
        "{check}"
    );
}

#[test]
fn a_directory_without_a_project_config_passes_that_check() {
    let sb = Sandbox::new("nocfg");
    let plain = sb.root.join("plain");
    std::fs::create_dir_all(&plain).unwrap();
    let out = run_in(
        &sb.root,
        &plain,
        &["doctor", "--json", "--check", "iceoryx2.cwd-config"],
    );
    assert_eq!(out.status.code(), Some(0));
    assert_eq!(json_of(&out)["checks"][0]["status"], "pass");
}

#[test]
fn an_unknown_check_is_a_usage_error() {
    let sb = Sandbox::new("unknown");
    let out = sb.run(&["doctor", "--check", "no.such"]);
    assert_eq!(out.status.code(), Some(2));
    assert!(out.stdout.is_empty());
    let err = String::from_utf8(out.stderr).unwrap();
    assert!(err.contains("no.such"), "{err}");
}

/// A read-only current directory with no recordings directory is where a
/// user runs a diagnosis from (`/` on macOS is a sealed volume), not a
/// recording destination: the check warns and the run exits 0.
#[cfg(unix)]
#[test]
fn a_read_only_current_directory_warns_and_exits_0() {
    use std::os::unix::fs::PermissionsExt;
    let sb = Sandbox::new("rocwd");
    let ro = sb.root.join("readonly");
    std::fs::create_dir_all(&ro).unwrap();
    std::fs::set_permissions(&ro, std::fs::Permissions::from_mode(0o555)).unwrap();
    let out = run_in(
        &sb.root,
        &ro,
        &["doctor", "--json", "--check", "disk.recordings"],
    );
    std::fs::set_permissions(&ro, std::fs::Permissions::from_mode(0o755)).unwrap();
    assert_eq!(
        out.status.code(),
        Some(0),
        "{}",
        String::from_utf8_lossy(&out.stdout)
    );
    let check = &json_of(&out)["checks"][0];
    assert_eq!(check["readings"]["recordings_dir"], false);
    let uid = Command::new("id").arg("-u").output().expect("run id -u");
    if String::from_utf8_lossy(&uid.stdout).trim() == "0" {
        // Root passes `access(W_OK)` whatever the mode bits say; a tmpfs
        // TMPDIR still warns.
        assert_ne!(check["status"], "fail", "{check}");
    } else {
        assert_eq!(check["status"], "warn", "{check}");
        assert!(
            check["fix"].as_str().unwrap().contains("--record=<DIR>"),
            "{check}"
        );
    }
}

/// A malformed sign-in file whose parse error quotes a token-like value: the
/// value reaches neither the table nor the JSON, and nothing on stderr.
#[test]
fn a_malformed_sign_in_file_never_reaches_the_output() {
    const SECRET: &str = "tok_SECRET_FRAGMENT_7f3a";
    let sb = Sandbox::new("authleak");
    std::fs::write(
        sb.root.join("cerulion-home/auth.json"),
        format!(
            r#"{{"account_id":"a","session_token":"s","refresh_token":"r","expires_at_ns":"{SECRET}","logged_in_ever":true}}"#
        ),
    )
    .unwrap();
    for args in [
        &["doctor", "--check", "account.login"][..],
        &["doctor", "--json", "--check", "account.login"][..],
    ] {
        let out = sb.run(args);
        assert_eq!(out.status.code(), Some(1));
        let all = format!(
            "{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        );
        assert!(!all.contains(SECRET), "{args:?} leaked the token: {all}");
        assert!(all.contains("cannot be parsed at line 1 column"), "{all}");
    }
}
