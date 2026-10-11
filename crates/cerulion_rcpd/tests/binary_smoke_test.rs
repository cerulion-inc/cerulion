// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion-rcpd` binary refuses bad invocations before opening any
//! transport.

use std::process::Command;

fn manifest(dir: &std::path::Path) -> std::path::PathBuf {
    let path = dir.join("rcp.yaml");
    std::fs::write(&path, "robot: smoke\n").expect("write manifest");
    path
}

#[test]
fn http_without_a_token_refuses_to_start() {
    let dir = tempfile::tempdir().expect("tempdir");
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion-rcpd"))
        .arg("--manifest")
        .arg(manifest(dir.path()))
        .arg("--http")
        .env_remove(cerulion_rcpd::TOKEN_ENV)
        .output()
        .expect("run cerulion-rcpd");
    assert!(!out.status.success());
    assert!(out.stdout.is_empty(), "stdout stays clean");
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(stderr.contains(cerulion_rcpd::TOKEN_ENV), "{stderr}");
}

#[test]
fn stdio_and_http_together_are_refused() {
    let dir = tempfile::tempdir().expect("tempdir");
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion-rcpd"))
        .arg("--manifest")
        .arg(manifest(dir.path()))
        .args(["--stdio", "--http"])
        .output()
        .expect("run cerulion-rcpd");
    assert_eq!(out.status.code(), Some(2), "a usage error");
}
