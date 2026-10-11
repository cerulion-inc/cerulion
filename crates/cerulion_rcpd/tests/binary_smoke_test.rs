// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion-rcpd` binary refuses bad invocations before opening any
//! transport, and answers `initialize` on stdio when started properly.

use std::io::{BufRead, BufReader, Write};
use std::process::{Command, Stdio};
use std::time::Duration;

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

#[test]
fn stdio_answers_initialize_with_the_server_name() {
    let dir = tempfile::tempdir().expect("tempdir");
    let mut child = Command::new(env!("CARGO_BIN_EXE_cerulion-rcpd"))
        .arg("--manifest")
        .arg(manifest(dir.path()))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .expect("run cerulion-rcpd");
    let mut stdin = child.stdin.take().expect("stdin");
    let stdout = child.stdout.take().expect("stdout");
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let mut line = String::new();
        let read = BufReader::new(stdout).read_line(&mut line);
        let _ = tx.send(read.map(|_| line));
    });
    stdin
        .write_all(
            br#"{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-06-18","capabilities":{},"clientInfo":{"name":"smoke","version":"0"}}}"#,
        )
        .and_then(|()| stdin.write_all(b"\n"))
        .and_then(|()| stdin.flush())
        .expect("write initialize");
    let reply = rx.recv_timeout(Duration::from_secs(60));
    let _ = child.kill();
    let _ = child.wait();
    let line = reply.expect("a reply within 60 s").expect("read stdout");
    let reply: serde_json::Value = serde_json::from_str(&line).expect("one JSON-RPC line");
    assert_eq!(reply["id"], 1, "{line}");
    assert_eq!(
        reply["result"]["serverInfo"]["name"],
        cerulion_rcpd::SERVER_NAME,
        "{line}"
    );
}
