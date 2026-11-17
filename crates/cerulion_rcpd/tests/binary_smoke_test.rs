// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion-rcpd` binary refuses bad invocations before opening any
//! transport, and answers `initialize` on stdio when started properly.

use std::io::{BufRead, BufReader, Write};
use std::process::{Command, Stdio};
use std::time::Duration;

fn manifest(dir: &std::path::Path) -> std::path::PathBuf {
    let path = dir.join("rcp.yaml");
    let prefix = format!("rcpd_smoke_init_{}", std::process::id());
    std::fs::write(
        &path,
        format!(
            "robot: smoke\njournal:\n  calls_topic: /{prefix}/calls\n  results_topic: /{prefix}/results\n"
        ),
    )
    .expect("write manifest");
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

/// Start the binary on `manifest_yaml`, initialize, and return the
/// `tools/list` reply's tools.
fn tools_over_stdio(manifest_yaml: &str) -> Vec<serde_json::Value> {
    tools_and_stderr_over_stdio(manifest_yaml).0
}

/// [`tools_over_stdio`], and what the binary wrote to stderr by then.
fn tools_and_stderr_over_stdio(manifest_yaml: &str) -> (Vec<serde_json::Value>, String) {
    let dir = tempfile::tempdir().expect("tempdir");
    let path = dir.path().join("rcp.yaml");
    std::fs::write(&path, manifest_yaml).expect("write manifest");
    let stderr_path = dir.path().join("stderr.log");
    let stderr = std::fs::File::create(&stderr_path).expect("stderr file");
    let mut child = Command::new(env!("CARGO_BIN_EXE_cerulion-rcpd"))
        .arg("--manifest")
        .arg(&path)
        .env_remove(cerulion_rcpd::eye::JEV_KEY_FILE_ENV)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(stderr)
        .spawn()
        .expect("run cerulion-rcpd");
    let mut stdin = child.stdin.take().expect("stdin");
    let stdout = child.stdout.take().expect("stdout");
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        for line in BufReader::new(stdout).lines() {
            let Ok(line) = line else { return };
            if tx.send(line).is_err() {
                return;
            }
        }
    });
    let mut send = |text: &str| {
        stdin
            .write_all(text.as_bytes())
            .and_then(|()| stdin.write_all(b"\n"))
            .and_then(|()| stdin.flush())
            .expect("write request");
    };
    send(
        r#"{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-06-18","capabilities":{},"clientInfo":{"name":"smoke","version":"0"}}}"#,
    );
    let first = rx.recv_timeout(Duration::from_secs(60));
    send(r#"{"jsonrpc":"2.0","method":"notifications/initialized"}"#);
    send(r#"{"jsonrpc":"2.0","id":2,"method":"tools/list"}"#);
    let mut tools = None;
    while let Ok(line) = rx.recv_timeout(Duration::from_secs(60)) {
        let reply: serde_json::Value = serde_json::from_str(&line).expect("one JSON-RPC line");
        if reply["id"] == 2 {
            tools = reply["result"]["tools"].as_array().cloned();
            break;
        }
    }
    let _ = child.kill();
    let _ = child.wait();
    first.expect("an initialize reply within 60 s");
    let stderr = std::fs::read_to_string(&stderr_path).unwrap_or_default();
    (tools.expect("a tools/list reply within 60 s"), stderr)
}

/// A manifest on journal topics private to this process and `tag`, with one
/// irreversible tool.
fn cut_manifest(tag: &str) -> String {
    let prefix = format!("rcpd_smoke_{tag}_{}", std::process::id());
    format!("robot: smoke\njournal:\n  calls_topic: /{prefix}/calls\n  results_topic: /{prefix}/results\n{CUT_TOOL}")
}

const CUT_TOOL: &str = "tools:
  - name: arm.cut
    description: cut the cable
    safety_class: irreversible
    input_schema: { type: object, properties: {} }
";

fn names(tools: &[serde_json::Value]) -> Vec<&str> {
    tools.iter().filter_map(|t| t["name"].as_str()).collect()
}

#[test]
fn the_binary_serves_an_irreversible_tool_labelled_and_estop_clear_by_default() {
    let tools = tools_over_stdio(&cut_manifest("default"));
    assert_eq!(names(&tools), ["arm.cut", "estop", "estop_clear"]);
    assert_eq!(
        tools[0]["annotations"]["destructiveHint"], true,
        "{}",
        tools[0]
    );
}

#[test]
fn the_binary_applies_operator_clearing_from_the_manifest() {
    let tools = tools_over_stdio(&format!(
        "{}limits:\n  estop_clear: operator\n",
        cut_manifest("operator")
    ));
    assert_eq!(names(&tools), ["arm.cut", "estop"]);
}

#[test]
fn the_binary_refuses_an_irreversible_tool_under_the_refuse_switch() {
    let dir = tempfile::tempdir().expect("tempdir");
    let path = dir.path().join("rcp.yaml");
    std::fs::write(
        &path,
        format!(
            "{}limits:\n  irreversible: refuse\n",
            cut_manifest("refuse")
        ),
    )
    .expect("write manifest");
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion-rcpd"))
        .arg("--manifest")
        .arg(&path)
        .stdin(Stdio::null())
        .output()
        .expect("run cerulion-rcpd");
    assert_eq!(out.status.code(), Some(1), "the load fails");
    assert!(out.stdout.is_empty(), "stdout stays clean");
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        stderr.contains("arm.cut") && stderr.contains("limits.irreversible"),
        "{stderr}"
    );
}

#[test]
fn jev_off_on_and_status_drive_the_switch_file() {
    let dir = tempfile::tempdir().expect("tempdir");
    let prefix = format!("rcpd_smoke_jev_{}", std::process::id());
    let path = dir.path().join("rcp.yaml");
    std::fs::write(
        &path,
        format!(
            "robot: smoke
eyes:
  jev:
    frame_topic: /{prefix}/wrist
    ask_topic: /{prefix}/ask
    answer_topic: /{prefix}/answer
    question_sets:
      - name: press
        state: A wrist camera frame.
        questions:
          rim: {{ type: box, instructions: The rim in `frame`. }}
"
        ),
    )
    .expect("write manifest");
    let run = |args: &[&str]| {
        let out = Command::new(env!("CARGO_BIN_EXE_cerulion-rcpd"))
            .arg("jev")
            .args(args)
            .arg("--manifest")
            .arg(&path)
            .env_remove(cerulion_rcpd::eye::JEV_KEY_FILE_ENV)
            .output()
            .expect("run cerulion-rcpd");
        assert!(
            out.status.success(),
            "{args:?}: {}",
            String::from_utf8_lossy(&out.stderr)
        );
        String::from_utf8_lossy(&out.stdout).into_owned()
    };
    let status = run(&["status"]);
    assert!(status.contains("jev: off"), "no key on file: {status}");
    assert!(
        status.contains("credential: credential_missing"),
        "{status}"
    );
    assert!(status.contains("model: typesafe/jev-1.14"), "{status}");
    let off = run(&["off"]);
    assert!(off.contains("Jev is off"), "{off}");
    let switch = dir.path().join(".cerulion/rcp/jev.switch");
    assert_eq!(
        std::fs::read_to_string(&switch).expect("switch").trim(),
        "off"
    );
    assert!(run(&["status"]).contains("switch: off"));
    let on = run(&["on"]);
    assert!(
        on.contains("cerulion-rcpd jev off"),
        "the notice names the off switch: {on}"
    );
    assert_eq!(
        std::fs::read_to_string(&switch).expect("switch").trim(),
        "on"
    );
}

#[test]
fn an_eye_that_cannot_open_its_ports_leaves_the_tools_served() {
    let prefix = format!("rcpd_smoke_eye_{}", std::process::id());
    // A frame topic longer than the transport allows: the eye's ports fail.
    let long = "f".repeat(300);
    let yaml = format!(
        "robot: smoke
journal:
  calls_topic: /{prefix}/calls
  results_topic: /{prefix}/results
eyes:
  jev:
    base_url: http://127.0.0.1:9/api/v1
    frame_topic: /{prefix}/{long}
    ask_topic: /{prefix}/ask
    answer_topic: /{prefix}/answer
    question_sets:
      - name: press
        state: A wrist camera frame.
        questions:
          rim: {{ type: box, instructions: The rim in `frame`. }}
{CUT_TOOL}"
    );
    let (tools, stderr) = tools_and_stderr_over_stdio(&yaml);
    assert_eq!(
        names(&tools),
        ["arm.cut", "estop", "estop_clear"],
        "{stderr}"
    );
    assert!(
        stderr.contains("the Jev eye did not start; serving without it"),
        "{stderr}"
    );
}
