// SPDX-License-Identifier: AGPL-3.0-only
//! End to end over the real daemon socket: the three create verbs return the
//! new file's `version`, and `node.build` streams structured diagnostics.
//!
//! The build tests compile a dependency-free node crate with cargo, so they
//! need no network and no Cerulion runtime build; the create tests only write
//! files.

#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use cerulion_cli_engine::workspace;
use cerulion_wsd::daemon::{self, RunningWsd, WsdConfig};
use serde_json::{json, Value};
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::unix::{OwnedReadHalf, OwnedWriteHalf};
use tokio::net::UnixStream;

static NEXT_ID: AtomicU64 = AtomicU64::new(0);

type Reader = BufReader<OwnedReadHalf>;

struct Fixture {
    parent: PathBuf,
    root: PathBuf,
    socket: PathBuf,
    running: RunningWsd,
    /// The node crate `build_fixture` wrote (empty for `workspace_fixture`).
    node: String,
}

impl Fixture {
    async fn stop(mut self) {
        self.running.shutdown().await;
        let _ = std::fs::remove_dir_all(&self.parent);
        let _ = std::fs::remove_file(&self.socket);
    }
}

fn unique_path(prefix: &str) -> PathBuf {
    let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
    std::env::temp_dir().join(format!(
        "cerulion-wsd-cb-{prefix}-{}-{id}",
        std::process::id()
    ))
}

async fn start(parent: PathBuf, root: PathBuf) -> Fixture {
    let socket = unique_path("socket");
    let running = daemon::start(WsdConfig {
        socket_path: socket.clone(),
        ..WsdConfig::default()
    })
    .await
    .expect("daemon start");
    Fixture {
        parent,
        root,
        socket,
        running,
        node: String::new(),
    }
}

/// A scaffolded workspace, as `cerulion workspace create` makes one.
async fn workspace_fixture() -> Fixture {
    let parent = unique_path("parent");
    std::fs::create_dir_all(&parent).expect("parent");
    let root = workspace::workspace_create(&parent, "test")
        .expect("workspace")
        .root;
    start(parent, root).await
}

/// The smallest workspace `node.build` accepts, with one node crate that has
/// NO dependencies, so cargo needs neither the network nor a runtime build.
async fn build_fixture(lib_rs: &str, build_rs: Option<&str>) -> Fixture {
    let parent = unique_path("parent");
    let root = parent.join("ws");
    // A name of its own: cargo keys a workspace member's artifacts by name
    // and workspace-relative path, so two fixtures sharing one `target`
    // directory (a developer's `CARGO_TARGET_DIR`) must not share a name.
    let name = format!("probe_{}", NEXT_ID.fetch_add(1, Ordering::Relaxed));
    let node = root.join("nodes").join(&name);
    std::fs::create_dir_all(node.join("src")).expect("node dirs");
    std::fs::create_dir_all(root.join("graphs")).expect("graphs");
    std::fs::create_dir_all(root.join("schemas")).expect("schemas");
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nmembers = [\"nodes/*\"]\nresolver = \"2\"\n",
    )
    .expect("workspace manifest");
    std::fs::write(
        node.join("Cargo.toml"),
        format!("[package]\nname = \"{name}\"\nversion = \"0.1.0\"\nedition = \"2021\"\n"),
    )
    .expect("node manifest");
    std::fs::write(node.join("src").join("lib.rs"), lib_rs).expect("lib.rs");
    if let Some(build_rs) = build_rs {
        std::fs::write(node.join("build.rs"), build_rs).expect("build.rs");
    }
    let root = root.canonicalize().expect("canonical root");
    let mut fixture = start(parent, root).await;
    fixture.node = name;
    fixture
}

async fn connect(socket: &Path) -> (Reader, OwnedWriteHalf) {
    let stream = UnixStream::connect(socket).await.expect("connect");
    let (reader, writer) = stream.into_split();
    let mut reader = BufReader::new(reader);
    let mut hello = String::new();
    reader.read_line(&mut hello).await.expect("hello");
    (reader, writer)
}

async fn send(writer: &mut OwnedWriteHalf, request: Value) {
    let mut line = request.to_string();
    line.push('\n');
    writer
        .write_all(line.as_bytes())
        .await
        .expect("request write");
}

async fn read_line(reader: &mut Reader) -> Value {
    let mut line = String::new();
    reader.read_line(&mut line).await.expect("read");
    serde_json::from_str(line.trim_end())
        .unwrap_or_else(|error| panic!("not json ({error}): {line:?}"))
}

/// One request, one response line.
async fn call(socket: &Path, request: Value) -> Value {
    let (mut reader, mut writer) = connect(socket).await;
    send(&mut writer, request).await;
    read_line(&mut reader).await
}

/// Every line of a `node.build` reply: up to and including the one that ends
/// it (`event: "done"`, or an error response).
async fn build_lines(reader: &mut Reader) -> Vec<Value> {
    let mut lines = Vec::new();
    loop {
        let line = read_line(reader).await;
        let terminal = line["event"] == "done" || line.get("error").is_some();
        lines.push(line);
        if terminal {
            return lines;
        }
    }
}

fn diagnostics(lines: &[Value]) -> Vec<&Value> {
    lines
        .iter()
        .filter(|line| line["event"] == "diagnostic")
        .collect()
}

fn read_version(root: &Path, relative: &str) -> String {
    use sha2::{Digest, Sha256};
    format!(
        "{:x}",
        Sha256::digest(std::fs::read(root.join(relative)).expect("file"))
    )
}

fn error_code(response: &Value) -> &str {
    response["error"]["code"]
        .as_str()
        .unwrap_or_else(|| panic!("expected an error: {response}"))
}

#[tokio::test]
async fn graph_create_returns_the_version_of_the_new_file() {
    let fixture = workspace_fixture().await;
    let root = fixture.root.to_str().unwrap();
    let created = call(
        &fixture.socket,
        json!({"id": 1, "verb": "graph.create", "root": root, "name": "main", "prefix": "bot"}),
    )
    .await;
    assert_eq!(created["ok"], true, "{created}");
    assert_eq!(created["id"], 1);
    assert_eq!(
        created["result"]["version"],
        read_version(&fixture.root, "graphs/main.yaml")
    );
    assert_eq!(
        created["result"].as_object().unwrap().len(),
        1,
        "the result is exactly {{version}}: {created}"
    );

    // The returned version is the one `graph.read` reports, so it can be sent
    // back as `expect_version` without another read.
    let read = call(
        &fixture.socket,
        json!({"id": 2, "verb": "graph.read", "root": root, "graph": "main"}),
    )
    .await;
    assert_eq!(read["result"]["version"], created["result"]["version"]);
    assert_eq!(read["result"]["parsed"]["prefix"], "bot");

    let again = call(
        &fixture.socket,
        json!({"id": 3, "verb": "graph.create", "root": root, "name": "main"}),
    )
    .await;
    assert_eq!(error_code(&again), "invalid_request", "{again}");
    let escaping = call(
        &fixture.socket,
        json!({"id": 4, "verb": "graph.create", "root": root, "name": "../escape"}),
    )
    .await;
    assert_eq!(error_code(&escaping), "bad_request", "{escaping}");
    assert!(!fixture.parent.join("escape.yaml").exists());
    fixture.stop().await;
}

#[tokio::test]
async fn node_create_returns_the_source_version_and_the_node_stages_into_a_created_graph() {
    let fixture = workspace_fixture().await;
    let root = fixture.root.to_str().unwrap();
    let created = call(
        &fixture.socket,
        json!({"id": 1, "verb": "node.create", "root": root, "spec": {
            "node_type": "camera",
            "outputs": [{"schema": "geometry_msgs/Vector3", "name": "pose"}],
            "policy": {"kind": "period", "period_ms": 50}
        }}),
    )
    .await;
    assert_eq!(created["ok"], true, "{created}");
    assert_eq!(
        created["result"]["version"],
        read_version(&fixture.root, "nodes/camera/src/lib.rs")
    );

    let info = call(
        &fixture.socket,
        json!({"id": 2, "verb": "node.info", "root": root, "node_type": "camera"}),
    )
    .await;
    assert_eq!(info["result"]["version"], created["result"]["version"]);
    assert_eq!(info["result"]["outputs"][0]["name"], "pose");
    assert_eq!(
        info["result"]["policy"],
        json!({"kind": "period", "period_ms": 50})
    );

    // Created through the daemon, staged through the daemon: the whole
    // builder gesture without a CLI child.
    let graph = call(
        &fixture.socket,
        json!({"id": 3, "verb": "graph.create", "root": root, "name": "main"}),
    )
    .await;
    assert_eq!(graph["ok"], true, "{graph}");
    let staged = call(
        &fixture.socket,
        json!({"id": 4, "verb": "graph.stage_node", "root": root, "graph": "main",
               "node_type": "camera", "expect_version": graph["result"]["version"]}),
    )
    .await;
    assert_eq!(staged["ok"], true, "{staged}");
    assert!(staged["result"]["raw"].as_str().unwrap().contains("pose"));
    fixture.stop().await;
}

#[tokio::test]
async fn node_create_applies_the_cli_defaulting_and_refusals() {
    let fixture = workspace_fixture().await;
    let root = fixture.root.to_str().unwrap();

    // `-T` alone declares the trigger: the policy defaults to data_trigger.
    let sink = call(
        &fixture.socket,
        json!({"id": 1, "verb": "node.create", "root": root, "spec": {
            "node_type": "sink",
            "trigger_input": {"schema": "geometry_msgs/Vector3", "name": "scan"}
        }}),
    )
    .await;
    assert_eq!(sink["ok"], true, "{sink}");
    let info = call(
        &fixture.socket,
        json!({"id": 2, "verb": "node.info", "root": root, "node_type": "sink"}),
    )
    .await;
    assert_eq!(
        info["result"]["policy"],
        json!({"kind": "data_trigger", "input_name": "scan"})
    );
    assert_eq!(info["result"]["inputs"][0]["trigger"], true);

    // A node with no input and no policy has no firing rule: refused, and
    // nothing is created.
    let source_only = call(
        &fixture.socket,
        json!({"id": 3, "verb": "node.create", "root": root, "spec": {"node_type": "src"}}),
    )
    .await;
    assert_eq!(error_code(&source_only), "invalid_request", "{source_only}");
    assert!(source_only["error"]["message"]
        .as_str()
        .unwrap()
        .contains("source-only nodes"));
    assert!(!fixture.root.join("nodes/src").exists());

    let zero_period = call(
        &fixture.socket,
        json!({"id": 4, "verb": "node.create", "root": root, "spec": {
            "node_type": "spin", "policy": {"kind": "period", "period_ms": 0}}}),
    )
    .await;
    assert_eq!(error_code(&zero_period), "invalid_request", "{zero_period}");
    assert!(!fixture.root.join("nodes/spin").exists());

    // node.modify holds the same line: a zero duration is refused and the
    // node's policy is as it was.
    for (offset, policy) in [
        json!({"kind": "period", "period_ms": 0}),
        json!({"kind": "sync", "window_ms": 0}),
    ]
    .into_iter()
    .enumerate()
    {
        let zero = call(
            &fixture.socket,
            json!({"id": 20 + offset, "verb": "node.modify", "root": root,
                   "node_type": "sink", "op": {"op": "set_policy", "policy": policy}}),
        )
        .await;
        assert_eq!(error_code(&zero), "invalid_request", "{zero}");
    }
    let after = call(
        &fixture.socket,
        json!({"id": 22, "verb": "node.info", "root": root, "node_type": "sink"}),
    )
    .await;
    assert_eq!(
        after["result"]["policy"],
        json!({"kind": "data_trigger", "input_name": "scan"})
    );

    let duplicate = call(
        &fixture.socket,
        json!({"id": 5, "verb": "node.create", "root": root, "spec": {
            "node_type": "sink", "policy": {"kind": "external"}}}),
    )
    .await;
    assert_eq!(error_code(&duplicate), "invalid_request", "{duplicate}");

    let unknown_field = call(
        &fixture.socket,
        json!({"id": 6, "verb": "node.create", "root": root, "spec": {
            "node_type": "x", "policy": {"kind": "external"}, "outputs_typo": []}}),
    )
    .await;
    assert_eq!(error_code(&unknown_field), "bad_request", "{unknown_field}");

    let escaping = call(
        &fixture.socket,
        json!({"id": 7, "verb": "node.create", "root": root, "spec": {
            "node_type": "../escape", "policy": {"kind": "external"}}}),
    )
    .await;
    assert_eq!(error_code(&escaping), "bad_request", "{escaping}");
    fixture.stop().await;
}

#[tokio::test]
async fn schema_create_returns_the_version_and_the_schema_is_listed() {
    let fixture = workspace_fixture().await;
    let root = fixture.root.to_str().unwrap();
    let created = call(
        &fixture.socket,
        json!({"id": 1, "verb": "schema.create", "root": root, "spec": {"name": "lidar_scan"}}),
    )
    .await;
    assert_eq!(created["ok"], true, "{created}");
    assert_eq!(
        created["result"]["version"],
        read_version(&fixture.root, "schemas/lidar_scan.yaml")
    );
    let info = call(
        &fixture.socket,
        json!({"id": 2, "verb": "workspace.info", "root": root}),
    )
    .await;
    assert!(info["result"]["schemas"]
        .as_array()
        .unwrap()
        .contains(&json!("LidarScan")));

    let again = call(
        &fixture.socket,
        json!({"id": 3, "verb": "schema.create", "root": root, "spec": {"name": "lidar_scan"}}),
    )
    .await;
    assert_eq!(error_code(&again), "invalid_request", "{again}");
    let escaping = call(
        &fixture.socket,
        json!({"id": 4, "verb": "schema.create", "root": root, "spec": {"name": "../escape"}}),
    )
    .await;
    assert_eq!(error_code(&escaping), "bad_request", "{escaping}");
    assert!(!fixture.root.join("escape.yaml").exists());
    // A name with a colon, a newline or no characters would write a YAML key
    // that cannot be read.
    for (offset, name) in [
        "foo: bar", "foo\nbar", "", "a b", "scan#1", "123", "true", "null", "1e5",
    ]
    .into_iter()
    .enumerate()
    {
        let refused = call(
            &fixture.socket,
            json!({"id": 10 + offset, "verb": "schema.create", "root": root,
                   "spec": {"name": name}}),
        )
        .await;
        assert_eq!(error_code(&refused), "bad_request", "{name:?}: {refused}");
    }
    // Names the CLI takes are taken here too.
    for (offset, name) in ["1scan", "scan-2"].into_iter().enumerate() {
        let created = call(
            &fixture.socket,
            json!({"id": 30 + offset, "verb": "schema.create", "root": root,
                   "spec": {"name": name}}),
        )
        .await;
        assert!(
            created["result"]["version"].is_string(),
            "{name:?}: {created}"
        );
        assert!(fixture
            .root
            .join("schemas")
            .join(format!("{name}.yaml"))
            .exists());
    }
    fixture.stop().await;
}

#[tokio::test]
async fn node_build_streams_a_compile_error_as_a_structured_diagnostic() {
    let fixture = build_fixture(
        "pub fn broken() -> u32 {\n    let n: u32 = \"text\";\n    n\n}\n",
        None,
    )
    .await;
    let root = fixture.root.to_str().unwrap();
    let (mut reader, mut writer) = connect(&fixture.socket).await;
    send(
        &mut writer,
        json!({"id": 9, "verb": "node.build", "root": root, "node_type": fixture.node}),
    )
    .await;
    let lines = build_lines(&mut reader).await;

    for line in &lines {
        assert_eq!(line["id"], 9, "every line carries the request id: {line}");
    }
    let done = lines.last().unwrap();
    assert_eq!(*done, json!({"id": 9, "event": "done", "ok": false}));
    let errors: Vec<_> = diagnostics(&lines)
        .into_iter()
        .filter(|line| line["level"] == "error")
        .collect();
    assert_eq!(errors.len(), 1, "exactly one error: {lines:#?}");
    let error = errors[0];
    assert_eq!(error["file"], format!("nodes/{}/src/lib.rs", fixture.node));
    assert_eq!(error["line"], 2);
    assert_eq!(error["col"], 18);
    assert_eq!(error["end_line"], 2);
    assert_eq!(error["end_col"], 24);
    assert_eq!(error["message"], "mismatched types");
    assert_eq!(error["code"], "E0308");
    assert!(error["rendered"]
        .as_str()
        .unwrap()
        .contains("expected `u32`"));
    // rustc's "aborting due to 1 previous error" restates the event above.
    assert!(
        diagnostics(&lines)
            .iter()
            .all(|line| !line["message"].as_str().unwrap().starts_with("aborting")),
        "{lines:#?}"
    );

    // The connection is still good for the next request.
    send(
        &mut writer,
        json!({"id": 10, "verb": "workspace.info", "root": root}),
    )
    .await;
    let info = read_line(&mut reader).await;
    assert_eq!(info["id"], 10);
    assert_eq!(info["ok"], true, "{info}");
    fixture.stop().await;
}

#[tokio::test]
async fn node_build_reports_a_warning_and_succeeds() {
    let fixture = build_fixture("pub fn used() {\n    let unused = 1;\n}\n", None).await;
    let root = fixture.root.to_str().unwrap();
    let (mut reader, mut writer) = connect(&fixture.socket).await;
    send(
        &mut writer,
        json!({"id": 1, "verb": "node.build", "root": root, "node_type": fixture.node,
               "release": false}),
    )
    .await;
    let lines = build_lines(&mut reader).await;
    assert_eq!(
        *lines.last().unwrap(),
        json!({"id": 1, "event": "done", "ok": true})
    );
    let diagnostics = diagnostics(&lines);
    assert_eq!(
        diagnostics.len(),
        1,
        "one warning and nothing else: {lines:#?}"
    );
    assert_eq!(diagnostics[0]["level"], "warning");
    assert_eq!(
        diagnostics[0]["file"],
        format!("nodes/{}/src/lib.rs", fixture.node)
    );
    assert_eq!(diagnostics[0]["line"], 2);
    assert_eq!(diagnostics[0]["code"], "unused_variables");
    fixture.stop().await;
}

#[tokio::test]
async fn node_build_refusals_before_cargo_starts_are_ordinary_errors() {
    let fixture = build_fixture("pub fn ok() {}\n", None).await;
    let root = fixture.root.to_str().unwrap();
    let missing = call(
        &fixture.socket,
        json!({"id": 1, "verb": "node.build", "root": root, "node_type": "nope"}),
    )
    .await;
    assert_eq!(error_code(&missing), "not_found", "{missing}");
    assert_eq!(missing["id"], 1);
    assert!(missing.get("event").is_none());

    let escaping = call(
        &fixture.socket,
        json!({"id": 2, "verb": "node.build", "root": root, "node_type": "../x"}),
    )
    .await;
    assert_eq!(error_code(&escaping), "bad_request", "{escaping}");

    let no_workspace = call(
        &fixture.socket,
        json!({"id": 3, "verb": "node.build", "root": "/nonexistent/ws", "node_type": fixture.node}),
    )
    .await;
    assert_eq!(
        error_code(&no_workspace),
        "workspace_not_found",
        "{no_workspace}"
    );

    let unknown_field = call(
        &fixture.socket,
        json!({"id": 4, "verb": "node.build", "root": root, "node_type": fixture.node,
               "profile": "dev"}),
    )
    .await;
    assert_eq!(error_code(&unknown_field), "bad_request", "{unknown_field}");
    fixture.stop().await;
}

#[tokio::test]
async fn closing_the_connection_cancels_a_build_and_the_daemon_keeps_serving() {
    cancelled_by_hangup(false).await;
}

#[tokio::test]
async fn closing_the_connection_cancels_a_build_with_a_request_queued_behind_it() {
    cancelled_by_hangup(true).await;
}

#[tokio::test]
async fn a_request_queued_behind_a_build_is_answered_after_done() {
    let fixture = build_fixture("pub fn ok() {}\n", None).await;
    let root = fixture.root.to_str().unwrap();
    let (mut reader, mut writer) = connect(&fixture.socket).await;
    send(
        &mut writer,
        json!({"id": 1, "verb": "node.build", "root": root, "node_type": fixture.node}),
    )
    .await;
    send(
        &mut writer,
        json!({"id": 2, "verb": "workspace.info", "root": root}),
    )
    .await;
    let lines = build_lines(&mut reader).await;
    assert_eq!(
        *lines.last().unwrap(),
        json!({"id": 1, "event": "done", "ok": true})
    );
    let info = read_line(&mut reader).await;
    assert_eq!(info["id"], 2, "{info}");
    assert_eq!(info["ok"], true, "{info}");
    fixture.stop().await;
}

/// A client hangs up mid-build; `queue_next` first sends another request
/// behind the build, which must not hide the hangup.
async fn cancelled_by_hangup(queue_next: bool) {
    // The build script records its pid, then outlasts the test unless cancelled.
    let build_rs = r#"fn main() {
    let dir = std::path::Path::new(&std::env::var("CARGO_MANIFEST_DIR").unwrap()).join("../..");
    std::fs::write(dir.join("build_script.pid"), std::process::id().to_string()).unwrap();
    std::thread::sleep(std::time::Duration::from_secs(300));
}
"#;
    let fixture = build_fixture("pub fn ok() {}\n", Some(build_rs)).await;
    let root = fixture.root.to_str().unwrap();
    let (reader, mut writer) = connect(&fixture.socket).await;
    send(
        &mut writer,
        json!({"id": 1, "verb": "node.build", "root": root, "node_type": fixture.node}),
    )
    .await;

    if queue_next {
        send(
            &mut writer,
            json!({"id": 3, "verb": "workspace.info", "root": root}),
        )
        .await;
    }

    // The two waits below poll an OTHER process's observable state (its pid
    // file, its existence) under a deadline: nothing signals either event.
    let pid_file = fixture.root.join("build_script.pid");
    let deadline = Instant::now() + Duration::from_secs(120);
    let pid = loop {
        if let Some(pid) = std::fs::read_to_string(&pid_file)
            .ok()
            .and_then(|text| text.trim().parse::<i32>().ok())
        {
            break pid;
        }
        assert!(Instant::now() < deadline, "the build script never started");
        tokio::time::sleep(Duration::from_millis(50)).await;
    };
    // SAFETY: signal 0 only tests that the process exists.
    assert_eq!(
        unsafe { libc::kill(pid, 0) },
        0,
        "the build script is running"
    );

    drop(reader);
    drop(writer);
    let deadline = Instant::now() + Duration::from_secs(15);
    loop {
        // SAFETY: as above.
        if unsafe { libc::kill(pid, 0) } != 0 {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "the build script outlived its closed connection"
        );
        tokio::time::sleep(Duration::from_millis(50)).await;
    }

    let info = call(
        &fixture.socket,
        json!({"id": 2, "verb": "workspace.info", "root": root}),
    )
    .await;
    assert_eq!(info["ok"], true, "{info}");
    fixture.stop().await;
}
