#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};

use cerulion_cli_engine::graph_cmd;
use cerulion_cli_engine::node_cmd;
use cerulion_cli_engine::workspace;
use serde_json::Value;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::UnixStream;

use cerulion_wsd::daemon::{self, RunningWsd, WsdConfig, MAX_REQUEST_LINE_BYTES};
use cerulion_wsd::protocol::{HELLO_MARKER, PROTOCOL_VERSION};
use std::time::Duration;

static NEXT_ID: AtomicU64 = AtomicU64::new(0);

struct Fixture {
    root: PathBuf,
    socket: PathBuf,
    running: RunningWsd,
}

impl Fixture {
    async fn stop(mut self) {
        self.running.shutdown().await;
        let _ = std::fs::remove_dir_all(self.root.parent().expect("workspace parent"));
        let _ = std::fs::remove_file(self.socket);
    }
}

fn unique_path(prefix: &str) -> PathBuf {
    let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
    std::env::temp_dir().join(format!("cerulion-wsd-{prefix}-{}-{id}", std::process::id()))
}

async fn fixture() -> Fixture {
    fixture_with(None, Duration::from_secs(30)).await
}

/// A daemon whose `graph.validate` reads node libraries through `inspector`
/// (a script standing in for `cerulion-wsd --inspect-node`), killed after
/// `timeout`.
async fn fixture_with(inspector: Option<PathBuf>, timeout: Duration) -> Fixture {
    let parent = unique_path("workspace-parent");
    std::fs::create_dir_all(&parent).expect("workspace parent");
    let root = workspace::workspace_create(&parent, "test")
        .expect("workspace")
        .root;
    let socket = unique_path("socket");
    let running = daemon::start(WsdConfig {
        socket_path: socket.clone(),
        inspector_program: inspector,
        inspector_timeout: timeout,
    })
    .await
    .expect("daemon start");
    Fixture {
        root,
        socket,
        running,
    }
}

/// An executable shell script under the temp dir; the daemon runs it as
/// `<script> --inspect-node <cdylib>`, so the library path is `$2`.
fn inspector_script(body: &str) -> PathBuf {
    use std::os::unix::fs::PermissionsExt;
    let path = unique_path("inspector").with_extension("sh");
    std::fs::write(&path, format!("#!/bin/sh\n{body}\n")).expect("script");
    std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o755)).expect("chmod");
    path
}

/// A node `isoprobe` with a trigger input `cmd`, a graph wiring it to the
/// network ingress `/ext/cmd`, and a PLANTED library file where the resolver
/// looks — nothing in this test process ever loads it; the inspector script
/// stands in for the child that would.
fn ingress_workspace(root: &Path) {
    node_cmd::node_create(
        &root.join("nodes"),
        &root.join("Cargo.toml"),
        "isoprobe",
        None,
    )
    .expect("node create");
    node_cmd::node_modify_add_port(
        &root.join("nodes"),
        "isoprobe",
        "cmd",
        Some("geometry_msgs/Vector3"),
        false,
        true,
    )
    .expect("declare the input");
    std::fs::write(
        root.join("graphs").join("main.yaml"),
        "prefix: test\nnodes:\n  - id: probe0\n    type: isoprobe\n    inputs:\n      - name: cmd\n        source: /ext/cmd\nnetwork:\n  mode: peer\n  ingress:\n    - /ext/cmd\n",
    )
    .expect("graph yaml");
    let lib_dir = graph_cmd::cdylib_target_base(root).join("debug");
    std::fs::create_dir_all(&lib_dir).expect("target dir");
    std::fs::write(
        lib_dir.join(graph_cmd::cdylib_name("isoprobe")),
        b"not a real library",
    )
    .expect("plant");
}

fn ingress_check(response: &Value) -> Value {
    response["result"]["checks"]
        .as_array()
        .expect("checks")
        .iter()
        .find(|check| check["label"] == "network ingress")
        .cloned()
        .unwrap_or_else(|| panic!("no network ingress check in {response}"))
}

async fn connect(
    socket: &Path,
) -> (
    BufReader<tokio::net::unix::OwnedReadHalf>,
    tokio::net::unix::OwnedWriteHalf,
    Value,
) {
    let stream = UnixStream::connect(socket).await.expect("connect");
    let (reader, writer) = stream.into_split();
    let mut reader = BufReader::new(reader);
    let mut hello = String::new();
    reader.read_line(&mut hello).await.expect("hello read");
    let hello: Value = serde_json::from_str(hello.trim_end()).expect("hello json");
    (reader, writer, hello)
}

async fn request(
    reader: &mut BufReader<tokio::net::unix::OwnedReadHalf>,
    writer: &mut tokio::net::unix::OwnedWriteHalf,
    line: &str,
) -> Value {
    writer
        .write_all(line.as_bytes())
        .await
        .expect("request write");
    writer.write_all(b"\n").await.expect("request newline");
    let mut response = String::new();
    reader
        .read_line(&mut response)
        .await
        .expect("response read");
    serde_json::from_str(response.trim_end()).expect("response json")
}

fn root_json(root: &Path) -> String {
    serde_json::to_string(root.to_str().expect("utf8 root")).expect("root json")
}

fn create_graph(root: &Path) {
    graph_cmd::graph_create(&root.join("graphs"), "main", Some("test")).expect("graph create");
}

fn create_node(root: &Path) {
    node_cmd::node_create(
        &root.join("nodes"),
        &root.join("Cargo.toml"),
        "camera",
        None,
    )
    .expect("node create");
}

#[tokio::test]
async fn hello_names_the_daemon_and_advertises_protocol_v1() {
    let fixture = fixture().await;
    let (_, _, hello) = connect(&fixture.socket).await;
    assert_eq!(hello["hello"], HELLO_MARKER);
    assert_eq!(hello["protocol"], PROTOCOL_VERSION);
    assert_eq!(
        hello.as_object().expect("object").len(),
        2,
        "banner shape is pinned: {hello}"
    );
    fixture.stop().await;
}

#[tokio::test]
async fn workspace_info_lists_fresh_workspace() {
    let fixture = fixture().await;
    let (mut reader, mut writer, _) = connect(&fixture.socket).await;
    let response = request(
        &mut reader,
        &mut writer,
        &format!(
            r#"{{"id":1,"verb":"workspace.info","root":{}}}"#,
            root_json(&fixture.root)
        ),
    )
    .await;
    assert_eq!(response["ok"], true);
    assert_eq!(response["result"]["graphs"], serde_json::json!([]));
    assert_eq!(response["result"]["nodes"], serde_json::json!([]));
    fixture.stop().await;
}

#[tokio::test]
async fn graph_validate_reports_engine_check_labels() {
    let fixture = fixture().await;
    create_graph(&fixture.root);
    let (mut reader, mut writer, _) = connect(&fixture.socket).await;
    let response = request(
        &mut reader,
        &mut writer,
        &format!(
            r#"{{"id":1,"verb":"graph.validate","root":{},"graph":"main"}}"#,
            root_json(&fixture.root)
        ),
    )
    .await;
    assert_eq!(response["ok"], true);
    let labels = response["result"]["checks"]
        .as_array()
        .expect("checks")
        .iter()
        .map(|check| check["label"].as_str().expect("label"))
        .collect::<Vec<_>>();
    assert!(labels.contains(&"graph topology"));
    fixture.stop().await;
}

#[tokio::test]
async fn node_modify_add_port_round_trips_via_node_info() {
    let fixture = fixture().await;
    create_node(&fixture.root);
    let (mut reader, mut writer, _) = connect(&fixture.socket).await;
    let root = root_json(&fixture.root);
    let response = request(
        &mut reader,
        &mut writer,
        &format!(
            r#"{{"id":1,"verb":"node.modify","root":{root},"node_type":"camera","op":{{"op":"add_port","port_name":"image","is_output":true}}}}"#
        ),
    )
    .await;
    assert_eq!(response["ok"], true);
    assert_eq!(response["result"]["outputs"][0]["name"], "image");
    assert_eq!(
        response["result"]["outputs"][0]["backpressure"]["policy"],
        "drop_oldest"
    );

    let response = request(
        &mut reader,
        &mut writer,
        &format!(r#"{{"id":2,"verb":"node.info","root":{root},"node_type":"camera"}}"#),
    )
    .await;
    assert_eq!(response["ok"], true);
    assert_eq!(response["result"]["outputs"][0]["name"], "image");

    let response = request(
        &mut reader,
        &mut writer,
        &format!(
            r#"{{"id":3,"verb":"node.modify","root":{root},"node_type":"camera","op":{{"op":"set_policy","policy":{{"kind":"period","period_ms":25}}}}}}"#
        ),
    )
    .await;
    assert_eq!(
        response["result"]["policy"],
        serde_json::json!({"kind": "period", "period_ms": 25})
    );
    fixture.stop().await;
}

#[tokio::test]
async fn node_modify_data_trigger_policy_promotes_existing_input() {
    let fixture = fixture().await;
    create_node(&fixture.root);
    let (mut reader, mut writer, _) = connect(&fixture.socket).await;
    let root = root_json(&fixture.root);
    let add_input = request(
        &mut reader,
        &mut writer,
        &format!(
            r#"{{"id":1,"verb":"node.modify","root":{root},"node_type":"camera","op":{{"op":"add_port","port_name":"input","is_output":false}}}}"#
        ),
    )
    .await;
    assert_eq!(add_input["ok"], true);
    let response = request(
        &mut reader,
        &mut writer,
        &format!(
            r#"{{"id":2,"verb":"node.modify","root":{root},"node_type":"camera","op":{{"op":"set_policy","policy":{{"kind":"data_trigger","input_name":"input"}}}}}}"#
        ),
    )
    .await;
    assert_eq!(response["ok"], true);
    assert_eq!(
        response["result"]["policy"],
        serde_json::json!({"kind": "data_trigger", "input_name": "input"})
    );
    fixture.stop().await;
}

/// A staged node's `outputs:` are its DECLARED ports — name AND schema —
/// exactly what `cerulion node stage` writes. The daemon used to write the
/// request's own list (`schema: ""` for an omitted schema, nothing for an
/// empty list); a client-supplied `outputs` is now a `bad_request`.
#[tokio::test]
async fn graph_stage_node_writes_the_declared_ports_and_refuses_a_client_list() {
    let fixture = fixture().await;
    create_graph(&fixture.root);
    create_node(&fixture.root);
    node_cmd::node_modify_add_port(
        &fixture.root.join("nodes"),
        "camera",
        "image",
        Some("sensor_msgs/Image"),
        true,
        false,
    )
    .expect("declare an output");
    let (mut reader, mut writer, _) = connect(&fixture.socket).await;
    let root = root_json(&fixture.root);
    let response = request(
        &mut reader,
        &mut writer,
        &format!(
            r#"{{"id":1,"verb":"graph.stage_node","root":{root},"graph":"main","node_type":"camera","node_id":"cam1"}}"#
        ),
    )
    .await;
    assert_eq!(response["ok"], true, "{response}");
    let raw = response["result"]["raw"].as_str().expect("raw");
    assert!(raw.contains("id: cam1"), "{raw}");
    assert!(
        raw.contains("- name: image") && raw.contains("schema: sensor_msgs/Image"),
        "the declared output must be staged with its schema:\n{raw}"
    );
    let on_disk = std::fs::read_to_string(fixture.root.join("graphs").join("main.yaml")).unwrap();
    assert_eq!(on_disk, raw, "the returned raw YAML is what was written");
    assert!(response["result"]["version"].as_str().is_some());

    let rejected = request(
        &mut reader,
        &mut writer,
        &format!(
            r#"{{"id":2,"verb":"graph.stage_node","root":{root},"graph":"main","node_type":"camera","node_id":"cam2","outputs":[{{"name":"image"}}]}}"#
        ),
    )
    .await;
    assert_eq!(rejected["ok"], false);
    assert_eq!(rejected["error"]["code"], "bad_request");
    assert!(
        rejected["error"]["message"]
            .as_str()
            .unwrap()
            .contains("unknown field `outputs`"),
        "{rejected}"
    );
    let after = std::fs::read_to_string(fixture.root.join("graphs").join("main.yaml")).unwrap();
    assert_eq!(after, on_disk, "a rejected request writes nothing");
    fixture.stop().await;
}

#[tokio::test]
async fn graph_stage_node_with_stale_expect_version_is_version_conflict_and_writes_nothing() {
    let fixture = fixture().await;
    create_graph(&fixture.root);
    let (mut reader, mut writer, _) = connect(&fixture.socket).await;
    let root = root_json(&fixture.root);
    let before = request(
        &mut reader,
        &mut writer,
        &format!(r#"{{"id":1,"verb":"graph.read","root":{root},"graph":"main"}}"#),
    )
    .await;
    let raw_before = before["result"]["raw"].clone();
    let response = request(
        &mut reader,
        &mut writer,
        &format!(
            r#"{{"id":2,"verb":"graph.stage_node","root":{root},"graph":"main","node_type":"camera","node_id":"cam1","expect_version":"stale"}}"#
        ),
    )
    .await;
    assert_eq!(response["ok"], false);
    assert_eq!(response["error"]["code"], "version_conflict");
    let after = request(
        &mut reader,
        &mut writer,
        &format!(r#"{{"id":3,"verb":"graph.read","root":{root},"graph":"main"}}"#),
    )
    .await;
    assert_eq!(after["result"]["raw"], raw_before);
    fixture.stop().await;
}

#[tokio::test]
async fn node_modify_with_matching_expect_version_succeeds_and_returns_new_version() {
    let fixture = fixture().await;
    create_node(&fixture.root);
    let (mut reader, mut writer, _) = connect(&fixture.socket).await;
    let root = root_json(&fixture.root);
    let before = request(
        &mut reader,
        &mut writer,
        &format!(r#"{{"id":1,"verb":"node.info","root":{root},"node_type":"camera"}}"#),
    )
    .await;
    let old_version = before["result"]["version"].as_str().expect("version");
    let response = request(
        &mut reader,
        &mut writer,
        &format!(
            r#"{{"id":2,"verb":"node.modify","root":{root},"node_type":"camera","expect_version":"{old_version}","op":{{"op":"add_port","port_name":"image","is_output":true}}}}"#
        ),
    )
    .await;
    assert_eq!(response["ok"], true);
    let new_version = response["result"]["version"].as_str().expect("new version");
    assert_ne!(new_version, old_version);
    fixture.stop().await;
}

#[tokio::test]
async fn malformed_json_is_bad_request_with_a_null_id() {
    let fixture = fixture().await;
    let (mut reader, mut writer, _) = connect(&fixture.socket).await;
    let response = request(&mut reader, &mut writer, "{not json").await;
    assert_eq!(response["id"], Value::Null);
    assert_eq!(response["error"]["code"], "bad_request");
    fixture.stop().await;
}

/// A line of exactly `MAX_REQUEST_LINE_BYTES` is a legal (if silly) request
/// and is processed; one byte more is answered with a structured
/// `bad_request` (no id to echo) and the connection is closed, while the
/// daemon keeps serving new connections.
#[tokio::test]
async fn an_over_long_line_is_a_bad_request_then_a_closed_connection() {
    let fixture = fixture().await;
    let root = root_json(&fixture.root);
    let prefix = format!(r#"{{"id":5,"verb":"workspace.info","root":{root},"pad":""#);
    let suffix = r#""}"#;
    let at_bound = {
        let padding = MAX_REQUEST_LINE_BYTES - prefix.len() - suffix.len();
        format!("{prefix}{}{suffix}", "x".repeat(padding))
    };
    assert_eq!(at_bound.len(), MAX_REQUEST_LINE_BYTES);
    let (mut reader, mut writer, _) = connect(&fixture.socket).await;
    let response = request(&mut reader, &mut writer, &at_bound).await;
    // Processed: it reaches the parser (which rejects the unknown `pad` field).
    assert_eq!(response["id"], 5, "{response}");
    assert_eq!(response["error"]["code"], "bad_request");

    let over = format!("{at_bound}x");
    let (mut reader, mut writer, _) = connect(&fixture.socket).await;
    writer.write_all(over.as_bytes()).await.expect("write");
    writer.write_all(b"\n").await.expect("newline");
    let mut line = String::new();
    reader.read_line(&mut line).await.expect("rejection line");
    let rejection: Value = serde_json::from_str(line.trim_end()).expect("rejection json");
    assert_eq!(rejection["id"], Value::Null);
    assert_eq!(rejection["error"]["code"], "bad_request");
    assert!(
        rejection["error"]["message"]
            .as_str()
            .unwrap()
            .contains("maximum length"),
        "{rejection}"
    );
    let mut rest = String::new();
    let eof = reader
        .read_line(&mut rest)
        .await
        .expect("read after rejection");
    assert_eq!(
        eof, 0,
        "the connection must be closed after the rejection: {rest:?}"
    );

    let (mut reader, mut writer, _) = connect(&fixture.socket).await;
    let alive = request(
        &mut reader,
        &mut writer,
        &format!(r#"{{"id":6,"verb":"workspace.info","root":{root}}}"#),
    )
    .await;
    assert_eq!(
        alive["ok"], true,
        "the daemon keeps serving after an over-long line"
    );
    fixture.stop().await;
}

#[tokio::test]
async fn unknown_verb_is_rejected() {
    let fixture = fixture().await;
    let (mut reader, mut writer, _) = connect(&fixture.socket).await;
    let response = request(
        &mut reader,
        &mut writer,
        r#"{"id":1,"verb":"workspace.nope","root":"/tmp"}"#,
    )
    .await;
    assert_eq!(response["error"]["code"], "unknown_verb");
    fixture.stop().await;
}

#[tokio::test]
async fn missing_root_is_workspace_not_found() {
    let fixture = fixture().await;
    let (mut reader, mut writer, _) = connect(&fixture.socket).await;
    let response = request(
        &mut reader,
        &mut writer,
        r#"{"id":1,"verb":"workspace.info","root":"/definitely/not/a/workspace"}"#,
    )
    .await;
    assert_eq!(response["error"]["code"], "workspace_not_found");
    fixture.stop().await;
}

#[tokio::test]
async fn a_missing_graph_or_node_is_not_found_and_an_engine_refusal_is_invalid_request() {
    let fixture = fixture().await;
    create_node(&fixture.root);
    let (mut reader, mut writer, _) = connect(&fixture.socket).await;
    let root = root_json(&fixture.root);
    let response = request(
        &mut reader,
        &mut writer,
        &format!(r#"{{"id":1,"verb":"graph.read","root":{root},"graph":"missing"}}"#),
    )
    .await;
    assert_eq!(response["error"]["code"], "not_found", "{response}");
    let response = request(
        &mut reader,
        &mut writer,
        &format!(r#"{{"id":2,"verb":"node.info","root":{root},"node_type":"ghost"}}"#),
    )
    .await;
    assert_eq!(response["error"]["code"], "not_found", "{response}");
    // Promoting an input the node does not have is the engine's own refusal.
    let response = request(
        &mut reader,
        &mut writer,
        &format!(
            r#"{{"id":3,"verb":"node.modify","root":{root},"node_type":"camera","op":{{"op":"set_policy","policy":{{"kind":"data_trigger","input_name":"absent"}}}}}}"#
        ),
    )
    .await;
    assert_eq!(response["error"]["code"], "invalid_request", "{response}");
    assert!(
        response["error"]["message"]
            .as_str()
            .unwrap()
            .contains("absent"),
        "{response}"
    );
    fixture.stop().await;
}

#[tokio::test]
async fn concurrent_connections_are_independent() {
    let fixture = fixture().await;
    let root = root_json(&fixture.root);
    let socket_a = fixture.socket.clone();
    let root_a = root.clone();
    let first = async move {
        let (mut reader, mut writer, _) = connect(&socket_a).await;
        request(
            &mut reader,
            &mut writer,
            &format!(r#"{{"id":10,"verb":"workspace.info","root":{root_a}}}"#),
        )
        .await
    };
    let socket_b = fixture.socket.clone();
    let second = async move {
        let (mut reader, mut writer, _) = connect(&socket_b).await;
        request(
            &mut reader,
            &mut writer,
            &format!(r#"{{"id":11,"verb":"workspace.info","root":{root}}}"#),
        )
        .await
    };
    let (first, second) = tokio::join!(first, second);
    assert_eq!(first["id"], 10);
    assert_eq!(second["id"], 11);
    assert_eq!(first["ok"], true);
    assert_eq!(second["ok"], true);
    fixture.stop().await;
}

#[tokio::test]
async fn graph_validate_is_deterministic() {
    let fixture = fixture().await;
    create_graph(&fixture.root);
    let (mut reader, mut writer, _) = connect(&fixture.socket).await;
    let root = root_json(&fixture.root);
    let first = request(
        &mut reader,
        &mut writer,
        &format!(r#"{{"id":1,"verb":"graph.validate","root":{root},"graph":"main"}}"#),
    )
    .await;
    let second = request(
        &mut reader,
        &mut writer,
        &format!(r#"{{"id":2,"verb":"graph.validate","root":{root},"graph":"main"}}"#),
    )
    .await;
    assert_eq!(first["ok"], true, "{first}");
    assert_eq!(first["result"], second["result"]);
    fixture.stop().await;
}

#[tokio::test]
async fn mutations_serialize_across_two_daemons_on_one_workspace() {
    let fixture = fixture().await;
    create_node(&fixture.root);
    let second_socket = unique_path("second-socket");
    let mut second = daemon::start(WsdConfig {
        socket_path: second_socket.clone(),
        ..WsdConfig::default()
    })
    .await
    .expect("second daemon start");
    let root = root_json(&fixture.root);
    let root_a = root.clone();
    let socket_a = fixture.socket.clone();
    let first = async move {
        let (mut reader, mut writer, _) = connect(&socket_a).await;
        request(
            &mut reader,
            &mut writer,
            &format!(
                r#"{{"id":1,"verb":"node.modify","root":{root_a},"node_type":"camera","op":{{"op":"add_port","port_name":"left","is_output":true}}}}"#
            ),
        )
        .await
    };
    let root_b = root.clone();
    let second_socket_for_request = second_socket.clone();
    let second_request = async move {
        let (mut reader, mut writer, _) = connect(&second_socket_for_request).await;
        request(
            &mut reader,
            &mut writer,
            &format!(
                r#"{{"id":2,"verb":"node.modify","root":{root_b},"node_type":"camera","op":{{"op":"add_port","port_name":"right","is_output":true}}}}"#
            ),
        )
        .await
    };
    let (first, second_response) = tokio::join!(first, second_request);
    assert_eq!(first["ok"], true);
    assert_eq!(second_response["ok"], true);

    let (mut reader, mut writer, _) = connect(&fixture.socket).await;
    let info = request(
        &mut reader,
        &mut writer,
        &format!(r#"{{"id":3,"verb":"node.info","root":{root},"node_type":"camera"}}"#),
    )
    .await;
    let names = info["result"]["outputs"]
        .as_array()
        .expect("outputs")
        .iter()
        .map(|port| port["name"].as_str().expect("port name"))
        .collect::<Vec<_>>();
    assert!(names.contains(&"left"));
    assert!(names.contains(&"right"));
    second.shutdown().await;
    fixture.stop().await;
}

/// The measured defect: a node library that aborts on load killed the daemon
/// (SIGABRT), cut every client off and left the socket behind. With the
/// inspection in a child, the crash is a FAILING `network ingress` check that
/// names the signal, and the daemon keeps serving.
#[tokio::test]
async fn a_node_library_that_aborts_on_load_fails_the_ingress_check_and_the_daemon_survives() {
    let fixture = fixture_with(
        Some(inspector_script("kill -ABRT $$")),
        Duration::from_secs(30),
    )
    .await;
    ingress_workspace(&fixture.root);
    let (mut reader, mut writer, _) = connect(&fixture.socket).await;
    let root = root_json(&fixture.root);
    let response = request(
        &mut reader,
        &mut writer,
        &format!(r#"{{"id":1,"verb":"graph.validate","root":{root},"graph":"main"}}"#),
    )
    .await;
    assert_eq!(response["ok"], true, "{response}");
    let check = ingress_check(&response);
    assert_eq!(check["passed"], false, "{check}");
    let detail = check["detail"].as_str().expect("detail");
    assert!(
        detail.contains("CRASHED") && detail.contains("SIGABRT"),
        "{detail}"
    );
    assert!(
        detail.contains(&graph_cmd::cdylib_name("isoprobe")) && detail.contains("'isoprobe'"),
        "{detail}"
    );
    // The daemon is alive on the SAME connection and on a new one.
    let again = request(
        &mut reader,
        &mut writer,
        &format!(r#"{{"id":2,"verb":"workspace.info","root":{root}}}"#),
    )
    .await;
    assert_eq!(again["ok"], true, "{again}");
    let (mut reader, mut writer, _) = connect(&fixture.socket).await;
    let fresh = request(
        &mut reader,
        &mut writer,
        &format!(r#"{{"id":3,"verb":"workspace.info","root":{root}}}"#),
    )
    .await;
    assert_eq!(fresh["ok"], true, "{fresh}");
    fixture.stop().await;
}

/// The anti-tautology half: an inspector that answers with a real info
/// document (a declared `cmd` with a schema hash) makes the gate PASS through
/// the same child path — the failing arm above is not "the child always fails".
#[tokio::test]
async fn a_healthy_inspection_passes_the_ingress_check_through_the_child() {
    let script = format!(
        r#"[ "$1" = --inspect-node ] || exit 9; case "$2" in *{}) ;; *) exit 8;; esac; printf '%s' '{{"inputs":[{{"name":"cmd","schema_hash":4242,"trigger":true}}],"outputs":[]}}'"#,
        graph_cmd::cdylib_name("isoprobe")
    );
    let script = script.as_str();
    let fixture = fixture_with(Some(inspector_script(script)), Duration::from_secs(30)).await;
    ingress_workspace(&fixture.root);
    let (mut reader, mut writer, _) = connect(&fixture.socket).await;
    let root = root_json(&fixture.root);
    let response = request(
        &mut reader,
        &mut writer,
        &format!(r#"{{"id":1,"verb":"graph.validate","root":{root},"graph":"main"}}"#),
    )
    .await;
    assert_eq!(response["ok"], true, "{response}");
    let check = ingress_check(&response);
    assert_eq!(check["passed"], true, "{check}");
    assert!(
        check["detail"]
            .as_str()
            .unwrap()
            .contains("1 topic(s) resolve"),
        "{check}"
    );
    fixture.stop().await;
}

/// A library whose constructors never return must not wedge the daemon's
/// request: the child is killed at the deadline and the check says so.
#[tokio::test]
async fn a_hanging_inspection_is_killed_at_the_deadline_and_reported() {
    let fixture = fixture_with(
        Some(inspector_script("sleep 60")),
        Duration::from_millis(500),
    )
    .await;
    ingress_workspace(&fixture.root);
    let (mut reader, mut writer, _) = connect(&fixture.socket).await;
    let root = root_json(&fixture.root);
    let started = std::time::Instant::now();
    let response = request(
        &mut reader,
        &mut writer,
        &format!(r#"{{"id":1,"verb":"graph.validate","root":{root},"graph":"main"}}"#),
    )
    .await;
    assert!(
        started.elapsed() < Duration::from_secs(20),
        "the request must return at the deadline"
    );
    let check = ingress_check(&response);
    assert_eq!(check["passed"], false, "{check}");
    assert!(
        check["detail"]
            .as_str()
            .unwrap()
            .contains("did not finish inspection"),
        "{check}"
    );
    fixture.stop().await;
}
