#![cfg(unix)]
//! `graph.wire`, `graph.unwire` and `graph.unstage` end to end over the
//! daemon's socket: the version check, the typed refusals, and that a refusal
//! writes nothing.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

use cerulion_cli_engine::node_cmd;
use cerulion_cli_engine::workspace;
use cerulion_wsd::daemon::{self, RunningWsd, WsdConfig};
use cerulion_wsd::protocol::PROTOCOL_VERSION;
use serde_json::Value;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::UnixStream;

static NEXT_ID: AtomicU64 = AtomicU64::new(0);

type Reader = BufReader<tokio::net::unix::OwnedReadHalf>;
type Writer = tokio::net::unix::OwnedWriteHalf;

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
    std::env::temp_dir().join(format!(
        "cerulion-wsd-wire-{prefix}-{}-{id}",
        std::process::id()
    ))
}

/// A workspace with `camera` (output `image`), `detector` (input `image`,
/// output `boxes`) and `logger` (input `data`, a schema the camera does not
/// produce), plus a daemon serving it.
async fn fixture() -> Fixture {
    let parent = unique_path("workspace-parent");
    std::fs::create_dir_all(&parent).expect("workspace parent");
    let root = workspace::workspace_create(&parent, "test")
        .expect("workspace")
        .root;
    for (node, ports) in [
        ("camera", vec![("image", "sensor_msgs/Image", true)]),
        (
            "detector",
            vec![
                ("image", "sensor_msgs/Image", false),
                ("boxes", "geometry_msgs/Vector3", true),
            ],
        ),
        ("logger", vec![("data", "geometry_msgs/Vector3", false)]),
    ] {
        node_cmd::node_create(&root.join("nodes"), &root.join("Cargo.toml"), node, None)
            .expect("node create");
        for (name, schema, is_output) in ports {
            node_cmd::node_modify_add_port(
                &root.join("nodes"),
                node,
                name,
                Some(schema),
                is_output,
                false,
            )
            .expect("declare port");
        }
    }
    // Hand-authored: comments and a `network:` block that must survive.
    std::fs::write(
        root.join("graphs").join("main.yaml"),
        "# Hand-authored graph.\nprefix: test\nnodes:\n  # first\n  - id: cam\n    type: camera\n    outputs:\n      - name: image\n        schema: sensor_msgs/Image\n  - id: det\n    type: detector\n    outputs:\n      - name: boxes\n        schema: geometry_msgs/Vector3\n  - id: log\n    type: logger\n\n# kept\nnetwork:\n  mode: disabled\n",
    )
    .expect("graph yaml");
    let socket = unique_path("socket");
    let running = daemon::start(WsdConfig {
        socket_path: socket.clone(),
        inspector_program: None,
        inspector_timeout: Duration::from_secs(30),
    })
    .await
    .expect("daemon start");
    Fixture {
        root,
        socket,
        running,
    }
}

async fn connect(socket: &Path) -> (Reader, Writer) {
    let stream = UnixStream::connect(socket).await.expect("connect");
    let (reader, writer) = stream.into_split();
    let mut reader = BufReader::new(reader);
    let mut hello = String::new();
    reader.read_line(&mut hello).await.expect("hello read");
    let hello: Value = serde_json::from_str(hello.trim_end()).expect("hello json");
    assert_eq!(hello["protocol"], PROTOCOL_VERSION);
    assert_eq!(
        PROTOCOL_VERSION, 1,
        "the edit verbs do not bump the protocol"
    );
    (reader, writer)
}

async fn call(reader: &mut Reader, writer: &mut Writer, line: &str) -> Value {
    writer.write_all(line.as_bytes()).await.expect("write");
    writer.write_all(b"\n").await.expect("newline");
    let mut response = String::new();
    reader.read_line(&mut response).await.expect("read");
    serde_json::from_str(response.trim_end()).expect("response json")
}

fn graph_path(root: &Path) -> PathBuf {
    root.join("graphs").join("main.yaml")
}

fn edit_line(id: u64, verb: &str, root: &Path, extra: &str) -> String {
    let root = serde_json::to_string(root.to_str().expect("utf8")).unwrap();
    format!(r#"{{"id":{id},"verb":"{verb}","root":{root},"graph":"main",{extra}}}"#)
}

const CAM_TO_DET: &str =
    r#""from":{"node":"cam","port":"image"},"to":{"node":"det","port":"image"}"#;

#[tokio::test]
async fn wire_unwire_and_unstage_round_trip_with_the_version_check() {
    let fixture = fixture().await;
    let root = fixture.root.clone();
    let original = std::fs::read_to_string(graph_path(&root)).unwrap();
    let (mut reader, mut writer) = connect(&fixture.socket).await;

    let read = call(
        &mut reader,
        &mut writer,
        &edit_line(1, "graph.read", &root, "\"x\":0").replace(",\"x\":0", ""),
    )
    .await;
    let v0 = read["result"]["version"].as_str().unwrap().to_string();

    // graph.wire with the current version: written, bytes preserved.
    let wired = call(
        &mut reader,
        &mut writer,
        &edit_line(
            2,
            "graph.wire",
            &root,
            &format!(r#"{CAM_TO_DET},"expect_version":"{v0}""#),
        ),
    )
    .await;
    assert_eq!(wired["ok"], true, "{wired}");
    let raw = wired["result"]["raw"].as_str().unwrap();
    assert_eq!(
        raw,
        original.replace(
            "    type: detector\n",
            "    type: detector\n    inputs:\n      - name: image\n        source: cam/image\n"
        )
    );
    assert_eq!(std::fs::read_to_string(graph_path(&root)).unwrap(), raw);
    let v1 = wired["result"]["version"].as_str().unwrap().to_string();
    assert_ne!(v1, v0);

    // The old version is now stale: version_conflict, nothing written.
    let stale = call(
        &mut reader,
        &mut writer,
        &edit_line(
            3,
            "graph.unwire",
            &root,
            &format!(r#"{CAM_TO_DET},"expect_version":"{v0}""#),
        ),
    )
    .await;
    assert_eq!(stale["error"]["code"], "version_conflict", "{stale}");
    assert_eq!(std::fs::read_to_string(graph_path(&root)).unwrap(), raw);

    // Wiring an already wired input is an engine refusal.
    let twice = call(
        &mut reader,
        &mut writer,
        &edit_line(4, "graph.wire", &root, CAM_TO_DET),
    )
    .await;
    assert_eq!(twice["error"]["code"], "invalid_request", "{twice}");

    // A schema mismatch is typed and carries both schemas.
    let mismatch = call(
        &mut reader,
        &mut writer,
        &edit_line(
            5,
            "graph.wire",
            &root,
            r#""from":{"node":"cam","port":"image"},"to":{"node":"log","port":"data"}"#,
        ),
    )
    .await;
    assert_eq!(mismatch["ok"], false);
    assert_eq!(mismatch["error"]["code"], "schema_mismatch", "{mismatch}");
    assert_eq!(
        mismatch["error"]["data"]["expected"],
        "geometry_msgs/Vector3"
    );
    assert_eq!(mismatch["error"]["data"]["found"], "sensor_msgs/Image");
    assert_eq!(std::fs::read_to_string(graph_path(&root)).unwrap(), raw);

    // Unstaging a node that feeds another is refused with the wire list.
    let broken = call(
        &mut reader,
        &mut writer,
        &edit_line(6, "graph.unstage", &root, r#""node":"cam""#),
    )
    .await;
    assert_eq!(broken["error"]["code"], "would_break", "{broken}");
    assert_eq!(
        broken["error"]["data"]["wires"],
        serde_json::json!([{"from": {"node": "cam", "port": "image"}, "to": {"node": "det", "port": "image"}}])
    );
    assert_eq!(std::fs::read_to_string(graph_path(&root)).unwrap(), raw);

    // Unwire restores the original document.
    let unwired = call(
        &mut reader,
        &mut writer,
        &edit_line(
            7,
            "graph.unwire",
            &root,
            &format!(r#"{CAM_TO_DET},"expect_version":"{v1}""#),
        ),
    )
    .await;
    assert_eq!(unwired["ok"], true, "{unwired}");
    assert_eq!(unwired["result"]["raw"], original.as_str());
    assert_eq!(
        std::fs::read_to_string(graph_path(&root)).unwrap(),
        original
    );
    assert_eq!(unwired["result"]["version"], v0.as_str());

    // Wire again, then unstage with force: the node and its wire go together.
    let again = call(
        &mut reader,
        &mut writer,
        &edit_line(8, "graph.wire", &root, CAM_TO_DET),
    )
    .await;
    assert_eq!(again["ok"], true, "{again}");
    let forced = call(
        &mut reader,
        &mut writer,
        &edit_line(9, "graph.unstage", &root, r#""node":"cam","force":true"#),
    )
    .await;
    assert_eq!(forced["ok"], true, "{forced}");
    assert_eq!(
        forced["result"]["removed_wires"].as_array().unwrap().len(),
        1
    );
    let after = std::fs::read_to_string(graph_path(&root)).unwrap();
    assert!(!after.contains("id: cam"), "{after}");
    assert!(after.contains("  # first\n  - id: det\n"), "{after}");
    assert!(after.contains("# kept\nnetwork:\n"), "{after}");
    assert!(!after.contains("inputs:"), "{after}");

    // An unknown node is an engine refusal, not a crash.
    let unknown = call(
        &mut reader,
        &mut writer,
        &edit_line(10, "graph.unstage", &root, r#""node":"nope""#),
    )
    .await;
    assert_eq!(unknown["error"]["code"], "invalid_request", "{unknown}");
    fixture.stop().await;
}

#[tokio::test]
async fn edit_requests_are_strict_and_old_verbs_still_work() {
    let fixture = fixture().await;
    let root = fixture.root.clone();
    let (mut reader, mut writer) = connect(&fixture.socket).await;
    let before = std::fs::read_to_string(graph_path(&root)).unwrap();

    // Unknown field: bad_request, nothing written.
    let strict = call(
        &mut reader,
        &mut writer,
        &edit_line(
            1,
            "graph.wire",
            &root,
            &format!(r#"{CAM_TO_DET},"force":true"#),
        ),
    )
    .await;
    assert_eq!(strict["error"]["code"], "bad_request", "{strict}");
    // Missing endpoint: bad_request.
    let missing = call(
        &mut reader,
        &mut writer,
        &edit_line(
            2,
            "graph.wire",
            &root,
            r#""from":{"node":"cam","port":"image"}"#,
        ),
    )
    .await;
    assert_eq!(missing["error"]["code"], "bad_request", "{missing}");
    // Path-like graph names cannot escape the workspace.
    let escape = call(
        &mut reader,
        &mut writer,
        &edit_line(3, "graph.wire", &root, CAM_TO_DET)
            .replace("\"graph\":\"main\"", "\"graph\":\"../main\""),
    )
    .await;
    assert_eq!(escape["error"]["code"], "bad_request", "{escape}");
    // A verb the daemon does not have is still unknown_verb.
    let unknown = call(
        &mut reader,
        &mut writer,
        &edit_line(4, "graph.rewire", &root, CAM_TO_DET),
    )
    .await;
    assert_eq!(unknown["error"]["code"], "unknown_verb", "{unknown}");
    // An existing verb answers exactly as before.
    let read = call(
        &mut reader,
        &mut writer,
        &format!(
            r#"{{"id":5,"verb":"graph.read","root":{},"graph":"main"}}"#,
            serde_json::to_string(root.to_str().unwrap()).unwrap()
        ),
    )
    .await;
    assert_eq!(read["ok"], true);
    assert!(read["error"].is_null());
    assert_eq!(std::fs::read_to_string(graph_path(&root)).unwrap(), before);
    fixture.stop().await;
}
