// SPDX-License-Identifier: AGPL-3.0-only
//! The tool surface through an in-process MCP client, against a stand-in graph
//! thread that answers the journaled calls.
//!
//! Transport isolation is per test (`init_for_test` SHM roots, unique topics),
//! so the tests run in parallel.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::message::ShmMessage;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{TransportConfig, TransportManager};
use cerulion_rcpd::journal::Journal;
use cerulion_rcpd::manifest::{LoadedManifest, RcpManifest};
use cerulion_rcpd::observer::GraphSource;
use cerulion_rcpd::server::RcpServer;
use cerulion_rcpd::tools::{latch_path, ReceiptLock, ToolHost, ESTOP_CLEAR_TOOL, ESTOP_TOOL};
use cerulion_rcpd::RcpdError;
use native_ros2_messages::std_msgs::String as RosString;
use rmcp::model::{
    CallToolRequestParams, CallToolResult, ReadResourceRequestParams, ResourceContents,
};
use rmcp::service::RunningService;
use rmcp::{RoleClient, ServiceExt};
use serde_json::{json, Value};

static COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique(tag: &str) -> String {
    format!(
        "rcpd_tools_{tag}_{}_{}",
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::Relaxed)
    )
}

fn manager(tag: &str) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: unique(tag),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init isolated transport manager")
}

/// A manifest on topics private to this test: three reversible tools
/// (`arm.slow` is answered late, `arm.mute` never), one read tool, one
/// irreversible tool, one state topic.
fn manifest(dir: &Path, prefix: &str) -> LoadedManifest {
    manifest_with(dir, prefix, "")
}

/// [`manifest`] with `extra` (top-level YAML, such as a `limits` section)
/// appended.
fn manifest_with(dir: &Path, prefix: &str, extra: &str) -> LoadedManifest {
    let step = |name: &str, timeout_ms: u32| {
        format!(
            "  - name: {name}
    description: one period
    safety_class: reversible
    timeout_ms: {timeout_ms}
    input_schema:
      type: object
      properties:
        q: {{ type: array, items: {{ type: number }}, minItems: 2, maxItems: 2 }}
      required: [q]
"
        )
    };
    let yaml = format!(
        "robot: tools_fixture
journal:
  calls_topic: /{prefix}/calls
  results_topic: /{prefix}/results
state:
  - name: arm
    topic: /{prefix}/state
tools:
{}{}{}  - name: arm.peek
    description: read only
    safety_class: read
    input_schema: {{ type: object, properties: {{}} }}
  - name: arm.cut
    description: cut the cable
    safety_class: irreversible
    input_schema: {{ type: object, properties: {{}} }}
{extra}",
        step("arm.step", 3000),
        step("arm.slow", 3000),
        step("arm.mute", 300),
    );
    let path = dir.join("rcp.yaml");
    std::fs::write(&path, yaml).expect("write manifest");
    RcpManifest::load_full(&path).expect("load manifest")
}

fn publish_json(publisher: &mut cerulion_core::CerulionPublisher, value: &Value) {
    let mut proxy = publisher.loan_proxy::<RosString>().expect("loan");
    proxy.set_data(&value.to_string()).expect("set");
}

/// Answers every call frame, oldest first, with `{call_id, tool, ok, n}` on
/// the results topic followed by a decoy answer to no call (`call_id` 0), and
/// publishes `{"n": n}` on the state topic. `arm.slow` is answered after
/// 1.5 s; `arm.mute` never. Returns the calls seen once `stop` is set.
fn spawn_graph(
    mgr: Arc<TransportManager>,
    prefix: String,
    stop: Arc<AtomicBool>,
) -> std::thread::JoinHandle<Vec<Value>> {
    let (ready_tx, ready_rx) = std::sync::mpsc::channel();
    let handle = std::thread::spawn(move || {
        let calls = mgr
            .create_subscriber(&format!("/{prefix}/calls"))
            .expect("calls subscriber");
        let mut results = mgr
            .create_publisher_simple(&format!("/{prefix}/results"), MaxSliceLen::const_new(4096))
            .expect("results publisher");
        let mut state = mgr
            .create_publisher_simple(&format!("/{prefix}/state"), MaxSliceLen::const_new(4096))
            .expect("state publisher");
        ready_tx.send(()).expect("ready");
        let mut seen: Vec<Value> = Vec::new();
        while !stop.load(Ordering::Relaxed) {
            let mut batch = Vec::new();
            calls
                .try_receive(|m| {
                    let text = RosString::build_reader(m.payload())
                        .data()
                        .expect("utf-8")
                        .to_owned();
                    batch.push(serde_json::from_str::<Value>(&text).expect("call is JSON"));
                })
                .expect("receive");
            if batch.is_empty() {
                std::thread::sleep(Duration::from_millis(1));
                continue;
            }
            for call in batch {
                seen.push(call.clone());
                let n = seen.len();
                if call["tool"] == json!("arm.mute") {
                    continue;
                }
                if call["tool"] == json!("arm.slow") {
                    std::thread::sleep(Duration::from_millis(1500));
                }
                let answer =
                    json!({ "call_id": call["call_id"], "tool": call["tool"], "ok": true, "n": n });
                publish_json(&mut results, &answer);
                publish_json(&mut results, &json!({ "call_id": 0, "decoy": true }));
                publish_json(&mut state, &json!({ "n": n }));
            }
        }
        seen
    });
    ready_rx.recv().expect("graph thread ready");
    handle
}

fn open_host(
    mgr: &Arc<TransportManager>,
    loaded: &LoadedManifest,
    receipts: &Path,
) -> Result<Arc<ToolHost>, RcpdError> {
    let lock = ReceiptLock::acquire(receipts)?;
    let latch = Arc::new(AtomicBool::new(false));
    let journal = Journal::spawn(
        mgr.clone(),
        loaded.journal.clone(),
        loaded.state.clone(),
        latch.clone(),
    )?;
    ToolHost::open(loaded.tools.clone(), journal, latch, receipts, lock).map(Arc::new)
}

struct Fixture {
    client: Arc<RunningService<RoleClient, ()>>,
    mgr: Arc<TransportManager>,
    prefix: String,
    receipts: PathBuf,
    stop: Arc<AtomicBool>,
    graph: Option<std::thread::JoinHandle<Vec<Value>>>,
    _dir: tempfile::TempDir,
}

impl Fixture {
    async fn new(tag: &str, with_graph: bool) -> Self {
        let dir = tempfile::tempdir().expect("tempdir");
        Self::in_dir(tag, with_graph, dir, "").await
    }

    /// A fixture whose manifest ends with `extra` (a `limits` section).
    async fn with_extra(tag: &str, extra: &str) -> Self {
        let dir = tempfile::tempdir().expect("tempdir");
        Self::in_dir(tag, true, dir, extra).await
    }

    async fn in_dir(tag: &str, with_graph: bool, dir: tempfile::TempDir, extra: &str) -> Self {
        let prefix = unique(tag);
        let loaded = manifest_with(dir.path(), &prefix, extra);
        let mgr = manager(tag);
        let stop = Arc::new(AtomicBool::new(false));
        let graph = with_graph.then(|| spawn_graph(mgr.clone(), prefix.clone(), stop.clone()));
        let receipts = dir.path().join("receipts/log.jsonl");
        let host = open_host_as_configured(&mgr, &loaded, &receipts).expect("host");
        let feed = GraphSource::new(mgr.clone(), loaded.identity.clone()).feed();
        let server = RcpServer::with_tools(feed, host, loaded.state.clone());
        let (client_io, server_io) = tokio::io::duplex(64 * 1024);
        let (server, client) = tokio::join!(server.serve(server_io), ().serve(client_io));
        let server = server.expect("server session");
        tokio::spawn(async move {
            let _ = server.waiting().await;
        });
        Self {
            client: Arc::new(client.expect("client session")),
            mgr,
            prefix,
            receipts,
            stop,
            graph,
            _dir: dir,
        }
    }

    async fn call(&self, tool: &str, args: Value) -> (bool, Value) {
        call(&self.client, tool, args).await
    }

    fn receipts(&self) -> Vec<(String, String)> {
        let report = cerulion_rcpd::audit::verify_log(&self.receipts).expect("chain intact");
        let entries = cerud::receipt::read_all(&self.receipts).expect("read receipts");
        assert_eq!(report.entries, entries.len());
        entries
            .iter()
            .map(|r| {
                let outcome = serde_json::to_value(&r.outcome).expect("outcome");
                let tag = match outcome.get("detail").and_then(Value::as_str) {
                    Some(detail) => format!("error:{detail}"),
                    None => outcome["outcome"].as_str().unwrap_or("").to_string(),
                };
                (r.verb.clone(), tag)
            })
            .collect()
    }

    fn stop_graph(&mut self) -> Vec<Value> {
        self.stop.store(true, Ordering::Relaxed);
        self.graph
            .take()
            .map(|h| h.join().expect("graph thread"))
            .unwrap_or_default()
    }
}

async fn call(client: &RunningService<RoleClient, ()>, tool: &str, args: Value) -> (bool, Value) {
    let params = CallToolRequestParams::new(tool.to_string())
        .with_arguments(args.as_object().cloned().expect("object args"));
    let result: CallToolResult = client.call_tool(params).await.expect("call_tool");
    let text = result
        .content
        .first()
        .and_then(|c| c.as_text())
        .map(|t| t.text.clone())
        .expect("one text block");
    (
        result.is_error == Some(true),
        serde_json::from_str(&text).expect("JSON body"),
    )
}

fn pairs(items: &[(&str, &str)]) -> Vec<(String, String)> {
    items
        .iter()
        .map(|(a, b)| ((*a).to_string(), (*b).to_string()))
        .collect()
}

/// [`open_host`] with the manifest's `limits.estop_clear` applied, as the
/// binary applies it.
fn open_host_as_configured(
    mgr: &Arc<TransportManager>,
    loaded: &LoadedManifest,
    receipts: &Path,
) -> Result<Arc<ToolHost>, RcpdError> {
    let lock = ReceiptLock::acquire(receipts)?;
    let latch = Arc::new(AtomicBool::new(false));
    let journal = Journal::spawn(
        mgr.clone(),
        loaded.journal.clone(),
        loaded.state.clone(),
        latch.clone(),
    )?;
    ToolHost::open(loaded.tools.clone(), journal, latch, receipts, lock)
        .map(|host| Arc::new(host.with_estop_clear(loaded.limits.estop_clear)))
}

#[tokio::test(flavor = "multi_thread")]
async fn tools_list_is_the_manifest_plus_estop() {
    let f = Fixture::new("list", false).await;
    let names: Vec<String> = f
        .client
        .list_all_tools()
        .await
        .expect("list")
        .into_iter()
        .map(|t| t.name.to_string())
        .collect();
    assert_eq!(
        names,
        [
            "arm.step",
            "arm.slow",
            "arm.mute",
            "arm.peek",
            "arm.cut",
            ESTOP_TOOL,
            ESTOP_CLEAR_TOOL
        ]
    );
}

#[tokio::test(flavor = "multi_thread")]
async fn a_call_is_receipted_journaled_and_answered_past_a_decoy() {
    let mut f = Fixture::new("happy", true).await;
    for i in 0..3 {
        let (is_error, body) = f.call("arm.step", json!({ "q": [0.1, -0.2] })).await;
        assert!(!is_error, "call {i}: {body}");
        assert_eq!(body["result"]["ok"], json!(true));
        assert_eq!(body["result"]["call_id"], body["call_id"]);
        assert!(body["call_id"].as_u64().is_some_and(|id| id < (1 << 52)));
    }
    let seen = f.stop_graph();
    assert_eq!(seen.len(), 3, "exactly one journaled frame per call");
    assert_eq!(seen[0]["tool"], json!("arm.step"));
    assert_eq!(seen[0]["args"], json!({ "q": [0.1, -0.2] }));
    assert_eq!(
        f.receipts(),
        pairs(&[
            ("arm.step", "intent"),
            ("arm.step", "ok"),
            ("arm.step", "intent"),
            ("arm.step", "ok"),
            ("arm.step", "intent"),
            ("arm.step", "ok"),
        ])
    );
}

#[tokio::test(flavor = "multi_thread")]
async fn undeclared_tools_and_bad_args_never_reach_the_graph() {
    let mut f = Fixture::new("deny", true).await;
    let (is_error, body) = f.call("arm.fly", json!({})).await;
    assert!(is_error);
    assert_eq!(body["error"]["code"], json!("not_in_manifest"));
    let (is_error, _) = f.call(&"x".repeat(300), json!({})).await;
    assert!(is_error);
    let (is_error, body) = f.call("arm.step", json!({ "q": [1.0] })).await;
    assert!(is_error);
    assert_eq!(body["error"]["code"], json!("invalid_args"));
    let (is_error, body) = f
        .call("arm.step", json!({ "q": [1.0, 2.0], "speed": 3 }))
        .await;
    assert!(is_error);
    assert!(
        body["error"]["message"]
            .as_str()
            .unwrap_or("")
            .contains("speed"),
        "{body}"
    );

    assert!(f.stop_graph().is_empty(), "nothing was journaled");
    assert_eq!(
        f.receipts(),
        pairs(&[
            ("arm.fly", "denied"),
            ("<invalid-tool-name>", "denied"),
            ("arm.step", "error:invalid_args"),
            ("arm.step", "error:invalid_args"),
        ])
    );
}

#[tokio::test(flavor = "multi_thread")]
async fn estop_latches_and_refuses_later_motion_but_not_reads() {
    let mut f = Fixture::new("estop", true).await;
    let (is_error, body) = f
        .call(ESTOP_TOOL, json!({ "reason": "x".repeat(500) }))
        .await;
    assert!(!is_error, "{body}");
    assert_eq!(body["estopped"], json!(true));
    assert_eq!(body["graph_acknowledged"], json!(true));
    assert_eq!(
        body["result"]["tool"],
        json!(ESTOP_TOOL),
        "the graph was told"
    );

    let (is_error, body) = f.call("arm.step", json!({ "q": [0.0, 0.0] })).await;
    assert!(is_error);
    assert_eq!(body["error"]["code"], json!("estopped"));
    let (is_error, _) = f.call("arm.peek", json!({})).await;
    assert!(!is_error, "a read tool still runs while e-stopped");

    let seen = f.stop_graph();
    let tools: Vec<&str> = seen.iter().filter_map(|c| c["tool"].as_str()).collect();
    assert_eq!(
        tools,
        [ESTOP_TOOL, "arm.peek"],
        "the refused step was never journaled"
    );
    let reason = seen[0]["args"]["reason"].as_str().expect("reason");
    assert_eq!(reason.chars().count(), 200, "the reason is truncated");
    assert_eq!(
        f.receipts(),
        pairs(&[
            (ESTOP_TOOL, "intent"),
            (ESTOP_TOOL, "ok"),
            ("arm.step", "denied"),
            ("arm.peek", "ok"),
        ])
    );
}

/// Records when each call frame is published, on a subscriber of its own.
fn spawn_tap(
    mgr: Arc<TransportManager>,
    prefix: String,
    stop: Arc<AtomicBool>,
) -> std::thread::JoinHandle<Vec<(String, Instant)>> {
    let (ready_tx, ready_rx) = std::sync::mpsc::channel();
    let handle = std::thread::spawn(move || {
        let calls = mgr
            .create_subscriber(&format!("/{prefix}/calls"))
            .expect("tap subscriber");
        ready_tx.send(()).expect("ready");
        let mut seen = Vec::new();
        while !stop.load(Ordering::Relaxed) {
            calls
                .try_receive(|m| {
                    let text = RosString::build_reader(m.payload())
                        .data()
                        .expect("utf-8")
                        .to_owned();
                    let call: Value = serde_json::from_str(&text).expect("JSON");
                    let tool = call["tool"].as_str().unwrap_or("").to_string();
                    seen.push((tool, Instant::now()));
                })
                .expect("receive");
            std::thread::sleep(Duration::from_millis(1));
        }
        seen
    });
    ready_rx.recv().expect("tap ready");
    handle
}

#[tokio::test(flavor = "multi_thread")]
async fn estop_does_not_wait_behind_a_call_in_flight() {
    let mut f = Fixture::new("estop_inflight", true).await;
    let tap_stop = Arc::new(AtomicBool::new(false));
    let tap = spawn_tap(f.mgr.clone(), f.prefix.clone(), tap_stop.clone());
    let client = Arc::clone(&f.client);
    let slow =
        tokio::spawn(async move { call(&client, "arm.slow", json!({ "q": [0.0, 0.0] })).await });
    // Let the slow call reach the graph, which then sleeps 1.5 s before answering.
    tokio::time::sleep(Duration::from_millis(200)).await;
    let (is_error, body) = f.call(ESTOP_TOOL, json!({ "reason": "now" })).await;
    assert!(!is_error, "{body}");
    // A call made while the slow one may still be in flight is refused by the
    // latch, which `estop` closed without waiting for it.
    let (queued_error, queued) = f.call("arm.step", json!({ "q": [0.0, 0.0] })).await;
    assert!(queued_error);
    assert_eq!(queued["error"]["code"], json!("estopped"));
    let (slow_error, slow_body) = slow.await.expect("slow call task");
    assert!(!slow_error, "the call in flight completes: {slow_body}");
    tap_stop.store(true, Ordering::Relaxed);
    let published = tap.join().expect("tap thread");
    let at = |tool: &str| {
        published
            .iter()
            .find(|(t, _)| t == tool)
            .map(|(_, at)| *at)
            .unwrap_or_else(|| panic!("{tool} was never published: {published:?}"))
    };
    let gap = at(ESTOP_TOOL).duration_since(at("arm.slow"));
    assert!(
        gap < Duration::from_millis(1000),
        "the e-stop was published {gap:?} after the slow call, while that call was still \
         unanswered (the graph answers it after 1.5 s)"
    );
    let seen = f.stop_graph();
    let tools: Vec<&str> = seen.iter().filter_map(|c| c["tool"].as_str()).collect();
    assert_eq!(tools, ["arm.slow", ESTOP_TOOL]);
}

#[tokio::test(flavor = "multi_thread")]
async fn the_latch_survives_a_restart() {
    let dir = tempfile::tempdir().expect("tempdir");
    let receipts = dir.path().join("receipts/log.jsonl");
    std::fs::create_dir_all(receipts.parent().expect("parent")).expect("mkdir");
    std::fs::write(latch_path(&receipts), "earlier stop\n").expect("write latch");
    let f = Fixture::in_dir("latch", true, dir, "").await;
    let (is_error, body) = f.call("arm.step", json!({ "q": [0.0, 0.0] })).await;
    assert!(is_error);
    assert_eq!(body["error"]["code"], json!("estopped"));
}

#[tokio::test(flavor = "multi_thread")]
async fn a_second_daemon_on_the_same_receipt_log_is_refused() {
    let dir = tempfile::tempdir().expect("tempdir");
    let loaded = manifest(dir.path(), &unique("lock"));
    let receipts = dir.path().join("log.jsonl");
    let mgr = manager("lock");
    let first = open_host(&mgr, &loaded, &receipts).expect("first daemon");
    let names: Vec<String> = first.list().iter().map(|t| t.name.to_string()).collect();
    assert!(
        names.iter().any(|n| n == ESTOP_CLEAR_TOOL),
        "a host built without with_estop_clear serves estop_clear: {names:?}"
    );
    let second = open_host(&mgr, &loaded, &receipts);
    let Err(RcpdError::Receipts { reason, .. }) = second else {
        panic!("a second daemon must be refused");
    };
    assert!(reason.contains("another cerulion-rcpd"), "{reason}");
    drop(first);
    open_host(&mgr, &loaded, &receipts).expect("the lock is released with the daemon");
}

#[tokio::test(flavor = "multi_thread")]
async fn a_silent_graph_times_out_and_says_the_call_may_have_run() {
    let mut f = Fixture::new("timeout", true).await;
    let (is_error, body) = f.call("arm.mute", json!({ "q": [0.0, 0.0] })).await;
    assert!(is_error);
    assert_eq!(body["error"]["code"], json!("timeout"));
    assert_eq!(body["may_have_executed"], json!(true));
    assert_eq!(f.stop_graph().len(), 1, "the call was published");
    assert_eq!(
        f.receipts(),
        pairs(&[("arm.mute", "intent"), ("arm.mute", "error:timeout")])
    );
}

#[tokio::test(flavor = "multi_thread")]
async fn the_state_resource_serves_the_newest_frame() {
    let mut f = Fixture::new("state", true).await;
    for _ in 0..3 {
        let (is_error, _) = f.call("arm.step", json!({ "q": [0.0, 0.0] })).await;
        assert!(!is_error);
    }
    // The state frame follows its answer by microseconds; poll briefly.
    let mut newest = Value::Null;
    for _ in 0..200 {
        let result = f
            .client
            .read_resource(ReadResourceRequestParams::new("robot://state/arm"))
            .await
            .expect("read state");
        let ResourceContents::TextResourceContents { text, .. } = &result.contents[0] else {
            panic!("state is text");
        };
        newest = serde_json::from_str(text).expect("JSON");
        if newest == json!({ "n": 3 }) {
            break;
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    assert_eq!(newest, json!({ "n": 3 }));
    let missing = f
        .client
        .read_resource(ReadResourceRequestParams::new("robot://state/ghost"))
        .await;
    assert!(
        missing.is_err(),
        "an undeclared state resource does not exist"
    );
    f.stop_graph();
}

#[test]
fn the_audit_follows_rotation_and_catches_an_altered_entry() {
    let dir = tempfile::tempdir().expect("tempdir");
    let path = dir.path().join("log.jsonl");
    {
        let mut log =
            cerud::receipt::ReceiptLog::open_with_limits(&path, 2048, 2).expect("open log");
        for i in 0..60 {
            log.record(
                "rcp-agent",
                "arm.step",
                &format!("{i:064}"),
                cerud::receipt::ReceiptOutcome::Ok,
            )
            .expect("record");
        }
    }
    assert!(
        cerud::receipt::rotated_path(&path, 2).exists(),
        "the log rotated"
    );
    let report = cerulion_rcpd::audit::verify_log(&path).expect("intact");
    assert!(!report.from_genesis, "the oldest files were deleted");
    assert_eq!(report.files.len(), 3);

    let text = std::fs::read_to_string(&path).expect("read");
    let tampered = text.replacen("\"arm.step\"", "\"arm.fly\"", 1);
    assert_ne!(text, tampered);
    std::fs::write(&path, tampered).expect("write");
    let err = cerulion_rcpd::audit::verify_log(&path).expect_err("altered");
    assert!(err.contains("altered"), "{err}");

    assert!(cerulion_rcpd::audit::verify_log(&dir.path().join("absent.jsonl")).is_err());
}

fn write_log(path: &Path, max_bytes: u64, entries: usize) {
    let mut log =
        cerud::receipt::ReceiptLog::open_with_limits(path, max_bytes, 2).expect("open log");
    for i in 0..entries {
        log.record(
            "rcp-agent",
            "arm.step",
            &format!("{i:064}"),
            cerud::receipt::ReceiptOutcome::Ok,
        )
        .expect("record");
    }
}

#[test]
fn the_audit_refuses_a_cut_head_and_a_missing_rotated_file() {
    let dir = tempfile::tempdir().expect("tempdir");
    let flat = dir.path().join("flat.jsonl");
    write_log(&flat, 0, 10);
    cerulion_rcpd::audit::verify_log(&flat).expect("intact before the cut");
    let text = std::fs::read_to_string(&flat).expect("read");
    let cut: String = text.lines().skip(3).map(|l| format!("{l}\n")).collect();
    std::fs::write(&flat, cut).expect("write");
    let err = cerulion_rcpd::audit::verify_log(&flat).expect_err("head cut");
    assert!(err.contains("removed from the head"), "{err}");

    let rotated = dir.path().join("rotated.jsonl");
    write_log(&rotated, 2048, 60);
    assert!(
        cerud::receipt::rotated_path(&rotated, 2).exists(),
        "the log rotated"
    );
    std::fs::remove_file(cerud::receipt::rotated_path(&rotated, 1)).expect("delete .1");
    let err = cerulion_rcpd::audit::verify_log(&rotated).expect_err("gap");
    assert!(err.contains("is missing"), "{err}");
}

const OPERATOR_CLEARS: &str = "limits:\n  estop_clear: operator\n";

/// The `tools/list` entries as JSON, keyed by name.
async fn listed(f: &Fixture) -> std::collections::BTreeMap<String, Value> {
    f.client
        .list_all_tools()
        .await
        .expect("list")
        .into_iter()
        .map(|t| {
            let json = serde_json::to_value(&t).expect("tool JSON");
            (t.name.to_string(), json)
        })
        .collect()
}

#[tokio::test(flavor = "multi_thread")]
async fn tools_list_labels_every_safety_class() {
    let f = Fixture::new("labels", false).await;
    let tools = listed(&f).await;
    // (name, description, readOnlyHint, destructiveHint)
    let expected = [
        (
            "arm.step",
            "one period (safety class: reversible)",
            false,
            false,
        ),
        ("arm.peek", "read only (safety class: read)", true, false),
        (
            "arm.cut",
            "cut the cable (safety class: irreversible)",
            false,
            true,
        ),
    ];
    for (name, description, read_only, destructive) in expected {
        let tool = &tools[name];
        assert_eq!(tool["description"], json!(description), "{tool}");
        assert_eq!(
            tool["annotations"]["readOnlyHint"],
            json!(read_only),
            "{tool}"
        );
        assert_eq!(
            tool["annotations"]["destructiveHint"],
            json!(destructive),
            "{tool}"
        );
        assert_eq!(tool["annotations"]["openWorldHint"], json!(false), "{tool}");
    }
    for (name, destructive) in [(ESTOP_TOOL, false), (ESTOP_CLEAR_TOOL, true)] {
        let tool = &tools[name];
        assert_eq!(
            tool["annotations"]["destructiveHint"],
            json!(destructive),
            "{tool}"
        );
        assert_eq!(tool["annotations"]["idempotentHint"], json!(true), "{tool}");
    }
    let estop = tools[ESTOP_TOOL]["description"].as_str().expect("text");
    assert!(estop.contains("estop_clear"), "{estop}");
}

#[tokio::test(flavor = "multi_thread")]
async fn an_irreversible_tool_runs_receipted_labelled_and_latch_gated() {
    let mut f = Fixture::new("irreversible", true).await;
    let (is_error, body) = f.call("arm.cut", json!({})).await;
    assert!(!is_error, "{body}");
    assert_eq!(body["safety_class"], json!("irreversible"));
    assert_eq!(body["result"]["tool"], json!("arm.cut"));
    let (is_error, _) = f.call(ESTOP_TOOL, json!({})).await;
    assert!(!is_error);
    let (is_error, body) = f.call("arm.cut", json!({})).await;
    assert!(is_error);
    assert_eq!(body["error"]["code"], json!("estopped"));
    assert_eq!(body["safety_class"], json!("irreversible"));
    let (_, body) = f.call("arm.peek", json!({})).await;
    assert_eq!(body["safety_class"], json!("read"));
    let tools: Vec<Value> = f
        .stop_graph()
        .into_iter()
        .map(|c| c["tool"].clone())
        .collect();
    assert_eq!(
        tools,
        [json!("arm.cut"), json!(ESTOP_TOOL), json!("arm.peek")]
    );
    assert_eq!(
        f.receipts(),
        pairs(&[
            ("arm.cut", "intent"),
            ("arm.cut", "ok"),
            (ESTOP_TOOL, "intent"),
            (ESTOP_TOOL, "ok"),
            ("arm.cut", "denied"),
            ("arm.peek", "ok"),
        ])
    );
}

#[tokio::test(flavor = "multi_thread")]
async fn the_agent_clears_the_estop_and_a_later_estop_latches_again() {
    let mut f = Fixture::new("clear", true).await;
    let latch = latch_path(&f.receipts);
    let step = json!({ "q": [0.0, 0.0] });

    let (is_error, _) = f.call(ESTOP_TOOL, json!({ "reason": "first" })).await;
    assert!(!is_error);
    assert!(latch.exists(), "the latch file is written on engage");
    let (_, body) = f.call("arm.step", step.clone()).await;
    assert_eq!(body["error"]["code"], json!("estopped"));
    assert!(
        body["error"]["message"]
            .as_str()
            .unwrap_or("")
            .contains("estop_clear"),
        "{body}"
    );

    let (is_error, body) = f.call(ESTOP_CLEAR_TOOL, json!({ "reason": "safe" })).await;
    assert!(!is_error, "{body}");
    assert_eq!(body["estopped"], json!(false));
    assert_eq!(body["was_estopped"], json!(true));
    assert_eq!(body["graph_acknowledged"], json!(true));
    assert_eq!(
        body["result"]["tool"],
        json!(ESTOP_CLEAR_TOOL),
        "the graph was told"
    );
    assert!(body["receipt"]["seq"].is_u64(), "{body}");
    assert!(!latch.exists(), "the clear deletes the latch file");

    let (is_error, body) = f.call("arm.step", step.clone()).await;
    assert!(!is_error, "motion runs after the clear: {body}");

    let (is_error, _) = f.call(ESTOP_TOOL, json!({ "reason": "second" })).await;
    assert!(!is_error);
    assert!(
        latch.exists(),
        "an engage after a clear writes the file again"
    );
    assert_eq!(std::fs::read_to_string(&latch).expect("latch"), "second\n");
    let (_, body) = f.call("arm.step", step.clone()).await;
    assert_eq!(
        body["error"]["code"],
        json!("estopped"),
        "an engage after a clear latches"
    );

    let seen = f.stop_graph();
    let tools: Vec<&str> = seen.iter().filter_map(|c| c["tool"].as_str()).collect();
    assert_eq!(
        tools,
        [ESTOP_TOOL, ESTOP_CLEAR_TOOL, "arm.step", ESTOP_TOOL]
    );
    assert_eq!(
        seen[1]["args"],
        json!({ "reason": "safe" }),
        "the clear is journaled"
    );
    assert_eq!(
        f.receipts(),
        pairs(&[
            (ESTOP_TOOL, "intent"),
            (ESTOP_TOOL, "ok"),
            ("arm.step", "denied"),
            (ESTOP_CLEAR_TOOL, "intent"),
            (ESTOP_CLEAR_TOOL, "ok"),
            ("arm.step", "intent"),
            ("arm.step", "ok"),
            (ESTOP_TOOL, "intent"),
            (ESTOP_TOOL, "ok"),
            ("arm.step", "denied"),
        ])
    );
}

#[tokio::test(flavor = "multi_thread")]
async fn a_clear_while_not_estopped_is_receipted_and_changes_nothing() {
    let mut f = Fixture::new("clear_idle", true).await;
    let (is_error, body) = f.call(ESTOP_CLEAR_TOOL, json!({ "speed": 1 })).await;
    assert!(is_error, "arguments outside the schema are refused: {body}");
    assert_eq!(body["error"]["code"], json!("invalid_args"));
    let (is_error, body) = f.call(ESTOP_CLEAR_TOOL, json!({})).await;
    assert!(!is_error, "{body}");
    assert_eq!(body["was_estopped"], json!(false));
    assert_eq!(body["estopped"], json!(false));
    let (is_error, _) = f.call("arm.step", json!({ "q": [0.0, 0.0] })).await;
    assert!(!is_error);
    let seen = f.stop_graph();
    let tools: Vec<&str> = seen.iter().filter_map(|c| c["tool"].as_str()).collect();
    assert_eq!(
        tools,
        ["arm.step"],
        "a clear with nothing latched is not journaled"
    );
    assert_eq!(
        f.receipts(),
        pairs(&[
            (ESTOP_CLEAR_TOOL, "error:invalid_args"),
            (ESTOP_CLEAR_TOOL, "intent"),
            (ESTOP_CLEAR_TOOL, "ok"),
            ("arm.step", "intent"),
            ("arm.step", "ok"),
        ])
    );
}

#[tokio::test(flavor = "multi_thread")]
async fn the_agent_clears_a_latch_left_by_an_earlier_daemon() {
    let dir = tempfile::tempdir().expect("tempdir");
    let receipts = dir.path().join("receipts/log.jsonl");
    std::fs::create_dir_all(receipts.parent().expect("parent")).expect("mkdir");
    std::fs::write(latch_path(&receipts), "earlier stop\n").expect("write latch");
    let mut f = Fixture::in_dir("clear_restart", true, dir, "").await;
    let (is_error, body) = f.call(ESTOP_CLEAR_TOOL, json!({})).await;
    assert!(!is_error, "{body}");
    assert_eq!(body["was_estopped"], json!(true));
    assert!(!latch_path(&receipts).exists());
    let (is_error, body) = f.call("arm.step", json!({ "q": [0.0, 0.0] })).await;
    assert!(!is_error, "{body}");
    f.stop_graph();
}

#[tokio::test(flavor = "multi_thread")]
async fn a_latch_file_that_cannot_be_deleted_keeps_the_estop_engaged() {
    let dir = tempfile::tempdir().expect("tempdir");
    let receipts = dir.path().join("receipts/log.jsonl");
    // A non-empty directory where the latch file goes: it exists, so the
    // daemon starts latched, and `remove_file` cannot delete it.
    std::fs::create_dir_all(latch_path(&receipts).join("pinned")).expect("mkdir");
    let mut f = Fixture::in_dir("clear_fails", true, dir, "").await;
    let (is_error, body) = f.call(ESTOP_CLEAR_TOOL, json!({})).await;
    assert!(is_error, "{body}");
    assert_eq!(body["error"]["code"], json!("clear_failed"));
    let (_, body) = f.call("arm.step", json!({ "q": [0.0, 0.0] })).await;
    assert_eq!(
        body["error"]["code"],
        json!("estopped"),
        "the latch stays closed"
    );
    assert!(f.stop_graph().is_empty(), "nothing was journaled");
    assert_eq!(
        f.receipts(),
        pairs(&[
            (ESTOP_CLEAR_TOOL, "intent"),
            (ESTOP_CLEAR_TOOL, "error:clear_failed"),
            ("arm.step", "denied"),
        ])
    );
}

#[tokio::test(flavor = "multi_thread")]
async fn under_operator_clearing_the_agent_cannot_clear_the_estop() {
    let mut f = Fixture::with_extra("operator", OPERATOR_CLEARS).await;
    let tools = listed(&f).await;
    assert!(
        !tools.contains_key(ESTOP_CLEAR_TOOL),
        "estop_clear is not listed"
    );
    let estop = tools[ESTOP_TOOL]["description"].as_str().expect("text");
    assert!(!estop.contains("estop_clear"), "{estop}");

    let (is_error, _) = f.call(ESTOP_TOOL, json!({})).await;
    assert!(!is_error);
    let (is_error, body) = f.call(ESTOP_CLEAR_TOOL, json!({})).await;
    assert!(is_error, "{body}");
    assert_eq!(body["error"]["code"], json!("estop_clear_disabled"));
    assert!(latch_path(&f.receipts).exists(), "the latch file stays");
    let (_, body) = f.call("arm.step", json!({ "q": [0.0, 0.0] })).await;
    assert_eq!(body["error"]["code"], json!("estopped"));
    assert!(
        !body["error"]["message"]
            .as_str()
            .unwrap_or("")
            .contains("estop_clear"),
        "{body}"
    );

    let seen = f.stop_graph();
    let tools: Vec<&str> = seen.iter().filter_map(|c| c["tool"].as_str()).collect();
    assert_eq!(tools, [ESTOP_TOOL], "the refused clear was never journaled");
    assert_eq!(
        f.receipts(),
        pairs(&[
            (ESTOP_TOOL, "intent"),
            (ESTOP_TOOL, "ok"),
            (ESTOP_CLEAR_TOOL, "denied"),
            ("arm.step", "denied"),
        ])
    );
}

/// The latch state the receipt log implies: an `estop` intent closes it; an
/// `estop_clear` intent opens it unless that clear's closing receipt is
/// `error:clear_failed`, since every other clear outcome is written after the
/// latch opened. Clears are serialized, so a clear's closing receipt is the
/// next `estop_clear` line after its intent.
fn replay_latch(receipts: &[(String, String)], initially: bool) -> bool {
    let mut closed = initially;
    for (i, (verb, outcome)) in receipts.iter().enumerate() {
        match (verb.as_str(), outcome.as_str()) {
            (ESTOP_TOOL, "intent") => closed = true,
            (ESTOP_CLEAR_TOOL, "intent") => {
                let closing = receipts[i + 1..]
                    .iter()
                    .find(|(v, _)| v == ESTOP_CLEAR_TOOL)
                    .map(|(_, o)| o.as_str());
                if closing != Some("error:clear_failed") {
                    closed = false;
                }
            }
            _ => {}
        }
    }
    closed
}

#[tokio::test(flavor = "multi_thread")]
async fn racing_engages_and_clears_leave_the_latch_the_log_and_the_graph_agreeing() {
    let mut f = Fixture::new("race", true).await;
    let mut tasks = Vec::new();
    for i in 0..24 {
        let client = Arc::clone(&f.client);
        let tool = if i % 2 == 0 {
            ESTOP_TOOL
        } else {
            ESTOP_CLEAR_TOOL
        };
        tasks.push(tokio::spawn(async move {
            call(&client, tool, json!({ "reason": format!("r{i}") })).await
        }));
    }
    for task in tasks {
        let (is_error, body) = task.await.expect("task");
        if is_error {
            assert_eq!(body["error"]["code"], json!("estop_overtaken"), "{body}");
        }
    }
    // The settled state, read through a motion call.
    let (refused, _) = f.call("arm.step", json!({ "q": [0.0, 0.0] })).await;
    let receipts = f.receipts();
    let seen = f.stop_graph();
    let latched = replay_latch(&receipts, false);
    assert_eq!(
        refused, latched,
        "the latch matches the receipt log: {receipts:?}"
    );
    assert_eq!(
        latch_path(&f.receipts).exists(),
        latched,
        "the latch file matches the latch"
    );
    let last_builtin = seen
        .iter()
        .filter_map(|c| c["tool"].as_str())
        .rfind(|t| *t == ESTOP_TOOL || *t == ESTOP_CLEAR_TOOL);
    if latched {
        assert_eq!(
            last_builtin,
            Some(ESTOP_TOOL),
            "the graph's last word is the estop"
        );
    }
}
