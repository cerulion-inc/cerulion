// SPDX-License-Identifier: AGPL-3.0-only
//! The tool surface through an in-process MCP client, against a stand-in graph
//! thread that answers the journaled calls: the hand-written event/effect
//! vectors of [`oracle`] run against the `ToolHost` daemon, beside plain tests
//! of the state, audit and scope paths.
//!
//! Transport isolation is per test (`init_for_test` SHM roots, unique topics),
//! so the tests run in parallel.

mod oracle;

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::wire::MaxSliceLen;
use cerulion_core::TransportManager;
use cerulion_rcpd::journal::Journal;
use cerulion_rcpd::manifest::LoadedManifest;
use cerulion_rcpd::observer::GraphSource;
use cerulion_rcpd::server::RcpServer;
use cerulion_rcpd::tools::{ReceiptLock, ToolHost, ESTOP_CLEAR_TOOL, ESTOP_TOOL};
use cerulion_rcpd::RcpdError;
use oracle::{manager, manifest, publish_json, unique};
use rmcp::model::{ReadResourceRequestParams, ResourceContents};
use rmcp::ServiceExt;
use serde_json::{json, Value};

static REMOTE_COUNTER_LOCK: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

/// The `ToolHost` daemon for the vectors: a `ToolHost` behind `RcpServer`,
/// built as the binary builds it.
struct ToolHostDaemon {
    receipts: PathBuf,
}

impl oracle::Daemon for ToolHostDaemon {
    fn boot(
        mgr: &Arc<TransportManager>,
        manifest: &LoadedManifest,
        receipts: &Path,
    ) -> (Self, tokio::io::DuplexStream) {
        let host = open_host_as_configured(mgr, manifest, receipts).expect("host");
        let feed = GraphSource::new(mgr.clone(), manifest.identity.clone()).feed();
        let server = RcpServer::with_tools(feed, host, manifest.state.clone());
        let (client_io, server_io) = tokio::io::duplex(64 * 1024);
        tokio::spawn(async move {
            if let Ok(server) = server.serve(server_io).await {
                let _ = server.waiting().await;
            }
        });
        let daemon = Self {
            receipts: receipts.to_path_buf(),
        };
        (daemon, client_io)
    }

    fn entries(&self) -> Vec<(String, String)> {
        receipt_pairs(&self.receipts)
    }
}

/// The receipt log at `path` as `(verb, outcome)`, `error:<detail>` for an
/// error outcome, after checking its chain. A log with no lines has no chain
/// to check (the audit refuses an empty log).
fn receipt_pairs(path: &Path) -> Vec<(String, String)> {
    let entries = cerud::receipt::read_all(path).expect("read receipts");
    if !entries.is_empty() {
        let report = cerulion_rcpd::audit::verify_log(path).expect("chain intact");
        assert_eq!(report.entries, entries.len());
    }
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

/// One test per vector, each against a fresh `ToolHost`.
macro_rules! vectors {
    ($($name:ident),* $(,)?) => {$(
        #[tokio::test(flavor = "multi_thread")]
        async fn $name() {
            oracle::run::<ToolHostDaemon>(oracle::vectors::$name()).await;
        }
    )*};
}

vectors!(
    the_tool_list_is_the_manifest_then_estop_then_estop_clear,
    each_call_is_published_once_answered_by_its_call_id_and_receipted,
    an_undeclared_tool_and_bad_arguments_never_reach_the_graph,
    a_name_no_tool_can_have_is_receipted_under_a_fixed_verb,
    a_silent_graph_times_out_and_the_call_may_have_run,
    estop_refuses_motion_and_lets_reads_run,
    a_latch_left_by_an_earlier_daemon_refuses_motion,
    an_irreversible_tool_is_receipted_labelled_and_latch_gated,
    a_clear_reopens_motion_and_a_later_estop_closes_it_again,
    a_clear_with_nothing_latched_is_receipted_and_publishes_nothing,
    a_clear_opens_a_latch_left_by_an_earlier_daemon,
    an_undeletable_latch_file_keeps_the_estop_engaged,
    under_operator_clearing_the_agent_cannot_clear,
    estop_does_not_wait_behind_a_call_in_flight,
    an_estop_during_a_waiting_clear_is_not_cleared_by_it,
);

#[tokio::test(flavor = "multi_thread")]
async fn racing_engages_and_clears_leave_the_latch_the_log_and_the_graph_agreeing() {
    oracle::properties::racing_engages_and_clears_leave_the_latch_the_log_and_the_graph_agreeing::<
        ToolHostDaemon,
    >()
    .await;
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
async fn the_state_resource_serves_the_newest_frame() {
    let session = oracle::Session::<ToolHostDaemon>::open(&oracle::Given::fresh()).await;
    for _ in 0..3 {
        let (is_error, _) = session.call("arm.step", &json!({ "q": [0.0, 0.0] })).await;
        assert!(!is_error);
    }
    // The state frame follows its answer by microseconds; poll briefly.
    let mut newest = Value::Null;
    for _ in 0..200 {
        let result = session
            .client()
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
    let missing = session
        .client()
        .read_resource(ReadResourceRequestParams::new("robot://state/ghost"))
        .await;
    assert!(
        missing.is_err(),
        "an undeclared state resource does not exist"
    );
    session.stop();
}

#[tokio::test(flavor = "multi_thread")]
async fn cached_private_state_is_counted_and_refused_after_its_producer_exits() {
    let _counter_guard = REMOTE_COUNTER_LOCK.lock().await;
    let dir = tempfile::tempdir().unwrap();
    let prefix = unique("cached_private");
    let loaded = manifest(dir.path(), &prefix);
    let mgr = manager("cached_private");
    let topic = format!("/{prefix}/state");
    let scope = mgr.exclude_remote_topic(&topic).unwrap();
    let journal = Journal::spawn(
        Arc::clone(&mgr),
        loaded.journal,
        loaded.state,
        Arc::new(AtomicBool::new(false)),
    )
    .unwrap();
    let mut private = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(4096))
        .unwrap();
    let private_oracle = json!({"n": 11});
    publish_json(&mut private, &private_oracle);
    for _ in 0..200 {
        if journal.state("arm").as_deref() == Some("{\"n\":11}") {
            break;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert_eq!(journal.state("arm").as_deref(), Some("{\"n\":11}"));
    drop(private);
    drop(scope);
    drop(mgr.acquire_remote_topic_lease(&topic).unwrap());
    let refused = mgr.remote_access_refusals();
    assert!(journal.remote_state("arm").is_err());
    assert_eq!(mgr.remote_access_refusals(), refused + 1);
    assert_eq!(journal.remote_state("unknown").unwrap(), None);
    let mut public = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(4096))
        .unwrap();
    publish_json(&mut public, &json!({"n": 23}));
    for _ in 0..200 {
        if journal.state("arm").as_deref() == Some("{\"n\":23}") {
            break;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert_eq!(
        journal.remote_state("arm").unwrap().as_deref(),
        Some("{\"n\":23}")
    );
    drop(public);
    assert_eq!(
        journal.remote_state("arm").unwrap().as_deref(),
        Some("{\"n\":23}")
    );
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

#[tokio::test(flavor = "multi_thread")]
async fn tools_list_labels_every_safety_class() {
    let session = oracle::Session::<ToolHostDaemon>::open(&oracle::Given::fresh()).await;
    let tools: BTreeMap<String, Value> = session
        .client()
        .list_all_tools()
        .await
        .expect("list")
        .into_iter()
        .map(|t| {
            let json = serde_json::to_value(&t).expect("tool JSON");
            (t.name.to_string(), json)
        })
        .collect();
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
    session.stop();
}

#[tokio::test]
async fn local_only_journal_refuses_remote_tools_but_delivers_local_calls_and_emergency_stop() {
    let _counter_guard = REMOTE_COUNTER_LOCK.lock().await;
    let dir = tempfile::tempdir().unwrap();
    let prefix = unique("local_only");
    let loaded = manifest(dir.path(), &prefix);
    let mgr = manager("local_only");
    let _scope = mgr
        .exclude_remote_topic(&loaded.journal.calls_topic)
        .unwrap();
    let graph = oracle::graph::Graph::spawn(mgr.clone(), &prefix);
    let host = open_host(&mgr, &loaded, &dir.path().join("receipts/log.jsonl")).unwrap();
    let args = json!({"q":[1.25,-2.5]}).as_object().unwrap().clone();
    let before = mgr.remote_access_refusals();
    let denied = cerulion_rcpd::caller::scope(
        "iroh:peer".into(),
        host.call("arm.step", Some(args.clone())),
    )
    .await;
    assert_eq!(denied.is_error, Some(true));
    assert!(denied.content[0]
        .as_text()
        .unwrap()
        .text
        .contains("local_only"));
    assert!(mgr.remote_access_refusals() > before);
    let local = host.call("arm.step", Some(args)).await;
    assert_ne!(local.is_error, Some(true));
    let stopped = cerulion_rcpd::caller::scope(
        "iroh:peer".into(),
        host.call(
            ESTOP_TOOL,
            Some(json!({"reason":"halt"}).as_object().unwrap().clone()),
        ),
    )
    .await;
    assert_ne!(stopped.is_error, Some(true));
    let delivered = graph.stop();
    assert_eq!(delivered.len(), 2);
    assert_eq!(delivered[0]["tool"], "arm.step");
    assert_eq!(delivered[0]["args"], json!({"q":[1.25,-2.5]}));
    assert_eq!(delivered[1]["tool"], ESTOP_TOOL);
}
