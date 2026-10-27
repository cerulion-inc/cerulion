// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion/rcp/1` plane end to end: a desk iroh endpoint dials the
//! robot's endpoint on loopback (relays disabled); the robot forwards each
//! admitted MCP message to a REAL `cerulion-rcpd` (streamable HTTP, bearer
//! token, tool host, journal) on loopback; a stand-in graph thread answers the
//! journaled calls.
//!
//! Every account row is established through a REAL certificate chain with a
//! chosen scope (deterministic seeds). Expected values (the caller string, the
//! JSON-RPC error codes, the decision JSON) are written out by hand here.

use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr};
use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::message::ShmMessage;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{TransportConfig, TransportManager};
use cerulion_link::{
    alpn, build_endpoint, dial, direct_addr, open_frame_stream, read_frame, write_frame,
    Connection, Endpoint, EndpointAddr, EndpointConfig, EndpointId, RecvStream, RelayConfig,
    SendStream,
};
use cerulion_pairing::format::{
    AccessListEpoch, AccountId, DeviceCert, Grant, IntermediateCert, PrincipalKind, PublicKey,
    RobotId, Role, RootSet, Scope, Validity, FORMAT_VERSION,
};
use cerulion_pairing::verify::{PairingPresentation, TrustStore};
use cerulion_rcpd::http::{BearerToken, HttpServer};
use cerulion_rcpd::journal::Journal;
use cerulion_rcpd::manifest::RcpManifest;
use cerulion_rcpd::observer::GraphSource;
use cerulion_rcpd::server::RcpServer;
use cerulion_rcpd::tools::{ReceiptLock, ToolHost};
use cerulion_remoted::rcp::{RcpLimits, RcpLink, RcpPlane, DENIED_CODE, INVALID_REQUEST_CODE};
use cerulion_remoted::{DeviceAccountIndex, PairingAuthorizer, RemotedClock, SharedTrust};
use ed25519_dalek::SigningKey;
use native_ros2_messages::std_msgs::String as RosString;
use serde_json::{json, Value};

const STEP: Duration = Duration::from_secs(20);
const T_NOW: u64 = 1_000_000_000_000;
const ISSUED: u64 = 500_000_000_000;
const CHASSIS: &[u8] = b"rcp-e2e-chassis-secret-not-serial-derived";
const MAC_KEY: &[u8] = b"rcp-e2e-firmware-mac-key";
const TOKEN: &str = "rcp-e2e-fixture-token";
const ROBOT_SEED: u8 = 60;
const OWNER: AccountId = AccountId([10; 32]);
const ROBOT_ID: RobotId = RobotId([5; 32]);

/// (desk seed, account, scope) for every paired fixture desk.
const OPERATOR: (u8, AccountId, Scope) = (
    31,
    AccountId([30; 32]),
    Scope {
        role: Role::OPERATOR,
        caps: Scope::CAP_TELEOP | Scope::CAP_OBSERVE,
    },
);
const VIEWER: (u8, AccountId, Scope) = (
    21,
    AccountId([20; 32]),
    Scope {
        role: Role::VIEWER,
        caps: Scope::CAP_OBSERVE,
    },
);
const UNPAIRED_SEED: u8 = 99;
/// The fixture's idle re-check interval, short so the quiet-revoke test runs fast.
const QUIET_REVOKE_CHECK: Duration = Duration::from_millis(400);

static COUNTER: AtomicU64 = AtomicU64::new(0);

async fn bounded<F: std::future::Future>(what: &str, fut: F) -> F::Output {
    tokio::time::timeout(STEP, fut)
        .await
        .unwrap_or_else(|_| panic!("'{what}' timed out after {STEP:?}"))
}

fn sk(seed: u8) -> SigningKey {
    SigningKey::from_bytes(&[seed; 32])
}
fn pk(k: &SigningKey) -> PublicKey {
    PublicKey(k.verifying_key().to_bytes())
}
fn wide() -> Validity {
    Validity {
        not_before_ns: 0,
        not_after_ns: u64::MAX / 2,
    }
}

fn intermediate_cert() -> cerulion_pairing::format::SignedIntermediateCert {
    IntermediateCert {
        version: FORMAT_VERSION,
        intermediate_key: pk(&sk(2)),
        validity: wide(),
        issued_at_ns: ISSUED,
        max_scope: Scope::OWNER_FULL,
    }
    .sign_by_roots(&[&sk(1)])
}

fn establish(store: &mut TrustStore, seed: u8, account: AccountId, scope: Scope) -> PublicKey {
    let device_key = pk(&sk(seed));
    store
        .verify_and_establish(
            &presentation(seed, account, scope),
            &device_key,
            T_NOW,
            Some(format!("desk-{seed}")),
        )
        .expect("the fixture chain verifies");
    device_key
}

fn presentation(seed: u8, account: AccountId, scope: Scope) -> PairingPresentation {
    let intermediate = sk(2);
    let int_pk = pk(&intermediate);
    let device_key = pk(&sk(seed));
    let device_cert = DeviceCert {
        version: FORMAT_VERSION,
        device_key,
        account,
        principal_kind: PrincipalKind::Human,
        scope,
        validity: wide(),
        issued_at_ns: ISSUED,
        issuer_key: int_pk,
    }
    .sign(&intermediate);
    let grant = Grant {
        version: FORMAT_VERSION,
        subject: account,
        robot: ROBOT_ID,
        scope,
        principal_kind: PrincipalKind::Human,
        delegation_depth: 0,
        validity: wide(),
        issued_at_ns: ISSUED,
        issuer: AccountId([2; 32]),
        issuer_key: int_pk,
    }
    .sign(&intermediate);
    PairingPresentation {
        intermediate: intermediate_cert(),
        device_cert,
        grant,
        delegation: None,
    }
}

/// A claimed robot whose store is bound to `robot_key`, with the OPERATOR and
/// VIEWER desks paired, on a fixed clock at `T_NOW`.
fn robot_trust(dir: &Path, robot_key: [u8; 32]) -> SharedTrust {
    let root_set = RootSet::new(vec![pk(&sk(1))], 1).unwrap();
    let mut store = TrustStore::provision(ROBOT_ID, PublicKey(robot_key), root_set, CHASSIS, T_NOW)
        .unwrap()
        .with_path(dir.join("trust_store"));
    store
        .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    let mut index = DeviceAccountIndex::new().with_path(dir.join("device_index.json"));
    for (seed, account, scope) in [OPERATOR, VIEWER] {
        let key = establish(&mut store, seed, account, scope);
        index.bind(key, account);
    }
    store.save(MAC_KEY).unwrap();
    index.save(MAC_KEY).unwrap();
    SharedTrust::new_with_clock(store, index, MAC_KEY.to_vec(), RemotedClock::fixed(T_NOW))
}

fn loopback_addr(id: EndpointId, bound: &[SocketAddr]) -> EndpointAddr {
    direct_addr(
        id,
        bound.iter().map(|s| match s {
            SocketAddr::V4(v4) if v4.ip().is_unspecified() => {
                SocketAddr::from((Ipv4Addr::LOCALHOST, v4.port()))
            }
            SocketAddr::V6(v6) if v6.ip().is_unspecified() => {
                SocketAddr::from((Ipv6Addr::LOCALHOST, v6.port()))
            }
            other => *other,
        }),
    )
}

async fn endpoint(seed: u8) -> Endpoint {
    bounded(
        "build_endpoint",
        build_endpoint(EndpointConfig::new([seed; 32]).with_relay(RelayConfig::Disabled)),
    )
    .await
    .expect("endpoint binds")
}

/// The stand-in graph: answers every journaled call with
/// `{call_id, ok, caller}` and records the frames it saw.
fn spawn_graph(
    mgr: Arc<TransportManager>,
    prefix: String,
    stop: Arc<AtomicBool>,
    seen: Arc<Mutex<Vec<Value>>>,
) -> std::thread::JoinHandle<()> {
    let (ready_tx, ready_rx) = std::sync::mpsc::channel();
    let handle = std::thread::spawn(move || {
        let calls = mgr
            .create_subscriber(&format!("/{prefix}/calls"))
            .expect("calls subscriber");
        let mut results = mgr
            .create_publisher_simple(&format!("/{prefix}/results"), MaxSliceLen::const_new(4096))
            .expect("results publisher");
        ready_tx.send(()).expect("ready");
        while !stop.load(Ordering::Relaxed) {
            let mut batch = Vec::new();
            calls
                .try_receive(|m| {
                    let text = RosString::build_reader(m.payload())
                        .data()
                        .expect("utf-8")
                        .to_owned();
                    batch.push(serde_json::from_str::<Value>(&text).expect("JSON call"));
                })
                .expect("receive");
            if batch.is_empty() {
                std::thread::sleep(Duration::from_millis(1));
                continue;
            }
            for call in batch {
                seen.lock().unwrap().push(call.clone());
                if call["tool"] == json!("arm.mute")
                    || (call["tool"] == json!("estop") && call["args"]["reason"] == json!("hold"))
                {
                    continue;
                }
                if call["tool"] == json!("arm.slow") {
                    std::thread::sleep(Duration::from_millis(1500));
                }
                let answer = json!({ "call_id": call["call_id"], "ok": true });
                let mut proxy = results.loan_proxy::<RosString>().expect("loan");
                proxy.set_data(&answer.to_string()).expect("set");
            }
        }
    });
    ready_rx.recv().expect("graph ready");
    handle
}

struct Fixture {
    robot: Endpoint,
    trust: SharedTrust,
    receipts: std::path::PathBuf,
    seen: Arc<Mutex<Vec<Value>>>,
    stop: Arc<AtomicBool>,
    graph: Option<std::thread::JoinHandle<()>>,
    stop_serving: Option<tokio::sync::oneshot::Sender<()>>,
    stop_rcpd: Option<tokio::sync::oneshot::Sender<()>>,
    _dir: tempfile::TempDir,
}

impl Fixture {
    async fn new() -> Self {
        let dir = tempfile::tempdir().unwrap();
        let n = COUNTER.fetch_add(1, Ordering::Relaxed);
        let prefix = format!("rcp_e2e_{}_{n}", std::process::id());
        let manifest = format!(
            "robot: rcp_e2e
journal:
  calls_topic: /{prefix}/calls
  results_topic: /{prefix}/results
tools:
  - name: arm.step
    description: one period
    safety_class: reversible
    timeout_ms: 5000
    input_schema:
      type: object
      properties:
        q: {{ type: array, items: {{ type: number }}, minItems: 2, maxItems: 2 }}
      required: [q]
  - name: arm.slow
    description: answered after 1.5 s
    safety_class: reversible
    timeout_ms: 5000
    input_schema: {{ type: object, properties: {{}} }}
  - name: arm.mute
    description: never answered
    safety_class: reversible
    timeout_ms: 3000
    input_schema: {{ type: object, properties: {{}} }}
"
        );
        let manifest_path = dir.path().join("rcp.yaml");
        std::fs::write(&manifest_path, manifest).unwrap();
        let loaded = RcpManifest::load_full(&manifest_path).expect("manifest");
        let mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: prefix.clone(),
                ..Default::default()
            },
            cerulion_core::testing::iceoryx_test_config(),
        )
        .expect("isolated transport");
        let stop = Arc::new(AtomicBool::new(false));
        let seen = Arc::new(Mutex::new(Vec::new()));
        let graph = spawn_graph(mgr.clone(), prefix.clone(), stop.clone(), seen.clone());

        // The real rcpd: tool host, journal, bearer HTTP on an ephemeral port.
        let receipts = dir.path().join("rcp/receipts.jsonl");
        let lock = ReceiptLock::acquire(&receipts).expect("receipt lock");
        let latch = Arc::new(AtomicBool::new(false));
        let journal = Journal::spawn(
            mgr.clone(),
            loaded.journal.clone(),
            loaded.state.clone(),
            latch.clone(),
        )
        .expect("journal");
        let host = Arc::new(
            ToolHost::open(loaded.tools.clone(), journal, latch, &receipts, lock).expect("host"),
        );
        let feed = GraphSource::new(mgr, loaded.identity.clone()).feed();
        let state = loaded.state.clone();
        let token = BearerToken::from_env_value(Some(TOKEN.into())).unwrap();
        let http = HttpServer::bind_serving("127.0.0.1:0".parse().unwrap(), token, move || {
            RcpServer::with_tools(feed.clone(), host.clone(), state.clone())
        })
        .await
        .expect("rcpd binds");
        let rcpd_addr = http.local_addr().unwrap();
        let (stop_rcpd, rcpd_rx) = tokio::sync::oneshot::channel::<()>();
        tokio::spawn(http.serve(async {
            let _ = rcpd_rx.await;
        }));

        // The robot: one endpoint, the rcp plane over the live trust state.
        let token_file = dir.path().join("rcp.token");
        std::fs::write(&token_file, format!("CERULION_RCP_TOKEN={TOKEN}\n")).unwrap();
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            std::fs::set_permissions(&token_file, std::fs::Permissions::from_mode(0o600)).unwrap();
        }
        let robot = endpoint(ROBOT_SEED).await;
        let trust = robot_trust(dir.path(), *robot.id().as_bytes());
        let classify = Arc::new(PairingAuthorizer::from_shared(trust.clone()));
        let mut link = RcpLink::under(dir.path());
        link.addr = rcpd_addr;
        link.token_file = token_file;
        link.limits.revoke_check = QUIET_REVOKE_CHECK;
        let plane = Arc::new(RcpPlane::new(link, trust.clone()));
        let (stop_serving, serve_rx) = tokio::sync::oneshot::channel::<()>();
        let serving_endpoint = robot.clone();
        tokio::spawn(async move {
            let _ = cerulion_remoted::serve_planes(
                &serving_endpoint,
                classify,
                None,
                None,
                None,
                Some(plane),
                async {
                    let _ = serve_rx.await;
                },
            )
            .await;
        });
        Fixture {
            robot,
            trust,
            receipts,
            seen,
            stop,
            graph: Some(graph),
            stop_serving: Some(stop_serving),
            stop_rcpd: Some(stop_rcpd),
            _dir: dir,
        }
    }

    fn robot_addr(&self) -> EndpointAddr {
        loopback_addr(self.robot.id(), &self.robot.bound_sockets())
    }

    async fn session(&self, seed: u8) -> Session {
        let desk = endpoint(seed).await;
        let conn = bounded("dial", dial(&desk, self.robot_addr(), alpn::RCP))
            .await
            .expect("dial");
        let (send, recv) = bounded("open_bi", open_frame_stream(&conn))
            .await
            .expect("open_bi");
        Session {
            _desk: desk,
            _conn: conn,
            send,
            recv,
        }
    }

    fn receipt_callers(&self) -> Vec<(String, String)> {
        cerud::receipt::read_all(&self.receipts)
            .unwrap_or_default()
            .into_iter()
            .map(|r| (r.verb, r.caller))
            .collect()
    }
}

impl Drop for Fixture {
    fn drop(&mut self) {
        if let Some(tx) = self.stop_serving.take() {
            let _ = tx.send(());
        }
        if let Some(tx) = self.stop_rcpd.take() {
            let _ = tx.send(());
        }
        self.stop.store(true, Ordering::Relaxed);
        if let Some(g) = self.graph.take() {
            let _ = g.join();
        }
    }
}

struct Session {
    _desk: Endpoint,
    _conn: Connection,
    send: SendStream,
    recv: RecvStream,
}

impl Session {
    async fn send_raw(&mut self, frame: &[u8]) {
        bounded("write_frame", write_frame(&mut self.send, frame))
            .await
            .expect("write");
    }

    /// Read frames until one carries `id`; notifications in between are skipped.
    async fn reply(&mut self, id: &Value) -> Value {
        loop {
            let frame = bounded("read_frame", read_frame(&mut self.recv, 16 << 20))
                .await
                .expect("a reply frame");
            let v: Value = serde_json::from_slice(&frame).expect("JSON reply");
            if v.get("id") == Some(id) {
                return v;
            }
        }
    }

    async fn request(&mut self, id: i64, method: &str, params: Value) -> Value {
        let msg = json!({ "jsonrpc": "2.0", "id": id, "method": method, "params": params });
        self.send_raw(msg.to_string().as_bytes()).await;
        self.reply(&json!(id)).await
    }

    async fn initialize(&mut self) {
        let r = self
            .request(
                1,
                "initialize",
                json!({
                    "protocolVersion": "2025-06-18",
                    "capabilities": {},
                    "clientInfo": { "name": "rcp-e2e", "version": "0" }
                }),
            )
            .await;
        assert!(r.get("result").is_some(), "initialize: {r}");
        let n = json!({ "jsonrpc": "2.0", "method": "notifications/initialized" });
        self.send_raw(n.to_string().as_bytes()).await;
    }

    async fn call_step(&mut self, id: i64) -> Value {
        self.request(
            id,
            "tools/call",
            json!({ "name": "arm.step", "arguments": { "q": [0.1, -0.2] } }),
        )
        .await
    }
}

fn desk_key(seed: u8) -> [u8; 32] {
    pk(&sk(seed)).0
}

fn expected_caller(seed: u8, account: AccountId) -> String {
    format!(
        "iroh:{}/account:{}",
        hex::encode(desk_key(seed)),
        hex::encode(account.0)
    )
}

fn denied(reply: &Value, code: i64) -> bool {
    reply["error"]["code"] == json!(code)
}

#[tokio::test(flavor = "multi_thread")]
async fn operator_with_teleop_calls_a_tool_under_its_own_identity() {
    let f = Fixture::new().await;
    let mut s = f.session(OPERATOR.0).await;
    s.initialize().await;
    let listed = s.request(2, "tools/list", json!({})).await;
    let names: Vec<&str> = listed["result"]["tools"]
        .as_array()
        .expect("tool list")
        .iter()
        .filter_map(|t| t["name"].as_str())
        .collect();
    assert_eq!(names, ["arm.step", "arm.slow", "arm.mute", "estop"]);
    let r = s.call_step(3).await;
    let text = r["result"]["content"][0]["text"]
        .as_str()
        .expect("text block");
    let body: Value = serde_json::from_str(text).unwrap();
    assert_eq!(body["result"]["ok"], json!(true), "{r}");

    let caller = expected_caller(OPERATOR.0, OPERATOR.1);
    assert_eq!(
        f.receipt_callers(),
        [
            ("arm.step".to_string(), caller.clone()),
            ("arm.step".to_string(), caller.clone())
        ],
        "intent + ok receipts name the remote caller"
    );
    let seen = f.seen.lock().unwrap().clone();
    assert_eq!(seen.len(), 1);
    assert_eq!(
        seen[0]["caller"],
        json!(caller),
        "the journaled call names it"
    );
    assert_eq!(seen[0]["tool"], json!("arm.step"));
}

#[tokio::test(flavor = "multi_thread")]
async fn viewer_reads_resources_but_its_tool_call_never_reaches_rcpd() {
    let f = Fixture::new().await;
    let mut s = f.session(VIEWER.0).await;
    s.initialize().await;
    let listed = s.request(2, "resources/list", json!({})).await;
    let uris: Vec<&str> = listed["result"]["resources"]
        .as_array()
        .expect("resources")
        .iter()
        .filter_map(|r| r["uri"].as_str())
        .collect();
    assert!(uris.contains(&"robot://graph"), "{listed}");
    let read = s
        .request(3, "resources/read", json!({ "uri": "robot://graph" }))
        .await;
    let doc = read["result"]["contents"][0]["text"]
        .as_str()
        .expect("graph text");
    assert!(serde_json::from_str::<Value>(doc).is_ok(), "graph is JSON");

    let r = s.call_step(4).await;
    assert!(denied(&r, DENIED_CODE), "viewer tools/call: {r}");
    assert!(
        r["error"]["message"]
            .as_str()
            .unwrap_or("")
            .contains("needs more access"),
        "denied before forwarding: {r}"
    );
    assert!(
        f.receipt_callers().is_empty(),
        "nothing reached rcpd's tools"
    );
    assert!(
        f.seen.lock().unwrap().is_empty(),
        "nothing reached the graph"
    );
}

#[tokio::test(flavor = "multi_thread")]
async fn any_paired_viewer_engages_the_estop_but_no_other_tool() {
    let f = Fixture::new().await;
    let mut s = f.session(VIEWER.0).await;
    s.initialize().await;
    let r = s.call_step(2).await;
    assert!(denied(&r, DENIED_CODE), "viewer arm.step: {r}");
    let r = s
        .request(
            3,
            "tools/call",
            json!({ "name": "estop", "arguments": { "reason": "rcp e2e" } }),
        )
        .await;
    let text = r["result"]["content"][0]["text"]
        .as_str()
        .expect("estop body");
    let body: Value = serde_json::from_str(text).unwrap();
    assert_eq!(body["estopped"], json!(true), "{r}");
    let caller = expected_caller(VIEWER.0, VIEWER.1);
    assert_eq!(
        f.receipt_callers(),
        [
            ("estop".to_string(), caller.clone()),
            ("estop".to_string(), caller.clone())
        ]
    );
    let seen = f.seen.lock().unwrap().clone();
    assert_eq!(seen.len(), 1, "only the estop reached the graph");
    assert_eq!(seen[0]["tool"], json!("estop"));
    assert_eq!(seen[0]["caller"], json!(caller));
}

#[tokio::test(flavor = "multi_thread")]
async fn an_unpaired_key_is_refused_at_accept() {
    let f = Fixture::new().await;
    let mut s = f.session(UNPAIRED_SEED).await;
    s.send_raw(br#"{"jsonrpc":"2.0","id":1,"method":"ping"}"#)
        .await;
    let frame = bounded("decision", read_frame(&mut s.recv, 4096))
        .await
        .expect("the skeleton decision frame");
    let v: Value = serde_json::from_slice(&frame).unwrap();
    assert_eq!(v["decision"], json!("refuse"), "{v}");
    assert!(
        v["reason"].as_str().unwrap_or("").contains("unpaired"),
        "{v}"
    );
    assert!(f.receipt_callers().is_empty());
}

#[tokio::test(flavor = "multi_thread")]
async fn malformed_bodies_are_denied_and_the_session_continues() {
    let f = Fixture::new().await;
    let mut s = f.session(OPERATOR.0).await;
    s.initialize().await;
    for bad in [
        &b"not json"[..],
        br#"[{"jsonrpc":"2.0","id":7,"method":"tools/call","params":{"name":"arm.step"}}]"#,
        br#"{"jsonrpc":"2.0","id":7.5,"method":"ping"}"#,
    ] {
        s.send_raw(bad).await;
        let r = s.reply(&Value::Null).await;
        assert!(denied(&r, INVALID_REQUEST_CODE), "{r}");
    }
    s.send_raw(br#"{"jsonrpc":"2.0","id":8,"method":"tools/call","params":{"arguments":{}}}"#)
        .await;
    assert!(denied(&s.reply(&json!(8)).await, INVALID_REQUEST_CODE));
    s.send_raw(br#"{"jsonrpc":"2.0","id":9,"method":"prompts/list","params":{}}"#)
        .await;
    assert!(denied(&s.reply(&json!(9)).await, INVALID_REQUEST_CODE));
    let again = s
        .request(
            11,
            "initialize",
            json!({ "protocolVersion": "2025-06-18", "capabilities": {},
                    "clientInfo": { "name": "again", "version": "0" } }),
        )
        .await;
    assert!(
        denied(&again, INVALID_REQUEST_CODE),
        "one initialize per stream: {again}"
    );
    let pong = s.request(10, "ping", json!({})).await;
    assert!(
        pong.get("result").is_some(),
        "the session still serves: {pong}"
    );
    assert!(f.receipt_callers().is_empty());
    assert!(f.seen.lock().unwrap().is_empty());
}

#[tokio::test(flavor = "multi_thread")]
async fn a_revoke_mid_session_denies_the_next_call() {
    let f = Fixture::new().await;
    let mut s = f.session(OPERATOR.0).await;
    s.initialize().await;
    let first = s.call_step(2).await;
    assert!(first.get("result").is_some(), "{first}");
    // Revoke the operator's DEVICE by a signed epoch on the LIVE trust state.
    revoke_operator_device(&f);
    s.send_raw(
        json!({ "jsonrpc": "2.0", "id": 3, "method": "tools/call",
                "params": { "name": "arm.step", "arguments": { "q": [0.0, 0.0] } } })
        .to_string()
        .as_bytes(),
    )
    .await;
    // The writer re-checks the session floor before each frame back and ends
    // the session: the desk sees the stream close, never a result.
    let next = bounded("closed", read_frame(&mut s.recv, 1 << 20)).await;
    assert!(next.is_err(), "no frame reaches a revoked desk: {next:?}");
    assert_eq!(
        f.seen.lock().unwrap().len(),
        1,
        "the revoked call never reached the graph"
    );
}

#[test]
fn the_default_rcpd_address_matches_rcpd() {
    assert_eq!(
        cerulion_remoted::rcp::DEFAULT_RCPD_ADDR.to_string(),
        cerulion_rcpd::DEFAULT_HTTP_ADDR
    );
    assert_eq!(cerulion_remoted::rcp::TOKEN_ENV, cerulion_rcpd::TOKEN_ENV);
    assert_eq!(
        cerulion_remoted::rcp::CALLER_HEADER,
        cerulion_rcpd::caller::CALLER_HEADER
    );
    assert_eq!(
        cerulion_remoted::rcp::ESTOP_TOOL,
        cerulion_rcpd::tools::ESTOP_TOOL
    );
}

#[tokio::test(flavor = "multi_thread")]
async fn the_estop_never_waits_behind_full_request_slots() {
    let f = Fixture::new().await;
    let mut s = f.session(OPERATOR.0).await;
    s.initialize().await;
    // Fill every in-flight slot with calls the graph never answers.
    let slots = RcpLimits::default().in_flight as i64;
    for id in 0..slots {
        let m = json!({ "jsonrpc": "2.0", "id": 100 + id, "method": "tools/call",
                        "params": { "name": "arm.mute", "arguments": {} } });
        s.send_raw(m.to_string().as_bytes()).await;
    }
    // Let the forwards take their slots before the next calls.
    tokio::time::sleep(Duration::from_millis(300)).await;
    let busy = s.request(4, "ping", json!({})).await;
    assert_eq!(
        busy["error"]["code"],
        json!(cerulion_remoted::rcp::UPSTREAM_CODE),
        "a lifecycle body takes a slot too: {busy}"
    );
    let over = s.call_step(2).await;
    assert_eq!(
        over["error"]["code"],
        json!(cerulion_remoted::rcp::UPSTREAM_CODE),
        "{over}"
    );
    let started = std::time::Instant::now();
    let r = s
        .request(3, "tools/call", json!({ "name": "estop", "arguments": {} }))
        .await;
    let text = r["result"]["content"][0]["text"]
        .as_str()
        .expect("estop body");
    assert_eq!(
        serde_json::from_str::<Value>(text).unwrap()["estopped"],
        json!(true)
    );
    assert!(
        started.elapsed() < Duration::from_millis(2500),
        "the estop answered behind the muted calls: {:?}",
        started.elapsed()
    );
}

#[tokio::test(flavor = "multi_thread")]
async fn a_reply_is_withheld_when_the_scope_loses_its_class_mid_call() {
    let f = Fixture::new().await;
    let mut s = f.session(OPERATOR.0).await;
    s.initialize().await;
    let m = json!({ "jsonrpc": "2.0", "id": 2, "method": "tools/call",
                    "params": { "name": "arm.slow", "arguments": {} } });
    s.send_raw(m.to_string().as_bytes()).await;
    tokio::time::sleep(Duration::from_millis(300)).await;
    // Re-establish the operator's account without CAP_TELEOP while the graph
    // is still answering.
    let pres = presentation(
        OPERATOR.0,
        OPERATOR.1,
        Scope {
            role: Role::OPERATOR,
            caps: Scope::CAP_OBSERVE,
        },
    );
    f.trust
        .verify_and_establish(
            &pres,
            &desk_key(OPERATOR.0),
            Some("downgraded".into()),
            T_NOW,
        )
        .expect("re-established with less scope");
    let r = s.reply(&json!(2)).await;
    assert_eq!(
        r["error"]["code"],
        json!(DENIED_CODE),
        "the result was withheld: {r}"
    );
    // The session itself stays: the account is still on the access list.
    let pong = s.request(3, "ping", json!({})).await;
    assert!(pong.get("result").is_some(), "{pong}");
}

fn revoke_operator_device(f: &Fixture) {
    let epoch = AccessListEpoch {
        version: FORMAT_VERSION,
        robot: ROBOT_ID,
        epoch: 2,
        revoked_accounts: vec![],
        revoked_devices: vec![PublicKey(desk_key(OPERATOR.0))],
        issued_at_ns: ISSUED,
        issuer_key: pk(&sk(2)),
    }
    .sign(&sk(2));
    f.trust
        .apply_epoch(&epoch, &intermediate_cert(), T_NOW)
        .expect("epoch applies");
}

#[tokio::test(flavor = "multi_thread")]
async fn a_quiet_revoked_session_is_closed_by_the_periodic_check() {
    let f = Fixture::new().await;
    let mut s = f.session(OPERATOR.0).await;
    s.initialize().await;
    revoke_operator_device(&f);
    // The desk sends nothing; the robot still closes the connection.
    let limit = QUIET_REVOKE_CHECK * 5;
    let next = tokio::time::timeout(limit, read_frame(&mut s.recv, 1 << 20))
        .await
        .expect("the robot closed the session within the check interval");
    assert!(next.is_err(), "{next:?}");
}

#[tokio::test(flavor = "multi_thread")]
async fn a_connection_past_the_per_key_cap_supersedes_the_oldest() {
    let f = Fixture::new().await;
    let cap = RcpLimits::default().connections_per_key;
    let mut held = Vec::new();
    for _ in 0..cap {
        let mut s = f.session(VIEWER.0).await;
        s.initialize().await;
        held.push(s);
    }
    // A reconnect past the cap (a desk that crashed leaves its old
    // connections open on the robot) is admitted at once.
    let started = std::time::Instant::now();
    let mut newest = f.session(VIEWER.0).await;
    newest.initialize().await;
    let pong = newest.request(2, "ping", json!({})).await;
    assert!(pong.get("result").is_some(), "{pong}");
    assert!(
        started.elapsed() < Duration::from_secs(3),
        "admitted at once: {:?}",
        started.elapsed()
    );
    // The oldest is closed with the superseded code; the rest still serve.
    let oldest = bounded("oldest closed", read_frame(&mut held[0].recv, 1 << 20)).await;
    let err = format!("{:?}", oldest.expect_err("the oldest connection is closed"));
    assert!(
        err.contains(&cerulion_remoted::rcp::CLOSE_SUPERSEDED.to_string()),
        "closed with CLOSE_SUPERSEDED: {err}"
    );
    let pong = held[1].request(9, "ping", json!({})).await;
    assert!(
        pong.get("result").is_some(),
        "the others still serve: {pong}"
    );
}

#[tokio::test(flavor = "multi_thread")]
async fn estops_in_flight_are_bounded_and_the_excess_answered_at_once() {
    let f = Fixture::new().await;
    let mut s = f.session(VIEWER.0).await;
    s.initialize().await;
    let cap = RcpLimits::default().estops_in_flight as i64;
    // The graph never acknowledges reason "hold", so these stay in flight.
    for id in 0..cap {
        let m = json!({ "jsonrpc": "2.0", "id": 100 + id, "method": "tools/call",
                        "params": { "name": "estop", "arguments": { "reason": "hold" } } });
        s.send_raw(m.to_string().as_bytes()).await;
    }
    tokio::time::sleep(Duration::from_millis(300)).await;
    let started = std::time::Instant::now();
    let over = s
        .request(
            7,
            "tools/call",
            json!({ "name": "estop", "arguments": { "reason": "hold" } }),
        )
        .await;
    assert_eq!(
        over["error"]["code"],
        json!(cerulion_remoted::rcp::UPSTREAM_CODE),
        "{over}"
    );
    assert!(
        started.elapsed() < Duration::from_secs(1),
        "answered at once"
    );
}
