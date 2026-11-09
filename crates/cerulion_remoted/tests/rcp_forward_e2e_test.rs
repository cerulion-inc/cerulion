// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion/rcp/1` plane end to end: a desk iroh endpoint dials the
//! robot's endpoint on loopback (relays disabled); the robot forwards each
//! admitted MCP message to a REAL `cerulion-rcpd` (streamable HTTP, bearer
//! token, tool host, journal) on loopback; a stand-in graph thread answers the
//! journaled calls. Some arms forward to a hand-rolled HTTP/1 stand-in for
//! rcpd that holds a request unanswered, and some hold a request between its
//! open connection to rcpd and its send, through the plane's test seam
//! (`RcpPlane::hold_after_connect_for_test`). The ops e-stop arms serve the
//! whole daemon (`serve_endpoint`, its ops and rcp planes over one e-stop
//! lease) and reach the real rcpd through a loopback proxy that reports each
//! request, can hold it, and reports when rcpd begins to answer it.
//!
//! Every account row is established through a REAL certificate chain with a
//! chosen scope (deterministic seeds). Expected values (the caller string, the
//! JSON-RPC error codes, the decision JSON) are written out by hand here.

use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr};
use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerud::client::OpsClient;
use cerud::lease::{ControlLease, EstopState};
use cerud::verbs::VerbHandler;
use cerud::CerudError;
use cerulion_core::message::ShmMessage;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{TransportConfig, TransportManager};
use cerulion_link::{
    alpn, build_endpoint, dial, direct_addr, open_frame_stream, read_frame, write_frame,
    Connection, Endpoint, EndpointAddr, EndpointConfig, EndpointId, QuicOpsStream, RecvStream,
    RelayConfig, SendStream,
};
use cerulion_pairing::format::{
    AccessListEpoch, AccountId, DeviceCert, Grant, IntermediateCert, PrincipalKind, PublicKey,
    RobotId, Role, RootSet, Scope, Validity, FORMAT_VERSION,
};
use cerulion_pairing::verify::{PairingPresentation, TrustStore};
use cerulion_rcpd::http::{BearerToken, HttpServer};
use cerulion_rcpd::journal::Journal;
use cerulion_rcpd::manifest::{RcpManifest, StateResource};
use cerulion_rcpd::observer::{GraphFeed, GraphSource};
use cerulion_rcpd::server::RcpServer;
use cerulion_rcpd::tools::{ReceiptLock, ToolHost};
use cerulion_remoted::act_verbs::TopicPublishVerb;
use cerulion_remoted::rcp::{ConnectHold, RcpLimits, RcpLink, RcpPlane};
use cerulion_remoted::{
    serve_endpoint, DeviceAccountIndex, EstopLease, PairingAuthorizer, RemotedClock, RemotedConfig,
    SharedTrust,
};
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
    let (store, index) = robot_store(dir, robot_key);
    SharedTrust::new_with_clock(store, index, MAC_KEY.to_vec(), RemotedClock::fixed(T_NOW))
}

/// The store and device index of [`robot_trust`].
fn robot_store(dir: &Path, robot_key: [u8; 32]) -> (TrustStore, DeviceAccountIndex) {
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
    (store, index)
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
/// `{call_id, ok, caller}` and records the frames it saw. It never answers
/// `arm.mute` or an `estop` with reason `hold`, and holds each `arm.hold`
/// until `release` is set.
fn spawn_graph(
    mgr: Arc<TransportManager>,
    prefix: String,
    stop: Arc<AtomicBool>,
    seen: Arc<Mutex<Vec<Value>>>,
    release: Arc<AtomicBool>,
) -> std::thread::JoinHandle<()> {
    let (ready_tx, ready_rx) = std::sync::mpsc::channel();
    let handle = std::thread::spawn(move || {
        let calls = mgr
            .create_subscriber(&format!("/{prefix}/calls"))
            .expect("calls subscriber");
        let mut results = mgr
            .create_publisher_simple(&format!("/{prefix}/results"), MaxSliceLen::const_new(4096))
            .expect("results publisher");
        let mut answer = |call: &Value| {
            let answer = json!({ "call_id": call["call_id"], "ok": true });
            let mut proxy = results.loan_proxy::<RosString>().expect("loan");
            proxy.set_data(&answer.to_string()).expect("set");
        };
        let mut held = Vec::new();
        ready_tx.send(()).expect("ready");
        while !stop.load(Ordering::Relaxed) {
            if release.load(Ordering::Relaxed) {
                held.drain(..).for_each(|call: Value| answer(&call));
            }
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
                if call["tool"] == json!("arm.hold") {
                    held.push(call);
                    continue;
                }
                if call["tool"] == json!("arm.mute")
                    || (call["tool"] == json!("estop") && call["args"]["reason"] == json!("hold"))
                {
                    continue;
                }
                if call["tool"] == json!("arm.slow") {
                    std::thread::sleep(Duration::from_millis(1500));
                }
                answer(&call);
            }
        }
    });
    ready_rx.recv().expect("graph ready");
    handle
}

/// A real rcpd serving `host` over bearer HTTP on `addr`.
struct Rcpd {
    host: Arc<ToolHost>,
    feed: GraphFeed,
    state: Vec<StateResource>,
}

impl Rcpd {
    /// Bind `addr` and serve until the returned sender fires or drops.
    async fn serve(
        &self,
        addr: SocketAddr,
    ) -> Result<(SocketAddr, tokio::sync::oneshot::Sender<()>), cerulion_rcpd::RcpdError> {
        let (feed, host, state) = (self.feed.clone(), self.host.clone(), self.state.clone());
        let token = BearerToken::from_env_value(Some(TOKEN.into())).unwrap();
        let http = HttpServer::bind_serving(addr, token, move || {
            RcpServer::with_tools(feed.clone(), host.clone(), state.clone())
        })
        .await?;
        let bound = http.local_addr().unwrap();
        let (stop, rx) = tokio::sync::oneshot::channel::<()>();
        tokio::spawn(http.serve(async {
            let _ = rx.await;
        }));
        Ok((bound, stop))
    }
}

struct Fixture {
    robot: Endpoint,
    trust: SharedTrust,
    lease: EstopLease,
    stop_serving: Option<tokio::sync::oneshot::Sender<()>>,
    side: RcpdSide,
    _dir: tempfile::TempDir,
}

impl Fixture {
    async fn new() -> Self {
        Self::with_limits(|_| {}).await
    }

    /// The fixture with the plane's limits adjusted by `adjust`.
    async fn with_limits(adjust: impl FnOnce(&mut RcpLimits)) -> Self {
        let dir = tempfile::tempdir().unwrap();
        let side = RcpdSide::start(dir.path()).await;
        let ServedPlane {
            robot,
            trust,
            lease,
            plane: _,
            stop_serving,
        } = serve_rcp_plane(dir.path(), side.rcpd_addr, adjust).await;
        Fixture {
            robot,
            trust,
            lease,
            stop_serving: Some(stop_serving),
            side,
            _dir: dir,
        }
    }

    async fn session(&self, seed: u8) -> Session {
        open_session(&self.robot, seed).await
    }
}

impl std::ops::Deref for Fixture {
    type Target = RcpdSide;
    fn deref(&self) -> &RcpdSide {
        &self.side
    }
}

impl std::ops::DerefMut for Fixture {
    fn deref_mut(&mut self) -> &mut RcpdSide {
        &mut self.side
    }
}

impl Drop for Fixture {
    fn drop(&mut self) {
        if let Some(tx) = self.stop_serving.take() {
            let _ = tx.send(());
        }
    }
}

/// The robot's rcpd half: a real rcpd (tool host, journal, bearer HTTP on an
/// ephemeral loopback port) and the stand-in graph that answers it.
struct RcpdSide {
    mgr: Arc<TransportManager>,
    receipts: std::path::PathBuf,
    seen: Arc<Mutex<Vec<Value>>>,
    stop: Arc<AtomicBool>,
    release: Arc<AtomicBool>,
    graph: Option<std::thread::JoinHandle<()>>,
    rcpd: Rcpd,
    rcpd_addr: SocketAddr,
    stop_rcpd: Option<tokio::sync::oneshot::Sender<()>>,
}

impl RcpdSide {
    /// Start the graph and rcpd, their files under `dir`.
    async fn start(dir: &Path) -> Self {
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
  - name: arm.hold
    description: never answered, held up to 60 s
    safety_class: reversible
    timeout_ms: 60000
    input_schema: {{ type: object, properties: {{}} }}
"
        );
        let manifest_path = dir.join("rcp.yaml");
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
        let release = Arc::new(AtomicBool::new(false));
        let graph = spawn_graph(
            mgr.clone(),
            prefix.clone(),
            stop.clone(),
            seen.clone(),
            release.clone(),
        );

        // The real rcpd: tool host, journal, bearer HTTP on an ephemeral port.
        let receipts = dir.join("rcp/receipts.jsonl");
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
        let rcpd = Rcpd {
            host,
            feed: GraphSource::new(mgr.clone(), loaded.identity.clone()).feed(),
            state: loaded.state.clone(),
        };
        let (rcpd_addr, stop_rcpd) = rcpd
            .serve("127.0.0.1:0".parse().unwrap())
            .await
            .expect("rcpd binds");
        RcpdSide {
            mgr,
            receipts,
            seen,
            stop,
            release,
            graph: Some(graph),
            rcpd,
            rcpd_addr,
            stop_rcpd: Some(stop_rcpd),
        }
    }

    /// Stop rcpd: it stops accepting and ends every response stream it has
    /// open.
    fn stop_rcpd(&mut self) {
        let _ = self.stop_rcpd.take().expect("rcpd is serving").send(());
    }

    /// Serve a fresh rcpd (a new session table, the same tool host) on the
    /// address and token of the stopped one.
    async fn restart_rcpd(&mut self) {
        let addr = self.rcpd_addr;
        let rcpd = &self.rcpd;
        let stop = bounded("rcpd binds its address again", async {
            loop {
                match rcpd.serve(addr).await {
                    Ok((_, stop)) => break stop,
                    Err(_) => tokio::time::sleep(Duration::from_millis(20)).await,
                }
            }
        })
        .await;
        self.stop_rcpd = Some(stop);
    }

    /// Let the stand-in graph answer every `arm.hold` call, held or later.
    fn release_held_calls(&self) {
        self.release.store(true, Ordering::Relaxed);
    }

    /// Wait until the stand-in graph has seen a call of `tool`.
    async fn graph_sees(&self, tool: &str) {
        bounded("the graph sees the call", async {
            while !self
                .seen
                .lock()
                .unwrap()
                .iter()
                .any(|c| c["tool"] == json!(tool))
            {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await;
    }

    fn receipt_callers(&self) -> Vec<(String, String)> {
        cerud::receipt::read_all(&self.receipts)
            .unwrap_or_default()
            .into_iter()
            .map(|r| (r.verb, r.caller))
            .collect()
    }
}

impl Drop for RcpdSide {
    fn drop(&mut self) {
        if let Some(tx) = self.stop_rcpd.take() {
            let _ = tx.send(());
        }
        self.stop.store(true, Ordering::Relaxed);
        if let Some(g) = self.graph.take() {
            let _ = g.join();
        }
    }
}

/// The fixture token in `dir/rcp.token`, private to its owner.
fn write_token_file(dir: &Path) -> std::path::PathBuf {
    let token_file = dir.join("rcp.token");
    std::fs::write(&token_file, format!("CERULION_RCP_TOKEN={TOKEN}\n")).unwrap();
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(&token_file, std::fs::Permissions::from_mode(0o600)).unwrap();
    }
    token_file
}

/// A robot serving the rcp plane alone, through `serve_planes`.
struct ServedPlane {
    robot: Endpoint,
    trust: SharedTrust,
    lease: EstopLease,
    plane: Arc<RcpPlane>,
    stop_serving: tokio::sync::oneshot::Sender<()>,
}

/// The robot: one endpoint, the rcp plane over the live trust state under
/// `dir`, forwarding to the rcpd at `rcpd_addr` with the fixture token, its
/// limits adjusted by `adjust`.
async fn serve_rcp_plane(
    dir: &Path,
    rcpd_addr: SocketAddr,
    adjust: impl FnOnce(&mut RcpLimits),
) -> ServedPlane {
    let token_file = write_token_file(dir);
    let robot = endpoint(ROBOT_SEED).await;
    let trust = robot_trust(dir, *robot.id().as_bytes());
    let classify = Arc::new(PairingAuthorizer::from_shared(trust.clone()));
    let mut link = RcpLink::under(dir);
    link.addr = rcpd_addr;
    link.token_file = token_file;
    link.limits.revoke_check = QUIET_REVOKE_CHECK;
    adjust(&mut link.limits);
    let lease = EstopLease::new(Arc::new(Mutex::new(ControlLease::new(1_000_000_000))));
    let plane = Arc::new(RcpPlane::new(link, trust.clone(), lease.clone()));
    let (stop_serving, serve_rx) = tokio::sync::oneshot::channel::<()>();
    let serving_endpoint = robot.clone();
    let serving_plane = plane.clone();
    tokio::spawn(async move {
        let _ = cerulion_remoted::serve_planes(
            &serving_endpoint,
            classify,
            None,
            None,
            None,
            Some(serving_plane),
            async {
                let _ = serve_rx.await;
            },
        )
        .await;
    });
    ServedPlane {
        robot,
        trust,
        lease,
        plane,
        stop_serving,
    }
}

/// One rcp stream from the desk `seed` to `robot`.
async fn open_session(robot: &Endpoint, seed: u8) -> Session {
    let desk = endpoint(seed).await;
    let addr = loopback_addr(robot.id(), &robot.bound_sockets());
    open_session_from(desk, addr).await
}

/// One rcp stream from `desk` to the robot at `addr`.
async fn open_session_from(desk: Endpoint, addr: EndpointAddr) -> Session {
    let conn = bounded("dial", dial(&desk, addr, alpn::RCP))
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

    /// Read frames until each of `ids` is answered, in any order; the
    /// replies in the order of `ids`.
    async fn replies(&mut self, ids: &[Value]) -> Vec<Value> {
        let mut got: Vec<Option<Value>> = vec![None; ids.len()];
        while got.iter().any(Option::is_none) {
            let frame = bounded("read_frame", read_frame(&mut self.recv, 16 << 20))
                .await
                .expect("a reply frame");
            let v: Value = serde_json::from_slice(&frame).expect("JSON reply");
            if let Some(at) = ids.iter().position(|id| v.get("id") == Some(id)) {
                got[at] = Some(v);
            }
        }
        got.into_iter().flatten().collect()
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
    assert_eq!(
        names,
        [
            "arm.step",
            "arm.slow",
            "arm.mute",
            "arm.hold",
            "estop",
            "estop_clear"
        ]
    );
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
    assert!(denied(&r, -32001), "viewer tools/call: {r}");
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
    assert!(denied(&r, -32001), "viewer arm.step: {r}");
    // Clearing is not under the e-stop floor: it is a tool like any other.
    let r = s
        .request(
            9,
            "tools/call",
            json!({ "name": cerulion_rcpd::tools::ESTOP_CLEAR_TOOL, "arguments": {} }),
        )
        .await;
    assert!(denied(&r, -32001), "viewer estop_clear: {r}");
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
        assert!(denied(&r, -32600), "{r}");
    }
    s.send_raw(br#"{"jsonrpc":"2.0","id":8,"method":"tools/call","params":{"arguments":{}}}"#)
        .await;
    assert!(denied(&s.reply(&json!(8)).await, -32600));
    s.send_raw(br#"{"jsonrpc":"2.0","id":9,"method":"prompts/list","params":{}}"#)
        .await;
    assert!(denied(&s.reply(&json!(9)).await, -32600));
    let again = s
        .request(
            11,
            "initialize",
            json!({ "protocolVersion": "2025-06-18", "capabilities": {},
                    "clientInfo": { "name": "again", "version": "0" } }),
        )
        .await;
    assert!(denied(&again, -32600), "one initialize per stream: {again}");
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
    assert_eq!(
        cerulion_remoted::rcp::ESTOP_CLEAR_TOOL,
        cerulion_rcpd::tools::ESTOP_CLEAR_TOOL
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
        json!(-32603),
        "a lifecycle body takes a slot too: {busy}"
    );
    let over = s.call_step(2).await;
    assert_eq!(over["error"]["code"], json!(-32603), "{over}");
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
        json!(-32001),
        "the result was withheld: {r}"
    );
    // The session itself stays: the account is still on the access list.
    let pong = s.request(3, "ping", json!({})).await;
    assert!(pong.get("result").is_some(), "{pong}");
}

fn revoke_operator_device(f: &Fixture) {
    revoke_operator_device_in(&f.trust);
}

/// Revoke the operator's DEVICE by a signed epoch on the live trust state
/// `trust`.
fn revoke_operator_device_in(trust: &SharedTrust) {
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
    trust
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
    assert_eq!(over["error"]["code"], json!(-32603), "{over}");
    assert!(
        started.elapsed() < Duration::from_secs(1),
        "answered at once"
    );
}

#[tokio::test(flavor = "multi_thread")]
async fn an_rcpd_restart_answers_the_open_call_then_ends_the_stream() {
    let mut f = Fixture::new().await;
    let mut s = f.session(OPERATOR.0).await;
    s.initialize().await;
    let first = s.call_step(2).await;
    assert!(first.get("result").is_some(), "{first}");
    let held = json!({ "jsonrpc": "2.0", "id": 3, "method": "tools/call",
                       "params": { "name": "arm.hold", "arguments": {} } });
    s.send_raw(held.to_string().as_bytes()).await;
    f.graph_sees("arm.hold").await;
    // rcpd stops with the call in flight: its answer ends without a response.
    f.stop_rcpd();
    let r = s.reply(&json!(3)).await;
    assert_eq!(r["error"]["code"], json!(-32603), "{r}");
    assert_eq!(
        r["error"]["message"],
        json!("rcpd ended its answer without a response"),
        "{r}"
    );
    // A fresh rcpd on the same address and token does not know the session:
    // the next request, the e-stop included, is answered. Later requests get
    // the same error, and the stream ends once the desk ends its side.
    f.restart_rcpd().await;
    let r = s
        .request(4, "tools/call", json!({ "name": "estop", "arguments": {} }))
        .await;
    assert_eq!(r["error"]["code"], json!(-32603), "{r}");
    assert_eq!(
        r["error"]["message"],
        json!("rcpd lost this MCP session (it restarted); restart this MCP server"),
        "{r}"
    );
    let r = s.request(5, "ping", json!({})).await;
    assert_eq!(r["error"]["code"], json!(-32603), "{r}");
    assert_eq!(
        r["error"]["message"],
        json!("rcpd lost this MCP session (it restarted); restart this MCP server"),
        "{r}"
    );
    s.send.finish().expect("finish");
    let next = bounded("the stream ends", read_frame(&mut s.recv, 1 << 20)).await;
    assert!(next.is_err(), "the robot ended the stream: {next:?}");
}

#[tokio::test(flavor = "multi_thread")]
async fn a_cancel_reaches_rcpd_while_every_request_slot_is_taken() {
    let f = Fixture::with_limits(|l| l.in_flight = 1).await;
    let mut s = f.session(OPERATOR.0).await;
    s.initialize().await;
    // The one slot: a call the graph holds for up to 60 s.
    let held = json!({ "jsonrpc": "2.0", "id": 2, "method": "tools/call",
                       "params": { "name": "arm.hold", "arguments": {} } });
    s.send_raw(held.to_string().as_bytes()).await;
    f.graph_sees("arm.hold").await;
    let cancel = json!({ "jsonrpc": "2.0", "method": "notifications/cancelled",
                         "params": { "requestId": 2, "reason": "rcp e2e" } });
    s.send_raw(cancel.to_string().as_bytes()).await;
    // rcpd ends the cancelled call's response stream, which frees the slot.
    let r = s.reply(&json!(2)).await;
    assert_eq!(
        r["error"]["message"],
        json!("rcpd ended its answer without a response"),
        "{r}"
    );
    let pong = bounded("a request is admitted again", async {
        for id in 3.. {
            let r = s.request(id, "ping", json!({})).await;
            if r.get("result").is_some() {
                return r;
            }
            assert_eq!(
                r["error"]["message"],
                json!("refused: too many requests in flight on this rcp stream"),
                "{r}"
            );
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        unreachable!()
    })
    .await;
    assert!(pong.get("result").is_some(), "{pong}");
}

#[tokio::test(flavor = "multi_thread")]
async fn an_engaged_ops_estop_refuses_rcp_tool_calls_but_not_the_estop() {
    let f = Fixture::new().await;
    let mut s = f.session(OPERATOR.0).await;
    s.initialize().await;
    // The first step of the ops plane's engage-estop: it engages this lease.
    // Its estop into rcpd is covered by the ops e-stop arms below.
    f.lease.engage("iroh:ops-desk");
    assert_eq!(
        f.lease.lock().unwrap().estop(),
        &EstopState::Engaged {
            by: "iroh:ops-desk".into()
        }
    );
    let r = s.call_step(2).await;
    assert_eq!(r["error"]["code"], json!(-32001), "{r}");
    assert_eq!(
        r["error"]["message"],
        json!("refused: the robot's e-stop is engaged"),
        "{r}"
    );
    let r = s
        .request(
            3,
            "tools/call",
            json!({ "name": "estop_clear", "arguments": {} }),
        )
        .await;
    assert_eq!(r["error"]["code"], json!(-32001), "{r}");
    assert_eq!(
        r["error"]["message"],
        json!(
            "refused: the robot's ops e-stop is engaged; it clears when cerulion-remoted restarts"
        ),
        "{r}"
    );
    // Reading and the e-stop still pass.
    let listed = s.request(4, "tools/list", json!({})).await;
    assert!(listed["result"]["tools"].is_array(), "{listed}");
    let r = s
        .request(5, "tools/call", json!({ "name": "estop", "arguments": {} }))
        .await;
    let text = r["result"]["content"][0]["text"]
        .as_str()
        .expect("estop body");
    assert_eq!(
        serde_json::from_str::<Value>(text).unwrap()["estopped"],
        json!(true),
        "{r}"
    );
    let caller = expected_caller(OPERATOR.0, OPERATOR.1);
    assert_eq!(
        f.receipt_callers(),
        [
            ("estop".to_string(), caller.clone()),
            ("estop".to_string(), caller)
        ],
        "only the estop reached rcpd's tools"
    );
    let seen = f.seen.lock().unwrap().clone();
    assert_eq!(seen.len(), 1, "only the estop reached the graph: {seen:?}");
    assert_eq!(seen[0]["tool"], json!("estop"));
    assert_eq!(
        f.lease.lock().unwrap().estop(),
        &EstopState::Engaged {
            by: "iroh:ops-desk".into()
        },
        "the first engager is kept"
    );
}

/// `topic-publish` on `lease` with a type no robot knows: past the e-stop
/// check it fails on the type lookup, before any shared memory is opened.
fn publish_unknown_type(lease: &EstopLease) -> String {
    let args = json!({ "topic": "/rcp_e2e/latch", "type": "nope_msgs/msg/Nope", "message": {} });
    match TopicPublishVerb::new(lease.clone()).execute(&args) {
        Err(CerudError::Verb(m)) => m,
        other => panic!("expected a verb error, got {other:?}"),
    }
}

#[tokio::test(flavor = "multi_thread")]
async fn an_rcp_estop_engages_the_latch_the_ops_act_verbs_obey() {
    let f = Fixture::new().await;
    let before = publish_unknown_type(&f.lease);
    assert!(
        before.starts_with("usage: unknown message type"),
        "clear: {before}"
    );
    let mut viewer = f.session(VIEWER.0).await;
    viewer.initialize().await;
    let r = viewer
        .request(2, "tools/call", json!({ "name": "estop", "arguments": {} }))
        .await;
    let text = r["result"]["content"][0]["text"]
        .as_str()
        .expect("estop body");
    assert_eq!(
        serde_json::from_str::<Value>(text).unwrap()["estopped"],
        json!(true),
        "{r}"
    );
    // The answer came back, so the plane engaged the latch before forwarding.
    assert_eq!(
        f.lease.lock().unwrap().estop(),
        &EstopState::Engaged {
            by: expected_caller(VIEWER.0, VIEWER.1)
        }
    );
    assert_eq!(
        publish_unknown_type(&f.lease),
        "unavailable: the robot's e-stop is engaged, so topic-publish is refused"
    );
    let mut op = f.session(OPERATOR.0).await;
    op.initialize().await;
    let r = op.call_step(3).await;
    assert_eq!(
        r["error"]["message"],
        json!("refused: the robot's e-stop is engaged"),
        "{r}"
    );
}

/// A stand-in rcpd on loopback, one HTTP/1 request per connection as the
/// plane sends them. It answers `initialize` (200, a session id and a JSON
/// result) and every notification (202), refuses the standalone `GET` (405)
/// after reporting it on `gets`, and answers a `tools/call` with a JSON result
/// after reporting its `params` on `tool_calls`. It holds `ping` id 2 without
/// writing a byte, after reporting its id on `pinged`, until `release` is
/// notified, then answers it; every other ping is answered at once. A
/// `tools/call` of `lost.step` is answered 404, as rcpd answers a session it
/// no longer knows. Any other request is answered 404. The method of every
/// POST goes on `posts` in the order the stand-in reads them.
struct StandIn {
    addr: SocketAddr,
    pinged: tokio::sync::mpsc::UnboundedReceiver<Value>,
    tool_calls: tokio::sync::mpsc::UnboundedReceiver<Value>,
    gets: tokio::sync::mpsc::UnboundedReceiver<()>,
    posts: tokio::sync::mpsc::UnboundedReceiver<String>,
    release: Arc<tokio::sync::Notify>,
}

async fn stand_in_rcpd() -> StandIn {
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let (pinged_tx, pinged) = tokio::sync::mpsc::unbounded_channel();
    let (calls_tx, tool_calls) = tokio::sync::mpsc::unbounded_channel();
    let (gets_tx, gets) = tokio::sync::mpsc::unbounded_channel();
    let (posts_tx, posts) = tokio::sync::mpsc::unbounded_channel();
    let release = Arc::new(tokio::sync::Notify::new());
    let held = release.clone();
    let json_reply = |result: Value| {
        let body = result.to_string();
        format!(
            "HTTP/1.1 200 OK\r\ncontent-type: application/json\r\n\
             mcp-session-id: stand-in\r\ncontent-length: {}\r\n\r\n{body}",
            body.len()
        )
    };
    tokio::spawn(async move {
        loop {
            let Ok((mut tcp, _)) = listener.accept().await else {
                return;
            };
            let (pinged, calls, held) = (pinged_tx.clone(), calls_tx.clone(), held.clone());
            let (gets, posts) = (gets_tx.clone(), posts_tx.clone());
            tokio::spawn(async move {
                let mut buf = Vec::new();
                let mut chunk = [0u8; 4096];
                let head_end = loop {
                    if let Some(at) = buf.windows(4).position(|w| w == b"\r\n\r\n") {
                        break at + 4;
                    }
                    match tcp.read(&mut chunk).await {
                        Ok(0) | Err(_) => return,
                        Ok(n) => buf.extend_from_slice(&chunk[..n]),
                    }
                };
                let head = String::from_utf8_lossy(&buf[..head_end]).to_ascii_lowercase();
                let length: usize = head
                    .lines()
                    .find_map(|l| l.strip_prefix("content-length:"))
                    .map_or(0, |v| v.trim().parse().unwrap());
                while buf.len() < head_end + length {
                    match tcp.read(&mut chunk).await {
                        Ok(0) | Err(_) => return,
                        Ok(n) => buf.extend_from_slice(&chunk[..n]),
                    }
                }
                let body: Value = serde_json::from_slice(&buf[head_end..head_end + length])
                    .unwrap_or(Value::Null);
                let method = body["method"].as_str().unwrap_or("");
                let verb = head.split(' ').next();
                if verb == Some("post") {
                    let _ = posts.send(method.to_string());
                }
                let reply = match verb {
                    Some("post") if method == "initialize" => json_reply(json!({
                        "jsonrpc": "2.0", "id": body["id"], "result": {
                            "protocolVersion": "2025-06-18",
                            "capabilities": {},
                            "serverInfo": { "name": "stand-in", "version": "0" }
                        }
                    })),
                    Some("post") if method.starts_with("notifications/") => {
                        "HTTP/1.1 202 Accepted\r\ncontent-length: 0\r\n\r\n".to_string()
                    }
                    Some("post") if method == "ping" => {
                        if body["id"] == json!(2) {
                            let _ = pinged.send(body["id"].clone());
                            held.notified().await;
                        }
                        json_reply(json!({ "jsonrpc": "2.0", "id": body["id"], "result": {} }))
                    }
                    Some("post")
                        if method == "tools/call" && body["params"]["name"] == "lost.step" =>
                    {
                        "HTTP/1.1 404 Not Found\r\ncontent-length: 0\r\n\r\n".to_string()
                    }
                    Some("post") if method == "tools/call" => {
                        let _ = calls.send(body["params"].clone());
                        json_reply(json!({ "jsonrpc": "2.0", "id": body["id"], "result": {
                            "content": [{ "type": "text", "text": "{\"stand_in\":true}" }]
                        } }))
                    }
                    Some("get") => {
                        let _ = gets.send(());
                        "HTTP/1.1 405 Method Not Allowed\r\ncontent-length: 0\r\n\r\n".to_string()
                    }
                    _ => "HTTP/1.1 404 Not Found\r\ncontent-length: 0\r\n\r\n".to_string(),
                };
                let _ = tcp.write_all(reply.as_bytes()).await;
                let _ = tcp.shutdown().await;
            });
        }
    });
    StandIn {
        addr,
        pinged,
        tool_calls,
        gets,
        posts,
        release,
    }
}

/// The plane over `stand_in` with a head timeout past every bound here (no
/// answer to a held ping can come from the plane itself), an operator stream
/// initialized, a `ping` (id 2) that the stand-in holds, `queued` sent after
/// it, and a second `initialize` (id 5) refused: the reader handles frames in
/// order, so `queued` was admitted and waits behind the ping. The ping is sent
/// once the stand-in has seen the standalone `GET`, which the forwarder opens
/// only after it is done with `initialize`, so the ping finds that request's
/// slot free. Returns the frames that came before that refusal too.
async fn queue_behind_a_held_ping(
    dir: &Path,
    stand_in: &mut StandIn,
    adjust: impl FnOnce(&mut RcpLimits),
    queued: &[Value],
) -> (ServedPlane, Session, Vec<Value>) {
    let served = serve_rcp_plane(dir, stand_in.addr, |l| {
        l.head_timeout = Duration::from_secs(3600);
        adjust(l);
    })
    .await;
    let mut s = open_session(&served.robot, OPERATOR.0).await;
    s.initialize().await;
    bounded(
        "the forwarder opens the standalone stream",
        stand_in.gets.recv(),
    )
    .await
    .expect("the stand-in saw the standalone GET");
    s.send_raw(
        json!({ "jsonrpc": "2.0", "id": 2, "method": "ping" })
            .to_string()
            .as_bytes(),
    )
    .await;
    let held = bounded("rcpd holds the ping", stand_in.pinged.recv())
        .await
        .expect("the stand-in saw the ping");
    assert_eq!(held, json!(2));
    for message in queued {
        s.send_raw(message.to_string().as_bytes()).await;
    }
    let again = json!({ "jsonrpc": "2.0", "id": 5, "method": "initialize", "params": {
        "protocolVersion": "2025-06-18",
        "capabilities": {},
        "clientInfo": { "name": "rcp-e2e", "version": "0" }
    } });
    s.send_raw(again.to_string().as_bytes()).await;
    let mut early = Vec::new();
    loop {
        let frame = bounded("read_frame", read_frame(&mut s.recv, 16 << 20))
            .await
            .expect("a reply frame");
        let v: Value = serde_json::from_slice(&frame).expect("JSON reply");
        if v.get("id") == Some(&json!(5)) {
            assert_eq!(
                v["error"]["message"],
                json!("refused: one initialize per rcp stream"),
                "{v}"
            );
            break;
        }
        early.push(v);
    }
    (served, s, early)
}

/// The reply to `id`: from `early` if it came before, else read from `s`.
async fn reply_after(s: &mut Session, early: &[Value], id: i64) -> Value {
    match early.iter().find(|v| v.get("id") == Some(&json!(id))) {
        Some(v) => v.clone(),
        None => s.reply(&json!(id)).await,
    }
}

fn step_call(id: i64) -> Value {
    json!({ "jsonrpc": "2.0", "id": id, "method": "tools/call",
            "params": { "name": "arm.step", "arguments": { "q": [0.1, -0.2] } } })
}

#[tokio::test(flavor = "multi_thread")]
async fn an_rcp_estop_engages_the_ops_latch_while_rcpd_holds_a_ping() {
    let dir = tempfile::tempdir().unwrap();
    let mut stand_in = stand_in_rcpd().await;
    // No answer to the held ping can come from the plane while this test
    // runs: its head timeout is far past every bound here.
    let served = serve_rcp_plane(dir.path(), stand_in.addr, |l| {
        l.head_timeout = Duration::from_secs(3600);
    })
    .await;
    let mut s = open_session(&served.robot, OPERATOR.0).await;
    s.initialize().await;
    s.send_raw(
        json!({ "jsonrpc": "2.0", "id": 2, "method": "ping" })
            .to_string()
            .as_bytes(),
    )
    .await;
    let held = bounded("rcpd holds the ping", stand_in.pinged.recv())
        .await
        .expect("the stand-in saw the ping");
    assert_eq!(held, json!(2));

    // An estop on the same stream while rcpd holds the ping.
    let estop = json!({ "jsonrpc": "2.0", "id": 3, "method": "tools/call",
                        "params": { "name": "estop", "arguments": {} } });
    s.send_raw(estop.to_string().as_bytes()).await;
    bounded("the rcp estop engages the ops latch", async {
        while matches!(served.lease.lock().unwrap().estop(), EstopState::Clear) {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    })
    .await;
    assert_eq!(
        served.lease.lock().unwrap().estop(),
        &EstopState::Engaged {
            by: expected_caller(OPERATOR.0, OPERATOR.1)
        }
    );

    // The plane itself answers an Act call now; no answer to the held ping
    // reaches the desk before that refusal.
    s.send_raw(step_call(4).to_string().as_bytes()).await;
    loop {
        let frame = bounded("read_frame", read_frame(&mut s.recv, 16 << 20))
            .await
            .expect("a reply frame");
        let v: Value = serde_json::from_slice(&frame).expect("JSON reply");
        assert_ne!(
            v.get("id"),
            Some(&json!(2)),
            "the held ping was answered: {v}"
        );
        if v.get("id") == Some(&json!(4)) {
            assert_eq!(v["error"]["code"], json!(-32001), "{v}");
            assert_eq!(
                v["error"]["message"],
                json!("refused: the robot's e-stop is engaged"),
                "{v}"
            );
            break;
        }
    }
    let _ = served.stop_serving.send(());
}

#[tokio::test(flavor = "multi_thread")]
async fn an_act_call_queued_behind_a_held_ping_is_refused_after_an_ops_engage() {
    let dir = tempfile::tempdir().unwrap();
    let mut stand_in = stand_in_rcpd().await;
    let (served, mut s, early) =
        queue_behind_a_held_ping(dir.path(), &mut stand_in, |_| {}, &[step_call(4)]).await;
    // The first step of the ops plane's engage-estop, while the call waits.
    served.lease.engage("iroh:ops-desk");
    stand_in.release.notify_one();
    let r = reply_after(&mut s, &early, 4).await;
    assert_eq!(r["error"]["code"], json!(-32001), "{r}");
    assert_eq!(
        r["error"]["message"],
        json!("refused: the robot's e-stop is engaged"),
        "{r}"
    );
    assert!(
        stand_in.tool_calls.try_recv().is_err(),
        "the queued call reached rcpd after the engage"
    );
    let _ = served.stop_serving.send(());
}

#[tokio::test(flavor = "multi_thread")]
async fn a_queued_call_is_refused_when_its_scope_loses_the_class_before_it_is_sent() {
    let dir = tempfile::tempdir().unwrap();
    let mut stand_in = stand_in_rcpd().await;
    let (served, mut s, early) =
        queue_behind_a_held_ping(dir.path(), &mut stand_in, |_| {}, &[step_call(4)]).await;
    // Re-establish the operator's account without CAP_TELEOP while the call
    // waits; the account stays on the access list.
    let pres = presentation(
        OPERATOR.0,
        OPERATOR.1,
        Scope {
            role: Role::OPERATOR,
            caps: Scope::CAP_OBSERVE,
        },
    );
    served
        .trust
        .verify_and_establish(
            &pres,
            &desk_key(OPERATOR.0),
            Some("downgraded".into()),
            T_NOW,
        )
        .expect("re-established with less scope");
    stand_in.release.notify_one();
    let r = reply_after(&mut s, &early, 4).await;
    assert_eq!(r["error"]["code"], json!(-32001), "{r}");
    let text = r["error"]["message"].as_str().unwrap_or_default();
    assert!(
        text.starts_with("denied: tools/call needs more access: "),
        "{r}"
    );
    assert!(
        stand_in.tool_calls.try_recv().is_err(),
        "the queued call reached rcpd after its scope lost the class"
    );
    let _ = served.stop_serving.send(());
}

#[tokio::test(flavor = "multi_thread")]
async fn notifications_past_their_room_never_take_a_queued_estops_place() {
    let dir = tempfile::tempdir().unwrap();
    let mut stand_in = stand_in_rcpd().await;
    // One request slot (the held ping) and one e-stop slot: the forward queue
    // holds 1 + 1 + 4 messages. Six cancels then the estop.
    let cancel = json!({ "jsonrpc": "2.0", "method": "notifications/cancelled",
                         "params": { "requestId": 2, "reason": "rcp e2e" } });
    let estop = json!({ "jsonrpc": "2.0", "id": 7, "method": "tools/call",
                        "params": { "name": "estop", "arguments": {} } });
    let mut queued = vec![cancel; 6];
    queued.push(estop);
    let (served, mut s, early) = queue_behind_a_held_ping(
        dir.path(),
        &mut stand_in,
        |l| {
            l.in_flight = 1;
            l.estops_in_flight = 1;
        },
        &queued,
    )
    .await;
    stand_in.release.notify_one();
    let r = reply_after(&mut s, &early, 7).await;
    assert!(
        r.get("result").is_some(),
        "the estop was not forwarded: {r}"
    );
    let call = bounded("rcpd sees the estop", stand_in.tool_calls.recv())
        .await
        .expect("a tools/call");
    assert_eq!(call["name"], json!("estop"), "{call}");
    let _ = served.stop_serving.send(());
}

/// The plane over `stand_in` with a head timeout past every bound here, an
/// operator stream initialized, and the call `step_call(4)` sent and held by
/// the plane's test seam once its connection to rcpd is open, before the
/// checks that come just before its send.
async fn hold_a_call_at_its_connect(
    dir: &Path,
    stand_in: &StandIn,
) -> (ServedPlane, Session, Arc<ConnectHold>) {
    let served = serve_rcp_plane(dir, stand_in.addr, |l| {
        l.head_timeout = Duration::from_secs(3600);
    })
    .await;
    let hold = served.plane.hold_after_connect_for_test(json!(4));
    let mut s = open_session(&served.robot, OPERATOR.0).await;
    s.initialize().await;
    s.send_raw(step_call(4).to_string().as_bytes()).await;
    bounded("the call's connection to rcpd opens", hold.connected()).await;
    (served, s, hold)
}

#[tokio::test(flavor = "multi_thread")]
async fn an_ops_engage_while_a_calls_connection_opens_refuses_the_call() {
    let dir = tempfile::tempdir().unwrap();
    let mut stand_in = stand_in_rcpd().await;
    let (served, mut s, hold) = hold_a_call_at_its_connect(dir.path(), &stand_in).await;
    // The first step of the ops plane's engage-estop, after the plane admitted
    // the call and while its connection to rcpd is open.
    served.lease.engage("iroh:ops-desk");
    hold.release();
    let r = s.reply(&json!(4)).await;
    assert_eq!(r["error"]["code"], json!(-32001), "{r}");
    assert_eq!(
        r["error"]["message"],
        json!("refused: the robot's e-stop is engaged"),
        "{r}"
    );
    assert!(
        stand_in.tool_calls.try_recv().is_err(),
        "the call reached rcpd after the engage"
    );
    let _ = served.stop_serving.send(());
}

#[tokio::test(flavor = "multi_thread")]
async fn a_call_is_refused_when_its_scope_loses_the_class_while_its_connection_opens() {
    let dir = tempfile::tempdir().unwrap();
    let mut stand_in = stand_in_rcpd().await;
    let (served, mut s, hold) = hold_a_call_at_its_connect(dir.path(), &stand_in).await;
    // Re-establish the operator's account without CAP_TELEOP; the account
    // stays on the access list.
    let pres = presentation(
        OPERATOR.0,
        OPERATOR.1,
        Scope {
            role: Role::OPERATOR,
            caps: Scope::CAP_OBSERVE,
        },
    );
    served
        .trust
        .verify_and_establish(
            &pres,
            &desk_key(OPERATOR.0),
            Some("downgraded".into()),
            T_NOW,
        )
        .expect("re-established with less scope");
    hold.release();
    let r = s.reply(&json!(4)).await;
    assert_eq!(r["error"]["code"], json!(-32001), "{r}");
    let text = r["error"]["message"].as_str().unwrap_or_default();
    assert!(
        text.starts_with("denied: tools/call needs more access: "),
        "{r}"
    );
    assert!(
        stand_in.tool_calls.try_recv().is_err(),
        "the call reached rcpd after its scope lost the class"
    );
    let _ = served.stop_serving.send(());
}

#[tokio::test(flavor = "multi_thread")]
async fn a_cancel_reaches_rcpd_after_the_call_it_names_while_that_call_connects() {
    let dir = tempfile::tempdir().unwrap();
    let mut stand_in = stand_in_rcpd().await;
    let (served, mut s, hold) = hold_a_call_at_its_connect(dir.path(), &stand_in).await;
    let cancel = json!({ "jsonrpc": "2.0", "method": "notifications/cancelled",
                         "params": { "requestId": 4, "reason": "rcp e2e" } });
    s.send_raw(cancel.to_string().as_bytes()).await;
    // A ping the stand-in answers at once. The forwarder sends it inline after
    // the cancel, so its answer means the cancel is off the forward queue.
    let pong = s.request(6, "ping", json!({})).await;
    assert!(pong.get("result").is_some(), "{pong}");
    hold.release();
    let r = s.reply(&json!(4)).await;
    assert!(r.get("result").is_some(), "{r}");
    let order = bounded("rcpd sees the cancel", async {
        let mut order = Vec::new();
        while let Some(method) = stand_in.posts.recv().await {
            if method == "tools/call" || method == "notifications/cancelled" {
                order.push(method.clone());
            }
            if method == "notifications/cancelled" {
                return order;
            }
        }
        panic!("the stand-in stopped before it saw the cancel")
    })
    .await;
    assert_eq!(order, ["tools/call", "notifications/cancelled"]);
    let _ = served.stop_serving.send(());
}

#[tokio::test(flavor = "multi_thread")]
async fn a_key_revoked_after_accept_cannot_engage_the_estop() {
    let dir = tempfile::tempdir().unwrap();
    let mut stand_in = stand_in_rcpd().await;
    // Only a frame back checks the session floor in this test: the periodic
    // check's next tick is past its end.
    let served = serve_rcp_plane(dir.path(), stand_in.addr, |l| {
        l.revoke_check = Duration::from_secs(3600);
    })
    .await;
    let mut s = open_session(&served.robot, OPERATOR.0).await;
    s.initialize().await;
    revoke_operator_device_in(&served.trust);
    let estop = json!({ "jsonrpc": "2.0", "id": 3, "method": "tools/call",
                        "params": { "name": "estop", "arguments": {} } });
    s.send_raw(estop.to_string().as_bytes()).await;
    // The reader denies the estop, and the writer closes the revoked desk's
    // connection on that denial: the desk sees the stream end, never a result.
    loop {
        match bounded("the connection closes", read_frame(&mut s.recv, 16 << 20)).await {
            Err(_) => break,
            Ok(frame) => {
                let v: Value = serde_json::from_slice(&frame).expect("JSON frame");
                assert!(
                    v.get("id") != Some(&json!(3)) || v.get("result").is_none(),
                    "the revoked key's estop was answered: {v}"
                );
            }
        }
    }
    assert_eq!(
        served.lease.lock().unwrap().estop(),
        &EstopState::Clear,
        "the revoked key engaged the ops latch"
    );
    assert!(
        stand_in.tool_calls.try_recv().is_err(),
        "the revoked key's estop reached rcpd"
    );
    let _ = served.stop_serving.send(());
}

#[tokio::test(flavor = "multi_thread")]
async fn an_estop_after_rcpd_loses_the_session_still_engages_the_ops_latch() {
    let dir = tempfile::tempdir().unwrap();
    let mut stand_in = stand_in_rcpd().await;
    let served = serve_rcp_plane(dir.path(), stand_in.addr, |_| {}).await;
    let mut s = open_session(&served.robot, OPERATOR.0).await;
    s.initialize().await;
    let session_lost = json!("rcpd lost this MCP session (it restarted); restart this MCP server");
    // rcpd answers this call 404: it no longer knows the session.
    let r = s
        .request(
            2,
            "tools/call",
            json!({ "name": "lost.step", "arguments": {} }),
        )
        .await;
    assert_eq!(r["error"]["code"], json!(-32603), "{r}");
    assert_eq!(r["error"]["message"], session_lost, "{r}");
    // The estop is sent once the reader has seen the loss.
    bounded(
        "the reader sees the loss",
        served.plane.session_loss_seen_for_test(),
    )
    .await;
    let r = s
        .request(3, "tools/call", json!({ "name": "estop", "arguments": {} }))
        .await;
    assert_eq!(r["error"]["code"], json!(-32603), "{r}");
    assert_eq!(
        r["error"]["message"],
        json!(
            "rcpd lost this MCP session (it restarted), so this estop did not reach it; \
             the robot's ops e-stop is engaged; restart this MCP server"
        ),
        "{r}"
    );
    assert_eq!(
        served.lease.lock().unwrap().estop(),
        &EstopState::Engaged {
            by: expected_caller(OPERATOR.0, OPERATOR.1)
        }
    );
    // Any other request gets the session-lost error, and nothing more reaches
    // rcpd.
    let r = s.request(4, "ping", json!({})).await;
    assert_eq!(r["error"]["message"], session_lost, "{r}");
    assert!(
        stand_in.tool_calls.try_recv().is_err(),
        "a tool call reached rcpd after the loss"
    );
    let _ = served.stop_serving.send(());
}

/// One ops call on its own connection from `desk` to `addr`.
async fn ops_call(
    desk: &Endpoint,
    addr: EndpointAddr,
    verb: &'static str,
    args: Value,
) -> Result<Value, CerudError> {
    let conn = bounded("dial ops", dial(desk, addr, alpn::OPS))
        .await
        .expect("dial ops");
    let (send, recv) = bounded("open ops stream", open_frame_stream(&conn))
        .await
        .expect("open ops stream");
    let result = bounded(
        "ops call",
        tokio::task::spawn_blocking(move || {
            let mut client =
                OpsClient::connect_current(QuicOpsStream::new(send, recv)).expect("handshake");
            client.call(verb, args)
        }),
    )
    .await
    .expect("ops task");
    conn.close(0u32.into(), b"done");
    result
}

#[tokio::test(flavor = "multi_thread")]
async fn the_daemon_engages_the_ops_latch_from_an_rcp_estop_with_rcpd_down() {
    let dir = tempfile::tempdir().unwrap();
    let robot = endpoint(ROBOT_SEED).await;
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());
    let (store, index) = robot_store(dir.path(), *robot.id().as_bytes());
    // No rcp.token and no rcpd: every rcp request is answered "not reachable".
    std::fs::create_dir_all(dir.path().join("remoted")).unwrap();
    let config = RemotedConfig::from_state_root(dir.path(), RelayConfig::Disabled, false);
    std::fs::create_dir_all(&config.log_root).unwrap();
    let n = COUNTER.fetch_add(1, Ordering::Relaxed);
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("rcp_e2e_daemon_{}_{n}", std::process::id()),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("isolated transport");
    let (stop, stopped) = tokio::sync::oneshot::channel::<()>();
    let serving = robot.clone();
    let daemon = tokio::spawn(async move {
        serve_endpoint(
            &config,
            store,
            index,
            MAC_KEY.to_vec(),
            &serving,
            RemotedClock::fixed(T_NOW),
            None,
            Some(manager),
            async {
                let _ = stopped.await;
            },
        )
        .await
    });
    let desk = endpoint(OPERATOR.0).await;
    let args = json!({ "topic": "/rcp_e2e/latch", "type": "nope_msgs/msg/Nope", "message": {} });
    let remote_message = |r: Result<Value, CerudError>| match r {
        Err(CerudError::Remote { kind, message }) if kind == "verb_error" => message,
        other => panic!("expected the verb's error, got {other:?}"),
    };
    let before =
        remote_message(ops_call(&desk, robot_addr.clone(), "topic-publish", args.clone()).await);
    assert!(
        before.starts_with("verb error: usage: unknown message type"),
        "clear: {before}"
    );
    // `restart` with no `graph`: while the latch is clear its gate lets it
    // read the args, so it answers the args error and runs nothing.
    let restart_before =
        remote_message(ops_call(&desk, robot_addr.clone(), "restart", json!({})).await);
    assert_eq!(
        restart_before,
        "verb error: invalid restart args: missing field `graph`"
    );

    let conn = bounded("dial rcp", dial(&desk, robot_addr.clone(), alpn::RCP))
        .await
        .expect("dial rcp");
    let (mut send, mut recv) = bounded("open rcp stream", open_frame_stream(&conn))
        .await
        .expect("open rcp stream");
    let estop = json!({ "jsonrpc": "2.0", "id": 1, "method": "tools/call",
                        "params": { "name": "estop", "arguments": {} } });
    bounded(
        "estop",
        write_frame(&mut send, estop.to_string().as_bytes()),
    )
    .await
    .expect("write");
    let frame = bounded("estop answer", read_frame(&mut recv, 1 << 20))
        .await
        .expect("an answer");
    let r: Value = serde_json::from_slice(&frame).unwrap();
    assert_eq!(r["id"], json!(1), "{r}");
    assert_eq!(r["error"]["code"], json!(-32603), "{r}");
    assert_eq!(
        r["error"]["message"],
        json!("rcpd is not reachable on this robot"),
        "{r}"
    );

    let after = remote_message(ops_call(&desk, robot_addr.clone(), "topic-publish", args).await);
    assert_eq!(
        after,
        "verb error: unavailable: the robot's e-stop is engaged, so topic-publish is refused"
    );
    let restart_after = remote_message(ops_call(&desk, robot_addr, "restart", json!({})).await);
    assert_eq!(
        restart_after,
        "verb error: unavailable: the robot's e-stop is engaged, so restart is refused"
    );
    let _ = stop.send(());
    let served = bounded("daemon stops", daemon).await.expect("daemon task");
    assert!(served.is_ok(), "{served:?}");
}

/// What a [`Proxy`] does with each request it reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Gate {
    /// Forward it to rcpd.
    Open,
    /// Hold it until the gate opens.
    Shut,
}

/// A loopback proxy in front of rcpd. The robot sends one HTTP/1 request per
/// connection; for each one the proxy reads the request whole and reports its
/// text on `requests`, holds it while the gate is shut, then forwards it and
/// reports its text on `answered` once the first bytes of rcpd's answer come
/// back, and relays the rest both ways. When rcpd refuses the connection, it
/// reports the request on `refused` and closes the robot's connection.
struct Proxy {
    addr: SocketAddr,
    requests: tokio::sync::mpsc::UnboundedReceiver<String>,
    answered: tokio::sync::mpsc::UnboundedReceiver<String>,
    /// Each request rcpd refused the connection for, after which the proxy
    /// closed the robot's connection.
    refused: tokio::sync::mpsc::UnboundedReceiver<String>,
    gate: tokio::sync::watch::Sender<Gate>,
}

impl Proxy {
    async fn start(rcpd: SocketAddr, gate: Gate) -> Self {
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let (requests_tx, requests) = tokio::sync::mpsc::unbounded_channel();
        let (answered_tx, answered) = tokio::sync::mpsc::unbounded_channel();
        let (refused_tx, refused) = tokio::sync::mpsc::unbounded_channel();
        let (gate, gate_rx) = tokio::sync::watch::channel(gate);
        tokio::spawn(async move {
            while let Ok((client, _)) = listener.accept().await {
                let relay = relay(
                    client,
                    rcpd,
                    Reports {
                        requests: requests_tx.clone(),
                        answered: answered_tx.clone(),
                        refused: refused_tx.clone(),
                    },
                    gate_rx.clone(),
                );
                tokio::spawn(async move {
                    let _ = relay.await;
                });
            }
        });
        Proxy {
            addr,
            requests,
            answered,
            refused,
            gate,
        }
    }

    /// The text of the next request rcpd refused the connection for.
    async fn next_refused(&mut self) -> String {
        bounded("rcpd refuses a connection", self.refused.recv())
            .await
            .expect("the proxy runs")
    }

    /// Forward every held request and every later one.
    fn open(&self) {
        let _ = self.gate.send(Gate::Open);
    }

    /// The text of the next request the proxy reads.
    async fn next_request(&mut self) -> String {
        bounded("the proxy reads a request", self.requests.recv())
            .await
            .expect("the proxy runs")
    }

    /// Wait until rcpd has begun to answer a request whose text contains
    /// `needle`.
    async fn answered_with(&mut self, needle: &str) {
        bounded("rcpd begins its answer", async {
            while !self
                .answered
                .recv()
                .await
                .expect("the proxy runs")
                .contains(needle)
            {}
        })
        .await;
    }
}

/// Where a [`Proxy`] reports what each request met.
struct Reports {
    requests: tokio::sync::mpsc::UnboundedSender<String>,
    answered: tokio::sync::mpsc::UnboundedSender<String>,
    refused: tokio::sync::mpsc::UnboundedSender<String>,
}

/// One request through the [`Proxy`]: see there.
async fn relay(
    mut client: tokio::net::TcpStream,
    rcpd: SocketAddr,
    reports: Reports,
    mut gate: tokio::sync::watch::Receiver<Gate>,
) -> std::io::Result<()> {
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    let request = read_http_request(&mut client).await?;
    let text = String::from_utf8_lossy(&request).into_owned();
    let _ = reports.requests.send(text.clone());
    if gate.wait_for(|g| *g == Gate::Open).await.is_err() {
        return Ok(());
    }
    let mut upstream = match tokio::net::TcpStream::connect(rcpd).await {
        Ok(upstream) => upstream,
        Err(e) => {
            drop(client);
            let _ = reports.refused.send(text);
            return Err(e);
        }
    };
    upstream.write_all(&request).await?;
    let mut first = vec![0u8; 64 * 1024];
    let n = upstream.read(&mut first).await?;
    let _ = reports.answered.send(text);
    client.write_all(&first[..n]).await?;
    let _ = tokio::io::copy_bidirectional(&mut client, &mut upstream).await;
    Ok(())
}

/// One HTTP/1 request from `stream`: its head, then `content-length` bytes.
async fn read_http_request(stream: &mut tokio::net::TcpStream) -> std::io::Result<Vec<u8>> {
    use tokio::io::AsyncReadExt;
    let mut buf = Vec::new();
    let mut chunk = [0u8; 4096];
    loop {
        if let Some(end) = buf.windows(4).position(|w| w == b"\r\n\r\n") {
            let head = String::from_utf8_lossy(&buf[..end]).to_ascii_lowercase();
            let length = head
                .lines()
                .find_map(|l| l.strip_prefix("content-length:"))
                .map_or(0, |v| v.trim().parse::<usize>().expect("a content length"));
            if buf.len() >= end + 4 + length {
                return Ok(buf);
            }
        }
        let n = stream.read(&mut chunk).await?;
        if n == 0 {
            return Ok(buf);
        }
        buf.extend_from_slice(&chunk[..n]);
    }
}

/// The `cerulion-remoted` daemon (`serve_endpoint`, so the ops and rcp planes
/// share its one e-stop lease) beside the real rcpd of `side`, which it
/// reaches through `proxy`.
struct Daemon {
    endpoint: Endpoint,
    robot: EndpointAddr,
    proxy: Proxy,
    stop: Option<tokio::sync::oneshot::Sender<()>>,
    side: RcpdSide,
    _dir: tempfile::TempDir,
}

impl Daemon {
    /// The daemon with its proxy's gate at `gate`.
    async fn start(gate: Gate) -> Self {
        let dir = tempfile::tempdir().unwrap();
        let side = RcpdSide::start(dir.path()).await;
        let proxy = Proxy::start(side.rcpd_addr, gate).await;
        let robot_endpoint = endpoint(ROBOT_SEED).await;
        let robot = loopback_addr(robot_endpoint.id(), &robot_endpoint.bound_sockets());
        let (store, index) = robot_store(dir.path(), *robot_endpoint.id().as_bytes());
        std::fs::create_dir_all(dir.path().join("remoted")).unwrap();
        let mut config = RemotedConfig::from_state_root(dir.path(), RelayConfig::Disabled, false);
        std::fs::create_dir_all(&config.log_root).unwrap();
        config.rcp.addr = proxy.addr;
        config.rcp.token_file = write_token_file(dir.path());
        // Past every bounded step: an engage that waited for rcpd's answer
        // fails its step.
        config.rcp.limits.head_timeout = 2 * STEP;
        let (stop, stopped) = tokio::sync::oneshot::channel::<()>();
        let (serving, manager) = (robot_endpoint.clone(), side.mgr.clone());
        tokio::spawn(async move {
            serve_endpoint(
                &config,
                store,
                index,
                MAC_KEY.to_vec(),
                &serving,
                RemotedClock::fixed(T_NOW),
                None,
                Some(manager),
                async {
                    let _ = stopped.await;
                },
            )
            .await
        });
        Daemon {
            endpoint: robot_endpoint,
            robot,
            proxy,
            stop: Some(stop),
            side,
            _dir: dir,
        }
    }

    /// `engage-estop` from `desk`.
    async fn engage(&self, desk: &Endpoint) -> Value {
        ops_call(desk, self.robot.clone(), "engage-estop", json!({}))
            .await
            .expect("engage-estop answers")
    }

    /// The ops latch, as `topic-publish` from `desk` reads it: its refusal of
    /// a type no robot knows, past the e-stop check.
    async fn publish_refusal(&self, desk: &Endpoint) -> String {
        let args =
            json!({ "topic": "/rcp_e2e/latch", "type": "nope_msgs/msg/Nope", "message": {} });
        match ops_call(desk, self.robot.clone(), "topic-publish", args).await {
            Err(CerudError::Remote { kind, message }) if kind == "verb_error" => message,
            other => panic!("expected the verb's error, got {other:?}"),
        }
    }
}

impl Drop for Daemon {
    fn drop(&mut self) {
        if let Some(stop) = self.stop.take() {
            let _ = stop.send(());
        }
    }
}

/// The reason, and the field the graph reads it in, of the estop an ops
/// `engage-estop` sends to rcpd.
fn ops_estop_args() -> Value {
    json!({ "reason": "the robot's ops e-stop was engaged (engage-estop)" })
}

const PUBLISH_REFUSED: &str =
    "verb error: unavailable: the robot's e-stop is engaged, so topic-publish is refused";

#[tokio::test(flavor = "multi_thread")]
async fn an_ops_engage_estop_closes_rcpds_latch_and_rcpd_refuses_the_call_it_holds() {
    let mut d = Daemon::start(Gate::Open).await;
    let mut s = open_session(&d.endpoint, OPERATOR.0).await;
    s.initialize().await;
    // A call the graph holds open, then a mutating call that waits behind it
    // in rcpd.
    let held = json!({ "jsonrpc": "2.0", "id": 2, "method": "tools/call",
                       "params": { "name": "arm.hold", "arguments": {} } });
    s.send_raw(held.to_string().as_bytes()).await;
    d.side.graph_sees("arm.hold").await;
    s.send_raw(step_call(3).to_string().as_bytes()).await;
    // rcpd has taken the waiting call into its session: its answer began.
    d.proxy.answered_with(r#""name":"arm.step""#).await;

    // Any paired account engages the ops e-stop.
    let viewer = endpoint(VIEWER.0).await;
    let r = d.engage(&viewer).await;
    assert_eq!(r["engaged"], json!(true), "{r}");
    let r = s.call_step(4).await;
    assert_eq!(r["error"]["code"], json!(-32001), "{r}");
    assert_eq!(
        r["error"]["message"],
        json!("refused: the robot's e-stop is engaged"),
        "{r}"
    );

    // The estop reached rcpd under the engager's identity.
    d.side.graph_sees("estop").await;
    let estop = d
        .side
        .seen
        .lock()
        .unwrap()
        .iter()
        .find(|c| c["tool"] == json!("estop"))
        .cloned()
        .unwrap();
    assert_eq!(estop["args"], ops_estop_args(), "{estop}");
    assert_eq!(
        estop["caller"],
        json!(expected_caller(VIEWER.0, VIEWER.1)),
        "{estop}"
    );
    assert!(d.side.rcpd.host.estopped(), "rcpd's latch is open");
    assert!(
        cerulion_rcpd::tools::latch_path(&d.side.receipts).exists(),
        "rcpd wrote no latch file"
    );

    // The graph answers the held call; the waiting call takes its turn and
    // is refused under rcpd's latch, never published.
    d.side.release_held_calls();
    let [held, r] = <[Value; 2]>::try_from(s.replies(&[json!(2), json!(3)]).await).unwrap();
    assert!(held.get("result").is_some(), "the held call ran on: {held}");
    let text = r["result"]["content"][0]["text"]
        .as_str()
        .expect("rcpd's refusal");
    let body: Value = serde_json::from_str(text).unwrap();
    assert_eq!(body["error"]["code"], json!("estopped"), "{r}");
    assert!(
        !d.side
            .seen
            .lock()
            .unwrap()
            .iter()
            .any(|c| c["tool"] == json!("arm.step")),
        "rcpd published the waiting call"
    );
    let steps: Vec<_> = cerud::receipt::read_all(&d.side.receipts)
        .unwrap()
        .into_iter()
        .filter(|r| r.verb == "arm.step")
        .map(|r| (r.caller, r.outcome))
        .collect();
    assert_eq!(
        steps,
        [(
            expected_caller(OPERATOR.0, OPERATOR.1),
            cerud::receipt::ReceiptOutcome::Denied
        )]
    );
}

#[tokio::test(flavor = "multi_thread")]
async fn an_ops_engage_estop_answers_before_its_estop_reaches_rcpd() {
    let mut d = Daemon::start(Gate::Shut).await;
    let (viewer, operator) = (endpoint(VIEWER.0).await, endpoint(OPERATOR.0).await);
    // The estop waits at the shut gate; engage-estop has answered.
    let r = d.engage(&viewer).await;
    assert_eq!(r["engaged"], json!(true), "{r}");
    let initialize = d.proxy.next_request().await;
    assert!(
        initialize.contains(r#""method":"initialize""#),
        "{initialize}"
    );
    // A second engage while that estop is in flight answers too.
    let r = d.engage(&viewer).await;
    assert_eq!(r["engaged"], json!(true), "{r}");
    assert_eq!(d.publish_refusal(&operator).await, PUBLISH_REFUSED);

    d.proxy.open();
    let initialized = d.proxy.next_request().await;
    assert!(
        initialized.contains(r#""method":"notifications/initialized""#),
        "{initialized}"
    );
    let call = d.proxy.next_request().await;
    let bearer = format!("authorization: Bearer {TOKEN}");
    let caller = format!("x-cerulion-caller: {}", expected_caller(VIEWER.0, VIEWER.1));
    for want in [
        r#""method":"tools/call""#,
        r#""name":"estop""#,
        r#""reason":"the robot's ops e-stop was engaged (engage-estop)""#,
        bearer.as_str(),
        caller.as_str(),
        "mcp-session-id: ",
    ] {
        assert!(call.contains(want), "{want:?} missing from {call}");
    }
    d.side.graph_sees("estop").await;
    assert!(d.side.rcpd.host.estopped(), "rcpd's latch is open");
}

#[tokio::test(flavor = "multi_thread")]
async fn an_ops_engage_estop_with_rcpd_stopped_engages_and_its_estop_arrives_once_rcpd_serves() {
    let mut d = Daemon::start(Gate::Open).await;
    d.side.stop_rcpd();
    let rcpd_addr = d.side.rcpd_addr;
    bounded("rcpd stops accepting", async {
        while tokio::net::TcpStream::connect(rcpd_addr).await.is_ok() {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    })
    .await;
    let (viewer, operator) = (endpoint(VIEWER.0).await, endpoint(OPERATOR.0).await);
    let r = d.engage(&viewer).await;
    assert_eq!(r["engaged"], json!(true), "{r}");
    // Its estop finds no rcpd, and is sent again with no new engage.
    for _ in 0..2 {
        let refused = d.proxy.next_refused().await;
        assert!(refused.contains(r#""method":"initialize""#), "{refused}");
    }
    // The ops latch is engaged, for the act verbs and for every rcp tool call.
    assert_eq!(d.publish_refusal(&operator).await, PUBLISH_REFUSED);
    let mut s = open_session_from(operator.clone(), d.robot.clone()).await;
    let r = s.call_step(2).await;
    assert_eq!(r["error"]["code"], json!(-32001), "{r}");
    assert!(
        !d.side.rcpd.host.estopped(),
        "the stopped rcpd closed its latch"
    );

    // rcpd serves again: the estop reaches it with no second engage.
    d.side.restart_rcpd().await;
    bounded("rcpd closes its latch", async {
        while !d.side.rcpd.host.estopped() {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    })
    .await;
    d.side.graph_sees("estop").await;
}
