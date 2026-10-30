// SPDX-License-Identifier: AGPL-3.0-only
//! The remote plane hosted as a library: [`RemotePlane`] over a temporary state
//! root, a [`KeyProvider`] that holds its keys in memory (no key file is
//! written), and its schema metadata from an injected local network daemon
//! control socket or, for `RemotePlane::in_process`, an in-process
//! `SchemaSource`.
//!
//! Every endpoint is loopback iroh with the relay disabled and every await is
//! bounded. Keys come from fixed seeds; each expected value is a hand-written
//! literal (the topic, its schema name, the refusal text), never a second run.

mod common;
// This test uses only the local network daemon from the provider fixture.
#[allow(dead_code)]
#[path = "local_schema_provider/fixture.rs"]
mod fixture;

use std::path::Path;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use cerud::client::OpsClient;
use cerud::error::CerudError;
use cerud::receipt::{read_all, ReceiptOutcome};
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{
    GatewayEgressPolicy, GatewayPlan, SchemaDoc, SchemaEncoding, SchemaServing, TopicSchema,
};
use cerulion_link::{
    alpn, dial, open_frame_stream, read_frame, write_frame, Endpoint, EndpointAddr, QuicOpsStream,
    RelayConfig, DEFAULT_MAX_FRAME_LEN,
};
use cerulion_pairing::format::{PrincipalKind, PublicKey, RootSet};
use cerulion_pairing::verify::TrustStore;
use cerulion_remoted::{
    BoundPlane, DeviceAccountIndex, KeyFiles, KeyProvider, RemotePlane, RemotedConfig,
    RemotedError, SchemaSource, WireRequest, WireResponse,
};
use common::{
    bounded, disabled_endpoint, loopback_addr, pk, sk, CHASSIS, MAC_KEY, OWNER, ROBOT_ID, T_NOW,
};
use fixture::LocalNetd;
use tokio::sync::oneshot;
use tokio::task::JoinHandle;

/// The identity the hosted plane is given; every wire reply names it.
const ROBOT: &str = "library-host";
/// The hosted plane's device key seed; its public half is the endpoint id.
const ROBOT_SEED: u8 = 71;
/// A desk the owner has paired.
const DESK_SEED: u8 = 72;
/// A key nobody has paired.
const STRANGER_SEED: u8 = 73;
const TOPIC: &str = "/library/reading";
const SCHEMA: &str = "example_msgs/Reading";
/// The provider's error when no daemon answers on the control socket.
const NO_DAEMON: &str = "cannot reach the local network daemon";

/// A key provider that holds both keys in memory and records each MAC key
/// request's `required` flag.
struct MemoryKeys {
    device: Result<[u8; 32], String>,
    asked_mac: Mutex<Vec<bool>>,
}

impl MemoryKeys {
    fn new(seed: u8) -> Self {
        MemoryKeys {
            device: Ok([seed; 32]),
            asked_mac: Mutex::new(Vec::new()),
        }
    }
    fn refusing(reason: &str) -> Self {
        MemoryKeys {
            device: Err(reason.to_owned()),
            asked_mac: Mutex::new(Vec::new()),
        }
    }
    fn asked_mac(&self) -> Vec<bool> {
        self.asked_mac.lock().unwrap().clone()
    }
}

impl KeyProvider for MemoryKeys {
    fn device_key(&self) -> Result<[u8; 32], RemotedError> {
        self.device.clone().map_err(RemotedError::DeviceKey)
    }
    fn store_mac_key(&self, required: bool) -> Result<Vec<u8>, RemotedError> {
        self.asked_mac.lock().unwrap().push(required);
        Ok(MAC_KEY.to_vec())
    }
}

fn public(seed: u8) -> PublicKey {
    pk(&sk(seed))
}

/// Write a claimed trust store bound to `robot` and a side map that pairs the
/// desk with the owner, both MAC'd with the provider's key, under `root`.
fn provision(root: &Path, robot: PublicKey) {
    let remoted = root.join("remoted");
    std::fs::create_dir_all(&remoted).unwrap();
    let mut store = TrustStore::provision(
        ROBOT_ID,
        robot,
        RootSet::new(vec![pk(&sk(1))], 1).unwrap(),
        CHASSIS,
        0,
    )
    .unwrap()
    .with_path(remoted.join("trust_store"));
    store
        .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    store.save(MAC_KEY).unwrap();
    let mut index = DeviceAccountIndex::new().with_path(remoted.join("device_index.json"));
    index.bind(public(DESK_SEED), OWNER);
    index.save(MAC_KEY).unwrap();
}

/// [`RemotePlane::bind`] under the step bound.
trait BindBounded {
    async fn bind_bounded(self, keys: &dyn KeyProvider) -> Result<BoundPlane, RemotedError>;
}

impl BindBounded for RemotePlane {
    async fn bind_bounded(self, keys: &dyn KeyProvider) -> Result<BoundPlane, RemotedError> {
        bounded("bind", self.bind(keys)).await
    }
}

/// Every file under `<root>/remoted/` with its bytes, sorted by name.
fn state_files(root: &Path) -> Vec<(String, Vec<u8>)> {
    let Ok(entries) = std::fs::read_dir(root.join("remoted")) else {
        return Vec::new();
    };
    let mut files: Vec<_> = entries
        .map(|entry| {
            let path = entry.unwrap().path();
            let name = path.file_name().unwrap().to_string_lossy().into_owned();
            (name, std::fs::read(&path).unwrap())
        })
        .collect();
    files.sort();
    files
}

/// A bound plane serving on its own task until [`Hosted::stop`].
struct Hosted {
    address: EndpointAddr,
    stop: oneshot::Sender<()>,
    task: JoinHandle<Result<(), RemotedError>>,
}

impl Hosted {
    fn start(bound: BoundPlane) -> Self {
        let address = loopback_addr(bound.endpoint_id(), &bound.bound_sockets());
        let (stop, stopped) = oneshot::channel::<()>();
        let task = tokio::spawn(bound.serve(async move {
            let _ = stopped.await;
        }));
        Hosted {
            address,
            stop,
            task,
        }
    }
    async fn stop(self) {
        let _ = self.stop.send(());
        bounded("plane stops", self.task)
            .await
            .expect("serve task joins")
            .expect("serve returns Ok");
    }
}

/// One `cerulion/wire/2` control request from `desk` and its reply.
async fn wire_requests(desk: &Endpoint, plane: &Hosted, requests: &[WireRequest]) -> Vec<Vec<u8>> {
    let connection = bounded("dial wire", dial(desk, plane.address.clone(), alpn::WIRE))
        .await
        .expect("wire dial");
    let (mut send, mut recv) = bounded("open control", open_frame_stream(&connection))
        .await
        .expect("control stream");
    let mut replies = Vec::new();
    for request in requests {
        let bytes = serde_json::to_vec(request).unwrap();
        bounded("send request", write_frame(&mut send, &bytes))
            .await
            .unwrap();
        replies.push(
            bounded("read reply", read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN))
                .await
                .unwrap(),
        );
    }
    connection.close(0u32.into(), b"test finished");
    replies
}

/// The result of each call in one ops session.
type CallResults = Vec<Result<serde_json::Value, CerudError>>;

/// One ops session from `desk`: the server's handshake name, then each verb
/// with its args and each result.
async fn ops_calls(
    desk: &Endpoint,
    plane: &Hosted,
    calls: Vec<(&'static str, serde_json::Value)>,
) -> (String, CallResults) {
    let connection = bounded("dial ops", dial(desk, plane.address.clone(), alpn::OPS))
        .await
        .expect("ops dial");
    let (send, recv) = bounded("open ops stream", open_frame_stream(&connection))
        .await
        .expect("ops stream");
    let stream = QuicOpsStream::new(send, recv);
    let results = bounded(
        "ops session",
        tokio::task::spawn_blocking(move || {
            let mut client = OpsClient::connect_current(stream).expect("ops handshake");
            let server = client.server_name().to_owned();
            let results = calls
                .into_iter()
                .map(|(verb, args)| client.call(verb, args))
                .collect();
            (server, results)
        }),
    )
    .await
    .expect("ops session joins");
    drop(connection);
    results
}

/// Each receipt's verb and outcome, in log order.
fn receipt_rows(root: &Path) -> Vec<(String, ReceiptOutcome)> {
    read_all(&root.join("remoted").join("receipts.log"))
        .unwrap()
        .into_iter()
        .map(|receipt| (receipt.verb, receipt.outcome))
        .collect()
}

fn remote_error(result: &Result<serde_json::Value, CerudError>) -> (String, String) {
    match result {
        Err(CerudError::Remote { kind, message }) => (kind.clone(), message.clone()),
        other => panic!("expected a remote error, got {other:?}"),
    }
}

/// Register `TOPIC` as `SCHEMA` with the local daemon in the plane's namespace.
fn register_topic(netd: &LocalNetd) {
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![TOPIC.into()],
        ingress: vec![],
    };
    let serving = SchemaServing {
        served_topics: vec![TOPIC.into()],
        topic_schemas: vec![TopicSchema {
            topic: TOPIC.into(),
            schema_name: SCHEMA.into(),
        }],
        schema_hashes: vec![],
        schema_docs: vec![],
    };
    let namespace = serde_json::to_string(&netd.manager.iox_config()).unwrap();
    netd.client()
        .register_egress(&plan, &serving, Some(&namespace))
        .unwrap();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_library_host_serves_the_catalog_through_the_injected_control_socket() {
    let root = tempfile::tempdir().unwrap();
    provision(root.path(), public(ROBOT_SEED));
    let netd = LocalNetd::start();
    register_topic(&netd);
    let _publisher = netd
        .manager
        .create_publisher_simple(TOPIC, MaxSliceLen::const_new(256))
        .unwrap();
    let keys = MemoryKeys::new(ROBOT_SEED);
    let bound = RemotePlane::new(root.path(), ROBOT, netd.path.clone(), RelayConfig::Disabled)
        .with_transport(netd.manager.clone())
        .bind_bounded(&keys)
        .await
        .expect("the library entry binds");
    assert_eq!(*bound.endpoint_id().as_bytes(), public(ROBOT_SEED).0);
    // bind publishes the public endpoint facts under the injected root.
    let facts: serde_json::Value = serde_json::from_slice(
        &std::fs::read(root.path().join("remoted").join("beacon_facts.json")).unwrap(),
    )
    .unwrap();
    assert_eq!(facts["eid"], hex::encode(public(ROBOT_SEED).0));
    assert_eq!(facts["claimable"], "0", "the store is claimed");
    let plane = Hosted::start(bound);
    let desk = disabled_endpoint([DESK_SEED; 32]).await;

    let schema = WireRequest::Schema {
        topic: TOPIC.into(),
    };
    let replies = wire_requests(
        &desk,
        &plane,
        &[WireRequest::Catalog, WireRequest::Catalog, schema],
    )
    .await;
    let WireResponse::Catalog(catalog) = serde_json::from_slice(&replies[0]).unwrap() else {
        panic!(
            "expected a catalog: {}",
            String::from_utf8_lossy(&replies[0])
        );
    };
    assert_eq!(catalog.robot, ROBOT);
    assert_eq!(catalog.error, None);
    let rows: Vec<_> = catalog
        .entries
        .iter()
        .filter(|row| row.topic == TOPIC)
        .collect();
    assert_eq!(rows.len(), 1, "one row for {TOPIC}: {:?}", catalog.entries);
    assert_eq!(rows[0].schema_name.as_deref(), Some(SCHEMA));
    assert_eq!(
        replies[0], replies[1],
        "a repeated catalog is byte-identical"
    );
    // The schema route reads the same daemon: it knows the topic's type and has
    // no custom closure for it.
    let WireResponse::Schema(schema) = serde_json::from_slice(&replies[2]).unwrap() else {
        panic!(
            "expected a schema reply: {}",
            String::from_utf8_lossy(&replies[2])
        );
    };
    assert_eq!(
        (schema.robot.as_str(), schema.requested.as_str()),
        (ROBOT, TOPIC)
    );
    assert!(schema.docs.is_empty());
    let error = schema.error.unwrap_or_default();
    assert!(error.contains("no custom schema closure"), "error: {error}");

    plane.stop().await;
    netd.shutdown();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_control_socket_with_no_daemon_refuses_the_catalog_with_the_provider_error() {
    let root = tempfile::tempdir().unwrap();
    provision(root.path(), public(ROBOT_SEED));
    let manager = fixture::manager(false);
    let absent = root.path().join("no-daemon.sock");
    let keys = MemoryKeys::new(ROBOT_SEED);
    let bound = RemotePlane::new(root.path(), ROBOT, absent, RelayConfig::Disabled)
        .with_transport(manager)
        .bind_bounded(&keys)
        .await
        .unwrap();
    let plane = Hosted::start(bound);
    let desk = disabled_endpoint([DESK_SEED; 32]).await;

    let replies = wire_requests(&desk, &plane, &[WireRequest::Catalog]).await;
    match serde_json::from_slice(&replies[0]).unwrap() {
        WireResponse::Error { message, .. } => {
            assert!(message.contains(NO_DAEMON), "message: {message}")
        }
        other => panic!("expected the provider error, got {other:?}"),
    }
    plane.stop().await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn an_in_process_schema_source_names_the_catalog_and_serves_the_closure() {
    let root = tempfile::tempdir().unwrap();
    provision(root.path(), public(ROBOT_SEED));
    let manager = fixture::manager(false);
    let _publisher = manager
        .create_publisher_simple(TOPIC, MaxSliceLen::const_new(256))
        .unwrap();
    let reads = Arc::new(AtomicUsize::new(0));
    let counted = reads.clone();
    let source: SchemaSource = Arc::new(move || {
        counted.fetch_add(1, Ordering::SeqCst);
        SchemaServing {
            served_topics: vec![TOPIC.into()],
            topic_schemas: vec![TopicSchema {
                topic: TOPIC.into(),
                schema_name: SCHEMA.into(),
            }],
            schema_hashes: vec![],
            schema_docs: vec![SchemaDoc {
                qualified: SCHEMA.into(),
                encoding: SchemaEncoding::Msg,
                text: "float64 value\n".into(),
                deps: vec![],
            }],
        }
    });
    let keys = MemoryKeys::new(ROBOT_SEED);
    let bound = RemotePlane::in_process(root.path(), ROBOT, RelayConfig::Disabled, source)
        .with_transport(manager)
        .bind_bounded(&keys)
        .await
        .expect("the in-process entry binds");
    let plane = Hosted::start(bound);
    let desk = disabled_endpoint([DESK_SEED; 32]).await;

    let schema = WireRequest::Schema {
        topic: TOPIC.into(),
    };
    let replies = wire_requests(&desk, &plane, &[WireRequest::Catalog, schema]).await;
    let WireResponse::Catalog(catalog) = serde_json::from_slice(&replies[0]).unwrap() else {
        panic!(
            "expected a catalog: {}",
            String::from_utf8_lossy(&replies[0])
        );
    };
    assert_eq!(catalog.error, None);
    let rows: Vec<_> = catalog
        .entries
        .iter()
        .filter(|row| row.topic == TOPIC)
        .collect();
    assert_eq!(rows.len(), 1, "one row for {TOPIC}: {:?}", catalog.entries);
    assert_eq!(rows[0].schema_name.as_deref(), Some(SCHEMA));
    let WireResponse::Schema(schema) = serde_json::from_slice(&replies[1]).unwrap() else {
        panic!(
            "expected a schema reply: {}",
            String::from_utf8_lossy(&replies[1])
        );
    };
    assert_eq!(schema.error, None);
    let docs: Vec<_> = schema
        .docs
        .iter()
        .map(|doc| (doc.qualified.as_str(), doc.text.as_str()))
        .collect();
    assert_eq!(docs, vec![(SCHEMA, "float64 value\n")]);
    assert_eq!(
        reads.load(Ordering::SeqCst),
        2,
        "each metadata reply reads the source once"
    );
    plane.stop().await;
}

#[tokio::test]
async fn an_in_process_host_with_no_keys_binds_nothing_and_writes_nothing() {
    let root = tempfile::tempdir().unwrap();
    let keys = KeyFiles {
        key_file: root.path().join("remoted").join("device_key"),
        store_mac_key_file: root.path().join("remoted").join("trust_store.mac_key"),
    };
    let source: SchemaSource = Arc::new(SchemaServing::default);
    let refused = RemotePlane::in_process(root.path(), ROBOT, RelayConfig::Disabled, source)
        .bind_bounded(&keys)
        .await;
    assert!(
        matches!(refused, Err(RemotedError::DeviceKey(_))),
        "a fresh state root is refused at the device key"
    );
    assert_eq!(state_files(root.path()), Vec::new(), "nothing is written");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_library_host_refuses_the_daemon_only_verbs_by_name() {
    let root = tempfile::tempdir().unwrap();
    provision(root.path(), public(ROBOT_SEED));
    let keys = MemoryKeys::new(ROBOT_SEED);
    let bound = RemotePlane::new(
        root.path(),
        ROBOT,
        root.path().join("no-daemon.sock"),
        RelayConfig::Disabled,
    )
    .bind_bounded(&keys)
    .await
    .unwrap();
    let plane = Hosted::start(bound);
    let desk = disabled_endpoint([DESK_SEED; 32]).await;

    let (server, results) = ops_calls(
        &desk,
        &plane,
        vec![
            ("inventory", serde_json::json!({})),
            ("restart", serde_json::json!({"graph": "demo"})),
            ("engage-estop", serde_json::json!({})),
        ],
    )
    .await;
    assert_eq!(
        server, "cerulion-remote-plane",
        "a library host is not the daemon"
    );
    assert!(results[0].is_ok(), "inventory is served: {:?}", results[0]);
    let (kind, message) = remote_error(&results[1]);
    assert_eq!(kind, "unsupported_host");
    assert!(
        message.contains("restart is served only by the cerulion-remoted daemon"),
        "message: {message}"
    );
    assert!(message.contains("systemd"), "message: {message}");
    let (kind, message) = remote_error(&results[2]);
    assert_eq!(kind, "unsupported_host");
    assert!(
        message.contains("engage-estop is served only by the cerulion-remoted daemon"),
        "message: {message}"
    );
    assert!(
        message.contains("stop through the host's own stop path"),
        "message: {message}"
    );

    // The authorizer still runs first: an unpaired key is denied, not refused.
    let stranger = disabled_endpoint([STRANGER_SEED; 32]).await;
    let (_, results) = ops_calls(
        &stranger,
        &plane,
        vec![
            ("restart", serde_json::json!({"graph": "demo"})),
            ("engage-estop", serde_json::json!({})),
        ],
    )
    .await;
    assert_eq!(remote_error(&results[0]).0, "denied");
    assert_eq!(remote_error(&results[1]).0, "denied");
    plane.stop().await;

    // Every call is receipted; a refusal is an error outcome with no intent row.
    let unsupported = || ReceiptOutcome::Error("unsupported_host".into());
    assert_eq!(
        receipt_rows(root.path()),
        vec![
            ("inventory".into(), ReceiptOutcome::Ok),
            ("restart".into(), unsupported()),
            ("engage-estop".into(), unsupported()),
            ("restart".into(), ReceiptOutcome::Denied),
            ("engage-estop".into(), ReceiptOutcome::Denied),
        ]
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn the_daemon_configuration_serves_the_real_restart_verb() {
    let root = tempfile::tempdir().unwrap();
    provision(root.path(), public(ROBOT_SEED));
    let config = RemotedConfig::from_state_root(root.path(), RelayConfig::Disabled, false);
    let keys = MemoryKeys::new(ROBOT_SEED);
    let bound = config.daemon_plane().bind_bounded(&keys).await.unwrap();
    let plane = Hosted::start(bound);
    let desk = disabled_endpoint([DESK_SEED; 32]).await;

    // An empty graph name fails validation before any systemd command runs
    // (`RestartVerb` validates the name before it checks the host), so this
    // reaches the real verb on every platform without running anything.
    let (server, results) = ops_calls(
        &desk,
        &plane,
        vec![("restart", serde_json::json!({"graph": ""}))],
    )
    .await;
    let (kind, message) = remote_error(&results[0]);
    assert_eq!(kind, "verb_error");
    assert!(
        message.contains("graph name is empty"),
        "message: {message}"
    );
    assert_eq!(server, "cerulion-remoted");
    plane.stop().await;
}

#[tokio::test]
async fn a_key_provider_error_surfaces_from_bind_before_any_other_key_is_read() {
    let root = tempfile::tempdir().unwrap();
    provision(root.path(), public(ROBOT_SEED));
    let keys = MemoryKeys::refusing("the host keystore is locked");
    let before = state_files(root.path());
    let error = RemotePlane::new(
        root.path(),
        ROBOT,
        root.path().join("s"),
        RelayConfig::Disabled,
    )
    .bind_bounded(&keys)
    .await
    .err()
    .expect("bind fails");
    assert_eq!(
        state_files(root.path()),
        before,
        "a refused bind changes no state file"
    );
    assert!(
        matches!(&error, RemotedError::DeviceKey(reason) if reason == "the host keystore is locked"),
        "error: {error:?}"
    );
    assert_eq!(keys.asked_mac(), Vec::<bool>::new());
}

#[tokio::test]
async fn the_mac_key_is_required_only_when_trust_state_exists() {
    let fresh = tempfile::tempdir().unwrap();
    let keys = MemoryKeys::new(ROBOT_SEED);
    let before = state_files(fresh.path());
    let error = RemotePlane::new(
        fresh.path(),
        ROBOT,
        fresh.path().join("s"),
        RelayConfig::Disabled,
    )
    .bind_bounded(&keys)
    .await
    .err()
    .expect("a fresh root has no trust store");
    assert_eq!(
        state_files(fresh.path()),
        before,
        "a refused bind changes no state file"
    );
    assert!(matches!(error, RemotedError::Store(_)), "error: {error:?}");
    assert_eq!(keys.asked_mac(), vec![false]);

    let provisioned = tempfile::tempdir().unwrap();
    provision(provisioned.path(), public(ROBOT_SEED));
    let keys = MemoryKeys::new(ROBOT_SEED);
    let bound = RemotePlane::new(
        provisioned.path(),
        ROBOT,
        provisioned.path().join("s"),
        RelayConfig::Disabled,
    )
    .bind_bounded(&keys)
    .await
    .unwrap();
    assert_eq!(keys.asked_mac(), vec![true]);
    Hosted::start(bound).stop().await;
}

#[tokio::test]
async fn a_device_key_that_is_not_the_stores_key_is_refused() {
    let root = tempfile::tempdir().unwrap();
    provision(root.path(), public(ROBOT_SEED));
    let keys = MemoryKeys::new(STRANGER_SEED);
    let before = state_files(root.path());
    let error = RemotePlane::new(
        root.path(),
        ROBOT,
        root.path().join("s"),
        RelayConfig::Disabled,
    )
    .bind_bounded(&keys)
    .await
    .err()
    .expect("a store for another device is refused");
    assert_eq!(
        state_files(root.path()),
        before,
        "a refused bind changes no state file"
    );
    assert!(
        matches!(error, RemotedError::ProvisioningMismatch(_)),
        "error: {error:?}"
    );
}

/// A provider that returns an empty MAC key even when trust state exists.
struct EmptyMacKeys;

impl KeyProvider for EmptyMacKeys {
    fn device_key(&self) -> Result<[u8; 32], RemotedError> {
        Ok([ROBOT_SEED; 32])
    }
    fn store_mac_key(&self, _required: bool) -> Result<Vec<u8>, RemotedError> {
        Ok(Vec::new())
    }
}

#[tokio::test]
async fn an_empty_mac_key_for_existing_trust_state_is_a_mac_key_error() {
    let root = tempfile::tempdir().unwrap();
    provision(root.path(), public(ROBOT_SEED));
    let before = state_files(root.path());
    let error = RemotePlane::new(
        root.path(),
        ROBOT,
        root.path().join("s"),
        RelayConfig::Disabled,
    )
    .bind_bounded(&EmptyMacKeys)
    .await
    .err()
    .expect("an empty key cannot verify the existing store");
    assert_eq!(
        state_files(root.path()),
        before,
        "a refused bind changes no state file"
    );
    match error {
        RemotedError::MacKey(message) => {
            assert!(
                message.contains("empty trust-store MAC key"),
                "message: {message}"
            )
        }
        other => panic!("expected a MAC key error, got {other:?}"),
    }
}
