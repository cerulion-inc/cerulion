// SPDX-License-Identifier: AGPL-3.0-only
//! Live metadata authorization on real, demandless loopback connections.

use std::collections::BTreeMap;
use std::future::Future;
use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::transport::cerulion_q::{SchemaDoc, SchemaEncoding};
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{TransportConfig, TransportManager};
use cerulion_link::{
    accept_one, alpn, build_endpoint, dial, direct_addr, open_frame_stream, read_frame,
    write_frame, Connection, Endpoint, EndpointAddr, EndpointConfig, RecvStream, RelayConfig,
    SendStream, DEFAULT_MAX_FRAME_LEN,
};
use cerulion_pairing::format::{
    AccessListEpoch, AccountId, DeviceCert, Grant, IntermediateCert, PrincipalKind, PublicKey,
    RobotId, RootSet, Scope, SignedIntermediateCert, Validity, FORMAT_VERSION,
};
use cerulion_pairing::verify::{EpochSyncWire, PairingPresentation, TrustStore};
use cerulion_remoted::{
    handle_accepted_with_wire, DeviceAccountIndex, KeyAccess, PairingAuthorizer, RemotedClock,
    SharedTrust, WirePlane, WireRequest, WireResponse,
};
use ed25519_dalek::SigningKey;
use tokio::task::{JoinHandle, JoinSet};

const NOW: u64 = 1_000_000;
const OWNER: AccountId = AccountId([10; 32]);
const ROBOT: RobotId = RobotId([11; 32]);
const TOPIC: &str = "/metadata/sample";
const SCHEMA: &str = "sample_msgs/Reading";
const SCHEMA_TEXT: &str = "uint64 value\n";

async fn bounded<F: Future>(future: F) -> F::Output {
    tokio::time::timeout(Duration::from_secs(20), future)
        .await
        .expect("metadata test exceeded its deadline")
}
fn signing(seed: u8) -> SigningKey {
    SigningKey::from_bytes(&[seed; 32])
}
fn public(key: &SigningKey) -> PublicKey {
    PublicKey(key.verifying_key().to_bytes())
}
fn validity() -> Validity {
    Validity {
        not_before_ns: 0,
        not_after_ns: 10_000_000,
    }
}
fn intermediate() -> SignedIntermediateCert {
    IntermediateCert {
        version: FORMAT_VERSION,
        intermediate_key: public(&signing(2)),
        validity: validity(),
        issued_at_ns: 1,
        max_scope: Scope::OWNER_FULL,
    }
    .sign_by_roots(&[&signing(1)])
}
fn presentation(device: PublicKey) -> PairingPresentation {
    PairingPresentation {
        intermediate: intermediate(),
        device_cert: DeviceCert {
            version: FORMAT_VERSION,
            device_key: device,
            account: OWNER,
            principal_kind: PrincipalKind::Human,
            scope: Scope::OWNER_FULL,
            validity: validity(),
            issued_at_ns: 1,
            issuer_key: public(&signing(2)),
        }
        .sign(&signing(2)),
        grant: Grant {
            version: FORMAT_VERSION,
            subject: OWNER,
            robot: ROBOT,
            scope: Scope::OWNER_FULL,
            principal_kind: PrincipalKind::Human,
            delegation_depth: 0,
            validity: validity(),
            issued_at_ns: 1,
            issuer: OWNER,
            issuer_key: public(&signing(2)),
        }
        .sign(&signing(2)),
        delegation: None,
    }
}
fn revoke(device: PublicKey) -> WireRequest {
    let epoch = AccessListEpoch {
        version: FORMAT_VERSION,
        robot: ROBOT,
        epoch: 1,
        revoked_accounts: Vec::new(),
        revoked_devices: vec![device],
        issued_at_ns: NOW,
        issuer_key: public(&signing(2)),
    }
    .sign(&signing(2));
    WireRequest::SyncEpoch {
        epoch_postcard: hex::encode(
            EpochSyncWire::new(intermediate(), epoch)
                .to_postcard()
                .unwrap(),
        ),
    }
}
async fn endpoint(seed: u8) -> Endpoint {
    bounded(build_endpoint(
        EndpointConfig::new([seed; 32]).with_relay(RelayConfig::Disabled),
    ))
    .await
    .unwrap()
}

struct Robot {
    endpoint: Endpoint,
    address: EndpointAddr,
    shared: SharedTrust,
    manager: Arc<TransportManager>,
    accept: JoinHandle<()>,
}
impl Robot {
    async fn start(keys: &[PublicKey]) -> Self {
        let endpoint = endpoint(50).await;
        let address = direct_addr(
            endpoint.id(),
            endpoint
                .bound_sockets()
                .into_iter()
                .map(|socket| match socket {
                    SocketAddr::V4(v4) if v4.ip().is_unspecified() => {
                        SocketAddr::from((Ipv4Addr::LOCALHOST, v4.port()))
                    }
                    SocketAddr::V6(v6) if v6.ip().is_unspecified() => {
                        SocketAddr::from((Ipv6Addr::LOCALHOST, v6.port()))
                    }
                    other => other,
                }),
        );
        let roots = RootSet::new(vec![public(&signing(1))], 1).unwrap();
        let chassis = b"metadata-test-physical-claim-secret";
        let mut store = TrustStore::provision(
            ROBOT,
            PublicKey(*endpoint.id().as_bytes()),
            roots,
            chassis,
            0,
        )
        .unwrap();
        store
            .claim(OWNER, chassis, PrincipalKind::Human, NOW)
            .unwrap();
        let clock = RemotedClock::fixed(NOW);
        let shared = SharedTrust::new_with_clock(
            store,
            DeviceAccountIndex::new(),
            Vec::new(),
            clock.clone(),
        );
        for key in keys {
            shared
                .verify_and_establish(&presentation(*key), &key.0, None, NOW)
                .unwrap();
        }
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let manager = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!(
                    "metadata_{}_{}",
                    std::process::id(),
                    NEXT.fetch_add(1, Ordering::Relaxed)
                ),
                ..Default::default()
            },
            cerulion_core::testing::iceoryx_test_config(),
        )
        .unwrap();
        let authorizer = Arc::new(PairingAuthorizer::from_shared(shared.clone()));
        let plane = Arc::new(
            WirePlane::with_manager("metadata-robot", manager.clone())
                .with_demand_authorizer(authorizer.clone())
                .with_epoch_sink(shared.clone(), clock)
                .with_revocation_sweep_interval(Duration::from_millis(20))
                .with_schema(
                    BTreeMap::from([(TOPIC.to_string(), SCHEMA.to_string())]),
                    BTreeMap::from([(
                        SCHEMA.to_string(),
                        SchemaDoc {
                            qualified: SCHEMA.to_string(),
                            encoding: SchemaEncoding::Msg,
                            text: SCHEMA_TEXT.to_string(),
                            deps: Vec::new(),
                        },
                    )]),
                ),
        );
        let serving = endpoint.clone();
        let accept = tokio::spawn(async move {
            let mut sessions = JoinSet::new();
            while let Ok(Some(accepted)) = accept_one(&serving).await {
                let authorizer = authorizer.clone();
                let plane = plane.clone();
                sessions.spawn(async move {
                    handle_accepted_with_wire(accepted, authorizer, None, Some(plane)).await;
                });
            }
            while sessions.join_next().await.is_some() {}
        });
        Self {
            endpoint,
            address,
            shared,
            manager,
            accept,
        }
    }
    async fn shutdown(self) {
        bounded(self.endpoint.close()).await;
        bounded(self.accept).await.unwrap();
    }
}

struct Desk {
    connection: Connection,
    send: SendStream,
    recv: RecvStream,
}
impl Desk {
    async fn connect(endpoint: &Endpoint, robot: &Robot) -> Self {
        let connection = bounded(dial(endpoint, robot.address.clone(), alpn::WIRE))
            .await
            .unwrap();
        let (send, recv) = bounded(open_frame_stream(&connection)).await.unwrap();
        Self {
            connection,
            send,
            recv,
        }
    }
    async fn send(&mut self, request: &WireRequest) {
        bounded(write_frame(
            &mut self.send,
            &serde_json::to_vec(request).unwrap(),
        ))
        .await
        .unwrap();
    }
    async fn receive(&mut self) -> WireResponse {
        let bytes = bounded(read_frame(&mut self.recv, DEFAULT_MAX_FRAME_LEN))
            .await
            .unwrap();
        serde_json::from_slice(&bytes).unwrap()
    }
    async fn request(&mut self, request: &WireRequest) -> WireResponse {
        self.send(request).await;
        self.receive().await
    }
    async fn assert_allowed(&mut self) {
        match self.request(&WireRequest::Catalog).await {
            WireResponse::Catalog(reply) => assert_eq!(reply.robot, "metadata-robot"),
            other => panic!("authorized catalog was refused: {other:?}"),
        }
        match self
            .request(&WireRequest::Schema {
                topic: TOPIC.to_string(),
            })
            .await
        {
            WireResponse::Schema(reply) => {
                assert!(reply.error.is_none());
                assert_eq!(reply.docs.len(), 1);
                assert_eq!(reply.docs[0].qualified, SCHEMA);
                assert_eq!(reply.docs[0].text, SCHEMA_TEXT);
            }
            other => panic!("authorized schema was refused: {other:?}"),
        }
    }
    async fn assert_refused(&mut self) {
        for request in [
            WireRequest::Catalog,
            WireRequest::Schema {
                topic: TOPIC.to_string(),
            },
        ] {
            assert_revoked(self.request(&request).await);
        }
        match self.request(&WireRequest::Status).await {
            WireResponse::Status(status) => {
                assert!(status.topics.is_empty(), "test must remain demandless")
            }
            other => panic!("connection unexpectedly closed: {other:?}"),
        }
    }
    fn close(self) {
        self.connection.close(0u32.into(), b"test finished");
    }
}
fn assert_revoked(reply: WireResponse) {
    match reply {
        WireResponse::Error { message, .. } => {
            assert!(message.contains("device key is revoked"), "{message}")
        }
        other => panic!("revoked requester received metadata: {other:?}"),
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn self_revoking_epoch_refuses_subsequent_metadata_without_any_demand() {
    let endpoint = endpoint(60).await;
    let key = PublicKey(*endpoint.id().as_bytes());
    let robot = Robot::start(&[key]).await;
    let mut desk = Desk::connect(&endpoint, &robot).await;
    desk.assert_allowed().await;
    assert_eq!(
        desk.request(&revoke(key)).await,
        WireResponse::EpochSynced {
            epoch: 1,
            applied: true
        }
    );
    assert_eq!(
        robot.shared.snapshot_for_key(&key.0).1,
        KeyAccess::DeviceRevoked
    );
    desk.assert_refused().await;
    desk.assert_refused().await;
    desk.close();
    robot.shutdown().await;
    bounded(endpoint.close()).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn another_connection_revokes_an_already_healthy_metadata_only_connection() {
    let endpoint_a = endpoint(61).await;
    let endpoint_b = endpoint(62).await;
    let key_a = PublicKey(*endpoint_a.id().as_bytes());
    let key_b = PublicKey(*endpoint_b.id().as_bytes());
    let robot = Robot::start(&[key_a, key_b]).await;
    let mut a = Desk::connect(&endpoint_a, &robot).await;
    let mut b = Desk::connect(&endpoint_b, &robot).await;
    a.assert_allowed().await;
    b.assert_allowed().await;
    assert_eq!(
        b.request(&revoke(key_a)).await,
        WireResponse::EpochSynced {
            epoch: 1,
            applied: true
        }
    );
    a.assert_refused().await;
    b.assert_allowed().await;
    a.close();
    b.close();
    robot.shutdown().await;
    bounded(endpoint_a.close()).await;
    bounded(endpoint_b.close()).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn revocation_during_catalog_peeking_refuses_the_pending_catalog() {
    let endpoint_a = endpoint(63).await;
    let endpoint_b = endpoint(64).await;
    let key_a = PublicKey(*endpoint_a.id().as_bytes());
    let key_b = PublicKey(*endpoint_b.id().as_bytes());
    let robot = Robot::start(&[key_a, key_b]).await;
    let mut a = Desk::connect(&endpoint_a, &robot).await;
    let mut b = Desk::connect(&endpoint_b, &robot).await;
    a.assert_allowed().await;
    // A real silent publisher makes the catalog await its schema-header peek.
    // Observe the actual SHM subscriber before revoking; no timing-only sleep.
    let publisher = robot
        .manager
        .create_publisher_simple(TOPIC, MaxSliceLen::const_new(256))
        .unwrap();
    assert_eq!(robot.manager.topic_subscriber_count(TOPIC), 0);
    a.send(&WireRequest::Catalog).await;
    bounded(async {
        while robot.manager.topic_subscriber_count(TOPIC) == 0 {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    })
    .await;
    assert_eq!(
        b.request(&revoke(key_a)).await,
        WireResponse::EpochSynced {
            epoch: 1,
            applied: true
        }
    );
    assert_revoked(a.receive().await);
    drop(publisher);
    a.close();
    b.close();
    robot.shutdown().await;
    bounded(endpoint_a.close()).await;
    bounded(endpoint_b.close()).await;
}
