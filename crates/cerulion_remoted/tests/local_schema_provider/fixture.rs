// SPDX-License-Identifier: AGPL-3.0-only
//! Real loopback wire server and isolated local gateway for provider tests.

use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use super::common::{
    bounded, disabled_endpoint, loopback_addr, pk, sk, wide, OWNER, ROBOT_ID, T_NOW,
};
use cerulion_core::testing::IsolatedRoot;
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::{TransportConfig, TransportManager};
use cerulion_link::{
    accept_one, alpn, dial, open_frame_stream, read_frame, write_frame, Connection, Endpoint,
    EndpointAddr, RecvStream, SendStream, DEFAULT_MAX_FRAME_LEN,
};
use cerulion_netd::daemon::RunningNetd;
use cerulion_netd::egress::GatewayEgressPlane;
use cerulion_netd::query::{CatalogGather, QueryError, QueryPlane, RunsGather, SchemaGather};
use cerulion_netd::{MirrorError, MirrorPlane, MirrorRelease, NetdClient, NetdConfig, TopicKey};
use cerulion_pairing::format::{
    AccessListEpoch, IntermediateCert, PrincipalKind, PublicKey, RootSet, Scope, FORMAT_VERSION,
};
use cerulion_pairing::verify::{EpochSyncWire, TrustStore};
use cerulion_remoted::{
    handle_accepted_with_wire, DeviceAccountIndex, PairingAuthorizer, RemotedClock, SharedTrust,
    WirePlane, WireRequest, WireResponse,
};
use iceoryx2::prelude::{FileName, SemanticString};
use tokio::task::{JoinHandle, JoinSet};

pub const ROBOT_NAME: &str = "schema-robot";

static ROOT: OnceLock<IsolatedRoot> = OnceLock::new();
static LIVE_USES: Mutex<usize> = Mutex::new(0);

// Bind first in each test so every transport handle drops before the last holder
// removes the process's root. Later tests recreate the same directory on acquire.
pub struct RootUse;

impl RootUse {
    pub fn acquire() -> Self {
        let mut live = LIVE_USES.lock().unwrap_or_else(|error| error.into_inner());
        let root = ROOT.get_or_init(|| IsolatedRoot::mint("s"));
        std::fs::create_dir_all(&root.root).unwrap();
        *live += 1;
        Self
    }
}

impl Drop for RootUse {
    fn drop(&mut self) {
        let mut live = LIVE_USES.lock().unwrap_or_else(|error| error.into_inner());
        *live -= 1;
        if *live == 0 {
            std::fs::remove_dir_all(&ROOT.get().unwrap().root).unwrap();
        }
    }
}

pub fn manager(network: bool) -> (RootUse, Arc<TransportManager>) {
    static NEXT: AtomicU64 = AtomicU64::new(0);
    let root = RootUse::acquire();
    let isolated = ROOT.get().unwrap();
    let mut iox_config = isolated.config();
    let n = NEXT.fetch_add(1, Ordering::Relaxed);
    // Listener socket paths include the root, prefix and ID within a 108-byte budget.
    let prefix = format!("{}{n}_", isolated.prefix);
    iox_config.global.prefix = FileName::new(prefix.as_bytes()).unwrap();
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("schema_{}_{}", std::process::id(), n),
            network: network.then(|| NetworkConfig {
                listen_endpoints: vec!["tcp/127.0.0.1:0".into()],
                robot_identity: Some(ROBOT_NAME.into()),
                ..Default::default()
            }),
            ..Default::default()
        },
        iox_config,
    )
    .unwrap();
    (root, manager)
}

struct NoRemoteQueries;
impl MirrorPlane for NoRemoteQueries {
    fn ensure_mirror(&self, _: &TopicKey, _: u64) -> Result<(), MirrorError> {
        panic!("schema source must never create a remote mirror")
    }
    fn release_mirror(&self, _: &TopicKey) -> MirrorRelease {
        panic!("schema source must never own a remote mirror")
    }
}
impl QueryPlane for NoRemoteQueries {
    fn query_catalog(&self, _: Option<&str>) -> Result<CatalogGather, QueryError> {
        panic!("schema source must not query the network")
    }
    fn query_schema(&self, _: Option<&str>, _: &str) -> Result<SchemaGather, QueryError> {
        panic!("schema source must not query the network")
    }
    fn query_runs(&self, _: Option<&str>) -> Result<RunsGather, QueryError> {
        panic!("schema source must not query the network")
    }
}

pub struct LocalNetd {
    pub manager: Arc<TransportManager>,
    pub path: PathBuf,
    pub plane: Arc<GatewayEgressPlane>,
    daemon: RunningNetd,
    _directory: tempfile::TempDir,
    _root: RootUse,
}
impl LocalNetd {
    pub fn start() -> Self {
        let (root, manager) = manager(true);
        let plane = Arc::new(GatewayEgressPlane::new_without_mdns_for_test(
            manager.clone(),
        ));
        assert!(plane.boot_standing_gateway(&Default::default()).unwrap());
        let directory = tempfile::tempdir_in("/tmp").unwrap();
        let path = directory.path().join("netd.sock");
        let daemon = cerulion_netd::daemon::start_with_planes(
            path.clone(),
            Arc::new(NoRemoteQueries),
            plane.clone(),
            Arc::new(NoRemoteQueries),
            NetdConfig::default(),
        )
        .unwrap();
        Self {
            manager,
            path,
            plane,
            daemon,
            _directory: directory,
            _root: root,
        }
    }
    pub fn client(&self) -> NetdClient {
        NetdClient::connect_existing_at_until(
            self.path.clone(),
            Instant::now() + Duration::from_secs(5),
        )
        .unwrap()
    }
    pub fn shutdown(mut self) {
        self.daemon.shutdown();
    }
}

pub struct Robot {
    endpoint: Endpoint,
    address: EndpointAddr,
    accept: JoinHandle<()>,
}
impl Robot {
    pub async fn start(
        manager: Arc<TransportManager>,
        socket: PathBuf,
        keys: &[PublicKey],
    ) -> Self {
        let endpoint = disabled_endpoint([71; 32]).await;
        let address = loopback_addr(endpoint.id(), &endpoint.bound_sockets());
        let chassis = b"schema-test-physical-claim-secret";
        let mut store = TrustStore::provision(
            ROBOT_ID,
            PublicKey(*endpoint.id().as_bytes()),
            RootSet::new(vec![pk(&sk(1))], 1).unwrap(),
            chassis,
            0,
        )
        .unwrap();
        store
            .claim(OWNER, chassis, PrincipalKind::Human, T_NOW)
            .unwrap();
        // Seed an already-established owner pairing; the authenticated endpoint
        // key is still checked by the real accept and per-request authorizers.
        let mut index = DeviceAccountIndex::new();
        for key in keys {
            index.bind(*key, OWNER);
        }
        let clock = RemotedClock::fixed(T_NOW);
        let shared = SharedTrust::new_with_clock(store, index, Vec::new(), clock.clone());
        let authorizer = Arc::new(PairingAuthorizer::from_shared(shared.clone()));
        let plane = Arc::new(
            WirePlane::with_manager(ROBOT_NAME, manager)
                .with_local_schema_provider(socket)
                .with_demand_authorizer(authorizer.clone())
                .with_epoch_sink(shared, clock),
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
            accept,
        }
    }
    pub async fn shutdown(self) {
        bounded("robot close", self.endpoint.close()).await;
        bounded("robot accept exit", self.accept).await.unwrap();
    }
}

pub struct Desk {
    connection: Connection,
    send: SendStream,
    recv: RecvStream,
}
impl Desk {
    pub async fn connect(endpoint: &Endpoint, robot: &Robot) -> Self {
        let connection = bounded("dial", dial(endpoint, robot.address.clone(), alpn::WIRE))
            .await
            .unwrap();
        let (send, recv) = bounded("open control", open_frame_stream(&connection))
            .await
            .unwrap();
        Self {
            connection,
            send,
            recv,
        }
    }
    pub async fn send(&mut self, request: &WireRequest) {
        bounded(
            "send control",
            write_frame(&mut self.send, &serde_json::to_vec(request).unwrap()),
        )
        .await
        .unwrap();
    }
    pub async fn receive(&mut self) -> WireResponse {
        let bytes = bounded(
            "receive control",
            read_frame(&mut self.recv, DEFAULT_MAX_FRAME_LEN),
        )
        .await
        .unwrap();
        serde_json::from_slice(&bytes).unwrap()
    }
    pub async fn request(&mut self, request: &WireRequest) -> WireResponse {
        self.send(request).await;
        self.receive().await
    }
    pub fn close(self) {
        self.connection.close(0u32.into(), b"test finished");
    }
}

pub fn revoke(device: PublicKey) -> WireRequest {
    let intermediate = IntermediateCert {
        version: FORMAT_VERSION,
        intermediate_key: pk(&sk(2)),
        validity: wide(),
        issued_at_ns: 1,
        max_scope: Scope::OWNER_FULL,
    }
    .sign_by_roots(&[&sk(1)]);
    let epoch = AccessListEpoch {
        version: FORMAT_VERSION,
        robot: ROBOT_ID,
        epoch: 1,
        revoked_accounts: vec![],
        revoked_devices: vec![device],
        issued_at_ns: T_NOW,
        issuer_key: pk(&sk(2)),
    }
    .sign(&sk(2));
    WireRequest::SyncEpoch {
        epoch_postcard: hex::encode(
            EpochSyncWire::new(intermediate, epoch)
                .to_postcard()
                .unwrap(),
        ),
    }
}
