// SPDX-License-Identifier: AGPL-3.0-only

use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use cerulion_core::{
    GatewayEgressPolicy, GatewayPlan, SchemaDoc, SchemaEncoding, SchemaServing, TopicSchema,
    TransportManager,
};
use cerulion_link::{build_endpoint, Endpoint, EndpointConfig, RelayConfig};
use cerulion_netd::daemon::RunningNetd;
use cerulion_netd::egress::GatewayEgressPlane;
use cerulion_netd::query::{CatalogGather, QueryError, QueryPlane, RunsGather, SchemaGather};
use cerulion_netd::{MirrorError, MirrorPlane, MirrorRelease, NetdClient, NetdConfig, TopicKey};
use cerulion_pairing::client::DeviceIdentity;
use cerulion_pairing::format::{PrincipalKind, PublicKey};
use cerulion_pairing::verify::TrustStore;
use cerulion_remoted::{
    DeviceAccountIndex, OpsServing, PairingAuthorizer, RemotedClock, RemotedConfig, SharedTrust,
    WirePlane,
};

use super::account::Account;
use super::support::{manager, BOUND, DESK_SEED, ROBOT_SEED, TOPIC, TYPE};

const MAC: &[u8] = b"account-viz-test-only-store-mac";

struct NoRemoteQueries;
impl MirrorPlane for NoRemoteQueries {
    fn ensure_mirror(&self, _: &TopicKey, _: u64) -> Result<(), MirrorError> {
        panic!("robot schema provider must never create a mirror")
    }
    fn release_mirror(&self, _: &TopicKey) -> MirrorRelease {
        panic!("robot schema provider must never release a mirror")
    }
}
impl QueryPlane for NoRemoteQueries {
    fn query_catalog(&self, _: Option<&str>) -> Result<CatalogGather, QueryError> {
        panic!("account route must never query LAN")
    }
    fn query_schema(&self, _: Option<&str>, _: &str) -> Result<SchemaGather, QueryError> {
        panic!("account route must never query LAN")
    }
    fn query_runs(&self, _: Option<&str>) -> Result<RunsGather, QueryError> {
        panic!("fixture does not query runs")
    }
}

pub struct Robot {
    pub manager: Arc<TransportManager>,
    pub public_key: [u8; 32],
    pub direct: Vec<SocketAddr>,
    pub hash: u64,
    config: RemotedConfig,
    registration: Option<NetdClient>,
    gateway: RunningNetd,
    endpoint: Endpoint,
    runtime: Option<tokio::runtime::Runtime>,
    accept: tokio::task::JoinHandle<()>,
}
impl Robot {
    pub fn start(account: &Account, state_root: &Path, shm_root: &Path, socket: PathBuf) -> Self {
        let manager = manager(shm_root, "account_viz_robot");
        let egress = Arc::new(GatewayEgressPlane::new_without_mdns_for_test(
            manager.clone(),
        ));
        assert!(egress.boot_standing_gateway(&Default::default()).unwrap());
        let gateway = cerulion_netd::daemon::start_with_planes(
            socket.clone(),
            Arc::new(NoRemoteQueries),
            egress,
            Arc::new(NoRemoteQueries),
            NetdConfig::default(),
        )
        .unwrap();
        let doc = SchemaDoc {
            qualified: TYPE.into(),
            encoding: SchemaEncoding::Msg,
            text: "float64 x\nfloat64 y\n".into(),
            deps: vec![],
        };
        let hash = cerulion_viz::schema_registry::walker_with_store_and_docs(
            &[],
            std::slice::from_ref(&doc),
        )
        .schema_hash_for(TYPE)
        .unwrap();
        let mut registration = NetdClient::connect_existing_at_until(
            socket.clone(),
            std::time::Instant::now() + BOUND,
        )
        .unwrap();
        registration
            .register_egress(
                &GatewayPlan {
                    egress_policy: GatewayEgressPolicy::AllowAll,
                    announce: vec![TOPIC.into()],
                    ingress: vec![],
                },
                &SchemaServing {
                    topic_schemas: vec![TopicSchema {
                        topic: TOPIC.into(),
                        schema_name: TYPE.into(),
                    }],
                    schema_docs: vec![doc],
                    schema_hashes: vec![],
                },
                Some(&serde_json::to_string(&manager.iox_config()).unwrap()),
            )
            .unwrap();

        let config = RemotedConfig::from_state_root(state_root, RelayConfig::Disabled, false);
        std::fs::create_dir_all(config.store_file.parent().unwrap()).unwrap();
        std::fs::create_dir_all(&config.log_root).unwrap();
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .enable_all()
            .build()
            .unwrap();
        let endpoint = runtime.block_on(async {
            tokio::time::timeout(
                BOUND,
                build_endpoint(EndpointConfig::new(ROBOT_SEED).with_relay(RelayConfig::Disabled)),
            )
            .await
            .unwrap()
            .unwrap()
        });
        let public_key = *endpoint.id().as_bytes();
        let direct = endpoint
            .bound_sockets()
            .iter()
            .map(|address| {
                SocketAddr::new(
                    match address.ip() {
                        IpAddr::V4(_) => IpAddr::V4(Ipv4Addr::LOCALHOST),
                        IpAddr::V6(_) => IpAddr::V6(Ipv6Addr::LOCALHOST),
                    },
                    address.port(),
                )
            })
            .collect();
        let chassis = b"account-viz-test-only-physical-claim";
        let mut store = TrustStore::provision(
            account.robot,
            PublicKey(public_key),
            account.app.ca.root_set().clone(),
            chassis,
            account.now,
        )
        .unwrap()
        .with_path(&config.store_file);
        store
            .claim(account.owner, chassis, PrincipalKind::Human, account.now)
            .unwrap();
        store.save(MAC).unwrap();
        let index = DeviceAccountIndex::new().with_path(&config.index_file);
        index.save(MAC).unwrap();
        assert_eq!(
            index.account_for(&DeviceIdentity::from_seed(&DESK_SEED).public_key()),
            None
        );
        let clock = RemotedClock::fixed(account.now);
        let shared = SharedTrust::new_with_clock(store, index, MAC.to_vec(), clock.clone());
        let authorizer = Arc::new(PairingAuthorizer::from_shared(shared.clone()));
        let ops = Arc::new(
            OpsServing::new(
                shared.clone(),
                Arc::new(Mutex::new(cerud::lease::ControlLease::with_default_window())),
                cerulion_remoted::pairing_verbs::SharedCodePairSessions::new(),
                clock.clone(),
                &config.receipt_file,
                config.log_root.clone(),
                "account-viz-robot",
            )
            .unwrap(),
        );
        let wire = Arc::new(
            WirePlane::with_manager("loopback-robot", manager.clone())
                .with_local_schema_provider(socket)
                .with_demand_authorizer(authorizer.clone())
                .with_epoch_sink(shared, clock),
        );
        let serving = endpoint.clone();
        let accept = runtime.spawn(async move {
            cerulion_remoted::serve_with_wire(
                &serving,
                authorizer,
                Some(ops),
                Some(wire),
                std::future::pending(),
            )
            .await
            .unwrap();
        });
        Self {
            manager,
            public_key,
            direct,
            hash,
            config,
            registration: Some(registration),
            gateway,
            endpoint,
            runtime: Some(runtime),
            accept,
        }
    }

    pub fn assert_one_owner_pair(&self, account: &Account) {
        let index = DeviceAccountIndex::load(&self.config.index_file, MAC).unwrap();
        assert_eq!(
            index.account_for(&DeviceIdentity::from_seed(&DESK_SEED).public_key()),
            Some(account.owner)
        );
        let receipts: Vec<serde_json::Value> = std::fs::read_to_string(&self.config.receipt_file)
            .unwrap()
            .lines()
            .map(|line| serde_json::from_str(line).unwrap())
            .collect();
        assert_eq!(
            receipts
                .iter()
                .filter(|r| r["verb"] == "pair" && r["outcome"]["outcome"] == "ok")
                .count(),
            1
        );
    }
}
impl Drop for Robot {
    fn drop(&mut self) {
        let mut closed = true;
        if let Some(runtime) = self.runtime.take() {
            closed = runtime.block_on(async {
                let closed = tokio::time::timeout(BOUND, self.endpoint.close())
                    .await
                    .is_ok();
                let accepted = tokio::time::timeout(BOUND, &mut self.accept).await;
                let joined = matches!(accepted, Ok(Ok(())));
                if !joined {
                    self.accept.abort();
                }
                closed && joined
            });
            runtime.shutdown_timeout(std::time::Duration::from_secs(2));
        }
        self.registration.take();
        self.gateway.shutdown();
        if !std::thread::panicking() {
            assert!(
                closed,
                "robot endpoint and accept loop must stop within their bounds"
            );
        }
    }
}
