// SPDX-License-Identifier: AGPL-3.0-only
//! Which accepted connections hold the serving fence that keeps netd's idle
//! teardown off, through the production serving core `daemon::serve_endpoint`
//! over real iroh loopback. The fence is read the way netd reads it: an
//! exclusive `IdleFence::try_acquire` on `<state_root>/serving/serving.hold`
//! succeeds only while no connection, call or source holds the shared lock.
//!
//! Each test has its own state root, endpoints and SHM root; parallel-safe.

use std::future::Future;
use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr};
use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerud::client::OpsClient;
use cerud::error::CerudError;
use cerulion_core::{TransportConfig, TransportManager};
use cerulion_link::{
    alpn, build_endpoint, dial, direct_addr, open_frame_stream, read_frame, write_frame,
    Connection, ConnectionError, Endpoint, EndpointAddr, EndpointConfig, EndpointId, QuicOpsStream,
    RecvStream, RelayConfig, SendStream,
};
use cerulion_netd::serving_hold::IdleFence;
use cerulion_pairing::format::{AccountId, PrincipalKind, PublicKey, RobotId, RootSet};
use cerulion_pairing::verify::TrustStore;
use tokio::sync::oneshot;

use cerulion_remoted::serving_hold::fence_path;
use cerulion_remoted::{
    serve_endpoint, DeviceAccountIndex, RemotedClock, RemotedConfig, RemotedError,
};

const STEP: Duration = Duration::from_secs(20);
const T_NOW: u64 = 1_000_000_000_000;
const CHASSIS: &[u8] = b"serving-fence-chassis-secret-not-serial-derived";
const MAC_KEY: &[u8] = b"serving-fence-mac-key";
const OWNER: AccountId = AccountId([10; 32]);
/// The ops/1 hello: protocol version 1 only.
const HELLO: &[u8] = br#"{"min_version":1,"max_version":1}"#;

async fn bounded<F: Future>(what: &str, fut: F) -> F::Output {
    match tokio::time::timeout(STEP, fut).await {
        Ok(v) => v,
        Err(_) => panic!("serving fence admission: '{what}' timed out after {STEP:?}"),
    }
}

async fn disabled_endpoint(secret: [u8; 32]) -> Endpoint {
    bounded(
        "build_endpoint",
        build_endpoint(EndpointConfig::new(secret).with_relay(RelayConfig::Disabled)),
    )
    .await
    .expect("endpoint binds")
}

fn loopback_addr(id: EndpointId, bound: &[SocketAddr]) -> EndpointAddr {
    let dialable = bound.iter().map(|s| match s {
        SocketAddr::V4(v4) if v4.ip().is_unspecified() => {
            SocketAddr::from((Ipv4Addr::LOCALHOST, v4.port()))
        }
        SocketAddr::V6(v6) if v6.ip().is_unspecified() => {
            SocketAddr::from((Ipv6Addr::LOCALHOST, v6.port()))
        }
        other => *other,
    });
    direct_addr(id, dialable)
}

fn test_manager() -> Arc<TransportManager> {
    static N: AtomicU64 = AtomicU64::new(0);
    TransportManager::init_for_test(
        TransportConfig {
            node_name: format!(
                "serving_fence_{}_{}",
                std::process::id(),
                N.fetch_add(1, Ordering::Relaxed)
            ),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init_for_test")
}

/// A provisioned store persisted under `root`, claimed by [`OWNER`] when
/// `claimed`.
fn store(root: &Path, claimed: bool) -> TrustStore {
    let root_set = RootSet::new(vec![PublicKey([1; 32])], 1).unwrap();
    let mut store = TrustStore::provision(
        RobotId([5; 32]),
        PublicKey([6; 32]),
        root_set,
        CHASSIS,
        T_NOW,
    )
    .unwrap()
    .with_path(root.join("remoted/trust_store"));
    if claimed {
        store
            .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
            .unwrap();
    }
    store
}

/// Whether netd could take the exclusive fence now. A taken fence is released
/// before this returns.
fn fence_free(fence: &Path) -> bool {
    IdleFence::try_acquire(fence)
        .expect("the serving fence opens")
        .is_some()
}

/// Wait, bounded, until netd could take the exclusive fence. Only for a
/// release: the exclusive try races any admission made while it waits.
async fn fence_released(fence: &Path) {
    bounded("fence release", async {
        while !fence_free(fence) {
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await;
}

/// The daemon's serving core over `robot`, stopped by [`Daemon::stop`].
struct Daemon {
    stop: oneshot::Sender<()>,
    serving: tokio::task::JoinHandle<Result<(), RemotedError>>,
}

impl Daemon {
    fn start(root: &Path, robot: Endpoint, store: TrustStore, index: DeviceAccountIndex) -> Self {
        std::fs::create_dir_all(root.join("remoted")).unwrap();
        let config = RemotedConfig::from_state_root(root, RelayConfig::Disabled, false);
        std::fs::create_dir_all(&config.log_root).unwrap();
        let index = index.with_path(root.join("remoted/device_index.json"));
        let manager = test_manager();
        let (stop, stopped) = oneshot::channel::<()>();
        let serving = tokio::spawn(async move {
            let served = serve_endpoint(
                &config,
                store,
                index,
                MAC_KEY.to_vec(),
                &robot,
                RemotedClock::fixed(T_NOW),
                None,
                Some(manager),
                async {
                    let _ = stopped.await;
                },
            )
            .await;
            robot.close().await;
            served
        });
        Self { stop, serving }
    }

    async fn stop(self) {
        let _ = self.stop.send(());
        bounded("daemon stop", self.serving)
            .await
            .expect("serving task")
            .expect("serving ends cleanly");
    }
}

/// Write one ops/1 frame (big-endian length, then JSON) and read one back.
async fn ops_frame(
    send: &mut SendStream,
    recv: &mut RecvStream,
    payload: &[u8],
) -> serde_json::Value {
    bounded("ops frame", async {
        send.write_all(&(payload.len() as u32).to_be_bytes())
            .await
            .unwrap();
        send.write_all(payload).await.unwrap();
        let mut length = [0; 4];
        recv.read_exact(&mut length).await.unwrap();
        let mut bytes = vec![0; u32::from_be_bytes(length) as usize];
        recv.read_exact(&mut bytes).await.unwrap();
        serde_json::from_slice(&bytes).unwrap()
    })
    .await
}

/// Dial `plane` and open its stream; on the ops plane, complete the ops/1
/// handshake, which acks only once the robot serves the session.
async fn open(
    from: &Endpoint,
    to: EndpointAddr,
    plane: &[u8],
) -> (Connection, SendStream, RecvStream) {
    let connection = bounded("dial", dial(from, to, plane)).await.expect("dial");
    let (mut send, mut recv) = bounded("open stream", open_frame_stream(&connection))
        .await
        .expect("open stream");
    if plane == alpn::OPS {
        let ack = ops_frame(&mut send, &mut recv, HELLO).await;
        assert_eq!(ack["type"], "ack", "{ack}");
    }
    (connection, send, recv)
}

/// A bootstrap-only ops session and a refused rcp connection, each held open,
/// leave the fence free; a paired ops session holds it while it is open and
/// releases it when it closes, and so does a paired rcp connection once the
/// robot has ended its one stream.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn only_a_paired_session_holds_the_serving_fence() {
    let dir = tempfile::tempdir().unwrap();
    let fence = fence_path(dir.path());
    let robot = disabled_endpoint([0x61; 32]).await;
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());
    let paired = disabled_endpoint([0x62; 32]).await;
    let stranger = disabled_endpoint([0x63; 32]).await;
    let mut index = DeviceAccountIndex::new();
    index.bind(PublicKey(*paired.id().as_bytes()), OWNER);
    let daemon = Daemon::start(dir.path(), robot, store(dir.path(), true), index);

    // An unpaired key on the ops plane reaches the bootstrap surface: the
    // robot serves its session, and the session holds no fence.
    let (bootstrap, _bootstrap_send, _bootstrap_recv) =
        open(&stranger, robot_addr.clone(), alpn::OPS).await;
    assert!(
        fence_free(&fence),
        "a bootstrap-only ops session holds no fence"
    );

    // An unpaired key on the rcp plane is refused; its connection, held open
    // past the refusal, holds no fence either.
    let (refused, mut refused_send, mut refused_recv) =
        open(&stranger, robot_addr.clone(), alpn::RCP).await;
    bounded("rcp hello", write_frame(&mut refused_send, b"hello"))
        .await
        .expect("rcp hello");
    let decision = bounded("rcp decision", read_frame(&mut refused_recv, 65536))
        .await
        .expect("rcp decision");
    assert_eq!(
        decision,
        br#"{"decision":"refuse","reason":"rcp plane refused: unpaired device key"}"#
    );
    assert!(
        fence_free(&fence),
        "a refused rcp connection holds no fence"
    );

    let (session, session_send, session_recv) = open(&paired, robot_addr.clone(), alpn::OPS).await;
    assert!(!fence_free(&fence), "a paired ops session holds the fence");
    assert!(
        bootstrap.close_reason().is_none() && refused.close_reason().is_none(),
        "both unpaired connections stayed open throughout"
    );
    drop((session_send, session_recv));
    session.close(0u32.into(), b"done");
    fence_released(&fence).await;

    // A paired key on the rcp plane is admitted: the plane itself refuses this
    // frame, so the reply shows the stream was served. Once the robot has
    // ended that stream, the connection alone holds the fence until it closes.
    let (rcp, mut rcp_send, mut rcp_recv) = open(&paired, robot_addr, alpn::RCP).await;
    bounded(
        "rcp frame",
        write_frame(
            &mut rcp_send,
            br#"{"jsonrpc":"1.0","id":7,"method":"ping"}"#,
        ),
    )
    .await
    .expect("rcp frame");
    let reply = bounded("rcp reply", read_frame(&mut rcp_recv, 65536))
        .await
        .expect("rcp reply");
    assert_eq!(
        serde_json::from_slice::<serde_json::Value>(&reply).unwrap(),
        serde_json::json!({ "jsonrpc": "2.0", "id": 7,
            "error": { "code": -32600, "message": "refused: jsonrpc must be \"2.0\"" } })
    );
    rcp_send.finish().expect("finish the rcp stream");
    let end = bounded("rcp stream end", read_frame(&mut rcp_recv, 65536)).await;
    assert!(end.is_err(), "the robot ended the rcp stream: {end:?}");
    assert!(
        !fence_free(&fence),
        "a paired rcp connection holds the fence with no stream open"
    );
    rcp.close(0u32.into(), b"done");
    fence_released(&fence).await;

    bootstrap.close(0u32.into(), b"done");
    refused.close(0u32.into(), b"done");
    daemon.stop().await;
}

/// A key that claims the robot on a bootstrap-only session gains its
/// authority on the same connection, and each later call runs under the
/// fence: while netd's idle stop is committed the call is refused with the
/// stopping text, and once the stop is withdrawn it runs.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_key_claimed_on_a_bootstrap_session_runs_its_later_calls_under_the_fence() {
    let dir = tempfile::tempdir().unwrap();
    let fence = fence_path(dir.path());
    let robot = disabled_endpoint([0x64; 32]).await;
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());
    let desk = disabled_endpoint([0x65; 32]).await;
    let daemon = Daemon::start(
        dir.path(),
        robot,
        store(dir.path(), false),
        DeviceAccountIndex::new(),
    );

    let connection = bounded("dial", dial(&desk, robot_addr, alpn::OPS))
        .await
        .expect("dial");
    let (send, recv) = bounded("open stream", open_frame_stream(&connection))
        .await
        .expect("open stream");
    let (idle_committed, claimed, refused, served) = bounded(
        "ops calls",
        tokio::task::spawn_blocking(move || {
            let mut client =
                OpsClient::connect_current(QuicOpsStream::new(send, recv)).expect("handshake");
            // The unclaimed robot admits this key to the bootstrap surface only,
            // so its session holds no fence and netd's idle stop can commit.
            let idle = IdleFence::try_acquire(&fence).expect("the serving fence opens");
            let idle_committed = idle.is_some();
            let claimed = client.call(
                "claim",
                serde_json::json!({
                    "account": hex::encode(OWNER.0),
                    "chassis_secret": hex::encode(CHASSIS),
                }),
            );
            let refused = client.call("inventory", serde_json::json!({}));
            drop(idle);
            let served = client.call("inventory", serde_json::json!({}));
            (idle_committed, claimed, refused, served)
        }),
    )
    .await
    .expect("ops task");

    assert!(idle_committed, "a bootstrap-only session holds no fence");
    let claimed = claimed.expect("claim with the chassis secret");
    assert_eq!(claimed["claimed"], true, "{claimed}");
    match refused {
        Err(CerudError::Remote { kind, message }) => {
            assert_eq!(kind, "verb_error");
            assert_eq!(
                message,
                "verb error: unavailable: the remote server is stopping, so inventory is refused"
            );
        }
        other => panic!("inventory during a committed idle stop must refuse, got {other:?}"),
    }
    let served = served.expect("inventory runs once the idle stop is withdrawn");
    assert!(served["arch"].is_string(), "{served}");

    connection.close(0u32.into(), b"done");
    daemon.stop().await;
}

/// While netd's idle stop is committed, a paired ops session is closed at
/// admission with the reason `remote server is stopping`, and a bootstrap-only
/// session, which takes no fence, is still served; once the stop is withdrawn
/// the paired key is admitted and holds the fence again.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_committed_idle_stop_closes_a_paired_session_and_still_serves_bootstrap() {
    let dir = tempfile::tempdir().unwrap();
    let fence = fence_path(dir.path());
    let robot = disabled_endpoint([0x66; 32]).await;
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());
    let paired = disabled_endpoint([0x67; 32]).await;
    let stranger = disabled_endpoint([0x68; 32]).await;
    let mut index = DeviceAccountIndex::new();
    index.bind(PublicKey(*paired.id().as_bytes()), OWNER);
    let daemon = Daemon::start(dir.path(), robot, store(dir.path(), true), index);

    let idle = IdleFence::try_acquire(&fence)
        .expect("the serving fence opens")
        .expect("no work holds the fence");
    let refused = bounded("dial paired", dial(&paired, robot_addr.clone(), alpn::OPS))
        .await
        .expect("dial paired");
    let closed = bounded("admission close", refused.closed()).await;
    match closed {
        ConnectionError::ApplicationClosed(close) => {
            assert_eq!(&close.reason[..], b"remote server is stopping");
        }
        other => panic!("a paired session must be closed at admission, got {other:?}"),
    }
    let (bootstrap, _bootstrap_send, _bootstrap_recv) =
        open(&stranger, robot_addr.clone(), alpn::OPS).await;
    drop(idle);

    let (session, session_send, session_recv) = open(&paired, robot_addr, alpn::OPS).await;
    assert!(
        !fence_free(&fence),
        "the admitted paired session holds the fence"
    );
    drop((session_send, session_recv));
    session.close(0u32.into(), b"done");
    bootstrap.close(0u32.into(), b"done");
    daemon.stop().await;
}
