// SPDX-License-Identifier: AGPL-3.0-only
//! Arming a pairing code from a request file through `daemon::serve_endpoint`,
//! the test entry over `serve_core`, the serving core `daemon::run` reaches
//! through `RemotePlane::serve`:
//!
//! - a request a step on the robot leaves in `<root>/remoted/` is armed by the
//!   daemon's own poller, its result names the request's nonce and this robot's
//!   endpoint id and never the code, one desk then pairs with the code over the
//!   real ops plane (the code-paired row read back from disk), the result is
//!   written again as `paired` with that desk's account and label, the wire
//!   plane's `status` reports the code as paired, and a second desk is refused
//!   because the code is single use;
//! - three desks with a wrong code, each seeing the mismatch itself and
//!   dropping its connection, use the shared tries up: the result reads
//!   `exhausted` with 3 of 3 failed, the code keeps `serving.hold` taken past
//!   that result (it serves to its window plus the margin, which the
//!   in-module tests pin on a fixed clock), and a fourth desk with the right
//!   code reaches the robot and is refused;
//! - a claimed request the daemon refuses is answered `refused` with its
//!   reason, it replaces the code an earlier request armed, and the wire
//!   `status` counts it;
//! - the daemon's stop, on a stop request and when the endpoint closes on its
//!   own, withdraws a code no desk paired with and the result says `refused`
//!   with the reason `stopping`;
//! - while a code is armed and no paired connection is open, the code's own
//!   hold keeps `serving.hold` taken; an ops session of a desk that is not
//!   paired, opened before the code, while it is live or after its end, holds
//!   nothing of its own, so once a desk pairs with the code the fence is free
//!   with those sessions still open;
//! - once the serve has ended, no poller is left to claim a request;
//! - handoff files an earlier run left, a request among them, are removed
//!   before the ops plane answers, and the code they carried is never armed.
//!
//! Real iroh loopback (relay disabled, direct dial, every await bounded) and a
//! per-test iceoryx2 SHM root injected into `serve_endpoint`; parallel-safe.
#![cfg(unix)]

use std::future::Future;
use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr};
use std::os::unix::fs::PermissionsExt as _;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerud::client::OpsClient;
use cerud::error::CerudError;
use cerulion_core::{TransportConfig, TransportManager};
use cerulion_link::{
    alpn, build_endpoint, dial, direct_addr, open_frame_stream, read_frame, write_frame,
    Connection, Endpoint, EndpointAddr, EndpointConfig, EndpointId, QuicOpsStream, RelayConfig,
    DEFAULT_MAX_FRAME_LEN,
};
use cerulion_netd::serving_hold::IdleFence;
use cerulion_pairing::format::{AccountId, PrincipalKind, PublicKey, RobotId, RootSet};
use cerulion_pairing::pair_code::{
    PairCodeNonce, PairCodeRefusal, PairCodeRequest, PairCodeResult, PairCodeState, PairCodeStatus,
    PairedDesk, CLAIMED_FILE, PAIR_CODE_POLICY, REQUEST_FILE, REQUEST_STAGING_PREFIX, RESULT_FILE,
};
use cerulion_pairing::pake::{CpaceInitiator, PakeIdentities};
use cerulion_pairing::verify::{PairingSource, TrustStore};
use tokio::sync::oneshot;

use cerulion_remoted::{serve_endpoint, DeviceAccountIndex, RemotedClock, RemotedConfig};

const T_NOW: u64 = 1_000_000_000_000;
const CHASSIS: &[u8] = b"pair-code-request-chassis-secret-not-serial-derived";
const MAC_KEY: &[u8] = b"pair-code-request-mac-key";
const OWNER: AccountId = AccountId([10; 32]);
const ROBOT: RobotId = RobotId([0x0B; 32]);
const NONCE: PairCodeNonce = PairCodeNonce::from_bytes([0x5A; 16]);
const CODE: &str = "60428135";
const WRONG_CODE: &str = "11111111";
const TRY_THROTTLED: &str = cerulion_remoted::pairing_verbs::TRY_THROTTLED;

fn unique_id() -> String {
    static N: AtomicU64 = AtomicU64::new(0);
    format!(
        "{}_{}",
        std::process::id(),
        N.fetch_add(1, Ordering::Relaxed)
    )
}

async fn bounded<F: Future>(what: &str, fut: F) -> F::Output {
    match tokio::time::timeout(Duration::from_secs(20), fut).await {
        Ok(v) => v,
        Err(_) => panic!("pair-code request e2e: '{what}' timed out"),
    }
}

async fn disabled_endpoint(secret: [u8; 32]) -> Endpoint {
    build_endpoint(EndpointConfig::new(secret).with_relay(RelayConfig::Disabled))
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

fn test_manager(tag: &str) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("pair_code_{tag}_{}", unique_id()),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init_for_test")
}

/// A claimed robot's state under `root`: the remoted directory 0700, and a
/// store whose transport key is the robot endpoint's, saved there.
fn robot_state(
    root: &Path,
    robot_key: [u8; 32],
) -> (RemotedConfig, TrustStore, DeviceAccountIndex) {
    let remoted = root.join("remoted");
    std::fs::create_dir_all(&remoted).unwrap();
    std::fs::set_permissions(&remoted, std::fs::Permissions::from_mode(0o700)).unwrap();
    let config = RemotedConfig::from_state_root(root, RelayConfig::Disabled, false);
    std::fs::create_dir_all(&config.log_root).unwrap();
    let root_key = ed25519_dalek::SigningKey::from_bytes(&[0x11; 32]).verifying_key();
    let root_set = RootSet::new(vec![PublicKey(root_key.to_bytes())], 1).unwrap();
    let mut store = TrustStore::provision(ROBOT, PublicKey(robot_key), root_set, CHASSIS, 0)
        .unwrap()
        .with_path(&config.store_file);
    store
        .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    store.save(MAC_KEY).unwrap();
    let index = DeviceAccountIndex::new().with_path(&config.index_file);
    (config, store, index)
}

/// Serve the daemon over `robot` until the returned sender fires.
fn serve(
    config: RemotedConfig,
    store: TrustStore,
    index: DeviceAccountIndex,
    robot: Endpoint,
    tag: &str,
) -> (oneshot::Sender<()>, tokio::task::JoinHandle<()>) {
    let manager = test_manager(tag);
    let (shutdown_tx, shutdown_rx) = oneshot::channel::<()>();
    let serving = tokio::spawn(async move {
        let _ = serve_endpoint(
            &config,
            store,
            index,
            MAC_KEY.to_vec(),
            &robot,
            RemotedClock::fixed(T_NOW),
            None,
            Some(manager),
            async {
                let _ = shutdown_rx.await;
            },
        )
        .await;
    });
    (shutdown_tx, serving)
}

/// Leave a request as the step on the robot does: a 0600 staging file renamed
/// into place.
fn leave_request(remoted: &Path) -> PathBuf {
    let staging = remoted.join(format!("{REQUEST_STAGING_PREFIX}1.tmp"));
    std::fs::write(
        &staging,
        PairCodeRequest::new(NONCE, CODE).unwrap().render(),
    )
    .unwrap();
    std::fs::set_permissions(&staging, std::fs::Permissions::from_mode(0o600)).unwrap();
    let request = remoted.join(REQUEST_FILE);
    std::fs::rename(&staging, &request).unwrap();
    request
}

/// Assert `held` stays true over `watch`, checked every 50 ms.
async fn held_for(what: &str, watch: Duration, mut held: impl FnMut() -> bool) {
    let end = Instant::now() + watch;
    while Instant::now() < end {
        assert!(held(), "{what}: not held over {watch:?}");
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
}

async fn until(what: &str, mut done: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(10);
    while !done() {
        assert!(Instant::now() < deadline, "{what}: not within 10 s");
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
}

/// On one ops connection from `desk`, run a CPace ceremony with `code`:
/// `code-pair-start`, and, when it answers, `code-pair-finish`.
async fn pair_with_code(
    desk: &Endpoint,
    robot: EndpointAddr,
    robot_key: [u8; 32],
    code: &'static str,
) -> Result<serde_json::Value, CerudError> {
    let desk_key = *desk.id().as_bytes();
    let conn = bounded("dial ops", dial(desk, robot, alpn::OPS))
        .await
        .expect("dial ops");
    let (send, recv) = bounded("open ops stream", open_frame_stream(&conn))
        .await
        .expect("open ops stream");
    let result = bounded(
        "code pairing",
        tokio::task::spawn_blocking(move || {
            let mut client =
                OpsClient::connect_current(QuicOpsStream::new(send, recv)).expect("handshake");
            let initiator = CpaceInitiator::new(
                code,
                PakeIdentities::new(PublicKey(desk_key), PublicKey(robot_key), Vec::new()),
            );
            let (attempt, msg1) = initiator.begin().expect("cpace begin");
            let start = client.call(
                "code-pair-start",
                serde_json::json!({ "msg1": hex::encode(msg1) }),
            )?;
            let msg2 = hex::decode(start["msg2"].as_str().unwrap()).unwrap();
            let confirm = hex::decode(start["responder_confirm"].as_str().unwrap()).unwrap();
            let (_keys, initiator_confirm) = attempt
                .finish(&msg2, &confirm)
                .expect("the desk agrees with the robot on the code");
            client.call(
                "code-pair-finish",
                serde_json::json!({
                    "initiator_confirm": hex::encode(initiator_confirm),
                    "account": hex::encode(desk_key),
                    "name": "guest",
                }),
            )
        }),
    )
    .await
    .expect("ops task");
    conn.close(0u32.into(), b"done");
    result
}

/// On one ops connection from `desk`, a `code-pair-start` with a wrong code:
/// the desk sees the mismatch itself, sends no finish and closes the
/// connection. A start the robot refused answers its refusal.
async fn try_wrong_code(
    desk: &Endpoint,
    robot: EndpointAddr,
    robot_key: [u8; 32],
) -> Result<(), CerudError> {
    let desk_key = *desk.id().as_bytes();
    let conn = bounded("dial ops", dial(desk, robot, alpn::OPS))
        .await
        .expect("dial ops");
    let (send, recv) = bounded("open ops stream", open_frame_stream(&conn))
        .await
        .expect("open ops stream");
    let result = bounded(
        "wrong-code try",
        tokio::task::spawn_blocking(move || {
            let mut client =
                OpsClient::connect_current(QuicOpsStream::new(send, recv)).expect("handshake");
            let initiator = CpaceInitiator::new(
                WRONG_CODE,
                PakeIdentities::new(PublicKey(desk_key), PublicKey(robot_key), Vec::new()),
            );
            let (attempt, msg1) = initiator.begin().expect("cpace begin");
            let start = client.call(
                "code-pair-start",
                serde_json::json!({ "msg1": hex::encode(msg1) }),
            )?;
            let msg2 = hex::decode(start["msg2"].as_str().unwrap()).unwrap();
            let confirm = hex::decode(start["responder_confirm"].as_str().unwrap()).unwrap();
            assert!(
                attempt.finish(&msg2, &confirm).is_err(),
                "the desk sees the wrong code"
            );
            Ok(())
        }),
    )
    .await
    .expect("ops task");
    conn.close(0u32.into(), b"wrong code");
    result
}

/// [`try_wrong_code`] once the robot admits it: a start refused as throttled
/// is tried again.
async fn wrong_code_admitted(desk: &Endpoint, robot: EndpointAddr, robot_key: [u8; 32]) {
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        match try_wrong_code(desk, robot.clone(), robot_key).await {
            Ok(()) => return,
            Err(CerudError::Remote { message, .. }) if message.contains(TRY_THROTTLED) => {
                assert!(Instant::now() < deadline, "throttled for 20 s");
                tokio::time::sleep(Duration::from_millis(250)).await;
            }
            Err(other) => panic!("a wrong-code try was refused: {other:?}"),
        }
    }
}

/// An ops session from `desk`, its handshake done; it stays open until the
/// returned connection is closed.
async fn open_ops_session(
    desk: &Endpoint,
    robot: EndpointAddr,
) -> (Connection, OpsClient<QuicOpsStream>) {
    let conn = bounded("dial ops", dial(desk, robot, alpn::OPS))
        .await
        .expect("dial ops");
    let (send, recv) = bounded("open ops stream", open_frame_stream(&conn))
        .await
        .expect("open ops stream");
    let client = bounded(
        "ops handshake",
        tokio::task::spawn_blocking(move || {
            OpsClient::connect_current(QuicOpsStream::new(send, recv)).expect("handshake")
        }),
    )
    .await
    .expect("ops task");
    (conn, client)
}

/// Whether nothing holds the robot serving: the idle fence can be taken at
/// once. It is let go again before this returns.
fn fence_free(fence: &Path) -> bool {
    IdleFence::try_acquire(fence).unwrap().is_some()
}

/// The robot's wire-plane `status` reply, asked by `desk`.
async fn wire_status(desk: &Endpoint, robot: EndpointAddr) -> serde_json::Value {
    let conn = bounded("dial wire", dial(desk, robot, alpn::WIRE))
        .await
        .expect("dial wire");
    let (mut send, mut recv) = bounded("open wire stream", open_frame_stream(&conn))
        .await
        .expect("open wire stream");
    bounded(
        "send status",
        write_frame(&mut send, br#"{"verb":"status"}"#),
    )
    .await
    .expect("send status");
    let frame = bounded("status reply", read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN))
        .await
        .expect("status reply");
    conn.close(0u32.into(), b"done");
    serde_json::from_slice(&frame).expect("a JSON reply")
}

/// The result file's line, parsed.
fn result_in(remoted: &Path) -> Option<PairCodeResult> {
    std::fs::read(remoted.join(RESULT_FILE))
        .ok()
        .map(|bytes| PairCodeResult::parse(&bytes).expect("a well-formed result"))
}

fn assert_refused(result: Result<serde_json::Value, CerudError>, needle: &str) {
    match result {
        Err(CerudError::Remote { kind, message }) => {
            assert_eq!(kind, "verb_error", "{message}");
            assert!(message.contains(needle), "{message}");
        }
        other => panic!("expected a refusal containing {needle:?}, got {other:?}"),
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_request_left_on_the_robot_arms_a_code_one_desk_pairs_with_over_the_ops_plane() {
    let robot_ep = disabled_endpoint([0x61; 32]).await;
    let robot_key = *robot_ep.id().as_bytes();
    let robot_addr = loopback_addr(robot_ep.id(), &robot_ep.bound_sockets());
    let root = tempfile::tempdir().unwrap();
    let (config, store, index) = robot_state(root.path(), robot_key);
    let store_file = config.store_file.clone();
    let remoted = root.path().join("remoted");
    let earlier = remoted.join("pair-code.request.withdrawn.1");
    std::fs::write(&earlier, b"left").unwrap();
    let (shutdown_tx, serving) = serve(config, store, index, robot_ep, "arm");
    // Once the ops plane answers, the daemon has removed the file left before
    // its start, and nothing is armed yet.
    let probe = disabled_endpoint([0x64; 32]).await;
    assert_refused(
        pair_with_code(&probe, robot_addr.clone(), robot_key, CODE).await,
        "no code pairing is armed",
    );
    assert!(!earlier.exists(), "removed before the daemon serves");

    // A request left now is this run's to arm.
    let request = leave_request(&remoted);
    let result_file = remoted.join(RESULT_FILE);
    until("the daemon answers the request", || result_file.exists()).await;
    let bytes = std::fs::read(&result_file).unwrap();
    let armed = PairCodeResult::parse(&bytes).expect("a well-formed result");
    assert_eq!(armed.nonce(), Some(NONCE));
    assert_eq!(armed.endpoint_id(), &robot_key);
    assert_eq!(
        (armed.status(), armed.ttl_s(), armed.tries(), armed.failed()),
        (PairCodeStatus::Armed, 600, 3, 0)
    );
    assert!(
        !String::from_utf8_lossy(&bytes).contains(CODE),
        "the result never carries the code"
    );
    assert!(!request.exists() && !remoted.join(CLAIMED_FILE).exists());

    // One desk pairs with the code; the row is on disk.
    let desk_a = disabled_endpoint([0x62; 32]).await;
    let desk_a_key = *desk_a.id().as_bytes();
    let paired = pair_with_code(&desk_a, robot_addr.clone(), robot_key, CODE)
        .await
        .expect("the armed code pairs desk A");
    assert_eq!(paired["source"], "code_paired");
    assert_eq!(paired["account"], hex::encode(desk_a_key));
    let on_disk = TrustStore::load(&store_file, MAC_KEY).unwrap();
    let row = on_disk
        .account_access(&AccountId(desk_a_key), T_NOW)
        .expect("desk A's row on disk");
    assert_eq!(row.source, PairingSource::CodePaired);
    // The result says so, naming desk A's account and label.
    let paired = PairCodeResult::new(
        NONCE,
        robot_key,
        600,
        3,
        0,
        PairCodeState::Paired(PairedDesk::new(desk_a_key, "guest").unwrap()),
    )
    .unwrap();
    until("the result reads paired", || {
        result_in(&remoted).as_ref() == Some(&paired)
    })
    .await;
    // Desk A, now paired, reads the code's state in the wire `status`.
    let status = wire_status(&desk_a, robot_addr.clone()).await;
    assert_eq!(
        status["pair_code"],
        serde_json::json!({"status": "paired", "tries": 3, "failed": 0}),
        "{status}"
    );

    // The code is single use: a second desk with the same code is refused.
    let desk_b = disabled_endpoint([0x63; 32]).await;
    assert_refused(
        pair_with_code(&desk_b, robot_addr, robot_key, CODE).await,
        "this pairing code was already used by a desk",
    );

    let _ = shutdown_tx.send(());
    let _ = bounded("serve join", serving).await;

    // No poller outlives the serve: a request left now stays.
    let request = leave_request(&remoted);
    tokio::time::sleep(Duration::from_millis(2_500)).await;
    assert!(request.exists(), "no pair-code poller survived the serve");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn only_the_armed_code_holds_serving_for_desks_that_are_not_paired() {
    let robot_ep = disabled_endpoint([0x91; 32]).await;
    let robot_key = *robot_ep.id().as_bytes();
    let robot_addr = loopback_addr(robot_ep.id(), &robot_ep.bound_sockets());
    let root = tempfile::tempdir().unwrap();
    let (config, store, index) = robot_state(root.path(), robot_key);
    let remoted = root.path().join("remoted");
    let fence = cerulion_remoted::serving_hold::fence_path(root.path());
    let (shutdown_tx, serving) = serve(config, store, index, robot_ep, "hold");
    let probe = disabled_endpoint([0x92; 32]).await;
    assert_refused(
        pair_with_code(&probe, robot_addr.clone(), robot_key, CODE).await,
        "no code pairing is armed",
    );

    // With no code armed, an open session of a desk that is not paired holds
    // nothing: the robot can be stopped for idle while it is open.
    let before = disabled_endpoint([0x93; 32]).await;
    let (before_conn, before_client) = open_ops_session(&before, robot_addr.clone()).await;
    assert!(
        fence_free(&fence),
        "a session of a desk that is not paired holds no serving"
    );

    // The armed code holds serving, and no paired connection is open.
    leave_request(&remoted);
    until("the daemon arms the code", || {
        result_in(&remoted).is_some_and(|r| r.status() == PairCodeStatus::Armed)
    })
    .await;
    assert!(!fence_free(&fence), "the armed code holds serving");

    // A session opened while the code is live holds nothing of its own.
    let during = disabled_endpoint([0x94; 32]).await;
    let (during_conn, during_client) = open_ops_session(&during, robot_addr.clone()).await;
    assert!(!fence_free(&fence), "the armed code still holds serving");
    let desk = disabled_endpoint([0x95; 32]).await;
    pair_with_code(&desk, robot_addr.clone(), robot_key, CODE)
        .await
        .expect("the code pairs a desk");
    until("the result reads paired", || {
        result_in(&remoted).is_some_and(|r| r.status() == PairCodeStatus::Paired)
    })
    .await;
    // A paired code's hold is released once its end is written, and the daemon
    // refreshes its holds every 100 ms: then nothing holds serving, with both
    // sessions of desks that are not paired still open.
    until("nothing holds the robot serving", || fence_free(&fence)).await;
    assert!(
        before_conn.close_reason().is_none() && during_conn.close_reason().is_none(),
        "both sessions stayed open"
    );

    // A session opened after the code's end holds nothing either.
    let after = disabled_endpoint([0x96; 32]).await;
    let (after_conn, after_client) = open_ops_session(&after, robot_addr.clone()).await;
    assert!(
        fence_free(&fence),
        "a session opened after the code ended holds no serving"
    );

    for conn in [&after_conn, &during_conn, &before_conn] {
        conn.close(0u32.into(), b"done");
    }
    drop((after_client, during_client, before_client));
    let _ = shutdown_tx.send(());
    let _ = bounded("serve join", serving).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn files_left_by_an_earlier_run_are_removed_at_start_and_arm_nothing() {
    let robot_ep = disabled_endpoint([0x71; 32]).await;
    let robot_key = *robot_ep.id().as_bytes();
    let robot_addr = loopback_addr(robot_ep.id(), &robot_ep.bound_sockets());
    let root = tempfile::tempdir().unwrap();
    let (config, store, index) = robot_state(root.path(), robot_key);
    let remoted = root.path().join("remoted");
    // A request with a code, and the other handoff files, from a run that
    // ended before the daemon starts.
    let leftovers = [
        leave_request(&remoted),
        remoted.join(CLAIMED_FILE),
        remoted.join(RESULT_FILE),
        remoted.join(".pair-code.request.7.tmp"),
    ];
    for path in &leftovers[1..] {
        std::fs::write(path, b"left").unwrap();
    }
    let (shutdown_tx, serving) = serve(config, store, index, robot_ep, "leftover");

    // They are gone once the ops plane answers.
    let desk = disabled_endpoint([0x72; 32]).await;
    assert_refused(
        pair_with_code(&desk, robot_addr.clone(), robot_key, CODE).await,
        "no code pairing is armed",
    );
    for path in &leftovers {
        assert!(!path.exists(), "{} is removed", path.display());
    }
    // Polls run after the removal; none arms the code or answers it.
    tokio::time::sleep(Duration::from_millis(2_500)).await;
    assert!(!remoted.join(RESULT_FILE).exists(), "nothing was armed");
    assert_refused(
        pair_with_code(&desk, robot_addr, robot_key, CODE).await,
        "no code pairing is armed",
    );

    let _ = shutdown_tx.send(());
    let _ = bounded("serve join", serving).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn three_desks_with_a_wrong_code_use_the_shared_tries_up_and_the_result_says_so() {
    let robot_ep = disabled_endpoint([0x81; 32]).await;
    let robot_key = *robot_ep.id().as_bytes();
    let robot_addr = loopback_addr(robot_ep.id(), &robot_ep.bound_sockets());
    let root = tempfile::tempdir().unwrap();
    let (config, store, index) = robot_state(root.path(), robot_key);
    let store_file = config.store_file.clone();
    let remoted = root.path().join("remoted");
    let fence = cerulion_remoted::serving_hold::fence_path(root.path());
    let (shutdown_tx, serving) = serve(config, store, index, robot_ep, "burn");
    // The ops plane answers before a request is left, so the start's removal
    // is over.
    let probe = disabled_endpoint([0x85; 32]).await;
    assert_refused(
        pair_with_code(&probe, robot_addr.clone(), robot_key, CODE).await,
        "no code pairing is armed",
    );
    leave_request(&remoted);
    until("the daemon arms the code", || {
        result_in(&remoted).is_some_and(|r| r.status() == PairCodeStatus::Armed)
    })
    .await;
    assert!(!fence_free(&fence), "the armed code holds serving");

    // Three desks, distinct keys, a wrong code each; each sees the mismatch
    // itself, sends no finish and drops its connection.
    for (seed, failed) in [(0x82, 1), (0x83, 2), (0x84, 3)] {
        let desk = disabled_endpoint([seed; 32]).await;
        wrong_code_admitted(&desk, robot_addr.clone(), robot_key).await;
        until("the robot counts the try failed", || {
            result_in(&remoted).is_some_and(|r| r.failed() == failed)
        })
        .await;
    }
    let exhausted =
        PairCodeResult::new(NONCE, robot_key, 600, 3, 3, PairCodeState::Exhausted).unwrap();
    assert_eq!(result_in(&remoted), Some(exhausted.clone()));
    assert_eq!(
        String::from_utf8(exhausted.render()).unwrap(),
        format!(
            "cerulion-pair-code-result/1 {} exhausted - {} 600 3 3 - -\n",
            "5a".repeat(16),
            hex::encode(robot_key)
        )
    );
    // An exhausted code keeps the robot serving to its window plus the
    // margin, ten minutes on: over a bounded watch of the daemon's 100 ms
    // refreshes the fence stays taken.
    held_for(
        "the exhausted code holds serving",
        Duration::from_millis(1_500),
        || !fence_free(&fence),
    )
    .await;

    // A fourth desk with the RIGHT code is refused: the tries are used up.
    let desk = disabled_endpoint([0x86; 32]).await;
    let desk_key = *desk.id().as_bytes();
    assert_refused(
        pair_with_code(&desk, robot_addr, robot_key, CODE).await,
        &cerulion_remoted::pairing_verbs::code_exhausted(3),
    );
    let on_disk = TrustStore::load(&store_file, MAC_KEY).unwrap();
    assert!(
        on_disk
            .account_access(&AccountId(desk_key), T_NOW)
            .is_none(),
        "no row for the fourth desk"
    );
    assert!(
        !fence_free(&fence),
        "the exhausted code still holds serving"
    );

    let _ = shutdown_tx.send(());
    let _ = bounded("serve join", serving).await;
}

/// The robot's endpoint, the address a desk dials and its key, and a claimed
/// robot's state under a fresh root.
struct Started {
    root: tempfile::TempDir,
    remoted: PathBuf,
    robot_addr: EndpointAddr,
    robot_key: [u8; 32],
    shutdown_tx: oneshot::Sender<()>,
    serving: tokio::task::JoinHandle<()>,
    robot: Endpoint,
}

/// Serve a fresh robot on the key `[seed; 32]`, once its ops plane answers
/// and the start's removal is over.
async fn started(seed: u8, tag: &str) -> Started {
    let robot_ep = disabled_endpoint([seed; 32]).await;
    let robot = robot_ep.clone();
    let robot_key = *robot_ep.id().as_bytes();
    let robot_addr = loopback_addr(robot_ep.id(), &robot_ep.bound_sockets());
    let root = tempfile::tempdir().unwrap();
    let (config, store, index) = robot_state(root.path(), robot_key);
    let remoted = root.path().join("remoted");
    let (shutdown_tx, serving) = serve(config, store, index, robot_ep, tag);
    let probe = disabled_endpoint([seed.wrapping_add(1); 32]).await;
    assert_refused(
        pair_with_code(&probe, robot_addr.clone(), robot_key, CODE).await,
        "no code pairing is armed",
    );
    Started {
        root,
        remoted,
        robot_addr,
        robot_key,
        shutdown_tx,
        serving,
        robot,
    }
}

/// Leave `bytes` as a request at `mode`, through a staging file and a rename.
fn leave_bytes(remoted: &Path, bytes: &[u8], mode: u32) {
    let staging = remoted.join(format!("{REQUEST_STAGING_PREFIX}2.tmp"));
    std::fs::write(&staging, bytes).unwrap();
    std::fs::set_permissions(&staging, std::fs::Permissions::from_mode(mode)).unwrap();
    std::fs::rename(&staging, remoted.join(REQUEST_FILE)).unwrap();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_claimed_request_the_daemon_refuses_is_answered_refused_and_counted_in_the_status() {
    let s = started(0xA1, "refuse").await;
    // A desk pairs with an armed code, so it can read the wire `status`.
    leave_request(&s.remoted);
    until("the daemon arms the code", || {
        result_in(&s.remoted).is_some_and(|r| r.status() == PairCodeStatus::Armed)
    })
    .await;
    let desk = disabled_endpoint([0xA3; 32]).await;
    pair_with_code(&desk, s.robot_addr.clone(), s.robot_key, CODE)
        .await
        .expect("the armed code pairs");

    // A request the owner-only read refuses: its nonce is not read.
    leave_bytes(
        &s.remoted,
        &PairCodeRequest::new(NONCE, CODE).unwrap().render(),
        0o640,
    );
    let unreadable = PairCodeResult::refused(
        None,
        s.robot_key,
        &PAIR_CODE_POLICY,
        PairCodeRefusal::Unreadable,
    )
    .unwrap();
    until("the result reads refused unreadable", || {
        result_in(&s.remoted).as_ref() == Some(&unreadable)
    })
    .await;
    assert_eq!(
        std::fs::read(s.remoted.join(RESULT_FILE)).unwrap(),
        format!(
            "cerulion-pair-code-result/1 - refused unreadable {} 600 3 0 - -\n",
            hex::encode(s.robot_key)
        )
        .into_bytes()
    );
    // The refusal replaced the paired code, and the status counts it.
    let status = wire_status(&desk, s.robot_addr.clone()).await;
    assert_eq!(status.get("pair_code"), None, "{status}");
    assert_eq!(status["pair_code_refused"], 1, "{status}");

    // A request that does not parse.
    leave_bytes(&s.remoted, b"cerulion-pair-code-request/2 x\n", 0o600);
    until("the result reads refused malformed", || {
        result_in(&s.remoted).and_then(|r| r.refusal()) == Some(PairCodeRefusal::Malformed)
    })
    .await;
    let status = wire_status(&desk, s.robot_addr.clone()).await;
    assert_eq!(status["pair_code_refused"], 2, "{status}");
    assert!(!s.remoted.join(REQUEST_FILE).exists());
    assert!(!s.remoted.join(CLAIMED_FILE).exists());

    let _ = s.shutdown_tx.send(());
    let _ = bounded("serve join", s.serving).await;
    drop(s.root);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_stop_request_withdraws_the_armed_code_and_the_result_says_the_daemon_stopped() {
    let s = started(0xB1, "stop").await;
    leave_request(&s.remoted);
    until("the daemon arms the code", || {
        result_in(&s.remoted).is_some_and(|r| r.status() == PairCodeStatus::Armed)
    })
    .await;
    let _ = s.shutdown_tx.send(());
    let _ = bounded("serve join", s.serving).await;
    let stopped = PairCodeResult::new(
        NONCE,
        s.robot_key,
        600,
        3,
        0,
        PairCodeState::Refused(PairCodeRefusal::Stopping),
    )
    .unwrap();
    assert_eq!(result_in(&s.remoted), Some(stopped));
    drop(s.root);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn an_endpoint_that_closes_on_its_own_withdraws_the_armed_code_and_the_result_says_so() {
    let s = started(0xC1, "close").await;
    leave_request(&s.remoted);
    until("the daemon arms the code", || {
        result_in(&s.remoted).is_some_and(|r| r.status() == PairCodeStatus::Armed)
    })
    .await;
    // No stop request: the robot's endpoint closes, so the accept loop ends
    // on its own.
    bounded("close the endpoint", s.robot.close()).await;
    let _ = bounded("serve join", s.serving).await;
    assert_eq!(
        result_in(&s.remoted).and_then(|r| r.refusal()),
        Some(PairCodeRefusal::Stopping)
    );
    drop(s.shutdown_tx);
    drop(s.root);
}
