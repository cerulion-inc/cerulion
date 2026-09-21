// SPDX-License-Identifier: AGPL-3.0-only
//! The REAL ops plane over iroh: two endpoints on loopback (RelayConfig::Disabled,
//! direct dial), the robot hosting `cerud::OpsServer` via a `QuicOpsStream`
//! bridge, a `cerud::OpsClient` on the desk side. Every await is bounded so CI can
//! never hang; every oracle is HAND-BUILT (real crypto, never fake data — Principle
//! #13), and the trust state is verified against a subsequent verb / a disk reload,
//! never a self-compare.
//!
//! Scenarios:
//! - a PAIRED caller runs `inventory` → a real result + a receipt on disk;
//! - an UNPAIRED caller's normal verb → a receipted denial, the connection survives;
//! - an unpaired caller `pair`s with a hand-built valid presentation → an access
//!   row + a side-map binding (verified from DISK); a garbage presentation → clean
//!   deny, no state change;
//! - an UNCLAIMED robot admits ONLY `claim`; after `claim`, normal verbs work for
//!   the owner (the LIVE-shared-state pin — no restart);
//! - CPace `code-pair` start→finish establishes a row via `CpaceConfirmed`; a wrong
//!   code burns bounded attempts + denies; an expired TTL denies;
//! - `engage-estop` from a VIEWER-scoped session engages the safety floor;
//! - a DOUBLE-FRAMING pin: the ops bytes on the wire are cerud's BE-u32 frames, NOT
//!   `cerulion_link`'s LE frames (a raw stream read of the first frame's prefix).

use std::future::Future;
use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr};
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_link::{
    alpn, build_endpoint, dial, direct_addr, open_frame_stream, Endpoint, EndpointAddr,
    EndpointConfig, EndpointId, QuicOpsStream, RelayConfig,
};

use cerud::client::OpsClient;
use cerud::error::CerudError;
use cerud::lease::{ControlLease, EstopState};
use cerud::protocol::{HandshakeReply, Hello};
use cerud::receipt::{read_all, verify_chain, Receipt, ReceiptOutcome};

use cerulion_pairing::format::{
    AccountId, DeviceCert, Grant, IntermediateCert, PrincipalKind, PublicKey, RobotId, Role,
    RootSet, Scope, Validity, FORMAT_VERSION,
};
use cerulion_pairing::pake::{CeremonyConfig, CpaceInitiator, PakeIdentities};
use cerulion_pairing::verify::{PairingSource, TrustStore};
use ed25519_dalek::SigningKey;

use cerulion_remoted::pairing_verbs::{PresentationWire, SharedCodePairSessions};
use cerulion_remoted::{OpsServing, PairingAuthorizer, RemotedClock, SharedTrust};

// ── constants + tiny helpers ────────────────────────────────────────────────

const STEP_TIMEOUT: Duration = Duration::from_secs(20);
const T_NOW: u64 = 1_000_000_000_000;
const ISSUED: u64 = 500_000_000_000;
const CHASSIS: &[u8] = b"ops-loopback-chassis-secret-not-serial-derived";
const MAC_KEY: &[u8] = b"ops-loopback-firmware-mac-key";
const OWNER: AccountId = AccountId([10; 32]);
const ROBOT_ID: RobotId = RobotId([5; 32]);

async fn bounded<F: Future>(what: &str, fut: F) -> F::Output {
    match tokio::time::timeout(STEP_TIMEOUT, fut).await {
        Ok(v) => v,
        Err(_) => panic!("ops loopback: '{what}' timed out after {STEP_TIMEOUT:?}"),
    }
}

async fn disabled_endpoint(secret: [u8; 32]) -> Endpoint {
    bounded(
        "build_endpoint",
        build_endpoint(EndpointConfig::new(secret).with_relay(RelayConfig::Disabled)),
    )
    .await
    .expect("endpoint should bind")
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

fn sk(seed: u8) -> SigningKey {
    SigningKey::from_bytes(&[seed; 32])
}
fn pk(k: &SigningKey) -> PublicKey {
    PublicKey(k.verifying_key().to_bytes())
}
fn wide() -> Validity {
    Validity {
        not_before_ns: 0,
        not_after_ns: 100_000_000_000_000,
    }
}

// ── robot harness ────────────────────────────────────────────────────────────

/// A robot's live trust state + the shared handles a test asserts on / arms.
struct Robot {
    shared: SharedTrust,
    lease: Arc<Mutex<ControlLease>>,
    sessions: SharedCodePairSessions,
    clock: RemotedClock,
    classify: Arc<PairingAuthorizer>,
    ops: Arc<OpsServing>,
    store_path: std::path::PathBuf,
    index_path: std::path::PathBuf,
    receipt_path: std::path::PathBuf,
    /// The state dir (receipt log + log-tail root) — used to rebuild a
    /// short-deadline ops in the deadline-unwedge test.
    dir: std::path::PathBuf,
}

/// Build a robot over a provisioned (optionally CLAIMED) store whose
/// `robot_transport_key` equals `robot_transport_key` (the robot's iroh endpoint
/// id — so CPace's responder key matches the client's dial target). `seed_bindings`
/// pre-binds `device_key → account` rows in the side-map (a paired caller without
/// running the whole ceremony). Persistence is wired to `dir` under a fixed MAC
/// key; a Fixed clock pinned at `T_NOW`.
fn build_robot(
    dir: &Path,
    robot_transport_key: [u8; 32],
    claim_owner: bool,
    seed_bindings: &[([u8; 32], AccountId)],
) -> Robot {
    let store_path = dir.join("trust_store");
    let index_path = dir.join("device_index.json");
    let receipt_path = dir.join("receipts.log");

    let root_set = RootSet::new(vec![pk(&sk(1))], 1).unwrap();
    let mut store = TrustStore::provision(
        ROBOT_ID,
        PublicKey(robot_transport_key),
        root_set,
        CHASSIS,
        T_NOW,
    )
    .unwrap()
    .with_path(&store_path);
    if claim_owner {
        store
            .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
            .unwrap();
        store.save(MAC_KEY).unwrap();
    }
    let mut index = cerulion_remoted::DeviceAccountIndex::new().with_path(&index_path);
    for (key, account) in seed_bindings {
        index.bind(PublicKey(*key), *account);
    }
    if !seed_bindings.is_empty() {
        index.save(MAC_KEY).unwrap();
    }

    let shared = SharedTrust::new(store, index, MAC_KEY.to_vec());
    let lease = Arc::new(Mutex::new(ControlLease::with_default_window()));
    let sessions = SharedCodePairSessions::new();
    let clock = RemotedClock::fixed(T_NOW);
    let classify = Arc::new(PairingAuthorizer::from_shared(shared.clone()));
    let ops = Arc::new(
        OpsServing::new(
            shared.clone(),
            lease.clone(),
            sessions.clone(),
            clock.clone(),
            &receipt_path,
            dir.to_path_buf(),
            "cerulion-remoted-test",
        )
        .expect("ops serving builds"),
    );
    Robot {
        shared,
        lease,
        sessions,
        clock,
        classify,
        ops,
        store_path,
        index_path,
        receipt_path,
        dir: dir.to_path_buf(),
    }
}

/// Build the `pair` verb's `presentation_postcard` argument: a REAL offline cert
/// chain (root-signed intermediate → device cert for `device_key` → grant) with a
/// chosen `scope`, postcard-encoded + hex. The device cert's key is the dialer's
/// actual endpoint key, so the robot's `device_cert.device_key ==
/// authenticated_peer_key` assertion holds.
fn build_presentation_hex(account: AccountId, device_key: [u8; 32], scope: Scope) -> String {
    let root = sk(1);
    let intermediate = sk(2);
    let int_pk = pk(&intermediate);
    let int_cert = IntermediateCert {
        version: FORMAT_VERSION,
        intermediate_key: int_pk,
        validity: wide(),
        issued_at_ns: ISSUED,
        max_scope: Scope::OWNER_FULL,
    }
    .sign_by_roots(&[&root]);
    let device_cert = DeviceCert {
        version: FORMAT_VERSION,
        device_key: PublicKey(device_key),
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
    PresentationWire {
        intermediate: int_cert,
        device_cert,
        grant,
        delegation: None,
    }
    .to_postcard_hex()
    .expect("presentation encodes")
}

/// Run `f` (a synchronous cerud client session) against the robot over ONE ops
/// connection. Bridges async QUIC → cerud's sync `OpsClient` on a blocking task.
async fn run_client<F, R>(client: &Endpoint, robot_addr: EndpointAddr, f: F) -> R
where
    F: FnOnce(&mut OpsClient<QuicOpsStream>) -> R + Send + 'static,
    R: Send + 'static,
{
    let conn = bounded("dial", dial(client, robot_addr, alpn::OPS))
        .await
        .expect("dial ok");
    let (send, recv) = bounded("open_bi", open_frame_stream(&conn))
        .await
        .expect("open_bi ok");
    let ops = QuicOpsStream::new(send, recv);
    let handle = tokio::task::spawn_blocking(move || {
        let mut client = OpsClient::connect_current(ops).expect("cerud handshake");
        f(&mut client)
        // `client` (owning the QuicOpsStream) drops → clean EOF to the robot.
    });
    let r = bounded("client task", handle)
        .await
        .expect("client task join");
    drop(conn);
    r
}

/// Drive `server_fut` (the robot's `serve`) concurrently with a client scenario,
/// signalling shutdown after the scenario so `serve` returns cleanly.
async fn with_serving<C, R>(robot: &Robot, endpoint: &Endpoint, client: C) -> R
where
    C: Future<Output = R>,
{
    let (tx, rx) = tokio::sync::oneshot::channel::<()>();
    let classify = robot.classify.clone();
    let ops = robot.ops.clone();
    let server_fut = async move {
        serve_endpoint(endpoint, classify, ops, rx).await;
    };
    let client_fut = async {
        let out = client.await;
        let _ = tx.send(());
        out
    };
    let (_, out) = tokio::join!(server_fut, client_fut);
    out
}

async fn serve_endpoint(
    endpoint: &Endpoint,
    classify: Arc<PairingAuthorizer>,
    ops: Arc<OpsServing>,
    rx: tokio::sync::oneshot::Receiver<()>,
) {
    let _ = cerulion_remoted::serve(endpoint, classify, Some(ops), async {
        let _ = rx.await;
    })
    .await;
}

fn receipts(path: &Path) -> Vec<Receipt> {
    read_all(path).expect("read receipts")
}

// ── scenario 1: paired inventory + receipt ───────────────────────────────────

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn paired_caller_runs_inventory_and_a_receipt_lands_on_disk() {
    let dir = tempfile::tempdir().unwrap();
    let robot = disabled_endpoint([200; 32]).await;
    let client = disabled_endpoint([9; 32]).await;
    let client_key = *client.id().as_bytes();
    let robot_key = *robot.id().as_bytes();
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());

    // Owner's device key → OWNER (OWNER_FULL) — a paired caller.
    let r = build_robot(
        dir.path(),
        robot_key,
        /*claim_owner=*/ true,
        &[(client_key, OWNER)],
    );

    let result = with_serving(&r, &robot, async {
        run_client(&client, robot_addr, |c| {
            c.call("inventory", serde_json::json!({}))
                .expect("inventory ok")
        })
        .await
    })
    .await;

    assert!(result["arch"].is_string(), "inventory returns arch");
    assert!(
        result["cerulion_version"].is_string(),
        "inventory returns version"
    );

    let rx = receipts(&r.receipt_path);
    assert_eq!(rx.len(), 1, "one receipt for the one inventory call");
    assert_eq!(rx[0].verb, "inventory");
    assert_eq!(
        rx[0].caller,
        hex::encode(client_key),
        "caller is the paired key"
    );
    assert_eq!(rx[0].outcome, ReceiptOutcome::Ok);
    verify_chain(&rx).unwrap();
}

// ── scenario 2: unpaired normal verb denied + receipted, connection survives ──

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn unpaired_caller_normal_verb_is_denied_and_the_connection_survives() {
    let dir = tempfile::tempdir().unwrap();
    let robot = disabled_endpoint([201; 32]).await;
    let client = disabled_endpoint([10; 32]).await;
    let robot_key = *robot.id().as_bytes();
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());
    let r = build_robot(dir.path(), robot_key, /*claim_owner=*/ true, &[]); // no binding for client

    let (first, second) = with_serving(&r, &robot, async {
        run_client(&client, robot_addr, |c| {
            let first = c.call("inventory", serde_json::json!({}));
            // The connection MUST survive the denial: a SECOND request over the
            // SAME connection is served (denied again) — no accept-loop / session
            // teardown on a refusal.
            let second = c.call("inventory", serde_json::json!({}));
            (first, second)
        })
        .await
    })
    .await;

    for (label, res) in [("first", first), ("second", second)] {
        match res {
            Err(CerudError::Remote { kind, message }) => {
                assert_eq!(kind, "denied", "{label}: normal verb denied for unpaired");
                assert!(
                    message.contains("unpaired"),
                    "{label}: reason names unpaired, got {message}"
                );
            }
            other => panic!("{label}: expected a remote denial, got {other:?}"),
        }
    }

    let rx = receipts(&r.receipt_path);
    assert_eq!(rx.len(), 2, "both denied attempts are receipted");
    assert!(rx.iter().all(|e| e.outcome == ReceiptOutcome::Denied));
    verify_chain(&rx).unwrap();
}

// ── scenario 3: unpaired can pair with a valid presentation; garbage denies ───

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn unpaired_caller_pairs_with_a_valid_presentation_and_a_garbage_one_is_refused() {
    let dir = tempfile::tempdir().unwrap();
    let robot = disabled_endpoint([202; 32]).await;
    let client = disabled_endpoint([21; 32]).await; // the device the cert is issued for
    let client_key = *client.id().as_bytes();
    let robot_key = *robot.id().as_bytes();
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());
    let r = build_robot(dir.path(), robot_key, /*claim_owner=*/ true, &[]);

    let viewer = AccountId([20; 32]);
    let presentation = build_presentation_hex(
        viewer,
        client_key,
        Scope {
            role: Role::VIEWER,
            caps: Scope::CAP_OBSERVE,
        },
    );

    let (garbage_res, pair_res, inv_res) = with_serving(&r, &robot, async {
        run_client(&client, robot_addr, move |c| {
            // A garbage presentation → clean deny, no state change.
            let garbage_res = c.call(
                "pair",
                serde_json::json!({ "presentation_postcard": "not-valid-hex-zz!!" }),
            );
            // A valid presentation → paired.
            let pair_res = c.call(
                "pair",
                serde_json::json!({ "presentation_postcard": presentation, "name": "viewer" }),
            );
            // Now that the client is paired (VIEWER + CAP_OBSERVE) → inventory works.
            let inv_res = c.call("inventory", serde_json::json!({}));
            (garbage_res, pair_res, inv_res)
        })
        .await
    })
    .await;

    // Garbage → a verb error (not a panic, not fail-open).
    match garbage_res {
        Err(CerudError::Remote { kind, .. }) => assert_eq!(kind, "verb_error"),
        other => panic!("garbage presentation should be a verb error, got {other:?}"),
    }
    // Valid → paired.
    let paired = pair_res.expect("valid presentation should pair");
    assert_eq!(paired["paired"], true);
    assert_eq!(paired["account"], hex::encode(viewer.0));
    assert_eq!(paired["source"], "strong_chain");
    // Paired now → inventory is served.
    let inv = inv_res.expect("a paired viewer runs inventory");
    assert!(inv["arch"].is_string());

    // Verify the DURABLE state on DISK (never a self-compare): the store carries
    // the viewer's non-revoked row AND the side-map binds the client key → viewer.
    let store = TrustStore::load(&r.store_path, MAC_KEY).expect("store reloads");
    let row = store
        .is_allowed(&viewer, T_NOW)
        .expect("viewer row on disk");
    assert_eq!(row.source, PairingSource::StrongChain);
    assert_eq!(row.name.as_deref(), Some("viewer"));
    let index =
        cerulion_remoted::DeviceAccountIndex::load(&r.index_path, MAC_KEY).expect("index reloads");
    assert_eq!(index.account_for(&PublicKey(client_key)), Some(viewer));
}

// ── scenario 4: unclaimed admits only claim; then normal verbs work live ──────

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn unclaimed_robot_admits_only_claim_then_normal_verbs_work_without_restart() {
    let dir = tempfile::tempdir().unwrap();
    let robot = disabled_endpoint([203; 32]).await;
    let client = disabled_endpoint([12; 32]).await;
    let client_key = *client.id().as_bytes();
    let robot_key = *robot.id().as_bytes();
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());
    let r = build_robot(dir.path(), robot_key, /*claim_owner=*/ false, &[]); // UNCLAIMED

    let (before, claim, after) = with_serving(&r, &robot, async {
        run_client(&client, robot_addr, |c| {
            // Before claiming: a normal verb is denied (only `claim` admissible).
            let before = c.call("inventory", serde_json::json!({}));
            // Claim with the chassis secret → the owner row + side-map binding.
            let claim = c.call(
                "claim",
                serde_json::json!({
                    "account": hex::encode(OWNER.0),
                    "chassis_secret": hex::encode(CHASSIS),
                }),
            );
            // Immediately after — SAME connection, NO restart — inventory works
            // (the accept gate reads the LIVE trust state the claim just wrote).
            let after = c.call("inventory", serde_json::json!({}));
            (before, claim, after)
        })
        .await
    })
    .await;

    match before {
        Err(CerudError::Remote { kind, message }) => {
            assert_eq!(kind, "denied");
            assert!(
                message.contains("only the `claim` bootstrap verb is admissible"),
                "got {message}"
            );
        }
        other => panic!("unclaimed inventory should be denied, got {other:?}"),
    }
    let claimed = claim.expect("claim with the right chassis secret succeeds");
    assert_eq!(claimed["claimed"], true);
    assert_eq!(claimed["account"], hex::encode(OWNER.0));
    let inv = after.expect("owner runs inventory right after claiming (no restart)");
    assert!(inv["arch"].is_string());

    // Durable on disk: the store is now claimed by the owner, side-map bound.
    let store = TrustStore::load(&r.store_path, MAC_KEY).expect("store reloads");
    assert!(store.is_claimed());
    assert_eq!(store.owner(), Some(OWNER));
    let index =
        cerulion_remoted::DeviceAccountIndex::load(&r.index_path, MAC_KEY).expect("index reloads");
    assert_eq!(index.account_for(&PublicKey(client_key)), Some(OWNER));
}

// ── scenario 5: CPace code-pair (happy / wrong code / expired TTL) ────────────

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn code_pair_start_finish_establishes_a_row_via_a_cpace_witness() {
    let dir = tempfile::tempdir().unwrap();
    let robot = disabled_endpoint([204; 32]).await;
    let client = disabled_endpoint([31; 32]).await;
    let client_key = *client.id().as_bytes();
    let robot_key = *robot.id().as_bytes();
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());
    let r = build_robot(dir.path(), robot_key, /*claim_owner=*/ true, &[]);

    // The owner arms a guest code (3 attempts, default TTL).
    const CODE: &str = "SWAN-42";
    r.sessions.arm(CODE, CeremonyConfig::default());

    let guest = AccountId([77; 32]);
    let finish = with_serving(&r, &robot, async {
        run_client(&client, robot_addr, move |c| {
            run_cpace(c, CODE, robot_key, client_key, guest, "guest")
        })
        .await
    })
    .await;

    let paired = finish.expect("a correct code establishes the row");
    assert_eq!(paired["paired"], true);
    assert_eq!(paired["account"], hex::encode(guest.0));
    assert_eq!(paired["source"], "code_paired");

    // The row is on disk with the conservative code-pair default scope.
    let store = TrustStore::load(&r.store_path, MAC_KEY).expect("store reloads");
    let row = store.is_allowed(&guest, T_NOW).expect("guest row on disk");
    assert_eq!(row.source, PairingSource::CodePaired);
    assert_eq!(row.scope, Scope::CODE_PAIR_DEFAULT);
    let index =
        cerulion_remoted::DeviceAccountIndex::load(&r.index_path, MAC_KEY).expect("index reloads");
    assert_eq!(index.account_for(&PublicKey(client_key)), Some(guest));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn code_pair_wrong_code_denies_and_burns_bounded_attempts() {
    let dir = tempfile::tempdir().unwrap();
    let robot = disabled_endpoint([205; 32]).await;
    let client = disabled_endpoint([32; 32]).await;
    let client_key = *client.id().as_bytes();
    let robot_key = *robot.id().as_bytes();
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());
    let r = build_robot(dir.path(), robot_key, /*claim_owner=*/ true, &[]);

    // Arm 2 attempts; the guest presents the WRONG code repeatedly.
    r.sessions.arm(
        "RIGHT-CODE",
        CeremonyConfig::new(2, 60_000_000_000).unwrap(),
    );
    let guest = AccountId([88; 32]);

    let (a1, a2, a3) = with_serving(&r, &robot, async {
        run_client(&client, robot_addr, move |c| {
            let a1 = run_cpace(c, "WRONG-CODE", robot_key, client_key, guest, "g");
            let a2 = run_cpace(c, "WRONG-CODE", robot_key, client_key, guest, "g");
            // The third `code-pair-start` finds the responder BURNED (2 attempts used).
            let a3 = run_cpace(c, "WRONG-CODE", robot_key, client_key, guest, "g");
            (a1, a2, a3)
        })
        .await
    })
    .await;

    // Attempts 1 and 2 (of the armed 2) fail at the FINISH with the wrong-code
    // confirmation rejection — NOT yet burned (the `respond` at each `start`
    // consumed the attempt, the `finish` tag mismatched).
    for (label, res) in [("attempt1", &a1), ("attempt2", &a2)] {
        match res {
            Err(CerudError::Remote { kind, message }) => {
                assert_eq!(kind, "verb_error", "{label}");
                assert!(
                    message.contains("confirmation failed") || message.contains("wrong code"),
                    "{label}: expected a wrong-code confirmation rejection, got {message}"
                );
                assert!(
                    !message.contains("burned"),
                    "{label}: attempt must NOT be burned before the bound is spent, got {message}"
                );
            }
            other => panic!("{label}: wrong code should be denied, got {other:?}"),
        }
    }
    // The THIRD start (attempts 1+2 spent) is refused as EXHAUSTED — the code is
    // burned at EXACTLY the (max+1)th start, naming the max (`all 2 attempts`), so
    // a brute-force cannot continue past the bound.
    match &a3 {
        Err(CerudError::Remote { message, .. }) => {
            assert!(
                message.contains("burned"),
                "burned code names burn, got {message}"
            );
            assert!(
                message.contains("2 attempts exhausted"),
                "the burn must name the exact max (2), got {message}"
            );
        }
        other => panic!("a burned code should refuse at the 3rd start, got {other:?}"),
    }

    // No row was ever established (never fail-open).
    let store = TrustStore::load(&r.store_path, MAC_KEY).expect("store reloads");
    assert!(
        store.is_allowed(&guest, T_NOW).is_none(),
        "no row on a wrong code"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn code_pair_expired_ttl_denies() {
    let dir = tempfile::tempdir().unwrap();
    let robot = disabled_endpoint([206; 32]).await;
    let client = disabled_endpoint([33; 32]).await;
    let client_key = *client.id().as_bytes();
    let robot_key = *robot.id().as_bytes();
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());
    let r = build_robot(dir.path(), robot_key, /*claim_owner=*/ true, &[]);

    // A short TTL; the trusted clock is advanced PAST it between start and finish.
    let ttl = 10_000_000; // 10 ms
    r.sessions
        .arm("TTL-CODE", CeremonyConfig::new(3, ttl).unwrap());
    let guest = AccountId([99; 32]);
    let clock = r.clock.clone();

    let finish = with_serving(&r, &robot, async {
        run_client(&client, robot_addr, move |c| {
            let code = "TTL-CODE";
            let initiator = CpaceInitiator::new(
                code,
                PakeIdentities::new(PublicKey(client_key), PublicKey(robot_key), Vec::new()),
            );
            let (attempt, msg1) = initiator.begin().expect("cpace begin");
            let start = c
                .call(
                    "code-pair-start",
                    serde_json::json!({ "msg1": hex::encode(msg1) }),
                )
                .expect("start ok");
            let msg2 = hex::decode(start["msg2"].as_str().unwrap()).unwrap();
            let responder_confirm =
                hex::decode(start["responder_confirm"].as_str().unwrap()).unwrap();
            let (_keys, initiator_confirm) = attempt
                .finish(&msg2, &responder_confirm)
                .expect("cpace initiator finish");
            // The trusted clock jumps PAST the TTL before the finish reaches the robot.
            clock.set(T_NOW + ttl + 1);
            c.call(
                "code-pair-finish",
                serde_json::json!({
                    "initiator_confirm": hex::encode(initiator_confirm),
                    "account": hex::encode(guest.0),
                    "name": "late-guest",
                }),
            )
        })
        .await
    })
    .await;

    match finish {
        Err(CerudError::Remote { kind, message }) => {
            assert_eq!(kind, "verb_error");
            assert!(
                message.to_lowercase().contains("expire")
                    || message.to_lowercase().contains("code pairing"),
                "expired TTL should be a code-pairing rejection, got {message}"
            );
        }
        other => panic!("an expired ceremony should be refused, got {other:?}"),
    }
    let store = TrustStore::load(&r.store_path, MAC_KEY).expect("store reloads");
    assert!(
        store.is_allowed(&guest, T_NOW).is_none(),
        "no row after an expired TTL"
    );
}

/// Run ONE full CPace ceremony over the ops connection with `code`, returning the
/// `code-pair-finish` result (Ok = paired witness accepted, Err = rejected).
fn run_cpace(
    c: &mut OpsClient<QuicOpsStream>,
    code: &str,
    robot_key: [u8; 32],
    client_key: [u8; 32],
    account: AccountId,
    name: &str,
) -> Result<serde_json::Value, CerudError> {
    let initiator = CpaceInitiator::new(
        code,
        PakeIdentities::new(PublicKey(client_key), PublicKey(robot_key), Vec::new()),
    );
    let (attempt, msg1) = initiator.begin().expect("cpace begin");
    let start = c.call(
        "code-pair-start",
        serde_json::json!({ "msg1": hex::encode(msg1) }),
    )?;
    let msg2 = hex::decode(start["msg2"].as_str().unwrap()).unwrap();
    let responder_confirm = hex::decode(start["responder_confirm"].as_str().unwrap()).unwrap();
    // A wrong code makes the initiator's own confirm check fail FIRST (the
    // responder tag mismatches) — that is still a genuine wrong-code rejection, so
    // fall back to sending a zero tag the robot will reject if the initiator side
    // already detected the mismatch.
    let initiator_confirm = match attempt.finish(&msg2, &responder_confirm) {
        Ok((_keys, tag)) => tag,
        Err(_wrong_code) => vec![0u8; 32], // the robot's finish will also reject this
    };
    c.call(
        "code-pair-finish",
        serde_json::json!({
            "initiator_confirm": hex::encode(initiator_confirm),
            "account": hex::encode(account.0),
            "name": name,
        }),
    )
}

// ── scenario 6: e-stop from a viewer-scoped session engages the floor ─────────

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn viewer_scoped_session_engages_the_estop_floor() {
    let dir = tempfile::tempdir().unwrap();
    let robot = disabled_endpoint([207; 32]).await;
    let client = disabled_endpoint([41; 32]).await;
    let client_key = *client.id().as_bytes();
    let robot_key = *robot.id().as_bytes();
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());
    let r = build_robot(dir.path(), robot_key, /*claim_owner=*/ true, &[]);

    // Establish a VIEWER + CAP_OBSERVE account for the client via a real chain,
    // then bind it — the LEAST-privileged paired role, to prove the e-stop floor
    // is scope-independent.
    let viewer = AccountId([50; 32]);
    let presentation = build_presentation_hex(
        viewer,
        client_key,
        Scope {
            role: Role::VIEWER,
            caps: Scope::CAP_OBSERVE,
        },
    );

    let (pair, estop) = with_serving(&r, &robot, async {
        run_client(&client, robot_addr, move |c| {
            let pair = c.call(
                "pair",
                serde_json::json!({ "presentation_postcard": presentation, "name": "viewer" }),
            );
            let estop = c.call("engage-estop", serde_json::json!({}));
            (pair, estop)
        })
        .await
    })
    .await;

    pair.expect("viewer pairs");
    let e = estop.expect("a viewer-scoped session engages the e-stop floor");
    assert_eq!(e["engaged"], true);
    assert_eq!(e["by"], hex::encode(client_key));
    assert!(e["safe_frame"].is_string());

    // The shared lease is now in the Engaged floor (the effect actually happened).
    let lease = r.lease.lock().unwrap();
    assert_eq!(
        lease.estop(),
        &EstopState::Engaged {
            by: hex::encode(client_key)
        }
    );
}

// ── scenario 6b: a stalled session is closed at the deadline; the plane recovers ─

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_stalled_ops_session_is_closed_at_the_deadline_and_the_plane_serves_the_next() {
    // The load-bearing DoS defense: an authed peer that completes the QUIC
    // handshake then STALLS (holding the serialized ops permit) must be closed at
    // the session deadline so the permit releases and a LATER session is served.
    // A short injected deadline pins this in ~1s instead of the 2-minute default.
    let dir = tempfile::tempdir().unwrap();
    let robot = disabled_endpoint([209; 32]).await;
    let good = disabled_endpoint([61; 32]).await; // paired → runs inventory
    let stall = disabled_endpoint([62; 32]).await; // UNPAIRED bootstrap-surface staller
    let good_key = *good.id().as_bytes();
    let robot_key = *robot.id().as_bytes();
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());
    let r = build_robot(
        dir.path(),
        robot_key,
        /*claim_owner=*/ true,
        &[(good_key, OWNER)],
    );

    // A short-deadline ops-serving context over the SAME live state.
    let deadline = Duration::from_millis(1000);
    let short_ops = Arc::new(
        OpsServing::new(
            r.shared.clone(),
            r.lease.clone(),
            r.sessions.clone(),
            r.clock.clone(),
            &r.receipt_path,
            r.dir.clone(),
            "cerulion-remoted-test",
        )
        .expect("ops serving builds")
        .with_deadline_for_test(deadline),
    );

    let classify = r.classify.clone();
    let (tx, rx) = tokio::sync::oneshot::channel::<()>();
    let robot_ref = &robot;
    let server_fut = async move {
        serve_endpoint(robot_ref, classify, short_ops, rx).await;
    };

    let stall_ref = &stall;
    let good_ref = &good;
    let stall_addr = robot_addr.clone();
    let good_addr = robot_addr.clone();
    let client_fut = async move {
        // The STALL client acquires the permit, then idles WELL past the deadline.
        let stall_task = run_client(stall_ref, stall_addr, |_c| {
            std::thread::sleep(Duration::from_millis(2000));
        });
        // The GOOD client connects mid-way (permit still held by the staller), so
        // its session must WAIT for the stalled session to be closed at the
        // deadline before it is served — proving the unwedge + permit release.
        let good_task = async move {
            tokio::time::sleep(Duration::from_millis(500)).await;
            run_client(good_ref, good_addr, |c| {
                c.call("inventory", serde_json::json!({}))
            })
            .await
        };
        let (_, good_res) = tokio::join!(stall_task, good_task);
        let _ = tx.send(());
        good_res
    };

    let (_, good_res) = tokio::join!(server_fut, client_fut);
    let inv = good_res.expect("the good client is served after the stalled session times out");
    assert!(
        inv["arch"].is_string(),
        "the ops plane recovered and served inventory after unwedging the staller"
    );
}

// ── scenario 6c: a POISONED receipt sink refuses the session (fail-closed) ────

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_poisoned_receipt_sink_refuses_the_session_fail_closed() {
    // If a prior session panicked mid hash-chain receipt write, the shared
    // OpsServer mutex is POISONED — the receipt log may be half-written. The ops
    // plane must REFUSE to serve (never silently recover into a possibly-corrupt
    // chain), closing the connection so the client's handshake fails.
    let dir = tempfile::tempdir().unwrap();
    let robot = disabled_endpoint([210; 32]).await;
    let client = disabled_endpoint([71; 32]).await;
    let client_key = *client.id().as_bytes();
    let robot_key = *robot.id().as_bytes();
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());
    // A PAIRED client — so a refusal is provably the poison, not an authz denial.
    let r = build_robot(
        dir.path(),
        robot_key,
        /*claim_owner=*/ true,
        &[(client_key, OWNER)],
    );

    // Poison the receipt sink (as a panicking session would).
    r.ops.poison_receipt_sink_for_test();

    let handshake: Result<(), CerudError> = with_serving(&r, &robot, async {
        let conn = bounded("dial", dial(&client, robot_addr, alpn::OPS))
            .await
            .expect("dial ok");
        let (send, recv) = bounded("open_bi", open_frame_stream(&conn))
            .await
            .expect("open_bi ok");
        let ops = QuicOpsStream::new(send, recv);
        // Return the handshake RESULT (do not `expect`): a poisoned sink closes the
        // connection, so the handshake must ERROR rather than succeed.
        let handle =
            tokio::task::spawn_blocking(move || OpsClient::connect_current(ops).map(|_| ()));
        let out = bounded("client task", handle)
            .await
            .expect("client task join");
        drop(conn);
        out
    })
    .await;

    assert!(
        handshake.is_err(),
        "a poisoned receipt sink must REFUSE the session (fail-closed), got {handshake:?}"
    );

    // Control: no receipt was appended after the poison — the chain was not touched.
    let rx = receipts(&r.receipt_path);
    assert!(
        rx.is_empty(),
        "the refused session must NOT append to the (possibly-corrupt) chain, got {rx:?}"
    );
}

// ── scenario 7: double-framing pin (cerud BE-u32, NOT link LE) ────────────────

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn ops_wire_bytes_are_cerud_be_frames_not_link_le_frames() {
    let dir = tempfile::tempdir().unwrap();
    let robot = disabled_endpoint([208; 32]).await;
    let client = disabled_endpoint([51; 32]).await;
    let robot_key = *robot.id().as_bytes();
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());
    let r = build_robot(dir.path(), robot_key, /*claim_owner=*/ true, &[]);

    let reply_bytes = with_serving(&r, &robot, async {
        // The client sends a cerud Hello as RAW bytes with cerud's OWN big-endian
        // u32 length prefix (bypassing both `QuicOpsStream` and `OpsClient`), then
        // reads the robot's raw reply. If the robot double-framed (link's LE frame
        // wrapping cerud's frame), the first bytes on the wire would be a LE prefix.
        let conn = bounded("dial", dial(&client, robot_addr, alpn::OPS))
            .await
            .expect("dial");
        let (mut send, mut recv) = bounded("open_bi", open_frame_stream(&conn))
            .await
            .expect("open_bi");
        let hello = serde_json::to_vec(&Hello::current()).unwrap();
        // cerud framing: BIG-endian u32 length prefix + JSON.
        bounded(
            "write len",
            send.write_all(&(hello.len() as u32).to_be_bytes()),
        )
        .await
        .expect("write len");
        bounded("write hello", send.write_all(&hello))
            .await
            .expect("write hello");

        // Read the reply's length prefix + JSON as cerud (BE) frames.
        let mut len_buf = [0u8; 4];
        bounded("read len", recv.read_exact(&mut len_buf))
            .await
            .expect("read reply len");
        let be_len = u32::from_be_bytes(len_buf) as usize;
        let le_len = u32::from_le_bytes(len_buf) as usize;
        // A small cerud reply BE-encodes with zero high bytes; a link LE frame's
        // first byte would be the (nonzero) low byte of the wrapper length.
        assert_eq!(
            &len_buf[0..2],
            &[0u8, 0u8],
            "the length prefix high bytes must be zero (cerud BIG-endian), got {len_buf:?} — a \
             link-style little-endian frame would put a nonzero low byte first (DOUBLE-FRAMED)"
        );
        assert!(
            be_len < 4096 && be_len != le_len,
            "the BE length ({be_len}) must be the small correct one, distinct from the LE reading \
             ({le_len}); equal/huge means the framing was ambiguous or double-wrapped"
        );
        let mut json = vec![0u8; be_len];
        bounded("read json", recv.read_exact(&mut json))
            .await
            .expect("read reply json");
        drop(conn);
        json
    })
    .await;

    // The reply parses as a cerud HandshakeReply::Ack — proving it was a SINGLE
    // cerud frame, not a link frame wrapping a cerud frame.
    let reply: HandshakeReply =
        serde_json::from_slice(&reply_bytes).expect("reply is a single cerud HandshakeReply frame");
    match reply {
        HandshakeReply::Ack {
            chosen_version,
            server_name,
        } => {
            assert_eq!(chosen_version, 1);
            assert_eq!(server_name, "cerulion-remoted-test");
        }
        HandshakeReply::Reject { message, .. } => {
            panic!("expected an Ack, got a Reject: {message}")
        }
    }
}
