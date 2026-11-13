// SPDX-License-Identifier: AGPL-3.0-only
//! The desk-side `run_pair` CPace ceremony against a REAL in-process
//! `cerulion_remoted` ops plane over iroh loopback (RelayConfig::Disabled, direct
//! dial). The gold-standard pin: a real PAKE round-trip — never fake data
//! (Principle #13) — with the robot's durable trust state verified from DISK
//! (never a self-compare).
//!
//! Every await is BOUNDED (a generous 20s desk-side timeout the fast loopback
//! ceremony never reaches) and the harness is load-tolerant — no tight deadline
//! the ops-loopback deadline class warns about.
//!
//! Scenarios:
//! - a correct code establishes a code-paired row for the desk's device key +
//!   the self-account derived from it;
//! - a WRONG code is a distinct `PairError::CodeMismatch` (detected structurally
//!   desk-side) and writes NO row;
//! - a robot with NO armed code is a `PairError::Refused` and writes NO row;
//! - a robot that accepts the ops stream and never answers holds the ceremony
//!   until the desk-side timeout, a `PairError::Unreachable` naming it, and
//!   sees the desk close the connection;
//! - a desk interrupted while the robot holds its try is a
//!   `PairError::Interrupted`, and the robot counts that try failed within
//!   [`TRY_END_LIMIT`] of letting its held step go, with no row written;
//! - a desk whose ceremony paired, interrupted while it waits for its
//!   endpoint close, still returns paired, the robot's row on disk;
//! - each refusal the robot's code-pairing store answers maps to its desk exit
//!   class: every try failed is a code mismatch; a used code, a throttled try,
//!   every try taken by a desk still pairing, an expired window, no armed code
//!   and no try in progress are refusals.

use std::future::Future;
use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr};
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::task::Poll;
use std::time::Duration;

use cerulion_connectd::pair::{run_pair, PairConfig, PairError};

use cerulion_link::{
    accept_frame_stream, accept_one, build_endpoint, ConnectionError, Endpoint, EndpointConfig,
    RelayConfig,
};

use cerulion_pairing::client::DeviceIdentity;
use cerulion_pairing::format::{AccountId, PrincipalKind, PublicKey, RobotId, RootSet};
use cerulion_pairing::pair_code::PairCodeStatus;
use cerulion_pairing::pake::CeremonyConfig;
use cerulion_pairing::verify::{PairingSource, TrustStore};

use cerud::lease::ControlLease;
use cerulion_remoted::pairing_verbs::SharedCodePairSessions;
use cerulion_remoted::{
    DeviceAccountIndex, OpsServing, PairingAuthorizer, RemotedClock, SharedTrust,
};

// ── constants ─────────────────────────────────────────────────────────────────

const STEP_TIMEOUT: Duration = Duration::from_secs(20);
/// The desk-side timeout of the run a silent robot holds.
const CEREMONY_TIMEOUT: Duration = Duration::from_secs(5);
/// How long the robot may take to count an interrupted desk's pending try
/// failed once its held `code-pair-start` is let go. A desk that closed its
/// endpoint before `run_pair` returned has closed the connection the robot's
/// ops session for it reads, so that session ends, and the try is counted,
/// as the step's reply fails; a connection the desk only dropped ends at the
/// robot at its 30 s QUIC idle timeout, past this bound.
const TRY_END_LIMIT: Duration = Duration::from_secs(5);
const T_NOW: u64 = 1_000_000_000_000;
const CHASSIS: &[u8] = b"pair-e2e-chassis-secret-not-serial-derived";
const MAC_KEY: &[u8] = b"pair-e2e-firmware-mac-key";
const OWNER: AccountId = AccountId([10; 32]);
const ROBOT_ID: RobotId = RobotId([5; 32]);

// ── tiny helpers (cribbed from ops_loopback_test) ─────────────────────────────

async fn bounded<F: Future>(what: &str, fut: F) -> F::Output {
    match tokio::time::timeout(STEP_TIMEOUT, fut).await {
        Ok(v) => v,
        Err(_) => panic!("pair e2e: '{what}' timed out after {STEP_TIMEOUT:?}"),
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

/// The robot's bound sockets rewritten so an unspecified `0.0.0.0` / `::` becomes
/// loopback — the direct-dial addresses `run_pair` uses.
fn localhost_sockets(bound: &[SocketAddr]) -> Vec<SocketAddr> {
    bound
        .iter()
        .map(|s| match s {
            SocketAddr::V4(v4) if v4.ip().is_unspecified() => {
                SocketAddr::from((Ipv4Addr::LOCALHOST, v4.port()))
            }
            SocketAddr::V6(v6) if v6.ip().is_unspecified() => {
                SocketAddr::from((Ipv6Addr::LOCALHOST, v6.port()))
            }
            other => *other,
        })
        .collect()
}

/// A public key derived from a fixed 32-byte seed via the pairing crate's own
/// identity seam (no direct `ed25519_dalek` dep needed here).
fn pk_from_seed(seed: u8) -> PublicKey {
    DeviceIdentity::from_seed(&[seed; 32]).public_key()
}

/// The desk device pubkey derived from a seed (== its iroh EndpointId + its CPace
/// initiator key + its self-account). NOT a self-compare of `run_pair` — this is
/// the independent oracle for the account the robot must record.
fn desk_pubkey(seed: [u8; 32]) -> [u8; 32] {
    DeviceIdentity::from_seed(&seed).public_key().0
}

// ── robot harness ─────────────────────────────────────────────────────────────

struct Robot {
    sessions: SharedCodePairSessions,
    classify: Arc<PairingAuthorizer>,
    ops: Arc<OpsServing>,
    store_path: std::path::PathBuf,
    index_path: std::path::PathBuf,
}

/// Build a CLAIMED robot whose transport key == `robot_transport_key` (its iroh
/// endpoint id), persisted under `dir`.
fn build_robot(dir: &Path, robot_transport_key: [u8; 32]) -> Robot {
    let store_path = dir.join("trust_store");
    let index_path = dir.join("device_index.json");
    let receipt_path = dir.join("receipts.log");

    let root_set = RootSet::new(vec![pk_from_seed(1)], 1).unwrap();
    let mut store = TrustStore::provision(
        ROBOT_ID,
        PublicKey(robot_transport_key),
        root_set,
        CHASSIS,
        T_NOW,
    )
    .unwrap()
    .with_path(&store_path);
    store
        .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    store.save(MAC_KEY).unwrap();

    let index = DeviceAccountIndex::new().with_path(&index_path);
    let shared = SharedTrust::new(store, index, MAC_KEY.to_vec());
    let lease = Arc::new(Mutex::new(ControlLease::with_default_window()));
    let sessions = SharedCodePairSessions::new();
    let clock = RemotedClock::fixed(T_NOW);
    let classify = Arc::new(PairingAuthorizer::from_shared(shared.clone()));
    let ops = Arc::new(
        OpsServing::new(
            shared,
            lease,
            sessions.clone(),
            clock,
            &receipt_path,
            dir.to_path_buf(),
            "cerulion-remoted-pair-e2e",
        )
        .expect("ops serving builds"),
    );
    Robot {
        sessions,
        classify,
        ops,
        store_path,
        index_path,
    }
}

/// Serve the robot's ops plane concurrently with `client` (the `run_pair` future),
/// signalling shutdown once the client returns.
async fn with_serving<C, R>(robot: &Robot, endpoint: &Endpoint, client: C) -> R
where
    C: Future<Output = R>,
{
    let (tx, rx) = tokio::sync::oneshot::channel::<()>();
    let classify = robot.classify.clone();
    let ops = robot.ops.clone();
    let server_fut = async move {
        let _ = cerulion_remoted::serve(endpoint, classify, Some(ops), async {
            let _ = rx.await;
        })
        .await;
    };
    let client_fut = async {
        let out = client.await;
        let _ = tx.send(());
        out
    };
    let (_, out) = tokio::join!(server_fut, client_fut);
    out
}

fn pair_config(robot: &Endpoint, desk_seed: [u8; 32], code: &str) -> PairConfig {
    PairConfig {
        robot_eid: robot.id(),
        direct_addrs: localhost_sockets(&robot.bound_sockets()),
        desk_seed,
        relay: RelayConfig::Disabled,
        code: code.to_string(),
        account: None, // self-account derived from the desk key
        label: "test-desk".to_string(),
        robot_display: "go2".to_string(),
        timeout: STEP_TIMEOUT,
    }
}

// ── scenarios ─────────────────────────────────────────────────────────────────

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn correct_code_establishes_a_code_paired_row_for_the_desk_key() {
    let dir = tempfile::tempdir().unwrap();
    let robot_ep = disabled_endpoint([200; 32]).await;
    let robot_key = *robot_ep.id().as_bytes();
    let r = build_robot(dir.path(), robot_key);

    const CODE: &str = "SWAN-42";
    r.sessions.arm(CODE, CeremonyConfig::default());

    let desk_seed = [9u8; 32];
    let outcome = with_serving(&r, &robot_ep, async {
        run_pair(
            pair_config(&robot_ep, desk_seed, CODE),
            std::future::pending(),
        )
        .await
    })
    .await;

    let outcome = outcome.expect("a correct code pairs");
    // The self-account is the desk device pubkey (an independent oracle).
    let desk_pk = desk_pubkey(desk_seed);
    assert_eq!(
        outcome.account_hex,
        hex::encode(desk_pk),
        "self-account = desk pubkey"
    );
    assert_eq!(outcome.source, "code_paired");
    assert_eq!(outcome.desk_eid_hex, hex::encode(desk_pk));

    // The robot's DURABLE state on DISK (never a self-compare): a code-paired row
    // for the account + the side-map binding the desk device key → that account.
    let account = AccountId(desk_pk);
    let store = TrustStore::load(&r.store_path, MAC_KEY).expect("store reloads");
    let row = store.account_access(&account, 0).expect("desk row on disk");
    assert_eq!(row.source, PairingSource::CodePaired);
    let index = DeviceAccountIndex::load(&r.index_path, MAC_KEY).expect("index reloads");
    assert_eq!(index.account_for(&PublicKey(desk_pk)), Some(account));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn wrong_code_is_a_code_mismatch_and_writes_no_row() {
    let dir = tempfile::tempdir().unwrap();
    let robot_ep = disabled_endpoint([201; 32]).await;
    let robot_key = *robot_ep.id().as_bytes();
    let r = build_robot(dir.path(), robot_key);

    r.sessions.arm(
        "RIGHT-CODE",
        CeremonyConfig::new(3, 60_000_000_000).unwrap(),
    );

    let desk_seed = [11u8; 32];
    let result = with_serving(&r, &robot_ep, async {
        run_pair(
            pair_config(&robot_ep, desk_seed, "WRONG-CODE"),
            std::future::pending(),
        )
        .await
    })
    .await;

    match result {
        Err(PairError::CodeMismatch(_)) => {}
        other => panic!("a wrong code must be PairError::CodeMismatch, got {other:?}"),
    }
    // Never fail-open: no row was established for the desk key.
    let account = AccountId(desk_pubkey(desk_seed));
    let store = TrustStore::load(&r.store_path, MAC_KEY).expect("store reloads");
    assert!(
        store.account_access(&account, 0).is_none(),
        "no row on a wrong code"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn no_armed_code_is_refused_and_writes_no_row() {
    let dir = tempfile::tempdir().unwrap();
    let robot_ep = disabled_endpoint([202; 32]).await;
    let robot_key = *robot_ep.id().as_bytes();
    // A CLAIMED robot, but NO code armed (no request was left on the robot).
    let r = build_robot(dir.path(), robot_key);

    let desk_seed = [13u8; 32];
    let result = with_serving(&r, &robot_ep, async {
        run_pair(
            pair_config(&robot_ep, desk_seed, "ANY-CODE"),
            std::future::pending(),
        )
        .await
    })
    .await;

    match result {
        Err(PairError::Refused(msg)) => {
            assert!(
                msg.to_lowercase().contains("no code pairing is armed"),
                "the refusal names the missing armed code, got: {msg}"
            );
        }
        other => panic!("no armed code must be PairError::Refused, got {other:?}"),
    }
    let account = AccountId(desk_pubkey(desk_seed));
    let store = TrustStore::load(&r.store_path, MAC_KEY).expect("store reloads");
    assert!(
        store.account_access(&account, 0).is_none(),
        "no row without an armed code"
    );
}

/// A robot that accepts the desk's ops stream and never answers holds the
/// ceremony on its blocking read. At the desk-side timeout the desk closes
/// the connection, which fails that read: `run_pair` returns the timeout as
/// unreachable, and the robot sees the desk's close with code 0 and no
/// reason. A connection the desk only dropped would stay open, its stream
/// halves held by the parked read and both endpoints sending keep-alives, so
/// the run would not return within [`STEP_TIMEOUT`].
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_ceremony_timeout_closes_the_connection_and_is_unreachable() {
    let robot_ep = disabled_endpoint([203; 32]).await;
    let mut config = pair_config(&robot_ep, [15u8; 32], "ANY-CODE");
    config.timeout = CEREMONY_TIMEOUT;
    let silent_robot = async {
        let accepted = accept_one(&robot_ep)
            .await
            .expect("accept the desk's connection")
            .expect("the robot endpoint is open");
        let streams = accept_frame_stream(&accepted.connection)
            .await
            .expect("accept the desk's ops stream");
        let closed = accepted.connection.closed().await;
        drop(streams);
        closed
    };
    let (result, closed) = bounded("the timed-out ceremony and its close", async {
        tokio::join!(run_pair(config, std::future::pending()), silent_robot)
    })
    .await;
    robot_ep.close().await;

    match result {
        Err(PairError::Unreachable(msg)) => {
            assert_eq!(msg, "the pairing ceremony timed out after 5s");
        }
        other => panic!("a silent robot must time the ceremony out, got {other:?}"),
    }
    match closed {
        ConnectionError::ApplicationClosed(close) => {
            assert_eq!(close.error_code.into_inner(), 0);
            assert!(close.reason.is_empty(), "{close:?}");
        }
        other => panic!("the robot must see the desk close the connection, got {other:?}"),
    }
}

/// A desk interrupted while the robot holds its try closes its endpoint
/// before `run_pair` returns [`PairError::Interrupted`]. The robot's admitted
/// `code-pair-start` is held before its CPace step until then, so the
/// interrupt lands while the desk waits on the robot's reply. Let go, the step
/// leaves the try pending, the robot's ops session for the desk ends on the
/// closed connection, and the try is counted failed within
/// [`TRY_END_LIMIT`]. The code was right and no row is written: the desk's
/// ceremony ended with its run.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn an_interrupted_desk_closes_and_the_robot_counts_its_pending_try_failed() {
    let dir = tempfile::tempdir().unwrap();
    let robot_ep = disabled_endpoint([204; 32]).await;
    let r = build_robot(dir.path(), *robot_ep.id().as_bytes());
    const CODE: &str = "RIGHT-CODE";
    r.sessions
        .arm(CODE, CeremonyConfig::new(3, 60_000_000_000).unwrap());
    let held = r.sessions.hold_next_build_for_test();
    let (interrupt, interrupted) = tokio::sync::oneshot::channel::<()>();
    let desk_seed = [16u8; 32];

    let (result, counted) = with_serving(&r, &robot_ep, async {
        let desk = run_pair(pair_config(&robot_ep, desk_seed, CODE), async move {
            let _ = interrupted.await;
        });
        let robot = async move {
            let (reached, held) =
                tokio::task::spawn_blocking(move || (held.reached_within(STEP_TIMEOUT), held))
                    .await
                    .expect("the hold watch ends");
            assert!(reached, "the desk's code-pair-start reached the held step");
            let _ = interrupt.send(());
            held
        };
        let (result, held) =
            bounded("the interrupted run", async { tokio::join!(desk, robot) }).await;
        held.release();
        let counted = tokio::time::timeout(TRY_END_LIMIT, async {
            loop {
                match r.sessions.snapshot() {
                    Some(code) if code.failed > 0 => return code,
                    _ => tokio::time::sleep(Duration::from_millis(20)).await,
                }
            }
        })
        .await
        .map_err(|_| r.sessions.snapshot());
        (result, counted)
    })
    .await;
    robot_ep.close().await;

    assert!(matches!(result, Err(PairError::Interrupted)), "{result:?}");
    let code = counted.unwrap_or_else(|at_the_limit| {
        panic!(
            "the robot did not count the interrupted desk's try failed within \
             {TRY_END_LIMIT:?}: {at_the_limit:?}"
        )
    });
    assert_eq!((code.failed, code.tries), (1, 3), "{code:?}");
    let store = TrustStore::load(&r.store_path, MAC_KEY).expect("store reloads");
    assert!(
        store
            .account_access(&AccountId(desk_pubkey(desk_seed)), 0)
            .is_none(),
        "an interrupted desk writes no row"
    );
}

/// A desk whose ceremony paired, interrupted while it waits for its
/// endpoint close, returns paired. The interrupt resolves on the first poll
/// that finds the robot's code paired and arranges no wake of its own. The
/// robot serves on a task of its own, so the run is polled only on wakes of
/// its own futures: while the ceremony runs on its blocking task, only when
/// that task ends, and then it polls the ceremony before the interrupt. The
/// robot pairs the code before it answers `code-pair-finish`, so the
/// interrupt is first polled with the code paired in the wait for the close,
/// which waits on the robot's answer to the close. The run polled the
/// interrupt to its end, returns the desk key's own account, and the robot's
/// code-paired row is on disk.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn an_interrupt_during_the_close_of_a_paired_desk_returns_paired() {
    let dir = tempfile::tempdir().unwrap();
    let robot_ep = disabled_endpoint([205; 32]).await;
    let r = build_robot(dir.path(), *robot_ep.id().as_bytes());
    const CODE: &str = "RIGHT-CODE";
    r.sessions.arm(CODE, CeremonyConfig::default());
    let desk_seed = [17u8; 32];
    let resolved = Arc::new(AtomicBool::new(false));
    let interrupt = {
        let sessions = r.sessions.clone();
        let resolved = resolved.clone();
        std::future::poll_fn(move |_| {
            let paired = sessions
                .snapshot()
                .is_some_and(|code| code.state.status() == PairCodeStatus::Paired);
            if paired {
                resolved.store(true, Ordering::SeqCst);
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
    };

    let (stop, stopped) = tokio::sync::oneshot::channel::<()>();
    let serving = tokio::spawn({
        let endpoint = robot_ep.clone();
        let classify = r.classify.clone();
        let ops = r.ops.clone();
        async move {
            let _ = cerulion_remoted::serve(&endpoint, classify, Some(ops), async {
                let _ = stopped.await;
            })
            .await;
        }
    });

    let result = run_pair(pair_config(&robot_ep, desk_seed, CODE), interrupt).await;
    let _ = stop.send(());
    bounded("the robot's serving", serving)
        .await
        .expect("the robot's serving task ends");
    robot_ep.close().await;

    let outcome = result.expect("a paired desk interrupted during its close returns paired");
    assert!(
        resolved.load(Ordering::SeqCst),
        "the run polled the interrupt to its end"
    );
    let desk_pk = desk_pubkey(desk_seed);
    assert_eq!(outcome.account_hex, hex::encode(desk_pk));
    assert_eq!(outcome.source, "code_paired");
    let store = TrustStore::load(&r.store_path, MAC_KEY).expect("store reloads");
    let row = store
        .account_access(&AccountId(desk_pk), 0)
        .expect("desk row on disk");
    assert_eq!(row.source, PairingSource::CodePaired);
}

#[test]
fn each_robot_refusal_maps_to_its_desk_exit_class() {
    use cerulion_connectd::pair::{classify_verb_error, VerbErrorClass};
    use cerulion_remoted::pairing_verbs::{
        code_exhausted, code_expired, CODE_USED, NOT_ARMED, NO_SESSION, TRIES_IN_PROGRESS,
        TRY_THROTTLED,
    };
    // The texts the robot's store answers, as `code-pair-start` and
    // `code-pair-finish` carry them in a `verb_error`.
    let wrong = "code pairing rejected: pairing key confirmation failed (wrong code?)";
    let cases = [
        (code_exhausted(3), VerbErrorClass::CodeMismatch),
        (wrong.to_string(), VerbErrorClass::CodeMismatch),
        (CODE_USED.to_string(), VerbErrorClass::Refused),
        (TRY_THROTTLED.to_string(), VerbErrorClass::Refused),
        (TRIES_IN_PROGRESS.to_string(), VerbErrorClass::Refused),
        (code_expired(), VerbErrorClass::Refused),
        (NOT_ARMED.to_string(), VerbErrorClass::Refused),
        (NO_SESSION.to_string(), VerbErrorClass::Refused),
    ];
    for (message, want) in cases {
        assert_eq!(
            classify_verb_error("verb_error", &message),
            want,
            "{message}"
        );
    }
}
