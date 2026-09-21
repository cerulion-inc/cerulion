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
//! - a robot with NO armed code is a `PairError::Refused` and writes NO row.

use std::future::Future;
use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr};
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_connectd::pair::{run_pair, PairConfig, PairError};

use cerulion_link::{build_endpoint, Endpoint, EndpointConfig, RelayConfig};

use cerulion_pairing::client::DeviceIdentity;
use cerulion_pairing::format::{AccountId, PrincipalKind, PublicKey, RobotId, RootSet};
use cerulion_pairing::pake::CeremonyConfig;
use cerulion_pairing::verify::{PairingSource, TrustStore};

use cerud::lease::ControlLease;
use cerulion_remoted::pairing_verbs::SharedCodePairSessions;
use cerulion_remoted::{
    DeviceAccountIndex, OpsServing, PairingAuthorizer, RemotedClock, SharedTrust,
};

// ── constants ─────────────────────────────────────────────────────────────────

const STEP_TIMEOUT: Duration = Duration::from_secs(20);
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
        run_pair(pair_config(&robot_ep, desk_seed, CODE)).await
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
    let row = store.is_allowed(&account, 0).expect("desk row on disk");
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
        run_pair(pair_config(&robot_ep, desk_seed, "WRONG-CODE")).await
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
        store.is_allowed(&account, 0).is_none(),
        "no row on a wrong code"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn no_armed_code_is_refused_and_writes_no_row() {
    let dir = tempfile::tempdir().unwrap();
    let robot_ep = disabled_endpoint([202; 32]).await;
    let robot_key = *robot_ep.id().as_bytes();
    // A CLAIMED robot, but NO code armed (the owner never started code pairing).
    let r = build_robot(dir.path(), robot_key);

    let desk_seed = [13u8; 32];
    let result = with_serving(&r, &robot_ep, async {
        run_pair(pair_config(&robot_ep, desk_seed, "ANY-CODE")).await
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
        store.is_allowed(&account, 0).is_none(),
        "no row without an armed code"
    );
}
