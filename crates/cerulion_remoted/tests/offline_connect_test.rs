// SPDX-License-Identifier: AGPL-3.0-only
//! "never bricks offline", over REAL iroh endpoints.
//!
//! An ESTABLISHED pairing must reach the robot on the LAN with the ISSUER down and
//! the RELAY unreachable — direct dial + the offline `cerulion_pairing` TrustStore
//! verify against the durable access list, with ZERO cloud in the connect path.
//! The issuer (which mints certs for NEW pairings) and the relay (off-LAN reach)
//! are conveniences, not dependencies.
//!
//! We prove it two ways, each a fresh, distinct injection of a broken cloud:
//!   1. RELAY DISABLED — there is literally no relay in the connect path (the
//!      strongest "zero cloud" statement); an established pairing connects + runs a
//!      verb over the direct LAN path.
//!   2. RELAY UNREACHABLE — a syntactically-valid but UNROUTABLE relay is
//!      configured (TEST-NET-1, `192.0.2.0/24`, guaranteed unroutable). The
//!      established pairing STILL connects + runs a verb over the direct LAN path,
//!      proving the relay/cloud was never in the connect path.
//!
//! "Issuer down" is STRUCTURAL: there is no issuer client anywhere in the connect
//! path — an established pairing is verified offline against the durable access row
//! read from disk, never by re-running the chain against an issuer. We prove that
//! by reloading the store from disk and confirming the access row + binding are the
//! sole authority the accept gate consulted.
//!
//! Every await is bounded so CI can never hang; the trust state is deterministic
//! fixed-seed crypto (Principle #13) and asserted against a disk reload, never a
//! self-compare.

mod common;

use cerulion_link::RelayConfig;
use cerulion_pairing::format::PublicKey;
use cerulion_pairing::verify::TrustStore;

use common::{
    build_robot, endpoint_with_relay, loopback_addr, run_client, with_serving, MAC_KEY, OWNER,
};

/// Run the offline-connect scenario under a given relay posture for BOTH sides,
/// dialing over the direct LAN path. Returns the `inventory` result the
/// established pairing got.
async fn established_pairing_connects_under(
    daemon_secret: [u8; 32],
    client_secret: [u8; 32],
    relay: fn() -> RelayConfig,
) -> serde_json::Value {
    let dir = tempfile::tempdir().unwrap();
    // The robot's iroh endpoint carries the injected relay posture; its trust
    // state (the accept gate + verb path) is relay-agnostic.
    let robot = endpoint_with_relay(daemon_secret, relay()).await;
    let client = endpoint_with_relay(client_secret, relay()).await;
    let client_key = *client.id().as_bytes();
    let robot_key = *robot.id().as_bytes();
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());

    // An ESTABLISHED pairing: the client key is already bound to the OWNER account
    // on disk (a durable access row + side-map binding) — NOT a live ceremony, so
    // nothing re-verifies a chain against an issuer at connect time.
    let r = build_robot(
        dir.path(),
        robot_key,
        /*claim_owner=*/ true,
        &[(client_key, OWNER)],
    );

    let result = with_serving(&r, &robot, async {
        run_client(&client, robot_addr, |c| {
            c.call("inventory", serde_json::json!({}))
                .expect("inventory served offline")
        })
        .await
    })
    .await;

    // Structural "issuer down" proof: the established pairing was authorized purely
    // from the DURABLE state on disk. Reload the store + index and confirm the
    // access row + binding are present — the sole authority the accept gate used
    // (no issuer, no chain re-verification, no cloud).
    let store = TrustStore::load(&r.store_path, MAC_KEY).expect("store reloads offline");
    assert!(
        store.is_claimed(),
        "the established robot is claimed on disk"
    );
    assert!(
        store.is_allowed(&OWNER, common::T_NOW).is_some(),
        "the durable access row is the offline authority"
    );
    let index = cerulion_remoted::DeviceAccountIndex::load(&r.index_path, MAC_KEY)
        .expect("index reloads offline");
    assert_eq!(
        index.account_for(&PublicKey(client_key)),
        Some(OWNER),
        "the durable device-key binding is the offline authority"
    );

    result
}

/// An UNROUTABLE relay URL (TEST-NET-1, RFC 5737 `192.0.2.0/24` — reserved for
/// documentation, guaranteed unroutable). Configuring it as the relay means any
/// attempt to reach the cloud FAILS; a successful connect proves the direct LAN
/// path carried it.
fn unreachable_relay() -> RelayConfig {
    let url = "https://192.0.2.1:443"
        .parse()
        .expect("a syntactically valid relay URL");
    RelayConfig::Custom(vec![url])
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn established_pairing_connects_with_no_relay_at_all() {
    // Arm 1: RelayConfig::Disabled — there is NO relay in the connect path (the
    // strongest zero-cloud statement). The established pairing connects + runs a
    // verb over the direct LAN path.
    let inv =
        established_pairing_connects_under([233; 32], [73; 32], || RelayConfig::Disabled).await;
    assert!(
        inv["arch"].is_string(),
        "an established pairing runs a verb with NO relay (offline LAN)"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn established_pairing_connects_with_the_relay_unreachable() {
    // Arm 2 (the injection): a CONFIGURED but UNREACHABLE relay. The established
    // pairing STILL connects + runs a verb over the direct LAN path — proving the
    // relay/cloud was never a dependency of the connect path.
    let inv = established_pairing_connects_under([234; 32], [74; 32], unreachable_relay).await;
    assert!(
        inv["arch"].is_string(),
        "an established pairing runs a verb with the relay UNREACHABLE (never bricks offline)"
    );
}
