// SPDX-License-Identifier: AGPL-3.0-only
//! Daemon `run` WIRING pins that do NOT need a real iroh endpoint — every arm
//! resolves BEFORE (or instead of) binding:
//!
//! - R8 kill-switch: `run` with the network disabled REFUSES to serve and
//!   returns cleanly WITHOUT loading state or binding an endpoint (a bogus,
//!   absent key file is never even read).
//! - absent device key → a LOUD provisioning-gap error (before any endpoint).
//! - absent trust store (key + mac key present) → a LOUD store error (before any
//!   endpoint).
//! - present store but a MISSING MAC key → a LOUD provisioning-class MacKey error
//!   (the secret verifying the trust state is gone — NOT a config slip).
//! - genuinely-fresh robot (only the device key present) → the absent store's own
//!   Store provisioning gap (the MAC key's absence is benign — nothing needs it).
//!
//! The iroh-dependent arms (accept-loop survives a bad handshake; the store↔
//! device binding mismatch/match through `run`) live in `loopback_test.rs`.

use std::fs;

use cerulion_link::RelayConfig;
use cerulion_remoted::{run, RemotedConfig, RemotedError};
use tempfile::tempdir;

/// The `remoted/` subdir + a valid 32-byte device key + a MAC key, so the daemon
/// gets PAST the key load and reaches the store/index. Returns the state root.
fn seed_key_and_mac(root: &std::path::Path) {
    let remoted = root.join("remoted");
    fs::create_dir_all(&remoted).unwrap();
    fs::write(remoted.join("device_key"), [7u8; 32]).unwrap();
    fs::write(remoted.join("trust_store.mac_key"), b"wiring-mac-key").unwrap();
}

#[tokio::test]
async fn network_disabled_refuses_to_serve_without_binding() {
    // The kill-switch short-circuits BEFORE load_state: even though the key file
    // is absent (which would otherwise be a hard error), `run` returns Ok(())
    // cleanly — it never loads state, never binds an endpoint. If it had tried to
    // bind, the absent key would have surfaced an Err; the Ok proves it did not.
    let dir = tempdir().unwrap();
    let cfg =
        RemotedConfig::from_state_root(dir.path(), RelayConfig::Disabled, /*disabled=*/ true);
    let result = run(cfg, async {}).await;
    assert!(
        result.is_ok(),
        "network-disabled run must exit cleanly without binding, got {result:?}"
    );
}

#[tokio::test]
async fn absent_device_key_is_a_loud_provisioning_error() {
    // Network enabled, but the state root is empty → the device key load fails
    // (before any endpoint bind). A provisioning gap, never fabricated.
    let dir = tempdir().unwrap();
    let cfg =
        RemotedConfig::from_state_root(dir.path(), RelayConfig::Disabled, /*disabled=*/ false);
    let err = run(cfg, async {}).await.unwrap_err();
    assert!(
        matches!(err, RemotedError::DeviceKey(_)),
        "expected DeviceKey error, got {err:?}"
    );
    assert!(err.to_string().contains("device key"), "err: {err}");
}

#[tokio::test]
async fn absent_trust_store_is_a_loud_store_error() {
    // Key + MAC key present, but the trust store is absent → a LOUD store error
    // (the store is factory-provisioned; an absent store is a provisioning gap,
    // never a silently-fabricated fresh store). Resolves before any endpoint.
    let dir = tempdir().unwrap();
    seed_key_and_mac(dir.path());
    let cfg =
        RemotedConfig::from_state_root(dir.path(), RelayConfig::Disabled, /*disabled=*/ false);
    let err = run(cfg, async {}).await.unwrap_err();
    assert!(
        matches!(err, RemotedError::Store(_)),
        "expected Store error, got {err:?}"
    );
    assert!(err.to_string().contains("trust store"), "err: {err}");
}

#[tokio::test]
async fn present_store_but_missing_mac_key_is_a_provisioning_class_error() {
    // A trust store EXISTS but the MAC key that authenticates it is GONE → a LOUD
    // provisioning-class MacKey error (the secure-storage secret verifying this
    // robot's trust state is missing), NOT a generic Config error, and NOT the
    // Store error (the MAC-key requirement is checked BEFORE the store is decoded).
    // Reverting `load_mac_key`'s classification to Config fails
    // this exact assertion.
    let dir = tempdir().unwrap();
    let remoted = dir.path().join("remoted");
    fs::create_dir_all(&remoted).unwrap();
    fs::write(remoted.join("device_key"), [7u8; 32]).unwrap();
    // A present store file (content is never read — the MAC-key requirement fires
    // first). NO `trust_store.mac_key` file is written.
    fs::write(remoted.join("trust_store"), b"present-but-unverifiable").unwrap();
    let cfg =
        RemotedConfig::from_state_root(dir.path(), RelayConfig::Disabled, /*disabled=*/ false);
    let err = run(cfg, async {}).await.unwrap_err();
    assert!(
        matches!(err, RemotedError::MacKey(_)),
        "expected a provisioning-class MacKey error, got {err:?}"
    );
    assert!(
        !matches!(err, RemotedError::Config(_)),
        "a required-but-missing MAC key must NOT be a config error: {err:?}"
    );
    let msg = err.to_string();
    assert!(msg.contains("trust-store MAC key"), "err: {msg}");
    assert!(msg.contains("provisioning gap"), "err: {msg}");
}

#[tokio::test]
async fn present_mac_d_index_but_missing_mac_key_is_a_provisioning_class_error() {
    // The MAC-key requirement is also raised by a present MAC'd device index (not
    // just the store): index present + MAC key absent → provisioning-class MacKey.
    let dir = tempdir().unwrap();
    let remoted = dir.path().join("remoted");
    fs::create_dir_all(&remoted).unwrap();
    fs::write(remoted.join("device_key"), [7u8; 32]).unwrap();
    // A present (MAC'd) index file; the store is absent, the MAC key is absent.
    fs::write(
        remoted.join("device_index.json"),
        b"present-but-unverifiable",
    )
    .unwrap();
    let cfg =
        RemotedConfig::from_state_root(dir.path(), RelayConfig::Disabled, /*disabled=*/ false);
    let err = run(cfg, async {}).await.unwrap_err();
    assert!(
        matches!(err, RemotedError::MacKey(_)),
        "a present MAC'd index must raise the MAC-key requirement, got {err:?}"
    );
}

#[tokio::test]
async fn fresh_robot_missing_mac_key_defers_to_the_absent_store_gap() {
    // Genuinely-fresh robot: only the device key is present — NO store, NO index,
    // NO MAC key. The MAC key is required by nothing, so its absence is benign; the
    // fundamental provisioning gap (an absent trust store) is what surfaces, as a
    // Store error — NOT a MacKey error, NOT a Config error. Proves the fix does not
    // turn a fresh robot's missing key into a scary security error.
    let dir = tempdir().unwrap();
    let remoted = dir.path().join("remoted");
    fs::create_dir_all(&remoted).unwrap();
    fs::write(remoted.join("device_key"), [7u8; 32]).unwrap();
    let cfg =
        RemotedConfig::from_state_root(dir.path(), RelayConfig::Disabled, /*disabled=*/ false);
    let err = run(cfg, async {}).await.unwrap_err();
    assert!(
        matches!(err, RemotedError::Store(_)),
        "a fresh robot's absent store is the provisioning gap, got {err:?}"
    );
}
