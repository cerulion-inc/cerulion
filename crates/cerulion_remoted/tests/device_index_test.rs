// SPDX-License-Identifier: AGPL-3.0-only
//! Persistence + lookup + MAC-integrity contract for the `device_key → account`
//! side-map (R1).
//!
//! The index is the SOLE device_key→account binding the accept gate trusts, so
//! it is MAC-authenticated with the SAME firmware secure-storage key as the
//! trust store. A disk-tamper adversary must NOT be able to forge a
//! `{their key → owner account}` binding (escalating to OWNER) nor delete the
//! MAC to reset the index.
//!
//! Oracle-vector (never a self-compare — Principle #13): every binding is a
//! hand-chosen key→account pair, asserted back literally after a round-trip.

use cerulion_pairing::format::{AccountId, PublicKey};
use cerulion_remoted::DeviceAccountIndex;
use tempfile::tempdir;

/// The firmware secure-storage MAC key stand-in (deterministic — not fake data;
/// a real fixed key exercises the real HMAC path).
const MAC_KEY: &[u8] = b"device-index-integration-mac-key";
/// A DIFFERENT key, for the wrong-key-rejected control.
const OTHER_KEY: &[u8] = b"a-completely-different-mac-key!!";

fn key(b: u8) -> PublicKey {
    PublicKey([b; 32])
}
fn account(b: u8) -> AccountId {
    AccountId([b; 32])
}

#[test]
fn round_trip_write_reload_lookup() {
    let dir = tempdir().unwrap();
    let path = dir.path().join("device_index.idx");

    // Hand oracle: two devices for account 10, one for account 20.
    let mut index = DeviceAccountIndex::new().with_path(&path);
    index.bind(key(1), account(10));
    index.bind(key(2), account(10)); // multiple keys per account
    index.bind(key(3), account(20));
    index.save(MAC_KEY).unwrap();

    // Reload from disk (MAC verifies) and assert every binding survives.
    let reloaded = DeviceAccountIndex::load(&path, MAC_KEY).unwrap();
    assert_eq!(reloaded.len(), 3);
    assert_eq!(reloaded.account_for(&key(1)), Some(account(10)));
    assert_eq!(reloaded.account_for(&key(2)), Some(account(10)));
    assert_eq!(reloaded.account_for(&key(3)), Some(account(20)));
    // An unknown key maps to nothing.
    assert_eq!(reloaded.account_for(&key(99)), None);
}

#[test]
fn multiple_keys_one_account_all_resolve() {
    let dir = tempdir().unwrap();
    let path = dir.path().join("index.idx");
    let mut index = DeviceAccountIndex::new().with_path(&path);
    // One account (a human with a laptop + a phone + a workstation).
    for k in [5u8, 6, 7, 8] {
        index.bind(key(k), account(42));
    }
    index.save(MAC_KEY).unwrap();

    let reloaded = DeviceAccountIndex::load(&path, MAC_KEY).unwrap();
    assert_eq!(reloaded.len(), 4);
    for k in [5u8, 6, 7, 8] {
        assert_eq!(reloaded.account_for(&key(k)), Some(account(42)), "key {k}");
    }
}

#[test]
fn rebinding_a_key_replaces_the_account() {
    let mut index = DeviceAccountIndex::new();
    index.bind(key(1), account(10));
    assert_eq!(index.account_for(&key(1)), Some(account(10)));
    index.bind(key(1), account(20)); // a device key binds to exactly one account
    assert_eq!(index.account_for(&key(1)), Some(account(20)));
    assert_eq!(index.len(), 1);
}

#[test]
fn absent_file_is_a_fresh_empty_index_not_an_error() {
    let dir = tempdir().unwrap();
    let path = dir.path().join("does_not_exist.idx");
    // An absent file is the ONE benign-absent case: a fresh, empty index.
    let index = DeviceAccountIndex::load(&path, MAC_KEY).unwrap();
    assert!(index.is_empty());
    assert_eq!(index.account_for(&key(1)), None);
    // The loaded (empty) index knows its path and can be written to.
    let mut index = index;
    index.bind(key(1), account(10));
    index.save(MAC_KEY).unwrap();
    assert_eq!(
        DeviceAccountIndex::load(&path, MAC_KEY)
            .unwrap()
            .account_for(&key(1)),
        Some(account(10))
    );
}

// ── MAC integrity (R1 hardening — the escalation-to-OWNER defense) ────────────

#[test]
fn a_tampered_binding_is_refused_at_load_not_silently_admitted() {
    // The escalation vector: an attacker rewrites a byte of a saved index to
    // forge a binding. The MAC catches it → LOUD refusal, never a silent load.
    let dir = tempdir().unwrap();
    let path = dir.path().join("tampered.idx");
    let mut index = DeviceAccountIndex::new().with_path(&path);
    index.bind(key(1), account(10));
    index.save(MAC_KEY).unwrap();

    // Flip a byte somewhere in the middle of the file (inside the JSON body).
    let mut bytes = std::fs::read(&path).unwrap();
    let mid = bytes.len() / 2;
    bytes[mid] ^= 0x40;
    std::fs::write(&path, &bytes).unwrap();

    let err = DeviceAccountIndex::load(&path, MAC_KEY)
        .unwrap_err()
        .to_string();
    assert!(err.contains("device index"), "err: {err}");
    assert!(
        err.contains("MAC verification FAILED"),
        "tamper should trip the MAC, err: {err}"
    );
}

#[test]
fn loading_with_the_wrong_mac_key_is_refused() {
    // A store MAC'd with key A cannot be loaded with key B (a transplanted index
    // whose owner does not hold this robot's secure-storage key).
    let dir = tempdir().unwrap();
    let path = dir.path().join("wrongkey.idx");
    let mut index = DeviceAccountIndex::new().with_path(&path);
    index.bind(key(1), account(10));
    index.save(MAC_KEY).unwrap();

    let err = DeviceAccountIndex::load(&path, OTHER_KEY)
        .unwrap_err()
        .to_string();
    assert!(err.contains("MAC verification FAILED"), "err: {err}");
}

#[test]
fn a_raw_unframed_json_file_is_refused_not_silently_loaded() {
    // The pre-MAC on-disk shape (plain JSON, no frame + no tag) must NOT load —
    // an attacker cannot downgrade to an unauthenticated format. Its bytes are
    // long enough to pass the length check but carry no valid MAC.
    let dir = tempdir().unwrap();
    let path = dir.path().join("rawjson.idx");
    std::fs::write(
        &path,
        br#"{"version":1,"bindings":[{"device_key":"aa","account":"bb"}]}"#,
    )
    .unwrap();
    let err = DeviceAccountIndex::load(&path, MAC_KEY)
        .unwrap_err()
        .to_string();
    assert!(err.contains("device index"), "err: {err}");
    // A raw JSON blob's trailing bytes are not a valid HMAC tag → MAC failure.
    assert!(err.contains("MAC verification FAILED"), "err: {err}");
}

#[test]
fn a_truncated_present_file_is_refused_not_treated_as_empty() {
    // The delete-the-MAC-to-reset vector: a present-but-truncated file must be
    // refused (fail-closed), NOT silently treated as a fresh empty index (only a
    // fully ABSENT file is empty).
    let dir = tempdir().unwrap();
    let path = dir.path().join("truncated.idx");
    std::fs::write(&path, b"CERIDX01\x01\x00").unwrap(); // header start only, no body/tag
    let err = DeviceAccountIndex::load(&path, MAC_KEY)
        .unwrap_err()
        .to_string();
    assert!(err.contains("device index"), "err: {err}");
    assert!(err.contains("shorter than"), "err: {err}");
}

#[test]
fn corrupt_or_garbage_bytes_are_a_loud_error_never_a_silent_fresh_map() {
    // Fail-closed: a present-but-garbage file refuses to load rather than
    // silently becoming empty (which would deny-by-default every paired device
    // and could mask tampering).
    let dir = tempdir().unwrap();
    let path = dir.path().join("corrupt.idx");
    std::fs::write(&path, b"{ this is not valid json").unwrap();
    let err = DeviceAccountIndex::load(&path, MAC_KEY)
        .unwrap_err()
        .to_string();
    assert!(err.contains("device index"), "err: {err}");
    // Too short for a header + tag → the fail-closed length refusal.
    assert!(
        err.contains("shorter than"),
        "err should explain why: {err}"
    );
}

#[test]
fn save_without_a_path_is_a_loud_error() {
    // A pathless in-memory index cannot be saved.
    let index = DeviceAccountIndex::new();
    let err = index.save(MAC_KEY).unwrap_err().to_string();
    assert!(err.contains("no backing path"), "err: {err}");
}
