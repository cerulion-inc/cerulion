// SPDX-License-Identifier: AGPL-3.0-only
//! The PUBLIC beacon-facts writer contract — atomic write,
//! reader round-trip, overwrite, the `claimable` flip, the shared on-disk shape,
//! and the endpoint-driven `refresh` seam over a REAL iroh endpoint.
//!
//! Oracle-vector (never a self-compare — Principle #13): every field is a
//! hand-chosen value asserted back literally after a round-trip.

use cerulion_pairing::format::AccountId;
use cerulion_pairing::verify::OwnershipState;
use cerulion_remoted::beacon_facts::{self, BeaconFacts, BEACON_FACTS_VERSION};
use tempfile::tempdir;

#[test]
fn write_then_load_round_trips_against_a_hand_oracle() {
    let dir = tempdir().unwrap();
    let path = dir.path().join("beacon_facts.json");

    // Hand oracle: a specific key, port, and claimable flag.
    let facts = BeaconFacts {
        eid: "aa".repeat(32),
        iroh_port: 41234,
        claimable: "1".to_string(),
    };
    facts.write_atomically(&path).unwrap();

    let loaded = BeaconFacts::load(&path).unwrap();
    assert_eq!(loaded.eid, "aa".repeat(32));
    assert_eq!(loaded.iroh_port, 41234);
    assert_eq!(loaded.claimable, "1");
    assert_eq!(loaded, facts, "the whole struct round-trips");
}

#[test]
fn write_is_atomic_no_tmp_file_left_and_the_file_is_complete_json() {
    let dir = tempdir().unwrap();
    let path = dir.path().join("beacon_facts.json");
    let tmp = dir.path().join("beacon_facts.json.tmp");

    BeaconFacts {
        eid: "bb".repeat(32),
        iroh_port: 7000,
        claimable: "0".to_string(),
    }
    .write_atomically(&path)
    .unwrap();

    // No sibling `.tmp` survives the rename (atomic — a reader never sees a
    // torn/partial file).
    assert!(
        !tmp.exists(),
        "the temp file must be renamed away, not left behind"
    );
    // The persisted file parses as complete JSON carrying every field.
    let value: serde_json::Value = serde_json::from_slice(&std::fs::read(&path).unwrap()).unwrap();
    assert_eq!(value["version"], BEACON_FACTS_VERSION);
    assert_eq!(value["eid"], "bb".repeat(32));
    assert_eq!(value["iroh_port"], 7000);
    assert_eq!(value["claimable"], "0");
}

#[test]
fn the_on_disk_shape_matches_the_mdns_reader_oracle() {
    // The CROSS-CRATE lockstep pin: the gateway reader
    // (cerulion_cli_engine::mdns_discovery) parses this exact key SHAPE. This
    // test asserts the writer emits EXACTLY the {version, eid, iroh_port,
    // claimable} key set (value TYPES fixed by the struct); the mDNS side's
    // `parse_beacon_facts_full_valid_document` parses a literal with those same
    // keys. If either drifts a key name / type, one pin fails. (The two are NOT
    // byte-for-byte equal — this writer pretty-prints; the parse contract is the
    // shared invariant, not the serialization.)
    let dir = tempdir().unwrap();
    let path = dir.path().join("beacon_facts.json");
    BeaconFacts {
        eid: "cd".repeat(32),
        iroh_port: 55555,
        claimable: "1".to_string(),
    }
    .write_atomically(&path)
    .unwrap();

    let value: serde_json::Value = serde_json::from_slice(&std::fs::read(&path).unwrap()).unwrap();
    // Exactly the four documented keys — no more, no fewer.
    let obj = value.as_object().expect("a JSON object");
    let mut keys: Vec<&str> = obj.keys().map(String::as_str).collect();
    keys.sort_unstable();
    assert_eq!(keys, vec!["claimable", "eid", "iroh_port", "version"]);
}

#[test]
fn refresh_overwrites_and_claimable_flips_with_ownership() {
    // "refresh overwrites" + "claimable flips with a hand-built Unclaimed vs
    // Claimed store state" — driven through `write_atomically` (the write half
    // `refresh` calls) with a hand-built OwnershipState (no full store needed).
    let dir = tempdir().unwrap();
    let path = dir.path().join("beacon_facts.json");
    let key = [7u8; 32];

    // Unclaimed → claimable "1".
    BeaconFacts::from_parts(&key, 41234, OwnershipState::Unclaimed)
        .write_atomically(&path)
        .unwrap();
    let first = BeaconFacts::load(&path).unwrap();
    assert_eq!(first.claimable, "1");

    // A later refresh after a claim OVERWRITES the same file; only claimable
    // flips to "0" (the eid + port are unchanged).
    BeaconFacts::from_parts(&key, 41234, OwnershipState::Claimed(AccountId([2u8; 32])))
        .write_atomically(&path)
        .unwrap();
    let second = BeaconFacts::load(&path).unwrap();
    assert_eq!(second.claimable, "0", "claimable flips on claim");
    assert_eq!(second.eid, first.eid, "eid unchanged");
    assert_eq!(second.iroh_port, first.iroh_port, "port unchanged");
}

#[test]
fn load_missing_file_is_a_loud_error() {
    // The reader half surfaces a loud error for an absent file (the gateway
    // treats a missing file as "no enrichment", but `load` itself reports the absence).
    let dir = tempdir().unwrap();
    let err = BeaconFacts::load(&dir.path().join("nope.json"))
        .unwrap_err()
        .to_string();
    assert!(err.contains("beacon facts"), "err: {err}");
    assert!(
        err.contains("reading the beacon facts failed"),
        "err: {err}"
    );
}

#[test]
fn load_malformed_json_is_a_loud_error() {
    let dir = tempdir().unwrap();
    let path = dir.path().join("bad.json");
    std::fs::write(&path, b"{ not valid json").unwrap();
    let err = BeaconFacts::load(&path).unwrap_err().to_string();
    assert!(
        err.contains("parsing the beacon facts failed"),
        "err: {err}"
    );
}

/// The endpoint-driven `refresh` seam over a REAL iroh endpoint: the published
/// facts carry the endpoint's OWN public key (`eid`) and its REAL bound UDP port
/// — proving the `bound_port` + `endpoint.id()` wiring end-to-end.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn refresh_publishes_the_real_endpoint_eid_and_bound_port() {
    use cerulion_link::{build_endpoint, EndpointConfig, RelayConfig};

    let dir = tempdir().unwrap();
    let path = dir.path().join("beacon_facts.json");

    let secret = [7u8; 32];
    let endpoint = build_endpoint(EndpointConfig::new(secret).with_relay(RelayConfig::Disabled))
        .await
        .expect("endpoint binds");

    beacon_facts::refresh(&path, &endpoint, OwnershipState::Unclaimed)
        .expect("refresh publishes the facts");

    let loaded = BeaconFacts::load(&path).unwrap();
    // eid == the endpoint's OWN public key hex (independent oracle: the endpoint
    // id, not the input secret re-derived through beacon_facts).
    assert_eq!(
        loaded.eid,
        hex::encode(endpoint.id().as_bytes()),
        "published eid is the endpoint's own public key"
    );
    // A real, non-zero bound UDP port — exactly the endpoint's bound port.
    let expected_port = cerulion_link::bound_port(&endpoint).expect("a bound endpoint has a port");
    assert_ne!(expected_port, 0, "a bound endpoint reports a non-zero port");
    assert_eq!(
        loaded.iroh_port, expected_port,
        "the published port is the endpoint's bound port"
    );
    // Unclaimed → claimable "1".
    assert_eq!(loaded.claimable, "1");

    endpoint.close().await;
}
