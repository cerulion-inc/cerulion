// SPDX-License-Identifier: AGPL-3.0-only
use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use cerulion_cli_engine::account_identity::pairing_account_id;

#[test]
fn hosted_uuid_mapping_matches_shared_protocol_oracles_without_changing_legacy_ids() {
    let vectors: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/pairing-v1.json")).unwrap();
    for row in vectors["accounts"].as_array().unwrap() {
        let uuid = row["uuid"].as_str().unwrap();
        let expected = row["accountBase64url"].as_str().unwrap();
        let account = pairing_account_id(uuid).unwrap();
        assert_eq!(URL_SAFE_NO_PAD.encode(account.0), expected);
        assert_eq!(
            pairing_account_id(&uuid.to_ascii_uppercase()).unwrap(),
            account
        );
        assert_eq!(pairing_account_id(expected).unwrap(), account);
        assert_eq!(pairing_account_id(uuid).unwrap(), account);
    }
}

#[test]
fn malformed_auth_labels_do_not_become_pairing_identities() {
    for invalid in [
        "",
        "account-b",
        " 00112233-4455-4677-8899-aabbccddeeff",
        "00112233-4455-4677-8899-aabbccddeeff ",
        "{00112233-4455-4677-8899-aabbccddeeff}",
        "00112233445546778899aabbccddeeff",
        "00112233_4455-4677-8899-aabbccddeeff",
        "00112233-4455-4677-8899-aabbccddeefg",
        "00112233-4455-4677-8899-aabbccddeefé",
        "Ah8KAGhm_eUlb1fPrQKgIvrp6ORzpuTTpHHx9L2D6Rc=",
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA",
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA",
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB",
    ] {
        assert!(pairing_account_id(invalid).is_err(), "accepted {invalid:?}");
    }
}
