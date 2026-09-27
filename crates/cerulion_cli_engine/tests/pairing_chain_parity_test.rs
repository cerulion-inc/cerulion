// SPDX-License-Identifier: AGPL-3.0-only
//! Independent canonical/postcard oracles shared with the account web app.

use cerulion_pairing::format::{
    AccountId, DeviceCert, IntermediateCert, PrincipalKind, PublicKey, Role, RootSet,
    RootSignature, Scope, Signature, SignedDeviceCert, SignedIntermediateCert, SignedPayload,
    Validity,
};
use serde_json::Value;

fn fixture() -> Value {
    serde_json::from_str(include_str!("fixtures/pairing-chain-v1.json")).unwrap()
}
fn unhex(value: &str) -> Vec<u8> {
    assert!(value.len().is_multiple_of(2));
    value
        .as_bytes()
        .as_chunks::<2>()
        .0
        .iter()
        .map(|pair| u8::from_str_radix(std::str::from_utf8(pair).unwrap(), 16).unwrap())
        .collect()
}
fn bytes(value: &Value, field: &str) -> Vec<u8> {
    unhex(value[field].as_str().unwrap())
}
fn key(value: &Value, field: &str) -> PublicKey {
    PublicKey(bytes(value, field).try_into().unwrap())
}
fn decimal(value: &Value, field: &str) -> u64 {
    value[field].as_str().unwrap().parse().unwrap()
}
fn validity(value: &Value) -> Validity {
    Validity {
        not_before_ns: decimal(value, "notBeforeNs"),
        not_after_ns: decimal(value, "notAfterNs"),
    }
}
fn scope(value: &Value) -> Scope {
    Scope {
        role: Role(value["role"].as_u64().unwrap().try_into().unwrap()),
        caps: decimal(value, "caps"),
    }
}

#[test]
fn root_set_postcard_matches_fixed_public_key_order_and_threshold() {
    let fixture = fixture();
    let value = &fixture["rootSet"];
    let keys = value["keysHex"]
        .as_array()
        .unwrap()
        .iter()
        .map(|value| PublicKey(unhex(value.as_str().unwrap()).try_into().unwrap()))
        .collect();
    let roots = RootSet::new(
        keys,
        value["threshold"].as_u64().unwrap().try_into().unwrap(),
    )
    .unwrap();
    let oracle = bytes(value, "postcardHex");
    assert_eq!(postcard::to_stdvec(&roots).unwrap(), oracle);
    assert_eq!(postcard::from_bytes::<RootSet>(&oracle).unwrap(), roots);
}

#[test]
fn intermediate_signing_bytes_and_postcard_preserve_u64_and_open_scope_values() {
    let fixture = fixture();
    let value = &fixture["intermediate"];
    let fields = &value["fields"];
    let signed = SignedIntermediateCert {
        cert: IntermediateCert {
            version: 1,
            intermediate_key: key(fields, "intermediateKeyHex"),
            validity: validity(fields),
            issued_at_ns: decimal(fields, "issuedAtNs"),
            max_scope: scope(fields),
        },
        // Fixed byte-pattern signatures test serialization, not authenticity.
        signatures: vec![RootSignature {
            root_key: key(fields, "rootKeyHex"),
            signature: Signature(bytes(fields, "signatureHex").try_into().unwrap()),
        }],
    };
    assert_eq!(signed.cert.signing_payload(), bytes(value, "canonicalHex"));
    let oracle = bytes(value, "postcardHex");
    assert_eq!(postcard::to_stdvec(&signed).unwrap(), oracle);
    assert_eq!(
        postcard::from_bytes::<SignedIntermediateCert>(&oracle).unwrap(),
        signed
    );
}

#[test]
fn device_principals_have_different_canonical_and_postcard_discriminants() {
    let fixture = fixture();
    for value in fixture["devices"].as_array().unwrap() {
        let fields = &value["fields"];
        let principal_kind = match fields["principalKind"].as_str().unwrap() {
            "human" => PrincipalKind::Human,
            "machine" => PrincipalKind::Machine,
            unexpected => panic!("unexpected fixture principal: {unexpected}"),
        };
        let signed = SignedDeviceCert {
            cert: DeviceCert {
                version: 1,
                device_key: key(fields, "deviceKeyHex"),
                account: AccountId(bytes(fields, "accountHex").try_into().unwrap()),
                principal_kind,
                scope: scope(fields),
                validity: validity(fields),
                issued_at_ns: decimal(fields, "issuedAtNs"),
                issuer_key: key(fields, "issuerKeyHex"),
            },
            signature: Signature(bytes(fields, "signatureHex").try_into().unwrap()),
        };
        assert_eq!(signed.cert.signing_payload(), bytes(value, "canonicalHex"));
        let oracle = bytes(value, "postcardHex");
        assert_eq!(postcard::to_stdvec(&signed).unwrap(), oracle);
        assert_eq!(
            postcard::from_bytes::<SignedDeviceCert>(&oracle).unwrap(),
            signed
        );
    }
}
