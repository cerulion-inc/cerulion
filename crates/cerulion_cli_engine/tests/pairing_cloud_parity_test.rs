// SPDX-License-Identifier: AGPL-3.0-only
//! Independent canonical/postcard oracles for a cloud-principal device
//! certificate, shared with the account web app (`fixtures/pairing-cloud-v1.json`).

use cerulion_pairing::format::{
    AccountId, DeviceCert, PrincipalKind, PublicKey, Role, Scope, Signature, SignedDeviceCert,
    SignedPayload, Validity,
};
use serde::Deserialize;
use serde_json::Value;

fn fixture() -> Value {
    serde_json::from_str(include_str!("fixtures/pairing-cloud-v1.json")).unwrap()
}
fn unhex(value: &str) -> Vec<u8> {
    assert!(value.len().is_multiple_of(2));
    (0..value.len())
        .step_by(2)
        .map(|at| u8::from_str_radix(&value[at..at + 2], 16).unwrap())
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

#[test]
fn cloud_device_signing_bytes_and_postcard_match_the_fixture() {
    let fixture = fixture();
    let value = &fixture["cloudDevice"];
    let fields = &value["fields"];
    assert_eq!(fields["principalKind"], "cloud");
    let signed = SignedDeviceCert {
        cert: DeviceCert {
            version: 1,
            device_key: key(fields, "deviceKeyHex"),
            account: AccountId(bytes(fields, "accountHex").try_into().unwrap()),
            principal_kind: PrincipalKind::Cloud,
            scope: Scope {
                role: Role(fields["role"].as_u64().unwrap().try_into().unwrap()),
                caps: decimal(fields, "caps"),
            },
            validity: Validity {
                not_before_ns: decimal(fields, "notBeforeNs"),
                not_after_ns: decimal(fields, "notAfterNs"),
            },
            issued_at_ns: decimal(fields, "issuedAtNs"),
            issuer_key: key(fields, "issuerKeyHex"),
        },
        // A fixed byte-pattern signature tests serialization, not authenticity.
        signature: Signature(bytes(fields, "signatureHex").try_into().unwrap()),
    };
    let canonical = bytes(value, "canonicalHex");
    assert_eq!(signed.cert.signing_payload(), canonical);
    // Signing discriminant 3: the byte after domain, version, key and account.
    assert_eq!(canonical[4 + 31 + 2 + 32 + 32], 3);
    let oracle = bytes(value, "postcardHex");
    assert_eq!(postcard::to_stdvec(&signed).unwrap(), oracle);
    // Postcard variant index 2: the byte after version, key and account.
    assert_eq!(oracle[1 + 32 + 32], 2);
    assert_eq!(
        postcard::from_bytes::<SignedDeviceCert>(&oracle).unwrap(),
        signed
    );
}

/// The principal enum as a decoder built before the cloud principal knew it.
#[derive(Deserialize, Debug)]
enum PrincipalKindBeforeCloud {
    Human,
    Machine,
}

/// The signed device certificate with that older principal enum.
#[derive(Deserialize, Debug)]
#[allow(dead_code)]
struct SignedDeviceCertBeforeCloud {
    version: u16,
    device_key: PublicKey,
    account: AccountId,
    principal_kind: PrincipalKindBeforeCloud,
    scope: Scope,
    validity: Validity,
    issued_at_ns: u64,
    issuer_key: PublicKey,
    signature: Signature,
}

#[test]
fn a_decoder_without_the_cloud_principal_refuses_a_cloud_certificate() {
    // Positive control: the older decoder reads a human certificate.
    let chain: Value =
        serde_json::from_str(include_str!("fixtures/pairing-chain-v1.json")).unwrap();
    let human = chain["devices"]
        .as_array()
        .unwrap()
        .iter()
        .find(|d| d["fields"]["principalKind"] == "human")
        .unwrap();
    let decoded: SignedDeviceCertBeforeCloud =
        postcard::from_bytes(&bytes(human, "postcardHex")).unwrap();
    assert!(matches!(
        decoded.principal_kind,
        PrincipalKindBeforeCloud::Human
    ));
    // The cloud certificate fails closed there.
    let cloud = &fixture()["cloudDevice"];
    assert!(
        postcard::from_bytes::<SignedDeviceCertBeforeCloud>(&bytes(cloud, "postcardHex")).is_err()
    );
}
