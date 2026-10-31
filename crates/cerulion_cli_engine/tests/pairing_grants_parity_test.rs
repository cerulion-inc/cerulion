// SPDX-License-Identifier: AGPL-3.0-only
//! Hand-specified grant/epoch vectors shared with the account service.

use cerulion_pairing::format::{
    AccessListEpoch, AccountId, Grant, PrincipalKind, PublicKey, RobotId, Role, Scope, Signature,
    SignedEpoch, SignedGrant, SignedPayload, Validity,
};
use serde_json::Value;

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
fn fixed(value: &Value, field: &str) -> [u8; 32] {
    unhex(value[field].as_str().unwrap()).try_into().unwrap()
}
fn number(value: &Value, field: &str) -> u64 {
    value[field].as_str().unwrap().parse().unwrap()
}
fn signature(value: &Value) -> Signature {
    // These explicit byte patterns test serialization, not authentication.
    Signature(
        unhex(value["signatureHex"].as_str().unwrap())
            .try_into()
            .unwrap(),
    )
}
fn fixture() -> Value {
    serde_json::from_str(include_str!("fixtures/pairing-grants-v1.json")).unwrap()
}

#[test]
fn grant_oracles_pin_principal_depth_scope_and_both_wire_formats() {
    for vector in fixture()["grants"].as_array().unwrap() {
        let f = &vector["fields"];
        let signed = SignedGrant {
            grant: Grant {
                version: 1,
                subject: AccountId(fixed(f, "subjectHex")),
                robot: RobotId(fixed(f, "robotHex")),
                scope: Scope {
                    role: Role(f["role"].as_u64().unwrap().try_into().unwrap()),
                    caps: number(f, "caps"),
                },
                principal_kind: match f["principalKind"].as_str().unwrap() {
                    "human" => PrincipalKind::Human,
                    "machine" => PrincipalKind::Machine,
                    other => panic!("unknown fixture principal: {other}"),
                },
                delegation_depth: f["delegationDepth"].as_u64().unwrap().try_into().unwrap(),
                validity: Validity {
                    not_before_ns: number(f, "notBeforeNs"),
                    not_after_ns: number(f, "notAfterNs"),
                },
                issued_at_ns: number(f, "issuedAtNs"),
                issuer: AccountId(fixed(f, "issuerHex")),
                issuer_key: PublicKey(fixed(f, "issuerKeyHex")),
            },
            signature: signature(f),
        };
        assert_eq!(
            signed.grant.signing_payload(),
            unhex(vector["canonicalHex"].as_str().unwrap())
        );
        let oracle = unhex(vector["postcardHex"].as_str().unwrap());
        assert_eq!(postcard::to_stdvec(&signed).unwrap(), oracle);
        assert_eq!(
            postcard::from_bytes::<SignedGrant>(&oracle).unwrap(),
            signed
        );
    }
}

#[test]
fn epoch_oracles_pin_v2_domain_and_both_revocation_lists_in_order() {
    for vector in fixture()["epochs"].as_array().unwrap() {
        let f = &vector["fields"];
        let signed = SignedEpoch {
            epoch_data: AccessListEpoch {
                version: 1,
                robot: RobotId(fixed(f, "robotHex")),
                epoch: number(f, "epoch"),
                revoked_accounts: f["revokedAccountsHex"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|v| AccountId(unhex(v.as_str().unwrap()).try_into().unwrap()))
                    .collect(),
                revoked_devices: f["revokedDevicesHex"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|v| PublicKey(unhex(v.as_str().unwrap()).try_into().unwrap()))
                    .collect(),
                issued_at_ns: number(f, "issuedAtNs"),
                issuer_key: PublicKey(fixed(f, "issuerKeyHex")),
            },
            signature: signature(f),
        };
        assert_eq!(
            signed.epoch_data.signing_payload(),
            unhex(vector["canonicalHex"].as_str().unwrap())
        );
        let oracle = unhex(vector["postcardHex"].as_str().unwrap());
        assert_eq!(postcard::to_stdvec(&signed).unwrap(), oracle);
        assert_eq!(
            postcard::from_bytes::<SignedEpoch>(&oracle).unwrap(),
            signed
        );
    }
}
