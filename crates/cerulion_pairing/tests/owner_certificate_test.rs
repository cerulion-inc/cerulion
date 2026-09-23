// SPDX-License-Identifier: MIT OR Apache-2.0
//! Owner-device admission verifies real fixed-key chains and preserves owner state.

mod common;
use cerulion_pairing::format::*;
use cerulion_pairing::verify::{OwnerCertificatePresentationWire, TrustStore};
use cerulion_pairing::{CertKind, PairingError};
use common::*;

fn proof() -> OwnerCertificatePresentationWire {
    OwnerCertificatePresentationWire::new(
        make_intermediate(pk(&sk(10)), wide_validity(), ISSUED).sign_by_roots(&[&sk(1)]),
        make_device_cert(
            pk(&sk(20)),
            acct(1),
            pk(&sk(10)),
            Scope::OWNER_FULL,
            wide_validity(),
            ISSUED,
        )
        .sign(&sk(10)),
    )
}

fn store(claimed: bool) -> TrustStore {
    let mut s = TrustStore::provision(
        rob(1),
        pk(&sk(30)),
        RootSet::new(vec![pk(&sk(1))], 1).unwrap(),
        CHASSIS,
        T_NOW,
    )
    .unwrap();
    if claimed {
        s.claim(acct(1), CHASSIS, PrincipalKind::Human, T_NOW)
            .unwrap();
    }
    s
}

#[test]
fn owner_certificate_keeps_row_bytes_and_only_advances_floor() {
    let mut s = store(true);
    let before = postcard::to_stdvec(s.access_rows()).unwrap();
    assert_eq!(
        s.verify_owner_certificate(&proof(), &pk(&sk(20)), T_NOW + 1)
            .unwrap(),
        acct(1)
    );
    assert_eq!(s.high_water_ns(), T_NOW + 1);
    assert_eq!(s.owner(), Some(acct(1)));
    assert_eq!(postcard::to_stdvec(s.access_rows()).unwrap(), before);
    let row = s.is_allowed(&acct(1), T_NOW + 1).unwrap();
    assert_eq!(row.scope, Scope::OWNER_FULL);
    assert_eq!(row.added_at_ns, T_NOW);
    assert_eq!(row.expires_at_ns, None);
    assert_eq!(row.source, cerulion_pairing::verify::PairingSource::Claim);
    let dir = tempfile::tempdir().unwrap();
    let a = s.clone().with_path(dir.path().join("a"));
    a.save(MAC_KEY).unwrap();
    let mut b = store(true).with_path(dir.path().join("b"));
    b.verify_owner_certificate(&proof(), &pk(&sk(20)), T_NOW + 1)
        .unwrap();
    b.save(MAC_KEY).unwrap();
    assert_eq!(
        std::fs::read(dir.path().join("a")).unwrap(),
        std::fs::read(dir.path().join("b")).unwrap()
    );
    let loaded = TrustStore::load(dir.path().join("a"), MAC_KEY).unwrap();
    assert_eq!(loaded.high_water_ns(), T_NOW + 1);
    assert_eq!(postcard::to_stdvec(loaded.access_rows()).unwrap(), before);
}

#[test]
fn unclaimed_and_different_owner_and_peer_are_refused_without_state_change() {
    assert!(matches!(
        store(false).verify_owner_certificate(&proof(), &pk(&sk(20)), T_NOW),
        Err(PairingError::RobotUnclaimed)
    ));
    let mut s = store(true);
    let before = postcard::to_stdvec(s.access_rows()).unwrap();
    let mut p = proof();
    p.device_cert.cert.account = acct(2);
    p.device_cert = p.device_cert.cert.sign(&sk(10));
    assert!(matches!(
        s.verify_owner_certificate(&p, &pk(&sk(20)), T_NOW + 1),
        Err(PairingError::NotOwner)
    ));
    assert!(matches!(
        s.verify_owner_certificate(&proof(), &pk(&sk(21)), T_NOW + 1),
        Err(PairingError::PeerKeyMismatch)
    ));
    assert_eq!(s.high_water_ns(), T_NOW);
    assert_eq!(postcard::to_stdvec(s.access_rows()).unwrap(), before);
}

#[test]
fn certificate_signatures_issuer_scope_and_time_remain_load_bearing() {
    let mut s = store(true);
    let mut bad = proof();
    bad.device_cert.signature.0[0] ^= 1;
    assert!(matches!(
        s.verify_owner_certificate(&bad, &pk(&sk(20)), T_NOW),
        Err(PairingError::BadSignature(CertKind::Device))
    ));
    bad = proof();
    bad.intermediate.signatures.clear();
    assert!(matches!(
        s.verify_owner_certificate(&bad, &pk(&sk(20)), T_NOW),
        Err(PairingError::RootThresholdNotMet { .. })
    ));
    bad = proof();
    bad.device_cert.cert.issuer_key = pk(&sk(11));
    bad.device_cert = bad.device_cert.cert.sign(&sk(11));
    assert!(matches!(
        s.verify_owner_certificate(&bad, &pk(&sk(20)), T_NOW),
        Err(PairingError::IssuerMismatch(CertKind::Device))
    ));
    bad = proof();
    bad.intermediate.cert.max_scope = op_scope();
    bad.intermediate = bad.intermediate.cert.sign_by_roots(&[&sk(1)]);
    assert!(matches!(
        s.verify_owner_certificate(&bad, &pk(&sk(20)), T_NOW),
        Err(PairingError::ScopeExceedsIntermediate)
    ));
    bad.device_cert.cert.scope = op_scope();
    bad.device_cert = bad.device_cert.cert.sign(&sk(10));
    assert!(matches!(
        s.verify_owner_certificate(&bad, &pk(&sk(20)), T_NOW),
        Err(PairingError::OwnerCertificateScopeInsufficient)
    ));
    for (not_before_ns, not_after_ns, expired) in [(0, T_NOW, true), (T_NOW + 1, T_NOW + 2, false)]
    {
        bad = proof();
        bad.device_cert.cert.validity = Validity {
            not_before_ns,
            not_after_ns,
        };
        bad.device_cert = bad.device_cert.cert.sign(&sk(10));
        let err = s
            .verify_owner_certificate(&bad, &pk(&sk(20)), T_NOW)
            .unwrap_err();
        assert!(if expired {
            matches!(
                err,
                PairingError::Expired {
                    kind: CertKind::Device,
                    ..
                }
            )
        } else {
            matches!(
                err,
                PairingError::NotYetValid {
                    kind: CertKind::Device,
                    ..
                }
            )
        });
    }
    assert!(matches!(
        s.verify_owner_certificate(&proof(), &pk(&sk(20)), T_NOW - 1),
        Err(PairingError::RollbackDetected { .. })
    ));
    assert_eq!(s.high_water_ns(), T_NOW);
}

#[test]
fn account_and_device_revocations_cannot_be_extended_by_owner_certificates() {
    for device_only in [false, true] {
        let mut s = store(true);
        let epoch = make_epoch_with_devices(
            rob(1),
            1,
            if device_only { vec![] } else { vec![acct(1)] },
            if device_only {
                vec![pk(&sk(20))]
            } else {
                vec![]
            },
            pk(&sk(10)),
            ISSUED,
        )
        .sign(&sk(10));
        s.apply_epoch(&epoch, &proof().intermediate, T_NOW).unwrap();
        let err = s
            .verify_owner_certificate(&proof(), &pk(&sk(20)), T_NOW + 1)
            .unwrap_err();
        assert!(if device_only {
            matches!(err, PairingError::RevokedDeviceByEpoch { epoch: 1 })
        } else {
            matches!(err, PairingError::RevokedByEpoch { epoch: 1 })
        });
        assert_eq!(s.high_water_ns(), T_NOW);
    }
}

#[test]
fn owner_certificate_wire_refuses_wrong_version_truncation_and_trailing_bytes() {
    let good = proof();
    let bytes = good.to_postcard().unwrap();
    assert_eq!(bytes[0], 1);
    assert_eq!(
        OwnerCertificatePresentationWire::from_postcard(&bytes).unwrap(),
        good
    );
    for len in 0..bytes.len() {
        assert!(
            OwnerCertificatePresentationWire::from_postcard(&bytes[..len]).is_err(),
            "prefix {len}"
        );
    }
    let mut appended = bytes;
    appended.push(0);
    assert!(OwnerCertificatePresentationWire::from_postcard(&appended)
        .unwrap_err()
        .to_string()
        .contains("trailing bytes"));
    assert!(OwnerCertificatePresentationWire::from_postcard(&[2])
        .unwrap_err()
        .to_string()
        .contains("version 2"));
    let mut future = good;
    future.version = 2;
    assert!(future.to_postcard().is_err());
    assert!(store(true)
        .verify_owner_certificate(&future, &pk(&sk(20)), T_NOW)
        .is_err());
}

#[test]
fn owner_certificate_wire_field_order_matches_an_independent_postcard_oracle() {
    // Postcard integers are unsigned varints; fixed arrays have no length prefix,
    // signatures use serialize_bytes (a length prefix), Human is enum variant 0.
    fn uint(out: &mut Vec<u8>, mut value: u64) {
        while value >= 128 {
            out.push((value as u8 & 127) | 128);
            value >>= 7;
        }
        out.push(value as u8);
    }
    let p = proof();
    let mut expected = vec![1, 1]; // envelope version, intermediate version
    expected.extend_from_slice(&pk(&sk(10)).0);
    uint(&mut expected, 0);
    uint(&mut expected, 100_000_000_000_000);
    uint(&mut expected, ISSUED);
    uint(&mut expected, 1); // OWNER role
    uint(&mut expected, Scope::OWNER_FULL.caps);
    expected.push(1); // one root signature
    expected.extend_from_slice(&pk(&sk(1)).0);
    expected.push(64);
    expected.extend_from_slice(&p.intermediate.signatures[0].signature.0);
    expected.push(1); // device certificate version
    expected.extend_from_slice(&pk(&sk(20)).0);
    expected.extend_from_slice(&[1; 32]); // owner account
    expected.push(0); // Human
    uint(&mut expected, 1);
    uint(&mut expected, Scope::OWNER_FULL.caps);
    uint(&mut expected, 0);
    uint(&mut expected, 100_000_000_000_000);
    uint(&mut expected, ISSUED);
    expected.extend_from_slice(&pk(&sk(10)).0);
    expected.push(64);
    expected.extend_from_slice(&p.device_cert.signature.0);
    assert_eq!(p.to_postcard().unwrap(), expected);
    assert_eq!(
        OwnerCertificatePresentationWire::from_postcard(&expected).unwrap(),
        p
    );
}
