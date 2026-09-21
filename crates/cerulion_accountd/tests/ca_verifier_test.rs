// SPDX-License-Identifier: AGPL-3.0-only
//! The A0 headline cross-check: the account service is the ISSUER end of the
//! SHIPPED `cerulion_pairing` cert chain, so an artifact it issues must be
//! byte-identical to a hand-built one AND must be accepted by the real verifier.
//!
//! Two independent proofs, never a self-compare:
//! 1. **Byte parity** — build a `DeviceCert` BY HAND from known keys, sign it, and
//!    byte-compare (canonical signing payload + full postcard container) against
//!    the issuer's output for the same inputs. Catches any field-mapping bug in
//!    the issuer (swapped account/issuer, wrong validity, wrong issuer key).
//! 2. **Verifier acceptance** — feed the issued device cert + intermediate +
//!    grant, anchored on the CA's root set, through the shipped
//!    `TrustStore::verify_new_pairing` and assert the resolved account/scope.

use ed25519_dalek::SigningKey;

use cerulion_accountd::{Ca, CaConfig};
use cerulion_pairing::format::{
    AccountId, DeviceCert, IntermediateCert, PrincipalKind, PublicKey, RobotId, RootSet, Scope,
    SignedPayload, Validity, FORMAT_VERSION,
};
use cerulion_pairing::verify::{PairingPresentation, TrustStore};

const SEC_NS: u64 = 1_000_000_000;
// A realistic fixed "now" (~2024) so the validity windows are meaningful.
const NOW: u64 = 1_700_000_000 * SEC_NS;

fn sk(seed: u8) -> SigningKey {
    SigningKey::from_bytes(&[seed; 32])
}

fn pk(k: &SigningKey) -> PublicKey {
    PublicKey(k.verifying_key().to_bytes())
}

/// Build a CA from KNOWN keys (via `from_parts`) so the byte-parity cross-check
/// can hand-sign with the same intermediate key. Returns `(ca, intermediate_key)`.
fn known_ca(
    config: &CaConfig,
) -> (
    Ca,
    SigningKey,
    cerulion_pairing::format::SignedIntermediateCert,
    RootSet,
) {
    let roots = [sk(1), sk(2), sk(3)];
    let root_set = RootSet::new(roots.iter().map(pk).collect(), 2).unwrap();
    let inter_key = sk(10);
    let cert = IntermediateCert {
        version: FORMAT_VERSION,
        intermediate_key: pk(&inter_key),
        validity: Validity {
            not_before_ns: NOW,
            not_after_ns: NOW + config.intermediate_ttl_ns,
        },
        issued_at_ns: NOW,
        max_scope: Scope::OWNER_FULL,
    };
    let root_refs: Vec<&SigningKey> = roots.iter().collect();
    let signed_inter = cert.sign_by_roots(&root_refs);
    let ca = Ca::from_parts(
        root_set.clone(),
        signed_inter.clone(),
        inter_key.clone(),
        AccountId([9u8; 32]),
        config.clone(),
    )
    .expect("matching key pair");
    (ca, inter_key, signed_inter, root_set)
}

#[test]
fn issued_device_cert_is_byte_identical_to_a_hand_built_one() {
    let config = CaConfig::default();
    let (ca, inter_key, _signed_inter, _root_set) = known_ca(&config);

    let device_key = pk(&sk(20));
    let subject = AccountId([42u8; 32]);

    let issued = ca.issue_device_cert(
        device_key,
        subject,
        PrincipalKind::Human,
        Scope::OWNER_FULL,
        NOW,
    );

    // The independent oracle: hand-build the SAME cert body and sign it with the
    // SAME intermediate key. Equality proves the issuer mapped every field
    // correctly (nothing swapped, right validity/issuer key).
    let hand = DeviceCert {
        version: FORMAT_VERSION,
        device_key,
        account: subject,
        principal_kind: PrincipalKind::Human,
        scope: Scope::OWNER_FULL,
        validity: Validity {
            not_before_ns: NOW,
            not_after_ns: NOW + config.device_cert_ttl_ns,
        },
        issued_at_ns: NOW,
        issuer_key: pk(&inter_key),
    }
    .sign(&inter_key);

    // (a) Canonical signing-byte parity — the deepest layout cross-check.
    assert_eq!(
        issued.cert.signing_payload(),
        hand.cert.signing_payload(),
        "issuer canonical signing bytes must equal the hand-built cert's"
    );
    // (b) ed25519 is deterministic → identical signature over identical bytes.
    assert_eq!(
        issued.signature, hand.signature,
        "signatures must be identical"
    );
    // (c) Full postcard container parity — the exact bytes that cross the wire.
    assert_eq!(
        postcard::to_stdvec(&issued).unwrap(),
        postcard::to_stdvec(&hand).unwrap(),
        "issuer wire bytes must equal the hand-built cert's"
    );
}

#[test]
fn from_parts_rejects_a_mismatched_intermediate_key_pair() {
    let config = CaConfig::default();
    let roots = [sk(1), sk(2), sk(3)];
    let root_set = RootSet::new(roots.iter().map(pk).collect(), 2).unwrap();
    // The cert embeds inter_key_a's PUBLIC key ...
    let inter_key_a = sk(10);
    let cert = IntermediateCert {
        version: FORMAT_VERSION,
        intermediate_key: pk(&inter_key_a),
        validity: Validity {
            not_before_ns: NOW,
            not_after_ns: NOW + config.intermediate_ttl_ns,
        },
        issued_at_ns: NOW,
        max_scope: Scope::OWNER_FULL,
    };
    let root_refs: Vec<&SigningKey> = roots.iter().collect();
    let signed_inter = cert.sign_by_roots(&root_refs);
    // ... but a DIFFERENT private key is loaded alongside it → mismatch → loud Err.
    let mismatched_private = sk(11);
    let err = Ca::from_parts(
        root_set,
        signed_inter,
        mismatched_private,
        AccountId([9u8; 32]),
        config,
    )
    .expect_err("a mismatched intermediate key pair must be rejected");
    assert!(
        matches!(err, cerulion_accountd::AccountdError::Internal(_)),
        "got {err:?}"
    );
}

#[test]
fn issued_chain_is_accepted_by_the_shipped_verifier() {
    let config = CaConfig::default();
    let (ca, _inter_key, signed_inter, root_set) = known_ca(&config);

    let device_key = pk(&sk(20));
    let subject = AccountId([42u8; 32]);
    let robot = RobotId([77u8; 32]);

    let device_cert = ca.issue_device_cert(
        device_key,
        subject,
        PrincipalKind::Human,
        Scope::OWNER_FULL,
        NOW,
    );
    let grant = ca.issue_grant(subject, robot, Scope::OWNER_FULL, PrincipalKind::Human, NOW);

    // A real robot-side trust store, anchored on the CA's root set, claimed by an
    // owner (a new pairing requires a claimed robot).
    let chassis = b"unit-test-chassis-secret";
    let mut store = TrustStore::provision(robot, pk(&sk(30)), root_set, chassis, NOW).unwrap();
    store
        .claim(AccountId([1u8; 32]), chassis, PrincipalKind::Human, NOW)
        .unwrap();

    let pres = PairingPresentation {
        intermediate: signed_inter,
        device_cert,
        grant,
        delegation: None,
    };
    let verified = store
        .verify_new_pairing(&pres, &device_key, NOW)
        .expect("the shipped verifier must accept the issuer's chain");
    assert_eq!(verified.account(), subject);
    assert_eq!(verified.scope(), Scope::OWNER_FULL);
    assert_eq!(verified.device_key(), device_key);
}

#[test]
fn dev_provisioned_ca_chain_also_verifies() {
    // The dev-provisioning path (random roots + intermediate) produces a working
    // chain end to end — the shape the running service uses.
    let ca = Ca::dev_provision(CaConfig::default(), NOW).unwrap();

    let device_key = pk(&sk(21));
    let subject = AccountId([43u8; 32]);
    let robot = RobotId([78u8; 32]);

    let device_cert = ca.issue_device_cert(
        device_key,
        subject,
        PrincipalKind::Human,
        Scope::OWNER_FULL,
        NOW,
    );
    let grant = ca.issue_grant(subject, robot, Scope::OWNER_FULL, PrincipalKind::Human, NOW);

    let chassis = b"chassis";
    let mut store =
        TrustStore::provision(robot, pk(&sk(31)), ca.root_set().clone(), chassis, NOW).unwrap();
    store
        .claim(AccountId([2u8; 32]), chassis, PrincipalKind::Human, NOW)
        .unwrap();

    let pres = PairingPresentation {
        intermediate: ca.intermediate().clone(),
        device_cert,
        grant,
        delegation: None,
    };
    let verified = store.verify_new_pairing(&pres, &device_key, NOW).unwrap();
    assert_eq!(verified.account(), subject);
}

#[test]
fn verifier_rejects_a_cert_whose_key_is_not_the_authenticated_peer() {
    // The device-key ↔ authenticated-peer binding is real, not vacuous: a cert for
    // key K presented by a DIFFERENT authenticated peer is rejected.
    let ca = Ca::dev_provision(CaConfig::default(), NOW).unwrap();
    let device_key = pk(&sk(22));
    let wrong_peer = pk(&sk(99));
    let subject = AccountId([44u8; 32]);
    let robot = RobotId([79u8; 32]);

    let device_cert = ca.issue_device_cert(
        device_key,
        subject,
        PrincipalKind::Human,
        Scope::OWNER_FULL,
        NOW,
    );
    let grant = ca.issue_grant(subject, robot, Scope::OWNER_FULL, PrincipalKind::Human, NOW);

    let chassis = b"chassis";
    let mut store =
        TrustStore::provision(robot, pk(&sk(32)), ca.root_set().clone(), chassis, NOW).unwrap();
    store
        .claim(AccountId([3u8; 32]), chassis, PrincipalKind::Human, NOW)
        .unwrap();

    let pres = PairingPresentation {
        intermediate: ca.intermediate().clone(),
        device_cert,
        grant,
        delegation: None,
    };
    // Present the cert against the WRONG peer key → PeerKeyMismatch.
    let err = store
        .verify_new_pairing(&pres, &wrong_peer, NOW)
        .expect_err("a cert must not verify against a non-matching peer key");
    assert!(
        matches!(err, cerulion_pairing::PairingError::PeerKeyMismatch),
        "expected PeerKeyMismatch, got {err:?}"
    );
}
