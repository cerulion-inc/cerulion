// SPDX-License-Identifier: MIT OR Apache-2.0
//! Chain-verification positive path + the full negative matrix, including
//! M-of-N root threshold, peer-key binding, expiry, delegation (depth 0 and 1,
//! and the depth>1 cap), epoch revocation, and anti-rollback.

mod common;
use common::*;

use cerulion_pairing::format::*;
use cerulion_pairing::verify::*;
use cerulion_pairing::{CertKind, PairingError, MAX_DELEGATION_DEPTH};

// -- fixtures ----------------------------------------------------------------

const A_CLIENT: u8 = 0x0A; // the client's account
const ROBOT: u8 = 0x0B;
const A_ISSUER: u8 = 0x0C; // an issuing account label
const OWNER: u8 = 0x01; // the robot owner (distinct from every pairing subject)

fn root_set() -> RootSet {
    RootSet::new(vec![pk(&sk(1)), pk(&sk(2)), pk(&sk(3))], 2).unwrap()
}

fn expired() -> Validity {
    Validity {
        not_before_ns: 0,
        not_after_ns: 100, // < T_NOW
    }
}
fn future() -> Validity {
    Validity {
        not_before_ns: T_NOW + 1,
        not_after_ns: T_NOW + 1_000,
    }
}

fn good_intermediate() -> SignedIntermediateCert {
    make_intermediate(pk(&sk(10)), wide_validity(), ISSUED).sign_by_roots(&[&sk(1), &sk(2)])
}
fn good_device() -> SignedDeviceCert {
    make_device_cert(
        pk(&sk(20)),
        acct(A_CLIENT),
        pk(&sk(10)),
        op_scope(),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10))
}
fn good_grant() -> SignedGrant {
    make_grant(
        acct(A_CLIENT),
        rob(ROBOT),
        op_scope(),
        0,
        acct(A_ISSUER),
        pk(&sk(10)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10))
}
fn good_presentation() -> PairingPresentation {
    PairingPresentation {
        intermediate: good_intermediate(),
        device_cert: good_device(),
        grant: good_grant(),
        delegation: None,
    }
}
fn fresh_store() -> TrustStore {
    // A new pairing can only be established on a CLAIMED robot (RobotUnclaimed
    // otherwise), so these chain-verification tests operate on a claimed store.
    // The owner (0x01) is distinct from every pairing subject exercised here, so
    // the owner row never interferes with the account under test. `claim` at
    // T_NOW seeds the anti-rollback floor to T_NOW (the rollback/high-water tests
    // below account for this).
    // The robot's own transport key is irrelevant to these chain-verification
    // tests (none run code pairing), so any fixed key serves.
    let mut s = TrustStore::provision(rob(ROBOT), pk(&sk(0x77)), root_set(), CHASSIS, 0).unwrap();
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    s
}
fn peer() -> PublicKey {
    pk(&sk(20))
}

// -- positive ----------------------------------------------------------------

#[test]
fn happy_path_verifies_and_binds_peer_key_account_and_scope() {
    let mut store = fresh_store();
    let v = store
        .verify_new_pairing(&good_presentation(), &peer(), T_NOW)
        .expect("valid chain should verify");
    assert_eq!(v.account(), acct(A_CLIENT));
    assert_eq!(v.device_key(), peer());
    assert_eq!(v.principal_kind(), PrincipalKind::Human);
    // effective scope = intersect(op, op) = op.
    assert_eq!(v.scope(), op_scope());
}

#[test]
fn establish_then_access_decision() {
    let mut store = fresh_store();
    let v = store
        .verify_new_pairing(&good_presentation(), &peer(), T_NOW)
        .unwrap();
    assert!(
        store.is_allowed(&acct(A_CLIENT), T_NOW).is_none(),
        "no row yet"
    );
    store
        .establish_pairing(
            &v,
            PairingSource::StrongChain,
            Some("laptop".into()),
            T_NOW,
            None,
        )
        .unwrap();
    let row = store
        .is_allowed(&acct(A_CLIENT), T_NOW)
        .expect("now allowed");
    assert_eq!(row.scope, op_scope());
    assert_eq!(row.source, PairingSource::StrongChain);
    assert!(
        store.is_allowed(&acct(0xEE), T_NOW).is_none(),
        "unknown account"
    );
}

#[test]
fn effective_scope_is_the_least_privilege_intersection() {
    // Device cert grants OWNER_FULL, but the grant only VIEWER+OBSERVE.
    let dev = make_device_cert(
        pk(&sk(20)),
        acct(A_CLIENT),
        pk(&sk(10)),
        Scope::OWNER_FULL,
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let grant = make_grant(
        acct(A_CLIENT),
        rob(ROBOT),
        Scope {
            role: Role::VIEWER,
            caps: Scope::CAP_OBSERVE,
        },
        0,
        acct(A_ISSUER),
        pk(&sk(10)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let pres = PairingPresentation {
        intermediate: good_intermediate(),
        device_cert: dev,
        grant,
        delegation: None,
    };
    let mut store = fresh_store();
    let v = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap();
    assert_eq!(v.scope().role, Role::VIEWER); // max(OWNER=1, VIEWER=3) => VIEWER
    assert_eq!(v.scope().caps, Scope::CAP_OBSERVE); // OWNER_FULL.caps & OBSERVE
}

// -- peer-key binding --------------------------------------------------------

#[test]
fn wrong_peer_key_is_rejected() {
    let mut store = fresh_store();
    let other = pk(&sk(99));
    let err = store
        .verify_new_pairing(&good_presentation(), &other, T_NOW)
        .unwrap_err();
    assert!(matches!(err, PairingError::PeerKeyMismatch));
}

// -- validity ----------------------------------------------------------------

#[test]
fn expired_device_cert_rejected() {
    let dev = make_device_cert(
        pk(&sk(20)),
        acct(A_CLIENT),
        pk(&sk(10)),
        op_scope(),
        expired(),
        ISSUED,
    )
    .sign(&sk(10));
    let pres = PairingPresentation {
        device_cert: dev,
        ..good_presentation()
    };
    let mut store = fresh_store();
    let err = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap_err();
    assert!(matches!(
        err,
        PairingError::Expired {
            kind: CertKind::Device,
            ..
        }
    ));
}

#[test]
fn not_yet_valid_intermediate_rejected() {
    let inter = make_intermediate(pk(&sk(10)), future(), ISSUED).sign_by_roots(&[&sk(1), &sk(2)]);
    let pres = PairingPresentation {
        intermediate: inter,
        ..good_presentation()
    };
    let mut store = fresh_store();
    let err = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap_err();
    assert!(matches!(
        err,
        PairingError::NotYetValid {
            kind: CertKind::Intermediate,
            ..
        }
    ));
}

#[test]
fn expired_grant_rejected() {
    let grant = make_grant(
        acct(A_CLIENT),
        rob(ROBOT),
        op_scope(),
        0,
        acct(A_ISSUER),
        pk(&sk(10)),
        expired(),
        ISSUED,
    )
    .sign(&sk(10));
    let pres = PairingPresentation {
        grant,
        ..good_presentation()
    };
    let mut store = fresh_store();
    let err = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap_err();
    assert!(matches!(
        err,
        PairingError::Expired {
            kind: CertKind::Grant,
            ..
        }
    ));
}

// -- M-of-N root threshold ---------------------------------------------------

#[test]
fn intermediate_signed_by_untrusted_root_does_not_count() {
    // sk(99) is not in the root set; only sk(2) is trusted => 1 distinct < 2.
    let inter =
        make_intermediate(pk(&sk(10)), wide_validity(), ISSUED).sign_by_roots(&[&sk(99), &sk(2)]);
    let pres = PairingPresentation {
        intermediate: inter,
        ..good_presentation()
    };
    let mut store = fresh_store();
    let err = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap_err();
    assert!(matches!(
        err,
        PairingError::RootThresholdNotMet { have: 1, need: 2 }
    ));
}

#[test]
fn duplicate_root_signature_counts_once() {
    // The same trusted root signs twice => still only 1 distinct root.
    let inter =
        make_intermediate(pk(&sk(10)), wide_validity(), ISSUED).sign_by_roots(&[&sk(1), &sk(1)]);
    let pres = PairingPresentation {
        intermediate: inter,
        ..good_presentation()
    };
    let mut store = fresh_store();
    let err = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap_err();
    assert!(matches!(
        err,
        PairingError::RootThresholdNotMet { have: 1, need: 2 }
    ));
}

#[test]
fn three_of_three_distinct_roots_exceeds_threshold() {
    let inter = make_intermediate(pk(&sk(10)), wide_validity(), ISSUED).sign_by_roots(&[
        &sk(1),
        &sk(2),
        &sk(3),
    ]);
    let pres = PairingPresentation {
        intermediate: inter,
        ..good_presentation()
    };
    let mut store = fresh_store();
    assert!(store.verify_new_pairing(&pres, &peer(), T_NOW).is_ok());
}

#[test]
fn tampered_root_signature_drops_below_threshold() {
    let mut inter = good_intermediate();
    inter.signatures[0].signature.0[0] ^= 0xFF; // corrupt one of the two sigs
    let pres = PairingPresentation {
        intermediate: inter,
        ..good_presentation()
    };
    let mut store = fresh_store();
    let err = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap_err();
    assert!(matches!(
        err,
        PairingError::RootThresholdNotMet { have: 1, need: 2 }
    ));
}

// -- issuer / signature ------------------------------------------------------

#[test]
fn device_cert_issuer_key_must_match_intermediate() {
    // issuer_key points at a root, not the intermediate.
    let dev = make_device_cert(
        pk(&sk(20)),
        acct(A_CLIENT),
        pk(&sk(1)),
        op_scope(),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(1));
    let pres = PairingPresentation {
        device_cert: dev,
        ..good_presentation()
    };
    let mut store = fresh_store();
    let err = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap_err();
    assert!(matches!(
        err,
        PairingError::IssuerMismatch(CertKind::Device)
    ));
}

#[test]
fn device_cert_signed_by_wrong_key_fails_signature() {
    // issuer_key = intermediate, but actually signed by a different key.
    let dev = make_device_cert(
        pk(&sk(20)),
        acct(A_CLIENT),
        pk(&sk(10)),
        op_scope(),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(77)); // wrong signer
    let pres = PairingPresentation {
        device_cert: dev,
        ..good_presentation()
    };
    let mut store = fresh_store();
    let err = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap_err();
    assert!(matches!(err, PairingError::BadSignature(CertKind::Device)));
}

// -- grant checks ------------------------------------------------------------

#[test]
fn grant_for_a_different_robot_is_rejected() {
    let grant = make_grant(
        acct(A_CLIENT),
        rob(0xFF),
        op_scope(),
        0,
        acct(A_ISSUER),
        pk(&sk(10)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let pres = PairingPresentation {
        grant,
        ..good_presentation()
    };
    let mut store = fresh_store();
    let err = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap_err();
    assert!(matches!(err, PairingError::WrongRobot));
}

#[test]
fn grant_subject_must_match_device_cert_account() {
    let grant = make_grant(
        acct(0xFF), // different subject
        rob(ROBOT),
        op_scope(),
        0,
        acct(A_ISSUER),
        pk(&sk(10)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let pres = PairingPresentation {
        grant,
        ..good_presentation()
    };
    let mut store = fresh_store();
    let err = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap_err();
    assert!(matches!(err, PairingError::SubjectMismatch));
}

#[test]
fn grant_issuer_key_must_be_the_intermediate_for_depth0() {
    let grant = make_grant(
        acct(A_CLIENT),
        rob(ROBOT),
        op_scope(),
        0,
        acct(A_ISSUER),
        pk(&sk(1)), // not the intermediate
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(1));
    let pres = PairingPresentation {
        grant,
        ..good_presentation()
    };
    let mut store = fresh_store();
    let err = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap_err();
    assert!(matches!(err, PairingError::IssuerMismatch(CertKind::Grant)));
}

// -- delegation --------------------------------------------------------------

#[test]
fn delegation_depth_greater_than_one_is_capped() {
    let grant = make_grant(
        acct(A_CLIENT),
        rob(ROBOT),
        op_scope(),
        2, // > MAX_DELEGATION_DEPTH
        acct(A_ISSUER),
        pk(&sk(10)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let pres = PairingPresentation {
        grant,
        ..good_presentation()
    };
    let mut store = fresh_store();
    let err = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap_err();
    assert!(matches!(
        err,
        PairingError::DelegationDepthExceeded {
            depth: 2,
            max
        } if max == MAX_DELEGATION_DEPTH
    ));
}

#[test]
fn depth1_grant_without_delegation_chain_is_rejected() {
    let grant = make_grant(
        acct(A_CLIENT),
        rob(ROBOT),
        op_scope(),
        1,
        acct(A_ISSUER),
        pk(&sk(30)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(30));
    let pres = PairingPresentation {
        grant,
        delegation: None,
        ..good_presentation()
    };
    let mut store = fresh_store();
    let err = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap_err();
    assert!(matches!(err, PairingError::InvalidDelegation));
}

// A full valid depth-1 delegation: the delegator (account DD, device key sk 30)
// holds a CAP_DELEGATE grant and delegates to the client (account A_CLIENT).
const A_DELEGATOR: u8 = 0xDD;

fn delegator_parent_grant(caps: u64) -> SignedGrant {
    make_grant(
        acct(A_DELEGATOR),
        rob(ROBOT),
        Scope {
            role: Role::OPERATOR,
            caps,
        },
        0,
        acct(A_ISSUER),
        pk(&sk(10)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10))
}
fn delegator_device_cert() -> SignedDeviceCert {
    make_device_cert(
        pk(&sk(30)),
        acct(A_DELEGATOR),
        pk(&sk(10)),
        op_scope(),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10))
}
fn depth1_grant() -> SignedGrant {
    make_grant(
        acct(A_CLIENT),
        rob(ROBOT),
        op_scope(),
        1,
        acct(A_DELEGATOR),
        pk(&sk(30)), // signed by the delegator's device key
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(30))
}

#[test]
fn valid_depth1_delegation_verifies_and_effective_scope_is_bounded() {
    // Parent (delegator) holds a SUPERSET of the delegated grant's scope.
    let pres = PairingPresentation {
        intermediate: good_intermediate(),
        device_cert: good_device(),
        grant: depth1_grant(), // op_scope() = OPERATOR + TELEOP|OBSERVE
        delegation: Some(Delegation {
            parent: delegator_parent_grant(
                Scope::CAP_DELEGATE | Scope::CAP_TELEOP | Scope::CAP_OBSERVE,
            ),
            delegator_cert: delegator_device_cert(),
        }),
    };
    let mut store = fresh_store();
    let v = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap();
    assert_eq!(v.account(), acct(A_CLIENT));
    // Effective scope = intersect(device_cert, grant, parent) — bounded by all
    // three, never wider than the delegator's own.
    assert_eq!(v.scope(), op_scope());
}

#[test]
fn delegated_grant_cannot_escalate_caps_beyond_delegator() {
    // Delegator holds ONLY CAP_DELEGATE, but the (validly signed) delegated grant
    // requests TELEOP|OBSERVE — a capability escalation.
    let pres = PairingPresentation {
        intermediate: good_intermediate(),
        device_cert: good_device(),
        grant: depth1_grant(), // op_scope() caps = TELEOP|OBSERVE, signed by sk(30)
        delegation: Some(Delegation {
            parent: delegator_parent_grant(Scope::CAP_DELEGATE),
            delegator_cert: delegator_device_cert(),
        }),
    };
    let mut store = fresh_store();
    let err = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap_err();
    assert!(matches!(err, PairingError::ScopeEscalation));
}

#[test]
fn delegated_grant_cannot_escalate_role_beyond_delegator() {
    // Delegator is an OPERATOR; the delegated grant asks for the OWNER role.
    let owner_grant = make_grant(
        acct(A_CLIENT),
        rob(ROBOT),
        Scope {
            role: Role::OWNER,
            caps: Scope::CAP_DELEGATE,
        },
        1,
        acct(A_DELEGATOR),
        pk(&sk(30)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(30));
    let pres = PairingPresentation {
        intermediate: good_intermediate(),
        device_cert: good_device(),
        grant: owner_grant,
        delegation: Some(Delegation {
            parent: delegator_parent_grant(Scope::CAP_DELEGATE),
            delegator_cert: delegator_device_cert(),
        }),
    };
    let mut store = fresh_store();
    let err = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap_err();
    assert!(matches!(err, PairingError::ScopeEscalation));
}

#[test]
fn depth1_delegation_without_cap_delegate_is_rejected() {
    let pres = PairingPresentation {
        intermediate: good_intermediate(),
        device_cert: good_device(),
        grant: depth1_grant(),
        delegation: Some(Delegation {
            parent: delegator_parent_grant(Scope::CAP_TELEOP), // no CAP_DELEGATE
            delegator_cert: delegator_device_cert(),
        }),
    };
    let mut store = fresh_store();
    let err = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap_err();
    assert!(matches!(err, PairingError::InvalidDelegation));
}

#[test]
fn depth1_grant_signed_by_a_non_delegator_key_is_rejected() {
    // delegator_cert binds issuer_key to sk(30), but the grant is signed by sk(31).
    let bad = make_grant(
        acct(A_CLIENT),
        rob(ROBOT),
        op_scope(),
        1,
        acct(A_DELEGATOR),
        pk(&sk(30)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(31)); // wrong signer
    let pres = PairingPresentation {
        intermediate: good_intermediate(),
        device_cert: good_device(),
        grant: bad,
        delegation: Some(Delegation {
            parent: delegator_parent_grant(Scope::CAP_DELEGATE),
            delegator_cert: delegator_device_cert(),
        }),
    };
    let mut store = fresh_store();
    let err = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap_err();
    assert!(matches!(err, PairingError::BadSignature(CertKind::Grant)));
}

// -- intermediate max_scope enforcement --------------------------------------

/// A signed intermediate with a custom `max_scope` (the fixtures otherwise use
/// `OWNER_FULL`). Same key `sk(10)` as every other fixture so it issues the same
/// device certs / grants.
fn intermediate_with_max_scope(max: Scope) -> SignedIntermediateCert {
    IntermediateCert {
        version: FORMAT_VERSION,
        intermediate_key: pk(&sk(10)),
        validity: wide_validity(),
        issued_at_ns: ISSUED,
        max_scope: max,
    }
    .sign_by_roots(&[&sk(1), &sk(2)])
}

/// A conservative scope well inside `OWNER_FULL`: viewer role, observe only.
fn narrow_scope() -> Scope {
    Scope {
        role: Role::VIEWER,
        caps: Scope::CAP_OBSERVE,
    }
}

#[test]
fn device_cert_scope_exceeding_intermediate_max_scope_is_rejected() {
    // A VIEWER/observe-only intermediate signs a device cert claiming OWNER_FULL —
    // privilege the intermediate was never authorized to confer.
    let dev = make_device_cert(
        pk(&sk(20)),
        acct(A_CLIENT),
        pk(&sk(10)),
        Scope::OWNER_FULL,
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let pres = PairingPresentation {
        intermediate: intermediate_with_max_scope(narrow_scope()),
        device_cert: dev,
        grant: good_grant(),
        delegation: None,
    };
    let mut store = fresh_store();
    let err = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap_err();
    assert!(
        matches!(err, PairingError::ScopeExceedsIntermediate),
        "an over-scoped device cert must be rejected, got {err:?}"
    );
}

#[test]
fn grant_scope_exceeding_intermediate_max_scope_is_rejected() {
    // The device cert is WITHIN max (so `verify_device_cert` passes), isolating the
    // rejection to the depth-0 grant, which claims op_scope (OPERATOR + teleop) —
    // wider than the narrow intermediate max_scope.
    let dev = make_device_cert(
        pk(&sk(20)),
        acct(A_CLIENT),
        pk(&sk(10)),
        narrow_scope(),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let grant = make_grant(
        acct(A_CLIENT),
        rob(ROBOT),
        op_scope(),
        0,
        acct(A_ISSUER),
        pk(&sk(10)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let pres = PairingPresentation {
        intermediate: intermediate_with_max_scope(narrow_scope()),
        device_cert: dev,
        grant,
        delegation: None,
    };
    let mut store = fresh_store();
    let err = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap_err();
    assert!(
        matches!(err, PairingError::ScopeExceedsIntermediate),
        "an over-scoped depth-0 grant must be rejected, got {err:?}"
    );
}

#[test]
fn delegated_parent_grant_exceeding_intermediate_max_scope_is_rejected() {
    // The headline delegation case: the device cert AND the depth-1 grant are both
    // WITHIN the narrow max_scope, so the rejection is attributable to the
    // intermediate-issued PARENT grant (OPERATOR role + CAP_DELEGATE — CAP_DELEGATE
    // is absent from the narrow max, so the parent exceeds it). Without gating the
    // parent, a VIEWER-only intermediate could back a delegation carrying more
    // authority than it holds.
    let dev = make_device_cert(
        pk(&sk(20)),
        acct(A_CLIENT),
        pk(&sk(10)),
        narrow_scope(),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    // Depth-1 grant is narrow (viewer/observe) — within max and an attenuation of
    // the wider parent.
    let grant = make_grant(
        acct(A_CLIENT),
        rob(ROBOT),
        narrow_scope(),
        1,
        acct(A_DELEGATOR),
        pk(&sk(30)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(30));
    let pres = PairingPresentation {
        intermediate: intermediate_with_max_scope(narrow_scope()),
        device_cert: dev,
        grant,
        delegation: Some(Delegation {
            // OPERATOR role + CAP_DELEGATE|TELEOP|OBSERVE — exceeds the narrow max.
            parent: delegator_parent_grant(
                Scope::CAP_DELEGATE | Scope::CAP_TELEOP | Scope::CAP_OBSERVE,
            ),
            delegator_cert: delegator_device_cert(),
        }),
    };
    let mut store = fresh_store();
    let err = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap_err();
    assert!(
        matches!(err, PairingError::ScopeExceedsIntermediate),
        "an over-scoped intermediate-issued PARENT grant must be rejected, got {err:?}"
    );
}

#[test]
fn scope_exactly_at_intermediate_max_scope_is_accepted() {
    // Anti-tautology control: with max_scope == op_scope, a device cert + grant
    // BOTH exactly at op_scope verify (is_attenuation_of is reflexive) — the gate
    // rejects only credentials that EXCEED the max, not those that meet it.
    let pres = PairingPresentation {
        intermediate: intermediate_with_max_scope(op_scope()),
        device_cert: good_device(),
        grant: good_grant(),
        delegation: None,
    };
    let mut store = fresh_store();
    let v = store
        .verify_new_pairing(&pres, &peer(), T_NOW)
        .expect("a scope exactly at the intermediate max_scope must verify");
    assert_eq!(v.scope(), op_scope());
}

// -- revocation + rollback ---------------------------------------------------

#[test]
fn epoch_revoked_account_cannot_pair() {
    let mut store = fresh_store();
    let epoch = make_epoch(rob(ROBOT), 1, vec![acct(A_CLIENT)], pk(&sk(10)), ISSUED).sign(&sk(10));
    store
        .apply_epoch(&epoch, &good_intermediate(), T_NOW)
        .unwrap();
    let err = store
        .verify_new_pairing(&good_presentation(), &peer(), T_NOW)
        .unwrap_err();
    assert!(matches!(err, PairingError::RevokedByEpoch { epoch: 1 }));
}

#[test]
fn clock_rollback_behind_high_water_is_rejected() {
    let mut store = fresh_store();
    // First pairing at T_NOW advances the high-water to T_NOW.
    store
        .verify_new_pairing(&good_presentation(), &peer(), T_NOW)
        .unwrap();
    assert_eq!(store.high_water_ns(), T_NOW);
    // A later attempt with an earlier clock is a rollback.
    let err = store
        .verify_new_pairing(&good_presentation(), &peer(), T_NOW - 1)
        .unwrap_err();
    assert!(matches!(
        err,
        PairingError::RollbackDetected {
            now_ns,
            high_water_ns
        } if now_ns == T_NOW - 1 && high_water_ns == T_NOW
    ));
}

#[test]
fn a_rejected_verification_does_not_advance_the_high_water() {
    let mut store = fresh_store();
    let before = store.high_water_ns(); // == T_NOW (fresh_store claims at T_NOW)
                                        // A wrong-peer failure must not move the anti-rollback floor. Use `T_NOW + 1`
                                        // (strictly AHEAD of the floor, and still within skew) so a buggy
                                        // advance-on-rejection would push the floor to `T_NOW + 1 != before` — with
                                        // `T_NOW` the assertion is vacuous (`max(T_NOW, T_NOW) == before` regardless).
    let _ = store.verify_new_pairing(&good_presentation(), &pk(&sk(99)), T_NOW + 1);
    assert_eq!(
        store.high_water_ns(),
        before,
        "a rejected verify must not advance the floor, even for a future-of-floor now_ns"
    );
}

#[test]
fn future_dated_issued_at_is_rejected_and_does_not_poison_the_floor() {
    // A device cert whose issued_at is far in the future (beyond the allowed
    // skew) but INSIDE a valid window must be rejected, and MUST NOT advance the
    // anti-rollback floor (the pairing-DoS class).
    let far_future = T_NOW + 10 * 60 * 1_000_000_000; // 10 min ahead > 5 min skew
    let dev = make_device_cert(
        pk(&sk(20)),
        acct(A_CLIENT),
        pk(&sk(10)),
        op_scope(),
        wide_validity(),
        far_future,
    )
    .sign(&sk(10));
    let pres = PairingPresentation {
        device_cert: dev,
        ..good_presentation()
    };
    let mut store = fresh_store();
    let before = store.high_water_ns();
    let err = store.verify_new_pairing(&pres, &peer(), T_NOW).unwrap_err();
    assert!(matches!(
        err,
        PairingError::IssuedInFuture {
            kind: CertKind::Device,
            ..
        }
    ));
    assert_eq!(
        store.high_water_ns(),
        before,
        "a rejected future-dated cert must not poison the anti-rollback floor"
    );
}

#[test]
fn a_normal_past_issued_at_within_skew_is_accepted() {
    // Anti-tautology control: issued_at slightly ahead of now (within skew) is OK.
    let dev = make_device_cert(
        pk(&sk(20)),
        acct(A_CLIENT),
        pk(&sk(10)),
        op_scope(),
        wide_validity(),
        T_NOW + 60_000_000_000, // 1 min ahead, inside the 5-min skew
    )
    .sign(&sk(10));
    let pres = PairingPresentation {
        device_cert: dev,
        ..good_presentation()
    };
    let mut store = fresh_store();
    assert!(store.verify_new_pairing(&pres, &peer(), T_NOW).is_ok());
}

#[test]
fn root_set_validation_rejects_malformed_sets() {
    // Empty set.
    assert!(matches!(
        RootSet::new(vec![], 1),
        Err(PairingError::InvalidRootSet(_))
    ));
    // Threshold 0.
    assert!(matches!(
        RootSet::new(vec![pk(&sk(1))], 0),
        Err(PairingError::InvalidRootSet(_))
    ));
    // Threshold greater than the number of keys.
    assert!(matches!(
        RootSet::new(vec![pk(&sk(1)), pk(&sk(2))], 3),
        Err(PairingError::InvalidRootSet(_))
    ));
    // Duplicate key.
    assert!(matches!(
        RootSet::new(vec![pk(&sk(1)), pk(&sk(1))], 1),
        Err(PairingError::InvalidRootSet(_))
    ));
    // Valid.
    assert!(RootSet::new(vec![pk(&sk(1)), pk(&sk(2))], 2).is_ok());
}
