// SPDX-License-Identifier: MIT OR Apache-2.0
//! A5: robot-side OFFLINE verification of a desk-carried, owner-signed
//! access grant against the robot's OWN owner chain.
//!
//! Every fixture is a REAL certificate chain built from fixed-seed keys (Principle
//! #13); every assertion is against a HAND oracle or a crafted perturbation, never
//! a self-compare. The positive path establishes the subject; the negative matrix
//! flips exactly one thing per case and pins the precise refusal.

mod common;
use common::*;

use cerulion_pairing::format::*;
use cerulion_pairing::verify::*;
use cerulion_pairing::{CertKind, PairingError};

// -- fixtures ----------------------------------------------------------------

const OWNER: u8 = 0x01; // the robot's claimed owner account
const ROBOT: u8 = 0x0B;
const SUBJECT: u8 = 0x0A; // the granted (guest) account

// Seeds: 10 = intermediate, 20 = subject device key, 30 = owner device key.
fn int_sk() -> ed25519_dalek::SigningKey {
    sk(10)
}
fn subject_sk() -> ed25519_dalek::SigningKey {
    sk(20)
}
fn owner_sk() -> ed25519_dalek::SigningKey {
    sk(30)
}

fn root_set() -> RootSet {
    RootSet::new(vec![pk(&sk(1)), pk(&sk(2)), pk(&sk(3))], 2).unwrap()
}

fn good_intermediate() -> SignedIntermediateCert {
    make_intermediate(pk(&int_sk()), wide_validity(), ISSUED).sign_by_roots(&[&sk(1), &sk(2)])
}

/// A device cert for `account` bound to `device_pk` at `scope`, issued by the
/// intermediate, over `validity`.
fn device_cert(
    account: AccountId,
    device_pk: PublicKey,
    scope: Scope,
    v: Validity,
) -> SignedDeviceCert {
    make_device_cert(device_pk, account, pk(&int_sk()), scope, v, ISSUED).sign(&int_sk())
}

/// An owner-signed access grant for `subject` at `scope`, signed by the owner's
/// device key (`owner_sk`), naming `OWNER` + the owner device key.
fn access_grant(
    subject: AccountId,
    robot: RobotId,
    scope: Scope,
    owner: AccountId,
    owner_device: PublicKey,
    signer: &ed25519_dalek::SigningKey,
    v: Validity,
) -> SignedAccessGrant {
    AccessGrant {
        version: FORMAT_VERSION,
        subject,
        robot,
        scope,
        principal_kind: PrincipalKind::Human,
        validity: v,
        issued_at_ns: ISSUED,
        owner,
        owner_device_key: owner_device,
    }
    .sign(signer)
}

/// An owner-grant presentation whose OWNER cert + access grant are minted with
/// `owner_device_sk` as the owner's SIGNING device (a different owner device than
/// `good_presentation`'s `owner_sk`) — so a test can revoke one owner device and prove
/// another still signs.
fn presentation_with_owner_device(
    owner_device_sk: &ed25519_dalek::SigningKey,
) -> OwnerGrantPresentation {
    OwnerGrantPresentation {
        intermediate: good_intermediate(),
        owner_cert: device_cert(
            acct(OWNER),
            pk(owner_device_sk),
            Scope::OWNER_FULL,
            wide_validity(),
        ),
        subject_cert: device_cert(
            acct(SUBJECT),
            pk(&subject_sk()),
            op_scope(),
            wide_validity(),
        ),
        access_grant: access_grant(
            acct(SUBJECT),
            rob(ROBOT),
            op_scope(),
            acct(OWNER),
            pk(owner_device_sk),
            owner_device_sk,
            wide_validity(),
        ),
    }
}

/// The canonical, all-correct presentation: owner (OWNER_FULL) grants SUBJECT the
/// operator scope on ROBOT.
fn good_presentation() -> OwnerGrantPresentation {
    OwnerGrantPresentation {
        intermediate: good_intermediate(),
        owner_cert: device_cert(
            acct(OWNER),
            pk(&owner_sk()),
            Scope::OWNER_FULL,
            wide_validity(),
        ),
        subject_cert: device_cert(
            acct(SUBJECT),
            pk(&subject_sk()),
            op_scope(),
            wide_validity(),
        ),
        access_grant: access_grant(
            acct(SUBJECT),
            rob(ROBOT),
            op_scope(),
            acct(OWNER),
            pk(&owner_sk()),
            &owner_sk(),
            wide_validity(),
        ),
    }
}

/// A CLAIMED store owned by OWNER (claim at T_NOW seeds the anti-rollback floor).
fn claimed_store() -> TrustStore {
    let mut s = TrustStore::provision(rob(ROBOT), pk(&sk(0x77)), root_set(), CHASSIS, 0).unwrap();
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    s
}

fn subject_peer() -> PublicKey {
    pk(&subject_sk())
}

fn expired_v() -> Validity {
    Validity {
        not_before_ns: 0,
        not_after_ns: 100, // < T_NOW
    }
}

/// A validity window that is VALID at `T_NOW` but expires at `not_after` (which the
/// caller sets to `> T_NOW`), so the grant establishes cleanly yet later rots.
fn bounded_v(not_after: u64) -> Validity {
    Validity {
        not_before_ns: 0,
        not_after_ns: not_after,
    }
}

/// A time-boxed owner grant's expiry: 30 days past `T_NOW` (well within `u64`).
const GRANT_EXPIRY_NS: u64 = T_NOW + 30 * 24 * 3600 * 1_000_000_000;

// -- positive ----------------------------------------------------------------

#[test]
fn a_valid_owner_grant_verifies_and_yields_the_least_privilege_scope() {
    let mut store = claimed_store();
    let verified = store
        .verify_owner_grant(&good_presentation(), &subject_peer(), T_NOW)
        .expect("a correct owner-signed grant verifies");
    // Hand oracle: the SUBJECT account, the SUBJECT device key, and the
    // least-privilege intersection of the subject cert (op_scope) and the grant
    // (op_scope) — which is op_scope itself.
    assert_eq!(verified.account(), acct(SUBJECT));
    assert_eq!(verified.device_key(), pk(&subject_sk()));
    assert_eq!(verified.scope(), op_scope());
    assert_eq!(verified.principal_kind(), PrincipalKind::Human);
}

#[test]
fn effective_scope_is_the_intersection_of_subject_cert_and_grant() {
    // Owner (OWNER_FULL) grants OWNER_FULL, but the subject cert is only op_scope
    // → the effective scope is clamped to op_scope (least privilege), NOT OWNER_FULL.
    let mut pres = good_presentation();
    pres.access_grant = access_grant(
        acct(SUBJECT),
        rob(ROBOT),
        Scope::OWNER_FULL,
        acct(OWNER),
        pk(&owner_sk()),
        &owner_sk(),
        wide_validity(),
    );
    let mut store = claimed_store();
    let v = store
        .verify_owner_grant(&pres, &subject_peer(), T_NOW)
        .unwrap();
    assert_eq!(
        v.scope(),
        op_scope(),
        "the subject cert's scope must clamp the granted scope (least privilege)"
    );
}

#[test]
fn establish_writes_an_owner_signed_row_and_is_allowed_reports_it() {
    let mut store = claimed_store();
    let v = store
        .establish_by_owner_grant(
            &good_presentation(),
            &subject_peer(),
            T_NOW,
            Some("alice".into()),
        )
        .expect("establish");
    assert_eq!(v.account(), acct(SUBJECT));

    // The durable row exists with the OwnerSignedGrant provenance + the effective scope.
    let row = store
        .is_allowed(&acct(SUBJECT), T_NOW)
        .expect("subject is allowed");
    assert_eq!(row.source, PairingSource::OwnerSignedGrant);
    assert_eq!(row.scope, op_scope());
    assert_eq!(row.name.as_deref(), Some("alice"));
    assert!(!row.revoked);

    // The owner's own row is untouched (still OWNER_FULL from the claim).
    let owner_row = store
        .is_allowed(&acct(OWNER), T_NOW)
        .expect("owner still allowed");
    assert_eq!(owner_row.scope, Scope::OWNER_FULL);
}

#[test]
fn a_grant_signed_by_a_revoked_owner_device_is_refused_but_the_owners_other_device_still_signs() {
    // A6 SECURITY: a stolen owner laptop whose DEVICE key is epoch-revoked
    // retains a valid owner cert + owner account, so WITHOUT the owner-signing-device
    // gate it could keep SIGNING grants that verify. Refuse a grant signed by a revoked
    // owner device; the owner's OTHER device is NOT penalized.
    let mut store = claimed_store();
    // Control: the good presentation (owner device = owner_sk) verifies pre-revoke.
    store
        .verify_owner_grant(&good_presentation(), &subject_peer(), T_NOW)
        .expect("verifies before revocation");

    // Revoke ONLY the owner's signing device (pk(owner_sk)) — the owner ACCOUNT stays
    // valid.
    let e = make_epoch_with_devices(
        rob(ROBOT),
        2,
        vec![],
        vec![pk(&owner_sk())],
        pk(&int_sk()),
        ISSUED,
    )
    .sign(&int_sk());
    store
        .apply_epoch(&e, &good_intermediate(), T_NOW)
        .expect("epoch applies");

    // A grant signed by the REVOKED owner device is refused as a device revocation.
    assert!(matches!(
        store.verify_owner_grant(&good_presentation(), &subject_peer(), T_NOW),
        Err(PairingError::RevokedDeviceByEpoch { epoch: 2 })
    ));

    // The owner's OTHER device (a different key, SAME OWNER account) still signs a
    // grant that verifies — the account is not penalized, only the cut device.
    let other_owner_device = sk(31);
    store
        .verify_owner_grant(
            &presentation_with_owner_device(&other_owner_device),
            &subject_peer(),
            T_NOW,
        )
        .expect("the owner's OTHER device still signs valid grants");
}

#[test]
fn a_revoked_subject_device_is_refused_through_the_owner_grant_path_tombstone() {
    // A6: the desk-carried owner-grant (A5) path honors the per-device
    // tombstone. A subject whose DEVICE key is epoch-revoked cannot re-establish via
    // `present-grant`, even with a still-valid owner-signed grant — so a revoked desk
    // is non-resurrectable through the offline verify path.
    let mut store = claimed_store();
    // Control: the grant establishes cleanly before revocation.
    store
        .establish_by_owner_grant(
            &good_presentation(),
            &subject_peer(),
            T_NOW,
            Some("desk".into()),
        )
        .expect("the owner grant establishes before revocation");
    assert!(store.is_allowed(&acct(SUBJECT), T_NOW).is_some());

    // Epoch 2 revokes the subject's DEVICE key (the SUBJECT account is untouched).
    let e = make_epoch_with_devices(
        rob(ROBOT),
        2,
        vec![],
        vec![subject_peer()],
        pk(&int_sk()),
        ISSUED,
    )
    .sign(&int_sk());
    store
        .apply_epoch(&e, &good_intermediate(), T_NOW)
        .expect("epoch applies");

    // Re-presenting the SAME still-valid owner grant is refused as a DEVICE
    // revocation on every attempt (no fresh row written).
    for _ in 0..3 {
        assert!(matches!(
            store.verify_owner_grant(&good_presentation(), &subject_peer(), T_NOW),
            Err(PairingError::RevokedDeviceByEpoch { epoch: 2 })
        ));
    }
}

#[test]
fn verification_is_deterministic_across_two_runs() {
    // Two independent stores verifying the identical presentation yield the identical
    // verified pairing (offline verification is a pure function of inputs).
    let pres = good_presentation();
    let v1 = claimed_store()
        .verify_owner_grant(&pres, &subject_peer(), T_NOW)
        .unwrap();
    let v2 = claimed_store()
        .verify_owner_grant(&pres, &subject_peer(), T_NOW)
        .unwrap();
    assert_eq!(v1, v2);
}

#[test]
fn success_advances_the_anti_rollback_floor_to_now() {
    let mut store = claimed_store();
    let before = store.high_water_ns();
    let later = T_NOW + 5_000_000_000;
    store
        .verify_owner_grant(&good_presentation(), &subject_peer(), later)
        .unwrap();
    assert!(store.high_water_ns() >= later);
    assert!(store.high_water_ns() > before);
}

// -- negative matrix ---------------------------------------------------------

#[test]
fn refuses_when_the_owner_cert_is_not_the_robots_owner() {
    // The owner cert binds a DIFFERENT account (0x0C) than the robot's claimed owner
    // (OWNER) — a grant signed by some other "owner" is refused.
    let stranger = acct(0x0C);
    let stranger_sk = sk(31);
    let mut pres = good_presentation();
    pres.owner_cert = device_cert(
        stranger,
        pk(&stranger_sk),
        Scope::OWNER_FULL,
        wide_validity(),
    );
    pres.access_grant = access_grant(
        acct(SUBJECT),
        rob(ROBOT),
        op_scope(),
        stranger,
        pk(&stranger_sk),
        &stranger_sk,
        wide_validity(),
    );
    let err = claimed_store()
        .verify_owner_grant(&pres, &subject_peer(), T_NOW)
        .unwrap_err();
    assert!(matches!(err, PairingError::NotRobotOwner), "got {err:?}");
}

#[test]
fn refuses_when_the_grant_names_a_different_owner_than_its_cert() {
    // The owner cert correctly binds OWNER, but the grant claims a different owner
    // (0x0C) — the grant's `owner` field must match the presenting owner cert.
    let mut pres = good_presentation();
    pres.access_grant = access_grant(
        acct(SUBJECT),
        rob(ROBOT),
        op_scope(),
        acct(0x0C), // wrong owner in the grant body
        pk(&owner_sk()),
        &owner_sk(),
        wide_validity(),
    );
    let err = claimed_store()
        .verify_owner_grant(&pres, &subject_peer(), T_NOW)
        .unwrap_err();
    assert!(matches!(err, PairingError::NotRobotOwner), "got {err:?}");
}

#[test]
fn refuses_when_the_grant_signing_key_differs_from_the_owner_cert() {
    // The grant claims a different `owner_device_key` (and is signed by it) than the
    // owner cert authenticates — the OWNER_FULL owner cert binds owner_sk, but the
    // grant is signed by a rogue key.
    let rogue = sk(99);
    let mut pres = good_presentation();
    pres.access_grant = access_grant(
        acct(SUBJECT),
        rob(ROBOT),
        op_scope(),
        acct(OWNER),
        pk(&rogue), // grant claims the rogue key
        &rogue,     // ...and is signed by it (so the key check fires before the sig)
        wide_validity(),
    );
    let err = claimed_store()
        .verify_owner_grant(&pres, &subject_peer(), T_NOW)
        .unwrap_err();
    assert!(
        matches!(err, PairingError::OwnerGrantKeyMismatch),
        "got {err:?}"
    );
}

#[test]
fn refuses_when_the_subject_cert_is_not_the_authenticated_peer() {
    // The subject cert is for subject_sk, but the connection authenticated a
    // DIFFERENT peer key — the app-layer binding (device_key == authed peer) fails.
    let other_peer = pk(&sk(21));
    let err = claimed_store()
        .verify_owner_grant(&good_presentation(), &other_peer, T_NOW)
        .unwrap_err();
    assert!(matches!(err, PairingError::PeerKeyMismatch), "got {err:?}");
}

#[test]
fn refuses_a_grant_for_a_different_robot() {
    let mut pres = good_presentation();
    pres.access_grant = access_grant(
        acct(SUBJECT),
        rob(0x0E), // a different robot
        op_scope(),
        acct(OWNER),
        pk(&owner_sk()),
        &owner_sk(),
        wide_validity(),
    );
    let err = claimed_store()
        .verify_owner_grant(&pres, &subject_peer(), T_NOW)
        .unwrap_err();
    assert!(matches!(err, PairingError::WrongRobot), "got {err:?}");
}

#[test]
fn refuses_when_grant_subject_differs_from_the_subject_cert() {
    let mut pres = good_presentation();
    pres.access_grant = access_grant(
        acct(0x0D), // grant names a different subject than the subject cert (SUBJECT)
        rob(ROBOT),
        op_scope(),
        acct(OWNER),
        pk(&owner_sk()),
        &owner_sk(),
        wide_validity(),
    );
    let err = claimed_store()
        .verify_owner_grant(&pres, &subject_peer(), T_NOW)
        .unwrap_err();
    assert!(matches!(err, PairingError::SubjectMismatch), "got {err:?}");
}

#[test]
fn refuses_an_expired_grant() {
    let mut pres = good_presentation();
    pres.access_grant = access_grant(
        acct(SUBJECT),
        rob(ROBOT),
        op_scope(),
        acct(OWNER),
        pk(&owner_sk()),
        &owner_sk(),
        expired_v(),
    );
    let err = claimed_store()
        .verify_owner_grant(&pres, &subject_peer(), T_NOW)
        .unwrap_err();
    assert!(
        matches!(
            err,
            PairingError::Expired {
                kind: CertKind::AccessGrant,
                ..
            }
        ),
        "got {err:?}"
    );
}

#[test]
fn refuses_an_expired_owner_cert() {
    let mut pres = good_presentation();
    pres.owner_cert = device_cert(acct(OWNER), pk(&owner_sk()), Scope::OWNER_FULL, expired_v());
    let err = claimed_store()
        .verify_owner_grant(&pres, &subject_peer(), T_NOW)
        .unwrap_err();
    assert!(
        matches!(
            err,
            PairingError::Expired {
                kind: CertKind::Device,
                ..
            }
        ),
        "got {err:?}"
    );
}

#[test]
fn refuses_an_expired_subject_cert() {
    let mut pres = good_presentation();
    pres.subject_cert = device_cert(acct(SUBJECT), pk(&subject_sk()), op_scope(), expired_v());
    let err = claimed_store()
        .verify_owner_grant(&pres, &subject_peer(), T_NOW)
        .unwrap_err();
    assert!(
        matches!(
            err,
            PairingError::Expired {
                kind: CertKind::Device,
                ..
            }
        ),
        "got {err:?}"
    );
}

#[test]
fn refuses_a_tampered_grant_signature() {
    // Sign a valid grant, then mutate a covered field (scope) WITHOUT re-signing →
    // the signature no longer covers the bytes → BadSignature(AccessGrant).
    let mut pres = good_presentation();
    let mut signed = access_grant(
        acct(SUBJECT),
        rob(ROBOT),
        op_scope(),
        acct(OWNER),
        pk(&owner_sk()),
        &owner_sk(),
        wide_validity(),
    );
    // Tamper: widen the scope after signing (the attenuation check would also catch
    // a widen, but the signature check fires FIRST, so this pins the sig gate).
    signed.grant.scope = Scope {
        role: Role::VIEWER,
        caps: Scope::CAP_OBSERVE,
    };
    pres.access_grant = signed;
    let err = claimed_store()
        .verify_owner_grant(&pres, &subject_peer(), T_NOW)
        .unwrap_err();
    assert!(
        matches!(err, PairingError::BadSignature(CertKind::AccessGrant)),
        "got {err:?}"
    );
}

#[test]
fn refuses_a_grant_that_exceeds_the_owners_own_scope() {
    // The owner cert is only VIEWER/observe, but the grant tries to confer op_scope
    // (teleop) — an owner cannot grant more than it holds.
    let viewer = Scope {
        role: Role::VIEWER,
        caps: Scope::CAP_OBSERVE,
    };
    let mut pres = good_presentation();
    pres.owner_cert = device_cert(acct(OWNER), pk(&owner_sk()), viewer, wide_validity());
    // grant scope (op_scope) is NOT an attenuation of the owner cert's viewer scope.
    let err = claimed_store()
        .verify_owner_grant(&pres, &subject_peer(), T_NOW)
        .unwrap_err();
    assert!(
        matches!(err, PairingError::OwnerGrantScopeExceedsOwner),
        "got {err:?}"
    );
}

#[test]
fn refuses_a_grant_naming_the_owner_itself_as_subject() {
    // A grant for the OWNER account: build a subject cert binding subject_sk → OWNER,
    // the grant subject = OWNER. The owner already has full access; a self-grant could
    // only downgrade the owner row, so it is refused.
    let pres = OwnerGrantPresentation {
        intermediate: good_intermediate(),
        owner_cert: device_cert(
            acct(OWNER),
            pk(&owner_sk()),
            Scope::OWNER_FULL,
            wide_validity(),
        ),
        subject_cert: device_cert(acct(OWNER), pk(&subject_sk()), op_scope(), wide_validity()),
        access_grant: access_grant(
            acct(OWNER),
            rob(ROBOT),
            op_scope(),
            acct(OWNER),
            pk(&owner_sk()),
            &owner_sk(),
            wide_validity(),
        ),
    };
    let err = claimed_store()
        .verify_owner_grant(&pres, &subject_peer(), T_NOW)
        .unwrap_err();
    assert!(
        matches!(err, PairingError::OwnerGrantForOwner),
        "got {err:?}"
    );
}

#[test]
fn refuses_on_an_unclaimed_robot() {
    // No owner to verify against → RobotUnclaimed (before any chain work).
    let mut store =
        TrustStore::provision(rob(ROBOT), pk(&sk(0x77)), root_set(), CHASSIS, 0).unwrap();
    let err = store
        .verify_owner_grant(&good_presentation(), &subject_peer(), T_NOW)
        .unwrap_err();
    assert!(matches!(err, PairingError::RobotUnclaimed), "got {err:?}");
}

#[test]
fn refuses_an_intermediate_below_the_root_threshold() {
    // Sign the intermediate by only ONE root when the set needs 2-of-3.
    let mut pres = good_presentation();
    pres.intermediate =
        make_intermediate(pk(&int_sk()), wide_validity(), ISSUED).sign_by_roots(&[&sk(1)]);
    let err = claimed_store()
        .verify_owner_grant(&pres, &subject_peer(), T_NOW)
        .unwrap_err();
    assert!(
        matches!(err, PairingError::RootThresholdNotMet { need: 2, .. }),
        "got {err:?}"
    );
}

#[test]
fn refuses_a_clock_rollback() {
    // The claim seeded the floor to T_NOW; verifying at an earlier time is a rollback.
    let err = claimed_store()
        .verify_owner_grant(&good_presentation(), &subject_peer(), T_NOW - 1)
        .unwrap_err();
    assert!(
        matches!(err, PairingError::RollbackDetected { .. }),
        "got {err:?}"
    );
}

#[test]
fn refuses_a_subject_revoked_by_the_owner() {
    // Establish the subject, then owner-revoke it; re-verifying its owner grant is
    // refused with the sticky owner-revocation signal (not silently re-admitted).
    let mut store = claimed_store();
    store
        .establish_by_owner_grant(&good_presentation(), &subject_peer(), T_NOW, None)
        .unwrap();
    store.owner_revoke(&acct(OWNER), &acct(SUBJECT)).unwrap();
    let err = store
        .verify_owner_grant(&good_presentation(), &subject_peer(), T_NOW)
        .unwrap_err();
    assert!(matches!(err, PairingError::RevokedByOwner), "got {err:?}");
}

#[test]
fn refuses_a_subject_revoked_by_epoch() {
    let mut store = claimed_store();
    // Push an epoch revoking the subject.
    let epoch =
        make_epoch(rob(ROBOT), 1, vec![acct(SUBJECT)], pk(&int_sk()), ISSUED).sign(&int_sk());
    store
        .apply_epoch(&epoch, &good_intermediate(), T_NOW)
        .unwrap();
    let err = store
        .verify_owner_grant(&good_presentation(), &subject_peer(), T_NOW)
        .unwrap_err();
    assert!(
        matches!(err, PairingError::RevokedByEpoch { .. }),
        "got {err:?}"
    );
}

#[test]
fn refuses_when_the_owner_itself_is_revoked() {
    // If the OWNER (the grant's issuer) is epoch-revoked, its grants must not admit
    // new guests — DelegatorRevoked (the owner is the delegator here).
    let mut store = claimed_store();
    let epoch = make_epoch(rob(ROBOT), 1, vec![acct(OWNER)], pk(&int_sk()), ISSUED).sign(&int_sk());
    store
        .apply_epoch(&epoch, &good_intermediate(), T_NOW)
        .unwrap();
    let err = store
        .verify_owner_grant(&good_presentation(), &subject_peer(), T_NOW)
        .unwrap_err();
    assert!(matches!(err, PairingError::DelegatorRevoked), "got {err:?}");
}

#[test]
fn a_failed_verification_never_advances_the_floor_or_writes_a_row() {
    // Anti-tautology on the negative side: a rejected grant leaves the store pristine.
    let mut store = claimed_store();
    let before = store.high_water_ns();
    let mut pres = good_presentation();
    pres.access_grant = access_grant(
        acct(SUBJECT),
        rob(0x0E), // wrong robot → WrongRobot
        op_scope(),
        acct(OWNER),
        pk(&owner_sk()),
        &owner_sk(),
        wide_validity(),
    );
    // Verify at a LATER time so a (buggy) floor-advance would be observable.
    let later = T_NOW + 9_000_000_000;
    assert!(store
        .verify_owner_grant(&pres, &subject_peer(), later)
        .is_err());
    assert_eq!(
        store.high_water_ns(),
        before,
        "a rejected grant must not move the floor"
    );
    assert!(
        store.is_allowed(&acct(SUBJECT), T_NOW).is_none(),
        "no row written on refusal"
    );
}

// -- owner-grant expiry is ENFORCED after establishment --------------------------

#[test]
fn a_time_boxed_owner_grant_row_is_denied_once_it_expires() {
    // Decision: desk-carried signed grants exist specifically for offline,
    // time-boxed access (e.g. a 30-day contractor). Presentation-time expiry alone
    // is not enough: the DURABLE row must also stop admitting once the clock passes
    // the grant's bound — without an explicit revoke. This pins that end-to-end at
    // the `is_allowed` access check the accept gate reads.
    let mut store = claimed_store();
    let mut pres = good_presentation();
    // A grant VALID at T_NOW but expiring at GRANT_EXPIRY_NS (30 days later).
    pres.access_grant = access_grant(
        acct(SUBJECT),
        rob(ROBOT),
        op_scope(),
        acct(OWNER),
        pk(&owner_sk()),
        &owner_sk(),
        bounded_v(GRANT_EXPIRY_NS),
    );
    store
        .establish_by_owner_grant(&pres, &subject_peer(), T_NOW, Some("contractor".into()))
        .expect("a grant valid at establishment time establishes");

    // The durable row carries the grant's finite bound (hand oracle).
    let row = store
        .is_allowed(&acct(SUBJECT), T_NOW)
        .expect("allowed at establishment time");
    assert_eq!(
        row.expires_at_ns,
        Some(GRANT_EXPIRY_NS),
        "the row must carry the grant's not_after_ns"
    );

    // Admitted strictly BEFORE the bound (T_NOW and one ns before expiry).
    assert!(store.is_allowed(&acct(SUBJECT), T_NOW).is_some());
    assert!(
        store
            .is_allowed(&acct(SUBJECT), GRANT_EXPIRY_NS - 1)
            .is_some(),
        "allowed up to the instant before expiry"
    );
    // DENIED at the bound and beyond (`now_ns >= not_after_ns`), with NO revoke.
    assert!(
        store.is_allowed(&acct(SUBJECT), GRANT_EXPIRY_NS).is_none(),
        "denied exactly at the expiry instant (half-open window)"
    );
    assert!(
        store
            .is_allowed(&acct(SUBJECT), GRANT_EXPIRY_NS + 1_000_000_000)
            .is_none(),
        "still denied well past expiry"
    );

    // CONTROL: the OWNER's own row (from the claim) carries no expiry, so it admits
    // indefinitely — proving expiry is scoped to the owner-signed grant, not global.
    assert!(
        store
            .is_allowed(&acct(OWNER), GRANT_EXPIRY_NS + 1_000_000_000)
            .is_some(),
        "an established (None-expiry) row never rots"
    );
}

#[test]
fn an_unbounded_owner_grant_row_never_expires() {
    // Anti-tautology for the expiry pin: an UNBOUNDED grant (`not_after_ns ==
    // u64::MAX`, the AccessGrant "no expiry" encoding) maps to a `None` row expiry,
    // so it admits at any far-future time exactly like an established pairing.
    let mut store = claimed_store();
    let mut pres = good_presentation();
    pres.access_grant = access_grant(
        acct(SUBJECT),
        rob(ROBOT),
        op_scope(),
        acct(OWNER),
        pk(&owner_sk()),
        &owner_sk(),
        bounded_v(u64::MAX),
    );
    store
        .establish_by_owner_grant(&pres, &subject_peer(), T_NOW, None)
        .expect("an unbounded grant establishes");

    let row = store
        .is_allowed(&acct(SUBJECT), T_NOW)
        .expect("allowed now");
    assert_eq!(
        row.expires_at_ns, None,
        "an unbounded grant carries no row expiry"
    );
    assert!(
        store
            .is_allowed(&acct(SUBJECT), GRANT_EXPIRY_NS + 1_000_000_000)
            .is_some(),
        "an unbounded owner grant never rots"
    );
}

// -- a LATE-gate failure writes no row (floor-order guard) ------------------------

#[test]
fn a_delegator_revoked_late_gate_writes_no_row_and_does_not_advance_the_floor() {
    // DelegatorRevoked fires AFTER the full crypto chain verifies (owner cert +
    // subject cert + grant sig + attenuation) but BEFORE the floor advance. A
    // reorder that advanced the floor — or wrote the row — before this gate would be
    // caught here (the pre-existing no-write test only exercised an EARLY WrongRobot
    // failure).
    let mut store = claimed_store();
    // Epoch-revoke the OWNER (the grant's issuer / "delegator").
    let epoch = make_epoch(rob(ROBOT), 1, vec![acct(OWNER)], pk(&int_sk()), ISSUED).sign(&int_sk());
    store
        .apply_epoch(&epoch, &good_intermediate(), T_NOW)
        .unwrap();
    let before = store.high_water_ns();

    // Establish (verify-then-write) at a LATER time so a buggy floor-advance shows.
    let later = T_NOW + 9_000_000_000;
    let err = store
        .establish_by_owner_grant(
            &good_presentation(),
            &subject_peer(),
            later,
            Some("g".into()),
        )
        .unwrap_err();
    assert!(matches!(err, PairingError::DelegatorRevoked), "got {err:?}");
    assert_eq!(
        store.high_water_ns(),
        before,
        "a late-gate refusal must not advance the anti-rollback floor"
    );
    assert!(
        store.is_allowed(&acct(SUBJECT), T_NOW).is_none(),
        "a late-gate refusal must write NO subject row"
    );
}

#[test]
fn an_owner_grant_for_owner_late_gate_writes_no_row_and_leaves_the_owner_row_intact() {
    // OwnerGrantForOwner is the LAST gate before the floor advance. A reorder that
    // wrote the row first would clobber the owner's own claim row (downgrading it to
    // an OwnerSignedGrant row). Pin: the owner row stays the untouched Claim/OWNER_FULL
    // and the floor does not move.
    let mut store = claimed_store();
    let before = store.high_water_ns();
    // A grant naming the OWNER as its own subject (subject cert binds subject_sk → OWNER).
    let pres = OwnerGrantPresentation {
        intermediate: good_intermediate(),
        owner_cert: device_cert(
            acct(OWNER),
            pk(&owner_sk()),
            Scope::OWNER_FULL,
            wide_validity(),
        ),
        subject_cert: device_cert(acct(OWNER), pk(&subject_sk()), op_scope(), wide_validity()),
        access_grant: access_grant(
            acct(OWNER),
            rob(ROBOT),
            op_scope(),
            acct(OWNER),
            pk(&owner_sk()),
            &owner_sk(),
            wide_validity(),
        ),
    };
    let later = T_NOW + 9_000_000_000;
    let err = store
        .establish_by_owner_grant(&pres, &subject_peer(), later, Some("self".into()))
        .unwrap_err();
    assert!(
        matches!(err, PairingError::OwnerGrantForOwner),
        "got {err:?}"
    );
    assert_eq!(
        store.high_water_ns(),
        before,
        "a late-gate refusal must not advance the anti-rollback floor"
    );
    // The owner's own row is UNTOUCHED — no OwnerSignedGrant write clobbered it.
    let owner_row = store
        .is_allowed(&acct(OWNER), T_NOW)
        .expect("owner still allowed");
    assert_eq!(owner_row.source, PairingSource::Claim);
    assert_eq!(owner_row.scope, Scope::OWNER_FULL);
}
