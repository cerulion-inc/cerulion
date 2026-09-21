// SPDX-License-Identifier: AGPL-3.0-only
//! A5: the robot-side `present-grant` ops verb + the `is_allowed`-backed
//! `DemandAuthorizer` implementation, end-to-end over the LIVE `SharedTrust`.
//!
//! A desk carries a grant its OWNER signed and presents it; the robot verifies it
//! OFFLINE against its own owner and admits the subject — no cloud contact. Every
//! fixture is a REAL certificate chain (fixed-seed keys — Principle #13); every
//! assertion is against a hand oracle or a crafted refusal, never a self-compare.

use cerud::transport::CallerIdentity;
use cerud::verbs::VerbHandler;

use cerulion_core::transport::demand_authorizer::{
    DemandAuthorizer, DemandDecision, DemandSubject,
};
use cerulion_link::alpn;
use cerulion_pairing::format::{
    AccessGrant, AccountId, DeviceCert, IntermediateCert, PrincipalKind, PublicKey, RobotId, Role,
    RootSet, Scope, SignedAccessGrant, SignedDeviceCert, SignedIntermediateCert, Validity,
    FORMAT_VERSION,
};
use cerulion_pairing::verify::TrustStore;
use ed25519_dalek::SigningKey;

use cerulion_remoted::pairing_verbs::{OwnerGrantWire, PresentGrantVerb};
use cerulion_remoted::{
    AcceptDecision, DeviceAccountIndex, KeyAccess, PairingAuthorizer, SharedTrust,
};

// ── deterministic fixtures ──────────────────────────────────────────────────

const T_NOW: u64 = 1_000_000_000_000;
const ISSUED: u64 = 500_000_000_000;
const CHASSIS: &[u8] = b"a5-present-grant-random-chassis-secret-not-serial-derived";

const OWNER: AccountId = AccountId([0x01; 32]);
const SUBJECT: AccountId = AccountId([0x0A; 32]);
const ROBOT: RobotId = RobotId([0x0B; 32]);

fn sk(seed: u8) -> SigningKey {
    SigningKey::from_bytes(&[seed; 32])
}
fn pk(k: &SigningKey) -> PublicKey {
    PublicKey(k.verifying_key().to_bytes())
}
fn wide() -> Validity {
    Validity {
        not_before_ns: 0,
        not_after_ns: 100_000_000_000_000,
    }
}
fn op_scope() -> Scope {
    Scope {
        role: Role::OPERATOR,
        caps: Scope::CAP_TELEOP | Scope::CAP_OBSERVE,
    }
}
fn caller_for(key: &PublicKey) -> CallerIdentity {
    CallerIdentity::verified(hex::encode(key.0))
}

// Seeds: 1 = root, 10 = intermediate, 20 = subject device key, 30 = owner device key.
fn intermediate() -> SignedIntermediateCert {
    IntermediateCert {
        version: FORMAT_VERSION,
        intermediate_key: pk(&sk(10)),
        validity: wide(),
        issued_at_ns: ISSUED,
        max_scope: Scope::OWNER_FULL,
    }
    .sign_by_roots(&[&sk(1)])
}

fn device_cert(account: AccountId, device: &SigningKey, scope: Scope) -> SignedDeviceCert {
    DeviceCert {
        version: FORMAT_VERSION,
        device_key: pk(device),
        account,
        principal_kind: PrincipalKind::Human,
        scope,
        validity: wide(),
        issued_at_ns: ISSUED,
        issuer_key: pk(&sk(10)),
    }
    .sign(&sk(10))
}

fn access_grant(
    subject: AccountId,
    robot: RobotId,
    scope: Scope,
    owner: AccountId,
    owner_device: &SigningKey,
) -> SignedAccessGrant {
    AccessGrant {
        version: FORMAT_VERSION,
        subject,
        robot,
        scope,
        principal_kind: PrincipalKind::Human,
        validity: wide(),
        issued_at_ns: ISSUED,
        owner,
        owner_device_key: pk(owner_device),
    }
    .sign(owner_device)
}

/// The canonical presentation the desk carries: OWNER (via sk(30)) grants SUBJECT
/// (via sk(20)) the operator scope on ROBOT.
fn good_wire() -> OwnerGrantWire {
    OwnerGrantWire {
        intermediate: intermediate(),
        owner_cert: device_cert(OWNER, &sk(30), Scope::OWNER_FULL),
        subject_cert: device_cert(SUBJECT, &sk(20), op_scope()),
        access_grant: access_grant(SUBJECT, ROBOT, op_scope(), OWNER, &sk(30)),
    }
}

/// A CLAIMED store owned by OWNER (the robot's own transport key is sk(0x77)),
/// wrapped in a read-only in-memory `SharedTrust` (empty MAC — mutations apply in
/// memory only, no disk, sufficient for the verb/authorizer logic).
fn claimed_shared() -> SharedTrust {
    let root_set = RootSet::new(vec![pk(&sk(1))], 1).unwrap();
    let mut store = TrustStore::provision(ROBOT, pk(&sk(0x77)), root_set, CHASSIS, 0).unwrap();
    store
        .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    // The accept-gate access read is evaluated at the robot's clock. Pin it to the
    // test's T_NOW universe (the same time `present()` uses) so an owner grant whose
    // finite validity lives in that universe reads VALID (a wall clock would read the
    // ~1970-era `wide()` bound as already expired).
    SharedTrust::new_with_clock(
        store,
        DeviceAccountIndex::new(),
        Vec::new(),
        cerulion_remoted::RemotedClock::fixed(T_NOW),
    )
}

fn present(
    shared: &SharedTrust,
    wire: &OwnerGrantWire,
    caller: &CallerIdentity,
) -> cerud::error::CerudResult<serde_json::Value> {
    let verb = PresentGrantVerb::new(shared.clone(), cerulion_remoted::RemotedClock::fixed(T_NOW));
    let blob = hex::encode(wire.to_postcard().unwrap());
    verb.execute_with_caller(caller, &serde_json::json!({ "grant_postcard": blob }))
}

// ── present-grant verb ───────────────────────────────────────────────────────

#[test]
fn present_grant_establishes_the_subject_and_admits_it_afterwards() {
    let shared = claimed_shared();
    let subject_key = pk(&sk(20));

    // Before: the subject is unpaired.
    assert_eq!(
        shared.snapshot_for_key(&subject_key.0).1,
        KeyAccess::Unpaired
    );

    // Present the owner-signed grant as the SUBJECT (its device key = the caller).
    let out = present(&shared, &good_wire(), &caller_for(&subject_key)).expect("present-grant");
    assert_eq!(out["paired"], serde_json::json!(true));
    assert_eq!(out["source"], serde_json::json!("owner_signed_grant"));
    assert_eq!(out["account"], serde_json::json!(hex::encode(SUBJECT.0)));

    // After: the subject is Allowed at the effective scope (op_scope — subject cert
    // ∩ grant). The accept gate + the authorizer now admit it.
    match shared.snapshot_for_key(&subject_key.0) {
        (true, KeyAccess::Allowed(scope)) => assert_eq!(scope, op_scope()),
        other => panic!("expected Allowed(op_scope), got {other:?}"),
    }

    // The wire plane admits the now-paired subject (CAP_OBSERVE present).
    let authz = PairingAuthorizer::from_shared(shared.clone());
    assert_eq!(
        authz.classify_accept(alpn::WIRE, &subject_key.0),
        AcceptDecision::WireAdmit
    );
}

#[test]
fn present_grant_refuses_a_grant_not_signed_by_the_robot_owner() {
    // A grant signed by a DIFFERENT "owner" (sk(31), account 0x0C) — the robot's
    // claimed owner is OWNER (sk(30)), so it is refused and NO row is written.
    let shared = claimed_shared();
    let subject_key = pk(&sk(20));
    let stranger = AccountId([0x0C; 32]);
    let wire = OwnerGrantWire {
        intermediate: intermediate(),
        owner_cert: device_cert(stranger, &sk(31), Scope::OWNER_FULL),
        subject_cert: device_cert(SUBJECT, &sk(20), op_scope()),
        access_grant: access_grant(SUBJECT, ROBOT, op_scope(), stranger, &sk(31)),
    };
    let err = present(&shared, &wire, &caller_for(&subject_key)).unwrap_err();
    assert!(
        err.to_string().contains("not signed by this robot's owner"),
        "{err}"
    );
    // No row: the subject stays unpaired (fail-closed).
    assert_eq!(
        shared.snapshot_for_key(&subject_key.0).1,
        KeyAccess::Unpaired
    );
}

#[test]
fn present_grant_refuses_when_the_caller_is_not_the_grant_subject() {
    // The grant is for SUBJECT (sk(20)), but a DIFFERENT authed peer (sk(21))
    // presents it — the subject cert's device_key must equal the authenticated peer.
    let shared = claimed_shared();
    let wrong_caller = pk(&sk(21));
    let err = present(&shared, &good_wire(), &caller_for(&wrong_caller)).unwrap_err();
    assert!(
        err.to_string()
            .contains("does not match the authenticated peer"),
        "{err}"
    );
    // Neither key is paired.
    assert_eq!(
        shared.snapshot_for_key(&wrong_caller.0).1,
        KeyAccess::Unpaired
    );
    assert_eq!(
        shared.snapshot_for_key(&pk(&sk(20)).0).1,
        KeyAccess::Unpaired
    );
}

#[test]
fn present_grant_refuses_bad_hex_and_non_postcard_loudly() {
    let shared = claimed_shared();
    let verb = PresentGrantVerb::new(shared, cerulion_remoted::RemotedClock::fixed(T_NOW));
    let caller = caller_for(&pk(&sk(20)));
    // Bad hex.
    let e1 = verb
        .execute_with_caller(
            &caller,
            &serde_json::json!({ "grant_postcard": "zz!!not-hex" }),
        )
        .unwrap_err();
    assert!(e1.to_string().contains("not valid hex"), "{e1}");
    // Valid hex, not a postcard OwnerGrantWire.
    let e2 = verb
        .execute_with_caller(
            &caller,
            &serde_json::json!({ "grant_postcard": hex::encode([0xff; 8]) }),
        )
        .unwrap_err();
    assert!(e2.to_string().contains("did not decode"), "{e2}");
    // Missing the field entirely.
    let e3 = verb
        .execute_with_caller(&caller, &serde_json::json!({}))
        .unwrap_err();
    assert!(e3.to_string().contains("grant_postcard"), "{e3}");
}

#[test]
fn present_grant_is_reachable_by_an_unpaired_authed_key_but_only_on_a_claimed_robot() {
    // Bootstrap classification: on a CLAIMED robot, an UNPAIRED authed key reaches
    // present-grant (Allow) — that is exactly how a not-yet-paired desk presents its
    // owner-signed grant to get onto the ACL.
    let shared = claimed_shared();
    let authz = PairingAuthorizer::from_shared(shared);
    let unpaired = caller_for(&pk(&sk(20)));
    assert!(
        matches!(
            authz.authorize_verb(&unpaired, cerulion_remoted::verbs::PRESENT_GRANT),
            cerud::authz::AuthzDecision::Allow
        ),
        "present-grant must be admissible for an unpaired authed key on a claimed robot"
    );

    // On an UNCLAIMED robot, ONLY `claim` is admissible — present-grant is refused.
    let root_set = RootSet::new(vec![pk(&sk(1))], 1).unwrap();
    let store = TrustStore::provision(ROBOT, pk(&sk(0x77)), root_set, CHASSIS, 0).unwrap();
    let unclaimed = SharedTrust::new(store, DeviceAccountIndex::new(), Vec::new());
    let authz2 = PairingAuthorizer::from_shared(unclaimed);
    match authz2.authorize_verb(&unpaired, cerulion_remoted::verbs::PRESENT_GRANT) {
        cerud::authz::AuthzDecision::Deny { reason } => {
            assert!(reason.contains("UNCLAIMED"), "{reason}")
        }
        other => panic!("present-grant must be refused on an unclaimed robot, got {other:?}"),
    }
}

// ── the DemandAuthorizer (is_allowed) implementation ─────────────────────────

#[test]
fn demand_authorizer_admits_a_wan_subject_that_is_allowed() {
    let shared = claimed_shared();
    // Establish the subject via present-grant, then the WAN demand for its device key
    // is admitted (the SAME is_allowed the accept gate reads).
    present(&shared, &good_wire(), &caller_for(&pk(&sk(20)))).unwrap();
    let authz = PairingAuthorizer::from_shared(shared);

    let subject = DemandSubject::Wan {
        demander_key: pk(&sk(20)).0,
    };
    assert_eq!(
        authz.authorize_demand(&subject, "/tf"),
        DemandDecision::Allow
    );
    // Anti-tautology: a topic name never changes the verdict for an allowed subject
    // (the gate is account-scoped, not per-topic on the WAN plane).
    assert!(authz.authorize_demand(&subject, "/some/other").is_allowed());
}

#[test]
fn demand_authorizer_denies_an_unpaired_wan_subject_loudly() {
    let shared = claimed_shared();
    let authz = PairingAuthorizer::from_shared(shared);
    // sk(20) has NOT presented a grant → unpaired → Deny with a loud, topic-naming reason.
    let subject = DemandSubject::Wan {
        demander_key: pk(&sk(20)).0,
    };
    match authz.authorize_demand(&subject, "/secret") {
        DemandDecision::Deny { reason } => {
            assert!(reason.contains("iroh-wan"), "{reason}");
            assert!(reason.contains("/secret"), "{reason}");
            assert!(reason.contains("unpaired"), "{reason}");
        }
        other => panic!("expected Deny for an unpaired WAN subject, got {other:?}"),
    }
}

#[test]
fn demand_authorizer_denies_a_revoked_wan_subject() {
    // Build a store where the subject was established via its owner grant and then
    // owner-revoked; bind its device key in the side-map (as a real pairing would).
    // The revoked account is a distinct deny signal from an unpaired key.
    let root_set = RootSet::new(vec![pk(&sk(1))], 1).unwrap();
    let mut store = TrustStore::provision(ROBOT, pk(&sk(0x77)), root_set, CHASSIS, 0).unwrap();
    store
        .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    let subject_key = pk(&sk(20));
    store
        .establish_by_owner_grant(
            &good_wire().into_presentation(),
            &subject_key,
            T_NOW,
            Some("g".into()),
        )
        .expect("establish subject via owner grant");
    // The owner revokes the subject → it leaves the allow list (is_allowed → None).
    store.owner_revoke(&OWNER, &SUBJECT).unwrap();
    assert!(
        store.is_allowed(&SUBJECT, T_NOW).is_none(),
        "revoked subject must be denied by is_allowed"
    );

    // Bind the device key in the side-map (SharedTrust::commit does this on a real
    // pairing; here we build the bound state directly to isolate the revoked path).
    let mut index = DeviceAccountIndex::new();
    index.bind(subject_key, SUBJECT);
    // Pin the access-read clock to T_NOW so this test isolates the REVOKED path (a
    // wall clock would ALSO read the ~1970 `wide()` grant as expired, masking it).
    let shared = SharedTrust::new_with_clock(
        store,
        index,
        Vec::new(),
        cerulion_remoted::RemotedClock::fixed(T_NOW),
    );

    // Sanity: the key is BOUND but NotAllowed (distinct from Unpaired).
    assert_eq!(
        shared.snapshot_for_key(&subject_key.0).1,
        KeyAccess::NotAllowed
    );

    let authz = PairingAuthorizer::from_shared(shared);
    match authz.authorize_demand(
        &DemandSubject::Wan {
            demander_key: subject_key.0,
        },
        "/tf",
    ) {
        DemandDecision::Deny { reason } => {
            assert!(reason.contains("iroh-wan"), "{reason}");
            assert!(reason.contains("not on the access list"), "{reason}");
        }
        other => panic!("expected Deny for a revoked WAN subject, got {other:?}"),
    }
}

#[test]
fn demand_authorizer_allows_all_lan_subjects_a5b() {
    // A5b: the LAN plane carries no authenticated identity yet → allow-all
    // (byte-identical to the pre-A5 AllowAllAuthorizer on that plane).
    let shared = claimed_shared();
    let authz = PairingAuthorizer::from_shared(shared);
    assert_eq!(
        authz.authorize_demand(&DemandSubject::Lan { locator: None }, "/anything"),
        DemandDecision::Allow
    );
    assert!(authz
        .authorize_demand(
            &DemandSubject::Lan {
                locator: Some("tcp/1.2.3.4:7683".into())
            },
            "/tf"
        )
        .is_allowed());
}

// ── An EXPIRED owner grant is denied AT THE ACCEPT GATE ──

/// The canonical presentation, but with the access grant's validity upper bound set
/// to `not_after` (valid at `T_NOW`, expiring later) so a robot whose clock later
/// passes `not_after` denies the subject.
fn wire_expiring_at(not_after: u64) -> OwnerGrantWire {
    OwnerGrantWire {
        intermediate: intermediate(),
        owner_cert: device_cert(OWNER, &sk(30), Scope::OWNER_FULL),
        subject_cert: device_cert(SUBJECT, &sk(20), op_scope()),
        access_grant: AccessGrant {
            version: FORMAT_VERSION,
            subject: SUBJECT,
            robot: ROBOT,
            scope: op_scope(),
            principal_kind: PrincipalKind::Human,
            validity: Validity {
                not_before_ns: 0,
                not_after_ns: not_after,
            },
            issued_at_ns: ISSUED,
            owner: OWNER,
            owner_device_key: pk(&sk(30)),
        }
        .sign(&sk(30)),
    }
}

#[test]
fn an_expired_owner_grant_is_denied_at_the_accept_gate() {
    // The headline capability: a time-boxed owner-signed grant stops admitting once
    // the robot's own clock passes its bound — offline, with NO explicit revoke.
    // Established at T_NOW, valid until `EXPIRY` (both inside the wide() universe so
    // presentation-time verification passes at T_NOW).
    const EXPIRY: u64 = 2_000_000_000_000; // > T_NOW (1e12), < wide() (1e14)

    // Build a claimed SharedTrust whose access-read clock we RETAIN so we can drive
    // it past the grant's expiry (production reads a wall clock; this pins the seam).
    let root_set = RootSet::new(vec![pk(&sk(1))], 1).unwrap();
    let mut store = TrustStore::provision(ROBOT, pk(&sk(0x77)), root_set, CHASSIS, 0).unwrap();
    store
        .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    let clock = cerulion_remoted::RemotedClock::fixed(T_NOW);
    let shared =
        SharedTrust::new_with_clock(store, DeviceAccountIndex::new(), Vec::new(), clock.clone());

    let subject_key = pk(&sk(20));
    present(
        &shared,
        &wire_expiring_at(EXPIRY),
        &caller_for(&subject_key),
    )
    .expect("present-grant");
    let authz = PairingAuthorizer::from_shared(shared.clone());
    let wan = DemandSubject::Wan {
        demander_key: subject_key.0,
    };

    // At T_NOW (and right up to the instant before expiry) the subject is ADMITTED
    // across all three read surfaces (accept gate, wire classification, WAN demand).
    assert!(matches!(
        shared.snapshot_for_key(&subject_key.0),
        (true, KeyAccess::Allowed(_))
    ));
    assert_eq!(
        authz.classify_accept(alpn::WIRE, &subject_key.0),
        AcceptDecision::WireAdmit
    );
    assert!(authz.authorize_demand(&wan, "/tf").is_allowed());
    clock.set(EXPIRY - 1);
    assert!(
        matches!(
            shared.snapshot_for_key(&subject_key.0),
            (true, KeyAccess::Allowed(_))
        ),
        "still admitted the instant before expiry"
    );

    // Advance the robot's clock TO the grant's bound: the SAME durable row now DENIES
    // everywhere — no revoke, no epoch, no owner action.
    clock.set(EXPIRY);
    assert_eq!(
        shared.snapshot_for_key(&subject_key.0).1,
        KeyAccess::NotAllowed,
        "the accept-gate read denies an expired grant"
    );
    match authz.classify_accept(alpn::WIRE, &subject_key.0) {
        AcceptDecision::Refuse { reason } => assert!(reason.contains("expired"), "{reason}"),
        other => panic!("expected the wire plane to Refuse an expired grant, got {other:?}"),
    }
    match authz.authorize_demand(&wan, "/tf") {
        DemandDecision::Deny { reason } => assert!(reason.contains("expired"), "{reason}"),
        other => panic!("expected the WAN demand to Deny an expired grant, got {other:?}"),
    }
}
