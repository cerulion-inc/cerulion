// SPDX-License-Identifier: MIT OR Apache-2.0
//! Client-module tests (device-identity seam, cert carry) plus the two
//! end-to-end pairing flows composed across all four modules.

mod common;
use common::*;

use cerulion_pairing::client::*;
use cerulion_pairing::format::*;
use cerulion_pairing::pake::*;
use cerulion_pairing::verify::*;
use cerulion_pairing::PairingError;

const ROBOT: u8 = 0xB0;
const ROBOT_2: u8 = 0xB2;
const CLIENT_ACCT: u8 = 0x0A;
const OWNER: u8 = 0x01; // the robot owner (distinct from the pairing subject)
/// The seed for the robot's OWN transport key that `claimed_store` provisions
/// with. The code-pairing e2e below runs its CPace ceremony against the SAME key
/// (`pk(&sk(ROBOT_TRANSPORT_SEED))`), so the witness binds this robot.
const ROBOT_TRANSPORT_SEED: u8 = 200;

fn root_set() -> RootSet {
    RootSet::new(vec![pk(&sk(1)), pk(&sk(2))], 2).unwrap()
}

/// Provision a store for `robot` and CLAIM it (the first physical-possession
/// pairing) so subsequent strong / code pairings are permitted — an unclaimed
/// robot refuses every new pairing with `RobotUnclaimed`. The owner (0x01) is
/// distinct from `CLIENT_ACCT`, so the owner row never collides with the account
/// under test.
fn claimed_store(robot: RobotId) -> TrustStore {
    let mut s = TrustStore::provision(robot, pk(&sk(ROBOT_TRANSPORT_SEED)), root_set(), CHASSIS, 0)
        .unwrap();
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    s
}

// -- device identity (the key seam) ------------------------------------------

#[test]
fn device_identity_from_seed_is_deterministic_and_signs_verifiably() {
    use ed25519_dalek::Verifier;
    let id = DeviceIdentity::from_seed(&[42; 32]);
    let id2 = DeviceIdentity::from_seed(&[42; 32]);
    assert_eq!(id.public_key(), id2.public_key());
    assert_ne!(
        id.public_key(),
        DeviceIdentity::from_seed(&[43; 32]).public_key()
    );

    let sig = id.sign(b"hello");
    let vk = ed25519_dalek::VerifyingKey::from_bytes(&id.public_key().0).unwrap();
    assert!(vk
        .verify(b"hello", &ed25519_dalek::Signature::from_bytes(&sig.0))
        .is_ok());
}

// -- cert bundle -------------------------------------------------------------

fn build_bundle(client_key: PublicKey) -> CertBundle {
    let intermediate =
        make_intermediate(pk(&sk(10)), wide_validity(), ISSUED).sign_by_roots(&[&sk(1), &sk(2)]);
    let device_cert = make_device_cert(
        client_key,
        acct(CLIENT_ACCT),
        pk(&sk(10)),
        op_scope(),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let grant = make_grant(
        acct(CLIENT_ACCT),
        rob(ROBOT),
        op_scope(),
        0,
        acct(0x0C),
        pk(&sk(10)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    CertBundle::new(intermediate, device_cert).with_grant(grant)
}

#[test]
fn cert_bundle_picks_the_grant_for_the_target_robot() {
    let id = DeviceIdentity::from_seed(&[7; 32]);
    let bundle = build_bundle(id.public_key());
    assert!(bundle.presentation_for(&rob(ROBOT)).is_some());
    assert!(
        bundle.presentation_for(&rob(0xEE)).is_none(),
        "no grant for an unknown robot"
    );
}

// -- multi-robot DELEGATED bundle (per-robot delegation carry) ---------------

/// A depth-0 delegator scope that carries `CAP_DELEGATE` (so the depth-1 grant it
/// backs can attenuate down to `op_scope()`).
fn deleg_scope() -> Scope {
    Scope {
        role: Role::OPERATOR,
        caps: Scope::CAP_DELEGATE | Scope::CAP_TELEOP | Scope::CAP_OBSERVE,
    }
}

/// Build the delegatee's depth-1 grant for `robot` PLUS the `Delegation` that
/// backs it (the delegator's intermediate-issued depth-0 parent grant with
/// `CAP_DELEGATE` + the delegator's device cert). The delegator is a distinct
/// account/key per robot so the two robots' delegations are unambiguous.
fn delegated_grant_and_delegation(
    robot: RobotId,
    delegator_acct: u8,
    delegator_seed: u8,
) -> (SignedGrant, Delegation) {
    // The delegator's depth-0 grant: intermediate-issued (sk 10), for THIS robot,
    // subject == the delegator account, carrying CAP_DELEGATE.
    let parent = make_grant(
        acct(delegator_acct),
        robot,
        deleg_scope(),
        0,
        acct(0x0C),
        pk(&sk(10)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    // The delegator's device cert: intermediate-issued, binds the delegating
    // signing key (sk delegator_seed) to the delegator account.
    let delegator_cert = make_device_cert(
        pk(&sk(delegator_seed)),
        acct(delegator_acct),
        pk(&sk(10)),
        op_scope(),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    // The delegatee's depth-1 grant: subject == the client, issued + SIGNED by the
    // delegator's device key, attenuated to op_scope().
    let grant = make_grant(
        acct(CLIENT_ACCT),
        robot,
        op_scope(),
        1,
        acct(delegator_acct),
        pk(&sk(delegator_seed)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(delegator_seed));
    (
        grant,
        Delegation {
            parent,
            delegator_cert,
        },
    )
}

/// A single client bundle carrying depth-1 grants + delegations for TWO robots
/// (each via a distinct delegator).
fn build_delegated_bundle(client_key: PublicKey) -> CertBundle {
    let intermediate =
        make_intermediate(pk(&sk(10)), wide_validity(), ISSUED).sign_by_roots(&[&sk(1), &sk(2)]);
    let device_cert = make_device_cert(
        client_key,
        acct(CLIENT_ACCT),
        pk(&sk(10)),
        op_scope(),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let (g1, d1) = delegated_grant_and_delegation(rob(ROBOT), 0xD1, 31);
    let (g2, d2) = delegated_grant_and_delegation(rob(ROBOT_2), 0xD2, 32);
    CertBundle::new(intermediate, device_cert)
        .with_grant(g1)
        .with_grant(g2)
        .with_delegation(d1)
        .with_delegation(d2)
}

#[test]
fn multi_robot_delegated_bundle_verifies_both_robots() {
    let id = DeviceIdentity::from_seed(&[7; 32]);
    let client_key = id.public_key();
    let client = PairingClient::new(id, build_delegated_bundle(client_key));

    // Robot 1: the delegated chain must verify end-to-end.
    let pres1 = client
        .strong_presentation(&rob(ROBOT))
        .expect("grant for robot 1");
    let mut store1 = claimed_store(rob(ROBOT));
    let v1 = store1
        .verify_new_pairing(&pres1, &client_key, T_NOW)
        .expect("delegated chain must verify for robot 1");
    assert_eq!(v1.account(), acct(CLIENT_ACCT));

    // Robot 2: the regression this pins — a single `Option<Delegation>` slot can
    // satisfy only ONE robot; the other robot's presentation would carry the wrong
    // delegation and fail `InvalidDelegation`. With per-robot carry it verifies.
    let pres2 = client
        .strong_presentation(&rob(ROBOT_2))
        .expect("grant for robot 2");
    let mut store2 = claimed_store(rob(ROBOT_2));
    let v2 = store2
        .verify_new_pairing(&pres2, &client_key, T_NOW)
        .expect("delegated chain must verify for robot 2");
    assert_eq!(v2.account(), acct(CLIENT_ACCT));
}

#[test]
fn presentation_selects_the_matching_delegation_per_robot() {
    let id = DeviceIdentity::from_seed(&[7; 32]);
    let bundle = build_delegated_bundle(id.public_key());

    // Each presentation carries the delegation bound to ITS robot (parent grant
    // robot + delegator account), not just whichever was pushed last.
    let del1 = bundle
        .presentation_for(&rob(ROBOT))
        .unwrap()
        .delegation
        .expect("robot 1 carries a delegation");
    assert_eq!(del1.parent.grant.robot, rob(ROBOT));
    assert_eq!(del1.parent.grant.subject, acct(0xD1));

    let del2 = bundle
        .presentation_for(&rob(ROBOT_2))
        .unwrap()
        .delegation
        .expect("robot 2 carries a delegation");
    assert_eq!(del2.parent.grant.robot, rob(ROBOT_2));
    assert_eq!(del2.parent.grant.subject, acct(0xD2));

    // The two presentations selected DIFFERENT delegations (not the same clone).
    assert_ne!(del1.parent.grant.subject, del2.parent.grant.subject);
}

#[test]
fn presentation_selects_the_delegation_whose_delegator_cert_account_binds_the_issuer() {
    // Two delegations for the SAME robot with the SAME delegator device key,
    // differing ONLY in `delegator_cert.account`. `verify_delegation` requires
    // `delegator_cert.account == grant.issuer`, so `presentation_for` MUST select
    // the delegation that satisfies it — even though it was pushed SECOND — or the
    // presentation would carry a delegation the verifier rejects
    // (`InvalidDelegation`). The account-blind predicate would pick the decoy
    // (pushed first), so this test flips exactly on the added
    // `delegator_cert.cert.account == grant.issuer` clause.
    const DELEGATOR_ACCT: u8 = 0xD1;
    const DELEGATOR_SEED: u8 = 31;
    const DECOY_ACCT: u8 = 0xD9;

    let id = DeviceIdentity::from_seed(&[7; 32]);
    let client_key = id.public_key();

    let intermediate =
        make_intermediate(pk(&sk(10)), wide_validity(), ISSUED).sign_by_roots(&[&sk(1), &sk(2)]);
    let device_cert = make_device_cert(
        client_key,
        acct(CLIENT_ACCT),
        pk(&sk(10)),
        op_scope(),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));

    // The depth-1 grant: issuer account 0xD1, signed by the delegator device key.
    let grant = make_grant(
        acct(CLIENT_ACCT),
        rob(ROBOT),
        op_scope(),
        1,
        acct(DELEGATOR_ACCT),
        pk(&sk(DELEGATOR_SEED)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(DELEGATOR_SEED));

    // Both delegations share this identical, intermediate-issued parent grant.
    let parent = make_grant(
        acct(DELEGATOR_ACCT),
        rob(ROBOT),
        deleg_scope(),
        0,
        acct(0x0C),
        pk(&sk(10)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));

    // The VERIFYING delegator cert: account == the grant issuer (0xD1), key sk(31).
    let good_cert = make_device_cert(
        pk(&sk(DELEGATOR_SEED)),
        acct(DELEGATOR_ACCT),
        pk(&sk(10)),
        op_scope(),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    // A DECOY delegator cert differing ONLY in account (0xD9 != the grant issuer),
    // same delegator device key sk(31) — so the old, account-blind selection
    // predicate would match it just as well.
    let decoy_cert = make_device_cert(
        pk(&sk(DELEGATOR_SEED)),
        acct(DECOY_ACCT),
        pk(&sk(10)),
        op_scope(),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));

    let good = Delegation {
        parent: parent.clone(),
        delegator_cert: good_cert,
    };
    let decoy = Delegation {
        parent,
        delegator_cert: decoy_cert,
    };

    // Push the DECOY first so an account-blind `find` returns it.
    let bundle = CertBundle::new(intermediate, device_cert)
        .with_grant(grant)
        .with_delegation(decoy)
        .with_delegation(good);

    let pres = bundle
        .presentation_for(&rob(ROBOT))
        .expect("grant for the robot");
    let del = pres
        .delegation
        .clone()
        .expect("a delegation must be selected");
    assert_eq!(
        del.delegator_cert.cert.account,
        acct(DELEGATOR_ACCT),
        "must select the delegation whose delegator_cert.account binds the grant issuer, \
         not the decoy pushed first"
    );

    // End-to-end: the selected delegation VERIFIES (the decoy would have failed
    // InvalidDelegation), so the selection picks a delegation the verifier accepts.
    let mut store = claimed_store(rob(ROBOT));
    let v = store
        .verify_new_pairing(&pres, &client_key, T_NOW)
        .expect("the selected delegation must verify end-to-end");
    assert_eq!(v.account(), acct(CLIENT_ACCT));
}

// -- verification-shaped grant selection (direct + delegated for one robot) --

/// A depth-0 (direct) intermediate-issued grant for `robot` — the `build_bundle`
/// grant, extracted so a test can carry it alongside a delegated sibling.
fn direct_grant(robot: RobotId) -> SignedGrant {
    make_grant(
        acct(CLIENT_ACCT),
        robot,
        op_scope(),
        0,
        acct(0x0C),
        pk(&sk(10)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10))
}

#[test]
fn presentation_prefers_the_direct_grant_over_a_delegated_sibling_in_any_order() {
    // A bundle carrying BOTH a depth-0 (direct) and a depth-1 (delegated) grant for
    // the SAME robot — the "teammate upgraded from delegated to direct access
    // without pruning the old grant" case. `presentation_for` must present the
    // DIRECT grant (it verifies on its own, no delegation needed) REGARDLESS of
    // push order. A first-match `find` would present whichever grant was
    // pushed first, so a delegated-first bundle would carry a grant needing a
    // delegation and could fail while the direct sibling sat unused.
    let id = DeviceIdentity::from_seed(&[7; 32]);
    let client_key = id.public_key();

    let intermediate =
        make_intermediate(pk(&sk(10)), wide_validity(), ISSUED).sign_by_roots(&[&sk(1), &sk(2)]);
    let device_cert = make_device_cert(
        client_key,
        acct(CLIENT_ACCT),
        pk(&sk(10)),
        op_scope(),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let (delegated, delegation) = delegated_grant_and_delegation(rob(ROBOT), 0xD1, 31);

    // Identical material, opposite grant push order.
    let direct_first = CertBundle::new(intermediate.clone(), device_cert.clone())
        .with_grant(direct_grant(rob(ROBOT)))
        .with_grant(delegated.clone())
        .with_delegation(delegation.clone());
    let delegated_first = CertBundle::new(intermediate, device_cert)
        .with_grant(delegated)
        .with_grant(direct_grant(rob(ROBOT)))
        .with_delegation(delegation);

    for bundle in [direct_first, delegated_first] {
        let pres = bundle
            .presentation_for(&rob(ROBOT))
            .expect("a grant for the robot");
        // Hand oracle: the depth-0 direct grant is selected in BOTH orders, with
        // no delegation (selecting the delegated sibling would give depth 1).
        assert_eq!(
            pres.grant.grant.delegation_depth, 0,
            "the depth-0 direct grant must be preferred over the delegated sibling"
        );
        assert!(
            pres.delegation.is_none(),
            "a direct grant needs no delegation"
        );
        // And it verifies end-to-end (the actual point of preferring it).
        let mut store = claimed_store(rob(ROBOT));
        let v = store
            .verify_new_pairing(&pres, &client_key, T_NOW)
            .expect("the direct grant must verify end-to-end");
        assert_eq!(v.account(), acct(CLIENT_ACCT));
    }
}

#[test]
fn presentation_selects_the_delegation_pair_for_a_delegated_only_bundle() {
    // Regression pin for the delegated arm: a bundle with ONLY a depth-1 grant for
    // the robot (no direct grant) must still select that grant TOGETHER with its
    // matching delegation (the `find_map` arm), not drop the delegation.
    let id = DeviceIdentity::from_seed(&[7; 32]);
    let client_key = id.public_key();

    let intermediate =
        make_intermediate(pk(&sk(10)), wide_validity(), ISSUED).sign_by_roots(&[&sk(1), &sk(2)]);
    let device_cert = make_device_cert(
        client_key,
        acct(CLIENT_ACCT),
        pk(&sk(10)),
        op_scope(),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let (delegated, delegation) = delegated_grant_and_delegation(rob(ROBOT), 0xD1, 31);
    let bundle = CertBundle::new(intermediate, device_cert)
        .with_grant(delegated)
        .with_delegation(delegation);

    let pres = bundle
        .presentation_for(&rob(ROBOT))
        .expect("a grant for the robot");
    // Hand oracle: the depth-1 grant, carrying its bound delegation.
    assert_eq!(pres.grant.grant.delegation_depth, 1);
    let del = pres
        .delegation
        .clone()
        .expect("the delegated grant must carry its matching delegation");
    assert_eq!(del.parent.grant.robot, rob(ROBOT));
    assert_eq!(del.parent.grant.subject, acct(0xD1));

    let mut store = claimed_store(rob(ROBOT));
    let v = store
        .verify_new_pairing(&pres, &client_key, T_NOW)
        .expect("the delegated pair must verify end-to-end");
    assert_eq!(v.account(), acct(CLIENT_ACCT));
}

#[test]
fn robot_grant_without_matching_delegation_fails_invalid_delegation() {
    // A depth-1 grant is carried, but NO delegation is attached for it — the
    // presentation must carry `delegation: None` and fail exactly as before.
    let id = DeviceIdentity::from_seed(&[7; 32]);
    let client_key = id.public_key();
    let (g1, _unused_delegation) = delegated_grant_and_delegation(rob(ROBOT), 0xD1, 31);
    let intermediate =
        make_intermediate(pk(&sk(10)), wide_validity(), ISSUED).sign_by_roots(&[&sk(1), &sk(2)]);
    let device_cert = make_device_cert(
        client_key,
        acct(CLIENT_ACCT),
        pk(&sk(10)),
        op_scope(),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let bundle = CertBundle::new(intermediate, device_cert).with_grant(g1);

    let pres = bundle
        .presentation_for(&rob(ROBOT))
        .expect("grant for robot 1");
    assert!(
        pres.delegation.is_none(),
        "no delegation was attached for robot 1"
    );

    let mut store = claimed_store(rob(ROBOT));
    let err = store
        .verify_new_pairing(&pres, &client_key, T_NOW)
        .unwrap_err();
    assert!(
        matches!(err, PairingError::InvalidDelegation),
        "a depth-1 grant with no delegation must fail InvalidDelegation, got {err:?}"
    );
}

// -- end-to-end: strong path -------------------------------------------------

#[test]
fn strong_path_end_to_end_client_to_robot() {
    // Client side.
    let id = DeviceIdentity::from_seed(&[7; 32]);
    let client_key = id.public_key();
    let client = PairingClient::new(id, build_bundle(client_key));
    let pres = client
        .strong_presentation(&rob(ROBOT))
        .expect("client has a grant for this robot");

    // Robot side: provision, then verify the presentation against the peer key
    // the transport already authenticated (== the client's device key).
    let mut store = claimed_store(rob(ROBOT));
    let verified = store
        .verify_new_pairing(&pres, &client.public_key(), T_NOW)
        .expect("chain must verify");
    assert_eq!(verified.account(), acct(CLIENT_ACCT));
    assert_eq!(verified.device_key(), client_key);

    store
        .establish_pairing(
            &verified,
            PairingSource::StrongChain,
            Some("studio".into()),
            T_NOW,
            None,
        )
        .unwrap();
    assert!(store.is_allowed(&acct(CLIENT_ACCT), T_NOW).is_some());
}

#[test]
fn strong_path_rejects_a_forged_peer_key() {
    let id = DeviceIdentity::from_seed(&[7; 32]);
    let client = PairingClient::new(
        DeviceIdentity::from_seed(&[7; 32]),
        build_bundle(id.public_key()),
    );
    let pres = client.strong_presentation(&rob(ROBOT)).unwrap();

    let mut store = claimed_store(rob(ROBOT));
    // An attacker who is NOT the certified device key cannot pair, even holding
    // the (public) cert bundle.
    let attacker_key = DeviceIdentity::from_seed(&[66; 32]).public_key();
    assert!(store
        .verify_new_pairing(&pres, &attacker_key, T_NOW)
        .is_err());
}

// -- end-to-end: code (CPace) fallback path ----------------------------------

#[test]
fn code_path_end_to_end_establishes_a_visible_revocable_row() {
    // The ceremony's responder key MUST equal the key `claimed_store` provisioned
    // as the robot's own transport identity, else `establish_code_pairing` refuses
    // with `WitnessRobotMismatch` (cross-robot replay guard).
    let robot_key = pk(&sk(ROBOT_TRANSPORT_SEED));
    let client = PairingClient::new(DeviceIdentity::from_seed(&[7; 32]), {
        // The client can carry a bundle, but the code path does not need a grant.
        let intermediate = make_intermediate(pk(&sk(10)), wide_validity(), ISSUED)
            .sign_by_roots(&[&sk(1), &sk(2)]);
        let device_cert = make_device_cert(
            DeviceIdentity::from_seed(&[7; 32]).public_key(),
            acct(CLIENT_ACCT),
            pk(&sk(10)),
            op_scope(),
            wide_validity(),
            ISSUED,
        )
        .sign(&sk(10));
        CertBundle::new(intermediate, device_cert)
    });

    let code = "704 216"; // a short pairing code
    let context = b"tls-exporter".to_vec();

    // Client begins the (deliberate, explicit) fallback ceremony.
    let init = client.begin_code_pairing(code, robot_key, context.clone());

    // Robot runs the responder with the SAME code + bound identities.
    let robot_ids = PakeIdentities::new(client.public_key(), robot_key, context);
    let mut resp = CpaceResponder::new(code, robot_ids, CeremonyConfig::default(), 0);

    // Message exchange.
    let (attempt, msg1) = init.begin().unwrap();
    let r = resp.respond(&msg1, 0).unwrap();
    let (ik, ic) = attempt.finish(&r.msg2, &r.responder_confirm).unwrap();
    // The robot's finish mints the account-bound witness required to persist a row.
    let confirmed = resp
        .finish(&ic, 0, acct(CLIENT_ACCT), PrincipalKind::Human)
        .unwrap();
    assert_eq!(ik.session_key, confirmed.keys().session_key);

    // On success the robot records a visible, named, revocable, limited-scope row
    // — but ONLY with a valid ceremony-success witness, and ONLY on a claimed robot.
    let mut store = claimed_store(rob(ROBOT));
    store
        .establish_code_pairing(
            &confirmed,
            acct(CLIENT_ACCT),
            "Studio (code)".into(),
            Scope::CODE_PAIR_DEFAULT,
            T_NOW,
        )
        .unwrap();
    let row = store
        .is_allowed(&acct(CLIENT_ACCT), T_NOW)
        .expect("code-paired");
    assert_eq!(row.source, PairingSource::CodePaired);
    assert_eq!(row.scope, Scope::CODE_PAIR_DEFAULT);
    assert_eq!(row.name.as_deref(), Some("Studio (code)"));
}

// -- unclaimed -> claim -> reset lifecycle (composed) ------------------------

#[test]
fn unclaimed_claim_reset_lifecycle() {
    let mut store = TrustStore::provision(
        rob(ROBOT),
        pk(&sk(ROBOT_TRANSPORT_SEED)),
        root_set(),
        CHASSIS,
        0,
    )
    .unwrap();
    assert!(!store.is_claimed());

    // First physical-possession pairing writes the owner.
    store
        .claim(acct(CLIENT_ACCT), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    assert_eq!(store.owner(), Some(acct(CLIENT_ACCT)));

    // Physical factory reset returns to unclaimed.
    store.factory_reset(CHASSIS).unwrap();
    assert!(!store.is_claimed());
    assert!(store.is_allowed(&acct(CLIENT_ACCT), T_NOW).is_none());
}
