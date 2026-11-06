// SPDX-License-Identifier: AGPL-3.0-only
//! The allow-from-cloud switch, oracle-tested end to end on the live trust
//! state: owner devices and a cloud device are admitted by certificate
//! presentation (real chains, fixed seeds), then the owner turns the switch off
//! and every plane and every verb refuses the cloud device only.

use cerud::authz::{Authorizer, AuthzDecision};
use cerud::transport::CallerIdentity;
use cerud::verbs::VerbHandler;
use cerulion_core::transport::demand_authorizer::{
    DemandAuthorizer, DemandDecision, DemandSubject,
};
use cerulion_link::alpn;
use cerulion_pairing::format::{
    AccountId, DeviceCert, IntermediateCert, PrincipalKind, PublicKey, RobotId, RootSet, Scope,
    Validity, FORMAT_VERSION,
};
use cerulion_pairing::verify::{OwnerCertificatePresentationWire, TrustStore};
use cerulion_remoted::robot_settings::CloudAccessVerb;
use cerulion_remoted::trust::{KeyAccess, SharedTrust};
use cerulion_remoted::{verbs, AcceptDecision, DeviceAccountIndex, PairingAuthorizer};
use ed25519_dalek::SigningKey;

const T_NOW: u64 = 1_000_000_000_000;
const CHASSIS: &[u8] = b"cloud-access-test-random-chassis-secret";
const OWNER: AccountId = AccountId([10; 32]);
const REFUSAL: &str = "allow-from-cloud switch is off";

fn sk(seed: u8) -> SigningKey {
    SigningKey::from_bytes(&[seed; 32])
}
fn pk(seed: u8) -> PublicKey {
    PublicKey(sk(seed).verifying_key().to_bytes())
}
fn caller(seed: u8) -> CallerIdentity {
    CallerIdentity::verified(hex::encode(pk(seed).0))
}

/// The owner account's certificate chain for device `seed`.
fn chain(seed: u8, principal_kind: PrincipalKind) -> OwnerCertificatePresentationWire {
    chain_until(seed, principal_kind, u64::MAX)
}

/// [`chain`] with the device certificate valid until `not_after_ns`.
fn chain_until(
    seed: u8,
    principal_kind: PrincipalKind,
    not_after_ns: u64,
) -> OwnerCertificatePresentationWire {
    let validity = Validity {
        not_before_ns: 0,
        not_after_ns: u64::MAX,
    };
    let intermediate = IntermediateCert {
        version: FORMAT_VERSION,
        intermediate_key: pk(2),
        validity,
        issued_at_ns: 1,
        max_scope: Scope::OWNER_FULL,
    }
    .sign_by_roots(&[&sk(1)]);
    let device_cert = DeviceCert {
        version: FORMAT_VERSION,
        device_key: pk(seed),
        account: OWNER,
        principal_kind,
        scope: Scope::OWNER_FULL,
        validity: Validity {
            not_before_ns: 0,
            not_after_ns,
        },
        issued_at_ns: 1,
        issuer_key: pk(2),
    }
    .sign(&sk(2));
    OwnerCertificatePresentationWire::new(intermediate, device_cert)
}

/// A claimed robot with owner desk 11 and cloud device 90 admitted.
fn robot() -> (PairingAuthorizer, SharedTrust) {
    let roots = RootSet::new(vec![pk(1)], 1).unwrap();
    let mut store = TrustStore::provision(RobotId([5; 32]), pk(6), roots, CHASSIS, T_NOW).unwrap();
    store
        .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    let authz = PairingAuthorizer::new(store, DeviceAccountIndex::new());
    let shared = authz.shared().clone();
    for (seed, kind) in [(11, PrincipalKind::Human), (90, PrincipalKind::Cloud)] {
        shared
            .verify_owner_certificate_and_establish(&chain(seed, kind), &pk(seed).0, T_NOW)
            .unwrap();
    }
    (authz, shared)
}

fn set(shared: &SharedTrust, seed: u8, allow: bool) -> Result<serde_json::Value, String> {
    CloudAccessVerb::new(shared.clone())
        .execute_with_caller(&caller(seed), &serde_json::json!({ "allow": allow }))
        .map_err(|e| e.to_string())
}

fn denied(decision: AuthzDecision) -> String {
    match decision {
        AuthzDecision::Deny { reason } => reason,
        other => panic!("expected a denial, got {other:?}"),
    }
}

#[test]
fn a_cloud_device_is_admitted_like_any_owner_device_while_the_switch_is_on() {
    let (authz, shared) = robot();
    assert!(shared.allow_cloud());
    assert_eq!(
        shared.snapshot_for_key(&pk(90).0).1,
        KeyAccess::Allowed(Scope::OWNER_FULL)
    );
    let row = shared.snapshot_row_for_key(&pk(90).0).2.unwrap();
    assert_eq!(row.principal_kind, PrincipalKind::Cloud);
    assert_eq!(row.device, Some(pk(90)));
    assert_eq!(
        authz.authorize_verb(&caller(90), verbs::INVENTORY),
        AuthzDecision::Allow
    );
    assert_eq!(
        authz.classify_accept(alpn::OPS, &pk(90).0),
        AcceptDecision::OpsAdmit
    );
    assert_eq!(
        authz.classify_accept(alpn::WIRE, &pk(90).0),
        AcceptDecision::WireAdmit
    );
}

#[test]
fn switch_off_refuses_the_cloud_device_on_every_plane_and_verb_and_nothing_else() {
    let (authz, shared) = robot();
    assert_eq!(
        set(&shared, 11, false).unwrap(),
        serde_json::json!({ "allow_cloud": false })
    );
    assert_eq!(
        shared.snapshot_for_key(&pk(90).0).1,
        KeyAccess::CloudRefused
    );
    assert_eq!(shared.snapshot_row_for_key(&pk(90).0).2, None);
    for verb in [
        verbs::INVENTORY,
        verbs::RESTART,
        verbs::TELEOP,
        verbs::ENGAGE_ESTOP,
        verbs::CLAIM,
        verbs::PAIR,
        verbs::PRESENT_GRANT,
        verbs::CODE_PAIR_START,
        verbs::CODE_PAIR_FINISH,
        verbs::CLOUD_ACCESS,
    ] {
        let reason = denied(authz.authorize_verb(&caller(90), verb));
        assert!(reason.contains(REFUSAL), "{verb}: {reason}");
        // The cerud seam routes through the same decision.
        assert!(matches!(
            Authorizer::authorize(&authz, &caller(90), verb, &serde_json::Value::Null),
            AuthzDecision::Deny { .. }
        ));
    }
    for plane in [alpn::OPS, alpn::WIRE, b"cerulion/next/1".as_slice()] {
        match authz.classify_accept(plane, &pk(90).0) {
            AcceptDecision::Refuse { reason } => assert!(reason.contains(REFUSAL), "{reason}"),
            other => panic!("{plane:?} admitted a cloud device: {other:?}"),
        }
    }
    match authz.authorize_demand(
        &DemandSubject::Wan {
            demander_key: pk(90).0,
        },
        "/tf",
    ) {
        DemandDecision::Deny { reason } => assert!(reason.contains(REFUSAL), "{reason}"),
        other => panic!("the wire demand admitted a cloud device: {other:?}"),
    }
    // The owner's own device is untouched.
    assert_eq!(
        authz.authorize_verb(&caller(11), verbs::INVENTORY),
        AuthzDecision::Allow
    );
    assert_eq!(
        authz.classify_accept(alpn::WIRE, &pk(11).0),
        AcceptDecision::WireAdmit
    );
    // A new cloud certificate is refused while the switch is off.
    let err = shared
        .verify_owner_certificate_and_establish(&chain(91, PrincipalKind::Cloud), &pk(91).0, T_NOW)
        .unwrap_err()
        .to_string();
    assert!(
        err.contains("does not allow access from the cloud"),
        "{err}"
    );
    assert_eq!(shared.snapshot_for_key(&pk(91).0).1, KeyAccess::Unpaired);
    // Back on: the cloud device is admitted again.
    set(&shared, 11, true).unwrap();
    assert_eq!(
        authz.authorize_verb(&caller(90), verbs::INVENTORY),
        AuthzDecision::Allow
    );
}

#[test]
fn only_an_owner_non_cloud_device_may_set_the_switch() {
    let (authz, shared) = robot();
    // The cloud device holds owner scope but never controls its own gate.
    assert!(set(&shared, 90, false).is_err());
    assert!(shared.allow_cloud());
    // An unbound key is refused by the handler, and the authorizer table
    // demands the owner role for the verb.
    assert!(set(&shared, 44, false).is_err());
    assert!(shared.allow_cloud());
    assert!(matches!(
        authz.authorize_verb(&caller(44), verbs::CLOUD_ACCESS),
        AuthzDecision::Deny { .. }
    ));
    assert_eq!(
        authz.authorize_verb(&caller(11), verbs::CLOUD_ACCESS),
        AuthzDecision::Allow
    );
    // Reading needs no argument and changes nothing.
    assert_eq!(
        CloudAccessVerb::new(shared.clone())
            .execute_with_caller(&caller(11), &serde_json::json!({}))
            .unwrap(),
        serde_json::json!({ "allow_cloud": true })
    );
    assert!(CloudAccessVerb::new(shared)
        .execute_with_caller(&caller(11), &serde_json::json!({ "allow": "no" }))
        .is_err());
}

/// An epoch for this robot revoking `devices`, signed by the intermediate.
fn epoch(
    n: u64,
    devices: Vec<PublicKey>,
    issued_at_ns: u64,
) -> cerulion_pairing::format::SignedEpoch {
    cerulion_pairing::format::AccessListEpoch {
        version: FORMAT_VERSION,
        robot: RobotId([5; 32]),
        epoch: n,
        revoked_accounts: vec![],
        revoked_devices: devices,
        issued_at_ns,
        issuer_key: pk(2),
    }
    .sign(&sk(2))
}

/// The robot's trust state viewed at `now_ns` on its own clock.
fn at(store: TrustStore, now_ns: u64) -> PairingAuthorizer {
    let mut index = DeviceAccountIndex::new();
    index.bind(pk(90), OWNER);
    index.bind(pk(11), OWNER);
    PairingAuthorizer::from_shared(SharedTrust::new_with_clock(
        store,
        index,
        Vec::new(),
        cerulion_remoted::RemotedClock::fixed(now_ns),
    ))
}

#[test]
fn a_lapsed_cloud_row_is_refused_at_accept_and_per_verb_after_its_revocation_is_pruned() {
    let lapses = T_NOW + 50;
    let roots = RootSet::new(vec![pk(1)], 1).unwrap();
    let mut store = TrustStore::provision(RobotId([5; 32]), pk(6), roots, CHASSIS, T_NOW).unwrap();
    store
        .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    let cloud = chain_until(90, PrincipalKind::Cloud, lapses);
    store
        .verify_owner_certificate(&cloud, &pk(90), T_NOW + 1)
        .unwrap();
    store
        .verify_owner_certificate(&chain(11, PrincipalKind::Human), &pk(11), T_NOW + 1)
        .unwrap();
    store
        .apply_epoch(
            &epoch(1, vec![pk(90)], T_NOW + 10),
            &cloud.intermediate,
            T_NOW + 10,
        )
        .unwrap();
    store
        .apply_epoch(
            &epoch(2, vec![], T_NOW + 60),
            &cloud.intermediate,
            T_NOW + 60,
        )
        .unwrap();
    assert!(
        store.revoked_devices().is_empty(),
        "the revocation was pruned"
    );
    let authz = at(store, T_NOW + 60);
    let shared = authz.shared().clone();
    assert_eq!(shared.snapshot_for_key(&pk(90).0).1, KeyAccess::NotAllowed);
    for verb in [verbs::INVENTORY, verbs::TELEOP, verbs::ENGAGE_ESTOP] {
        assert!(
            matches!(
                authz.authorize_verb(&caller(90), verb),
                AuthzDecision::Deny { .. }
            ),
            "{verb}"
        );
    }
    assert!(matches!(
        authz.classify_accept(alpn::WIRE, &pk(90).0),
        AcceptDecision::Refuse { .. }
    ));
    assert_eq!(
        authz.classify_accept(alpn::OPS, &pk(90).0),
        AcceptDecision::OpsBootstrapOnly
    );
    // Re-presenting the lapsed certificate does not bring the row back.
    assert!(shared
        .verify_owner_certificate_and_establish(&cloud, &pk(90).0, T_NOW + 61)
        .is_err());
    assert_eq!(shared.snapshot_for_key(&pk(90).0).1, KeyAccess::NotAllowed);
    // The owner's own device is admitted throughout.
    assert_eq!(
        authz.authorize_verb(&caller(11), verbs::INVENTORY),
        AuthzDecision::Allow
    );
}

#[test]
fn a_cloud_certificate_renewed_before_it_lapses_stays_admitted_at_accept_and_per_verb() {
    let roots = RootSet::new(vec![pk(1)], 1).unwrap();
    let mut store = TrustStore::provision(RobotId([5; 32]), pk(6), roots, CHASSIS, T_NOW).unwrap();
    store
        .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    store
        .verify_owner_certificate(
            &chain_until(90, PrincipalKind::Cloud, T_NOW + 50),
            &pk(90),
            T_NOW + 1,
        )
        .unwrap();
    store
        .verify_owner_certificate(
            &chain_until(90, PrincipalKind::Cloud, T_NOW + 90),
            &pk(90),
            T_NOW + 45,
        )
        .unwrap();
    let authz = at(store, T_NOW + 60);
    assert_eq!(
        authz.authorize_verb(&caller(90), verbs::INVENTORY),
        AuthzDecision::Allow
    );
    assert_eq!(
        authz.classify_accept(alpn::WIRE, &pk(90).0),
        AcceptDecision::WireAdmit
    );
}
