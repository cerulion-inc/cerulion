// SPDX-License-Identifier: AGPL-3.0-only
//! The existing pair verb admits owner devices only with a bound certificate chain.

use cerud::transport::CallerIdentity;
use cerud::verbs::VerbHandler;
use cerulion_pairing::format::{
    AccountId, DeviceCert, IntermediateCert, PrincipalKind, PublicKey, RobotId, RootSet, Scope,
    Validity, FORMAT_VERSION,
};
use cerulion_pairing::verify::{OwnerCertificatePresentationWire, TrustStore};
use cerulion_remoted::pairing_verbs::{PairVerb, PresentationWire};
use cerulion_remoted::{DeviceAccountIndex, KeyAccess, RemotedClock, SharedTrust};
use ed25519_dalek::SigningKey;

const NOW: u64 = 10;
const OWNER: AccountId = AccountId([4; 32]);
fn key(seed: u8) -> SigningKey {
    SigningKey::from_bytes(&[seed; 32])
}
fn public(seed: u8) -> PublicKey {
    PublicKey(key(seed).verifying_key().to_bytes())
}
fn proof(account: AccountId, scope: Scope) -> OwnerCertificatePresentationWire {
    let validity = Validity {
        not_before_ns: 0,
        not_after_ns: 100,
    };
    OwnerCertificatePresentationWire::new(
        IntermediateCert {
            version: FORMAT_VERSION,
            intermediate_key: public(2),
            validity,
            issued_at_ns: 1,
            max_scope: Scope::OWNER_FULL,
        }
        .sign_by_roots(&[&key(1)]),
        DeviceCert {
            version: FORMAT_VERSION,
            device_key: public(3),
            account,
            principal_kind: PrincipalKind::Human,
            scope,
            validity,
            issued_at_ns: 1,
            issuer_key: public(2),
        }
        .sign(&key(2)),
    )
}
fn fixture() -> (SharedTrust, PairVerb, CallerIdentity) {
    let mut store = TrustStore::provision(
        RobotId([5; 32]),
        public(6),
        RootSet::new(vec![public(1)], 1).unwrap(),
        b"unit recovery",
        NOW,
    )
    .unwrap();
    store
        .claim(OWNER, b"unit recovery", PrincipalKind::Human, NOW)
        .unwrap();
    let shared = SharedTrust::new_with_clock(
        store,
        DeviceAccountIndex::new(),
        vec![],
        RemotedClock::fixed(NOW),
    );
    let verb = PairVerb::new(shared.clone(), RemotedClock::fixed(NOW));
    (
        shared,
        verb,
        CallerIdentity::verified(hex::encode(public(3).0)),
    )
}
fn owner_args(p: &OwnerCertificatePresentationWire) -> serde_json::Value {
    serde_json::json!({"owner_certificate_postcard": hex::encode(p.to_postcard().unwrap())})
}

#[test]
fn pair_owner_certificate_binds_current_owner_and_preserves_response_contract() {
    let (shared, verb, caller) = fixture();
    assert_eq!(shared.snapshot_for_key(&public(3).0).1, KeyAccess::Unpaired);
    let result = verb
        .execute_with_caller(&caller, &owner_args(&proof(OWNER, Scope::OWNER_FULL)))
        .unwrap();
    assert_eq!(
        result,
        serde_json::json!({"paired": true, "account": hex::encode(OWNER.0), "source": "strong_chain"})
    );
    assert_eq!(
        shared.snapshot_for_key(&public(3).0).1,
        KeyAccess::Allowed(Scope::OWNER_FULL)
    );
}

#[test]
fn pair_requires_exactly_one_well_formed_proof_and_authenticated_caller() {
    let (shared, verb, caller) = fixture();
    let valid = owner_args(&proof(OWNER, Scope::OWNER_FULL));
    let mut both = valid.clone();
    both["presentation_postcard"] = serde_json::json!("");
    let mut trailing = proof(OWNER, Scope::OWNER_FULL).to_postcard().unwrap();
    trailing.push(0);
    for args in [
        serde_json::json!({}),
        both,
        serde_json::json!({"owner_certificate_postcard": null}),
        serde_json::json!({"owner_certificate_postcard": "zz"}),
        serde_json::json!({"owner_certificate_postcard": ""}),
        serde_json::json!({"owner_certificate_postcard": "02"}),
        serde_json::json!({"owner_certificate_postcard": hex::encode(trailing)}),
    ] {
        assert!(verb.execute_with_caller(&caller, &args).is_err(), "{args}");
        assert_eq!(shared.snapshot_for_key(&public(3).0).1, KeyAccess::Unpaired);
    }
    assert!(verb.execute(&valid).is_err());
    assert!(verb
        .execute_with_caller(
            &CallerIdentity {
                id: caller.id.clone(),
                authenticated: false
            },
            &valid
        )
        .is_err());
}

#[test]
fn pair_owner_certificate_never_admits_nonowner_narrow_scope_or_other_transport_key() {
    let (shared, verb, caller) = fixture();
    for p in [
        proof(AccountId([9; 32]), Scope::OWNER_FULL),
        proof(OWNER, Scope::CODE_PAIR_DEFAULT),
    ] {
        assert!(verb.execute_with_caller(&caller, &owner_args(&p)).is_err());
    }
    let other = CallerIdentity::verified(hex::encode(public(7).0));
    assert!(verb
        .execute_with_caller(&other, &owner_args(&proof(OWNER, Scope::OWNER_FULL)))
        .is_err());
    assert_eq!(shared.snapshot_for_key(&public(3).0).1, KeyAccess::Unpaired);
    assert_eq!(shared.snapshot_for_key(&public(7).0).1, KeyAccess::Unpaired);
}

#[test]
fn existing_grant_presentation_still_admits_nonowner() {
    use cerulion_pairing::format::Grant;
    let (shared, verb, caller) = fixture();
    let guest = AccountId([9; 32]);
    let proof = proof(guest, Scope::CODE_PAIR_DEFAULT);
    let wire = PresentationWire {
        intermediate: proof.intermediate,
        device_cert: proof.device_cert,
        grant: Grant {
            version: FORMAT_VERSION,
            subject: guest,
            robot: RobotId([5; 32]),
            scope: Scope::CODE_PAIR_DEFAULT,
            principal_kind: PrincipalKind::Human,
            delegation_depth: 0,
            validity: Validity {
                not_before_ns: 0,
                not_after_ns: 100,
            },
            issued_at_ns: 1,
            issuer: AccountId([2; 32]),
            issuer_key: public(2),
        }
        .sign(&key(2)),
        delegation: None,
    };
    let result = verb
        .execute_with_caller(
            &caller,
            &serde_json::json!({"presentation_postcard": wire.to_postcard_hex().unwrap()}),
        )
        .unwrap();
    assert_eq!(result["account"], hex::encode(guest.0));
    assert_eq!(
        shared.snapshot_for_key(&public(3).0).1,
        KeyAccess::Allowed(Scope::CODE_PAIR_DEFAULT)
    );
}
