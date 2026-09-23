// SPDX-License-Identifier: AGPL-3.0-only
//! A pairing code proves code possession, never membership of a named account.

use cerud::authz::AuthzDecision;
use cerud::transport::CallerIdentity;
use cerud::verbs::VerbHandler;
use cerulion_pairing::format::{
    AccountId, DeviceCert, Grant, IntermediateCert, PrincipalKind, PublicKey, RobotId, RootSet,
    Scope, Validity, FORMAT_VERSION,
};
use cerulion_pairing::pake::{CeremonyConfig, CpaceInitiator, PakeIdentities};
use cerulion_pairing::verify::{PairingSource, TrustStore};
use cerulion_remoted::pairing_verbs::{
    CodePairFinishVerb, CodePairStartVerb, PairVerb, PresentationWire, SharedCodePairSessions,
};
use cerulion_remoted::{
    DeviceAccountIndex, KeyAccess, PairingAuthorizer, RemotedClock, SharedTrust,
};
use ed25519_dalek::SigningKey;

const NOW: u64 = 10;
const OWNER: AccountId = AccountId([42; 32]);
const ROBOT: RobotId = RobotId([5; 32]);
const CODE: &str = "guest-test-code";
const MAC: &[u8] = b"code-account-test-store-key";

fn key(seed: u8) -> SigningKey {
    SigningKey::from_bytes(&[seed; 32])
}
fn public(seed: u8) -> PublicKey {
    PublicKey(key(seed).verifying_key().to_bytes())
}
fn caller(seed: u8) -> CallerIdentity {
    CallerIdentity::verified(hex::encode(public(seed).0))
}

struct Fixture {
    directory: tempfile::TempDir,
    shared: SharedTrust,
    start: CodePairStartVerb,
    finish: CodePairFinishVerb,
}

impl Fixture {
    fn new() -> Self {
        let directory = tempfile::tempdir().unwrap();
        let mut store = TrustStore::provision(
            ROBOT,
            public(6),
            RootSet::new(vec![public(1)], 1).unwrap(),
            b"unit recovery",
            NOW,
        )
        .unwrap()
        .with_path(directory.path().join("store"));
        store
            .claim(OWNER, b"unit recovery", PrincipalKind::Human, NOW)
            .unwrap();
        store.save(MAC).unwrap();
        let index = DeviceAccountIndex::new().with_path(directory.path().join("index"));
        index.save(MAC).unwrap();
        let clock = RemotedClock::fixed(NOW);
        let shared = SharedTrust::new_with_clock(store, index, MAC.to_vec(), clock.clone());
        let sessions = SharedCodePairSessions::new();
        sessions.arm(CODE, CeremonyConfig::default());
        Self {
            directory,
            start: CodePairStartVerb::new(shared.clone(), sessions.clone(), clock.clone()),
            finish: CodePairFinishVerb::new(shared.clone(), sessions, clock),
            shared,
        }
    }

    fn confirmation(&self, seed: u8) -> Vec<u8> {
        let initiator =
            CpaceInitiator::new(CODE, PakeIdentities::new(public(seed), public(6), vec![]));
        let (attempt, msg1) = initiator.begin().unwrap();
        let start = self
            .start
            .execute_with_caller(
                &caller(seed),
                &serde_json::json!({"msg1":hex::encode(msg1)}),
            )
            .unwrap();
        let msg2 = hex::decode(start["msg2"].as_str().unwrap()).unwrap();
        let confirm = hex::decode(start["responder_confirm"].as_str().unwrap()).unwrap();
        attempt.finish(&msg2, &confirm).unwrap().1
    }

    fn store(&self) -> TrustStore {
        TrustStore::load(self.directory.path().join("store"), MAC).unwrap()
    }

    fn binding(&self, seed: u8) -> Option<AccountId> {
        DeviceAccountIndex::load(self.directory.path().join("index"), MAC)
            .unwrap()
            .account_for(&public(seed))
    }
}

fn finish_args(account: AccountId, confirmation: &[u8]) -> serde_json::Value {
    serde_json::json!({"account":hex::encode(account.0),"initiator_confirm":hex::encode(confirmation),"name":"guest"})
}

fn strong_owner_presentation() -> serde_json::Value {
    let validity = Validity {
        not_before_ns: 0,
        not_after_ns: 100,
    };
    let presentation = PresentationWire {
        intermediate: IntermediateCert {
            version: FORMAT_VERSION,
            intermediate_key: public(2),
            validity,
            issued_at_ns: 1,
            max_scope: Scope::OWNER_FULL,
        }
        .sign_by_roots(&[&key(1)]),
        device_cert: DeviceCert {
            version: FORMAT_VERSION,
            device_key: public(3),
            account: OWNER,
            principal_kind: PrincipalKind::Human,
            scope: Scope::OWNER_FULL,
            validity,
            issued_at_ns: 1,
            issuer_key: public(2),
        }
        .sign(&key(2)),
        grant: Grant {
            version: FORMAT_VERSION,
            subject: OWNER,
            robot: ROBOT,
            scope: Scope::OWNER_FULL,
            principal_kind: PrincipalKind::Human,
            delegation_depth: 0,
            issuer: OWNER,
            issuer_key: public(2),
            validity,
            issued_at_ns: 1,
        }
        .sign(&key(2)),
        delegation: None,
    };
    serde_json::json!({"presentation_postcard":hex::encode(postcard::to_stdvec(&presentation).unwrap())})
}

#[test]
fn guest_cannot_name_owner_and_a_later_strong_owner_pair_cannot_elevate_guest() {
    let fixture = Fixture::new();
    let confirmation = fixture.confirmation(4);
    let error = fixture
        .finish
        .execute_with_caller(&caller(4), &finish_args(OWNER, &confirmation))
        .unwrap_err();
    assert!(error.to_string().contains("self-account"), "{error}");
    assert_eq!(fixture.binding(4), None);
    assert_eq!(
        fixture.shared.snapshot_for_key(&public(4).0).1,
        KeyAccess::Unpaired
    );
    let store = fixture.store();
    let row = store.is_allowed(&OWNER, NOW).unwrap();
    assert_eq!(row.scope, Scope::OWNER_FULL);
    assert_eq!(row.source, PairingSource::Claim);

    // The same valid code ceremony can enroll only its authenticated self-account.
    let guest = AccountId(public(4).0);
    let result = fixture
        .finish
        .execute_with_caller(&caller(4), &finish_args(guest, &confirmation))
        .unwrap();
    assert_eq!(
        result,
        serde_json::json!({"paired":true,"account":hex::encode(guest.0),"source":"code_paired"})
    );
    assert_eq!(fixture.binding(4), Some(guest));
    assert_eq!(
        fixture.store().is_allowed(&guest, NOW).unwrap().scope,
        Scope::CODE_PAIR_DEFAULT
    );

    // Exercise the original full strong-chain path that can replace an account row.
    PairVerb::new(fixture.shared.clone(), RemotedClock::fixed(NOW))
        .execute_with_caller(&caller(3), &strong_owner_presentation())
        .unwrap();
    assert_eq!(fixture.binding(3), Some(OWNER));
    assert_eq!(
        fixture.shared.snapshot_for_key(&public(3).0).1,
        KeyAccess::Allowed(Scope::OWNER_FULL)
    );
    assert_eq!(
        fixture.shared.snapshot_for_key(&public(4).0).1,
        KeyAccess::Allowed(Scope::CODE_PAIR_DEFAULT)
    );
    let authorizer = PairingAuthorizer::from_shared(fixture.shared.clone());
    assert!(matches!(
        authorizer.authorize_verb(&caller(4), "restart"),
        AuthzDecision::Deny { .. }
    ));
    assert_eq!(
        authorizer.authorize_verb(&caller(3), "restart"),
        AuthzDecision::Allow
    );
}

#[test]
fn bare_cloud_account_and_other_devices_confirmation_cannot_enroll_a_guest() {
    let fixture = Fixture::new();
    let confirmation = fixture.confirmation(4);
    let cloud = AccountId([77; 32]);
    let error = fixture
        .finish
        .execute_with_caller(&caller(4), &finish_args(cloud, &confirmation))
        .unwrap_err();
    assert!(error.to_string().contains("self-account"));
    assert!(fixture
        .finish
        .execute_with_caller(
            &caller(7),
            &finish_args(AccountId(public(7).0), &confirmation)
        )
        .is_err());
    assert_eq!(fixture.binding(4), None);
    assert_eq!(fixture.binding(7), None);
    assert!(fixture.store().is_allowed(&cloud, NOW).is_none());
}

#[test]
fn self_account_still_requires_authenticated_caller_and_valid_code_confirmation() {
    let fixture = Fixture::new();
    let mut confirmation = fixture.confirmation(4);
    confirmation[0] ^= 1;
    let args = finish_args(AccountId(public(4).0), &confirmation);
    assert!(fixture.finish.execute(&args).is_err());
    assert!(fixture
        .finish
        .execute_with_caller(
            &CallerIdentity {
                id: caller(4).id,
                authenticated: false
            },
            &args
        )
        .is_err());
    assert!(fixture
        .finish
        .execute_with_caller(&caller(4), &args)
        .is_err());
    assert_eq!(fixture.binding(4), None);
    assert!(fixture
        .store()
        .is_allowed(&AccountId(public(4).0), NOW)
        .is_none());
}
