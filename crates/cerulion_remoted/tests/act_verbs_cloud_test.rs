// SPDX-License-Identifier: AGPL-3.0-only
//! The act verbs under the allow-from-cloud switch: a cloud device admitted by
//! certificate presentation may call `topic-publish` (mutating) and
//! `topic-snapshot` (observe) while the switch is on, and is refused both with
//! the switch's reason once the owner turns it off; the owner's own device is
//! untouched.

use cerud::authz::AuthzDecision;
use cerud::transport::CallerIdentity;
use cerud::verbs::VerbHandler;
use cerulion_pairing::format::{
    AccountId, DeviceCert, IntermediateCert, PrincipalKind, PublicKey, RobotId, RootSet, Scope,
    Validity, FORMAT_VERSION,
};
use cerulion_pairing::verify::{OwnerCertificatePresentationWire, TrustStore};
use cerulion_remoted::act_verbs::{TOPIC_PUBLISH, TOPIC_SNAPSHOT};
use cerulion_remoted::robot_settings::CloudAccessVerb;
use cerulion_remoted::trust::SharedTrust;
use cerulion_remoted::{DeviceAccountIndex, PairingAuthorizer};
use ed25519_dalek::SigningKey;

const T_NOW: u64 = 1_000_000_000_000;
const CHASSIS: &[u8] = b"act-verbs-cloud-test-random-chassis-secret";
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

fn chain(seed: u8, principal_kind: PrincipalKind) -> OwnerCertificatePresentationWire {
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
        validity,
        issued_at_ns: 1,
        issuer_key: pk(2),
    }
    .sign(&sk(2));
    OwnerCertificatePresentationWire::new(intermediate, device_cert)
}

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

#[test]
fn a_cloud_device_is_refused_the_act_verbs_only_while_the_switch_is_off() {
    let (authz, shared) = robot();
    for verb in [TOPIC_PUBLISH, TOPIC_SNAPSHOT] {
        assert_eq!(authz.authorize_verb(&caller(90), verb), AuthzDecision::Allow, "{verb} on");
    }
    CloudAccessVerb::new(shared.clone())
        .execute_with_caller(&caller(11), &serde_json::json!({ "allow": false }))
        .unwrap();
    for verb in [TOPIC_PUBLISH, TOPIC_SNAPSHOT] {
        match authz.authorize_verb(&caller(90), verb) {
            AuthzDecision::Deny { reason } => assert!(reason.contains(REFUSAL), "{verb}: {reason}"),
            other => panic!("{verb}: a cloud device was admitted with the switch off: {other:?}"),
        }
        assert_eq!(authz.authorize_verb(&caller(11), verb), AuthzDecision::Allow, "{verb} owner");
    }
}
