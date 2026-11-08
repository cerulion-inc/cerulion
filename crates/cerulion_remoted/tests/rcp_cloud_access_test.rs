// SPDX-License-Identifier: AGPL-3.0-only
//! The allow-from-cloud switch on the rcp plane, on the live trust state: an
//! owner desk and a cloud device admitted by certificate presentation (real
//! chains, fixed seeds); with the switch off the cloud device is refused at
//! accept and on every message class, and the owner desk is not.

use cerud::transport::CallerIdentity;
use cerud::verbs::VerbHandler;
use cerulion_link::alpn;
use cerulion_pairing::format::{
    AccountId, DeviceCert, IntermediateCert, PrincipalKind, PublicKey, RobotId, RootSet, Scope,
    Validity, FORMAT_VERSION,
};
use cerulion_pairing::verify::{OwnerCertificatePresentationWire, TrustStore};
use cerulion_remoted::rcp::{authorize_key, RcpClass};
use cerulion_remoted::robot_settings::CloudAccessVerb;
use cerulion_remoted::trust::SharedTrust;
use cerulion_remoted::{AcceptDecision, DeviceAccountIndex, PairingAuthorizer};
use ed25519_dalek::SigningKey;

const T_NOW: u64 = 1_000_000_000_000;
const CHASSIS: &[u8] = b"rcp-cloud-access-test-chassis-secret";
const OWNER: AccountId = AccountId([10; 32]);
const REFUSAL: &str = "allow-from-cloud switch is off";
const DESK: u8 = 11;
const CLOUD: u8 = 90;
const CLASSES: [RcpClass; 4] = [
    RcpClass::Session,
    RcpClass::Estop,
    RcpClass::Observe,
    RcpClass::Act,
];

fn sk(seed: u8) -> SigningKey {
    SigningKey::from_bytes(&[seed; 32])
}
fn pk(seed: u8) -> PublicKey {
    PublicKey(sk(seed).verifying_key().to_bytes())
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
    for (seed, kind) in [(DESK, PrincipalKind::Human), (CLOUD, PrincipalKind::Cloud)] {
        shared
            .verify_owner_certificate_and_establish(&chain(seed, kind), &pk(seed).0, T_NOW)
            .unwrap();
    }
    (authz, shared)
}

fn set_allow_cloud(shared: &SharedTrust, allow: bool) {
    CloudAccessVerb::new(shared.clone())
        .execute_with_caller(
            &CallerIdentity::verified(hex::encode(pk(DESK).0)),
            &serde_json::json!({ "allow": allow }),
        )
        .expect("the owner desk sets the switch");
}

#[test]
fn the_cloud_device_reaches_rcp_while_the_switch_is_on() {
    let (authz, shared) = robot();
    assert_eq!(
        authz.classify_accept(alpn::RCP, &pk(CLOUD).0),
        AcceptDecision::RcpAdmit
    );
    for class in CLASSES {
        assert_eq!(
            authorize_key(&shared, &pk(CLOUD).0, Some(class)),
            Ok(()),
            "{class:?}"
        );
    }
}

#[test]
fn switch_off_refuses_the_cloud_device_at_accept_and_on_every_message() {
    let (authz, shared) = robot();
    set_allow_cloud(&shared, false);
    match authz.classify_accept(alpn::RCP, &pk(CLOUD).0) {
        AcceptDecision::Refuse { reason } => assert!(reason.contains(REFUSAL), "{reason}"),
        other => panic!("the rcp plane admitted a cloud device: {other:?}"),
    }
    // A session admitted before the switch went off: every message is refused.
    let err = authorize_key(&shared, &pk(CLOUD).0, None).unwrap_err();
    assert!(err.contains(REFUSAL), "{err}");
    for class in CLASSES {
        let err = authorize_key(&shared, &pk(CLOUD).0, Some(class)).unwrap_err();
        assert!(err.contains(REFUSAL), "{class:?}: {err}");
    }
    // The owner's own desk is untouched.
    assert_eq!(
        authz.classify_accept(alpn::RCP, &pk(DESK).0),
        AcceptDecision::RcpAdmit
    );
    for class in CLASSES {
        assert_eq!(
            authorize_key(&shared, &pk(DESK).0, Some(class)),
            Ok(()),
            "{class:?}"
        );
    }
    // Back on: admitted again.
    set_allow_cloud(&shared, true);
    assert_eq!(
        authorize_key(&shared, &pk(CLOUD).0, Some(RcpClass::Act)),
        Ok(())
    );
}
