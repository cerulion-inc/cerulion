// SPDX-License-Identifier: AGPL-3.0-only
//! The `PairingAuthorizer` decision MATRIX, oracle-tested against hand vectors.
//!
//! Every cell of the sacred verb → capability table is asserted literally:
//! {paired-with-cap, paired-without-cap, paired-any + engage-estop,
//! unpaired + normal, unpaired + bootstrap, unclaimed × {claim, pair, normal}},
//! plus the deny-by-default unclassified verb and the unauthenticated caller.
//!
//! Access rows are seeded with REAL cryptography (deterministic fixed-seed keys
//! — Principle #13): the owner via a physical-possession `claim`, and a
//! restricted VIEWER+CAP_OBSERVE account via a full offline certificate chain
//! (`VerifiedPairing::new` is crate-private to cerulion_pairing, so the chain is
//! the only legitimate way to mint a controlled-scope row).

use cerud::authz::AuthzDecision;
use cerud::transport::CallerIdentity;
use cerud::verbs::{InventoryVerb, LogTailVerb, RestartVerb, VerbHandler};

use cerulion_link::alpn;
use cerulion_pairing::format::{
    AccessListEpoch, AccountId, DeviceCert, Grant, IntermediateCert, PrincipalKind, PublicKey,
    RobotId, Role, RootSet, Scope, Validity, FORMAT_VERSION,
};
use cerulion_pairing::verify::{PairingPresentation, TrustStore};
use ed25519_dalek::SigningKey;

use cerulion_remoted::{verbs, AcceptDecision, DeviceAccountIndex, PairingAuthorizer};

// ── Fixed deterministic fixtures (never fake/simulated — Principle #13) ──────

const T_NOW: u64 = 1_000_000_000_000;
const ISSUED: u64 = 500_000_000_000;
const CHASSIS: &[u8] = b"remoted-unit-random-chassis-secret-not-serial-derived";

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

/// Caller identity for a device key = the iroh transport's `verified(hex)`.
fn caller_for(key: &PublicKey) -> CallerIdentity {
    CallerIdentity::verified(hex::encode(key.0))
}

const OWNER_ACCOUNT: AccountId = AccountId([10; 32]);
const VIEWER_ACCOUNT: AccountId = AccountId([20; 32]);
const OPERATOR_ACCOUNT: AccountId = AccountId([30; 32]);

/// Establish an account row via a REAL offline certificate chain with a chosen
/// scope (`VerifiedPairing::new` is crate-private, so the chain is the only
/// legitimate way to mint a controlled-scope row). Returns the account's device key.
#[allow(clippy::too_many_arguments)]
fn establish_via_chain(
    store: &mut TrustStore,
    root: &SigningKey,
    intermediate: &SigningKey,
    robot: RobotId,
    account: AccountId,
    device: &SigningKey,
    scope: Scope,
    name: &str,
) -> PublicKey {
    let device_key = pk(device);
    let int_pk = pk(intermediate);
    let int_cert = IntermediateCert {
        version: FORMAT_VERSION,
        intermediate_key: int_pk,
        validity: wide(),
        issued_at_ns: ISSUED,
        max_scope: Scope::OWNER_FULL,
    }
    .sign_by_roots(&[root]);
    let device_cert = DeviceCert {
        version: FORMAT_VERSION,
        device_key,
        account,
        principal_kind: PrincipalKind::Human,
        scope,
        validity: wide(),
        issued_at_ns: ISSUED,
        issuer_key: int_pk,
    }
    .sign(intermediate);
    let grant = Grant {
        version: FORMAT_VERSION,
        subject: account,
        robot,
        scope,
        principal_kind: PrincipalKind::Human,
        delegation_depth: 0,
        validity: wide(),
        issued_at_ns: ISSUED,
        issuer: AccountId([2; 32]),
        issuer_key: int_pk,
    }
    .sign(intermediate);
    let pres = PairingPresentation {
        intermediate: int_cert,
        device_cert,
        grant,
        delegation: None,
    };
    store
        .verify_and_establish(&pres, &device_key, T_NOW, Some(name.into()))
        .expect("the crafted chain should verify + establish");
    device_key
}

/// Build a CLAIMED store + side-map with THREE accounts and return the pieces so
/// callers can revoke before constructing the authorizer:
/// - OWNER (OWNER_FULL — all caps, role OWNER, via physical-possession claim),
/// - VIEWER (role VIEWER + CAP_OBSERVE, via the chain),
/// - OPERATOR (role OPERATOR + **NO caps** — the fixture that isolates the
///   capability conjunct from the role gate: its role passes the operator gate
///   but it lacks CAP_TELEOP / CAP_OBSERVE).
fn build_claimed() -> (TrustStore, DeviceAccountIndex, Fixtures) {
    let root = sk(1);
    let intermediate = sk(2);
    let robot = RobotId([5; 32]);
    let robot_transport_key = PublicKey([6; 32]);
    let owner_device_key = PublicKey([11; 32]); // owner's transport key (bound in the side-map)

    let root_set = RootSet::new(vec![pk(&root)], 1).unwrap();
    let mut store =
        TrustStore::provision(robot, robot_transport_key, root_set, CHASSIS, T_NOW).unwrap();
    store
        .claim(OWNER_ACCOUNT, CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();

    let viewer_device_key = establish_via_chain(
        &mut store,
        &root,
        &intermediate,
        robot,
        VIEWER_ACCOUNT,
        &sk(21),
        Scope {
            role: Role::VIEWER,
            caps: Scope::CAP_OBSERVE,
        },
        "viewer",
    );
    // Role OPERATOR but ZERO caps: the role passes the operator gate while every
    // capability conjunct is absent (isolates caps_ok / CAP_OBSERVE from role).
    let operator_device_key = establish_via_chain(
        &mut store,
        &root,
        &intermediate,
        robot,
        OPERATOR_ACCOUNT,
        &sk(31),
        Scope {
            role: Role::OPERATOR,
            caps: 0,
        },
        "operator-no-caps",
    );

    let mut index = DeviceAccountIndex::new();
    index.bind(owner_device_key, OWNER_ACCOUNT);
    index.bind(viewer_device_key, VIEWER_ACCOUNT);
    index.bind(operator_device_key, OPERATOR_ACCOUNT);

    let fixtures = Fixtures {
        owner_device_key,
        viewer_device_key,
        operator_device_key,
        unpaired_device_key: PublicKey([200; 32]),
    };
    (store, index, fixtures)
}

/// A claimed robot with owner + viewer + operator-no-caps accounts.
fn claimed_authorizer() -> (PairingAuthorizer, Fixtures) {
    let (store, index, f) = build_claimed();
    (PairingAuthorizer::new(store, index), f)
}

/// The same claimed robot, but the VIEWER account has been owner-revoked (its
/// row is present but `revoked` — distinct from an UNPAIRED key with no row).
fn claimed_authorizer_viewer_revoked() -> (PairingAuthorizer, Fixtures) {
    let (mut store, index, f) = build_claimed();
    store
        .owner_revoke(&OWNER_ACCOUNT, &VIEWER_ACCOUNT)
        .expect("owner may revoke the viewer");
    (PairingAuthorizer::new(store, index), f)
}

/// The same claimed robot, but the VIEWER's DEVICE key is revoked by a synced epoch
/// (A6) — its ACCOUNT is untouched, so a sibling device of the same account
/// would still be allowed. `build_claimed`'s store is anchored at root sk(1) /
/// intermediate sk(2) / robot [5;32], so the epoch is signed by the same intermediate.
fn claimed_authorizer_viewer_device_revoked() -> (PairingAuthorizer, Fixtures) {
    let (mut store, index, f) = build_claimed();
    let root = sk(1);
    let intermediate = sk(2);
    let robot = RobotId([5; 32]);
    let int_cert = IntermediateCert {
        version: FORMAT_VERSION,
        intermediate_key: pk(&intermediate),
        validity: wide(),
        issued_at_ns: ISSUED,
        max_scope: Scope::OWNER_FULL,
    }
    .sign_by_roots(&[&root]);
    let epoch = AccessListEpoch {
        version: FORMAT_VERSION,
        robot,
        epoch: 2,
        revoked_accounts: vec![],
        revoked_devices: vec![f.viewer_device_key],
        issued_at_ns: ISSUED,
        issuer_key: pk(&intermediate),
    }
    .sign(&intermediate);
    store
        .apply_epoch(&epoch, &int_cert, T_NOW)
        .expect("the epoch revoking the viewer's device applies");
    (PairingAuthorizer::new(store, index), f)
}

/// A provisioned-but-UNCLAIMED robot with an empty access list + empty side-map.
fn unclaimed_authorizer() -> PairingAuthorizer {
    let root = sk(1);
    let root_set = RootSet::new(vec![pk(&root)], 1).unwrap();
    let store = TrustStore::provision(
        RobotId([5; 32]),
        PublicKey([6; 32]),
        root_set,
        CHASSIS,
        T_NOW,
    )
    .unwrap();
    PairingAuthorizer::new(store, DeviceAccountIndex::new())
}

struct Fixtures {
    owner_device_key: PublicKey,
    viewer_device_key: PublicKey,
    /// Role OPERATOR but NO caps — isolates the capability conjunct from the role.
    operator_device_key: PublicKey,
    unpaired_device_key: PublicKey,
}

fn assert_allow(d: AuthzDecision, ctx: &str) {
    assert_eq!(d, AuthzDecision::Allow, "expected Allow: {ctx}");
}
fn assert_deny(d: AuthzDecision, needle: &str, ctx: &str) {
    match d {
        AuthzDecision::Deny { reason } => assert!(
            reason.contains(needle),
            "{ctx}: deny reason {reason:?} should contain {needle:?}"
        ),
        AuthzDecision::Allow => panic!("{ctx}: expected Deny, got Allow"),
    }
}

// ── The matrix ───────────────────────────────────────────────────────────────

#[test]
fn paired_with_cap_is_allowed() {
    let (authz, f) = claimed_authorizer();
    // Owner (OWNER_FULL) has CAP_OBSERVE + owner role → observe + operator ops.
    let owner = caller_for(&f.owner_device_key);
    assert_allow(
        authz.authorize_verb(&owner, verbs::INVENTORY),
        "owner inventory",
    );
    assert_allow(
        authz.authorize_verb(&owner, verbs::LOG_TAIL),
        "owner log-tail",
    );
    assert_allow(
        authz.authorize_verb(&owner, verbs::RESTART),
        "owner restart",
    );
    assert_allow(authz.authorize_verb(&owner, verbs::TELEOP), "owner teleop");
    // Viewer has CAP_OBSERVE + viewer role → observe-class verbs.
    let viewer = caller_for(&f.viewer_device_key);
    assert_allow(
        authz.authorize_verb(&viewer, verbs::INVENTORY),
        "viewer inventory",
    );
    assert_allow(
        authz.authorize_verb(&viewer, verbs::LOG_TAIL),
        "viewer log-tail",
    );
}

#[test]
fn paired_without_cap_is_denied() {
    let (authz, f) = claimed_authorizer();
    let viewer = caller_for(&f.viewer_device_key);
    // Viewer lacks CAP_TELEOP → teleop denied on caps.
    assert_deny(
        authz.authorize_verb(&viewer, verbs::TELEOP),
        "lacks the requirement",
        "viewer teleop",
    );
    // Viewer role (3) fails the operator-role gate (2) → restart/deploy denied.
    assert_deny(
        authz.authorize_verb(&viewer, verbs::RESTART),
        "role value <= 2",
        "viewer restart",
    );
    assert_deny(
        authz.authorize_verb(&viewer, verbs::DEPLOY),
        "lacks the requirement",
        "viewer deploy",
    );
}

#[test]
fn any_paired_account_may_engage_estop_regardless_of_scope() {
    let (authz, f) = claimed_authorizer();
    // Owner AND the observe-only viewer both engage the e-stop floor.
    assert_allow(
        authz.authorize_verb(&caller_for(&f.owner_device_key), verbs::ENGAGE_ESTOP),
        "owner estop",
    );
    assert_allow(
        authz.authorize_verb(&caller_for(&f.viewer_device_key), verbs::ENGAGE_ESTOP),
        "viewer (observe-only) estop",
    );
    // An UNPAIRED key is NOT on the floor.
    assert_deny(
        authz.authorize_verb(&caller_for(&f.unpaired_device_key), verbs::ENGAGE_ESTOP),
        "requires a PAIRED account",
        "unpaired estop",
    );
}

#[test]
fn unpaired_key_normal_verb_is_denied() {
    let (authz, f) = claimed_authorizer();
    let unpaired = caller_for(&f.unpaired_device_key);
    assert_deny(
        authz.authorize_verb(&unpaired, verbs::INVENTORY),
        "unpaired",
        "unpaired inventory",
    );
    assert_deny(
        authz.authorize_verb(&unpaired, verbs::RESTART),
        "unpaired",
        "unpaired restart",
    );
}

#[test]
fn unpaired_key_bootstrap_verb_is_allowed_on_a_claimed_robot() {
    let (authz, f) = claimed_authorizer();
    let unpaired = caller_for(&f.unpaired_device_key);
    // Bootstrap verbs self-gate in the handler; the authorizer admits them.
    assert_allow(
        authz.authorize_verb(&unpaired, verbs::PAIR),
        "unpaired pair",
    );
    assert_allow(
        authz.authorize_verb(&unpaired, verbs::CODE_PAIR_START),
        "unpaired code-pair-start",
    );
    assert_allow(
        authz.authorize_verb(&unpaired, verbs::CODE_PAIR_FINISH),
        "unpaired code-pair-finish",
    );
    // `claim` is bootstrap too (its handler rejects an already-claimed robot).
    assert_allow(
        authz.authorize_verb(&unpaired, verbs::CLAIM),
        "unpaired claim (claimed robot)",
    );
}

#[test]
fn unclaimed_robot_admits_only_claim() {
    let authz = unclaimed_authorizer();
    let key = caller_for(&PublicKey([123; 32])); // any authed key
                                                 // ONLY claim is admissible.
    assert_allow(authz.authorize_verb(&key, verbs::CLAIM), "unclaimed claim");
    // Everything else — bootstrap, normal, e-stop — is refused.
    assert_deny(
        authz.authorize_verb(&key, verbs::PAIR),
        "only the `claim` bootstrap verb is admissible",
        "unclaimed pair",
    );
    assert_deny(
        authz.authorize_verb(&key, verbs::INVENTORY),
        "only the `claim` bootstrap verb is admissible",
        "unclaimed inventory",
    );
    assert_deny(
        authz.authorize_verb(&key, verbs::ENGAGE_ESTOP),
        "only the `claim` bootstrap verb is admissible",
        "unclaimed estop",
    );
}

#[test]
fn unclassified_verb_is_denied_by_default() {
    let (authz, f) = claimed_authorizer();
    // Even the all-powerful owner cannot run an unclassified verb.
    assert_deny(
        authz.authorize_verb(&caller_for(&f.owner_device_key), "frobnicate"),
        "deny-by-default",
        "owner frobnicate",
    );
    assert_deny(
        authz.authorize_verb(&caller_for(&f.owner_device_key), "restart-now"),
        "deny-by-default",
        "owner restart-now (typo of restart)",
    );
}

#[test]
fn unauthenticated_caller_is_denied_even_for_bootstrap() {
    let (authz, _f) = claimed_authorizer();
    // The dev unix-socket transport yields an unauthenticated caller — the
    // remote plane admits only TLS-authed iroh peers.
    let dev = CallerIdentity::local_dev();
    assert_deny(
        authz.authorize_verb(&dev, verbs::PAIR),
        "not transport-authenticated",
        "unauthenticated pair",
    );
    assert_deny(
        authz.authorize_verb(&dev, verbs::INVENTORY),
        "not transport-authenticated",
        "unauthenticated inventory",
    );
}

// ── Revocation: a revoked account is NOT an unpaired account ─────────────────

#[test]
fn revoked_account_is_denied_normal_verbs_and_the_estop_floor() {
    // Contract decision (pinned): a REVOKED account has been explicitly cut off
    // (its row is present but `revoked`), so `is_allowed` returns None and it is
    // NOT on the access list. It is therefore denied normal verbs AND the e-stop
    // floor — a fired/removed operator must not retain even the ability to halt
    // the robot (an engage-estop DoS). This differs from an UNPAIRED key only in
    // the deny REASON ("revoked, removed, or ..." vs "unpaired"), never the outcome.
    let (authz, f) = claimed_authorizer_viewer_revoked();
    let viewer = caller_for(&f.viewer_device_key);

    // Normal observe-class verbs: denied with the revoked/removed reason.
    assert_deny(
        authz.authorize_verb(&viewer, verbs::INVENTORY),
        "revoked, removed, or",
        "revoked viewer inventory",
    );
    assert_deny(
        authz.authorize_verb(&viewer, verbs::LOG_TAIL),
        "revoked, removed, or",
        "revoked viewer log-tail",
    );
    // The e-stop floor: a revoked account is NOT on the floor.
    assert_deny(
        authz.authorize_verb(&viewer, verbs::ENGAGE_ESTOP),
        "revoked, removed, or",
        "revoked viewer estop",
    );

    // Control: the still-valid OWNER (same store) is unaffected by the viewer's
    // revocation — the revoke is targeted, not a global lockout.
    assert_allow(
        authz.authorize_verb(&caller_for(&f.owner_device_key), verbs::ENGAGE_ESTOP),
        "owner estop after viewer revoked",
    );
    assert_allow(
        authz.authorize_verb(&caller_for(&f.owner_device_key), verbs::INVENTORY),
        "owner inventory after viewer revoked",
    );
}

#[test]
fn epoch_revoked_device_is_denied_everywhere_but_the_accounts_other_devices_survive() {
    // A6: a synced epoch revoking the viewer's DEVICE key cuts THAT desk
    // across every plane — with the distinct device-revocation reason, not the
    // account "revoked, removed" reason — while leaving the OTHER accounts (and, by
    // construction, any other device of the same account) untouched.
    let (authz, f) = claimed_authorizer_viewer_device_revoked();
    let viewer = caller_for(&f.viewer_device_key);

    // Normal + e-stop verbs: denied with the DEVICE reason (A6), not the
    // account reason — the precise signal that a desk was cut, not the account.
    assert_deny(
        authz.authorize_verb(&viewer, verbs::INVENTORY),
        "this desk was cut",
        "device-revoked viewer inventory",
    );
    assert_deny(
        authz.authorize_verb(&viewer, verbs::ENGAGE_ESTOP),
        "this desk was cut",
        "device-revoked viewer estop",
    );

    // Wire plane: the accept gate REFUSES the device-revoked key (the sweep
    // re-checks this, so an established stream would be evicted).
    match authz.classify_accept(alpn::WIRE, &f.viewer_device_key.0) {
        AcceptDecision::Refuse { reason } => {
            assert!(reason.contains("this desk was cut"), "reason: {reason}");
        }
        other => panic!("expected Refuse (device revoked), got {other:?}"),
    }
    // Ops plane: routed to the bootstrap surface only (NOT the full ops surface).
    assert_eq!(
        authz.classify_accept(alpn::OPS, &f.viewer_device_key.0),
        AcceptDecision::OpsBootstrapOnly,
    );

    // Control: the OTHER accounts are unaffected — the device revocation is
    // surgical, not a global lockout (the multi-device selectivity).
    assert_allow(
        authz.authorize_verb(&caller_for(&f.owner_device_key), verbs::ENGAGE_ESTOP),
        "owner estop after viewer device revoked",
    );
    assert_eq!(
        authz.classify_accept(alpn::WIRE, &f.owner_device_key.0),
        AcceptDecision::WireAdmit,
        "the owner's device still wire-admits after the viewer's device is cut",
    );
    // The operator account (a distinct device) is also unaffected.
    assert_allow(
        authz.authorize_verb(&caller_for(&f.operator_device_key), verbs::RESTART),
        "operator restart after viewer device revoked",
    );
}

// ── Capability conjunct isolated from the role gate ──────────────────────────

#[test]
fn capability_conjunct_is_checked_independently_of_the_role_gate() {
    // The operator-no-caps account has role OPERATOR (passes the operator gate)
    // but ZERO caps. This isolates the `caps_ok` conjunct: teleop needs role
    // OPERATOR AND CAP_TELEOP — the role passes, the cap does not.
    let (authz, f) = claimed_authorizer();
    let operator = caller_for(&f.operator_device_key);

    // teleop: role OK, cap MISSING → denied ON THE CAP. Deleting the caps_ok
    // conjunct would ALLOW this (the mutation this test kills).
    assert_deny(
        authz.authorize_verb(&operator, verbs::TELEOP),
        "lacks the requirement",
        "operator-no-caps teleop (role passes, CAP_TELEOP missing)",
    );

    // Control that the ROLE gate is genuinely satisfied: restart/deploy require
    // role OPERATOR and NO caps → allowed. (Proves the teleop denial above is the
    // cap conjunct, not the role gate.)
    assert_allow(
        authz.authorize_verb(&operator, verbs::RESTART),
        "operator-no-caps restart (role gate passes, no cap required)",
    );
    assert_allow(
        authz.authorize_verb(&operator, verbs::DEPLOY),
        "operator-no-caps deploy (role gate passes, no cap required)",
    );
}

// ── Accept-time wire-plane routing (classify_accept) ─────────────────────────

#[test]
fn wire_plane_requires_cap_observe_independently_of_pairing() {
    let (authz, f) = claimed_authorizer();

    // A paired account WITH CAP_OBSERVE (viewer) → admitted to the wire plane.
    assert_eq!(
        authz.classify_accept(alpn::WIRE, &f.viewer_device_key.0),
        AcceptDecision::WireAdmit,
        "viewer (CAP_OBSERVE) should be wire-admitted"
    );

    // A paired account WITHOUT CAP_OBSERVE (operator-no-caps) → REFUSED. This
    // isolates the ~277 CAP_OBSERVE conjunct: the key IS paired + allowed, yet
    // the wire plane refuses it for lacking CAP_OBSERVE. Deleting that check
    // would WireAdmit this key (the mutation this test kills).
    match authz.classify_accept(alpn::WIRE, &f.operator_device_key.0) {
        AcceptDecision::Refuse { reason } => {
            assert!(reason.contains("CAP_OBSERVE"), "reason: {reason}");
        }
        other => panic!("expected Refuse (no CAP_OBSERVE), got {other:?}"),
    }

    // An unpaired key → refused (control).
    match authz.classify_accept(alpn::WIRE, &f.unpaired_device_key.0) {
        AcceptDecision::Refuse { reason } => {
            assert!(reason.contains("unpaired"), "reason: {reason}");
        }
        other => panic!("expected Refuse (unpaired), got {other:?}"),
    }
}

#[test]
fn ops_plane_routes_paired_to_admit_and_unpaired_to_bootstrap() {
    let (authz, f) = claimed_authorizer();
    // Paired+allowed → full ops surface.
    assert_eq!(
        authz.classify_accept(alpn::OPS, &f.viewer_device_key.0),
        AcceptDecision::OpsAdmit,
    );
    // Unpaired → bootstrap surface only (per-verb authz still runs downstream).
    assert_eq!(
        authz.classify_accept(alpn::OPS, &f.unpaired_device_key.0),
        AcceptDecision::OpsBootstrapOnly,
    );
    // A revoked account is NOT paired-and-allowed → bootstrap only on ops.
    let (revoked_authz, rf) = claimed_authorizer_viewer_revoked();
    assert_eq!(
        revoked_authz.classify_accept(alpn::OPS, &rf.viewer_device_key.0),
        AcceptDecision::OpsBootstrapOnly,
        "revoked account falls to bootstrap-only on ops"
    );
    // ...and is refused on the wire plane (revoked/removed).
    match revoked_authz.classify_accept(alpn::WIRE, &rf.viewer_device_key.0) {
        AcceptDecision::Refuse { reason } => {
            assert!(reason.contains("revoked, removed, or"), "reason: {reason}");
        }
        other => panic!("expected Refuse (revoked), got {other:?}"),
    }
}

#[test]
fn unknown_alpn_is_classified_as_unknown() {
    let (authz, f) = claimed_authorizer();
    match authz.classify_accept(b"cerulion/bogus/9", &f.owner_device_key.0) {
        AcceptDecision::UnknownAlpn { alpn } => assert_eq!(alpn, b"cerulion/bogus/9"),
        other => panic!("expected UnknownAlpn, got {other:?}"),
    }
}

#[test]
fn ops_verb_name_constants_mirror_cerud_handlers() {
    // Anti-drift: the classification constants MUST match cerud's real handler
    // names, else the authorizer classifies a verb cerud never dispatches.
    assert_eq!(verbs::INVENTORY, InventoryVerb.name());
    assert_eq!(verbs::RESTART, RestartVerb::new().name());
    assert_eq!(
        verbs::LOG_TAIL,
        LogTailVerb::new("/tmp/cerulion-test-logs").name()
    );
}
