// SPDX-License-Identifier: MIT OR Apache-2.0
//! Trust-store tests: MAC tamper-evidence, atomic save/load, anti-rollback
//! persistence, the unclaimed→claim→reset lifecycle, epoch monotonicity, and
//! owner revoke/readmit.

mod common;
use common::*;

use cerulion_pairing::format::*;
use cerulion_pairing::pake::{
    CeremonyConfig, CpaceConfirmed, CpaceInitiator, CpaceResponder, PakeIdentities,
};
use cerulion_pairing::verify::*;
use cerulion_pairing::PairingError;

/// The seed for the robot's OWN transport key. `store()` provisions with it, and
/// `code_confirm` runs its CPace ceremony against the SAME key (as the responder),
/// so a code-pairing witness binds THIS robot — required now that
/// `establish_code_pairing` rejects a cross-robot witness (`WitnessRobotMismatch`).
const ROBOT_TRANSPORT_SEED: u8 = 61;

/// Mint a REAL CPace-success witness for `account` against `store()`'s robot key
/// (runs a full ceremony — no fake data; the only real way to obtain the proof
/// `establish_code_pairing` requires).
fn code_confirm(account: AccountId) -> CpaceConfirmed {
    code_confirm_for_robot(account, pk(&sk(ROBOT_TRANSPORT_SEED)))
}

/// Mint a witness whose ceremony ran against an ARBITRARY robot transport key
/// (the responder key) — lets a test mint a witness for a DIFFERENT robot to
/// exercise the cross-robot replay guard.
fn code_confirm_for_robot(account: AccountId, robot_key: PublicKey) -> CpaceConfirmed {
    let ids = PakeIdentities::new(pk(&sk(60)), robot_key, b"store-test".to_vec());
    let mut resp = CpaceResponder::new("code", ids.clone(), CeremonyConfig::default(), 0);
    let init = CpaceInitiator::new("code", ids);
    let (attempt, msg1) = init.begin().unwrap();
    let r = resp.respond(&msg1, 0).unwrap();
    let (_ik, ic) = attempt.finish(&r.msg2, &r.responder_confirm).unwrap();
    resp.finish(&ic, 0, account, PrincipalKind::Human).unwrap()
}

/// Convenience: establish a code-paired row for `account` given a fresh witness.
fn code_pair(s: &mut TrustStore, account: AccountId, name: &str, now_ns: u64) {
    s.establish_code_pairing(
        &code_confirm(account),
        account,
        name.into(),
        Scope::CODE_PAIR_DEFAULT,
        now_ns,
    )
    .unwrap();
}

const ROBOT: u8 = 0xB0;
const OWNER: u8 = 0x01;
const GUEST: u8 = 0x02;

fn root_set() -> RootSet {
    RootSet::new(vec![pk(&sk(1)), pk(&sk(2))], 2).unwrap()
}
fn store() -> TrustStore {
    TrustStore::provision(
        rob(ROBOT),
        pk(&sk(ROBOT_TRANSPORT_SEED)),
        root_set(),
        CHASSIS,
        0,
    )
    .unwrap()
}
fn good_intermediate() -> SignedIntermediateCert {
    make_intermediate(pk(&sk(10)), wide_validity(), ISSUED).sign_by_roots(&[&sk(1), &sk(2)])
}

fn good_presentation() -> PairingPresentation {
    let device_cert = make_device_cert(
        pk(&sk(20)),
        acct(0x0A),
        pk(&sk(10)),
        op_scope(),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let grant = make_grant(
        acct(0x0A),
        rob(ROBOT),
        op_scope(),
        0,
        acct(0x0C),
        pk(&sk(10)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    PairingPresentation {
        intermediate: good_intermediate(),
        device_cert,
        grant,
        delegation: None,
    }
}

// -- MAC / persistence -------------------------------------------------------

#[test]
fn save_load_round_trips_all_state() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("trust.store");

    let mut s = store().with_path(&path);
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    code_pair(&mut s, acct(GUEST), "phone", T_NOW);
    s.save(MAC_KEY).unwrap();

    let loaded = TrustStore::load(&path, MAC_KEY).unwrap();
    assert_eq!(loaded.owner(), Some(acct(OWNER)));
    assert!(loaded.is_claimed());
    assert_eq!(loaded.access_rows().len(), 2);
    assert!(loaded.is_allowed(&acct(OWNER), T_NOW).is_some());
    assert!(loaded.is_allowed(&acct(GUEST), T_NOW).is_some());
    assert_eq!(loaded.high_water_ns(), s.high_water_ns());
    assert_eq!(loaded.robot_id(), rob(ROBOT));
}

#[test]
fn atomic_write_leaves_no_temp_file() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("trust.store");
    let s = store().with_path(&path);
    s.save(MAC_KEY).unwrap();
    assert!(path.exists());
    // The `<name>.tmp` sibling must have been renamed away.
    let tmp = dir.path().join("trust.store.tmp");
    assert!(!tmp.exists(), "atomic write should leave no .tmp behind");
}

#[test]
fn a_flipped_byte_is_detected_as_tamper() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("trust.store");
    store().with_path(&path).save(MAC_KEY).unwrap();

    let mut bytes = std::fs::read(&path).unwrap();
    let mid = bytes.len() / 2;
    bytes[mid] ^= 0xFF; // corrupt a body byte
    std::fs::write(&path, &bytes).unwrap();

    let err = TrustStore::load(&path, MAC_KEY).unwrap_err();
    assert!(matches!(err, PairingError::TamperDetected));
}

#[test]
fn wrong_mac_key_is_tamper() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("trust.store");
    store().with_path(&path).save(MAC_KEY).unwrap();
    let err = TrustStore::load(&path, b"a-completely-different-mac-key!!!").unwrap_err();
    assert!(matches!(err, PairingError::TamperDetected));
}

#[test]
fn truncated_file_is_corrupt() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("trust.store");
    std::fs::write(&path, b"too short").unwrap();
    let err = TrustStore::load(&path, MAC_KEY).unwrap_err();
    assert!(matches!(err, PairingError::StoreCorrupt(_)));
}

#[test]
fn save_without_path_errors() {
    let err = store().save(MAC_KEY).unwrap_err();
    assert!(matches!(err, PairingError::NoStorePath));
}

// -- ownership lifecycle -----------------------------------------------------

#[test]
fn ships_unclaimed() {
    let s = store();
    assert!(!s.is_claimed());
    assert_eq!(s.owner(), None);
    assert_eq!(s.ownership(), OwnershipState::Unclaimed);
}

#[test]
fn claim_requires_chassis_secret_and_writes_owner_row() {
    let mut s = store();
    // Wrong secret => no claim.
    assert!(matches!(
        s.claim(acct(OWNER), b"wrong", PrincipalKind::Human, T_NOW),
        Err(PairingError::WrongChassisSecret)
    ));
    assert!(!s.is_claimed());

    // Correct secret => claimed, owner row with full scope.
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    assert_eq!(s.owner(), Some(acct(OWNER)));
    let row = s.is_allowed(&acct(OWNER), T_NOW).unwrap();
    assert_eq!(row.scope, Scope::OWNER_FULL);
    assert_eq!(row.source, PairingSource::Claim);

    // A second claim is refused (single owner slot).
    assert!(matches!(
        s.claim(acct(0x09), CHASSIS, PrincipalKind::Human, T_NOW),
        Err(PairingError::AlreadyClaimed)
    ));
}

#[test]
fn factory_reset_returns_to_unclaimed_but_preserves_the_rollback_floor() {
    let mut s = store();
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    code_pair(&mut s, acct(GUEST), "phone", T_NOW);
    let epoch = make_epoch(rob(ROBOT), 5, vec![], pk(&sk(10)), ISSUED).sign(&sk(10));
    s.apply_epoch(&epoch, &good_intermediate(), T_NOW).unwrap();
    let floor = s.high_water_ns();
    assert!(floor >= T_NOW);

    // Wrong secret cannot reset.
    assert!(matches!(
        s.factory_reset(b"nope"),
        Err(PairingError::WrongChassisSecret)
    ));

    // Correct secret resets ownership, access list, and epoch — but NOT the
    // monotonic anti-rollback floor.
    s.factory_reset(CHASSIS).unwrap();
    assert!(!s.is_claimed());
    assert_eq!(s.access_rows().len(), 0);
    assert_eq!(s.current_epoch(), 0);
    assert_eq!(
        s.high_water_ns(),
        floor,
        "rollback floor must persist across reset"
    );
    // And it can be re-claimed afresh.
    s.claim(acct(0x09), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    assert_eq!(s.owner(), Some(acct(0x09)));
}

// -- epoch monotonicity ------------------------------------------------------

#[test]
fn epochs_apply_monotonically_and_replace_the_revocation_set() {
    let mut s = store();
    // A code pairing requires a claimed robot (RobotUnclaimed otherwise).
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    code_pair(&mut s, acct(GUEST), "phone", T_NOW);
    assert!(s.is_allowed(&acct(GUEST), T_NOW).is_some());

    // Epoch 2 revokes GUEST.
    let e2 = make_epoch(rob(ROBOT), 2, vec![acct(GUEST)], pk(&sk(10)), ISSUED).sign(&sk(10));
    assert_eq!(
        s.apply_epoch(&e2, &good_intermediate(), T_NOW).unwrap(),
        EpochOutcome::Applied { newly_revoked: 1 }
    );
    assert!(s.is_allowed(&acct(GUEST), T_NOW).is_none());
    assert_eq!(s.current_epoch(), 2);

    // Re-applying epoch 2 (or an older one) is a no-op.
    assert_eq!(
        s.apply_epoch(&e2, &good_intermediate(), T_NOW).unwrap(),
        EpochOutcome::NotNewer { current: 2 }
    );
    let e1 = make_epoch(rob(ROBOT), 1, vec![], pk(&sk(10)), ISSUED).sign(&sk(10));
    assert_eq!(
        s.apply_epoch(&e1, &good_intermediate(), T_NOW).unwrap(),
        EpochOutcome::NotNewer { current: 2 }
    );
    // The ACCESS DECISION must not regress: a stale epoch (whose empty set would,
    // if applied, un-revoke GUEST) is a true no-op — GUEST stays denied and the
    // epoch number is unchanged.
    assert!(
        s.is_allowed(&acct(GUEST), T_NOW).is_none(),
        "a stale epoch must not restore revoked access"
    );
    assert_eq!(s.current_epoch(), 2);

    // Epoch 3 with an empty revocation set restores access (authoritative set).
    let e3 = make_epoch(rob(ROBOT), 3, vec![], pk(&sk(10)), ISSUED).sign(&sk(10));
    assert_eq!(
        s.apply_epoch(&e3, &good_intermediate(), T_NOW).unwrap(),
        EpochOutcome::Applied { newly_revoked: 0 }
    );
    assert!(s.is_allowed(&acct(GUEST), T_NOW).is_some());
    assert_eq!(s.current_epoch(), 3);
}

// -- per-device revocation (multi-device + tombstone) ------------------------

/// A strong-path presentation for `device_seed`'s key bound to `account` — the
/// sibling-device generator for the multi-device tests (`good_presentation` is the
/// `sk(20)`/`acct(0x0A)` special case).
fn presentation_for(device_seed: u8, account: AccountId) -> PairingPresentation {
    let device_cert = make_device_cert(
        pk(&sk(device_seed)),
        account,
        pk(&sk(10)),
        op_scope(),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let grant = make_grant(
        account,
        rob(ROBOT),
        op_scope(),
        0,
        acct(0x0C),
        pk(&sk(10)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    PairingPresentation {
        intermediate: good_intermediate(),
        device_cert,
        grant,
        delegation: None,
    }
}

#[test]
fn epoch_revokes_a_device_key_and_apply_replaces_it_monotonically() {
    let mut s = store();
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    let dev = pk(&sk(20));
    assert!(!s.is_device_revoked(&dev));
    assert!(s.revoked_devices().is_empty());

    // Epoch 2 revokes the device (empty account set → account-scoped
    // `newly_revoked` stays 0, since devices are not access-list rows).
    let e2 = make_epoch_with_devices(rob(ROBOT), 2, vec![], vec![dev], pk(&sk(10)), ISSUED)
        .sign(&sk(10));
    assert_eq!(
        s.apply_epoch(&e2, &good_intermediate(), T_NOW).unwrap(),
        EpochOutcome::Applied { newly_revoked: 0 }
    );
    assert!(s.is_device_revoked(&dev));
    assert_eq!(s.revoked_devices(), &[dev]);

    // Epoch 3 with an EMPTY device set re-admits it (authoritative replace — the
    // monotonic set, not an accumulate).
    let e3 =
        make_epoch_with_devices(rob(ROBOT), 3, vec![], vec![], pk(&sk(10)), ISSUED).sign(&sk(10));
    s.apply_epoch(&e3, &good_intermediate(), T_NOW).unwrap();
    assert!(!s.is_device_revoked(&dev));
    assert!(s.revoked_devices().is_empty());
}

#[test]
fn a_revoked_device_is_refused_at_verify_but_a_sibling_device_of_the_same_account_verifies() {
    // THE multi-device pin: revoke ONE device without touching the account's other
    // devices. Desk A + desk B share one account; revoking desk A's KEY cuts desk A
    // (RevokedDeviceByEpoch — NOT RevokedByEpoch) while desk B keeps its access.
    let mut s = store();
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    let account = acct(0x0A);
    let dev_a = pk(&sk(20)); // desk A
    let dev_b = pk(&sk(21)); // desk B (SAME account, different key)

    // Anti-tautology control: BOTH desks pair cleanly before revocation.
    s.verify_new_pairing(&presentation_for(20, account), &dev_a, T_NOW)
        .expect("desk A pairs before revocation");
    s.verify_new_pairing(&presentation_for(21, account), &dev_b, T_NOW)
        .expect("desk B pairs before revocation");

    // Epoch revokes ONLY desk A's device key; the account is NOT in revoked_accounts.
    let e = make_epoch_with_devices(rob(ROBOT), 2, vec![], vec![dev_a], pk(&sk(10)), ISSUED)
        .sign(&sk(10));
    s.apply_epoch(&e, &good_intermediate(), T_NOW).unwrap();

    // Desk A is refused as a DEVICE revocation (a distinct, accurate signal).
    assert!(
        matches!(
            s.verify_new_pairing(&presentation_for(20, account), &dev_a, T_NOW),
            Err(PairingError::RevokedDeviceByEpoch { epoch: 2 })
        ),
        "desk A's device key is epoch-revoked"
    );

    // Desk B — a DIFFERENT device key of the SAME account — still verifies: the
    // account and its other devices are untouched.
    s.verify_new_pairing(&presentation_for(21, account), &dev_b, T_NOW)
        .expect("a sibling device of the same account keeps its access after desk A is cut");
}

#[test]
fn a_revoked_device_cannot_repair_to_escape_the_tombstone() {
    // The tombstone: after a device is epoch-revoked, RE-PRESENTING its still-valid
    // cert + grant does NOT resurrect it — only a newer epoch dropping it re-admits.
    let mut s = store();
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    let account = acct(0x0A);
    let dev = pk(&sk(20));
    let e = make_epoch_with_devices(rob(ROBOT), 2, vec![], vec![dev], pk(&sk(10)), ISSUED)
        .sign(&sk(10));
    s.apply_epoch(&e, &good_intermediate(), T_NOW).unwrap();

    // The presentation is byte-for-byte the same valid cert chain each time; it is
    // refused on every re-presentation (no fresh witness is minted).
    for _ in 0..3 {
        assert!(matches!(
            s.verify_and_establish(&presentation_for(20, account), &dev, T_NOW, None),
            Err(PairingError::RevokedDeviceByEpoch { epoch: 2 })
        ));
    }
    // No row was written for the account by the refused attempts.
    assert!(s.is_allowed(&account, T_NOW).is_none());

    // Only a NEWER epoch dropping the device re-admits it — then it pairs.
    let e3 =
        make_epoch_with_devices(rob(ROBOT), 3, vec![], vec![], pk(&sk(10)), ISSUED).sign(&sk(10));
    s.apply_epoch(&e3, &good_intermediate(), T_NOW).unwrap();
    s.verify_and_establish(&presentation_for(20, account), &dev, T_NOW, None)
        .expect("a newer epoch dropping the device re-admits it (the only un-revoke)");
    assert!(s.is_allowed(&account, T_NOW).is_some());
}

#[test]
fn device_revocation_survives_save_load() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("trust.store");
    let mut s = store().with_path(&path);
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    let dev = pk(&sk(20));
    let e = make_epoch_with_devices(rob(ROBOT), 2, vec![], vec![dev], pk(&sk(10)), ISSUED)
        .sign(&sk(10));
    s.apply_epoch(&e, &good_intermediate(), T_NOW).unwrap();
    s.save(MAC_KEY).unwrap();

    let loaded = TrustStore::load(&path, MAC_KEY).unwrap();
    assert!(loaded.is_device_revoked(&dev));
    assert_eq!(loaded.revoked_devices(), &[dev]);
    assert_eq!(loaded.current_epoch(), 2);
}

#[test]
fn factory_reset_clears_the_revoked_device_set() {
    let mut s = store();
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    let dev = pk(&sk(20));
    let e = make_epoch_with_devices(rob(ROBOT), 2, vec![], vec![dev], pk(&sk(10)), ISSUED)
        .sign(&sk(10));
    s.apply_epoch(&e, &good_intermediate(), T_NOW).unwrap();
    assert!(s.is_device_revoked(&dev));

    s.factory_reset(CHASSIS).unwrap();
    assert!(
        !s.is_device_revoked(&dev),
        "a new owner starts from a blank device-revocation slate"
    );
    assert!(s.revoked_devices().is_empty());
}

#[test]
fn an_epoch_revoking_the_owner_applies_as_is_no_robot_side_carveout() {
    // A6: the robot does NOT special-case its own owner in a synced epoch — it
    // applies the (authoritative) epoch AS-IS (a robot-side enforcement carve-out could
    // silently break a legitimate future owner change; the self-lockout guard belongs at
    // the account-service MINT, with the robot only WARNING — observability, not
    // enforcement). So an epoch naming the owner DOES lock the owner out (which is why
    // the mint refuses to produce one).
    let mut s = store();
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    assert!(s.is_allowed(&acct(OWNER), T_NOW).is_some());
    let e = make_epoch(rob(ROBOT), 2, vec![acct(OWNER)], pk(&sk(10)), ISSUED).sign(&sk(10));
    assert_eq!(
        s.apply_epoch(&e, &good_intermediate(), T_NOW).unwrap(),
        EpochOutcome::Applied { newly_revoked: 1 }
    );
    assert!(
        s.is_allowed(&acct(OWNER), T_NOW).is_none(),
        "the epoch applies as-is; there is no robot-side owner carve-out"
    );
}

#[test]
fn epoch_for_a_different_robot_is_rejected() {
    let mut s = store();
    let e = make_epoch(rob(0xFE), 1, vec![], pk(&sk(10)), ISSUED).sign(&sk(10));
    assert!(matches!(
        s.apply_epoch(&e, &good_intermediate(), T_NOW),
        Err(PairingError::WrongRobot)
    ));
}

#[test]
fn epoch_signed_by_non_intermediate_is_rejected() {
    let mut s = store();
    // issuer_key claims the intermediate but the epoch is signed by sk(11).
    let e = make_epoch(rob(ROBOT), 1, vec![], pk(&sk(10)), ISSUED).sign(&sk(11));
    assert!(matches!(
        s.apply_epoch(&e, &good_intermediate(), T_NOW),
        Err(PairingError::BadSignature(_))
    ));
}

#[test]
fn epoch_with_undertrusted_intermediate_is_rejected() {
    let mut s = store();
    // Intermediate signed by only one root (threshold is 2).
    let weak = make_intermediate(pk(&sk(10)), wide_validity(), ISSUED).sign_by_roots(&[&sk(1)]);
    let e = make_epoch(rob(ROBOT), 1, vec![], pk(&sk(10)), ISSUED).sign(&sk(10));
    assert!(matches!(
        s.apply_epoch(&e, &weak, T_NOW),
        Err(PairingError::RootThresholdNotMet { .. })
    ));
}

// -- owner revoke / readmit --------------------------------------------------

#[test]
fn owner_revoke_and_readmit() {
    let mut s = store();
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    code_pair(&mut s, acct(GUEST), "phone", T_NOW);
    assert!(s.is_allowed(&acct(GUEST), T_NOW).is_some());

    // Revoke-now by the owner.
    s.owner_revoke(&acct(OWNER), &acct(GUEST)).unwrap();
    assert!(s.is_allowed(&acct(GUEST), T_NOW).is_none());

    // A non-owner cannot revoke.
    assert!(matches!(
        s.owner_revoke(&acct(GUEST), &acct(OWNER)),
        Err(PairingError::NotOwner)
    ));

    // The owner cannot lock itself out.
    s.owner_revoke(&acct(OWNER), &acct(OWNER)).unwrap();
    assert!(s.is_allowed(&acct(OWNER), T_NOW).is_some());

    // Readmit restores access.
    s.owner_readmit(&acct(OWNER), &acct(GUEST)).unwrap();
    assert!(s.is_allowed(&acct(GUEST), T_NOW).is_some());
}

#[test]
fn revoke_on_unclaimed_store_errors() {
    let mut s = store();
    assert!(matches!(
        s.owner_revoke(&acct(OWNER), &acct(GUEST)),
        Err(PairingError::Unclaimed)
    ));
}

#[test]
fn owner_revoked_leaf_is_refused_at_verify_and_owner_readmit_restores_pairing() {
    // A sticky owner "revoke-now" on the LEAF subject gates a
    // NEW pairing at `verify_new_pairing` — the owner-revoked leaf is REFUSED (it
    // does NOT verify), symmetric with the delegator gate, so a re-presented grant
    // cannot mint a fresh witness. The error is the DISTINCT `RevokedByOwner`
    // (not `RevokedByEpoch`), and `owner_readmit` restores pairability.
    let mut s = TrustStore::provision(
        rob(ROBOT),
        pk(&sk(ROBOT_TRANSPORT_SEED)),
        root_set(),
        CHASSIS,
        0,
    )
    .unwrap();
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();

    let dev = make_device_cert(
        pk(&sk(20)),
        acct(GUEST),
        pk(&sk(10)),
        op_scope(),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let grant = make_grant(
        acct(GUEST),
        rob(ROBOT),
        op_scope(),
        0,
        acct(0x0C),
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

    let v = s.verify_new_pairing(&pres, &pk(&sk(20)), T_NOW).unwrap();
    s.establish_pairing(&v, PairingSource::StrongChain, None, T_NOW, None)
        .unwrap();
    s.owner_revoke(&acct(OWNER), &acct(GUEST)).unwrap();
    assert!(s.is_allowed(&acct(GUEST), T_NOW).is_none());
    // Anti-tautology: GUEST is NOT in any epoch set (epoch 0, empty), so the
    // refusal below is genuinely the sticky owner-revoke path, not the epoch path.
    assert_eq!(s.current_epoch(), 0);

    // Re-present the still-valid grant: verification now FAILS for the
    // owner-revoked leaf with the distinct `RevokedByOwner` error (NOT
    // `RevokedByEpoch`) — it never reaches `establish_pairing`.
    let err = s
        .verify_new_pairing(&pres, &pk(&sk(20)), T_NOW)
        .unwrap_err();
    assert!(
        matches!(err, PairingError::RevokedByOwner),
        "an owner-revoked leaf must be refused at verify with RevokedByOwner, got {err:?}"
    );
    // The row also stays revoked (belt-and-suspenders — `is_allowed` still denies).
    assert!(s.is_allowed(&acct(GUEST), T_NOW).is_none());

    // owner_readmit restores pairability: verification succeeds again and the
    // re-established row is allowed.
    s.owner_readmit(&acct(OWNER), &acct(GUEST)).unwrap();
    let v2 = s
        .verify_new_pairing(&pres, &pk(&sk(20)), T_NOW)
        .expect("a re-admitted leaf verifies again");
    s.establish_pairing(&v2, PairingSource::StrongChain, None, T_NOW, None)
        .unwrap();
    assert!(
        s.is_allowed(&acct(GUEST), T_NOW).is_some(),
        "owner_readmit followed by re-pairing restores access"
    );
}

// ---- Code pairing must not readmit an owner-revoked account -----------------

#[test]
fn code_pairing_cannot_readmit_an_owner_revoked_account() {
    let mut s = store();
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    code_pair(&mut s, acct(GUEST), "phone", T_NOW);
    s.owner_revoke(&acct(OWNER), &acct(GUEST)).unwrap();
    assert!(s.is_allowed(&acct(GUEST), T_NOW).is_none());

    // Re-code-pair the SAME account id — must STAY denied (sticky revocation).
    code_pair(&mut s, acct(GUEST), "phone-again", T_NOW);
    assert!(
        s.is_allowed(&acct(GUEST), T_NOW).is_none(),
        "a code pairing must not readmit an owner-revoked account"
    );
    // Only an explicit owner_readmit restores access.
    s.owner_readmit(&acct(OWNER), &acct(GUEST)).unwrap();
    assert!(s.is_allowed(&acct(GUEST), T_NOW).is_some());
}

// ---- Code-pairing requires an unforgeable ceremony-success witness ----------

#[test]
fn code_pairing_persists_a_row_given_a_valid_witness() {
    // A code-paired row persists ONLY given a valid CpaceConfirmed.
    let mut s = store();
    // A code pairing requires a claimed robot (RobotUnclaimed otherwise).
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    let confirmed = code_confirm(acct(GUEST));
    s.establish_code_pairing(
        &confirmed,
        acct(GUEST),
        "phone".into(),
        Scope::CODE_PAIR_DEFAULT,
        T_NOW,
    )
    .unwrap();
    let row = s.is_allowed(&acct(GUEST), T_NOW).expect("code-paired");
    assert_eq!(row.source, PairingSource::CodePaired);
    // The principal kind comes from the witness (not a caller-chosen argument).
    assert_eq!(row.principal_kind, PrincipalKind::Human);
}

#[test]
fn a_witness_for_one_account_cannot_establish_another() {
    // A confirmation for GUEST cannot be replayed to persist OWNER.
    let mut s = store();
    // Claim with a NEUTRAL owner (0x03) so neither OWNER (0x01) nor GUEST (0x02)
    // gets a row from the claim — the "nothing persisted for either account"
    // assertion below stays about the establish attempt, not the claim.
    s.claim(acct(0x03), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    let confirmed_for_guest = code_confirm(acct(GUEST));
    let err = s
        .establish_code_pairing(
            &confirmed_for_guest,
            acct(OWNER), // mismatched account
            "sneaky".into(),
            Scope::CODE_PAIR_DEFAULT,
            T_NOW,
        )
        .unwrap_err();
    assert!(matches!(err, PairingError::WitnessAccountMismatch));
    // Nothing was persisted for either account.
    assert!(s.is_allowed(&acct(OWNER), T_NOW).is_none());
    assert!(s.is_allowed(&acct(GUEST), T_NOW).is_none());
}

// The impossibility of forging a CpaceConfirmed outside the ceremony
// is pinned at compile time by the `compile_fail` doctest on
// `cerulion_pairing::pake::CpaceConfirmed`.

// ---- code-pairing witness bound to THIS robot's transport identity ----------

/// A DIFFERENT robot's transport-key seed (`store()` provisions with
/// `ROBOT_TRANSPORT_SEED`).
const OTHER_ROBOT_TRANSPORT_SEED: u8 = 62;

#[test]
fn code_pairing_rejects_a_cross_robot_witness() {
    // A `CpaceConfirmed` minted by a ceremony against a DIFFERENT robot's transport
    // identity (R2) cannot be replayed to establish a row on THIS robot (R1) — the
    // cross-robot witness replay guard, symmetric to the same-path account guard.
    let mut s = store(); // provisioned with robot key pk(&sk(ROBOT_TRANSPORT_SEED))
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();

    // Mint a witness whose ceremony ran against a FOREIGN robot key. Anti-tautology:
    // its account MATCHES the account being established, so the refusal is genuinely
    // the ROBOT binding, not `WitnessAccountMismatch`.
    let foreign = code_confirm_for_robot(acct(GUEST), pk(&sk(OTHER_ROBOT_TRANSPORT_SEED)));
    assert_eq!(foreign.account(), acct(GUEST));
    assert_ne!(foreign.responder_key(), s.robot_transport_key());

    let err = s
        .establish_code_pairing(
            &foreign,
            acct(GUEST),
            "impostor".into(),
            Scope::CODE_PAIR_DEFAULT,
            T_NOW,
        )
        .unwrap_err();
    assert!(
        matches!(err, PairingError::WitnessRobotMismatch),
        "a witness for a different robot must be refused, got {err:?}"
    );
    assert!(
        s.is_allowed(&acct(GUEST), T_NOW).is_none(),
        "a cross-robot witness must persist no access row"
    );
}

#[test]
fn code_pairing_accepts_a_witness_bound_to_this_robot() {
    // The control for the cross-robot guard, pinning the key EQUALITY that gates
    // it: a witness minted against THIS robot's own transport key (== the key
    // `store()` provisioned) establishes the row.
    let mut s = store();
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    let confirmed = code_confirm(acct(GUEST)); // responder == store()'s robot key
    assert_eq!(
        confirmed.responder_key(),
        s.robot_transport_key(),
        "the happy-path witness binds this robot's transport key"
    );
    s.establish_code_pairing(
        &confirmed,
        acct(GUEST),
        "phone".into(),
        Scope::CODE_PAIR_DEFAULT,
        T_NOW,
    )
    .unwrap();
    assert!(s.is_allowed(&acct(GUEST), T_NOW).is_some());
}

#[test]
fn robot_transport_key_round_trips_and_is_mac_covered() {
    // The new field survives an encode/decode round-trip AND lives inside the MAC'd
    // body (postcard(TrustStoreInner)), so corrupting it is tamper — the property
    // the cross-robot guard relies on to keep the recorded robot key authentic.
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("trust.store");
    let key = pk(&sk(ROBOT_TRANSPORT_SEED));

    let s = store().with_path(&path);
    assert_eq!(s.robot_transport_key(), key);
    s.save(MAC_KEY).unwrap();

    let loaded = TrustStore::load(&path, MAC_KEY).unwrap();
    assert_eq!(
        loaded.robot_transport_key(),
        key,
        "the robot transport key must round-trip through save/load"
    );

    // Corrupting a body byte (the field lives in the MAC'd body) is detected.
    let mut bytes = std::fs::read(&path).unwrap();
    let mid = bytes.len() / 2;
    bytes[mid] ^= 0xFF;
    std::fs::write(&path, &bytes).unwrap();
    let err = TrustStore::load(&path, MAC_KEY).unwrap_err();
    assert!(matches!(err, PairingError::TamperDetected));
}

// ---- A: no access rows on an UNCLAIMED robot ------------------------

#[test]
fn code_pairing_on_an_unclaimed_store_is_refused() {
    // The FIRST pairing must prove physical possession via claim(); an unclaimed
    // robot must carry zero access rows, so even a valid CPace witness cannot
    // persist a code-paired row before the owner claims.
    let mut s = store();
    assert!(!s.is_claimed());
    let err = s
        .establish_code_pairing(
            &code_confirm(acct(GUEST)),
            acct(GUEST),
            "backdoor".into(),
            Scope::CODE_PAIR_DEFAULT,
            T_NOW,
        )
        .unwrap_err();
    assert!(matches!(err, PairingError::RobotUnclaimed), "got {err:?}");
    assert!(
        s.access_rows().is_empty(),
        "an unclaimed robot must carry zero access rows"
    );
}

#[test]
fn strong_establish_on_an_unclaimed_store_is_refused() {
    // A `VerifiedPairing` is `Copy` and could be minted while claimed, then
    // replayed after a `factory_reset` returned the robot to unclaimed. The
    // independent unclaimed gate on `establish_pairing` blocks that stale-witness
    // vector (BOTH persistence seams gate, not just verify).
    let mut s = store();
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    // Mint a real witness while claimed.
    let v = s
        .verify_new_pairing(&good_presentation(), &pk(&sk(20)), T_NOW)
        .unwrap();
    // Reset returns the robot to unclaimed (and clears rows).
    s.factory_reset(CHASSIS).unwrap();
    assert!(!s.is_claimed());
    // Replaying the stale witness must be refused; no row is written.
    let err = s
        .establish_pairing(&v, PairingSource::StrongChain, None, T_NOW, None)
        .unwrap_err();
    assert!(matches!(err, PairingError::RobotUnclaimed), "got {err:?}");
    assert!(
        s.access_rows().is_empty(),
        "a stale witness must not persist a row on an unclaimed robot"
    );
}

#[test]
fn verify_new_pairing_on_an_unclaimed_store_is_refused() {
    // Refuse at the EARLIEST seam. The witness is
    // never minted (and the anti-rollback floor never advanced) on an unclaimed
    // robot — consistent with the two persistence seams.
    let mut s = store();
    assert!(!s.is_claimed());
    let before = s.high_water_ns();
    let err = s
        .verify_new_pairing(&good_presentation(), &pk(&sk(20)), T_NOW)
        .unwrap_err();
    assert!(matches!(err, PairingError::RobotUnclaimed), "got {err:?}");
    assert_eq!(
        s.high_water_ns(),
        before,
        "a refused unclaimed verify must not advance the anti-rollback floor"
    );
}

#[test]
fn pre_claim_pairing_cannot_plant_a_backdoor_that_survives_the_claim() {
    // The attack: plant a code-paired guest row on an ownerless robot so the
    // legitimate owner's later claim() inherits an invisible, unrevocable
    // backdoor. The pre-claim pairing is refused, so no row exists at claim time;
    // the claimed access list contains ONLY what post-claim ceremonies added.
    let mut s = store();
    assert!(!s.is_claimed());

    // (1) The attacker's pre-claim code pairing is refused.
    let err = s
        .establish_code_pairing(
            &code_confirm(acct(GUEST)),
            acct(GUEST),
            "backdoor".into(),
            Scope::CODE_PAIR_DEFAULT,
            T_NOW,
        )
        .unwrap_err();
    assert!(matches!(err, PairingError::RobotUnclaimed));
    assert!(s.access_rows().is_empty(), "no pre-claim row was planted");

    // (2) The legitimate owner claims (physical possession).
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();

    // (3) The access list contains ONLY the owner row — no surviving backdoor.
    assert_eq!(
        s.access_rows().len(),
        1,
        "only the owner row exists post-claim"
    );
    assert_eq!(s.owner(), Some(acct(OWNER)));
    assert!(
        s.is_allowed(&acct(GUEST), T_NOW).is_none(),
        "no backdoor guest row survived the claim"
    );

    // (4) Post-claim ceremonies now legitimately add rows.
    code_pair(&mut s, acct(GUEST), "phone", T_NOW);
    assert!(s.is_allowed(&acct(GUEST), T_NOW).is_some());
}

// ---- B: epoch-vs-owner revocation errors name their cause ------------

#[test]
fn epoch_revoked_leaf_verify_reports_revoked_by_epoch_not_owner() {
    // The twin of `owner_revoked_leaf_is_refused_at_verify_and_owner_readmit_
    // restores_pairing`: an EPOCH-revoked leaf reports `RevokedByEpoch`, keeping
    // the operator signal accurate about WHICH revocation fired (the leaf here has
    // NO local row, so it cannot be the owner path — anti-tautology).
    let mut s = store();
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    // Revoke the good-presentation subject (0x0A) by epoch.
    let epoch = make_epoch(rob(ROBOT), 7, vec![acct(0x0A)], pk(&sk(10)), ISSUED).sign(&sk(10));
    s.apply_epoch(&epoch, &good_intermediate(), T_NOW).unwrap();
    let err = s
        .verify_new_pairing(&good_presentation(), &pk(&sk(20)), T_NOW)
        .unwrap_err();
    assert!(
        matches!(err, PairingError::RevokedByEpoch { epoch: 7 }),
        "an epoch-revoked leaf must report RevokedByEpoch, got {err:?}"
    );
}

// ---- factory_reset preserves the anti-rollback floor -----------------------

#[test]
fn factory_reset_preserves_high_water_ns() {
    // Mutation guard: adding `self.inner.high_water_ns = 0` to factory_reset
    // fails exactly this test.
    let mut s = store();
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    let floor = s.high_water_ns();
    assert!(floor >= T_NOW);
    s.factory_reset(CHASSIS).unwrap();
    assert_eq!(
        s.high_water_ns(),
        floor,
        "the anti-rollback floor must survive a factory reset (resale wipe)"
    );
}

#[test]
fn stale_revoked_credential_is_still_rejected_after_factory_reset() {
    // The high-water defends the credential-replay vector even after the reset
    // CLEARS revoked_accounts: a rolled-back clock cannot resurrect a credential
    // issued before the floor, epoch set or no epoch set.
    let mut s = store();
    // Advance the floor to T_NOW via the first physical-possession claim (claim
    // seeds the anti-rollback floor to now).
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    assert!(s.high_water_ns() >= T_NOW);

    s.factory_reset(CHASSIS).unwrap();
    assert_eq!(s.current_epoch(), 0); // revocation set cleared by the reset

    // A credential-acceptance attempt with a rolled-back clock (before the
    // surviving floor) is still rejected — the reset did not weaken the
    // anti-rollback defense. `apply_epoch` honors the floor and, unlike
    // `verify_new_pairing`, does not require a claimed robot, so it exercises the
    // rollback gate directly on the post-reset (unclaimed) store.
    let epoch = make_epoch(rob(ROBOT), 1, vec![], pk(&sk(10)), ISSUED).sign(&sk(10));
    let err = s
        .apply_epoch(&epoch, &good_intermediate(), T_NOW - 1)
        .unwrap_err();
    assert!(matches!(err, PairingError::RollbackDetected { .. }));
}

// ---- A revoked delegator blocks its delegatees -----------------------------

fn delegated_presentation() -> PairingPresentation {
    // Delegatee = A_CLIENT (device sk 20); delegator = A_DELEGATOR (device sk 30).
    let device_cert = make_device_cert(
        pk(&sk(20)),
        acct(0x0A),
        pk(&sk(10)),
        op_scope(),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let parent = make_grant(
        acct(0xDD),
        rob(ROBOT),
        Scope {
            role: Role::OPERATOR,
            caps: Scope::CAP_DELEGATE | Scope::CAP_TELEOP | Scope::CAP_OBSERVE,
        },
        0,
        acct(0x0C),
        pk(&sk(10)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let delegator_cert = make_device_cert(
        pk(&sk(30)),
        acct(0xDD),
        pk(&sk(10)),
        op_scope(),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let grant = make_grant(
        acct(0x0A),
        rob(ROBOT),
        op_scope(),
        1,
        acct(0xDD),
        pk(&sk(30)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(30));
    PairingPresentation {
        intermediate: good_intermediate(),
        device_cert,
        grant,
        delegation: Some(Delegation {
            parent,
            delegator_cert,
        }),
    }
}

#[test]
fn revoked_delegator_blocks_delegatee_new_pairing() {
    let mut s = store();
    // A new pairing requires a claimed robot; claim so the verify path is reached.
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    // Sanity: the delegated pairing verifies BEFORE the delegator is revoked.
    assert!(s
        .verify_new_pairing(&delegated_presentation(), &pk(&sk(20)), T_NOW)
        .is_ok());

    // Revoke the DELEGATOR (account 0xDD) — not the delegatee (0x0A) — via epoch.
    let epoch = make_epoch(rob(ROBOT), 1, vec![acct(0xDD)], pk(&sk(10)), ISSUED).sign(&sk(10));
    s.apply_epoch(&epoch, &good_intermediate(), T_NOW).unwrap();

    // The delegatee's NEW pairing is now refused, even though the delegatee
    // itself is not revoked.
    let err = s
        .verify_new_pairing(&delegated_presentation(), &pk(&sk(20)), T_NOW)
        .unwrap_err();
    assert!(matches!(err, PairingError::DelegatorRevoked));
}

#[test]
fn revoked_delegator_signing_device_blocks_delegatee_new_pairing() {
    // A6 SECURITY (strong-path asymmetry): a depth-1 delegated grant is signed
    // by the delegator's DEVICE key (pk(sk(30))). A stolen delegator laptop whose
    // device key is epoch-revoked — but whose ACCOUNT (0xDD) is NOT — must not keep
    // signing delegated grants that verify. Symmetric with the owner-signing-device
    // gate in `verify_owner_grant`.
    let mut s = store();
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    // Sanity: verifies before any revocation.
    assert!(s
        .verify_new_pairing(&delegated_presentation(), &pk(&sk(20)), T_NOW)
        .is_ok());

    // Revoke ONLY the delegator's signing DEVICE (pk(sk(30))) — the delegatee device
    // (sk 20) and the delegator ACCOUNT (0xDD) are untouched.
    let e = make_epoch_with_devices(
        rob(ROBOT),
        2,
        vec![],
        vec![pk(&sk(30))],
        pk(&sk(10)),
        ISSUED,
    )
    .sign(&sk(10));
    s.apply_epoch(&e, &good_intermediate(), T_NOW).unwrap();

    let err = s
        .verify_new_pairing(&delegated_presentation(), &pk(&sk(20)), T_NOW)
        .unwrap_err();
    assert!(
        matches!(err, PairingError::RevokedDeviceByEpoch { epoch: 2 }),
        "a delegated grant signed by a revoked delegator device must be refused, got {err:?}"
    );

    // Anti-tautology: revoking an UNRELATED device (not the delegator's, not the
    // delegatee's) does NOT block the pairing — the gate is device-specific, not a
    // blanket account penalty.
    let e3 = make_epoch_with_devices(
        rob(ROBOT),
        3,
        vec![],
        vec![pk(&sk(99))],
        pk(&sk(10)),
        ISSUED,
    )
    .sign(&sk(10));
    s.apply_epoch(&e3, &good_intermediate(), T_NOW).unwrap();
    assert!(
        s.verify_new_pairing(&delegated_presentation(), &pk(&sk(20)), T_NOW)
            .is_ok(),
        "revoking an unrelated device must not block the delegated pairing"
    );
}

#[test]
fn owner_revoked_delegator_blocks_delegatee_new_pairing() {
    // Symmetric to the epoch arm above: here the delegator is OWNER-revoked
    // LOCALLY (revoke-now on its access-list row), not via an epoch push. This
    // pins the owner-revoked branch of `is_account_revoked`. (Mutation check:
    // reverting `is_account_revoked` to only consult the epoch set makes this
    // delegatee pairing succeed → `unwrap_err()` panics → this test fails.)
    let mut s = store();
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();

    // Give the delegator (0xDD) an access-list row, then owner-revoke it locally.
    code_pair(&mut s, acct(0xDD), "delegator", T_NOW);
    s.owner_revoke(&acct(OWNER), &acct(0xDD)).unwrap();
    assert!(s.is_allowed(&acct(0xDD), T_NOW).is_none());
    // The delegator is NOT in the epoch revocation set — only its local row is
    // revoked (anti-tautology for the mutation check).
    assert_eq!(s.current_epoch(), 0);

    let err = s
        .verify_new_pairing(&delegated_presentation(), &pk(&sk(20)), T_NOW)
        .unwrap_err();
    assert!(matches!(err, PairingError::DelegatorRevoked));
}

// ---- apply_epoch honors the anti-rollback floor -----------------------------

#[test]
fn apply_epoch_rejects_a_rolled_back_clock() {
    let mut s = store();
    // Advance the floor to T_NOW.
    s.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    assert!(s.high_water_ns() >= T_NOW);
    // An epoch applied with an earlier clock is a rollback.
    let epoch = make_epoch(rob(ROBOT), 1, vec![], pk(&sk(10)), ISSUED).sign(&sk(10));
    let err = s
        .apply_epoch(&epoch, &good_intermediate(), T_NOW - 1)
        .unwrap_err();
    assert!(matches!(err, PairingError::RollbackDetected { .. }));
}

// ---- test-hardening 8: tag-region flip + valid-HMAC structural arms --------

fn tag_hmac(key: &[u8], framed: &[u8]) -> [u8; 32] {
    use hmac::{Hmac, Mac};
    use sha2::Sha256;
    let mut m = <Hmac<Sha256> as Mac>::new_from_slice(key).unwrap();
    m.update(framed);
    m.finalize().into_bytes().into()
}

/// Load a saved file, mutate its header-region bytes, then RE-COMPUTE a valid
/// HMAC tag over the mutated framed region and reattach it. The MAC therefore
/// passes on load, isolating the structural (magic/version) check.
fn reframe_with_valid_tag(path: &std::path::Path, mutate: impl FnOnce(&mut Vec<u8>)) {
    let bytes = std::fs::read(path).unwrap();
    let mut framed = bytes[..bytes.len() - 32].to_vec();
    mutate(&mut framed);
    let tag = tag_hmac(MAC_KEY, &framed);
    framed.extend_from_slice(&tag);
    std::fs::write(path, &framed).unwrap();
}

#[test]
fn flipping_a_tag_byte_is_tamper() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("trust.store");
    store().with_path(&path).save(MAC_KEY).unwrap();

    let mut bytes = std::fs::read(&path).unwrap();
    let last = bytes.len() - 1;
    bytes[last] ^= 0xFF; // corrupt a byte INSIDE the HMAC tag
    std::fs::write(&path, &bytes).unwrap();

    let err = TrustStore::load(&path, MAC_KEY).unwrap_err();
    assert!(matches!(err, PairingError::TamperDetected));
}

#[test]
fn valid_hmac_over_wrong_magic_is_store_corrupt_not_tamper() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("trust.store");
    store().with_path(&path).save(MAC_KEY).unwrap();

    // Corrupt the magic but keep a VALID HMAC — the MAC passes, so the structural
    // check must run and reject with StoreCorrupt (proving check order).
    reframe_with_valid_tag(&path, |framed| framed[0] ^= 0xFF);
    let err = TrustStore::load(&path, MAC_KEY).unwrap_err();
    assert!(matches!(err, PairingError::StoreCorrupt(_)));
}

#[test]
fn valid_hmac_over_wrong_version_is_store_corrupt_not_tamper() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("trust.store");
    store().with_path(&path).save(MAC_KEY).unwrap();

    // Bump the version field (bytes 8..10, LE u16) to an unsupported value with a
    // valid HMAC → StoreCorrupt.
    reframe_with_valid_tag(&path, |framed| {
        framed[8] = 0x63; // 99
        framed[9] = 0x00;
    });
    let err = TrustStore::load(&path, MAC_KEY).unwrap_err();
    assert!(matches!(err, PairingError::StoreCorrupt(_)));
}

// -- login-gated owner claim (claim_by_owner_grant) ---------------------------
//
// The primary owner-claim path: an OWNER_FULL grant issued by the account service
// for the installer's account takes an UNCLAIMED robot to Claimed(owner) with zero
// robot↔cloud contact. The device cert binds the robot's OWN transport key to the
// installer account (shell access is the authority proof, by design), so the
// authenticated peer key is the robot's transport key. Chassis `claim()` survives
// unchanged as the recovery path.

/// The owner's device cert is issued for the robot's OWN transport key (the
/// installer logs in ON the robot; that key is certified to the installer account).
const OWNER_DEVICE_SEED: u8 = ROBOT_TRANSPORT_SEED;

/// A full OWNER-scoped presentation for `owner`: an OWNER_FULL device cert (for the
/// robot's transport key) + an OWNER_FULL depth-0 grant for THIS robot, both
/// intermediate-issued. `scope` overrides the (device-cert AND grant) scope so a
/// test can present a NON-owner grant.
fn owner_presentation_scoped(owner: AccountId, scope: Scope) -> PairingPresentation {
    let device_cert = make_device_cert(
        pk(&sk(OWNER_DEVICE_SEED)),
        owner,
        pk(&sk(10)),
        scope,
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let grant = make_grant(
        owner,
        rob(ROBOT),
        scope,
        0,
        acct(0x0C),
        pk(&sk(10)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    PairingPresentation {
        intermediate: good_intermediate(),
        device_cert,
        grant,
        delegation: None,
    }
}

/// The default OWNER_FULL owner presentation.
fn owner_presentation(owner: AccountId) -> PairingPresentation {
    owner_presentation_scoped(owner, Scope::OWNER_FULL)
}

/// Like [`owner_presentation_scoped`] but sets the device cert and grant scopes
/// INDEPENDENTLY, so a test can present an OWNER grant behind a narrower device
/// cert and exercise the EFFECTIVE-scope (device cert ∩ grant) gate that
/// `claim_by_owner_grant` checks (rather than the grant's scope alone).
fn owner_presentation_split(
    owner: AccountId,
    device_scope: Scope,
    grant_scope: Scope,
) -> PairingPresentation {
    let device_cert = make_device_cert(
        pk(&sk(OWNER_DEVICE_SEED)),
        owner,
        pk(&sk(10)),
        device_scope,
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let grant = make_grant(
        owner,
        rob(ROBOT),
        grant_scope,
        0,
        acct(0x0C),
        pk(&sk(10)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    PairingPresentation {
        intermediate: good_intermediate(),
        device_cert,
        grant,
        delegation: None,
    }
}

/// The transport key the owner install authenticates as (the robot's own key).
fn owner_peer_key() -> PublicKey {
    pk(&sk(OWNER_DEVICE_SEED))
}

#[test]
fn owner_grant_claim_flips_unclaimed_to_claimed_owner() {
    let mut s = store();
    assert!(!s.is_claimed(), "provisioned store is unclaimed");

    let account = s
        .claim_by_owner_grant(&owner_presentation(acct(OWNER)), &owner_peer_key(), T_NOW)
        .expect("an OWNER_FULL owner grant claims an unclaimed robot");

    // The returned account, the ownership state, and the owner row all agree.
    assert_eq!(account, acct(OWNER));
    assert_eq!(s.ownership(), OwnershipState::Claimed(acct(OWNER)));
    assert_eq!(s.owner(), Some(acct(OWNER)));
    assert!(s.is_claimed());

    let row = s
        .is_allowed(&acct(OWNER), T_NOW)
        .expect("the owner is allowed after the claim");
    assert_eq!(row.scope, Scope::OWNER_FULL, "owner row carries OWNER_FULL");
    assert_eq!(
        row.source,
        PairingSource::OwnerGrant,
        "the owner row's provenance is the login-gated owner grant, not a physical Claim"
    );
    assert_eq!(row.name.as_deref(), Some("owner"));
    assert!(!row.revoked);
    // The anti-rollback floor advanced to the observation time.
    assert!(s.high_water_ns() >= T_NOW);
    assert_eq!(s.access_rows().len(), 1, "exactly one owner row");
}

#[test]
fn owner_grant_claim_refused_on_already_claimed() {
    let mut s = store();
    s.claim_by_owner_grant(&owner_presentation(acct(OWNER)), &owner_peer_key(), T_NOW)
        .unwrap();
    // A second owner-grant claim (even for the SAME account) is refused — a claim
    // is the FIRST ownership write, never a silent re-own.
    let err = s
        .claim_by_owner_grant(&owner_presentation(acct(OWNER)), &owner_peer_key(), T_NOW)
        .unwrap_err();
    assert!(matches!(err, PairingError::AlreadyClaimed));
    // And a chassis claim on the already-owned robot is likewise refused.
    let err = s
        .claim(acct(0x09), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap_err();
    assert!(matches!(err, PairingError::AlreadyClaimed));
}

#[test]
fn non_owner_grant_cannot_claim_ownership() {
    // A well-formed, correctly-signed presentation whose scope is OPERATOR (not
    // OWNER) must NOT be able to claim ownership — only an OWNER-scoped grant may.
    let mut s = store();
    let err = s
        .claim_by_owner_grant(
            &owner_presentation_scoped(acct(OWNER), op_scope()),
            &owner_peer_key(),
            T_NOW,
        )
        .unwrap_err();
    assert!(
        matches!(err, PairingError::NotOwnerGrant),
        "an OPERATOR-scoped grant is refused with NotOwnerGrant, got {err:?}"
    );
    // The robot stays UNCLAIMED with zero rows — a refused claim writes nothing.
    assert!(!s.is_claimed());
    assert_eq!(s.ownership(), OwnershipState::Unclaimed);
    assert!(s.access_rows().is_empty());
}

#[test]
fn owner_grant_for_a_different_robot_is_refused() {
    // The grant names a DIFFERENT robot; the chain verifier rejects it (WrongRobot)
    // and the store is untouched.
    let mut s = store();
    let mut pres = owner_presentation(acct(OWNER));
    pres.grant = make_grant(
        acct(OWNER),
        rob(0xEE), // not this robot
        Scope::OWNER_FULL,
        0,
        acct(0x0C),
        pk(&sk(10)),
        wide_validity(),
        ISSUED,
    )
    .sign(&sk(10));
    let err = s
        .claim_by_owner_grant(&pres, &owner_peer_key(), T_NOW)
        .unwrap_err();
    assert!(matches!(err, PairingError::WrongRobot), "got {err:?}");
    assert!(!s.is_claimed());
    assert!(s.access_rows().is_empty());
}

#[test]
fn owner_grant_with_mismatched_peer_key_is_refused() {
    // The device cert binds the robot's transport key; presenting a DIFFERENT
    // authenticated peer key is refused (PeerKeyMismatch) — a device cert for
    // another key cannot claim this robot.
    let mut s = store();
    let err = s
        .claim_by_owner_grant(
            &owner_presentation(acct(OWNER)),
            &pk(&sk(99)), // not the robot's transport key
            T_NOW,
        )
        .unwrap_err();
    assert!(matches!(err, PairingError::PeerKeyMismatch), "got {err:?}");
    assert!(!s.is_claimed());
    assert!(s.access_rows().is_empty());
}

#[test]
fn owner_grant_claim_refused_on_clock_rollback() {
    // Provision with a high-water floor in the future; a claim whose `now_ns` is
    // BEHIND the floor is rejected before any chain work (anti-rollback holds on
    // the owner-claim path too).
    let future = T_NOW + 10_000_000_000;
    let mut s = TrustStore::provision(
        rob(ROBOT),
        pk(&sk(ROBOT_TRANSPORT_SEED)),
        root_set(),
        CHASSIS,
        future,
    )
    .unwrap();
    let err = s
        .claim_by_owner_grant(&owner_presentation(acct(OWNER)), &owner_peer_key(), T_NOW)
        .unwrap_err();
    assert!(
        matches!(err, PairingError::RollbackDetected { .. }),
        "got {err:?}"
    );
    assert!(!s.is_claimed());
}

#[test]
fn owner_grant_claim_persists_across_save_load() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("trust.store");
    let mut s = store().with_path(&path);
    s.claim_by_owner_grant(&owner_presentation(acct(OWNER)), &owner_peer_key(), T_NOW)
        .unwrap();
    s.save(MAC_KEY).unwrap();

    let loaded = TrustStore::load(&path, MAC_KEY).unwrap();
    assert_eq!(loaded.owner(), Some(acct(OWNER)));
    assert!(loaded.is_claimed());
    let row = loaded.is_allowed(&acct(OWNER), T_NOW).unwrap();
    assert_eq!(row.source, PairingSource::OwnerGrant);
    assert_eq!(row.scope, Scope::OWNER_FULL);
}

#[test]
fn owner_grant_claimed_robot_admits_a_new_guest_pairing() {
    // A robot claimed via the login-gated owner grant is a genuinely-claimed robot:
    // a subsequent guest strong-chain pairing is admitted, exactly as after a
    // chassis claim (proves claim_by_owner_grant is not a partial/half claim).
    let mut s = store();
    s.claim_by_owner_grant(&owner_presentation(acct(OWNER)), &owner_peer_key(), T_NOW)
        .unwrap();
    // good_presentation() pairs account 0x0A (device key sk(20)) for this robot.
    let verified = s
        .verify_and_establish(
            &good_presentation(),
            &pk(&sk(20)),
            T_NOW,
            Some("guest".into()),
        )
        .expect("a claimed robot admits a guest strong-chain pairing");
    assert_eq!(verified.account(), acct(0x0A));
    assert!(s.is_allowed(&acct(0x0A), T_NOW).is_some());
}

#[test]
fn chassis_claim_still_works_as_recovery_and_matches_owner_grant_claim() {
    // The demotion of chassis claim() is documentation-only — it still functions.
    // Two fresh robots: one claimed by chassis secret, one by owner grant, both land
    // Claimed(owner) with an OWNER_FULL owner row (only the source differs).
    let mut a = store();
    a.claim(acct(OWNER), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    let mut b = store();
    b.claim_by_owner_grant(&owner_presentation(acct(OWNER)), &owner_peer_key(), T_NOW)
        .unwrap();

    assert_eq!(a.owner(), b.owner());
    assert_eq!(a.owner(), Some(acct(OWNER)));
    let ra = a.is_allowed(&acct(OWNER), T_NOW).unwrap();
    let rb = b.is_allowed(&acct(OWNER), T_NOW).unwrap();
    assert_eq!(ra.scope, Scope::OWNER_FULL);
    assert_eq!(rb.scope, Scope::OWNER_FULL);
    assert_eq!(ra.source, PairingSource::Claim);
    assert_eq!(rb.source, PairingSource::OwnerGrant);

    // And chassis claim survives a factory_reset as the recovery route.
    b.factory_reset(CHASSIS).unwrap();
    assert!(!b.is_claimed());
    b.claim(acct(0x07), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    assert_eq!(b.owner(), Some(acct(0x07)));
}

#[test]
fn owner_grant_narrowed_below_owner_by_device_cert_is_refused() {
    // The claim gate is on the EFFECTIVE scope (device cert ∩ grant), NOT the
    // grant's scope alone. A grant that SAYS OWNER but whose effective intersection
    // narrows below OWNER (here the device cert is only OPERATOR, so the intersected
    // role is OPERATOR) must NOT claim ownership → NotOwnerGrant. Anti-tautology vs
    // the existing both-links-OPERATOR test (`non_owner_grant_cannot_claim_ownership`):
    // the GRANT here is genuinely OWNER_FULL, so a grant-only check would WRONGLY
    // accept it — only an effective-scope gate refuses.
    let mut s = store();
    let pres = owner_presentation_split(acct(OWNER), op_scope(), Scope::OWNER_FULL);
    // Sanity: the grant confers OWNER on its own; only the device-cert intersection
    // narrows the effective role to OPERATOR.
    assert_eq!(pres.grant.grant.scope.role, Role::OWNER);
    assert_eq!(pres.device_cert.cert.scope.role, Role::OPERATOR);

    let err = s
        .claim_by_owner_grant(&pres, &owner_peer_key(), T_NOW)
        .unwrap_err();
    assert!(
        matches!(err, PairingError::NotOwnerGrant),
        "an OWNER grant whose effective (device ∩ grant) scope narrows to OPERATOR \
         must be refused with NotOwnerGrant, got {err:?}"
    );
    // Nothing was written — the robot stays unclaimed with zero rows.
    assert!(!s.is_claimed());
    assert_eq!(s.ownership(), OwnershipState::Unclaimed);
    assert!(s.access_rows().is_empty());
}

#[test]
fn a_valid_owner_grant_for_a_second_account_cannot_take_over_a_claimed_robot() {
    // Robot Claimed(A). Account B presents a fully-VALID OWNER grant for THIS robot
    // → AlreadyClaimed (the `is_claimed()` gate fires FIRST, before any chain work);
    // ownership stays Claimed(A), and B gets NO access row. A claim is the FIRST
    // ownership write, never a silent takeover by a second account.
    const SECOND_OWNER: u8 = 0x08;
    let mut s = store();
    let first_owner = s
        .claim_by_owner_grant(&owner_presentation(acct(OWNER)), &owner_peer_key(), T_NOW)
        .expect("account A claims the unclaimed robot");
    assert_eq!(first_owner, acct(OWNER));

    let err = s
        .claim_by_owner_grant(
            &owner_presentation(acct(SECOND_OWNER)),
            &owner_peer_key(),
            T_NOW,
        )
        .unwrap_err();
    assert!(
        matches!(err, PairingError::AlreadyClaimed),
        "a second account's owner grant must be refused on a claimed robot, got {err:?}"
    );
    // Ownership unchanged; B has no row; exactly the one A owner row remains.
    assert_eq!(s.ownership(), OwnershipState::Claimed(acct(OWNER)));
    assert_eq!(s.owner(), Some(acct(OWNER)));
    assert!(
        s.is_allowed(&acct(SECOND_OWNER), T_NOW).is_none(),
        "the takeover account must have no access row"
    );
    assert_eq!(s.access_rows().len(), 1);
    assert_eq!(s.access_rows()[0].account, acct(OWNER));

    // Anti-tautology: the SAME B presentation genuinely WOULD claim a FRESH robot,
    // so the refusal above is the AlreadyClaimed gate, not a malformed B grant.
    let mut fresh = store();
    fresh
        .claim_by_owner_grant(
            &owner_presentation(acct(SECOND_OWNER)),
            &owner_peer_key(),
            T_NOW,
        )
        .expect("B's grant is itself valid — it claims a fresh unclaimed robot");
    assert_eq!(fresh.owner(), Some(acct(SECOND_OWNER)));
}
