// SPDX-License-Identifier: AGPL-3.0-only
//! DB-layer coverage for the root fixes (over `Db::open_in_memory`, pub
//! API, hand oracles):
//! - **A** — device registration is account-scoped: same-account is idempotent
//!   (stored principal wins), a foreign account is a loud Conflict.
//! - **C** — oauth flow `state` is expiry-gated + single-use; the sweep bounds
//!   the housekeeping tables.
//! - **G** — magic-link consume is single-use + expiry-gated (rows-affected CAS).

use cerulion_accountd::{
    poll_outcome, should_record_poll, AccountdError, Db, PollOutcome, RevocationTarget,
};

const NOW: u64 = 1_000_000_000_000;
const FUTURE: u64 = NOW + 600_000_000_000; // now + 10 min
const SEC: u64 = 1_000_000_000;

const ACCT_A: [u8; 32] = [0xAA; 32];
const ACCT_B: [u8; 32] = [0xBB; 32];
const KEY_K: [u8; 32] = [0x11; 32];

const HUMAN: u8 = 1;
const MACHINE: u8 = 2;

// ============================================================================
// A — register_device account scoping
// ============================================================================

#[test]
fn same_account_reregistration_is_idempotent() {
    let db = Db::open_in_memory().unwrap();
    let first = db.register_device(&ACCT_A, &KEY_K, HUMAN, NOW).unwrap();
    let again = db.register_device(&ACCT_A, &KEY_K, HUMAN, NOW).unwrap();
    // Same row (no second INSERT).
    assert_eq!(first.device_id, again.device_id);
    assert_eq!(again.account_id, ACCT_A);
}

#[test]
fn a_key_owned_by_another_account_is_a_loud_conflict() {
    let db = Db::open_in_memory().unwrap();
    db.register_device(&ACCT_A, &KEY_K, HUMAN, NOW).unwrap();
    // Account B tries to register A's key → Conflict, NOT a silent foreign-row.
    let err = db
        .register_device(&ACCT_B, &KEY_K, MACHINE, NOW)
        .expect_err("registering a foreign-owned key must conflict");
    assert!(matches!(err, AccountdError::Conflict(_)), "got {err:?}");
    // A's row is untouched (still A, still HUMAN).
    let devices = db.list_devices(&ACCT_A).unwrap();
    assert_eq!(devices.len(), 1);
    assert_eq!(devices[0].principal_kind, HUMAN);
    // B has no device.
    assert!(db.list_devices(&ACCT_B).unwrap().is_empty());
}

#[test]
fn reregistration_returns_the_stored_principal_not_the_request() {
    let db = Db::open_in_memory().unwrap();
    db.register_device(&ACCT_A, &KEY_K, HUMAN, NOW).unwrap();
    // Same account re-registers claiming MACHINE — the STORED row (HUMAN) is
    // authoritative and returned unchanged (the cert is minted from this).
    let row = db.register_device(&ACCT_A, &KEY_K, MACHINE, NOW).unwrap();
    assert_eq!(row.principal_kind, HUMAN);
}

// ============================================================================
// C — oauth flow expiry + single-use + sweep
// ============================================================================

#[test]
fn peek_oauth_flow_rejects_an_expired_state() {
    let db = Db::open_in_memory().unwrap();
    // expires in the PAST.
    db.insert_oauth_flow(
        "st-1",
        "google",
        "verifier",
        Some("USER-CODE"),
        NOW - 1,
        NOW - 2,
    )
    .unwrap();
    assert!(db.peek_oauth_flow("st-1", NOW).unwrap().is_none());
}

#[test]
fn peek_is_repeatable_but_consume_is_single_use() {
    let db = Db::open_in_memory().unwrap();
    db.insert_oauth_flow("st-2", "github", "verifier2", None, FUTURE, NOW)
        .unwrap();
    // PEEK does NOT consume — a failed exchange can retry. Repeatable.
    let peeked = db.peek_oauth_flow("st-2", NOW).unwrap();
    assert_eq!(
        peeked,
        Some(("github".to_string(), "verifier2".to_string(), None))
    );
    assert!(db.peek_oauth_flow("st-2", NOW).unwrap().is_some());

    // CONSUME (on exchange success) is the single-use CAS: first wins, second loses.
    assert!(db.consume_oauth_flow("st-2").unwrap());
    assert!(!db.consume_oauth_flow("st-2").unwrap());
    // After consume, the row is gone.
    assert!(db.peek_oauth_flow("st-2", NOW).unwrap().is_none());
}

#[test]
fn sweep_deletes_expired_rows_and_spares_fresh_ones() {
    let db = Db::open_in_memory().unwrap();
    // An EXPIRED device code (verifiable via snapshot) + a FRESH one.
    db.insert_device_code_retrying("dc-expired", NOW - 1, 0, NOW - 2, 4, || {
        Ok("EXPIRED-CODE".to_string())
    })
    .unwrap();
    db.insert_device_code_retrying("dc-fresh", FUTURE, 0, NOW, 4, || {
        Ok("FRESH-CODE".to_string())
    })
    .unwrap();
    // An expired oauth flow + an expired magic link.
    db.insert_oauth_flow("st-old", "google", "v", None, NOW - 1, NOW - 2)
        .unwrap();
    db.insert_magic_link("ml-old", "a@b.c", "UC", NOW - 1)
        .unwrap();

    let deleted = db.sweep_expired(NOW).unwrap();
    assert_eq!(
        deleted, 3,
        "the 3 expired rows are swept, the fresh one is not"
    );

    assert!(db.device_code_snapshot("dc-expired").unwrap().is_none());
    assert!(db.device_code_snapshot("dc-fresh").unwrap().is_some());
    assert!(db.peek_oauth_flow("st-old", NOW).unwrap().is_none());
}

// ============================================================================
// G — magic-link single-use + expiry
// ============================================================================

#[test]
fn magic_link_consume_is_single_use() {
    let db = Db::open_in_memory().unwrap();
    db.insert_magic_link("ml-1", "pilot@example.com", "UC-1", FUTURE)
        .unwrap();
    let first = db.consume_magic_link("ml-1", NOW).unwrap();
    assert_eq!(
        first,
        Some(("pilot@example.com".to_string(), "UC-1".to_string()))
    );
    // Single-use: a second consume returns None.
    assert!(db.consume_magic_link("ml-1", NOW).unwrap().is_none());
}

#[test]
fn expired_magic_link_is_rejected() {
    let db = Db::open_in_memory().unwrap();
    db.insert_magic_link("ml-2", "x@y.z", "UC-2", NOW - 1)
        .unwrap();
    assert!(db.consume_magic_link("ml-2", NOW).unwrap().is_none());
}

// ============================================================================
// register_robot ownership scoping (one-key-one-robot-owner)
// ============================================================================

const ROBOT_KEY: [u8; 32] = [0x22; 32];

#[test]
fn register_robot_binds_the_owner_and_mints_a_robot_id() {
    let db = Db::open_in_memory().unwrap();
    let r = db
        .register_robot(&ACCT_A, "orin-01", &ROBOT_KEY, None, NOW)
        .unwrap();
    assert_eq!(r.owner_account_id, ACCT_A);
    assert_eq!(r.hostname, "orin-01");
    assert_eq!(r.robot_transport_key, ROBOT_KEY);
    assert_eq!(r.org_id, None);
    assert_eq!(r.created_at_ns, NOW);
    // A fresh robot id was minted (not all-zero, distinct from the transport key).
    assert_ne!(r.robot_id, [0u8; 32]);
    assert_ne!(r.robot_id, ROBOT_KEY);
    // It reads back by id.
    let got = db.get_robot(&r.robot_id).unwrap().expect("robot exists");
    assert_eq!(got.owner_account_id, ACCT_A);
    assert_eq!(got.hostname, "orin-01");
}

#[test]
fn same_owner_robot_reregistration_is_idempotent() {
    let db = Db::open_in_memory().unwrap();
    let first = db
        .register_robot(&ACCT_A, "orin-01", &ROBOT_KEY, None, NOW)
        .unwrap();
    // Re-register the SAME transport key for the SAME owner (a retried/re-run
    // install) -> the stored row, no new robot id.
    let again = db
        .register_robot(&ACCT_A, "orin-renamed", &ROBOT_KEY, Some("org-x"), FUTURE)
        .unwrap();
    assert_eq!(first.robot_id, again.robot_id, "no second INSERT");
    assert_eq!(
        again.hostname, "orin-01",
        "the stored row is returned unchanged"
    );
    assert_eq!(again.created_at_ns, NOW);
    assert_eq!(again.org_id, None);
    assert_eq!(
        db.get_robot(&first.robot_id).unwrap().unwrap().hostname,
        "orin-01"
    );
}

#[test]
fn a_transport_key_owned_by_another_account_is_a_loud_conflict() {
    let db = Db::open_in_memory().unwrap();
    let a = db
        .register_robot(&ACCT_A, "orin-01", &ROBOT_KEY, None, NOW)
        .unwrap();
    // Account B tries to claim A's robot transport key -> Conflict, never a silent
    // foreign-row return that would let B mint an owner grant for A's robot.
    let err = db
        .register_robot(&ACCT_B, "impostor", &ROBOT_KEY, None, NOW)
        .expect_err("a foreign-owned transport key must conflict");
    assert!(matches!(err, AccountdError::Conflict(_)), "got {err:?}");
    // A's robot is untouched (still A, still orin-01).
    let got = db.get_robot(&a.robot_id).unwrap().unwrap();
    assert_eq!(got.owner_account_id, ACCT_A);
    assert_eq!(got.hostname, "orin-01");
}

#[test]
fn org_id_is_stored_verbatim_when_present() {
    let db = Db::open_in_memory().unwrap();
    let r = db
        .register_robot(&ACCT_A, "orin-01", &ROBOT_KEY, Some("acme"), NOW)
        .unwrap();
    assert_eq!(r.org_id.as_deref(), Some("acme"));
    assert_eq!(
        db.get_robot(&r.robot_id)
            .unwrap()
            .unwrap()
            .org_id
            .as_deref(),
        Some("acme")
    );
}

#[test]
fn get_robot_for_an_unknown_id_is_none() {
    let db = Db::open_in_memory().unwrap();
    assert!(db.get_robot(&[0x77; 32]).unwrap().is_none());
}

#[test]
fn two_distinct_transport_keys_are_two_robots_for_one_owner() {
    let db = Db::open_in_memory().unwrap();
    let r1 = db
        .register_robot(&ACCT_A, "orin-01", &[0x22; 32], None, NOW)
        .unwrap();
    let r2 = db
        .register_robot(&ACCT_A, "orin-02", &[0x33; 32], None, NOW)
        .unwrap();
    assert_ne!(r1.robot_id, r2.robot_id);
    assert_eq!(
        db.get_robot(&r1.robot_id).unwrap().unwrap().hostname,
        "orin-01"
    );
    assert_eq!(
        db.get_robot(&r2.robot_id).unwrap().unwrap().hostname,
        "orin-02"
    );
}

// ============================================================================
// A SlowDown poll does not advance the interval gate
// ============================================================================

#[test]
fn a_slowdown_poll_does_not_lock_out_a_client_polling_near_the_interval() {
    const INTERVAL: u64 = 5;
    let db = Db::open_in_memory().unwrap();
    db.insert_device_code_retrying("dc", FUTURE, INTERVAL, NOW, 4, || Ok("UC".to_string()))
        .unwrap();

    // Poll 1 at NOW: pending, allowed → the handler records it.
    let s1 = db.device_code_snapshot("dc").unwrap().unwrap();
    let o1 = poll_outcome(&s1, NOW);
    assert!(matches!(o1, PollOutcome::AuthorizationPending));
    assert!(should_record_poll(&o1));
    db.touch_device_code_poll("dc", NOW).unwrap();

    // Poll 2 at NOW+2s: within the 5s interval → SlowDown → the handler must NOT
    // record it (the fix). We faithfully skip the touch here.
    let t2 = NOW + 2 * SEC;
    let s2 = db.device_code_snapshot("dc").unwrap().unwrap();
    let o2 = poll_outcome(&s2, t2);
    assert!(matches!(o2, PollOutcome::SlowDown));
    assert!(!should_record_poll(&o2));

    // Poll 3 at NOW+6s: last_poll is STILL NOW (the SlowDown poll did not reset it),
    // so 6s > 5s → allowed. The client is NOT permanently locked out.
    let t3 = NOW + 6 * SEC;
    let s3 = db.device_code_snapshot("dc").unwrap().unwrap();
    assert!(
        matches!(poll_outcome(&s3, t3), PollOutcome::AuthorizationPending),
        "a SlowDown poll must not lock out a client polling near the interval"
    );

    // CONCRETE anti-tautology: a second code where the NOW+2s poll IS recorded (as it
    // would be without the SlowDown exemption) IS still throttled at NOW+6s (t3 - t2 = 4s < 5s).
    db.insert_device_code_retrying("dc2", FUTURE, INTERVAL, NOW, 4, || Ok("UC2".to_string()))
        .unwrap();
    db.touch_device_code_poll("dc2", NOW).unwrap();
    db.touch_device_code_poll("dc2", t2).unwrap(); // recorded the throttled poll (no exemption)
    let s = db.device_code_snapshot("dc2").unwrap().unwrap();
    assert!(
        matches!(poll_outcome(&s, t3), PollOutcome::SlowDown),
        "recording the SlowDown poll DOES lock out — the behavior the exemption removes"
    );
}

// ============================================================================
// rotate_session hardened CAS (revoked + old-hash guards)
// ============================================================================

#[test]
fn rotate_session_is_guarded_on_revoked_and_the_old_refresh_hash() {
    let db = Db::open_in_memory().unwrap();
    let id = db
        .insert_session("user-1", "sess-h", "refr-h", FUTURE, FUTURE, NOW)
        .unwrap();

    // WRONG old refresh hash → the CAS misses → false (the #4 single-use guard).
    assert!(!db
        .rotate_session(&id, "WRONG-OLD-HASH", "ns", "nr", FUTURE, FUTURE)
        .unwrap());

    // Correct id + correct old hash + not revoked → true (rotates once).
    assert!(db
        .rotate_session(&id, "refr-h", "ns", "nr", FUTURE, FUTURE)
        .unwrap());
    // The old hash is now dead — replaying it → false (rotation is single-use).
    assert!(!db
        .rotate_session(&id, "refr-h", "ns2", "nr2", FUTURE, FUTURE)
        .unwrap());

    // REVOKED guard (#3): a revoked session cannot rotate even with the right hash.
    let id2 = db
        .insert_session("user-2", "s2", "r2", FUTURE, FUTURE, NOW)
        .unwrap();
    assert!(db.revoke_session_by_token_hash("s2").unwrap());
    assert!(
        !db.rotate_session(&id2, "r2", "ns", "nr", FUTURE, FUTURE)
            .unwrap(),
        "a revoked session must never rotate (a concurrent revoke is not clobbered)"
    );
}

// ============================================================================
// authorize_device_code honors expiry
// ============================================================================

#[test]
fn an_expired_pending_device_code_cannot_be_authorized() {
    let db = Db::open_in_memory().unwrap();
    // A pending code that is ALREADY expired (expires_at in the past).
    db.insert_device_code_retrying(
        "dc-exp",
        NOW - 1,
        0,
        NOW - 2,
        4,
        || Ok("EXP-UC".to_string()),
    )
    .unwrap();
    // Authorizing it at NOW → false: consistent with the poll side (an expired
    // pending code is dead and cannot be authorized into a live session).
    assert!(!db.authorize_device_code("EXP-UC", "user-x", NOW).unwrap());

    // Control: a fresh pending code authorizes → true (anti-tautology).
    db.insert_device_code_retrying("dc-live", FUTURE, 0, NOW, 4, || Ok("LIVE-UC".to_string()))
        .unwrap();
    assert!(db.authorize_device_code("LIVE-UC", "user-y", NOW).unwrap());

    // An unknown user_code → false.
    assert!(!db
        .authorize_device_code("NO-SUCH-CODE", "user-z", NOW)
        .unwrap());
}

// ============================================================================
// device self-revoke + access-list revocation epochs
// ============================================================================

const ROBOT_ID_EPOCHS: [u8; 32] = [0x33; 32];
const DEV_A: [u8; 32] = [0xD1; 32];
const DEV_B: [u8; 32] = [0xD2; 32];

#[test]
fn revoke_device_is_account_scoped_and_idempotent() {
    let db = Db::open_in_memory().unwrap();
    // ACCT_A owns a device (KEY_K); ACCT_B owns another (DEV_A).
    let a = db.register_device(&ACCT_A, &KEY_K, HUMAN, NOW).unwrap();
    let b = db.register_device(&ACCT_B, &DEV_A, HUMAN, NOW).unwrap();

    // ACCT_A revokes its OWN device → the updated row, revoked == true.
    let row = db
        .revoke_device(&ACCT_A, &a.device_id)
        .unwrap()
        .expect("owner may revoke its own device");
    assert!(row.revoked);
    assert_eq!(row.device_id, a.device_id);

    // Idempotent: re-revoking still returns the (already-revoked) row.
    let again = db.revoke_device(&ACCT_A, &a.device_id).unwrap().unwrap();
    assert!(again.revoked);

    // ACCT_A CANNOT revoke ACCT_B's device — indistinguishably None (never learns it
    // exists), and B's device stays un-revoked.
    assert!(db.revoke_device(&ACCT_A, &b.device_id).unwrap().is_none());
    let b_devices = db.list_devices(&ACCT_B).unwrap();
    assert_eq!(b_devices.len(), 1);
    assert!(
        !b_devices[0].revoked,
        "B's device is untouched by A's revoke"
    );

    // An unknown device id → None.
    assert!(db
        .revoke_device(&ACCT_A, "no-such-device")
        .unwrap()
        .is_none());
}

#[test]
fn revoke_device_shows_up_in_list_devices() {
    let db = Db::open_in_memory().unwrap();
    let d = db.register_device(&ACCT_A, &KEY_K, HUMAN, NOW).unwrap();
    assert!(!db.list_devices(&ACCT_A).unwrap()[0].revoked);
    db.revoke_device(&ACCT_A, &d.device_id).unwrap();
    assert!(
        db.list_devices(&ACCT_A).unwrap()[0].revoked,
        "the enumeration reflects the revocation"
    );
}

#[test]
fn epochs_insert_and_latest_reads_back_the_flat_blobs() {
    let db = Db::open_in_memory().unwrap();
    // No epoch yet.
    assert!(db.latest_epoch(&ROBOT_ID_EPOCHS).unwrap().is_none());

    // Epoch 1 revokes an account.
    db.insert_epoch(&ROBOT_ID_EPOCHS, 1, &[ACCT_A], &[], NOW)
        .unwrap();
    let e1 = db.latest_epoch(&ROBOT_ID_EPOCHS).unwrap().unwrap();
    assert_eq!(e1.epoch, 1);
    assert_eq!(e1.revoked_accounts, vec![ACCT_A]);
    assert!(e1.revoked_devices.is_empty());
    assert_eq!(e1.issued_at_ns, NOW);

    // Epoch 2 revokes an account AND two devices (the flat 32*N blob round-trips).
    db.insert_epoch(&ROBOT_ID_EPOCHS, 2, &[ACCT_A], &[DEV_A, DEV_B], FUTURE)
        .unwrap();
    let e2 = db.latest_epoch(&ROBOT_ID_EPOCHS).unwrap().unwrap();
    assert_eq!(e2.epoch, 2, "latest_epoch returns the HIGHEST epoch");
    assert_eq!(e2.revoked_accounts, vec![ACCT_A]);
    assert_eq!(e2.revoked_devices, vec![DEV_A, DEV_B]);
    assert_eq!(e2.issued_at_ns, FUTURE);
}

#[test]
fn a_duplicate_epoch_number_is_a_loud_conflict() {
    let db = Db::open_in_memory().unwrap();
    db.insert_epoch(&ROBOT_ID_EPOCHS, 1, &[], &[], NOW).unwrap();
    // Inserting epoch 1 again for the same robot collides on the PK.
    let err = db
        .insert_epoch(&ROBOT_ID_EPOCHS, 1, &[ACCT_A], &[], FUTURE)
        .unwrap_err();
    assert!(matches!(err, AccountdError::Conflict(_)), "got {err:?}");
    // The original epoch is intact (the failed insert changed nothing).
    let e = db.latest_epoch(&ROBOT_ID_EPOCHS).unwrap().unwrap();
    assert!(e.revoked_accounts.is_empty());
}

#[test]
fn concurrent_add_revocation_on_one_robot_mints_monotonic_epochs_no_conflict() {
    // `Db::add_revocation` does the read-modify-write under
    // ONE lock, so two CONCURRENT revokes on the SAME robot cannot slip an epoch in
    // between and force a spurious Conflict (the two-acquisition race). A
    // bounded HAMMER over many robots, each hit by two threads gated on a start barrier
    // to maximize interleaving: EVERY round must have both threads succeed (no
    // Conflict), mint the monotonic epochs {1,2}, and end with BOTH devices in the
    // latest epoch. With a two-step read-then-write some rounds would raise Conflict.
    let db = Db::open_in_memory().unwrap();
    const ROUNDS: usize = 40;
    for r in 0..ROUNDS {
        let mut robot = [0u8; 32];
        robot[0] = 0xC0;
        robot[1] = r as u8;
        robot[2] = (r >> 8) as u8;
        let dev_a = [0xA0u8; 32];
        let dev_b = [0xB0u8; 32];

        let start = std::sync::Arc::new(std::sync::Barrier::new(2));
        let (d1, d2) = (db.clone(), db.clone());
        let (s1, s2) = (start.clone(), start.clone());
        let t1 = std::thread::spawn(move || {
            s1.wait();
            d1.add_revocation(&robot, RevocationTarget::Device(dev_a), 1)
        });
        let t2 = std::thread::spawn(move || {
            s2.wait();
            d2.add_revocation(&robot, RevocationTarget::Device(dev_b), 2)
        });
        let r1 = t1
            .join()
            .unwrap()
            .expect("no error")
            .expect("first mints an epoch");
        let r2 = t2
            .join()
            .unwrap()
            .expect("no error (no Conflict — the atomic single-lock fix)")
            .expect("second mints an epoch");

        // Both succeeded with distinct, monotonic epoch numbers {1,2}.
        let mut epochs = [r1.epoch, r2.epoch];
        epochs.sort_unstable();
        assert_eq!(
            epochs,
            [1, 2],
            "round {r}: two concurrent revokes must mint monotonic epochs, no conflict"
        );
        // The latest epoch (2) accumulated BOTH devices (the second read the first's write).
        let latest = db.latest_epoch(&robot).unwrap().unwrap();
        assert_eq!(latest.epoch, 2);
        assert!(
            latest.revoked_devices.contains(&dev_a) && latest.revoked_devices.contains(&dev_b),
            "round {r}: the final epoch must contain both concurrently-revoked devices"
        );
    }
}

#[test]
fn add_revocation_is_idempotent_and_monotonic() {
    let db = Db::open_in_memory().unwrap();
    let robot = ROBOT_ID_EPOCHS;
    // First device revoke → epoch 1.
    let r1 = db
        .add_revocation(&robot, RevocationTarget::Device(DEV_A), NOW)
        .unwrap()
        .unwrap();
    assert_eq!(r1.epoch, 1);
    // Re-revoking the SAME device → None (no new epoch minted).
    assert!(db
        .add_revocation(&robot, RevocationTarget::Device(DEV_A), FUTURE)
        .unwrap()
        .is_none());
    assert_eq!(db.latest_epoch(&robot).unwrap().unwrap().epoch, 1);
    // A DIFFERENT target (account) → epoch 2, accumulating over the device set.
    let r2 = db
        .add_revocation(&robot, RevocationTarget::Account(ACCT_A), FUTURE)
        .unwrap()
        .unwrap();
    assert_eq!(r2.epoch, 2);
    assert_eq!(r2.revoked_devices, vec![DEV_A]);
    assert_eq!(r2.revoked_accounts, vec![ACCT_A]);
}

#[test]
fn list_robots_by_owner_is_account_scoped() {
    let db = Db::open_in_memory().unwrap();
    // ACCT_A owns two robots; ACCT_B owns one.
    db.register_robot(&ACCT_A, "a-orin-1", &[0x01; 32], None, NOW)
        .unwrap();
    db.register_robot(&ACCT_A, "a-orin-2", &[0x02; 32], None, FUTURE)
        .unwrap();
    db.register_robot(&ACCT_B, "b-orin-1", &[0x03; 32], None, NOW)
        .unwrap();
    let a = db.list_robots_by_owner(&ACCT_A).unwrap();
    assert_eq!(a.len(), 2, "A owns exactly two robots");
    assert!(a.iter().all(|r| r.owner_account_id == ACCT_A));
    let b = db.list_robots_by_owner(&ACCT_B).unwrap();
    assert_eq!(b.len(), 1);
    // An account with no robots gets an empty list.
    assert!(db.list_robots_by_owner(&KEY_K).unwrap().is_empty());
}

#[test]
fn epochs_are_isolated_per_robot() {
    let db = Db::open_in_memory().unwrap();
    let robot_b: [u8; 32] = [0x44; 32];
    db.insert_epoch(&ROBOT_ID_EPOCHS, 5, &[ACCT_A], &[], NOW)
        .unwrap();
    // A different robot has no epoch even though its sibling does.
    assert!(db.latest_epoch(&robot_b).unwrap().is_none());
    db.insert_epoch(&robot_b, 1, &[], &[DEV_A], NOW).unwrap();
    assert_eq!(db.latest_epoch(&ROBOT_ID_EPOCHS).unwrap().unwrap().epoch, 5);
    assert_eq!(db.latest_epoch(&robot_b).unwrap().unwrap().epoch, 1);
}
