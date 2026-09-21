// SPDX-License-Identifier: AGPL-3.0-only
//! `user_code` collision-retry coverage — the `device/start` allocator no longer
//! surfaces a rare short-code birthday collision as a 500.
//!
//! Two layers:
//! 1. **Pure oracle vectors** for the retry driver (`insert_with_collision_retry`)
//!    — collision-then-success, first-success-no-waste, exhaustion, fatal
//!    passthrough, gen-error-is-fatal. No DB.
//! 2. **Real-DB injection**: an injected code generator forces the first candidate
//!    to collide with an already-stored `user_code` (a REAL SQLite UNIQUE
//!    violation), and the retry is proven to land the next candidate — plus the
//!    exhaustion path and the pin that a `device_code_hash` PK collision is FATAL
//!    (never misclassified as a retryable `user_code` collision).

use std::cell::Cell;
use std::collections::HashSet;

use cerulion_accountd::{
    insert_with_collision_retry, AccountdError, AttemptOutcome, Db, RetryGiveUp,
};

const NOW: u64 = 1_000_000_000_000;
const EXP: u64 = NOW + 600_000_000_000;

// ============================================================================
// pure driver oracle vectors
// ============================================================================

fn accept_unless(existing: HashSet<String>) -> impl FnMut(&str) -> AttemptOutcome<String, String> {
    move |code: &str| {
        if existing.contains(code) {
            AttemptOutcome::Collision
        } else {
            AttemptOutcome::Inserted(code.to_string())
        }
    }
}

#[test]
fn collision_then_fresh_code_succeeds() {
    let mut codes = ["COLLIDE", "FRESH"].into_iter();
    let out = insert_with_collision_retry::<String, String>(
        5,
        || Ok(codes.next().unwrap().to_string()),
        accept_unless(HashSet::from(["COLLIDE".to_string()])),
    );
    assert!(matches!(out, Ok(ref c) if c == "FRESH"), "got {out:?}");
}

#[test]
fn first_success_wastes_no_further_codes() {
    let calls = Cell::new(0usize);
    let out = insert_with_collision_retry::<String, String>(
        5,
        || {
            calls.set(calls.get() + 1);
            Ok(format!("CODE{}", calls.get()))
        },
        |code| AttemptOutcome::Inserted(code.to_string()),
    );
    assert_eq!(out.unwrap(), "CODE1");
    assert_eq!(
        calls.get(),
        1,
        "a single success must not generate more codes"
    );
}

#[test]
fn repeated_collisions_exhaust_the_budget() {
    let out = insert_with_collision_retry::<String, String>(
        3,
        || Ok("DUP".to_string()),
        accept_unless(HashSet::from(["DUP".to_string()])),
    );
    assert!(
        matches!(out, Err(RetryGiveUp::Exhausted { attempts: 3 })),
        "got {out:?}"
    );
}

#[test]
fn a_fatal_attempt_aborts_immediately() {
    let calls = Cell::new(0usize);
    let out = insert_with_collision_retry::<String, String>(
        5,
        || {
            calls.set(calls.get() + 1);
            Ok("ANY".to_string())
        },
        |_code| AttemptOutcome::Fatal("boom".to_string()),
    );
    assert!(
        matches!(out, Err(RetryGiveUp::Fatal(ref e)) if e == "boom"),
        "got {out:?}"
    );
    assert_eq!(calls.get(), 1, "a fatal error must not retry");
}

#[test]
fn a_generator_error_is_fatal() {
    let out = insert_with_collision_retry::<String, String>(
        5,
        || Err("rng down".to_string()),
        |code| AttemptOutcome::Inserted(code.to_string()),
    );
    assert!(
        matches!(out, Err(RetryGiveUp::Fatal(ref e)) if e == "rng down"),
        "got {out:?}"
    );
}

// ============================================================================
// real-DB injection (a genuine SQLite UNIQUE violation drives the retry)
// ============================================================================

#[test]
fn injected_collision_is_retried_against_a_real_unique_violation() {
    let db = Db::open_in_memory().unwrap();

    // Seed a row that OWNS the user_code "COLLIDE".
    let stored = db
        .insert_device_code_retrying("hash-1", EXP, 0, NOW, 8, || Ok("COLLIDE".to_string()))
        .unwrap();
    assert_eq!(stored, "COLLIDE");

    // Inject [COLLIDE, FRESH]: the first candidate hits the real UNIQUE
    // constraint on device_codes.user_code, so the retry advances to FRESH.
    let mut seq = ["COLLIDE", "FRESH"].into_iter();
    let stored2 = db
        .insert_device_code_retrying("hash-2", EXP, 0, NOW, 8, || {
            Ok(seq.next().unwrap().to_string())
        })
        .unwrap();
    assert_eq!(stored2, "FRESH");

    // Both rows persist (the retry inserted a real second row).
    assert!(db.device_code_snapshot("hash-1").unwrap().is_some());
    assert!(db.device_code_snapshot("hash-2").unwrap().is_some());
}

#[test]
fn a_permanently_colliding_generator_exhausts_to_an_error_not_a_panic() {
    let db = Db::open_in_memory().unwrap();
    db.insert_device_code_retrying("hash-a", EXP, 0, NOW, 8, || Ok("DUP".to_string()))
        .unwrap();

    // A generator that never advances past the taken code exhausts the budget and
    // returns an Internal error (never a 500-triggering raw DB error, never a hang).
    let err = db
        .insert_device_code_retrying("hash-b", EXP, 0, NOW, 3, || Ok("DUP".to_string()))
        .unwrap_err();
    assert!(matches!(err, AccountdError::Internal(_)), "got {err:?}");
}

#[test]
fn a_device_code_hash_pk_collision_is_fatal_never_retried() {
    let db = Db::open_in_memory().unwrap();
    db.insert_device_code_retrying("dup-hash", EXP, 0, NOW, 8, || Ok("UC1".to_string()))
        .unwrap();

    // Re-inserting the SAME device_code_hash violates the PRIMARY KEY — which is a
    // UNIQUE violation but NOT on user_code, so it must be fatal (a Db error), and
    // the generator must run EXACTLY ONCE (no retry storm). This is the mutation
    // kill for an over-broad collision classifier: were the PK collision treated
    // as retryable, the generator would run repeatedly and exhaust instead.
    let calls = Cell::new(0usize);
    let err = db
        .insert_device_code_retrying("dup-hash", EXP, 0, NOW, 8, || {
            calls.set(calls.get() + 1);
            Ok(format!("UC-fresh-{}", calls.get()))
        })
        .unwrap_err();
    assert!(matches!(err, AccountdError::Db(_)), "got {err:?}");
    assert_eq!(calls.get(), 1, "a PK collision is fatal and must not retry");
}
