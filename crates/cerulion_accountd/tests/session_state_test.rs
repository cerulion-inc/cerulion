// SPDX-License-Identifier: AGPL-3.0-only
//! Oracle-vector tests for the session + refresh validity decisions
//! (`authenticate_session` / `evaluate_refresh` / `reuse_window_open`) and the
//! rotation derivation (`derive_rotated_pair`). Pure: no server, no DB.

use cerulion_accountd::{
    authenticate_session, derive_rotated_pair, evaluate_refresh, reuse_window_open,
    RefreshDecision, SessionAuth, SessionSnapshot,
};

const S: u64 = 1_000_000_000;
const NOW: u64 = 1_000 * S;

fn snap(revoked: bool, session_exp: u64, refresh_exp: u64) -> SessionSnapshot {
    SessionSnapshot {
        revoked,
        session_expires_at_ns: session_exp,
        refresh_expires_at_ns: refresh_exp,
    }
}

// -- session authentication --------------------------------------------------

#[test]
fn live_session_is_valid() {
    let s = snap(false, NOW + 60 * S, NOW + 3600 * S);
    assert_eq!(authenticate_session(&s, NOW), SessionAuth::Valid);
}

#[test]
fn expired_session_is_expired() {
    let s = snap(false, NOW - S, NOW + 3600 * S);
    assert_eq!(authenticate_session(&s, NOW), SessionAuth::Expired);
}

#[test]
fn session_expiry_boundary_is_exclusive() {
    // now == session_expires_at → expired.
    assert_eq!(
        authenticate_session(&snap(false, NOW, NOW + 3600 * S), NOW),
        SessionAuth::Expired
    );
    // one ns before → valid.
    assert_eq!(
        authenticate_session(&snap(false, NOW + 1, NOW + 3600 * S), NOW),
        SessionAuth::Valid
    );
}

#[test]
fn revoked_session_is_revoked_even_within_window() {
    // Revocation dominates expiry: a revoked-but-unexpired session is Revoked.
    let s = snap(true, NOW + 60 * S, NOW + 3600 * S);
    assert_eq!(authenticate_session(&s, NOW), SessionAuth::Revoked);
}

// -- refresh -----------------------------------------------------------------

#[test]
fn live_refresh_is_ok() {
    // The session token can be expired while the refresh window is still live.
    let s = snap(false, NOW - S, NOW + 3600 * S);
    assert_eq!(evaluate_refresh(&s, NOW), RefreshDecision::Ok);
}

#[test]
fn expired_refresh_requires_relogin() {
    let s = snap(false, NOW - S, NOW - S);
    assert_eq!(evaluate_refresh(&s, NOW), RefreshDecision::Expired);
}

#[test]
fn refresh_expiry_boundary_is_exclusive() {
    assert_eq!(
        evaluate_refresh(&snap(false, NOW - S, NOW), NOW),
        RefreshDecision::Expired
    );
    assert_eq!(
        evaluate_refresh(&snap(false, NOW - S, NOW + 1), NOW),
        RefreshDecision::Ok
    );
}

#[test]
fn revoked_refresh_is_revoked_even_within_window() {
    let s = snap(true, NOW + 60 * S, NOW + 3600 * S);
    assert_eq!(evaluate_refresh(&s, NOW), RefreshDecision::Revoked);
}

// -- refresh reuse window ------------------------------------------------------

#[test]
fn the_reuse_window_is_open_strictly_before_rotation_plus_window() {
    let rotated = NOW;
    let w = 60 * S;
    assert!(reuse_window_open(rotated, rotated, w));
    assert!(reuse_window_open(rotated, rotated + w - 1, w));
    assert!(!reuse_window_open(rotated, rotated + w, w));
    assert!(!reuse_window_open(rotated, rotated + 3600 * S, w));
}

#[test]
fn a_zero_window_is_never_open_even_on_a_clock_step_back() {
    assert!(!reuse_window_open(NOW, NOW, 0));
    assert!(!reuse_window_open(NOW, NOW + 1, 0));
    assert!(!reuse_window_open(NOW, NOW - 5 * S, 0));
}

#[test]
fn a_clock_step_back_is_bounded_by_the_window() {
    let w = 60 * S;
    // A request whose clock read lands just before the rotation stays inside.
    assert!(reuse_window_open(NOW, NOW - 5 * S, w));
    assert!(reuse_window_open(NOW, NOW - w + 1, w));
    // A clock stepped back by the window or more is outside.
    assert!(!reuse_window_open(NOW, NOW - w, w));
    assert!(!reuse_window_open(NOW, NOW - 600 * S, w));
}

#[test]
fn the_rotated_pair_matches_independent_hmac_sha256_vectors() {
    // Expected values computed outside this crate (Python hmac + hashlib):
    // base64url_nopad(hmac_sha256(key=[0x11; 32], label || [0x5A; 32] || "presented-refresh-token")).
    let (session, refresh) =
        derive_rotated_pair(&[0x11; 32], &[0x5A; 32], "presented-refresh-token");
    assert_eq!(session, "xVr1T5Ez0XgLLtbjPjxQh8DT5sNp6DwnJGARjIGTe7A");
    assert_eq!(refresh, "dSMRoHQuVBBz5lqb1FQd0Rf9i-OxJrM22U7QbhkY7jk");
}

#[test]
fn the_rotated_pair_depends_on_key_salt_and_token() {
    let base = derive_rotated_pair(&[9; 32], &[1; 32], "tok");
    assert_eq!(base, derive_rotated_pair(&[9; 32], &[1; 32], "tok"));
    assert_ne!(base, derive_rotated_pair(&[8; 32], &[1; 32], "tok"));
    assert_ne!(base, derive_rotated_pair(&[9; 32], &[2; 32], "tok"));
    assert_ne!(base, derive_rotated_pair(&[9; 32], &[1; 32], "tok2"));
    assert_ne!(base.0, base.1, "the two tokens use distinct labels");
}
