// SPDX-License-Identifier: AGPL-3.0-only
//! Oracle-vector tests for the session + refresh validity decisions
//! (`authenticate_session` / `evaluate_refresh`). Pure — no server, no DB.

use cerulion_accountd::{
    authenticate_session, evaluate_refresh, RefreshDecision, SessionAuth, SessionSnapshot,
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
