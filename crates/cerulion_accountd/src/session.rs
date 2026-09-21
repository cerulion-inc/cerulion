// SPDX-License-Identifier: AGPL-3.0-only
//! Session + refresh token semantics (pure decisions).
//!
//! A login mints an opaque **session token** (short-lived — minutes) and an
//! opaque **refresh token** (days-scale). Only their SHA-256 hashes are stored
//! ([`crate::hash::hash_token`]). A session authenticates the account-service API;
//! a refresh mints a fresh session (and rotates the refresh) while the refresh
//! window is live; a revoke kills both server-side (an I4 enforcement point).
//!
//! The validity verdicts are pure functions over a snapshot so every arm — valid,
//! expired, revoked — is oracle-testable without a database.

/// The fields the session/refresh decisions read (a snapshot of one session row).
#[derive(Clone, Debug)]
pub struct SessionSnapshot {
    /// Whether the session/refresh pair was revoked (server-side kill).
    pub revoked: bool,
    /// Session-token expiry (Unix ns).
    pub session_expires_at_ns: u64,
    /// Refresh-token expiry (Unix ns).
    pub refresh_expires_at_ns: u64,
}

/// The verdict of authenticating a presented session token.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SessionAuth {
    /// The session is live.
    Valid,
    /// The session token expired (refresh required).
    Expired,
    /// The session was revoked.
    Revoked,
}

/// Authenticate a session by its snapshot. Revocation dominates expiry (a revoked
/// session is revoked even if it also happens to be within its window).
pub fn authenticate_session(snap: &SessionSnapshot, now_ns: u64) -> SessionAuth {
    if snap.revoked {
        return SessionAuth::Revoked;
    }
    if now_ns >= snap.session_expires_at_ns {
        return SessionAuth::Expired;
    }
    SessionAuth::Valid
}

/// The verdict of a refresh request.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RefreshDecision {
    /// The refresh window is live — mint a fresh session (+ rotate the refresh).
    Ok,
    /// The refresh token expired — a full re-login is required.
    Expired,
    /// The session/refresh pair was revoked.
    Revoked,
}

/// Decide whether a refresh may proceed. Revocation dominates expiry.
pub fn evaluate_refresh(snap: &SessionSnapshot, now_ns: u64) -> RefreshDecision {
    if snap.revoked {
        return RefreshDecision::Revoked;
    }
    if now_ns >= snap.refresh_expires_at_ns {
        return RefreshDecision::Expired;
    }
    RefreshDecision::Ok
}
