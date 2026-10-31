// SPDX-License-Identifier: AGPL-3.0-only
//! Session + refresh token semantics (pure decisions).
//!
//! A login mints an opaque **session token** (short-lived — minutes) and an
//! opaque **refresh token** (days-scale). Only their SHA-256 hashes are stored
//! ([`crate::hash::hash_token`]). A session authenticates the account-service API;
//! a refresh mints a fresh session (and rotates the refresh) while the refresh
//! window is live; a revoke kills both server-side (an I4 enforcement point).
//! The refresh token a rotation replaced is accepted once more within
//! [`crate::ServiceConfig::refresh_reuse_window_ns`] of that rotation, and returns
//! the pair the rotation issued ([`reuse_window_open`], [`derive_rotated_pair`]).
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

/// Whether a refresh token replaced at `rotated_at_ns` may still be replayed at
/// `now_ns` under a reuse window of `window_ns`: true while `now_ns` is less than
/// `window_ns` away from the rotation, on either side. A `window_ns` of `0` is
/// never open. The two-sided distance keeps a concurrent request whose `now_ns`
/// was read just before the rotation committed inside the window, and bounds a
/// clock stepped backwards to the same `window_ns`.
pub fn reuse_window_open(rotated_at_ns: u64, now_ns: u64, window_ns: u64) -> bool {
    now_ns.abs_diff(rotated_at_ns) < window_ns
}

/// Domain label of the session token a rotation derives.
const ROTATED_SESSION_LABEL: &[u8] = b"cerulion-accountd/v1/rotated-session\0";
/// Domain label of the refresh token a rotation derives.
const ROTATED_REFRESH_LABEL: &[u8] = b"cerulion-accountd/v1/rotated-refresh\0";

/// HMAC-SHA-256 (RFC 2104) of the concatenated `parts` under a 32-byte `key`.
fn hmac_sha256(key: &[u8; 32], parts: &[&[u8]]) -> [u8; 32] {
    use sha2::{Digest, Sha256};
    const BLOCK: usize = 64;
    let mut ipad = [0x36u8; BLOCK];
    let mut opad = [0x5cu8; BLOCK];
    for (i, k) in key.iter().enumerate() {
        ipad[i] ^= k;
        opad[i] ^= k;
    }
    let mut inner = Sha256::new();
    inner.update(ipad);
    for part in parts {
        inner.update(part);
    }
    let inner = inner.finalize();
    let mut outer = Sha256::new();
    outer.update(opad);
    outer.update(inner);
    outer.finalize().into()
}

/// The `(session_token, refresh_token)` pair a rotation issues for the presented
/// refresh token: each is base64url (no padding) of
/// `HMAC-SHA-256(key, label || salt || presented_refresh)`, with a distinct label
/// per token, a fresh random 32-byte `salt` per rotation (stored on the session
/// row) and a random 32-byte `key` held only in the service's memory.
///
/// Recomputing the pair needs all three: the row's salt, the replaced refresh
/// token (the row stores only its hash) and the in-memory key. A replay inside
/// the reuse window therefore returns exactly the pair the rotation issued, while
/// a copy of the database, even together with the replaced token, yields no
/// usable token. A service restart draws a new key, so a rotation from before the
/// restart is not replayable.
pub fn derive_rotated_pair(
    key: &[u8; 32],
    salt: &[u8; 32],
    presented_refresh: &str,
) -> (String, String) {
    use base64::engine::general_purpose::URL_SAFE_NO_PAD;
    use base64::Engine;
    let derive = |label: &[u8]| {
        URL_SAFE_NO_PAD.encode(hmac_sha256(
            key,
            &[label, salt, presented_refresh.as_bytes()],
        ))
    };
    (derive(ROTATED_SESSION_LABEL), derive(ROTATED_REFRESH_LABEL))
}
