// SPDX-License-Identifier: AGPL-3.0-only
//! Oracle-vector tests for the RFC 8628 device-code poll state machine
//! (`poll_outcome`). Pure — no server, no DB. Each case is a hand-written
//! (snapshot, now) → expected-outcome oracle, never a self-compare.

use cerulion_accountd::{
    poll_outcome, should_record_poll, DeviceCodeSnapshot, DeviceCodeState, PollOutcome,
};

const S: u64 = 1_000_000_000; // one second in ns
const NOW: u64 = 1_000 * S; // an arbitrary fixed "now"

fn snap(
    state: DeviceCodeState,
    expires_at_ns: u64,
    interval_secs: u64,
    last_poll_at_ns: Option<u64>,
) -> DeviceCodeSnapshot {
    DeviceCodeSnapshot {
        state,
        expires_at_ns,
        interval_secs,
        last_poll_at_ns,
    }
}

fn authorized(user: &str) -> DeviceCodeState {
    DeviceCodeState::Authorized {
        user_id: user.to_string(),
    }
}

#[test]
fn pending_within_window_first_poll_is_pending() {
    let s = snap(DeviceCodeState::Pending, NOW + 60 * S, 5, None);
    assert_eq!(poll_outcome(&s, NOW), PollOutcome::AuthorizationPending);
}

#[test]
fn pending_polled_too_fast_is_slow_down() {
    // last poll 2s ago, interval 5s → too fast.
    let s = snap(DeviceCodeState::Pending, NOW + 60 * S, 5, Some(NOW - 2 * S));
    assert_eq!(poll_outcome(&s, NOW), PollOutcome::SlowDown);
}

#[test]
fn pending_polled_after_interval_is_pending() {
    // last poll 6s ago, interval 5s → legal cadence.
    let s = snap(DeviceCodeState::Pending, NOW + 60 * S, 5, Some(NOW - 6 * S));
    assert_eq!(poll_outcome(&s, NOW), PollOutcome::AuthorizationPending);
}

#[test]
fn pending_at_exact_interval_boundary_is_pending() {
    // Exactly `interval` since the last poll → allowed (`< interval` is the gate).
    let s = snap(DeviceCodeState::Pending, NOW + 60 * S, 5, Some(NOW - 5 * S));
    assert_eq!(poll_outcome(&s, NOW), PollOutcome::AuthorizationPending);
}

#[test]
fn interval_zero_never_slows_down() {
    // interval 0 disables the gate even when polled at the same instant.
    let s = snap(DeviceCodeState::Pending, NOW + 60 * S, 0, Some(NOW));
    assert_eq!(poll_outcome(&s, NOW), PollOutcome::AuthorizationPending);
}

#[test]
fn authorized_within_window_returns_the_token_immediately() {
    let s = snap(authorized("user-42"), NOW + 60 * S, 5, None);
    assert_eq!(
        poll_outcome(&s, NOW),
        PollOutcome::Authorized {
            user_id: "user-42".to_string()
        }
    );
}

#[test]
fn authorized_beats_slow_down() {
    // A ready token is returned even when the client polled faster than interval —
    // the slow-down gate paces PENDING polling, it does not withhold a ready grant.
    let s = snap(authorized("user-7"), NOW + 60 * S, 5, Some(NOW - S));
    assert_eq!(
        poll_outcome(&s, NOW),
        PollOutcome::Authorized {
            user_id: "user-7".to_string()
        }
    );
}

#[test]
fn consumed_is_already_redeemed() {
    let s = snap(DeviceCodeState::Consumed, NOW + 60 * S, 5, None);
    assert_eq!(poll_outcome(&s, NOW), PollOutcome::AlreadyRedeemed);
}

#[test]
fn expired_dominates_pending() {
    let s = snap(DeviceCodeState::Pending, NOW, 5, None);
    assert_eq!(poll_outcome(&s, NOW), PollOutcome::Expired); // now >= expires_at
}

#[test]
fn expired_dominates_authorized() {
    // A code authorized but past expiry is Expired, never Authorized — expiry is
    // checked first.
    let s = snap(authorized("user-1"), NOW - S, 5, None);
    assert_eq!(poll_outcome(&s, NOW), PollOutcome::Expired);
}

#[test]
fn expired_dominates_consumed() {
    let s = snap(DeviceCodeState::Consumed, NOW - S, 5, None);
    assert_eq!(poll_outcome(&s, NOW), PollOutcome::Expired);
}

#[test]
fn only_a_slowdown_poll_is_not_recorded() {
    // The gate advances (records `last_poll_at`) on EVERY outcome except SlowDown —
    // recording a throttled poll would reset the timer and lock out a client polling
    // just inside the interval.
    assert!(!should_record_poll(&PollOutcome::SlowDown));
    assert!(should_record_poll(&PollOutcome::AuthorizationPending));
    assert!(should_record_poll(&PollOutcome::Expired));
    assert!(should_record_poll(&PollOutcome::AlreadyRedeemed));
    assert!(should_record_poll(&PollOutcome::Authorized {
        user_id: "u".to_string()
    }));
}

#[test]
fn expiry_boundary_is_exclusive() {
    // now == expires_at → expired (the window is `[created, expires)`).
    let at = snap(DeviceCodeState::Pending, NOW, 0, None);
    assert_eq!(poll_outcome(&at, NOW), PollOutcome::Expired);
    // one ns before → still pending.
    let before = snap(DeviceCodeState::Pending, NOW + 1, 0, None);
    assert_eq!(
        poll_outcome(&before, NOW),
        PollOutcome::AuthorizationPending
    );
}
