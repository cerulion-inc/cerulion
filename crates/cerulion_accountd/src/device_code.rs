// SPDX-License-Identifier: AGPL-3.0-only
//! The RFC 8628 device-authorization-grant state machine (pure).
//!
//! A device code is minted `Pending`. A browser login (magic-link or OAuth)
//! carrying the matching `user_code` transitions it to `Authorized(user_id)`. The
//! first poll that observes `Authorized` issues the token pair and marks it
//! `Consumed`. The polling verdict — pending / slow-down / expired / redeemed /
//! authorized — is decided here as a pure function over a snapshot, so every arm
//! is oracle-testable without a database.

/// The authorization state of a device code.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DeviceCodeState {
    /// Minted, awaiting browser authorization.
    Pending,
    /// A user authorized it via the browser; the next poll redeems it.
    Authorized {
        /// The authorizing user's id.
        user_id: String,
    },
    /// Already redeemed for a token pair (single-use).
    Consumed,
}

/// The fields the poll decision reads (a snapshot of one device-code row).
#[derive(Clone, Debug)]
pub struct DeviceCodeSnapshot {
    /// Current authorization state.
    pub state: DeviceCodeState,
    /// Expiry instant (Unix ns).
    pub expires_at_ns: u64,
    /// Minimum seconds between polls (RFC 8628 `interval`; `0` = no gate).
    pub interval_secs: u64,
    /// The previous poll instant, if any (Unix ns).
    pub last_poll_at_ns: Option<u64>,
}

/// The verdict of a poll.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PollOutcome {
    /// Not yet authorized — keep polling.
    AuthorizationPending,
    /// Polled sooner than `interval` after the last poll — back off.
    SlowDown,
    /// The device code expired.
    Expired,
    /// The device code was already redeemed.
    AlreadyRedeemed,
    /// Authorized — issue tokens for this user and mark the code consumed.
    Authorized {
        /// The authorizing user's id.
        user_id: String,
    },
}

/// Decide a poll outcome. Precedence (each short-circuits the rest):
/// 1. **Expired** — a dead code is dead regardless of state.
/// 2. **AlreadyRedeemed** — a consumed code is single-use.
/// 3. **Authorized** — a ready token is returned immediately (a fast poll is NOT
///    slowed down once authorization is complete — better client UX, and the
///    slow-down gate exists to pace *pending* polling, not to withhold a ready
///    grant).
/// 4. **SlowDown** — a still-pending code polled faster than `interval`.
/// 5. **AuthorizationPending** — a pending code polled at a legal cadence.
pub fn poll_outcome(snap: &DeviceCodeSnapshot, now_ns: u64) -> PollOutcome {
    if now_ns >= snap.expires_at_ns {
        return PollOutcome::Expired;
    }
    if matches!(snap.state, DeviceCodeState::Consumed) {
        return PollOutcome::AlreadyRedeemed;
    }
    if let DeviceCodeState::Authorized { user_id } = &snap.state {
        return PollOutcome::Authorized {
            user_id: user_id.clone(),
        };
    }
    // Still pending: enforce the poll interval.
    if let Some(last) = snap.last_poll_at_ns {
        let interval_ns = snap.interval_secs.saturating_mul(1_000_000_000);
        if now_ns.saturating_sub(last) < interval_ns {
            return PollOutcome::SlowDown;
        }
    }
    PollOutcome::AuthorizationPending
}

/// Whether a poll should advance the interval gate's `last_poll_at`. A `SlowDown`
/// poll must NOT — recording a throttled poll resets the timer, permanently
/// locking out a client that polls just inside the interval (each SlowDown pushes
/// the reference forward). Every other outcome records normally so the next poll
/// is interval-gated.
pub fn should_record_poll(outcome: &PollOutcome) -> bool {
    !matches!(outcome, PollOutcome::SlowDown)
}

// ============================================================================
// user_code collision-retry driver (pure)
// ============================================================================

/// The classification of one "generate a code, try to insert it" attempt.
pub enum AttemptOutcome<T, E> {
    /// Inserted successfully, carrying the value produced.
    Inserted(T),
    /// The generated code collided with an existing one — retry with a fresh code.
    Collision,
    /// A non-retryable error — abort the loop immediately.
    Fatal(E),
}

/// Why a bounded collision-retry loop gave up.
#[derive(Debug)]
pub enum RetryGiveUp<E> {
    /// The attempt budget was exhausted by repeated collisions.
    Exhausted {
        /// How many attempts were made.
        attempts: usize,
    },
    /// A non-collision error aborted the loop.
    Fatal(E),
}

/// Drive a bounded "generate a candidate code, try to insert it" retry — the
/// `device/start` `user_code` allocator's control flow, isolated so it is
/// oracle-testable without a database (inject a colliding candidate and assert
/// the retry advances to a fresh one). For up to `max_attempts` iterations:
/// generate a candidate with `gen`, run `attempt`; on `Collision` retry with a
/// fresh candidate, on `Fatal` abort, on `Inserted` return immediately (a single
/// success wastes no further codes). A `gen` error is itself fatal.
pub fn insert_with_collision_retry<T, E>(
    max_attempts: usize,
    mut gen: impl FnMut() -> Result<String, E>,
    mut attempt: impl FnMut(&str) -> AttemptOutcome<T, E>,
) -> Result<T, RetryGiveUp<E>> {
    for _ in 0..max_attempts {
        let code = match gen() {
            Ok(c) => c,
            Err(e) => return Err(RetryGiveUp::Fatal(e)),
        };
        match attempt(&code) {
            AttemptOutcome::Inserted(v) => return Ok(v),
            AttemptOutcome::Collision => continue,
            AttemptOutcome::Fatal(e) => return Err(RetryGiveUp::Fatal(e)),
        }
    }
    Err(RetryGiveUp::Exhausted {
        attempts: max_attempts,
    })
}
