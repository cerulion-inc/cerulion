// SPDX-License-Identifier: AGPL-3.0-only
//! The repo's shared flood-suppression state machine.
//!
//! Cerulion already carried FOUR hand-written copies of one policy —
//! [`DrainWarnLatch`](crate::graph::drain_latch::DrainWarnLatch),
//! [`OutputDiscardLatch`](super::output_discard_latch::OutputDiscardLatch),
//! [`NotifyDeliveryLatch`](super::notify_delivery_latch::NotifyDeliveryLatch)
//! and `rmw_cerulion`'s `DecodeFailureLatch`. The hash-mismatch work
//! needed a fifth, for the per-frame schema-hash-mismatch arms; instead of
//! copying the machine again this module IS the machine, and `DecodeFailureLatch`
//! was rebased onto it (see `rmw_cerulion::decode_failure_latch`), so the count
//! of independent implementations went DOWN.
//!
//! The three older latches are deliberately left alone: each is heavily pinned
//! by its own oracle suite, and re-plumbing them buys nothing this change
//! needs. New sites should build on this type.
//!
//! # The policy
//!
//! A repeating failure on a per-message path is a log-volume hazard, not just
//! a nuisance: one per-publish `warn!` filled a 234 GB robot disk, and a
//! 100 Hz topic whose every frame fails is ~100 lines/s forever. So:
//!
//! * the FIRST failure of a regime is LOUD ([`RegimeDecision::Loud`]) — full
//!   context and remedy, because a condition nobody is told about is a
//!   Principle #2 violation;
//! * repeats are DOWNGRADED ([`RegimeDecision::Suppressed`]) and carry a
//!   running count of what has been suppressed so far in this regime;
//! * an open regime RE-ANNOUNCES itself loudly at each DECADE of the running
//!   total ([`RegimeDecision::StillFailing`] — see below);
//! * RECOVERY (the first success after a failure) reports ONCE, and only when
//!   the closed regime actually suppressed something — a lone failure that
//!   heals immediately re-arms SILENTLY, or an every-other-message flapper
//!   would simply double its log volume;
//! * a recovery always RE-ARMS, so a fresh breakage is loud again;
//! * and [`total_failures`](FailureRegimeLatch::total_failures) counts EVERY
//!   failure unconditionally, never reset by recovery — the Principle #3
//!   queryability signal that survives the loud head scrolling away and the
//!   repeats being filtered out at `info`.
//!
//! This type decides only WHICH of those a given observation is; it holds no
//! transport state and emits no logs. Callers map the decision onto `tracing`
//! events, which is what lets the policy be oracle-tested with no transport,
//! no SHM and no mock (`crates/cerulion_core/tests/failure_regime_latch_test.rs`).
//!
//! # Decade re-announcement — why the loud arm re-opens without a recovery
//!
//! Suppressing repeats is only safe while the running total stays reachable.
//! At the `cerulion_core` sites it is: `ServiceClient::schema_mismatch_count`
//! and `ServiceServer::schema_mismatch_count` are ordinary Rust accessors. At
//! the `rmw_cerulion` sites it is NOT — the counters live on
//! `SubscriptionData` / `ServiceData` behind an opaque `*mut c_void` the rmw C
//! ABI hands to rclcpp/rclpy, the ABI is STANDARDIZED (a new accessor cannot
//! be added), and the only reader today is a test that casts the pointer back.
//! So a ROS user's ENTIRE window onto the condition is the log, and "one line,
//! hours ago, then silence forever" is not an accurate report of a regime that
//! is still dropping every frame.
//!
//! The fix is deliberately CLOCK-FREE (a clock would make the emission
//! sequence wall-dependent, and Principle #7 wants a decision derivable from
//! the observation stream alone): while a regime is open, the failure that
//! takes the running total across a power of ten — the 10th, 100th, 1000th, …
//! — is re-announced at the LOUD level carrying that total. The cost is
//! bounded by `log10(total)`: a topic that drops every frame at 1 kHz for a
//! week is ~6.05e8 failures, which is one head plus re-announcements at
//! 10¹…10⁸ — NINE lines, not 600 million.
//!
//! A re-announced failure was NOT downgraded, so it does not bump
//! `suppressed`; the recovery line therefore still reports exactly how many
//! failures the operator never saw.
//!
//! # What this latch does NOT bound: interleaved writers
//!
//! The regime is keyed on the OBSERVING entity, not on the peer that produced
//! the frame. On a topic carrying frames from a healthy writer and a skewed
//! one INTERLEAVED, every good frame closes the regime (recovery) and the next
//! bad one opens a fresh one (loud head) — so the loud volume tracks the
//! number of REGIMES, which in the worst alternating case is one loud line per
//! bad frame. The latch bounds SAME-WRITER floods (the shipping shape: a
//! single-writer iceoryx2 topic, or a service peer that is uniformly skewed),
//! not interleaved ones.
//!
//! This is a deliberate accepted residual, not an oversight: distinguishing
//! writers would need per-writer state keyed on something the wire header does
//! not carry, and the recovery line is what makes the "it healed" claim
//! true. The decade rule bounds the interleaved case too, but only in the
//! weaker sense that it is the REGIME COUNT (not the failure count) that sets
//! the volume.
//!
//! # State machine
//!
//! ```text
//!             on_failure() -> Loud
//!   Healthy ─────────────────────────► Failing (suppressed = 0)
//!      ▲                                  │  on_failure() -> Suppressed { suppressed += 1 }
//!      │ on_success() -> recovery         │  ...or StillFailing at each decade
//!      │   Some(n) iff n > 0,             │     of total_failures (10, 100, …)
//!      │   else None (silent re-arm)      │
//!      └──────────────────────────────────┘
//!
//!   Healthy + on_success() -> None   (steady state: one predictable branch)
//! ```
//!
//! # Keying
//!
//! One latch per OBSERVING ENTITY **and CONDITION** — a publisher port, a
//! subscription, a service client, a service server, each with a separate
//! latch per distinct failure condition it can observe. Every such entity
//! binds exactly one topic/service name for its lifetime, so per-entity keying
//! IS per-topic keying, with no map, no lookup on the message path, and
//! nothing that can grow without bound. A keyed `HashMap<topic, latch>` would
//! add exactly that unbounded-growth question for no gain.
//!
//! Conditions must NOT share a latch: an open schema-hash regime sharing a
//! latch with the decode-failure regime would swallow the other's loud head,
//! and the two have different remedies (redeploy the disagreeing TYPE vs
//! redeploy the disagreeing BUILD).

/// The base of the re-announcement ladder: an open regime re-announces at each
/// power of this value (10th, 100th, 1000th … failure). See the module docs.
///
/// `pub(crate)` so a consumer's oracle can DERIVE the burst length that reaches
/// the first boundary instead of typing `10`. A literal there would
/// go on compiling if this base ever moved, and would silently stop covering the
/// `StillFailing` arm — the arm measured as the easiest of the
/// three to leave unpinned.
pub(crate) const DECADE_BASE: u64 = 10;

/// What the caller should do with one observed failure, as decided by
/// [`FailureRegimeLatch::on_failure`].
///
/// A typed enum rather than a bare `bool` because the arms are trivially
/// invertible and the inversion is SILENT in both directions: an always-loud
/// latch re-opens the disk-fill hazard, and a never-loud one hides the
/// condition entirely. Call sites read as `RegimeDecision::Loud` instead of an
/// easily-negated `already_suppressed` flag — the same surface the sibling
/// latches' level enums exist to pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegimeDecision {
    /// First failure of a regime: log LOUDLY at the site's own level (the
    /// existing `warn!`/`error!`), with full context and the remedy.
    Loud,
    /// A repeat while the regime is open: log at `tracing::debug!`, carrying
    /// the number of failures DOWNGRADED so far in this regime. The loud head
    /// is deliberately not counted — it was not suppressed.
    Suppressed {
        /// Failures downgraded since this regime opened.
        suppressed: u64,
    },
    /// A repeat that took the running total across a DECADE (10, 100, 1000, …)
    /// while the regime was open: log at the SAME loud level as
    /// [`Loud`](Self::Loud), stating that the regime is still open and how
    /// large it has grown.
    ///
    /// This exists because the unconditional counter is not reachable at the
    /// rmw sites (standardized C ABI — see the module docs), so the log is the
    /// operator's only window. It is NOT counted as suppressed: the operator
    /// saw it.
    StillFailing {
        /// The running total this failure took across the decade boundary.
        total: u64,
        /// Failures downgraded so far in this regime (unchanged by this one).
        suppressed: u64,
    },
}

/// The shared pure flood-suppression state machine. One per observing entity
/// AND condition (see the module docs' keying section).
#[derive(Debug)]
pub struct FailureRegimeLatch {
    /// True while a regime is open (≥1 failure since the last success).
    failing: bool,
    /// Failures downgraded since this regime opened. Neither the loud head nor
    /// a decade re-announcement is counted — they were not suppressed. Reset
    /// at each regime open and at recovery.
    suppressed: u64,
    /// UNCONDITIONAL running total across ALL regimes — bumped on every
    /// [`on_failure`](Self::on_failure) regardless of state, NEVER reset by
    /// recovery. See the module docs.
    total_failures: u64,
    /// The running total at which the NEXT decade re-announcement is due (10,
    /// then 100, …). Monotone across regimes, because the total it tracks is
    /// never reset either — a recovery must not buy a skewed peer a fresh
    /// quota of nine silent failures per regime.
    next_decade: u64,
}

impl Default for FailureRegimeLatch {
    /// Identical to [`new`](Self::new).
    ///
    /// Hand-written rather than derived on purpose: a derived `Default` would
    /// zero `next_decade`, and `total >= 0` is true of EVERY failure, so the
    /// suppressed arm would be unreachable and the latch would flood — the
    /// exact defect it exists to prevent, produced by a one-word omission.
    fn default() -> Self {
        Self::new()
    }
}

impl FailureRegimeLatch {
    /// Construct a latch in the Healthy state.
    pub const fn new() -> Self {
        Self {
            failing: false,
            suppressed: 0,
            total_failures: 0,
            next_decade: DECADE_BASE,
        }
    }

    /// Record one failure and return how the caller should log it.
    #[inline]
    pub fn on_failure(&mut self) -> RegimeDecision {
        // UNCONDITIONAL: independent of the log-level regime (Principle #3).
        // Cold path only — a healthy entity never reaches this.
        self.total_failures += 1;
        let crossed_decade = self.total_failures >= self.next_decade;
        if crossed_decade {
            self.advance_decade();
        }
        if !self.failing {
            // A head is loud anyway; the decade it may have consumed is still
            // spent, so the ladder keeps marching.
            self.failing = true;
            self.suppressed = 0;
            RegimeDecision::Loud
        } else if crossed_decade {
            RegimeDecision::StillFailing {
                total: self.total_failures,
                suppressed: self.suppressed,
            }
        } else {
            self.suppressed += 1;
            RegimeDecision::Suppressed {
                suppressed: self.suppressed,
            }
        }
    }

    /// Move `next_decade` strictly past the running total.
    ///
    /// One `on_failure` can only cross one boundary, but the loop keeps the
    /// field correct for any future caller that bumps the total by more than
    /// one, and the saturation break makes an overflow impossible.
    #[inline]
    fn advance_decade(&mut self) {
        while self.next_decade <= self.total_failures {
            let advanced = self.next_decade.saturating_mul(DECADE_BASE);
            if advanced == self.next_decade {
                // Saturated at u64::MAX — unreachable in practice (it needs
                // 1.8e19 failures) but must not spin.
                break;
            }
            self.next_decade = advanced;
        }
    }

    /// Record one success. Always re-arms the loud path. Returns
    /// `Some(suppressed)` — the caller logs recovery ONCE — only when the
    /// regime it closed actually downgraded at least one failure; a
    /// lone-failure regime re-arms silently (see the module docs).
    #[inline]
    pub fn on_success(&mut self) -> Option<u64> {
        if !self.failing {
            return None;
        }
        self.failing = false;
        let suppressed = self.suppressed;
        self.suppressed = 0;
        if suppressed > 0 {
            Some(suppressed)
        } else {
            None
        }
    }

    /// Running total of failures on this entity across all regimes —
    /// independent of log level, never reset by recovery.
    #[inline]
    pub fn total_failures(&self) -> u64 {
        self.total_failures
    }

    /// True while a regime is open (the last observation was a failure).
    #[inline]
    pub fn is_failing(&self) -> bool {
        self.failing
    }
}

/// Lock a latch that lives behind a `Mutex`, recovering from poisoning.
///
/// A DIAGNOSTIC latch must never wedge or fail the path it observes: the state
/// it protects is four integers with no cross-field invariant a panic could
/// tear, so a poisoned value is always usable. (Contrast the transport
/// mutexes, where the callers deliberately fail the call — those guard torn
/// iceoryx2 state.)
///
/// Entities that own their latch by value (`&mut self` message paths, e.g.
/// [`ServiceServer`](super::service::ServiceServer)) need no `Mutex` and never
/// call this.
pub fn lock_regime_latch(
    latch: &std::sync::Mutex<FailureRegimeLatch>,
) -> std::sync::MutexGuard<'_, FailureRegimeLatch> {
    latch.lock().unwrap_or_else(|e| e.into_inner())
}
