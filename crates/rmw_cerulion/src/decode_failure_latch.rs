// SPDX-License-Identifier: AGPL-3.0-only
//! Flood-suppression latch for the rmw take path's DECODE-failure signal
//! (a frame that passed the schema-hash gate but that the bridge cannot decode).
//!
//! The suppression POLICY itself is the repo's shared
//! [`FailureRegimeLatch`], which this type wraps: a pure, transport-free state machine, so
//! the policy is unit-testable with no subscriber / SHM / rosidl mock. This module
//! originally hand-wrote that machine here — the fourth copy in the repo —
//! and a later change hoisted it into `cerulion_core` rather than adding a fifth for
//! the schema-hash-mismatch arms. This module keeps the DECODE-failure
//! vocabulary (the levels, the site nouns, the message text and its remedy) on
//! top of it.
//!
//! **The hoist changed one thing about the behaviour, deliberately:** an open
//! regime now RE-ANNOUNCES itself loudly when the running total crosses a
//! power of ten ([`DecodeFailureLevel::ErrorStillFailing`]). Everything else —
//! the loud head, the `debug!` repeats, the suppressed-count math, the
//! recovery rule, the unconditional total — is unchanged, and the oracle suite
//! below is the proof (only `sustained_regime_…` moved, to assert the new
//! bounded ladder instead of "exactly one error over 1000").
//!
//! # Why this exists
//!
//! `BridgedMessage::unflatten` returns a bare `bool`. Every rmw take site
//! branched on it and, on `false`, simply DID NOT SET `taken` — no `warn!`, no
//! `error!`, no counter. The receive returned `RMW_RET_OK` with `taken =
//! false`, which is the SAME observable an empty queue produces, so a frame
//! the bridge could not decode was indistinguishable from no frame at all.
//! Contrast the schema-hash-mismatch arm a few lines above each site, which
//! has always warned. A subscriber silently discarding every frame on a live
//! topic is the worst shape of Principle #2 violation: the data is there, the
//! decode is wrong, and nothing says so.
//!
//! This matters most exactly when it is hardest to debug. The variable-nested
//! element framing the rmw bridge converged on is INVISIBLE to `schema_hash` (the recipe
//! folds no `DynamicArray` element layout — `MessageSchema::schema_hash`), so
//! an old-publisher/new-subscriber pair passes the hash gate and then fails to
//! decode. Without this signal that skew is silent in BOTH directions.
//!
//! # Why a latch rather than a bare `error!`
//!
//! A framing skew fails EVERY frame, not one: a 100 Hz topic emits 100 error
//! lines/s per subscription, and one per-publish `warn!` flood filled a
//! 234 GB disk on a Go2. So the first failure of a regime is loud
//! (`error!`, full context + remedy), sustained failures downgrade to `debug!`
//! carrying a running suppressed count, and RECOVERY (the next frame that does
//! decode) reports once at `info!` before re-arming — so a fresh breakage is
//! loud again. A lone failure that heals immediately re-arms SILENTLY: there
//! was no flood to announce the end of.
//!
//! # State machine
//!
//! ```text
//!             on_failure() -> Error
//!   Healthy ─────────────────────────► Failing (suppressed = 0)
//!      ▲                                  │  on_failure() -> Debug { suppressed += 1 }
//!      │ on_success() -> recovery         │    (repeat failures downgraded)
//!      │   Some(total) iff total > 0      │  ...or ErrorStillFailing at each decade
//!      │   (caller logs recovery @info),  │     of total_failures (10, 100, …)
//!      │   else None (silent re-arm)      │
//!      └──────────────────────────────────┘
//!
//!   Healthy + on_success() -> None   (steady state: branch-only, no log)
//! ```
//!
//! One latch per take-side entity (subscription / service server / service
//! client), so there is no key lookup. The happy-path
//! [`on_success`](DecodeFailureLatch::on_success) is a single predictable
//! branch returning `None`; the `error!` / `debug!` / `info!` events are
//! constructed only on the cold failure / recovery transitions.

use cerulion_core::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};

/// Level the caller should log a decode failure at, as decided by
/// [`DecodeFailureLatch::on_failure`].
///
/// A typed enum rather than a bare bool so call sites read as
/// `DecodeFailureLevel::Error` / `Debug { .. }` instead of an easily-inverted
/// `already_suppressed` flag — the same inversion-regression surface its
/// siblings exist to pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeFailureLevel {
    /// First failure of a regime: log at `tracing::error!` with the topic /
    /// service name, the type, and the remedy.
    Error,
    /// A repeated failure while the latch is set: log at `tracing::debug!`,
    /// carrying the count of failures SUPPRESSED (downgraded) so far in this
    /// regime. The loud first one is not counted — it was not suppressed.
    Debug {
        /// Number of failures downgraded to `debug!` since the regime opened.
        suppressed: u64,
    },
    /// A repeated failure that took the running total across a DECADE (the
    /// 10th, 100th, 1000th, …) while the regime was open: log at
    /// `tracing::error!` again, stating that the regime is STILL open and how
    /// large it has grown.
    ///
    /// This arm exists because the rmw counters are not reachable from ROS
    /// code — the rmw C ABI is standardized, so `decode_failures` can only be
    /// read by casting the opaque entity pointer back, which only a test does.
    /// The log is therefore the operator's whole window, and "one line, hours
    /// ago, then silence" would misreport a subscription still dropping every
    /// frame. Bounded by `log10(total)`. Not counted as suppressed — the
    /// operator saw it.
    ErrorStillFailing {
        /// The running total this failure took across the decade boundary.
        total: u64,
        /// Failures downgraded so far in this regime (unchanged by this one).
        suppressed: u64,
    },
}

/// Flood-suppression latch for the rmw take path's decode failures. One latch
/// per take-side entity (see module docs).
///
/// A newtype over the repo's shared [`FailureRegimeLatch`] — the decode-failure
/// VOCABULARY (this crate's `Error`/`Debug` levels) over the one shared
/// POLICY, so a change to the suppression rules lands in exactly one place.
/// Holds no transport state and emits no logs itself — callers map the
/// returned decisions onto `tracing` events.
#[derive(Debug, Default)]
pub struct DecodeFailureLatch(FailureRegimeLatch);

impl DecodeFailureLatch {
    /// Construct a latch in the Healthy state.
    pub const fn new() -> Self {
        Self(FailureRegimeLatch::new())
    }

    /// Record a decode failure; returns the level the caller should log it at.
    /// First failure of a regime → [`DecodeFailureLevel::Error`] (the latch
    /// sets); repeats → [`DecodeFailureLevel::Debug`] carrying the running
    /// suppressed count.
    #[inline]
    pub fn on_failure(&mut self) -> DecodeFailureLevel {
        match self.0.on_failure() {
            RegimeDecision::Loud => DecodeFailureLevel::Error,
            RegimeDecision::Suppressed { suppressed } => DecodeFailureLevel::Debug { suppressed },
            RegimeDecision::StillFailing { total, suppressed } => {
                DecodeFailureLevel::ErrorStillFailing { total, suppressed }
            }
        }
    }

    /// Record a SUCCESSFUL decode; always re-arms the loud path (a subsequent
    /// failure is `Error` again). Returns `Some(total_suppressed)` — the
    /// caller logs recovery at `info!` once — ONLY when the closed regime
    /// actually SUPPRESSED at least one failure (`suppressed > 0`).
    ///
    /// A lone-failure regime (one `error!`, immediate recovery, `suppressed ==
    /// 0`) re-arms SILENTLY (returns `None`): the single error already told
    /// the whole story, and reporting recovery there would just double the log
    /// volume of an entity flapping every other frame.
    #[inline]
    pub fn on_success(&mut self) -> Option<u64> {
        self.0.on_success()
    }

    /// Unconditional running total of decode failures on this entity across
    /// all regimes — independent of log level and never reset by recovery.
    /// The Principle #3 queryability signal: a persistently-skewed
    /// subscription whose loud head `error!` has scrolled away and whose
    /// sustained failures are `debug!`-suppressed is otherwise invisible and
    /// uncountable.
    #[inline]
    pub fn total_failures(&self) -> u64 {
        self.0.total_failures()
    }

    /// True while a failure regime is open (the last decode attempt failed).
    #[inline]
    pub fn is_failing(&self) -> bool {
        self.0.is_failing()
    }
}

/// What kind of take-side entity a decode failure happened on — chooses the
/// noun in the log line so an operator can tell a topic frame from a service
/// request/response without reading the call site, AND the structured FIELD
/// NAME the entity's name is logged under.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeSite {
    /// `rmw_take*` on a subscription. Logs under `topic=`.
    Subscription,
    /// `rmw_take_request` on a service server. Logs under `service=`.
    ServiceRequest,
    /// `rmw_take_response` on a service client. Logs under `service=`.
    ServiceResponse,
}

impl DecodeSite {
    const fn noun(self) -> &'static str {
        match self {
            Self::Subscription => "message",
            Self::ServiceRequest => "service request",
            Self::ServiceResponse => "service response",
        }
    }

    /// The structured field name this site's `name` is logged under —
    /// `"topic"` for a subscription, `"service"` for a service request or
    /// response.
    ///
    /// This encodes a fixed inconsistency: this module originally shipped these
    /// reporters emitting a GENERIC `name=` at every site, while the take-side
    /// arms converted right beside them emit `topic=` / `service=`. An
    /// operator watching one subscription greps by that key, so a topic whose
    /// hash-mismatch drops appear under `topic=` and whose decode failures
    /// appear under `name=` is a topic they cannot follow with one query.
    ///
    /// Informational only: `tracing` field keys must be literals, so the
    /// reporters below cannot pass this value to the macro; they match on the
    /// site with one arm per key. This accessor exists so the contract is
    /// stated once and can be oracle-tested — the same shape (and the same
    /// reason) as
    /// [`FrameDropSite::field_name`](cerulion_core::transport::frame_drop_latch::FrameDropSite::field_name).
    pub const fn field_name(self) -> &'static str {
        match self {
            Self::Subscription => "topic",
            Self::ServiceRequest | Self::ServiceResponse => "service",
        }
    }
}

/// Recover the latch even if a previous panic poisoned its mutex.
///
/// A DIAGNOSTIC latch must never wedge or fail the data path it observes: the
/// state it protects is three integers with no cross-field invariant a panic
/// could tear, so the poisoned value is always usable. (Contrast the transport
/// mutexes, where `runtime::lock_unpoisoned` deliberately fails the call —
/// those guard torn iceoryx2 state.)
fn lock_latch(
    latch: &std::sync::Mutex<DecodeFailureLatch>,
) -> std::sync::MutexGuard<'_, DecodeFailureLatch> {
    latch.lock().unwrap_or_else(|e| e.into_inner())
}

/// Expand the three-way decision for a DECODE FAILURE under ONE structured
/// field name.
///
/// `tracing` field keys must be literals, so the `topic=`-vs-`service=` split
/// cannot be a runtime value — the reporter matches on the site and invokes
/// this once per key. Keeping the whole `match` inside the macro means the
/// message text and the field set exist exactly once (the same shape,
/// and the same reason, as `cerulion_core`'s `frame_drop_latch`).
macro_rules! emit_decode_failure {
    ($field:ident, $name:expr, $type_name:expr, $payload_len:expr, $noun:expr, $total:expr, $level:expr) => {
        match $level {
            DecodeFailureLevel::Error => tracing::error!(
                $field = %$name,
                r#type = %$type_name,
                payload_len = $payload_len,
                total_failures = $total,
                kind = $noun,
                "dropping a frame the bridge could not decode — the schema hash \
                 MATCHED, so this is a framing problem the hash cannot see, not a type \
                 mismatch. Two causes reach here. (1) A wire-framing SKEW: the publisher \
                 and this subscriber are running different rmw_cerulion builds — redeploy \
                 BOTH ends from the same build. (2) A native (non-rmw) Cerulion publisher \
                 sent a nested field EXPLICITLY EMPTY (the `set_<field>_bytes(&[])` \
                 idiom), and a variable nested message cannot be decoded from an empty \
                 sub-frame — redeploying will not help; populate the nested field on the \
                 publisher. \
                 Repeats are suppressed to debug until a frame decodes again."
            ),
            DecodeFailureLevel::Debug { suppressed } => tracing::debug!(
                $field = %$name,
                r#type = %$type_name,
                payload_len = $payload_len,
                suppressed,
                total_failures = $total,
                kind = $noun,
                "decode failure suppressed (regime still open)"
            ),
            DecodeFailureLevel::ErrorStillFailing {
                total: crossed,
                suppressed,
            } => tracing::error!(
                $field = %$name,
                r#type = %$type_name,
                payload_len = $payload_len,
                suppressed,
                total_failures = crossed,
                kind = $noun,
                "decode failures are STILL dropping every frame — the running \
                 total has crossed another decade since the last loud report. \
                 The schema hash MATCHED, so this is a wire-framing skew: redeploy BOTH \
                 ends from the same build."
            ),
        }
    };
}

macro_rules! emit_entry_refused {
    ($field:ident, $name:expr, $type_name:expr, $payload_len:expr, $noun:expr, $total:expr,
     $level:expr, $var_idx:expr, $reason:expr) => {
        match $level {
            DecodeFailureLevel::Error => tracing::error!(
                $field = %$name,
                r#type = %$type_name,
                payload_len = $payload_len,
                var_idx = $var_idx,
                reason = %$reason,
                total_failures = $total,
                kind = $noun,
                "refusing a frame before decoding it — the schema hash MATCHED, so this is \
                 not a type mismatch. reason= says which check the entry named by var_idx= \
                 failed. entry_out_of_bounds: its offset-table entry does not resolve inside \
                 the payload. partial_element: its length is not a whole number of elements. \
                 Both mean a MALFORMED frame the wire format forbids — truncated or corrupt, \
                 or a writer building the table by hand — and redeploying both ends does NOT \
                 help unless the producer is an rmw_cerulion build. bound_violated is a \
                 different thing: the entry is well formed, but its element COUNT exceeds the \
                 bound the type declares, so the producer and this subscriber disagree about \
                 the TYPE's bound — check that both were built from the same message \
                 definition. In every case the caller's message was NOT written. Repeats are \
                 suppressed to debug until a frame decodes again."
            ),
            DecodeFailureLevel::Debug { suppressed } => tracing::debug!(
                $field = %$name,
                r#type = %$type_name,
                payload_len = $payload_len,
                var_idx = $var_idx,
                reason = %$reason,
                suppressed,
                total_failures = $total,
                kind = $noun,
                "frame refused before decoding (regime still open) — see reason="
            ),
            DecodeFailureLevel::ErrorStillFailing {
                total: crossed,
                suppressed,
            } => tracing::error!(
                $field = %$name,
                r#type = %$type_name,
                payload_len = $payload_len,
                var_idx = $var_idx,
                reason = %$reason,
                suppressed,
                total_failures = crossed,
                kind = $noun,
                "frames are STILL being refused before decoding — the running total has \
                 crossed another decade since the last loud report. See var_idx= and \
                 reason=: entry_out_of_bounds and partial_element mean the producer emits a \
                 shape the wire format forbids; bound_violated means the two ends disagree \
                 about the type's declared bound."
            ),
        }
    };
}

/// Recovery line for a closed decode-failure regime, under one field name.
/// Same literal-field-key reason as [`emit_decode_failure`].
macro_rules! emit_decode_recovery {
    ($field:ident, $name:expr, $noun:expr, $suppressed:expr, $total:expr) => {
        tracing::info!(
            $field = %$name,
            suppressed_count = $suppressed,
            total_failures = $total,
            kind = $noun,
            "decoding recovered"
        )
    };
}

/// Report a frame that passed the schema-hash gate but that the bridge could
/// not DECODE. Loud on the first failure of a regime, `debug!` while
/// it persists — see the module docs.
///
/// `name` is the topic or service name; `type_name` the ROS type. The message
/// names remedies because the dominant cause is a wire-framing skew that
/// `schema_hash` cannot see (an old-vs-new `rmw_cerulion` on the two ends),
/// whose fix is to redeploy both ends together.
///
/// # Why the text names a SECOND cause
///
/// That is not the only way to reach this arm, and the earlier wording
/// asserted it was. A NATIVE (non-rmw) Cerulion publisher using the documented
/// empty-nested idiom — an EXPLICIT `set_<f>_bytes(&[])` — writes a
/// ZERO-LENGTH offset entry (`(cursor, 0)`: the generated setter always records
/// the running payload cursor, so the entry is well-formed and in bounds, and
/// the field counts as WRITTEN for the publish gate). The frame is entirely
/// legitimate; what fails is one level down. `read_var_entry` hands that entry
/// back as an EMPTY slice, and a VARIABLE nested schema's sub-frame cannot be
/// empty — it must carry at least its own offset table — so
/// `CanonicalBodyReader::new` rejects it as truncated and the frame is dropped
/// here. Both ends are correct and current; "redeploy BOTH ends" is advice that
/// cannot work, sending the operator to rebuild a robot over a publisher-side
/// field it deliberately left empty. The message therefore states the skew as
/// the LIKELY cause with its remedy, and names the empty-nested-field case as
/// the alternative with its own (publisher-side) remedy.
/// Report a frame refused by the take path's READ-ONLY pre-write entry gate
/// (the verdict in `take_gate`), on the SAME latch and regime as
/// [`report_decode_failure`].
///
/// One CONDITION, one regime: the gate refuses exactly what the decode would
/// have refused, one step earlier and without writing — so a frame detected
/// here must not open a second regime that could swallow the decode's own
/// loud head, and both must share one counter. What differs is only the TEXT
/// and two extra fields: this arm knows WHICH member failed and HOW, which
/// the decode's own `warn_bad_entry` used to print and which would otherwise
/// be lost to the gate front-running it.
///
/// The text matters as much as the latching. The shared message enumerates a
/// wire-framing SKEW and an explicitly-empty nested field, and neither is
/// this condition — an offset-table entry that does not resolve, or a
/// sequence length that is not a whole number of elements, is a MALFORMED
/// frame, for which "redeploy both ends" is the wrong advice. Sending an
/// operator after the wrong remedy is exactly what the second-cause wording fixed on this
/// module once already.
pub fn report_decode_entry_refused(
    latch: &std::sync::Mutex<DecodeFailureLatch>,
    site: DecodeSite,
    name: &str,
    type_name: &str,
    payload_len: usize,
    var_idx: usize,
    reason: &'static str,
) {
    let mut latch = lock_latch(latch);
    let level = latch.on_failure();
    let total = latch.total_failures();
    let noun = site.noun();
    match site {
        DecodeSite::Subscription => {
            emit_entry_refused!(
                topic,
                name,
                type_name,
                payload_len,
                noun,
                total,
                level,
                var_idx,
                reason
            )
        }
        DecodeSite::ServiceRequest | DecodeSite::ServiceResponse => {
            emit_entry_refused!(
                service,
                name,
                type_name,
                payload_len,
                noun,
                total,
                level,
                var_idx,
                reason
            )
        }
    }
}

pub fn report_decode_failure(
    latch: &std::sync::Mutex<DecodeFailureLatch>,
    site: DecodeSite,
    name: &str,
    type_name: &str,
    payload_len: usize,
) {
    let mut latch = lock_latch(latch);
    let level = latch.on_failure();
    let total = latch.total_failures();
    let noun = site.noun();
    match site {
        DecodeSite::Subscription => {
            emit_decode_failure!(topic, name, type_name, payload_len, noun, total, level)
        }
        DecodeSite::ServiceRequest | DecodeSite::ServiceResponse => {
            emit_decode_failure!(service, name, type_name, payload_len, noun, total, level)
        }
    }
}

/// Report a SUCCESSFUL decode, closing any open failure regime.
/// Emits one `info!` recovery line iff the closed regime actually suppressed
/// failures; a healthy entity pays one uncontended lock and a branch.
pub fn report_decode_success(
    latch: &std::sync::Mutex<DecodeFailureLatch>,
    site: DecodeSite,
    name: &str,
) {
    let mut latch = lock_latch(latch);
    if let Some(suppressed) = latch.on_success() {
        let total = latch.total_failures();
        let noun = site.noun();
        match site {
            DecodeSite::Subscription => emit_decode_recovery!(topic, name, noun, suppressed, total),
            DecodeSite::ServiceRequest | DecodeSite::ServiceResponse => {
                emit_decode_recovery!(service, name, noun, suppressed, total)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::testing::{debug_level_compiled_in, line_level, lines_at_exclusively};
    // no-env-filter, so the capture sees this crate's own target.
    use tracing_test::traced_test;

    /// The canonical cycle, against a HAND-WRITTEN oracle (never a
    /// self-compare): error → debug(1) → debug(2) → recovery(2) → error.
    #[test]
    fn canonical_error_then_debug_then_recovery_then_rearm() {
        let mut latch = DecodeFailureLatch::new();
        assert!(!latch.is_failing());

        assert_eq!(latch.on_failure(), DecodeFailureLevel::Error);
        assert!(latch.is_failing());
        assert_eq!(
            latch.on_failure(),
            DecodeFailureLevel::Debug { suppressed: 1 }
        );
        assert_eq!(
            latch.on_failure(),
            DecodeFailureLevel::Debug { suppressed: 2 }
        );

        // Recovery reports the two SUPPRESSED failures (not all three).
        assert_eq!(latch.on_success(), Some(2));
        assert!(!latch.is_failing());

        // Re-armed: the next failure is loud again.
        assert_eq!(latch.on_failure(), DecodeFailureLevel::Error);
        assert_eq!(latch.total_failures(), 4);
    }

    /// A lone failure that heals immediately re-arms SILENTLY — no recovery
    /// `info!`. The anti-flood rule for an every-other-frame flapper.
    #[test]
    fn lone_failure_rearms_silently() {
        let mut latch = DecodeFailureLatch::new();
        assert_eq!(latch.on_failure(), DecodeFailureLevel::Error);
        assert_eq!(latch.on_success(), None, "no flood ⇒ no recovery line");
        assert!(!latch.is_failing());
        // Still re-armed even though recovery was silent.
        assert_eq!(latch.on_failure(), DecodeFailureLevel::Error);
        assert_eq!(latch.total_failures(), 2);
    }

    /// Steady state: success on a healthy latch is a branch returning `None`,
    /// forever, and never fabricates a recovery line.
    #[test]
    fn healthy_success_is_silent_and_idempotent() {
        let mut latch = DecodeFailureLatch::new();
        for _ in 0..1000 {
            assert_eq!(latch.on_success(), None);
        }
        assert!(!latch.is_failing());
        assert_eq!(latch.total_failures(), 0);
    }

    /// INVERSION REGRESSION 1 — "always loud". A long failure regime must NOT
    /// emit a loud line per failure. Since the shared latch landed, the loud arm re-opens on a
    /// bounded DECADE ladder, so the hand oracle over 1000 failures is: one
    /// head at #1, re-announcements at #10 / #100 / #1000, everything else
    /// `Debug` — 4 loud, 996 debug. (Before the shared latch this asserted 1 + 999; the
    /// ladder is the deliberate change, and 4-over-1000 is still three orders
    /// of magnitude away from the per-frame flood this exists to prevent.)
    #[test]
    fn sustained_regime_is_a_bounded_decade_ladder_and_the_rest_debug() {
        let mut latch = DecodeFailureLatch::new();
        let mut errors = 0;
        let mut debugs = 0;
        let mut announced_totals = Vec::new();
        // Failures downgraded so far — the hand-tracked mirror of the value
        // the latch is expected to report.
        let mut expected_suppressed = 0u64;
        for i in 1..=1000u64 {
            match latch.on_failure() {
                DecodeFailureLevel::Error => {
                    errors += 1;
                    assert_eq!(i, 1, "only the head may take the plain loud arm");
                }
                DecodeFailureLevel::Debug { suppressed } => {
                    debugs += 1;
                    expected_suppressed += 1;
                    assert_eq!(suppressed, expected_suppressed);
                }
                DecodeFailureLevel::ErrorStillFailing { total, suppressed } => {
                    errors += 1;
                    announced_totals.push(total);
                    assert_eq!(total, i, "the re-announcement carries the running total");
                    assert_eq!(
                        suppressed, expected_suppressed,
                        "a re-announced failure was NOT downgraded, so it must not \
                         bump the suppressed count"
                    );
                }
            }
        }
        assert_eq!(
            announced_totals,
            vec![10, 100, 1000],
            "an open regime re-announces at each decade of the running total"
        );
        assert_eq!(errors, 4, "1 head + 3 decades — never one per failure");
        assert_eq!(debugs, 996);
        assert_eq!(latch.total_failures(), 1000);
        assert_eq!(expected_suppressed, 996);
    }

    /// INVERSION REGRESSION 2 — "never loud". Each regime that is CLOSED by a
    /// success must re-open loud. Three regimes ⇒ three `Error`s.
    #[test]
    fn each_regime_reopens_loud() {
        let mut latch = DecodeFailureLatch::new();
        let mut errors = 0;
        for _ in 0..3 {
            if latch.on_failure() == DecodeFailureLevel::Error {
                errors += 1;
            }
            // Two suppressed repeats, then heal.
            latch.on_failure();
            latch.on_failure();
            assert_eq!(latch.on_success(), Some(2));
        }
        assert_eq!(errors, 3);
        assert_eq!(latch.total_failures(), 9);
    }

    /// The UNCONDITIONAL total is independent of the log-level regime and is
    /// NEVER reset by recovery — the release-safe complement to the
    /// debug-only lines (Principle #3).
    #[test]
    fn total_is_unconditional_and_survives_recovery() {
        let mut latch = DecodeFailureLatch::new();
        latch.on_failure(); // 1 (loud)
        latch.on_failure(); // 2 (suppressed)
        latch.on_failure(); // 3 (suppressed)
        assert_eq!(latch.total_failures(), 3);
        assert_eq!(latch.on_success(), Some(2));
        // Recovery does NOT reset the running total.
        assert_eq!(latch.total_failures(), 3);
        latch.on_failure(); // 4 (loud again)
        assert_eq!(latch.total_failures(), 4);
        // Successes never bump it.
        latch.on_success();
        assert_eq!(latch.total_failures(), 4);
    }

    /// A second consecutive success is a no-op (cannot double-report a
    /// recovery for one regime).
    #[test]
    fn recovery_is_reported_at_most_once_per_regime() {
        let mut latch = DecodeFailureLatch::new();
        latch.on_failure();
        latch.on_failure();
        assert_eq!(latch.on_success(), Some(1));
        assert_eq!(latch.on_success(), None);
        assert_eq!(latch.on_success(), None);
    }

    /// The per-site FIELD-NAME contract, as a hand-written oracle
    /// vector. `topic=` for a topic frame, `service=` for either service
    /// direction — the same keys `cerulion_core`'s `FrameDropSite` uses, so an
    /// operator following ONE subscription greps ONE key across the
    /// hash-mismatch and decode-failure reports on it.
    #[test]
    fn field_name_maps_each_site_to_its_operator_key() {
        let oracle = [
            (DecodeSite::Subscription, "topic"),
            (DecodeSite::ServiceRequest, "service"),
            (DecodeSite::ServiceResponse, "service"),
        ];
        for (site, expected) in oracle {
            assert_eq!(site.field_name(), expected, "wrong key for {site:?}");
        }
    }

    /// The reporters really EMIT under the key
    /// [`DecodeSite::field_name`] promises — on the failure line AND on the
    /// recovery line, for both key groups.
    ///
    /// The accessor above is informational (a `tracing` field key must be a
    /// literal, so the reporters match on the site with one arm per key), which
    /// means the accessor and the emission can drift apart silently. This is
    /// the arm that couples them.
    ///
    /// The negative half is the actual regression: these lines originally shipped
    /// under a generic `name=`, so a subscription whose hash-mismatch
    /// drops logged `topic=` and whose decode failures logged `name=` could not
    /// be followed with one query.
    #[test]
    #[traced_test]
    fn reporters_emit_under_the_field_name_their_site_promises() {
        const TOPIC: &str = "probe/topic";
        const SERVICE: &str = "probe/service";

        // A subscription regime: two failures (so recovery is not silent),
        // then a decode.
        let sub = std::sync::Mutex::new(DecodeFailureLatch::new());
        for _ in 0..2 {
            report_decode_failure(&sub, DecodeSite::Subscription, TOPIC, "pkg/Type", 8);
        }
        report_decode_success(&sub, DecodeSite::Subscription, TOPIC);

        // The same for a service server.
        let srv = std::sync::Mutex::new(DecodeFailureLatch::new());
        for _ in 0..2 {
            report_decode_failure(&srv, DecodeSite::ServiceRequest, SERVICE, "pkg/Srv", 8);
        }
        report_decode_success(&srv, DecodeSite::ServiceRequest, SERVICE);

        logs_assert(|lines: &[&str]| {
            // Level-free twin: the suppressed repeat must never be LOUD. This is
            // the half of the contract that survives `release_max_level_info`,
            // where the gated presence checks below are skipped — without it a
            // suppressed arm promoted to `warn!` passes the release test.
            for level in ["WARN", "INFO", "ERROR"] {
                let loud = lines
                    .iter()
                    .filter(|l| {
                        l.contains("decode failure suppressed") && line_level(l) == Some(level)
                    })
                    .count();
                if loud != 0 {
                    return Err(format!(
                        "the suppressed repeat was emitted at {level} ({loud} line(s)) — \
                         sustained decode failures are downgraded to debug!"
                    ));
                }
            }
            // (marker, its LEVEL, required key=value, the generic form that must be gone)
            let cases = [
                (
                    "dropping a frame the bridge could not decode",
                    "ERROR",
                    TOPIC,
                    "topic",
                ),
                ("decode failure suppressed", "DEBUG", TOPIC, "topic"),
                ("decoding recovered", "INFO", TOPIC, "topic"),
            ];
            for (marker, level, name, key) in cases {
                // The suppressed repeat is a `debug!` line: it does not exist
                // where `debug!` is compiled out (`release_max_level_info`).
                if marker == "decode failure suppressed" && !debug_level_compiled_in() {
                    continue;
                }
                // The level token is matched as well as the marker: a recovery
                // demoted to `debug!` must not pass as a recovery.
                // The ONE shared body: the level read from the line HEADER,
                // AND the level-free total of the same conjunction, so a copy
                // of the line at another level cannot pass unseen.
                let line =
                    *lines_at_exclusively(lines, level, &[marker, &format!("{key}={name}")])?
                        .first()
                        .ok_or_else(|| format!("no {level} `{key}={name}` line for {marker}"))?;
                if line.contains(&format!("name={name}")) {
                    return Err(format!(
                        "the generic `name=` key is the field-name regression — operators \
                         grep by the per-site key: {line}"
                    ));
                }
            }
            for (marker, level) in [
                ("dropping a frame the bridge could not decode", "ERROR"),
                ("decode failure suppressed", "DEBUG"),
                ("decoding recovered", "INFO"),
            ] {
                if marker == "decode failure suppressed" && !debug_level_compiled_in() {
                    continue;
                }
                let line =
                    *lines_at_exclusively(lines, level, &[marker, &format!("service={SERVICE}")])?
                        .first()
                        .ok_or_else(|| format!("no {level} `service=` line for {marker}"))?;
                if line.contains(&format!("name={SERVICE}")) {
                    return Err(format!("the generic `name=` key must be gone: {line}"));
                }
            }
            Ok(())
        });
    }
}
