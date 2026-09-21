// SPDX-License-Identifier: AGPL-3.0-only
//! Flood-suppressed reporting for the per-frame DROP arms on the take paths.
//!
//! The suppression policy itself lives in [`FailureRegimeLatch`];
//! this module is the thin reporting layer that maps its decisions onto the
//! `tracing` events the drop sites emit, so every one of them says the same
//! thing in the same shape.
//!
//! # Why (the defect)
//!
//! A frame a consumer cannot accept is DROPPED, and every drop site logged a
//! BARE per-frame `warn!`. Two conditions are covered here:
//!
//! * **schema-hash mismatch** — the wire `schema_hash` does not match the
//!   consumer's type. The two ends disagree about the message DEFINITION:
//!   [`ServiceClient::try_take_one_response`](super::service::ServiceClient::try_take_one_response),
//!   [`ServiceServer::try_take_one_request`](super::service::ServiceServer::try_take_one_request),
//!   and `rmw_cerulion`'s `rmw_take` arm.
//! * **short envelope** — the hash MATCHED but the frame is too small to hold
//!   the [`ServiceEnvelope`](super::service::ServiceEnvelope). The two ends
//!   agree on the type and disagree on the FRAMING, which is the same class of
//!   version skew with a different remedy, so it gets its own latch (see the
//!   keying note below).
//!
//! Neither is a transient: both fail EVERY frame until somebody redeploys. On
//! a 100 Hz skewed topic that is ~100 lines/s ≈ 860 MB/day — the same class
//! that filled a 234 GB robot disk once, and the same class two sibling latches
//! fixed for their own arms. So the first drop of a regime stays loud
//! with its full context and remedy, repeats are downgraded to `debug!` with a
//! running suppressed count, an open regime RE-ANNOUNCES itself loudly at each
//! decade of the running total, and the first acceptable frame reports
//! recovery once at `info!` before re-arming.
//!
//! # The counter is the part that survives filtering
//!
//! Suppressing repeats is only safe because
//! [`FailureRegimeLatch::total_failures`](super::failure_regime_latch::FailureRegimeLatch::total_failures)
//! keeps counting them unconditionally, independent of log level (Principle
//! #3). The entity-level accessors
//! ([`ServiceClient::schema_mismatch_count`](super::service::ServiceClient::schema_mismatch_count),
//! [`ServiceServer::schema_mismatch_count`](super::service::ServiceServer::schema_mismatch_count))
//! surface it, so "is this consumer discarding everything?" is answerable at
//! `RUST_LOG=error` and hours after the loud head scrolled away.
//!
//! At the rmw sites that accessor CANNOT exist (the rmw C ABI is standardized;
//! the counter is reachable only by casting the opaque entity pointer back,
//! which only a test does) — which is exactly why the shared machine
//! re-announces an open regime at each decade. See the
//! [`failure_regime_latch`](super::failure_regime_latch) module docs.
//!
//! In the LOG that total is `total_failures=`, at every consumer of the shared
//! machine. It used to differ. THREE reporting modules build on that machine
//! (this one, `rmw_cerulion`'s `decode_failure_latch`, and its
//! `publish_reject_latch`), and between them they had spelled the one number
//! FOUR ways: `total_mismatches=` on this module's hash arms, `total_drops=` on
//! its framing arms, `total_failures=` in the decode reporters, `total_rejects=`
//! in the publish-reject ones. Four spellings of one quantity contradict the
//! very argument made below for per-site `topic=`/`service=` keys — operators
//! grep by key. Which CONDITION a line reports is carried by its message text
//! and its `kind=` field; the running total is the same quantity everywhere, so
//! it takes the shared machine's own noun.
//!
//! # Keying
//!
//! One latch per consuming entity **and condition**, held on the entity itself
//! — and each entity binds exactly ONE topic or service name, so that is
//! per-topic keying with no map and nothing unbounded. Conditions do not share
//! a latch: an open hash-mismatch regime must not swallow the framing regime's
//! loud head, because the remedies differ.

use super::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};

/// Which consuming entity observed the drop — chooses the noun in the log line
/// (so an operator can tell a topic frame from a service request or response
/// without reading the call site) AND the structured FIELD NAME the name is
/// logged under.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameDropSite {
    /// A frame taken on a topic subscription (`rmw_take*`). Logs under
    /// `topic=`.
    Message,
    /// A request taken by a service server. Logs under `service=`.
    ServiceRequest,
    /// A response taken by a service client. Logs under `service=`.
    ServiceResponse,
}

impl FrameDropSite {
    /// The noun for the dropped thing, carried as the `kind` field.
    pub const fn noun(self) -> &'static str {
        match self {
            Self::Message => "message",
            Self::ServiceRequest => "service request",
            Self::ServiceResponse => "service response",
        }
    }

    /// The structured field name this site's `name` is logged under —
    /// `"topic"` for messages, `"service"` for service requests/responses.
    ///
    /// Informational only: `tracing` field keys must be literals, so the
    /// reporters below cannot pass this value to the macro; they match on the
    /// site with one arm per key. This accessor exists so the contract is
    /// stated once and can be oracle-tested — logging everything under a
    /// generic `name=` would break every operator query that greps `topic=` or
    /// `service=`, which is exactly the regression this pins.
    pub const fn field_name(self) -> &'static str {
        match self {
            Self::Message => "topic",
            Self::ServiceRequest | Self::ServiceResponse => "service",
        }
    }
}

/// Expand the three-way decision for a SCHEMA-HASH mismatch under one
/// structured field name.
///
/// `tracing` field keys must be literals, so the `topic=`-vs-`service=` split
/// cannot be a runtime value — the reporter matches on the site and invokes
/// this once per key. Keeping the whole `match` inside the macro means the
/// message text and the field set exist exactly once.
macro_rules! emit_hash_mismatch {
    ($field:ident, $name:expr, $noun:expr, $expected:expr, $actual:expr, $total:expr, $decision:expr) => {
        match $decision {
            RegimeDecision::Loud => tracing::warn!(
                $field = %$name,
                kind = $noun,
                expected_hash = format_args!("0x{:016X}", $expected),
                actual_hash = format_args!("0x{:016X}", $actual),
                total_failures = $total,
                "dropping a frame whose wire schema hash does not match this \
                 consumer's type — the two ends disagree on the message definition. \
                 Rebuild and redeploy BOTH ends from the same schemas. Repeats are \
                 suppressed to debug until a matching frame arrives."
            ),
            RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
                $field = %$name,
                kind = $noun,
                expected_hash = format_args!("0x{:016X}", $expected),
                actual_hash = format_args!("0x{:016X}", $actual),
                suppressed,
                total_failures = total,
                "schema-hash mismatch STILL dropping every frame — the running \
                 total has crossed another decade since the last loud report. Rebuild \
                 and redeploy BOTH ends from the same schemas."
            ),
            RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                $field = %$name,
                kind = $noun,
                expected_hash = format_args!("0x{:016X}", $expected),
                actual_hash = format_args!("0x{:016X}", $actual),
                suppressed,
                total_failures = $total,
                "schema-hash mismatch suppressed (regime still open)"
            ),
        }
    };
}

/// Recovery line for a closed SCHEMA-HASH regime, under one field name.
macro_rules! emit_hash_recovery {
    ($field:ident, $name:expr, $noun:expr, $suppressed:expr, $total:expr) => {
        tracing::info!(
            $field = %$name,
            kind = $noun,
            suppressed_count = $suppressed,
            total_failures = $total,
            "schema hashes match again"
        )
    };
}

/// Report ONE dropped frame whose `schema_hash` did not match the consumer's
/// expected hash. Loud on the first of a regime and at each decade of the
/// running total, `debug!` in between.
///
/// `name` is the topic or service name — logged under `topic=` or `service=`
/// per [`FrameDropSite::field_name`]. Both hashes are logged because the pair
/// is the diagnosis: it names which type the consumer expected and which one
/// actually arrived, which is what an operator needs to find the skewed end.
pub fn report_schema_hash_mismatch(
    latch: &mut FailureRegimeLatch,
    site: FrameDropSite,
    name: &str,
    expected_hash: u64,
    actual_hash: u64,
) {
    let decision = latch.on_failure();
    let total = latch.total_failures();
    let noun = site.noun();
    match site {
        FrameDropSite::Message => {
            emit_hash_mismatch!(
                topic,
                name,
                noun,
                expected_hash,
                actual_hash,
                total,
                decision
            )
        }
        FrameDropSite::ServiceRequest | FrameDropSite::ServiceResponse => {
            emit_hash_mismatch!(
                service,
                name,
                noun,
                expected_hash,
                actual_hash,
                total,
                decision
            )
        }
    }
}

/// Report a frame that PASSED the hash gate, closing any open mismatch regime.
///
/// Recovery is genuinely observable at every mismatch site: each one sits on a
/// take path that keeps running, so the next frame carrying the expected hash
/// IS the recovery observation, and it is reported at that exact point (before
/// any later decode step, which is a different condition with its own latch).
/// Emits one `info!` iff the closed regime actually suppressed something; a
/// healthy consumer pays one predictable branch.
pub fn report_schema_hash_match(latch: &mut FailureRegimeLatch, site: FrameDropSite, name: &str) {
    if let Some(suppressed) = latch.on_success() {
        let total = latch.total_failures();
        let noun = site.noun();
        match site {
            FrameDropSite::Message => emit_hash_recovery!(topic, name, noun, suppressed, total),
            FrameDropSite::ServiceRequest | FrameDropSite::ServiceResponse => {
                emit_hash_recovery!(service, name, noun, suppressed, total)
            }
        }
    }
}

/// Expand the three-way decision for a SHORT-ENVELOPE drop under one field
/// name. Same literal-field-key reason as `emit_hash_mismatch`.
macro_rules! emit_short_envelope {
    ($field:ident, $name:expr, $noun:expr, $size:expr, $total:expr, $decision:expr) => {
        match $decision {
            RegimeDecision::Loud => tracing::warn!(
                $field = %$name,
                kind = $noun,
                size_bytes = $size,
                total_failures = $total,
                "dropping a frame too short to hold the service envelope — the \
                 schema hash MATCHED, so the two ends agree on the type and disagree on \
                 the FRAMING. Rebuild and redeploy BOTH ends from the same build. \
                 Repeats are suppressed to debug until a well-formed frame arrives."
            ),
            RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
                $field = %$name,
                kind = $noun,
                size_bytes = $size,
                suppressed,
                total_failures = total,
                "short-envelope drops STILL happening on every frame — the \
                 running total has crossed another decade since the last loud report. \
                 Rebuild and redeploy BOTH ends from the same build."
            ),
            RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                $field = %$name,
                kind = $noun,
                size_bytes = $size,
                suppressed,
                total_failures = $total,
                "short-envelope drop suppressed (regime still open)"
            ),
        }
    };
}

/// Recovery line for a closed SHORT-ENVELOPE regime, under one field name.
macro_rules! emit_envelope_recovery {
    ($field:ident, $name:expr, $noun:expr, $suppressed:expr, $total:expr) => {
        tracing::info!(
            $field = %$name,
            kind = $noun,
            suppressed_count = $suppressed,
            total_failures = $total,
            "service envelopes parse again"
        )
    };
}

/// Report ONE dropped frame that passed the hash gate but was too short to
/// hold a [`ServiceEnvelope`](super::service::ServiceEnvelope).
///
/// The framing-skew twin of [`report_schema_hash_mismatch`], on its OWN latch
/// (see the module docs' keying note): the hash gate agreeing while the
/// framing disagrees is a different condition with a different remedy, and one
/// open regime must never swallow the other's loud head.
///
/// # The `Message` arm is unreachable, and KEPT for match totality
///
/// Only a service frame carries a [`ServiceEnvelope`](super::service::ServiceEnvelope),
/// so every call site of this function and of [`report_envelope_intact`]
/// passes [`FrameDropSite::ServiceRequest`] or [`FrameDropSite::ServiceResponse`];
/// the `Message` arm below (and its twin in the recovery reporter) can never
/// run in production and is deliberately not tested.
///
/// It is not deletable: the two reporters match over the SHARED
/// [`FrameDropSite`], which the hash reporter genuinely needs all three
/// variants of, so dropping the arm makes the match non-exhaustive. The
/// alternatives — a second, narrower site enum for the envelope condition, or
/// a runtime `unreachable!()` on a diagnostic path — are a type-design change
/// with its own call-site churn and its own oracle to write, deliberately out
/// of this change's scope rather than smuggled in beside it. Named here so the
/// arm reads as a documented consequence of sharing one enum, not as an
/// untested code path nobody noticed.
pub fn report_short_envelope(
    latch: &mut FailureRegimeLatch,
    site: FrameDropSite,
    name: &str,
    size_bytes: usize,
) {
    let decision = latch.on_failure();
    let total = latch.total_failures();
    let noun = site.noun();
    match site {
        FrameDropSite::Message => {
            emit_short_envelope!(topic, name, noun, size_bytes, total, decision)
        }
        FrameDropSite::ServiceRequest | FrameDropSite::ServiceResponse => {
            emit_short_envelope!(service, name, noun, size_bytes, total, decision)
        }
    }
}

/// Report a frame whose envelope PARSED, closing any open short-envelope
/// regime. One `info!` iff the closed regime suppressed something.
///
/// Its [`FrameDropSite::Message`] arm is unreachable for the same reason, and
/// kept for the same reason — see [`report_short_envelope`].
pub fn report_envelope_intact(latch: &mut FailureRegimeLatch, site: FrameDropSite, name: &str) {
    if let Some(suppressed) = latch.on_success() {
        let total = latch.total_failures();
        let noun = site.noun();
        match site {
            FrameDropSite::Message => emit_envelope_recovery!(topic, name, noun, suppressed, total),
            FrameDropSite::ServiceRequest | FrameDropSite::ServiceResponse => {
                emit_envelope_recovery!(service, name, noun, suppressed, total)
            }
        }
    }
}
