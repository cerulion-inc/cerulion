// SPDX-License-Identifier: AGPL-3.0-only
//! Flood-suppressed reporting for the PUBLISH-side frame-REJECT arms
//! of `rmw_publish_serialized_message`.
//!
//! The suppression POLICY is the repo's ONE shared [`FailureRegimeLatch`]
//! in `cerulion_core`. This module is the thin reporting layer that maps its decisions
//! onto the `tracing` events `rmw_publish_serialized_message` emits — the
//! publish-side sibling of `cerulion_core`'s take-side
//! [`frame_drop_latch`](cerulion_core::transport::frame_drop_latch) and of this
//! crate's [`decode_failure_latch`](crate::decode_failure_latch).
//!
//! # Why (the defect)
//!
//! `rmw_publish_serialized_message` REJECTS a caller-supplied frame on two
//! adjacent arms, each of which was a BARE per-call `error!`:
//!
//! * **malformed wire header** — the buffer is too short to hold the 32-byte
//!   [`WireHeader`](cerulion_core::wire::WireHeader), so it is not a Cerulion
//!   frame at all;
//! * **schema-hash mismatch** — it IS a Cerulion frame, of the WRONG type for
//!   this publisher.
//!
//! It is the per-MESSAGE publish entry point. Rosbag-class callers (`ros2 bag
//! play`, any serialized republisher) hit it once per frame, so neither
//! condition is a one-shot: a bag recorded against an older message definition
//! rejects EVERY frame it replays, at frame rate, until somebody re-records or
//! redeploys. On a 100 Hz topic that is ~100 lines/s ≈ 860 MB/day — the class
//! that once filled a 234 GB robot disk, and the class the earlier latches
//! already fixed for their own arms.
//!
//! So the first reject of a regime stays LOUD (`error!`, full context and
//! remedy), repeats are downgraded to `debug!` with a running suppressed count,
//! an open regime RE-ANNOUNCES itself loudly at each decade of the running
//! total, and the first ACCEPTED frame reports recovery once at `info!` before
//! re-arming.
//!
//! # Why the vocabulary is publish-side, not a reuse of the take side
//!
//! `frame_drop_latch` says "dropping a frame" — correct there, because a take
//! path has already accepted the frame onto the wire and the consumer is
//! discarding it, with nothing to tell the producer. Here the frame is
//! REJECTED at the door: it never reaches the topic, and the caller is told
//! synchronously (`RMW_RET_INVALID_ARGUMENT`). Telling a bag-player operator
//! that frames are being "dropped" would point them at the subscriber end of a
//! problem that is entirely on theirs. The levels differ too — the take-side
//! hash arm is a `warn!`, this one keeps the site's earlier `error!`,
//! because a rejected publish is a caller-visible failed call, not a silently
//! discarded input.
//!
//! # Why there is no `Site` parameter (contrast the take side)
//!
//! [`FrameDropSite`](cerulion_core::transport::frame_drop_latch::FrameDropSite)
//! exists because the take side has three sites — a topic message, a service
//! request, a service response — that must log under DIFFERENT structured
//! field names (`topic=` vs `service=`); an operator greps by that key, so
//! collapsing them onto a generic `name=` breaks every such query.
//!
//! The publish-reject condition has exactly ONE site: rmw declares
//! `rmw_publish_serialized_message` and no serialized-request twin, so a
//! rejected serialized frame is always a topic message. A one-variant site
//! enum here would be a discriminator that discriminates nothing and a
//! `field_name()` that can only ever return one value — a misleading name
//! rather than a contract. The field-name contract is instead satisfied by
//! CONSTRUCTION — `topic=` is written literally in all EIGHT line kinds this
//! module emits (loud head / decade re-announcement / suppressed repeat /
//! recovery, for each of the two conditions) — and pinned by
//! `rmw_publish_reject_test::reject_lines_log_the_topic_under_topic`,
//! which drives a regime past the decade boundary for BOTH conditions so all
//! eight are reachable by its oracle. The two decade arms are ALSO key-asserted
//! by their own tests — `an_open_header_reject_regime_re_announces_at_the_decade_at_error`
//! and its `..._hash_..._at_error` twin — because a re-announcement is read by
//! the operator who missed the head and is therefore the line most likely to be
//! grepped alone. If rmw ever grows a second serialized-publish site, that is
//! when the enum earns its place.
//!
//! # The running total is `total_failures=`, the same key at every latch site
//!
//! This module's arrival unified it. THREE reporting modules build on the shared machine, and
//! between them they had spelled that one number FOUR ways —
//! `total_mismatches=` on the hash arms and `total_drops=` on the framing arms
//! of `cerulion_core`'s
//! [`frame_drop_latch`](cerulion_core::transport::frame_drop_latch),
//! `total_failures=` in [`decode_failure_latch`](crate::decode_failure_latch),
//! and `total_rejects=` here — which contradicts the very argument these
//! modules make for per-site `topic=`/`service=` keys: operators grep by key,
//! and "how bad has this got?" should not need four queries to answer across
//! one robot's logs. The SITE is what the message text and the `kind=` field
//! say; the running total of a [`FailureRegimeLatch`] is the same quantity
//! everywhere, so it carries the shared machine's own noun.
//!
//! # Two latches, not one
//!
//! The two conditions do NOT share a latch. They are different problems with
//! different remedies — "your bytes are not a frame" (re-record / stop feeding
//! us CDR) versus "your frame is the wrong type" (re-record against the
//! current definition / redeploy both ends) — and one open regime must never
//! swallow the other's loud head. Same rule, same reason, as
//! `SubscriptionData`'s hash-vs-decode split on the take side.

use cerulion_core::transport::failure_regime_latch::{
    lock_regime_latch, FailureRegimeLatch, RegimeDecision,
};
use std::sync::Mutex;

/// The noun for the thing being rejected, carried as the `kind` field so a
/// reader can tell this apart from the take-side reports at a glance.
const KIND: &str = "serialized message";

/// Report ONE serialized frame rejected because its buffer cannot hold a wire
/// header. Loud on the first of a regime and at each decade of the running
/// total, `debug!` in between.
///
/// `buffer_len` and `header_size` are logged because the PAIR is the whole
/// diagnosis: a caller sees immediately whether they passed an UNFILLED message
/// (`buffer_len=0` — an uninitialised `rmw_serialized_message_t` reaches this
/// arm too; see the null-buffer note in `rmw_publish_serialized_message`), a
/// truncated frame, or a buffer in some other format entirely. They therefore
/// ride EVERY arm, the decade re-announcement included: that line exists for the
/// operator who MISSED the loud head, so serving them a thinner field set than
/// the head carried defeats its purpose.
pub fn report_malformed_header(latch: &Mutex<FailureRegimeLatch>, topic: &str, buffer_len: usize) {
    let mut latch = lock_regime_latch(latch);
    let decision = latch.on_failure();
    let total = latch.total_failures();
    let header_size = cerulion_core::wire::WireHeader::SIZE;
    match decision {
        RegimeDecision::Loud => tracing::error!(
            topic = %topic,
            kind = KIND,
            buffer_len,
            header_size,
            total_failures = total,
            "REJECTING a serialized frame whose wire header is malformed — the \
             buffer is too short to hold the Cerulion wire header, so it is not a Cerulion \
             frame at all and nothing was published. This entry point accepts ONLY frames \
             produced by `rmw_serialize` on this rmw implementation; a CDR buffer recorded \
             under a different rmw, a truncated one, or an UNFILLED serialized message \
             (`buffer_len=0`) lands here. Repeats are suppressed to debug until a \
             well-formed frame arrives."
        ),
        RegimeDecision::StillFailing { total, suppressed } => tracing::error!(
            topic = %topic,
            kind = KIND,
            buffer_len,
            header_size,
            suppressed,
            total_failures = total,
            "malformed serialized frames are STILL being rejected — the running \
             total has crossed another decade since the last loud report. Nothing is being \
             published on this topic. Re-record the source, or feed frames produced by \
             `rmw_serialize` on this rmw implementation."
        ),
        RegimeDecision::Suppressed { suppressed } => tracing::debug!(
            topic = %topic,
            kind = KIND,
            buffer_len,
            header_size,
            suppressed,
            total_failures = total,
            "malformed-header reject suppressed (regime still open)"
        ),
    }
}

/// Report a serialized frame whose wire header PARSED, closing any open
/// malformed-header regime. One `info!` iff the closed regime suppressed
/// something; a healthy caller pays one uncontended lock and a branch.
///
/// Recovery is genuinely observable here: the site keeps accepting frames, so
/// the next one carrying a readable header IS the recovery observation, and it
/// is reported at that exact point — BEFORE the hash gate, which is a
/// different condition with its own latch.
pub fn report_header_intact(latch: &Mutex<FailureRegimeLatch>, topic: &str) {
    let mut latch = lock_regime_latch(latch);
    if let Some(suppressed) = latch.on_success() {
        let total = latch.total_failures();
        tracing::info!(
            topic = %topic,
            kind = KIND,
            suppressed_count = suppressed,
            total_failures = total,
            "serialized frames carry a well-formed wire header again"
        );
    }
}

/// Report ONE serialized frame rejected because its `schema_hash` disagrees
/// with the publisher's type.
///
/// Both hashes are logged because the pair is the diagnosis: it names the type
/// the publisher was created with and the one the frame was serialized
/// against, which is what an operator needs to find the stale end.
pub fn report_schema_hash_reject(
    latch: &Mutex<FailureRegimeLatch>,
    topic: &str,
    expected_hash: u64,
    actual_hash: u64,
) {
    let mut latch = lock_regime_latch(latch);
    let decision = latch.on_failure();
    let total = latch.total_failures();
    match decision {
        RegimeDecision::Loud => tracing::error!(
            topic = %topic,
            kind = KIND,
            expected_hash = format_args!("0x{expected_hash:016X}"),
            actual_hash = format_args!("0x{actual_hash:016X}"),
            total_failures = total,
            "REJECTING a serialized frame whose wire schema hash does not match \
             this publisher's type — the frame was serialized against a DIFFERENT message \
             definition, so publishing it would put a wrong-typed frame on this topic. \
             Nothing was published. Serialize with the type the publisher was created \
             with, or re-record / redeploy BOTH ends from the same schemas. Repeats are \
             suppressed to debug until a matching frame arrives."
        ),
        RegimeDecision::StillFailing { total, suppressed } => tracing::error!(
            topic = %topic,
            kind = KIND,
            expected_hash = format_args!("0x{expected_hash:016X}"),
            actual_hash = format_args!("0x{actual_hash:016X}"),
            suppressed,
            total_failures = total,
            "schema-hash rejects are STILL refusing every serialized frame — the \
             running total has crossed another decade since the last loud report. Nothing \
             is being published on this topic. Re-record / redeploy BOTH ends from the \
             same schemas."
        ),
        RegimeDecision::Suppressed { suppressed } => tracing::debug!(
            topic = %topic,
            kind = KIND,
            expected_hash = format_args!("0x{expected_hash:016X}"),
            actual_hash = format_args!("0x{actual_hash:016X}"),
            suppressed,
            total_failures = total,
            "schema-hash reject suppressed (regime still open)"
        ),
    }
}

/// Report a serialized frame whose hash MATCHED, closing any open hash-reject
/// regime. One `info!` iff the closed regime suppressed something.
pub fn report_schema_hash_accepted(latch: &Mutex<FailureRegimeLatch>, topic: &str) {
    let mut latch = lock_regime_latch(latch);
    if let Some(suppressed) = latch.on_success() {
        let total = latch.total_failures();
        tracing::info!(
            topic = %topic,
            kind = KIND,
            suppressed_count = suppressed,
            total_failures = total,
            "serialized frames match the publisher type again"
        );
    }
}
