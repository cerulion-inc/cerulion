// SPDX-License-Identifier: AGPL-3.0-only
//! Configuration, outcome rules and audit inspection for authenticated sessions.

mod inspection;
mod limits;

pub(crate) const MAX_RECORDING_EVENT_BYTES: usize =
    2 * cerulion_ops_proto::session::MAX_SESSION_FRAME_BYTES;

pub use inspection::{
    list_recordings, list_recordings_with_incomplete, verify_recording_file,
    verify_recording_prefix_file, RecordingListing, RecordingPrefixSummary,
};
pub use limits::SessionLimits;
pub(crate) use limits::MAX_REAUTHORIZE_INTERVAL;

use cerud::receipt::ReceiptOutcome;
use cerulion_ops_proto::session::ExitReason;
use std::time::Duration;

/// Classify limits before authorization, with equality already at the limit.
pub fn termination_reason(
    authorized: bool,
    elapsed: Duration,
    idle: Duration,
    duration_limit: Option<Duration>,
    idle_limit: Option<Duration>,
) -> Option<ExitReason> {
    if duration_limit.is_some_and(|cap| elapsed >= cap) || idle_limit.is_some_and(|cap| idle >= cap)
    {
        Some(ExitReason::TimedOut)
    } else if !authorized {
        Some(ExitReason::Revoked)
    } else {
        None
    }
}

/// Return the prefix allowed by the cap and count all observed bytes.
/// Once truncated, the flag stays set even for an empty subsequent chunk.
pub fn delivered_count(bytes: &mut u64, truncated: &mut bool, cap: u64, incoming: usize) -> usize {
    let delivered = cap.saturating_sub(*bytes).min(incoming as u64) as usize;
    *bytes = bytes.saturating_add(incoming as u64);
    *truncated |= delivered < incoming;
    delivered
}

/// Continue natural-exit drain only while no independent stop was observed.
pub fn forward_drained_output(reason: ExitReason, stopped: Option<ExitReason>) -> bool {
    stopped.is_none() && matches!(reason, ExitReason::Exited | ExitReason::CleanupFailed)
}

/// A failed witness dominates status; only ordinary exit earns an OK receipt.
pub fn final_receipt_outcome(reason: ExitReason, witness_failed: bool) -> ReceiptOutcome {
    if witness_failed {
        ReceiptOutcome::Error("recording_witness_failed".into())
    } else if reason == ExitReason::Exited {
        ReceiptOutcome::Ok
    } else {
        ReceiptOutcome::Error(format!("{reason:?}"))
    }
}

#[cfg(test)]
mod decision_test;
