// SPDX-License-Identifier: AGPL-3.0-only
//! Work which retains the owned remote endpoint through idle teardown.
//!
//! Shell hosts acquire a Shell guard before admitting work and retain it through
//! child drain and final witness. The daemon takes a Session guard for an ops or
//! rcp session admitted with paired authority, and around each ops verb outside
//! the bootstrap surface; a bootstrap-only, wire, refused or unknown-ALPN
//! connection takes none. It registers its latch and owed e-stop sources, so
//! disconnect cannot strand safety state, and an armed pairing code's source,
//! which holds the fence from the arm until the code's end is written and never
//! past its window plus a margin (an exhausted code to that margin); the
//! pair-code poller takes a PairCode guard while it arms a request. Both
//! admissions log through `admit_logged`. The native exclusive fence used by
//! netd prevents fresh work from racing a committed idle stop.

pub use cerulion_netd::serving_hold::{fence_path, HoldReason, ServingHold, ServingHolds};

use std::sync::{Arc, Mutex};

use cerulion_core::transport::failure_regime_latch::{
    lock_regime_latch, FailureRegimeLatch, RegimeDecision,
};

/// What [`AdmissionFence::admit`] logs as the work it admits.
const SESSION_WORK: &str = "a paired session or an ops call outside the bootstrap surface";

/// A `reason` hold from `holds`, `work` naming what it admits in the log. Each
/// refusal is counted in `refusals` and logged through it, at warn when loud
/// and at debug while suppressed, with `error`, `total_failures`, `hold` and
/// `work`; the first hold taken after refusals logs the recovery at info, with
/// `suppressed` and `total_failures`.
pub(crate) fn admit_logged(
    holds: &ServingHolds,
    reason: HoldReason,
    refusals: &Mutex<FailureRegimeLatch>,
    work: &'static str,
) -> std::io::Result<ServingHold> {
    match holds.acquire(reason) {
        Ok(hold) => {
            let mut latch = lock_regime_latch(refusals);
            if let Some(suppressed) = latch.on_success() {
                let total_failures = latch.total_failures();
                drop(latch);
                tracing::info!(
                    suppressed,
                    total_failures,
                    hold = ?reason,
                    work,
                    "remote serving admission recovered"
                );
            }
            Ok(hold)
        }
        Err(error) => {
            let mut latch = lock_regime_latch(refusals);
            let decision = latch.on_failure();
            let total_failures = latch.total_failures();
            drop(latch);
            match decision {
                RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => tracing::warn!(
                    error = %error,
                    total_failures,
                    hold = ?reason,
                    work,
                    "remote serving admission refused during idle stop"
                ),
                RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                    error = %error,
                    total_failures,
                    suppressed,
                    hold = ?reason,
                    work,
                    "remote serving admission refused during idle stop"
                ),
            }
            Err(error)
        }
    }
}

/// Admission to the serving fence for the daemon's paired sessions and its
/// ops calls outside the bootstrap surface: its serving holds and one failure
/// regime over the admissions the fence refuses.
#[derive(Clone)]
pub(crate) struct AdmissionFence {
    holds: ServingHolds,
    refusals: Arc<Mutex<FailureRegimeLatch>>,
}

impl AdmissionFence {
    pub(crate) fn new(holds: ServingHolds) -> Self {
        Self {
            holds,
            refusals: Arc::default(),
        }
    }

    /// The serving holds this fence admits to.
    pub(crate) fn holds(&self) -> &ServingHolds {
        &self.holds
    }

    /// A Session guard, or `None` when the shared lock cannot be taken: netd
    /// holds the exclusive fence for a committed idle stop, or the lock call
    /// fails. A refusal is logged through the regime with its error; the first
    /// admission after refusals logs the recovery ([`admit_logged`]).
    pub(crate) fn admit(&self) -> Option<ServingHold> {
        admit_logged(
            &self.holds,
            HoldReason::Session,
            &self.refusals,
            SESSION_WORK,
        )
        .ok()
    }
}
