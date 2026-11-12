// SPDX-License-Identifier: AGPL-3.0-only
//! Work which retains the owned remote endpoint through idle teardown.
//!
//! Shell hosts acquire a Shell guard before admitting work and retain it through
//! child drain and final witness. The daemon takes a Session guard for an ops or
//! rcp session admitted with paired authority, and around each ops verb outside
//! the bootstrap surface; a bootstrap-only, wire, refused or unknown-ALPN
//! connection takes none. It registers its latch and owed e-stop sources, so
//! disconnect cannot strand safety state. The native exclusive fence used by
//! netd prevents fresh work from racing a committed idle stop.

pub use cerulion_netd::serving_hold::{fence_path, HoldReason, ServingHold, ServingHolds};

use std::sync::{Arc, Mutex};

use cerulion_core::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};

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
    /// admission after refusals logs the recovery.
    pub(crate) fn admit(&self) -> Option<ServingHold> {
        match self.holds.acquire(HoldReason::Session) {
            Ok(hold) => {
                let recovered = self
                    .refusals
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .on_success();
                if let Some(suppressed) = recovered {
                    tracing::info!(suppressed, "remote serving admission recovered");
                }
                Some(hold)
            }
            Err(error) => {
                let (decision, total_failures) = {
                    let mut refusals = self.refusals.lock().unwrap_or_else(|e| e.into_inner());
                    (refusals.on_failure(), refusals.total_failures())
                };
                match decision {
                    RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => tracing::warn!(
                        error = %error,
                        total_failures,
                        "remote serving admission refused during idle stop"
                    ),
                    RegimeDecision::Suppressed { .. } => tracing::debug!(
                        error = %error,
                        total_failures,
                        "remote serving admission refused during idle stop"
                    ),
                }
                None
            }
        }
    }
}
