// SPDX-License-Identifier: AGPL-3.0-only
//! Session capabilities and resource limits supplied by an authenticated transport.

use std::io;
use std::path::PathBuf;
use std::sync::Arc;

use cerud::receipt::ReceiptOutcome;
use cerud::transport::CallerIdentity;
use cerulion_ops_proto::SessionKind;

use crate::clock::RemotedClock;
pub use crate::remote_session::SessionLimits;
use crate::remote_session::MAX_REAUTHORIZE_INTERVAL;

pub type AuthorizeSession = Arc<dyn Fn() -> Result<(), String> + Send + Sync>;
pub type SessionReceipt =
    Arc<dyn Fn(&serde_json::Value, ReceiptOutcome) -> Result<(), String> + Send + Sync>;

/// Capabilities supplied only by the authenticated transport integration.
pub struct SessionContext {
    pub session: SessionKind,
    pub caller: CallerIdentity,
    pub account_id: Option<String>,
    pub recording_dir: PathBuf,
    pub clock: RemotedClock,
    pub authorize: AuthorizeSession,
    pub receipt: SessionReceipt,
    pub limits: SessionLimits,
}

impl SessionContext {
    /// Check the declared session and required timing bounds before starting work.
    ///
    /// This does not authenticate a peer or call the authorization/receipt
    /// callbacks. The transport supplies the verified identity, and the driver
    /// must still perform fresh authorization before spawning and while running.
    /// Byte budgets and optional duration limits keep their configured values;
    /// the recording and I/O paths enforce them when work is attempted.
    pub fn validate(&self) -> io::Result<()> {
        self.session.validate().map_err(io::Error::other)?;
        if !self.caller.authenticated
            || self.session.verb().is_none()
            || self.limits.reauthorize_interval.is_zero()
            || self.limits.reauthorize_interval > MAX_REAUTHORIZE_INTERVAL
            || self.limits.io_timeout.is_zero()
        {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "invalid authenticated session configuration",
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "session_context_test.rs"]
mod tests;
