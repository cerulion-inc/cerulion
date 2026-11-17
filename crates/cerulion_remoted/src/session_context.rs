// SPDX-License-Identifier: AGPL-3.0-only
//! Session capabilities and resource limits supplied by an authenticated transport.

use std::io;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use cerud::receipt::ReceiptOutcome;
use cerud::transport::CallerIdentity;
use cerulion_ops_proto::SessionKind;

use crate::clock::RemotedClock;

// A one-second sweep leaves 500 ms for authorization and 250 ms for ordinary-job
// cleanup inside the two-second revocation budget, with scheduling margin.
const MAX_REAUTHORIZE_INTERVAL: Duration = Duration::from_secs(1);

/// Robot-configurable resource limits. These never classify command contents.
#[derive(Clone, Debug)]
pub struct SessionLimits {
    pub reauthorize_interval: Duration,
    pub io_timeout: Duration,
    pub max_recording_bytes: u64,
    pub max_stdin_bytes: u64,
    pub max_stdout_bytes: u64,
    pub max_stderr_bytes: u64,
    pub exec_timeout: Option<Duration>,
    pub shell_timeout: Option<Duration>,
    pub shell_idle_timeout: Option<Duration>,
}

impl Default for SessionLimits {
    fn default() -> Self {
        Self {
            reauthorize_interval: MAX_REAUTHORIZE_INTERVAL,
            io_timeout: Duration::from_secs(2),
            max_recording_bytes: 64 * 1024 * 1024,
            max_stdin_bytes: 1024 * 1024,
            max_stdout_bytes: 1024 * 1024,
            max_stderr_bytes: 1024 * 1024,
            // One minute bounds unattended short commands; operators can extend
            // or disable it for longer work through the exec timeout setting.
            exec_timeout: Some(Duration::from_secs(60)),
            // Eight hours permits a working session while bounding abandoned
            // shells; the robot operator can extend or disable this wall limit.
            shell_timeout: Some(Duration::from_secs(8 * 60 * 60)),
            // Thirty minutes without input permits breaks while retiring idle
            // terminals; the robot operator can extend or disable this limit.
            shell_idle_timeout: Some(Duration::from_secs(30 * 60)),
        }
    }
}

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
