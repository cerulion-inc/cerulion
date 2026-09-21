// SPDX-License-Identifier: AGPL-3.0-only
//! The email magic-link provider's delivery seam.
//!
//! The magic-link *flow* (mint token → store → deliver → complete → authorize the
//! device code) lives in the HTTP layer; this module owns only the pluggable
//! **email transport**. Two real impls ship: [`LoggingEmailSender`] (logs
//! the link — the dev sink) and [`CapturingEmailSender`] (records links for the
//! test harness). A production SMTP/SES transport is a future impl of the SAME
//! trait — the flow is identical, so this is a transport seam, NOT a bypass tier.

use std::sync::Mutex;

use crate::error::Result;

/// Delivers a magic-link login URL to an email address.
pub trait EmailSender: Send + Sync {
    /// Deliver `link` to `email`. Returns `Ok` once accepted for delivery.
    fn send_magic_link(&self, email: &str, link: &str) -> Result<()>;
}

/// The dev sink: log the link at `info`. Correct local behavior when no email
/// transport is configured — the operator reads the link from the service log.
#[derive(Debug, Default)]
pub struct LoggingEmailSender;

impl EmailSender for LoggingEmailSender {
    fn send_magic_link(&self, email: &str, link: &str) -> Result<()> {
        tracing::info!(
            email,
            link,
            "magic-link dispatched (dev sink: no email transport configured)"
        );
        Ok(())
    }
}

/// A test-harness sink: record every `(email, link)` for later assertion. A real
/// implementation of the transport seam — the harness reads the captured link
/// exactly as a mailbox would, so no login step is faked.
#[derive(Debug, Default)]
pub struct CapturingEmailSender {
    sent: Mutex<Vec<(String, String)>>,
}

impl CapturingEmailSender {
    /// A fresh capturing sender.
    pub fn new() -> Self {
        Self::default()
    }

    /// The most recently captured link, if any.
    pub fn last_link(&self) -> Option<String> {
        self.sent.lock().ok()?.last().map(|(_, link)| link.clone())
    }

    /// The number of links captured.
    pub fn count(&self) -> usize {
        self.sent.lock().map(|v| v.len()).unwrap_or(0)
    }
}

impl EmailSender for CapturingEmailSender {
    fn send_magic_link(&self, email: &str, link: &str) -> Result<()> {
        if let Ok(mut v) = self.sent.lock() {
            v.push((email.to_string(), link.to_string()));
        }
        Ok(())
    }
}
