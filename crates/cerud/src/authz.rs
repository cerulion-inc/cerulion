// SPDX-License-Identifier: AGPL-3.0-only
//! The per-verb authorization seam.
//!
//! Every verb re-checks capability scope server-side: [`crate::server`] calls
//! [`Authorizer::authorize`] with the caller's authenticated identity, the
//! verb name, and the args BEFORE dispatching — and records the outcome in the
//! receipt log whether allowed or denied.
//!
//! The real access-list check plugs in later from the pairing layer. This
//! chunk ships two placeholder impls:
//! - [`DenyAllAuthorizer`] — deny-by-default (the production-safe default).
//! - [`PermissiveDevAuthorizer`] — allow everything; **DEV ONLY**, clearly
//!   marked, never a production default.

use crate::transport::CallerIdentity;

/// The decision an [`Authorizer`] returns for one `(caller, verb, args)`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AuthzDecision {
    /// The caller may run the verb.
    Allow,
    /// The caller may not; `reason` is surfaced to the caller + receipted.
    Deny { reason: String },
}

impl AuthzDecision {
    /// Convenience: a deny with a formatted reason.
    pub fn deny(reason: impl Into<String>) -> Self {
        AuthzDecision::Deny {
            reason: reason.into(),
        }
    }

    /// Whether this decision permits the call.
    pub fn is_allowed(&self) -> bool {
        matches!(self, AuthzDecision::Allow)
    }
}

/// The pairing/access-list seam. The real implementation (an access list keyed
/// on the paired peer identity + a per-verb capability grant) plugs in here
/// later; the server depends only on this trait.
pub trait Authorizer: Send + Sync {
    /// Decide whether `caller` may run `verb` with `args`.
    fn authorize(
        &self,
        caller: &CallerIdentity,
        verb: &str,
        args: &serde_json::Value,
    ) -> AuthzDecision;
}

/// Deny-by-default placeholder — the production-safe default until the pairing
/// layer supplies a real access list. Refuses EVERY verb, naming the missing
/// pairing configuration so the refusal is diagnosable rather than silent.
#[derive(Debug, Default, Clone, Copy)]
pub struct DenyAllAuthorizer;

impl Authorizer for DenyAllAuthorizer {
    fn authorize(
        &self,
        caller: &CallerIdentity,
        verb: &str,
        _args: &serde_json::Value,
    ) -> AuthzDecision {
        AuthzDecision::deny(format!(
            "deny-by-default: no pairing access-list is configured, so caller \
             '{}' (authenticated={}) is not authorized for verb '{}'",
            caller.id, caller.authenticated, verb
        ))
    }
}

/// **DEV ONLY** permissive authorizer: allows every verb from every caller.
///
/// This exists so the dev binary + tests can exercise the verb path without a
/// pairing layer. It is NEVER a production default — the binary requires an
/// explicit `--dev-permissive` flag to select it and logs a loud warning.
#[derive(Debug, Default, Clone, Copy)]
pub struct PermissiveDevAuthorizer;

impl Authorizer for PermissiveDevAuthorizer {
    fn authorize(
        &self,
        _caller: &CallerIdentity,
        _verb: &str,
        _args: &serde_json::Value,
    ) -> AuthzDecision {
        AuthzDecision::Allow
    }
}
