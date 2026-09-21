// SPDX-License-Identifier: AGPL-3.0-only
//! The verb framework and the first three working verbs.
//!
//! A verb is a mechanical operation the ops service exposes. Each implements
//! [`VerbHandler`]: a stable name, and an `execute` taking JSON args and
//! returning a JSON result (or a [`crate::CerudError`]). The [`VerbRegistry`] maps
//! names to handlers; [`crate::server`] looks up + dispatches after the authz
//! check, and receipts every call.

use std::collections::BTreeMap;

use crate::error::CerudResult;
use crate::transport::CallerIdentity;

pub mod inventory;
pub mod log_tail;
pub mod restart;

pub use inventory::InventoryVerb;
pub use log_tail::LogTailVerb;
pub use restart::RestartVerb;

/// A mechanical ops verb.
pub trait VerbHandler: Send + Sync {
    /// The stable verb name (matches the `Request::verb` string + the authz
    /// scope name).
    fn name(&self) -> &'static str;

    /// Run the verb with `args`, returning its JSON result.
    fn execute(&self, args: &serde_json::Value) -> CerudResult<serde_json::Value>;

    /// Run the verb with the transport-authenticated [`CallerIdentity`] in scope.
    ///
    /// [`crate::server`] dispatches EVERY verb through this method (never
    /// [`VerbHandler::execute`] directly), so a verb whose behavior is bound to
    /// the authenticated peer — the remote-plane pairing/claim bootstrap verbs,
    /// whose proof is the authenticated device key itself — overrides this to
    /// read `caller`. It defaults to [`VerbHandler::execute`], so a caller-agnostic
    /// verb (inventory, log-tail, restart) needs no change and simply ignores the
    /// identity. Keeping the caller OUT of `args` is deliberate: the identity is
    /// supplied by the transport, never by client-controlled arguments.
    fn execute_with_caller(
        &self,
        caller: &CallerIdentity,
        args: &serde_json::Value,
    ) -> CerudResult<serde_json::Value> {
        let _ = caller;
        self.execute(args)
    }

    /// Whether this verb has an externally-visible side effect (restarts a
    /// unit, swaps a symlink, ...). The server writes a durable INTENT receipt
    /// BEFORE dispatching a mutating verb, so a crash after the side effect
    /// still leaves an audit trail.
    ///
    /// REQUIRED (no default) so adding a new verb forces an explicit
    /// mutating/read-only decision at compile time — a future mutating verb
    /// cannot silently inherit `false` and run fail-open with no intent receipt.
    fn is_mutating(&self) -> bool;
}

/// A registry mapping verb names to handlers. `BTreeMap` keeps iteration
/// (e.g. [`VerbRegistry::names`]) deterministic.
#[derive(Default)]
pub struct VerbRegistry {
    handlers: BTreeMap<String, Box<dyn VerbHandler>>,
}

impl VerbRegistry {
    /// An empty registry.
    pub fn new() -> Self {
        VerbRegistry::default()
    }

    /// Register a handler under its own name. A duplicate name replaces the
    /// prior handler (the caller controls registration order).
    pub fn register(&mut self, handler: Box<dyn VerbHandler>) -> &mut Self {
        self.handlers.insert(handler.name().to_string(), handler);
        self
    }

    /// Look up a handler by verb name.
    pub fn get(&self, name: &str) -> Option<&dyn VerbHandler> {
        self.handlers.get(name).map(|b| b.as_ref())
    }

    /// The registered verb names, sorted.
    pub fn names(&self) -> Vec<&str> {
        self.handlers.keys().map(|s| s.as_str()).collect()
    }

    /// How many verbs are registered.
    pub fn len(&self) -> usize {
        self.handlers.len()
    }

    /// Whether the registry is empty.
    pub fn is_empty(&self) -> bool {
        self.handlers.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A caller-agnostic verb that only implements [`VerbHandler::execute`]. It is
    /// used to pin that the provided [`VerbHandler::execute_with_caller`] delegates
    /// to `execute` (so every existing verb keeps working unchanged) while making
    /// the authenticated caller available to verbs that need it.
    struct EchoCallerVerb;
    impl VerbHandler for EchoCallerVerb {
        fn name(&self) -> &'static str {
            "echo"
        }
        fn is_mutating(&self) -> bool {
            false
        }
        fn execute(&self, args: &serde_json::Value) -> CerudResult<serde_json::Value> {
            Ok(serde_json::json!({ "via": "execute", "args": args }))
        }
    }

    #[test]
    fn execute_with_caller_defaults_to_execute() {
        // The provided caller-aware method delegates to `execute` for a verb that
        // does not override it — the zero-churn contract for caller-agnostic verbs.
        let v = EchoCallerVerb;
        let args = serde_json::json!({ "k": 1 });
        let verified = CallerIdentity::verified("deadbeef");
        let local = CallerIdentity::local_dev();
        // Same result regardless of who the caller is (the default ignores it).
        let expected = serde_json::json!({ "via": "execute", "args": { "k": 1 } });
        assert_eq!(v.execute_with_caller(&verified, &args).unwrap(), expected);
        assert_eq!(v.execute_with_caller(&local, &args).unwrap(), expected);
        assert_eq!(v.execute(&args).unwrap(), expected);
    }
}
