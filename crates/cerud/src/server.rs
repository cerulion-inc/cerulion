// SPDX-License-Identifier: AGPL-3.0-only
//! The ops server: drives one connection through handshake → per-request
//! (authz → verb dispatch → receipt).
//!
//! The server is transport-agnostic — it takes an [`OpsConnection`] (the
//! stream + the transport-authenticated caller identity) and speaks the
//! [`crate::protocol`] framing on top. Every request is authorized server-side
//! and receipted whether allowed, denied, unknown, or failed.
//!
//! # Concurrency (— the e-stop channel is never queued behind other sessions)
//!
//! Every method is `&self` and the ONLY mutable state — the hash-chained receipt
//! log — lives behind a [`Mutex`], held ONLY for the brief append (a write +
//! `sync_data`), never for a whole session. So an [`Arc<OpsServer>`](std::sync::Arc)
//! can serve MANY sessions CONCURRENTLY (the iroh remote-plane host does exactly
//! this), and a stalled or hostile session — parked on a blocking read, holding
//! NO lock — can never delay another session. In particular a paired
//! `engage-estop` runs concurrently and reaches the lease immediately: its only
//! possible contention is the brief receipt append, and the safety EFFECT itself
//! runs BEFORE that audit write (e-stop is non-mutating, so no intent receipt
//! precedes the effect). This is the transport-layer expression of the
//! rule "e-stop = permission floor, any session, always wins."
//!
//! Hash-chain integrity is preserved under concurrent appends because the
//! `Mutex` serializes each `record` (read-prev-hash → write → advance) as one
//! atomic critical section; the interleaved entries still form one valid chain.

use std::sync::Mutex;

use crate::authz::{Authorizer, AuthzDecision};
use crate::error::{CerudError, CerudResult};
use crate::protocol::{
    self, HandshakeReply, Hello, Request, Response, MIN_PROTOCOL_VERSION, PROTOCOL_VERSION,
};
use crate::receipt::{args_digest, ReceiptLog, ReceiptOutcome};
use crate::transport::{CallerIdentity, OpsConnection, OpsListener};
use crate::verbs::VerbRegistry;

/// The ops server: an authorizer, a verb registry, and the hash-chained receipt
/// sink (behind a brief-append [`Mutex`] so one `Arc<OpsServer>` serves sessions
/// concurrently — see the module docs).
pub struct OpsServer {
    server_name: String,
    authorizer: Box<dyn Authorizer>,
    verbs: VerbRegistry,
    /// The append-only, hash-chained audit sink, behind a `Mutex` so concurrent
    /// sessions serialize ONLY at the brief append (the chain needs a single
    /// writer at a time) — never for a whole session. A stalled/hostile session
    /// never holds it, so it never delays another session's receipt (and never
    /// the `engage-estop` safety floor)..
    receipts: Mutex<ReceiptLog>,
}

impl OpsServer {
    /// Build a server from its parts. The [`ReceiptLog`] is wrapped in the
    /// server's brief-append [`Mutex`], so the server is `Send + Sync` and one
    /// `Arc<OpsServer>` can serve sessions concurrently (see the module docs).
    pub fn new(
        server_name: impl Into<String>,
        authorizer: Box<dyn Authorizer>,
        verbs: VerbRegistry,
        receipts: ReceiptLog,
    ) -> Self {
        OpsServer {
            server_name: server_name.into(),
            authorizer,
            verbs,
            receipts: Mutex::new(receipts),
        }
    }

    /// Accept and fully serve connections one at a time until the listener
    /// errors (e.g. the socket is removed). Each connection is served to its
    /// clean close; a per-connection error is logged and the loop continues.
    ///
    /// The dev Unix-socket transport is inherently serial (one `accept` at a
    /// time), so this loop serves one connection at a time — but that is a
    /// TRANSPORT property, not a receipt-log constraint: the receipt sink is a
    /// brief-append [`Mutex`], so an `Arc<OpsServer>` served from many tasks
    /// (the iroh remote-plane host) is fully concurrent (see the module docs).
    pub fn serve_forever<L: OpsListener>(&self, listener: &mut L) -> CerudResult<()> {
        tracing::info!(addr = %listener.local_addr(), server = %self.server_name, "cerud serving");
        loop {
            let conn = listener.accept()?;
            if let Err(e) = self.serve_connection(conn) {
                match e {
                    CerudError::ConnectionClosed => {}
                    other => tracing::warn!(error = %other, "connection ended with error"),
                }
            }
        }
    }

    /// Serve one connection: handshake, then the request loop until the peer
    /// closes. `&self` (the only mutable state is the receipt sink's brief-append
    /// `Mutex`), so many sessions run concurrently over one `Arc<OpsServer>`.
    pub fn serve_connection(&self, mut conn: OpsConnection) -> CerudResult<()> {
        self.handshake(&mut conn)?;
        loop {
            let frame = match protocol::read_frame(&mut conn.stream) {
                Ok(f) => f,
                Err(CerudError::ConnectionClosed) => return Ok(()),
                Err(e) => return Err(e),
            };
            let request: Request = protocol::decode(&frame)?;
            let response = self.handle_request(&conn.caller, &request);
            let payload = protocol::encode(&response)?;
            protocol::write_frame(&mut conn.stream, &payload)?;
        }
    }

    /// Perform the version-negotiation handshake. On success, writes an `Ack`;
    /// on failure, writes a `Reject` and returns the error (closing the conn).
    fn handshake(&self, conn: &mut OpsConnection) -> CerudResult<()> {
        let frame = protocol::read_frame(&mut conn.stream)?;
        let hello: Hello = protocol::decode(&frame)?;

        match protocol::negotiate_version(
            hello.min_version,
            hello.max_version,
            MIN_PROTOCOL_VERSION,
            PROTOCOL_VERSION,
        ) {
            Ok(chosen) => {
                let reply = HandshakeReply::Ack {
                    chosen_version: chosen,
                    server_name: self.server_name.clone(),
                };
                let payload = protocol::encode(&reply)?;
                protocol::write_frame(&mut conn.stream, &payload)?;
                tracing::debug!(caller = %conn.caller.id, version = chosen, "handshake ok");
                Ok(())
            }
            Err(e) => {
                let reply = HandshakeReply::Reject {
                    server_min: MIN_PROTOCOL_VERSION,
                    server_max: PROTOCOL_VERSION,
                    message: e.to_string(),
                };
                // Best-effort: tell the client why before closing.
                if let Ok(payload) = protocol::encode(&reply) {
                    let _ = protocol::write_frame(&mut conn.stream, &payload);
                }
                tracing::warn!(caller = %conn.caller.id, error = %e, "handshake rejected");
                Err(e)
            }
        }
    }

    /// Authorize + dispatch + receipt one request, always returning a
    /// [`Response`] (errors become `Response::Err`, never propagate).
    ///
    /// The audit trail is a HARD dependency, not best-effort: if a receipt
    /// cannot be written DURABLY, the request fails loudly (`audit_failure`) so
    /// the caller knows the record is missing. For a
    /// mutating verb the INTENT receipt is written BEFORE the side effect, so a
    /// failed intent write means the side effect never runs (fail-closed), and
    /// a crash after the effect still leaves the intent on disk.
    fn handle_request(&self, caller: &CallerIdentity, req: &Request) -> Response {
        let digest = args_digest(&req.args);

        // Receipt writes go through `record_receipt`, which locks the shared
        // sink ONLY for the brief append — so `handler` (a `&self` borrow of
        // `self.verbs`) and the audit write are both shared `&self` borrows that
        // coexist without a `&mut self` conflict, and concurrent sessions
        // serialize only at the append (never for a whole session).

        // 1. Per-verb authorization, re-checked server-side on every call.
        match self.authorizer.authorize(caller, &req.verb, &req.args) {
            AuthzDecision::Allow => {}
            AuthzDecision::Deny { reason } => {
                if let Err(e) =
                    self.record_receipt(&caller.id, &req.verb, &digest, ReceiptOutcome::Denied)
                {
                    return audit_failure_response(req.id, &e, &req.verb, caller);
                }
                return Response::from_error(
                    req.id,
                    &CerudError::Denied {
                        caller: caller.id.clone(),
                        verb: req.verb.clone(),
                        reason,
                    },
                );
            }
        }

        // 2. Dispatch lookup.
        let handler = match self.verbs.get(&req.verb) {
            Some(h) => h,
            None => {
                let err = CerudError::UnknownVerb(req.verb.clone());
                if let Err(e) = self.record_receipt(
                    &caller.id,
                    &req.verb,
                    &digest,
                    ReceiptOutcome::Error(err.kind().to_string()),
                ) {
                    return audit_failure_response(req.id, &e, &req.verb, caller);
                }
                return Response::from_error(req.id, &err);
            }
        };

        // 3. For a mutating verb, write a DURABLE intent receipt BEFORE the
        //    side effect. If it cannot be recorded, fail closed — do not run
        //    the side effect.
        if handler.is_mutating() {
            if let Err(e) =
                self.record_receipt(&caller.id, &req.verb, &digest, ReceiptOutcome::Intent)
            {
                return audit_failure_response(req.id, &e, &req.verb, caller);
            }
        }

        // 4. Execute. Dispatch through the caller-aware seam so a verb whose
        //    proof IS the authenticated peer key (the remote-plane pairing/claim
        //    bootstrap verbs) can bind the transport-authenticated identity; a
        //    caller-agnostic verb ignores it via the default delegation.
        let (outcome, response) = match handler.execute_with_caller(caller, &req.args) {
            Ok(value) => (
                ReceiptOutcome::Ok,
                Response::Ok {
                    id: req.id,
                    result: value,
                },
            ),
            Err(err) => (
                ReceiptOutcome::Error(err.kind().to_string()),
                Response::from_error(req.id, &err),
            ),
        };

        // 5. Record the outcome DURABLY. A failure here is loud: the side
        //    effect (if any) already ran, but the caller must learn the
        //    outcome was not audited.
        if let Err(e) = self.record_receipt(&caller.id, &req.verb, &digest, outcome) {
            return audit_failure_response(req.id, &e, &req.verb, caller);
        }
        response
    }

    /// Append one receipt to the hash-chained sink, holding the append lock ONLY
    /// for the brief write (never for a whole session), so concurrent sessions
    /// serialize only here.
    ///
    /// **FAIL-CLOSED on a poisoned lock (integrity, not availability).** A `Mutex`
    /// poison means a PRIOR append PANICKED mid-write — exactly when the hash chain
    /// could be left with an inconsistent `next_seq`/`prev_hash`. Silently
    /// recovering the guard (`into_inner`) would hand that possibly-corrupt state
    /// to this append and produce a BROKEN-chain receipt on the integrity-critical,
    /// safety-plane audit log (the sink that audits `engage-estop` + every mutating
    /// op). A poisoned receipt sink is NOT a recoverable condition, so this returns
    /// a loud [`CerudError::Receipt`]: the request then fails audit-closed (the
    /// caller learns the audit is missing) and the daemon must restart to reopen a
    /// fresh log. The `engage-estop` safety EFFECT is unaffected — it runs BEFORE
    /// its audit receipt (e-stop is non-mutating), so the floor still engages.
    fn record_receipt(
        &self,
        caller: &str,
        verb: &str,
        args_digest: &str,
        outcome: ReceiptOutcome,
    ) -> CerudResult<()> {
        let mut receipts = self.receipts.lock().map_err(|_poisoned| {
            CerudError::Receipt(
                "the hash-chained receipt log mutex is POISONED — a prior append panicked \
                 mid-write, so the audit sink may hold an inconsistent next_seq/prev_hash and a \
                 further append could produce a BROKEN chain. Refusing to append (fail-CLOSED) \
                 rather than silently corrupt the integrity-critical, safety-plane audit log; the \
                 daemon must be restarted to reopen a fresh receipt log"
                    .to_string(),
            )
        })?;
        receipts
            .record(caller, verb, args_digest, outcome)
            .map(|_receipt| ())
    }

    /// Whether the hash-chained receipt sink's `Mutex` is POISONED — a prior
    /// append panicked mid-write, so the audit log may be inconsistent. An
    /// observability signal (Principle #3): the remote-plane host reads this to
    /// refuse a NEW session fail-closed (and close with an accurate reason),
    /// while `record_receipt` fails EACH append closed regardless.
    /// A poisoned integrity-critical sink is not recoverable — the daemon must
    /// restart to reopen a fresh log..
    pub fn receipt_sink_poisoned(&self) -> bool {
        self.receipts.is_poisoned()
    }

    /// Test-only: POISON the receipt-sink `Mutex` the way a session that panicked
    /// mid hash-chain receipt write would (lock it, then panic; the guard drops
    /// during unwinding, marking the `Mutex` poisoned). Lets the fail-closed
    /// poison-refusal path be pinned without crafting a real mid-write panic.
    ///
    /// **EXCLUDED FROM PRODUCTION** via `#[cfg(any(test, feature = "test-seam"))]`
    /// — exactly like the crate's other fault-injection seams
    /// (`ReceiptLog::fail_next_record_for_test` / `fail_next_sync_for_test`). This
    /// method DELIBERATELY corrupts the safety-plane audit sink, so it must never
    /// compile into the production binary (a plain `cargo build -p cerud` does not
    /// expose it); the cross-crate tests (`cerulion_remoted`) reach it via the
    /// `test-seam` feature. The read-only `receipt_sink_poisoned` check STAYS
    /// ungated (a legitimate production observability path).
    #[doc(hidden)]
    #[cfg(any(test, feature = "test-seam"))]
    pub fn poison_receipt_sink_for_test(&self) {
        let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _guard = self.receipts.lock().expect("lock to poison");
            panic!("poison the receipt sink for test");
        }));
    }
}

/// Build the loud `audit_failure` response for a request whose receipt could
/// not be written durably, logging the failure at `error` level.
fn audit_failure_response(
    id: u64,
    err: &CerudError,
    verb: &str,
    caller: &CallerIdentity,
) -> Response {
    tracing::error!(
        error = %err,
        caller = %caller.id,
        verb,
        "receipt write failed; refusing the request so the caller knows the audit is missing"
    );
    Response::Err {
        id,
        kind: "audit_failure".to_string(),
        message: format!("audit write failed for verb '{verb}': {err}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::authz::PermissiveDevAuthorizer;
    use crate::receipt::{read_all, verify_chain, ReceiptLog};
    use crate::verbs::VerbRegistry;

    /// FAIL-CLOSED on a poisoned receipt mutex (the c4-parity
    /// integrity fix): a poisoned hash-chained audit sink means a prior append
    /// panicked mid-write, so the next append could produce a broken chain. The
    /// safety-plane receipt log must REFUSE the append loudly, never silently
    /// recover into possibly-corrupt state.
    ///
    /// Reverting `record_receipt` to
    /// `lock().unwrap_or_else(|p| p.into_inner())` makes the post-poison append
    /// SUCCEED (returns `Ok`, grows the log to 2 entries) — both asserts below then
    /// fail. Fail-closed keeps the on-disk chain at exactly its pre-poison length +
    /// valid.
    #[test]
    fn poisoned_receipt_mutex_fails_closed_no_silent_broken_chain_append() {
        let dir = tempfile::tempdir().unwrap();
        let log_path = dir.path().join("receipts.log");
        let receipts = ReceiptLog::open(&log_path).unwrap();
        let server = OpsServer::new(
            "cerud-poison-test",
            Box::new(PermissiveDevAuthorizer),
            VerbRegistry::new(),
            receipts,
        );

        // One valid append succeeds → the log holds a single, chain-valid entry.
        server
            .record_receipt("caller-a", "inventory", "00", ReceiptOutcome::Ok)
            .expect("the first append succeeds");
        assert_eq!(read_all(&log_path).unwrap().len(), 1);

        // Poison the receipt mutex: panic while holding the lock (a no-op panic hook
        // keeps the expected-panic message out of the test output). The guard drops
        // during unwind, marking the std Mutex poisoned.
        let prev_hook = std::panic::take_hook();
        std::panic::set_hook(Box::new(|_| {}));
        let poisoned = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _guard = server.receipts.lock().unwrap();
            panic!("deliberately poison the receipt mutex");
        }));
        std::panic::set_hook(prev_hook);
        assert!(
            poisoned.is_err(),
            "the closure panicked (poisoning the mutex)"
        );
        assert!(
            server.receipts.is_poisoned(),
            "the receipt mutex is now poisoned"
        );

        // The next append REFUSES loudly (fail-closed), naming the poison — it does
        // NOT silently recover and append a possibly-broken entry.
        let err = server
            .record_receipt("caller-b", "inventory", "01", ReceiptOutcome::Ok)
            .expect_err("a poisoned receipt sink refuses the append (fail-closed)");
        match err {
            CerudError::Receipt(msg) => {
                assert!(msg.contains("POISONED"), "names the poison: {msg}");
                assert!(msg.contains("fail-CLOSED"), "states fail-closed: {msg}");
            }
            other => panic!("expected a Receipt error, got {other:?}"),
        }

        // No phantom append: the on-disk log is STILL exactly one entry and its
        // hash chain still verifies (the refused append left it clean).
        let entries = read_all(&log_path).unwrap();
        assert_eq!(
            entries.len(),
            1,
            "the refused append must not have grown the log (into_inner would make this 2)"
        );
        verify_chain(&entries).expect("the on-disk chain is intact after the refused append");
    }
}
