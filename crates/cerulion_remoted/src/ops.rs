// SPDX-License-Identifier: AGPL-3.0-only
//! The ops plane over iroh: `cerulion/ops/1` → [`cerud::server::OpsServer`].
//!
//! An accepted `cerulion/ops/1` connection carries ONE bidirectional QUIC stream
//! per request/response session. This module bridges that async QUIC stream to
//! `cerud`'s SYNCHRONOUS `serve_connection` seam exactly as `cerud`'s
//! `transport.rs` anticipates: accept the bidi stream, wrap it in the
//! [`QuicOpsStream`] adapter (a channel-backed, cancel-safe `Read + Write` byte
//! pipe), and hand it to `cerud` on a [`tokio::task::spawn_blocking`] task with a
//! [`CallerIdentity::verified`] built from the TLS-authenticated `remote_id`.
//!
//! ## Do NOT double-frame
//!
//! `cerud` does its OWN big-endian `u32` + JSON framing INSIDE the stream. The ops
//! plane therefore bypasses `cerulion_link::write_frame` entirely — [`QuicOpsStream`]
//! moves RAW bytes; `cerud` frames them. Layering the link's little-endian frame
//! on top would frame the stream twice and corrupt it.
//!
//! ## The verb surface
//!
//! [`OpsServing::new`] registers `cerud`'s mechanical verbs (`inventory`,
//! `log-tail`, `restart`) alongside the remote-plane bootstrap + safety verbs
//! (`claim`, `pair`, `code-pair-start`, `code-pair-finish`, `engage-estop`, in
//! [`crate::pairing_verbs`]) into ONE registry. Every verb is gated by the
//! [`PairingAuthorizer`] over the LIVE trust state — so a `claim`/`pair` takes
//! effect at the next request WITHOUT a restart — and receipted by `cerud`'s
//! hash-chained audit log.
//!
//! ## Concurrent serving + the un-starvable e-stop channel
//!
//! Serializing ops sessions behind one permit (as a single hash-chained receipt
//! sink would require) is a safety hazard: a reconnect-loop
//! attacker could hold the permit up to the session deadline, DELAYING a
//! legitimate remote `engage-estop`. There is no whole-session
//! serialization here. `cerud`'s [`OpsServer`] is `Send + Sync` with the receipt
//! log behind a brief-append `Mutex`, so ONE [`Arc<OpsServer>`] serves EVERY
//! session CONCURRENTLY. A stalled/hostile session — parked on a blocking read,
//! holding NO lock — cannot delay another session. In particular a paired
//! `engage-estop` runs on its OWN concurrent session and reaches the lease
//! immediately: the safety EFFECT (`lease.engage_estop`) runs BEFORE its audit
//! receipt (e-stop is non-mutating, so no intent receipt precedes it), and the
//! ONLY contention it can ever see is the brief receipt append — never a whole
//! session. This is "e-stop = permission floor, any session, always wins" applied
//! to the transport layer. Hash-chain integrity holds under concurrent appends
//! because the sink's `Mutex` serializes each `record` as one atomic append.
//!
//! Each session is still BOUNDED by [`OPS_SESSION_DEADLINE`] (an authed-then-
//! stalled peer's connection is CLOSED at the deadline, unwedging its blocking
//! read so its task ends and its resources free) — a per-session resource bound,
//! not a serialization point. A GLOBAL concurrency cap with a RESERVED
//! safety-floor slot (to bound the total blocking-task footprint under a flood
//! while keeping `engage-estop` always admissible) is not implemented; neither is
//! the daemon's accept-loop concurrency cap.
//!
//! ## Fail-closed on a poisoned receipt sink
//!
//! If a session panics mid hash-chain receipt write, `cerud`'s receipt `Mutex`
//! POISONS (the audit log may be half-written). `cerud`'s `record_receipt` then
//! fails EACH append closed (returns a loud `CerudError::Receipt`, never a silent
//! corrupt append), and — for an accurate operator signal — [`OpsServing`] also
//! fast-fails a NEW session before it serves: it checks
//! [`OpsServer::receipt_sink_poisoned`] and, if poisoned, refuses the session and
//! closes the QUIC connection with the ACCURATE reason `ops receipt sink poisoned`
//! (not the generic deadline reason). A poisoned integrity-critical audit sink is
//! not a recoverable condition — the daemon must restart (a fresh
//! `ReceiptLog::open` recovers the torn tail).

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_link::{accept_frame_stream, Accepted, Connection, QuicOpsStream};

use cerud::authz::Authorizer;
use cerud::lease::ControlLease;
use cerud::receipt::ReceiptLog;
use cerud::server::OpsServer;
use cerud::transport::{CallerIdentity, OpsConnection};
use cerud::verbs::{InventoryVerb, LogTailVerb, RestartVerb, VerbRegistry};

use crate::authorizer::PairingAuthorizer;
use crate::clock::RemotedClock;
use crate::error::RemotedError;
use crate::pairing_verbs::{
    ClaimVerb, CodePairFinishVerb, CodePairStartVerb, EngageEstopVerb, PairVerb, PresentGrantVerb,
    SharedCodePairSessions,
};
use crate::trust::SharedTrust;

/// Sentinel returned by the blocking session task when the shared [`OpsServer`]'s
/// hash-chained receipt SINK is POISONED (a prior session panicked mid receipt
/// write): the NEW session is refused fail-closed — with an accurate QUIC close
/// reason — rather than risk appending to a possibly-corrupt audit log. `cerud`'s
/// `record_receipt` ALSO fails EACH append closed, so no session ever silently
/// corrupts the chain; this is the session-level fast-fail + the operator signal.
struct ReceiptSinkPoisoned;

/// Per-session wall bound. An authed-then-stalled peer must not hold a session's
/// blocking task + connection indefinitely (a resource-hold DoS on the always-on
/// daemon — the bootstrap surface admits any TLS-authed key). Generous for a real
/// deploy/log-tail exchange, but bounded: on expiry the connection is closed,
/// unwedging the blocking session so its task ends and its resources free. It no
/// longer gates OTHER sessions (serving is concurrent), so it never delays the
/// `engage-estop` safety floor.
pub const OPS_SESSION_DEADLINE: Duration = Duration::from_secs(120);

/// The ops-plane serving context: ONE shared [`OpsServer`] (the mechanical +
/// bootstrap verb registry, the pairing authorizer over the live trust state, and
/// the hash-chained receipt sink). Built once at daemon boot;
/// [`OpsServing::serve_session`] drives each accepted `cerulion/ops/1` connection
/// through it CONCURRENTLY — the receipt sink serializes only the brief append,
/// so no session ever blocks another (see the module docs).
pub struct OpsServing {
    /// Shared, `&self`-served across concurrent sessions. Its receipt sink is a
    /// brief-append `Mutex` (in `cerud`), so concurrent appends stay chain-valid
    /// without any whole-session serialization here.
    server: Arc<OpsServer>,
    /// The per-session wall bound (see [`OPS_SESSION_DEADLINE`]). Overridable in
    /// tests via [`OpsServing::with_deadline_for_test`] so the stalled-session
    /// unwedge can be pinned without a 2-minute wait.
    deadline: Duration,
}

impl OpsServing {
    /// Build the ops-serving context: register `cerud`'s mechanical verbs +
    /// the remote-plane bootstrap/safety verbs, gate them with the pairing
    /// authorizer over `shared` (the LIVE trust state), and open the receipt log.
    ///
    /// `lease` is the shared control lease the `engage-estop` floor engages;
    /// `sessions` is the CPace fallback session store; `clock` is the robot's
    /// TRUSTED clock the pairing verbs verify against; `log_root` is the
    /// `log-tail` allow-list root; `receipt_path` is the audit log.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        shared: SharedTrust,
        lease: Arc<Mutex<ControlLease>>,
        sessions: SharedCodePairSessions,
        clock: RemotedClock,
        receipt_path: &Path,
        log_root: PathBuf,
        server_name: impl Into<String>,
    ) -> Result<Self, RemotedError> {
        let mut verbs = VerbRegistry::new();
        // cerud's mechanical ops verbs.
        verbs
            .register(Box::new(InventoryVerb))
            .register(Box::new(LogTailVerb::new(log_root)))
            .register(Box::new(RestartVerb::new()))
            // The remote-plane bootstrap + safety verbs (self-gating; the pairing
            // authorizer classifies them bootstrap/e-stop-floor).
            .register(Box::new(ClaimVerb::new(shared.clone(), clock.clone())))
            .register(Box::new(PairVerb::new(shared.clone(), clock.clone())))
            // Establish access from a desk-carried, owner-signed grant.
            .register(Box::new(PresentGrantVerb::new(
                shared.clone(),
                clock.clone(),
            )))
            .register(Box::new(CodePairStartVerb::new(
                shared.clone(),
                sessions.clone(),
                clock.clone(),
            )))
            .register(Box::new(CodePairFinishVerb::new(
                shared.clone(),
                sessions,
                clock,
            )))
            .register(Box::new(EngageEstopVerb::new(lease)));

        let authorizer: Box<dyn Authorizer> = Box::new(PairingAuthorizer::from_shared(shared));
        let receipts = ReceiptLog::open(receipt_path).map_err(|e| {
            RemotedError::Ops(format!(
                "opening the ops receipt log {}: {e}",
                receipt_path.display()
            ))
        })?;
        let server = OpsServer::new(server_name, authorizer, verbs, receipts);
        Ok(OpsServing {
            server: Arc::new(server),
            deadline: OPS_SESSION_DEADLINE,
        })
    }

    /// Override the per-session deadline (tests only) so the deadline-unwedge +
    /// permit-release path can be pinned in milliseconds instead of 2 minutes.
    #[doc(hidden)]
    pub fn with_deadline_for_test(mut self, deadline: Duration) -> Self {
        self.deadline = deadline;
        self
    }

    /// Test-only: POISON the shared receipt-sink mutex (as a session that panicked
    /// mid hash-chain receipt write would), so the fail-closed poison-refusal path
    /// can be pinned without crafting a real mid-write panic. Delegates to `cerud`'s
    /// `OpsServer::poison_receipt_sink_for_test` — the receipt `Mutex` lives inside
    /// the shared `OpsServer`.
    ///
    /// **EXCLUDED FROM PRODUCTION** via `#[cfg(any(test, feature = "test-seam"))]`
    /// (the crate's `test-seam` feature forwards to `cerud/test-seam`): the whole
    /// poison-INJECTION path — this delegator AND the cerud seam it calls — is
    /// compiled out of a plain `cargo build`. The read-only poison CHECK the live
    /// `serve_session` uses (`OpsServer::receipt_sink_poisoned`) stays available.
    #[doc(hidden)]
    #[cfg(any(test, feature = "test-seam"))]
    pub fn poison_receipt_sink_for_test(&self) {
        self.server.poison_receipt_sink_for_test();
    }

    /// Serve ONE accepted `cerulion/ops/1` connection to its clean close (or the
    /// session deadline), CONCURRENTLY with every other in-flight session — there
    /// is no whole-session serialization. The shared [`OpsServer`] is
    /// `&self`-served and its receipt sink serializes only the brief append, so a
    /// stalled or hostile session (parked on a blocking read, holding NO lock)
    /// never delays another session — in particular never the `engage-estop`
    /// safety floor.
    ///
    /// The session — accepting the bidi stream AND running `serve_connection` — is
    /// bounded by ONE session-deadline window (the [`OPS_SESSION_DEADLINE`]
    /// default, or the [`OpsServing::with_deadline_for_test`] override): a peer
    /// that opens the connection then stalls is dropped at the deadline so its
    /// blocking task ends and its resources free. That bound is now a per-session
    /// resource guard, not a serialization point.
    ///
    /// If the shared receipt sink is POISONED (a prior session panicked mid hash-
    /// chain write), the session is refused fail-closed and the connection is
    /// closed with the accurate reason `ops receipt sink poisoned` — appending to a
    /// possibly-corrupt audit log is never attempted.
    pub async fn serve_session(self: Arc<Self>, accepted: Accepted) {
        let Accepted {
            connection,
            remote_id,
            ..
        } = accepted;
        let deadline = self.deadline;
        let caller = CallerIdentity::verified(hex::encode(remote_id.as_bytes()));

        // ONE overall deadline for the whole session (accept + serve).
        let overall = tokio::time::sleep(deadline);
        tokio::pin!(overall);

        // 1) Accept the ONE bidi stream (a peer that opens the connection but
        //    never opens a stream is dropped at the deadline — a resource bound,
        //    concurrent with every other session).
        let (send, recv) = tokio::select! {
            r = accept_frame_stream(&connection) => match r {
                Ok(pair) => pair,
                Err(e) => {
                    tracing::warn!(remote_id = %remote_id, error = %e,
                        "cerulion_remoted ops: failed to accept the bidi stream; closing");
                    close(&connection);
                    return;
                }
            },
            _ = &mut overall => {
                tracing::warn!(remote_id = %remote_id, deadline_secs = deadline.as_secs(),
                    "cerulion_remoted ops: no bidi stream within the deadline; closing");
                close(&connection);
                return;
            }
        };

        // 2) Bridge async QUIC → cerud's SYNC seam on a blocking task. cerud frames
        //    its own bytes inside the stream; QuicOpsStream is a raw byte pipe. The
        //    shared server is served via `&self`, so this session runs concurrently
        //    with every other — the brief receipt-append `Mutex` (in cerud) is the
        //    only serialization, and only for the append.
        let ops_stream = QuicOpsStream::new(send, recv);
        let server = self.server.clone();
        let mut join = tokio::task::spawn_blocking(move || {
            // FAIL-CLOSED on a POISONED receipt sink: a poisoned receipt `Mutex`
            // (inside the shared OpsServer) means a prior session PANICKED mid
            // hash-chain write — exactly when the audit log could be half-written.
            // Refuse to serve THIS session rather than risk appending to a
            // possibly-inconsistent chain; the daemon must restart (a fresh
            // `ReceiptLog::open` recovers the torn tail). The accurate cause is
            // named on the QUIC close below. cerud's `record_receipt` ALSO fails
            // each append closed, so an in-flight CONCURRENT session already past
            // this check never silently corrupts the chain either — this is the
            // session-level fast-fail + the accurate operator signal.
            if server.receipt_sink_poisoned() {
                Err(ReceiptSinkPoisoned)
            } else {
                Ok(server.serve_connection(OpsConnection {
                    stream: Box::new(ops_stream),
                    caller,
                }))
            }
        });

        tokio::select! {
            res = &mut join => match res {
                Ok(Ok(Ok(()))) => tracing::debug!(remote_id = %remote_id, "cerulion_remoted ops: session ended cleanly"),
                Ok(Ok(Err(e))) => tracing::warn!(remote_id = %remote_id, error = %e, "cerulion_remoted ops: session ended with error"),
                Ok(Err(ReceiptSinkPoisoned)) => {
                    tracing::error!(remote_id = %remote_id,
                        "cerulion_remoted ops: REFUSING to serve — the receipt sink mutex is POISONED \
                         (a prior session panicked mid hash-chain write); appending now could corrupt \
                         the audit log. Fail-closed; restart the daemon to recover the chain tail.");
                    // Name the ACTUAL cause on the QUIC close reason (NOT the generic
                    // deadline helper) so peer-side/operator diagnostics are accurate.
                    connection.close(0u32.into(), b"ops receipt sink poisoned");
                }
                Err(join_err) => tracing::error!(remote_id = %remote_id, error = %join_err, "cerulion_remoted ops: session task panicked"),
            },
            _ = &mut overall => {
                tracing::warn!(remote_id = %remote_id, deadline_secs = deadline.as_secs(),
                    "cerulion_remoted ops: session exceeded its deadline; closing the connection to \
                     unwedge the blocking session (an authed-then-stalled peer must not hold its \
                     session task/connection indefinitely) — other sessions are unaffected");
                close(&connection);
                // Now bounded: the blocking read errors on the reset stream and
                // serve_connection returns, freeing the session's blocking task.
                let _ = join.await;
            }
        }
    }
}

/// Close a QUIC connection with the ops-deadline application code + reason.
/// `0u32.into()` resolves to the connection's `VarInt` error-code type by
/// inference, so this needs no direct `iroh` dependency.
fn close(connection: &Connection) {
    connection.close(0u32.into(), b"ops session deadline");
}
