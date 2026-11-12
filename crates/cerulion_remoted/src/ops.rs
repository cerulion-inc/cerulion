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
//! The daemon's `restart` reads the robot's e-stop through a
//! [`cerud::verbs::restart::RestartGate`] over the shared [`EstopLease`]:
//! before its args are read and its stop command runs, and again after the
//! stop and before the start command. While the latch is engaged or an engage
//! is counted ([`EstopLease::holds`]), the first check answers
//! `unavailable: the robot's e-stop is engaged, so restart is refused` and runs
//! nothing, and the second answers `unavailable: ` followed by
//! [`RESTART_STOPPED_NOT_STARTED`] and runs no start command. Each check
//! takes the lease's lock at most once, for one read, and never holds it
//! across a command, so an engage after the second check does not stop the
//! graph the start command starts.
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
//! `engage-estop` runs on its OWN concurrent session: the safety EFFECT
//! (`lease.engage_estop`) runs BEFORE its audit receipt (e-stop is
//! non-mutating, so no intent receipt precedes it). Before the effect it waits
//! on the lease's lock for at most the one act-verb send that holds it and one
//! brief check by each running act verb: the engage is counted on the shared
//! [`EstopLease`] before it waits, and an act verb that reads the count under
//! the lock sends nothing. After the effect it waits for the brief receipt
//! append. No wait is for a whole session. This is "e-stop = permission floor,
//! any session, always wins" applied to the transport layer. Hash-chain
//! integrity holds under concurrent appends because the sink's `Mutex`
//! serializes each `record` as one atomic append.
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
use std::sync::Arc;
use std::time::Duration;

use cerulion_link::{accept_frame_stream, Accepted, Connection, QuicOpsStream};

use cerud::authz::Authorizer;
use cerud::error::{CerudError, CerudResult};
use cerud::receipt::ReceiptLog;
use cerud::server::OpsServer;
use cerud::transport::{CallerIdentity, OpsConnection};
use cerud::verbs::restart::{RestartGate, RestartStep};
use cerud::verbs::{InventoryVerb, LogTailVerb, RestartVerb, VerbHandler, VerbRegistry};
use cerulion_msgio::text::VerbError;

use crate::authorizer::PairingAuthorizer;
use crate::clock::RemotedClock;
use crate::error::RemotedError;
use crate::estop_lease::EstopLease;
use crate::estop_policy::{restart_check, RestartCheck};
use crate::host::OpsHost;
use crate::pairing_verbs::{
    ClaimVerb, CodePairFinishVerb, CodePairStartVerb, EngageEstopVerb, PairVerb, PresentGrantVerb,
    SharedCodePairSessions,
};
use crate::shutdown::ShutdownStage;
use crate::trust::SharedTrust;
use crate::verbs;

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

/// The daemon's `restart` refusal when the robot's e-stop holds after the
/// verb's stop command ran: the start command does not run. The verb's
/// error is `unavailable: ` followed by this text.
pub const RESTART_STOPPED_NOT_STARTED: &str = "the robot's e-stop was engaged during the restart, \
                                               so the graph was not started again after its stop";

/// The ops-plane serving context: ONE shared [`OpsServer`] (the mechanical +
/// bootstrap verb registry, the pairing authorizer over the live trust state, and
/// the hash-chained receipt sink). Built once at daemon boot;
/// [`OpsServing::serve_session`] drives each accepted `cerulion/ops/1` connection
/// through it CONCURRENTLY — the receipt sink serializes only the brief append,
/// so no session ever blocks another (see the module docs).
pub struct OpsServing {
    fence: Option<crate::serving_hold::AdmissionFence>,
    /// The code-pairing store the code-pair verbs use; each session is opened
    /// on it for its peer's key ([`SharedCodePairSessions::open_session`]).
    code_pairing: SharedCodePairSessions,
    /// Shared, `&self`-served across concurrent sessions. Its receipt sink is a
    /// brief-append `Mutex` (in `cerud`), so concurrent appends stay chain-valid
    /// without any whole-session serialization here.
    server: Arc<OpsServer>,
    /// The per-session wall bound (see [`OPS_SESSION_DEADLINE`]). Overridable in
    /// tests via [`OpsServing::with_deadline_for_test`] so the stalled-session
    /// unwedge can be pinned without a 2-minute wait.
    deadline: Duration,
    /// The stage the serving loop runs after its accept loop stops and before
    /// the daemon tears down ([`crate::shutdown`]).
    shutdown: ShutdownStage,
}

impl OpsServing {
    /// Build the ops-serving context: register `cerud`'s mechanical verbs +
    /// the remote-plane bootstrap/safety verbs, gate them with the pairing
    /// authorizer over `shared` (the LIVE trust state), and open the receipt log.
    ///
    /// `lease` is the shared control lease the `engage-estop` floor engages;
    /// the e-stop verb and the act verbs share one [`EstopLease`] over it, so
    /// an engage waiting for its lock is counted where the act verbs read it.
    /// A bare `Arc<Mutex<ControlLease>>` becomes an `EstopLease` with a count
    /// of its own; pass a clone of an `EstopLease` to share its count with
    /// another plane, as the daemon does with the rcp plane. `restart` reads
    /// the e-stop on `lease` before its stop and again before its start. This
    /// plane's `engage-estop` sends no `estop` to rcpd: the daemon adds that
    /// send from its rcp link ([`crate::rcp`]).
    /// `sessions` is the CPace fallback session store; `clock` is the robot's
    /// TRUSTED clock the pairing verbs verify against; `log_root` is the
    /// `log-tail` allow-list root; `receipt_path` is the audit log.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        shared: SharedTrust,
        lease: impl Into<EstopLease>,
        sessions: SharedCodePairSessions,
        clock: RemotedClock,
        receipt_path: &Path,
        log_root: PathBuf,
        server_name: impl Into<String>,
    ) -> Result<Self, RemotedError> {
        Self::for_host(
            OpsHost::Daemon {
                lease: lease.into(),
                rcpd: None,
            },
            shared,
            sessions,
            clock,
            receipt_path,
            log_root,
            server_name,
            None,
        )
    }

    /// [`OpsServing::new`] with the verb set of `host`: a library host registers
    /// [`DaemonOnlyVerb`] under the `restart`, `engage-estop` and act verb
    /// names, so the authorizer still classifies each call and a permitted
    /// caller gets a clear refusal instead of an unknown verb. With
    /// `serving_holds`, the daemon's native idle admission, every verb outside
    /// the bootstrap surface runs as a [`FencedVerb`].
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn for_host(
        host: OpsHost,
        shared: SharedTrust,
        sessions: SharedCodePairSessions,
        clock: RemotedClock,
        receipt_path: &Path,
        log_root: PathBuf,
        server_name: impl Into<String>,
        serving_holds: Option<crate::serving_hold::ServingHolds>,
    ) -> Result<Self, RemotedError> {
        let shutdown = ShutdownStage::default();
        #[cfg(feature = "test-seam")]
        crate::shutdown::register_probe_for_test(&shutdown, host.rcpd_estop());
        let fence = serving_holds.map(crate::serving_hold::AdmissionFence::new);
        let code_pairing = sessions.clone();
        let verbs = verb_registry(host, &shared, sessions, clock, log_root, fence.as_ref());
        let authorizer: Box<dyn Authorizer> = Box::new(PairingAuthorizer::from_shared(shared));
        let receipts = ReceiptLog::open(receipt_path).map_err(|e| {
            RemotedError::Ops(format!(
                "opening the ops receipt log {}: {e}",
                receipt_path.display()
            ))
        })?;
        let server = OpsServer::new(server_name, authorizer, verbs, receipts);
        Ok(OpsServing {
            fence,
            code_pairing,
            server: Arc::new(server),
            deadline: OPS_SESSION_DEADLINE,
            shutdown,
        })
    }

    /// The stage the serving loop runs after its accept loop stops, while this
    /// plane's receipt sink is alive (the endpoint too after a stop request;
    /// see [`crate::shutdown`]); each
    /// [`ShutdownParticipant`](crate::shutdown::ShutdownParticipant) registered
    /// on it is driven to settlement there.
    pub fn shutdown_stage(&self) -> &ShutdownStage {
        &self.shutdown
    }

    /// Native idle admission for a daemon; absent on an embedded ops host.
    pub fn serving_holds(&self) -> Option<&crate::serving_hold::ServingHolds> {
        self.fence
            .as_ref()
            .map(crate::serving_hold::AdmissionFence::holds)
    }

    /// The admission the daemon's accept loop and its fenced verbs share,
    /// over [`OpsServing::serving_holds`].
    pub(crate) fn admission_fence(&self) -> Option<&crate::serving_hold::AdmissionFence> {
        self.fence.as_ref()
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
    ///
    /// The session's blocking run is opened on the code-pairing store for the
    /// peer's key ([`SharedCodePairSessions::open_session`]) and closed after
    /// `serve_connection` returns, also on the deadline path, so a pending
    /// code-pairing try of that key is counted failed only once no session of
    /// the key is open and no `code-pair-finish` for it can arrive.
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
        let code_pairing = self.code_pairing.clone();
        let peer_key = *remote_id.as_bytes();
        let mut join = tokio::task::spawn_blocking(move || {
            // Open for the session's whole blocking run, and dropped after
            // `serve_connection` returned, so after any `code-pair-finish`
            // this session sends: a code-pairing try this peer left pending
            // is counted failed only once no session of its key is open. The
            // store's lock is taken here, on the blocking thread.
            let _code_pairing = code_pairing.open_session(peer_key);
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

/// The ops verb registry of `host`: cerud's mechanical verbs, the remote-plane
/// bootstrap and safety verbs and the act verbs, with `restart`, `engage-estop`
/// and the act verbs refused by name on an embedded host. Both hosts register
/// the same names. The daemon's `restart` carries an [`EstopRestartGate`] over
/// the daemon's lease. With `fence`, each verb the authorizer does not class
/// as bootstrap is registered as a [`FencedVerb`] behind it.
fn verb_registry(
    host: OpsHost,
    shared: &SharedTrust,
    sessions: SharedCodePairSessions,
    clock: RemotedClock,
    log_root: PathBuf,
    fence: Option<&crate::serving_hold::AdmissionFence>,
) -> VerbRegistry {
    let (restart, engage_estop, act_lease): (Box<dyn VerbHandler>, Box<dyn VerbHandler>, _) =
        match host {
            OpsHost::Daemon { lease, rcpd } => {
                // One `EstopLease` for the plane: the e-stop verb engages through
                // it and the act verbs read its count before each send. With
                // `rcpd`, the e-stop verb also sends an `estop` into rcpd.
                let engage = EngageEstopVerb::new(lease.clone());
                let engage = match rcpd {
                    Some(rcpd) => engage.with_rcpd_estop(rcpd),
                    None => engage,
                };
                let restart =
                    RestartVerb::new().with_gate(Box::new(EstopRestartGate(lease.clone())));
                (Box::new(restart), Box::new(engage), Some(lease))
            }
            OpsHost::Embedded => (
                Box::new(DaemonOnlyVerb::RESTART),
                Box::new(DaemonOnlyVerb::ENGAGE_ESTOP),
                None,
            ),
        };
    let mut handlers: Vec<Box<dyn VerbHandler>> = vec![
        // cerud's mechanical ops verbs.
        Box::new(InventoryVerb),
        Box::new(LogTailVerb::new(log_root)),
        restart,
        // The remote-plane bootstrap + safety verbs (self-gating; the pairing
        // authorizer classifies them bootstrap/e-stop-floor).
        Box::new(ClaimVerb::new(shared.clone(), clock.clone())),
        Box::new(PairVerb::new(shared.clone(), clock.clone())),
        // Establish access from a desk-carried, owner-signed grant.
        Box::new(PresentGrantVerb::new(shared.clone(), clock.clone())),
        Box::new(CodePairStartVerb::new(shared.clone(), sessions.clone())),
        Box::new(CodePairFinishVerb::new(shared.clone(), sessions, clock)),
        engage_estop,
        Box::new(crate::robot_settings::CloudAccessVerb::new(shared.clone())),
    ];
    // The act verbs: publish, service call, snapshot, one message.
    match act_lease {
        Some(lease) => handlers.extend(crate::act_verbs::handlers(lease)),
        None => handlers.extend([
            Box::new(DaemonOnlyVerb::TOPIC_PUBLISH) as Box<dyn VerbHandler>,
            Box::new(DaemonOnlyVerb::SERVICE_CALL),
            Box::new(DaemonOnlyVerb::TOPIC_SNAPSHOT),
            Box::new(DaemonOnlyVerb::TOPIC_ECHO_ONCE),
        ]),
    }
    let mut verbs = VerbRegistry::new();
    for verb in handlers {
        verbs.register(match fence {
            Some(fence) if !crate::authorizer::is_bootstrap_verb(verb.name()) => {
                Box::new(FencedVerb {
                    floor: crate::authorizer::is_estop_floor_verb(verb.name()),
                    verb,
                    fence: fence.clone(),
                })
            }
            _ => verb,
        });
    }
    verbs
}

/// A verb outside the bootstrap surface, run under a serving-hold work guard
/// taken before the verb and released after it returns, so idle teardown
/// waits for the call. A session admitted only to the bootstrap surface holds
/// no guard of its own, and a key it claims or pairs may then call these verbs
/// on the same connection; a paired session already holds one, so its calls
/// are never refused here. When the fence cannot be taken (netd holds the
/// exclusive fence for a committed idle stop, or the lock call fails), the
/// call is refused ([`stopping_refusal`]), the refusal is logged through the
/// fence's regime and the verb does not run. The e-stop floor
/// (`engage-estop`) is the exception: it takes the guard when it can and runs
/// either way, so a committed idle stop never refuses an e-stop.
struct FencedVerb {
    verb: Box<dyn VerbHandler>,
    fence: crate::serving_hold::AdmissionFence,
    /// The verb is the e-stop floor, which an idle stop never refuses.
    floor: bool,
}

impl FencedVerb {
    fn guard(&self) -> CerudResult<Option<crate::serving_hold::ServingHold>> {
        match self.fence.admit() {
            Some(hold) => Ok(Some(hold)),
            None if self.floor => Ok(None),
            None => Err(stopping_refusal(self.verb.name())),
        }
    }
}

impl VerbHandler for FencedVerb {
    fn name(&self) -> &'static str {
        self.verb.name()
    }
    fn is_mutating(&self) -> bool {
        self.verb.is_mutating()
    }
    fn execute(&self, args: &serde_json::Value) -> CerudResult<serde_json::Value> {
        let _hold = self.guard()?;
        self.verb.execute(args)
    }
    fn execute_with_caller(
        &self,
        caller: &CallerIdentity,
        args: &serde_json::Value,
    ) -> CerudResult<serde_json::Value> {
        let _hold = self.guard()?;
        self.verb.execute_with_caller(caller, args)
    }
}

/// The refusal of a [`FencedVerb`] call to `verb` while the server's idle stop
/// is committed: `unavailable: the remote server is stopping, so <verb> is
/// refused`.
fn stopping_refusal(verb: &str) -> CerudError {
    CerudError::Verb(
        VerbError::Unavailable(format!(
            "the remote server is stopping, so {verb} is refused"
        ))
        .to_classed(),
    )
}

/// The daemon's `restart` gate over the robot's e-stop on its lease. Each
/// check reads [`EstopLease::holds`] once, taking the lease's lock at most
/// once, for that read, and [`restart_check`] decides: a refused stop
/// answers the act verbs' refusal for `restart`, and a refused start answers
/// `unavailable: ` followed by [`RESTART_STOPPED_NOT_STARTED`].
struct EstopRestartGate(EstopLease);

impl RestartGate for EstopRestartGate {
    fn check(&self, step: RestartStep) -> CerudResult<()> {
        match restart_check(step, self.0.holds()) {
            RestartCheck::Go => Ok(()),
            RestartCheck::RefuseStop => Err(crate::act_verbs::estop_refusal(verbs::RESTART)),
            RestartCheck::RefuseStart => Err(CerudError::Verb(
                VerbError::Unavailable(RESTART_STOPPED_NOT_STARTED.to_string()).to_classed(),
            )),
        }
    }
}

/// A verb a library host refuses by name: it acts on something only the
/// `cerulion-remoted` daemon has. Registered under the daemon verb's own name, so
/// the authorizer classifies the call exactly as on the daemon (an unpaired caller
/// is still denied) and a permitted caller gets an `unsupported_host` error that
/// says why. It has no side effect, so it is not mutating and writes no intent
/// receipt.
pub(crate) struct DaemonOnlyVerb {
    name: &'static str,
    why: &'static str,
}

impl DaemonOnlyVerb {
    const RESTART: Self = DaemonOnlyVerb {
        name: verbs::RESTART,
        why: "it restarts a graph's systemd unit",
    };
    const ENGAGE_ESTOP: Self = DaemonOnlyVerb {
        name: verbs::ENGAGE_ESTOP,
        why: "it engages the daemon's control lease and asks for an incident capture, so \
              this host would report an e-stop that stops nothing; stop through the host's \
              own stop path",
    };
    const TOPIC_PUBLISH: Self = DaemonOnlyVerb {
        name: crate::act_verbs::TOPIC_PUBLISH,
        why: ACT_WRITE_WHY,
    };
    const SERVICE_CALL: Self = DaemonOnlyVerb {
        name: crate::act_verbs::SERVICE_CALL,
        why: ACT_WRITE_WHY,
    };
    const TOPIC_SNAPSHOT: Self = DaemonOnlyVerb {
        name: crate::act_verbs::TOPIC_SNAPSHOT,
        why: ACT_READ_WHY,
    };
    const TOPIC_ECHO_ONCE: Self = DaemonOnlyVerb {
        name: crate::act_verbs::TOPIC_ECHO_ONCE,
        why: ACT_READ_WHY,
    };
}

/// Why a library host refuses `topic-publish` and `service-call`.
const ACT_WRITE_WHY: &str = "it sends through the process's transport singleton only while the \
                             daemon's e-stop is clear, and this host serves no e-stop that \
                             could stop it";
/// Why a library host refuses `topic-snapshot` and `topic-echo-once`.
const ACT_READ_WHY: &str = "it reads through the process's transport singleton and the daemon's \
                            schema directory, not this host's transport and schemas; read the \
                            topic over the wire plane";

impl VerbHandler for DaemonOnlyVerb {
    fn name(&self) -> &'static str {
        self.name
    }
    fn is_mutating(&self) -> bool {
        false
    }
    fn execute(&self, _args: &serde_json::Value) -> CerudResult<serde_json::Value> {
        Err(CerudError::UnsupportedHost(format!(
            "{} is served only by the cerulion-remoted daemon: {}. This host serves the \
             remote plane as a library",
            self.name, self.why
        )))
    }
}

/// Close a QUIC connection with the ops-deadline application code + reason.
/// `0u32.into()` resolves to the connection's `VarInt` error-code type by
/// inference, so this needs no direct `iroh` dependency.
fn close(connection: &Connection) {
    connection.close(0u32.into(), b"ops session deadline");
}

#[cfg(test)]
mod verb_set_tests {
    use super::*;
    use crate::device_index::DeviceAccountIndex;
    use cerulion_pairing::format::{PublicKey, RobotId, RootSet};
    use cerulion_pairing::verify::TrustStore;

    fn registry(host: OpsHost) -> VerbRegistry {
        registry_with(host, None)
    }

    /// The registry of `host`, its verbs outside the bootstrap surface behind
    /// `fence` when given.
    fn registry_with(
        host: OpsHost,
        fence: Option<&crate::serving_hold::AdmissionFence>,
    ) -> VerbRegistry {
        let root = PublicKey(
            ed25519_dalek::SigningKey::from_bytes(&[1; 32])
                .verifying_key()
                .to_bytes(),
        );
        let store = TrustStore::provision(
            RobotId([5; 32]),
            PublicKey([7; 32]),
            RootSet::new(vec![root], 1).unwrap(),
            b"verb-set-chassis-secret",
            0,
        )
        .unwrap();
        let shared = SharedTrust::new(store, DeviceAccountIndex::new(), Vec::new());
        verb_registry(
            host,
            &shared,
            SharedCodePairSessions::new(),
            RemotedClock::fixed(0),
            PathBuf::from("logs"),
            fence,
        )
    }

    fn names(host: OpsHost) -> Vec<String> {
        let registry = registry(host);
        let mut names: Vec<String> = registry.names().into_iter().map(str::to_owned).collect();
        names.sort();
        names
    }

    #[test]
    fn an_embedded_host_registers_every_daemon_verb_name() {
        // The authorizer classifies by name before dispatch, so the refusing
        // stand-ins must carry the daemon verbs' exact names.
        let daemon = names(OpsHost::daemon());
        let expected = [
            "claim",
            "cloud-access",
            "code-pair-finish",
            "code-pair-start",
            "engage-estop",
            "inventory",
            "log-tail",
            "pair",
            "present-grant",
            "restart",
            "service-call",
            "topic-echo-once",
            "topic-publish",
            "topic-snapshot",
        ];
        assert_eq!(daemon, expected);
        assert_eq!(names(OpsHost::Embedded), expected);
    }

    #[test]
    fn the_act_verbs_read_the_engage_count_the_daemon_lends() {
        use std::time::{Duration, Instant};
        let host = OpsHost::daemon();
        let lent = host.estop_lease().expect("the daemon has an e-stop lease");
        let registry = registry(host);
        let publish = registry.get(crate::act_verbs::TOPIC_PUBLISH).unwrap();
        let args = serde_json::json!({
            "topic": "/ops_unit/latch",
            "type": "nope_msgs/msg/Nope",
            "message": {},
        });
        let (tx, rx) = std::sync::mpsc::channel();
        // An engage through the lent lease, as the rcp plane makes, waits for
        // the lock this test holds; `topic-publish` must see it counted and
        // refuse without taking that lock. Every guard is dropped before an
        // assertion, so a failure never strands the scoped threads.
        let held = lent.lock().unwrap();
        let (counted, refusal) = std::thread::scope(|scope| {
            scope.spawn(|| {
                lent.engage("iroh:ab");
            });
            let until = Instant::now() + Duration::from_secs(10);
            while !lent.engage_pending() && Instant::now() < until {
                std::thread::sleep(Duration::from_millis(1));
            }
            let counted = lent.engage_pending();
            let args = &args;
            scope.spawn(move || {
                let _ = tx.send(publish.execute(args));
            });
            let refusal = rx.recv_timeout(Duration::from_secs(10));
            drop(held);
            (counted, refusal)
        });
        assert!(counted, "the lent lease did not count the engage");
        let err = refusal
            .expect("topic-publish waited for the held lock: it does not read the lent count")
            .unwrap_err();
        assert!(
            err.to_string()
                .contains("the robot's e-stop is engaged, so topic-publish is refused"),
            "{err}"
        );
    }

    /// The text of a verb error; panics on any other error.
    fn verb_message(e: CerudError) -> String {
        match e {
            CerudError::Verb(m) => m,
            other => panic!("not a verb error: {other:?}"),
        }
    }

    #[test]
    fn the_daemon_refuses_restart_under_the_estop_before_it_reads_the_args() {
        let host = OpsHost::daemon();
        let lent = host.estop_lease().expect("the daemon has an e-stop lease");
        let registry = registry(host);
        let restart = registry.get(verbs::RESTART).unwrap();
        // `{}` has no `graph`: an ungated `restart` answers the args error.
        let clear = verb_message(restart.execute(&serde_json::json!({})).unwrap_err());
        assert!(clear.starts_with("invalid restart args"), "clear: {clear}");
        lent.lock().unwrap().engage_estop("test");
        let engaged = verb_message(restart.execute(&serde_json::json!({})).unwrap_err());
        assert_eq!(
            engaged,
            "unavailable: the robot's e-stop is engaged, so restart is refused"
        );
    }

    #[test]
    fn the_daemon_refuses_restart_on_a_counted_engage_without_the_lease_lock() {
        use std::time::{Duration, Instant};
        let host = OpsHost::daemon();
        let lent = host.estop_lease().expect("the daemon has an e-stop lease");
        let registry = registry(host);
        let restart = registry.get(verbs::RESTART).unwrap();
        let args = serde_json::json!({});
        let (tx, rx) = std::sync::mpsc::channel();
        // An engage through the lent lease waits for the lock this test holds;
        // `restart` must see it counted and refuse without taking that lock.
        // Every guard is dropped before an assertion, so a failure never
        // strands the scoped threads.
        let held = lent.lock().unwrap();
        let (counted, refusal) = std::thread::scope(|scope| {
            scope.spawn(|| {
                lent.engage("iroh:ab");
            });
            let until = Instant::now() + Duration::from_secs(10);
            while !lent.engage_pending() && Instant::now() < until {
                std::thread::sleep(Duration::from_millis(1));
            }
            let counted = lent.engage_pending();
            let args = &args;
            scope.spawn(move || {
                let _ = tx.send(restart.execute(args));
            });
            let refusal = rx.recv_timeout(Duration::from_secs(10));
            drop(held);
            (counted, refusal)
        });
        assert!(counted, "the lent lease did not count the engage");
        let err = refusal
            .expect("restart waited for the held lock: it does not read the lent count")
            .unwrap_err();
        assert_eq!(
            verb_message(err),
            "unavailable: the robot's e-stop is engaged, so restart is refused"
        );
    }

    /// The argv list an [`EngagingRunner`] records into.
    type Calls = Arc<std::sync::Mutex<Vec<Vec<String>>>>;

    /// A runner that records each argv and engages `engage_on_stop`, when it
    /// is set, as it is handed the stop command.
    struct EngagingRunner {
        engage_on_stop: Option<EstopLease>,
        calls: Calls,
    }

    impl cerud::verbs::restart::RestartRunner for EngagingRunner {
        fn supported(&self) -> bool {
            true
        }
        fn run(&self, argv: &[String]) -> CerudResult<()> {
            self.calls.lock().unwrap().push(argv.to_vec());
            if let Some(lease) = &self.engage_on_stop {
                if argv.first().map(String::as_str) == Some("systemctl") {
                    lease.engage("iroh:ab");
                }
            }
            Ok(())
        }
    }

    /// A `restart` gated on `lease` over an [`EngagingRunner`], with the list
    /// it records into.
    fn gated_restart(lease: &EstopLease, engage_on_stop: bool) -> (RestartVerb, Calls) {
        let calls = Arc::new(std::sync::Mutex::new(Vec::new()));
        let runner = EngagingRunner {
            engage_on_stop: engage_on_stop.then(|| lease.clone()),
            calls: calls.clone(),
        };
        let verb = RestartVerb::with_runner(Box::new(runner))
            .with_gate(Box::new(EstopRestartGate(lease.clone())));
        (verb, calls)
    }

    fn argv(words: &[&str]) -> Vec<String> {
        words.iter().map(|w| w.to_string()).collect()
    }

    #[test]
    fn an_engage_during_the_restart_stop_runs_no_start_command() {
        let stop = argv(&["systemctl", "stop", "cerulion-graph-g1"]);
        let start = argv(&[
            "systemd-run",
            "--unit=cerulion-graph-g1",
            "--collect",
            "cerulion",
            "graph",
            "run",
            "g1",
        ]);
        let args = serde_json::json!({"graph": "g1"});

        // Clear throughout: both steps run.
        let lease = OpsHost::daemon().estop_lease().unwrap();
        let (verb, calls) = gated_restart(&lease, false);
        let result = verb.execute(&args).unwrap();
        assert_eq!(result["restarted"], serde_json::json!(true), "{result}");
        assert_eq!(*calls.lock().unwrap(), vec![stop.clone(), start]);

        // Engaged before the call: nothing runs.
        let lease = OpsHost::daemon().estop_lease().unwrap();
        lease.engage("iroh:ab");
        let (verb, calls) = gated_restart(&lease, false);
        let err = verb_message(verb.execute(&args).unwrap_err());
        assert_eq!(
            err,
            "unavailable: the robot's e-stop is engaged, so restart is refused"
        );
        assert!(calls.lock().unwrap().is_empty(), "a command ran");

        // Engaged while the stop runs: the start is refused.
        let lease = OpsHost::daemon().estop_lease().unwrap();
        let (verb, calls) = gated_restart(&lease, true);
        let err = verb_message(verb.execute(&args).unwrap_err());
        assert_eq!(
            err,
            format!("unavailable: {RESTART_STOPPED_NOT_STARTED}"),
            "{err}"
        );
        assert_eq!(
            err,
            "unavailable: the robot's e-stop was engaged during the restart, so the graph \
             was not started again after its stop"
        );
        assert_eq!(*calls.lock().unwrap(), vec![stop]);
    }

    /// The daemon's verbs outside the bootstrap surface, by hand.
    const FENCED: [&str; 9] = [
        "cloud-access",
        "engage-estop",
        "inventory",
        "log-tail",
        "restart",
        "service-call",
        "topic-echo-once",
        "topic-publish",
        "topic-snapshot",
    ];
    /// The bootstrap surface, by hand.
    const BOOTSTRAP: [&str; 5] = [
        "claim",
        "code-pair-finish",
        "code-pair-start",
        "pair",
        "present-grant",
    ];

    /// The bootstrap predicate splits the daemon's verb names into exactly
    /// the two hand-written sets.
    #[test]
    fn only_the_five_pairing_verbs_are_on_the_bootstrap_surface() {
        let mut split: Vec<(&str, bool)> = FENCED
            .iter()
            .map(|verb| (*verb, false))
            .chain(BOOTSTRAP.iter().map(|verb| (*verb, true)))
            .collect();
        split.sort();
        let names = names(OpsHost::daemon());
        assert_eq!(
            names,
            split.iter().map(|(verb, _)| *verb).collect::<Vec<_>>()
        );
        for (verb, bootstrap) in split {
            assert_eq!(
                crate::authorizer::is_bootstrap_verb(verb),
                bootstrap,
                "{verb}"
            );
        }
    }

    /// While netd holds the exclusive fence for a committed idle stop, each
    /// daemon verb outside the bootstrap surface is refused with the stopping
    /// text before it runs, except the e-stop floor: `engage-estop` still runs
    /// and latches the e-stop. Each bootstrap verb still runs, and once the
    /// stop is withdrawn the same registry runs a fenced verb again.
    #[test]
    fn a_committed_idle_stop_refuses_every_verb_outside_the_bootstrap_surface() {
        use cerulion_netd::serving_hold::IdleFence;
        let dir = tempfile::tempdir().unwrap();
        let path = crate::serving_hold::fence_path(dir.path());
        let fence = crate::serving_hold::AdmissionFence::new(
            crate::serving_hold::ServingHolds::open(&path).unwrap(),
        );
        let host = OpsHost::daemon();
        let lease = host.estop_lease().expect("the daemon has an e-stop lease");
        let registry = registry_with(host, Some(&fence));
        let caller = CallerIdentity::verified("ab".repeat(32));
        let args = serde_json::json!({});
        let idle = IdleFence::try_acquire(&path)
            .unwrap()
            .expect("no work holds the fence");
        for verb in FENCED.iter().filter(|verb| **verb != "engage-estop") {
            let err = registry
                .get(verb)
                .unwrap()
                .execute_with_caller(&caller, &args)
                .unwrap_err();
            assert_eq!(
                verb_message(err),
                format!("unavailable: the remote server is stopping, so {verb} is refused")
            );
        }
        assert!(!lease.holds(), "no refused verb reached the latch");
        let engaged = registry
            .get(verbs::ENGAGE_ESTOP)
            .unwrap()
            .execute_with_caller(&caller, &args)
            .expect("an idle stop never refuses the e-stop floor");
        assert_eq!(engaged["engaged"], serde_json::json!(true), "{engaged}");
        assert!(
            lease.holds(),
            "the e-stop latched during the committed stop"
        );
        for verb in BOOTSTRAP {
            let result = registry
                .get(verb)
                .unwrap()
                .execute_with_caller(&caller, &args);
            assert!(
                !format!("{result:?}").contains("remote server is stopping"),
                "{verb} ran its own handler: {result:?}"
            );
        }
        assert!(registry.get(verbs::RESTART).unwrap().is_mutating());
        assert!(!registry.get(verbs::INVENTORY).unwrap().is_mutating());
        drop(idle);
        let inventory = registry
            .get(verbs::INVENTORY)
            .unwrap()
            .execute(&args)
            .expect("inventory runs once the idle stop is withdrawn");
        assert!(inventory["arch"].is_string(), "{inventory}");
    }

    /// A fenced call holds the serving fence from before its verb runs until
    /// the verb returns, on both dispatch paths, and releases it afterwards;
    /// the same verb unfenced runs with the fence free.
    #[test]
    fn a_fenced_call_holds_the_serving_fence_while_its_verb_runs() {
        use cerulion_netd::serving_hold::IdleFence;
        struct Probe(PathBuf);
        impl VerbHandler for Probe {
            fn name(&self) -> &'static str {
                verbs::INVENTORY
            }
            fn is_mutating(&self) -> bool {
                false
            }
            fn execute(&self, _args: &serde_json::Value) -> CerudResult<serde_json::Value> {
                let free = IdleFence::try_acquire(&self.0)
                    .expect("the fence opens")
                    .is_some();
                Ok(serde_json::json!({ "fence_free": free }))
            }
        }
        let dir = tempfile::tempdir().unwrap();
        let path = crate::serving_hold::fence_path(dir.path());
        let verb = FencedVerb {
            verb: Box::new(Probe(path.clone())),
            fence: crate::serving_hold::AdmissionFence::new(
                crate::serving_hold::ServingHolds::open(&path).unwrap(),
            ),
            floor: false,
        };
        let args = serde_json::json!({});
        let held = serde_json::json!({ "fence_free": false });
        assert_eq!(verb.execute(&args).unwrap(), held);
        assert_eq!(
            verb.execute_with_caller(&CallerIdentity::verified("ab".repeat(32)), &args)
                .unwrap(),
            held
        );
        assert!(
            IdleFence::try_acquire(&path).unwrap().is_some(),
            "the call released the fence"
        );
        assert_eq!(
            Probe(path.clone()).execute(&args).unwrap(),
            serde_json::json!({ "fence_free": true })
        );
    }

    #[test]
    fn an_embedded_host_refuses_each_act_verb_by_name() {
        let registry = registry(OpsHost::Embedded);
        let args = serde_json::json!({
            "topic": "/chatter",
            "type": "std_msgs/msg/String",
            "message": {"data": "hi"},
            "service": "/add",
        });
        for verb in [
            crate::act_verbs::TOPIC_PUBLISH,
            crate::act_verbs::SERVICE_CALL,
            crate::act_verbs::TOPIC_SNAPSHOT,
            crate::act_verbs::TOPIC_ECHO_ONCE,
        ] {
            let handler = registry.get(verb).unwrap();
            assert!(!handler.is_mutating(), "{verb} writes no intent receipt");
            let err = handler.execute(&args).unwrap_err();
            assert!(
                matches!(err, CerudError::UnsupportedHost(_)),
                "{verb}: {err:?}"
            );
            assert!(
                err.to_string().contains(&format!(
                    "{verb} is served only by the cerulion-remoted daemon"
                )),
                "{verb}: {err}"
            );
        }
    }
}
