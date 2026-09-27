// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion-netd` CLIENT — the consumer half of the demand
//! plane.
//!
//! Every desk consumer that wants a remote robot's topic (vizd's `attach_remote`,
//! `cerulion topic echo`/`info`/`hz`, a user graph's ingress)
//! goes through THIS client instead of opening its OWN zenoh session and
//! calling [`register_ingress_topic`](cerulion_core::transport::TransportManager::register_ingress_topic).
//! It CONNECTS to the one-per-computer `cerulion-netd` daemon over the well-known
//! UDS control socket — SPAWNING the daemon detached if it is not yet running (the
//! *first-consumer-spawns-detached* lifecycle) — reads the [`Hello`] version
//! banner, and exchanges the [`Request::Demand`]/[`Request::Release`] verbs.
//!
//! # The pairing guarantee (why demands never leak)
//!
//! netd refcounts demands **per connection**, and **the UDS connection close IS an
//! implicit release** (the crash-safe refcount — [`crate::registry`]). So a
//! [`NetdClient`] holds its demands alive EXACTLY as long as it is alive: dropping
//! it (a clean exit, a panic, a `SIGINT`/`SIGKILL` — anything that closes the fd)
//! releases every demand it held, and netd's refcount-0 teardown tears the
//! shared mirror down when the LAST consumer leaves. A consumer therefore gets a
//! leak-free release on EVERY exit path just by keeping the client alive for as
//! long as it needs the topic and letting it drop — no `Drop` bookkeeping, no
//! signal handler. [`NetdClient::release`] is the explicit EARLY release (vizd's
//! `detach`) for a consumer that outlives one topic's need.
//!
//! # netd-not-running: SPAWN (never degrade, never silently succeed)
//!
//! The first consumer SPAWNS netd detached, then every
//! later consumer connects to the running one. [`NetdClient::connect_or_spawn`]
//! tries to connect; on a not-running socket it spawns `cerulion-netd` detached
//! (into its OWN session via `setsid`, so a `Ctrl-C` on `topic echo` never kills
//! the shared daemon — the connection close releases the demand instead) and
//! retries the connect under a bounded budget. The daemon's `flock` singleton
//! ([`crate::hygiene`]) makes concurrent spawns safe: N racing consumers all spawn,
//! exactly one wins the lock and the rest exit `AddrInUse`, and every consumer's
//! retry-connect finds the winner. A spawn/connect that still fails after the
//! budget is a LOUD [`ClientError`] — NEVER a silent fall-back to a private
//! per-consumer mirror (which would re-introduce the exact collision + double-
//! network-crossing the shared daemon dissolves).

use std::io::{self, BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::os::unix::process::CommandExt;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

use cerulion_core::{CatalogReply, SchemaReply};

use cerulion_core::{GatewayPlan, SchemaServing};

mod bounded;

use crate::convergence::{Converged, ConvergenceWait, WaitDecision, WaitOutcome};
use crate::hygiene::{default_socket_path, SOCKET_ENV};
use crate::protocol::{
    classify_control_line, CatalogChanged, ControlLine, DemandResponse, DiscoveryState,
    EgressReleaseResponse, EgressResponse, Hello, ReleaseResponse, Request, Response,
    SubscribeCatalogResponse, HELLO_MARKER, PROTOCOL_VERSION, RUNS_MIN_DAEMON_VERSION,
};
use crate::query::{CatalogGather, RunsAnswer, RunsGather, SchemaGather};

/// The env var overriding the `cerulion-netd` BINARY the client spawns (an
/// absolute path). Unset → the client looks for `cerulion-netd` alongside the
/// current executable (the sibling-binary install layout: `cerulion`,
/// `cerulion-vizd`, and `cerulion-netd` share one bin dir). Set by tests to point
/// at the freshly-built daemon.
///
/// # Wrapper scripts MUST `exec`
///
/// If you point this at a wrapper (a shell script that sets up an environment and then
/// runs the daemon), the wrapper must `exec` the real binary rather than run it as a
/// child and wait. The readiness wait watches the process it spawned: a
/// non-`exec` wrapper exits as soon as it has forked, and the client reads that exit as
/// "the daemon died", cutting the readiness wait from its full ceiling (10 s) to the
/// post-exit grace (~500 ms) and reporting a startup failure for a daemon that is in
/// fact still booting. `exec` keeps the spawned PID *being* the daemon, which is
/// what the liveness discriminator assumes. (Behavior is unchanged either way on a fast
/// boot — the socket appears inside the grace and the wait succeeds; the trap only bites
/// on a slow one.)
pub const NETD_BIN_ENV: &str = "CERULION_NETD_BIN";

/// The default `cerulion-netd` binary name (looked up alongside `current_exe`).
const NETD_BIN_NAME: &str = "cerulion-netd";

/// The CEILING on how long [`NetdClient::connect_or_spawn`] waits for a
/// freshly spawned daemon's control socket to become connectable.
///
/// This is a READINESS wait, not an attempt budget. A fixed
/// `40 × 25 ms` retry count has a ~1 s total that sits right on top of the daemon's real cold
/// boot (iceoryx2 transport init + the startup dead-node sweep measured ~1.1 s
/// on a desk with ~50 stale `/tmp/iceoryx2/nodes` entries), so the first consumer of a
/// remote topic would fail with "it did not come
/// up in time" while the daemon it had just spawned was seconds from listening. A fixed
/// count silently re-breaks the moment startup grows again; a generous CEILING plus a
/// tight poll does not, and it costs nothing on the healthy path (the loop exits on the
/// first successful connect, and a daemon that is ALREADY running never reaches it —
/// that is the fast path above the spawn).
///
/// Generous on purpose: the failure mode this bounds is a WEDGED spawn (a consumer
/// hanging forever), and 10 s is still an unmistakable, loud, bounded failure. The
/// common "the daemon died" case does not wait for it at all — [`SPAWN_POST_EXIT_GRACE`]
/// cuts the wait short as soon as the spawned child is observed to have exited.
///
/// # Composition with the retry — the real worst case is ~2×
///
/// [`NetdClient::connect_or_spawn_at`] retries the WHOLE connect-or-spawn once when the
/// first attempt hit the daemon idle-exit race, and each attempt carries its own
/// readiness wait. So the worst-case wall time a caller can observe is ~2 ×
/// `SPAWN_READY_TIMEOUT` (~20 s), not 10 s. That is deliberate — both attempts expiring
/// means two consecutive wedged spawns, which is a genuinely broken install worth
/// waiting to diagnose precisely — but it is the number to hold when tuning this
/// ceiling, and any caller-side deadline must budget for it.
const SPAWN_READY_TIMEOUT: Duration = Duration::from_secs(10);

/// How long a first-use readiness wait may stay silent on the QUIET verbs
/// before it is reported as a stall.
///
/// The first-use spawn line itself rides `info!`, which the one-shot verbs
/// (`topic echo`/`info`/`hz`, `schema info`; default filter `cerulion=warn`)
/// do not show, so on a fresh desk they would otherwise sit silent for the
/// whole [`SPAWN_READY_TIMEOUT`]: indistinguishable from a hang. Past this
/// bound the wait emits ONE `warn!`. The bound clears a real cold boot
/// (measured ~1.1 s: transport init + the startup dead-node sweep) with
/// margin, so the healthy first use draws no warning, while staying a small
/// fraction of the ceiling so the notice arrives while a user is still
/// looking. Pinned against both neighbours in `shipped_spawn_wait_constants_...`.
///
/// CALIBRATION, stated at its real strength: the ~1.1 s figure is ONE desk-class
/// machine. A slower target (an embedded board, a loaded CI runner) can spend
/// longer than this bound on a perfectly healthy first use and will then draw
/// the warn, which is why its text names the bound instead of asserting what a
/// normal boot takes. The constants pin only bounds this from BELOW (1.5 s).
/// Measuring a cold boot on the slowest supported target, and raising this if
/// that boot outlives it, is an open item.
const SPAWN_STALL_NOTICE_AFTER: Duration = Duration::from_secs(2);

/// The poll interval while waiting for the spawned daemon's socket. Tight, so a
/// daemon that comes up quickly is connected to quickly (the whole wait is a sleep
/// loop on an idle thread — the interval only bounds the post-readiness latency).
const SPAWN_POLL_INTERVAL: Duration = Duration::from_millis(20);

/// Fail-fast: once the spawned child has been observed to EXIT, how much longer
/// to keep polling before giving up (instead of burning the full
/// [`SPAWN_READY_TIMEOUT`]).
///
/// It is not zero because a child exiting is NOT proof that no daemon is coming: the
/// `flock` singleton elects ONE winner among racing spawns and every loser exits
/// `AddrInUse` — but a loser only reaches the `flock` AFTER its own transport init, by
/// which point the winner has already bound (it binds microseconds after taking the
/// lock). So a real winner's socket is there within a poll or two; 500 ms is ~25× that
/// margin, and a child that died for a REAL reason (missing library, bad config,
/// exec failure) surfaces its loud error in ~half a second instead of ten.
const SPAWN_POST_EXIT_GRACE: Duration = Duration::from_millis(500);

/// The per-request round-trip timeout — a demand/release must be answered within
/// this, else the consumer gets a loud timeout error instead of hanging on a
/// wedged daemon. Generous: a demand's first-ensure opens netd's lazy zenoh session
/// under the registry lock, which can take a beat on a cold daemon.
///
/// **This is the CEILING on netd's cold-start grace.** A query verb is
/// answered only after the plane has spent
/// [`COLD_START_DISCOVERY_BUDGET`](crate::query::COLD_START_DISCOVERY_BUDGET), so a
/// grace that approached this timeout would make the client give up BEFORE the
/// daemon answered — turning the explicit "no robots discovered" verdict into an
/// opaque IO timeout. The relationship is guarded by
/// `query::tests::the_shipped_cold_start_budget_fits_inside_the_client_round_trip_timeout`,
/// which is why this is `pub(crate)` rather than private.
pub(crate) const ROUNDTRIP_TIMEOUT: Duration = Duration::from_secs(5);

/// The longest the first-contact wait sleeps without re-checking the
/// caller's cancellation flag.
///
/// `std::thread::sleep` resumes across `EINTR`, and `cerulion_cli` REPLACES the
/// default SIGINT/SIGTERM/SIGHUP disposition with a flag-flip handler before
/// `topic echo`/`topic hz` run — so during an un-sliced sleep Ctrl-C, repeated Ctrl-C
/// and `kill` are all no-ops and only SIGKILL ends the command.
///
/// # The bound this actually buys
///
/// Slicing bounds the deafness of the SLEEP to one slice. It does NOT bound the whole
/// loop: cancellation is observed at round-trip BOUNDARIES and inside the sleep, and
/// a round trip in flight is uninterruptible for up to [`ROUNDTRIP_TIMEOUT`] (5 s,
/// and ~3.75 s on the cold-daemon shape this feature targets — `BufRead::read_line`
/// retries `ErrorKind::Interrupted` internally, so a signal does not break the read).
/// The real worst case is therefore **one round trip plus one slice**, not one
/// slice, which is why the
/// statement is written out rather than implied.
///
/// 100 ms is below the ~150 ms an interactive user reads as instant, and the loop's
/// real cadence is the poll interval, so slicing costs nothing but a few extra
/// `Instant` reads per poll.
const CANCEL_CHECK_SLICE: Duration = Duration::from_millis(100);

/// Everything the first-contact wait needs from its caller — the policy, a
/// progress sink, and a cancellation flag.
///
/// Bundled rather than passed as three parameters because both query verbs take it
/// and it is threaded through four CLI seams; a struct keeps the addition of a fourth
/// concern from re-touching every call site.
///
/// `running` follows the CLI's own idiom: `Some(flag)` means "cancel when this goes
/// FALSE" (exactly what `setup_ctrlc_handler` writes), and `None` means this caller
/// has no cancellation source (`topic info`, `schema info` — neither installs a
/// handler, so both still die on the default SIGINT disposition).
pub struct FirstContactWait<'a> {
    policy: ConvergenceWait,
    progress: &'a mut dyn FnMut(Duration),
    running: Option<&'a std::sync::atomic::AtomicBool>,
}

impl<'a> FirstContactWait<'a> {
    /// Build a wait context. `progress` is invoked with the elapsed wait once before
    /// the first round trip and once per subsequent poll — and NEVER at all when
    /// `policy` is [`ConvergenceWait::off`], so a no-wait caller prints nothing.
    pub fn new(
        policy: ConvergenceWait,
        progress: &'a mut dyn FnMut(Duration),
        running: Option<&'a std::sync::atomic::AtomicBool>,
    ) -> Self {
        Self {
            policy,
            progress,
            running,
        }
    }

    /// The policy this wait runs under.
    pub fn policy(&self) -> ConvergenceWait {
        self.policy
    }

    fn note_progress(&mut self, elapsed: Duration) {
        (self.progress)(elapsed);
    }

    /// Has the caller asked us to stop? `None` (no flag) is never cancelled.
    fn is_cancelled(&self) -> bool {
        self.running
            .is_some_and(|r| !r.load(std::sync::atomic::Ordering::Relaxed))
    }

    /// Sleep `total`, re-checking cancellation every [`CANCEL_CHECK_SLICE`].
    /// Returns `false` if the wait was cancelled part-way through.
    fn sleep_cancellable(&self, total: Duration) -> bool {
        let deadline = Instant::now() + total;
        loop {
            if self.is_cancelled() {
                return false;
            }
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return true;
            }
            std::thread::sleep(remaining.min(CANCEL_CHECK_SLICE));
        }
    }
}

/// The two decision inputs one round trip yields, plus the answering
/// daemon's plane age. Private glue so [`NetdClient::query_converged`] can stay
/// generic over the two query verbs without a three-tuple at every call site.
struct RoundTripVerdict {
    discovery: DiscoveryState,
    answer_empty: bool,
    plane_unsettled_for: Option<Duration>,
}

/// A first-contact wait ABANDONED by a transport error, carrying how long it
/// had already run.
///
/// The elapsed matters because the user has been watching a progress counter: without
/// it the consumer's fallback prints a differently-shaped warn and the counter is
/// simply never closed, which reads as if the command gave up instantly. The consumer
/// closes the lines it opened (`progress_lines > 0`) and then degrades as before.
#[derive(Debug)]
pub struct ConvergenceAbort {
    /// The transport error that ended the wait.
    pub error: ClientError,
    /// How long the wait had run when it was abandoned.
    pub waited: Duration,
    /// How many progress lines the caller's sink had already emitted.
    pub progress_lines: u32,
}

/// The minimum daemon [`PROTOCOL_VERSION`] this CONSUMER needs at the
/// handshake — the v1 demand/release/status vocabulary [`NetdClient`] is built on.
/// A daemon at or above this serves the baseline (a newer one is a superset); only a
/// daemon BELOW it is refused at connect. The v2 egress verbs are gated per-verb at
/// the send site ([`verb_compat_error`]), not here — a v2 client still connects to a
/// v1 daemon and uses demand/release, refusing only an egress verb the v1 daemon
/// cannot serve.
const CLIENT_MIN_DAEMON_VERSION: u32 = 1;

/// The first daemon [`PROTOCOL_VERSION`] that REPORTS a
/// [`DiscoveryState`] on its query responses.
///
/// Below this the field is absent and serde defaults it to
/// [`DiscoveryState::Settled`] — a positive assertion
/// ("discovery ran, an empty answer is real absence") that the old daemon never made
/// and cannot make. See [`trust_reported_discovery`].
///
/// This is a FLOOR frozen at the version that INTRODUCED the field — a historical
/// fact. A later, unrelated [`PROTOCOL_VERSION`] bump must NOT raise it: doing so
/// would distrust every genuine v5 daemon's accurate report. Pinned by
/// `a_pre_916_daemons_defaulted_settled_is_never_believed`.
pub const DISCOVERY_MIN_DAEMON_VERSION: u32 = 5;

/// The trust downgrade must never become a REFUSAL: an older daemon still
/// serves `query_catalog`/`query_schema` perfectly well, it just cannot vouch for its
/// own discovery. Compile-time, so raising the connect floor to the version
/// (which would lock every consumer out of a working older daemon) cannot compile.
const _: () = assert!(CLIENT_MIN_DAEMON_VERSION < DISCOVERY_MIN_DAEMON_VERSION);

/// The trust boundary is a FLOOR frozen at the version that INTRODUCED
/// the discovery field — it must never EXCEED the current [`PROTOCOL_VERSION`], or
/// every fresh daemon's own accurate report stops being believed. Compile-time, and
/// deliberately `<=` rather than `==`: an `assert_eq!` would MANDATE a
/// regression, because the next unrelated protocol bump (→ 6) would fail it and
/// push the reader to raise this constant to match — which would distrust
/// every genuine v5 daemon. A later bump raises [`PROTOCOL_VERSION`] ONLY.
const _: () = assert!(DISCOVERY_MIN_DAEMON_VERSION <= PROTOCOL_VERSION);

/// The `query_runs` verb needs a daemon at least as new as the one that
/// introduced [`DiscoveryState`], which is what lets its response carry a MANDATORY
/// `discovery` field with no trust downgrade — any daemon old enough to need one is
/// already too old to serve the verb, so its per-verb gate refuses BEFORE the trust
/// question can arise.
///
/// Compile-time, because that reasoning is the ONLY thing standing between
/// `RunsQueryResponse`'s non-defaulted `discovery` and a silent false absence claim.
/// Lowering the runs minimum below 5 would make the verb reachable on a daemon whose
/// state cannot be believed — and nothing in the wire shape would say so.
const _: () = assert!(RUNS_MIN_DAEMON_VERSION >= DISCOVERY_MIN_DAEMON_VERSION);

/// What a query answer's discovery state MEANS given the daemon that sent
/// it. PURE — oracle-tested.
///
/// A daemon at [`DISCOVERY_MIN_DAEMON_VERSION`] or above genuinely reports its state, so
/// it is used verbatim. An OLDER daemon sent no field at all and serde defaulted it
/// to `Settled`, so believing it would silently produce a false "not
/// found". The correct reading of "this daemon cannot tell me whether discovery
/// converged" is [`DiscoveryState::NotConverged`]:
/// **not** a claim that nothing is out there, but a refusal to let the consumer claim
/// absence. The cost is that a GENUINE miss against a stale daemon reads "unknown"
/// instead of "not found" — correct, since that cannot be determined — and the caller's warn
/// names the one-step remedy (restart the daemon).
///
/// netd is spawn-once and long-lived, so this skew is real: upgrading the CLI while a
/// running vizd holds an old netd alive hits it on every query until that daemon exits.
pub fn trust_reported_discovery(daemon_protocol: u32, reported: DiscoveryState) -> DiscoveryState {
    if daemon_protocol >= DISCOVERY_MIN_DAEMON_VERSION {
        reported
    } else {
        DiscoveryState::NotConverged
    }
}

/// Warn ONCE PER PROCESS that the daemon is too old to report a discovery
/// state. Once, not per query — a `topic hz` polls, and the remedy does not change.
fn warn_stale_daemon_once(daemon_protocol: u32) {
    static WARNED: std::sync::Once = std::sync::Once::new();
    WARNED.call_once(|| {
        tracing::warn!(
            daemon_protocol,
            needs = DISCOVERY_MIN_DAEMON_VERSION,
            "the running cerulion-netd is older than this CLI and cannot report whether its \
             network discovery has converged — remote lookups will say 'unknown' instead of \
             'not found' rather than risk a false 'topic does not exist'. Restart the daemon \
             to restore precise answers (it is spawned again automatically on the next demand)."
        );
    });
}

/// The per-verb version gate. Returns `Some(message)` when a daemon at
/// `daemon_version` is TOO OLD to serve `req` (naming the daemon version, the verb,
/// and the upgrade fix), else `None`. The client refuses the verb LOCALLY rather than
/// sending a request the daemon cannot parse. A daemon at or above the verb's
/// minimum ([`Request::min_daemon_version`]) serves it — a NEWER daemon always does.
/// Pure — oracle-tested.
fn verb_compat_error(daemon_version: u32, req: &Request) -> Option<String> {
    let need = req.min_daemon_version();
    if daemon_version < need && matches!(req, Request::AccountAccess { .. }) {
        return Some(format!("the running cerulion-netd speaks protocol v{daemon_version}; account robot access requires v{need}. Upgrade and restart cerulion-netd."));
    }
    if daemon_version >= need {
        None
    } else {
        Some(format!(
            "the connected cerulion-netd speaks protocol v{daemon_version} but the '{}' verb needs \
             protocol v{need} (this client speaks v{PROTOCOL_VERSION}). Upgrade cerulion-netd to \
             use this feature.",
            req.method_name()
        ))
    }
}

/// Which connect path produced a [`ClientError::Connect`].
///
/// DECLARED by the site that builds the error, never inferred from the `source`
/// — the same rule `ListenerCountTiming` follows in `cerulion_core`, and for the
/// same reason: an `io::Error` cannot say whether the caller was *willing* to
/// spawn, so any inference would be a guess baked into an operator-facing
/// sentence.
///
/// It exists because the message asserted a spawn UNCONDITIONALLY ("spawned it
/// but it did not come up in time") while two of its three construction sites
/// never spawn anything — and the one that reads it most is the
/// recorder, whose whole contract is that it does NOT start a daemon on the
/// machine it is recording. A remedy that misdescribes what just happened sends
/// the operator to the wrong end of their own problem.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectAttempt {
    /// The connect-or-spawn path: the client was willing to spawn a daemon.
    ///
    /// Covers BOTH of its exits — the fast-path arm that surfaces a real
    /// connect error without spawning, and the post-spawn readiness wait (whose
    /// own `source` names the spawn and the ceiling).
    ConnectOrSpawn,
    /// [`NetdClient::connect_existing_at`]: the client did not reach a daemon,
    /// and this path NEVER spawns one.
    ///
    /// It labels the PATH, not a diagnosis. The `source` is whatever
    /// `UnixStream::connect` returned and the variant is built without
    /// inspecting its kind, so it does NOT establish that no daemon was running
    /// — `PermissionDenied` is a state in which one IS, and is merely out of
    /// reach. Read the `source` for the condition.
    ExistingOnly,
}

/// A failure talking to `cerulion-netd`. Every arm is LOUD (Principle #12) — the
/// consumer surfaces it verbatim; the demand plane NEVER silently falls back to a
/// private per-consumer mirror.
#[derive(Debug)]
pub enum ClientError {
    /// Could not launch the `cerulion-netd` binary at all (not found / not
    /// executable). The consumer cannot reach the shared daemon.
    Spawn {
        /// The binary path the client tried to launch.
        bin: PathBuf,
        /// The OS spawn error.
        source: io::Error,
    },
    /// Could not connect to the daemon socket.
    ///
    /// Whether a spawn was even possible is carried by
    /// [`attempt`](ClientError::Connect::attempt) — see [`ConnectAttempt`].
    Connect {
        /// The control-socket path.
        socket: PathBuf,
        /// WHICH path failed, DECLARED by the site that built this
        /// error. See [`ConnectAttempt`].
        attempt: ConnectAttempt,
        /// The last connect error.
        source: io::Error,
    },
    /// The daemon answered, but with a [`crate::protocol::Response::Error`] (e.g. a
    /// mirror-registration failure, a schema conflict, or an empty robot/topic).
    Netd {
        /// The daemon's error message.
        error: String,
        /// The offending robot, when the error is topic-scoped.
        robot: Option<String>,
        /// The offending topic, when the error is topic-scoped.
        topic: Option<String>,
    },
    /// A protocol violation: a bad/mismatched [`Hello`] banner, an unexpected
    /// response shape, or an unparseable line.
    Protocol(String),
    /// A wire I/O error (broken pipe, timeout) exchanging a request/response.
    Io(io::Error),
}

impl std::fmt::Display for ClientError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ClientError::Spawn { bin, source } => write!(
                f,
                "could not launch the cerulion-netd daemon `{}`: {source} — is it installed \
                 alongside this binary, or set {NETD_BIN_ENV} to its path?",
                bin.display()
            ),
            ClientError::Connect {
                socket,
                attempt,
                source,
            } => match attempt {
                // The spawn/no-spawn detail that IS true here already rides the
                // `source` built at the readiness-wait site ("waited N.Ns for
                // the control socket after spawning cerulion-netd — …"), so the
                // wrapper states only what holds on every ConnectOrSpawn exit.
                // Asserting "spawned it but it did not come up in time"
                // UNCONDITIONALLY would be false on BOTH no-spawn sites: the
                // `connect_existing*` path below, and this path's own
                // fast-path arm, which surfaces a real connect error
                // (permissions, …) after deliberately NOT spawning.
                ConnectAttempt::ConnectOrSpawn => write!(
                    f,
                    "could not reach the cerulion-netd daemon at {}: {source}",
                    socket.display()
                ),
                // States what this path KNOWS — it did not reach a daemon — and
                // no more. It does not assert a CAUSE such as "no cerulion-netd daemon
                // was running at …", because `try_connect` can return any
                // `io::ErrorKind` and this arm never inspects the kind.
                //
                // `PermissionDenied` is the kind that makes that claim the
                // OPPOSITE of the truth: a daemon can be listening while EACCES
                // on the socket (or on a directory in its path) keeps us out.
                // MEASURED against a live `UnixListener` under a `0o000` parent
                // directory — the daemon is up and `connect` gives EACCES, so a
                // "none was running" line would be false. `Interrupted` (a signal landed
                // mid-connect) and the path-shape errors establish nothing about
                // a daemon either: `NotADirectory` for an ENOTDIR path prefix,
                // and `InvalidInput` for a path over `SUN_LEN`, which std
                // refuses BEFORE the syscall (also measured, while writing the
                // EACCES probe above).
                //
                // `NotFound` and `ConnectionRefused` really do mean nothing is
                // bound and listening — that is the classification
                // `is_not_running` makes, and the ONLY reading this error ever
                // establishes — but the wrapper does not branch on kind, so it
                // must not print what only one branch would support. The
                // `source` names the real condition in every case.
                ConnectAttempt::ExistingOnly => write!(
                    f,
                    "could not reach a running cerulion-netd daemon at {} and this path never \
                     spawns one: {source}",
                    socket.display()
                ),
            },
            ClientError::Netd {
                error,
                robot,
                topic,
            } => match (robot, topic) {
                (Some(r), Some(t)) => {
                    write!(f, "cerulion-netd refused robot='{r}' topic='{t}': {error}")
                }
                _ => write!(f, "cerulion-netd error: {error}"),
            },
            ClientError::Protocol(msg) => write!(f, "cerulion-netd protocol error: {msg}"),
            ClientError::Io(e) => write!(f, "cerulion-netd I/O error: {e}"),
        }
    }
}

impl std::error::Error for ClientError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            ClientError::Spawn { source, .. } | ClientError::Connect { source, .. } => Some(source),
            ClientError::Io(e) => Some(e),
            _ => None,
        }
    }
}

/// A live connection to the one-per-computer `cerulion-netd` daemon.
///
/// Keep it alive for as long as the demanded topic(s) are needed; dropping it
/// closes the UDS connection, which RELEASES every demand it held (the crash-safe
/// refcount — see the module docs). One client = one netd connection = one refcount
/// slot per demanded `(robot, topic)`.
#[derive(Debug)]
pub struct NetdClient {
    stream: UnixStream,
    reader: BufReader<UnixStream>,
    next_id: u64,
    socket_path: PathBuf,
    /// The [`PROTOCOL_VERSION`] the connected daemon announced in its
    /// [`Hello`] banner. Recorded (not equality-refused) so the client can serve the
    /// verbs a DIFFERENT-version daemon supports — accepting a NEWER daemon always
    /// (its vocabulary is a superset) and an OLDER daemon for the verbs it still
    /// speaks (a v2 client keeps working against a pinned v1 daemon for demand /
    /// release / status), refusing a specific verb only when the daemon is older than
    /// THAT verb requires ([`Request::min_daemon_version`]).
    daemon_protocol: u32,
    /// A partial or invalid bounded exchange may leave a reply in flight.
    poisoned: bool,
}

impl NetdClient {
    /// Connect to the well-known `cerulion-netd` control socket
    /// ([`default_socket_path`]), SPAWNING the daemon detached if it is not running
    /// (the first-consumer-spawns lifecycle). See [`Self::connect_or_spawn_at`].
    pub fn connect_or_spawn() -> Result<Self, ClientError> {
        Self::connect_or_spawn_at(default_socket_path())
    }

    /// Connect to `socket`, spawning `cerulion-netd` detached (pointed at THIS
    /// socket) if nothing is listening, then retrying the connect under the bounded
    /// budget. Validates the [`Hello`] banner (marker + [`PROTOCOL_VERSION`]).
    ///
    /// The explicit-socket entry (tests / an operator running a non-default
    /// instance). The spawned daemon inherits the parent env AND is handed
    /// `CERULION_NETD_SOCK={socket}` so it binds the SAME path the client connects
    /// to.
    ///
    /// A SINGLE silent retry absorbs the daemon idle-exit / shutdown RACE.
    /// If the daemon accepted us then closed at/before the `Hello` (it committed to
    /// self-exit as we connected, or unlinked the socket underfoot — the wedge
    /// signature), the whole connect-or-spawn is retried ONCE. The retry
    /// RE-CLASSIFIES for free: its `try_connect` sees a now-gone socket and spawns a
    /// FRESH daemon, or connects to a just-started one. A second early close is
    /// surfaced LOUDLY (never a silent hang). The retry is invisible on the healthy
    /// (non-race) path — no early close, no retry.
    pub fn connect_or_spawn_at(socket: PathBuf) -> Result<Self, ClientError> {
        match Self::connect_or_spawn_once(&socket) {
            Err(e) if is_closed_early(&e) => Self::connect_or_spawn_once(&socket),
            result => result,
        }
    }

    /// Connect to a netd that is ALREADY RUNNING, at the well-known
    /// socket. Never spawns. See [`Self::connect_existing_at`].
    pub fn connect_existing() -> Result<Self, ClientError> {
        Self::connect_existing_at(default_socket_path())
    }

    /// Connect to `socket` if a daemon is listening there, and fail
    /// LOUDLY if it cannot. **Never spawns, never waits.**
    ///
    /// The failure is `ClientError::Connect` carrying
    /// [`ConnectAttempt::ExistingOnly`] and the `UnixStream::connect` error
    /// VERBATIM. It reports that no daemon was REACHED — not that none was
    /// running, which this path does not establish (see [`ConnectAttempt`]).
    ///
    /// # Why this exists beside [`Self::connect_or_spawn_at`]
    ///
    /// Every other consumer of `NetdClient` is an INTERACTIVE verb
    /// (`topic echo`, `schema info`, `cerulion viz`) or a desk daemon, for which
    /// "start the network daemon on first use" is the chosen lifecycle and a
    /// one-off multi-second boot is an acceptable price.
    ///
    /// A RECORDER is neither. `cerulion bagd` is an OBSERVER of a machine, and
    /// two properties of the spawning path are disqualifying for it:
    ///
    /// * It would **start a network daemon on a robot that deliberately has
    ///   none.** A recorder must not change what is running on the machine it is
    ///   recording.
    /// * It **blocks**. The spawn path waits up to `SPAWN_READY_TIMEOUT` for
    ///   readiness and [`Self::connect_or_spawn_at`] retries the whole flow once
    ///   on the idle-exit race, so the worst case is ~2x that — spent
    ///   inside the recorder's arm-time window, against a discovery
    ///   settle window measured in hundreds of milliseconds.
    ///
    /// So the recorder asks whether a daemon is THERE and takes "no" for an
    /// answer: with no netd running, its schema-demand rung is simply
    /// unavailable and the ladder falls through to a hash-only channel, loudly
    /// labelled. That is the correct outcome, and it is bounded.
    ///
    /// The `Hello` banner is validated exactly as on the spawning path (this is
    /// the same `finish_handshake`), so a connected client is
    /// indistinguishable from one the other constructor returned.
    pub fn connect_existing_at(socket: PathBuf) -> Result<Self, ClientError> {
        match try_connect(&socket) {
            Ok(stream) => Self::finish_handshake(stream, socket),
            Err(source) => Err(ClientError::Connect {
                socket,
                attempt: ConnectAttempt::ExistingOnly,
                source,
            }),
        }
    }

    /// ONE attempt of the connect-or-spawn flow (see [`Self::connect_or_spawn_at`],
    /// which wraps this with the single retry): fast-path connect to a
    /// running daemon, else spawn detached and retry the connect under the bounded
    /// budget, then validate the [`Hello`] banner.
    fn connect_or_spawn_once(socket: &Path) -> Result<Self, ClientError> {
        // Fast path: a daemon is already running — just connect.
        match try_connect(socket) {
            Ok(stream) => return Self::finish_handshake(stream, socket.to_path_buf()),
            Err(e) if !is_not_running(&e) => {
                // A real connect error (permissions, …) — NOT "not running". Do not
                // spawn; surface it.
                return Err(ClientError::Connect {
                    socket: socket.to_path_buf(),
                    attempt: ConnectAttempt::ConnectOrSpawn,
                    source: e,
                });
            }
            Err(_) => { /* not running — spawn below */ }
        }

        // Not running: spawn netd detached (best-effort; the flock singleton elects
        // ONE winner among racing consumers — the rest exit AddrInUse and our
        // retry-connect finds the winner).
        let mut child = spawn_netd_detached(socket)?;

        // TWO notices, each at the level its case earns: this first-use line at
        // `info!`, and ONE stall `warn!` inside the wait loop below. `topic echo` /
        // `viz` / `schema info` are INTERACTIVE verbs, and a first-use spawn can
        // legitimately sit here for seconds (a cold daemon boot measured ~1.1 s on a
        // desk; the ceiling is 10 s) with nothing on the terminal, indistinguishable
        // from a hang. No spinner, no progress: the healthy path emits the info line
        // and is connected about a second later; only a wait that outlives
        // `SPAWN_STALL_NOTICE_AFTER` draws the warn. Neither is a raw `eprintln!`.
        //
        // WHY neither notice is an `eprintln!`, and why the info line alone
        // is not enough:
        //
        // An unconditional `eprintln!` cannot be silenced by a caller with no terminal
        // (vizd's daemon loop, `graph run`), and gating it behind
        // `stderr().is_terminal()` on the theory that
        // "the structured twin below is unconditional and silenceable, so nothing is
        // LOST for a logging caller" is FALSE for the DOMINANT callers:
        // `topic echo` / `info` / `hz` and `schema info` are `OneShot` verbs, and
        // `is_quiet_default()` maps `OneShot` to a `warn` default filter — so the `info!`
        // is dropped by default, and with stderr piped a gated `eprintln!` does not fire.
        // A piped `cerulion topic echo /x > out.txt` would then emit NOTHING for up to
        // `SPAWN_READY_TIMEOUT` — precisely the "indistinguishable from a hang" scenario
        // this line exists to prevent.
        //
        // INFO, not warn: spawning the daemon on first use is the NORMAL path on a
        // fresh desk (every first `graph run`, `topic echo` or `viz` takes it), and a
        // warning for the expected case teaches a first user to ignore warnings. The
        // long-running verbs show it at their `cerulion=info` default. The quiet
        // one-shot verbs (`cerulion=warn`) hide it, as they hide every other
        // lifecycle line, so THEIR notice is the stall `warn!` inside the wait loop
        // below: silent through a healthy cold boot, one line once the wait has run
        // past `SPAWN_STALL_NOTICE_AFTER`, so no verb stares at nothing for the
        // whole ceiling. Going through `tracing` keeps both silenceable
        // (`RUST_LOG=error`), honors the repo's "never println! in library code"
        // rule, and removes the terminal-vs-pipe divergence entirely.
        //
        // SCOPE: a caller with NO tracing subscriber sees nothing, true of every
        // log line in this crate. The FAILURE path does not depend on it: the returned
        // `ClientError` is self-contained (see `unknown_cause` below), so a wedge is
        // still attributable without a subscriber.
        tracing::info!(
            ceiling_s = SPAWN_READY_TIMEOUT.as_secs(),
            "cerulion-netd is not running, starting it (first use) and waiting for readiness"
        );

        // Wait for READINESS (the socket becomes connectable), bounded by
        // SPAWN_READY_TIMEOUT — not for a fixed number of attempts. While waiting, watch
        // the spawned child: once it has EXITED, only SPAWN_POST_EXIT_GRACE remains, so a
        // daemon that died on startup is reported in ~half a second rather than at the
        // ceiling (and a `flock` loser still leaves ample room for the winner's socket).
        let started = Instant::now();
        let mut child_state = ChildLiveness::Running;
        // The `try_wait` failure that produced `ChildLiveness::Unknown`, carried so the
        // expiry message is SELF-CONTAINED. A pointer such as "see the earlier warning"
        // would dangle: that warning is a `tracing::warn!`, dropped under `RUST_LOG=error` or with
        // no subscriber, while the returned `ClientError` string is ALWAYS visible.
        // That is the same "no subscriber ⇒ no log
        // line" gap the spawn breadcrumb above documents as its scope: the
        // difference is that a breadcrumb may be missed, whereas a FAILURE must never be
        // — hence the cause is inlined here rather than referenced.
        // `ChildLiveness` stays `Copy`; the cause rides alongside it.
        let mut unknown_cause: Option<String> = None;
        // The ONE stall notice for the quiet verbs (see
        // `SPAWN_STALL_NOTICE_AFTER`): `warn!` is the level `is_quiet_default`
        // lets through, and a boot still pending this far past a real cold boot
        // is a stall, not the normal case, so a warning is the right level. Once
        // per wait; the ceiling's own expiry is reported by the `Err` below.
        let mut stall_notified = false;
        let last_err = loop {
            std::thread::sleep(SPAWN_POLL_INTERVAL);
            let err = match try_connect(socket) {
                Ok(stream) => return Self::finish_handshake(stream, socket.to_path_buf()),
                Err(e) => e,
            };
            if stall_notice_due(started.elapsed(), SPAWN_STALL_NOTICE_AFTER, stall_notified) {
                stall_notified = true;
                tracing::warn!(
                    waited_s = format_args!("{:.1}", started.elapsed().as_secs_f64()),
                    notice_after_s = SPAWN_STALL_NOTICE_AFTER.as_secs(),
                    ceiling_s = SPAWN_READY_TIMEOUT.as_secs(),
                    "cerulion-netd is still starting (its first-use boot has outlived the \
                     stall-notice bound); waiting up to the readiness ceiling before \
                     giving up"
                );
            }
            // Observe the child's exit ONCE (`try_wait` also reaps it, so a `flock`
            // loser does not linger as a zombie for this consumer's lifetime).
            //
            // The Err arm is NOT swallowed. `try_wait` can fail for reasons that say
            // nothing about the daemon — most notably ECHILD when the process has
            // SIG_IGN'd SIGCHLD and the child was auto-reaped (a class this codebase
            // documents; a CLI embedded in such a host hits it). Treating that as
            // "exited" would fail-fast a healthy daemon at ~500ms; treating it as
            // "running" would assert liveness that was never confirmed. So: log it ONCE, and go to
            // UNKNOWN — the wait continues to the full ceiling (never cut short on a
            // liveness answer we could not get) and the expiry message says so instead
            // of claiming the daemon is still running.
            if matches!(child_state, ChildLiveness::Running) {
                match child.try_wait() {
                    Ok(Some(status)) => {
                        child_state = ChildLiveness::Exited(status, Instant::now());
                    }
                    Ok(None) => { /* still running — keep polling */ }
                    Err(e) => {
                        tracing::warn!(
                            error = %e,
                            "cerulion-netd: cannot observe the spawned daemon's liveness \
                             (try_wait failed — e.g. ECHILD under a SIG_IGN'd SIGCHLD); the \
                             readiness wait continues to the full ceiling and its liveness \
                             verdict will be reported as UNKNOWN"
                        );
                        unknown_cause = Some(e.to_string());
                        child_state = ChildLiveness::Unknown;
                    }
                }
            }
            if spawn_wait_exhausted(
                started.elapsed(),
                child_state.exited_for(),
                SPAWN_READY_TIMEOUT,
                SPAWN_POST_EXIT_GRACE,
            ) {
                break err;
            }
        };

        // LOUD on expiry: name what we waited for, how long, and — the single most
        // useful discriminator — whether the daemon we spawned is still alive. Each limb
        // also names the bound that ACTUALLY expired, since the exited limb is cut short
        // by the post-exit grace and never reaches the ceiling.
        let waited = started.elapsed();
        let liveness = match child_state {
            ChildLiveness::Exited(status, _) => format!(
                "the spawned daemon EXITED ({status}) before the socket appeared, so the wait was \
                 cut to the {}ms post-exit grace rather than the full ceiling — check its stderr \
                 for the startup error (a daemon that merely lost the singleton flock race exits \
                 harmlessly, but then the winner should have bound this socket)",
                SPAWN_POST_EXIT_GRACE.as_millis()
            ),
            ChildLiveness::Running => format!(
                "the spawned daemon is STILL RUNNING but never bound the socket within the {}s \
                 ceiling (still booting, or wedged during startup)",
                SPAWN_READY_TIMEOUT.as_secs()
            ),
            ChildLiveness::Unknown => format!(
                "the spawned daemon's liveness is UNKNOWN — we could not observe it (try_wait \
                 failed: {}), so the full {}s ceiling was used. Check whether a cerulion-netd \
                 process is running and inspect its stderr",
                unknown_cause.as_deref().unwrap_or("cause unrecorded"),
                SPAWN_READY_TIMEOUT.as_secs()
            ),
        };
        Err(ClientError::Connect {
            socket: socket.to_path_buf(),
            attempt: ConnectAttempt::ConnectOrSpawn,
            source: io::Error::new(
                io::ErrorKind::TimedOut,
                format!(
                    "waited {:.1}s for the control socket after spawning cerulion-netd (polled \
                     every {}ms, ceiling {}s) — {liveness}; last connect error: {last_err}",
                    waited.as_secs_f64(),
                    SPAWN_POLL_INTERVAL.as_millis(),
                    SPAWN_READY_TIMEOUT.as_secs(),
                ),
            ),
        })
    }

    /// Read + validate the [`Hello`] banner and build the client.
    fn finish_handshake(stream: UnixStream, socket: PathBuf) -> Result<Self, ClientError> {
        stream
            .set_read_timeout(Some(ROUNDTRIP_TIMEOUT))
            .map_err(ClientError::Io)?;
        stream
            .set_write_timeout(Some(ROUNDTRIP_TIMEOUT))
            .map_err(ClientError::Io)?;
        let reader = BufReader::new(stream.try_clone().map_err(ClientError::Io)?);
        let mut client = NetdClient {
            stream,
            reader,
            next_id: 1,
            socket_path: socket,
            daemon_protocol: 0,
            poisoned: false,
        };
        let line = client.read_line()?;
        client.daemon_protocol = validate_hello(&line)?;
        Ok(client)
    }

    /// Demand the shared desk mirror for remote `(robot, topic)`, validating every
    /// inbound frame against `schema_hash`. On success netd holds ONE mirror for
    /// this topic (creating it on the FIRST demander, joining an existing one
    /// otherwise) and this connection now holds a refcount on it until the client
    /// drops or [`Self::release`]s it. The consumer then reads the mirror at
    /// `topic` via a normal local SHM subscriber (netd re-injects the frames into
    /// the desk's default iceoryx2 namespace).
    pub fn demand(
        &mut self,
        robot: &str,
        topic: &str,
        schema_hash: u64,
    ) -> Result<DemandResponse, ClientError> {
        // The FIRST request on this connection can race the daemon's
        // idle-exit / shutdown — the daemon served the Hello then closed before
        // serving our demand (the accepted-then-EOF wedge signature). Retry the
        // demand ONCE, silently: RECONNECT (re-classifying — a now-gone socket
        // respawns a fresh daemon; a live one reconnects) and re-send. A later
        // demand failing is a genuine mid-session error, not the race, so only the
        // first request (no request sent yet: `next_id == 1`) is retried; a second
        // early close is surfaced LOUDLY.
        let first_request = self.next_id == 1;
        match self.demand_once(robot, topic, schema_hash) {
            Err(e) if first_request && is_closed_early(&e) => {
                self.reconnect()?;
                self.demand_once(robot, topic, schema_hash)
            }
            other => other,
        }
    }

    /// One bounded account operation. Never respawns or retries implicitly.
    /// The running daemon must support protocol v8; upgrade and restart it otherwise.
    pub fn account_access_once(
        &mut self,
        action: crate::account_access::AccountAccessRequest,
    ) -> Result<crate::account_access::AccountAccessReply, ClientError> {
        self.account_access_until(action, Instant::now() + ROUNDTRIP_TIMEOUT)
    }

    /// One `demand` round-trip (see [`Self::demand`], which wraps this with the
    /// first-request retry).
    fn demand_once(
        &mut self,
        robot: &str,
        topic: &str,
        schema_hash: u64,
    ) -> Result<DemandResponse, ClientError> {
        let id = self.next_request_id();
        let req = Request::Demand {
            id,
            robot: robot.to_string(),
            topic: topic.to_string(),
            schema_hash,
        };
        match self.round_trip(&req)? {
            Response::Demand(d) => Ok(d),
            Response::Error(e) => Err(ClientError::Netd {
                error: e.error,
                robot: e.robot,
                topic: e.topic,
            }),
            other => Err(ClientError::Protocol(format!(
                "expected a demand response, got {other:?}"
            ))),
        }
    }

    /// Rebuild this client's connection in place (the demand-retry path) —
    /// re-run [`Self::connect_or_spawn_at`] against the same socket (which itself
    /// re-classifies: a gone socket respawns a fresh daemon, a live one reconnects)
    /// and adopt its fresh stream/reader + reset `next_id`. The socket path is
    /// unchanged.
    fn reconnect(&mut self) -> Result<(), ClientError> {
        self.ensure_usable()?;
        let fresh = Self::connect_or_spawn_at(self.socket_path.clone())?;
        self.stream = fresh.stream;
        self.reader = fresh.reader;
        self.next_id = fresh.next_id;
        // Adopt the fresh daemon's protocol version
        // too — a reconnect can land on a DIFFERENT daemon (a v2 daemon idle-exited
        // and a v1 daemon respawned, or vice versa), so the per-verb compat gate must
        // re-gate against whoever we are now talking to, not the pre-reconnect version.
        self.daemon_protocol = fresh.daemon_protocol;
        Ok(())
    }

    /// Release a previously-demanded `(robot, topic)` for this connection (vizd's
    /// `detach` — the EARLY release for a client that outlives one topic's need).
    /// Dropping the whole client releases every demand it holds; this releases just
    /// one while keeping the connection open for other topics.
    pub fn release(&mut self, robot: &str, topic: &str) -> Result<ReleaseResponse, ClientError> {
        let id = self.next_request_id();
        let req = Request::Release {
            id,
            robot: robot.to_string(),
            topic: topic.to_string(),
        };
        match self.round_trip(&req)? {
            Response::Release(r) => Ok(r),
            Response::Error(e) => Err(ClientError::Netd {
                error: e.error,
                robot: e.robot,
                topic: e.topic,
            }),
            other => Err(ClientError::Protocol(format!(
                "expected a release response, got {other:?}"
            ))),
        }
    }

    /// Query the LAN topic CATALOG over netd's ONE zenoh session — the
    /// consumer's replacement for opening its own transient discovery session.
    /// `robot: Some` scopes it to ONE robot's catalog (vizd's resolve); `None`
    /// harvests every announcing robot. Returns every decoded [`CatalogReply`]. An
    /// `Ok(vec![])` is AUTHORITATIVE — netd reached the LAN and no robot answered
    /// (the consumer does NOT then open its own session); only a [`ClientError`]
    /// (netd unreachable / a transport failure) is the fallback trigger. A refusal
    /// reply (`CatalogReply.error`) rides through the vec verbatim so the consumer
    /// surfaces it loudly. STATELESS: this creates no demand and holds nothing.
    pub fn query_catalog(&mut self, robot: Option<&str>) -> Result<Vec<CatalogReply>, ClientError> {
        self.query_catalog_with_discovery(robot)
            .map(|gather| gather.catalogs)
    }

    /// [`Self::query_catalog`] plus the [`DiscoveryState`] netd answered
    /// under — the discriminator between "the LAN was searched and nobody has it"
    /// ([`DiscoveryState::Settled`]) and "netd has not completed a discovery pass, so
    /// an empty answer proves NOTHING" ([`DiscoveryState::NotConverged`]).
    ///
    /// **This is NOT the verb for a consumer about to render an absence
    /// claim** — that is [`Self::query_catalog_converged`], which waits through
    /// discovery convergence first, and `cerulion_cli_engine`'s structural guard
    /// FAILS the build if this one appears in the CLI's code. Use this when the
    /// caller runs its OWN wait, or cannot afford one (vizd holds a single
    /// mutex-held `NetdClient`, so a ten-second wait there would block its whole
    /// demand plane). [`Self::query_catalog`] drops the state entirely and is the
    /// right call when the caller has no absence claim to make.
    pub fn query_catalog_with_discovery(
        &mut self,
        robot: Option<&str>,
    ) -> Result<CatalogGather, ClientError> {
        // Like `demand`, the FIRST request on a fresh connection can race the daemon's
        // idle-exit / shutdown (accepted-then-EOF); retry it ONCE (reconnect + resend),
        // silently, and only on the first request (`next_id == 1`).
        let first_request = self.next_id == 1;
        match self.query_catalog_once(robot) {
            Err(e) if first_request && is_closed_early(&e) => {
                self.reconnect()?;
                self.query_catalog_once(robot)
            }
            other => other,
        }
    }

    /// [`Self::query_catalog`] with the first-request RESPAWN arm
    /// removed — one round trip, and an accepted-then-EOF is reported rather than
    /// answered by starting a daemon.
    ///
    /// # Why a separate verb rather than a flag
    ///
    /// [`Self::connect_existing`] exists so a RECORDER can ask whether a daemon is
    /// there and take "no" for an answer — it must not start a network daemon on
    /// the machine it is recording, and it must not block for
    /// the spawn-readiness ceiling inside its own arm-time window. That contract covers
    /// the CONSTRUCTOR only: `query_catalog` delegates to
    /// [`Self::query_catalog_with_discovery`], whose `first_request && is_closed_early`
    /// arm calls `reconnect()` → `connect_or_spawn_at`, which SPAWNS. And
    /// `first_request` is `next_id == 1`, which is ALWAYS true of a recorder's first
    /// query — so through that path the guarantee is lost on exactly the round trip it is
    /// for. MEASURED against a daemon that hangs up after its `Hello`: a daemon
    /// is started and the call blocks 10.06 s.
    ///
    /// This is the same choice the convergence loop makes, which
    /// uses the non-reconnecting `_once` verbs for the same reason (the wall it
    /// documents could otherwise be overrun by ~20 s of respawn ladder). The trade is stated
    /// rather than hidden: an accepted-then-EOF first query here is an `Err` the
    /// caller must handle, not a transparently-retried success.
    pub fn query_catalog_no_respawn(
        &mut self,
        robot: Option<&str>,
    ) -> Result<Vec<CatalogReply>, ClientError> {
        self.query_catalog_once(robot).map(|gather| gather.catalogs)
    }

    /// [`Self::query_schema`] without the respawn arm — see
    /// [`Self::query_catalog_no_respawn`] for why this verb exists.
    pub fn query_schema_no_respawn(
        &mut self,
        robot: Option<&str>,
        requested: &str,
    ) -> Result<Vec<SchemaReply>, ClientError> {
        self.query_schema_once(robot, requested)
            .map(|gather| gather.replies)
    }

    /// Interpret a query response's discovery state against the daemon that
    /// sent it, warning ONCE if that daemon is too old to report one. See
    /// [`trust_reported_discovery`].
    fn reported_discovery(&self, reported: DiscoveryState) -> DiscoveryState {
        if self.daemon_protocol < DISCOVERY_MIN_DAEMON_VERSION {
            warn_stale_daemon_once(self.daemon_protocol);
        }
        trust_reported_discovery(self.daemon_protocol, reported)
    }

    /// One `query_catalog` round-trip (see [`Self::query_catalog_with_discovery`],
    /// which wraps this with the first-request retry).
    fn query_catalog_once(&mut self, robot: Option<&str>) -> Result<CatalogGather, ClientError> {
        let id = self.next_request_id();
        let req = Request::QueryCatalog {
            id,
            robot: robot.map(str::to_string),
        };
        match self.round_trip(&req)? {
            Response::CatalogQuery(c) => Ok(CatalogGather {
                catalogs: c.catalogs,
                // The answering daemon's un-settled plane age (absent on a
                // daemon that predates the field => `None` => UNKNOWN, which caps nothing).
                unsettled_for: c.plane_unsettled_ms.map(Duration::from_millis),
                // An older daemon sent NO state and serde defaulted it to
                // `Settled` — believing that would silently restore the false
                // "not found". `trust_reported_discovery` downgrades it instead.
                discovery: self.reported_discovery(c.discovery),
            }),
            Response::Error(e) => Err(ClientError::Netd {
                error: e.error,
                robot: e.robot,
                topic: e.topic,
            }),
            other => Err(ClientError::Protocol(format!(
                "expected a catalog-query response, got {other:?}"
            ))),
        }
    }

    /// Fetch a remote type's `.msg`/YAML closure over netd's ONE zenoh
    /// session — the consumer's replacement for a transient `schema` GET. `robot:
    /// Some` scopes the GET to ONE robot; `None` asks every announcing robot.
    /// `requested` is a qualified `pkg/Type` OR a package-less bare `Name`. Returns
    /// every decoded [`SchemaReply`] (found / not-found / refused — the consumer picks
    /// the first with non-empty `docs`). `Ok(vec![])` is authoritative (nobody
    /// answered); only a [`ClientError`] triggers the consumer's fallback.
    pub fn query_schema(
        &mut self,
        robot: Option<&str>,
        requested: &str,
    ) -> Result<Vec<SchemaReply>, ClientError> {
        self.query_schema_with_discovery(robot, requested)
            .map(|gather| gather.replies)
    }

    /// [`Self::query_schema`] plus the [`DiscoveryState`] netd answered
    /// under — see [`Self::query_catalog_with_discovery`] for when it matters.
    pub fn query_schema_with_discovery(
        &mut self,
        robot: Option<&str>,
        requested: &str,
    ) -> Result<SchemaGather, ClientError> {
        let first_request = self.next_id == 1;
        match self.query_schema_once(robot, requested) {
            Err(e) if first_request && is_closed_early(&e) => {
                self.reconnect()?;
                self.query_schema_once(robot, requested)
            }
            other => other,
        }
    }

    /// One `query_schema` round-trip (see [`Self::query_schema_with_discovery`]).
    fn query_schema_once(
        &mut self,
        robot: Option<&str>,
        requested: &str,
    ) -> Result<SchemaGather, ClientError> {
        let id = self.next_request_id();
        let req = Request::QuerySchema {
            id,
            robot: robot.map(str::to_string),
            requested: requested.to_string(),
        };
        match self.round_trip(&req)? {
            Response::SchemaQuery(s) => Ok(SchemaGather {
                replies: s.replies,
                // See the catalog twin.
                unsettled_for: s.plane_unsettled_ms.map(Duration::from_millis),
                discovery: self.reported_discovery(s.discovery),
            }),
            Response::Error(e) => Err(ClientError::Netd {
                error: e.error,
                robot: e.robot,
                topic: e.topic,
            }),
            other => Err(ClientError::Protocol(format!(
                "expected a schema-query response, got {other:?}"
            ))),
        }
    }

    /// Ask which runs are LIVE on a robot right now, each carrying its
    /// effective `graph.yaml` + `run.json`, over netd's ONE zenoh session. `robot:
    /// Some` scopes the GET to ONE robot (the desk's fetch for a run it is about to
    /// render); `None` asks every announcing robot.
    ///
    /// # Reading the answer
    ///
    /// [`RunsAnswer::replies`] is one reply per robot that ANSWERED, and a robot that
    /// did not answer is simply ABSENT rather than present with an empty run set. That
    /// distinction is the whole contract: a robot running a Strict ingress-only
    /// gateway (no query surface) or an older binary contributes nothing,
    /// and reading its absence as "that robot is running nothing" would be a
    /// confident lie. Key on reply PRESENCE per robot; within a reply, whether an
    /// empty `runs` means "nothing is running" is carried by that reply's own
    /// `RunsCompleteness`.
    ///
    /// An empty `replies` is therefore "nobody answered USABLY", not "nobody is
    /// running anything"; only a [`ClientError`] means netd could not run the query.
    ///
    /// [`RunsAnswer::unusable`] is the third case, and it is on the PRIMARY verb
    /// deliberately: a robot that answered with bytes this binary could not use is
    /// neither a reply nor a silence, and its remedy — redeploy that robot — is one no
    /// caller can name from an empty list. Returning a bare `Vec<RunsReply>` here
    /// would have collapsed it into "nothing answered", which is the exact confident
    /// lie the paragraph above refuses. See [`RunsAnswer`] for why that makes it an
    /// answer rather than the gather metadata the bare catalog/schema verbs drop.
    ///
    /// # Cost — fetch once per `run_id`, never in a poll
    ///
    /// The robot's serve builds a fresh iceoryx2 reader node per call (the ~620 ms
    /// shape the tab-completion work measured) plus up to one gather window. A run's DAG is immutable
    /// for the life of its `run_id`, so a consumer caches on that and refreshes
    /// liveness off the catalog poll it already makes. **Never fold this verb into
    /// the ~2 s catalog poll.**
    ///
    /// # Version
    ///
    /// Needs a v7 daemon ([`crate::protocol::RUNS_MIN_DAEMON_VERSION`]).
    /// Against an older one the per-verb gate refuses it BEFORE it is sent, with a
    /// [`ClientError::Protocol`] naming both versions and the upgrade — never a
    /// generic unknown-method error a consumer could mistake for a serve failure.
    pub fn query_runs(&mut self, robot: Option<&str>) -> Result<RunsAnswer, ClientError> {
        self.query_runs_with_discovery(robot).map(RunsAnswer::from)
    }

    /// [`Self::query_runs`] plus the [`DiscoveryState`] netd answered
    /// under — the discriminator between "the LAN was searched and no robot serves
    /// runs" ([`DiscoveryState::Settled`]) and "netd has not completed a discovery
    /// pass, so an empty answer proves NOTHING" ([`DiscoveryState::NotConverged`]).
    ///
    /// There is deliberately NO `_converged` sibling (the waiting verb the
    /// catalog and schema queries have). A consumer of this verb is rendering a run's
    /// topology, not making an absence claim about a topic somebody typed, and the
    /// one consumer in view — `cerulion-vizd` — may not wait inside a control handler
    /// at all. A caller that genuinely needs to wait out convergence runs
    /// its own loop over this verb.
    pub fn query_runs_with_discovery(
        &mut self,
        robot: Option<&str>,
    ) -> Result<RunsGather, ClientError> {
        // Like `demand`, the FIRST request on a fresh connection can race the daemon's
        // idle-exit / shutdown (accepted-then-EOF); retry it ONCE (reconnect + resend),
        // silently, and only on the first request (`next_id == 1`).
        let first_request = self.next_id == 1;
        match self.query_runs_once(robot) {
            Err(e) if first_request && is_closed_early(&e) => {
                self.reconnect()?;
                self.query_runs_once(robot)
            }
            other => other,
        }
    }

    /// [`Self::query_runs`] with the first-request RESPAWN arm removed:
    /// one round trip, and an accepted-then-EOF is reported rather than answered by
    /// starting a daemon. See [`Self::query_catalog_no_respawn`] for the contract and
    /// why a separate verb rather than a flag.
    ///
    /// It carries [`RunsAnswer::unusable`] for the same reason the respawning verb
    /// does — a recorder that cannot start a daemon is if anything MORE likely to be
    /// looking at a robot whose binary has drifted from its own.
    pub fn query_runs_no_respawn(
        &mut self,
        robot: Option<&str>,
    ) -> Result<RunsAnswer, ClientError> {
        self.query_runs_once(robot).map(RunsAnswer::from)
    }

    /// One `query_runs` round-trip (see [`Self::query_runs_with_discovery`]).
    fn query_runs_once(&mut self, robot: Option<&str>) -> Result<RunsGather, ClientError> {
        let id = self.next_request_id();
        let req = Request::QueryRuns {
            id,
            robot: robot.map(str::to_string),
        };
        match self.round_trip(&req)? {
            Response::RunsQuery(r) => Ok(RunsGather {
                replies: r.run_replies,
                // A robot that answered UNUSABLY is carried through
                // rather than folded into the missing ones — its remedy is a
                // redeploy, not a wait.
                unusable: r.unusable,
                // And the robots that answered NOTHING are carried
                // too — a third remedy again (wait, or upgrade that robot). An
                // OLDER v7 daemon omits the key and decodes to an empty list,
                // which is its pre-B4 behaviour exactly: it asserted full coverage
                // implicitly, and a consumer that believes it is no worse off than
                // it was. What the field buys is a NEWER daemon's ability to stop
                // asserting it.
                silent: r.silent,
                // See the catalog twin.
                unsettled_for: r.plane_unsettled_ms.map(Duration::from_millis),
                // Routed through the SAME trust gate as its siblings even
                // though it is structurally a no-op here — the verb needs a v7 daemon
                // and `RUNS_MIN_DAEMON_VERSION >= DISCOVERY_MIN_DAEMON_VERSION`
                // (const-asserted below), so no daemon that could answer this is old
                // enough to be distrusted. One policy, applied at every site, beats a
                // site that is correct today because of an inequality elsewhere.
                discovery: self.reported_discovery(r.discovery),
            }),
            Response::Error(e) => Err(ClientError::Netd {
                error: e.error,
                robot: e.robot,
                topic: e.topic,
            }),
            other => Err(ClientError::Protocol(format!(
                "expected a runs-query response, got {other:?}"
            ))),
        }
    }

    /// [`Self::query_catalog_with_discovery`], but KEEP ASKING while the
    /// daemon reports [`DiscoveryState::NotConverged`] with an empty answer — the
    /// first-contact wait.
    ///
    /// This is the verb a consumer should call when it is about to render an empty
    /// answer to a HUMAN. The trust gate made that answer accurate ("UNKNOWN, not absent");
    /// the convergence wait makes the common case not need it, because a cold desk's daemon is
    /// typically seconds away from seeing the robot that is right there on the LAN.
    ///
    /// A caller that renders NO absence claim must pass [`ConvergenceWait::off`]
    /// rather than the default — see that constructor.
    ///
    /// See [`FirstContactWait`] for the progress sink and the cancellation flag, and
    /// [`Converged`] for what comes back.
    pub fn query_catalog_converged(
        &mut self,
        robot: Option<&str>,
        wait: &mut FirstContactWait<'_>,
    ) -> Result<Converged<CatalogGather>, ConvergenceAbort> {
        let robot = robot.map(str::to_string);
        self.query_converged(
            wait,
            // The NON-reconnecting `_once` verb, deliberately.
            // `query_catalog_with_discovery` retries its FIRST request through
            // `reconnect()` on the netd idle-exit race, and `reconnect` re-runs the
            // whole connect-or-spawn ladder — TWICE, each with its own
            // `SPAWN_READY_TIMEOUT`. Inside this loop the first round trip always IS
            // the first request, so that path would be live and the documented ~15 s wall
            // would be false by up to ~20 s. A wait loop that is about to re-ask does not
            // need a reconnect: the connection was established by the caller's own
            // `connect_or_spawn`, and an early close here surfaces as a
            // `ConvergenceAbort` the consumer already degrades on (a transient
            // session, loudly). Cost: the idle-exit retry does not cover
            // this one seam — an accepted-then-EOF first query falls back instead of
            // respawning. See `crate::convergence::worst_case_wall`.
            |client| client.query_catalog_once(robot.as_deref()),
            |gather| RoundTripVerdict {
                discovery: gather.discovery,
                answer_empty: gather.catalogs.is_empty(),
                plane_unsettled_for: gather.unsettled_for,
            },
        )
    }

    /// [`Self::query_schema_with_discovery`] under the same first-contact
    /// wait — see [`Self::query_catalog_converged`].
    ///
    /// Both query verbs route through it because both render an empty answer to a
    /// human: `cerulion topic echo/info/hz` resolves a topic through the CATALOG,
    /// while `cerulion schema info` resolves a type through the SCHEMA verb, and a
    /// cold daemon fails the second exactly as it failed the first.
    pub fn query_schema_converged(
        &mut self,
        robot: Option<&str>,
        requested: &str,
        wait: &mut FirstContactWait<'_>,
    ) -> Result<Converged<SchemaGather>, ConvergenceAbort> {
        let robot = robot.map(str::to_string);
        let requested = requested.to_string();
        self.query_converged(
            wait,
            // Non-reconnecting — see the catalog twin.
            |client| client.query_schema_once(robot.as_deref(), &requested),
            |gather| RoundTripVerdict {
                discovery: gather.discovery,
                answer_empty: gather.replies.is_empty(),
                plane_unsettled_for: gather.unsettled_for,
            },
        )
    }

    /// The ONE first-contact wait loop, shared by both query verbs.
    ///
    /// Deliberately THIN — every policy question (proceed / wait / give up, and for
    /// how long) is answered by the pure [`ConvergenceWait::decide`]; this function
    /// only runs round trips, calls the progress sink, and sleeps in
    /// cancellation-checked slices.
    ///
    /// A transport error propagates IMMEDIATELY rather than being retried: the wait
    /// exists to bridge DISCOVERY latency, and a broken control seam is a different
    /// failure whose existing consumer-side fallback (a transient session, a loud
    /// degrade) must not be delayed by ten seconds of hopeful polling. It carries the
    /// elapsed wait out with it ([`ConvergenceAbort`]) so the consumer can close the
    /// progress lines the user has been watching.
    fn query_converged<T>(
        &mut self,
        wait: &mut FirstContactWait<'_>,
        mut round_trip: impl FnMut(&mut Self) -> Result<T, ClientError>,
        verdict: impl Fn(&T) -> RoundTripVerdict,
    ) -> Result<Converged<T>, ConvergenceAbort> {
        let started = Instant::now();
        let mut progress_lines = 0u32;
        // Announce the wait BEFORE the first round trip when one is possible.
        // That first round trip is the LONGEST silent stretch of the whole wait (up to
        // ~3.75 s against a cold daemon), so a sink that only fires on a KeepWaiting
        // decision leaves the user staring at nothing for exactly the interval in
        // which they decide the command is hung. A no-wait policy prints nothing.
        if wait.policy.enabled() {
            wait.note_progress(Duration::ZERO);
            progress_lines += 1;
        }
        loop {
            let answer = round_trip(self).map_err(|error| ConvergenceAbort {
                error,
                waited: started.elapsed(),
                progress_lines,
            })?;
            let v = verdict(&answer);
            let waited = started.elapsed();
            // Cancellation is checked AFTER the round trip so there is always an
            // answer to hand back. The round trip itself is UNINTERRUPTIBLE (up to
            // `ROUNDTRIP_TIMEOUT`, ~3.75 s on a cold daemon) and is the longest such
            // stretch; the sleep is the one this loop can slice, and it does. See
            // `CANCEL_CHECK_SLICE` for the real bound.
            if wait.is_cancelled() {
                return Ok(Converged {
                    answer,
                    waited,
                    outcome: WaitOutcome::Cancelled,
                    progress_lines,
                });
            }
            match wait
                .policy
                .decide(v.discovery, v.answer_empty, waited, v.plane_unsettled_for)
            {
                WaitDecision::Proceed => {
                    return Ok(Converged {
                        answer,
                        waited,
                        outcome: WaitOutcome::Answered,
                        progress_lines,
                    });
                }
                WaitDecision::GiveUpHonestUnknown => {
                    return Ok(Converged {
                        answer,
                        waited,
                        outcome: WaitOutcome::GaveUp,
                        progress_lines,
                    });
                }
                WaitDecision::KeepWaiting { next_poll_delay } => {
                    wait.note_progress(waited);
                    progress_lines += 1;
                    if !wait.sleep_cancellable(next_poll_delay) {
                        return Ok(Converged {
                            answer,
                            waited: started.elapsed(),
                            outcome: WaitOutcome::Cancelled,
                            progress_lines,
                        });
                    }
                }
            }
        }
    }

    /// Register THIS connection's egress plan with the shared
    /// daemon — the produced topics a desk graph wants netd to announce +
    /// egress-on-demand over the machine's ONE zenoh session (replacing a per-run
    /// gateway child on the desk). netd boots the shared embedded gateway on the
    /// FIRST egress plan and pushes each of `plan`'s announce topics onto it. `serving`
    /// is the workspace's catalog/schema closure (netd cannot reach the `.msg` store
    /// itself, so the producing side hands it across).
    ///
    /// The registration is SCOPED to this connection: dropping the client (a clean
    /// exit, a panic, a `SIGINT`/`SIGKILL` — anything that closes the fd) releases it,
    /// exactly like a held demand (the crash-safe refcount). [`Self::release_egress`]
    /// is the explicit EARLY release. Keep the client alive for as long as the run
    /// needs to egress.
    ///
    /// `ix_config_json` is the producing run's RESOLVED iceoryx2
    /// namespace (serialized `iceoryx2::config::Config`) — `Some` for a MULTI-PROCESS
    /// run (the supervisor's shared worker namespace), `None` for a MONOLITH run
    /// (it shares netd's default namespace by construction). netd's embedded gateway
    /// VERIFIES a `Some` config matches its shared session before tapping; a config
    /// carrying a DIFFERENT namespace is refused (the caller then falls back to a
    /// per-run gateway child on its own namespace). A config-carrying register needs
    /// a v4 daemon.
    ///
    /// A daemon too OLD to serve the verb is refused LOCALLY (before the request is
    /// sent) with a precise version error (the per-verb compat gate) — never a
    /// silently-lost registration, and never a config silently ignored (a v4 client
    /// refuses to hand a namespace to a v<4 daemon that could not honor it).
    pub fn register_egress(
        &mut self,
        plan: &GatewayPlan,
        serving: &SchemaServing,
        ix_config_json: Option<&str>,
    ) -> Result<EgressResponse, ClientError> {
        // Like `demand`, the FIRST request on this connection can race the
        // daemon's idle-exit / shutdown (Hello served then closed before the register).
        // Retry ONCE — RECONNECT (re-classifying: a gone socket respawns a fresh daemon;
        // a live one reconnects) and re-send. Only the first request (no request sent
        // yet: `next_id == 1`) is retried; a later register failing is a genuine
        // mid-session error and a second early close is surfaced LOUDLY.
        let first_request = self.next_id == 1;
        match self.register_egress_once(plan, serving, ix_config_json) {
            Err(e) if first_request && is_closed_early(&e) => {
                self.reconnect()?;
                self.register_egress_once(plan, serving, ix_config_json)
            }
            other => other,
        }
    }

    /// One `register_egress` round-trip (see [`Self::register_egress`], which wraps
    /// this with the first-request retry).
    fn register_egress_once(
        &mut self,
        plan: &GatewayPlan,
        serving: &SchemaServing,
        ix_config_json: Option<&str>,
    ) -> Result<EgressResponse, ClientError> {
        let id = self.next_request_id();
        let req = Request::RegisterEgress {
            id,
            plan: plan.clone(),
            schema_serving: serving.clone(),
            ix_config_json: ix_config_json.map(str::to_string),
        };
        match self.round_trip(&req)? {
            Response::Egress(e) => Ok(e),
            Response::Error(e) => Err(ClientError::Netd {
                error: e.error,
                robot: e.robot,
                topic: e.topic,
            }),
            other => Err(ClientError::Protocol(format!(
                "expected an egress response, got {other:?}"
            ))),
        }
    }

    /// Release THIS connection's egress registration: the explicit
    /// EARLY release (the inverse of [`Self::register_egress`]). A no-op if this
    /// connection holds none. Dropping the whole client releases it implicitly (the
    /// pairing guarantee), so most callers never need this; it is the symmetric verb
    /// for a client that outlives one graph's egress need.
    pub fn release_egress(&mut self) -> Result<EgressReleaseResponse, ClientError> {
        let id = self.next_request_id();
        let req = Request::ReleaseEgress { id };
        match self.round_trip(&req)? {
            Response::EgressRelease(r) => Ok(r),
            Response::Error(e) => Err(ClientError::Netd {
                error: e.error,
                robot: e.robot,
                topic: e.topic,
            }),
            other => Err(ClientError::Protocol(format!(
                "expected an egress release response, got {other:?}"
            ))),
        }
    }

    /// The raw fd of this client's live control connection, for a
    /// liveness WATCH that POLLS the connection (a non-blocking `recv(MSG_PEEK)`) to
    /// detect the daemon dying mid-run so the run can warn once and continue
    /// LOCAL-ONLY. BORROWED — the fd is owned by this client and is valid ONLY while
    /// the client is alive; the caller MUST stop using it before dropping the client,
    /// and MUST NEVER `close(2)` it (the client's own `Drop` owns the close, which is
    /// the crash-safe egress release). A poll-only borrow keeps the "connection-close
    /// = release" contract intact (no dup extends the connection's lifetime).
    pub fn control_fd(&self) -> std::os::unix::io::RawFd {
        use std::os::unix::io::AsRawFd;
        self.stream.as_raw_fd()
    }

    /// The control-socket path this client is connected to.
    pub fn socket_path(&self) -> &Path {
        &self.socket_path
    }

    /// The [`PROTOCOL_VERSION`] the CONNECTED daemon announced (Principle
    /// #3 — the version the per-verb compat gate checks against). Updated on a
    /// reconnect, which can land on a different-version daemon.
    pub fn daemon_protocol(&self) -> u32 {
        self.daemon_protocol
    }

    /// The next monotonic correlation id.
    fn next_request_id(&mut self) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    /// Send one request and read + parse its response line. `Response` is an
    /// untagged enum whose variants carry non-overlapping REQUIRED discriminating
    /// fields (`mirror_created` / `last_release` / `demands` / `gateway_started` /
    /// `released_topics` / `catalogs` / `replies` / `error`), so serde deserializes
    /// each response line unambiguously to its concrete variant.
    ///
    /// Refuses a verb the connected daemon is too old to serve BEFORE
    /// sending it ([`verb_compat_error`]) — a v2 client never sends an egress verb to
    /// a v1 daemon that would fail to parse it; it surfaces a precise, actionable
    /// version error instead.
    fn round_trip(&mut self, req: &Request) -> Result<Response, ClientError> {
        self.ensure_usable()?;
        if let Some(msg) = verb_compat_error(self.daemon_protocol, req) {
            return Err(ClientError::Protocol(msg));
        }
        self.write_line(&req.to_json_line())?;
        let line = self.read_line()?;
        serde_json::from_str::<Response>(line.trim())
            .map_err(|e| ClientError::Protocol(format!("unparseable response: {e}")))
    }

    /// Write one NDJSON line + newline.
    fn write_line(&mut self, line: &str) -> Result<(), ClientError> {
        self.ensure_usable()?;
        writeln!(self.stream, "{line}").map_err(ClientError::Io)?;
        self.stream.flush().map_err(ClientError::Io)
    }

    /// Read one line (a full NDJSON record). A clean EOF (the daemon closed the
    /// connection) is an I/O error, never a silent empty read.
    fn read_line(&mut self) -> Result<String, ClientError> {
        self.ensure_usable()?;
        let mut line = String::new();
        let n = self.reader.read_line(&mut line).map_err(ClientError::Io)?;
        if n == 0 {
            return Err(ClientError::Io(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "cerulion-netd closed the connection",
            )));
        }
        Ok(line)
    }

    /// Turn this connection into a catalog-change EVENT subscriber.
    ///
    /// Sends `subscribe_catalog`, reads back the snapshot response, and hands over a
    /// [`NetdEventClient`] that reads the unsolicited [`CatalogChanged`] pushes netd
    /// writes down this connection from then on.
    ///
    /// It CONSUMES the client deliberately. A subscribed connection carries two kinds
    /// of line — responses AND unsolicited pushes — so the plain request/response
    /// `round_trip` (write one line, read one line) is not sound on it: a push
    /// landing between a request and its response would be returned as the response.
    /// Making the subscription a one-way door means that desync is unrepresentable
    /// rather than merely documented. A consumer that needs both keeps TWO
    /// connections, which is what `cerulion-vizd` does — its demand plane's client is
    /// untouched.
    ///
    /// # Errors
    ///
    /// A daemon older than v6 is refused by the per-verb version gate BEFORE anything
    /// is sent ([`Request::min_daemon_version`]), so the caller gets a precise
    /// [`ClientError::Protocol`] naming the skew and can degrade to its own refresh
    /// path. The connection is consumed either way — the caller reconnects if it wants
    /// a plain client back.
    pub fn subscribe_catalog(
        mut self,
    ) -> Result<(SubscribeCatalogResponse, NetdEventClient), ClientError> {
        let id = self.next_request_id();
        let req = Request::SubscribeCatalog { id };
        if let Some(msg) = verb_compat_error(self.daemon_protocol, &req) {
            return Err(ClientError::Protocol(msg));
        }
        self.write_line(&req.to_json_line())?;
        // Read until OUR response arrives. A push CAN legitimately land first (netd
        // registers the subscription before writing the response, so a change in that
        // gap is delivered rather than dropped), so lines are CLASSIFIED, never
        // assumed — and an early push is queued, not discarded.
        let mut queued: Vec<CatalogChanged> = Vec::new();
        let response = loop {
            let line = self.read_line()?;
            match classify_control_line(&line) {
                ControlLine::Event(event) => queued.push(event),
                ControlLine::Response(text) => {
                    match serde_json::from_str::<Response>(&text) {
                        Ok(Response::SubscribeCatalog(r)) if r.id == id => break r,
                        Ok(Response::Error(e)) => {
                            return Err(ClientError::Netd {
                                error: e.error,
                                robot: e.robot,
                                topic: e.topic,
                            });
                        }
                        Ok(_) | Err(_) => {
                            return Err(ClientError::Protocol(format!(
                                "expected a subscribe_catalog response, got: {text}"
                            )));
                        }
                    };
                }
                ControlLine::Unknown(text) => {
                    return Err(ClientError::Protocol(format!(
                        "unclassifiable control line while subscribing: {text}"
                    )));
                }
            }
        };
        // A long-lived reader must be able to notice a shutdown request, so it reads
        // under a bounded timeout and loops rather than blocking forever.
        self.stream
            .set_read_timeout(Some(EVENT_READ_TIMEOUT))
            .map_err(ClientError::Io)?;
        Ok((
            response,
            NetdEventClient {
                _stream: self.stream,
                reader: self.reader,
                queued,
                socket_path: self.socket_path,
                daemon_protocol: self.daemon_protocol,
            },
        ))
    }
}

/// How long [`NetdEventClient::next_event`] blocks before reporting "nothing yet".
/// Bounds only how quickly a reader thread notices its own shutdown — an actual push
/// wakes the read immediately.
const EVENT_READ_TIMEOUT: Duration = Duration::from_millis(500);

/// The READ half of a catalog-change subscription — a netd connection that
/// has been turned into an event stream by [`NetdClient::subscribe_catalog`].
///
/// Dropping it closes the connection, which unsubscribes it (the same crash-safe rule
/// as the demand plane: connection close IS the release).
#[derive(Debug)]
pub struct NetdEventClient {
    /// Held so the connection stays open — the reader is a `try_clone` of it.
    _stream: UnixStream,
    reader: BufReader<UnixStream>,
    /// Pushes that arrived before the subscribe response (netd subscribes the
    /// connection before answering, so a change in that gap is DELIVERED, not lost).
    queued: Vec<CatalogChanged>,
    socket_path: PathBuf,
    daemon_protocol: u32,
}

/// The outcome of one [`NetdEventClient::next_event`] poll.
#[derive(Debug)]
pub enum NextEvent {
    /// A catalog change arrived.
    Changed(Box<CatalogChanged>),
    /// The poll window elapsed with nothing to report. NOT an error and NOT "nothing
    /// changed forever" — the caller loops (checking its own shutdown flag first).
    Idle,
}

impl NetdEventClient {
    /// The socket this subscription is connected to.
    pub fn socket_path(&self) -> &Path {
        &self.socket_path
    }

    /// The [`PROTOCOL_VERSION`] the connected daemon announced.
    pub fn daemon_protocol(&self) -> u32 {
        self.daemon_protocol
    }

    /// Block up to a bounded poll window (`EVENT_READ_TIMEOUT`) for the next catalog
    /// change.
    ///
    /// # Errors
    ///
    /// An [`ClientError::Io`] means the connection is GONE (netd exited, or the daemon
    /// dropped us) — the caller reconnects. A read TIMEOUT is not an error: it is
    /// [`NextEvent::Idle`].
    pub fn next_event(&mut self) -> Result<NextEvent, ClientError> {
        if !self.queued.is_empty() {
            return Ok(NextEvent::Changed(Box::new(self.queued.remove(0))));
        }
        let mut line = String::new();
        match self.reader.read_line(&mut line) {
            Ok(0) => Err(ClientError::Io(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "cerulion-netd closed the catalog-change subscription",
            ))),
            Ok(_) => match classify_control_line(&line) {
                ControlLine::Event(event) => Ok(NextEvent::Changed(Box::new(event))),
                // A subscribed connection sends no further requests, so a RESPONSE
                // here is netd talking about something we did not ask for. Skipping it
                // keeps the stream in sync (the alternative — treating it as an event —
                // is the desync this classification exists to prevent) and it is
                // reported at `debug!` rather than silently swallowed.
                ControlLine::Response(text) => {
                    tracing::debug!(line = %text, "netd: unexpected response on a catalog-change subscription — skipped");
                    Ok(NextEvent::Idle)
                }
                ControlLine::Unknown(text) => {
                    tracing::debug!(line = %text, "netd: unclassifiable line on a catalog-change subscription — skipped");
                    Ok(NextEvent::Idle)
                }
            },
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock
                        | io::ErrorKind::TimedOut
                        | io::ErrorKind::Interrupted
                ) =>
            {
                Ok(NextEvent::Idle)
            }
            Err(e) => Err(ClientError::Io(e)),
        }
    }
}

/// One Hello interpretation shared by the legacy and deadline-bound constructors.
fn validate_hello(line: &str) -> Result<u32, ClientError> {
    let hello: Hello = serde_json::from_str(line.trim())
        .map_err(|e| ClientError::Protocol(format!("unparseable hello banner: {e}")))?;
    if hello.hello != HELLO_MARKER {
        return Err(ClientError::Protocol(format!(
            "unexpected banner marker '{}' (expected '{HELLO_MARKER}') — is this really \
             cerulion-netd?",
            hello.hello
        )));
    }
    // Accept any daemon whose vocabulary INCLUDES this consumer's
    // baseline (demand/release/status = v1) — a NEWER daemon is a superset, and an
    // OLDER-but-still-v1 daemon serves the baseline (so a v2 client keeps working
    // against a pinned v1 daemon). Refuse ONLY a daemon too old for the baseline.
    // Per-verb gating for the v2 egress verbs happens at the send site
    // (`verb_compat_error`), so this is deliberately NOT a strict `!=` (which would
    // wedge a v2 client against a v1 daemon it can fully serve, and vice versa).
    if hello.protocol < CLIENT_MIN_DAEMON_VERSION {
        return Err(ClientError::Protocol(format!(
            "cerulion-netd speaks protocol v{} but this client needs at least v{CLIENT_MIN_DAEMON_VERSION} \
             for the demand/release control vocabulary (this client speaks v{PROTOCOL_VERSION}). \
             Upgrade cerulion-netd — it is older than this client can talk to.",
            hello.protocol
        )));
    }
    Ok(hello.protocol)
}

/// Try to connect to the control socket. A [`io::ErrorKind::NotFound`] /
/// [`io::ErrorKind::ConnectionRefused`] means "no daemon is listening" (see
/// [`is_not_running`]); any other error is a real failure.
fn try_connect(socket: &Path) -> io::Result<UnixStream> {
    UnixStream::connect(socket)
}

/// What the readiness wait knows about the spawned daemon process.
///
/// The third state is the point. `Child::try_wait` can FAIL —
/// classically `ECHILD` when the calling process has SIG_IGN'd `SIGCHLD` and the child
/// was auto-reaped before we asked. That is not evidence of death and not evidence of
/// life, so it must be neither [`Exited`](ChildLiveness::Exited) (which would fail-fast
/// a healthy daemon at the post-exit grace) nor [`Running`](ChildLiveness::Running)
/// (which would state liveness as fact in the expiry message). It is
/// [`Unknown`](ChildLiveness::Unknown): keep waiting to the full ceiling, and SAY that
/// the verdict is unknown.
#[derive(Debug, Clone, Copy)]
enum ChildLiveness {
    /// Not yet observed to exit (the initial state, and the state while `try_wait`
    /// keeps answering "still running").
    Running,
    /// Observed to have exited, and WHEN we observed it — the fail-fast limb.
    Exited(std::process::ExitStatus, Instant),
    /// We could not observe it (a `try_wait` error). No fail-fast, no liveness claim.
    Unknown,
}

impl ChildLiveness {
    /// How long ago the child was observed to EXIT, or `None` when it has not been
    /// observed to exit — which includes [`Unknown`](ChildLiveness::Unknown), so an
    /// unobservable child never triggers the fail-fast limb.
    fn exited_for(self) -> Option<Duration> {
        match self {
            ChildLiveness::Exited(_, at) => Some(at.elapsed()),
            ChildLiveness::Running | ChildLiveness::Unknown => None,
        }
    }
}

/// Whether the post-spawn READINESS wait has run out.
///
/// Two independent bounds, whichever bites first:
/// - `since_child_exit` — set once the spawned child has been observed to exit; from
///   that moment only `post_exit_grace` remains (the fail-fast path, which still leaves
///   room for a `flock` winner's socket to appear). `None` covers BOTH "still running"
///   and "liveness unobservable" ([`ChildLiveness::Unknown`]) — an answer we could not
///   get must never shorten the wait;
/// - `elapsed` — the overall `ready_timeout` ceiling for a child that is still alive but
///   has not bound (still booting, or wedged).
///
/// Pure — oracle-tested.
fn spawn_wait_exhausted(
    elapsed: Duration,
    since_child_exit: Option<Duration>,
    ready_timeout: Duration,
    post_exit_grace: Duration,
) -> bool {
    if since_child_exit.is_some_and(|d| d >= post_exit_grace) {
        return true;
    }
    elapsed >= ready_timeout
}

/// Whether the readiness wait should emit its ONE stall notice now.
///
/// Due exactly once: the first poll at or past `notice_after` with nothing
/// connected, never again for that wait (`already_notified`). The comparison
/// is `>=`, so a bound equal to the elapsed wall is due (a `>` would let a
/// poll landing exactly on the bound slip through to the next tick and shift
/// the notice by one poll, which the boundary arm of the oracle pins).
///
/// Pure, oracle-tested.
fn stall_notice_due(elapsed: Duration, notice_after: Duration, already_notified: bool) -> bool {
    !already_notified && elapsed >= notice_after
}

/// Whether a [`UnixStream::connect`] error means "no daemon is running" (so the
/// client should spawn one) vs a real error to surface. A missing socket file is
/// `NotFound`; a stale socket left by a crashed daemon is `ConnectionRefused`. Pure
/// — oracle-tested.
pub fn is_not_running(e: &io::Error) -> bool {
    matches!(
        e.kind(),
        io::ErrorKind::NotFound | io::ErrorKind::ConnectionRefused
    )
}

/// Whether a [`ClientError`] is the daemon-accepted-then-closed WEDGE/RACE
/// signature — the daemon accepted our connection but closed it (EOF / broken pipe
/// / reset / an unusable socket) at or before the first exchange, because it
/// committed to idle self-exit or shutdown as we arrived. These are the errors
/// [`NetdClient::connect_or_spawn_at`] and [`NetdClient::demand`] retry ONCE
/// (re-classifying + respawning). A protocol error, a daemon `Netd` error, or a
/// genuine `Connect` failure is NOT retried — only a mid-handshake / first-request
/// close. Pure — oracle-tested.
///
/// **`InvalidInput` is that same close, reported at a different syscall.**
/// The daemon's refusal path is deliberate — `handle_connection` drops the stream
/// WITHOUT a `Hello` when [`crate::daemon`]'s idle-watch has already committed to
/// self-exit — and it is SAFE only because this predicate makes the client retry.
/// On macOS the race has TWO observable shapes, decided by whether the peer's close
/// lands before or after [`NetdClient::finish_handshake`]'s `set_read_timeout`:
///
/// * close lands SECOND → the timeouts install, the `Hello` read hits a clean EOF →
///   `UnexpectedEof`. MEASURED at 19_999/20_000 on the `client_e2e_test` fixture.
/// * close lands FIRST → xnu refuses `setsockopt` on a fully-closed socket with
///   `EINVAL`, so the handshake fails BEFORE it ever reads → `InvalidInput`.
///   MEASURED at 1/20_000 naturally, and 200/200 when the ordering is forced.
///
/// Were only the first classified, the single retry would not fire on the
/// second, and `topic echo` / `viz` / `schema info` would surface a bare
/// `Invalid argument (os error 22)` instead of silently reconnecting — the exact
/// class the first-request retry exists to prevent. (Both shapes are correct loud behaviour on the
/// GIVE-UP arm; on the FIRST attempt an unclassified shape
/// defeats the retry.)
///
/// Widening is bounded on both sides. Every `ClientError::Io` this client can
/// produce comes from a socket operation on the netd connection; EVERY duration it
/// passes to `set_{read,write}_timeout` is a nonzero const — [`ROUNDTRIP_TIMEOUT`]
/// in [`NetdClient::finish_handshake`] and [`EVENT_READ_TIMEOUT`] in
/// [`NetdClient::subscribe_catalog`] — so `EINVAL` cannot mean "bad argument" here;
/// and the retry is SINGLE, so a genuinely invalid input fails again and surfaces.
///
/// That enumeration is stated over the WHOLE client rather than over the retry
/// scopes, deliberately: `subscribe_catalog` consumes `self` and sits inside none of
/// the `is_closed_early` scopes, so it is out of the widening's blast radius today —
/// but the property that makes the widening safe is "no reachable `set_*_timeout`
/// takes a caller-supplied duration", and checking that over the whole client is
/// what keeps the argument true if a future timeout call site lands INSIDE a retried
/// path. (`ROUNDTRIP_TIMEOUT` is NOT the only such
/// duration, so the enumeration above names both: a doc that IS
/// the safety argument has to be true at the point it does the work.)
///
/// **`NotConnected` is the same close at yet another syscall.** macOS reports a
/// write or read on a UDS whose peer has already closed as `ENOTCONN` ("Socket
/// is not connected") where Linux reports `EPIPE`/`ECONNRESET`/EOF — OBSERVED on
/// the `Test (macOS)` CI shard, on the retried-then-closed-again arm of
/// `client_e2e_test`. The same kind can land on the FIRST attempt of the race,
/// where an unclassified kind means a loud error instead of the single retry —
/// so it joins the set on the argument: every `Io` error here is a socket
/// op on the netd connection, and the retry is SINGLE.
fn is_closed_early(e: &ClientError) -> bool {
    matches!(
        e,
        ClientError::Io(io) if matches!(
            io.kind(),
            io::ErrorKind::UnexpectedEof
                | io::ErrorKind::BrokenPipe
                | io::ErrorKind::ConnectionReset
                | io::ErrorKind::NotConnected
                | io::ErrorKind::InvalidInput
        )
    )
}

/// Which `cerulion-netd` binary the spawn should launch — or, when no rung of the
/// ladder holds one, every path that was tried.
///
/// `NoneFound` carries the whole `tried` list rather than one path because that
/// list IS the operator's next step: "beside this binary" alone does not say
/// WHICH directory when the running binary reached its rung through a symlink.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NetdBinChoice {
    /// Launch this.
    Found(PathBuf),
    /// No rung held a launchable daemon. Every candidate, in ladder order.
    NoneFound {
        /// Every path examined, in the order the ladder examined it.
        tried: Vec<PathBuf>,
    },
}

/// Resolve the `cerulion-netd` binary to spawn (see [`resolve_netd_bin_from`] for
/// the ladder + its rationale). The IO shell: it reads the env, asks the OS for
/// this process's executable path AND for that path with symlinks resolved, then
/// hands all three to the pure decision.
fn resolve_netd_bin() -> io::Result<PathBuf> {
    resolve_netd_bin_for(&std::env::current_exe()?)
}

/// The IO shell over ONE known executable path.
///
/// Split from [`resolve_netd_bin`] so the `canonicalize` step below is REACHABLE
/// FROM A TEST. Every oracle for the ladder injects `resolved_exe` by hand, so
/// without this seam nothing observes whether production ever asks for it — and
/// that one line is what makes rung three reachable. Replacing it
/// with `None` leaves a symlinked vizd finding no
/// daemon, and no other `cerulion_netd` test notices.
///
/// Pinned by `the_io_shell_really_canonicalises_so_a_symlinked_exe_reaches_rung_three`.
pub(crate) fn resolve_netd_bin_for(current_exe: &Path) -> io::Result<PathBuf> {
    let env = std::env::var(NETD_BIN_ENV).ok().filter(|s| !s.is_empty());
    // Rung 3's input. `current_exe()` is NOT canonicalised on macOS — MEASURED:
    // a binary invoked through a symlink reports the SYMLINK's path, and only
    // Linux's `/proc/self/exe` hands back the target. That is why this rung
    // exists at all; see `resolve_netd_bin_from`.
    let resolved_exe = match std::fs::canonicalize(current_exe) {
        Ok(p) => Some(p),
        Err(e) => {
            // NOT a refusal — rung 2 may still hold the daemon — but an operator
            // must be able to tell "rung 3 found nothing" from "rung 3 never
            // ran", and the latter is byte-for-byte the condition.
            tracing::debug!(
                error = %e,
                exe = %current_exe.display(),
                "cerulion-netd: could not resolve the running binary's real path; the \
                 resolved-directory rung is unavailable"
            );
            None
        }
    };
    match resolve_netd_bin_from(
        env.as_deref(),
        current_exe,
        resolved_exe.as_deref(),
        &is_executable_file,
    ) {
        NetdBinChoice::Found(bin) => Ok(bin),
        NetdBinChoice::NoneFound { tried } => Err(io::Error::new(
            io::ErrorKind::NotFound,
            describe_no_netd_binary(current_exe, &tried),
        )),
    }
}

/// The PURE binary-resolution LADDER (oracle-tested), in order:
///
/// 1. **[`NETD_BIN_ENV`]**, verbatim and EXCLUSIVE — an explicit override is used
///    whether or not it exists, so a wrong one fails naming ITSELF instead of
///    being silently papered over by a sibling that happens to be there.
/// 2. **`cerulion-netd` beside `current_exe`** — the sibling-binary install layout
///    (a release dir, a `Contents/MacOS` bundle, `/usr/local/bin`).
/// 3. **`cerulion-netd` beside `resolved_exe`** — the same directory *after*
///    symlinks are resolved, deduped against rung 2 when they agree.
///
/// # Why rung 3 exists (seen on a live desk)
///
/// Studio's deploy convention SYMLINKS `cerulion-vizd` beside the shell, and on
/// macOS `std::env::current_exe()` reports the path the process was EXECed with —
/// the symlink — so rung 2 resolved to the shell's own directory, which carries no
/// daemon. Every vizd spawn attempt failed `No such file or directory` (8000+
/// suppressed failures in one session) and the Studio sidebar was empty. The
/// daemon was sitting in the checkout the symlink pointed INTO, which is exactly
/// what rung 3 looks at. Every prior session had accidentally worked because some
/// CLI-spawned netd was already holding the socket.
///
/// Rung 3 is a no-op on Linux (`/proc/self/exe` is already resolved, so it dedupes
/// away) and on any un-symlinked install — it can only ever ADD a directory the
/// running binary genuinely came from.
///
/// `exists` decides what counts as launchable, injected so the whole ladder is
/// pure. Production passes `is_executable_file`: a present-but-not-executable
/// file is not a daemon, and treating it as one would stop the ladder at a rung
/// that can only produce `EACCES`.
pub fn resolve_netd_bin_from(
    env: Option<&str>,
    current_exe: &Path,
    resolved_exe: Option<&Path>,
    exists: &dyn Fn(&Path) -> bool,
) -> NetdBinChoice {
    if let Some(explicit) = env {
        return NetdBinChoice::Found(PathBuf::from(explicit));
    }
    let mut tried: Vec<PathBuf> = Vec::new();
    for dir in [Some(current_exe), resolved_exe].into_iter().flatten() {
        if let Some(candidate) = dir.parent().map(|d| d.join(NETD_BIN_NAME)) {
            if tried.contains(&candidate) {
                continue;
            }
            if exists(&candidate) {
                return NetdBinChoice::Found(candidate);
            }
            tried.push(candidate);
        }
    }
    NetdBinChoice::NoneFound { tried }
}

/// Whether `path` is a file this process could actually exec.
///
/// Unix-only, like the rest of this module (the control plane is a UDS).
fn is_executable_file(path: &Path) -> bool {
    use std::os::unix::fs::PermissionsExt as _;
    std::fs::metadata(path).is_ok_and(|m| m.is_file() && m.permissions().mode() & 0o111 != 0)
}

/// The PURE failure text (oracle-tested): every rung that was tried, in order,
/// plus the override that would end the argument.
///
/// It names the paths because an operator told
/// only "No such file or directory" has no reason to suspect the right directory.
fn describe_no_netd_binary(current_exe: &Path, tried: &[PathBuf]) -> String {
    let mut msg = format!("no `{NETD_BIN_NAME}` found for `{}`", current_exe.display());
    if tried.is_empty() {
        // `current_exe` has no parent at all — defensive; a real exec path always
        // has one. Say so rather than printing an empty list.
        msg.push_str(" (it has no parent directory to look in)");
    } else {
        msg.push_str(" — tried, in order:");
        for (i, path) in tried.iter().enumerate() {
            msg.push_str(&format!(" ({}) {}", i + 1, path.display()));
        }
    }
    msg.push_str(&format!(
        ". Install it beside the binary that spawns it, or set {NETD_BIN_ENV} to its path."
    ));
    msg
}

/// Spawn `cerulion-netd` DETACHED (its own session via `setsid`), pointed at
/// `socket`. Best-effort against the singleton race: if another consumer already
/// won the `flock`, this child exits `AddrInUse` immediately (harmless — the
/// caller's retry-connect finds the winner). Only a failure to LAUNCH the process
/// (binary not found) is surfaced.
///
/// Returns the [`Child`] handle so the caller's readiness wait can `try_wait`
/// it — turning "the daemon I just spawned died" into a fast, ATTRIBUTED failure
/// instead of a silent wait to the ceiling. The daemon is still detached and NOT
/// reaped on the healthy path (it outlives this consumer and self-exits on idle); the
/// handle is dropped when the wait ends.
fn spawn_netd_detached(socket: &Path) -> Result<Child, ClientError> {
    let bin = resolve_netd_bin().map_err(|source| ClientError::Spawn {
        bin: PathBuf::from(NETD_BIN_NAME),
        source,
    })?;
    let mut cmd = Command::new(&bin);
    // Point the child at the SAME socket the client connects to (belt-and-suspenders
    // for the explicit-socket entry; on the default path both resolve identically).
    cmd.env(SOCKET_ENV, socket);
    // Detach: no stdin/stdout (a daemon), stderr INHERITED so its startup logs are
    // visible to a debugging user (a broken pipe on stderr is non-fatal for
    // tracing). `setsid` puts it in its OWN session so a Ctrl-C on the consumer's
    // process group never kills the shared daemon.
    cmd.stdin(Stdio::null()).stdout(Stdio::null());
    // SAFETY: `pre_exec` runs in the forked child before exec; `setsid()` is
    // async-signal-safe and touches only the child's own session.
    unsafe {
        cmd.pre_exec(|| {
            if libc::setsid() == -1 {
                return Err(io::Error::last_os_error());
            }
            Ok(())
        });
    }
    // `spawn` (not `output`/`wait`) — the daemon runs independently and is never
    // WAITED ON: it self-exits on idle, long after this consumer is gone. The handle
    // is returned only so the readiness wait can poll `try_wait` and
    // attribute an early death; on the healthy path it is simply dropped.
    match cmd.spawn() {
        Ok(child) => {
            tracing::debug!(
                bin = %bin.display(),
                socket = %socket.display(),
                "cerulion-netd: spawned the shared daemon detached (first-consumer-spawns)"
            );
            Ok(child)
        }
        Err(source) => Err(ClientError::Spawn { bin, source }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A query answer's discovery state is only BELIEVED from a daemon old
    /// enough to actually report one. An older daemon sends no field and serde
    /// defaults it to `Settled` — a positive "discovery ran" assertion it never made —
    /// so believing it silently produces a false "not found".
    /// netd is spawn-once and long-lived, so a CLI upgraded while a running vizd holds
    /// an old daemon alive hits this on every query.
    ///
    /// Hand oracle across the version boundary, BOTH reported values on each side.
    #[test]
    fn a_pre_916_daemons_defaulted_settled_is_never_believed() {
        // AT and ABOVE the boundary: the daemon really reports, so use it verbatim —
        // including `Settled`, or a genuine not-found would be softened into "unknown"
        // forever (the over-correction this arm rules out).
        for v in [
            DISCOVERY_MIN_DAEMON_VERSION,
            DISCOVERY_MIN_DAEMON_VERSION + 1,
            99,
        ] {
            assert_eq!(
                trust_reported_discovery(v, DiscoveryState::Settled),
                DiscoveryState::Settled,
                "v{v} reports its own state — believe it"
            );
            assert_eq!(
                trust_reported_discovery(v, DiscoveryState::NotConverged),
                DiscoveryState::NotConverged
            );
        }
        // BELOW the boundary: whatever arrives (which is always the `Settled` DEFAULT,
        // since the field is absent) must NOT be read as an absence licence.
        for v in [0, 1, 3, DISCOVERY_MIN_DAEMON_VERSION - 1] {
            assert_eq!(
                trust_reported_discovery(v, DiscoveryState::Settled),
                DiscoveryState::NotConverged,
                "v{v} cannot assert that discovery converged — its defaulted Settled is \
                 not evidence, and treating it as one yields a false cold-start \"not found\""
            );
            assert_eq!(
                trust_reported_discovery(v, DiscoveryState::NotConverged),
                DiscoveryState::NotConverged
            );
        }
        // NB the boundary's relationship to `PROTOCOL_VERSION` is guarded at COMPILE
        // TIME (`const _: () = assert!(DISCOVERY_MIN_DAEMON_VERSION <= PROTOCOL_VERSION)`,
        // beside the constant), not here. It is `<=`, never `==`: the
        // constant is a FLOOR frozen at the version that INTRODUCED the field, so the
        // next unrelated protocol bump raises `PROTOCOL_VERSION` alone. An
        // equality would MANDATE a regression — it would fail on that bump and
        // push the reader to raise this constant, which would distrust every genuine
        // v5 daemon (the over-correction the first loop above rules out).
    }

    /// The pure post-spawn readiness-wait bound. Hand oracles over both
    /// independent limbs (the overall ceiling and the post-child-exit grace) plus their
    /// boundaries — a `>=` silently flipped to `>` shows up here, and so does a limb
    /// dropped entirely.
    #[test]
    fn spawn_wait_exhausted_bounds_both_the_ceiling_and_the_post_exit_grace() {
        let ceiling = Duration::from_secs(10);
        let grace = Duration::from_millis(500);
        let ms = Duration::from_millis;

        // Child still ALIVE: only the ceiling bounds the wait — a long-but-under-ceiling
        // wait KEEPS waiting (this is the regression limb: a fixed ~1s budget gives
        // up where a real daemon needs ~1.1s).
        assert!(!spawn_wait_exhausted(ms(0), None, ceiling, grace));
        assert!(!spawn_wait_exhausted(ms(1_000), None, ceiling, grace));
        assert!(!spawn_wait_exhausted(ms(1_200), None, ceiling, grace));
        assert!(!spawn_wait_exhausted(ms(9_999), None, ceiling, grace));
        // …and the ceiling is INCLUSIVE.
        assert!(spawn_wait_exhausted(ceiling, None, ceiling, grace));
        assert!(spawn_wait_exhausted(ms(30_000), None, ceiling, grace));

        // Child EXITED: the grace bounds the wait, well before the ceiling — but not
        // instantly (a `flock` loser's exit still leaves room for the winner's socket).
        assert!(!spawn_wait_exhausted(
            ms(1_200),
            Some(ms(0)),
            ceiling,
            grace
        ));
        assert!(!spawn_wait_exhausted(
            ms(1_200),
            Some(ms(499)),
            ceiling,
            grace
        ));
        assert!(spawn_wait_exhausted(ms(1_200), Some(grace), ceiling, grace));
        assert!(spawn_wait_exhausted(
            ms(1_200),
            Some(ms(900)),
            ceiling,
            grace
        ));

        // The two limbs are independent: an exhausted CEILING still bites even when the
        // child exited moments ago (a child that dies at t=9.99s does not buy 500ms more).
        assert!(spawn_wait_exhausted(
            ms(10_500),
            Some(ms(10)),
            ceiling,
            grace
        ));
    }

    /// An UNOBSERVABLE child (a `try_wait` error — ECHILD under a
    /// SIG_IGN'd SIGCHLD) must not shorten the readiness wait.
    ///
    /// A loop that matches `Ok(Some(status))` and drops the `Err` arm silently leaves
    /// the state "running" — which happens to keep the full ceiling, but ALSO
    /// makes the expiry message assert "the spawned daemon is STILL RUNNING" as fact about
    /// a process whose state it has just failed to read. [`ChildLiveness::Unknown`]
    /// separates the two: no fail-fast (`exited_for() == None`, pinned here) AND no
    /// liveness claim (a distinct expiry limb).
    ///
    /// Mapping the Err arm to `Exited` instead would fail-fast a HEALTHY daemon at the
    /// 500 ms grace — this pin is what catches that.
    #[test]
    fn an_unobservable_child_never_triggers_the_fail_fast_limb() {
        let ceiling = Duration::from_secs(10);
        let grace = Duration::from_millis(500);
        let ms = Duration::from_millis;

        // Unknown and Running agree: neither reports an exit…
        assert!(ChildLiveness::Unknown.exited_for().is_none());
        assert!(ChildLiveness::Running.exited_for().is_none());

        // …so long after the grace would have expired, the wait is still ALIVE and only
        // the full ceiling can end it (the `Some(grace)` control on the same numbers
        // expires immediately — the discriminator).
        assert!(!spawn_wait_exhausted(
            ms(2_000),
            ChildLiveness::Unknown.exited_for(),
            ceiling,
            grace
        ));
        assert!(
            spawn_wait_exhausted(ms(2_000), Some(grace), ceiling, grace),
            "control: an OBSERVED exit at the same elapsed DOES fail fast"
        );
        assert!(spawn_wait_exhausted(
            ms(10_000),
            ChildLiveness::Unknown.exited_for(),
            ceiling,
            grace
        ));
    }

    /// The pure stall-notice decision: due exactly once, at or past the bound,
    /// never before it and never a second time. Hand oracles on both sides of
    /// the boundary (a `>=` flipped to `>` fails the at-bound arm) plus the
    /// once-only bit.
    #[test]
    fn stall_notice_is_due_once_at_or_past_its_bound() {
        let bound = Duration::from_secs(2);
        let ms = Duration::from_millis;

        // Before the bound: never, notified or not.
        assert!(!stall_notice_due(ms(0), bound, false));
        assert!(!stall_notice_due(ms(1_999), bound, false));
        assert!(!stall_notice_due(ms(1_999), bound, true));
        // AT the bound: due (inclusive), and at any later poll.
        assert!(stall_notice_due(bound, bound, false));
        assert!(stall_notice_due(ms(2_001), bound, false));
        assert!(stall_notice_due(ms(9_999), bound, false));
        // Once per wait: after the notice went out, never again.
        assert!(!stall_notice_due(bound, bound, true));
        assert!(!stall_notice_due(ms(9_999), bound, true));
    }

    /// The SHIPPED constants, pinned so a ~1.0 s budget cannot silently come back and so
    /// the two bounds keep their intended relationship (the fail-fast grace must be a
    /// small fraction of the ceiling, or "fail fast" is a lie).
    #[test]
    fn shipped_spawn_wait_constants_clear_a_real_daemon_cold_boot() {
        // A real cold boot measured ~1.1s (transport init + the dead-node
        // sweep). The ceiling must clear that with room for a loaded machine.
        assert!(
            SPAWN_READY_TIMEOUT >= Duration::from_secs(5),
            "the readiness ceiling must clear a real daemon cold boot with margin"
        );
        assert!(
            SPAWN_POST_EXIT_GRACE < SPAWN_READY_TIMEOUT / 4,
            "the post-exit grace must be a small fraction of the ceiling to be 'fast'"
        );
        assert!(
            SPAWN_POLL_INTERVAL <= Duration::from_millis(50),
            "the poll must be tight enough that readiness is noticed promptly"
        );
        // A 40 × 25ms total budget is the exact value that fails against a
        // ~1.1s boot. The ceiling must be strictly beyond it.
        assert!(SPAWN_READY_TIMEOUT > Duration::from_millis(1_000));

        // …and a SANITY CEILING on the ceiling. The pin above is
        // one-sided: it stops the budget shrinking to the value that breaks, but
        // nothing in it stops the budget growing without bound. This is a wait with ZERO
        // user-visible progress beyond one breadcrumb, on INTERACTIVE verbs (`topic
        // echo`, `viz`, `schema info`) — and the single retry means a caller can
        // observe ~2× it (see `SPAWN_READY_TIMEOUT`'s docs). At 30s that worst case is
        // already a minute of apparent hang; anything beyond needs a real progress
        // surface, not a bigger number.
        assert!(
            SPAWN_READY_TIMEOUT <= Duration::from_secs(30),
            "a first-use spawn wait with no progress surface must stay diagnosable — and \
             the first-request retry doubles whatever this is"
        );

        // The stall notice sits BETWEEN a real cold boot and the ceiling: above the
        // measured ~1.1 s boot with margin (a healthy first use must draw no warning,
        // or the quiet verbs learn to ignore it) and well under the ceiling (a notice
        // that arrives with the expiry is no notice at all).
        assert!(
            SPAWN_STALL_NOTICE_AFTER >= Duration::from_millis(1_500),
            "the stall notice must clear a real ~1.1 s cold boot with margin"
        );
        assert!(
            SPAWN_STALL_NOTICE_AFTER <= SPAWN_READY_TIMEOUT / 4,
            "the stall notice must arrive while most of the ceiling is still ahead"
        );
    }

    #[test]
    fn verb_compat_error_gates_per_verb_both_directions() {
        let demand = Request::Demand {
            id: 1,
            robot: "r".into(),
            topic: "/t".into(),
            schema_hash: 0,
        };
        let reg = Request::RegisterEgress {
            id: 1,
            plan: cerulion_core::GatewayPlan {
                egress_policy: cerulion_core::GatewayEgressPolicy::AllowAll,
                announce: vec!["/t".into()],
                ingress: vec![],
            },
            schema_serving: cerulion_core::SchemaServing::default(),
            ix_config_json: None,
        };

        // A v1 daemon serves demand/release/status (the baseline) — no error, either
        // direction (a v2 CLIENT against a v1 daemon is the "pinned old daemon" case).
        assert!(verb_compat_error(1, &demand).is_none());
        // A NEWER daemon serves every older verb (superset) — no error.
        assert!(verb_compat_error(2, &demand).is_none());
        assert!(verb_compat_error(99, &demand).is_none());

        // A v1 daemon is TOO OLD for the v2 egress verb → a precise refusal naming the
        // daemon version + the verb + the upgrade fix.
        let msg = verb_compat_error(1, &reg).expect("v1 daemon refuses the v2 egress verb");
        assert!(msg.contains("v1"), "names the daemon version: {msg}");
        assert!(msg.contains("register_egress"), "names the verb: {msg}");
        assert!(msg.contains("v2"), "names the required version: {msg}");
        assert!(
            msg.contains("Upgrade cerulion-netd"),
            "names the fix: {msg}"
        );
        // A config-LESS register (monolith) is served by any v2+ daemon.
        assert!(verb_compat_error(2, &reg).is_none());
        assert!(verb_compat_error(3, &reg).is_none());

        // A CONFIG-CARRYING register (multi-process) needs a v4 daemon —
        // a v2/v3 daemon would silently ignore the forwarded namespace, so the client
        // refuses LOCALLY (the "never a config silently ignored" gate). A v4+ daemon
        // serves it.
        let reg_cfg = Request::RegisterEgress {
            id: 1,
            plan: cerulion_core::GatewayPlan {
                egress_policy: cerulion_core::GatewayEgressPolicy::AllowAll,
                announce: vec!["/t".into()],
                ingress: vec![],
            },
            schema_serving: cerulion_core::SchemaServing::default(),
            ix_config_json: Some(r#"{"global":{}}"#.to_string()),
        };
        for old in [2u32, 3] {
            let msg = verb_compat_error(old, &reg_cfg).unwrap_or_else(|| {
                panic!("v{old} daemon accepted the config-carrying register but should have refused it")
            });
            assert!(
                msg.contains(&format!("v{old}")),
                "names the daemon version: {msg}"
            );
            assert!(msg.contains("register_egress"), "names the verb: {msg}");
            assert!(msg.contains("v4"), "names the required version: {msg}");
        }
        assert!(
            verb_compat_error(4, &reg_cfg).is_none(),
            "a v4 daemon serves the config-carrying register"
        );
    }

    #[test]
    fn verb_compat_error_gates_the_v3_query_verbs() {
        // The query verbs need a v3 daemon. A v1/v2 daemon is refused
        // LOCALLY (naming the daemon version, the verb, and the upgrade fix) before the
        // request is sent; a v3+ daemon serves them.
        let cat = Request::QueryCatalog { id: 1, robot: None };
        let sch = Request::QuerySchema {
            id: 1,
            robot: Some("go2".into()),
            requested: "pkg/Type".into(),
        };
        for old in [1u32, 2] {
            let msg = verb_compat_error(old, &cat)
                .unwrap_or_else(|| panic!("v{old} daemon refuses query_catalog"));
            assert!(
                msg.contains(&format!("v{old}")),
                "names the daemon version: {msg}"
            );
            assert!(msg.contains("query_catalog"), "names the verb: {msg}");
            assert!(msg.contains("v3"), "names the required version: {msg}");
            assert!(
                verb_compat_error(old, &sch).is_some(),
                "query_schema also gated"
            );
        }
        // A v3 (or newer) daemon serves both.
        assert!(verb_compat_error(3, &cat).is_none());
        assert!(verb_compat_error(3, &sch).is_none());
        assert!(verb_compat_error(99, &sch).is_none());
    }

    /// The NEW-client-vs-OLD-daemon direction of the skew: `subscribe_catalog`
    /// needs a v6 daemon, and every older one is refused LOCALLY — before the line is
    /// sent — with a message naming the daemon version, the verb, and the fix. That
    /// refusal is what lets a consumer degrade to its own refresh path instead of
    /// waiting forever on a push an old daemon can never send.
    ///
    /// netd is spawn-once and long-lived, so this skew is real rather than theoretical:
    /// upgrading the desk while a running vizd holds an OLD daemon alive hits it.
    #[test]
    fn subscribe_catalog_is_gated_to_a_v6_daemon_in_both_directions() {
        let sub = Request::SubscribeCatalog { id: 1 };
        assert_eq!(sub.min_daemon_version(), 6);
        assert_eq!(sub.method_name(), "subscribe_catalog");
        for old in [1u32, 2, 3, 4, 5] {
            let msg = verb_compat_error(old, &sub)
                .unwrap_or_else(|| panic!("a v{old} daemon must refuse subscribe_catalog"));
            assert!(
                msg.contains(&format!("v{old}")),
                "names the daemon version: {msg}"
            );
            assert!(msg.contains("subscribe_catalog"), "names the verb: {msg}");
            assert!(msg.contains("v6"), "names the required version: {msg}");
        }
        // A v6 (or newer) daemon serves it.
        assert!(verb_compat_error(6, &sub).is_none());
        assert!(verb_compat_error(99, &sub).is_none());

        // The OTHER direction needs no gate at all, and this is why: every PRE-928 verb
        // still reports the version it always did, so an old CLIENT talking to a v6
        // daemon sends exactly what it always sent — and, because the push is opt-in,
        // receives exactly what it always received. A regression that raised any of
        // these to 6 would break every pinned-old consumer, so they are pinned here.
        assert_eq!(
            Request::Status { id: 1 }.min_daemon_version(),
            1,
            "the v1 vocabulary must not have moved"
        );
        assert_eq!(
            Request::QueryCatalog { id: 1, robot: None }.min_daemon_version(),
            3,
            "the query verbs must not have moved"
        );
        assert_eq!(
            Request::ReleaseEgress { id: 1 }.min_daemon_version(),
            2,
            "the egress verbs must not have moved"
        );
    }

    /// The CLASSIFIER that keeps a subscribed connection in sync, against a
    /// hand-written table. A push carries `event` and no `id`; a response carries `id`
    /// and no `event`. Getting this wrong in either direction IS the desync the opt-in
    /// design exists to prevent, so both mistakes are pinned as `Unknown` rather than
    /// silently resolved.
    #[test]
    fn control_lines_classify_by_shape_not_by_order() {
        use crate::protocol::CATALOG_CHANGED_EVENT;

        let push = crate::protocol::CatalogChanged {
            event: CATALOG_CHANGED_EVENT.to_string(),
            version: 7,
            robots: vec!["go2".into()],
            robots_added: vec!["go2".into()],
            robots_removed: vec![],
            topics_added: 75,
            topics_removed: 0,
            coalesced: 76,
        };
        match classify_control_line(&push.to_json_line()) {
            ControlLine::Event(e) => {
                assert_eq!(e, push, "a push round-trips through the classifier")
            }
            other => panic!("a push must classify as an event, got {other:?}"),
        }

        // A response.
        let resp = Response::SubscribeCatalog(SubscribeCatalogResponse {
            id: 3,
            version: 7,
            robots: vec!["go2".into()],
            watching: true,
        });
        match classify_control_line(&resp.to_json_line()) {
            ControlLine::Response(text) => {
                assert!(text.contains("\"id\":3"));
                assert!(!text.contains("\"event\""));
            }
            other => panic!("a response must classify as one, got {other:?}"),
        }

        // Neither: unparseable, an object with neither key, and — the one that matters —
        // a line TAGGED as our event but shaped wrong. Guessing on that last one is how
        // a stream desyncs, so it is `Unknown`, never a response.
        for bad in [
            "not json at all",
            "{}",
            "{\"hello\":\"cerulion-netd\"}",
            "{\"event\":\"catalog_changed\"}",
            "{\"event\":\"catalog_changed\",\"version\":\"seven\"}",
        ] {
            assert!(
                matches!(classify_control_line(bad), ControlLine::Unknown(_)),
                "must not be guessed at: {bad}"
            );
        }
        // An unrecognized event NAME is not ours — it falls through to the id check.
        assert!(matches!(
            classify_control_line("{\"event\":\"something_else\",\"id\":4}"),
            ControlLine::Response(_)
        ));
    }

    #[test]
    fn reconnect_adopts_the_fresh_daemon_protocol_version() {
        // A reconnect can land on a different-version daemon (a v2
        // daemon idle-exited and a pinned v1 daemon respawned at the same path), so
        // `reconnect` must adopt the FRESH daemon's protocol — else the per-verb
        // compat gate keeps checking the pre-reconnect version. A listener answers TWO
        // connections: the first with a v2 banner, the second (the reconnect) with v1.
        use std::io::{Read, Write};
        use std::os::unix::net::UnixListener;

        let dir = std::env::temp_dir().join(format!(
            "cer_netd_reconn_{}_{}",
            std::process::id(),
            // a unique suffix so parallel runs never collide on the path
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let sock = dir.join("netd.sock");
        let listener = UnixListener::bind(&sock).expect("bind");
        // Accept two connections; handle EACH on its OWN thread (sends its banner
        // immediately) so the reconnect's handshake never deadlocks waiting for the
        // first connection — which only closes AFTER the reconnect completes — to free
        // a sequential server.
        let accept_thread = std::thread::spawn(move || {
            for n in 0..2u32 {
                if let Ok((mut stream, _)) = listener.accept() {
                    let protocol = if n == 0 { 2 } else { 1 };
                    std::thread::spawn(move || {
                        let banner = Hello {
                            hello: HELLO_MARKER.to_string(),
                            protocol,
                        }
                        .to_json_line();
                        let _ = writeln!(stream, "{banner}");
                        let _ = stream.flush();
                        let mut buf = [0u8; 64];
                        while matches!(stream.read(&mut buf), Ok(n) if n > 0) {}
                    });
                }
            }
        });

        let mut client = NetdClient::connect_or_spawn_at(sock.clone()).expect("connect v2");
        assert_eq!(
            client.daemon_protocol(),
            2,
            "first connect sees the v2 daemon"
        );
        client.reconnect().expect("reconnect to the v1 daemon");
        assert_eq!(
            client.daemon_protocol(),
            1,
            "reconnect adopts the FRESH v1 version (re-gates the egress verbs)"
        );

        drop(client);
        let _ = accept_thread.join();
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The trust downgrade at its CALL SITE, not just as a pure function.
    ///
    /// `trust_reported_discovery` is oracle-tested, but only this test pins that the query
    /// paths actually CALL it — deleting `self.reported_discovery(..)` from
    /// `query_catalog_once` / `query_schema_once` leaves every other test green while
    /// silently producing a false "not found" against a stale daemon (the
    /// exact shape the cold-start grace exists to remove, and the exact shape netd's spawn-once
    /// lifetime makes reachable: a CLI upgraded while a running vizd holds an old
    /// daemon alive).
    ///
    /// A fake daemon answers over a REAL UDS with a `< DISCOVERY_MIN_DAEMON_VERSION`
    /// banner and a query response carrying an EXPLICIT `Settled` — stronger than the
    /// serde default, so the pin cannot be satisfied by "the field was absent". The
    /// client must still report `NotConverged` on BOTH query verbs. The v5 arm is the
    /// anti-tautology control: the same explicit `Settled` from a current daemon must
    /// come through verbatim, or a "fix" that hardcoded `NotConverged` would pass.
    #[test]
    fn a_stale_daemons_settled_is_downgraded_at_the_query_call_sites() {
        use std::io::{BufRead, BufReader, Write};
        use std::os::unix::net::UnixListener;

        // One fake daemon at `protocol`, answering every request line with an
        // explicitly-`Settled` empty query response of the matching kind.
        fn serve(protocol: u32) -> (PathBuf, std::path::PathBuf) {
            let dir = std::env::temp_dir().join(format!(
                "trust_{}_{}_{}",
                protocol,
                std::process::id(),
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_nanos()
            ));
            std::fs::create_dir_all(&dir).unwrap();
            let sock = dir.join("netd.sock");
            let listener = UnixListener::bind(&sock).expect("bind");
            std::thread::spawn(move || {
                let Ok((stream, _)) = listener.accept() else {
                    return;
                };
                let mut w = stream.try_clone().expect("clone");
                let banner = Hello {
                    hello: HELLO_MARKER.to_string(),
                    protocol,
                }
                .to_json_line();
                let _ = writeln!(w, "{banner}");
                let _ = w.flush();
                let reader = BufReader::new(stream);
                for line in reader.lines().map_while(Result::ok) {
                    let v: serde_json::Value = match serde_json::from_str(&line) {
                        Ok(v) => v,
                        Err(_) => continue,
                    };
                    let id = v["id"].as_u64().unwrap_or(0);
                    // EXPLICIT `Settled` — the daemon positively claims discovery ran.
                    let resp = if v["method"] == "query_catalog" {
                        Response::CatalogQuery(crate::protocol::CatalogQueryResponse {
                            id,
                            catalogs: vec![],
                            discovery: DiscoveryState::Settled,
                            plane_unsettled_ms: None,
                        })
                    } else {
                        Response::SchemaQuery(crate::protocol::SchemaQueryResponse {
                            id,
                            replies: vec![],
                            discovery: DiscoveryState::Settled,
                            plane_unsettled_ms: None,
                        })
                    };
                    let _ = writeln!(w, "{}", resp.to_json_line());
                    let _ = w.flush();
                }
            });
            (sock, dir)
        }

        // STALE daemon: below the version that introduced the field.
        let stale = DISCOVERY_MIN_DAEMON_VERSION - 1;
        let (sock, dir) = serve(stale);
        let mut client =
            NetdClient::connect_or_spawn_at(sock).expect("connect to the stale daemon");
        assert_eq!(client.daemon_protocol(), stale);
        assert_eq!(
            client
                .query_catalog_with_discovery(None)
                .expect("the query still WORKS against a stale daemon (trust, not refusal)")
                .discovery,
            DiscoveryState::NotConverged,
            "a v{stale} daemon cannot vouch for its discovery — its Settled must be \
             downgraded AT THE CALL SITE, or the desk resumes claiming absence"
        );
        assert_eq!(
            client
                .query_schema_with_discovery(None, "pkg/Type")
                .expect("schema query runs")
                .discovery,
            DiscoveryState::NotConverged,
            "the schema call site carries the same downgrade"
        );
        drop(client);
        let _ = std::fs::remove_dir_all(&dir);

        // ANTI-TAUTOLOGY: a CURRENT daemon's explicit Settled comes through verbatim
        // (a hardcoded NotConverged would fail here, and would soften every genuine
        // not-found into "unknown" forever).
        let (sock, dir) = serve(DISCOVERY_MIN_DAEMON_VERSION);
        let mut client =
            NetdClient::connect_or_spawn_at(sock).expect("connect to the current daemon");
        assert_eq!(
            client
                .query_catalog_with_discovery(None)
                .expect("catalog query runs")
                .discovery,
            DiscoveryState::Settled,
            "a daemon old enough to report really is believed"
        );
        assert_eq!(
            client
                .query_schema_with_discovery(None, "pkg/Type")
                .expect("schema query runs")
                .discovery,
            DiscoveryState::Settled
        );
        drop(client);
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// **The skew matrix**, and the half that matters is that a too-old
    /// daemon is refused BEFORE the request is sent.
    ///
    /// The bump gates only the new verb, so both directions must hold on ONE daemon:
    ///
    /// - **new client + v6 daemon** — `query_runs` is refused locally, LOUDLY, naming
    ///   both versions; and `query_catalog` on that SAME connection still works, which
    ///   is the whole point of a per-verb gate rather than a connect floor.
    /// - **new client + v7 daemon** — the verb is sent and served.
    ///
    /// The ORACLE for "before send" is the fake daemon's own SERVED-REQUEST COUNT, not
    /// the `Err` — a client that sent the request and mapped the daemon's
    /// unknown-method error to a `Protocol` error would produce an indistinguishable
    /// `Err` while burning a round trip and, worse, giving the consumer a message it
    /// cannot tell from "the verb exists and the serve failed".
    #[test]
    fn the_runs_verb_is_refused_before_send_against_a_daemon_that_cannot_serve_it() {
        use std::io::{BufRead, BufReader, Write};
        use std::os::unix::net::UnixListener;
        use std::sync::atomic::{AtomicUsize, Ordering};
        use std::sync::Arc;

        /// A fake daemon at `protocol` that COUNTS the request lines it reads and
        /// answers `query_catalog` / `query_runs` with empty results.
        ///
        /// # It is a DI double, and every line it writes is a line a real daemon
        /// of that version writes
        ///
        /// A scripted `UnixListener` is this repo's established stand-in for a
        /// daemon a test cannot spawn (`client_e2e_test`'s `write_fake_daemon`,
        /// `convergence_wait_e2e_test`, `viz_attach_convergence_test`'s `FakeVizd`),
        /// and here it is not merely convenient but STRUCTURALLY required: the arm
        /// under test is a version SKEW, and a v6 daemon cannot be built from this
        /// tree — `PROTOCOL_VERSION` is a constant, so the only v6 binary is a
        /// historical one.
        ///
        /// What keeps it faithful is that nothing is hand-written onto the wire:
        /// every response is minted through the PRODUCTION `Response` serializer
        /// from the production structs, so a shape this daemon cannot express is a
        /// shape it cannot emit. The one place that could have drifted is a request
        /// whose verb POSTDATES the fake's own version — answered here exactly as a
        /// real daemon of that version answers it, with the structured
        /// unknown-method error `parse_request` produces (a v6 daemon has no
        /// `QueryRuns` variant, so its serde decode fails). That branch is
        /// UNREACHABLE while the per-verb gate holds, which is the point: under the
        /// gate's absence, the client meets the answer it would
        /// really have met.
        fn serve(protocol: u32) -> (PathBuf, std::path::PathBuf, Arc<AtomicUsize>) {
            let dir = std::env::temp_dir().join(format!(
                "skew_{}_{}_{}",
                protocol,
                std::process::id(),
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_nanos()
            ));
            std::fs::create_dir_all(&dir).unwrap();
            let sock = dir.join("netd.sock");
            let listener = UnixListener::bind(&sock).expect("bind");
            let served = Arc::new(AtomicUsize::new(0));
            let counter = Arc::clone(&served);
            std::thread::spawn(move || {
                let Ok((stream, _)) = listener.accept() else {
                    return;
                };
                let mut w = stream.try_clone().expect("clone");
                let banner = Hello {
                    hello: HELLO_MARKER.to_string(),
                    protocol,
                }
                .to_json_line();
                let _ = writeln!(w, "{banner}");
                let _ = w.flush();
                let reader = BufReader::new(stream);
                for line in reader.lines().map_while(Result::ok) {
                    counter.fetch_add(1, Ordering::SeqCst);
                    let v: serde_json::Value = match serde_json::from_str(&line) {
                        Ok(v) => v,
                        Err(_) => continue,
                    };
                    let id = v["id"].as_u64().unwrap_or(0);
                    let resp = if v["method"] == "query_runs" {
                        if protocol < RUNS_MIN_DAEMON_VERSION {
                            // A daemon of this version has no `QueryRuns` variant, so
                            // its `parse_request` serde decode fails and it answers the
                            // structured unknown-method error. Reproduced rather than
                            // approximated, so a removed gate meets the answer
                            // a real v6 daemon would have given it.
                            Response::error(
                                Some(id),
                                "malformed request: unknown variant `query_runs`",
                                None,
                                None,
                            )
                        } else {
                            Response::RunsQuery(crate::protocol::RunsQueryResponse {
                                id,
                                run_replies: vec![],
                                unusable: Vec::new(),
                                silent: Vec::new(),
                                discovery: DiscoveryState::Settled,
                                plane_unsettled_ms: None,
                            })
                        }
                    } else {
                        Response::CatalogQuery(crate::protocol::CatalogQueryResponse {
                            id,
                            catalogs: vec![],
                            discovery: DiscoveryState::Settled,
                            plane_unsettled_ms: None,
                        })
                    };
                    let _ = writeln!(w, "{}", resp.to_json_line());
                    let _ = w.flush();
                }
            });
            (sock, dir, served)
        }

        // ---- NEW client, OLD daemon: refused locally, nothing sent. ----
        let old = RUNS_MIN_DAEMON_VERSION - 1;
        let (sock, dir, served) = serve(old);
        let mut client = NetdClient::connect_or_spawn_at(sock).expect("connect to the old daemon");
        assert_eq!(client.daemon_protocol(), old);

        let err = client
            .query_runs(None)
            .expect_err("a v{old} daemon cannot serve the runs verb");
        match &err {
            ClientError::Protocol(msg) => {
                assert!(
                    msg.contains("query_runs"),
                    "the refusal must name the verb: {msg}"
                );
                assert!(
                    msg.contains(&format!("v{old}"))
                        && msg.contains(&format!("v{RUNS_MIN_DAEMON_VERSION}")),
                    "the refusal must name BOTH versions so the operator knows what to \
                     upgrade: {msg}"
                );
            }
            other => panic!("expected a Protocol refusal, got {other:?}"),
        }
        assert_eq!(
            served.load(Ordering::SeqCst),
            0,
            "the gate must refuse BEFORE the send — a request that reached the daemon \
             would come back as a generic unknown-method error the consumer cannot \
             distinguish from a serve failure"
        );

        // …and the SAME connection still serves the verbs that daemon does speak.
        // Without this, "the bump gates only the new verb" is unproven and a connect
        // floor would pass the assertion above.
        assert!(
            client.query_catalog(None).is_ok(),
            "a v{old} daemon must keep serving its own vocabulary"
        );
        assert_eq!(
            served.load(Ordering::SeqCst),
            1,
            "exactly the catalog request crossed the wire"
        );
        drop(client);
        let _ = std::fs::remove_dir_all(&dir);

        // ---- ANTI-TAUTOLOGY: a v7 daemon is asked, and answers. ----
        let (sock, dir, served) = serve(RUNS_MIN_DAEMON_VERSION);
        let mut client = NetdClient::connect_or_spawn_at(sock).expect("connect to the new daemon");
        let gather = client
            .query_runs_with_discovery(None)
            .expect("a v7 daemon serves the runs verb");
        assert!(gather.replies.is_empty());
        assert_eq!(
            gather.discovery,
            DiscoveryState::Settled,
            "a v7 daemon is at or above DISCOVERY_MIN_DAEMON_VERSION, so its reported \
             state is believed with no downgrade"
        );
        assert_eq!(
            served.load(Ordering::SeqCst),
            1,
            "the request really was sent"
        );
        drop(client);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn client_baseline_min_version_is_the_v1_vocabulary() {
        // The handshake floor is the v1 demand/release/status vocabulary this consumer
        // is built on (a drift guard): a v0 daemon is refused, every v>=1 daemon
        // accepted. The accept/refuse behavior itself is pinned by the crafted-banner
        // e2e in `client_e2e_test.rs`.
        assert_eq!(CLIENT_MIN_DAEMON_VERSION, 1);
    }

    #[test]
    fn is_not_running_classifies_missing_and_refused_as_not_running() {
        // A missing socket file / a stale socket left by a crash both mean "spawn".
        assert!(is_not_running(&io::Error::from(io::ErrorKind::NotFound)));
        assert!(is_not_running(&io::Error::from(
            io::ErrorKind::ConnectionRefused
        )));
        // A real error (permissions, …) is NOT "not running" — surface it.
        assert!(!is_not_running(&io::Error::from(
            io::ErrorKind::PermissionDenied
        )));
        assert!(!is_not_running(&io::Error::from(io::ErrorKind::TimedOut)));
    }

    #[test]
    fn is_closed_early_matches_only_the_accepted_then_closed_signature() {
        // The wedge/race signature — accepted then closed at/before the first
        // exchange — is retried once. EOF is the dominant one (read of a closed fd);
        // BrokenPipe/ConnectionReset are the write-side twins.
        assert!(is_closed_early(&ClientError::Io(io::Error::from(
            io::ErrorKind::UnexpectedEof
        ))));
        assert!(is_closed_early(&ClientError::Io(io::Error::from(
            io::ErrorKind::BrokenPipe
        ))));
        assert!(is_closed_early(&ClientError::Io(io::Error::from(
            io::ErrorKind::ConnectionReset
        ))));
        // The SAME close, reported at a different syscall. On macOS a peer
        // close that lands BEFORE `finish_handshake`'s `set_read_timeout` makes xnu
        // refuse the `setsockopt` with EINVAL, so the handshake never reaches the
        // read that would have produced `UnexpectedEof`. Unclassified, the
        // single retry does not fire on that side of the race. The OS behaviour this
        // rests on is not assumed — it is reproduced by
        // `a_daemon_refusal_that_lands_before_the_handshake_is_still_retryable`.
        assert!(is_closed_early(&ClientError::Io(io::Error::from(
            io::ErrorKind::InvalidInput
        ))));
        // macOS reports a write/read on a peer-closed UDS as ENOTCONN
        // (`NotConnected`) where Linux gives EPIPE/ECONNRESET/EOF — observed on
        // the macOS CI shard. The same close, one more syscall spelling.
        assert!(is_closed_early(&ClientError::Io(io::Error::from(
            io::ErrorKind::NotConnected
        ))));
        // A different I/O error (e.g. a timeout — a wedged-but-not-closed daemon) is
        // NOT the retry signature; surface it.
        assert!(!is_closed_early(&ClientError::Io(io::Error::from(
            io::ErrorKind::TimedOut
        ))));
        // Non-I/O errors are never the race signature — a protocol/daemon/connect
        // error is a real failure the caller must see, not a transient close.
        assert!(!is_closed_early(&ClientError::Protocol(
            "bad banner".into()
        )));
        assert!(!is_closed_early(&ClientError::Netd {
            error: "schema conflict".into(),
            robot: None,
            topic: None,
        }));
        assert!(!is_closed_early(&ClientError::Connect {
            socket: PathBuf::from("/x/netd.sock"),
            attempt: ConnectAttempt::ConnectOrSpawn,
            source: io::Error::from(io::ErrorKind::NotFound),
        }));
    }

    /// A connect failure describes the path it actually took.
    ///
    /// The message asserted a spawn UNCONDITIONALLY — "spawned it but it did not
    /// come up in time" — while TWO of the three sites that build this error
    /// never spawn anything. The one an operator meets most is the recorder's:
    /// `cerulion bagd` connects with `connect_existing_at` precisely so it does
    /// NOT start a daemon on the machine it is recording, and then logged a line
    /// claiming it had tried to.
    ///
    /// Pinned NEGATIVELY as well as positively: asserting only that the
    /// no-spawn arm names itself would pass a Display that ALSO kept the false
    /// spawn clause, which is exactly what this guards against.
    #[test]
    fn a_connect_failure_never_claims_a_spawn_the_path_could_not_have_made() {
        let socket = PathBuf::from("/x/netd.sock");
        let existing = ClientError::Connect {
            socket: socket.clone(),
            attempt: ConnectAttempt::ExistingOnly,
            source: io::Error::from(io::ErrorKind::NotFound),
        }
        .to_string();
        assert!(
            existing.contains("never spawns one"),
            "the no-spawn path must say so: {existing}"
        );
        assert!(
            !existing.contains("spawned it"),
            "and must NOT claim a spawn it structurally cannot make: {existing}"
        );

        // The spawn-capable path keeps its own wording, and the spawn/ceiling
        // detail rides its `source` (built at the readiness-wait site) rather
        // than being asserted by the wrapper on every exit — the fast-path arm
        // reaches this same variant WITHOUT spawning.
        let or_spawn = ClientError::Connect {
            socket: socket.clone(),
            attempt: ConnectAttempt::ConnectOrSpawn,
            source: io::Error::from(io::ErrorKind::PermissionDenied),
        }
        .to_string();
        assert!(
            !or_spawn.contains("never spawns one"),
            "the connect-or-spawn path must not borrow the no-spawn sentence: {or_spawn}"
        );
        for msg in [&existing, &or_spawn] {
            assert!(
                msg.contains("/x/netd.sock"),
                "every arm still names the socket it tried: {msg}"
            );
        }
    }

    /// The no-spawn arm reports what it REACHED, never a
    /// cause it did not establish.
    ///
    /// The variant is built at ONE site — `connect_existing_at` — which wraps
    /// whatever `UnixStream::connect` returned WITHOUT inspecting its kind, and
    /// the message asserted "no cerulion-netd daemon was running at …" for all
    /// of them. `PermissionDenied` is the OPPOSITE state — a daemon IS listening
    /// and EACCES keeps us out — so the one line an operator sees (the recorder
    /// logs this Display as its resolver `reason`) sent them to start a daemon
    /// that was already up. `Interrupted` and the path-shape errors establish
    /// nothing about a daemon either.
    ///
    /// Driven over the kinds this path can carry, INCLUDING the two where the
    /// old claim held (`NotFound`/`ConnectionRefused` — `is_not_running`'s own
    /// classification): the wrapper does not branch, so the accurate wording must
    /// be the one wording, and pinning only the false-claim kinds would pass an
    /// implementation that branched back to the overstatement for the rest.
    ///
    /// Pinned NEGATIVELY (the retired claim is absent) as well as positively
    /// (the reached-vs-running distinction and the `source` are both carried) —
    /// asserting only the new sentence would pass a Display that appended it to
    /// the old one.
    ///
    /// The negative half is a VOCABULARY, not one substring (the
    /// cancelled-wait precedent): a single `!contains("daemon was running")`
    /// is satisfied by any re-tensing of the same false claim — "no daemon
    /// **is** running", "the daemon was **not** running" — so the assertion
    /// would silently stop pinning anything the next time that line is edited.
    #[test]
    fn the_no_spawn_arm_reports_what_it_reached_not_a_cause_it_did_not_establish() {
        let socket = PathBuf::from("/x/netd.sock");
        for kind in [
            io::ErrorKind::NotFound,
            io::ErrorKind::ConnectionRefused,
            io::ErrorKind::PermissionDenied,
            io::ErrorKind::Interrupted,
            io::ErrorKind::NotADirectory,
            io::ErrorKind::InvalidInput,
        ] {
            let source = io::Error::from(kind);
            let source_text = source.to_string();
            let msg = ClientError::Connect {
                socket: socket.clone(),
                attempt: ConnectAttempt::ExistingOnly,
                source,
            }
            .to_string();

            for forbidden in [
                "daemon was running",
                "daemon is running",
                "daemon was not running",
                "daemon is not running",
                "no cerulion-netd daemon",
            ] {
                assert!(
                    !msg.contains(forbidden),
                    "{kind:?}: the message must not assert a cause the code never \
                     inspected the kind to establish (found {forbidden:?}): {msg}"
                );
            }
            assert!(
                msg.contains("could not reach a running cerulion-netd daemon"),
                "{kind:?}: it must say what it DID establish — nothing was \
                 reached: {msg}"
            );
            assert!(
                msg.contains("never spawns one"),
                "{kind:?}: the no-spawn path must still say so: {msg}"
            );
            assert!(
                msg.contains(&source_text),
                "{kind:?}: the OS error is the only thing that names the real \
                 condition, so it must ride the line verbatim: {msg}"
            );
        }
    }

    /// The composed pin — the daemon's OWN refusal shape, driven through
    /// the REAL [`NetdClient::finish_handshake`], must classify as retryable.
    ///
    /// The pure oracle above asserts what the predicate does with a hand-built
    /// `ErrorKind`; this asserts what the OS actually HANDS it, so the classification cannot
    /// rest on a claim about xnu that a later toolchain quietly falsifies. The peer
    /// models `daemon::handle_connection`'s post-commit arm exactly — accept, then
    /// `return`, i.e. DROP the stream with no `Hello` — and the ordering is FORCED
    /// (the client waits on a channel until that drop has happened) so the test
    /// measures the OS, not a 1-in-20_000 race.
    ///
    /// It pins the PROPERTY rather than the errno, which is what makes it portable:
    /// on macOS the forced ordering yields `InvalidInput` (MEASURED 200/200) and on
    /// Linux the timeouts install and the read yields `UnexpectedEof`. Both are the
    /// same event and both must be retried. SCOPE: on Linux this arm passes
    /// with or without the widening, so it is a macOS-effective pin — which
    /// is correct, because the hole it closes is macOS-only.
    #[test]
    fn a_daemon_refusal_that_lands_before_the_handshake_is_still_retryable() {
        use std::os::unix::net::UnixListener;

        let dir = std::env::temp_dir().join(format!("refusal_{}", std::process::id()));
        let sock = dir.join("netd.sock");
        // The `sockaddr_un.sun_path` ceiling at the STRICTEST platform: macOS gives
        // 104 bytes INCLUDING the NUL (Linux 108). This guard is not decoration —
        // `client_e2e_test`'s sibling harness added the same one after an over-long
        // `$TMPDIR` made `bind` fail as a bare `Invalid argument (os error 22)`, which
        // is VERBATIM the error class this test exists to make legible. Without it a
        // long-TMPDIR host would fail here with the very errno under test and nothing
        // naming the path as the cause.
        //
        // It runs BEFORE `create_dir_all`: the path is a pure join and needs
        // no directory, and asserting after the mkdir would leave a stray
        // `refusal_<pid>/` behind on every run — the end-of-test cleanup is
        // never reached on the assert path — on precisely the long-`$TMPDIR` host the
        // guard exists for.
        const SUN_PATH_MAX: usize = 103;
        let len = sock.as_os_str().len();
        assert!(
            len <= SUN_PATH_MAX,
            "test socket path is {len} bytes, over the {SUN_PATH_MAX}-byte sockaddr_un \
             limit ({}) — a longer path fails as a bare EINVAL at bind/connect, not as \
             anything readable, which would be indistinguishable from this test's subject",
            sock.display()
        );
        std::fs::create_dir_all(&dir).expect("mk tempdir");
        let _ = std::fs::remove_file(&sock);

        let listener = UnixListener::bind(&sock).expect("bind");
        let (tx_closed, rx_closed) = std::sync::mpsc::channel::<()>();
        let peer = std::thread::spawn(move || {
            let (stream, _) = listener.accept().expect("accept");
            // EXACTLY the daemon's refusal: drop WITHOUT writing a Hello.
            drop(stream);
            // Signal only AFTER the close, so the client loses the race by design.
            let _ = tx_closed.send(());
        });

        let stream = try_connect(&sock).expect("connect to the listening peer");
        rx_closed
            .recv_timeout(Duration::from_secs(5))
            .expect("the peer reported its refusal");

        // `expect_err` would need `NetdClient: Debug`; match instead.
        let err = match NetdClient::finish_handshake(stream, sock.clone()) {
            Ok(_) => panic!("a refused connection cannot complete the handshake"),
            Err(e) => e,
        };
        assert!(
            is_closed_early(&err),
            "a daemon refusal that lands before the handshake must be RETRIED, not \
             surfaced — the accept-then-close design is safe only because \
             this predicate covers every shape of it. Got: {err:?}"
        );

        let _ = peer.join();
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn is_closed_early_still_refuses_non_race_errors() {
        // Split out so the widening above cannot be satisfied by a predicate that
        // simply says yes to every `Io`.
        assert!(!is_closed_early(&ClientError::Io(io::Error::from(
            io::ErrorKind::PermissionDenied
        ))));
        assert!(!is_closed_early(&ClientError::Io(io::Error::from(
            io::ErrorKind::NotFound
        ))));
        assert!(!is_closed_early(&ClientError::Connect {
            socket: PathBuf::from("/x/netd.sock"),
            attempt: ConnectAttempt::ConnectOrSpawn,
            source: io::Error::from(io::ErrorKind::NotFound),
        }));
    }

    /// An existence predicate over a HAND-WRITTEN set of paths — the injected
    /// half that makes the ladder pure. Nothing touches the filesystem.
    fn only<'a>(present: &'a [&'a str]) -> impl Fn(&Path) -> bool + 'a {
        move |p: &Path| present.iter().any(|q| Path::new(q) == p)
    }

    #[test]
    fn resolve_netd_bin_prefers_env_override_then_sibling() {
        let nothing = only(&[]);
        // The env override wins verbatim — and, deliberately, WITHOUT an
        // existence check: an operator who names a path is told about THAT path.
        assert_eq!(
            resolve_netd_bin_from(
                Some("/opt/cerulion/cerulion-netd"),
                Path::new("/usr/bin/cerulion"),
                None,
                &nothing,
            ),
            NetdBinChoice::Found(PathBuf::from("/opt/cerulion/cerulion-netd"))
        );
        // No override → cerulion-netd next to the current exe.
        let sibling = only(&["/usr/local/bin/cerulion-netd"]);
        assert_eq!(
            resolve_netd_bin_from(None, Path::new("/usr/local/bin/cerulion"), None, &sibling),
            NetdBinChoice::Found(PathBuf::from("/usr/local/bin/cerulion-netd"))
        );
        // vizd's bin dir resolves the SAME sibling.
        assert_eq!(
            resolve_netd_bin_from(
                None,
                Path::new("/usr/local/bin/cerulion-vizd"),
                None,
                &sibling
            ),
            NetdBinChoice::Found(PathBuf::from("/usr/local/bin/cerulion-netd"))
        );
        // A genuinely parentless exe path with no override finds nothing AND has
        // nothing to report (`Path::new("cerulion").parent()` is `Some("")`, so
        // the empty-ladder arm needs a root path whose parent is truly `None`).
        assert_eq!(
            resolve_netd_bin_from(None, Path::new("/"), None, &nothing),
            NetdBinChoice::NoneFound { tried: vec![] }
        );
    }

    /// **The oracle** — a symlinked desk layout, reproduced exactly.
    ///
    /// Studio's deploy symlinks `cerulion-vizd` beside the shell; macOS reports
    /// the SYMLINK as `current_exe()` (measured), so rung 2 lands in the shell's
    /// directory, which has no daemon. The daemon is in the checkout the symlink
    /// points into — rung 3's directory. Without rung 3
    /// every spawn fails `No such file or directory`, leaving the sidebar empty.
    ///
    /// Deleting `resolved_exe` from the rung list makes this
    /// fail with `NoneFound`.
    #[test]
    fn the_resolved_binarys_directory_is_a_rung_so_a_symlinked_vizd_finds_the_daemon() {
        let shell_dir_vizd = Path::new("/w/studio-shell/target/release/cerulion-vizd");
        let base_dir_vizd = Path::new("/w/cerulion/target/release/cerulion-vizd");
        // ONLY the base checkout has a daemon — the shell's release dir does not.
        let present = only(&["/w/cerulion/target/release/cerulion-netd"]);

        assert_eq!(
            resolve_netd_bin_from(None, shell_dir_vizd, Some(base_dir_vizd), &present),
            NetdBinChoice::Found(PathBuf::from("/w/cerulion/target/release/cerulion-netd")),
            "a vizd reached through a symlink must find the daemon beside its RESOLVED path"
        );

        // ANTI-TAUTOLOGY: rung 2 still WINS when it holds a daemon, so rung 3 is
        // a fallback and not a silent redirection of every spawn.
        let both = only(&[
            "/w/studio-shell/target/release/cerulion-netd",
            "/w/cerulion/target/release/cerulion-netd",
        ]);
        assert_eq!(
            resolve_netd_bin_from(None, shell_dir_vizd, Some(base_dir_vizd), &both),
            NetdBinChoice::Found(PathBuf::from(
                "/w/studio-shell/target/release/cerulion-netd"
            )),
            "rung 2 outranks rung 3 whenever it holds a daemon"
        );

        // And with NEITHER present, the failure names BOTH directories — the
        // whole point of carrying `tried` (an error naming one directory
        // gives nobody reason to suspect the other).
        let none = only(&[]);
        assert_eq!(
            resolve_netd_bin_from(None, shell_dir_vizd, Some(base_dir_vizd), &none),
            NetdBinChoice::NoneFound {
                tried: vec![
                    PathBuf::from("/w/studio-shell/target/release/cerulion-netd"),
                    PathBuf::from("/w/cerulion/target/release/cerulion-netd"),
                ]
            }
        );
    }

    /// Rung 3 DEDUPES against rung 2 — the Linux shape, where `/proc/self/exe` is
    /// already symlink-resolved so both rungs name one directory. A duplicate
    /// would make the failure text report the same path twice as if two places
    /// had been searched.
    #[test]
    fn an_already_resolved_exe_contributes_no_duplicate_rung() {
        let exe = Path::new("/opt/cer/bin/cerulion-vizd");
        assert_eq!(
            resolve_netd_bin_from(None, exe, Some(exe), &only(&[])),
            NetdBinChoice::NoneFound {
                tried: vec![PathBuf::from("/opt/cer/bin/cerulion-netd")]
            }
        );
    }

    /// A PRESENT but non-executable file is not a daemon: the ladder keeps going
    /// rather than stopping at a rung that can only ever produce `EACCES`.
    #[test]
    fn a_non_executable_candidate_does_not_end_the_ladder() {
        // The predicate models "executable file"; the decoy at rung 2 is absent
        // from it precisely because it is not executable.
        let exe = Path::new("/a/bin/cerulion-vizd");
        let resolved = Path::new("/b/bin/cerulion-vizd");
        assert_eq!(
            resolve_netd_bin_from(None, exe, Some(resolved), &only(&["/b/bin/cerulion-netd"])),
            NetdBinChoice::Found(PathBuf::from("/b/bin/cerulion-netd"))
        );
    }

    /// The ONE seam between the pure ladder and the real filesystem: what
    /// production passes as `exists`. Every ladder oracle above injects its own
    /// predicate, so without this arm the shipped predicate is pinned by nothing
    /// and could be `Path::exists` — which is exactly the "stops at a rung that
    /// can only produce EACCES" failure the ladder is written to avoid.
    ///
    /// Replacing `is_executable_file` with `Path::exists`
    /// fails exactly this test and nothing else.
    #[test]
    fn the_production_existence_predicate_requires_an_executable_file() {
        use std::os::unix::fs::PermissionsExt as _;
        let dir = std::env::temp_dir().join(format!(
            "execpred-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .expect("a clock after 1970")
                .as_nanos()
        ));
        std::fs::create_dir_all(&dir).expect("create the probe dir");

        let plain = dir.join("plain");
        std::fs::write(&plain, b"not a program").expect("write the plain file");
        assert!(
            !is_executable_file(&plain),
            "a present but non-executable file is not a daemon"
        );

        let exec = dir.join("exec");
        std::fs::write(&exec, b"#!/bin/sh\n").expect("write the executable");
        std::fs::set_permissions(&exec, std::fs::Permissions::from_mode(0o755))
            .expect("chmod the executable");
        assert!(
            is_executable_file(&exec),
            "an executable file IS a candidate (anti-tautology: the predicate is \
             not simply false)"
        );

        // A DIRECTORY carries the exec bit on every Unix; it is not a binary.
        assert!(
            !is_executable_file(&dir),
            "a directory is executable-by-mode and must still be refused"
        );
        assert!(
            !is_executable_file(&dir.join("absent")),
            "an absent path is not a candidate"
        );

        let _ = std::fs::remove_dir_all(&dir);
    }

    /// **THE SEAM PIN** — production really does canonicalise, over a REAL
    /// symlink on a REAL filesystem.
    ///
    /// Every oracle above INJECTS `resolved_exe`, so they pin the ladder's RULES
    /// and say nothing about whether the shell supplies rung 3's input. That gap
    /// is real: replacing `std::fs::canonicalize(..)` with
    /// `None` leaves a symlinked vizd
    /// resolving no daemon (every spawn fails ENOENT, the Studio sidebar stays empty)
    /// — and no other `cerulion_netd` test notices.
    ///
    /// The layout is a symlinked deploy: a binary reached through a symlink, the
    /// daemon beside its real self, and nothing beside the link.
    ///
    /// Neutralising the canonicalize fails exactly this test
    /// (every other test still passes, which is the point).
    #[test]
    fn the_io_shell_really_canonicalises_so_a_symlinked_exe_reaches_rung_three() {
        use std::os::unix::fs::PermissionsExt as _;

        let root = std::env::temp_dir().join(format!(
            "seam-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .expect("a clock after 1970")
                .as_nanos()
        ));
        let real_dir = root.join("cerulion/target/release");
        let link_dir = root.join("studio-shell/target/release");
        std::fs::create_dir_all(&real_dir).expect("create the real dir");
        std::fs::create_dir_all(&link_dir).expect("create the link dir");

        // The REAL binary, and the daemon beside it.
        let real_vizd = real_dir.join("cerulion-vizd");
        let real_netd = real_dir.join(NETD_BIN_NAME);
        for p in [&real_vizd, &real_netd] {
            std::fs::write(p, b"#!/bin/sh\n").expect("write a stand-in binary");
            std::fs::set_permissions(p, std::fs::Permissions::from_mode(0o755)).expect("chmod");
        }
        // The deploy's symlink — and NOTHING else in that directory.
        let linked_vizd = link_dir.join("cerulion-vizd");
        std::os::unix::fs::symlink(&real_vizd, &linked_vizd).expect("symlink");
        assert!(
            !link_dir.join(NETD_BIN_NAME).exists(),
            "precondition: the symlink's directory holds no daemon (the symlinked-deploy layout)"
        );

        // The CRATE-WIDE env guard, not a file-local one and not
        // `#[serial_test::serial]` — see `crate::test_env`'s docs. That no sibling
        // ladder test reads this variable is no reason to go without it;
        // that is VAR-NAME reasoning, which
        // `lib.rs` rejects in writing: concurrent `setenv`/`getenv` from different
        // threads is a data race on the environ block itself whatever keys are
        // touched, and `wan.rs` mutates 30 vars in this same lib-test binary.
        // `resolve_netd_bin_for` also READS the env on the path under test, so the
        // rule applies twice over.
        let _env = crate::test_env::env_lock();

        // No override, so the ladder is the only thing under test.
        let restore = std::env::var(NETD_BIN_ENV).ok();
        // SAFETY: the crate-wide guard above serializes this against every other
        // env-touching lib test; restored immediately below.
        unsafe { std::env::remove_var(NETD_BIN_ENV) };
        let resolved = resolve_netd_bin_for(&linked_vizd);
        if let Some(v) = restore {
            unsafe { std::env::set_var(NETD_BIN_ENV, v) };
        }

        // The EXPECTED path is canonicalised too, because the difference is
        // legitimate: macOS's `/var` is itself a symlink to `/private/var`, so a
        // resolved path under `TMPDIR` differs from the literal one by that
        // prefix. (That difference is itself evidence the canonicalize ran.)
        // Without the canonicalize fix, this
        // line is never reached: resolution returns `NoneFound` and the `expect`
        // below fails first.
        let expected = std::fs::canonicalize(&real_netd).expect("canonicalise the expected path");
        assert_eq!(
            resolved.expect("the daemon beside the RESOLVED binary must be found"),
            expected,
            "a binary reached through a symlink must still find the daemon beside \
             its real self — this is the whole of the symlink fix"
        );

        let _ = std::fs::remove_dir_all(&root);
    }

    /// The failure text is the operator's ONLY window (the spawn error is what
    /// vizd's flood latch suppresses), so it must name every rung, in order, and
    /// the override that ends the argument.
    #[test]
    fn the_no_binary_message_names_every_rung_it_tried() {
        let msg = describe_no_netd_binary(
            Path::new("/w/studio-shell/target/release/cerulion-vizd"),
            &[
                PathBuf::from("/w/studio-shell/target/release/cerulion-netd"),
                PathBuf::from("/w/cerulion/target/release/cerulion-netd"),
            ],
        );
        assert!(
            msg.contains("(1) /w/studio-shell/target/release/cerulion-netd"),
            "rung 1 must be numbered and named: {msg}"
        );
        assert!(
            msg.contains("(2) /w/cerulion/target/release/cerulion-netd"),
            "rung 2 must be numbered and named: {msg}"
        );
        assert!(
            msg.contains(NETD_BIN_ENV),
            "the override that ends the argument must be named: {msg}"
        );
        assert!(
            msg.contains("/w/studio-shell/target/release/cerulion-vizd"),
            "the binary doing the spawning must be named: {msg}"
        );

        // The parentless arm says WHY the list is empty instead of printing
        // "tried, in order:" followed by nothing.
        let empty = describe_no_netd_binary(Path::new("/"), &[]);
        assert!(
            empty.contains("no parent directory") && !empty.contains("tried, in order"),
            "an empty ladder must explain itself: {empty}"
        );
    }

    #[test]
    fn client_error_display_is_loud_and_actionable() {
        let e = ClientError::Netd {
            error: "schema conflict".to_string(),
            robot: Some("ubuntu".to_string()),
            topic: Some("/tf".to_string()),
        };
        let s = e.to_string();
        assert!(
            s.contains("ubuntu") && s.contains("/tf") && s.contains("schema conflict"),
            "{s}"
        );

        let e = ClientError::Spawn {
            bin: PathBuf::from("/x/cerulion-netd"),
            source: io::Error::from(io::ErrorKind::NotFound),
        };
        assert!(
            e.to_string().contains(NETD_BIN_ENV),
            "names the override env: {e}"
        );
    }
}
