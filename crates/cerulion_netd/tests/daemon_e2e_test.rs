// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion-netd` daemon e2e over the REAL UDS control
//! server — driven in-process with a COUNTING SPY mirror plane (no zenoh, no
//! iceoryx2), so the refcount → mirror wiring, the crash-safe connection-close
//! release, and the idle self-exit are all exercised end to end.
//!
//! The spy is a legitimate dependency-injection test double for the `MirrorPlane`
//! trait (NOT fabricated data — Principle #13): it records ensure/release calls so
//! the tests assert against HAND oracles (the exact keys, exact call counts),
//! never a self-compare. Each test drives its own daemon over a UNIQUE temp socket
//! ⇒ parallel-safe (no `#[serial]`).

use std::collections::BTreeSet;
use std::io::{self, BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::transport::cerulion_q::UnusableRunsAnswer;
use cerulion_core::{
    CatalogEntry, CatalogProvenance, CatalogReply, GatewayEgressPolicy, GatewayPlan, RunsReply,
    SchemaReply, SchemaServing, TopicSchema, TransportError,
};

use cerulion_netd::daemon::{self, NetdConfig, RunningNetd};
use cerulion_netd::egress::{EgressError, EgressId, EgressPlane, NoopEgressPlane};
use cerulion_netd::mirror::{MirrorError, MirrorPlane, MirrorRelease};
use cerulion_netd::protocol::{
    Hello, Request, Response, HELLO_MARKER, MAX_REQUEST_LINE_BYTES, PROTOCOL_VERSION,
};
use cerulion_netd::query::{CatalogGather, QueryError, QueryPlane, RunsGather, SchemaGather};
use cerulion_netd::registry::TopicKey;
use cerulion_netd::DiscoveryState;

// --------------------------------------------------------------------------
// The counting spy mirror plane (a DI test double for MirrorPlane).
//
// PRODUCTION-FAITHFUL: like the real `register_ingress_topic`, the spy REFUSES a
// re-ensure of a key whose bridge already exists. Two teardown modes model the
// two production `release_mirror` returns:
//
// - `retired == false` (default) → `release_mirror` returns `Lingering`, KEEPING
//   the key's bridge (a re-ensure would still be refused) — the daemon's
//   lingering-reuse path (in production this is the rare teardown-FAILURE
//   fallback; here it exercises the daemon's Lingering branch).
// - `retired == true` → `release_mirror` returns `Retired` and REMOVES the key
//   from `ensured` (models the REAL teardown freeing the mirror slot), so a
//   later re-ensure SUCCEEDS — the daemon's retire + re-create path.
//
// Either way a spurious double-ensure (the desync bug) would surface as an error.
// --------------------------------------------------------------------------

#[derive(Default)]
struct SpyPlane {
    /// Keys whose bridge currently exists (ensured, not yet retired). Lingering
    /// keeps a released key here; Retired removes it (mirror slot freed).
    ensured: Mutex<BTreeSet<TopicKey>>,
    /// Every SUCCESSFUL ensure_mirror (key, schema_hash), in call order.
    ensure_ok: Mutex<Vec<(TopicKey, u64)>>,
    /// Every ensure_mirror ATTEMPT count (success OR any refusal).
    ensure_attempts: AtomicUsize,
    /// Every release_mirror key, in call order.
    releases: Mutex<Vec<TopicKey>>,
    /// When set, ensure_mirror returns a forced Err (models a transport refusal).
    fail: AtomicBool,
    /// When set, `release_mirror` returns `Retired` and frees the mirror slot
    /// (models the real teardown); otherwise `Lingering`.
    retired: AtomicBool,
}

impl SpyPlane {
    fn set_fail(&self, fail: bool) {
        self.fail.store(fail, Ordering::SeqCst);
    }
    fn set_retired(&self, retired: bool) {
        self.retired.store(retired, Ordering::SeqCst);
    }
    fn ensure_ok_count(&self) -> usize {
        self.ensure_ok.lock().unwrap().len()
    }
    fn ensure_attempts(&self) -> usize {
        self.ensure_attempts.load(Ordering::SeqCst)
    }
    fn release_count(&self) -> usize {
        self.releases.lock().unwrap().len()
    }
    fn release_keys_sorted(&self) -> Vec<TopicKey> {
        let mut v = self.releases.lock().unwrap().clone();
        v.sort();
        v
    }
}

impl MirrorPlane for SpyPlane {
    fn ensure_mirror(&self, key: &TopicKey, schema_hash: u64) -> Result<(), MirrorError> {
        self.ensure_attempts.fetch_add(1, Ordering::SeqCst);
        if self.fail.load(Ordering::SeqCst) {
            return Err(MirrorError::Register {
                key: key.clone(),
                source: Box::new(TransportError::InvalidTransportConfig {
                    reason: "spy forced failure".to_string(),
                }),
            });
        }
        // PRODUCTION-FAITHFUL: `register_ingress_topic` refuses a re-registration.
        let mut ensured = self.ensured.lock().unwrap();
        if ensured.contains(key) {
            return Err(MirrorError::Register {
                key: key.clone(),
                source: Box::new(TransportError::Internal {
                    reason: format!(
                        "spy: topic '{}' already has a registered network ingress bridge \
                         (re-ensure refused, matching register_ingress_topic)",
                        key.topic
                    ),
                }),
            });
        }
        ensured.insert(key.clone());
        self.ensure_ok
            .lock()
            .unwrap()
            .push((key.clone(), schema_hash));
        Ok(())
    }

    fn release_mirror(&self, key: &TopicKey) -> MirrorRelease {
        self.releases.lock().unwrap().push(key.clone());
        if self.retired.load(Ordering::SeqCst) {
            // Real teardown: free the mirror slot so a later re-ensure succeeds.
            self.ensured.lock().unwrap().remove(key);
            MirrorRelease::Retired
        } else {
            // Lingering: the bridge STAYS in `ensured` — the daemon's lingering
            // reuse means it never re-ensures a lingering key.
            MirrorRelease::Lingering
        }
    }
}

// --------------------------------------------------------------------------
// The counting spy QUERY plane (a DI double for QueryPlane) — canned
// catalog/schema answers + a forced-fail flag, so the stateless query verbs are
// exercised end-to-end with no zenoh. Records every query for hand oracles.
// --------------------------------------------------------------------------

#[derive(Default)]
struct SpyQueryPlane {
    /// Canned catalogs `query_catalog` returns (default empty = authoritative none).
    catalogs: Mutex<Vec<CatalogReply>>,
    /// Canned schema replies `query_schema` returns.
    schemas: Mutex<Vec<SchemaReply>>,
    /// The `robot` filter of every `query_catalog` call, in order.
    catalog_robots: Mutex<Vec<Option<String>>>,
    /// The `(robot, requested)` of every `query_schema` call, in order.
    schema_calls: Mutex<Vec<(Option<String>, String)>>,
    /// Canned runs replies `query_runs` returns (default empty = "no
    /// robot ANSWERED", which is weaker than "no robot is running anything").
    runs: Mutex<Vec<RunsReply>>,
    /// Robots that ANSWERED UNUSABLY (a wire skew), a DIFFERENT
    /// absence from a robot that did not answer, and the one whose remedy is a
    /// redeploy rather than a wait.
    unusable_runs: Mutex<Vec<UnusableRunsAnswer>>,
    /// Robots that were ASKED and did not answer at all, the
    /// COVERAGE half, a THIRD absence again (it may answer later, or never).
    silent_runs: Mutex<Vec<String>>,
    /// The `robot` filter of every `query_runs` call, in order.
    runs_robots: Mutex<Vec<Option<String>>>,
    /// When set, both query methods return a `QueryError` (models a netd that could
    /// not run the query — the desk degrades to its own transient session).
    fail: AtomicBool,
    /// When set, both query methods answer with
    /// `DiscoveryState::NotConverged` — a netd whose session has not yet completed a
    /// discovery pass, so an empty answer is NOT evidence of absence. Default (unset)
    /// is the SETTLED plane, which is the original meaning of every existing arm.
    cold_start: AtomicBool,
    /// The plane's un-settled AGE in milliseconds, or `None` for a plane
    /// that cannot report one. Settable so a daemon-level arm can prove the number
    /// reaches the wire — before that arm existed, both spies hard-coded `None`, so
    /// the `.map(|d| d.as_millis() as u64)` arm in `daemon.rs` was exercised on its
    /// `None` branch ONLY and mutating it to a literal `None` was invisible.
    unsettled_ms: Mutex<Option<u64>>,
}

impl SpyQueryPlane {
    fn set_catalogs(&self, catalogs: Vec<CatalogReply>) {
        *self.catalogs.lock().unwrap() = catalogs;
    }
    /// Model a netd that has NOT completed a discovery pass.
    fn set_cold_start(&self, cold: bool) {
        self.cold_start.store(cold, Ordering::SeqCst);
    }
    /// Model a plane that has been trying (un-settled) for `ms`.
    fn set_unsettled_ms(&self, ms: Option<u64>) {
        *self.unsettled_ms.lock().unwrap() = ms;
    }
    fn unsettled_for(&self) -> Option<std::time::Duration> {
        self.unsettled_ms
            .lock()
            .unwrap()
            .map(std::time::Duration::from_millis)
    }
    fn discovery(&self) -> DiscoveryState {
        if self.cold_start.load(Ordering::SeqCst) {
            DiscoveryState::NotConverged
        } else {
            DiscoveryState::Settled
        }
    }
    fn set_schemas(&self, schemas: Vec<SchemaReply>) {
        *self.schemas.lock().unwrap() = schemas;
    }
    /// The canned runs answer, and the record of how each call was scoped.
    fn set_runs(&self, runs: Vec<RunsReply>) {
        *self.runs.lock().unwrap() = runs;
    }
    fn runs_robots(&self) -> Vec<Option<String>> {
        self.runs_robots.lock().unwrap().clone()
    }
    /// The robots the plane could not decode an answer from.
    /// Declare the robots this plane ASKED and never heard from.
    fn set_silent_runs(&self, silent: Vec<String>) {
        *self.silent_runs.lock().unwrap() = silent;
    }

    fn set_unusable_runs(&self, unusable: Vec<UnusableRunsAnswer>) {
        *self.unusable_runs.lock().unwrap() = unusable;
    }
    fn set_fail(&self, fail: bool) {
        self.fail.store(fail, Ordering::SeqCst);
    }
    fn catalog_call_count(&self) -> usize {
        self.catalog_robots.lock().unwrap().len()
    }
    fn schema_calls(&self) -> Vec<(Option<String>, String)> {
        self.schema_calls.lock().unwrap().clone()
    }
}

impl QueryPlane for SpyQueryPlane {
    fn query_catalog(&self, robot: Option<&str>) -> Result<CatalogGather, QueryError> {
        self.catalog_robots
            .lock()
            .unwrap()
            .push(robot.map(str::to_string));
        if self.fail.load(Ordering::SeqCst) {
            return Err(QueryError::NoNetwork);
        }
        Ok(CatalogGather {
            catalogs: self.catalogs.lock().unwrap().clone(),
            discovery: self.discovery(),
            // Settable — see `set_unsettled_ms`.
            unsettled_for: self.unsettled_for(),
        })
    }
    fn query_schema(
        &self,
        robot: Option<&str>,
        requested: &str,
    ) -> Result<SchemaGather, QueryError> {
        self.schema_calls
            .lock()
            .unwrap()
            .push((robot.map(str::to_string), requested.to_string()));
        if self.fail.load(Ordering::SeqCst) {
            return Err(QueryError::NoNetwork);
        }
        Ok(SchemaGather {
            replies: self.schemas.lock().unwrap().clone(),
            discovery: self.discovery(),
            // Settable — see `set_unsettled_ms`.
            unsettled_for: self.unsettled_for(),
        })
    }
    fn query_runs(&self, robot: Option<&str>) -> Result<RunsGather, QueryError> {
        self.runs_robots
            .lock()
            .unwrap()
            .push(robot.map(str::to_string));
        if self.fail.load(Ordering::SeqCst) {
            return Err(QueryError::NoNetwork);
        }
        Ok(RunsGather {
            replies: self.runs.lock().unwrap().clone(),
            unusable: self.unusable_runs.lock().unwrap().clone(),
            silent: self.silent_runs.lock().unwrap().clone(),
            discovery: self.discovery(),
            unsettled_for: self.unsettled_for(),
        })
    }
}

// --------------------------------------------------------------------------
// Harness: start a daemon over a unique temp socket + a UDS client.
// --------------------------------------------------------------------------

/// A short unique temp dir (the socket path must fit the sockaddr_un limit).
fn unique_socket(tag: &str) -> (PathBuf, PathBuf) {
    use std::sync::atomic::AtomicU64;
    static N: AtomicU64 = AtomicU64::new(0);
    let n = N.fetch_add(1, Ordering::Relaxed);
    let dir = std::env::temp_dir().join(format!("cer_netd_e2e_{tag}_{}_{n}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("mk tempdir");
    let sock = dir.join("netd.sock");
    (dir, sock)
}

/// Start an in-process daemon with the spy plane. `grace` is the idle grace; the
/// idle-watch polls fast (25 ms) so idle tests are quick.
fn start_daemon(tag: &str, grace: Duration) -> (RunningNetd, PathBuf, PathBuf, Arc<SpyPlane>) {
    let (dir, sock) = unique_socket(tag);
    let spy = Arc::new(SpyPlane::default());
    let plane: Arc<dyn MirrorPlane> = Arc::clone(&spy) as Arc<dyn MirrorPlane>;
    let netd = daemon::start(
        sock.clone(),
        plane,
        NetdConfig {
            idle_grace: grace,
            idle_watch_poll: Duration::from_millis(25),
            ..NetdConfig::default()
        },
    )
    .expect("daemon start");
    (netd, dir, sock, spy)
}

/// Start an in-process daemon with the spy mirror plane AND a spy query
/// plane (via the full-DI `start_with_planes`), so the stateless query verbs are
/// driven end-to-end. Returns the query spy too.
fn start_daemon_with_query(
    tag: &str,
    grace: Duration,
) -> (
    RunningNetd,
    PathBuf,
    PathBuf,
    Arc<SpyPlane>,
    Arc<SpyQueryPlane>,
) {
    let (dir, sock) = unique_socket(tag);
    let spy = Arc::new(SpyPlane::default());
    let query_spy = Arc::new(SpyQueryPlane::default());
    let netd = daemon::start_with_planes(
        sock.clone(),
        Arc::clone(&spy) as Arc<dyn MirrorPlane>,
        Arc::new(NoopEgressPlane),
        Arc::clone(&query_spy) as Arc<dyn QueryPlane>,
        NetdConfig {
            idle_grace: grace,
            idle_watch_poll: Duration::from_millis(25),
            ..NetdConfig::default()
        },
    )
    .expect("daemon start");
    (netd, dir, sock, spy, query_spy)
}

/// A long grace so idle self-exit never fires during a functional test.
const LONG_GRACE: Duration = Duration::from_secs(3600);

/// How many times [`start_short_grace_and_connect`] will rebuild its fixture before
/// giving up. Each attempt is a fresh daemon on a fresh socket, so the only way to
/// exhaust this is a machine that cannot get from `daemon::start` to a connected
/// client inside the grace FIVE times running — which is a sick box, not a flake.
const FIRST_CLIENT_ATTEMPTS: usize = 5;

/// Start a SHORT-grace daemon and establish its FIRST client, restarting
/// the whole fixture if the daemon idle-self-exits before that client arrives.
///
/// # The race this closes
///
/// [`DemandRegistry::new`] anchors `idle_since` at construction, so the idle-grace
/// clock starts when `daemon::start` builds the registry — NOT at the first
/// disconnect. A test written as
///
/// ```ignore
/// let (netd, dir, sock, spy) = start_daemon("t", Duration::from_millis(150));
/// let (mut c, _) = Client::connect(&sock);          // <-- must land inside 150 ms
/// ```
///
/// therefore gives itself the whole grace as a budget to get from one statement to
/// the next. Exceed it and the idle-watch commits self-exit, which unlinks the
/// socket under the registry lock, and the connect fails in one of two ways
/// depending on which side of the commit it lands (shown by sleeping between
/// the two statements on an otherwise idle desk):
///
/// | delay | outcome |
/// |---|---|
/// | 0–100 ms | hello served (healthy) |
/// | 140–160 ms | connect OK, **hello EOF `server closed`** — a backlog connection accepted after the commit, refused by `connect_unless_exiting` |
/// | ≥ 200 ms | connect `ENOENT` — the socket is already unlinked |
///
/// The middle row is verbatim the failure this race produces
/// (`hello line: Custom { kind: UnexpectedEof, error: "server closed" }`), which is
/// why this is a DETERMINISTIC hole rather than "a loaded runner": a 140–199 ms
/// deschedule between two adjacent statements is ordinary on a loaded macOS runner
/// running the rest of this binary in parallel.
///
/// # Why a restart rather than a reconnect
///
/// A reconnect cannot help: the socket is UNLINKED at the commit, so there is
/// nothing left to dial. The daemon is gone and the fixture must be rebuilt — the
/// same whole-setup bounded retry `network_gateway_mp_e2e_test` uses for its
/// verbatim-bind port race.
///
/// # Why this does not weaken anything
///
/// The retry is gated on the daemon's OWN observable, [`RunningNetd::self_exit_requested`]:
/// a first client that cannot reach a daemon which has NOT committed to self-exit is
/// a REAL failure and panics immediately, carrying the underlying error. So the
/// property each caller then asserts — that self-exit fires after ITS disconnect —
/// is untouched; only the pre-connect startup window is absorbed. In particular the
/// daemon's post-commit refusal is deliberate behaviour with its own pins
/// (`a_connection_after_self_exit_is_committed_finds_no_socket_and_respawns`), and
/// nothing here makes that path less observable.
fn start_short_grace_and_connect<T>(
    mut start: impl FnMut() -> (RunningNetd, PathBuf, PathBuf, T),
) -> (RunningNetd, PathBuf, PathBuf, T, Client, Hello) {
    let mut last_err = None;
    for attempt in 1..=FIRST_CLIENT_ATTEMPTS {
        let (mut netd, dir, sock, extra) = start();
        match Client::try_connect(&sock) {
            Ok((client, hello)) => return (netd, dir, sock, extra, client, hello),
            Err(e) => {
                // Read the daemon's own verdict BEFORE tearing anything down.
                let committed = netd.self_exit_requested();
                netd.shutdown();
                let _ = std::fs::remove_dir_all(&dir);
                assert!(
                    committed,
                    "the first client could not reach a daemon that had NOT committed \
                     to idle self-exit — that is a real failure, not the daemon \
                     startup race: {e:?}"
                );
                eprintln!(
                    "the daemon idle-self-exited before its first client \
                     arrived (attempt {attempt}/{FIRST_CLIENT_ATTEMPTS}, {e:?}) — \
                     rebuilding the fixture"
                );
                last_err = Some(e);
            }
        }
    }
    panic!(
        "could not get a first client onto a short-grace daemon in \
         {FIRST_CLIENT_ATTEMPTS} attempts; last error: {last_err:?}"
    );
}

// ── The helper's own self-tests ───────────────────────────────────
//
// The Err branch above — BOTH the rebuild and the
// `assert!(committed)` that separates the race from a real failure — is exercised by no
// other test on a healthy machine, since its trigger is a deschedule that an idle desk
// never produces. Without these arms `FIRST_CLIENT_ATTEMPTS = 5 -> 1` and an inverted
// `committed` polarity would both pass the suite. Both arms below are DETERMINISTIC and need
// no seam: the stimulus is supplied by the start closure, which is already an
// injection point.

#[test]
fn the_short_grace_helper_rebuilds_a_fixture_whose_daemon_self_exited_first() {
    // The macOS-runner shape, made deterministic: the first fixture is left un-connected past
    // its own grace (250 ms > 150 ms), so the idle-watch commits self-exit exactly as
    // it does on a loaded runner; every later attempt connects promptly. The helper
    // must discard that fixture and rebuild EXACTLY once, and the client it returns
    // must be a working client on the REBUILT daemon — not a husk.
    let mut calls = 0usize;
    let (mut netd, dir, sock, spy, first, hello) = start_short_grace_and_connect(|| {
        calls += 1;
        let started = start_daemon("helper_rebuild", Duration::from_millis(150));
        if calls == 1 {
            std::thread::sleep(Duration::from_millis(250));
        }
        started
    });
    // A FLOOR, not an equality — an equality would re-import the very
    // race this helper exists to absorb. The closure rigs attempt 1 to lose the 150 ms
    // grace; attempt 2 then runs the SAME unprotected `daemon::start` → `try_connect`
    // window with no stimulus, so on a runner that also deschedules it past 150 ms the
    // helper does exactly its job (rebuilds, succeeds on attempt 3) and `== 2` would
    // then report a correctly-behaving helper as a regression — a failure that looks
    // like the race this helper absorbs. The four production callers need 5 consecutive
    // races to fail (~p^5); an exact assert here is exposed at ~p, the unprotected rate, in
    // the same binary on the same platform. A 2-core macOS runner has been observed to
    // exceed this window.
    //
    // Exactness would buy nothing: every failure case is caught strictly before this line —
    // `FIRST_CLIENT_ATTEMPTS = 5 -> 1` by the helper's OWN panic, and both `committed`
    // polarity variants by the sibling `..._refuses_a_failure_that_is_not_the_startup_race`
    // arm (and, for the inverting one, by this test's own helper panic). Verified
    // against this floor, not assumed. `calls == 1` — the forced rebuild
    // never happening — is what this arm exists to catch, and the floor still catches
    // it; the ceiling is structural, since the helper panics past FIRST_CLIENT_ATTEMPTS.
    assert!(
        calls >= 2,
        "the fixture whose daemon self-exited was discarded and rebuilt (attempts: {calls})"
    );
    assert_eq!(
        hello.hello, HELLO_MARKER,
        "the rebuilt daemon served a Hello"
    );
    assert!(sock.exists(), "the rebuilt daemon's socket is the live one");
    assert!(
        !netd.self_exit_requested(),
        "the returned daemon is the REBUILT one, not the corpse that self-exited"
    );

    // The returned client really works against the returned daemon.
    let mut c = first;
    match c.request(&demand(1, "ubuntu", "/tf", 7)) {
        Response::Demand(d) => assert!(d.mirror_created),
        other => panic!("the rebuilt fixture serves a demand: {other:?}"),
    }
    assert_eq!(
        spy.ensure_ok_count(),
        1,
        "the demand reached the REBUILT plane"
    );
    drop(c);
    netd.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
#[should_panic(expected = "that is a real failure, not the daemon startup race")]
fn the_short_grace_helper_refuses_a_failure_that_is_not_the_startup_race() {
    // The anti-tautology half, and the pin on the classifier's POLARITY: the rebuild
    // is licensed ONLY by the daemon's own `self_exit_requested()`. Here the daemon is
    // healthy (LONG_GRACE, so it cannot idle-exit) but the helper is handed a socket
    // path nothing is bound to — a connect failure that is emphatically NOT the
    // startup race. The helper must refuse it LOUDLY on the first attempt rather than
    // burn five rebuilds and report a startup race that did not happen.
    //
    // Without this arm, deleting the `assert!(committed, ..)` — or inverting it —
    // ships green, and a genuinely broken daemon reads as "the machine was slow".
    let _ = start_short_grace_and_connect(|| {
        let (netd, dir, _sock, spy) = start_daemon("helper_notarace", LONG_GRACE);
        // Same LENGTH as the real `netd.sock` this fixture binds, so the connect
        // fails with a clean ENOENT — the shape this arm is about. A longer name
        // overflows `sun_path` and fails EINVAL instead, which still exercises the
        // classifier but for a reason this comment does not describe (and which a
        // shorter `$TMPDIR` could silently turn back into ENOENT).
        let bogus = dir.join("nope.sock");
        assert!(
            !bogus.exists(),
            "the bogus path must name nothing: {}",
            bogus.display()
        );
        (netd, dir, bogus, spy)
    });
}

struct Client {
    stream: UnixStream,
    reader: BufReader<UnixStream>,
}

impl Client {
    /// The non-panicking twin of [`Client::connect`], used by
    /// [`start_short_grace_and_connect`] to tell the startup race from a
    /// real failure. EVERY step is fallible here — including the two `setsockopt`
    /// calls, which on macOS fail `EINVAL` when the peer's close has already landed
    /// (see `client.rs::is_closed_early`), so a `.unwrap()` there would panic on the
    /// same race by a different route.
    fn try_connect(sock: &Path) -> io::Result<(Client, Hello)> {
        let stream = UnixStream::connect(sock)?;
        stream.set_read_timeout(Some(Duration::from_secs(3)))?;
        stream.set_write_timeout(Some(Duration::from_secs(3)))?;
        let reader = BufReader::new(stream.try_clone()?);
        let mut c = Client { stream, reader };
        let line = c.read_line()?;
        let hello: Hello = serde_json::from_str(line.trim()).map_err(|e| {
            io::Error::new(io::ErrorKind::InvalidData, format!("hello parses: {e}"))
        })?;
        Ok((c, hello))
    }

    fn connect(sock: &Path) -> (Client, Hello) {
        Self::try_connect(sock).expect("connect + hello")
    }

    fn read_line(&mut self) -> io::Result<String> {
        let mut line = String::new();
        let n = self.reader.read_line(&mut line)?;
        if n == 0 {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "server closed",
            ));
        }
        Ok(line)
    }

    fn send_line(&mut self, line: &str) {
        writeln!(self.stream, "{line}").expect("send");
        self.stream.flush().expect("flush");
    }

    fn request(&mut self, req: &Request) -> Response {
        self.send_line(&req.to_json_line());
        serde_json::from_str(self.read_line().expect("read resp").trim()).expect("resp parses")
    }

    fn send_raw(&mut self, raw: &str) -> Response {
        self.send_line(raw);
        serde_json::from_str(self.read_line().expect("read resp").trim()).expect("resp parses")
    }

    /// Write raw bytes WITHOUT a trailing newline (for the oversized-line guard).
    fn write_no_newline(&mut self, bytes: &[u8]) {
        self.stream.write_all(bytes).expect("write raw");
        self.stream.flush().expect("flush");
    }

    /// Read + parse one response line (the caller expects the daemon to have sent one).
    fn read_response(&mut self) -> Response {
        serde_json::from_str(self.read_line().expect("read resp").trim()).expect("resp parses")
    }
}

fn demand(id: u64, robot: &str, topic: &str, schema_hash: u64) -> Request {
    Request::Demand {
        id,
        robot: robot.to_string(),
        topic: topic.to_string(),
        schema_hash,
    }
}
fn release(id: u64, robot: &str, topic: &str) -> Request {
    Request::Release {
        id,
        robot: robot.to_string(),
        topic: topic.to_string(),
    }
}
fn query_catalog(id: u64, robot: Option<&str>) -> Request {
    Request::QueryCatalog {
        id,
        robot: robot.map(str::to_string),
    }
}
fn query_schema(id: u64, robot: Option<&str>, requested: &str) -> Request {
    Request::QuerySchema {
        id,
        robot: robot.map(str::to_string),
        requested: requested.to_string(),
    }
}
/// A `query_runs` request.
fn query_runs(id: u64, robot: Option<&str>) -> Request {
    Request::QueryRuns {
        id,
        robot: robot.map(str::to_string),
    }
}

/// A one-run reply the way a robot's serve side mints one: a hand-built
/// oracle (a DI test fixture, not fake data; the REAL serve is pinned in
/// `cerulion_core/tests/runs_serve_iox2_test.rs`).
fn one_runs_reply(robot: &str, run_id: &str, graph: &str) -> RunsReply {
    RunsReply {
        version: 1,
        robot: robot.to_string(),
        runs: vec![cerulion_core::RunEntry {
            run_id: run_id.to_string(),
            graph_name: graph.to_string(),
            run_started_at_ns: 1_700_000_000_000_000_000,
            state: cerulion_core::RunEntryState::Live,
            graph_yaml: format!("name: {graph}\nnodes:\n  - id: cam\n"),
            run_json: format!("{{\"run_id\":\"{run_id}\"}}\n"),
        }],
        completeness: cerulion_core::RunsCompleteness::Settled,
        undescribable: Vec::new(),
        error: None,
    }
}

/// Build a one-robot [`CatalogReply`] naming `topic`'s type (a hand-built oracle
/// catalog — NOT fake data, a DI test fixture).
fn one_catalog(robot: &str, topic: &str, schema_name: &str) -> CatalogReply {
    CatalogReply {
        version: 1,
        robot: robot.to_string(),
        entries: vec![CatalogEntry {
            topic: topic.to_string(),
            schema_hash: Some(9),
            schema_name: Some(schema_name.to_string()),
            provenance: CatalogProvenance::Runtime,
            producer_count: None,
            liveness: None,
        }],
        error: None,
    }
}

/// Poll `f` until true or `timeout` elapses. Returns whether it became true.
fn wait_until(mut f: impl FnMut() -> bool, timeout: Duration) -> bool {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if f() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    f()
}

// --------------------------------------------------------------------------
// Tests.
// --------------------------------------------------------------------------

#[test]
fn hello_banner_then_demand_creates_exactly_one_mirror() {
    let (netd, dir, sock, spy) = start_daemon("demand", LONG_GRACE);
    let (mut c, hello) = Client::connect(&sock);
    // The banner is the version handshake.
    assert_eq!(hello.hello, HELLO_MARKER);
    assert_eq!(hello.protocol, PROTOCOL_VERSION);

    let resp = c.request(&demand(1, "ubuntu", "/utlidar/robot_odom", 0xABCD));
    match resp {
        Response::Demand(d) => {
            assert_eq!(d.id, 1);
            assert_eq!(d.robot, "ubuntu");
            assert_eq!(d.topic, "/utlidar/robot_odom");
            assert_eq!(d.refcount, 1);
            assert!(d.mirror_created, "the first demand creates the mirror");
        }
        other => panic!("expected Demand, got {other:?}"),
    }
    // The daemon registered EXACTLY one mirror, with the right key + hash.
    assert_eq!(spy.ensure_ok_count(), 1);
    assert_eq!(
        spy.ensure_ok.lock().unwrap()[0],
        (TopicKey::new("ubuntu", "/utlidar/robot_odom"), 0xABCD)
    );
    assert_eq!(netd.mirror_count(), 1);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn two_connections_share_one_mirror() {
    let (netd, dir, sock, spy) = start_daemon("share", LONG_GRACE);
    let (mut a, _) = Client::connect(&sock);
    let (mut b, _) = Client::connect(&sock);

    match a.request(&demand(1, "ubuntu", "/tf", 7)) {
        Response::Demand(d) => assert!(d.mirror_created && d.refcount == 1),
        other => panic!("expected Demand: {other:?}"),
    }
    // The SECOND consumer demands the SAME stream → joins the one mirror.
    match b.request(&demand(1, "ubuntu", "/tf", 7)) {
        Response::Demand(d) => {
            assert!(!d.mirror_created, "second demander joins, does NOT create");
            assert_eq!(d.refcount, 2);
        }
        other => panic!("expected Demand: {other:?}"),
    }
    // ONE ensure across two demanders (the decision: cross the network once).
    assert_eq!(spy.ensure_ok_count(), 1, "frames cross the network ONCE");
    assert_eq!(netd.mirror_count(), 1);
    assert!(wait_until(
        || netd.active_connections() == 2,
        Duration::from_secs(2)
    ));
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn same_connection_redemand_is_idempotent_over_the_wire() {
    let (netd, dir, sock, spy) = start_daemon("redemand", LONG_GRACE);
    let (mut a, _) = Client::connect(&sock);
    a.request(&demand(1, "ubuntu", "/tf", 7));
    // The SAME connection re-demanding must not inflate the refcount or re-register.
    match a.request(&demand(2, "ubuntu", "/tf", 7)) {
        Response::Demand(d) => {
            assert!(!d.mirror_created);
            assert_eq!(d.refcount, 1, "a re-demand cannot pin the mirror");
        }
        other => panic!("expected Demand: {other:?}"),
    }
    assert_eq!(spy.ensure_ok_count(), 1);
    assert_eq!(netd.mirror_count(), 1);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn release_decrements_then_last_release_lingers_the_mirror() {
    let (netd, dir, sock, spy) = start_daemon("release", LONG_GRACE);
    let (mut a, _) = Client::connect(&sock);
    let (mut b, _) = Client::connect(&sock);
    a.request(&demand(1, "ubuntu", "/tf", 7));
    b.request(&demand(1, "ubuntu", "/tf", 7));

    // b releases → still demanded by a, no teardown drive.
    match b.request(&release(2, "ubuntu", "/tf")) {
        Response::Release(r) => {
            assert!(!r.last_release);
            assert_eq!(r.refcount, 1);
        }
        other => panic!("expected Release: {other:?}"),
    }
    assert_eq!(spy.release_count(), 0, "not the last demander");

    // a releases → LAST release: the daemon drives release_mirror (called once),
    // which for a LINGERING plane (the default spy — in production the
    // teardown-FAILURE fallback) keeps the bridge (reusable), NOT removed.
    match a.request(&release(2, "ubuntu", "/tf")) {
        Response::Release(r) => {
            assert!(r.last_release);
            assert_eq!(r.refcount, 0);
        }
        other => panic!("expected Release: {other:?}"),
    }
    assert_eq!(spy.release_count(), 1, "release_mirror driven exactly once");
    assert_eq!(
        spy.release_keys_sorted(),
        vec![TopicKey::new("ubuntu", "/tf")]
    );
    // The physical bridge LINGERS: no active demands, but the mirror is kept.
    assert_eq!(netd.active_demand_count(), 0);
    assert_eq!(
        netd.lingering_count(),
        1,
        "the bridge lingers (reused on re-demand)"
    );
    assert_eq!(netd.mirror_count(), 1);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn schema_conflict_is_refused_over_the_wire() {
    let (netd, dir, sock, spy) = start_daemon("conflict", LONG_GRACE);
    let (mut a, _) = Client::connect(&sock);
    let (mut b, _) = Client::connect(&sock);
    a.request(&demand(1, "ubuntu", "/tf", 0xAA));
    // A DIFFERENT schema for the same topic is refused (one topic = one schema).
    match b.request(&demand(1, "ubuntu", "/tf", 0xBB)) {
        Response::Error(e) => {
            assert!(e.error.contains("schema conflict"), "{}", e.error);
            assert_eq!(e.topic.as_deref(), Some("/tf"));
        }
        other => panic!("expected Error: {other:?}"),
    }
    // Only the first demand registered a mirror; the conflicting one did not.
    assert_eq!(spy.ensure_ok_count(), 1);
    assert_eq!(netd.mirror_count(), 1);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn connection_close_releases_every_held_demand() {
    let (netd, dir, sock, spy) = start_daemon("close", LONG_GRACE);
    let (mut a, _) = Client::connect(&sock);
    a.request(&demand(1, "ubuntu", "/tf", 1));
    a.request(&demand(2, "ubuntu", "/utlidar/robot_odom", 2));
    assert_eq!(netd.mirror_count(), 2);

    // Drop the client socket (models a crash / clean close) — the daemon must
    // release EVERY demand it held (the crash-safe refcount). The daemon's
    // connection thread processes the close ASYNCHRONOUSLY in TWO steps:
    // (1) registry `disconnect` (demands → lingering; flips active_demand_count to
    // 0), THEN (2) the `release_mirror` loop per last-released key (flips
    // spy.release_count). Poll the LAST step's observable (release_count == 2) —
    // NOT active_demand_count, which flips at step 1 BEFORE release_mirror runs, so
    // polling it can race the release_count assert.
    drop(a);
    assert!(
        wait_until(|| spy.release_count() == 2, Duration::from_secs(3)),
        "connection close drove release_mirror for every held demand (got {})",
        spy.release_count()
    );
    // release_count == 2 ⇒ step 1 (disconnect) definitely completed, so these are
    // now race-free direct asserts.
    assert_eq!(netd.active_demand_count(), 0);
    assert_eq!(
        netd.lingering_count(),
        2,
        "both bridges linger until idle self-exit"
    );
    assert_eq!(netd.active_connections(), 0);
    assert_eq!(spy.release_keys_sorted(), {
        let mut v = vec![
            TopicKey::new("ubuntu", "/tf"),
            TopicKey::new("ubuntu", "/utlidar/robot_odom"),
        ];
        v.sort();
        v
    });
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn status_reports_the_demand_table() {
    let (_netd, dir, sock, _spy) = start_daemon("status", LONG_GRACE);
    let (mut a, _) = Client::connect(&sock);
    let (mut b, _) = Client::connect(&sock);
    a.request(&demand(1, "ubuntu", "/tf", 7));
    b.request(&demand(1, "ubuntu", "/tf", 7)); // refcount → 2
    a.request(&demand(2, "go2", "/odom", 9));

    match a.request(&Request::Status { id: 99 }) {
        Response::Status(s) => {
            assert_eq!(s.id, 99);
            assert_eq!(s.active_connections, 2, "both consumers are connected");
            assert!(!s.idle, "busy while connected");
            // Sorted canonically: go2//odom then ubuntu//tf (refcount 2).
            assert_eq!(s.demands.len(), 2);
            assert_eq!(s.demands[0].robot, "go2");
            assert_eq!(s.demands[0].topic, "/odom");
            assert_eq!(s.demands[0].refcount, 1);
            assert_eq!(s.demands[1].robot, "ubuntu");
            assert_eq!(s.demands[1].topic, "/tf");
            assert_eq!(s.demands[1].refcount, 2);
        }
        other => panic!("expected Status: {other:?}"),
    }
    let _ = std::fs::remove_dir_all(&dir);
}

/// Principle #3: the connect set the daemon was configured with, the
/// explicit `CERULION_NETD_CONNECT` locators plus whatever the boot-time
/// peer-cache fold added — reaches the CONSUMER over the control seam, so an
/// operator can answer "why is my desk talking to that address?" directly
/// instead of inferring it from logs.
///
/// The fold decision itself is oracle-tested in `discovery_fold`; this pins the
/// WIRING, which is where it could go silently inert (a daemon that folded
/// correctly and then reported nothing looks identical to one that folded
/// nothing).
#[test]
fn status_reports_the_connect_set_the_daemon_dialled() {
    let (dir, sock) = unique_socket("status");
    let spy = Arc::new(SpyPlane::default());
    let netd = daemon::start(
        sock.clone(),
        Arc::clone(&spy) as Arc<dyn MirrorPlane>,
        NetdConfig {
            idle_grace: LONG_GRACE,
            idle_watch_poll: Duration::from_millis(25),
            connect_endpoints: vec![
                "tcp/10.0.0.5:7683".to_string(),
                "tcp/10.0.0.6:7683".to_string(),
            ],
            ..NetdConfig::default()
        },
    )
    .expect("daemon start");
    let (mut a, _) = Client::connect(&sock);
    match a.request(&Request::Status { id: 7 }) {
        Response::Status(s) => assert_eq!(
            s.connect_endpoints,
            Some(vec![
                "tcp/10.0.0.5:7683".to_string(),
                "tcp/10.0.0.6:7683".to_string()
            ]),
            "status must report the endpoints this daemon dialled, in order"
        ),
        other => panic!("expected Status: {other:?}"),
    }
    drop(a);
    drop(netd);

    // ANTI-TAUTOLOGY: a daemon that folded NOTHING reports `Some([])` — a real
    // answer ("this daemon dialled nothing"), never `None`, which is reserved
    // for an older daemon that cannot report at all.
    let (_netd2, dir2, sock2, _spy2) = start_daemon("statusempty", LONG_GRACE);
    let (mut b, _) = Client::connect(&sock2);
    match b.request(&Request::Status { id: 8 }) {
        Response::Status(s) => assert_eq!(
            s.connect_endpoints,
            Some(Vec::new()),
            "a daemon with nothing folded still reports, positively, that it has nothing"
        ),
        other => panic!("expected Status: {other:?}"),
    }
    let _ = std::fs::remove_dir_all(&dir);
    let _ = std::fs::remove_dir_all(&dir2);
}

#[test]
fn mirror_ensure_failure_is_refused_and_rolled_back() {
    let (netd, dir, sock, spy) = start_daemon("failmirror", LONG_GRACE);
    spy.set_fail(true);
    let (mut a, _) = Client::connect(&sock);

    match a.request(&demand(1, "ubuntu", "/tf", 7)) {
        Response::Error(e) => {
            assert!(
                e.error.contains("mirror registration failed"),
                "{}",
                e.error
            );
            assert_eq!(e.topic.as_deref(), Some("/tf"));
        }
        other => panic!("expected Error: {other:?}"),
    }
    // The ensure was ATTEMPTED (1) but not successful (0), and the refcount was
    // ROLLED BACK — no phantom mirror.
    assert_eq!(spy.ensure_attempts(), 1);
    assert_eq!(spy.ensure_ok_count(), 0);
    assert_eq!(netd.mirror_count(), 0, "the failed demand left no refcount");

    // Recovery: with failure cleared, the SAME demand now succeeds (no residue).
    spy.set_fail(false);
    match a.request(&demand(2, "ubuntu", "/tf", 7)) {
        Response::Demand(d) => assert!(d.mirror_created && d.refcount == 1),
        other => panic!("expected Demand after recovery: {other:?}"),
    }
    assert_eq!(netd.mirror_count(), 1);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn demand_release_redemand_reuses_the_mirror_without_re_ensuring() {
    // The reuse-not-re-ensure pin (over the STRICT spy): demand → release → re-demand on the
    // same key must SUCCEED by REUSING the lingering bridge — the daemon must NOT
    // re-ensure (the production-faithful spy refuses a re-ensure, exactly as
    // register_ingress_topic does, so a double-ensure would surface as an error).
    let (netd, dir, sock, spy) = start_daemon("reuse", LONG_GRACE);
    let (mut a, _) = Client::connect(&sock);

    match a.request(&demand(1, "ubuntu", "/tf", 7)) {
        Response::Demand(d) => assert!(d.mirror_created && d.refcount == 1),
        other => panic!("first demand: {other:?}"),
    }
    assert_eq!(spy.ensure_ok_count(), 1);

    // Release → the bridge LINGERS (spy release_mirror returns Lingering).
    match a.request(&release(2, "ubuntu", "/tf")) {
        Response::Release(r) => assert!(r.last_release && r.refcount == 0),
        other => panic!("release: {other:?}"),
    }
    assert_eq!(netd.lingering_count(), 1);
    assert_eq!(spy.release_count(), 1);

    // RE-DEMAND → must REUSE the lingering bridge (mirror_created false), NOT
    // re-ensure. Without the reuse this is a fresh FirstDemand → ensure → the strict spy
    // refuses → an Error response.
    match a.request(&demand(3, "ubuntu", "/tf", 7)) {
        Response::Demand(d) => {
            assert!(
                !d.mirror_created,
                "reuses the lingering bridge, not re-registers"
            );
            assert_eq!(d.refcount, 1);
        }
        Response::Error(e) => panic!("re-demand FAILED (the desync bug): {}", e.error),
        other => panic!("unexpected: {other:?}"),
    }
    // No re-ensure was EVEN ATTEMPTED — the reuse short-circuits in the registry.
    assert_eq!(spy.ensure_ok_count(), 1, "no re-ensure");
    assert_eq!(
        spy.ensure_attempts(),
        1,
        "the daemon never re-attempted ensure"
    );
    assert_eq!(netd.active_demand_count(), 1);
    assert_eq!(netd.lingering_count(), 0);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn last_release_with_a_teardown_plane_retires_and_redemand_recreates() {
    // Teardown at the daemon level (Retired spy = the production teardown): a last
    // release TEARS the bridge down (release_mirror → Retired) and RETIRES the
    // registry entry (no lingering); a re-demand is then a FRESH FirstDemand that
    // RE-ENSURES the mirror. The full demand→release→re-demand cycle over a real
    // teardown — the shape `mirror_plane_iox2_test` proves over real transport.
    let (netd, dir, sock, spy) = start_daemon("retire", LONG_GRACE);
    spy.set_retired(true);
    let (mut a, _) = Client::connect(&sock);

    match a.request(&demand(1, "ubuntu", "/tf", 7)) {
        Response::Demand(d) => assert!(d.mirror_created && d.refcount == 1),
        other => panic!("first demand: {other:?}"),
    }
    assert_eq!(spy.ensure_ok_count(), 1);

    // Last release → real teardown → Retired → the registry entry is RETIRED.
    match a.request(&release(2, "ubuntu", "/tf")) {
        Response::Release(r) => assert!(r.last_release && r.refcount == 0),
        other => panic!("release: {other:?}"),
    }
    assert_eq!(spy.release_count(), 1, "release_mirror driven once");
    assert_eq!(
        netd.lingering_count(),
        0,
        "a Retired teardown does NOT linger"
    );
    assert_eq!(netd.mirror_count(), 0, "the registry entry is retired");
    assert_eq!(netd.active_demand_count(), 0);

    // Re-demand → a FRESH FirstDemand (the entry was retired) → RE-ENSURE succeeds
    // (the Retired spy freed the slot; a lingering spy would refuse the re-ensure).
    match a.request(&demand(3, "ubuntu", "/tf", 7)) {
        Response::Demand(d) => {
            assert!(
                d.mirror_created,
                "re-demand RE-CREATES the mirror after a real teardown"
            );
            assert_eq!(d.refcount, 1);
        }
        Response::Error(e) => panic!("re-demand FAILED: {}", e.error),
        other => panic!("unexpected: {other:?}"),
    }
    assert_eq!(spy.ensure_ok_count(), 2, "the mirror was re-ensured");
    assert_eq!(spy.ensure_attempts(), 2);
    assert_eq!(netd.mirror_count(), 1);
    assert_eq!(netd.active_demand_count(), 1);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn disconnect_with_a_teardown_plane_retires_every_held_mirror() {
    // A connection close with a Retired plane tears down + RETIRES every held
    // mirror (nothing lingers) — the crash-safe refcount composed with real
    // teardown.
    let (netd, dir, sock, spy) = start_daemon("retire_close", LONG_GRACE);
    spy.set_retired(true);
    let (mut a, _) = Client::connect(&sock);
    a.request(&demand(1, "ubuntu", "/tf", 1));
    a.request(&demand(2, "ubuntu", "/utlidar/robot_odom", 2));
    assert_eq!(netd.mirror_count(), 2);

    drop(a);
    assert!(
        wait_until(|| spy.release_count() == 2, Duration::from_secs(3)),
        "connection close drove release_mirror for every held demand (got {})",
        spy.release_count()
    );
    assert_eq!(
        netd.mirror_count(),
        0,
        "every held mirror is retired on disconnect (none linger)"
    );
    assert_eq!(netd.lingering_count(), 0);
    assert_eq!(netd.active_connections(), 0);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn double_release_after_a_teardown_is_a_loud_not_held_error() {
    // A second release of an already-released (and torn-down) topic is a
    // LOUD NotHeld error — never a silent success.
    let (netd, dir, sock, spy) = start_daemon("double_rel", LONG_GRACE);
    spy.set_retired(true);
    let (mut a, _) = Client::connect(&sock);
    a.request(&demand(1, "ubuntu", "/tf", 7));
    match a.request(&release(2, "ubuntu", "/tf")) {
        Response::Release(r) => assert!(r.last_release),
        other => panic!("first release: {other:?}"),
    }
    assert_eq!(netd.mirror_count(), 0, "retired");

    // The connection no longer holds /tf → a second release is a structured error.
    match a.request(&release(3, "ubuntu", "/tf")) {
        Response::Error(e) => {
            assert!(e.error.contains("did not demand"), "{}", e.error);
            assert_eq!(e.topic.as_deref(), Some("/tf"));
        }
        other => panic!("expected a NotHeld error on the second release: {other:?}"),
    }
    assert_eq!(spy.release_count(), 1, "release_mirror only fired once");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn oversized_request_line_is_dropped() {
    // The MAX_REQUEST_LINE_BYTES guard: a consumer building an unbounded line (no
    // newline) is refused with a structured error and the connection is DROPPED,
    // so the daemon's read buffer can never OOM.
    let (_netd, dir, sock, _spy) = start_daemon("oversized", LONG_GRACE);
    let (mut a, _) = Client::connect(&sock);

    let huge = "x".repeat(MAX_REQUEST_LINE_BYTES + 4096);
    a.write_no_newline(huge.as_bytes());

    // The daemon refuses with a structured error naming the cap.
    match a.read_response() {
        Response::Error(e) => {
            assert!(e.error.contains("exceeded"), "{}", e.error);
            assert!(
                e.error.contains(&MAX_REQUEST_LINE_BYTES.to_string()),
                "names the cap: {}",
                e.error
            );
        }
        other => panic!("expected Error for the oversized line: {other:?}"),
    }
    // The connection was DROPPED — the next read fails with a CLOSED-CONNECTION kind,
    // NOT a read TIMEOUT (which would mean the daemon left the connection open).
    //
    // The kind is a SET, not one member of it. After the peer closes, a read
    // surfaces either the clean FIN (`read_line` yields 0 bytes, which the harness
    // synthesizes as `UnexpectedEof`) or an RST — Linux sends RST when a socket is
    // closed with unread bytes still in its receive buffer, which is EXACTLY this
    // shape: the daemon errors at the cap and closes without draining the oversized
    // line's tail. Which one arrives is a timing race over how much the daemon had
    // read before dropping, and BOTH prove the drop. Pinning `UnexpectedEof` alone
    // fails on Linux with `ConnectionReset` (os error 104).
    // `BrokenPipe` is deliberately NOT in the set: EPIPE is a WRITE-side errno and this
    // path only reads. A timeout kind (`WouldBlock` / `TimedOut`) still FAILS — that is
    // the discrimination this arm exists for.
    let err = a
        .read_line()
        .expect_err("the connection is dropped after an oversized line");
    assert!(
        matches!(
            err.kind(),
            io::ErrorKind::UnexpectedEof | io::ErrorKind::ConnectionReset
        ),
        "the daemon dropped the connection (EOF or RST), not merely a read timeout: {err:?}"
    );
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn malformed_request_is_a_structured_error_and_the_connection_survives() {
    let (_netd, dir, sock, _spy) = start_daemon("malformed", LONG_GRACE);
    let (mut a, _) = Client::connect(&sock);

    // Garbage line → structured error (no id recoverable), connection NOT dropped.
    match a.send_raw("this is not json") {
        Response::Error(e) => {
            assert!(e.error.contains("malformed request"), "{}", e.error);
            assert_eq!(e.id, None);
        }
        other => panic!("expected Error: {other:?}"),
    }
    // Unknown method with an id → error carrying the correlation id.
    match a.send_raw(r#"{"method":"bogus","id":5}"#) {
        Response::Error(e) => assert_eq!(e.id, Some(5)),
        other => panic!("expected Error: {other:?}"),
    }
    // The connection SURVIVED both — a valid demand now works.
    match a.request(&demand(6, "ubuntu", "/tf", 7)) {
        Response::Demand(d) => assert!(d.mirror_created),
        other => panic!("expected Demand after malformed: {other:?}"),
    }
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn spurious_spawn_self_exits_after_the_grace() {
    // A netd spawned but never demanded (a spurious spawn) self-exits after the
    // grace, leaving nothing behind.
    let (netd, dir, _sock, _spy) = start_daemon("spurious", Duration::from_millis(200));
    assert!(
        wait_until(|| netd.self_exit_requested(), Duration::from_secs(3)),
        "a consumer-less netd self-exits after the idle grace"
    );
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn a_live_connection_prevents_self_exit_until_it_disconnects() {
    // The connect must land inside the 200 ms grace, so the fixture is
    // rebuilt if the daemon self-exits before the client arrives.
    let (netd, dir, _sock, _spy, a, _) =
        start_short_grace_and_connect(|| start_daemon("liveconn", Duration::from_millis(200)));
    // A live connection keeps netd busy well beyond the grace.
    std::thread::sleep(Duration::from_millis(600));
    assert!(
        !netd.self_exit_requested(),
        "a live connection prevents self-exit"
    );
    assert!(!netd.is_idle());

    // Disconnect → idle arms → self-exit after the grace.
    drop(a);
    assert!(
        wait_until(|| netd.self_exit_requested(), Duration::from_secs(3)),
        "disconnecting the last consumer triggers self-exit after the grace"
    );
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn a_second_daemon_on_the_same_socket_is_refused() {
    // The daemon-level singleton guarantee (the first-consumer-spawns race): a
    // second start on the same socket while the first holds the flock is refused.
    let (_netd, dir, sock, _spy) = start_daemon("singleton", LONG_GRACE);
    let spy2 = Arc::new(SpyPlane::default());
    let plane2: Arc<dyn MirrorPlane> = spy2;
    let err = daemon::start(sock.clone(), plane2, NetdConfig::default())
        .expect_err("a second daemon on the same socket is refused");
    assert_eq!(err.kind(), io::ErrorKind::AddrInUse);
    assert!(err.to_string().contains("already running"), "{err}");
    let _ = std::fs::remove_dir_all(&dir);
}

// --------------------------------------------------------------------------
// Idle self-exit ACTUALLY terminates + the socket is unlinked FIRST, and
// the exit decision is atomic with the accept loop. These pin the daemon-LIBRARY
// half of the guarantee (the SpyPlane has no zenoh/network threads to wedge, so the
// PROCESS-level hard-exit watchdog against a real teardown block is pinned by the
// subprocess tests in `daemon_idle_exit_e2e_test.rs`).
// --------------------------------------------------------------------------

#[test]
fn idle_self_exit_unlinks_the_socket_at_commit_and_terminates() {
    // Drive a demand + release through the real daemon, disconnect, let the idle
    // grace elapse. The socket is unlinked AT the self-exit COMMIT (in the
    // idle-watch, under the registry lock) — NOT ~200ms later in shutdown. This test
    // is mutation-sensitive to that ordering: it asserts the socket is GONE the
    // instant self_exit_requested() flips, BEFORE calling shutdown (were the
    // socket to stay bound until shutdown, this would fail).
    // The connect must land inside the 150 ms grace, so the fixture is
    // rebuilt if the daemon self-exits before the client arrives.
    let (mut netd, dir, sock, spy, first, _) =
        start_short_grace_and_connect(|| start_daemon("idle_term", Duration::from_millis(150)));
    {
        let mut c = first;
        match c.request(&demand(1, "ubuntu", "/tf", 7)) {
            Response::Demand(d) => assert!(d.mirror_created),
            other => panic!("expected Demand: {other:?}"),
        }
        // The socket is live while a consumer is connected.
        assert!(sock.exists(), "socket bound while serving");
    } // c drops → connection close releases the demand → the daemon goes idle.
    assert!(
        wait_until(|| spy.release_count() == 1, Duration::from_secs(2)),
        "the disconnect released the demand"
    );
    // The idle-watch commits self-exit after the grace (the flag `main.rs` polls).
    assert!(
        wait_until(|| netd.self_exit_requested(), Duration::from_secs(3)),
        "idle self-exit is committed after the grace"
    );

    // UNLINK-AT-COMMIT: the socket is gone the instant self-exit is committed — no
    // shutdown() call yet. Poll briefly (the flag flips before the guard drop within
    // the same critical section, but the two observables are read separately).
    assert!(
        wait_until(|| !sock.exists(), Duration::from_secs(1)),
        "the control socket is unlinked AT the self-exit commit (before shutdown)"
    );
    // A client connecting now finds no daemon (NotFound) → it respawns a fresh one.
    let err = UnixStream::connect(&sock).expect_err("no daemon after the commit unlink");
    assert!(
        cerulion_netd::client::is_not_running(&err),
        "a post-commit connect classifies is_not_running (client respawns): {err:?}"
    );

    // shutdown() then just stops the threads (the guard is already gone) — idempotent.
    netd.shutdown();
    assert!(!sock.exists(), "socket stays gone after shutdown");
    let _ = std::fs::remove_dir_all(&dir);
}

/// A mirror plane whose `release_mirror` BLOCKS until unblocked — a DI double
/// modelling the real plane's blocking zenoh undeclare (which on a live robot can
/// stall indefinitely). `ensure_mirror` always succeeds. NOT fabricated data: a
/// legitimate fault-injection test double for the `MirrorPlane` trait.
#[derive(Default)]
struct BlockingReleasePlane {
    /// Every `ensure_mirror` call — proves a re-demand got a FRESH ensure (a real
    /// new mirror) rather than reusing a dying bridge.
    ensure_count: Arc<AtomicUsize>,
    /// Flips true when `release_mirror` is entered (the teardown started blocking).
    release_started: Arc<AtomicBool>,
    /// The test sets this to let a blocked `release_mirror` return (so teardown /
    /// the daemon join can complete instead of hanging the test).
    unblock: Arc<AtomicBool>,
}
impl MirrorPlane for BlockingReleasePlane {
    fn ensure_mirror(&self, _key: &TopicKey, _schema_hash: u64) -> Result<(), MirrorError> {
        self.ensure_count.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }
    fn release_mirror(&self, _key: &TopicKey) -> MirrorRelease {
        self.release_started.store(true, Ordering::SeqCst);
        // Block (like a stalled zenoh undeclare) until the test unblocks us.
        while !self.unblock.load(Ordering::SeqCst) {
            std::thread::sleep(Duration::from_millis(5));
        }
        MirrorRelease::Retired
    }
}

#[test]
fn a_blocking_mirror_teardown_does_not_starve_self_exit() {
    // The wedge regression: a client demands then
    // disconnects, driving the mirror teardown — which BLOCKS (a stalled zenoh
    // undeclare). A daemon holding the registry lock across that blocking
    // teardown STARVES the idle-watch so self-exit NEVER fires and the daemon
    // wedges forever. The teardown runs OFF the registry lock, so idle
    // self-exit still fires despite the block. Reverting the off-lock change makes
    // this test time out (the wedge).
    // This arm can fail on a loaded macOS runner with
    // `hello line: … UnexpectedEof "server closed"`. The 150 ms grace is the budget
    // this test gave itself to get from `daemon::start` to a served hello, and the
    // daemon correctly self-exits when nobody arrives inside it — so the fixture is
    // rebuilt rather than the connect being read as a product failure. The property
    // under test (self-exit fires despite a BLOCKING teardown) is unchanged: it is
    // asserted below, after this client has connected and disconnected.
    let (mut netd, dir, _sock, (started, unblock), first, _) =
        start_short_grace_and_connect(|| {
            let (dir, sock) = unique_socket("blocking_selfexit");
            let plane = Arc::new(BlockingReleasePlane::default());
            let started = Arc::clone(&plane.release_started);
            let unblock = Arc::clone(&plane.unblock);
            let plane_dyn: Arc<dyn MirrorPlane> = Arc::clone(&plane) as Arc<dyn MirrorPlane>;
            let netd = daemon::start(
                sock.clone(),
                plane_dyn,
                NetdConfig {
                    idle_grace: Duration::from_millis(150),
                    idle_watch_poll: Duration::from_millis(25),
                    ..NetdConfig::default()
                },
            )
            .expect("daemon start");
            (netd, dir, sock, (started, unblock))
        });

    {
        let mut c = first;
        match c.request(&demand(1, "ubuntu", "/tf", 7)) {
            Response::Demand(d) => assert!(d.mirror_created),
            other => panic!("expected Demand: {other:?}"),
        }
    } // c drops → disconnect → the daemon drives the (blocking) release_mirror.

    // The teardown IS blocking now (we're inside the stalled release_mirror).
    assert!(
        wait_until(|| started.load(Ordering::SeqCst), Duration::from_secs(2)),
        "the disconnect drove the (blocking) mirror teardown"
    );
    // CRITICAL: idle self-exit STILL fires despite the teardown blocking — because
    // it runs off the registry lock. A held lock times this out (the wedge).
    assert!(
        wait_until(|| netd.self_exit_requested(), Duration::from_secs(3)),
        "self-exit fires despite a blocking mirror teardown (a held lock wedges forever)"
    );

    // Unblock so the daemon's shutdown can join the (now-unblocked) teardown thread.
    unblock.store(true, Ordering::SeqCst);
    netd.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn a_blocking_mirror_teardown_does_not_block_other_connections() {
    // The sibling pin: while one topic's teardown BLOCKS (a stalled zenoh undeclare),
    // a NEW consumer connecting for a DIFFERENT topic is served normally — the
    // registry lock is FREE. A held lock would block the new
    // connection's `connect` and its Hello read would time out.
    let (dir, sock) = unique_socket("blocking_other");
    let plane = Arc::new(BlockingReleasePlane::default());
    let started = Arc::clone(&plane.release_started);
    let unblock = Arc::clone(&plane.unblock);
    let plane_dyn: Arc<dyn MirrorPlane> = Arc::clone(&plane) as Arc<dyn MirrorPlane>;
    let mut netd = daemon::start(
        sock.clone(),
        plane_dyn,
        NetdConfig {
            idle_grace: LONG_GRACE, // never idle-exit mid-test
            idle_watch_poll: Duration::from_millis(25),
            ..NetdConfig::default()
        },
    )
    .expect("daemon start");

    {
        let (mut a, _) = Client::connect(&sock);
        a.request(&demand(1, "ubuntu", "/tf", 7));
    } // a drops → disconnect → the (blocking) teardown of /tf begins.
    assert!(
        wait_until(|| started.load(Ordering::SeqCst), Duration::from_secs(2)),
        "the disconnect drove the (blocking) mirror teardown of /tf"
    );

    // A NEW consumer for a DIFFERENT topic connects + demands WHILE /tf's teardown
    // blocks — proving the registry lock is not held across the blocking teardown.
    let (mut b, hello_b) = Client::connect(&sock);
    assert_eq!(
        hello_b.hello, HELLO_MARKER,
        "the new connection got its Hello"
    );
    match b.request(&demand(2, "go2", "/odom", 9)) {
        Response::Demand(d) => assert!(d.mirror_created && d.refcount == 1),
        other => panic!("a new demand is served while another topic's teardown blocks: {other:?}"),
    }

    // Unblock + tear down.
    unblock.store(true, Ordering::SeqCst);
    drop(b);
    netd.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn a_same_key_redemand_during_teardown_waits_and_gets_a_real_mirror() {
    // The phantom-held-ack regression (pinned via the registry
    // Tearing state): while a topic's mirror is being TORN DOWN (blocking, off-lock),
    // a NEW demand for the SAME topic must NOT reuse the dying bridge (a phantom ack
    // on a slot about to be released). It WAITS for the teardown, then gets a FRESH
    // mirror (ensure_mirror called AGAIN → mirror_created:true). Without the Tearing
    // state the re-demand takes AlreadyMirrored on the lingering entry → mirror_created:false
    // backing a dead bridge, and no second ensure.
    let (dir, sock) = unique_socket("redemand_teardown");
    let plane = Arc::new(BlockingReleasePlane::default());
    let ensure_count = Arc::clone(&plane.ensure_count);
    let started = Arc::clone(&plane.release_started);
    let unblock = Arc::clone(&plane.unblock);
    let plane_dyn: Arc<dyn MirrorPlane> = Arc::clone(&plane) as Arc<dyn MirrorPlane>;
    let netd = daemon::start(
        sock.clone(),
        plane_dyn,
        NetdConfig {
            idle_grace: LONG_GRACE,
            idle_watch_poll: Duration::from_millis(25),
            ..NetdConfig::default()
        },
    )
    .expect("daemon start");

    // A demands /tf (ensure #1), then disconnects → the (blocking) teardown of /tf
    // begins (the entry is now `tearing`).
    {
        let (mut a, _) = Client::connect(&sock);
        a.request(&demand(1, "ubuntu", "/tf", 7));
    }
    assert!(
        wait_until(|| started.load(Ordering::SeqCst), Duration::from_secs(2)),
        "the disconnect drove the (blocking) mirror teardown of /tf"
    );
    assert_eq!(
        ensure_count.load(Ordering::SeqCst),
        1,
        "one ensure so far (A)"
    );

    // B demands the SAME topic /tf WHILE the teardown blocks — on a thread, since the
    // demand must WAIT (Retiring) for the teardown to finish. It reports its outcome
    // over a channel.
    let (tx, rx) = std::sync::mpsc::channel::<Response>();
    let sock_b = sock.clone();
    let b_thread = std::thread::spawn(move || {
        let (mut b, _) = Client::connect(&sock_b);
        let resp = b.request(&demand(2, "ubuntu", "/tf", 7));
        let _ = tx.send(resp);
        // Keep the connection alive until the test signals done (so B's demand is not
        // released before the assertions read it); drop happens when the thread ends.
        std::thread::sleep(Duration::from_millis(200));
    });

    // While the teardown is blocked, B's demand must NOT have completed (it is
    // waiting on the Retiring loop — the phantom-ack path would have returned it
    // immediately). recv_timeout Err proves B is still blocked.
    assert!(
        rx.recv_timeout(Duration::from_millis(400)).is_err(),
        "B's same-key demand WAITS during the teardown (never a phantom-held ack)"
    );

    // Unblock the teardown → /tf retires → B's waiting demand re-demands into a FRESH
    // mirror (ensure #2, mirror_created:true).
    unblock.store(true, Ordering::SeqCst);
    let b_resp = rx
        .recv_timeout(Duration::from_secs(5))
        .expect("B's demand completes after the teardown finishes");
    match b_resp {
        Response::Demand(d) => {
            assert!(
                d.mirror_created,
                "B gets a REAL new mirror after the teardown (not a phantom reuse)"
            );
            assert_eq!(d.refcount, 1);
        }
        other => panic!("B expected a Demand with a fresh mirror, got {other:?}"),
    }
    assert_eq!(
        ensure_count.load(Ordering::SeqCst),
        2,
        "the re-demand triggered a SECOND ensure (a real new mirror), not a reuse"
    );

    let _ = b_thread.join();
    drop(netd);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn a_connection_after_self_exit_is_committed_finds_no_socket_and_respawns() {
    // Atomicity: the idle-watch commits self-exit AND unlinks
    // the socket in ONE critical section (under the registry lock). So a client that
    // tries to connect after the commit finds NO socket → classifies is_not_running
    // → respawns a fresh daemon — it is NEVER accepted-then-dropped mid-handshake in
    // the commit→unlink window (there is no such window now). The already-queued
    // BACKLOG-connection backstop (`connect_unless_exiting` refusing without a Hello)
    // is unit-pinned by `connect_unless_exiting_gates_on_the_self_exit_commit`.
    let (netd, dir, sock, _spy) = start_daemon("boundary", Duration::from_millis(150));
    assert!(
        wait_until(|| netd.self_exit_requested(), Duration::from_secs(3)),
        "the idle-watch committed self-exit"
    );
    // The socket was unlinked AT the commit (before this harness ever calls
    // shutdown), so a connect finds no daemon.
    assert!(
        wait_until(|| !sock.exists(), Duration::from_secs(1)),
        "the socket is unlinked at the commit"
    );
    let err = UnixStream::connect(&sock).expect_err("no socket after the commit unlink");
    assert!(
        cerulion_netd::client::is_not_running(&err),
        "a post-commit connect classifies is_not_running (client respawns): {err:?}"
    );
    drop(netd);
    let _ = std::fs::remove_dir_all(&dir);
}

// --------------------------------------------------------------------------
// The counting spy EGRESS plane (a DI test double for EgressPlane)
// + the egress-verb e2e pins over the REAL UDS control server.
// --------------------------------------------------------------------------

#[derive(Default)]
struct SpyEgressPlane {
    /// Every register_egress `(EgressId, announce topics)`, in call order.
    registers: Mutex<Vec<(EgressId, Vec<String>)>>,
    /// The forwarded `ix_config_json` of every register_egress, in call
    /// order — the plane observes exactly what the daemon dispatched (the pass-through
    /// pin).
    configs: Mutex<Vec<Option<String>>>,
    /// Every release_egress EgressId, in call order.
    releases: Mutex<Vec<EgressId>>,
    /// Whether the shared gateway has "booted" (the first register) — models the
    /// production `gateway_started` bool.
    booted: AtomicBool,
    /// When set, register_egress returns an Err (models a gateway boot failure).
    fail: AtomicBool,
}

impl SpyEgressPlane {
    fn set_fail(&self, fail: bool) {
        self.fail.store(fail, Ordering::SeqCst);
    }
    fn register_count(&self) -> usize {
        self.registers.lock().unwrap().len()
    }
    /// The forwarded configs the daemon dispatched to the plane, in call order.
    fn observed_configs(&self) -> Vec<Option<String>> {
        self.configs.lock().unwrap().clone()
    }
    fn release_count(&self) -> usize {
        self.releases.lock().unwrap().len()
    }
    fn registered_topics(&self) -> Vec<Vec<String>> {
        self.registers
            .lock()
            .unwrap()
            .iter()
            .map(|(_, t)| t.clone())
            .collect()
    }
}

impl EgressPlane for SpyEgressPlane {
    fn register_egress(
        &self,
        id: EgressId,
        plan: &GatewayPlan,
        _serving: &SchemaServing,
        ix_config_json: Option<&str>,
    ) -> Result<bool, EgressError> {
        self.registers
            .lock()
            .unwrap()
            .push((id, plan.announce.clone()));
        self.configs
            .lock()
            .unwrap()
            .push(ix_config_json.map(str::to_string));
        if self.fail.load(Ordering::SeqCst) {
            return Err(EgressError::Gateway {
                source: Box::new(TransportError::InvalidTransportConfig {
                    reason: "spy forced egress failure".to_string(),
                }),
            });
        }
        // The first register "boots" the shared gateway.
        Ok(!self.booted.swap(true, Ordering::SeqCst))
    }

    fn release_egress(&self, id: EgressId) {
        self.releases.lock().unwrap().push(id);
    }
}

/// Start an in-process daemon with BOTH a spy mirror plane and a spy egress plane.
fn start_daemon_with_egress(
    tag: &str,
    grace: Duration,
) -> (
    RunningNetd,
    PathBuf,
    PathBuf,
    Arc<SpyPlane>,
    Arc<SpyEgressPlane>,
) {
    start_daemon_with_egress_policy(tag, grace, Default::default())
}

fn start_daemon_with_egress_policy(
    tag: &str,
    grace: Duration,
    egress_login: cerulion_netd::serving_login::EgressLoginPolicy,
) -> (
    RunningNetd,
    PathBuf,
    PathBuf,
    Arc<SpyPlane>,
    Arc<SpyEgressPlane>,
) {
    let (dir, sock) = unique_socket(tag);
    let spy = Arc::new(SpyPlane::default());
    let egress_spy = Arc::new(SpyEgressPlane::default());
    let mirror: Arc<dyn MirrorPlane> = Arc::clone(&spy) as Arc<dyn MirrorPlane>;
    let egress: Arc<dyn EgressPlane> = Arc::clone(&egress_spy) as Arc<dyn EgressPlane>;
    let netd = daemon::start_with_egress(
        sock.clone(),
        mirror,
        egress,
        NetdConfig {
            idle_grace: grace,
            idle_watch_poll: Duration::from_millis(25),
            egress_login,
            ..NetdConfig::default()
        },
    )
    .expect("daemon start");
    (netd, dir, sock, spy, egress_spy)
}

fn register_egress(id: u64, topics: &[&str]) -> Request {
    register_egress_cfg(id, topics, None)
}

/// Build a `register_egress` carrying an optional forwarded namespace
/// config (the multi-process shape when `Some`).
fn register_egress_cfg(id: u64, topics: &[&str], ix_config_json: Option<&str>) -> Request {
    Request::RegisterEgress {
        id,
        plan: GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: topics.iter().map(|s| s.to_string()).collect(),
            ingress: vec![],
        },
        schema_serving: SchemaServing::default(),
        ix_config_json: ix_config_json.map(str::to_string),
    }
}

fn release_egress(id: u64) -> Request {
    Request::ReleaseEgress { id }
}

#[test]
fn register_egress_boots_the_plane_and_counts_topics() {
    let (netd, dir, sock, _spy, egress_spy) = start_daemon_with_egress("egboot", LONG_GRACE);
    let (mut a, _) = Client::connect(&sock);

    // First egress plan → the gateway BOOTS (gateway_started true), topics counted.
    match a.request(&register_egress(1, &["/robot/odom", "/robot/imu"])) {
        Response::Egress(e) => {
            assert_eq!(e.id, 1);
            assert_eq!(e.registered_topics, 2);
            assert!(e.gateway_started, "first egress plan boots the gateway");
        }
        other => panic!("expected Egress, got {other:?}"),
    }
    assert_eq!(egress_spy.register_count(), 1);
    assert_eq!(
        egress_spy.registered_topics()[0],
        vec!["/robot/odom".to_string(), "/robot/imu".to_string()]
    );

    // A re-register on the SAME connection ADDS a topic (additive) — gateway already
    // running, so gateway_started is now false and the total grows.
    match a.request(&register_egress(2, &["/robot/scan", "/robot/odom"])) {
        Response::Egress(e) => {
            assert_eq!(e.registered_topics, 3, "additive: only /robot/scan is new");
            assert!(!e.gateway_started, "the gateway is already running");
        }
        other => panic!("expected Egress, got {other:?}"),
    }
    assert_eq!(egress_spy.register_count(), 2);
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

#[test]
fn register_egress_forwards_the_ix_config_json_to_the_plane() {
    // The daemon dispatches the forwarded `ix_config_json` VERBATIM to
    // the egress plane (the multi-process namespace-verification input). Hand oracle:
    // a config-carrying register and a config-less register are BOTH observed by the
    // spy plane exactly as sent — the pass-through the production plane's
    // namespace-check depends on. (The production plane's actual match/mismatch
    // decision over REAL iceoryx2 lives in `egress_plane_iox2_test.rs`.)
    let (netd, dir, sock, _spy, egress_spy) = start_daemon_with_egress("egcfg", LONG_GRACE);
    let (mut a, _) = Client::connect(&sock);

    let mp_cfg = r#"{"global":{"prefix":"iox2_"}}"#;
    match a.request(&register_egress_cfg(1, &["/mp/telemetry"], Some(mp_cfg))) {
        Response::Egress(e) => assert!(e.gateway_started),
        other => panic!("expected Egress, got {other:?}"),
    }
    // A config-LESS register on the same connection (the monolith shape).
    match a.request(&register_egress_cfg(2, &["/mono/status"], None)) {
        Response::Egress(_) => {}
        other => panic!("expected Egress, got {other:?}"),
    }

    // The plane observed BOTH forwards exactly as sent — the daemon threaded the field
    // through untouched (a hardcoded `None` in the dispatch would fail the first arm).
    assert_eq!(
        egress_spy.observed_configs(),
        vec![Some(mp_cfg.to_string()), None],
        "the daemon forwards ix_config_json verbatim (Some then None)"
    );
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

#[test]
fn two_connections_register_egress_independently() {
    let (netd, dir, sock, _spy, egress_spy) = start_daemon_with_egress("egindep", LONG_GRACE);
    let (mut a, _) = Client::connect(&sock);
    let (mut b, _) = Client::connect(&sock);

    // A registers /a1 (boots), B registers /b1 (joins). Each carries ONLY its own plan.
    match a.request(&register_egress(1, &["/a1"])) {
        Response::Egress(e) => assert!(e.gateway_started && e.registered_topics == 1),
        other => panic!("expected Egress: {other:?}"),
    }
    match b.request(&register_egress(1, &["/b1"])) {
        Response::Egress(e) => assert!(!e.gateway_started && e.registered_topics == 1),
        other => panic!("expected Egress: {other:?}"),
    }
    assert_eq!(egress_spy.register_count(), 2);
    assert_eq!(egress_spy.registered_topics()[0], vec!["/a1".to_string()]);
    assert_eq!(egress_spy.registered_topics()[1], vec!["/b1".to_string()]);

    // Both topics are egress-registered — a demand for either loop-conflicts (proves
    // each connection's independent registration landed).
    for t in ["/a1", "/b1"] {
        match a.request(&demand(9, "remote", t, 1)) {
            Response::Error(e) => {
                assert!(e.error.contains("echo-loop"), "loop guard: {}", e.error)
            }
            other => panic!("expected an EgressConflict error for {t}, got {other:?}"),
        }
    }
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

#[test]
fn connection_close_releases_only_its_own_egress() {
    let (netd, dir, sock, _spy, egress_spy) = start_daemon_with_egress("egscope", LONG_GRACE);
    let (mut a, _) = Client::connect(&sock);
    let (mut b, _) = Client::connect(&sock);

    // BOTH connections register the SAME topic /shared (netd does not enforce
    // single-writer — that is the gateway/SHM's job).
    a.request(&register_egress(1, &["/shared"]));
    b.request(&register_egress(1, &["/shared"]));
    assert_eq!(egress_spy.register_count(), 2);

    // A closes its socket → its egress is released (crash-safe), but B still holds
    // /shared, so a demand for /shared STILL loop-conflicts (per-connection scoping:
    // A's disconnect did NOT drop B's registration).
    drop(a);
    assert!(wait_until(
        || egress_spy.release_count() == 1,
        Duration::from_secs(2)
    ));
    let (mut probe, _) = Client::connect(&sock);
    match probe.request(&demand(9, "remote", "/shared", 1)) {
        Response::Error(e) => {
            assert!(
                e.error.contains("echo-loop"),
                "B still holds it: {}",
                e.error
            )
        }
        other => panic!("expected an EgressConflict (B still holds /shared), got {other:?}"),
    }

    // B closes too → both released (release_count 2, the per-connection scoping: each
    // close released exactly its own accounting). /shared now LINGERS-announced
    // — its bridge flag persists until netd idle-exits, so a demand for it is STILL
    // refused EXPLICITLY (never passing the guard to die late in create_ingress_publisher).
    drop(b);
    assert!(wait_until(
        || egress_spy.release_count() == 2,
        Duration::from_secs(2)
    ));
    match probe.request(&demand(10, "remote", "/shared", 1)) {
        Response::Error(e) => assert!(
            e.error.contains("echo-loop"),
            "the lingering announce still blocks the demand: {}",
            e.error
        ),
        other => panic!("expected an EgressConflict (lingering announce), got {other:?}"),
    }
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

#[test]
fn explicit_release_egress_releases_and_relinquishes_the_registration() {
    let (netd, dir, sock, _spy, egress_spy) = start_daemon_with_egress("egrel", LONG_GRACE);
    let (mut a, _) = Client::connect(&sock);
    a.request(&register_egress(1, &["/x"]));

    // Explicit release returns the released count.
    match a.request(&release_egress(2)) {
        Response::EgressRelease(r) => {
            assert_eq!(r.id, 2);
            assert_eq!(r.released_topics, 1);
        }
        other => panic!("expected EgressRelease, got {other:?}"),
    }
    assert_eq!(egress_spy.release_count(), 1);

    // A second release (nothing held) → 0, a benign no-op (no plane release).
    match a.request(&release_egress(3)) {
        Response::EgressRelease(r) => assert_eq!(r.released_topics, 0),
        other => panic!("expected EgressRelease, got {other:?}"),
    }
    assert_eq!(
        egress_spy.release_count(),
        1,
        "no plane release for a no-op"
    );

    // /x LINGERS-announced — a demand for it is STILL refused explicitly (its
    // bridge flag persists until idle-exit), NOT re-opened for mirroring.
    match a.request(&demand(9, "remote", "/x", 1)) {
        Response::Error(e) => assert!(
            e.error.contains("echo-loop"),
            "the lingering announce blocks the demand: {}",
            e.error
        ),
        other => panic!("expected an EgressConflict (lingering announce), got {other:?}"),
    }
    // But the same connection can RE-register /x (revive the egress).
    match a.request(&register_egress(4, &["/x"])) {
        Response::Egress(e) => assert_eq!(e.registered_topics, 1, "re-egress revives it"),
        other => panic!("expected the re-register to succeed, got {other:?}"),
    }
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

#[test]
fn register_egress_plane_failure_is_refused_and_rolled_back() {
    let (netd, dir, sock, _spy, egress_spy) = start_daemon_with_egress("egfail", LONG_GRACE);
    let (mut a, _) = Client::connect(&sock);
    egress_spy.set_fail(true);

    // The plane fails → a loud error, and the registry egress accounting is rolled
    // back (proven by the topic NO LONGER being loop-registered afterward).
    match a.request(&register_egress(1, &["/rollme"])) {
        Response::Error(e) => assert!(
            e.error.contains("egress plan registration failed"),
            "loud failure: {}",
            e.error
        ),
        other => panic!("expected an error, got {other:?}"),
    }
    assert_eq!(
        egress_spy.register_count(),
        1,
        "the plane was attempted once"
    );

    // Rolled back: a demand for /rollme no longer conflicts (a fresh mirror is made).
    egress_spy.set_fail(false);
    match a.request(&demand(9, "remote", "/rollme", 1)) {
        Response::Demand(d) => assert!(d.mirror_created, "rollback freed the topic"),
        other => panic!("expected a Demand (rollback), got {other:?}"),
    }
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

#[test]
fn cross_plan_loop_guard_refuses_egress_of_a_demanded_topic() {
    // DEMAND-then-EGRESS: a topic mirrored IN cannot be egress-registered. The
    // refusal happens in the registry UNDER the lock, BEFORE the plane — so the
    // egress plane is NEVER called for the conflicting topic.
    let (netd, dir, sock, _spy, egress_spy) = start_daemon_with_egress("egloop1", LONG_GRACE);
    let (mut a, _) = Client::connect(&sock);

    a.request(&demand(1, "ubuntu", "/tf", 7)); // mirror /tf IN
    match a.request(&register_egress(2, &["/ok", "/tf"])) {
        Response::Error(e) => {
            assert!(e.error.contains("echo-loop"), "loop guard: {}", e.error);
            assert!(e.error.contains("Stop consuming"));
            assert_eq!(e.topic.as_deref(), Some("/tf"));
            assert_eq!(e.robot.as_deref(), Some("ubuntu"), "names the holder robot");
        }
        other => panic!("expected a LoopConflict error, got {other:?}"),
    }
    assert_eq!(
        egress_spy.register_count(),
        0,
        "the plane is never called for a conflicting plan"
    );
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

#[test]
fn cross_plan_loop_guard_refuses_demand_of_an_egress_topic() {
    // EGRESS-then-DEMAND: a topic produced + announced OUT cannot be mirrored IN.
    let (netd, dir, sock, spy, _egress_spy) = start_daemon_with_egress("egloop2", LONG_GRACE);
    let (mut a, _) = Client::connect(&sock);

    a.request(&register_egress(1, &["/cmd_vel"]));
    match a.request(&demand(2, "go2", "/cmd_vel", 1)) {
        Response::Error(e) => {
            assert!(e.error.contains("echo-loop"), "loop guard: {}", e.error);
            assert_eq!(e.topic.as_deref(), Some("/cmd_vel"));
        }
        other => panic!("expected an EgressConflict error, got {other:?}"),
    }
    assert_eq!(
        spy.ensure_ok_count(),
        0,
        "no mirror ensured for the conflicting demand"
    );
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

#[test]
fn egress_registration_then_disconnect_still_self_exits() {
    // The wedge-class guard at the daemon level: a connection that
    // registered egress keeps netd alive, and after it disconnects (releasing the
    // egress) netd goes idle → self-exits within the grace (no lingering egress
    // wedges the idle-watch).
    let short = Duration::from_millis(150);
    // The connect must land inside the 150 ms grace, so the fixture is
    // rebuilt if the daemon self-exits before the client arrives.
    let (netd, dir, _sock, (_spy, egress_spy), first, _) = start_short_grace_and_connect(|| {
        let (netd, dir, sock, spy, egress_spy) = start_daemon_with_egress("egwedge", short);
        (netd, dir, sock, (spy, egress_spy))
    });
    {
        let mut a = first;
        a.request(&register_egress(1, &["/produced"]));
        // While the connection (with egress) is live, netd does NOT self-exit.
        std::thread::sleep(Duration::from_millis(250));
        assert!(
            !netd.self_exit_requested(),
            "an egress registration pins liveness"
        );
        // a drops here → egress released.
    }
    assert!(wait_until(
        || egress_spy.release_count() == 1,
        Duration::from_secs(2)
    ));
    // Now idle → self-exit fires within the grace.
    assert!(
        wait_until(|| netd.self_exit_requested(), Duration::from_secs(3)),
        "netd self-exits after the egress connection leaves (no wedge)"
    );
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

// ── register_egress edge cases ─────────────────────────────────

#[test]
fn empty_announce_register_egress_is_a_loud_error() {
    // A register_egress with NO produced topics names nothing — refused loudly, the
    // plane never touched.
    let (netd, dir, sock, _spy, egress_spy) = start_daemon_with_egress("egempty", LONG_GRACE);
    let (mut a, _) = Client::connect(&sock);
    match a.request(&register_egress(1, &[])) {
        Response::Error(e) => assert!(
            e.error.contains("non-empty announce set"),
            "names the empty-announce cause: {}",
            e.error
        ),
        other => panic!("expected an error for an empty announce set, got {other:?}"),
    }
    assert_eq!(egress_spy.register_count(), 0, "the plane is never called");
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

#[test]
fn concurrent_first_plans_boot_the_gateway_exactly_once() {
    // FIX 4a race-to-first-plan: TWO connections concurrently register their FIRST
    // egress plan → EXACTLY ONE reports gateway_started (one boot), both plans land.
    let (netd, dir, sock, _spy, egress_spy) = start_daemon_with_egress("egrace", LONG_GRACE);
    let sock2 = sock.clone();
    let barrier = Arc::new(std::sync::Barrier::new(2));
    let b2 = Arc::clone(&barrier);
    let h1 = std::thread::spawn(move || {
        let (mut a, _) = Client::connect(&sock);
        barrier.wait();
        a.request(&register_egress(1, &["/race/a"]))
    });
    let h2 = std::thread::spawn(move || {
        let (mut b, _) = Client::connect(&sock2);
        b2.wait();
        b.request(&register_egress(1, &["/race/b"]))
    });
    let started = [h1.join().unwrap(), h2.join().unwrap()]
        .iter()
        .filter(|r| matches!(r, Response::Egress(e) if e.gateway_started))
        .count();
    assert_eq!(
        started, 1,
        "exactly ONE concurrent first-plan boots the shared gateway"
    );
    assert_eq!(
        egress_spy.register_count(),
        2,
        "both plans reached the plane"
    );
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

#[test]
fn release_then_re_register_same_topic_revives_it() {
    // A topic released by one connection lingers-announced (a demand
    // for it is still refused EXPLICITLY), and a NEW connection re-registering it
    // SUCCEEDS (revival — the mirror plane's lingering-reuse parity).
    let (netd, dir, sock, spy, _egress_spy) = start_daemon_with_egress("egrevive", LONG_GRACE);
    let (mut a, _) = Client::connect(&sock);
    let (mut b, _) = Client::connect(&sock);

    a.request(&register_egress(1, &["/prod"]));
    // A explicitly releases /prod — its announce LINGERS.
    a.request(&release_egress(2));
    // A demand for /prod is STILL refused (the lingering announce blocks it).
    match b.request(&demand(3, "remote", "/prod", 1)) {
        Response::Error(e) => assert!(
            e.error.contains("echo-loop") && e.error.contains("persists until netd idle-exits"),
            "explicit lingering refusal: {}",
            e.error
        ),
        other => panic!("expected an EgressConflict for the lingering topic, got {other:?}"),
    }
    assert_eq!(
        spy.ensure_ok_count(),
        0,
        "no mirror ensured for the refused demand"
    );
    // A DIFFERENT connection RE-registers /prod → revived (succeeds).
    match b.request(&register_egress(4, &["/prod"])) {
        Response::Egress(e) => assert_eq!(e.registered_topics, 1, "revived for b"),
        other => panic!("expected the re-register to succeed (revival), got {other:?}"),
    }
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

#[test]
fn large_register_egress_over_the_old_64k_cap_is_accepted() {
    // A realistic large RegisterEgress (a big schema_serving — a Go2-class
    // bridge's custom-type closure) that EXCEEDS 64 KiB on the request line is
    // ACCEPTED (the cap is 8 MiB), not silently hard-failed. A small announce keeps
    // the egress side trivial; the SIZE comes from the schema_serving.
    let (netd, dir, sock, _spy, egress_spy) = start_daemon_with_egress("egbig", LONG_GRACE);
    let (mut a, _) = Client::connect(&sock);

    // ~2000 catalog bindings → a JSON line well over 64 KiB (and well under 8 MiB).
    let serving = SchemaServing {
        topic_schemas: (0..2000)
            .map(|i| TopicSchema {
                topic: format!("/some/reasonably/long/topic/path/number/{i}"),
                schema_name: "some_pkg/AReasonablyLongCustomMessageTypeName".to_string(),
            })
            .collect(),
        ..Default::default()
    };
    let req = Request::RegisterEgress {
        id: 1,
        plan: GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: vec!["/big/produced".to_string()],
            ingress: vec![],
        },
        schema_serving: serving,
        ix_config_json: None,
    };
    // Sanity: the wire line really is over 64 KiB.
    assert!(
        req.to_json_line().len() > 64 * 1024,
        "the crafted request must exceed 64 KiB"
    );
    match a.request(&req) {
        Response::Egress(e) => {
            assert!(e.gateway_started);
            assert_eq!(e.registered_topics, 1);
        }
        other => panic!("a large-but-legitimate RegisterEgress must be accepted, got {other:?}"),
    }
    assert_eq!(egress_spy.register_count(), 1);
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

// --------------------------------------------------------------------------
// The catalog/schema QUERY verbs (stateless one-shots).
// --------------------------------------------------------------------------

#[test]
fn query_catalog_verb_returns_the_planes_catalogs_and_creates_no_mirror() {
    let (netd, dir, sock, mirror_spy, query_spy) = start_daemon_with_query("qcat", LONG_GRACE);
    query_spy.set_catalogs(vec![
        one_catalog("go2", "/scan", "sensor_msgs/LaserScan"),
        one_catalog("ubuntu", "/tf", "tf2_msgs/TFMessage"),
    ]);
    let (mut c, _) = Client::connect(&sock);
    // A LAN gather (no robot filter) returns every canned catalog VERBATIM.
    match c.request(&query_catalog(1, None)) {
        Response::CatalogQuery(q) => {
            assert_eq!(q.id, 1);
            assert_eq!(q.catalogs.len(), 2);
            // The desk's topic→type lookup resolves against the returned catalogs.
            let tf = q
                .catalogs
                .iter()
                .find(|cat| cat.robot == "ubuntu")
                .and_then(|cat| cat.entries.iter().find(|e| e.topic == "/tf"))
                .and_then(|e| e.schema_name.clone());
            assert_eq!(tf.as_deref(), Some("tf2_msgs/TFMessage"));
        }
        other => panic!("expected CatalogQuery, got {other:?}"),
    }
    // The plane was queried with no robot filter (the LAN gather path).
    assert_eq!(query_spy.catalog_call_count(), 1);
    // STATELESS: a query creates NO mirror and registers NO demand.
    assert_eq!(mirror_spy.ensure_ok_count(), 0, "a query creates no mirror");
    assert_eq!(netd.mirror_count(), 0);
    // A single-robot query forwards the filter.
    match c.request(&query_catalog(2, Some("go2"))) {
        Response::CatalogQuery(_) => {}
        other => panic!("expected CatalogQuery, got {other:?}"),
    }
    assert_eq!(
        query_spy.catalog_robots.lock().unwrap().as_slice(),
        &[None, Some("go2".to_string())]
    );
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

#[test]
fn query_schema_verb_returns_the_planes_replies_verbatim() {
    let (netd, dir, sock, _mirror_spy, query_spy) = start_daemon_with_query("qsch", LONG_GRACE);
    query_spy.set_schemas(vec![SchemaReply::found(
        "ubuntu",
        "tf2_msgs/TFMessage",
        vec![],
    )]);
    let (mut c, _) = Client::connect(&sock);
    match c.request(&query_schema(1, Some("ubuntu"), "tf2_msgs/TFMessage")) {
        Response::SchemaQuery(q) => {
            assert_eq!(q.id, 1);
            assert_eq!(q.replies.len(), 1);
            assert_eq!(q.replies[0].robot, "ubuntu");
            assert_eq!(q.replies[0].requested, "tf2_msgs/TFMessage");
        }
        other => panic!("expected SchemaQuery, got {other:?}"),
    }
    assert_eq!(
        query_spy.schema_calls(),
        vec![(Some("ubuntu".to_string()), "tf2_msgs/TFMessage".to_string())]
    );
    // An empty `requested` is refused with a structured error (never a panic).
    match c.request(&query_schema(2, None, "")) {
        Response::Error(e) => {
            assert_eq!(e.id, Some(2));
            assert!(e.error.contains("non-empty requested type"), "{}", e.error);
        }
        other => panic!("expected Error for empty requested, got {other:?}"),
    }
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

#[test]
fn the_discovery_marker_crosses_the_control_seam_on_both_query_verbs() {
    // The plane's "has a discovery pass completed?" verdict must reach the
    // CONSUMER — otherwise a cold-start empty stays indistinguishable from a settled
    // one on the wire and the desk keeps rendering a terminal "not found" for a
    // topic that exists. Driven over the REAL UDS NDJSON seam against a spy plane
    // (no zenoh), against hand oracles.
    let (netd, dir, sock, _mirror_spy, query_spy) = start_daemon_with_query("qdisc", LONG_GRACE);

    // ARM 1, the SETTLED default (every older arm's meaning): an empty answer is
    // authoritative and says so.
    let (mut c, _) = Client::connect(&sock);
    match c.request(&query_catalog(1, None)) {
        Response::CatalogQuery(q) => {
            assert!(q.catalogs.is_empty());
            assert_eq!(
                q.discovery,
                DiscoveryState::Settled,
                "a settled plane's empty catalog is an authoritative absence"
            );
        }
        other => panic!("expected CatalogQuery, got {other:?}"),
    }
    match c.request(&query_schema(2, None, "pkg/Type")) {
        Response::SchemaQuery(q) => {
            assert!(q.replies.is_empty());
            assert_eq!(q.discovery, DiscoveryState::Settled);
        }
        other => panic!("expected SchemaQuery, got {other:?}"),
    }

    // ARM 2 — THE cold start: netd has completed no discovery pass. The answer is
    // still an empty list (there is genuinely nothing to report) but it now carries
    // the marker that forbids the consumer from calling it absence.
    query_spy.set_cold_start(true);
    match c.request(&query_catalog(3, None)) {
        Response::CatalogQuery(q) => {
            assert_eq!(q.id, 3);
            assert!(q.catalogs.is_empty());
            assert_eq!(
                q.discovery,
                DiscoveryState::NotConverged,
                "a cold plane's empty catalog must NOT read as absence"
            );
        }
        other => panic!("expected CatalogQuery, got {other:?}"),
    }
    match c.request(&query_schema(4, Some("go2"), "pkg/Type")) {
        Response::SchemaQuery(q) => {
            assert_eq!(q.id, 4);
            assert!(q.replies.is_empty());
            assert_eq!(q.discovery, DiscoveryState::NotConverged);
        }
        other => panic!("expected SchemaQuery, got {other:?}"),
    }

    // ARM 3 (anti-tautology) — a cold plane that DID gather something reports the
    // catalogs it found; the marker is about DISCOVERY, not about emptiness, so it
    // must not silently rewrite a non-empty answer.
    query_spy.set_catalogs(vec![CatalogReply::refused("go2", "not authorized")]);
    match c.request(&query_catalog(5, None)) {
        Response::CatalogQuery(q) => {
            assert_eq!(
                q.catalogs.len(),
                1,
                "the gathered reply still rides through"
            );
            assert_eq!(q.discovery, DiscoveryState::NotConverged);
        }
        other => panic!("expected CatalogQuery, got {other:?}"),
    }

    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

#[test]
fn a_query_plane_failure_is_a_structured_error_the_desk_degrades_on() {
    // A netd that cannot RUN the query (its plane errs) answers with a structured
    // error — NOT an empty result — so the desk can distinguish "netd unreachable"
    // (fall back to its own session) from "netd reached the LAN, nobody answered".
    let (netd, dir, sock, _mirror_spy, query_spy) = start_daemon_with_query("qfail", LONG_GRACE);
    query_spy.set_fail(true);
    let (mut c, _) = Client::connect(&sock);
    match c.request(&query_catalog(1, None)) {
        Response::Error(e) => {
            assert_eq!(e.id, Some(1));
            assert!(e.error.contains("catalog query failed"), "{}", e.error);
        }
        other => panic!("expected Error, got {other:?}"),
    }
    match c.request(&query_schema(2, None, "pkg/Type")) {
        Response::Error(e) => {
            assert!(
                e.error.contains("schema query for 'pkg/Type' failed"),
                "{}",
                e.error
            );
        }
        other => panic!("expected Error, got {other:?}"),
    }
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

#[test]
fn a_serve_side_refusal_rides_through_the_query_response_loudly() {
    // A queried robot's authorizer REFUSAL is a decodable
    // CatalogReply with `error: Some` + empty entries — the query response carries it
    // VERBATIM so the desk surfaces the explicit refusal (never a silent empty).
    let (netd, dir, sock, _mirror_spy, query_spy) = start_daemon_with_query("qref", LONG_GRACE);
    query_spy.set_catalogs(vec![CatalogReply::refused(
        "go2",
        "not authorized (account/pairing)",
    )]);
    let (mut c, _) = Client::connect(&sock);
    match c.request(&query_catalog(1, Some("go2"))) {
        Response::CatalogQuery(q) => {
            assert_eq!(q.catalogs.len(), 1);
            assert_eq!(
                q.catalogs[0].error.as_deref(),
                Some("not authorized (account/pairing)")
            );
            assert!(q.catalogs[0].entries.is_empty(), "a refusal serves nothing");
        }
        other => panic!("expected CatalogQuery carrying the refusal, got {other:?}"),
    }
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

#[test]
fn queries_are_not_refcounted_and_leave_no_demand_on_disconnect() {
    // A query connection counts as live only while open (keeps netd busy), but holds
    // NO demand — so `status` shows an empty demand table during the query, and a
    // disconnect tears down NOTHING (no mirror release).
    let (netd, dir, sock, mirror_spy, query_spy) =
        start_daemon_with_query("qstateless", LONG_GRACE);
    query_spy.set_catalogs(vec![one_catalog("go2", "/scan", "sensor_msgs/LaserScan")]);
    {
        let (mut c, _) = Client::connect(&sock);
        let _ = c.request(&query_catalog(1, None));
        // The connection is live (keeps netd from idling) but holds no demand.
        assert!(wait_until(
            || netd.active_connections() == 1,
            Duration::from_secs(2)
        ));
        match c.request(&Request::Status { id: 2 }) {
            Response::Status(s) => {
                assert!(s.demands.is_empty(), "a query registers no demand");
                assert_eq!(s.active_connections, 1);
            }
            other => panic!("expected Status, got {other:?}"),
        }
    } // connection dropped here.
      // Disconnect released nothing (a query held no mirror).
    assert!(wait_until(
        || netd.active_connections() == 0,
        Duration::from_secs(2)
    ));
    assert_eq!(
        mirror_spy.release_count(),
        0,
        "a query disconnect tears down no mirror"
    );
    assert_eq!(netd.mirror_count(), 0);
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

// --------------------------------------------------------------------------
// A response line larger than the send buffer must round-trip
// UN-truncated. The hazard (a 75-topic catalog is enough to reach it): a daemon
// writing each NDJSON line with `writeln!` == `write_all` aborts on the
// FIRST `WouldBlock`. On macOS the accepted socket inherits O_NONBLOCK from the
// nonblocking listener, so once the ~8 KiB UDS send buffer fills the next
// `write` returns `WouldBlock` instantly and such a daemon drops the connection
// after ~one sndbuf — the client reads a truncated 8192-byte line then EOF
// ("EOF while parsing a string at line 1 column 8192"). The daemon RESUMES the
// partial write. These pins drive REAL daemon state (hundreds of genuine
// demands → a big `status` line, Principle #13 — no fabricated payload) through
// the production write path.
// --------------------------------------------------------------------------

/// A ~180-byte topic keyed by `i`, so a few hundred demands push the `status`
/// line well past 8 KiB / 64 KiB (predictable, distinct per index).
fn big_topic(i: usize) -> String {
    format!(
        "/big/status/line/topic/{i:06}/padding/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb/cccccccccccccccccccc"
    )
}

/// A ~900-byte topic keyed by `i` — for the block-regime tests, whose reply must
/// exceed the LARGER platform send buffer (Linux `SO_SNDBUF` 212992 ≈ 208 KiB,
/// measured on Linux) so the daemon's write genuinely BLOCKS on both platforms,
/// not just macOS's ~8 KiB. See the arithmetic at each call site.
fn wide_topic(i: usize) -> String {
    // ~30-char prefix + `{i:06}` + a 900-char 'a' pad ≈ 936 chars.
    format!("/wide/block/topic/{i:06}/{}", "a".repeat(900))
}

/// Register `n` distinct demands on `c` (reading each small ack), then `status`
/// (with a deterministic reader lag so the daemon's send buffer fills BEFORE the
/// client drains — the macOS truncation repro) and return the FULL response line.
fn register_demands_then_read_big_status(c: &mut Client, n: usize) -> String {
    for i in 0..n {
        match c.request(&demand(i as u64, "bigrobot", &big_topic(i), 7)) {
            Response::Demand(d) => assert_eq!(d.refcount, 1, "each is a fresh first-demand"),
            other => panic!("expected Demand ack, got {other:?}"),
        }
    }
    // Send the status request, then LAG the reader briefly. On a non-resuming daemon
    // (macOS nonblocking socket) the write aborts in microseconds — long before
    // this lag ends — so the line arrives truncated + un-terminated. The lag
    // (250 ms) is far under the 2 s no-progress budget, so the resuming daemon simply
    // resumes once we start reading. On Linux (blocking + SO_SNDTIMEO) the write
    // blocks through the lag and completes on both code paths — the pin is a
    // no-truncation guarantee there and the active regression catch on macOS.
    c.send_line(&Request::Status { id: 424_242 }.to_json_line());
    std::thread::sleep(Duration::from_millis(250));
    c.read_line().expect("read the status response line")
}

/// Assert the response `line` is a COMPLETE (newline-terminated) `status` reply of
/// at least `min_bytes`, carrying exactly `n` demands — i.e. NOT truncated.
fn assert_full_status(line: &str, min_bytes: usize, n: usize) {
    assert!(
        line.ends_with('\n'),
        "truncated: the response is not newline-terminated (got {} bytes, no '\\n' — the \
         send-buffer-abort shape)",
        line.len()
    );
    assert!(
        line.len() > min_bytes,
        "the response must exceed {min_bytes} bytes to exercise the >send-buffer path (got {}) — \
         the pin cannot silently shrink below the boundary",
        line.len()
    );
    match serde_json::from_str::<Response>(line.trim()).expect("the FULL line parses (untruncated)")
    {
        Response::Status(s) => assert_eq!(
            s.demands.len(),
            n,
            "every demanded row survived the big write"
        ),
        other => panic!("expected Status, got {other:?}"),
    }
}

#[test]
fn big_status_line_over_8k_round_trips_untruncated() {
    let (netd, dir, sock, _spy) = start_daemon("big8k", LONG_GRACE);
    let (mut c, _) = Client::connect(&sock);
    // ~60 × ~180-byte topics ≫ the 8192-byte UDS send buffer.
    const N: usize = 60;
    let line = register_demands_then_read_big_status(&mut c, N);
    assert_full_status(&line, 8192, N);
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

#[test]
fn big_status_line_over_64k_round_trips_untruncated() {
    // The >64 KiB variant: the write spans MANY send-buffer drains (each ~8 KiB),
    // so the resume loop is exercised across dozens of `WouldBlock`/poll cycles.
    let (netd, dir, sock, _spy) = start_daemon("big64k", LONG_GRACE);
    let (mut c, _) = Client::connect(&sock);
    const N: usize = 360; // × ~180 bytes ≈ 70+ KiB.
    let line = register_demands_then_read_big_status(&mut c, N);
    assert_full_status(&line, 64 * 1024, N);
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

/// Register `k` distinct WIDE demands (932-byte topics) on `c`, so the resulting
/// `status` reply exceeds BOTH platforms' UDS send buffers and the write genuinely
/// BLOCKS on either. Arithmetic (a K=60 ≈ 15 KiB reply fits inside
/// Linux's ~208 KiB buffer, so the write completes and the dead consumer is never
/// dropped there): [`wide_topic`] is 932 bytes, so each
/// `DemandEntry` JSON `{"robot":"deadrobot","topic":"<932>","refcount":1}` ≈ 45 + 932
/// ≈ 977 bytes, and K=400 ⇒ ~382 KiB. That is ≫ 212992 (Linux `SO_SNDBUF`, as
/// measured; effective ≈ 95 KiB) AND ≫ 8192 (macOS `SO_SNDBUF`, as
/// measured), so the reply is un-bufferable on both and the write BLOCKS — the only
/// regime that surfaces a stalled consumer.
const BLOCK_REGIME_DEMANDS: usize = 400;

fn register_wide_demands(c: &mut Client, robot: &str, k: usize) {
    for i in 0..k {
        match c.request(&demand(i as u64, robot, &wide_topic(i), 7)) {
            Response::Demand(d) => assert_eq!(d.refcount, 1, "each is a fresh first-demand"),
            other => panic!("expected Demand ack, got {other:?}"),
        }
    }
}

#[test]
fn dead_consumer_is_still_dropped_within_the_no_progress_timeout() {
    // The protective behavior MUST survive: a consumer that requests a big response
    // and then NEVER reads it (its send buffer stays full → the daemon can place no
    // further bytes) is dropped once no forward progress happens for the whole
    // no-progress budget — releasing every demand it held (the crash-safe refcount).
    let (netd, dir, sock, spy) = start_daemon("deadcons", LONG_GRACE);
    let (mut c, _) = Client::connect(&sock);
    const K: usize = BLOCK_REGIME_DEMANDS;
    register_wide_demands(&mut c, "deadrobot", K);
    assert_eq!(
        spy.ensure_ok_count(),
        K,
        "K first-demands → K mirrors ensured"
    );

    // Ask for the big status, then STOP reading (hold the stream open, drain
    // nothing). The daemon fills the buffer, makes no further progress, and after
    // CONN_WRITE_TIMEOUT (2 s) drops the connection → cleanup releases all K.
    c.send_line(&Request::Status { id: 7 }.to_json_line());

    // Bounded well past the 2 s budget (+ poll slack). The dead consumer's demands
    // are released exactly once each on the drop — the survival oracle.
    assert!(
        wait_until(|| spy.release_count() >= K, Duration::from_secs(8)),
        "the dead consumer was NOT dropped within the no-progress timeout \
         (released {} of {K})",
        spy.release_count()
    );
    assert_eq!(
        spy.release_count(),
        K,
        "each held demand released exactly once"
    );
    assert!(wait_until(
        || netd.active_connections() == 0,
        Duration::from_secs(2)
    ));
    // Keep the stream alive until here so the drop is the daemon's decision, not a
    // client-side close.
    drop(c);
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

#[test]
fn consumer_that_reads_part_of_a_big_reply_then_drops_is_cleaned_up() {
    // The e2e twin: a controller reads only PART of a >buffer reply then
    // CLOSES mid-stream. The daemon's in-flight write hits EPIPE/ECONNRESET → the
    // composed loop's IoError/PeerClosed arm drops the connection and releases every
    // demand it held — exactly once each, in a BOUNDED window (no 2 s no-progress
    // wait: a peer close is an immediate error, distinct from a silently-wedged
    // reader). Reuses the block-regime sizing so the write is genuinely in progress
    // when the client drops.
    use std::io::Read;
    let (netd, dir, sock, spy) = start_daemon("partread", LONG_GRACE);
    let (mut c, _) = Client::connect(&sock);
    const K: usize = BLOCK_REGIME_DEMANDS;
    register_wide_demands(&mut c, "partrobot", K);
    assert_eq!(spy.ensure_ok_count(), K);

    // Send the big status, read a small prefix (so the write is mid-flight), then
    // DROP the client — closing the read end mid-reply.
    c.send_line(&Request::Status { id: 9 }.to_json_line());
    let mut sink = [0u8; 4096];
    let got = c
        .reader
        .read(&mut sink)
        .expect("read a prefix of the reply");
    assert!(
        got > 0,
        "the daemon delivered at least one send-buffer's worth"
    );
    drop(c); // close mid-reply → the daemon's next write EPIPEs.

    // The daemon detects the closed peer and releases all K — bounded, no hang.
    assert!(
        wait_until(|| spy.release_count() >= K, Duration::from_secs(6)),
        "a mid-read client drop did NOT release the held demands \
         (released {} of {K})",
        spy.release_count()
    );
    assert_eq!(
        spy.release_count(),
        K,
        "each held demand released exactly once on the mid-read close"
    );
    assert!(wait_until(
        || netd.active_connections() == 0,
        Duration::from_secs(2)
    ));
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

// --------------------------------------------------------------------------
// An IDLE connection's read loop is PACED, not spinning.
//
// Measured: `cerulion-netd` at 198.9 % CPU with an EMPTY demand table
// and nothing being viewed — two idle connection threads, each running the loop
// above as fast as the kernel could return `EAGAIN`, because the socket accepted
// from the nonblocking listener inherited `O_NONBLOCK` (macOS/BSD) and the
// `set_read_timeout` that was supposed to pace the loop never cleared it.
//
// The oracle is an UPPER bound on loop iterations over a held-idle window. An
// upper bound is LOAD-IMMUNE in the right direction: every iteration costs a
// blocking read of `CONN_READ_TIMEOUT`, so a slow/contended runner can only
// produce FEWER, never more (the inverse of the trap, where a ceiling on
// *work* failed open under load). The spinner it must catch overshoots by ~5
// orders of magnitude, not by a factor.
//
// PLATFORM SCOPE of the mutation evidence: deleting `set_nonblocking(false)` is a
// behavioural no-op on LINUX — `accept(2)` inherits no file status flags there, so
// the accepted socket is already blocking and this arm stays green. The
// 1.7M/5.4M-iteration kill is a macOS result, and `test-macos` is the job that can
// reproduce it. The arm is NOT vacuous on Linux: dropping `set_read_timeout` from
// the same seam still fails it (the read blocks forever, tripping `delta >= 2`),
// and the ceiling remains a live guard against any Linux-reachable spin.
// --------------------------------------------------------------------------

#[test]
fn an_idle_connections_read_loop_is_paced_by_the_read_timeout_not_spinning() {
    let (netd, dir, sock, _spy) = start_daemon("pace", LONG_GRACE);
    let (mut c, hello) = Client::connect(&sock);
    assert_eq!(
        hello.protocol, PROTOCOL_VERSION,
        "the banner proves the handler thread really served this connection"
    );

    // Serve one request so the handler is provably INSIDE its read loop (not still
    // in accept) before the measured window opens.
    assert!(
        matches!(c.request(&Request::Status { id: 1 }), Response::Status(_)),
        "status must round-trip before the idle window"
    );

    const HOLD: Duration = Duration::from_millis(1500);
    let before = netd.conn_read_loop_iterations();
    std::thread::sleep(HOLD);
    let delta = netd.conn_read_loop_iterations() - before;

    // Hand oracle: one iteration per CONN_READ_TIMEOUT ⇒ 1500/200 = 7 (÷ any number
    // of runner stalls). The ceiling is 5× that — unreachable by timer jitter, and
    // ~5 orders of magnitude below a spinner (MEASURED on a spinning loop: reads returned in
    // ~0.6 µs each, i.e. millions of iterations in this window).
    let expected = HOLD.as_nanos() / daemon::CONN_READ_TIMEOUT.as_nanos();
    let ceiling = (expected * 5) as u64;
    assert!(
        delta <= ceiling,
        "an IDLE connection went round its read loop {delta} times in {HOLD:?} \
         (expected ~{expected}, ceiling {ceiling}) — the accepted socket is \
         nonblocking again and the handler is SPINNING"
    );
    // Anti-vacuity: the loop must actually be RUNNING. Without this, a handler that
    // died (or never started) would satisfy the ceiling with a delta of 0.
    assert!(
        delta >= 2,
        "the read loop ran only {delta} times in {HOLD:?} — the handler is not alive, \
         so the ceiling above proves nothing"
    );

    // And the connection is still SERVING after the idle window (not merely alive).
    assert!(
        matches!(c.request(&Request::Status { id: 2 }), Response::Status(_)),
        "the connection must still serve requests after an idle window"
    );

    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

/// The plane's un-settled AGE crosses the control seam on BOTH query verbs.
///
/// The twin of `the_discovery_marker_crosses_the_control_seam_on_both_query_verbs`,
/// and for the same reason: the consumer's first-contact cap is computed from this
/// number, so if the daemon drops it on the floor the cap is vacuously satisfied and a
/// robot-less desk pays the ceiling on EVERY command forever — the regression the gate
/// exists to prevent. With both spies hard-coding `None`, `daemon.rs`'s
/// `.map(|d| d.as_millis() as u64)` is only ever exercised on its `None` branch and
/// replacing the whole expression with a literal `None` is invisible.
///
/// Three arms: a plane with no age to report (an older-daemon shape) sends the field
/// ABSENT rather than a fabricated zero; a plane reporting an age puts THAT age on the
/// wire, on both verbs; and the value is carried verbatim, not rounded to a marker.
#[test]
fn the_plane_unsettled_age_crosses_the_control_seam_on_both_query_verbs() {
    let (mut netd, dir, sock, _mirror_spy, query_spy) = start_daemon_with_query("qage", LONG_GRACE);
    let (mut c, _) = Client::connect(&sock);

    // ARM 1 — no age to report ⇒ the field is ABSENT (`None`), never a fabricated 0.
    // A zero would be a positive claim ("this plane just started"), which is exactly
    // the reading the consumer must not be handed by a daemon that cannot say.
    query_spy.set_unsettled_ms(None);
    match c.request(&query_catalog(1, None)) {
        Response::CatalogQuery(q) => assert_eq!(
            q.plane_unsettled_ms, None,
            "a plane with no age must send the field absent, not a fabricated zero"
        ),
        other => panic!("expected CatalogQuery, got {other:?}"),
    }

    // ARM 2 — a plane that has been trying for 12_345 ms puts THAT number on the wire.
    query_spy.set_unsettled_ms(Some(12_345));
    match c.request(&query_catalog(2, None)) {
        Response::CatalogQuery(q) => assert_eq!(
            q.plane_unsettled_ms,
            Some(12_345),
            "the catalog verb must carry the plane's age verbatim — the client's \
             first-contact cap is computed from it"
        ),
        other => panic!("expected CatalogQuery, got {other:?}"),
    }

    // ARM 3 — the SCHEMA verb carries it too, or `cerulion schema info` is uncapped.
    match c.request(&query_schema(3, None, "pkg/Type")) {
        Response::SchemaQuery(q) => assert_eq!(
            q.plane_unsettled_ms,
            Some(12_345),
            "the schema verb carries the same quantity"
        ),
        other => panic!("expected SchemaQuery, got {other:?}"),
    }

    drop(c);
    netd.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// --------------------------------------------------------------------------
// The `runs` verb over the REAL UDS seam.
// --------------------------------------------------------------------------

/// The verb reaches the plane, scopes the GET the way the caller asked, and hands the
/// robot's reply back VERBATIM — including the two DOCUMENTS, which are the payload
/// the whole verb exists to move.
///
/// Also pins the STATELESS half in the same body: a query creates no mirror and is
/// never refcounted, so `status` must still report an empty demand table afterwards.
/// Without that, a `runs` handler that accidentally took the registry lock or minted
/// a demand would look identical from the reply alone.
#[test]
fn the_runs_verb_returns_the_planes_replies_verbatim_and_creates_no_mirror() {
    let (mut netd, dir, sock, _mirror_spy, query_spy) =
        start_daemon_with_query("qruns", LONG_GRACE);
    query_spy.set_runs(vec![one_runs_reply("go2", "0x2a", "perception")]);

    let (mut c, _) = Client::connect(&sock);

    // Fan-out (`robot: None`) — every announcing robot.
    match c.request(&query_runs(1, None)) {
        Response::RunsQuery(q) => {
            assert_eq!(q.id, 1);
            assert_eq!(q.run_replies.len(), 1);
            let reply = &q.run_replies[0];
            assert_eq!(reply.robot, "go2");
            assert_eq!(reply.runs.len(), 1);
            assert_eq!(reply.runs[0].run_id, "0x2a");
            assert_eq!(reply.runs[0].graph_name, "perception");
            // The DOCUMENTS crossed intact — the desk re-parses these with the same
            // parser the robot used, so a dropped or mangled one is a wrong graph.
            assert_eq!(
                reply.runs[0].graph_yaml,
                "name: perception\nnodes:\n  - id: cam\n"
            );
            assert_eq!(reply.runs[0].run_json, "{\"run_id\":\"0x2a\"}\n");
            assert!(reply.completeness.is_settled());
            assert_eq!(q.discovery, DiscoveryState::Settled);
        }
        other => panic!("expected RunsQuery, got {other:?}"),
    }

    // Single-robot scoping reaches the plane as `Some(robot)` — the desk's
    // fetch-one-run shape. Asserted on the SPY, which is the only place the scope is
    // observable (both shapes return the same canned reply).
    match c.request(&query_runs(2, Some("orin"))) {
        Response::RunsQuery(q) => assert_eq!(q.id, 2),
        other => panic!("expected RunsQuery, got {other:?}"),
    }
    assert_eq!(
        query_spy.runs_robots(),
        vec![None, Some("orin".to_string())],
        "the verb must scope the GET the caller asked for, in order"
    );

    // STATELESS: no mirror, no refcount.
    match c.request(&Request::Status { id: 3 }) {
        Response::Status(s) => assert!(
            s.demands.is_empty(),
            "a runs query must create no demand: {:?}",
            s.demands
        ),
        other => panic!("expected Status, got {other:?}"),
    }

    drop(c);
    netd.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// The two verdicts a runs answer carries are INDEPENDENT, and both must reach the
/// consumer or an empty list becomes a confident lie.
///
/// `discovery` is about netd's own LAN session ("was anything searched?"); a reply's
/// `completeness` is about that ROBOT's registry gather ("did I hear every live run
/// writer?"). The three arms drive them apart: a settled plane with an empty list,
/// a COLD plane with an empty list (same bytes for `run_replies`, opposite meaning),
/// and a cold plane that DID gather — so the marker cannot be read as "the answer
/// was empty".
#[test]
fn the_discovery_marker_and_the_reply_completeness_are_independent() {
    let (mut netd, dir, sock, _mirror_spy, query_spy) =
        start_daemon_with_query("qrunsd", LONG_GRACE);
    let (mut c, _) = Client::connect(&sock);

    // ARM 1 — settled plane, nothing gathered: an authoritative "no robot answered".
    match c.request(&query_runs(1, None)) {
        Response::RunsQuery(q) => {
            assert!(q.run_replies.is_empty());
            assert_eq!(q.discovery, DiscoveryState::Settled);
        }
        other => panic!("expected RunsQuery, got {other:?}"),
    }

    // ARM 2 — COLD plane, nothing gathered: byte-identical `run_replies`, and the
    // marker is the ONLY thing that forbids reading it as absence.
    query_spy.set_cold_start(true);
    match c.request(&query_runs(2, None)) {
        Response::RunsQuery(q) => {
            assert!(q.run_replies.is_empty());
            assert_eq!(
                q.discovery,
                DiscoveryState::NotConverged,
                "a cold plane's empty runs answer must NOT read as absence"
            );
        }
        other => panic!("expected RunsQuery, got {other:?}"),
    }

    // ARM 3 (anti-tautology) — a cold plane that DID gather still reports what it
    // found, and the reply's OWN non-settled verdict rides through beside the
    // plane's. Two different questions, two different answers, on one line.
    let mut incomplete = one_runs_reply("go2", "0x2a", "perception");
    incomplete.completeness = cerulion_core::RunsCompleteness::Incomplete {
        live_writers: 2,
        writers_heard: 1,
    };
    query_spy.set_runs(vec![incomplete]);
    match c.request(&query_runs(3, None)) {
        Response::RunsQuery(q) => {
            assert_eq!(q.run_replies.len(), 1, "the marker must not empty a result");
            assert_eq!(q.discovery, DiscoveryState::NotConverged);
            assert_eq!(
                q.run_replies[0].completeness,
                cerulion_core::RunsCompleteness::Incomplete {
                    live_writers: 2,
                    writers_heard: 1,
                },
                "the ROBOT's own gather verdict is a separate fact and must survive"
            );
        }
        other => panic!("expected RunsQuery, got {other:?}"),
    }

    // The plane age rides this verb too (it is what caps a consumer's wait).
    query_spy.set_unsettled_ms(Some(9_876));
    match c.request(&query_runs(4, None)) {
        Response::RunsQuery(q) => assert_eq!(q.plane_unsettled_ms, Some(9_876)),
        other => panic!("expected RunsQuery, got {other:?}"),
    }

    drop(c);
    netd.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// A robot that ANSWERED UNUSABLY is reported APART from the robots
/// that did not answer, because the two absences have opposite remedies.
///
/// Both render as "no runs from that robot", and collapsing them sends an operator
/// to wait out a wire skew that will never converge. The arm drives the sharp shape
/// — a skewed robot BESIDE a healthy one — so the skew cannot be inferred from the
/// answer being empty, and pins that the healthy robot's runs still cross.
#[test]
fn a_robot_that_answered_unusably_is_named_apart_from_one_that_did_not_answer() {
    let (mut netd, dir, sock, _mirror_spy, query_spy) =
        start_daemon_with_query("qrunsu", LONG_GRACE);
    query_spy.set_runs(vec![one_runs_reply("go2", "0x2a", "perception")]);
    query_spy.set_unusable_runs(vec![UnusableRunsAnswer {
        robot: "orin".to_string(),
        reason: "unknown runs wire version 2 (this binary supports 1)".to_string(),
    }]);

    let (mut c, _) = Client::connect(&sock);
    match c.request(&query_runs(1, None)) {
        Response::RunsQuery(q) => {
            // The healthy robot's answer is untouched — one robot's skew does not
            // suppress another's runs.
            assert_eq!(q.run_replies.len(), 1);
            assert_eq!(q.run_replies[0].robot, "go2");
            // …and the skewed one is NAMED, with the reason an operator acts on.
            assert_eq!(q.unusable.len(), 1);
            assert_eq!(q.unusable[0].robot, "orin");
            assert!(
                q.unusable[0].reason.contains("unknown runs wire version"),
                "the reason must reach the operator: {}",
                q.unusable[0].reason
            );
            // A skew is not a discovery failure — the LAN was reached.
            assert_eq!(q.discovery, DiscoveryState::Settled);
        }
        other => panic!("expected RunsQuery, got {other:?}"),
    }

    // ANTI-TAUTOLOGY: a healthy LAN reports NOTHING unusable, so the list above is
    // the plane's answer and not a field this daemon always populates.
    query_spy.set_unusable_runs(vec![]);
    match c.request(&query_runs(2, None)) {
        Response::RunsQuery(q) => {
            assert!(q.unusable.is_empty());
            assert_eq!(q.run_replies.len(), 1);
        }
        other => panic!("expected RunsQuery, got {other:?}"),
    }

    drop(c);
    netd.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// The COVERAGE half crosses the seam: which asked robots stayed
/// SILENT — so `run_replies` is readable as coverage and not merely as content.
///
/// Without it a consumer holding replies and unusable answers cannot tell a LAN
/// where everyone answered from one where half stayed quiet: both arrive as a list
/// of replies, and folding either into "these are the runs" makes a SETTLED ABSENCE
/// claim about machines nobody heard from. That is the confident-empty class the
/// `discovery` marker closes at the LATCH, reached instead through coverage — which
/// is why the arm holds `discovery` at `Settled` throughout: the LAN really was
/// searched, and the answer is still short.
#[test]
fn the_silent_robots_cross_the_seam_so_replies_are_readable_as_coverage() {
    let (mut netd, dir, sock, _mirror_spy, query_spy) =
        start_daemon_with_query("qrunssi", LONG_GRACE);
    query_spy.set_runs(vec![one_runs_reply("go2", "0x2a", "perception")]);
    query_spy.set_silent_runs(vec!["spot".to_string(), "orin".to_string()]);

    let (mut c, _) = Client::connect(&sock);
    match c.request(&query_runs(1, None)) {
        Response::RunsQuery(q) => {
            // The robot that ANSWERED is served in full…
            assert_eq!(q.run_replies.len(), 1);
            assert_eq!(q.run_replies[0].robot, "go2");
            // …and the two that did not are NAMED, so the answer's shortfall is
            // visible rather than inferable only by counting.
            assert_eq!(q.silent, vec!["spot".to_string(), "orin".to_string()]);
            // A silent robot is neither a skew nor a discovery failure.
            assert!(q.unusable.is_empty());
            assert_eq!(q.discovery, DiscoveryState::Settled);
        }
        other => panic!("expected RunsQuery, got {other:?}"),
    }

    // ANTI-TAUTOLOGY: a fully-covered LAN reports NOTHING silent, so the list above
    // is the plane's answer rather than a field this daemon always populates.
    query_spy.set_silent_runs(vec![]);
    match c.request(&query_runs(2, None)) {
        Response::RunsQuery(q) => {
            assert!(q.silent.is_empty());
            assert_eq!(q.run_replies.len(), 1);
        }
        other => panic!("expected RunsQuery, got {other:?}"),
    }

    drop(c);
    netd.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// A plane that could not run the query is a structured ERROR, never an empty answer.
///
/// The distinction is the consumer's whole fallback decision: an empty `run_replies`
/// means "netd asked and nobody answered", while an error means "netd could not ask",
/// and collapsing the second into the first would report a desk-local fault as a
/// property of every robot on the LAN.
#[test]
fn a_plane_that_cannot_run_the_query_is_an_error_not_an_empty_answer() {
    let (mut netd, dir, sock, _mirror_spy, query_spy) =
        start_daemon_with_query("qrunsf", LONG_GRACE);
    query_spy.set_runs(vec![one_runs_reply("go2", "0x2a", "perception")]);
    query_spy.set_fail(true);

    let (mut c, _) = Client::connect(&sock);
    match c.request(&query_runs(1, Some("go2"))) {
        Response::Error(e) => {
            assert_eq!(e.id, Some(1));
            assert!(
                e.error.contains("runs query failed"),
                "the error must name the verb that failed: {}",
                e.error
            );
            assert_eq!(
                e.robot.as_deref(),
                Some("go2"),
                "a robot-scoped query reports which robot it was for"
            );
        }
        other => panic!("expected an Error response, got {other:?}"),
    }

    // ANTI-TAUTOLOGY: the same plane, healed, answers normally — so the arm above is
    // about the failure and not about a daemon that never serves this verb.
    query_spy.set_fail(false);
    match c.request(&query_runs(2, Some("go2"))) {
        Response::RunsQuery(q) => assert_eq!(q.run_replies.len(), 1),
        other => panic!("expected RunsQuery, got {other:?}"),
    }

    drop(c);
    netd.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn raw_egress_login_policy_rechecks_before_registration_and_plane_calls() {
    use cerulion_netd::serving_login::{EgressLoginPolicy, REFUSAL};
    let login = tempfile::tempdir().unwrap();
    let auth_path = login.path().join("auth.json");
    let (netd, dir, sock, _mirror, plane) = start_daemon_with_egress_policy(
        "eglogin",
        LONG_GRACE,
        EgressLoginPolicy::PriorLogin {
            auth_path: Some(auth_path.clone()),
        },
    );
    let (mut client, _) = Client::connect(&sock);
    for id in [1, 2] {
        let reply = client.request(&register_egress(id, &["/refused"]));
        assert_eq!(reply, Response::error(Some(id), REFUSAL, None, None));
    }
    assert_eq!(plane.register_count(), 0);
    match client.request(&release_egress(3)) {
        Response::EgressRelease(reply) => assert_eq!(reply.released_topics, 0),
        reply => panic!("unexpected reply: {reply:?}"),
    }
    assert!(!auth_path.exists());
    let expired = br#"{"account_id":"saved-account","session_token":"expired-session","refresh_token":"expired-refresh","expires_at_ns":1,"logged_in_ever":true}"#;
    std::fs::write(&auth_path, expired).unwrap();
    match client.request(&register_egress(4, &["/accepted"])) {
        Response::Egress(reply) => {
            assert_eq!(reply.registered_topics, 1);
            assert!(reply.gateway_started);
        }
        reply => panic!("unexpected reply: {reply:?}"),
    }
    assert_eq!(plane.register_count(), 1);
    assert_eq!(std::fs::read(&auth_path).unwrap(), expired);
    for invalid in [None, Some(&b"{"[..]), Some(&b"{}"[..])] {
        if let Some(bytes) = invalid {
            std::fs::write(&auth_path, bytes).unwrap();
        } else {
            std::fs::remove_file(&auth_path).unwrap();
        }
        assert_eq!(
            client.request(&register_egress(5, &["/later"])),
            Response::error(Some(5), REFUSAL, None, None)
        );
    }
    assert_eq!(plane.register_count(), 1);
    match client.request(&release_egress(6)) {
        Response::EgressRelease(reply) => assert_eq!(reply.released_topics, 1),
        reply => panic!("unexpected reply: {reply:?}"),
    }
    drop(client);
    drop(netd);
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn raw_egress_requires_a_resolved_home_when_policy_is_enabled() {
    use cerulion_netd::serving_login::{EgressLoginPolicy, REFUSAL};
    let (netd, dir, sock, _mirror, plane) = start_daemon_with_egress_policy(
        "eghome",
        LONG_GRACE,
        EgressLoginPolicy::PriorLogin { auth_path: None },
    );
    let (mut client, _) = Client::connect(&sock);
    assert_eq!(
        client.request(&register_egress(1, &["/refused"])),
        Response::error(Some(1), REFUSAL, None, None)
    );
    assert_eq!(plane.register_count(), 0);
    drop(client);
    drop(netd);
    std::fs::remove_dir_all(dir).unwrap();
}
