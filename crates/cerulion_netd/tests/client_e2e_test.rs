// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion-netd` CLIENT e2e — the consumer half of the
//! demand plane, driven against a REAL daemon.
//!
//! Two harnesses, both hand-oracle-based (never a self-compare):
//!
//! - **connect-to-running** (the common path): an IN-PROCESS daemon
//!   ([`daemon::start`]) with a COUNTING SPY [`MirrorPlane`] over a unique temp
//!   socket. The client CONNECTS (no spawn — a daemon is already listening), does
//!   the hello handshake + `demand`/`release`, and — the pairing guarantee — a
//!   DROP of the client releases every demand it held (the daemon's crash-safe
//!   connection-close refcount). Parallel-safe (unique socket per test, spy plane,
//!   no env, no iceoryx2).
//! - **spawn-a-real-daemon** (the first-consumer-spawns path): points
//!   [`NETD_BIN_ENV`] at the freshly-BUILT `cerulion-netd` binary
//!   (`CARGO_BIN_EXE_cerulion-netd`), `CERULION_NETD_NETWORK=off` (local-only, no
//!   zenoh), and lets the client SPAWN it detached, connect, and `demand` — which,
//!   with no network, surfaces the structured mirror-registration error. That
//!   round-trip proves spawn→connect→hello→demand→response over a REAL spawned
//!   daemon PROCESS, hermetically (no robot). It mutates process env, so it holds a
//!   file-local env mutex.

use std::collections::BTreeSet;
use std::io;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Mutex, MutexGuard, OnceLock};
use std::time::{Duration, Instant};

use cerulion_core::transport::cerulion_q::UnusableRunsAnswer;
use cerulion_core::{
    CatalogEntry, CatalogProvenance, CatalogReply, GatewayEgressPolicy, GatewayPlan, RunsReply,
    SchemaReply, SchemaServing, TransportError,
};

use cerulion_netd::client::{ClientError, NetdClient, NETD_BIN_ENV};
use cerulion_netd::daemon::{self, NetdConfig, RunningNetd, IDLE_GRACE_ENV};
use cerulion_netd::egress::NoopEgressPlane;
use cerulion_netd::egress::{EgressError, EgressId, EgressPlane};
use cerulion_netd::hygiene::SOCKET_ENV;
use cerulion_netd::mirror::{MirrorError, MirrorPlane, MirrorRelease};
use cerulion_netd::net::NETWORK_ENV;
use cerulion_netd::protocol::{Hello, HELLO_MARKER, PROTOCOL_VERSION};
use cerulion_netd::query::{CatalogGather, QueryError, QueryPlane, RunsGather, SchemaGather};
use cerulion_netd::registry::TopicKey;
use cerulion_netd::DiscoveryState;

// --------------------------------------------------------------------------
// A counting spy mirror plane (a DI test double — Principle #13, not fake data).
// Production-faithful: refuses a re-ensure of a live key; `retired` frees the slot.
// --------------------------------------------------------------------------

#[derive(Default)]
struct SpyPlane {
    ensured: Mutex<BTreeSet<TopicKey>>,
    ensure_ok: AtomicUsize,
    release_calls: AtomicUsize,
}
impl MirrorPlane for SpyPlane {
    fn ensure_mirror(&self, key: &TopicKey, _schema_hash: u64) -> Result<(), MirrorError> {
        let mut ensured = self.ensured.lock().unwrap();
        if ensured.contains(key) {
            return Err(MirrorError::Register {
                key: key.clone(),
                source: Box::new(TransportError::Internal {
                    reason: "spy: already registered".to_string(),
                }),
            });
        }
        ensured.insert(key.clone());
        self.ensure_ok.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }
    fn release_mirror(&self, key: &TopicKey) -> MirrorRelease {
        self.release_calls.fetch_add(1, Ordering::SeqCst);
        // Retired: free the slot so a re-demand re-creates (models a real teardown).
        self.ensured.lock().unwrap().remove(key);
        MirrorRelease::Retired
    }
}

/// The `sockaddr_un.sun_path` ceiling, taken at the STRICTEST platform: macOS gives
/// 104 bytes INCLUDING the NUL, so 103 path characters. (Linux allows 108.) A path over
/// the limit does not fail nicely — `bind`/`connect` return a bare `EINVAL` /
/// "Invalid argument", which surfaces as an unattributed handshake failure.
const SUN_PATH_MAX: usize = 103;

/// A unique temp socket path, ENFORCED short enough for `sockaddr_un`.
///
/// A length ceiling that is only a comment silently bites: on macOS
/// `$TMPDIR` is a ~48-char `/var/folders/…` path, and with a longer prefix the longest tag in this file
/// (`reg_egress_retry_giveup`) produces exactly 103 characters at a single-digit
/// counter and 104 — one over — once the counter reaches double digits. The counter is
/// handed out in the ATOMIC ORDER the parallel tests happen to run in, so whether that
/// test draws a 1- or 2-digit `n` is a RACE: ~5% of whole-file runs fail with an
/// unattributed `Io(Os { code: 22, kind: InvalidInput })` at the handshake (measured on
/// macOS: 3 failures in 60 runs, spread across the three longest-tagged tests).
///
/// So the prefix is short, and the ceiling is ASSERTED — a future long tag fails
/// LOUDLY and deterministically, naming the length and the limit, instead of re-opening
/// a 5% flake with an error message that points nowhere.
///
/// The counter is ZERO-PADDED to a fixed width. Without it the
/// path LENGTH still depended on which `n` a test happened to draw — the very
/// order-dependence the assert exists to eliminate — so a tag sitting exactly at the
/// limit would pass under a single-digit counter and fail under a double-digit one,
/// i.e. the assert itself would be flaky. With the pad, a given tag has ONE length for
/// the whole run: the assert either always fires or never does. (`N_WIDTH` covers far
/// more sockets than this file mints; overflowing it would grow the path again, which
/// the assert then catches loudly rather than silently.)
///
/// Headroom: the binding tag is `reg_egress_retry_giveup` (23
/// chars) — reached through `FakeDaemon::spawn`, NOT a literal `unique_socket("…")`
/// call, so a grep-based survey of this file UNDER-counts and reports a shorter tag as
/// the worst case. On a macOS `$TMPDIR` (~48 chars) with a 5-digit pid its path lands in
/// the mid-90s against the 103 ceiling — single-digit headroom, and the zero-pad itself
/// spends `N_WIDTH - 1` of it. Adding a longer tag is expected to trip the assert; that
/// is the design (fail loudly at the source), not a surprise.
fn unique_socket(tag: &str) -> (PathBuf, PathBuf) {
    /// The fixed counter width — see the fn docs.
    const N_WIDTH: usize = 3;
    static N: AtomicUsize = AtomicUsize::new(0);
    let n = N.fetch_add(1, Ordering::Relaxed);
    let dir = std::env::temp_dir().join(format!(
        "cnd_{tag}_{}_{n:0width$}",
        std::process::id(),
        width = N_WIDTH
    ));
    std::fs::create_dir_all(&dir).expect("mk tempdir");
    let sock = dir.join("netd.sock");
    let len = sock.as_os_str().len();
    assert!(
        len <= SUN_PATH_MAX,
        "test socket path is {len} bytes, over the {SUN_PATH_MAX}-byte sockaddr_un limit \
         ({}) — shorten the `unique_socket` tag (a longer path fails as a bare EINVAL at \
         bind/connect, not as anything readable)",
        sock.display()
    );
    (dir, sock)
}

/// Start an in-process daemon with a spy plane over a unique temp socket.
fn start_daemon(tag: &str) -> (RunningNetd, PathBuf, PathBuf, std::sync::Arc<SpyPlane>) {
    let (dir, sock) = unique_socket(tag);
    let spy = std::sync::Arc::new(SpyPlane::default());
    let plane: std::sync::Arc<dyn MirrorPlane> = std::sync::Arc::clone(&spy) as _;
    let netd = daemon::start(
        sock.clone(),
        plane,
        NetdConfig {
            idle_grace: Duration::from_secs(3600), // never idle-exit mid-test
            idle_watch_poll: Duration::from_millis(25),
            ..NetdConfig::default()
        },
    )
    .expect("daemon start");
    (netd, dir, sock, spy)
}

/// The closed-connection `io::ErrorKind` family a peer-closed UDS surfaces as —
/// ONE oracle for every "the close surfaced LOUDLY" arm in this file, so a
/// platform spelling reaches all of them at once (the rule: assert the
/// error-KIND SET including platform variants, never a single kind).
///
/// * `UnexpectedEof` — a read hits a clean EOF (the dominant path, all platforms).
/// * `BrokenPipe` / `ConnectionReset` — the write-side twins (Linux).
/// * `NotConnected` — macOS reports a write/read on a UDS whose peer already
///   closed as `ENOTCONN` ("Socket is not connected"). OBSERVED on
///   macOS in `a_first_register_egress_retry_that_closes_again_
///   surfaces_loudly`, where a single-kind oracle would reject it.
///
/// Deliberately the SAME family `NetdClient`'s `is_closed_early` classifies (minus
/// the setsockopt-substituted `InvalidInput`, which only the give-up arm adds): the
/// give-up arm's "any permitted kind is necessarily the SECOND attempt's" argument
/// rests on the two sets agreeing.
fn is_closed_connection_kind(kind: io::ErrorKind) -> bool {
    matches!(
        kind,
        io::ErrorKind::UnexpectedEof
            | io::ErrorKind::BrokenPipe
            | io::ErrorKind::ConnectionReset
            | io::ErrorKind::NotConnected
    )
}

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
// A spy QUERY plane + a query-enabled daemon start, so the client's
// query_catalog/query_schema round-trip is driven against a REAL daemon.
// --------------------------------------------------------------------------

#[derive(Default)]
struct SpyQueryPlane {
    catalogs: Mutex<Vec<CatalogReply>>,
    schemas: Mutex<Vec<SchemaReply>>,
    /// Canned runs replies `query_runs` returns (default empty = "no
    /// robot answered", which is deliberately NOT an absence claim about any robot).
    runs: Mutex<Vec<RunsReply>>,
    /// Robots that ANSWERED UNUSABLY (a wire skew), a DIFFERENT
    /// absence from a robot that did not answer, and the one whose remedy is a
    /// redeploy rather than a wait.
    unusable_runs: Mutex<Vec<UnusableRunsAnswer>>,
    /// Robots this plane was ASKED about and never heard from, the
    /// COVERAGE half, a THIRD absence again (it may answer later, or never).
    silent_runs: Mutex<Vec<String>>,
    /// The `robot` filter of every `query_runs` call, in order — the oracle for
    /// whether the client really scoped the GET the caller asked for.
    runs_robots: Mutex<Vec<Option<String>>>,
}
impl QueryPlane for SpyQueryPlane {
    fn query_catalog(&self, _robot: Option<&str>) -> Result<CatalogGather, QueryError> {
        Ok(CatalogGather {
            catalogs: self.catalogs.lock().unwrap().clone(),
            // The spy models a SETTLED plane (the cold-start grace itself is
            // oracle-tested in `query.rs`); the daemon-level cold-start CARRIAGE arm
            // lives in `daemon_e2e_test.rs`.
            discovery: DiscoveryState::Settled,
            // A spy plane reports no plane age (it is not a real query plane).
            unsettled_for: None,
        })
    }
    fn query_schema(
        &self,
        _robot: Option<&str>,
        _requested: &str,
    ) -> Result<SchemaGather, QueryError> {
        Ok(SchemaGather {
            replies: self.schemas.lock().unwrap().clone(),
            discovery: DiscoveryState::Settled,
            // A spy plane reports no plane age (it is not a real query plane).
            unsettled_for: None,
        })
    }
    fn query_runs(&self, robot: Option<&str>) -> Result<RunsGather, QueryError> {
        self.runs_robots
            .lock()
            .unwrap()
            .push(robot.map(str::to_string));
        Ok(RunsGather {
            replies: self.runs.lock().unwrap().clone(),
            unusable: self.unusable_runs.lock().unwrap().clone(),
            silent: self.silent_runs.lock().unwrap().clone(),
            discovery: DiscoveryState::Settled,
            unsettled_for: None,
        })
    }
}

/// Start an in-process daemon with a spy mirror plane AND a spy query plane.
fn start_daemon_with_query(
    tag: &str,
) -> (RunningNetd, PathBuf, PathBuf, std::sync::Arc<SpyQueryPlane>) {
    let (dir, sock) = unique_socket(tag);
    let query_spy = std::sync::Arc::new(SpyQueryPlane::default());
    let netd = daemon::start_with_planes(
        sock.clone(),
        std::sync::Arc::new(SpyPlane::default()) as std::sync::Arc<dyn MirrorPlane>,
        std::sync::Arc::new(NoopEgressPlane),
        std::sync::Arc::clone(&query_spy) as std::sync::Arc<dyn QueryPlane>,
        NetdConfig {
            idle_grace: Duration::from_secs(3600),
            idle_watch_poll: Duration::from_millis(25),
            ..NetdConfig::default()
        },
    )
    .expect("daemon start");
    (netd, dir, sock, query_spy)
}

// --------------------------------------------------------------------------
// A counting spy EGRESS plane (a DI double — Principle #13, not fake
// data). Records register/release calls + the announced topics; the FIRST register
// "boots" the shared gateway (`Ok(true)`), later ones join it (`Ok(false)`).
// --------------------------------------------------------------------------

#[derive(Default)]
struct SpyEgressPlane {
    booted: AtomicBool,
    register_calls: AtomicUsize,
    release_calls: AtomicUsize,
    announced: Mutex<Vec<String>>,
}
impl EgressPlane for SpyEgressPlane {
    fn register_egress(
        &self,
        _id: EgressId,
        plan: &GatewayPlan,
        _serving: &SchemaServing,
        _ix_config_json: Option<&str>,
    ) -> Result<bool, EgressError> {
        self.register_calls.fetch_add(1, Ordering::SeqCst);
        self.announced
            .lock()
            .unwrap()
            .extend(plan.announce.iter().cloned());
        // First registration boots the (spied) gateway; later ones join it.
        Ok(!self.booted.swap(true, Ordering::SeqCst))
    }
    fn release_egress(&self, _id: EgressId) {
        self.release_calls.fetch_add(1, Ordering::SeqCst);
    }
}

/// Start an in-process daemon with a spy mirror plane AND a spy egress plane over a
/// unique temp socket (the egress-verb harness).
fn start_daemon_with_egress(
    tag: &str,
) -> (
    RunningNetd,
    PathBuf,
    PathBuf,
    std::sync::Arc<SpyPlane>,
    std::sync::Arc<SpyEgressPlane>,
) {
    let (dir, sock) = unique_socket(tag);
    let mirror = std::sync::Arc::new(SpyPlane::default());
    let egress = std::sync::Arc::new(SpyEgressPlane::default());
    let mirror_plane: std::sync::Arc<dyn MirrorPlane> = std::sync::Arc::clone(&mirror) as _;
    let egress_plane: std::sync::Arc<dyn EgressPlane> = std::sync::Arc::clone(&egress) as _;
    let netd = daemon::start_with_egress(
        sock.clone(),
        mirror_plane,
        egress_plane,
        NetdConfig {
            idle_grace: Duration::from_secs(3600), // never idle-exit mid-test
            idle_watch_poll: Duration::from_millis(25),
            ..NetdConfig::default()
        },
    )
    .expect("daemon start (with egress)");
    (netd, dir, sock, mirror, egress)
}

/// A small AllowAll egress plan announcing `topics`.
fn egress_plan(topics: &[&str]) -> GatewayPlan {
    GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: topics.iter().map(|t| t.to_string()).collect(),
        ingress: vec![],
    }
}

// --------------------------------------------------------------------------
// connect-to-running tests.
// --------------------------------------------------------------------------

#[test]
fn client_connects_to_a_running_daemon_and_demands() {
    let (netd, dir, sock, spy) = start_daemon("demand");
    let mut client = NetdClient::connect_or_spawn_at(sock.clone()).expect("connect");
    assert_eq!(client.socket_path(), sock.as_path());

    let d = client
        .demand("ubuntu", "/utlidar/robot_odom", 0xABCD)
        .expect("demand ok");
    assert_eq!(d.robot, "ubuntu");
    assert_eq!(d.topic, "/utlidar/robot_odom");
    assert_eq!(d.refcount, 1);
    assert!(d.mirror_created, "the first demand creates the mirror");
    // The daemon registered EXACTLY one mirror through the real UDS path.
    assert_eq!(spy.ensure_ok.load(Ordering::SeqCst), 1);
    assert_eq!(netd.active_demand_count(), 1);

    drop(client);
    drop(netd);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn two_clients_share_one_mirror_frames_cross_once() {
    // Decision: two consumers of the same remote topic share one mirror.
    let (netd, dir, sock, spy) = start_daemon("share");
    let mut a = NetdClient::connect_or_spawn_at(sock.clone()).expect("a");
    let mut b = NetdClient::connect_or_spawn_at(sock.clone()).expect("b");

    let da = a.demand("ubuntu", "/tf", 7).expect("a demand");
    assert!(da.mirror_created, "a is the first demander");
    let db = b.demand("ubuntu", "/tf", 7).expect("b demand");
    assert!(!db.mirror_created, "b JOINS a's mirror — no second mirror");
    assert_eq!(db.refcount, 2);

    // ONE ensure for two demanders — the frame crosses the network once.
    assert_eq!(spy.ensure_ok.load(Ordering::SeqCst), 1);
    assert_eq!(netd.mirror_count(), 1);

    drop(a);
    drop(b);
    drop(netd);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn explicit_release_decrements_then_drop_releases_the_rest() {
    let (netd, dir, sock, spy) = start_daemon("release");
    let mut client = NetdClient::connect_or_spawn_at(sock.clone()).expect("connect");
    client.demand("ubuntu", "/tf", 1).expect("demand tf");
    client.demand("ubuntu", "/odom", 2).expect("demand odom");
    assert_eq!(netd.active_demand_count(), 2);

    // Explicit early release of ONE topic (vizd's detach) — the OTHER stays.
    let r = client.release("ubuntu", "/tf").expect("release tf");
    assert!(r.last_release, "the only demander of /tf left");
    assert!(
        wait_until(|| netd.active_demand_count() == 1, Duration::from_secs(2)),
        "one demand remains after the explicit release"
    );
    assert_eq!(spy.release_calls.load(Ordering::SeqCst), 1);

    // Dropping the client closes the connection → releases the REST (/odom).
    drop(client);
    assert!(
        wait_until(|| netd.active_demand_count() == 0, Duration::from_secs(2)),
        "the connection close released every remaining demand (the pairing guarantee)"
    );
    assert!(wait_until(
        || spy.release_calls.load(Ordering::SeqCst) == 2,
        Duration::from_secs(2)
    ));
    drop(netd);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn drop_releases_every_held_demand_no_leak() {
    // The crash-safe pairing: a client that just DROPS (a panic / SIGKILL / clean
    // exit — all close the fd) releases every demand it held.
    let (netd, dir, sock, spy) = start_daemon("drop");
    let mut client = NetdClient::connect_or_spawn_at(sock.clone()).expect("connect");
    client.demand("ubuntu", "/tf", 1).expect("d1");
    client.demand("go2", "/scan", 2).expect("d2");
    assert_eq!(netd.active_demand_count(), 2);

    drop(client); // no explicit release — just drop.
    assert!(
        wait_until(|| netd.active_demand_count() == 0, Duration::from_secs(2)),
        "dropping the client released BOTH demands (no leak)"
    );
    assert!(wait_until(
        || spy.release_calls.load(Ordering::SeqCst) == 2,
        Duration::from_secs(2)
    ));
    drop(netd);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn a_daemon_error_surfaces_as_a_loud_client_error() {
    // A schema conflict (one topic, one schema) reaches the client as ClientError::Netd,
    // never a silent success.
    let (netd, dir, sock, _spy) = start_daemon("conflict");
    let mut a = NetdClient::connect_or_spawn_at(sock.clone()).expect("a");
    let mut b = NetdClient::connect_or_spawn_at(sock.clone()).expect("b");
    a.demand("ubuntu", "/tf", 0xAAAA).expect("a demand");
    let err = b
        .demand("ubuntu", "/tf", 0xBBBB)
        .expect_err("conflicting schema refused");
    match err {
        ClientError::Netd { error, topic, .. } => {
            assert!(error.contains("schema conflict"), "{error}");
            assert_eq!(topic.as_deref(), Some("/tf"));
        }
        other => panic!("expected a Netd error, got {other:?}"),
    }
    drop(a);
    drop(b);
    drop(netd);
    let _ = std::fs::remove_dir_all(&dir);
}

// --------------------------------------------------------------------------
// The egress-verb client half (register_egress / release_egress),
// driven against a REAL daemon with a spy egress plane. Hand oracles.
// --------------------------------------------------------------------------

#[test]
fn client_registers_and_releases_egress() {
    let (netd, dir, sock, _mirror, egress) = start_daemon_with_egress("egress_reg");
    let mut client = NetdClient::connect_or_spawn_at(sock.clone()).expect("connect");

    // Register a one-topic egress plan → the daemon boots the (spied) gateway and
    // records the announce topic through the REAL UDS + registry path.
    let plan = egress_plan(&["/robot/cmd_vel"]);
    let resp = client
        .register_egress(&plan, &SchemaServing::default(), None)
        .expect("register_egress ok");
    assert_eq!(resp.registered_topics, 1);
    assert!(
        resp.gateway_started,
        "the first egress plan boots the shared gateway"
    );
    assert_eq!(egress.register_calls.load(Ordering::SeqCst), 1);
    assert_eq!(
        egress.announced.lock().unwrap().as_slice(),
        &["/robot/cmd_vel".to_string()],
        "the plan's announce topic crossed the wire to the egress plane"
    );

    // Explicit early release — the daemon drops the connection-scoped accounting and
    // drives the plane's release exactly once.
    let rel = client.release_egress().expect("release_egress ok");
    assert_eq!(rel.released_topics, 1);
    assert!(
        wait_until(
            || egress.release_calls.load(Ordering::SeqCst) == 1,
            Duration::from_secs(2)
        ),
        "the explicit release drove the egress plane's release once"
    );

    // Dropping the client after an explicit release must NOT double-release (the
    // connection held nothing more).
    drop(client);
    // Give any spurious release a chance to (wrongly) fire, then assert still-1.
    std::thread::sleep(Duration::from_millis(100));
    assert_eq!(
        egress.release_calls.load(Ordering::SeqCst),
        1,
        "a drop after an explicit release does not double-release"
    );
    drop(netd);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn drop_releases_egress_no_leak() {
    // The crash-safe pairing for EGRESS: a client that just DROPS (a panic / SIGKILL /
    // clean exit — all close the fd) releases its egress registration, exactly like a
    // held demand. No explicit release — hold the client for the "run" and let it drop.
    let (netd, dir, sock, _mirror, egress) = start_daemon_with_egress("egress_drop");
    let mut client = NetdClient::connect_or_spawn_at(sock.clone()).expect("connect");
    let plan = egress_plan(&["/a", "/b"]);
    let resp = client
        .register_egress(&plan, &SchemaServing::default(), None)
        .expect("register_egress ok");
    assert_eq!(resp.registered_topics, 2, "both announce topics registered");
    assert_eq!(egress.register_calls.load(Ordering::SeqCst), 1);
    assert_eq!(egress.release_calls.load(Ordering::SeqCst), 0);

    drop(client); // no explicit release — just drop.
    assert!(
        wait_until(
            || egress.release_calls.load(Ordering::SeqCst) == 1,
            Duration::from_secs(2)
        ),
        "dropping the client released the egress registration (the pairing guarantee)"
    );
    drop(netd);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn two_client_egress_registrations_are_scoped_per_connection() {
    // The client/daemon e2e of per-connection egress
    // scoping (the registry-level oracle is `register_egress_records_topics_scoped_per_
    // connection`). TWO client connections against ONE real daemon each register a
    // DISTINCT topic; the registrations are independent — each sees its OWN per-connection
    // count, the FIRST boots the shared gateway and the second JOINS it, and one
    // connection releasing NEVER touches the other's registration.
    let (netd, dir, sock, _mirror, egress) = start_daemon_with_egress("egress_twoconn");

    // Client A boots the shared gateway with /a1.
    let mut a = NetdClient::connect_or_spawn_at(sock.clone()).expect("connect A");
    let ra = a
        .register_egress(&egress_plan(&["/a1"]), &SchemaServing::default(), None)
        .expect("A register_egress ok");
    assert_eq!(ra.registered_topics, 1);
    assert!(
        ra.gateway_started,
        "the first egress plan boots the shared gateway"
    );

    // Client B JOINS the already-booted gateway with a DISTINCT /b1 — its OWN scope.
    let mut b = NetdClient::connect_or_spawn_at(sock.clone()).expect("connect B");
    let rb = b
        .register_egress(&egress_plan(&["/b1"]), &SchemaServing::default(), None)
        .expect("B register_egress ok");
    assert_eq!(
        rb.registered_topics, 1,
        "B's per-connection count is its OWN topic only, not the machine total"
    );
    assert!(
        !rb.gateway_started,
        "the second egress plan JOINS the already-booted gateway (does not re-boot it)"
    );
    assert_eq!(egress.register_calls.load(Ordering::SeqCst), 2);
    {
        let ann = egress.announced.lock().unwrap();
        assert!(
            ann.contains(&"/a1".to_string()) && ann.contains(&"/b1".to_string()),
            "both connections' announce topics reached the egress plane: {ann:?}"
        );
    }

    // A releases by DROPPING — B's registration is UNTOUCHED (per-connection scoping):
    // exactly ONE plane release (A's), and B can still re-register.
    drop(a);
    assert!(
        wait_until(
            || egress.release_calls.load(Ordering::SeqCst) == 1,
            Duration::from_secs(2)
        ),
        "A's drop released EXACTLY A's egress registration (got {})",
        egress.release_calls.load(Ordering::SeqCst)
    );
    // The mutation-killing proof B survived A's release: B re-registers /b2 and its
    // per-connection total grows to 2 (it still holds /b1). A connection torn down by
    // A's release would report 1 (or the daemon would have released B's slot).
    let rb2 = b
        .register_egress(&egress_plan(&["/b2"]), &SchemaServing::default(), None)
        .expect("B re-register survives A's release");
    assert_eq!(
        rb2.registered_topics, 2,
        "B still holds /b1 (A's release did not touch B's connection scope) + the new /b2"
    );

    // B drops → its registration releases too (release_calls now 2).
    drop(b);
    assert!(
        wait_until(
            || egress.release_calls.load(Ordering::SeqCst) == 2,
            Duration::from_secs(2)
        ),
        "B's drop released B's egress registration (got {})",
        egress.release_calls.load(Ordering::SeqCst)
    );

    drop(netd);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn register_egress_is_refused_locally_by_an_older_daemon() {
    // Per-verb compat: a v1 daemon is too old for the v2 register_egress
    // verb. The client refuses it LOCALLY (before sending) with a precise version error —
    // never a silently-lost registration. Driven with a CRAFTED v1 banner (a v2 client
    // still connects to a v1 daemon and uses demand/release; only the egress verb is
    // refused).
    let (dir, sock) = serve_crafted_banner("egress_v1", 1);
    let mut client =
        NetdClient::connect_or_spawn_at(sock).expect("a v2 client connects to a v1 daemon");
    let plan = egress_plan(&["/t"]);
    let err = client
        .register_egress(&plan, &SchemaServing::default(), None)
        .expect_err("a v1 daemon cannot serve the v2 egress verb");
    match err {
        ClientError::Protocol(msg) => {
            assert!(msg.contains("register_egress"), "names the verb: {msg}");
            assert!(msg.contains("v1"), "names the daemon version: {msg}");
            assert!(
                msg.contains("Upgrade cerulion-netd"),
                "names the remedy: {msg}"
            );
        }
        other => panic!("expected a Protocol version error, got {other:?}"),
    }
    let _ = std::fs::remove_dir_all(&dir);
}

// --------------------------------------------------------------------------
// spawn-a-real-daemon test (env-mutating → file-local mutex).
// --------------------------------------------------------------------------

/// The first-use spawn line (`info!`) and the readiness-wait stall notice
/// (`warn!`), as the client spells them. Hand-pasted oracles.
const FIRST_USE_LINE: &str = "cerulion-netd is not running, starting it (first use)";
const STALL_LINE: &str = "cerulion-netd is still starting";

/// Count the captured lines carrying `marker` whose LEVEL token is `level`.
///
/// The level is matched as a whole whitespace token out of the line header,
/// never as a bare substring: `tracing-test` renders the test's own span name
/// (which could contain any word) into every line, and a field value could
/// carry an uppercase level word.
fn count_at(lines: &[&str], level: &str, marker: &str) -> usize {
    lines
        .iter()
        .filter(|l| l.contains(marker))
        .filter(|l| l.split_whitespace().take(3).any(|tok| tok == level))
        .count()
}

fn env_lock() -> MutexGuard<'static, ()> {
    static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
    LOCK.get_or_init(|| Mutex::new(()))
        .lock()
        .unwrap_or_else(|e| e.into_inner())
}

/// RAII env guard (panic-safe restore).
struct EnvGuard(&'static str, Option<String>);
impl EnvGuard {
    fn set(key: &'static str, val: &str) -> Self {
        let prev = std::env::var(key).ok();
        std::env::set_var(key, val);
        EnvGuard(key, prev)
    }
}
impl Drop for EnvGuard {
    fn drop(&mut self) {
        match &self.1 {
            Some(v) => std::env::set_var(self.0, v),
            None => std::env::remove_var(self.0),
        }
    }
}

#[test]
fn client_spawns_a_real_daemon_when_none_is_running() {
    let _lk = env_lock();
    let (dir, sock) = unique_socket("spawn");

    // Point the client at the FRESHLY-BUILT cerulion-netd binary + local-only (no
    // zenoh) + a short idle grace so the spawned daemon self-cleans quickly. All in
    // the PARENT env so the spawned child inherits network=off + the grace, and the
    // client's resolve_netd_bin picks up the binary.
    //
    // The grace is TEARDOWN hygiene, not a property under test. It was
    // 400 ms, which ALSO bounded the window between the child building its registry
    // (`idle_since` is anchored at construction) and this connect, so a loaded runner
    // could make the spawned daemon self-exit underneath us. That is survivable — the
    // socket is unlinked, so `connect_or_spawn_at` re-classifies and respawns, and
    // the widened `is_closed_early` retries the accepted-then-slammed shape,
    // but it spends the product's ONE retry on a harness artifact rather than on the
    // condition it exists for.
    //
    // What 2 s costs and what it does not buy:
    // there is no teardown assertion. The `wait_until(|| !sock.exists(), 5 s)` at the
    // end is a DISCARDED best-effort wait — `let _ = …` — so the daemon's self-clean
    // is never gated, and `remove_dir_all` runs unconditionally either way. Do not
    // read that wait as a live check, and do not tune this grace against it. The real
    // cost of the bump is that the spawned daemon is held alive ~1.6 s longer while
    // this test holds the process-wide `env_lock()`; the real benefit is only that
    // the pre-connect window stops being the binding constraint.
    let _b = EnvGuard::set(NETD_BIN_ENV, env!("CARGO_BIN_EXE_cerulion-netd"));
    let _n = EnvGuard::set(NETWORK_ENV, "off");
    let _g = EnvGuard::set(IDLE_GRACE_ENV, "2000");
    // Make sure no stray CERULION_NETD_SOCK from the environment overrides us; the
    // client sets it on the child explicitly, but keep the parent clean.
    let _s = EnvGuard::set(SOCKET_ENV, sock.to_str().unwrap());

    // Nothing is listening → the client SPAWNS the daemon detached and connects.
    let mut client = NetdClient::connect_or_spawn_at(sock.clone())
        .expect("connect_or_spawn spawns + connects to a real daemon");

    // The spawned daemon is LOCAL-ONLY, so a demand's mirror registration surfaces
    // the explicit no-network error — proving the full spawn→connect→hello→demand→
    // response path over a REAL spawned PROCESS (hermetic, no robot needed).
    let err = client
        .demand("ubuntu", "/utlidar/robot_odom", 0x1234)
        .expect_err("a local-only daemon cannot mirror a remote topic");
    match err {
        ClientError::Netd { error, .. } => assert!(
            error.contains("mirror registration failed"),
            "the no-network mirror failure crossed the wire: {error}"
        ),
        other => panic!("expected a Netd (mirror-registration) error, got {other:?}"),
    }

    // Drop the client → connection close → the daemon goes idle → self-exits after
    // the short grace (cleaning its own socket + pidfile). Give it time so it does
    // not linger past the test.
    drop(client);
    let _ = wait_until(|| !sock.exists(), Duration::from_secs(5));
    let _ = std::fs::remove_dir_all(&dir);
}

// --------------------------------------------------------------------------
// The post-spawn READINESS wait. Two hermetic pins driven by a FAKE
// "cerulion-netd" binary (a tiny `sh` script pointed at by NETD_BIN_ENV) — no real
// daemon, no iceoryx2, no zenoh, so they measure the WAIT and nothing else.
//
// Why it matters: a fixed `40 × 25 ms` ≈ 1.0 s attempt budget loses to a real
// daemon's cold boot (iceoryx2 transport init + the startup dead-node sweep),
// measured ~1.1 s on a desk with ~50 stale `/tmp/iceoryx2/nodes` entries — so the
// FIRST consumer of a remote topic would fail with "it did not come up in time" against a
// daemon that was about to listen. `NetdClient::connect_or_spawn` is the
// first-consumer-spawns path behind `topic echo`/`info`/`hz` and vizd's
// `attach_remote`, so that is a user-visible failure, not a test artifact.
// --------------------------------------------------------------------------

/// Write an executable `sh` script into `dir` and return its path — a stand-in
/// "cerulion-netd" binary for [`NETD_BIN_ENV`]. Unix-only, like the whole crate
/// (the control seam is a `UnixStream`).
fn write_fake_daemon(dir: &std::path::Path, name: &str, body: &str) -> PathBuf {
    use std::os::unix::fs::PermissionsExt;
    let path = dir.join(name);
    std::fs::write(&path, body).expect("write fake daemon script");
    std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o755))
        .expect("chmod fake daemon script");
    path
}

/// Bind `sock` and serve ONE current-version hello banner after `delay` — a daemon
/// whose socket appears LATE. Returns immediately; the bind happens on the thread.
///
/// The returned receiver carries the BIND OUTCOME. The bind used
/// to `return` on error, so a harness failure (a leftover socket file, a too-long
/// `sun_path`, a permissions problem) presented as "the client never connected" — i.e.
/// exactly the regression this file exists to catch. The caller must consult
/// this receiver BEFORE asserting on the client, so a harness failure fails the test
/// with a harness message.
fn bind_and_serve_banner_after(
    sock: PathBuf,
    delay: Duration,
) -> std::sync::mpsc::Receiver<Result<(), String>> {
    bind_and_serve_banner_when(sock, move || {
        std::thread::sleep(delay);
        Ok(())
    })
}

/// Bind `sock` as soon as `pidfile` EXISTS, i.e. once the client has spawned the
/// fake daemon (the script's first act is recording its pid).
///
/// This is the STRUCTURAL form of "a daemon that comes up at once". A timed bind
/// ("100 ms from now") races the client's fast-path connect: on a loaded runner
/// the test thread can stall past the delay, the fast path then SUCCEEDS, nothing
/// is spawned, no first-use line prints, and the arm fails with a product message
/// for a harness accident. Keyed on the pidfile, the socket cannot exist before
/// the spawn, so the fast path fails by construction however the threads are
/// scheduled, and the bind follows the spawn by one short poll.
///
/// A pidfile that never appears inside `give_up` is reported through the same
/// receiver as a bind failure (the caller decides whose fault that is: it holds
/// the client's own result).
fn bind_and_serve_banner_once_spawned(
    sock: PathBuf,
    pidfile: PathBuf,
    give_up: Duration,
) -> std::sync::mpsc::Receiver<Result<(), String>> {
    bind_and_serve_banner_when(sock, move || {
        let deadline = Instant::now() + give_up;
        while !pidfile.exists() {
            if Instant::now() >= deadline {
                return Err(format!(
                    "the fake daemon never recorded its pid at {} within {give_up:?} \
                     (the client never spawned it)",
                    pidfile.display()
                ));
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        Ok(())
    })
}

/// The shared binder body: run `ready` on a fresh thread, then bind `sock` and
/// serve ONE current-version hello banner. The receiver carries `ready`'s error
/// or the bind outcome.
fn bind_and_serve_banner_when(
    sock: PathBuf,
    ready: impl FnOnce() -> Result<(), String> + Send + 'static,
) -> std::sync::mpsc::Receiver<Result<(), String>> {
    use std::io::{Read, Write};
    use std::os::unix::net::UnixListener;

    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        if let Err(e) = ready() {
            let _ = tx.send(Err(e));
            return;
        }
        let listener = match UnixListener::bind(&sock) {
            Ok(l) => {
                let _ = tx.send(Ok(()));
                l
            }
            Err(e) => {
                let _ = tx.send(Err(format!("bind {}: {e}", sock.display())));
                return;
            }
        };
        if let Ok((mut stream, _)) = listener.accept() {
            let banner = Hello {
                hello: HELLO_MARKER.to_string(),
                protocol: PROTOCOL_VERSION,
            }
            .to_json_line();
            let _ = writeln!(stream, "{banner}");
            let _ = stream.flush();
            // Hold the connection open until the client drops it.
            let mut buf = [0u8; 256];
            while let Ok(n) = stream.read(&mut buf) {
                if n == 0 {
                    break;
                }
            }
        }
    });
    rx
}

/// THE readiness-wait regression pin: a daemon whose socket appears WELL AFTER the old
/// ~1.0 s budget is still connected to.
///
/// The fake daemon binary stays ALIVE and binds nothing (so the fail-fast path is not
/// what saves us — the wait genuinely has to keep waiting); a separate thread binds the
/// socket + serves the hello at `BOOT_DELAY`. Hand oracle, not a self-compare: the
/// client MUST return a live connection, and the wall time MUST be at least
/// `BOOT_DELAY` (proving the socket really was absent through a whole 1.0 s budget
/// — otherwise the test would pass vacuously against a fast bind).
///
/// Restoring a fixed `40 × 25 ms` budget makes this fail: 3.5 s of absence is
/// 3.5× that budget, with the readiness-wait message, NOT the harness one (see
/// `HARNESS_REPORT_MARGIN`).
///
/// Also the STALL-NOTICE pin: a 3.5 s boot runs 1.5 s past the 2 s notice bound
/// (a 2.5 s boot left the notice a 0.5 s window, which ONE scheduler stall on a
/// loaded runner could span, skipping the poll that emits it), so
/// the wait must have emitted exactly ONE `WARN` saying the daemon is still
/// starting (the quiet verbs' only notice, since the first-use line itself
/// rides `INFO` and is asserted here at that level, exactly once). The
/// prompt-daemon twin below pins the absence on a healthy boot. The client
/// runs on this thread, so `#[traced_test]`'s capture sees its lines.
#[tracing_test::traced_test]
#[test]
fn a_slow_booting_daemon_is_still_connected_to_past_the_pre_891_budget() {
    /// Comfortably past a fixed ~1.0 s budget AND the 2 s stall-notice bound (by
    /// 1.5 s, about sixty client polls), comfortably under the 10 s ceiling.
    const BOOT_DELAY: Duration = Duration::from_millis(3500);
    /// Slack ON TOP of the binder's own `BOOT_DELAY` for the harness-first wait.
    ///
    /// The budget MUST be derived from `BOOT_DELAY`, never fixed: the harness check runs
    /// AFTER `connect_or_spawn_at` returns, and under the very mutation this test exists
    /// to catch (a client that gives up at ~1 s) a fixed 1 s wait would expire at ~2 s —
    /// BEFORE the 3.5 s bind, and report a HARNESS FAILURE for a healthy harness,
    /// inverting the verdict on the one regression this file is the pin for. Deriving it
    /// means the harness verdict fires only when the binder genuinely never reported.
    const HARNESS_REPORT_MARGIN: Duration = Duration::from_secs(2);

    let _lk = env_lock();
    let (dir, sock) = unique_socket("slowboot");

    // A fake "daemon" that LIVES but never binds. It records its own pid (`exec` keeps
    // it) so the test reaps it instead of leaving a stray sleeper behind.
    let pidfile = dir.join("fake.pid");
    let script = write_fake_daemon(
        &dir,
        "slow-netd",
        &format!(
            "#!/bin/sh\necho $$ > '{}'\nexec sleep 30\n",
            pidfile.display()
        ),
    );
    let _b = EnvGuard::set(NETD_BIN_ENV, script.to_str().unwrap());
    let _s = EnvGuard::set(SOCKET_ENV, sock.to_str().unwrap());

    // The binder's own clock starts HERE (it sleeps `BOOT_DELAY` before binding), so this
    // is the reference point the harness-first budget below is derived from.
    let binder_spawned = Instant::now();
    let bound = bind_and_serve_banner_after(sock.clone(), BOOT_DELAY);

    let started = Instant::now();
    let connected = NetdClient::connect_or_spawn_at(sock.clone());
    let waited = started.elapsed();

    // HARNESS FIRST: if our own late-binder never bound, say THAT. Otherwise a broken
    // harness is indistinguishable from the regression — both look like "the
    // client did not connect".
    //
    // Budget = whatever is LEFT of the binder's `BOOT_DELAY` plus a margin. On the happy
    // path the bind already happened (`waited >= BOOT_DELAY`) and this returns at once;
    // when the client bails early — the regression — we still wait for the real bind
    // outcome, so the verdict below is the real one and not a false harness accusation.
    let binder_elapsed = binder_spawned.elapsed();
    let boot_remaining = BOOT_DELAY.saturating_sub(binder_elapsed);
    let harness_budget = boot_remaining + HARNESS_REPORT_MARGIN;
    match bound.recv_timeout(harness_budget) {
        Ok(Ok(())) => {}
        Ok(Err(e)) => panic!(
            "HARNESS FAILURE (not the readiness-wait regression): the late-binding stand-in \
             daemon could not bind — {e}"
        ),
        // The rendered budget must be the one actually USED. It is
        // `BOOT_DELAY - <elapsed since the binder was spawned>` plus the margin — which
        // on the healthy path (the client already waited past `BOOT_DELAY`) is just the
        // margin, NOT `BOOT_DELAY + margin`. Printing the latter would send a debugger
        // hunting a budget that was never applied.
        Err(e) => panic!(
            "HARNESS FAILURE (not the readiness-wait regression): the late-binding stand-in \
             daemon never reported a bind outcome within {harness_budget:?} \
             (BOOT_DELAY {BOOT_DELAY:?} - {binder_elapsed:?} elapsed = \
             {boot_remaining:?} remaining, + margin {HARNESS_REPORT_MARGIN:?}) — {e}"
        ),
    }

    let client = connected.expect("a daemon that binds at 3.5s must still be connected to");

    assert!(
        waited >= BOOT_DELAY,
        "the socket must genuinely have been absent through the pre-891 budget — waited \
         only {waited:?} (a vacuous pass)"
    );
    assert!(
        waited < Duration::from_secs(9),
        "connected at readiness, not at the ceiling — waited {waited:?}"
    );
    assert_eq!(client.socket_path(), sock.as_path());

    // The two notices, each pinned at its LEVEL as a whole token (a text-only
    // predicate would pass the stall notice demoted to `info`, where the quiet
    // verbs cannot see it): the first-use line at INFO exactly once, and the
    // stall notice at WARN exactly once for a 3.5 s boot.
    logs_assert(|lines: &[&str]| {
        match count_at(lines, "INFO", FIRST_USE_LINE) {
            1 => {}
            n => return Err(format!("expected exactly 1 INFO first-use line, got {n}")),
        }
        match count_at(lines, "WARN", STALL_LINE) {
            1 => Ok(()),
            n => Err(format!(
                "a 3.5 s boot must draw exactly 1 WARN stall notice, got {n}"
            )),
        }
    });

    drop(client);
    // Reap the fake daemon (its own recorded pid — `exec` preserved it).
    if let Ok(raw) = std::fs::read_to_string(&pidfile) {
        if let Ok(pid) = raw.trim().parse::<i32>() {
            // SAFETY: `kill` on a pid this test spawned; a stale pid at worst yields ESRCH.
            unsafe {
                libc::kill(pid, libc::SIGKILL);
            }
        }
    }
    let _ = std::fs::remove_dir_all(&dir);
}

/// The stall notice's ABSENCE twin: a daemon whose socket is there at the
/// first poll draws the first-use INFO line and NO stall WARN. Without this
/// arm the slow-boot pin above would pass a notice that fires unconditionally
/// (on every healthy first use, the very "warning for the expected case" the
/// level change removed). Same fake daemon shape (alive, binds nothing); the
/// binder thread binds the moment the fake daemon's pidfile appears, so the
/// client's fast-path connect FAILS by construction (the socket cannot exist
/// before the spawn; were it to succeed, nothing is spawned and the first-use
/// line never prints) and the socket is there within one short poll of the
/// spawn: the wait never comes near `SPAWN_STALL_NOTICE_AFTER`.
///
/// LOAD: a runner slow enough to stretch even that wait past `EARNED_AFTER`
/// has EARNED a stall notice, so the absence claim is skipped there with a
/// printed DEGRADE line (visible under `--nocapture`) instead of failing; the
/// first-use line's level and count are asserted in every case.
#[tracing_test::traced_test]
#[test]
fn a_promptly_booting_daemon_draws_no_stall_notice() {
    /// How long the binder waits for the client to spawn the fake daemon.
    const SPAWN_SEEN_WITHIN: Duration = Duration::from_secs(10);
    /// Past this wait the runner, not the product, produced the delay: the
    /// notice bound is 2 s, so a wait this long may legitimately have drawn it.
    const EARNED_AFTER: Duration = Duration::from_millis(1_500);
    let _lk = env_lock();
    let (dir, sock) = unique_socket("prompt");

    let pidfile = dir.join("fake.pid");
    let script = write_fake_daemon(
        &dir,
        "prompt-netd",
        &format!(
            "#!/bin/sh\necho $$ > '{}'\nexec sleep 30\n",
            pidfile.display()
        ),
    );
    let _b = EnvGuard::set(NETD_BIN_ENV, script.to_str().unwrap());
    let _s = EnvGuard::set(SOCKET_ENV, sock.to_str().unwrap());

    let bound =
        bind_and_serve_banner_once_spawned(sock.clone(), pidfile.clone(), SPAWN_SEEN_WITHIN);
    let started = Instant::now();
    let connected = NetdClient::connect_or_spawn_at(sock.clone());
    let waited = started.elapsed();

    // The binder reports only after the client spawned the fake daemon, so a
    // missing report is read TOGETHER with the client's own result: a client
    // that failed to spawn is a product failure, not a harness one.
    match bound.recv_timeout(SPAWN_SEEN_WITHIN + Duration::from_secs(2)) {
        Ok(Ok(())) => {}
        Ok(Err(e)) => panic!(
            "the stand-in daemon did not bind: {e}; the client returned {:?}",
            connected.as_ref().map(|_| "a connection")
        ),
        Err(e) => panic!("HARNESS FAILURE: the stand-in daemon never reported a bind: {e}"),
    }
    let client = connected.expect("a daemon that binds at once must be connected to");
    // The precondition that makes the ABSENCE meaningful: the wait really was
    // short of the notice bound. A loaded runner that took longer has EARNED
    // the notice, so the absence claim is excused there (and says so), never
    // failed; everything else below is asserted unconditionally.
    let notice_earned = waited >= EARNED_AFTER;
    if notice_earned {
        eprintln!(
            "DEGRADE a_promptly_booting_daemon_draws_no_stall_notice: the connect took \
             {waited:?} (>= {EARNED_AFTER:?}) on this runner, so a stall notice may have been \
             earned; the absence claim is skipped, the first-use line is still asserted"
        );
    }

    logs_assert(|lines: &[&str]| {
        match count_at(lines, "INFO", FIRST_USE_LINE) {
            1 => {}
            n => return Err(format!("expected exactly 1 INFO first-use line, got {n}")),
        }
        match count_at(lines, "WARN", STALL_LINE) {
            0 => Ok(()),
            // Once per wait is the most a slow runner can have earned.
            1 if notice_earned => Ok(()),
            n => Err(format!(
                "a prompt boot must draw NO stall notice, got {n} WARN line(s) \
                 (connect took {waited:?})"
            )),
        }
    });

    drop(client);
    if let Ok(raw) = std::fs::read_to_string(&pidfile) {
        if let Ok(pid) = raw.trim().parse::<i32>() {
            // SAFETY: `kill` on a pid this test spawned; a stale pid at worst yields ESRCH.
            unsafe {
                libc::kill(pid, libc::SIGKILL);
            }
        }
    }
    let _ = std::fs::remove_dir_all(&dir);
}

/// The fail-fast half: a spawned "daemon" that EXITS immediately is reported
/// LOUDLY well before the readiness ceiling — never a ten-second silent stare.
///
/// Hand oracle: the error must be a `Connect` naming the socket, the wait, and the
/// child's DEATH (the discriminator between "died" and "still booting"), and the wall
/// time must be a small fraction of the ceiling.
#[test]
fn a_spawned_daemon_that_dies_immediately_fails_fast_and_loudly() {
    let _lk = env_lock();
    let (dir, sock) = unique_socket("diesfast");

    let script = write_fake_daemon(&dir, "dead-netd", "#!/bin/sh\nexit 3\n");
    let _b = EnvGuard::set(NETD_BIN_ENV, script.to_str().unwrap());
    let _s = EnvGuard::set(SOCKET_ENV, sock.to_str().unwrap());

    let started = Instant::now();
    let err = NetdClient::connect_or_spawn_at(sock.clone())
        .expect_err("a daemon that exits immediately never binds the socket");
    let waited = started.elapsed();
    let msg = err.to_string();

    assert!(
        waited < Duration::from_secs(3),
        "a dead daemon must fail FAST (post-exit grace), not at the 10s ceiling — took {waited:?}"
    );
    assert!(
        matches!(err, ClientError::Connect { ref socket, .. } if socket == &sock),
        "expected a Connect error naming {}, got {err:?}",
        sock.display()
    );
    assert!(
        msg.contains(sock.to_str().unwrap()),
        "the error names the socket path: {msg}"
    );
    assert!(msg.contains("waited"), "the error names the wait: {msg}");
    assert!(
        msg.contains("EXITED"),
        "the error attributes the failure to the daemon's DEATH (not 'still booting'): {msg}"
    );

    let _ = std::fs::remove_dir_all(&dir);
}

// --------------------------------------------------------------------------
// The client's single silent retry on the daemon accepted-then-closed
// WEDGE/RACE signature. A FAKE UDS daemon (in-process, unique socket, NO real
// netd, NO env) closes the first N connections then serves — so it exercises the
// retry deterministically. Parallel-safe.
// --------------------------------------------------------------------------

/// What a fake-daemon connection does.
#[derive(Clone, Copy)]
enum ConnBehavior {
    /// Accept then immediately close (the client gets EOF reading the Hello).
    CloseImmediately,
    /// Write the Hello, then close (the client gets EOF on its first demand).
    HelloThenClose,
    /// Write the Hello, read one request line, write a demand response (a healthy
    /// daemon serving the demand).
    ServeHelloAndDemand,
    /// Write the Hello, read one request line, write an EGRESS response (a healthy
    /// daemon serving a `register_egress`).
    ServeHelloAndEgress,
}

/// An in-process fake `cerulion-netd` control server bound to a unique temp socket.
/// The Nth accepted connection follows `behaviors[N]` (the last entry repeats for
/// any further connections). Held for its `Drop`, which stops the accept thread.
struct FakeDaemon {
    stop: std::sync::Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
    /// Count of accepted connections — a retry opens a SECOND connection, so this
    /// pins whether the client reconnected.
    accepted: std::sync::Arc<AtomicUsize>,
    sock: PathBuf,
    dir: PathBuf,
}

impl FakeDaemon {
    fn spawn(tag: &str, behaviors: Vec<ConnBehavior>) -> Self {
        use std::io::{Read, Write};
        use std::os::unix::net::UnixListener;

        let (dir, sock) = unique_socket(tag);
        let listener = UnixListener::bind(&sock).expect("bind fake daemon socket");
        listener.set_nonblocking(true).expect("nonblocking");
        let stop = std::sync::Arc::new(AtomicBool::new(false));
        let stop_thread = std::sync::Arc::clone(&stop);
        let accepted = std::sync::Arc::new(AtomicUsize::new(0));
        let accepted_thread = std::sync::Arc::clone(&accepted);
        let handle = std::thread::spawn(move || {
            let mut conn_index = 0usize;
            while !stop_thread.load(Ordering::SeqCst) {
                match listener.accept() {
                    Ok((mut stream, _)) => {
                        accepted_thread.fetch_add(1, Ordering::SeqCst);
                        // Pick this connection's behavior (last entry repeats).
                        let behavior = behaviors
                            .get(conn_index)
                            .copied()
                            .or_else(|| behaviors.last().copied())
                            .unwrap_or(ConnBehavior::CloseImmediately);
                        conn_index += 1;
                        stream.set_read_timeout(Some(Duration::from_secs(2))).ok();
                        stream.set_write_timeout(Some(Duration::from_secs(2))).ok();
                        match behavior {
                            ConnBehavior::CloseImmediately => { /* drop → close */ }
                            ConnBehavior::HelloThenClose => {
                                let _ = writeln!(
                                    stream,
                                    "{}",
                                    cerulion_netd::protocol::Hello::new().to_json_line()
                                );
                                // drop → close before serving any request.
                            }
                            ConnBehavior::ServeHelloAndDemand => {
                                let _ = writeln!(
                                    stream,
                                    "{}",
                                    cerulion_netd::protocol::Hello::new().to_json_line()
                                );
                                // Read exactly one request line (the demand).
                                let mut buf = [0u8; 4096];
                                let mut pending: Vec<u8> = Vec::new();
                                let mut served = false;
                                let deadline = Instant::now() + Duration::from_secs(2);
                                while !served && Instant::now() < deadline {
                                    match stream.read(&mut buf) {
                                        Ok(0) => break,
                                        Ok(n) => {
                                            pending.extend_from_slice(&buf[..n]);
                                            if pending.contains(&b'\n') {
                                                let resp =
                                                    cerulion_netd::protocol::Response::Demand(
                                                        cerulion_netd::protocol::DemandResponse {
                                                            id: 1,
                                                            robot: "ubuntu".to_string(),
                                                            topic: "/tf".to_string(),
                                                            refcount: 1,
                                                            mirror_created: true,
                                                        },
                                                    );
                                                let _ = writeln!(stream, "{}", resp.to_json_line());
                                                served = true;
                                            }
                                        }
                                        Err(e)
                                            if e.kind() == io::ErrorKind::WouldBlock
                                                || e.kind() == io::ErrorKind::TimedOut =>
                                        {
                                            std::thread::sleep(Duration::from_millis(5));
                                        }
                                        Err(_) => break,
                                    }
                                }
                            }
                            ConnBehavior::ServeHelloAndEgress => {
                                let _ = writeln!(
                                    stream,
                                    "{}",
                                    cerulion_netd::protocol::Hello::new().to_json_line()
                                );
                                // Read exactly one request line (the register_egress).
                                let mut buf = [0u8; 4096];
                                let mut pending: Vec<u8> = Vec::new();
                                let mut served = false;
                                let deadline = Instant::now() + Duration::from_secs(2);
                                while !served && Instant::now() < deadline {
                                    match stream.read(&mut buf) {
                                        Ok(0) => break,
                                        Ok(n) => {
                                            pending.extend_from_slice(&buf[..n]);
                                            if pending.contains(&b'\n') {
                                                let resp =
                                                    cerulion_netd::protocol::Response::Egress(
                                                        cerulion_netd::protocol::EgressResponse {
                                                            id: 1,
                                                            registered_topics: 1,
                                                            gateway_started: true,
                                                        },
                                                    );
                                                let _ = writeln!(stream, "{}", resp.to_json_line());
                                                served = true;
                                            }
                                        }
                                        Err(e)
                                            if e.kind() == io::ErrorKind::WouldBlock
                                                || e.kind() == io::ErrorKind::TimedOut =>
                                        {
                                            std::thread::sleep(Duration::from_millis(5));
                                        }
                                        Err(_) => break,
                                    }
                                }
                            }
                        }
                    }
                    Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                        std::thread::sleep(Duration::from_millis(5));
                    }
                    Err(_) => break,
                }
            }
        });
        FakeDaemon {
            stop,
            handle: Some(handle),
            accepted,
            sock,
            dir,
        }
    }

    /// How many connections the fake daemon has accepted (a retry = a 2nd connect).
    fn accepted(&self) -> usize {
        self.accepted.load(Ordering::SeqCst)
    }
}

impl Drop for FakeDaemon {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

#[test]
fn client_retries_once_on_hello_close_then_succeeds() {
    // The dominant idle-exit race signature: accepted then closed
    // BEFORE the Hello. The client's connect_or_spawn_at must retry ONCE (silently)
    // and succeed on the second connection — the fake server keeps the socket bound,
    // so the retry re-connects (no spawn) and gets a Hello.
    // No env mutation / no spawn: the fake server keeps the socket bound, so
    // `try_connect` always succeeds and the spawn path is never reached — fully
    // parallel-safe (unique socket per fake).
    let fake = FakeDaemon::spawn(
        "retry_hello",
        vec![
            ConnBehavior::CloseImmediately,
            ConnBehavior::ServeHelloAndDemand,
        ],
    );

    let mut client =
        NetdClient::connect_or_spawn_at(fake.sock.clone()).expect("retry recovers the handshake");
    let d = client
        .demand("ubuntu", "/tf", 7)
        .expect("demand served on the retried connection");
    assert!(d.mirror_created);
    assert_eq!(d.topic, "/tf");
    drop(client);
}

#[test]
fn client_retries_once_on_first_demand_close_then_succeeds() {
    // The first-request race: the daemon serves the Hello then closes before the
    // demand (accepted-then-EOF on the first request). The demand must retry ONCE
    // via reconnect and succeed on the second connection.
    let fake = FakeDaemon::spawn(
        "retry_demand",
        vec![
            ConnBehavior::HelloThenClose,
            ConnBehavior::ServeHelloAndDemand,
        ],
    );

    // Connect succeeds (Hello served on conn 1), the demand EOFs (conn 1 closed) →
    // the client reconnects (conn 2) and re-sends → served.
    let mut client =
        NetdClient::connect_or_spawn_at(fake.sock.clone()).expect("first connection handshakes");
    let d = client
        .demand("ubuntu", "/tf", 7)
        .expect("demand retried on the second connection");
    assert!(d.mirror_created);
    drop(client);
}

#[test]
fn client_surfaces_a_loud_error_after_two_early_closes() {
    // Anti-tautology: a daemon that closes EVERY connection at the Hello is a
    // persistent wedge, not a transient race. The single retry does not mask it —
    // the give-up surfaces a LOUD ClientError (never a silent hang).
    //
    // An `assert_eq!` of the give-up error's kind to EXACTLY
    // `UnexpectedEof` is over-specific and flakes on macOS (observed
    // `left: InvalidInput`). An accepted-then-closed socket surfaces as one of TWO
    // loud `Io` errors, depending on which side of the peer's close the client's
    // socket setup lands (`NetdClient::finish_handshake`):
    //
    //   * `UnexpectedEof` / "cerulion-netd closed the connection" — the DOMINANT
    //     path: the `set_{read,write}_timeout` calls run on a still-live socket and
    //     the Hello read then hits a clean EOF, which `read_line` turns into this
    //     synthesized error. Measured 19_999/20_000 on this fixture's shape.
    //   * `InvalidInput` / "Invalid argument (os error 22)" — the close is ALREADY
    //     processed when the client sets its socket timeouts, and macOS refuses
    //     every `setsockopt` on a fully-shutdown socket with EINVAL (xnu
    //     `sosetoptlock`), so the handshake fails at the timeout call BEFORE it ever
    //     reads. Verified by forcing the close to land first: both
    //     `set_read_timeout` AND `set_write_timeout` then fail EINVAL 50/50.
    //
    // BOTH are correct, LOUD behavior — the client reports a broken connection and
    // never hangs or silently degrades, which is the property this test exists to
    // prove. So the assertions below pin that property, and keep the verbatim-text
    // and reconnect pins at FULL strength on the arm where each is invariant.
    let fake = FakeDaemon::spawn("retry_giveup", vec![ConnBehavior::CloseImmediately]);

    let err = NetdClient::connect_or_spawn_at(fake.sock.clone())
        .expect_err("two early closes surface a loud error");
    // LOUD and attributable: an `Io` error (never `Protocol`/`Netd`, never Ok) whose
    // kind is one the client ITSELF classifies as an accepted-then-closed signature
    // (`is_closed_early`), plus the `InvalidInput` arm the OS substitutes for it at
    // the setsockopt. Anything outside that set is a genuine regression.
    let io_err = match &err {
        ClientError::Io(e) => e,
        other => panic!("expected a loud Io error after two early closes, got {other:?}"),
    };
    let kind = io_err.kind();
    assert!(
        is_closed_connection_kind(kind) || kind == io::ErrorKind::InvalidInput,
        "expected a closed-connection Io error, got {kind:?} ({err})"
    );
    // The loud text is never swallowed on ANY arm: the operator-visible string
    // attributes the failure to netd I/O and interpolates the underlying cause.
    let msg = err.to_string();
    assert!(
        msg.contains("cerulion-netd I/O error") && msg.contains(&io_err.to_string()),
        "the give-up error is attributed to netd I/O and carries its cause: {msg}"
    );
    // On the EOF arm the message is still pinned VERBATIM — a regression
    // that kept the KIND but garbled that exact operator-visible string fails.
    if kind == io::ErrorKind::UnexpectedEof {
        assert!(
            msg.contains("cerulion-netd closed the connection"),
            "the loud error preserves the accepted-then-closed message: {msg}"
        );
    }
    // The retry actually reconnected — exactly TWO connections were accepted, on
    // EVERY arm.
    //
    // `connect_or_spawn_at` retries iff the FIRST attempt's error satisfies
    // `is_closed_early`. That classifier set is EXACTLY the kinds asserted above —
    // the closed-connection family (`is_closed_connection_kind`: EOF / BrokenPipe /
    // ConnectionReset / the macOS `NotConnected`) plus the `InvalidInput` the OS
    // substitutes at the setsockopt when the peer's close wins the race. So a
    // surfaced error of ANY kind this test permits CANNOT be an
    // un-retried first attempt (that one would have been retried); it is necessarily
    // the SECOND attempt's, hence exactly two connections.
    //
    // There is no `(1..=2)` band for an EINVAL arm: `InvalidInput` IS classified
    // as an early close, so an EINVAL first attempt is retried like any other and
    // `1` is unreachable — measured 300/300 `accepted == 2` with the
    // classification vs 300/300 `accepted == 1` without it, i.e. `1` is precisely the
    // UNCLASSIFIED outcome, and a band admitting it would admit the failure. With
    // the classifier set and the
    // permitted-kind set identical, one unconditional assertion is both simpler
    // and strictly stronger than a branch.
    //
    // SCOPE — this is not what pins the `InvalidInput` classification. The EINVAL arm is reached on
    // ~0.02-0.05 % of macOS runs (measured at 6/30_000,
    // 2/4000 and 1/20_000) and NEVER on Linux, so as
    // a check for that classification it is a lottery ticket: reverting it
    // passes this test 60 times out of 60. What catches the
    // revert deterministically is `client.rs`'s composed
    // `a_daemon_refusal_that_lands_before_the_handshake_is_still_retryable`
    // (measured 25/25 and 200/200 on macOS). The value here is that the assertion
    // never describes the unclassified behaviour as correct.
    assert!(
        wait_until(|| fake.accepted() == 2, Duration::from_secs(1)),
        "the single retry opened a second connection (got {}, kind {kind:?})",
        fake.accepted()
    );
}

#[test]
fn a_non_first_demand_early_close_does_not_retry_and_surfaces_loudly() {
    // The demand retry is guarded to the FIRST request only.
    // A MID-SESSION demand (next_id > 1) that hits an early close must NOT reconnect
    // — silently reconnecting would lose every prior demand the closed connection
    // held daemon-side (and could double-execute). It must surface a loud error, and
    // the client must NOT open a second connection.
    let fake = FakeDaemon::spawn("nonfirst_noretry", vec![ConnBehavior::ServeHelloAndDemand]);

    // Connect (Hello) + demand#1 is served on conn 1; conn 1 then closes.
    let mut client = NetdClient::connect_or_spawn_at(fake.sock.clone()).expect("handshake ok");
    let d = client
        .demand("ubuntu", "/tf", 7)
        .expect("first demand served");
    assert!(d.mirror_created, "the first demand was served");

    // demand#2 (next_id == 2, NOT first) hits the closed connection → loud error, NO
    // retry.
    let err = client
        .demand("ubuntu", "/odom", 9)
        .expect_err("a mid-session demand early close is not retried");
    // A closed-connection Io error (the write may EPIPE or the read may EOF depending
    // on OS timing) — the point is it is LOUD, not silently retried.
    match err {
        ClientError::Io(e) => assert!(
            is_closed_connection_kind(e.kind()),
            "expected a closed-connection Io error, got {e:?}"
        ),
        other => panic!("expected a loud Io error on the mid-session close, got {other:?}"),
    }
    // Only ONE connection was ever accepted — the client did NOT reconnect.
    assert_eq!(
        fake.accepted(),
        1,
        "a non-first demand must not reconnect (no double-execute / lost prior demands)"
    );
    drop(client);
}

#[test]
fn a_first_demand_retry_that_closes_again_surfaces_loudly() {
    // The DEMAND-level retry give-up path (distinct from the
    // connect-level give-up). The daemon serves the Hello then closes before the
    // demand on BOTH connections: the first-request demand EOFs → reconnect →
    // handshake ok → retried demand EOFs again → a loud error (never a silent hang).
    let fake = FakeDaemon::spawn(
        "demand_retry_giveup",
        vec![ConnBehavior::HelloThenClose, ConnBehavior::HelloThenClose],
    );

    // connect_or_spawn_at gets the Hello on conn 1 (handshake ok).
    let mut client = NetdClient::connect_or_spawn_at(fake.sock.clone()).expect("conn1 handshake");
    // demand#1 (first request) EOFs on conn 1 → reconnect → conn 2 handshake ok →
    // retried demand EOFs on conn 2 → loud error.
    let err = client
        .demand("ubuntu", "/tf", 7)
        .expect_err("the retried demand also closes → loud error");
    match err {
        ClientError::Io(e) => assert!(
            is_closed_connection_kind(e.kind()),
            "expected a closed-connection Io error, got {e:?}"
        ),
        other => panic!("expected a loud Io error after the demand retry, got {other:?}"),
    }
    // Exactly TWO connections were accepted (the reconnect happened once).
    assert!(
        wait_until(|| fake.accepted() == 2, Duration::from_secs(1)),
        "the demand retry reconnected exactly once (got {})",
        fake.accepted()
    );
    drop(client);
}

#[test]
fn client_register_egress_retries_once_on_first_request_close_then_succeeds() {
    // `register_egress` carries the same first-request
    // retry as `demand` — the FIRST request on a connection can race the daemon's
    // idle-exit / shutdown (Hello served, then closed before the register lands). The
    // daemon serves the Hello then closes before the register (accepted-then-EOF on the
    // first request) → register_egress must retry ONCE via reconnect and succeed on the
    // second connection.
    let fake = FakeDaemon::spawn(
        "reg_egress_retry",
        vec![
            ConnBehavior::HelloThenClose,
            ConnBehavior::ServeHelloAndEgress,
        ],
    );

    // Connect succeeds (Hello served on conn 1), the register EOFs (conn 1 closed) →
    // the client reconnects (conn 2) and re-sends → served.
    let mut client =
        NetdClient::connect_or_spawn_at(fake.sock.clone()).expect("first connection handshakes");
    let resp = client
        .register_egress(
            &egress_plan(&["/robot/cmd_vel"]),
            &SchemaServing::default(),
            None,
        )
        .expect("register_egress retried on the second connection");
    assert_eq!(resp.registered_topics, 1);
    assert!(
        resp.gateway_started,
        "the retried register served the egress response"
    );
    // The retry actually reconnected — exactly TWO connections were accepted.
    assert!(
        wait_until(|| fake.accepted() == 2, Duration::from_secs(1)),
        "the register_egress retry opened a second connection (got {})",
        fake.accepted()
    );
    drop(client);
}

#[test]
fn a_first_register_egress_retry_that_closes_again_surfaces_loudly() {
    // The give-up path for register_egress (twin of the demand give-up): the daemon
    // serves the Hello then closes before the register on BOTH connections. The
    // first-request register EOFs → reconnect → handshake ok → retried register EOFs
    // again → a LOUD error (never a silent hang), exactly ONE reconnect.
    let fake = FakeDaemon::spawn(
        "reg_egress_retry_giveup",
        vec![ConnBehavior::HelloThenClose, ConnBehavior::HelloThenClose],
    );

    let mut client = NetdClient::connect_or_spawn_at(fake.sock.clone()).expect("conn1 handshake");
    let err = client
        .register_egress(
            &egress_plan(&["/robot/cmd_vel"]),
            &SchemaServing::default(),
            None,
        )
        .expect_err("the retried register also closes → loud error");
    match err {
        ClientError::Io(e) => assert!(
            is_closed_connection_kind(e.kind()),
            "expected a closed-connection Io error, got {e:?}"
        ),
        other => panic!("expected a loud Io error after the register_egress retry, got {other:?}"),
    }
    // Exactly TWO connections were accepted (the reconnect happened once).
    assert!(
        wait_until(|| fake.accepted() == 2, Duration::from_secs(1)),
        "the register_egress retry reconnected exactly once (got {})",
        fake.accepted()
    );
    drop(client);
}

// --------------------------------------------------------------------------
// The protocol-version COMPAT contract at the
// handshake — a client accepts a daemon whose vocabulary includes its baseline
// (demand/release/status = v1), refusing only a daemon too old for the baseline.
// Driven with CRAFTED Hello banners over a raw UnixListener (no real daemon), so
// both directions (older/newer daemon) are exercised deterministically.
// --------------------------------------------------------------------------

/// Bind a raw UnixListener that answers ONE connection with a Hello banner carrying
/// `protocol`, then hold the connection open (read to EOF) so the client's handshake
/// completes. Returns the socket path + tempdir (kept alive by the caller).
fn serve_crafted_banner(tag: &str, protocol: u32) -> (PathBuf, PathBuf) {
    use std::io::{Read, Write};
    use std::os::unix::net::UnixListener;

    let (dir, sock) = unique_socket(tag);
    let listener = UnixListener::bind(&sock).expect("bind crafted-banner listener");
    std::thread::spawn(move || {
        if let Ok((mut stream, _)) = listener.accept() {
            let banner = Hello {
                hello: HELLO_MARKER.to_string(),
                protocol,
            }
            .to_json_line();
            let _ = writeln!(stream, "{banner}");
            let _ = stream.flush();
            // Hold the connection until the client drops (read to EOF) so an ACCEPTED
            // handshake sees a live connection, not a mid-banner close.
            let mut buf = [0u8; 256];
            while let Ok(n) = stream.read(&mut buf) {
                if n == 0 {
                    break;
                }
            }
        }
    });
    (dir, sock)
}

#[test]
fn client_accepts_an_older_daemon_for_the_baseline_and_a_newer_daemon_always() {
    // This client speaks the CURRENT PROTOCOL_VERSION (>= 2); the contract accepts a
    // daemon whose vocabulary includes the v1 baseline (demand/release/status).

    // (1) A PINNED OLD v1 daemon — the "post-upgrade consumer" case a strict
    //     equality check would WEDGE. It is accepted (demand/release still work).
    let (dir1, sock1) = serve_crafted_banner("banner_v1", 1);
    NetdClient::connect_or_spawn_at(sock1).expect("a v2 client accepts a v1 daemon (baseline)");
    let _ = std::fs::remove_dir_all(&dir1);

    // (2) A same-version daemon — accepted.
    let (dir2, sock2) = serve_crafted_banner("banner_cur", PROTOCOL_VERSION);
    NetdClient::connect_or_spawn_at(sock2).expect("accepts the same version");
    let _ = std::fs::remove_dir_all(&dir2);

    // (3) A NEWER daemon (its vocabulary is a superset) — accepted.
    let (dir3, sock3) = serve_crafted_banner("banner_newer", PROTOCOL_VERSION + 7);
    NetdClient::connect_or_spawn_at(sock3).expect("accepts a newer daemon (superset)");
    let _ = std::fs::remove_dir_all(&dir3);
}

#[test]
fn client_refuses_a_daemon_too_old_for_the_baseline() {
    // A v0 daemon is below the v1 baseline this consumer needs → refused LOUDLY at the
    // handshake, naming the daemon version + the upgrade fix (never a silent hang, and
    // never a spawn — the connect succeeded, the banner was just too old).
    let (dir, sock) = serve_crafted_banner("banner_v0", 0);
    let err = NetdClient::connect_or_spawn_at(sock).expect_err("a v0 daemon is refused");
    match err {
        ClientError::Protocol(msg) => {
            assert!(msg.contains("v0"), "names the daemon version: {msg}");
            assert!(
                msg.contains("Upgrade cerulion-netd"),
                "names the remedy: {msg}"
            );
        }
        other => panic!("expected a Protocol version error, got {other:?}"),
    }
    let _ = std::fs::remove_dir_all(&dir);
}

// --------------------------------------------------------------------------
// The client's catalog/schema query methods (connect-to-running).
// --------------------------------------------------------------------------

#[test]
fn client_query_catalog_and_schema_round_trip_against_a_real_daemon() {
    let (netd, dir, sock, query_spy) = start_daemon_with_query("query");
    *query_spy.catalogs.lock().unwrap() = vec![CatalogReply {
        version: 1,
        robot: "ubuntu".to_string(),
        entries: vec![CatalogEntry {
            topic: "/tf".to_string(),
            schema_hash: Some(7),
            schema_name: Some("tf2_msgs/TFMessage".to_string()),
            provenance: CatalogProvenance::Runtime,
            producer_count: None,
            liveness: None,
        }],
        error: None,
    }];
    *query_spy.schemas.lock().unwrap() =
        vec![SchemaReply::found("ubuntu", "tf2_msgs/TFMessage", vec![])];

    let mut client = NetdClient::connect_or_spawn_at(sock.clone()).expect("connect");
    // query_catalog decodes the CatalogQuery response into the reply vec.
    let catalogs = client.query_catalog(None).expect("catalog ok");
    assert_eq!(catalogs.len(), 1);
    assert_eq!(catalogs[0].robot, "ubuntu");
    assert_eq!(catalogs[0].entries[0].topic, "/tf");
    assert_eq!(
        catalogs[0].entries[0].schema_name.as_deref(),
        Some("tf2_msgs/TFMessage")
    );
    // query_schema decodes the SchemaQuery response.
    let replies = client
        .query_schema(Some("ubuntu"), "tf2_msgs/TFMessage")
        .expect("schema ok");
    assert_eq!(replies.len(), 1);
    assert_eq!(replies[0].requested, "tf2_msgs/TFMessage");

    drop(client);
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

/// The client's `query_runs` family against a REAL daemon: the three
/// verbs decode the same response and scope the GET the caller asked for.
///
/// The load-bearing assertion is the LAST one: a robot that did not answer is ABSENT
/// from the list, and the client must never synthesize an empty reply for it. A
/// desk-side fold keys on reply presence, so a fabricated entry would turn "we could
/// not ask that robot" into "that robot is running nothing".
#[test]
fn client_query_runs_round_trips_against_a_real_daemon() {
    let (netd, dir, sock, query_spy) = start_daemon_with_query("qruns");
    *query_spy.runs.lock().unwrap() = vec![RunsReply {
        version: 1,
        robot: "ubuntu".to_string(),
        runs: vec![cerulion_core::RunEntry {
            run_id: "0x0000000000000000000000000000002a".to_string(),
            graph_name: "perception".to_string(),
            run_started_at_ns: 1_700_000_000_000_000_000,
            state: cerulion_core::RunEntryState::Live,
            graph_yaml: "name: perception\nnodes:\n  - id: cam\n".to_string(),
            run_json: "{\"run_id\":\"0x2a\"}\n".to_string(),
        }],
        completeness: cerulion_core::RunsCompleteness::Settled,
        undescribable: Vec::new(),
        error: None,
    }];

    let mut client = NetdClient::connect_or_spawn_at(sock.clone()).expect("connect");

    // The plain verb drops the discovery state and yields the answer.
    let answer = client.query_runs(None).expect("runs ok");
    let replies = &answer.replies;
    assert_eq!(replies.len(), 1);
    assert_eq!(replies[0].robot, "ubuntu");
    assert_eq!(replies[0].runs.len(), 1);
    assert_eq!(replies[0].runs[0].graph_name, "perception");
    assert_eq!(
        replies[0].runs[0].graph_yaml, "name: perception\nnodes:\n  - id: cam\n",
        "the effective graph document crosses the UDS seam verbatim"
    );
    assert!(
        answer.unusable.is_empty(),
        "no robot answered unusably in this scenario"
    );

    // The `_with_discovery` verb carries the marker a consumer needs before it can
    // read an empty answer as absence.
    let gather = client
        .query_runs_with_discovery(Some("ubuntu"))
        .expect("runs ok");
    assert_eq!(gather.replies.len(), 1);
    assert_eq!(gather.discovery, DiscoveryState::Settled);

    // The no-respawn verb (the recorder-safe shape) decodes the same response.
    let answer = client.query_runs_no_respawn(None).expect("runs ok");
    assert_eq!(answer.replies.len(), 1);

    // A plane that gathered NOTHING yields an EMPTY list — never a fabricated reply
    // for a robot that did not answer.
    query_spy.runs.lock().unwrap().clear();
    let answer = client.query_runs(Some("go2")).expect("runs ok");
    assert!(
        answer.replies.is_empty(),
        "a robot that did not answer contributes NOTHING — a synthesized empty reply \
         would read as 'that robot is running nothing', which nobody established"
    );

    // Scoping reached the plane in order, `None` for the fan-out — asserted AFTER
    // the last call, not before it. Read early, this oracle covers only the calls
    // made so far, so a filter dropped on ONE verb (here the plain `query_runs`,
    // whose scope is otherwise unobservable — a scoped and an unscoped call return
    // the same canned answer) sails through.
    assert_eq!(
        query_spy.runs_robots.lock().unwrap().clone(),
        vec![
            None,
            Some("ubuntu".to_string()),
            None,
            Some("go2".to_string())
        ],
        "every call must scope the GET the caller asked for, in order — the LAST one \
         included"
    );

    drop(client);
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

/// **A robot that ANSWERED UNUSABLY reaches the PRIMARY verb's caller.**
///
/// `unusable` gives the wire a third state — answered / silent /
/// answered-with-bytes-we-cannot-use — because the three have different REMEDIES: read
/// it, wait for it, redeploy that robot. `NetdClient::query_runs` is the verb a
/// consumer reaches for, and were it to return a bare `Vec<RunsReply>` the third state
/// would arrive as `Ok([])` — wire-indistinguishable from silence, so the one remedy that
/// is actionable would be the one no caller could name.
///
/// The load-bearing shape is the FIRST arm: `replies` EMPTY while `unusable` names the
/// robot AND its reason. An assertion on `unusable` alone would pass a verb that also
/// invented a reply.
#[test]
fn an_unusable_answer_reaches_the_primary_query_runs_caller() {
    let (netd, dir, sock, query_spy) = start_daemon_with_query("qrunsunusable");
    // The shape exactly: a robot ANSWERED, nothing decodable came of it.
    *query_spy.unusable_runs.lock().unwrap() = vec![UnusableRunsAnswer {
        robot: "go2".to_string(),
        reason: "runs reply version 9 is newer than this binary understands".to_string(),
    }];

    let mut client = NetdClient::connect_or_spawn_at(sock.clone()).expect("connect");

    for (verb, answer) in [
        (
            "query_runs",
            client.query_runs(Some("go2")).expect("runs ok"),
        ),
        (
            "query_runs_no_respawn",
            client.query_runs_no_respawn(Some("go2")).expect("runs ok"),
        ),
    ] {
        assert!(
            answer.replies.is_empty(),
            "{verb}: a robot whose answer could not be used contributes no reply"
        );
        assert_eq!(
            answer.unusable,
            vec![UnusableRunsAnswer {
                robot: "go2".to_string(),
                reason: "runs reply version 9 is newer than this binary understands".to_string(),
            }],
            "{verb}: the robot AND the operator-readable reason must survive to the \
             caller — without the reason there is nothing to render but 'something is \
             wrong somewhere'"
        );
    }

    // ANTI-TAUTOLOGY, and the CONTRAST that makes `unusable` a third state rather
    // than a rename of silence: a robot that did not answer AT ALL lands on the
    // SILENT list, never this one. (Without a silent list it would land on no list at all,
    // which would make a half-answered LAN indistinguishable from a complete one
    // — see the coverage arm below.)
    query_spy.unusable_runs.lock().unwrap().clear();
    *query_spy.silent_runs.lock().unwrap() = vec!["go2".to_string()];
    let answer = client.query_runs(Some("go2")).expect("runs ok");
    assert!(answer.replies.is_empty());
    assert!(
        answer.unusable.is_empty(),
        "silence is not a skew — a verb that always reported something would make the \
         arms above vacuous"
    );
    assert_eq!(
        answer.silent,
        vec!["go2".to_string()],
        "…and the silence is NAMED: the two absences have opposite remedies (wait or \
         upgrade vs redeploy), so collapsing them would hide the remedy"
    );

    drop(client);
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

/// **The COVERAGE half survives the real UDS round trip**: which
/// robots were ASKED and never answered, on both `RunsAnswer`-returning verbs.
///
/// `replies` alone cannot distinguish a LAN where every robot answered from one
/// where half stayed silent: both arrive as a list of replies. A consumer folding
/// either into "these are the runs" then makes a SETTLED ABSENCE claim about
/// machines nobody heard from, the confident-empty class already killed at the
/// discovery latch, reached one layer up through the coverage door.
///
/// The sibling arm above drives silence with NOTHING else in flight; this one
/// drives the SHAPE a real gather produces — a robot that answered, a robot whose
/// answer was unusable, and two that said nothing — because the three lists are
/// three remedies and the failure mode is one bleeding into another. The
/// partition is asserted whole rather than one list at a time: a per-list arm
/// cannot see a robot appearing on two of them.
#[test]
fn the_silent_robots_survive_the_round_trip_to_the_primary_caller() {
    let (netd, dir, sock, query_spy) = start_daemon_with_query("qrunssilent");
    *query_spy.runs.lock().unwrap() = vec![RunsReply {
        version: 1,
        robot: "go2".to_string(),
        runs: Vec::new(),
        completeness: cerulion_core::RunsCompleteness::Settled,
        undescribable: Vec::new(),
        error: None,
    }];
    *query_spy.unusable_runs.lock().unwrap() = vec![UnusableRunsAnswer {
        robot: "orin".to_string(),
        reason: "runs reply version 9 is newer than this binary understands".to_string(),
    }];
    *query_spy.silent_runs.lock().unwrap() = vec!["spot".to_string(), "arm".to_string()];

    let mut client = NetdClient::connect_or_spawn_at(sock.clone()).expect("connect");

    for (verb, answer) in [
        ("query_runs", client.query_runs(None).expect("runs ok")),
        (
            "query_runs_no_respawn",
            client.query_runs_no_respawn(None).expect("runs ok"),
        ),
    ] {
        assert_eq!(
            answer.silent,
            vec!["spot".to_string(), "arm".to_string()],
            "{verb}: the robots that were asked and never answered must reach the \
             caller, IN ORDER — without them `replies` reads as complete coverage"
        );
        // The partition: each robot on exactly one list, none bleeding across.
        assert_eq!(answer.replies.len(), 1, "{verb}");
        assert_eq!(answer.replies[0].robot, "go2", "{verb}");
        assert_eq!(answer.unusable.len(), 1, "{verb}");
        assert_eq!(answer.unusable[0].robot, "orin", "{verb}");
        assert!(
            !answer.silent.iter().any(|r| r == "go2" || r == "orin"),
            "{verb}: a robot that ANSWERED is not silent: {:?}",
            answer.silent
        );
    }

    // ANTI-TAUTOLOGY: a fully-covered LAN reports NOTHING silent, so the arms above
    // are the plane's answer rather than a field this client always populates.
    query_spy.silent_runs.lock().unwrap().clear();
    let answer = client.query_runs(None).expect("runs ok");
    assert!(answer.silent.is_empty());
    assert_eq!(answer.replies.len(), 1);

    drop(client);
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

#[test]
fn client_query_against_a_mirror_only_daemon_is_a_loud_netd_error() {
    // A mirror-only daemon (daemon::start injects the NoopQueryPlane) has no query
    // surface — a query surfaces the daemon's structured error as a LOUD
    // ClientError::Netd (never a silent empty the desk cannot distinguish from
    // "nobody answered"). The desk maps this to its transient-session fallback.
    let (netd, dir, sock, _spy) = start_daemon("mirroronly");
    let mut client = NetdClient::connect_or_spawn_at(sock.clone()).expect("connect");
    match client.query_catalog(None) {
        Err(ClientError::Netd { error, .. }) => {
            assert!(error.contains("catalog query failed"), "{error}");
            assert!(error.contains("not network-configured"), "{error}");
        }
        other => panic!("expected a loud ClientError::Netd, got {other:?}"),
    }
    match client.query_schema(Some("go2"), "pkg/Type") {
        Err(ClientError::Netd { error, .. }) => {
            assert!(
                error.contains("schema query for 'pkg/Type' failed"),
                "{error}"
            );
        }
        other => panic!("expected a loud ClientError::Netd, got {other:?}"),
    }
    // The runs verb degrades the same way. A mirror-only daemon SPEAKS
    // v7 (its banner is this binary's), so the per-verb gate passes and the request
    // really is served — by the `NoopQueryPlane`, which refuses explicitly. An empty
    // list here would be indistinguishable from "netd asked and nobody answered".
    match client.query_runs(Some("go2")) {
        Err(ClientError::Netd { error, .. }) => {
            assert!(error.contains("runs query failed"), "{error}");
            assert!(error.contains("not network-configured"), "{error}");
        }
        other => panic!("expected a loud ClientError::Netd, got {other:?}"),
    }
    drop(client);
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

// --------------------------------------------------------------------------
// `connect_existing` — connect to a RUNNING daemon, never spawn one.
// --------------------------------------------------------------------------

/// A recorder must be able to reach a netd that is already up.
///
/// The ANTI-TAUTOLOGY half of the pair below: without this arm, "never spawns"
/// is trivially satisfied by a constructor that never connects to anything.
#[test]
fn connect_existing_reaches_a_running_daemon() {
    let (netd, dir, sock, spy) = start_daemon("cexist");

    let mut client =
        NetdClient::connect_existing_at(sock.clone()).expect("a live daemon must be reachable");
    client
        .demand("go2", "/lidar", 0xABCD)
        .expect("the connection is a full client, not a half-open handshake");
    assert_eq!(
        spy.ensure_ok.load(Ordering::SeqCst),
        1,
        "the demand really reached the daemon"
    );

    drop(client);
    let _ = std::fs::remove_dir_all(&dir);
    drop(netd);
}

/// THE property: with no daemon listening, `connect_existing_at` fails LOUDLY
/// and **starts nothing**.
///
/// The discriminator is the error VARIANT, not a wall. `NETD_BIN_ENV` points at
/// a path that does not exist, so on the SAME dead socket:
///
/// * `connect_or_spawn_at` reaches its spawn step and fails `ClientError::Spawn`
///   — proof that this socket really is a spawn-triggering "not running" state,
///   which is what makes the other half meaningful;
/// * `connect_existing_at` fails `ClientError::Connect` — it never reached a
///   spawn step at all.
///
/// A wall assertion would be the weaker oracle here: a loaded runner can stretch
/// any duration, while the variant a run took cannot be faked by load. No daemon
/// process is created either way, so the test costs nothing and cannot leak one.
#[test]
fn connect_existing_never_spawns_a_daemon() {
    let _lock = env_lock();
    let (dir, sock) = unique_socket("cnospawn");
    // Nothing is listening: `unique_socket` only makes the directory.
    let missing_bin = dir.join("there-is-no-netd-here");
    let _b = EnvGuard::set(NETD_BIN_ENV, missing_bin.to_str().unwrap());

    match NetdClient::connect_existing_at(sock.clone()) {
        Err(ClientError::Connect { socket, .. }) => {
            assert_eq!(socket, sock, "the error must name the socket it tried");
        }
        other => panic!("expected a loud ClientError::Connect, got {other:?}"),
    }

    // The control: the SPAWNING constructor gets as far as the spawn on this
    // very socket. If this arm ever stops being `Spawn`, the arm above proves
    // nothing about spawning.
    match NetdClient::connect_or_spawn_at(sock.clone()) {
        Err(ClientError::Spawn { bin, .. }) => {
            assert_eq!(
                bin, missing_bin,
                "the control must have reached the spawn step"
            );
        }
        other => panic!("expected the spawning constructor to reach its spawn step, got {other:?}"),
    }

    let _ = std::fs::remove_dir_all(&dir);
}

/// The no-respawn query verbs make ONE round trip and report an early
/// close, rather than answering it by starting a daemon.
///
/// `connect_existing` exists so a RECORDER can ask whether a daemon is there and
/// take "no" for an answer — it must not start a network daemon on the machine
/// it is recording, and must not block for the spawn-readiness ceiling inside
/// its arm-time window. That contract covered the CONSTRUCTOR only: the plain
/// `query_catalog` delegates to `query_catalog_with_discovery`, whose
/// `first_request && is_closed_early` arm calls `reconnect()` ->
/// `connect_or_spawn_at`, and `first_request` is `next_id == 1` — ALWAYS true of
/// the recorder's first query. MEASURED against this exact shape: a daemon
/// started and the call blocked 10.06 s.
///
/// The ORACLE is the fake's ACCEPT COUNT, the same one used for
/// the identical decision in the convergence loop: a reconnecting verb dials the
/// still-listening socket a SECOND time, which needs no wall assertion and
/// cannot be faked by load. Both verbs, and both PAIRED IN BODY with the plain
/// sibling over the same script — without that half, "did not reconnect" is
/// satisfied by a harness where nothing reconnects.
#[test]
fn the_no_respawn_query_verbs_do_not_reconnect_on_an_early_close() {
    for verb in ["catalog", "schema"] {
        let fake = FakeDaemon::spawn(
            &format!("norespawn_{verb}"),
            vec![ConnBehavior::HelloThenClose],
        );

        let mut client = NetdClient::connect_existing_at(fake.sock.clone()).expect("handshake ok");
        let err = match verb {
            "catalog" => client.query_catalog_no_respawn(None).unwrap_err(),
            _ => client
                .query_schema_no_respawn(None, "pkg/Type")
                .unwrap_err(),
        };
        // The trade this verb makes, stated as an assertion: an
        // accepted-then-EOF first query is an Err the caller handles, not a
        // transparently-retried success.
        let text = err.to_string();
        assert!(
            !text.is_empty(),
            "the failure must be reported, not swallowed"
        );
        assert_eq!(
            fake.accepted(),
            1,
            "the {verb} no-respawn verb must make ONE round trip and report the early close — \
             a second accepted connection is the respawn ladder the recorder must never enter"
        );

        // ANTI-TAUTOLOGY, same script, plain verb: it DOES reconnect, so the
        // assertion above is measuring the verb rather than the harness.
        let plain = FakeDaemon::spawn(
            &format!("respawn_{verb}"),
            vec![ConnBehavior::HelloThenClose],
        );
        let mut c2 = NetdClient::connect_or_spawn_at(plain.sock.clone()).expect("handshake ok");
        let _ = match verb {
            "catalog" => c2.query_catalog(None).map(|_| ()),
            _ => c2.query_schema(None, "pkg/Type").map(|_| ()),
        };
        assert!(
            wait_until(|| plain.accepted() >= 2, Duration::from_secs(2)),
            "precondition: the PLAIN {verb} verb really does re-dial on an early close (got {})",
            plain.accepted()
        );
    }
}
