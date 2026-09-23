// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion-netd` production entry.
//!
//! Thin wiring: parse args (`--help` / `--version`) → init tracing → `init` the
//! singleton
//! transport (Principle #8: ONE iceoryx2 node + ONE zenoh session, network-
//! configured by default but LAZY) → wrap it in the production
//! [`GatewayMirrorPlane`](cerulion_netd::mirror::GatewayMirrorPlane) → run the
//! daemon (the UDS NDJSON control server + the idle-watch thread) until
//! SIGINT/SIGTERM or the idle self-exit (by design), then shut down cleanly (drop the
//! session, close the socket, remove the pidfile).
//!
//! Normally you never launch netd by hand — the FIRST desk consumer that wants a
//! remote topic spawns it detached (the daemon's
//! singleton hygiene makes that spawn safe against races).

use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::transport::demand_authorizer::{AllowAllAuthorizer, DemandAuthorizer};
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::SchemaServing;

use cerulion_netd::catalog_events::{AnnounceWatchPlane, GatewayAnnounceWatchPlane};
use cerulion_netd::daemon::{self, NetdConfig};
use cerulion_netd::discovery_fold;
use cerulion_netd::egress::{EgressPlane, GatewayEgressPlane};
#[cfg(not(feature = "wan"))]
use cerulion_netd::mirror::GatewayMirrorPlane;
use cerulion_netd::mirror::MirrorPlane;
use cerulion_netd::net::LISTEN_ENV;
use cerulion_netd::query::{GatewayQueryPlane, QueryPlane};
use cerulion_netd::{default_socket_path, net};

/// The bounded teardown deadline. Once netd commits to exit (idle
/// self-exit OR a signal), graceful shutdown MUST finish within this or the daemon
/// HARD-exits — a wedged real-plane teardown (a blocked zenoh session close, a
/// stuck network-thread join, a mirror teardown that never returns) must NEVER
/// leave a zombie holding the control socket. `netd.shutdown()` unlinks the socket
/// FIRST, so a hard-exit here can never orphan the socket either. Overridable via
/// [`HARD_EXIT_ENV`] (ops + the subprocess test's short deadline).
const DEFAULT_HARD_EXIT_TIMEOUT: Duration = Duration::from_secs(5);

/// The env var overriding the hard-exit deadline, in MILLISECONDS.
/// Unset / invalid falls back to [`DEFAULT_HARD_EXIT_TIMEOUT`].
const HARD_EXIT_ENV: &str = "CERULION_NETD_HARD_EXIT_MS";

/// TEST-ONLY env seam (never set in normal operation): when present, `run` STALLS
/// after `netd.shutdown()` (which has already unlinked the socket) to SIMULATE a
/// teardown that wedges, so a subprocess test can prove the hard-exit
/// watchdog terminates a hung shutdown deterministically. The real wedge is a
/// blocked zenoh teardown, which needs a live robot to produce.
const STALL_SHUTDOWN_FOR_TEST_ENV: &str = "CERULION_NETD_STALL_SHUTDOWN_FOR_TEST";

/// `cerulion-netd --help` / `-h` usage.
const USAGE: &str = "\
cerulion-netd: the network gateway daemon, one per computer.

USAGE:
    cerulion-netd [--help] [--version]

netd owns this machine's single network session and shares remote topics between
every program that asks for them. When several consumers (the visualization
daemon, a graph's ingress, `cerulion topic echo`) demand the same remote topic,
its frames cross the network ONCE and every reader subscribes to the one local
mirror. It exits by itself when no consumer has demanded a topic for the idle
grace period. You normally never start it yourself: the first consumer starts it
in the background on demand.
Serving through LISTEN or local egress registration requires a prior `cerulion login`.
Expired saved logins permit offline serving; network-off operation is exempt.

OPTIONS:
    -h, --help       Print this help and exit.
    -V, --version    Print version information and exit.

ENVIRONMENT:
    CERULION_NETD_SOCK           Override the control-socket path.
    CERULION_NETD_IDLE_GRACE_MS  Idle grace before self-exit, in ms (default 30000).
    CERULION_NETD_HARD_EXIT_MS   Deadline for graceful shutdown before the daemon
                                 HARD-exits (nonzero) to avoid a zombie, in ms
                                 (default 5000).
    CERULION_NETD_CONNECT        Zenoh locators to CONNECT to for remote robots
                                 scouting can't reach (comma/space separated).
    CERULION_NETD_LISTEN         Zenoh locators to LISTEN on (comma/space separated).
                                 This ALSO decides whether this machine advertises
                                 itself as a robot on mDNS (`_cerulion._tcp`), the
                                 first place `cerulion topic list` looks: the first
                                 `tcp/` locator's port becomes the SRV record.
                                 UNSET = no advertisement at all, which is correct
                                 for a desk and is the reason a robot that should be
                                 discoverable must set it (for example
                                 tcp/0.0.0.0:7683).
                                 SET, it additionally makes this a STANDING daemon:
                                 the embedded egress gateway boots at daemon start
                                 (so topics registered at runtime, such as ROS 2
                                 publishers on rmw_cerulion, are served with no
                                 graph run) and idle self-exit is DISABLED (the
                                 daemon runs until SIGINT/SIGTERM). Requires a
                                 prior `cerulion login`; an expired saved login
                                 still permits offline LAN serving.
                                 CERULION_NETD_NETWORK=off wins.
    CERULION_NETD_NETWORK=off    Force a strictly LOCAL-ONLY daemon (no network ever).
";

/// The iroh WAN-plane env docs — appended to [`USAGE`] ONLY in a `--features wan`
/// build (empty otherwise, so the lean daemon never documents inert vars).
#[cfg(feature = "wan")]
const WAN_USAGE: &str = "    CERULION_NETD_WAN_ROBOTS     iroh WAN robots: a ';'-separated list of
                                 name=eid[@ip:port,ip:port...] entries. A listed
                                 robot is reached over iroh; others over zenoh.
    CERULION_NETD_DESK_KEY       Desk device key file (32 raw bytes) for WAN dials
                                 (unset = ephemeral, which an unpaired robot refuses).
    CERULION_NETD_DEVICE_CERT    Path to the cached device cert FILE (not the cert
                                 bytes themselves): resolves the logged-in
                                 account the WAN dial presents. Unset = the device.cert
                                 next to CERULION_NETD_DESK_KEY (~/.cerulion/device.cert).
    CERULION_EPOCH_DIR           Directory of cached revocation epochs:
                                 <robot>.epoch artifacts the desk PUSHES to each robot
                                 it dials, so an owner's revocation reaches the robot.
                                 DESK-WIDE (NOT netd-scoped): `cerulion connect` and the
                                 account-page cache writer honor the SAME var, so a
                                 relocated cache moves for all of them together.
                                 Unset = the epochs/ dir next to CERULION_NETD_DESK_KEY
                                 (~/.cerulion/epochs). Absent = no push; dials proceed.
    CERULION_NETD_RELAY_DISABLED Set (non-empty) to disable iroh relays (LAN-only).

A LISTEN-configured serving machine owns its WAN endpoint through remoted.
Consuming other robots over WAN from a serving machine is not supported in
this version; use the LAN plane. WAN demands are refused without a fallback.
Automatic robot state defaults to <CERULION_HOME or ~/.cerulion>/robot-state;
CERULION_STATE_ROOT preserves an explicit deployment root.
";

/// No iroh WAN plane compiled in — no extra env docs.
#[cfg(not(feature = "wan"))]
const WAN_USAGE: &str = "";

fn main() -> ExitCode {
    // Arg handling BEFORE tracing/transport: `--help`/`-h` prints usage and
    // `--version`/`-V` prints the version, both exiting 0; an unknown flag is a
    // loud usage error (exit 2).
    match parse_args(std::env::args().skip(1)) {
        ArgAction::Run => {}
        ArgAction::Help => {
            print!("{USAGE}{WAN_USAGE}");
            return ExitCode::SUCCESS;
        }
        ArgAction::Version => {
            println!("cerulion-netd {}", env!("CARGO_PKG_VERSION"));
            return ExitCode::SUCCESS;
        }
        ArgAction::Unknown(arg) => {
            eprintln!("cerulion-netd: unrecognized argument `{arg}`\n");
            eprint!("{USAGE}{WAN_USAGE}");
            return ExitCode::from(2);
        }
    }
    init_tracing();
    // Apply `IOX2_LOG_LEVEL` to THIS binary's `iceoryx2-log` static
    // before any iceoryx2 call can emit. `TransportManager::init` also does it,
    // but only once it runs — and netd's transport is LAZY, so an early
    // iceoryx2 diagnostic would otherwise print at the crate default (`Info`).
    cerulion_core::iceoryx_logger::init_iceoryx_log_level_from_env();
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            tracing::error!(error = %e, "cerulion-netd exiting with an error");
            ExitCode::FAILURE
        }
    }
}

/// What the parsed CLI args ask the daemon to do.
#[derive(Debug, PartialEq, Eq)]
enum ArgAction {
    /// Start the daemon (no args).
    Run,
    /// Print usage and exit 0 (`--help` / `-h`).
    Help,
    /// Print the version and exit 0 (`--version` / `-V`).
    Version,
    /// An unrecognized argument — print usage to stderr and exit nonzero.
    Unknown(String),
}

/// Parse `cerulion-netd`'s argv (already past argv[0]). `--help`/`-h` and
/// `--version`/`-V` each win as soon as they are seen — same precedence, so
/// either short-circuits a preceding unknown argument exactly as `--help`
/// already does; any other argument is [`ArgAction::Unknown`]. Pure — oracle-tested.
fn parse_args(args: impl IntoIterator<Item = String>) -> ArgAction {
    let mut unknown: Option<String> = None;
    for arg in args {
        match arg.as_str() {
            "--help" | "-h" => return ArgAction::Help,
            "--version" | "-V" => return ArgAction::Version,
            other if unknown.is_none() => unknown = Some(other.to_string()),
            _ => {}
        }
    }
    match unknown {
        Some(arg) => ArgAction::Unknown(arg),
        None => ArgAction::Run,
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    // ONE iceoryx2 node + ONE zenoh session for the whole machine (Principle #8).
    // Network-CONFIGURED by default (scouting ON) so demand-driven ingress works,
    // but the session stays LAZY — a netd spawned but never demanded opens no zenoh
    // session and then self-exits on idle. `init` (not `get_or_init`) so the
    // network config is honored: netd is the sole initializer of its transport.
    //
    // FIRST-CONTACT AUTOMAGIC: before init (the config is immutable
    // afterwards), fold the verified `~/.cerulion/peers.json` cache into the
    // connect endpoints, so a desk that has seen this robot before dials it with
    // NO `CERULION_NETD_CONNECT`. Explicit env locators stay first and verbatim;
    // cached rows are TCP-probed so a stale one cannot buy a background retry
    // connector for the life of the daemon. An empty cache costs nothing and
    // leaves the config byte-identical. See `discovery_fold` for the once-at-boot
    // cadence and its residual.
    let configured_network = net::network_config_from_env();
    if net::standing_gateway_requested(configured_network.as_ref()) {
        cerulion_netd::serving_login::require_prior_login()?;
    }
    let (network, peer_fold) = match configured_network {
        Some(cfg) => {
            let (cfg, fold) = discovery_fold::fold_cached_peers(cfg);
            (Some(cfg), fold)
        }
        // LOCAL-ONLY (`CERULION_NETD_NETWORK=off`): no session is ever opened, so
        // there is nothing to dial — do not read the cache at all.
        None => (None, discovery_fold::PeerFold::default()),
    };
    // Decide the STANDING-gateway posture BEFORE `network` moves
    // into the transport config. LISTEN-configured (a robot serving the network)
    // ⇒ boot the embedded egress gateway at daemon start (below, after the
    // singleton flock is held) AND disable idle self-exit — the gateway + beacon
    // are the machine's network presence and no refcounted demand holds them
    // open. LOCAL-ONLY (`network == None`) wins: nothing to boot.
    let standing_gateway = net::standing_gateway_requested(network.as_ref());
    let egress_login =
        cerulion_netd::serving_login::EgressLoginPolicy::for_network(network.is_some());
    let network_enabled = network.is_some();
    let manager = TransportManager::init(TransportConfig {
        network,
        ..TransportConfig::default()
    })?;
    // THE demand-authorization seam — install ONE authorizer on BOTH
    // demand planes. The zenoh LAN plane consults it via the shared manager's
    // NetworkManager (the egress plane's embedded gateway reads it); the iroh WAN plane
    // gets it via `build_mirror_plane` below. On the desk this is the deny-nothing
    // AllowAllAuthorizer (see `build_demand_authorizer`: the account grant is
    // enforced at the ROBOT); ONE body decides the gate and BOTH planes
    // consult it.
    let demand_authorizer = build_demand_authorizer();
    if let Some(net) = manager.network() {
        net.set_demand_authorizer(Arc::clone(&demand_authorizer));
    }
    // The mirror plane (ingress), the egress plane, AND the query
    // plane (catalog/schema) all share the SAME manager — ONE zenoh session for the
    // whole machine (Principle #8). The egress plane's embedded gateway boots LAZILY
    // on the first `register_egress` (a pure-ingress desk runs no egress gateway); the
    // query plane's GETs reuse the SAME lazy session (a query-only netd opens ONE
    // session and reuses it across every consumer's catalog/schema query, instead of
    // an N-transient-sessions-per-desk fan-out).
    let mirror_plane = build_mirror_plane(
        Arc::clone(&manager),
        Arc::clone(&demand_authorizer),
        standing_gateway,
        network_enabled,
    )?;
    // Keep the CONCRETE plane too: the standing-gateway boot below
    // is a `GatewayEgressPlane` method, not part of the `EgressPlane` seam (the
    // spy planes the daemon tests inject have no gateway to boot).
    let gateway_egress_plane = Arc::new(GatewayEgressPlane::new(Arc::clone(&manager)));
    let egress_plane: Arc<dyn EgressPlane> = Arc::clone(&gateway_egress_plane) as _;
    let query_plane: Arc<dyn QueryPlane> = Arc::new(GatewayQueryPlane::new(Arc::clone(&manager)));
    // The announce-watch plane — the SAME manager again, so the catalog-change
    // subscription rides the machine's ONE zenoh session like everything else. It is
    // declared LAZILY (only when a consumer sends `subscribe_catalog`), so a netd
    // nobody subscribes to opens no extra subscription.
    let announce_plane: Arc<dyn AnnounceWatchPlane> =
        Arc::new(GatewayAnnounceWatchPlane::new(manager));

    let idle_grace =
        daemon::resolve_idle_grace(std::env::var(daemon::IDLE_GRACE_ENV).ok().as_deref());
    let socket = default_socket_path();
    let mut netd = daemon::start_with_planes_and_events(
        socket,
        mirror_plane,
        egress_plane,
        query_plane,
        announce_plane,
        NetdConfig {
            idle_grace,
            // Principle #3: the connect set this daemon is CONFIGURED
            // TO DIAL, so `status` can answer "why is my desk talking to that
            // address?" without the operator inferring it from logs. NOT proof of
            // a live link — the session is lazy and a fiat-trusted dead locator
            // reads the same as a connected one (see `discovery_fold`).
            connect_endpoints: peer_fold.connect,
            // A LISTEN-configured daemon is a STANDING one: the
            // start-booted gateway (below) holds it open; it exits on
            // SIGINT/SIGTERM only. See `NetdConfig::idle_self_exit`.
            idle_self_exit: !standing_gateway,
            egress_login,
            account_identity_watch: cfg!(feature = "wan"),
            ..NetdConfig::default()
        },
    )?;
    // Boot the STANDING gateway AFTER `daemon::start`: only the
    // flock-holding singleton opens the session + raises the beacon (a losing
    // racer never advertises). Never fatal: the daemon keeps serving mirrors and
    // queries on a boot failure, warns loudly, and any later `register_egress`
    // retries the boot through the same `ensure_gateway_booted`.
    if standing_gateway {
        // A start-booted gateway gets NO CLI-built serving, and a
        // gateway names a runtime-registered topic (an rmw publisher, a raw
        // route) ONLY through the hash→name map in its serving — so hand it the
        // built-in corpus's bindings, or every `std_msgs/String` rmw topic on a
        // pure-ROS robot catalogues `schema_name: None` and no desk consumes it.
        // Custom types arrive two ways: a later `register_egress` MERGES its
        // serving into the running gateway (the backfill seam in
        // `egress.rs`), and an rmw process's OWN custom types need it to publish
        // `(hash, name)` + a served doc (the sibling lane's business).
        let serving = SchemaServing {
            schema_hashes: native_ros2_messages::builtin_hash_bindings(),
            ..SchemaServing::default()
        };
        if let Err(e) = gateway_egress_plane.boot_standing_gateway(&serving) {
            tracing::warn!(
                error = %e,
                "cerulion-netd: the standing gateway could NOT boot — this machine's topics are \
                 not reachable from the network until a graph registers egress (which retries the \
                 boot). Is the {LISTEN_ENV} locator bindable (port free, address owned by this \
                 machine)?"
            );
        }
    }
    // The lifecycle line says what is TRUE for the resolved mode: a standing
    // daemon cannot stop by idle self-exit (no idle-watch runs), so telling the
    // operator it can would be a misleading status. The mode-to-wording map is
    // the ONE pure `lifecycle_status_line`, pinned by wording in the tests below.
    tracing::info!(
        socket = %netd.socket_path().display(),
        lifecycle = lifecycle_status_line(standing_gateway),
        "cerulion-netd running"
    );

    // Block until a signal flips `running` OR the idle-watch requests self-exit.
    let running = Arc::new(AtomicBool::new(true));
    {
        let running = Arc::clone(&running);
        ctrlc::set_handler(move || running.store(false, Ordering::SeqCst))?;
    }
    let mut robot_supervisor = if cfg!(feature = "wan") && standing_gateway {
        match cerulion_netd::robot_supervisor::RobotSupervisor::start(
            Arc::clone(&running),
            Arc::clone(&gateway_egress_plane),
        ) {
            Ok(supervisor) => Some(supervisor),
            Err(error) => {
                tracing::error!(error = %error, "robot supervisor could not start; LAN remains available");
                None
            }
        }
    } else {
        None
    };
    while running.load(Ordering::SeqCst) && !netd.self_exit_requested() {
        std::thread::sleep(Duration::from_millis(200));
    }

    if netd.self_exit_requested() {
        tracing::info!("cerulion-netd: idle self-exit — dropping the network session");
    } else {
        tracing::info!("cerulion-netd shutting down");
    }

    // Arm the bounded hard-exit watchdog BEFORE teardown. `netd.shutdown()`
    // unlinks the control socket FIRST (so a client connecting during teardown
    // respawns instead of binding to us), then joins the daemon threads. If that
    // graceful teardown WEDGES — a blocked zenoh session close, a stuck
    // network-thread join, a mirror teardown that never returns — the watchdog
    // hard-exits within the deadline so the daemon can NEVER linger as a zombie
    // holding the socket (the production wedge). On the healthy path
    // shutdown() returns well within the deadline, `main` returns, and the process
    // exits normally before the watchdog fires.
    spawn_hard_exit_watchdog(resolve_hard_exit_timeout(
        std::env::var(HARD_EXIT_ENV).ok().as_deref(),
    ));
    if let Some(supervisor) = robot_supervisor.as_mut() {
        supervisor.shutdown();
    }
    netd.shutdown();

    // TEST-ONLY (never set in normal operation): simulate a teardown that WEDGES
    // AFTER the socket is unlinked, so a subprocess test can prove the hard-exit
    // watchdog terminates a hung shutdown. Placed after `shutdown()` (which already
    // unlinked the socket) so the test can also assert the socket is gone.
    if std::env::var_os(STALL_SHUTDOWN_FOR_TEST_ENV).is_some() {
        tracing::warn!(
            "cerulion-netd: {STALL_SHUTDOWN_FOR_TEST_ENV} set — stalling teardown to exercise the \
             hard-exit watchdog (TEST ONLY)"
        );
        loop {
            std::thread::sleep(Duration::from_secs(3600));
        }
    }
    Ok(())
}

/// The lifecycle wording of a STANDING daemon (listen-configured, the gateway
/// booted at start): there is no idle-watch, so idle self-exit is not a way it
/// can stop.
const LIFECYCLE_STANDING: &str =
    "standing daemon (listen-configured): stops on SIGINT/SIGTERM only; idle self-exit disabled";

/// The lifecycle wording of a DESK daemon — byte-identical to the pre-standing
/// status line, so a desk operator reads exactly what they always read.
const LIFECYCLE_DESK: &str = "SIGINT/SIGTERM or idle self-exit to stop";

/// How this daemon can stop, for the resolved mode — the ONE map from the
/// standing-gateway posture to the operator-facing lifecycle wording, so the
/// running-status line can never claim an exit path the mode has disabled.
/// Pure — pinned by wording on both arms.
fn lifecycle_status_line(standing_gateway: bool) -> &'static str {
    if standing_gateway {
        LIFECYCLE_STANDING
    } else {
        LIFECYCLE_DESK
    }
}

/// Resolve the hard-exit deadline from a [`HARD_EXIT_ENV`] value (already
/// read from the env): a positive integer number of milliseconds, else
/// [`DEFAULT_HARD_EXIT_TIMEOUT`]. A non-empty non-parseable / zero value warns
/// LOUDLY (never silently uses a surprising deadline — mirrors
/// [`daemon::resolve_idle_grace`]). Pure — oracle-tested.
fn resolve_hard_exit_timeout(env_value: Option<&str>) -> Duration {
    match env_value {
        None => DEFAULT_HARD_EXIT_TIMEOUT,
        Some(v) if v.trim().is_empty() => DEFAULT_HARD_EXIT_TIMEOUT,
        Some(v) => match v.trim().parse::<u64>() {
            Ok(ms) if ms > 0 => Duration::from_millis(ms),
            _ => {
                tracing::warn!(
                    value = v,
                    "cerulion-netd: {HARD_EXIT_ENV} is set but not a positive integer (ms) — \
                     using the default {}s hard-exit deadline",
                    DEFAULT_HARD_EXIT_TIMEOUT.as_secs()
                );
                DEFAULT_HARD_EXIT_TIMEOUT
            }
        },
    }
}

/// Spawn the hard-exit watchdog: a detached thread that force-exits the
/// process if graceful shutdown has not completed within `timeout`. Best-effort —
/// a spawn failure just means no watchdog (graceful shutdown still runs). On the
/// healthy path `main` returns first and this sleeping thread is torn down before
/// it fires.
fn spawn_hard_exit_watchdog(timeout: Duration) {
    let _ = std::thread::Builder::new()
        .name("netd-exit-watchdog".to_string())
        .spawn(move || {
            hard_exit_watchdog(timeout, || {
                tracing::error!(
                    timeout_ms = timeout.as_millis() as u64,
                    "cerulion-netd: graceful shutdown exceeded the deadline — HARD-exiting with a \
                     NONZERO status (abnormal) to avoid a zombie holding the control socket"
                );
                // `libc::_exit(1)`, NOT `std::process::exit`: the wedge class we
                // defend against includes a teardown / global-destructor / atexit
                // hang on a lock a live network thread holds — and
                // `std::process::exit` runs that SAME C `exit()` cleanup, so it could
                // deadlock exactly like the wedge it is meant to escape. `_exit(1)` is
                // an immediate syscall that runs no atexit handlers and no
                // destructors, so it CANNOT hang. The socket was already unlinked
                // (idle-watch at commit, or `netd.shutdown()`), and the loud `error!`
                // above already flushed its write, so skipping cleanup is safe here.
                // Status 1 (nonzero) so a supervisor that ever `wait()`s on netd reads
                // a wedge-escape as the ABNORMAL exit it is — a clean idle self-exit
                // returns 0 through the normal `main` path.
                // SAFETY: `_exit` is async-signal-safe and simply terminates the
                // process with the given status; there is nothing to make unsound.
                unsafe { libc::_exit(1) };
            });
        });
}

/// The watchdog body — separated from [`spawn_hard_exit_watchdog`] so the
/// timeout→expire behavior is unit-testable WITHOUT `std::process::exit`. Sleeps
/// `timeout`, then runs `on_expire`.
fn hard_exit_watchdog(timeout: Duration, on_expire: impl FnOnce()) {
    std::thread::sleep(timeout);
    on_expire();
}

/// EVERY env var the `wan` build's [`WanRegistry::from_env`] consumes — mirror the
/// `wan` module's consts (`WAN_ROBOTS_ENV` / `DESK_KEY_ENV` / `DEVICE_CERT_ENV` /
/// `EPOCH_DIR_ENV` / `RELAY_DISABLED_ENV` / `RELAY_URL_ENV`), kept in lockstep by
/// [`tests::wan_env_var_names_match_the_wan_module`] (which runs under `--features wan`
/// where both the literals and the consts are visible). The non-`wan` build has no
/// access to the `wan` module's consts, so it names the FULL set here to warn on
/// misuse — omitting any one would leave a silent-inertness gap.
#[cfg(not(feature = "wan"))]
const WAN_ENV_VARS: [&str; 6] = [
    "CERULION_NETD_WAN_ROBOTS",
    "CERULION_NETD_DESK_KEY",
    "CERULION_NETD_DEVICE_CERT",
    // DESK-wide, not netd-scoped: `cerulion connect`
    // and the cache writer honor the same var.
    "CERULION_EPOCH_DIR",
    "CERULION_NETD_RELAY_DISABLED",
    "CERULION_RELAY_URL",
];

/// The subset of [`WAN_ENV_VARS`] that is DESK-WIDE rather than netd's own: honored by
/// desk-side binaries no matter how netd was built (the newest entry is
/// `CERULION_RELAY_URL`). Each entry pairs the var with the desk-side consumers that DO
/// honor it, which the lean build's misuse warn names — "netd's WAN dials cannot honor
/// it, but X and Y still do", never the flat "it is IGNORED", which would be a half-truth
/// that sends a user chasing a netd rebuild for a var already working on the path they
/// are using.
///
/// - `CERULION_EPOCH_DIR` — the shared revocation-cache directory (`cerulion_wireclient::epoch`).
/// - `CERULION_RELAY_URL` — read by `cerulion-connectd`'s `connect` AND `pair` subcommands
///   via clap's `env` attribute, so a relay override reaches every desk dial and every
///   pairing ceremony regardless of how netd was built.
///
/// Every entry must also appear in [`WAN_ENV_VARS`] (pinned by
/// [`tests::desk_wide_env_vars_are_a_subset_of_the_wan_set`]).
#[cfg(not(feature = "wan"))]
const DESK_WIDE_ENV_VARS: [(&str, &str); 2] = [
    (
        "CERULION_EPOCH_DIR",
        "`cerulion connect` and the account-page revocation cache",
    ),
    (
        "CERULION_RELAY_URL",
        "`cerulion connect` and `cerulion pair` (cerulion-connectd reads it on both)",
    ),
];

/// The desk-side consumers of `var`, or `None` when the var is netd's OWN (in which
/// case this lean build really does ignore it entirely).
#[cfg(not(feature = "wan"))]
fn desk_wide_consumers(var: &str) -> Option<&'static str> {
    DESK_WIDE_ENV_VARS
        .iter()
        .find(|(name, _)| *name == var)
        .map(|(_, consumers)| *consumers)
}

/// The exact misuse-warn text the lean build emits for `var`. Pure (no logging, no env
/// reads) so the two message CLASSES can be pinned by hand-written oracles rather than
/// by reading a log capture — see [`tests::lean_build_warn_scopes_desk_wide_vars`].
#[cfg(not(feature = "wan"))]
fn lean_build_env_warn_message(var: &str) -> String {
    match desk_wide_consumers(var) {
        // The DESK-WIDE vars get a SCOPED message: they are not netd's to own, so
        // "it is IGNORED — rebuild with --features wan" would be a half-truth. Telling a
        // user their epoch dir or relay URL "is IGNORED" would send them chasing a netd
        // rebuild for a var that is working fine on the path they are actually using.
        Some(consumers) => format!(
            "cerulion-netd was built WITHOUT the `wan` feature, so its WAN dials cannot honor \
             {var} (this daemon serves the zenoh LAN plane only). {var} is DESK-WIDE and is \
             still honored by {consumers} — nothing else is affected. Rebuild with `cargo build \
             -p cerulion_netd --features wan` if you want netd's WAN dials to honor it too."
        ),
        None => format!(
            "cerulion-netd was built WITHOUT the `wan` feature — {var} is set but the iroh WAN \
             plane is NOT compiled in, so it is IGNORED (this daemon serves the zenoh LAN plane \
             only). Rebuild with `cargo build -p cerulion_netd --features wan` for WAN mirroring."
        ),
    }
}

/// Build netd's desk-side demand-authorization gate BOTH of netd's demand
/// planes consult. It STAYS the deny-nothing [`AllowAllAuthorizer`] and always will:
/// netd is the DESK (the demander) and holds no robot access list, so it cannot resolve
/// `is_allowed` — the ROBOT is the authoritative per-demand enforcement point. The
/// pairing crate ships the `is_allowed`-backed `PairingAuthorizer`, and the robot daemon INSTALLS it on the
/// ROBOT serving plane (`cerulion_remoted::wire`), keyed by the demanding desk's
/// authenticated key + a mid-session revocation sweep — NOT here. This gate is a
/// pre-dial hook that denies nothing; the account grant is enforced at
/// the robot, over BOTH planes.
fn build_demand_authorizer() -> Arc<dyn DemandAuthorizer> {
    Arc::new(AllowAllAuthorizer)
}

/// Build the daemon's mirror plane. Without the `wan` feature: the lean zenoh-only
/// [`GatewayMirrorPlane`]. If a user set the WAN env
/// vars expecting iroh mirroring, warn LOUDLY that this build cannot honor them — a
/// silent ignore is a foreseeable "why won't my remote robot connect" trap.
#[cfg(not(feature = "wan"))]
fn build_mirror_plane(
    manager: Arc<TransportManager>,
    // The WAN plane does not exist in this build; the LAN plane's authorizer is
    // already installed on the shared manager by the caller.
    _demand_authorizer: Arc<dyn DemandAuthorizer>,
    _serving_gateway: bool,
    _network_enabled: bool,
) -> Result<Arc<dyn MirrorPlane>, Box<dyn std::error::Error>> {
    // Which of the two message CLASSES each var gets — the flat "IGNORED" one for
    // netd's OWN vars, the scoped "netd's WAN dials cannot honor it, but X still does"
    // one for the DESK-WIDE vars — is decided by [`desk_wide_consumers`], NOT by this
    // loop. The loop is only the env probe; the wording lives in the pure
    // [`lean_build_env_warn_message`] so it can be pinned directly.
    for var in WAN_ENV_VARS {
        if std::env::var_os(var).is_some_and(|v| !v.is_empty()) {
            tracing::warn!(var, "{}", lean_build_env_warn_message(var));
        }
    }
    Ok(Arc::new(GatewayMirrorPlane::new(manager)))
}

/// Build one controller for manual and account WAN routes, including a daemon
/// started before login. Construction opens no outgoing endpoint; network-off
/// and serving posture are checked before every WAN network operation.
#[cfg(feature = "wan")]
fn build_mirror_plane(
    manager: Arc<TransportManager>,
    demand_authorizer: Arc<dyn DemandAuthorizer>,
    serving_gateway: bool,
    network_enabled: bool,
) -> Result<Arc<dyn MirrorPlane>, Box<dyn std::error::Error>> {
    use cerulion_netd::account_controller::{AccountControllerConfig, AccountWanController};
    use cerulion_netd::WanRegistry;

    let registry = WanRegistry::from_env().map_err(Box::<dyn std::error::Error>::from)?;
    let config_home = cerulion_discovery::robot_state::config_dir();
    let key_file = config_home.as_ref().map(|home| home.join("desk.key"));
    let epoch_dir = cerulion_wireclient::epoch::resolve_epoch_dir(
        std::env::var(cerulion_netd::wan::EPOCH_DIR_ENV)
            .ok()
            .as_deref(),
        key_file.as_deref(),
    );
    let controller = AccountWanController::new(
        manager,
        registry,
        demand_authorizer,
        AccountControllerConfig {
            config_home,
            network_enabled,
            serving_gateway,
            epoch_dir,
            trusted_direct: std::collections::HashMap::new(),
        },
    )
    .map_err(Box::<dyn std::error::Error>::from)?;
    Ok(Arc::new(controller))
}

/// The daemon's logging default when `RUST_LOG` is unset. An explicit
/// `RUST_LOG` governs the filter entirely, so a user who asks for
/// `RUST_LOG=warn` (or `debug`) sees the zenoh line again.
///
/// `zenoh::net::runtime::orchestrator=error`: that target logs
/// `Starting with no listener endpoints!` at WARN whenever a peer session opens
/// with no listen endpoint, which is exactly a desk's session (netd connects
/// out and listens only when `CERULION_NETD_LISTEN` is set). On a desk it is
/// the expected shape, not a warning, and the daemon inherits the spawning
/// command's stderr on first use, so unfiltered it lands in every first user's terminal.
/// A filter directive, never a change to the zenoh session config.
const DEFAULT_FILTER: &str = "info,zenoh::net::runtime::orchestrator=error";

fn init_tracing() {
    use tracing_subscriber::{fmt, EnvFilter};
    let filter =
        EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new(DEFAULT_FILTER));
    let _ = fmt()
        .with_env_filter(filter)
        .with_writer(std::io::stderr)
        .try_init();
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(args: &[&str]) -> ArgAction {
        parse_args(args.iter().map(|s| s.to_string()))
    }

    #[test]
    fn no_args_runs_the_daemon() {
        assert_eq!(parse(&[]), ArgAction::Run);
    }

    #[test]
    fn help_flags_ask_for_usage() {
        assert_eq!(parse(&["--help"]), ArgAction::Help);
        assert_eq!(parse(&["-h"]), ArgAction::Help);
        // --help wins over an otherwise-unknown arg on EITHER side of it.
        assert_eq!(parse(&["--bogus", "--help"]), ArgAction::Help);
        assert_eq!(parse(&["--help", "--bogus"]), ArgAction::Help);
    }

    #[test]
    fn version_flags_ask_for_the_version() {
        assert_eq!(parse(&["--version"]), ArgAction::Version);
        assert_eq!(parse(&["-V"]), ArgAction::Version);
        // --version wins over an otherwise-unknown arg on EITHER side of it —
        // the same precedence as --help above.
        assert_eq!(parse(&["--bogus", "--version"]), ArgAction::Version);
        assert_eq!(parse(&["--version", "--bogus"]), ArgAction::Version);
        // Lowercase -v is NOT a version alias: it is reported as unknown.
        assert_eq!(parse(&["-v"]), ArgAction::Unknown("-v".to_string()));
    }

    /// `--help` and `--version` share ONE precedence tier: whichever is seen
    /// first wins, exactly as the parser doc states.
    #[test]
    fn help_and_version_resolve_first_seen_wins() {
        assert_eq!(parse(&["--help", "--version"]), ArgAction::Help);
        assert_eq!(parse(&["--version", "--help"]), ArgAction::Version);
    }

    /// `--help` must advertise BOTH exit flags — the synopsis and the OPTIONS
    /// rows — so neither can vanish from the help text while it keeps working.
    #[test]
    fn usage_lists_the_help_and_version_flags() {
        assert!(USAGE.contains("cerulion-netd [--help] [--version]"));
        assert!(USAGE.contains("-h, --help"));
        assert!(USAGE.contains("-V, --version"));
    }

    #[test]
    fn an_unknown_arg_is_reported_not_silently_ignored() {
        assert_eq!(
            parse(&["--bogus"]),
            ArgAction::Unknown("--bogus".to_string())
        );
        // The FIRST unknown arg is named.
        assert_eq!(
            parse(&["--one", "--two"]),
            ArgAction::Unknown("--one".to_string())
        );
    }

    #[test]
    fn usage_names_the_socket_and_network_env_vars() {
        assert!(USAGE.contains(cerulion_netd::SOCKET_ENV));
        assert!(USAGE.contains(net::CONNECT_ENV));
        assert!(USAGE.contains(net::LISTEN_ENV));
        assert!(USAGE.contains(net::NETWORK_ENV));
        assert!(USAGE.contains(daemon::IDLE_GRACE_ENV));
        // The hard-exit deadline knob is discoverable from --help.
        assert!(USAGE.contains(HARD_EXIT_ENV));
    }

    /// `--help` states the SECOND meaning of `CERULION_NETD_LISTEN`.
    ///
    /// Since the beacon moved onto the netd-hosted plane, that variable is the
    /// SOLE gate on whether this machine advertises itself as a robot over mDNS
    /// — the PRIMARY discovery rung. Documenting it only as "Zenoh locators to
    /// LISTEN on" leaves an operator with no way to connect "my robot is
    /// invisible to `cerulion topic list`" to the knob that fixes it, and
    /// user-facing surface is the contract.
    #[test]
    fn usage_says_the_listen_var_also_gates_mdns_discoverability() {
        let listen_block = USAGE
            .split(net::LISTEN_ENV)
            .nth(1)
            .expect("USAGE must document CERULION_NETD_LISTEN");
        // Scoped to the block that FOLLOWS the var, so this cannot be satisfied
        // by an unrelated mention elsewhere in the help text.
        let block = listen_block
            .split(net::NETWORK_ENV)
            .next()
            .unwrap_or(listen_block);
        for needle in ["mDNS", "_cerulion._tcp", "discoverable"] {
            assert!(
                block.contains(needle),
                "the CERULION_NETD_LISTEN help must name its mDNS meaning ({needle:?} \
                 missing) — it is the only gate on the PRIMARY discovery rung:\n{block}"
            );
        }
    }

    /// `--help` states the THIRD meaning of `CERULION_NETD_LISTEN`
    /// — a LISTEN-configured daemon boots the standing gateway at start and never
    /// idle-self-exits. An operator who set LISTEN and finds netd still running
    /// after every consumer left must be able to read that this is the contract,
    /// not a leak; and one whose robot serves nothing must find the knob.
    #[test]
    fn usage_says_the_listen_var_also_makes_a_standing_daemon() {
        let listen_block = USAGE
            .split(net::LISTEN_ENV)
            .nth(1)
            .expect("USAGE must document CERULION_NETD_LISTEN");
        for needle in [
            "STANDING",
            "boots at daemon start",
            "idle",
            "SIGINT/SIGTERM",
        ] {
            assert!(
                listen_block.contains(needle),
                "the CERULION_NETD_LISTEN help must name its standing-daemon meaning \
                 ({needle:?} missing):\n{listen_block}"
            );
        }
    }

    /// The running-status line says what is TRUE for the resolved mode. The
    /// standing arm must name the only real exit path AND that idle self-exit is
    /// disabled; the desk arm must be the plain desk wording verbatim. Swapping
    /// the two strings fails BOTH arms (the standing one lacks "disabled", the
    /// desk one is not the desk sentence) — a status that told a standing
    /// daemon's operator it "can stop by idle self-exit" is the misleading line
    /// this pins out.
    #[test]
    fn lifecycle_status_line_states_the_truth_for_each_mode() {
        let standing = lifecycle_status_line(true);
        for needle in ["SIGINT/SIGTERM only", "idle self-exit disabled"] {
            assert!(
                standing.contains(needle),
                "the standing lifecycle line must say {needle:?}: {standing:?}"
            );
        }
        assert!(
            !standing.contains("idle self-exit to stop"),
            "a standing daemon must never be told it can stop by idle self-exit: {standing:?}"
        );
        assert_eq!(
            lifecycle_status_line(false),
            "SIGINT/SIGTERM or idle self-exit to stop",
            "the desk wording is the plain desk status line, byte for byte"
        );
    }

    /// The `--help` WAN addendum documents the WAN env vars (feature build only);
    /// the non-`wan` build appends nothing.
    #[cfg(feature = "wan")]
    #[test]
    fn wan_usage_documents_the_wan_env_vars() {
        assert!(WAN_USAGE.contains(cerulion_netd::wan::WAN_ROBOTS_ENV));
        assert!(WAN_USAGE.contains(cerulion_netd::wan::DESK_KEY_ENV));
        assert!(WAN_USAGE.contains(cerulion_netd::wan::DEVICE_CERT_ENV));
        // The epoch-cache dir is discoverable from --help (an operator must be
        // able to find where revocations are read from).
        assert!(WAN_USAGE.contains(cerulion_netd::wan::EPOCH_DIR_ENV));
        assert!(WAN_USAGE.contains(cerulion_netd::wan::RELAY_DISABLED_ENV));
    }

    #[cfg(not(feature = "wan"))]
    #[test]
    fn wan_usage_is_empty_without_the_feature() {
        assert!(WAN_USAGE.is_empty());
    }

    #[test]
    fn resolve_hard_exit_timeout_parses_ms_and_falls_back() {
        // Unset / empty / non-numeric / zero / negative → the default deadline.
        assert_eq!(resolve_hard_exit_timeout(None), DEFAULT_HARD_EXIT_TIMEOUT);
        assert_eq!(
            resolve_hard_exit_timeout(Some("")),
            DEFAULT_HARD_EXIT_TIMEOUT
        );
        assert_eq!(
            resolve_hard_exit_timeout(Some("   ")),
            DEFAULT_HARD_EXIT_TIMEOUT
        );
        assert_eq!(
            resolve_hard_exit_timeout(Some("nope")),
            DEFAULT_HARD_EXIT_TIMEOUT
        );
        assert_eq!(
            resolve_hard_exit_timeout(Some("0")),
            DEFAULT_HARD_EXIT_TIMEOUT
        );
        assert_eq!(
            resolve_hard_exit_timeout(Some("-5")),
            DEFAULT_HARD_EXIT_TIMEOUT
        );
        // A positive integer of ms is honored (trimmed).
        assert_eq!(
            resolve_hard_exit_timeout(Some("500")),
            Duration::from_millis(500)
        );
        assert_eq!(
            resolve_hard_exit_timeout(Some("  1200  ")),
            Duration::from_millis(1200)
        );
    }

    #[test]
    fn hard_exit_watchdog_runs_its_expire_fn_after_the_timeout() {
        // The watchdog waits the timeout, THEN runs on_expire (production's
        // on_expire is std::process::exit; here it flips a flag). This pins the
        // "fires after the deadline" contract — on the healthy path the process
        // exits first and this sleeping thread is torn down before it runs.
        use std::time::Instant;
        let fired = Arc::new(AtomicBool::new(false));
        let f = Arc::clone(&fired);
        let start = Instant::now();
        hard_exit_watchdog(Duration::from_millis(30), move || {
            f.store(true, Ordering::SeqCst)
        });
        assert!(
            fired.load(Ordering::SeqCst),
            "the watchdog ran its expire fn"
        );
        assert!(
            start.elapsed() >= Duration::from_millis(30),
            "it waited the full timeout first"
        );
    }

    /// Drift guard: the non-`wan` misuse-warn hardcodes the WAN env var NAMES (it has
    /// no access to the `wan` module's consts). This wan-build test is where BOTH are
    /// visible, so it pins the names against the source-of-truth consts.
    #[cfg(feature = "wan")]
    #[test]
    fn wan_env_var_names_match_the_wan_module() {
        assert_eq!(
            cerulion_netd::wan::WAN_ROBOTS_ENV,
            "CERULION_NETD_WAN_ROBOTS"
        );
        assert_eq!(cerulion_netd::wan::DESK_KEY_ENV, "CERULION_NETD_DESK_KEY");
        assert_eq!(
            cerulion_netd::wan::DEVICE_CERT_ENV,
            "CERULION_NETD_DEVICE_CERT"
        );
        // DESK-wide, deliberately NOT netd-scoped —
        // `cerulion connect` and the cache writer resolve the SAME var through the
        // SAME resolver, so a relocated cache can never be visible to only one path.
        assert_eq!(cerulion_netd::wan::EPOCH_DIR_ENV, "CERULION_EPOCH_DIR");
        assert_eq!(
            cerulion_netd::wan::EPOCH_DIR_ENV,
            cerulion_wireclient::epoch::EPOCH_DIR_ENV
        );
        assert_eq!(
            cerulion_netd::wan::RELAY_DISABLED_ENV,
            "CERULION_NETD_RELAY_DISABLED"
        );
        assert_eq!(cerulion_netd::wan::RELAY_URL_ENV, "CERULION_RELAY_URL");
    }

    /// The non-`wan` build names ALL SIX WAN env vars the wan build reads in its
    /// misuse warn (omitting any is a silent-inertness gap).
    #[cfg(not(feature = "wan"))]
    #[test]
    fn wan_env_vars_are_named_for_the_misuse_warn() {
        assert!(WAN_ENV_VARS.contains(&"CERULION_NETD_WAN_ROBOTS"));
        assert!(WAN_ENV_VARS.contains(&"CERULION_NETD_DESK_KEY"));
        assert!(WAN_ENV_VARS.contains(&"CERULION_NETD_DEVICE_CERT"));
        // A user who set the epoch dir expecting revocation delivery must be
        // told this lean build cannot honor it (silent ignore = revocations that never
        // land).
        assert!(WAN_ENV_VARS.contains(&"CERULION_EPOCH_DIR"));
        assert!(WAN_ENV_VARS.contains(&"CERULION_NETD_RELAY_DISABLED"));
        assert!(WAN_ENV_VARS.contains(&"CERULION_RELAY_URL"));
    }

    /// The DESK-WIDE subset is
    /// exactly the vars netd does not own, and every one of them is warned about (i.e. is
    /// in the WAN set) — so a desk-wide var can never be silently dropped from the misuse
    /// warn, nor get netd's flat "it is IGNORED" message, which would be a half-truth for
    /// a var the desk-side binaries honor regardless of netd's build.
    ///
    /// `CERULION_RELAY_URL` is desk-wide because `cerulion-connectd` declares it as the
    /// clap `env` fallback for `--relay-url` on BOTH its `connect` and `pair` subcommands
    /// — netd's build cannot make a desk dial stop reading it.
    #[cfg(not(feature = "wan"))]
    #[test]
    fn desk_wide_env_vars_are_a_subset_of_the_wan_set() {
        assert_eq!(
            DESK_WIDE_ENV_VARS.map(|(var, _)| var),
            ["CERULION_EPOCH_DIR", "CERULION_RELAY_URL"]
        );
        for (var, consumers) in DESK_WIDE_ENV_VARS {
            assert!(
                WAN_ENV_VARS.contains(&var),
                "{var} is desk-wide but is not warned about at all"
            );
            assert!(
                !consumers.is_empty(),
                "{var} must name the desk-side consumers that DO honor it"
            );
        }
        // The netd-OWNED vars are NOT desk-wide (they keep the flat "IGNORED" message).
        for var in [
            "CERULION_NETD_WAN_ROBOTS",
            "CERULION_NETD_DESK_KEY",
            "CERULION_NETD_DEVICE_CERT",
            "CERULION_NETD_RELAY_DISABLED",
        ] {
            assert!(desk_wide_consumers(var).is_none(), "{var} is netd's own");
        }
    }

    /// The two message CLASSES, pinned per var against hand-written
    /// oracles. A desk-wide var must NEVER be told it "is IGNORED" (the false-rebuild
    /// trap), and must name the consumers that still honor it; a netd-owned var must keep
    /// the flat IGNORED wording. Reclassifying either direction fails here.
    #[cfg(not(feature = "wan"))]
    #[test]
    fn lean_build_warn_scopes_desk_wide_vars() {
        for (var, consumers) in DESK_WIDE_ENV_VARS {
            let msg = lean_build_env_warn_message(var);
            assert!(
                !msg.contains("is IGNORED"),
                "{var} is desk-wide — the warn must not claim it is ignored: {msg}"
            );
            assert!(msg.contains("DESK-WIDE"), "{var}: {msg}");
            assert!(
                msg.contains(consumers),
                "{var}'s warn must name its desk-side consumers: {msg}"
            );
            assert!(
                msg.contains("WAN dials cannot honor"),
                "{var}: the warn must scope the loss to netd's WAN dials: {msg}"
            );
        }
        // `CERULION_RELAY_URL` specifically: this exact string is what the test pins.
        let relay = lean_build_env_warn_message("CERULION_RELAY_URL");
        assert!(relay.contains("`cerulion pair`"), "{relay}");

        for var in [
            "CERULION_NETD_WAN_ROBOTS",
            "CERULION_NETD_DESK_KEY",
            "CERULION_NETD_DEVICE_CERT",
            "CERULION_NETD_RELAY_DISABLED",
        ] {
            let msg = lean_build_env_warn_message(var);
            assert!(msg.contains("is IGNORED"), "{var} is netd's own: {msg}");
            assert!(!msg.contains("DESK-WIDE"), "{var}: {msg}");
        }
        // Every message names the rebuild remedy, both classes.
        for var in WAN_ENV_VARS {
            assert!(
                lean_build_env_warn_message(var).contains("--features wan"),
                "{var} must name the rebuild remedy"
            );
        }
    }

    /// A `tracing` layer recording `(target, level)` of every event that
    /// PASSES the filter it is attached to: the observable proof of what a
    /// built `EnvFilter` enables (the same probe `cerulion_core`'s
    /// `init_logging_tests` use).
    #[derive(Clone, Default)]
    struct Capture(std::sync::Arc<std::sync::Mutex<Vec<(String, tracing::Level)>>>);

    impl<S: tracing::Subscriber> tracing_subscriber::Layer<S> for Capture {
        fn on_event(
            &self,
            event: &tracing::Event<'_>,
            _ctx: tracing_subscriber::layer::Context<'_, S>,
        ) {
            let m = event.metadata();
            self.0
                .lock()
                .unwrap()
                .push((m.target().to_string(), *m.level()));
        }
    }

    /// Drive `emit` under a thread-scoped subscriber = the daemon's
    /// `DEFAULT_FILTER` + a `Capture` layer, and return what got through.
    fn events_passing_default_filter(emit: impl FnOnce()) -> Vec<(String, tracing::Level)> {
        use tracing_subscriber::layer::SubscriberExt;
        use tracing_subscriber::Layer;
        let cap = Capture::default();
        let subscriber = tracing_subscriber::registry().with(
            cap.clone()
                .with_filter(tracing_subscriber::EnvFilter::new(DEFAULT_FILTER)),
        );
        tracing::subscriber::with_default(subscriber, emit);
        let events = cap.0.lock().unwrap().clone();
        events
    }

    /// The daemon's DEFAULT filter (no `RUST_LOG`) drops the zenoh
    /// orchestrator's `Starting with no listener endpoints!` WARN, the expected
    /// shape of every desk-side session, while keeping that target's ERROR, a
    /// WARN from any OTHER zenoh target, and the daemon's own INFO. The global
    /// default here is `info`, so unlike the CLI (whose global `error` floor
    /// drops the line by itself) this filter has to name the target: delete the
    /// directive from `DEFAULT_FILTER` and the first assertion fails.
    #[test]
    fn the_default_filter_drops_the_orchestrator_no_listener_warn_and_nothing_else() {
        const ORCHESTRATOR: &str = "zenoh::net::runtime::orchestrator";
        const OTHER_ZENOH: &str = "zenoh::net::routing";
        let has = |events: &[(String, tracing::Level)], target: &str, level: tracing::Level| {
            events.iter().any(|(t, l)| t == target && *l == level)
        };
        let events = events_passing_default_filter(|| {
            tracing::warn!(target: ORCHESTRATOR, "Starting with no listener endpoints!");
            tracing::error!(target: ORCHESTRATOR, "orchestrator error");
            tracing::warn!(target: OTHER_ZENOH, "a real zenoh warning elsewhere");
            tracing::info!(target: "cerulion_netd", "daemon lifecycle line");
            tracing::info!(target: OTHER_ZENOH, "zenoh chatter");
        });
        assert!(
            !has(&events, ORCHESTRATOR, tracing::Level::WARN),
            "the orchestrator's no-listener WARN must be dropped by default: {events:?}"
        );
        assert!(
            has(&events, ORCHESTRATOR, tracing::Level::ERROR),
            "the orchestrator's ERROR must still pass: {events:?}"
        );
        assert!(
            has(&events, OTHER_ZENOH, tracing::Level::WARN),
            "the directive is scoped to ONE target; another zenoh WARN must pass: {events:?}"
        );
        assert!(
            has(&events, "cerulion_netd", tracing::Level::INFO),
            "the daemon's own INFO must pass (the global default is info): {events:?}"
        );
        assert!(
            has(&events, OTHER_ZENOH, tracing::Level::INFO),
            "the global default is `info` for every other target: {events:?}"
        );
    }
}
