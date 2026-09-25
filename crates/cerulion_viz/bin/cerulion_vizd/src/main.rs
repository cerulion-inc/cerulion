// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion-vizd` production entry.
//!
//! Thin wiring: init tracing → claim the control socket (the single-instance
//! election, won BEFORE the transport or any viz stream exists, so a losing
//! second daemon touches nothing shared) → `init` the singleton transport
//! (Principle #8: ONE iceoryx2 node) → resolve the Rerun endpoint
//! ([`host::resolve_stream`]: HOST a gRPC message proxy by default, else connect
//! as a CLIENT to `$CERULION_RERUN_URL`) → build the never-block viz worker over
//! that stream → build the schema-resolution walker → run the daemon (the UDS
//! NDJSON control server plus the tap poll thread) until SIGINT/SIGTERM, then
//! shut down cleanly (drop taps, close socket, remove pidfile).
//!
//! The daemon does NOT spawn the VIEWER — the `cerulion viz` verb / Studio own
//! the viewer. But the daemon DOES host the gRPC message PROXY by default: on
//! a bare machine nobody else hosts it, so the required
//! checkbox→scene bar needs the daemon to be the one endpoint every viewer
//! (native `rerun` app OR Studio's WASM viewer) connects to — advertised in the
//! Hello banner.

use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_viz::schema_registry::walker_from_bridge_config_env;
use cerulion_viz::sink::SinkState;
use cerulion_viz::worker::VizLogWorker;

use cerulion_vizd::{host, net, ControlSocket, NetdCatalogEventSource, NetdDemandPlane};

/// `cerulion-vizd --help` / `-h` usage. Kept terse — the daemon is normally
/// spawned by the `cerulion viz` verb, not launched by hand.
const USAGE: &str = "\
cerulion-vizd — the long-lived, schema-generic Cerulion viz daemon.

USAGE:
    cerulion-vizd [--help]

The daemon owns the runtime taps, the schema-generic decode/render, and a hosted
Rerun gRPC proxy; controllers (Cerulion Studio, the agent, and `cerulion viz`)
drive it over a Unix-domain-socket NDJSON control protocol. It runs until
SIGINT/SIGTERM. Normally you never launch it directly — `cerulion viz` starts it
detached on demand.

OPTIONS:
    -h, --help    Print this help and exit.

ENVIRONMENT:
    CERULION_VIZD_SOCK            Override the control-socket path.
    CERULION_RERUN_URL           Connect the daemon to an external Rerun endpoint
                                 as a CLIENT instead of hosting the proxy.
    CERULION_VIZD_CONNECT        Zenoh locators to CONNECT to for remote robots
                                 scouting can't reach (comma/space separated).
    CERULION_VIZD_LISTEN         Zenoh locators to LISTEN on (comma/space separated).
    CERULION_VIZD_NETWORK=off    Force a strictly LOCAL-ONLY daemon (no zenoh ever).
";

fn main() -> ExitCode {
    // Arg handling BEFORE tracing/transport: `--help`/`-h` prints usage and exits
    // 0 without starting the daemon (previously any arg was silently ignored and
    // the daemon started serving). An unknown flag is a loud usage error (exit 2).
    match parse_args(std::env::args().skip(1)) {
        ArgAction::Run => {}
        ArgAction::Help => {
            print!("{USAGE}");
            return ExitCode::SUCCESS;
        }
        ArgAction::Unknown(arg) => {
            eprintln!("cerulion-vizd: unrecognized argument `{arg}`\n");
            eprint!("{USAGE}");
            return ExitCode::from(2);
        }
    }
    init_tracing();
    // Apply `IOX2_LOG_LEVEL` to THIS binary's `iceoryx2-log` static
    // before any iceoryx2 call can emit (see `init_iceoryx_log_level`'s docs —
    // the level is a per-linked-copy static, so every entry point sets its own).
    cerulion_core::iceoryx_logger::init_iceoryx_log_level_from_env();
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            tracing::error!(error = %e, "cerulion-vizd exiting with an error");
            ExitCode::FAILURE
        }
    }
}

/// What the parsed CLI args ask the daemon to do.
#[derive(Debug, PartialEq, Eq)]
enum ArgAction {
    /// Start the daemon (no args, or only ignorable ones).
    Run,
    /// Print usage and exit 0 (`--help` / `-h`).
    Help,
    /// An unrecognized argument — print usage to stderr and exit nonzero.
    Unknown(String),
}

/// Parse `cerulion-vizd`'s argv (already past argv[0]). `--help`/`-h` wins over
/// everything; any other argument is [`ArgAction::Unknown`]. Pure — oracle-tested.
fn parse_args(args: impl IntoIterator<Item = String>) -> ArgAction {
    let mut unknown: Option<String> = None;
    for arg in args {
        match arg.as_str() {
            "--help" | "-h" => return ArgAction::Help,
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
    // Win the single-instance election BEFORE anything shared exists: the transport
    // init below creates an iceoryx2 node and runs the startup dead-node sweep, and
    // the viz worker sends the boot blueprint the moment it spawns — a loser that
    // had already connected to the live daemon's proxy (`$CERULION_RERUN_URL`)
    // clobbered that viewer's layout on its way out (see `ControlSocket`).
    let socket = ControlSocket::acquire(cerulion_vizd::default_socket_path())?;

    // ONE iceoryx2 node + ONE zenoh session for the whole process (Principle #8).
    // Network-CONFIGURED by default (scouting ON) so the remote arm's
    // `register_ingress_topic` works, but the session stays LAZY — a purely-local
    // daemon (never a `--robot` attach) opens no zenoh session. `init` (not
    // `get_or_init`) so the network config is honored: the daemon is the sole
    // initializer of its process's transport singleton.
    let manager = TransportManager::init(TransportConfig {
        network: net::network_config_from_env(),
        ..TransportConfig::default()
    })?;

    // Resolve the Rerun endpoint: HOST a gRPC message proxy by
    // default (the required checkbox→scene bar — nobody else hosts one on a bare
    // machine) and advertise the bound URL, OR connect as a CLIENT to
    // `$CERULION_RERUN_URL` when set. On a hard misconfiguration the stream falls
    // back to `RecordingStream::disabled()` (a genuine no-op — NOT a `memory()`
    // sink, which would buffer every frame in RAM forever → OOM on a long-lived
    // daemon) so the CONTROL plane still runs with viz disabled. `rerun_url` is
    // advertised VERBATIM in the Hello banner every viewer connects to.
    let host::StreamResolution { rec, rerun_url } = host::resolve_stream();

    // Resolve the schema universe ONCE (builtins ∪ the bridge config's .msg
    // store) and share it: the worker renders with it, the daemon resolves
    // discover/attach schemas with it. Building it once (then cloning) guarantees
    // BOTH halves see the IDENTICAL schema set (no env TOCTOU between two parses)
    // and avoids re-parsing the store twice at startup.
    let walker = walker_from_bridge_config_env();
    // Live-only: vizd logs the scene skeleton (statics + blueprint) on
    // demand — including a runtime `set_blueprint` when a Studio layout changes.
    // The hosted proxy runs the fork's drop-temporal mode (see
    // `host::server_options`), which RETAINS the LATEST skeleton — F1 evicts
    // superseded blueprint STORES, F2 dedups re-logged statics entity-level — and
    // delivers it to each new viewer via rerun's per-client connect-history, so a
    // late joiner gets the current scene skeleton on connect (no heartbeat, no
    // clobber, no accumulation) while temporal data is never replayed.
    let worker = VizLogWorker::spawn(rec, walker.clone(), SinkState::new())?;

    // Production planes: the netd demand plane opens ONE lazy persistent connection
    // to the one-per-computer cerulion-netd daemon (spawning it on the first remote
    // demand).
    //
    // Plus a SECOND, dedicated connection for the catalog-change
    // subscription — a long-lived read cannot share the demand plane's single
    // mutex-held client without wedging or desyncing it. Same daemon, same socket,
    // same zenoh session; it holds no demands, so the refcount plane is untouched.
    //
    // The connection is standing, and that is the decided lifecycle:
    // unlike the demand plane's LAZY client, the forwarder connects at STARTUP and
    // stays connected, and netd treats a live connection as busy — so netd stays up
    // for vizd's whole life, keeping discovery warm, and reclaims itself via its
    // idle grace once the desk's consumers are gone. Closing Studio must therefore
    // stop vizd; that teardown belongs to the Studio shell. See
    // `cerulion_vizd::events`.
    let mut daemon = cerulion_vizd::start_on_socket(
        socket,
        cerulion_vizd::DEFAULT_POLL_INTERVAL,
        manager,
        worker,
        walker,
        rerun_url,
        Arc::new(NetdDemandPlane::new()),
        Arc::new(NetdCatalogEventSource::new()),
    )?;
    tracing::info!(
        socket = %daemon.socket_path().display(),
        "cerulion-vizd running — SIGINT/SIGTERM to stop"
    );
    let telemetry = cerulion_vizd::telemetry::Telemetry::start();

    // Block until a signal flips the flag (SIGINT + SIGTERM via the `termination`
    // feature), then shut the daemon down cleanly.
    let running = Arc::new(AtomicBool::new(true));
    {
        let running = Arc::clone(&running);
        ctrlc::set_handler(move || running.store(false, Ordering::SeqCst))?;
    }
    while running.load(Ordering::SeqCst) {
        std::thread::sleep(Duration::from_millis(200));
    }

    tracing::info!("cerulion-vizd shutting down");
    if let Some(telemetry) = telemetry {
        telemetry.shutdown();
    }
    daemon.shutdown();
    Ok(())
}

fn init_tracing() {
    use tracing_subscriber::{fmt, EnvFilter};
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
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
        // --help wins even when it follows an otherwise-unknown arg.
        assert_eq!(parse(&["--bogus", "--help"]), ArgAction::Help);
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
    fn usage_names_the_new_network_env_vars() {
        // The help text documents the locator env the `viz` verb threads in.
        assert!(USAGE.contains(net::CONNECT_ENV));
        assert!(USAGE.contains(net::LISTEN_ENV));
        assert!(USAGE.contains(net::NETWORK_ENV));
    }
}
