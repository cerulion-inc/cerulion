#[cfg(unix)]
use std::process::ExitCode;

#[cfg(unix)]
use cerulion_wsd::daemon::{self, WsdConfig};

#[cfg(unix)]
const USAGE: &str = "\
cerulion-wsd: the local workspace daemon.

USAGE:
    cerulion-wsd [--socket <path>] [--help]

The daemon serves workspace, graph and node inspection, plus versioned node and
graph edits, over a private Unix-domain socket (one JSON document per line). It
wraps the same engine the `cerulion` CLI uses, so a GUI such as Cerulion Studio
follows the CLI's rules instead of re-implementing them. Every connection is
greeted with a hello line naming the protocol version; edits take the same
`<workspace>/.cerulion/workspace.lock` the CLI takes, so the two never
interleave. Running it with no arguments STARTS the daemon on the default socket.

OPTIONS:
    --socket <path>        Bind this socket path instead of the default ladder.
    --inspect-node <lib>   Internal: load the node library at <lib>, print its
                           info JSON on stdout and exit. The daemon runs itself
                           this way, in a separate process, to validate a graph;
                           it is not meant to be run by hand.
    -h, --help             Print this help and exit.

ENVIRONMENT:
    CERULION_WSD_SOCKET        Default socket path. When unset the daemon uses
                               $XDG_RUNTIME_DIR/cerulion/wsd.sock, then
                               $HOME/.cerulion/wsd.sock, then
                               /tmp/cerulion-<euid>/wsd.sock.
    CERULION_WSD_HARD_EXIT_MS  Deadline for graceful shutdown before in-flight
                               requests are aborted, in ms (default 5000).
    RUST_LOG                   tracing filter for the daemon's stderr log
                               (default `info`).
";

#[cfg(unix)]
enum Action {
    Start(std::path::PathBuf),
    InspectNode(std::path::PathBuf),
    Help,
}

#[cfg(unix)]
#[tokio::main]
async fn main() -> ExitCode {
    init_tracing();
    let socket_path = match parse_args(std::env::args().skip(1)) {
        Ok(Action::Start(path)) => path,
        Ok(Action::InspectNode(lib)) => return inspect_node(&lib),
        Ok(Action::Help) => {
            print!("{USAGE}");
            return ExitCode::SUCCESS;
        }
        Err(message) => {
            eprintln!("cerulion-wsd: {message}\n\n{USAGE}");
            return ExitCode::from(2);
        }
    };
    let mut running = match daemon::start(WsdConfig {
        socket_path,
        ..WsdConfig::default()
    })
    .await
    {
        Ok(running) => running,
        Err(error) => {
            eprintln!("cerulion-wsd: {error}");
            return ExitCode::FAILURE;
        }
    };
    tracing::info!(socket = %running.socket_path().display(), "cerulion-wsd listening");
    let signal = async {
        let ctrl_c = tokio::signal::ctrl_c();
        #[cfg(unix)]
        {
            match tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate()) {
                Ok(mut terminate) => {
                    tokio::select! {
                        result = ctrl_c => result,
                        result = terminate.recv() => {
                            let _ = result;
                            Ok(())
                        },
                    }
                }
                Err(error) => {
                    tracing::warn!(error = %error, "SIGTERM listener unavailable; waiting for Ctrl-C");
                    ctrl_c.await
                }
            }
        }
        #[cfg(not(unix))]
        {
            ctrl_c.await
        }
    };
    let signal_failed = signal.await.is_err();
    if signal_failed {
        tracing::error!("signal listener failed");
    }
    tracing::info!("cerulion-wsd shutting down");
    running.shutdown().await;
    if signal_failed {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}

/// The daemon's stderr log: `RUST_LOG` when set, else `info` — the same shape
/// as `cerulion-netd`. Without a subscriber every engine warning the CLI shows
/// its user (and every daemon diagnostic) would be discarded.
#[cfg(unix)]
fn init_tracing() {
    use tracing_subscriber::{fmt, EnvFilter};
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    let _ = fmt()
        .with_env_filter(filter)
        .with_writer(std::io::stderr)
        .try_init();
}

/// The child half of `graph.validate`'s isolation: load the library HERE (its
/// constructors run in this short-lived process), write the raw info JSON the
/// parent parses with the same parser to the ORIGINAL stdout, exit 0 — or
/// explain and exit 1: on stderr, except when stderr is the thing that failed,
/// where the explanation goes to fd 1 (see [`report_isolation_refusal`]). See
/// [`cerulion_wsd::inspect`] for why fd 1 is redirected to fd 2 before the
/// library is loaded.
#[cfg(unix)]
fn inspect_node(lib: &std::path::Path) -> ExitCode {
    use std::io::Write as _;
    let mut document = match cerulion_wsd::inspect::isolate_document_channel() {
        Ok(document) => document,
        Err(error) => return report_isolation_refusal(lib, &error),
    };
    match cerulion_core::graph::node::DylibNodeEntry::load(lib).and_then(|entry| entry.info_json())
    {
        Ok(json) => match document
            .write_all(json.as_bytes())
            .and_then(|()| document.flush())
        {
            Ok(()) => ExitCode::SUCCESS,
            Err(error) => {
                eprintln!(
                    "cerulion-wsd --inspect-node {}: could not write the document: {error}",
                    lib.display()
                );
                ExitCode::FAILURE
            }
        },
        Err(error) => {
            eprintln!("cerulion-wsd --inspect-node {}: {error}", lib.display());
            ExitCode::FAILURE
        }
    }
}

/// Explain an isolation refusal on the channel that refusal did NOT just find
/// unusable, and fail.
///
/// The refusal has two causes and they do not share a reporting channel — this
/// is the whole reason [`cerulion_wsd::inspect::IsolationRefusal`] is a type
/// and not a substring of the message:
///
/// * [`StderrUnusable`](cerulion_wsd::inspect::IsolationRefusal::StderrUnusable)
///   is the fd-2 check, which runs BEFORE any descriptor is touched. stderr is
///   the thing that failed, so an `eprintln!` here PANICS on the write error
///   and the caller gets exit 101 with an empty stderr and no cause at all;
///   fd 1 is still this process's own stdout and is the one channel known
///   open, so the explanation goes there. A write failure on it is swallowed
///   deliberately: there is nowhere left to complain to, and panicking would
///   throw the exit code away too.
/// * [`SurgeryFailed`](cerulion_wsd::inspect::IsolationRefusal::SurgeryFailed)
///   is `F_DUPFD_CLOEXEC` on fd 1 or `dup2(2, 1)` failing — an exhausted fd
///   table, say. stderr is perfectly good and fd 1 is STILL the parent's
///   document pipe, so reporting on fd 1 there would push a diagnostic onto
///   the document channel while leaving the channel built for diagnostics
///   silent.
///
/// MEASURED reachability, and it differs per arm — which is what decides how
/// each one is pinned:
///
/// * the fd-2 arm is NOT reachable from a spawn. Rust's runtime opens
///   `/dev/null` over any of fd 0/1/2 closed at startup, so a stderr closed AT
///   EXEC never reaches it — only losing the descriptor mid-process does. Its
///   routing is therefore pinned on the source
///   (`the_fd2_refusal_arm_reports_on_fd_1_never_on_stderr`) rather than by
///   spawning this binary with `2>&-`.
/// * the surgery arm IS. `F_DUPFD_CLOEXEC` needs a free descriptor, so a small
///   enough `ulimit -n` produces it from a plain spawn: below that window the
///   tokio runtime cannot build (a panic, not this arm), above it the surgery
///   succeeds and the library load fails instead. The window's exact position
///   depends on how many descriptors the runtime takes, so
///   `inspect_channel_test.rs::an_exhausted_fd_table_reaches_the_surgery_refusal_on_stderr_with_an_empty_document`
///   SWEEPS for it and asserts this arm's behaviour there — exit 1, the
///   explanation on stderr, and an EMPTY document channel.
#[cfg(unix)]
fn report_isolation_refusal(
    lib: &std::path::Path,
    error: &cerulion_wsd::inspect::IsolationError,
) -> ExitCode {
    use cerulion_wsd::inspect::IsolationRefusal;
    use std::io::Write as _;
    let report = format!(
        "cerulion-wsd --inspect-node {}: could not separate the document channel from stdout: {error}",
        lib.display()
    );
    match error.refusal() {
        IsolationRefusal::StderrUnusable => {
            let mut document_channel = std::io::stdout();
            let _ = writeln!(document_channel, "{report}");
            let _ = document_channel.flush();
        }
        IsolationRefusal::SurgeryFailed => eprintln!("{report}"),
    }
    ExitCode::FAILURE
}

#[cfg(unix)]
fn path_arg(
    flag: &str,
    args: &mut impl Iterator<Item = String>,
) -> Result<std::path::PathBuf, String> {
    let path = args
        .next()
        .map(std::path::PathBuf::from)
        .ok_or_else(|| format!("{flag} requires a path"))?;
    if path.to_str().is_some_and(|path| path.starts_with('-')) {
        return Err(format!("{flag} path cannot start with `-`"));
    }
    if let Some(arg) = args.next() {
        return Err(format!("unexpected argument `{arg}`"));
    }
    Ok(path)
}

#[cfg(unix)]
fn parse_args(mut args: impl Iterator<Item = String>) -> Result<Action, String> {
    match args.next().as_deref() {
        None => Ok(Action::Start(cerulion_wsd::hygiene::default_socket_path())),
        Some("--help") | Some("-h") => Ok(Action::Help),
        Some("--socket") => path_arg("--socket", &mut args).map(Action::Start),
        Some(flag) if flag == cerulion_wsd::daemon::INSPECT_NODE_FLAG => {
            path_arg(flag, &mut args).map(Action::InspectNode)
        }
        Some(arg) => Err(format!("unrecognized argument `{arg}`")),
    }
}

#[cfg(all(test, unix))]
mod tests {
    use super::{parse_args, Action, USAGE};

    #[test]
    fn socket_parser_rejects_trailing_arguments() {
        let args = ["--socket", "/tmp/wsd.sock", "--help"]
            .into_iter()
            .map(str::to_owned);
        assert_eq!(
            parse_args(args)
                .err()
                .expect("trailing argument should fail"),
            "unexpected argument `--help`"
        );
    }

    #[test]
    fn socket_parser_rejects_option_like_path() {
        let args = ["--socket", "--help"].into_iter().map(str::to_owned);
        assert_eq!(
            parse_args(args)
                .err()
                .expect("option-like path should fail"),
            "--socket path cannot start with `-`"
        );
    }

    #[test]
    fn help_flags_ask_for_usage_and_usage_names_every_knob() {
        for flag in ["--help", "-h"] {
            assert!(matches!(
                parse_args([flag.to_owned()].into_iter()),
                Ok(Action::Help)
            ));
        }
        for needle in [
            "--socket <path>",
            cerulion_wsd::hygiene::SOCKET_ENV,
            cerulion_wsd::daemon::HARD_EXIT_ENV,
            "/tmp/cerulion-<euid>/wsd.sock",
        ] {
            assert!(USAGE.contains(needle), "usage must name {needle}");
        }
    }

    #[test]
    fn inspect_node_takes_exactly_one_path() {
        assert!(matches!(
            parse_args(["--inspect-node", "/x/libcam.so"].into_iter().map(str::to_owned)),
            Ok(Action::InspectNode(p)) if p == std::path::Path::new("/x/libcam.so")
        ));
        assert_eq!(
            parse_args(["--inspect-node"].into_iter().map(str::to_owned))
                .err()
                .unwrap(),
            "--inspect-node requires a path"
        );
        assert_eq!(
            parse_args(
                ["--inspect-node", "/x/a.so", "extra"]
                    .into_iter()
                    .map(str::to_owned)
            )
            .err()
            .unwrap(),
            "unexpected argument `extra`"
        );
        assert!(USAGE.contains(cerulion_wsd::daemon::INSPECT_NODE_FLAG));
    }

    #[test]
    fn no_arguments_starts_on_the_default_ladder() {
        assert!(matches!(
            parse_args(std::iter::empty()),
            Ok(Action::Start(_))
        ));
    }
}

#[cfg(not(unix))]
fn main() -> std::process::ExitCode {
    eprintln!("cerulion-wsd requires a Unix platform");
    std::process::ExitCode::FAILURE
}
