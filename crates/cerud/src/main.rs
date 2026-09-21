// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerud` binary: serve the ops verbs over the Unix-domain-socket dev
//! transport.
//!
//! This is the dev/skeleton entry point. The production entry point (serving
//! over the iroh `cerulion/ops/1` transport on the well-known ops port) plugs
//! into the SAME [`cerud::server::OpsServer`] via the transport seam; that
//! entry point is not implemented here.

#[cfg(unix)]
fn main() -> std::process::ExitCode {
    unix_main::run()
}

#[cfg(not(unix))]
fn main() -> std::process::ExitCode {
    eprintln!("cerud requires a Unix platform (the dev transport is a Unix domain socket)");
    std::process::ExitCode::FAILURE
}

#[cfg(unix)]
mod unix_main {
    use std::path::PathBuf;
    use std::process::ExitCode;

    use clap::Parser;

    use cerud::authz::{Authorizer, DenyAllAuthorizer, PermissiveDevAuthorizer};
    use cerud::constants::resolve_ops_port;
    use cerud::receipt::ReceiptLog;
    use cerud::server::OpsServer;
    use cerud::transport::UnixSocketListener;
    use cerud::verbs::{InventoryVerb, LogTailVerb, RestartVerb, VerbRegistry};

    /// The open robot-side Cerulion ops service (mechanical verbs only).
    #[derive(Parser, Debug)]
    #[command(name = "cerud", version, about)]
    struct Args {
        /// Unix-domain-socket path to serve the dev transport on.
        #[arg(long, default_value = "/tmp/cerulion-ops.sock")]
        socket: PathBuf,

        /// Append-only, hash-chained receipt audit-log path.
        #[arg(long, default_value = "/var/lib/cerulion/cerud-receipts.log")]
        receipt_log: PathBuf,

        /// Allow-listed root directory for the `log-tail` verb.
        #[arg(long, default_value = "/var/log/cerulion")]
        log_root: PathBuf,

        /// Use the PERMISSIVE dev authorizer (allows every verb). NOT for
        /// production — the default is deny-by-default until pairing lands.
        #[arg(long)]
        dev_permissive: bool,
    }

    pub fn run() -> ExitCode {
        tracing_subscriber::fmt()
            .with_env_filter(
                tracing_subscriber::EnvFilter::try_from_default_env()
                    .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
            )
            .init();

        let args = Args::parse();

        // The production ops port is pinned even though the dev transport uses
        // a socket path — surface it so the operator sees the well-known port.
        match resolve_ops_port() {
            Ok(port) => tracing::info!(ops_port = port, "cerud ops port (production seam)"),
            Err(e) => {
                tracing::error!(error = %e, "invalid ops port override");
                return ExitCode::FAILURE;
            }
        }

        let authorizer: Box<dyn Authorizer> = if args.dev_permissive {
            tracing::warn!(
                "cerud: PERMISSIVE dev authorizer enabled — every verb is allowed. \
                 DO NOT use this in production."
            );
            Box::new(PermissiveDevAuthorizer)
        } else {
            tracing::info!("cerud: deny-by-default authorizer (no pairing configured yet)");
            Box::new(DenyAllAuthorizer)
        };

        let receipts = match ReceiptLog::open(&args.receipt_log) {
            Ok(log) => log,
            Err(e) => {
                tracing::error!(error = %e, path = %args.receipt_log.display(), "cannot open receipt log");
                return ExitCode::FAILURE;
            }
        };

        let mut verbs = VerbRegistry::new();
        verbs
            .register(Box::new(InventoryVerb))
            .register(Box::new(LogTailVerb::new(args.log_root.clone())))
            .register(Box::new(RestartVerb::new()));

        let server = OpsServer::new("cerud", authorizer, verbs, receipts);

        let mut listener = match UnixSocketListener::bind(&args.socket) {
            Ok(l) => l,
            Err(e) => {
                tracing::error!(error = %e, path = %args.socket.display(), "cannot bind ops socket");
                return ExitCode::FAILURE;
            }
        };

        match server.serve_forever(&mut listener) {
            Ok(()) => ExitCode::SUCCESS,
            Err(e) => {
                tracing::error!(error = %e, "cerud server stopped");
                ExitCode::FAILURE
            }
        }
    }
}
