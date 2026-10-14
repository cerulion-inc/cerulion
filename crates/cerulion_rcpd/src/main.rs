// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion-rcpd --manifest rcp.yaml [--stdio | --http [ADDR]]`.

use std::net::SocketAddr;
use std::path::PathBuf;
use std::process::ExitCode;

use cerulion_rcpd::http::{BearerToken, HttpServer};
use cerulion_rcpd::manifest::RcpManifest;
use cerulion_rcpd::observer::{GraphFeed, GraphSource};
use cerulion_rcpd::server::RcpServer;
use cerulion_rcpd::{RcpdError, DEFAULT_HTTP_ADDR, TOKEN_ENV};
use clap::Parser;
use rmcp::ServiceExt;

#[derive(Parser, Debug)]
#[command(name = "cerulion-rcpd", version, about)]
struct Args {
    /// The `rcp.yaml` manifest.
    #[arg(long)]
    manifest: PathBuf,
    /// Serve MCP over stdin/stdout (the default).
    #[arg(long, conflicts_with = "http")]
    stdio: bool,
    /// Serve MCP over streamable HTTP. Needs CERULION_RCP_TOKEN.
    #[arg(long, value_name = "ADDR", num_args = 0..=1, default_missing_value = DEFAULT_HTTP_ADDR)]
    http: Option<SocketAddr>,
}

#[tokio::main]
async fn main() -> ExitCode {
    init_tracing();
    match run(Args::parse()).await {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            tracing::error!(error = %e, "cerulion-rcpd stopped");
            ExitCode::FAILURE
        }
    }
}

fn init_tracing() {
    let filter = tracing_subscriber::EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info"));
    tracing_subscriber::fmt()
        .with_env_filter(filter)
        .with_writer(std::io::stderr)
        .init();
}

async fn run(args: Args) -> Result<(), RcpdError> {
    let token = match args.http {
        Some(_) => Some(BearerToken::from_env_value(std::env::var(TOKEN_ENV).ok())?),
        None => None,
    };
    let identity = RcpManifest::load(&args.manifest)?;
    let manager = cerulion_core::TransportManager::init_default()?;
    let source = GraphSource::new(manager, identity);
    let feed = source.feed();
    let _observer = source.spawn()?;
    match (args.http, token) {
        (Some(addr), Some(token)) => serve_http(addr, token, feed).await,
        _ => serve_stdio(feed).await,
    }
}

async fn serve_stdio(feed: GraphFeed) -> Result<(), RcpdError> {
    tracing::info!("serving MCP on stdio");
    let session = RcpServer::new(feed)
        .serve(rmcp::transport::stdio())
        .await
        .map_err(|e| RcpdError::Session {
            reason: e.to_string(),
        })?;
    session.waiting().await.map_err(|e| RcpdError::Session {
        reason: e.to_string(),
    })?;
    Ok(())
}

async fn serve_http(
    addr: SocketAddr,
    token: BearerToken,
    feed: GraphFeed,
) -> Result<(), RcpdError> {
    let server = HttpServer::bind(addr, token, feed).await?;
    let bound = server
        .local_addr()
        .map_err(|source| RcpdError::Bind { addr, source })?;
    tracing::info!(addr = %bound, "serving MCP on streamable HTTP");
    server
        .serve(async {
            if let Err(e) = tokio::signal::ctrl_c().await {
                tracing::error!(error = %e, "cannot listen for ctrl-c; shutting down");
            }
        })
        .await;
    Ok(())
}
