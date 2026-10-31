// SPDX-License-Identifier: AGPL-3.0-only
//! The streamable HTTP transport, guarded by a bearer token.
//!
//! Hosted on hyper directly. Every request must carry
//! `Authorization: Bearer <token>`; anything else is answered `401` before
//! the MCP layer sees it. rmcp's own host check (loopback names by default)
//! still applies behind the token.

use std::convert::Infallible;
use std::net::SocketAddr;
use std::sync::Arc;

use http_body_util::{combinators::BoxBody, BodyExt, Full};
use hyper::body::{Bytes, Incoming};
use hyper::header::{AUTHORIZATION, WWW_AUTHENTICATE};
use hyper::{Request, Response, StatusCode};
use hyper_util::rt::TokioIo;
use rmcp::transport::streamable_http_server::session::local::LocalSessionManager;
use rmcp::transport::streamable_http_server::{StreamableHttpServerConfig, StreamableHttpService};
use tokio::net::TcpListener;
use tower_service::Service;

use crate::observer::GraphFeed;
use crate::server::RcpServer;
use crate::RcpdError;

type HttpResponse = Response<BoxBody<Bytes, Infallible>>;
type McpService = StreamableHttpService<RcpServer, LocalSessionManager>;

/// A non-empty bearer token.
#[derive(Clone)]
pub struct BearerToken(Arc<str>);

impl std::fmt::Debug for BearerToken {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("BearerToken(..)")
    }
}

impl BearerToken {
    /// Accept the token read from [`TOKEN_ENV`](crate::TOKEN_ENV).
    ///
    /// # Errors
    ///
    /// [`RcpdError::MissingToken`] when the value is absent, empty or all
    /// whitespace.
    pub fn from_env_value(value: Option<String>) -> Result<Self, RcpdError> {
        match value {
            Some(token) if !token.trim().is_empty() => Ok(Self(Arc::from(token))),
            _ => Err(RcpdError::MissingToken),
        }
    }

    /// Whether `header` is exactly `Bearer <token>`. Compares in constant
    /// time over the token's length.
    fn admits(&self, header: Option<&[u8]>) -> bool {
        let Some(presented) = header.and_then(|h| h.strip_prefix(b"Bearer ")) else {
            return false;
        };
        let expected = self.0.as_bytes();
        presented.len() == expected.len()
            && presented
                .iter()
                .zip(expected)
                .fold(0u8, |acc, (a, b)| acc | (a ^ b))
                == 0
    }
}

/// A bound HTTP listener, not yet serving.
pub struct HttpServer {
    listener: TcpListener,
    token: BearerToken,
    mcp: McpService,
}

impl HttpServer {
    /// Bind `addr`. A non-loopback address is allowed but logged.
    ///
    /// # Errors
    ///
    /// [`RcpdError::Bind`] when the address cannot be bound.
    pub async fn bind(
        addr: SocketAddr,
        token: BearerToken,
        feed: GraphFeed,
    ) -> Result<Self, RcpdError> {
        Self::bind_serving(addr, token, move || RcpServer::new(feed.clone())).await
    }

    /// [`Self::bind`], with each session's handler made by `make_server`.
    ///
    /// # Errors
    ///
    /// [`RcpdError::Bind`] when the address cannot be bound.
    pub async fn bind_serving(
        addr: SocketAddr,
        token: BearerToken,
        make_server: impl Fn() -> RcpServer + Send + Sync + 'static,
    ) -> Result<Self, RcpdError> {
        if !addr.ip().is_loopback() {
            tracing::warn!(addr = %addr, "HTTP transport bound beyond loopback");
        }
        let listener = TcpListener::bind(addr)
            .await
            .map_err(|source| RcpdError::Bind { addr, source })?;
        let mcp = StreamableHttpService::new(
            move || Ok(make_server()),
            Arc::new(LocalSessionManager::default()),
            StreamableHttpServerConfig::default(),
        );
        Ok(Self {
            listener,
            token,
            mcp,
        })
    }

    /// The bound address.
    ///
    /// # Errors
    ///
    /// The socket's error when the address cannot be read.
    pub fn local_addr(&self) -> std::io::Result<SocketAddr> {
        self.listener.local_addr()
    }

    /// Serve connections until `shutdown` completes.
    pub async fn serve(self, shutdown: impl std::future::Future<Output = ()>) {
        tokio::pin!(shutdown);
        loop {
            let accepted = tokio::select! {
                () = &mut shutdown => break,
                accepted = self.listener.accept() => accepted,
            };
            let (stream, peer) = match accepted {
                Ok(pair) => pair,
                Err(e) => {
                    tracing::warn!(error = %e, "HTTP accept failed");
                    continue;
                }
            };
            let token = self.token.clone();
            let mcp = self.mcp.clone();
            tokio::spawn(async move {
                let service = hyper::service::service_fn(move |request: Request<Incoming>| {
                    let token = token.clone();
                    let mut mcp = mcp.clone();
                    async move {
                        let header = request.headers().get(AUTHORIZATION).map(|v| v.as_bytes());
                        if token.admits(header) {
                            mcp.call(request).await
                        } else {
                            Ok(unauthorized())
                        }
                    }
                });
                if let Err(e) = hyper::server::conn::http1::Builder::new()
                    .serve_connection(TokioIo::new(stream), service)
                    .await
                {
                    tracing::debug!(peer = %peer, error = %e, "HTTP connection ended");
                }
            });
        }
        self.mcp.config.cancellation_token.cancel();
    }
}

fn unauthorized() -> HttpResponse {
    let mut response = Response::new(Full::new(Bytes::from_static(b"unauthorized")).boxed());
    *response.status_mut() = StatusCode::UNAUTHORIZED;
    response.headers_mut().insert(
        WWW_AUTHENTICATE,
        hyper::header::HeaderValue::from_static("Bearer"),
    );
    response
}
