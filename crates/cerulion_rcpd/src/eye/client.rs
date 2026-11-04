// SPDX-License-Identifier: AGPL-3.0-only
//! The System One request and the HTTP client that sends it.
//!
//! One request is `{"model", "state", "questions", "assets"}`: the state is
//! `{"scene", "frame", "numbers"}` with `frame` holding the image placeholder,
//! and `assets` maps the placeholder to the JPEG, base64-encoded. The
//! placeholder is `frame-` and the first 16 hex digits of the image's
//! SHA-256, so the same frame always gives the same body, and the body's
//! SHA-256 can be recomputed from a recording. Every request sends
//! `TypeSafe-Danger-Zone-Images: true`, which the API requires for images
//! and Box questions.
//!
//! The client keeps its connections: HTTP/2 over TLS where the route offers
//! it (HTTP/1.1 otherwise, and on a plain `http` loopback route), a pool
//! idle timeout of 90 s, HTTP/2 keep-alive pings every 20 s while idle, and
//! TCP keep-alive, so a look after a quiet spell reuses a warm connection.
//! It follows no redirects. The key goes only into the `Authorization`
//! header, marked sensitive.

use std::sync::Arc;
use std::time::Duration;

use base64::Engine as _;
use http_body_util::{BodyExt as _, Full, Limited};
use hyper::body::Bytes;
use hyper::header::{self, HeaderValue};
use hyper::Request;
use hyper_util::client::legacy::connect::HttpConnector;
use hyper_util::client::legacy::Client;
use hyper_util::rt::{TokioExecutor, TokioTimer};
use serde_json::{json, Map, Value};

use crate::eye::credential::Credential;
use crate::eye::limits::LATE_GRACE;

/// The largest response body read, bytes.
pub const MAX_RESPONSE_BYTES: usize = 256 * 1024;

/// The header the System One API requires for image assets and Box
/// questions.
pub const IMAGES_HEADER: &str = "TypeSafe-Danger-Zone-Images";

/// The SHA-256 of `bytes`, lowercase hex.
pub fn sha256_hex(bytes: &[u8]) -> String {
    cerud::hash::sha256_hex(bytes)
}

/// The asset placeholder for an image whose SHA-256 is `image_sha256`.
pub fn placeholder(image_sha256: &str) -> String {
    format!("frame-{}", &image_sha256[..16.min(image_sha256.len())])
}

/// The state one request sends: the scene text, the image placeholder and
/// the numbers (omitted when there are none).
pub fn request_state(scene: &str, placeholder: &str, numbers: &Map<String, Value>) -> Value {
    let mut state = Map::new();
    state.insert(String::from("scene"), Value::from(scene));
    state.insert(String::from("frame"), Value::from(placeholder));
    if !numbers.is_empty() {
        state.insert(String::from("numbers"), Value::Object(numbers.clone()));
    }
    Value::Object(state)
}

/// The exact request body: `model`, the `state` and `questions` as given,
/// and the one image asset under its placeholder.
pub fn request_body(model: &str, state: &Value, questions: &Value, image: &[u8]) -> Vec<u8> {
    let image_sha256 = sha256_hex(image);
    let body = json!({
        "model": model,
        "state": state,
        "questions": questions,
        "assets": {
            placeholder(&image_sha256): {
                "type": "image",
                "format": "jpeg",
                "data_base64": base64::engine::general_purpose::STANDARD.encode(image),
            }
        },
    });
    serde_json::to_vec(&body).unwrap_or_default()
}

/// What the route answered.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HttpReply {
    /// The status code.
    pub status: u16,
    /// The `Retry-After` header in seconds, when present.
    pub retry_after: Option<Duration>,
    /// The body, at most [`MAX_RESPONSE_BYTES`].
    pub body: Vec<u8>,
}

/// Why the route gave no reply.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HttpFailure {
    /// The connection could not be made or broke before a status arrived.
    Connect,
    /// The body could not be read, or exceeded [`MAX_RESPONSE_BYTES`].
    Body,
}

impl HttpFailure {
    /// The `detail` an answer frame carries.
    pub fn code(self) -> &'static str {
        match self {
            Self::Connect => "connect",
            Self::Body => "response_body",
        }
    }
}

type Connector = hyper_rustls::HttpsConnector<HttpConnector>;

/// The HTTP client for one route.
#[derive(Clone)]
pub struct JevClient {
    http: Client<Connector, Full<Bytes>>,
    endpoint: url::Url,
}

impl std::fmt::Debug for JevClient {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("JevClient")
            .field("endpoint", &self.endpoint.as_str())
            .finish_non_exhaustive()
    }
}

impl JevClient {
    /// A client for `endpoint` (see [`crate::eye::config::endpoint`]): TLS
    /// with the `ring` provider and the platform's certificate verifier,
    /// HTTP/1.1 and HTTP/2 offered by ALPN.
    ///
    /// # Errors
    ///
    /// The reason the TLS configuration could not be built.
    pub fn new(endpoint: url::Url) -> Result<Self, String> {
        use rustls_platform_verifier::BuilderVerifierExt as _;
        let provider = Arc::new(rustls::crypto::ring::default_provider());
        let tls = rustls::ClientConfig::builder_with_provider(provider)
            .with_safe_default_protocol_versions()
            .and_then(|builder| builder.with_platform_verifier())
            .map_err(|e| format!("cannot build the Jev TLS configuration: {e}"))?
            .with_no_client_auth();
        let mut tcp = HttpConnector::new();
        tcp.enforce_http(false);
        tcp.set_connect_timeout(Some(Duration::from_millis(crate::eye::limits::TIMEOUT_MS)));
        tcp.set_nodelay(true);
        tcp.set_keepalive(Some(Duration::from_secs(30)));
        let connector = hyper_rustls::HttpsConnectorBuilder::new()
            .with_tls_config(tls)
            .https_or_http()
            .enable_http1()
            .enable_http2()
            .wrap_connector(tcp);
        let http = Client::builder(TokioExecutor::new())
            .timer(TokioTimer::new())
            .pool_timer(TokioTimer::new())
            .pool_idle_timeout(Duration::from_secs(90))
            .pool_max_idle_per_host(crate::eye::limits::MAX_IN_FLIGHT)
            .http2_keep_alive_interval(Duration::from_secs(20))
            .http2_keep_alive_timeout(Duration::from_secs(5))
            .http2_keep_alive_while_idle(true)
            .build(connector);
        Ok(Self { http, endpoint })
    }

    /// The endpoint requests go to.
    pub fn endpoint(&self) -> &url::Url {
        &self.endpoint
    }

    /// POST `body` with `key`, giving up after [`LATE_GRACE`].
    ///
    /// # Errors
    ///
    /// [`HttpFailure`] when no complete reply arrived.
    pub async fn post(&self, body: Vec<u8>, key: &Credential) -> Result<HttpReply, HttpFailure> {
        let uri: hyper::Uri = self
            .endpoint
            .as_str()
            .parse()
            .map_err(|_| HttpFailure::Connect)?;
        let mut auth = HeaderValue::from_str(&format!("Bearer {}", key.expose()))
            .map_err(|_| HttpFailure::Connect)?;
        auth.set_sensitive(true);
        let request = Request::post(uri)
            .header(header::AUTHORIZATION, auth)
            .header(header::CONTENT_TYPE, "application/json")
            .header(
                header::USER_AGENT,
                concat!("cerulion-rcpd/", env!("CARGO_PKG_VERSION")),
            )
            .header(IMAGES_HEADER, "true")
            .body(Full::new(Bytes::from(body)))
            .map_err(|_| HttpFailure::Connect)?;
        let exchange = async {
            let response = self.http.request(request).await.map_err(|e| {
                tracing::debug!(error = %e, "Jev request failed");
                HttpFailure::Connect
            })?;
            let status = response.status().as_u16();
            let retry_after = response
                .headers()
                .get(header::RETRY_AFTER)
                .and_then(|v| v.to_str().ok())
                .and_then(|v| v.trim().parse::<u64>().ok())
                .map(Duration::from_secs);
            let body = Limited::new(response.into_body(), MAX_RESPONSE_BYTES)
                .collect()
                .await
                .map_err(|_| HttpFailure::Body)?
                .to_bytes()
                .to_vec();
            Ok(HttpReply {
                status,
                retry_after,
                body,
            })
        };
        tokio::time::timeout(LATE_GRACE, exchange)
            .await
            .unwrap_or(Err(HttpFailure::Connect))
    }
}

#[cfg(test)]
mod tests {
    use serde_json::{json, Map};

    use super::{placeholder, request_body, request_state, sha256_hex};

    #[test]
    fn the_same_frame_and_questions_give_the_same_body() {
        let image = crate::eye::limits::jpeg_header_for_test(640, 480, 10);
        let state = request_state("scene", &placeholder(&sha256_hex(&image)), &Map::new());
        let questions = json!({ "rim": { "type": "box", "instructions": "The rim in `frame`." } });
        let a = request_body("typesafe/jev-1.14", &state, &questions, &image);
        let b = request_body("typesafe/jev-1.14", &state, &questions, &image);
        assert_eq!(a, b);
        let body: serde_json::Value = serde_json::from_slice(&a).expect("json");
        let key = placeholder(&sha256_hex(&image));
        assert_eq!(body["state"]["frame"], key.as_str());
        assert_eq!(body["assets"][&key]["format"], "jpeg");
        assert!(body["state"].get("numbers").is_none());
        assert_eq!(body["model"], "typesafe/jev-1.14");
    }
}
