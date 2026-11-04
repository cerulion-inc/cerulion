// SPDX-License-Identifier: AGPL-3.0-only
//! A local stand-in for the System One route, and the manifest and key the
//! eye tests share. No test here reaches the network or holds a real key.

#![allow(dead_code, reason = "each test binary uses a different part")]

use std::convert::Infallible;
use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::Duration;

use http_body_util::{BodyExt, Full};
use hyper::body::{Bytes, Incoming};
use hyper::server::conn::http1;
use hyper::service::service_fn;
use hyper::{Request, Response};
use hyper_util::rt::TokioIo;
use serde_json::{json, Value};

/// A key made up for tests: distinctive bytes a scan can look for.
pub const TEST_KEY: &str = "test-only-jev-key-7f3a9c2e5b1d4068aa";

/// What the stub answers one request with.
#[derive(Debug, Clone)]
pub struct Reply {
    pub status: u16,
    pub body: String,
    pub delay: Duration,
    pub retry_after: Option<u64>,
}

impl Reply {
    pub fn ok(body: Value) -> Self {
        Self::status(200, body)
    }

    pub fn status(status: u16, body: Value) -> Self {
        Self {
            status,
            body: body.to_string(),
            delay: Duration::ZERO,
            retry_after: None,
        }
    }

    pub fn after(mut self, delay: Duration) -> Self {
        self.delay = delay;
        self
    }
}

/// One request the stub received.
#[derive(Debug, Clone)]
pub struct Seen {
    pub path: String,
    pub headers: Vec<(String, String)>,
    pub body: Vec<u8>,
}

impl Seen {
    pub fn header(&self, name: &str) -> Option<&str> {
        self.headers
            .iter()
            .find(|(k, _)| k.eq_ignore_ascii_case(name))
            .map(|(_, v)| v.as_str())
    }

    pub fn json(&self) -> Value {
        serde_json::from_slice(&self.body).expect("request JSON")
    }
}

type Script = Box<dyn FnMut(&Value) -> Reply + Send>;

/// The stand-in route on 127.0.0.1. Dropping it stops accepting.
pub struct Stub {
    pub addr: SocketAddr,
    seen: Arc<Mutex<Vec<Seen>>>,
    accept: tokio::task::JoinHandle<()>,
}

impl Drop for Stub {
    fn drop(&mut self) {
        self.accept.abort();
    }
}

impl Stub {
    /// Serve `script`, which picks the reply from the request body.
    pub async fn start(script: impl FnMut(&Value) -> Reply + Send + 'static) -> Self {
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0")
            .await
            .expect("bind stub");
        let addr = listener.local_addr().expect("addr");
        let seen = Arc::new(Mutex::new(Vec::new()));
        let script: Arc<Mutex<Script>> = Arc::new(Mutex::new(Box::new(script)));
        let accept_seen = Arc::clone(&seen);
        let accept = tokio::spawn(async move {
            loop {
                let Ok((stream, _)) = listener.accept().await else {
                    return;
                };
                let seen = Arc::clone(&accept_seen);
                let script = Arc::clone(&script);
                tokio::spawn(async move {
                    let service = service_fn(move |request: Request<Incoming>| {
                        let seen = Arc::clone(&seen);
                        let script = Arc::clone(&script);
                        async move {
                            let path = request.uri().path().to_owned();
                            let headers = request
                                .headers()
                                .iter()
                                .map(|(k, v)| {
                                    (k.as_str().to_owned(), v.to_str().unwrap_or("").to_owned())
                                })
                                .collect();
                            let body = request
                                .into_body()
                                .collect()
                                .await
                                .map(|b| b.to_bytes().to_vec())
                                .unwrap_or_default();
                            let parsed: Value =
                                serde_json::from_slice(&body).unwrap_or(Value::Null);
                            let reply =
                                (script.lock().unwrap_or_else(PoisonError::into_inner))(&parsed);
                            seen.lock()
                                .unwrap_or_else(PoisonError::into_inner)
                                .push(Seen {
                                    path,
                                    headers,
                                    body,
                                });
                            if !reply.delay.is_zero() {
                                tokio::time::sleep(reply.delay).await;
                            }
                            let mut response = Response::builder()
                                .status(reply.status)
                                .header("content-type", "application/json");
                            if let Some(secs) = reply.retry_after {
                                response = response.header("retry-after", secs.to_string());
                            }
                            Ok::<_, Infallible>(
                                response
                                    .body(Full::new(Bytes::from(reply.body)))
                                    .expect("response"),
                            )
                        }
                    });
                    let _ = http1::Builder::new()
                        .serve_connection(TokioIo::new(stream), service)
                        .await;
                });
            }
        });
        Self { addr, seen, accept }
    }

    /// The base URL a manifest names.
    pub fn base_url(&self) -> String {
        format!("http://{}/api", self.addr)
    }

    /// Every request so far.
    pub fn seen(&self) -> Vec<Seen> {
        self.seen
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }

    pub fn count(&self) -> usize {
        self.seen
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .len()
    }
}

/// A System One reply from `model` with the edge visible at `visible`, the
/// rim's bottom edge at `bottom` (a fraction of the height).
pub fn good_body(model: &str, visible: f64, bottom: f64) -> Value {
    json!({
        "id": "gen-dec-test",
        "model": model,
        "provider": "TypeSafe",
        "answers": {
            "edge_visible": { "type": "noul", "noul": visible },
            "rim": {
                "type": "box",
                "box": { "top": 0.44, "bottom": bottom, "left": 0.25, "right": 0.63 },
                "confidence": 0.67
            }
        },
        "usage": { "input_tokens": 1425, "output_tokens": 56, "cost": 0.00006 }
    })
}

/// The model the stub answers as when it answers well.
pub const ANSWERED_MODEL: &str = "typesafe/jev-1.14-20261014";

/// An `eyes.jev` section on topics under `prefix`, routed to `base_url`,
/// with the key file at `key_file`.
pub fn eyes_yaml(prefix: &str, base_url: &str, key_file: &Path) -> String {
    format!(
        "eyes:
  jev:
    base_url: {base_url}
    model: typesafe/jev-1.14
    credential_file: {key}
    frame_topic: /{prefix}/wrist/image
    ask_topic: /{prefix}/eye/ask
    answer_topic: /{prefix}/eye/answer
    question_sets:
      - name: press
        state: Wrist camera frame from a small arm in front of a coffee machine; the rim is the bright band across the machine head.
        numbers: [case]
        questions:
          edge_visible:
            type: noul
            instructions: The lower edge of the chrome rim is visible in `frame`.
          rim:
            type: box
            instructions: The chrome rim band on the front of the machine head in `frame`.
        visible: {{ question: edge_visible, threshold: 0.7 }}
",
        key = key_file.display()
    )
}

/// Write `yaml` as `rcp.yaml` in `dir`.
pub fn write_manifest(dir: &Path, yaml: &str) -> PathBuf {
    let path = dir.join("rcp.yaml");
    std::fs::write(&path, yaml).expect("write manifest");
    path
}

/// The key file in `dir`, mode 0600.
pub fn key_file(dir: &Path) -> PathBuf {
    let path = dir.join("robot-jev.key");
    cerulion_rcpd::eye::credential::write_credential(&path, TEST_KEY).expect("key");
    path
}

/// Whether `haystack` holds the test key, whole or as a long fragment.
pub fn holds_key(haystack: &[u8]) -> bool {
    let key = TEST_KEY.as_bytes();
    // Any 16-byte window of the key counts.
    key.windows(16)
        .any(|w| haystack.windows(w.len()).any(|h| h == w))
}
