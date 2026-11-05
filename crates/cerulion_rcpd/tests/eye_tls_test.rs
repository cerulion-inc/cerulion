// SPDX-License-Identifier: AGPL-3.0-only
//! The eye's client over TLS against a local stand-in route with a
//! certificate from a test root: the shipped client refuses a route whose
//! certificate the platform does not trust and sends nothing; the same
//! client trusting the test root speaks HTTP/2 negotiated by ALPN, gets its
//! answer, and no log line at any level holds key bytes.
//!
//! One test per binary: it installs the process's log subscriber.

mod eye_common;

use std::path::Path;
use std::sync::{mpsc, Arc, Mutex, PoisonError};
use std::time::{Duration, Instant};

use cerulion_rcpd::eye::answer::Answer;
use cerulion_rcpd::eye::client::JevClient;
use cerulion_rcpd::eye::host::{request, Admit, Completed, EyeCore, EyeSettings, FrameStore};
use cerulion_rcpd::eye::limits::jpeg_header_for_test;
use cerulion_rcpd::eye::verdict::Outcome;
use cerulion_rcpd::manifest::RcpManifest;
use eye_common::{eyes_yaml, good_body, holds_key, key_file, Reply, Stub, ANSWERED_MODEL};
use serde_json::json;

/// Every log line of the process.
#[derive(Clone, Default)]
struct Logs(Arc<Mutex<Vec<u8>>>);

impl std::io::Write for Logs {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.0
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .extend_from_slice(buf);
        Ok(buf.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

fn install_logs() -> Logs {
    use tracing_subscriber::util::SubscriberInitExt as _;
    let logs = Logs::default();
    let writer = logs.clone();
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::TRACE)
        .with_ansi(false)
        .with_writer(move || writer.clone())
        .finish()
        .try_init()
        .expect("the one log subscriber");
    logs
}

/// The eye's rules for a manifest routed to `base_url`, with a key on file.
fn core(dir: &Path, base_url: &str) -> EyeCore {
    let key = key_file(dir);
    let manifest = eye_common::write_manifest(dir, &eyes_yaml("tls", base_url, &key));
    let loaded = RcpManifest::load_full(&manifest).expect("manifest");
    let settings = EyeSettings::resolve(&loaded.eyes, dir, dir, None).expect("settings");
    EyeCore::new(Arc::new(settings), Instant::now())
}

/// Admit one ask on a held 640 x 480 frame, send it through `client`, and
/// return the answer the gate gets.
async fn look(core: &mut EyeCore, client: JevClient, id: u64) -> Answer {
    let mut frames = FrameStore::default();
    frames.push(1, &jpeg_header_for_test(640, 480, 2_000));
    let ask = json!({
        "ask_id": id,
        "question_set": "press",
        "frame": { "topic": "/tls/wrist/image", "seq": 1 },
        "deadline_ms": 1_500,
        "numbers": { "case": 0 },
    })
    .to_string();
    let now = Instant::now();
    let Admit::Send(dispatch) = core.admit(&ask, now, now, now, &frames) else {
        panic!("the ask must be sent");
    };
    let (tx, rx) = mpsc::channel();
    request(Arc::new(client), *dispatch, tx).await;
    let mut answer = None;
    while let Ok(done) = rx.try_recv() {
        if let Completed::Answer { answer: a, .. } = done {
            answer = Some(*a);
        }
    }
    answer.expect("an answer")
}

#[tokio::test(flavor = "multi_thread")]
async fn the_client_checks_the_route_certificate_and_speaks_http2_over_tls() {
    let logs = install_logs();
    let (stub, root) = Stub::start_tls(|_| Reply::ok(good_body(ANSWERED_MODEL, 0.93, 0.68))).await;
    assert!(stub.base_url().starts_with("https://127.0.0.1:"));

    // The shipped client does not trust the test root: the handshake fails
    // and no request reaches the route.
    let dir = tempfile::tempdir().expect("tempdir");
    let mut untrusting = core(dir.path(), &stub.base_url());
    let endpoint = untrusting.settings().endpoint.clone().expect("endpoint");
    let shipped = JevClient::new(endpoint.clone()).expect("client");
    let refused = look(&mut untrusting, shipped, 1).await;
    assert_eq!(
        (refused.outcome, refused.detail.as_deref()),
        (Outcome::Unavailable, Some("connect")),
        "{refused:?}"
    );
    assert_eq!(stub.count(), 0, "no request over an untrusted connection");
    // The stub sees the refused handshake when it reads the client's alert,
    // which can come after the client has returned.
    let waited = Instant::now();
    while stub.handshake_failures() == 0 && waited.elapsed() < Duration::from_secs(10) {
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    assert!(stub.handshake_failures() >= 1, "the handshake was refused");

    // The same client trusting the test root: TLS, HTTP/2 by ALPN, an answer.
    let other = tempfile::tempdir().expect("tempdir");
    let mut trusting = core(other.path(), &stub.base_url());
    let client = JevClient::with_extra_roots(endpoint, vec![root]).expect("client");
    let answer = look(&mut trusting, client, 2).await;
    assert_eq!(answer.outcome, Outcome::Ok, "{answer:?}");
    let seen = stub.seen();
    assert_eq!(seen.len(), 1);
    assert_eq!(
        seen[0].version,
        hyper::Version::HTTP_2,
        "h2 negotiated by ALPN"
    );
    assert!(
        holds_key(seen[0].header("authorization").unwrap_or("").as_bytes()),
        "the route got the key, so the scan below can find it"
    );

    // No log line at any level holds key bytes, the HTTP/2 path included.
    let captured = logs
        .0
        .lock()
        .unwrap_or_else(PoisonError::into_inner)
        .clone();
    let text = String::from_utf8_lossy(&captured);
    assert!(text.contains("h2::"), "the capture holds the HTTP/2 trace");
    assert!(!holds_key(&captured), "a log line holds key bytes");
    assert!(!holds_key(answer.to_json().as_bytes()));
}
