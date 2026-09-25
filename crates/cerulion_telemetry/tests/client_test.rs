// SPDX-License-Identifier: AGPL-3.0-only
//! The live client against local sockets: delivery to a mock server, the hard
//! shutdown budget against a server that never answers, the no-op paths
//! (`POSTHOG_API_KEY` unset, consent off), and queue overflow.
#![cfg(feature = "posthog")]

use cerulion_telemetry::{
    Client, Common, EventSpec, ShutdownOutcome, Value, DEFAULT_SHUTDOWN_BUDGET, QUEUE_CAPACITY,
};
use std::io::{Read, Write};
use std::net::TcpListener;
use std::sync::{mpsc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

const SUB: &str = "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc";
const ANON: &str = "anon:6ba7b810-9dad-41d1-80b4-00c04fd430c8";
const CMD: EventSpec = EventSpec {
    name: "cli_command_run",
    allowlist: &["command", "duration_ms"],
};

static ENV: Mutex<()> = Mutex::new(());

fn common() -> Common {
    Common {
        surface: "cli".into(),
        env: "test".into(),
        app_version: "0.0.0".into(),
        channel: None,
    }
}

type Request = (String, serde_json::Value);

/// One-shot HTTP/1.1 server: reads one request, replies 200, returns the body.
fn mock_server() -> (String, mpsc::Receiver<Request>) {
    let (host, go, rx, _server) = gated_mock_server(1);
    go.send(()).expect("go");
    (host, rx)
}

/// Handle to a [`gated_mock_server`] thread; [`GatedServer::stop`] ends it
/// even when it still awaits connections that never came.
struct GatedServer {
    addr: std::net::SocketAddr,
    thread: thread::JoinHandle<()>,
}

impl GatedServer {
    /// Drop the release sender, then knock on the listener so an `accept`
    /// that would otherwise block forever returns and the thread notices.
    /// The knock is refused once the thread has already served every
    /// connection it was asked for and closed the listener.
    fn stop(self, go: mpsc::Sender<()>) {
        drop(go);
        drop(std::net::TcpStream::connect(self.addr));
        self.thread.join().expect("server thread");
    }
}

/// HTTP/1.1 server serving `requests` connections in turn. Each request is
/// read and handed over on the receiver at once, but the 200 reply is held
/// until one `()` arrives on the sender, so a test can pin the client's
/// worker mid-POST before doing anything else.
fn gated_mock_server(
    requests: usize,
) -> (
    String,
    mpsc::Sender<()>,
    mpsc::Receiver<Request>,
    GatedServer,
) {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
    let addr = listener.local_addr().expect("addr");
    let (tx, rx) = mpsc::channel();
    let (go_tx, go_rx) = mpsc::channel::<()>();
    let thread = thread::spawn(move || {
        for _ in 0..requests {
            let (mut stream, _) = listener.accept().expect("accept");
            let released = match go_rx.try_recv() {
                Ok(()) => true,
                Err(mpsc::TryRecvError::Empty) => false,
                Err(mpsc::TryRecvError::Disconnected) => return,
            };
            let request = read_request(&mut stream);
            if tx.send(request).is_err() || (!released && go_rx.recv().is_err()) {
                return;
            }
            // A cancelled POST has hung up by now; that write failing is expected.
            let _ = stream
                .write_all(b"HTTP/1.1 200 OK\r\nContent-Length: 2\r\nConnection: close\r\n\r\n{}");
        }
    });
    (
        format!("http://{addr}"),
        go_tx,
        rx,
        GatedServer { addr, thread },
    )
}

fn read_request(stream: &mut std::net::TcpStream) -> Request {
    {
        stream
            .set_read_timeout(Some(Duration::from_secs(5)))
            .expect("timeout");
        let mut buf = Vec::new();
        let mut chunk = [0_u8; 4096];
        let (head, body_start, content_length) = loop {
            let n = stream.read(&mut chunk).expect("read");
            assert!(n > 0, "client closed before headers");
            buf.extend_from_slice(&chunk[..n]);
            if let Some(pos) = buf.windows(4).position(|w| w == b"\r\n\r\n") {
                let head = String::from_utf8_lossy(&buf[..pos]).to_string();
                let len = head
                    .lines()
                    .find_map(|l| {
                        let (k, v) = l.split_once(':')?;
                        k.eq_ignore_ascii_case("content-length")
                            .then(|| v.trim().parse::<usize>().ok())?
                    })
                    .expect("content-length header");
                break (head, pos + 4, len);
            }
        };
        while buf.len() < body_start + content_length {
            let n = stream.read(&mut chunk).expect("read body");
            assert!(n > 0, "client closed mid-body");
            buf.extend_from_slice(&chunk[..n]);
        }
        let body: serde_json::Value =
            serde_json::from_slice(&buf[body_start..body_start + content_length]).expect("json");
        (head, body)
    }
}

#[test]
fn delivers_a_guarded_batch_to_the_batch_endpoint() {
    let (host, rx) = mock_server();
    let mut client = Client::new("phc_test".into(), &host, common()).expect("client");
    client.capture(
        CMD,
        "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc",
        vec![
            ("command".into(), Value::from("graph run")),
            ("command_path".into(), Value::from("/usr/bin/cerulion")),
            ("duration_ms".into(), Value::from(7_i64)),
        ],
    );
    let (head, body) = rx
        .recv_timeout(Duration::from_secs(5))
        .expect("request arrived");
    let request_line = head.lines().next().expect("request line");
    assert_eq!(request_line, "POST /batch HTTP/1.1");
    assert!(
        head.lines().any(|l| l
            .to_ascii_lowercase()
            .starts_with("content-type: application/json")),
        "{head}"
    );
    assert!(
        !head.to_ascii_lowercase().contains("content-encoding"),
        "gzip must be off: {head}"
    );
    assert_eq!(body["api_key"], "phc_test");
    let batch = body["batch"].as_array().expect("batch");
    assert_eq!(batch.len(), 1);
    let ev = &batch[0];
    assert_eq!(ev["event"], "cli_command_run");
    assert_eq!(ev["distinct_id"], "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc");
    assert_eq!(ev["uuid"].as_str().map(str::len), Some(36));
    assert_eq!(ev["uuid"].as_str().unwrap().as_bytes()[14], b'7', "uuid v7");
    let ts = ev["timestamp"].as_str().expect("timestamp");
    assert!(ts.ends_with('Z') && ts.len() == 24, "{ts}");
    let props = ev["properties"].as_object().expect("properties");
    assert_eq!(props["$lib"], "cerulion_telemetry");
    assert_eq!(props["surface"], "cli");
    assert_eq!(props["command"], "graph run");
    assert_eq!(props["duration_ms"], 7);
    assert!(
        !props.contains_key("command_path"),
        "path-valued property must be dropped"
    );
    assert_eq!(
        client.shutdown(Duration::from_secs(2)),
        ShutdownOutcome::Flushed
    );
    assert_eq!(client.post_failed(), 0);
    assert_eq!(
        client.shutdown(Duration::from_secs(2)),
        ShutdownOutcome::Noop
    );
}

#[test]
fn shutdown_returns_within_budget_when_the_server_never_answers() {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
    let host = format!("http://{}", listener.local_addr().expect("addr"));
    let mut client = Client::new("phc_test".into(), &host, common()).expect("client");
    client.capture(CMD, SUB, vec![("command".into(), Value::from("x"))]);
    let start = Instant::now();
    let outcome = client.shutdown(DEFAULT_SHUTDOWN_BUDGET);
    let elapsed = start.elapsed();
    assert_eq!(
        outcome,
        ShutdownOutcome::TimedOut { in_flight: 0 },
        "the hung POST was cancelled inside the budget"
    );
    assert_eq!(client.queue_dropped(), 1, "the cancelled batch is counted");
    assert_eq!(client.post_failed(), 0, "a cancelled POST is not a failure");
    assert!(
        elapsed >= DEFAULT_SHUTDOWN_BUDGET - DEFAULT_SHUTDOWN_BUDGET / 10
            && elapsed < Duration::from_millis(1500),
        "shutdown took {elapsed:?}"
    );
    drop(client);
    assert!(
        start.elapsed() < Duration::from_millis(1500),
        "drop must not re-wait"
    );
    drop(listener);
}

#[test]
fn timed_out_shutdown_abandons_the_queue_and_sends_nothing_more() {
    let (host, go, rx, server) = gated_mock_server(2);
    let mut client = Client::new("phc_test".into(), &host, common()).expect("client");
    client.capture(CMD, SUB, vec![("duration_ms".into(), Value::from(1_i64))]);
    let (_, first) = rx
        .recv_timeout(Duration::from_secs(5))
        .expect("first request");
    assert_eq!(first["batch"].as_array().map(Vec::len), Some(1));
    client.capture(CMD, SUB, vec![("duration_ms".into(), Value::from(2_i64))]);
    assert_eq!(
        client.shutdown(Duration::from_millis(100)),
        ShutdownOutcome::TimedOut { in_flight: 0 },
        "the pinned POST was cancelled, so nothing is in flight"
    );
    assert_eq!(
        client.queue_dropped(),
        2,
        "the cancelled batch and the queued event are both counted"
    );
    go.send(()).expect("release the in-flight request");
    assert!(
        rx.recv_timeout(Duration::from_millis(500)).is_err(),
        "nothing may be sent after a timed-out shutdown"
    );
    assert_eq!(client.post_failed(), 0);
    server.stop(go);
}

/// No POST may start after `shutdown` returns. Pin one POST, queue a second
/// batch, then release the first only after `shutdown` has returned: the
/// worker is live and would have drained the second batch next, but the
/// abort it acknowledged inside the budget means it never sends it.
#[test]
fn no_post_starts_after_a_timed_out_shutdown_returns() {
    for _ in 0..20 {
        let (host, go, rx, server) = gated_mock_server(2);
        let mut client = Client::new("phc_test".into(), &host, common()).expect("client");
        client.capture(CMD, SUB, vec![("duration_ms".into(), Value::from(1_i64))]);
        rx.recv_timeout(Duration::from_secs(5))
            .expect("worker pinned on the first POST");
        client.capture(CMD, SUB, vec![("duration_ms".into(), Value::from(2_i64))]);
        let budget = Duration::from_millis(60);
        let start = Instant::now();
        let outcome = client.shutdown(budget);
        assert!(
            start.elapsed() < budget + Duration::from_millis(200),
            "budget is hard: {:?}",
            start.elapsed()
        );
        assert_eq!(outcome, ShutdownOutcome::TimedOut { in_flight: 0 });
        assert_eq!(client.queue_dropped(), 2);
        go.send(())
            .expect("release the first POST after shutdown returned");
        assert!(
            rx.recv_timeout(Duration::from_millis(300)).is_err(),
            "the second batch must never reach the server"
        );
        drop(client);
        server.stop(go);
    }
}

#[test]
fn a_post_that_starts_during_shutdown_is_cancelled_at_the_deadline() {
    let (host, go, rx, server) = gated_mock_server(2);
    let mut client = Client::new("phc_test".into(), &host, common()).expect("client");
    client.capture(CMD, SUB, vec![("duration_ms".into(), Value::from(1_i64))]);
    rx.recv_timeout(Duration::from_secs(5))
        .expect("worker pinned on the first POST");
    client.capture(CMD, SUB, vec![("duration_ms".into(), Value::from(2_i64))]);

    let budget = Duration::from_millis(400);
    let start = Instant::now();
    let releaser = thread::spawn(move || {
        thread::sleep(Duration::from_millis(50));
        go.send(()).expect("release the first POST mid-shutdown");
        let (_, second) = rx
            .recv_timeout(Duration::from_secs(1))
            .expect("second POST reached the server");
        (start.elapsed(), second, go)
    });
    let outcome = client.shutdown(budget);
    let returned_at = start.elapsed();
    let (second_seen_at, second, go) = releaser.join().expect("releaser");
    assert!(
        second_seen_at < budget,
        "the second POST reached the server during the budget ({second_seen_at:?})"
    );
    assert_eq!(second["batch"][0]["properties"]["duration_ms"], 2);
    assert_eq!(
        outcome,
        ShutdownOutcome::TimedOut { in_flight: 0 },
        "the second POST was still pinned at the deadline and got cancelled"
    );
    assert!(
        returned_at < budget + Duration::from_millis(100),
        "shutdown returned at {returned_at:?}, not at HTTP_TIMEOUT"
    );
    assert_eq!(
        client.queue_dropped(),
        1,
        "the cancelled batch counts as dropped even though the server read it"
    );
    assert_eq!(client.post_failed(), 0, "cancelled, not failed");
    server.stop(go);
}

#[test]
fn queue_overflow_drops_oldest_and_counts() {
    let flood = 3 * QUEUE_CAPACITY as i64;
    let (host, go, rx, server) = gated_mock_server(2);
    let mut client = Client::new("phc_test".into(), &host, common()).expect("client");
    let ids = |body: &serde_json::Value| -> Vec<i64> {
        body["batch"]
            .as_array()
            .expect("batch")
            .iter()
            .map(|ev| ev["properties"]["duration_ms"].as_i64().expect("i64"))
            .collect()
    };

    client.capture(CMD, SUB, vec![("duration_ms".into(), Value::from(-1_i64))]);
    let (_, first) = rx
        .recv_timeout(Duration::from_secs(5))
        .expect("first request");
    assert_eq!(ids(&first), vec![-1], "worker is now pinned mid-POST");

    for i in 0..flood {
        client.capture(CMD, SUB, vec![("duration_ms".into(), Value::from(i))]);
    }
    assert_eq!(client.queue_dropped(), 2 * QUEUE_CAPACITY as u64);

    go.send(()).expect("release first");
    let (_, second) = rx
        .recv_timeout(Duration::from_secs(5))
        .expect("second request");
    let want: Vec<i64> = ((flood - QUEUE_CAPACITY as i64)..flood).collect();
    assert_eq!(ids(&second), want, "oldest events are the ones dropped");
    go.send(()).expect("release second");

    assert_eq!(
        client.shutdown(Duration::from_secs(2)),
        ShutdownOutcome::Flushed
    );
    assert_eq!(client.post_failed(), 0);
    server.stop(go);
}

#[test]
fn common_metadata_that_fails_the_guard_disables_the_client() {
    let bad = [
        Common {
            app_version: "0.1.0 (/home/bob/src)".into(),
            ..common()
        },
        Common {
            surface: "https://evil.example".into(),
            ..common()
        },
        Common {
            env: "bob@example.com".into(),
            ..common()
        },
        Common {
            channel: Some("x".repeat(129)),
            ..common()
        },
    ];
    for common in bad {
        assert!(
            Client::new("phc_test".into(), "http://127.0.0.1:9", common.clone()).is_none(),
            "{common:?} must not build a client"
        );
    }
    let ok = Common {
        channel: Some("nightly".into()),
        ..common()
    };
    let mut client = Client::new("phc_test".into(), "http://127.0.0.1:9", ok).expect("client");
    assert_eq!(
        client.shutdown(Duration::from_secs(2)),
        ShutdownOutcome::Flushed
    );
}

#[test]
fn from_env_is_none_without_key_or_consent() {
    let _lock = ENV.lock().unwrap_or_else(|p| p.into_inner());
    let dir = tempfile::tempdir().expect("tempdir");
    std::env::set_var("CERULION_HOME", dir.path().join("h"));
    std::env::remove_var("DO_NOT_TRACK");
    std::env::remove_var("CERULION_TELEMETRY");

    std::env::remove_var("POSTHOG_API_KEY");
    assert!(Client::from_env(common()).is_none(), "no key -> None");
    std::env::set_var("POSTHOG_API_KEY", "   ");
    assert!(Client::from_env(common()).is_none(), "blank key -> None");

    std::env::set_var("POSTHOG_API_KEY", "phc_test");
    std::env::set_var("DO_NOT_TRACK", "1");
    assert!(Client::from_env(common()).is_none(), "DO_NOT_TRACK -> None");
    std::env::remove_var("DO_NOT_TRACK");
    std::env::set_var("CERULION_TELEMETRY", "0");
    assert!(Client::from_env(common()).is_none(), "opt-out -> None");
    std::env::remove_var("CERULION_TELEMETRY");
    assert!(
        !dir.path().join("h").exists(),
        "from_env never writes the file when it answers None"
    );

    let (host, rx) = mock_server();
    std::env::set_var("POSTHOG_HOST", format!("{host}/"));
    let mut client = Client::from_env(common()).expect("key + default consent -> Some");
    assert!(
        !dir.path().join("h").exists(),
        "from_env never writes the file when it answers Some either"
    );
    client.alias(SUB, ANON);
    let (head, body) = rx.recv_timeout(Duration::from_secs(5)).expect("request");
    assert!(
        head.starts_with("POST /batch HTTP/1.1"),
        "trailing slash trimmed: {head}"
    );
    assert_eq!(body["batch"][0]["event"], "$create_alias");
    assert_eq!(body["batch"][0]["properties"]["alias"], ANON);
    assert_eq!(
        client.shutdown(Duration::from_secs(2)),
        ShutdownOutcome::Flushed
    );
    std::env::remove_var("POSTHOG_HOST");
    std::env::remove_var("POSTHOG_API_KEY");
}

#[test]
fn set_once_with_only_disallowed_keys_sends_nothing() {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
    let host = format!("http://{}", listener.local_addr().expect("addr"));
    let mut client = Client::new("phc_test".into(), &host, common()).expect("client");
    client.set_once(SUB, vec![("email".into(), Value::from("bob"))]);
    client.capture(CMD, "", vec![]);
    assert_eq!(
        client.shutdown(Duration::from_millis(200)),
        ShutdownOutcome::Flushed,
        "nothing queued, so the worker exits at once"
    );
}

#[test]
fn a_host_that_is_not_https_or_loopback_disables_the_client() {
    for host in [
        "http://example.com",
        "ftp://127.0.0.1",
        "not a url",
        "http://10.0.0.1:8000",
    ] {
        assert!(
            Client::new("k".into(), host, common()).is_none(),
            "{host} must be refused"
        );
    }
    for host in [
        "https://example.com",
        "http://127.0.0.1:1",
        "http://localhost:1",
        "http://[::1]:1",
    ] {
        let mut client = Client::new("k".into(), host, common()).expect(host);
        assert_eq!(
            client.shutdown(DEFAULT_SHUTDOWN_BUDGET),
            ShutdownOutcome::Flushed
        );
    }
}

#[test]
fn an_unrepresentable_budget_is_clamped_instead_of_panicking() {
    let (host, _rx) = mock_server();
    let mut client = Client::new("k".into(), &host, common()).expect("client");
    assert_eq!(client.shutdown(Duration::MAX), ShutdownOutcome::Flushed);
    assert_eq!(client.shutdown(Duration::MAX), ShutdownOutcome::Noop);
}
