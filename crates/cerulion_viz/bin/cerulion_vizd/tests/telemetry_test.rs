// SPDX-License-Identifier: AGPL-3.0-only
//! The vizd telemetry module: event specs, properties and the heartbeat
//! thread's cadence and prompt stop without a network, then the start,
//! heartbeat, live opt-out, abandon and shutdown lifecycle against a loopback
//! collector. The lifecycle tests set `CERULION_HOME` for the process, so they
//! serialize on one lock and the suite runs with `--test-threads=1`.

use std::collections::VecDeque;
use std::io::{Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant, SystemTime};

use cerulion_telemetry::{consent, guard, rfc3339, Client, Value, DEFAULT_SHUTDOWN_BUDGET};
/// Slack over the shutdown budget for scheduler jitter on a loaded runner.
const STOP_BOUND: Duration = Duration::from_millis(DEFAULT_SHUTDOWN_BUDGET.as_millis() as u64 * 3);

use cerulion_vizd::telemetry::{
    common, heartbeat_props, started_props, Heartbeat, Starting, Telemetry, HEARTBEAT_INTERVAL,
    VIZD_HEARTBEAT, VIZD_STARTED,
};

/// Serializes the tests that set process environment.
static ENV: Mutex<()> = Mutex::new(());

#[test]
fn heartbeat_interval_is_fifteen_minutes() {
    assert_eq!(HEARTBEAT_INTERVAL, Duration::from_secs(900));
}

#[test]
fn event_names_and_common_values_pass_the_guard() {
    assert_eq!(VIZD_STARTED.name, "vizd_started");
    assert_eq!(VIZD_HEARTBEAT.name, "vizd_heartbeat");
    for spec in [VIZD_STARTED, VIZD_HEARTBEAT] {
        guard::check_event_name(spec.name).expect(spec.name);
    }
    let c = common();
    assert_eq!(c.surface, "vizd");
    for value in [&c.surface, &c.env, &c.app_version] {
        guard::check_str(value).expect(value);
    }
}

#[test]
fn started_carries_the_platform() {
    assert_eq!(
        started_props(),
        vec![
            ("os".to_string(), Value::Str(std::env::consts::OS.into())),
            (
                "arch".to_string(),
                Value::Str(std::env::consts::ARCH.into())
            ),
        ]
    );
}

#[test]
fn properties_stay_inside_their_allowlists_and_pass_the_guard() {
    for (spec, props) in [
        (VIZD_STARTED, started_props()),
        (VIZD_HEARTBEAT, heartbeat_props(3 * HEARTBEAT_INTERVAL)),
    ] {
        let (kept, dropped) = guard::filter(props.clone(), spec.allowlist);
        assert!(dropped.is_empty(), "{}: {dropped:?}", spec.name);
        assert_eq!(kept, props, "{}", spec.name);
    }
}

#[test]
fn uptime_is_whole_minutes_of_elapsed_time() {
    let props = heartbeat_props(4 * HEARTBEAT_INTERVAL);
    assert_eq!(props, vec![("uptime_minutes".to_string(), Value::Int(60))]);
    assert_eq!(
        heartbeat_props(Duration::from_secs(179))[0].1,
        Value::Int(2)
    );
    assert_eq!(
        heartbeat_props(Duration::from_secs(180))[0].1,
        Value::Int(3)
    );
    assert_eq!(
        heartbeat_props(Duration::MAX)[0].1,
        Value::Int((u64::MAX / 60) as i64)
    );
}

#[test]
fn heartbeat_ticks_in_order_and_stops_promptly_on_drop() {
    let (tx, rx) = mpsc::channel();
    let beat = Heartbeat::spawn(Duration::from_millis(20), move |n| {
        let _ = tx.send(n);
    })
    .expect("spawn");
    let seen: Vec<u64> = (0..3)
        .map(|_| rx.recv_timeout(Duration::from_secs(5)).expect("tick"))
        .collect();
    assert_eq!(seen, vec![1, 2, 3]);
    drop(beat);
    while rx.try_recv().is_ok() {}
    assert!(
        rx.recv_timeout(Duration::from_millis(100)).is_err(),
        "no tick after drop"
    );
}

#[test]
fn dropping_a_long_interval_heartbeat_does_not_wait_for_the_interval() {
    let beat = Heartbeat::spawn(Duration::from_secs(3600), |_| {}).expect("spawn");
    let start = Instant::now();
    drop(beat);
    assert!(
        start.elapsed() < STOP_BOUND,
        "drop waited {:?}",
        start.elapsed()
    );
}

#[test]
fn dropping_a_heartbeat_stuck_in_a_tick_is_bounded() {
    let (entered_tx, entered) = mpsc::channel();
    let beat = Heartbeat::spawn(Duration::from_millis(10), move |_| {
        let _ = entered_tx.send(());
        std::thread::sleep(Duration::from_secs(5));
    })
    .expect("spawn");
    entered
        .recv_timeout(Duration::from_secs(5))
        .expect("tick entered");
    let start = Instant::now();
    drop(beat);
    assert!(
        start.elapsed() < STOP_BOUND,
        "drop waited {:?}",
        start.elapsed()
    );
}

/// A fresh consent home for one test: `CERULION_HOME` points at an empty
/// directory under the system temp dir and the environment opt-outs are
/// cleared, so consent resolves enabled and the anonymous id is minted
/// there. The variable is left set: a starter thread that outlives its test
/// must never fall back to the real home.
fn isolated_home(tag: &str) -> PathBuf {
    let home = std::env::temp_dir().join(format!(
        "cerulion-vizd-telemetry-{}-{tag}",
        std::process::id()
    ));
    let _ = std::fs::remove_dir_all(&home);
    std::env::set_var("CERULION_HOME", &home);
    std::env::remove_var("DO_NOT_TRACK");
    std::env::remove_var("CERULION_TELEMETRY");
    home
}

fn loopback_client(host: &str) -> Client {
    Client::new("phc_test".into(), host, common()).expect("loopback client")
}

/// A loopback `/batch` collector: every POST is answered 200 and its events
/// are read back in arrival order, batch by batch.
struct Collector {
    host: String,
    addr: SocketAddr,
    batches: mpsc::Receiver<serde_json::Value>,
    pending: VecDeque<serde_json::Value>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl Collector {
    fn start() -> Collector {
        let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
        let addr = listener.local_addr().expect("addr");
        let (tx, batches) = mpsc::channel();
        let stop = Arc::new(AtomicBool::new(false));
        let stopping = Arc::clone(&stop);
        let thread = thread::spawn(move || {
            while !stopping.load(Ordering::Acquire) {
                let Ok((mut stream, _)) = listener.accept() else {
                    break;
                };
                let Some(body) = read_body(&mut stream) else {
                    continue;
                };
                let _ = stream.write_all(
                    b"HTTP/1.1 200 OK\r\nContent-Length: 2\r\nConnection: close\r\n\r\n{}",
                );
                if tx.send(body).is_err() {
                    break;
                }
            }
        });
        Collector {
            host: format!("http://{addr}"),
            addr,
            batches,
            pending: VecDeque::new(),
            stop,
            thread: Some(thread),
        }
    }

    fn push_batch(&mut self, batch: &serde_json::Value) {
        let events = batch["batch"].as_array().expect("batch array");
        self.pending.extend(events.iter().cloned());
    }

    /// The next event in arrival order, waiting up to `wait` for a batch.
    fn next(&mut self, wait: Duration) -> serde_json::Value {
        if self.pending.is_empty() {
            let batch = self
                .batches
                .recv_timeout(wait)
                .unwrap_or_else(|_| panic!("no event within {wait:?}"));
            self.push_batch(&batch);
        }
        self.pending.pop_front().expect("a batch carries an event")
    }

    /// Every event that arrives until `quiet` passes with no new batch, or
    /// `at_most` has elapsed, in arrival order.
    fn settled(&mut self, quiet: Duration, at_most: Duration) -> Vec<serde_json::Value> {
        let deadline = Instant::now() + at_most;
        loop {
            let wait = quiet.min(deadline.saturating_duration_since(Instant::now()));
            let Ok(batch) = self.batches.recv_timeout(wait) else {
                break;
            };
            self.push_batch(&batch);
        }
        self.pending.drain(..).collect()
    }
}

impl Drop for Collector {
    /// Raise the stop flag, then knock on the listener so a blocked `accept`
    /// returns and the thread sees it.
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        drop(TcpStream::connect(self.addr));
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

/// One HTTP/1.1 request body as JSON; `None` for a connection that closes
/// before a complete request (the stop knock).
fn read_body(stream: &mut TcpStream) -> Option<serde_json::Value> {
    stream
        .set_read_timeout(Some(Duration::from_secs(5)))
        .expect("timeout");
    let mut buf = Vec::new();
    let mut chunk = [0_u8; 4096];
    let (body_start, content_length) = loop {
        let n = stream.read(&mut chunk).ok()?;
        if n == 0 {
            return None;
        }
        buf.extend_from_slice(&chunk[..n]);
        if let Some(pos) = buf.windows(4).position(|w| w == b"\r\n\r\n") {
            let head = String::from_utf8_lossy(&buf[..pos]);
            let len = head.lines().find_map(|l| {
                let (k, v) = l.split_once(':')?;
                k.eq_ignore_ascii_case("content-length")
                    .then(|| v.trim().parse::<usize>().ok())?
            })?;
            break (pos + 4, len);
        }
    };
    while buf.len() < body_start + content_length {
        let n = stream.read(&mut chunk).ok()?;
        if n == 0 {
            return None;
        }
        buf.extend_from_slice(&chunk[..n]);
    }
    serde_json::from_slice(&buf[body_start..body_start + content_length]).ok()
}

#[test]
fn start_sends_started_then_heartbeats_until_a_live_opt_out_and_stops_within_budget() {
    let _env = ENV.lock().unwrap_or_else(|p| p.into_inner());
    let home = isolated_home("lifecycle");
    let mut collector = Collector::start();
    let telemetry =
        Telemetry::start_with(loopback_client(&collector.host), Duration::from_millis(20))
            .expect("start");

    // The started event is the first thing on the wire, then the beats.
    let started = collector.next(Duration::from_secs(5));
    assert_eq!(started["event"], "vizd_started");
    let anon_id = started["distinct_id"].as_str().expect("distinct_id");
    assert!(anon_id.starts_with("anon:"), "{anon_id}");
    assert_eq!(started["properties"]["os"], std::env::consts::OS);
    assert_eq!(started["properties"]["arch"], std::env::consts::ARCH);
    assert_eq!(started["properties"]["surface"], "vizd");
    assert_eq!(started["properties"]["$process_person_profile"], false);

    let beat = collector.next(Duration::from_secs(5));
    assert_eq!(beat["event"], "vizd_heartbeat");
    assert_eq!(beat["distinct_id"], anon_id, "one id for the whole run");
    assert_eq!(beat["properties"]["uptime_minutes"], 0);

    consent::set_enabled(false).expect("opt out");
    // Every event is stamped as it is queued, under the consent lock the
    // opt-out wrote under, so a beat that lands after the opt-out returned
    // is legitimate only if it was stamped before. Collecting until a full
    // quiet second has passed gives a late beat fifty chances to arrive.
    let cutoff = rfc3339::format(SystemTime::now());
    let after = collector.settled(Duration::from_secs(1), Duration::from_secs(5));
    assert!(
        after.iter().all(|e| e["event"] == "vizd_heartbeat"),
        "only heartbeats follow the start: {after:?}"
    );
    let late: Vec<&str> = after
        .iter()
        .filter_map(|e| e["timestamp"].as_str())
        .filter(|stamp| *stamp > cutoff.as_str())
        .collect();
    assert!(
        late.is_empty(),
        "heartbeats stamped after the opt-out: {late:?}"
    );

    let start = Instant::now();
    telemetry.shutdown();
    assert!(
        start.elapsed() < STOP_BOUND,
        "shutdown waited {:?}",
        start.elapsed()
    );
    drop(collector);
    let _ = std::fs::remove_dir_all(home);
}

#[test]
fn a_start_still_blocked_at_the_deadline_is_abandoned_and_sends_nothing() {
    let _env = ENV.lock().unwrap_or_else(|p| p.into_inner());
    let home = isolated_home("abandon");
    let mut collector = Collector::start();
    let host = collector.host.clone();
    let (release, blocked) = mpsc::channel::<()>();
    let starting = Starting::spawn_with(
        move || {
            blocked.recv().ok()?;
            Some(loopback_client(&host))
        },
        Duration::from_millis(20),
    );

    let start = Instant::now();
    starting.shutdown();
    assert!(
        start.elapsed() < STOP_BOUND,
        "shutdown waited {:?}",
        start.elapsed()
    );

    // The start finishes only now, with a client that would send: it finds
    // the abandon flag set and queues nothing.
    release.send(()).expect("starter still waiting");
    let sent = collector.settled(Duration::from_secs(1), Duration::from_secs(5));
    assert!(sent.is_empty(), "events from an abandoned start: {sent:?}");
    drop(collector);
    let _ = std::fs::remove_dir_all(home);
}

#[test]
fn a_start_blocked_on_the_consent_lock_at_the_deadline_is_abandoned_and_sends_nothing() {
    let _env = ENV.lock().unwrap_or_else(|p| p.into_inner());
    let home = isolated_home("abandon-locked");
    // Mint the consent file first, so the lock file exists to be held.
    consent::anon_id().expect("mint the consent file");
    let mut collector = Collector::start();
    let host = collector.host.clone();
    // Hold the consent lock as another process would: the starter has its
    // client, then blocks on this lock for the anonymous id.
    let held = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .open(home.join("telemetry.json.lock"))
        .expect("open the consent lock file");
    held.lock().expect("hold the consent lock");
    let (client_built, built) = mpsc::channel::<()>();
    let starting = Starting::spawn_with(
        move || {
            let client = loopback_client(&host);
            client_built.send(()).expect("test waits for the client");
            Some(client)
        },
        Duration::from_millis(20),
    );
    built.recv().expect("starter built its client");

    let start = Instant::now();
    starting.shutdown();
    assert!(
        start.elapsed() < STOP_BOUND,
        "shutdown waited {:?} on a start blocked by the consent lock",
        start.elapsed()
    );

    // The lock is released only now, with a client that would send: the
    // start finds itself abandoned and queues nothing.
    drop(held);
    let sent = collector.settled(Duration::from_secs(1), Duration::from_secs(5));
    assert!(sent.is_empty(), "events from an abandoned start: {sent:?}");
    drop(collector);
    let _ = std::fs::remove_dir_all(home);
}
