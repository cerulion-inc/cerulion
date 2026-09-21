// SPDX-License-Identifier: AGPL-3.0-only
//! The desk half of the event-driven sidebar, end to end over the REAL vizd
//! UDS control server — a controller subscribes, an upstream catalog change arrives,
//! and vizd writes it down the connection UNPROMPTED. Nothing polls.
//!
//! Driven with a SCRIPTED `CatalogEventSource` (a dependency-injection double for the
//! trait — Principle #13, not fabricated data: the values it yields are the exact
//! `CatalogChangedEvent`s the production `cerulion-netd` relay produces, and the netd
//! half of the chain is pinned in `cerulion_netd/tests/catalog_events_e2e_test.rs` +
//! `catalog_events_live_test.rs`). Hermetic: no netd is spawned, no zenoh session is
//! opened, and each test drives its own daemon over a unique temp socket + an isolated
//! per-test SHM root ⇒ parallel-safe, no `#[serial]`.
//!
//! Every assertion is against a HAND-WRITTEN oracle — the exact number of unsolicited
//! LINES, the exact robot names — never a self-compare.

use std::io::{self, BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::clock::VirtualClock;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_viz::schema_registry::builtin_walker;
use cerulion_viz::sink::SinkState;
use cerulion_viz::worker::VizLogWorker;

use cerulion_vizd::daemon::{start_with_planes, DemandPlane, RunningDaemon};
use cerulion_vizd::events::{
    CatalogEventPoll, CatalogEventSource, CatalogEventStream, CatalogSnapshot, SubscribeFailure,
};
use cerulion_vizd::protocol::{
    CatalogChangedEvent, Hello, Request, CATALOG_CHANGED_EVENT, PROTOCOL_VERSION,
};

// --------------------------------------------------------------------------
// Harness.
// --------------------------------------------------------------------------

/// A demand plane that refuses everything — this file exercises the EVENT plane, and
/// an inert demand plane keeps netd out of every oracle (and keeps the test from
/// touching a live desk daemon).
struct NoNetdPlane;

impl DemandPlane for NoNetdPlane {
    fn demand(&self, _robot: &str, _topic: &str, _schema_hash: u64) -> Result<(), String> {
        Err("hermetic test: no netd".to_string())
    }
    fn release(&self, _robot: &str, _topic: &str) -> Result<(), String> {
        Ok(())
    }
    fn query_catalog(&self, _robot: &str) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        Ok(Vec::new())
    }
    fn query_catalog_all(&self) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        Ok(Vec::new())
    }
    fn query_schema(
        &self,
        _robot: &str,
        _requested: &str,
    ) -> Result<Vec<cerulion_core::SchemaReply>, String> {
        Ok(Vec::new())
    }
}

/// A scripted upstream: the test pushes `CatalogChangedEvent`s into a queue and the
/// forwarder thread relays them, exactly as it relays a real netd subscription. The
/// `snapshot` is what the subscription hands back BEFORE its first event — the
/// upstream state a restarted vizd adopts, and the `watching` flag it must report
/// unchanged.
struct ScriptedSource {
    queue: Arc<Mutex<std::collections::VecDeque<CatalogChangedEvent>>>,
    snapshot: Mutex<CatalogSnapshot>,
    subscribes: AtomicUsize,
    /// When set, `subscribe` refuses PERMANENTLY (the too-old-netd arm).
    unsupported: AtomicBool,
}

impl Default for ScriptedSource {
    fn default() -> Self {
        Self {
            queue: Arc::default(),
            // A watching upstream with an empty catalog — the ordinary case.
            snapshot: Mutex::new(CatalogSnapshot {
                watching: true,
                ..Default::default()
            }),
            subscribes: AtomicUsize::new(0),
            unsupported: AtomicBool::new(false),
        }
    }
}

impl ScriptedSource {
    fn push(&self, event: CatalogChangedEvent) {
        self.queue.lock().unwrap().push_back(event);
    }
    fn subscribes(&self) -> usize {
        self.subscribes.load(Ordering::SeqCst)
    }
    fn set_snapshot(&self, snapshot: CatalogSnapshot) {
        *self.snapshot.lock().unwrap() = snapshot;
    }
}

struct ScriptedStream {
    queue: Arc<Mutex<std::collections::VecDeque<CatalogChangedEvent>>>,
}

impl CatalogEventStream for ScriptedStream {
    fn poll(&mut self) -> CatalogEventPoll {
        // Block briefly for an event, mirroring a real subscription's bounded poll.
        let deadline = Instant::now() + Duration::from_millis(100);
        loop {
            if let Some(e) = self.queue.lock().unwrap().pop_front() {
                return CatalogEventPoll::Changed(Box::new(e));
            }
            if Instant::now() >= deadline {
                return CatalogEventPoll::Idle;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
    }
}

impl CatalogEventSource for ScriptedSource {
    fn subscribe(
        &self,
    ) -> Result<(CatalogSnapshot, Box<dyn CatalogEventStream>), SubscribeFailure> {
        self.subscribes.fetch_add(1, Ordering::SeqCst);
        if self.unsupported.load(Ordering::SeqCst) {
            return Err(SubscribeFailure::Unsupported(
                "scripted: this daemon is too old".to_string(),
            ));
        }
        Ok((
            self.snapshot.lock().unwrap().clone(),
            Box::new(ScriptedStream {
                queue: Arc::clone(&self.queue),
            }),
        ))
    }
}

fn change(
    version: u64,
    robots: &[&str],
    added: &[&str],
    removed: &[&str],
    topics: u64,
) -> CatalogChangedEvent {
    CatalogChangedEvent {
        event: CATALOG_CHANGED_EVENT.to_string(),
        version,
        robots: robots.iter().map(|s| s.to_string()).collect(),
        robots_added: added.iter().map(|s| s.to_string()).collect(),
        robots_removed: removed.iter().map(|s| s.to_string()).collect(),
        topics_added: topics,
        topics_removed: 0,
    }
}

fn temp_socket(tag: &str) -> (PathBuf, PathBuf) {
    use std::sync::atomic::AtomicU64;
    static N: AtomicU64 = AtomicU64::new(0);
    let n = N.fetch_add(1, Ordering::Relaxed);
    let dir = std::env::temp_dir().join(format!("vizd_{tag}_{}_{n}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("mk tempdir");
    (dir.join("vizd.sock"), dir)
}

fn isolated_manager(node: &str) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: node.to_string(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 16,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init isolated transport")
}

struct Harness {
    daemon: RunningDaemon,
    socket: PathBuf,
    dir: PathBuf,
    source: Arc<ScriptedSource>,
}

impl Drop for Harness {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

fn start(tag: &str) -> Harness {
    start_with(tag, false)
}

fn start_with(tag: &str, unsupported: bool) -> Harness {
    start_configured(tag, unsupported, None)
}

/// [`start_with`] plus an explicit upstream SNAPSHOT — the state netd hands back at
/// subscribe time. `None` uses the default (watching, empty catalog).
fn start_configured(tag: &str, unsupported: bool, snapshot: Option<CatalogSnapshot>) -> Harness {
    let (socket, dir) = temp_socket(tag);
    let manager = isolated_manager(tag);
    let (rec, _storage) = rerun::RecordingStreamBuilder::new("vizd")
        .recording_id(tag.to_string())
        .memory()
        .expect("memory sink");
    let worker = VizLogWorker::spawn(rec, builtin_walker(), SinkState::new()).expect("worker");
    let source = Arc::new(ScriptedSource::default());
    source.unsupported.store(unsupported, Ordering::SeqCst);
    if let Some(snapshot) = snapshot {
        source.set_snapshot(snapshot);
    }
    let daemon = start_with_planes(
        socket.clone(),
        Duration::from_millis(20),
        manager,
        worker,
        builtin_walker(),
        None,
        Arc::new(NoNetdPlane),
        Arc::clone(&source) as Arc<dyn CatalogEventSource>,
    )
    .expect("daemon start");
    Harness {
        daemon,
        socket,
        dir,
        source,
    }
}

/// Budget for reading the daemon's hello BANNER. Deliberately generous
/// and independent of the caller's per-session `read_timeout`: waiting for a
/// handshake is not the thing any test in this file is measuring, so a slow or
/// loaded runner must never turn it into a failure.
const BANNER_TIMEOUT: Duration = Duration::from_secs(10);

struct Controller {
    stream: UnixStream,
    reader: BufReader<UnixStream>,
}

impl Controller {
    fn connect(sock: &Path) -> (Controller, Hello) {
        Self::connect_with_timeout(sock, Duration::from_secs(3))
    }

    fn connect_with_timeout(sock: &Path, read_timeout: Duration) -> (Controller, Hello) {
        let stream = UnixStream::connect(sock).expect("connect");
        // The BANNER is read under a GENEROUS timeout, and the caller's
        // budget is applied only AFTERWARDS.
        //
        // `read_timeout` is small on purpose at six call sites (150 ms) — those
        // tests drain for "expect NOTHING" and a short budget is what keeps them
        // fast. But the banner is an ordinary HANDSHAKE: the daemon has to be
        // scheduled and write it. Applying the short budget to it made the
        // handshake, not the assertion, the thing being timed — and on a loaded
        // runner it loses: `a_quiet_upstream_pushes_nothing` and
        // `the_pre_928_entry_point_pushes_nothing_and_says_so` both die on
        // `banner: Os { code: 35, kind: WouldBlock }` under
        // load. The failure is NOT the property under test; a slow runner
        // could only ever make these tests MORE likely to see "nothing".
        //
        // `try_clone` dups the fd, so both handles share one open file
        // description and `SO_RCVTIMEO` set on either applies to both — which is
        // why setting it on `stream` governs reads made through `reader`.
        stream.set_read_timeout(Some(BANNER_TIMEOUT)).unwrap();
        stream
            .set_write_timeout(Some(Duration::from_secs(3)))
            .unwrap();
        let reader = BufReader::new(stream.try_clone().expect("clone"));
        let mut c = Controller { stream, reader };
        let hello: Hello =
            serde_json::from_str(c.read_line().expect("banner").trim()).expect("banner parses");
        c.stream.set_read_timeout(Some(read_timeout)).unwrap();
        (c, hello)
    }

    fn read_line(&mut self) -> io::Result<String> {
        let mut line = String::new();
        let n = self.reader.read_line(&mut line)?;
        if n == 0 {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "server closed",
            ));
        }
        Ok(line)
    }

    fn send(&mut self, req: &Request) {
        writeln!(self.stream, "{}", req.to_json_line()).expect("send");
        self.stream.flush().expect("flush");
    }

    /// Send `subscribe_events` and return the parsed snapshot, tolerating an event
    /// that legitimately lands first.
    fn subscribe(&mut self, id: u64) -> serde_json::Value {
        self.send(&Request::SubscribeEvents { id });
        let deadline = Instant::now() + Duration::from_secs(3);
        while Instant::now() < deadline {
            let Ok(line) = self.read_line() else { continue };
            let v: serde_json::Value = match serde_json::from_str(line.trim()) {
                Ok(v) => v,
                Err(_) => continue,
            };
            if v.get("event").is_some() {
                continue; // an event; keep looking for the response
            }
            if v.get("id").and_then(|i| i.as_u64()) == Some(id) {
                return v;
            }
        }
        panic!("no subscribe_events response arrived");
    }

    /// Drain every unsolicited event line that arrives within `within`.
    fn drain_events(&mut self, within: Duration) -> Vec<CatalogChangedEvent> {
        let deadline = Instant::now() + within;
        let mut out = Vec::new();
        while Instant::now() < deadline {
            match self.read_line() {
                Ok(line) => {
                    let v: serde_json::Value = match serde_json::from_str(line.trim()) {
                        Ok(v) => v,
                        Err(_) => continue,
                    };
                    if v.get("event").and_then(|e| e.as_str()) == Some(CATALOG_CHANGED_EVENT) {
                        out.push(
                            serde_json::from_str::<CatalogChangedEvent>(line.trim())
                                .expect("event parses"),
                        );
                    }
                }
                Err(e)
                    if matches!(
                        e.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                    ) =>
                {
                    continue
                }
                Err(_) => break,
            }
        }
        out
    }

    /// The FIRST unsolicited event to arrive within `within`, or `None`.
    ///
    /// Returns on the event itself rather than collecting a fixed 200 ms slice
    /// and then looking. The old shape cost every POSITIVE wait in this file the
    /// remainder of its slice — an event delivered 5 ms in was still reported
    /// 200 ms in — which on ten-odd call sites is most of this binary's idle
    /// wall. The oracle is unchanged: "an event arrives within `within`", with
    /// the same non-event lines skipped and the same `WouldBlock`/`TimedOut`
    /// handling as `drain_events` (a drain that needs the WHOLE window is the
    /// negative case, and that one keeps it).
    fn next_event(&mut self, within: Duration) -> Option<CatalogChangedEvent> {
        let deadline = Instant::now() + within;
        while Instant::now() < deadline {
            match self.read_line() {
                Ok(line) => {
                    let v: serde_json::Value = match serde_json::from_str(line.trim()) {
                        Ok(v) => v,
                        Err(_) => continue,
                    };
                    if v.get("event").and_then(|e| e.as_str()) == Some(CATALOG_CHANGED_EVENT) {
                        return Some(
                            serde_json::from_str::<CatalogChangedEvent>(line.trim())
                                .expect("event parses"),
                        );
                    }
                }
                Err(e)
                    if matches!(
                        e.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                    ) =>
                {
                    continue
                }
                Err(_) => break,
            }
        }
        None
    }
}

/// The short read timeout the "expect NOTHING" arms connect with (see
/// `connect_with_timeout`'s note on why the banner does NOT use it). It paces the
/// CLIENT's own retry loop; it is not, and must not be used as, a bound on how long
/// the daemon might take to deliver something.
const PLAIN_READ_TIMEOUT: Duration = Duration::from_millis(150);

/// Drive ONE request round-trip on `c` and return every unsolicited
/// catalog-change line seen on the way to the response.
///
/// This is the absence oracle for an unsubscribed controller, and it rests on two
/// facts in the daemon rather than on a timing window:
///
///  * `EventHub::publish` does **no I/O**. In ONE critical section it sets
///    `slot.pending` for every SUBSCRIBER, and `take_pending` takes the same lock
///    (`src/events.rs`). So once any subscriber's handler has taken its event, the
///    publish pass is over — and under a regression that fanned out to every
///    CONNECTION, this connection's slot is already set.
///  * The handler writes a pending event at the **TOP** of its loop, BEFORE the
///    blocking read (`src/daemon.rs`). So a slot that is set while the handler
///    serves request N is flushed no later than the top of the iteration that
///    serves request N+1 — i.e. it lands BEFORE that request's response, on the
///    same socket, in order.
///
/// Hence TWO round-trips bound the absence by ORDERING: one is not enough, because
/// a handler already blocked in `read` when the publish happened serves the request
/// first and only reaches the flush on its NEXT trip round the loop.
///
/// What this deliberately does NOT claim — because it would be wrong
/// — is that a sibling's RECEIPT proves anything about this
/// connection's writes. Each connection flushes on its own thread, paced by its own
/// `CONN_READ_TIMEOUT` (200 ms), so a fixed client-side window shorter than that
/// pacing can miss a stray line entirely.
fn round_trip_collecting_events(c: &mut Controller, id: u64) -> Vec<CatalogChangedEvent> {
    c.send(&Request::List { id });
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut events = Vec::new();
    while Instant::now() < deadline {
        let line = match c.read_line() {
            Ok(line) => line,
            Err(_) => continue, // read timeout: the response has not arrived yet
        };
        let v: serde_json::Value = match serde_json::from_str(line.trim()) {
            Ok(v) => v,
            Err(_) => continue,
        };
        if v.get("event").and_then(|e| e.as_str()) == Some(CATALOG_CHANGED_EVENT) {
            events.push(
                serde_json::from_str::<CatalogChangedEvent>(line.trim()).expect("event parses"),
            );
            continue;
        }
        if v.get("id").and_then(|i| i.as_u64()) == Some(id) {
            // The connection is still a working request/response stream — a desync
            // would show up here rather than on the absence check.
            assert_eq!(v["ok"], serde_json::json!(true));
            return events;
        }
    }
    panic!("the list response for id {id} must arrive and correlate");
}

fn wait_until(mut f: impl FnMut() -> bool, timeout: Duration) -> bool {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if f() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    f()
}

// --------------------------------------------------------------------------
// Tests.
// --------------------------------------------------------------------------

/// THE headline: an upstream catalog change reaches a subscribed controller
/// unprompted. This is the bug observed at the desk seam — the sidebar kept
/// the old view because nothing told it.
#[test]
fn a_catalog_change_reaches_a_subscribed_controller_unprompted() {
    let h = start("push");
    let (mut c, hello) = Controller::connect(&h.socket);
    assert_eq!(
        hello.protocol, PROTOCOL_VERSION,
        "the banner version is UNCHANGED — Studio compares it for exact equality"
    );
    assert_eq!(PROTOCOL_VERSION, 1, "and it must stay 1");

    let snapshot = c.subscribe(1);
    assert_eq!(snapshot["ok"], serde_json::json!(true));
    assert_eq!(snapshot["version"], serde_json::json!(0));
    assert_eq!(snapshot["robots"], serde_json::json!([]));
    assert!(
        wait_until(
            || h.daemon.is_catalog_stream_connected(),
            Duration::from_secs(3)
        ),
        "the forwarder subscribes upstream at startup"
    );

    h.source.push(change(1, &["go2"], &["go2"], &[], 75));
    let event = c
        .next_event(Duration::from_secs(3))
        .expect("a catalog-change event must arrive with NOBODY asking for it");
    assert_eq!(event.event, CATALOG_CHANGED_EVENT);
    assert_eq!(event.version, 1);
    assert_eq!(event.robots, vec!["go2".to_string()]);
    assert_eq!(event.robots_added, vec!["go2".to_string()]);
    assert_eq!(
        event.topics_added, 75,
        "the whole attach graph, in one line"
    );
    assert_eq!(h.daemon.event_subscriber_count(), 1);
    assert_eq!(h.daemon.catalog_event_version(), 1);
}

/// THE back-compat pin — and the reason this is additive rather than a version bump:
/// a controller that did NOT subscribe receives NOTHING unsolicited, so its
/// request/response stream is byte-identical to an older daemon's. A Studio built
/// against protocol 1 keeps working untouched.
#[test]
fn a_controller_that_never_subscribed_receives_no_unsolicited_line() {
    let h = start("nosub");
    // A subscribed controller proves the events really are flowing — without it the
    // "no unsolicited line" assertion below would pass on a broken forwarder.
    let (mut subscribed, _) = Controller::connect(&h.socket);
    subscribed.subscribe(1);
    let (mut plain, _) = Controller::connect_with_timeout(&h.socket, PLAIN_READ_TIMEOUT);

    h.source.push(change(1, &["go2"], &["go2"], &[], 3));
    assert!(
        subscribed.next_event(Duration::from_secs(3)).is_some(),
        "anti-tautology: the subscribed controller MUST get the event"
    );

    // TWO request round-trips, and every line they see must be a RESPONSE. This is
    // the absence oracle, and it is an ORDERING argument with no window in it — see
    // `round_trip_collecting_events` for the two facts in the daemon it rests on.
    // It also makes the request path the DETECTOR: the loop this replaces read past
    // an event line looking for its id, so a write-to-all regression could have been
    // delivered here and ignored.
    let mut stray = round_trip_collecting_events(&mut plain, 9);
    stray.extend(round_trip_collecting_events(&mut plain, 10));
    assert!(
        stray.is_empty(),
        "an unsubscribed controller must receive NO unsolicited line, got {stray:?}"
    );
}

/// THE no-wedge pin: a subscribed controller that has STOPPED READING must not block
/// the forwarder or a healthy sibling. The stalled controller's socket buffer fills;
/// the forwarder does a mutex store and moves on.
#[test]
fn a_stalled_controller_never_blocks_a_healthy_sibling() {
    let h = start("stall");
    let (mut stalled, _) = Controller::connect(&h.socket);
    stalled.subscribe(1);
    let (mut healthy, _) = Controller::connect(&h.socket);
    healthy.subscribe(2);
    assert!(wait_until(
        || h.daemon.event_subscriber_count() == 2,
        Duration::from_secs(3)
    ));

    // Hammer the upstream. The stalled controller reads NOTHING throughout.
    for v in 1..=30 {
        h.source.push(change(v, &["go2"], &[], &[], 1));
        std::thread::sleep(Duration::from_millis(20));
    }

    let events = healthy.drain_events(Duration::from_secs(2));
    assert!(
        !events.is_empty(),
        "a stalled sibling must not starve a healthy controller"
    );

    // And the daemon is still responsive on a THIRD connection.
    let (mut third, _) = Controller::connect(&h.socket);
    third.send(&Request::Status { id: 7 });
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut ok = false;
    while Instant::now() < deadline && !ok {
        if let Ok(line) = third.read_line() {
            let v: serde_json::Value = match serde_json::from_str(line.trim()) {
                Ok(v) => v,
                Err(_) => continue,
            };
            ok = v.get("id").and_then(|i| i.as_u64()) == Some(7);
        }
    }
    assert!(
        ok,
        "vizd must still serve requests while a controller stalls"
    );
    drop(stalled);
}

/// Connection lifecycle: a controller that connects MID-STREAM is told the current
/// state rather than having to infer it, and the version it is handed is one it can
/// compare against later events.
#[test]
fn a_mid_stream_controller_is_handed_the_current_snapshot() {
    let h = start("snapshot");
    let (mut first, _) = Controller::connect(&h.socket);
    first.subscribe(1);
    // The upstream numbers this change 5; this daemon stamps its OWN generation (1 —
    // its first relayed change), which is what a controller compares against.
    h.source
        .push(change(5, &["go2", "orin"], &["go2", "orin"], &[], 12));
    let seen = first.next_event(Duration::from_secs(3)).expect("an event");
    assert_eq!(seen.version, 1);

    let (mut late, _) = Controller::connect(&h.socket);
    let snapshot = late.subscribe(2);
    assert_eq!(
        snapshot["version"],
        serde_json::json!(1),
        "a mid-stream controller starts from the current version"
    );
    assert_eq!(
        snapshot["robots"],
        serde_json::json!(["go2", "orin"]),
        "and is told who is out there, not left to guess"
    );
    assert_eq!(snapshot["connected"], serde_json::json!(true));

    // A later change moves BOTH forward from that shared version.
    h.source.push(change(6, &["go2"], &[], &["orin"], 0));
    let a = first.next_event(Duration::from_secs(3)).expect("first");
    let b = late.next_event(Duration::from_secs(3)).expect("late");
    assert_eq!(a.version, 2);
    assert_eq!(b.version, 2);
    assert_eq!(a.robots_removed, vec!["orin".to_string()]);
    assert_eq!(b.robots_removed, vec!["orin".to_string()]);
}

/// Half 1 at the CONTROLLER surface — an upstream that ACCEPTED the
/// subscription but is NOT watching (a `CERULION_NETD_NETWORK=off` netd, or one whose
/// watch declaration failed) must reach the controller as `connected: false`.
///
/// A vizd that throws the snapshot away and reports `connected: true` unconditionally
/// makes `docs/networking.md` and the protocol docs both describe the OPPOSITE of shipped
/// behaviour: a controller is told a push is coming when netd has already said none
/// can, and waits for it forever instead of falling back to its own refresh.
#[test]
fn a_not_watching_upstream_reaches_the_controller_as_disconnected() {
    let h = start_configured(
        "notwatching",
        false,
        Some(CatalogSnapshot {
            version: 3,
            robots: vec!["go2".to_string()],
            watching: false,
        }),
    );
    assert!(wait_until(
        || h.source.subscribes() >= 1,
        Duration::from_secs(3)
    ));
    let (mut c, _) = Controller::connect_with_timeout(&h.socket, Duration::from_millis(150));
    let snapshot = c.subscribe(1);
    assert_eq!(
        snapshot["connected"],
        serde_json::json!(false),
        "netd said watching:false — the controller must be told plainly, not promised \
         an event that cannot come"
    );
    assert!(!h.daemon.is_catalog_stream_connected());
    // The subscription is still ACCEPTED and the catalog it DID report is still
    // adopted — the daemon degrades openly rather than refusing or going blank.
    assert_eq!(snapshot["version"], serde_json::json!(3));
    assert_eq!(snapshot["robots"], serde_json::json!(["go2"]));
    assert_eq!(h.daemon.event_subscriber_count(), 1);
}

/// Half 2 at the CONTROLLER surface — a controller connecting to a
/// vizd that just started against a LIVE netd is handed the catalog that already
/// exists, with NO change having been pushed at all.
///
/// This is the restart case the protocol's re-subscribe recovery documents. A vizd
/// that discards netd's snapshot and seeds an empty hub sees nothing replay what was
/// missed, because netd's watch is already running by then — a desk whose robots are all up
/// and steady would render an empty sidebar until something happened to change.
///
/// The sibling mid-stream arm cannot catch this: it pre-populates the hub by PUSHING a
/// change through it, which is the one path that was never broken.
#[test]
fn a_restarted_vizd_serves_the_existing_catalog_with_no_change_pushed() {
    let h = start_configured(
        "seeded",
        false,
        Some(CatalogSnapshot {
            version: 12,
            robots: vec!["go2".to_string(), "orin".to_string()],
            watching: true,
        }),
    );
    assert!(wait_until(
        || h.daemon.is_catalog_stream_connected(),
        Duration::from_secs(3)
    ));

    let (mut c, _) = Controller::connect_with_timeout(&h.socket, Duration::from_millis(150));
    let snapshot = c.subscribe(1);
    assert_eq!(
        snapshot["robots"],
        serde_json::json!(["go2", "orin"]),
        "a controller on a just-restarted vizd must be handed the CURRENT robots"
    );
    assert_eq!(
        snapshot["version"],
        serde_json::json!(12),
        "and the upstream generation it can compare later pushes against"
    );
    assert_eq!(snapshot["connected"], serde_json::json!(true));
    assert_eq!(h.daemon.catalog_event_version(), 12);
    // Nothing was pushed to get here — the anti-tautology half.
    assert!(c.drain_events(Duration::from_millis(300)).is_empty());
}

/// Reported degradation: a daemon whose upstream cannot serve pushes (a too-old netd)
/// still ACCEPTS the subscription — refusing would be indistinguishable from an old
/// vizd — but reports `connected: false`, so a controller keeps its own refresh path
/// instead of waiting on an event that cannot come.
#[test]
fn an_unsupported_upstream_is_reported_rather_than_pretended() {
    let h = start_with("unsupported", true);
    // The forwarder gives up at once on a permanent refusal (it does not spin).
    assert!(wait_until(
        || h.source.subscribes() >= 1,
        Duration::from_secs(3)
    ));
    let (mut c, _) = Controller::connect_with_timeout(&h.socket, Duration::from_millis(150));
    let snapshot = c.subscribe(1);
    assert_eq!(
        snapshot["connected"],
        serde_json::json!(false),
        "a controller must be told plainly that no event can arrive here"
    );
    assert!(!h.daemon.is_catalog_stream_connected());
    assert_eq!(h.daemon.event_subscriber_count(), 1, "still subscribed");
    // And no event ever arrives.
    let stray = c.drain_events(Duration::from_millis(400));
    assert!(stray.is_empty());
    // The permanent refusal is not retried in a loop.
    std::thread::sleep(Duration::from_millis(300));
    assert_eq!(
        h.source.subscribes(),
        1,
        "a permanent refusal must not spin — netd is spawn-once, so retrying cannot help"
    );
}

/// The event-less entry point (`start_with_demand_plane`, which every existing caller
/// uses) has
/// no event source, so nothing is pushed and `subscribe_events` says so.
#[test]
fn the_pre_928_entry_point_pushes_nothing_and_says_so() {
    let (socket, dir) = temp_socket("pre928");
    let manager = isolated_manager("pre928");
    let (rec, _storage) = rerun::RecordingStreamBuilder::new("vizd")
        .recording_id("pre928")
        .memory()
        .expect("memory sink");
    let worker = VizLogWorker::spawn(rec, builtin_walker(), SinkState::new()).expect("worker");
    let daemon = cerulion_vizd::daemon::start_with_demand_plane(
        socket.clone(),
        Duration::from_millis(20),
        manager,
        worker,
        builtin_walker(),
        None,
        Arc::new(NoNetdPlane),
    )
    .expect("daemon start");

    let (mut c, _) = Controller::connect_with_timeout(&socket, Duration::from_millis(150));
    let snapshot = c.subscribe(1);
    assert_eq!(snapshot["connected"], serde_json::json!(false));
    assert_eq!(snapshot["version"], serde_json::json!(0));
    assert!(c.drain_events(Duration::from_millis(300)).is_empty());
    assert!(!daemon.is_catalog_stream_connected());
    drop(daemon);
    let _ = std::fs::remove_dir_all(&dir);
}

/// Closing the connection unsubscribes it (the crash-safe rule), so a dead controller
/// never leaves a push slot nobody drains.
#[test]
fn closing_the_connection_unsubscribes_it() {
    let h = start("unsub");
    let (mut c, _) = Controller::connect(&h.socket);
    c.subscribe(1);
    assert!(wait_until(
        || h.daemon.event_subscriber_count() == 1,
        Duration::from_secs(3)
    ));
    drop(c);
    assert!(
        wait_until(
            || h.daemon.event_subscriber_count() == 0,
            Duration::from_secs(5)
        ),
        "the connection close must unsubscribe it, still {} subscribed",
        h.daemon.event_subscriber_count()
    );
    // The forwarder keeps running and still tracks the world for the next subscriber.
    // The version is THIS daemon's own generation (1 after one relayed change), not
    // the upstream's number — see `SubscribeEventsResponse::version`.
    h.source.push(change(9, &["go2"], &["go2"], &[], 1));
    assert!(wait_until(
        || h.daemon.catalog_event_version() == 1,
        Duration::from_secs(3)
    ));
}

/// The wire discriminator: an event is identifiable WITHOUT positional assumptions —
/// it carries `event` and no `id`; a response carries `id` and no `event`. That is
/// what lets a controller stay in sync on a stream carrying both.
#[test]
fn an_event_is_structurally_distinguishable_from_a_response() {
    let h = start("classify");
    let (mut c, _) = Controller::connect(&h.socket);
    c.subscribe(1);
    h.source.push(change(1, &["go2"], &["go2"], &[], 2));

    let deadline = Instant::now() + Duration::from_secs(3);
    let mut raw_event: Option<serde_json::Value> = None;
    while Instant::now() < deadline && raw_event.is_none() {
        if let Ok(line) = c.read_line() {
            if let Ok(v) = serde_json::from_str::<serde_json::Value>(line.trim()) {
                if v.get("event").is_some() {
                    raw_event = Some(v);
                }
            }
        }
    }
    let v = raw_event.expect("an event line");
    assert_eq!(
        v.get("event").and_then(|e| e.as_str()),
        Some(CATALOG_CHANGED_EVENT)
    );
    assert!(
        v.get("id").is_none(),
        "an event carries no correlation id — that is the discriminator"
    );
    assert!(v.get("ok").is_none(), "and no `ok` field");

    // A response is classified the other way.
    c.send(&Request::List { id: 33 });
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut saw = false;
    while Instant::now() < deadline && !saw {
        if let Ok(line) = c.read_line() {
            if let Ok(v) = serde_json::from_str::<serde_json::Value>(line.trim()) {
                if v.get("id").and_then(|i| i.as_u64()) == Some(33) {
                    assert!(v.get("event").is_none(), "a response carries no event key");
                    saw = true;
                }
            }
        }
    }
    assert!(saw, "the list response must arrive and classify");
}

/// A subscribed connection STILL serves ordinary requests — subscribing does not make
/// it write-only, and an event interleaving with a response does not break either.
#[test]
fn a_subscribed_connection_still_serves_ordinary_requests() {
    let h = start("mixed");
    let (mut c, _) = Controller::connect(&h.socket);
    c.subscribe(1);
    h.source.push(change(1, &["go2"], &["go2"], &[], 1));
    assert!(c.next_event(Duration::from_secs(3)).is_some());

    c.send(&Request::Discover { id: 4 });
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut saw = false;
    while Instant::now() < deadline && !saw {
        if let Ok(line) = c.read_line() {
            if let Ok(v) = serde_json::from_str::<serde_json::Value>(line.trim()) {
                if v.get("id").and_then(|i| i.as_u64()) == Some(4) {
                    assert_eq!(v["ok"], serde_json::json!(true));
                    saw = true;
                }
            }
        }
    }
    assert!(saw, "discover must still work on a subscribed connection");
}

/// A re-subscribe is idempotent (one subscriber, not two) and re-reports the snapshot.
#[test]
fn a_re_subscribe_is_idempotent() {
    let h = start("resub");
    let (mut c, _) = Controller::connect(&h.socket);
    c.subscribe(1);
    h.source.push(change(3, &["go2"], &["go2"], &[], 1));
    assert!(c.next_event(Duration::from_secs(3)).is_some());

    let again = c.subscribe(2);
    // This daemon's generation after one relayed change — not the upstream's 3.
    assert_eq!(again["version"], serde_json::json!(1));
    assert_eq!(again["robots"], serde_json::json!(["go2"]));
    assert_eq!(
        h.daemon.event_subscriber_count(),
        1,
        "a re-subscribe on one connection is ONE subscriber"
    );
}

/// Anti-tautology control for the whole file: with NO upstream change at all, a
/// subscribed controller receives NOTHING. Without this, every "exactly N events" arm
/// would also pass against a daemon that pushed on a timer.
#[test]
fn a_quiet_upstream_pushes_nothing() {
    let h = start("quiet");
    let (mut c, _) = Controller::connect_with_timeout(&h.socket, Duration::from_millis(150));
    let snapshot = c.subscribe(1);
    assert_eq!(snapshot["connected"], serde_json::json!(true));
    // This 600 ms stays. Unlike the arm above there is no positive witness to be
    // after: the premise is a QUIET upstream, so nothing happens that could prove
    // the fan-out pass is over, and the window length IS the claim — it bounds the
    // period of a timer-driven push this control can catch. Shortening it would
    // narrow that class, which is loosening, not polling.
    let stray = c.drain_events(Duration::from_millis(600));
    assert!(
        stray.is_empty(),
        "a quiet upstream must produce no event, got {stray:?}"
    );
    assert_eq!(h.daemon.catalog_event_version(), 0);
}

// --------------------------------------------------------------------------
// An IDLE controller's read loop is PACED, not spinning.
//
// This is the same read loop the pushes above ride, so it belongs here. We
// measured `cerulion-vizd` at 99.4 % CPU while rendering nothing: the
// socket accepted from the nonblocking listener inherited `O_NONBLOCK`
// (macOS/BSD), so the `read` that was supposed to pace the loop at
// `CONN_READ_TIMEOUT` returned `EAGAIN` in microseconds and the
// `WouldBlock => continue` arm became an unbounded tight loop.
//
// The oracle is an UPPER bound on loop iterations over a held-idle window, which
// is LOAD-IMMUNE in the right direction: every iteration costs a blocking read of
// `CONN_READ_TIMEOUT`, so a contended runner can only produce FEWER, never more.
//
// PLATFORM SCOPE of the mutation evidence: deleting `set_nonblocking(false)` is a
// behavioural no-op on LINUX — `accept(2)` inherits no file status flags there, so
// the accepted socket is already blocking and this arm stays green. The
// 1.7M/5.4M-iteration kill is a macOS result, and `test-macos` is the job that can
// reproduce it. The arm is NOT vacuous on Linux: dropping `set_read_timeout` from
// the same seam still fails it (the read blocks forever, tripping `delta >= 2`),
// and the ceiling remains a live guard against any Linux-reachable spin.
// --------------------------------------------------------------------------

#[test]
fn an_idle_controllers_read_loop_is_paced_by_the_read_timeout_not_spinning() {
    let h = start("pace");
    let (mut c, hello) = Controller::connect(&h.socket);
    assert_eq!(
        hello.protocol, PROTOCOL_VERSION,
        "the banner proves the handler thread really served this connection"
    );

    // Serve one request so the handler is provably INSIDE its read loop (not still
    // in accept) before the measured window opens.
    c.send(&Request::List { id: 1 });
    let first = c.read_line().expect("list response");
    let first: serde_json::Value = serde_json::from_str(first.trim()).expect("json");
    assert_eq!(first["id"], serde_json::json!(1));

    const HOLD: Duration = Duration::from_millis(1500);
    let before = h.daemon.conn_read_loop_iterations();
    std::thread::sleep(HOLD);
    let delta = h.daemon.conn_read_loop_iterations() - before;

    // Hand oracle: one iteration per CONN_READ_TIMEOUT ⇒ 1500/200 = 7. The ceiling
    // is 5× that — unreachable by timer jitter, and ~5 orders of magnitude below a
    // spinner (MEASURED without the read timeout: an idle read returns in ~0.6 µs).
    let expected = HOLD.as_nanos() / cerulion_vizd::daemon::CONN_READ_TIMEOUT.as_nanos();
    let ceiling = (expected * 5) as u64;
    assert!(
        delta <= ceiling,
        "an IDLE controller went round its read loop {delta} times in {HOLD:?} \
         (expected ~{expected}, ceiling {ceiling}) — the accepted socket is \
         nonblocking again and the handler is SPINNING"
    );
    // Anti-vacuity: the loop must actually be RUNNING. Without this, a handler that
    // died (or never started) would satisfy the ceiling with a delta of 0.
    assert!(
        delta >= 2,
        "the read loop ran only {delta} times in {HOLD:?} — the handler is not alive, \
         so the ceiling above proves nothing"
    );

    // And the connection is still SERVING after the idle window (not merely alive).
    c.send(&Request::List { id: 2 });
    let after = c.read_line().expect("list response after idle");
    let after: serde_json::Value = serde_json::from_str(after.trim()).expect("json");
    assert_eq!(
        after["id"],
        serde_json::json!(2),
        "the connection must still serve requests after an idle window"
    );
}
