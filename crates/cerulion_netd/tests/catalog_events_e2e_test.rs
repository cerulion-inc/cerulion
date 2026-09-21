// SPDX-License-Identifier: AGPL-3.0-only
//! The catalog-change PUSH plane end to end over the REAL UDS control server
//! — a consumer subscribes, netd's announce watch observes transitions, and the change
//! arrives UNPROMPTED down the connection. No polling anywhere.
//!
//! Driven in-process with a SCRIPTED `AnnounceWatchPlane` double (a legitimate
//! dependency-injection seam for the trait — Principle #13, not fabricated data: the
//! events it yields are the exact `AnnounceEvent`s cerulion_core's real liveliness
//! subscriber produces, and the LIVE zenoh half is pinned separately in
//! `catalog_events_live_test.rs`). No zenoh, no iceoryx2; each test drives its own
//! daemon over a UNIQUE temp socket ⇒ parallel-safe, no `#[serial]`.
//!
//! Every assertion is against a HAND-WRITTEN oracle (the exact counts, the exact robot
//! names, the exact number of pushed LINES), never a self-compare.

use std::io::{self, BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::transport::discovery::AnnounceEvent;

use cerulion_netd::catalog_events::{
    AnnounceStream, AnnounceWatchPlane, NoopAnnounceWatchPlane, WatchError,
};
use cerulion_netd::daemon::{self, NetdConfig, RunningNetd};
use cerulion_netd::egress::NoopEgressPlane;
use cerulion_netd::mirror::{MirrorError, MirrorPlane, MirrorRelease};
use cerulion_netd::protocol::{
    classify_control_line, CatalogChanged, ControlLine, Hello, Request, Response,
    CATALOG_CHANGED_EVENT, PROTOCOL_VERSION,
};
use cerulion_netd::query::NoopQueryPlane;
use cerulion_netd::registry::TopicKey;

// --------------------------------------------------------------------------
// Harness.
// --------------------------------------------------------------------------

/// A mirror plane that does nothing — this file exercises the EVENT plane, and a
/// no-op keeps the demand half out of every oracle.
struct InertMirrorPlane;

impl MirrorPlane for InertMirrorPlane {
    fn ensure_mirror(&self, _key: &TopicKey, _schema_hash: u64) -> Result<(), MirrorError> {
        Ok(())
    }
    fn release_mirror(&self, _key: &TopicKey) -> MirrorRelease {
        MirrorRelease::Retired
    }
}

/// A SCRIPTED announce stream: the test pushes `AnnounceEvent`s into a queue and the
/// daemon's watch thread drains it, exactly as it drains the real liveliness
/// subscriber. `watch_calls` proves the watch is declared ONCE (on the first
/// subscriber) and never per-subscription.
#[derive(Default)]
struct ScriptedAnnouncePlane {
    queue: Arc<Mutex<std::collections::VecDeque<AnnounceEvent>>>,
    watch_calls: AtomicUsize,
    /// When set, `watch()` refuses — the "no announce space to watch" arm.
    refuse: AtomicBool,
}

impl ScriptedAnnouncePlane {
    fn push(&self, event: AnnounceEvent) {
        self.queue.lock().unwrap().push_back(event);
    }
    fn push_alive(&self, robot: &str, topic: Option<&str>) {
        self.push(AnnounceEvent::Alive {
            robot: robot.to_string(),
            topic: topic.map(str::to_string),
        });
    }
    fn push_lost(&self, robot: &str, topic: Option<&str>) {
        self.push(AnnounceEvent::Lost {
            robot: robot.to_string(),
            topic: topic.map(str::to_string),
        });
    }
    fn watch_calls(&self) -> usize {
        self.watch_calls.load(Ordering::SeqCst)
    }
}

struct ScriptedStream {
    queue: Arc<Mutex<std::collections::VecDeque<AnnounceEvent>>>,
}

impl AnnounceStream for ScriptedStream {
    fn next_event(&mut self, timeout: Duration) -> Option<AnnounceEvent> {
        // Poll the queue for up to `timeout`, mirroring a real subscriber's
        // block-then-report shape (an empty window is `None`, never an error).
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(e) = self.queue.lock().unwrap().pop_front() {
                return Some(e);
            }
            if Instant::now() >= deadline {
                return None;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
    }
}

impl AnnounceWatchPlane for ScriptedAnnouncePlane {
    fn watch(&self) -> Result<Box<dyn AnnounceStream>, WatchError> {
        self.watch_calls.fetch_add(1, Ordering::SeqCst);
        if self.refuse.load(Ordering::SeqCst) {
            return Err(WatchError::NoNetwork);
        }
        Ok(Box::new(ScriptedStream {
            queue: Arc::clone(&self.queue),
        }))
    }
}

/// A long grace so idle self-exit never fires during a functional test.
const LONG_GRACE: Duration = Duration::from_secs(3600);

/// A SHORT coalescing window so an arm pins the burst contract without sleeping the
/// production 250 ms. The contract under test (a burst becomes ONE push) is
/// window-independent; only the wall is.
const TEST_WINDOW: Duration = Duration::from_millis(60);

/// The GENEROUS read budget every *protocol* read gets: hello, and each retry pass of a
/// request/response exchange. It is deliberately far larger than any scheduling delay a
/// loaded CI runner can impose, because none of these reads is an assertion about
/// TIMING — they assert that the daemon answers AT ALL.
const PROTOCOL_READ_TIMEOUT: Duration = Duration::from_secs(3);

fn unique_socket(tag: &str) -> (tempfile::TempDir, PathBuf) {
    let dir = tempfile::tempdir().expect("tempdir");
    let sock = dir.path().join(format!("{tag}.sock"));
    (dir, sock)
}

#[allow(clippy::type_complexity)]
fn start_daemon(
    tag: &str,
) -> (
    RunningNetd,
    tempfile::TempDir,
    PathBuf,
    Arc<ScriptedAnnouncePlane>,
) {
    start_daemon_with(tag, false)
}

#[allow(clippy::type_complexity)]
fn start_daemon_with(
    tag: &str,
    refuse_watch: bool,
) -> (
    RunningNetd,
    tempfile::TempDir,
    PathBuf,
    Arc<ScriptedAnnouncePlane>,
) {
    start_daemon_with_window(tag, refuse_watch, TEST_WINDOW)
}

#[allow(clippy::type_complexity)]
fn start_daemon_with_window(
    tag: &str,
    refuse_watch: bool,
    window: Duration,
) -> (
    RunningNetd,
    tempfile::TempDir,
    PathBuf,
    Arc<ScriptedAnnouncePlane>,
) {
    let (dir, sock) = unique_socket(tag);
    let announce = Arc::new(ScriptedAnnouncePlane::default());
    announce.refuse.store(refuse_watch, Ordering::SeqCst);
    let netd = daemon::start_with_planes_and_events(
        sock.clone(),
        Arc::new(InertMirrorPlane),
        Arc::new(NoopEgressPlane),
        Arc::new(NoopQueryPlane),
        Arc::clone(&announce) as Arc<dyn AnnounceWatchPlane>,
        NetdConfig {
            idle_grace: LONG_GRACE,
            idle_watch_poll: Duration::from_millis(25),
            change_coalesce_window: window,
            ..NetdConfig::default()
        },
    )
    .expect("daemon start");
    (netd, dir, sock, announce)
}

struct Client {
    stream: UnixStream,
    reader: BufReader<UnixStream>,
}

impl Client {
    fn connect(sock: &Path) -> (Client, Hello) {
        Self::connect_with_timeout(sock, PROTOCOL_READ_TIMEOUT)
    }

    /// `read_timeout` is the POLLING GRANULARITY the caller wants for its *drain* loops
    /// (`drain_pushes` / `next_push` re-check their own deadline once per expiry, so a
    /// short value makes a "no unsolicited line" drain quick). It is deliberately NOT
    /// applied to the hello read: hello is a protocol assertion ("the daemon greets us"),
    /// not a timing one, and reading it under a 150 ms budget races daemon scheduling —
    /// a loaded macOS runner fails exactly that way with `WouldBlock`. So hello is
    /// always read under `PROTOCOL_READ_TIMEOUT`, and the caller's value is installed
    /// only once hello has been parsed.
    fn connect_with_timeout(sock: &Path, read_timeout: Duration) -> (Client, Hello) {
        let stream = UnixStream::connect(sock).expect("connect");
        stream
            .set_read_timeout(Some(PROTOCOL_READ_TIMEOUT))
            .unwrap();
        stream
            .set_write_timeout(Some(PROTOCOL_READ_TIMEOUT))
            .unwrap();
        let reader = BufReader::new(stream.try_clone().expect("clone"));
        let mut c = Client { stream, reader };
        let hello: Hello =
            serde_json::from_str(c.read_line().expect("hello line").trim()).expect("hello parses");
        c.set_read_timeout(read_timeout);
        (c, hello)
    }

    /// Install `timeout` on BOTH handles. `try_clone` dups the fd, so the two share one
    /// socket and one `SO_RCVTIMEO` — but `self.reader` is what actually reads, so it is
    /// set explicitly rather than left to that aliasing.
    fn set_read_timeout(&mut self, timeout: Duration) {
        self.stream.set_read_timeout(Some(timeout)).unwrap();
        self.reader
            .get_ref()
            .set_read_timeout(Some(timeout))
            .unwrap();
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

    /// Send `subscribe_catalog` and return the parsed snapshot response, tolerating an
    /// event that legitimately lands first.
    ///
    /// The read is retried under a deadline for the same reason the hello read is exempt
    /// from the caller's timeout: a client built with a short DRAIN granularity would
    /// otherwise turn "the daemon answered" into a race with daemon scheduling.
    fn subscribe(&mut self, id: u64) -> cerulion_netd::SubscribeCatalogResponse {
        self.send(&Request::SubscribeCatalog { id });
        let deadline = Instant::now() + PROTOCOL_READ_TIMEOUT;
        while Instant::now() < deadline {
            let line = match self.read_line() {
                Ok(line) => line,
                Err(e)
                    if matches!(
                        e.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                    ) =>
                {
                    continue
                }
                Err(e) => panic!("the connection died instead of answering subscribe: {e}"),
            };
            match classify_control_line(&line) {
                ControlLine::Response(text) => {
                    match serde_json::from_str::<Response>(&text).expect("response parses") {
                        Response::SubscribeCatalog(r) => return r,
                        other => panic!("expected a subscribe response, got {other:?}"),
                    }
                }
                ControlLine::Event(_) => continue,
                ControlLine::Unknown(t) => panic!("unclassifiable line: {t}"),
            }
        }
        panic!("no subscribe response within {PROTOCOL_READ_TIMEOUT:?}");
    }

    /// Read lines until ONE catalog-change push arrives (or the deadline passes).
    fn next_push(&mut self, within: Duration) -> Option<CatalogChanged> {
        let deadline = Instant::now() + within;
        while Instant::now() < deadline {
            match self.read_line() {
                Ok(line) => match classify_control_line(&line) {
                    ControlLine::Event(e) => return Some(e),
                    _ => continue,
                },
                Err(e)
                    if matches!(
                        e.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                    ) =>
                {
                    continue
                }
                Err(_) => return None,
            }
        }
        None
    }

    /// Drain every push that arrives within `within` — the oracle for "EXACTLY N
    /// lines", which a single-shot read cannot establish.
    fn drain_pushes(&mut self, within: Duration) -> Vec<CatalogChanged> {
        let deadline = Instant::now() + within;
        let mut out = Vec::new();
        while Instant::now() < deadline {
            match self.read_line() {
                Ok(line) => {
                    if let ControlLine::Event(e) = classify_control_line(&line) {
                        out.push(e);
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

/// THE headline: a robot's topics coming up reaches a subscribed consumer UNPROMPTED.
/// Without the push the sidebar keeps the old view because nothing
/// tells it; this drives the seam where that would fail.
#[test]
fn a_robot_announcing_pushes_a_change_to_a_subscriber_unprompted() {
    let (netd, _dir, sock, announce) = start_daemon("push");
    let (mut c, hello) = Client::connect(&sock);
    assert_eq!(hello.protocol, PROTOCOL_VERSION);

    let snapshot = c.subscribe(1);
    assert_eq!(snapshot.id, 1);
    assert_eq!(snapshot.version, 0, "nothing has changed yet");
    assert!(snapshot.robots.is_empty(), "no robot announced yet");
    assert!(snapshot.watching, "the scripted plane declares a watch");
    assert_eq!(
        announce.watch_calls(),
        1,
        "the watch is declared once, on the first subscriber"
    );

    // The robot comes up: its identity token + two topics.
    announce.push_alive("go2", None);
    announce.push_alive("go2", Some("/utlidar/cloud"));
    announce.push_alive("go2", Some("/odom"));

    let push = c
        .next_push(Duration::from_secs(3))
        .expect("a catalog-change push must arrive with NOBODY asking for it");
    assert_eq!(push.event, CATALOG_CHANGED_EVENT);
    assert_eq!(push.version, 1, "the first published change");
    assert_eq!(push.robots, vec!["go2".to_string()]);
    assert_eq!(push.robots_added, vec!["go2".to_string()]);
    assert!(push.robots_removed.is_empty());
    assert_eq!(push.topics_added, 2, "the identity token is not a topic");
    assert_eq!(push.topics_removed, 0);
    assert_eq!(netd.catalog_version(), 1);
    assert_eq!(netd.catalog_subscriber_count(), 1);
    assert!(netd.is_watching_announces());
}

/// The push is delivered by a WAKE, not by the read timeout.
///
/// Before this, the push drain sat immediately before a blocking read, so a
/// change waited up to one `CONN_READ_TIMEOUT` (200 ms) for that read to give up.
/// The handler now waits on socket-readable OR push-slot-filled in one `poll(2)`.
///
/// **The load-bearing assertion is the `conn_push_wakes` COUNTER, not a wall.** A
/// wall tight enough to separate "woken" from "timed out" is ~200 ms, exactly the
/// margin macOS background-QoS timer coalescing eats (the timer-coalescing class); and no
/// elapsed time can distinguish a fired wake from a lucky timeout in the first place.
/// A timeout can never increment this counter, so load can delay the assertion but
/// never satisfy it by accident. The measured latency is PRINTED as evidence for a
/// human and deliberately ungated.
#[test]
fn a_catalog_push_is_delivered_by_a_wake_not_the_read_timeout() {
    let (netd, _dir, sock, announce) = start_daemon("wkwake");
    let (mut c, _) = Client::connect(&sock);
    c.subscribe(1);
    assert_eq!(
        netd.conn_push_wakes(),
        0,
        "nothing has been published yet, so nothing can have woken the loop"
    );

    let started = Instant::now();
    announce.push_alive("go2", Some("/odom"));
    let push = c
        .next_push(Duration::from_secs(10))
        .expect("the change must reach the subscriber");
    let elapsed = started.elapsed();
    assert_eq!(push.event, CATALOG_CHANGED_EVENT);

    println!("catalog push delivered in {elapsed:?} (before the waker: up to one 200 ms CONN_READ_TIMEOUT)");
    assert!(
        wait_until(|| netd.conn_push_wakes() >= 1, Duration::from_secs(10)),
        "the push must have WOKEN the control loop rather than waiting out the read \
         timeout — `conn_push_wakes` is only ever incremented by a wait that ended on \
         the push waker (delivered in {elapsed:?})"
    );
}

/// The ANTI-TAUTOLOGY control: a connection that never subscribed is
/// never woken.
///
/// Without it, "the counter grew" would be satisfied by a waker poked on every
/// publish regardless of subscription — which would wake every idle `topic echo`
/// connection on the machine for an event it did not ask for and cannot act on, and
/// would make the positive arm above pass for the wrong reason.
#[test]
fn an_unsubscribed_connection_is_never_woken_by_a_catalog_push() {
    let (netd, _dir, sock, announce) = start_daemon("wknosub");
    // ONE subscriber — the watch thread only starts on the first subscription, so a
    // daemon with nobody subscribed can never publish and the control would be
    // vacuous. This connection is the one that legitimately gets woken.
    let (mut sub, _) = Client::connect(&sock);
    sub.subscribe(1);
    // A SECOND connection that deliberately never subscribes.
    let (mut quiet, _) = Client::connect(&sock);
    assert!(
        wait_until(|| netd.active_connections() >= 2, Duration::from_secs(10)),
        "both connections must be registered before the push"
    );

    announce.push_alive("go2", Some("/odom"));
    let push = sub
        .next_push(Duration::from_secs(10))
        .expect("the SUBSCRIBED connection receives the push");
    assert_eq!(push.event, CATALOG_CHANGED_EVENT);
    assert_eq!(netd.catalog_version(), 1, "exactly one published change");
    // Settle past the coalescing window so a second poke, if the implementation made
    // one, would have arrived.
    std::thread::sleep(Duration::from_millis(300));

    assert_eq!(
        netd.conn_push_wakes(),
        1,
        "EXACTLY the subscribed connection was woken — a waker poked for every \
         connection would wake every idle `topic echo` on the machine for an event it \
         never asked for, and would make the positive arm above pass for the wrong \
         reason"
    );
    assert!(
        quiet.next_push(Duration::from_millis(200)).is_none(),
        "and the unsubscribed connection receives nothing unsolicited (the watch is opt-in)"
    );
}

/// The waker pipe does not accumulate, so
/// the drain can never meet the exact-multiple backlog that wedged the handler.
///
/// Were the drain to run on the `ControlWake::Push` arm alone:
/// `wait_for_control_input` gives the SOCKET the tie — so on a connection whose
/// socket is continuously readable the push arm never fires, every poke stays in the
/// pipe, and the backlog grows without bound (MEASURED against the real daemon:
/// 1,396 stale bytes on one connection). That matters because a `drain_waker` looping
/// on a full sink over a BLOCKING read end meets a multiple of 64 and the next drain
/// parks forever on an empty pipe whose only writer that same thread must close.
///
/// This drives the shape end to end — a subscribed connection flooded with BLANK
/// LINES, which `dispatch` skips with no response, so the socket stays readable
/// without building a reply backlog — and asserts the RESIDUE, which is the only
/// thing that distinguishes the two designs: both end with an empty pipe, so a
/// total-bytes counter agrees either way. It also asserts the connection is still
/// SERVING afterwards, which is what the wedge destroys.
///
/// LOAD DIRECTION, stated because it is the whole argument for a ceiling here: under
/// contention a drain-on-`Push`-only run accumulates MORE (the handler drains less often), so load
/// can only widen the gap from the loop-top design. With the loop-top drain
/// the residue is bounded by the pokes
/// landing inside ONE loop pass, and a poke is rate-limited by the coalescing window,
/// so reaching the ceiling would need the handler starved for `32 × WINDOW` between
/// two passes of a loop the flood keeps read-bound. MEASURED: **1 byte** on an idle
/// desk (5 runs) AND **1 byte** under 24× CPU oversubscription with macOS background
/// QoS (3 runs, ~48 s wall each), against **160** — every single poke — for the
/// drain-on-`Push`-only variant.
///
/// PLAIN CPU contention, rather than background QoS,
/// is the harsher shape for this arm (background QoS stretches the drive loop
/// too, so the two effects cancel; plain spinners starve the handler while the drive
/// loop keeps its ~2.2 s wall). The residue is then **single digits to low twenties**
/// on a 16-core desk — still comfortably under the 32-byte ceiling, but a ~2x
/// margin rather than the ~32x an idle desk suggests, because a longer pass admits
/// more of the window-rate-limited pokes.
///
/// It is recorded as an ORDER rather than a band, deliberately: the residue is a
/// function of the ambient load, not a property, and two measurement sessions at the
/// SAME nominal spinner count on one machine do not agree — 32 spinners gave
/// **6-13** on one and **6-14** on the other, 48 gave **8-14** and **10-21**, and 64
/// gave **8-21**. A band written down as if it were reproducible invites a false alarm
/// the next time the machine is busier than the day it was measured, which is the
/// false-alarm failure itself one layer up. What a future firing of the CEILING should be
/// read against is the ORDER: a residue in the twenties is a loaded handler, and one
/// in the hundreds is the `32 × WINDOW` stall this arm exists to catch.
///
/// Note also which quantity moves it. The LOAD AVERAGE does not: 14 runs
/// at a 1-minute average of 28-75 against 16 cores gave a residue of **1** every time,
/// with the flood at 9.4-10.0 MiB and 2307-2435 handler passes — i.e. idle-desk
/// numbers. What elevates it is CPU genuinely contended WITH THE HANDLER, which is why
/// the reproduction above uses spinners rather than an already-busy machine.
#[test]
fn a_flooded_connections_waker_never_accumulates_a_backlog() {
    // The coalescing window is the publish RATE limit, so a short one keeps the drive
    // loop below. The contract under test is window-independent.
    const WINDOW: Duration = Duration::from_millis(5);
    /// Well past the 64-byte sink, so a drain-on-`Push`-only run walks through TWO wedging
    /// multiples on its way up.
    const PUSHES: u64 = 160;
    /// A healthy residue is 1-2 (the pokes landing inside one pass). 32 sits far
    /// above that and far below both the drain-on-`Push`-only backlog (~PUSHES) and the 64-byte
    /// wedge boundary, so it cannot be met by a run that merely drained late.
    const MAX_DRAIN_CEILING: u64 = 32;
    /// The floor on the TOTAL wake bytes any drain ever consumed. The
    /// high-water mark is a `fetch_max`, so it is blind to a drain that never RAN;
    /// this is the same observation summed instead of maxed. With the loop-top drain it
    /// is EXACTLY `PUSHES` and under the drain-on-Push-only design it cannot exceed 96 — see the
    /// arithmetic in the verdict block below, which is what fixes the position.
    const MIN_TOTAL_DRAIN: u64 = PUSHES - MAX_DRAIN_CEILING;
    /// One flood write. 64 KiB against the daemon's 4 KiB-per-pass read keeps the
    /// socket readable across 16 passes, so a single batch already outruns the reader
    /// it is trying to stay ahead of.
    const FLOOD_BATCH: usize = 64 * 1024;

    let (netd, _dir, sock, announce) = start_daemon_with_window("wkflood", false, WINDOW);
    let (mut c, _) = Client::connect(&sock);
    c.subscribe(1);
    // A short read granularity so the reader can poll its stop flag while draining.
    c.set_read_timeout(Duration::from_millis(5));

    let stop = Arc::new(AtomicBool::new(false));
    let (served_tx, served_rx) = std::sync::mpsc::channel::<bool>();

    // The FLOOD gets its OWN thread and never reads. Interleaving writes with reads on
    // one thread does not work and the failure is silent: a `read_line` that finds
    // nothing costs the whole read timeout, so the writer stalls, the daemon empties
    // the socket, its poll finds nothing there and the push arm wins after all —
    // MEASURED at a 1-byte high-water mark, i.e. an arm that pinned nothing.
    let flood_writer = c.stream.try_clone().expect("clone for the flood");
    let flood_stop = Arc::clone(&stop);
    // ANTI-VACUITY EVIDENCE, in two parts — both LOAD-IMMUNE, and neither of them a
    // throughput floor (see the assertion block after the drive loop for why a
    // throughput floor is the wrong instrument).
    //
    // `flood_writes` counts COMPLETED 64 KiB writes and `flood_write_failed` records
    // the one shape that makes this arm vacuous: a flood thread that died
    // on its first write (a closed socket, a cloned handle that never worked) leaves
    // the connection IDLE, and an idle connection is exactly where the
    // drain-on-Push-only design also reports a 1-byte high-water mark.
    let flood_writes = Arc::new(AtomicU64::new(0));
    let flood_write_failed = Arc::new(AtomicBool::new(false));
    let (flood_writes_h, flood_failed_h) =
        (Arc::clone(&flood_writes), Arc::clone(&flood_write_failed));
    let flood = std::thread::spawn(move || {
        // Blank lines: `dispatch` skips them with NO response, so the socket stays
        // readable without building any reply backlog. 64 KiB per write against the
        // daemon's 4 KiB-per-pass read keeps it readable across many passes.
        let batch = vec![b'\n'; FLOOD_BATCH];
        let mut writer = flood_writer;
        while !flood_stop.load(Ordering::SeqCst) {
            if writer.write_all(&batch).is_err() {
                flood_failed_h.store(true, Ordering::SeqCst);
                return;
            }
            flood_writes_h.fetch_add(1, Ordering::Relaxed);
        }
    });

    // The reader drains pushes so the daemon's own writes never block (a peer that
    // stops reading is dropped for a slow consumer), then asks the one question the
    // wedge destroys: does the connection still SERVE?
    let reader_stop = Arc::clone(&stop);
    let reader = std::thread::spawn(move || {
        while !reader_stop.load(Ordering::SeqCst) {
            let _ = c.read_line();
        }
        c.set_read_timeout(Duration::from_millis(200));
        c.send(&Request::Status { id: 99 });
        let deadline = Instant::now() + Duration::from_secs(10);
        while Instant::now() < deadline {
            match c.read_line() {
                Ok(line) => {
                    if let ControlLine::Response(text) = classify_control_line(&line) {
                        if matches!(
                            serde_json::from_str::<Response>(&text).expect("response parses"),
                            Response::Status(_)
                        ) {
                            let _ = served_tx.send(true);
                            return;
                        }
                    }
                }
                Err(e)
                    if matches!(
                        e.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                    ) => {}
                Err(_) => break,
            }
        }
        let _ = served_tx.send(false);
    });

    // Drive PUSHES distinct catalog changes in LOCKSTEP — one event, then wait for
    // the version to move — so the count is exact with no sleeping anywhere.
    //
    // A stall is reported OUT OF THE LOOP rather than by a bare `assert!`, for the
    // reason spelled out at the verdict block: this is the one failure path that can
    // run while the handler is wedged, and a panic here would drop `netd` into a join
    // that never returns (and leave the flood/reader threads spinning on a dead socket
    // for the rest of the binary, because nothing would have set `stop`).
    let mut stalled_at: Option<u64> = None;
    for i in 0..PUSHES {
        announce.push_alive("go2", Some(&format!("/flood/{i}")));
        if !wait_until(|| netd.catalog_version() > i, Duration::from_secs(10)) {
            stalled_at = Some(i);
            break;
        }
    }

    stop.store(true, Ordering::SeqCst);
    if let Some(i) = stalled_at {
        std::mem::forget(netd);
        panic!(
            "catalog change {i} never published — the announce watch is not keeping up, \
             so this arm would be measuring nothing"
        );
    }
    let served = served_rx
        .recv_timeout(Duration::from_secs(30))
        .unwrap_or(false);

    let max_drain = netd.conn_waker_max_drain();
    // The same drains SUMMED rather than maxed — the half the high-water
    // mark structurally cannot see (a drain that never ran leaves the mark at 0).
    let total_drain = netd.conn_waker_total_drain();
    let writes = flood_writes.load(Ordering::Relaxed);
    let flooded = writes * FLOOD_BATCH as u64;
    let write_failed = flood_write_failed.load(Ordering::SeqCst);
    // `conn_push_wakes` counts the control-loop waits that ended on the PUSH arm —
    // i.e. the polls at which the socket had NOTHING pending. It is the anti-vacuity
    // oracle; see the block below.
    let push_arm_wakes = netd.conn_push_wakes();
    let pokes_per_push_arm_drain = PUSHES / (push_arm_wakes + 1);
    println!(
        "waker backlog high-water mark = {max_drain} bytes after {PUSHES} \
         pushes, {total_drain} of {PUSHES} poked bytes drained in total (flood: \
         {writes} writes = {flooded} bytes accepted across {} handler read passes; \
         push-arm wakes = {push_arm_wakes}, i.e. {pokes_per_push_arm_drain} pokes per \
         interval between them)",
        netd.conn_read_loop_iterations()
    );

    // ---- ANTI-VACUITY -------------------------------------------
    //
    // The ceiling below is only a claim about the DESIGN if the socket really was
    // kept readable throughout: on an IDLE connection the push arm wins every tie,
    // the drain-on-`Push`-only variant drains each poke as it lands, and its
    // high-water mark is 1 — so a run whose flood did not flood passes the ceiling
    // while measuring nothing.
    //
    // An absolute throughput floor (`flooded >= 4 MiB`) is the WRONG guard for this:
    // it fires against runs whose flood is fine
    // (one such run reported 2,949,120 bytes). Three things are wrong with it, all
    // MEASURED rather than argued:
    //
    //  * The byte count is not the FLOOD thread's throughput at all — it is the
    //    DAEMON's. `write_all` blocks on a full socket, so the writer advances only
    //    as fast as the handler's 4 KiB-per-pass read drains it: across 28 runs at
    //    five load levels `flooded` came out at `4096 x conn_read_loop_iterations`
    //    to within 1.2% EVERY time (e.g. 9,502,720 = 4096 x 2320 against 2323
    //    iterations; 1,900,544 = 4096 x 464 against 467). So a "floor on the flood"
    //    is a floor on the DAEMON's scheduler share — a load-dependent quantity
    //    asserted as if it were load-independent, the loaded-runner class.
    //  * Such a floor is therefore reachable by CPU load alone
    //    (driven, not inferred). Plain `yes` spinners against the test
    //    binary, no QoS games: at 32 spinners 4 of 4 runs fall under it (3.00-3.38 MiB)
    //    and at 48 spinners 4 of 4 do (1.81-2.62 MiB), bracketing the
    //    2,949,120 bytes above. Handler passes fall 2323 -> 467
    //    while the WALL stays at 2.15-2.26 s in every one of them — the drive loop
    //    is paced by the coalescing window and the scripted stream's 2 ms polls, so
    //    it does not stretch under contention while the handler's pass count does.
    //    `push_arm_wakes` is **0** on all 20 of those runs, i.e. every run such a
    //    floor rejects is measuring the property perfectly well.
    //  * It has no real margin. An idle desk does not
    //    "write hundreds of MiB", and a starved one does not "clear this by orders of
    //    magnitude"; MEASURED, an idle desk writes 8.9-9.4 MiB — 2.2x the floor, not
    //    orders of magnitude — and 1.8 MiB is what a starved one reaches.
    //
    // The guard is instead the PRECONDITION itself, read off the daemon rather than
    // off a byte count, in three parts:
    //
    //  (a) the writer was never REJECTED by the socket — the dead-flood shape.
    //      NOT a boolean no scheduler can move:
    //      `Client::connect_with_timeout` installs a 3 s `SO_SNDTIMEO` and `try_clone`
    //      dups the descriptor onto the SAME socket, so the option applies and
    //      `write_all` does not retry a timed-out write — a sufficiently starved
    //      handler would surface here as a "REJECTED" verdict. The margin is >20x and
    //      is derivable from this arm's OWN worst recorded throughput rather than from
    //      a separate measurement: the slowest run of the 85-run sweep below accepted
    //      1.00 MiB across its ~2.2 s wall, i.e. ~137 ms for a 64 KiB batch against a
    //      3 s deadline, and no run has reached it. The timeout
    //      is KEPT rather than dropped from the clone, because it is what bounds
    //      `flood.join()` at the end of this test: against a genuinely wedged handler
    //      an untimed `write_all` never returns and a FAILING binary HANGS instead of
    //      going red — the exact class this arm is written around;
    //  (b) the writer completed at least ONE 64 KiB batch, so it genuinely ran; one
    //      batch is 16 daemon read passes, and the 2,949,120-byte run above did
    //      45 of them;
    //  (c) the push arm essentially never won, which is what "the socket stayed
    //      readable" means at the seam that decides it.
    //
    // WHAT (c) ACTUALLY DOES. It is not "the load-bearing one", and an arithmetic
    // that places it as a bound on the variant is
    // FALSE. It is an anti-vacuity guard against a flood that ran but left the socket
    // EMPTY most of the time, and it is good at precisely that: DRIVEN with the flood
    // capped at 0 / 1 / 64 batches it reads 160 / 158 / 82 push-arm wakes against 0-1
    // on a healthy run. What it is not is a bound on the variant's high-water mark:
    //
    //   With `w` push-arm wakes the pokes partition into `w + 1` intervals, but the
    //   variant performs only `w` DRAINS — the interval AFTER the last wake is never
    //   drained and never reaches the `fetch_max` at all. `conn_waker_max_drain` is
    //   therefore the max over the FIRST `w` intervals, and when the wakes cluster
    //   EARLY it sits far BELOW the `PUSHES / (w + 1)` average one might expect it
    //   never to fall under. MEASURED across variant runs reporting exactly one
    //   push-arm wake, `max_drain` came out 160, 160, 160, 160, 73 and 19 — the
    //   POSITION of that one wake, not an average of anything. The 19 is a variant run
    //   that PASSES (c) and the ceiling: (c) computes 160 / 2 = 80 > 32 and clears, the ceiling clears at
    //   19 <= 32, and only the total-drain floor below stands in its way.
    //
    // Rate, recorded so the size of that hole is on the record rather than implied:
    // ONE undetected case in 118 variant runs (30 at ambient load ~70, 40 unloaded, 48 under
    // 8 / 32 / 96 spinners). And (c) fired on ZERO of those 118 — every kill came from
    // the ceiling or a zero-drain guard. So (c) earns its place
    // as the vacuity guard it is, and as the `w <= 3` term the verdict block's
    // arithmetic consumes; it is not what catches the broken variant.
    //
    // (c) is load-SAFE where a byte floor is not, but the reason is EMPIRICAL and is
    // stated as such because the mechanism cuts both ways: the flood writer blocks
    // on a full socket buffer, so a starved HANDLER leaves that buffer FULLER (fewer
    // wakes), while a starved WRITER would leave it emptier (more). Which dominates
    // is a measurement, not an argument. MEASURED it is the first, decisively — 0 on
    // 85 of 85 runs spanning an idle desk to 96 `yes` spinners on 16 cores, with a
    // single observation of 1 during a concurrent cargo build. Contrast the byte
    // count over those same 85 runs: it fell to 1.00 MiB, i.e. 63 of them would
    // fail a 4 MiB floor while every one of them measured the property.
    //
    // The bar is `PUSHES / (push_arm_wakes + 1) > MAX_DRAIN_CEILING`, i.e.
    // `push_arm_wakes <= 3` at PUSHES = 160 and a ceiling of 32. Its exact position is
    // not delicate, which is the point of recording the whole distribution: healthy
    // runs sit at 0-1 and the vacuous cases sit at 82-160 (DRIVEN, not reasoned — a
    // flood thread capped at 0 / 1 / 64 batches yields 160 / 158 / 82 on an idle
    // desk), so anything in 3..=80 separates them. 3 is chosen because that is the
    // value the verdict block's total-drain arithmetic needs — see there.
    let vacuous: Option<String> = if write_failed {
        Some(format!(
            "the flood thread's socket REJECTED a write after {writes} batches — the \
             cloned handle or the connection died under it, so the connection went \
             idle and the high-water mark below is measuring nothing"
        ))
    } else if writes == 0 {
        Some(format!(
            "the flood thread completed ZERO {FLOOD_BATCH}-byte writes — it never \
             ran, so the connection was idle and the high-water mark below would \
             pass against the drain-placement mutant too"
        ))
    } else if pokes_per_push_arm_drain <= MAX_DRAIN_CEILING {
        Some(format!(
            "the push arm won {push_arm_wakes} of this run's waits, so the socket \
             was EMPTY that often and a mutant draining only on that arm would have \
             averaged {pokes_per_push_arm_drain} pokes per drain — at or below the \
             {MAX_DRAIN_CEILING}-byte ceiling, i.e. this run could not have \
             distinguished the two designs. Not a throughput problem: the flood \
             accepted {writes} writes ({flooded} bytes). Something kept the socket \
             DRAINED between pokes"
        ))
    } else {
        None
    };

    // A WEDGED handler makes `RunningNetd::drop` hang on its join, so a FAILING
    // assertion must not be allowed to become a hung binary (the hung-test class).
    // Leak the daemon instead and report.
    //
    // SCOPE: NOT "every failure path
    // takes this route", which is false of five panics in this same function. What
    // is true is that every failure path that can run WHILE A WEDGE IS POSSIBLE takes
    // it — the drive loop (routed above) and the two verdicts below. The other panics
    // are: `Client::connect`, `subscribe` and the `try_clone().expect(..)`, all of
    // which run before the first poke exists, so there is no accumulated backlog for a
    // handler to park on; and the two `join`s at the very end, which are reached only
    // AFTER the verdict has proved the handler still serving, and which are bounded
    // anyway (the flood's 3 s write timeout, and a reader that has already returned).
    //
    // THE PROPERTY IS JUDGED FIRST, and the order is load-bearing rather than
    // cosmetic. An anti-vacuity guard exists to qualify a PASS — "was this green run
    // measuring anything?" — so on a run that already FAILED the property there is
    // nothing left to qualify, and reporting vacuity instead misattributes the
    // failure. MEASURED on the drain-on-Push-only variant: it produces `max_drain` = 64
    // with `push_arm_wakes` = 10 (that variant leaves the pipe permanently readable,
    // so the few polls at which the socket is momentarily empty take the push arm,
    // where the loop-top drain's extra drain per pass keeps that count at 0). Checking
    // vacuity first would therefore announce "this run could not have distinguished the
    // two designs" about the one run in the suite that has just distinguished them.
    //
    // THE TOTAL-DRAIN FLOOR is the second half of the property, and it
    // exists because the high-water mark alone leaves a hole the ceiling cannot see.
    // `conn_waker_max_drain` is a `fetch_max` over what a drain CONSUMED, so it only
    // moves when a drain RUNS; that variant runs one only after a push-arm wake, so a
    // run whose wakes land early (or not at all) reports a small mark that satisfies
    // the ceiling while the pipe kept everything after them. MEASURED on that variant:
    // `max_drain` = 0 / 77 / 22 over one set of three runs, and 19 with a single early
    // push-arm wake on the survivor described above — the ceiling catching two of
    // those four.
    //
    // `conn_waker_total_drain` is the same drains SUMMED, and it closes it
    // ARITHMETICALLY rather than statistically:
    //
    //   * that variant performs exactly one drain per push-arm wake, so it performs `w`
    //     of them;
    //   * clearing (c) requires `PUSHES / (w + 1) > MAX_DRAIN_CEILING`, i.e. `w <= 3`;
    //   * clearing the ceiling requires every one of those `w` drains to be
    //     `<= MAX_DRAIN_CEILING`, so its total cannot exceed `3 * 32 = 96`;
    //   * this floor is `PUSHES - MAX_DRAIN_CEILING = 128 > 96`.
    //
    // So no run of that variant can clear all three at once, whatever the scheduler does with
    // it. The production code meanwhile drains at the top of EVERY pass, so its total is
    // exactly `PUSHES`: the last poke strictly precedes `stop`, which precedes the
    // reader's Status line, which precedes the pass that reads it — and that pass
    // drains first. MEASURED at 160 of 160 on six runs, and the floor sits exactly
    // `MAX_DRAIN_CEILING` below that and `MAX_DRAIN_CEILING` above that variant's bound.
    //
    // The drain-on-Push-only variant over 24 runs: ALL 24 caught, in
    // two shapes — and the second is a DIRECT observation of this floor doing work the
    // ceiling cannot: 16 runs took one LATE push-arm wake (`max_drain` 160, total 160)
    // and 1 took two (max 111), all killed by the ceiling; the other 7 took NO push-arm
    // wake at all, so no drain ever ran and the run reported `max_drain` = 0 with
    // `still serving = true` — the ceiling PASSES that (0 <= 32) and the floor is the
    // only term that fires. The anti-tautology direction is driven too: an unwired
    // counter (the `fetch_add` removed, everything else intact) fails the production code at
    // "0 of 160", so this assertion cannot be satisfied by a counter that ships inert.
    //
    // SCOPE: the 1-in-118 survivor — a single EARLY wake, small nonzero mark —
    // has not been observed with this counter compiled in, so the kill for that exact shape
    // rests on the arithmetic above plus the measured `PUSHES` baseline rather than on
    // a direct observation of it being caught. What WAS observed directly is that variant's
    // total tracking the position of its last wake (160 when late, 0 when there was
    // none, 19 in the survivor's shape) against a production-code 160 on every one of 14
    // runs.
    //
    // It also SUBSUMES a separate `max_drain == 0` guard:
    // both statistics move on the same `drain.bytes > 0`, so a zero mark implies a
    // zero total, which is 128 short of this floor. Such a guard would be
    // unreachable, so there is none.
    if !served || max_drain > MAX_DRAIN_CEILING || total_drain < MIN_TOTAL_DRAIN {
        std::mem::forget(netd);
        panic!(
            "waker backlog high-water mark was {max_drain} bytes (ceiling \
             {MAX_DRAIN_CEILING}) and {total_drain} of {PUSHES} poked bytes were ever \
             drained (floor {MIN_TOTAL_DRAIN}) after {PUSHES} pushes, still serving = \
             {served} — the waker is drained only when the push arm happens to win, so \
             a continuously-readable socket lets the pipe accumulate until a drain \
             meets an exact multiple of the 64-byte sink and parks forever"
        );
    }
    if let Some(why) = vacuous {
        std::mem::forget(netd);
        panic!("ANTI-VACUITY: {why}");
    }
    flood.join().expect("flood writer");
    reader.join().expect("flood reader");
}

/// Every connection-handler exit path releases the push waker.
///
/// The waker is armed once per CONNECTION and released by `cleanup_connection`, which
/// the design requires to keep running on all four exit paths. A missed
/// release is a LEAK, not a wrong answer: netd is spawned once and lives for the
/// machine's uptime, so it would accumulate a pipe fd PAIR per dead consumer and
/// eventually fail to arm anything at all — a failure mode nothing else in this suite
/// can see, because the daemon keeps answering correctly the whole way down.
///
/// Three exit paths are drivable from a client and are each driven here: a clean EOF,
/// an ABRUPT drop mid-exchange, and the OVERSIZED-LINE refusal (which breaks the loop
/// from inside the read arm). The two WRITE-failure paths need a peer that stops
/// reading until its socket buffer fills; they share the same `cleanup_connection`
/// call, and the count returning to zero here is what proves the release is on the
/// shared path rather than bolted onto one arm.
#[test]
fn every_connection_exit_path_releases_its_push_waker() {
    let (netd, _dir, sock, _announce) = start_daemon("wkleak");
    assert_eq!(netd.catalog_waker_count(), 0, "nothing connected yet");

    // Path 1 — a clean EOF (the client closes after a normal exchange).
    {
        let (mut c, _) = Client::connect(&sock);
        c.subscribe(1);
        assert!(
            wait_until(|| netd.catalog_waker_count() == 1, Duration::from_secs(10)),
            "the connection must ARM a waker (else every arm below is vacuous)"
        );
    }
    assert!(
        wait_until(|| netd.catalog_waker_count() == 0, Duration::from_secs(10)),
        "a clean EOF must release the waker"
    );

    // Path 2 — an ABRUPT drop with no subscribe at all (the crash-safe shape).
    {
        let (_c, _) = Client::connect(&sock);
        assert!(
            wait_until(|| netd.catalog_waker_count() == 1, Duration::from_secs(10)),
            "a waker is armed per CONNECTION, before any subscription"
        );
    }
    assert!(
        wait_until(|| netd.catalog_waker_count() == 0, Duration::from_secs(10)),
        "an abrupt drop must release the waker"
    );

    // Path 3 — the OVERSIZED-LINE refusal: the daemon breaks the loop itself.
    {
        let (mut c, _) = Client::connect(&sock);
        assert!(
            wait_until(|| netd.catalog_waker_count() == 1, Duration::from_secs(10)),
            "armed before the offending write"
        );
        // A line far past the cap with NO newline — the daemon refuses and drops it.
        let huge = "x".repeat(2 * 1024 * 1024);
        let _ = c.stream.write_all(huge.as_bytes());
        let _ = c.stream.flush();
    }
    assert!(
        wait_until(|| netd.catalog_waker_count() == 0, Duration::from_secs(10)),
        "the oversized-line refusal path must release the waker too"
    );

    assert!(
        wait_until(|| netd.active_connections() == 0, Duration::from_secs(10)),
        "and every connection is gone (the waker count is not tracking something else)"
    );
}

/// THE debounce pin, at the WIRE: an attach graph's 75-topic announce burst arrives as
/// ONE line carrying all 75, not as 75 lines. Asserting the exact LINE COUNT is what
/// makes this a real pin — a per-event push would satisfy "the sidebar updates" just
/// as well while storming the consumer.
#[test]
fn a_seventy_five_topic_attach_burst_arrives_as_exactly_one_push() {
    let (netd, _dir, sock, announce) = start_daemon("burst");
    let (mut c, _) = Client::connect(&sock);
    c.subscribe(1);

    announce.push_alive("go2", None);
    for i in 0..75 {
        announce.push_alive("go2", Some(&format!("/attach/topic{i}")));
    }

    // Drain for several windows so a per-event implementation would have delivered
    // its storm by now.
    let pushes = c.drain_pushes(TEST_WINDOW * 12);
    assert_eq!(
        pushes.len(),
        1,
        "a 75-topic burst must coalesce into ONE push, got {} lines: {:?}",
        pushes.len(),
        pushes.iter().map(|p| p.version).collect::<Vec<_>>()
    );
    let push = &pushes[0];
    assert_eq!(push.topics_added, 75, "all 75 are reported in the one push");
    assert_eq!(push.robots_added, vec!["go2".to_string()]);
    assert_eq!(
        push.coalesced, 76,
        "the notification states exactly how many transitions it stands for \
         (75 topics + the identity token)"
    );
    assert_eq!(netd.catalog_version(), 1, "one push, one version bump");
}

/// A whole gateway dying drops EVERY token at once. The consumer must get ONE
/// robot-level removal (the whole-robot transition its sidebar renders), not 75
/// separate topic removals.
#[test]
fn a_dying_gateway_arrives_as_one_robot_level_removal() {
    let (_netd, _dir, sock, announce) = start_daemon("death");
    let (mut c, _) = Client::connect(&sock);
    c.subscribe(1);

    announce.push_alive("go2", None);
    for i in 0..20 {
        announce.push_alive("go2", Some(&format!("/t{i}")));
    }
    let up = c
        .next_push(Duration::from_secs(3))
        .expect("the arrival push");
    assert_eq!(up.robots_added, vec!["go2".to_string()]);

    // Session death: every token the gateway held goes at once.
    announce.push_lost("go2", None);
    for i in 0..20 {
        announce.push_lost("go2", Some(&format!("/t{i}")));
    }
    let pushes = c.drain_pushes(TEST_WINDOW * 12);
    assert_eq!(
        pushes.len(),
        1,
        "one session death = ONE push, got {}",
        pushes.len()
    );
    let down = &pushes[0];
    assert_eq!(
        down.robots_removed,
        vec!["go2".to_string()],
        "the whole-robot transition, not 20 topic removals"
    );
    assert!(down.robots_added.is_empty());
    assert_eq!(down.topics_removed, 20, "the churn is still reported");
    assert!(
        down.robots.is_empty(),
        "the authoritative post-change robot set is empty"
    );
}

/// THE back-compat pin — and the reason this feature is safe to ship into a
/// spawn-once daemon that long-lived consumers hold alive: a connection that did NOT
/// subscribe receives NOTHING unsolicited, so its request/response stream cannot be
/// desynced. An implementation that broadcast to every connection would fail here.
#[test]
fn a_connection_that_never_subscribed_receives_no_unsolicited_line() {
    let (_netd, _dir, sock, announce) = start_daemon("nosub");
    // Two connections: one subscribed (proving the events really are flowing — without
    // it this test would pass on a daemon whose watch is simply broken), one not.
    let (mut subscribed, _) = Client::connect(&sock);
    subscribed.subscribe(1);
    let (mut plain, _) = Client::connect_with_timeout(&sock, Duration::from_millis(150));

    announce.push_alive("go2", None);
    announce.push_alive("go2", Some("/tf"));

    assert!(
        subscribed.next_push(Duration::from_secs(3)).is_some(),
        "anti-tautology: the subscribed connection MUST get the push, or the \
         'no unsolicited line' assertion below proves nothing"
    );

    // The unsubscribed connection: nothing at all should have arrived on it.
    let stray = plain.drain_pushes(TEST_WINDOW * 6);
    assert!(
        stray.is_empty(),
        "an unsubscribed connection must receive NO unsolicited line, got {stray:?}"
    );

    // And it is still a working request/response connection — the real regression this
    // guards is a desync, which only shows when the next request is answered. The read
    // is retried under a deadline rather than taken once: this connection carries a
    // deliberately SHORT read timeout (so the "no unsolicited line" drain above is
    // quick), and a single `read_line` would then be a race with daemon scheduling
    // rather than an assertion about the protocol.
    plain.send(&Request::Status { id: 9 });
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut answered = false;
    while Instant::now() < deadline && !answered {
        let line = match plain.read_line() {
            Ok(line) => line,
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                continue
            }
            Err(e) => panic!("the connection died instead of answering: {e}"),
        };
        match classify_control_line(&line) {
            ControlLine::Response(text) => {
                let r: Response = serde_json::from_str(&text).expect("parses");
                match r {
                    Response::Status(s) => {
                        assert_eq!(s.id, 9, "the response correlates");
                        answered = true;
                    }
                    other => panic!("expected a status response, got {other:?}"),
                }
            }
            other => panic!("expected a response line, got {other:?}"),
        }
    }
    assert!(
        answered,
        "the status response must arrive on an unsubscribed connection"
    );
}

/// THE no-wedge pin (the daemon-wedge class this must not join): a subscriber that has
/// STOPPED READING must not block netd's announce watch or a healthy sibling
/// subscriber. The stalled consumer's socket buffer fills; the watch thread does a
/// mutex store and moves on.
#[test]
fn a_stalled_subscriber_never_blocks_a_healthy_sibling() {
    let (netd, _dir, sock, announce) = start_daemon("stall");

    // The stalled subscriber: subscribes, then never reads again.
    let (mut stalled, _) = Client::connect(&sock);
    stalled.subscribe(1);
    let healthy_conn = {
        let (mut healthy, _) = Client::connect(&sock);
        healthy.subscribe(2);
        healthy
    };
    let mut healthy = healthy_conn;
    assert!(wait_until(
        || netd.catalog_subscriber_count() == 2,
        Duration::from_secs(3)
    ));

    // Hammer the announce space. The stalled consumer reads NOTHING throughout.
    for round in 0..40 {
        announce.push_alive("go2", Some(&format!("/t{round}")));
        std::thread::sleep(TEST_WINDOW);
    }

    // The healthy sibling still receives changes.
    let pushes = healthy.drain_pushes(Duration::from_secs(3));
    assert!(
        !pushes.is_empty(),
        "a stalled sibling must not starve a healthy subscriber"
    );
    let last = pushes.last().expect("at least one");
    assert!(
        last.version >= 1,
        "the healthy consumer sees a real version, got {}",
        last.version
    );

    // And netd itself is still responsive on a THIRD connection — the daemon-level
    // liveness half of "no wedge".
    let (mut third, _) = Client::connect(&sock);
    third.send(&Request::Status { id: 7 });
    let line = third.read_line().expect("netd is still answering");
    assert!(
        matches!(classify_control_line(&line), ControlLine::Response(_)),
        "netd must still serve requests while a subscriber is stalled"
    );

    // Keep the stalled connection alive to the end so the scenario is what it claims.
    drop(stalled);
}

/// Connection lifecycle: a consumer that connects MID-STREAM is told the current state
/// (the snapshot) rather than having to infer it, and the version it is handed is one
/// it can compare against later pushes.
#[test]
fn a_mid_stream_subscriber_is_handed_the_current_snapshot() {
    let (_netd, _dir, sock, announce) = start_daemon("snapshot");

    // A first consumer drives the daemon to a known state.
    let (mut first, _) = Client::connect(&sock);
    first.subscribe(1);
    announce.push_alive("go2", None);
    announce.push_alive("go2", Some("/tf"));
    announce.push_alive("orin", None);
    let push = first.next_push(Duration::from_secs(3)).expect("a push");
    assert_eq!(push.robots, vec!["go2".to_string(), "orin".to_string()]);

    // A SECOND consumer arrives now — it missed everything above.
    let (mut late, _) = Client::connect(&sock);
    let snapshot = late.subscribe(2);
    assert_eq!(
        snapshot.robots,
        vec!["go2".to_string(), "orin".to_string()],
        "a mid-stream subscriber is told who is out there, not left to guess"
    );
    assert_eq!(
        snapshot.version, push.version,
        "and the version it starts from is the one the stream is at"
    );

    // A later change moves BOTH consumers forward from that shared version.
    announce.push_lost("orin", None);
    let a = first
        .next_push(Duration::from_secs(3))
        .expect("first sees it");
    let b = late
        .next_push(Duration::from_secs(3))
        .expect("late sees it");
    assert_eq!(a.version, snapshot.version + 1);
    assert_eq!(b.version, snapshot.version + 1);
    assert_eq!(a.robots_removed, vec!["orin".to_string()]);
    assert_eq!(b.robots_removed, vec!["orin".to_string()]);
}

/// Explicit degradation: a daemon with no announce space to watch (no network plane)
/// still ACCEPTS the subscription — refusing would be indistinguishable from an old
/// daemon — but says `watching: false` so the consumer keeps its own refresh path
/// instead of waiting forever on an event that cannot come.
#[test]
fn a_daemon_with_no_announce_space_says_so_rather_than_pretending() {
    let (netd, _dir, sock, _announce) = start_daemon_with("nowatch", true);
    let (mut c, _) = Client::connect(&sock);
    let snapshot = c.subscribe(1);
    assert!(
        !snapshot.watching,
        "the consumer must be told plainly that no push can arrive here"
    );
    assert!(snapshot.robots.is_empty());
    assert!(!netd.is_watching_announces());
    // The subscription is still registered — nothing is wedged or refused.
    assert_eq!(netd.catalog_subscriber_count(), 1);
}

/// The [`NoopAnnounceWatchPlane`] (what every older `start_with_planes` caller gets)
/// behaves exactly the same way — the pin that the additive default did not silently
/// change any existing daemon's behaviour.
#[test]
fn the_default_plane_of_a_pre_928_caller_reports_not_watching() {
    let (dir, sock) = unique_socket("noopplane");
    let netd = daemon::start_with_planes(
        sock.clone(),
        Arc::new(InertMirrorPlane),
        Arc::new(NoopEgressPlane),
        Arc::new(NoopQueryPlane),
        NetdConfig {
            idle_grace: LONG_GRACE,
            idle_watch_poll: Duration::from_millis(25),
            ..NetdConfig::default()
        },
    )
    .expect("daemon start");
    let (mut c, _) = Client::connect(&sock);
    let snapshot = c.subscribe(1);
    assert!(!snapshot.watching);
    assert!(!netd.is_watching_announces());
    // And the plane really is the no-op one (a direct call refuses).
    assert!(matches!(
        NoopAnnounceWatchPlane.watch().err(),
        Some(WatchError::NoNetwork)
    ));
    drop(dir);
}

/// Connection close IS the unsubscribe (the same crash-safe rule the demand plane
/// uses), so a dead consumer never leaves a push slot nobody drains.
#[test]
fn closing_the_connection_unsubscribes_it() {
    let (netd, _dir, sock, announce) = start_daemon("unsub");
    let (mut c, _) = Client::connect(&sock);
    c.subscribe(1);
    assert!(wait_until(
        || netd.catalog_subscriber_count() == 1,
        Duration::from_secs(3)
    ));
    drop(c);
    assert!(
        wait_until(
            || netd.catalog_subscriber_count() == 0,
            Duration::from_secs(5)
        ),
        "the connection close must unsubscribe it, still {} subscribed",
        netd.catalog_subscriber_count()
    );
    // The watch keeps running (it is daemon-scoped, not subscription-scoped) and a
    // change still bumps the version — a subscriber-less daemon publishes to nobody
    // rather than losing track of the world.
    announce.push_alive("go2", None);
    assert!(wait_until(
        || netd.catalog_version() >= 1,
        Duration::from_secs(3)
    ));
}

/// A re-subscribe on the SAME connection is idempotent (one subscriber, not two) and
/// re-reports the snapshot — a consumer that lost track can ask again cheaply.
#[test]
fn a_re_subscribe_is_idempotent_and_re_reports_the_snapshot() {
    let (netd, _dir, sock, announce) = start_daemon("resub");
    let (mut c, _) = Client::connect(&sock);
    c.subscribe(1);
    announce.push_alive("go2", None);
    let push = c.next_push(Duration::from_secs(3)).expect("a push");

    let again = c.subscribe(2);
    assert_eq!(again.id, 2);
    assert_eq!(again.version, push.version, "the same stream position");
    assert_eq!(again.robots, vec!["go2".to_string()]);
    assert_eq!(
        netd.catalog_subscriber_count(),
        1,
        "a re-subscribe on one connection is ONE subscriber"
    );
    assert_eq!(
        announce.watch_calls(),
        1,
        "and it does not re-declare the watch"
    );
}

/// A change observed while nobody has drained the previous push MERGES rather than
/// piling up: the one line the consumer eventually reads describes everything it
/// missed, and its version is the newest. That is the bounded-queue overflow policy —
/// lossless coalescing, never a wedge and never a silently dropped change.
#[test]
fn pushes_that_pile_up_behind_a_slow_consumer_merge_losslessly() {
    let (netd, _dir, sock, announce) = start_daemon("merge");
    let (mut c, _) = Client::connect(&sock);
    c.subscribe(1);

    // Drive several DISTINCT windows without reading anything.
    announce.push_alive("go2", None);
    std::thread::sleep(TEST_WINDOW * 3);
    announce.push_alive("orin", None);
    std::thread::sleep(TEST_WINDOW * 3);
    announce.push_alive("go2", Some("/tf"));
    std::thread::sleep(TEST_WINDOW * 3);
    assert!(
        wait_until(|| netd.catalog_version() >= 3, Duration::from_secs(3)),
        "three separate windows must have published three versions, got {}",
        netd.catalog_version()
    );

    // Now read. Everything that happened is described by what arrives.
    let pushes = c.drain_pushes(TEST_WINDOW * 6);
    assert!(!pushes.is_empty(), "the consumer is not starved");
    let last = pushes.last().expect("at least one");
    assert_eq!(
        last.version,
        netd.catalog_version(),
        "the delivered notification carries the NEWEST version, so a consumer that \
         refreshes on it is current"
    );
    assert_eq!(
        last.robots,
        vec!["go2".to_string(), "orin".to_string()],
        "and the authoritative post-change robot set"
    );
    // Nothing was lost: across everything delivered, both robots' arrivals and the
    // topic are accounted for.
    let all_added: Vec<String> = pushes
        .iter()
        .flat_map(|p| p.robots_added.iter().cloned())
        .collect();
    assert!(all_added.contains(&"go2".to_string()));
    assert!(all_added.contains(&"orin".to_string()));
    assert_eq!(
        pushes.iter().map(|p| p.topics_added).sum::<u64>(),
        1,
        "the one topic announce is reported exactly once across the delivered pushes"
    );
}

/// A re-delivered token (liveliness legitimately re-delivers; the history replay at
/// watch start is exactly that) must produce NO push — otherwise every reconnect would
/// storm every consumer with a change that did not happen.
#[test]
fn re_delivered_tokens_produce_no_push_at_all() {
    let (netd, _dir, sock, announce) = start_daemon("redeliver");
    let (mut c, _) = Client::connect(&sock);
    c.subscribe(1);

    announce.push_alive("go2", None);
    announce.push_alive("go2", Some("/tf"));
    let first = c
        .next_push(Duration::from_secs(3))
        .expect("the real change");
    assert_eq!(first.version, 1);

    // The SAME tokens again — a replay, not a change.
    for _ in 0..3 {
        announce.push_alive("go2", None);
        announce.push_alive("go2", Some("/tf"));
    }
    let stray = c.drain_pushes(TEST_WINDOW * 8);
    assert!(
        stray.is_empty(),
        "a replay of known tokens must push nothing, got {stray:?}"
    );
    assert_eq!(netd.catalog_version(), 1, "and must not bump the version");
}

/// The wire discriminator: a push is identifiable as one WITHOUT positional
/// assumptions — it carries `event` and no `id`; a response carries `id` and no
/// `event`. This is what lets a reader stay in sync on a stream that carries both.
#[test]
fn a_push_is_structurally_distinguishable_from_a_response_on_the_wire() {
    let (_netd, _dir, sock, announce) = start_daemon("classify");
    let (mut c, _) = Client::connect(&sock);
    c.subscribe(1);
    announce.push_alive("go2", None);

    // Read RAW lines and classify by shape, not by order.
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut raw_push: Option<String> = None;
    while Instant::now() < deadline && raw_push.is_none() {
        if let Ok(line) = c.read_line() {
            let v: serde_json::Value = serde_json::from_str(line.trim()).expect("json");
            if v.get("event").is_some() {
                raw_push = Some(line);
            }
        }
    }
    let raw = raw_push.expect("a push line");
    let v: serde_json::Value = serde_json::from_str(raw.trim()).expect("json");
    assert_eq!(
        v.get("event").and_then(|e| e.as_str()),
        Some(CATALOG_CHANGED_EVENT)
    );
    assert!(
        v.get("id").is_none(),
        "a push carries no correlation id — that is the discriminator"
    );
    assert!(matches!(classify_control_line(&raw), ControlLine::Event(_)));

    // And a response is classified the other way.
    c.send(&Request::Status { id: 42 });
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut saw_response = false;
    while Instant::now() < deadline && !saw_response {
        if let Ok(line) = c.read_line() {
            if let ControlLine::Response(text) = classify_control_line(&line) {
                let v: serde_json::Value = serde_json::from_str(&text).expect("json");
                assert!(v.get("event").is_none(), "a response carries no event key");
                assert_eq!(v.get("id").and_then(|i| i.as_u64()), Some(42));
                saw_response = true;
            }
        }
    }
    assert!(saw_response, "the status response must arrive and classify");
}

/// A subscriber that goes away MID-BURST is cleaned up without disturbing anything —
/// the abrupt-disconnect twin of the graceful close, and the shape a crashing Studio
/// produces.
#[test]
fn a_subscriber_that_dies_mid_burst_is_cleaned_up_and_the_rest_continue() {
    let (netd, _dir, sock, announce) = start_daemon("diemid");
    let (mut survivor, _) = Client::connect(&sock);
    survivor.subscribe(1);
    let (mut victim, _) = Client::connect(&sock);
    victim.subscribe(2);
    assert!(wait_until(
        || netd.catalog_subscriber_count() == 2,
        Duration::from_secs(3)
    ));

    // Kill the victim abruptly while changes are flowing.
    for i in 0..10 {
        announce.push_alive("go2", Some(&format!("/t{i}")));
    }
    let victim_fd = victim.stream.try_clone().expect("clone");
    drop(victim);
    drop(victim_fd);

    for i in 10..20 {
        announce.push_alive("go2", Some(&format!("/t{i}")));
    }
    assert!(
        wait_until(
            || netd.catalog_subscriber_count() == 1,
            Duration::from_secs(5)
        ),
        "the dead subscriber must be reaped, still {} subscribed",
        netd.catalog_subscriber_count()
    );
    let pushes = survivor.drain_pushes(Duration::from_secs(2));
    assert!(
        !pushes.is_empty(),
        "the survivor keeps receiving after a sibling dies"
    );
}

/// A subscribed connection can STILL issue ordinary requests — subscribing does not
/// make the connection write-only. (netd's own `NetdClient::subscribe_catalog`
/// deliberately consumes the client instead, because a plain request/response reader
/// cannot stay in sync on such a stream; this pins that the DAEMON is not the reason.)
#[test]
fn a_subscribed_connection_still_serves_ordinary_requests() {
    let (_netd, _dir, sock, announce) = start_daemon("mixed");
    let (mut c, _) = Client::connect(&sock);
    c.subscribe(1);
    announce.push_alive("go2", None);
    // Wait for the push to be queued so a response and a push really do interleave.
    assert!(c.next_push(Duration::from_secs(3)).is_some());

    c.send(&Request::Status { id: 5 });
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut got = None;
    while Instant::now() < deadline && got.is_none() {
        if let Ok(line) = c.read_line() {
            if let ControlLine::Response(text) = classify_control_line(&line) {
                got = Some(serde_json::from_str::<Response>(&text).expect("parses"));
            }
        }
    }
    match got.expect("a status response") {
        Response::Status(s) => {
            assert_eq!(s.id, 5);
            assert_eq!(s.active_connections, 1);
        }
        other => panic!("expected status, got {other:?}"),
    }
}

/// Anti-tautology control for the whole file: with NO announce transitions at all, a
/// subscribed connection receives NOTHING. Without this, every "exactly N pushes" arm
/// above would also pass against a daemon that pushed on a timer.
#[test]
fn a_quiet_announce_space_pushes_nothing() {
    let (netd, _dir, sock, _announce) = start_daemon("quiet");
    let (mut c, _) = Client::connect_with_timeout(&sock, Duration::from_millis(150));
    let snapshot = c.subscribe(1);
    assert!(snapshot.watching);
    let stray = c.drain_pushes(TEST_WINDOW * 12);
    assert!(
        stray.is_empty(),
        "a quiet announce space must produce no push, got {stray:?}"
    );
    assert_eq!(netd.catalog_version(), 0, "and no version bump");
}

/// The oversized-line guard still applies on a subscribed connection — subscribing
/// does not open a hole in the daemon's input hardening.
#[test]
fn the_oversized_line_guard_still_applies_to_a_subscribed_connection() {
    let (_netd, _dir, sock, _announce) = start_daemon("oversize");
    let (mut c, _) = Client::connect(&sock);
    c.subscribe(1);
    // Write a chunk that exceeds nothing yet but has no newline; the daemon must not
    // treat the connection as broken. (The full cap is pinned in daemon_e2e_test; this
    // arm only proves a subscribed connection is still parsed the same way.)
    c.stream
        .write_all(b"{\"method\":\"status\"")
        .expect("write");
    c.stream.flush().expect("flush");
    std::thread::sleep(Duration::from_millis(50));
    c.stream.write_all(b",\"id\":11}\n").expect("write");
    c.stream.flush().expect("flush");
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut ok = false;
    while Instant::now() < deadline && !ok {
        if let Ok(line) = c.read_line() {
            if let ControlLine::Response(text) = classify_control_line(&line) {
                if let Ok(Response::Status(s)) = serde_json::from_str::<Response>(&text) {
                    assert_eq!(s.id, 11);
                    ok = true;
                }
            }
        }
    }
    assert!(ok, "a split request line is still assembled and answered");
}
