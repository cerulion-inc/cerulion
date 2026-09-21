// SPDX-License-Identifier: AGPL-3.0-only
//! The frame-drain loop BLOCKS ON A WAKE instead of polling.
//!
//! # What is under test, and what the harness deliberately is not
//!
//! The desk chain was measured at **11.8 ms p50 / 19.4 ms p90** of SHM
//! residency between netd's re-inject and vizd's drain, against **78 µs** of
//! real work. Every one of those milliseconds is this loop's poll cadence. The wake
//! path replaces the sleep with a block on the tap's own wake listener, keeping the
//! interval as the timeout.
//!
//! **LOCAL topics get the wake too**, so this
//! file covers BOTH classes and the stimulus is the production shape of
//! each:
//!
//! * REMOTE — the publisher is [`TransportManager::create_ingress_publisher`],
//!   netd's own desk-side mirror publisher, `arm_publish_raw_notify`-armed by
//!   construction and never elision-armed; the attach is the real
//!   `attach {topic, robot}` verb;
//! * LOCAL — the publisher is `create_publisher`, the GRAPH publisher shape
//!   (the one notify elision arms, and the one the wake path bills),
//!   publishing through `loan_proxy` + `OutputProxy::Drop` because that is the
//!   only local path that NOTIFIES; the attach is the plain `attach {topic}`
//!   verb.
//!
//! In both cases the `WakeMode` decision is taken by production code, never by
//! the test — which is what makes a per-seam revert visible here.
//!
//! What is NOT here is the zenoh hop: `AcceptingDemandPlane` answers the demand
//! and the test injects into the mirror directly. That is deliberate: the
//! wake is about the SHM residency AFTER the re-inject, and
//! `vizd_e2e_test::schema_less_remote_attach_catalog_resolves_seeds_and_taps`
//! already pins the full P→gateway→netd→mirror chain. Adding a network hop here
//! would put a variance term into a latency measurement without adding a
//! property.
//!
//! # Assertion discipline — per arm (the loaded-runner lesson)
//!
//! **NO arm in this file asserts a wall in units of `DEFAULT_POLL_INTERVAL`.**
//! macOS CI runners execute at BACKGROUND QoS, where timer coalescing charges
//! its slack PER WAKEUP: a nominal 150 ms interval was measured taking
//! **1100–1696 ms** under `taskpolicy -b`, and a wall assertion in this file FAILED
//! in CI at **87.287 ms against a 16 ms interval** on an arm whose fallback had
//! worked perfectly (`wake fires during the window: 0`). Only the wall lied.
//! Locally, under `taskpolicy -b`, this reproduces at 67.203 ms with a wall assertion.
//!
//! So each arm carries its discipline explicitly:
//!
//! | Arm | Load-bearing assertion | Why load cannot invert it |
//! |---|---|---|
//! | headline residency | `poll_loop_wake_fires >= FRAMES` | a fire is a QUEUED notification; a late-scheduled thread still returns FIRED. Load delays it, never erases it |
//! | headline residency | `wake_waits`/`backlog_passes` FLOORS | counters load can suppress, never inflate |
//! | no-notify fallback | the frame IS drained, within SECONDS | a liveness ceiling; a drain-only-on-fire loop never drains it at all |
//! | no-notify fallback | `wake_fires` UNCHANGED | exact, and the stimulus's own anti-vacuity check |
//! | LOCAL residency | `poll_loop_wake_fires >= FRAMES - 1` | same as the headline: a queued fire cannot be erased by load |
//! | LOCAL storm | demotion/pacing COUNTS | counters load can delay, never suppress |
//! | quiet spin bound | iterations vs a nominal DERIVED from the MEASURED elapsed | coalescing lengthens sleeps, so iterations fall — the SAFE direction |
//! | notifier storm | demotion/pacing COUNTS, iterations vs a measured nominal | same, plus a seconds-scale liveness deadline |
//!
//! The median residency is **MEASURED and PRINTED** (0.027–0.093 ms on an idle
//! desk against a 16 ms interval) but deliberately **not gated**: on a throttled
//! runner the timer path's residency stretches too, so no wall can separate the
//! two. `wake_fires` states the same claim in a form load cannot fake — a frame
//! that ENDED A WAIT BY FIRING IT was delivered at wake latency by construction,
//! and under the timer path that counter is 0.
//!
//! Hermetic: isolated per-test SHM root, a unique temp socket, a memory Rerun
//! sink and an injected [`DemandPlane`] — no `cerulion-netd`, no LAN.

#![cfg(unix)]

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::clock::VirtualClock;
use cerulion_core::transport::{network::NetworkConfig, TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_viz::schema_registry::builtin_walker;
use cerulion_viz::sink::SinkState;
use cerulion_viz::worker::VizLogWorker;
use cerulion_vizd::daemon::{start_with_demand_plane, DemandPlane, RunningDaemon};
use cerulion_vizd::DEFAULT_POLL_INTERVAL;

// ── Harness ─────────────────────────────────────────────────────────────────

/// How long an arm will wait for the drain loop to reach the wake path before
/// declaring the build broken. SECONDS, deliberately: it bounds a FAILURE, not a
/// measurement, and under `taskpolicy -b` a single coalesced pass can take
/// hundreds of milliseconds.
const SETTLE_BOUND: Duration = Duration::from_secs(10);

/// A demand plane that ACCEPTS every demand and serves no queries. The tests
/// pin their schema on the attach (`"schema":"geometry_msgs/Vector3"`), which
/// the daemon resolves locally, so no catalog round trip is ever made.
///
/// The mirror this demand stands for is created by the TEST, with
/// `create_ingress_publisher` — the same call netd makes.
#[derive(Default)]
struct AcceptingDemandPlane;

impl DemandPlane for AcceptingDemandPlane {
    fn demand(&self, _robot: &str, _topic: &str, _schema_hash: u64) -> Result<(), String> {
        Ok(())
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

fn temp_socket(tag: &str) -> (PathBuf, PathBuf) {
    let dir = std::env::temp_dir().join(format!(
        "wk-{tag}-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("clock")
            .as_nanos()
    ));
    std::fs::create_dir_all(&dir).expect("temp dir");
    (dir.join("vizd.sock"), dir)
}

/// A per-test SHM root WITH a (lazy, scouting-off) network config — the daemon's
/// remote-attach arm refuses a `CERULION_VIZD_NETWORK=off` daemon, and the
/// session stays lazy because nothing here ever opens it.
fn isolated_transport(node_name: &str) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: node_name.to_string(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 64,
            network: Some(NetworkConfig::default()),
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init isolated transport")
}

fn memory_worker(recording_id: &str) -> VizLogWorker {
    let (rec, _storage) = rerun::RecordingStreamBuilder::new("wake")
        .recording_id(recording_id)
        .memory()
        .expect("memory sink");
    VizLogWorker::spawn(rec, builtin_walker(), SinkState::new()).expect("spawn worker")
}

struct Harness {
    daemon: RunningDaemon,
    socket: PathBuf,
    _dir: PathBuf,
    manager: Arc<TransportManager>,
}

fn start(tag: &str) -> Harness {
    let manager = isolated_transport(&format!("wake_{tag}"));
    let (socket, dir) = temp_socket(tag);
    let daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&manager),
        memory_worker(tag),
        builtin_walker(),
        None,
        Arc::new(AcceptingDemandPlane),
    )
    .expect("daemon starts");
    Harness {
        daemon,
        socket,
        _dir: dir,
        manager,
    }
}

/// A thread ringing a topic's event service as fast as it can
/// while publishing NOTHING: the notify-storm shape, and exactly the state a wake
/// can never pay for: every wait fires instantly and the drain finds nothing.
///
/// It rings through a SECOND `create_ingress_publisher` on the same topic and
/// calls `notify_sent_sample()` without ever publishing, so the stimulus is the
/// production notify path (never elision-armed) rather than a hand-built
/// iceoryx2 port.
struct NotifyFlood {
    stop: Arc<std::sync::atomic::AtomicBool>,
    handle: Option<std::thread::JoinHandle<u64>>,
}

impl NotifyFlood {
    fn start(mgr: &TransportManager, topic: &str) -> Self {
        use std::sync::atomic::{AtomicBool, Ordering};
        let ringer = mgr
            .create_ingress_publisher(topic, MaxSliceLen::const_new(64))
            .expect("a second publisher on the topic, used ONLY to ring");
        let stop = Arc::new(AtomicBool::new(false));
        let stop_c = Arc::clone(&stop);
        let handle = std::thread::spawn(move || {
            let mut rung = 0u64;
            while !stop_c.load(Ordering::Relaxed) {
                if ringer.notify_sent_sample().is_ok() {
                    rung += 1;
                }
                // Yield so a low-core runner still schedules the loop under
                // test: the point is a HIGH event rate, not starving it.
                std::thread::yield_now();
            }
            rung
        });
        Self {
            stop,
            handle: Some(handle),
        }
    }

    fn stop(mut self) -> u64 {
        self.stop.store(true, std::sync::atomic::Ordering::Relaxed);
        self.handle
            .take()
            .map(|h| h.join().unwrap_or(0))
            .unwrap_or(0)
    }
}

impl Drop for NotifyFlood {
    fn drop(&mut self) {
        self.stop.store(true, std::sync::atomic::Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// A `geometry_msgs/Vector3` wire frame — the 32-byte header + three f64s.
fn vector3_frame(seq: u32) -> Vec<u8> {
    use native_ros2_messages::geometry_msgs::Vector3;
    let mut buf = vec![0u8; WireHeader::SIZE + 24];
    let header = WireHeader {
        schema_hash: <Vector3 as cerulion_core::message::ShmMessage>::SCHEMA_HASH,
        total_size: buf.len() as u32,
        offset_table_offset: buf.len() as u32,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns: 10_000_000 * (seq as u64 + 1),
    };
    header.write_to_buf(&mut buf);
    for (i, v) in [seq as f64, 2.0, 3.0].iter().enumerate() {
        let at = WireHeader::SIZE + i * 8;
        buf[at..at + 8].copy_from_slice(&v.to_le_bytes());
    }
    buf
}

struct Client {
    writer: UnixStream,
    reader: BufReader<UnixStream>,
}

impl Client {
    fn connect(socket: &std::path::Path) -> Self {
        let deadline = Instant::now() + Duration::from_secs(5);
        let stream = loop {
            match UnixStream::connect(socket) {
                Ok(s) => break s,
                Err(_) if Instant::now() < deadline => {
                    std::thread::sleep(Duration::from_millis(10))
                }
                Err(e) => panic!("connect {}: {e}", socket.display()),
            }
        };
        let writer = stream.try_clone().expect("clone");
        let mut reader = BufReader::new(stream);
        let mut banner = String::new();
        reader.read_line(&mut banner).expect("banner");
        Client { writer, reader }
    }

    fn request(&mut self, line: &str) -> serde_json::Value {
        writeln!(self.writer, "{line}").expect("write");
        let mut resp = String::new();
        self.reader.read_line(&mut resp).expect("read");
        serde_json::from_str(&resp).expect("json")
    }
}

/// Spin (no sleep — this is the measurement) until the drain loop's frame
/// counter passes `target`, and return the wall it took. `None` if `bound`
/// expired, which every caller treats as a failure rather than a slow pass.
fn await_frames(daemon: &RunningDaemon, target: u64, bound: Duration) -> Option<Duration> {
    let started = Instant::now();
    while started.elapsed() < bound {
        if daemon.poll_loop_frames_drained() > target {
            return Some(started.elapsed());
        }
        std::hint::spin_loop();
    }
    None
}

/// Wait until the drain loop is demonstrably PARKED ON THE WAKE PATH — i.e. it
/// has completed at least one pass whose wait blocked on a wake set — and return
/// that count.
///
/// This replaces every `sleep(DEFAULT_POLL_INTERVAL * N)` settle in this file,
/// and the replacement is the point: a settle expressed as a MULTIPLE OF THE
/// INTERVAL is the timer-coalescing class in miniature. Under `taskpolicy -b` the loop's
/// own sleep coalesces, so a 3-interval (48 ms nominal) window reliably yielded
/// ZERO completed passes — MEASURED, 3 failures in 20 runs, on the arm that then
/// asserted the loop had been on the wake path.
///
/// A CONDITION cannot be inverted by load: contention delays when the loop gets
/// there, never whether. The bound is in SECONDS and exists only so a genuinely
/// broken build fails loudly instead of hanging.
fn await_wake_armed(daemon: &RunningDaemon, bound: Duration) -> u64 {
    let started = Instant::now();
    while started.elapsed() < bound {
        let waits = daemon.poll_loop_wake_waits();
        if waits > 0 {
            return waits;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    panic!(
        "the drain loop never blocked on a wake set within {bound:?} — the tap \
         carries no listener (the attach seam's WakeMode was reverted), or the \
         wake set failed to build"
    );
}

/// Median of a sample set (the order statistic every latency claim here is
/// stated on — a mean would be moved by one stalled sample).
fn median(mut v: Vec<Duration>) -> Duration {
    v.sort_unstable();
    v[v.len() / 2]
}

/// This topic's row from a `status` reply.
fn topic_status(client: &mut Client, id: u64, topic: &str) -> serde_json::Value {
    let resp = client.request(&format!(r#"{{"id":{id},"method":"status"}}"#));
    resp["topics"]
        .as_array()
        .and_then(|a| {
            a.iter()
                .find(|e| e.get("topic").and_then(|t| t.as_str()) == Some(topic))
        })
        .cloned()
        .unwrap_or_else(|| panic!("'{topic}' must appear in status: {resp}"))
}

/// Attach `topic` as a REMOTE topic of robot `go2` with a pinned built-in
/// schema, which is the production `WakeMode::Listener` path.
fn attach_remote(client: &mut Client, id: u64, topic: &str) -> serde_json::Value {
    client.request(&format!(
        r#"{{"id":{id},"method":"attach","topic":"{topic}","robot":"go2","schema":"geometry_msgs/Vector3"}}"#
    ))
}

// ===========================================================================
// THE HEADLINE — residency far below the poll interval, on a producer whose
// period is far ABOVE it (so the timer path cannot produce the answer by luck).
// ===========================================================================

/// The residency claim, stated as a CEILING on the median.
///
/// The producer emits once every `EMIT_PERIOD` (≫ `DEFAULT_POLL_INTERVAL`), so
/// under the earlier timer each frame would wait a uniform `[0, 16 ms]` —
/// median ~8 ms. Under a wake it waits a wake latency. The ceiling sits an order
/// of magnitude below the timer's median and two above the wake's, so neither a
/// fast runner nor a slow one can put the wrong implementation on the right side
/// of it.
///
/// The FLOOR assertions on `poll_loop_wake_waits` / `poll_loop_wake_fires` are
/// what make this a wake test rather than a timing coincidence: reverting
/// `attach_remote`'s `WakeMode::Listener` to `Timer` leaves the frames flowing
/// and the oracle intact, and drops both counters to 0.
#[test]
fn a_remote_topics_frames_are_drained_within_a_wake_not_within_a_poll_interval() {
    const EMIT_PERIOD: Duration = Duration::from_millis(120); // 7.5x the interval
    const FRAMES: usize = 12;
    // A LIVENESS ceiling, in SECONDS, not intervals: it says every frame was
    // drained, and nothing about how fast. The residency NUMBER is printed
    // instead of gated — see the module docs. Under `taskpolicy -b` the median
    // stays ~0.135 ms but the MAX reached 5.325 ms, so an interval-scaled
    // ceiling here is one contended runner away from the false fail.
    const DRAIN_LIVENESS_CEILING: Duration = Duration::from_secs(5);

    let h = start("hd");
    let topic = "/wake/headline";
    // The mirror, created exactly as netd creates it.
    let mut mirror = h
        .manager
        .create_ingress_publisher(topic, MaxSliceLen::const_new(1 << 16))
        .expect("netd's mirror-publisher shape");

    let mut client = Client::connect(&h.socket);
    let att = attach_remote(&mut client, 1, topic);
    assert_eq!(
        att["ok"].as_bool(),
        Some(true),
        "the remote attach must succeed — it is what asks for the wake: {att}"
    );

    // Wait for the loop to be demonstrably parked on the wake path — a CONDITION,
    // not an interval-scaled sleep (see `await_wake_armed`) — so the first
    // measured frame is not racing the attach.
    await_wake_armed(&h.daemon, SETTLE_BOUND);

    let mut residencies = Vec::with_capacity(FRAMES);
    for seq in 0..FRAMES {
        // Sit out the emit period FIRST, so the drain loop is genuinely parked
        // when the frame lands. Publishing back-to-back would measure the
        // backlog arm instead.
        std::thread::sleep(EMIT_PERIOD);
        let before = h.daemon.poll_loop_frames_drained();
        mirror
            .publish_raw(&vector3_frame(seq as u32))
            .expect("publish into the mirror");
        let waited = await_frames(&h.daemon, before, DRAIN_LIVENESS_CEILING)
            .expect("the drain loop must see every published frame");
        residencies.push(waited);
    }

    let med = median(residencies.clone());
    println!(
        "[wake-driven drain] frames={FRAMES} median={:.3}ms max={:.3}ms \
         (configured poll interval {:?}; wake_waits={} wake_fires={} backlog={})",
        med.as_secs_f64() * 1e3,
        residencies.iter().max().expect("nonempty").as_secs_f64() * 1e3,
        DEFAULT_POLL_INTERVAL,
        h.daemon.poll_loop_wake_waits(),
        h.daemon.poll_loop_wake_fires(),
        h.daemon.poll_loop_backlog_passes(),
    );

    // THE LOAD-BEARING CLAIM, and it is a COUNT, not a wall (see the module
    // docs): every published frame ENDED A WAIT BY FIRING IT. A frame delivered
    // that way was seen at wake latency BY CONSTRUCTION — there is no timeout in
    // its path — so this states the latency claim in the one form background-QoS
    // timer coalescing cannot fake. Under the timer path it is 0.
    //
    // A FLOOR, and load-safe in the strong sense: a notification is QUEUED on the
    // listener, so a thread descheduled for a second still returns FIRED when it
    // finally runs. Load delays the fire; it cannot erase it.
    // `FRAMES - 1`, and the minus one is ARITHMETIC, not slack: the counter is
    // posted at the BOTTOM of a pass while `await_frames` returns the moment the
    // frame count moves at its TOP, so the LAST frame's fire can still be in
    // flight when this reads. At most ONE pass is ever in flight, so at most one
    // fire is unposted — MEASURED as exactly that under `taskpolicy -b` (11 of
    // 12). The discrimination is untouched: the timer path posts ZERO.
    assert!(
        h.daemon.poll_loop_wake_fires() as usize >= FRAMES - 1,
        "every published frame must have ENDED a wait by FIRING it, bar at most \
         the one still in flight (got {} fires over {FRAMES} frames). A count near \
         0 means the timer, not the wake, is delivering — the tap carries no \
         listener (the attach seam's WakeMode was reverted) or the wake set failed \
         to build. Median residency this run: {:.3}ms against a {:?} interval.",
        h.daemon.poll_loop_wake_fires(),
        med.as_secs_f64() * 1e3,
        DEFAULT_POLL_INTERVAL
    );
    assert!(
        h.daemon.poll_loop_wake_waits() > 0,
        "the loop must actually have BLOCKED on the wake set"
    );
    // The BACKLOG arm is exercised too, and is not dead code: every pass that
    // drains a frame must skip its wait and re-drain, because the notify that
    // woke it has been consumed and any frames past `POLL_MAX` would otherwise
    // sit until a LATER frame's notify (or the timeout). A floor of one is
    // load-safe — contention can only coalesce frames into fewer draining
    // passes, never manufacture one.
    assert!(
        h.daemon.poll_loop_backlog_passes() > 0,
        "a pass that drained frames must skip its wait and re-drain (the \
         backlog-aware arm); 0 means the arm is unreachable"
    );
}

// ===========================================================================
// The timeout is a REAL fallback: a frame that never notifies is still drained.
// ===========================================================================

/// A frame for which NO wake can ever arrive must still be drained, one interval
/// later — the property that makes the new loop a strict SUPERSET of the polling
/// one (Principle #6).
///
/// The stimulus is exact rather than a race: `create_publisher`'s `publish_raw`
/// does NOT notify (only `create_ingress_publisher` arms that), so a
/// frame published through it is committed to SHM with no notification, ever.
/// The tap over it is nonetheless a `WakeMode::Listener` tap (a remote attach),
/// so the loop IS blocking on a wake set when the frame lands.
///
/// The reason this arm is worth its own test: a loop that
/// drained only when its wake FIRED would never see this frame at all. A loop
/// that waits before draining would see it one extra interval later. The bound
/// separates both from a healthy loop without depending on a microsecond-scale
/// interleaving.
///
/// (Pairing a remote attach with a locally-created graph topic is artificial —
/// deliberately. It is the only way to hold "the tap has a wake" and "this frame
/// carries no notify" true at the same time, which is exactly the state the
/// timeout exists for.)
#[test]
fn a_frame_that_never_notifies_is_still_drained_by_the_timeout_fallback() {
    // SECONDS, not intervals — the timeout under test is a `thread::sleep`, and
    // that is precisely what background-QoS coalescing stretches.
    const FALLBACK_LIVENESS_CEILING: Duration = Duration::from_secs(5);

    let h = start("rc");
    let topic = "/wake/silent";
    // NOT `create_ingress_publisher`: this publisher's `publish_raw` sends no
    // notification, so the wake listener can never fire for these frames.
    let mut silent = h
        .manager
        .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
        .expect("a publisher whose publish_raw does not notify");

    let mut client = Client::connect(&h.socket);
    let att = attach_remote(&mut client, 1, topic);
    assert_eq!(
        att["ok"].as_bool(),
        Some(true),
        "the remote attach must succeed — it is what arms the wake: {att}"
    );
    // Settle on a CONDITION, and take the wake-path evidence FROM it. Both halves
    // are deliberate. Sleeping a multiple of the interval did not reliably give
    // the loop one completed pass under `taskpolicy -b` (3 failures in 20), and
    // reading the counter AFTER the stimulus races the loop the other way — it
    // posts at the BOTTOM of a pass while `await_frames` returns at its TOP.
    let waits_before = await_wake_armed(&h.daemon, SETTLE_BOUND);
    let fires_before = h.daemon.poll_loop_wake_fires();
    let before = h.daemon.poll_loop_frames_drained();
    silent.publish_raw(&vector3_frame(0)).expect("publish");

    // The LOAD-BEARING assertion is that the frame is drained AT ALL, bounded in
    // SECONDS. A loop that drained only when its wake FIRED never sees this
    // frame — no notification exists for it — so a liveness ceiling is the whole
    // discrimination, and it needs no wall precision to make it.
    //
    // An interval-scaled bound FAILS under load (87.287 ms on a background-QoS
    // runner, reproduced locally under `taskpolicy -b` at 67.203 ms): background QoS
    // coalesces the very `thread::sleep` that IS the fallback under test, so the
    // arm gates on the one quantity the runner is free to stretch.
    let waited = await_frames(&h.daemon, before, FALLBACK_LIVENESS_CEILING).expect(
        "a frame that never notifies must STILL be drained — the timeout is the \
         fallback, and a loop that drains only on a fired wake never sees it",
    );
    println!(
        "[no-notify fallback] frame drained in {:.3}ms \
         (interval {:?}, wake fires during the window: {})",
        waited.as_secs_f64() * 1e3,
        DEFAULT_POLL_INTERVAL,
        h.daemon.poll_loop_wake_fires() - fires_before,
    );
    // ANTI-VACUITY: the wake really could not have delivered it. If a fire were
    // counted here the stimulus would be wrong and the arm would be measuring
    // the wake path under a "no-notify" label.
    assert_eq!(
        h.daemon.poll_loop_wake_fires(),
        fires_before,
        "the stimulus must be genuinely notification-free — a wake fired, so this \
         arm was not testing the timeout fallback at all"
    );
    // And the loop WAS on the wake path (not silently degraded to the timer),
    // which is what makes "the timeout is still the fallback" a claim about the
    // shipped loop rather than about the plain timer loop's.
    assert!(
        waits_before > 0,
        "the loop must have been blocking on a wake set when this frame landed — \
         otherwise the arm is measuring the plain poll timer, not the wake path's \
         fallback"
    );
}

// ===========================================================================
// A LOCAL topic gets the wake TOO.
// ===========================================================================

/// **The local wake, pinned where it is paid.** A LOCAL attach opens a wake
/// listener on a GENUINE GRAPH PUBLISHER — a producer this desk does not
/// own — and its frames are drained within a wake, not within a
/// poll interval.
///
/// This arm is the exact INVERSION of the earlier
/// `..._is_never_given_a_wake_and_still_delivers_on_the_timer`, deliberately
/// rather than by deletion: the property it asserted (`wake_waits == 0`) is now
/// FALSE, and the accurate record of that is an arm whose oracle moved,
/// not a gap where an arm used to be.
///
/// The stimulus is the production LOCAL shape end to end and every part of that
/// matters:
///
/// * the producer is `create_publisher` (the graph-publisher shape — the one
///   notify elision arms), not `create_ingress_publisher`;
/// * it publishes through `loan_proxy` + `OutputProxy::Drop`, the ONLY local
///   path that notifies (a raw `publish_raw` on this publisher never does —
///   that is armed for ingress publishers only), so a fired wake here is
///   proof the graph publisher really rings a vizd listener;
/// * the attach is the plain `attach {topic}` verb, so `attach_local` — not the
///   test — picks the `WakeMode`.
///
/// LOAD DISCIPLINE (the loaded-runner class): the load-bearing assertion is
/// `poll_loop_wake_fires >= FRAMES - 1`, a COUNT, not a wall. A fire is a
/// QUEUED notification, so a descheduled thread still returns FIRED; the timer
/// path posts ZERO. `FRAMES - 1` is arithmetic, not slack — the counter posts at
/// the BOTTOM of a pass while `await_frames` returns at its TOP, so at most one
/// fire is unposted. The residency median is MEASURED and PRINTED but not
/// gated: on a throttled runner the timer path's residency stretches too, and no
/// wall separates them.
#[test]
fn a_local_topic_is_given_a_wake_too_and_its_frames_are_drained_within_one() {
    const EMIT_PERIOD: Duration = Duration::from_millis(120); // 7.5x the interval
    const FRAMES: usize = 12;
    const DRAIN_LIVENESS_CEILING: Duration = Duration::from_secs(5);

    let h = start("lo");
    let topic = "/wake/local";
    // A GENUINE local producer (not a mirror): `create_publisher` is the graph
    // publisher shape, the one notify elision arms and the one the wake
    // path bills.
    let mut producer = h
        .manager
        .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
        .expect("local producer");

    let mut client = Client::connect(&h.socket);
    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true), "local attach: {att}");

    // The loop must be demonstrably parked on the wake path before the first
    // measured frame — a CONDITION, never an interval-scaled sleep. On a
    // timer-only local attach this call is what fails first, and it fails loudly:
    // a local-only daemon never reaches the wake path at all.
    await_wake_armed(&h.daemon, SETTLE_BOUND);

    let mut residencies = Vec::with_capacity(FRAMES);
    for seq in 0..FRAMES {
        // Sit out the emit period first, so the loop is genuinely parked when
        // the frame lands (back-to-back publishes measure the backlog arm).
        std::thread::sleep(EMIT_PERIOD);
        let before = h.daemon.poll_loop_frames_drained();
        // `loan_proxy` + Drop — the graph publisher's own commit path, and the
        // only local publish that NOTIFIES. `publish_raw` on this publisher
        // would leave the wake permanently silent and make the arm measure the
        // timeout fallback under a wake label.
        publish_local_vector3(&mut producer, seq as f64);
        let waited = await_frames(&h.daemon, before, DRAIN_LIVENESS_CEILING)
            .expect("the drain loop must see every locally published frame");
        residencies.push(waited);
    }

    let med = median(residencies.clone());
    println!(
        "[LOCAL wake-driven drain] frames={FRAMES} median={:.3}ms \
         max={:.3}ms (configured poll interval {:?}; wake_waits={} wake_fires={} \
         backlog={})",
        med.as_secs_f64() * 1e3,
        residencies.iter().max().expect("nonempty").as_secs_f64() * 1e3,
        DEFAULT_POLL_INTERVAL,
        h.daemon.poll_loop_wake_waits(),
        h.daemon.poll_loop_wake_fires(),
        h.daemon.poll_loop_backlog_passes(),
    );

    // The decision, as a load-safe count: every locally published frame ENDED A
    // WAIT BY FIRING IT, so it was seen at wake latency by construction.
    assert!(
        h.daemon.poll_loop_wake_fires() as usize >= FRAMES - 1,
        "a LOCAL topic's frames must end waits by FIRING them (got {} fires over \
         {FRAMES} frames). A count near 0 means `attach_local` reverted to \
         `WakeMode::Timer` — the local wake, undone. Median residency this \
         run: {:.3}ms against a {:?} interval.",
        h.daemon.poll_loop_wake_fires(),
        med.as_secs_f64() * 1e3,
        DEFAULT_POLL_INTERVAL
    );
    assert!(
        h.daemon.poll_loop_wake_waits() > 0,
        "a daemon whose only tap is LOCAL must still BLOCK on a wake set — that \
         is the whole of the local wake"
    );
    // Delivery is untouched by the wake — the same claim the earlier timer arm made
    // about the timer, which must survive the flip.
    assert_eq!(
        residencies.len(),
        FRAMES,
        "every published frame was drained — the wake changes WHEN a frame is \
         seen, never WHETHER"
    );
}

/// Publish one `geometry_msgs/Vector3` through the GRAPH publisher's own commit
/// path (`loan_proxy` + `OutputProxy::Drop`) — the only local publish that
/// NOTIFIES, and therefore the only one that can wake a local listener.
///
/// A raw `publish_raw` on a `create_publisher` sends no notification (that is
/// armed for ingress publishers only), so a local arm using it would be
/// measuring the timeout fallback under a wake label.
fn publish_local_vector3(
    publisher: &mut cerulion_core::transport::publisher::CerulionPublisher,
    x: f64,
) {
    use native_ros2_messages::geometry_msgs::Vector3;
    let mut proxy = publisher
        .loan_proxy::<Vector3>()
        .expect("loan_proxy on the local graph publisher");
    proxy.x = x;
    proxy.y = 2.0;
    proxy.z = 3.0;
}

// ===========================================================================
// The spin bound: the notify-storm lesson, stated as a counted ceiling.
// ===========================================================================

/// A wake-driven loop with a QUIET topic must not spin: its wait has to spend
/// the interval, so iterations over a window are bounded by the interval.
///
/// This is the arm a `timeout = 0` (or a backlog arm that skipped the wait
/// unconditionally) fails. It is stated as a CEILING on a COUNT because that is
/// the only shape that sees an event loop with an unspent budget — a period
/// register under-samples exactly when the loop outruns it.
#[test]
fn a_wake_driven_loop_on_a_quiet_topic_is_not_a_spin() {
    const WINDOW: Duration = Duration::from_secs(1);

    let h = start("sp");
    let topic = "/wake/spin";
    // The mirror exists and is TAPPED, but nothing is ever published into it, so
    // every pass drains nothing and must take the wake wait's full budget.
    let _mirror = h
        .manager
        .create_ingress_publisher(topic, MaxSliceLen::const_new(1 << 16))
        .expect("mirror publisher");
    let mut client = Client::connect(&h.socket);
    let att = attach_remote(&mut client, 1, topic);
    assert_eq!(att["ok"].as_bool(), Some(true), "remote attach: {att}");

    await_wake_armed(&h.daemon, SETTLE_BOUND);
    let before = h.daemon.poll_loop_iterations();
    let started = Instant::now();
    std::thread::sleep(WINDOW);
    let elapsed = started.elapsed();
    let iterations = h.daemon.poll_loop_iterations() - before;

    // The nominal rate is `WINDOW / interval` (~62). 4x that is a ceiling a
    // healthy loop cannot approach and an unspent-budget spin blows past by
    // orders of magnitude (a 0-timeout wait iterates in microseconds).
    let nominal = (elapsed.as_secs_f64() / DEFAULT_POLL_INTERVAL.as_secs_f64()) as u64;
    let ceiling = nominal * 4;
    println!(
        "[quiet spin bound] {iterations} iterations in {:.3}s \
         (nominal {nominal}, ceiling {ceiling}); wake_waits={} wake_fires={}",
        elapsed.as_secs_f64(),
        h.daemon.poll_loop_wake_waits(),
        h.daemon.poll_loop_wake_fires(),
    );
    assert!(
        iterations <= ceiling,
        "a wake-driven loop on a QUIET topic ran {iterations} passes in {:.3}s \
         against a nominal {nominal} — its wait is not spending its budget \
         (a zero/absent timeout, or an unconditional backlog skip)",
        elapsed.as_secs_f64()
    );
    // ANTI-VACUITY: the loop really was on the WAKE path, not the timer, so the
    // bound is about the thing it claims to bound.
    assert!(
        h.daemon.poll_loop_wake_waits() > 0,
        "the quiet loop must have blocked on the wake set at least once"
    );
    // A quiet topic yields nothing, so the backlog arm must never be taken —
    // the arm that, taken unconditionally, IS the spin.
    assert_eq!(
        h.daemon.poll_loop_backlog_passes(),
        0,
        "no frames were published, so no pass may have skipped its wait"
    );
}

// ===========================================================================
// CHUNK 5 — the wake that cannot pay for itself is paced, then dropped.
// ===========================================================================

/// A topic whose event service is RUNG without anything being published is the
/// state a wake can never pay for: every wait fires instantly, the drain finds
/// nothing, and the loop would spin at 100 % of a core (measured on the CLI
/// topic observer at ~3 000 000 events over 2.3 s).
///
/// Three things must happen, in order, and all three are asserted:
///
/// 1. the loop PACES itself past `WAKE_IDLE_CHURN_TOLERANCE` — the iteration
///    ceiling is what proves the budget is being spent;
/// 2. the topic's wake is DEMOTED past `WAKE_DEMOTE_STREAK`, so its producer
///    stops paying a notify for nothing;
/// 3. `status` SAYS so — `wake: false` on a topic that asked for one, which is
///    the only way an operator can tell a demoted row from a fast one.
///
/// The frames keep flowing throughout: after the demotion a real publish is
/// still drained, on the timer.
#[test]
fn a_wake_that_only_ever_rings_is_paced_then_dropped_and_the_topic_keeps_delivering() {
    let h = start("fl");
    let topic = "/wake/flood";
    let mut mirror = h
        .manager
        .create_ingress_publisher(topic, MaxSliceLen::const_new(1 << 16))
        .expect("mirror publisher");

    let mut client = Client::connect(&h.socket);
    let att = attach_remote(&mut client, 1, topic);
    assert_eq!(att["ok"].as_bool(), Some(true), "remote attach: {att}");
    // Before the storm: the row reports a LIVE wake.
    let st = topic_status(&mut client, 2, topic);
    assert_eq!(
        st["wake"].as_bool(),
        Some(true),
        "a healthy remote topic reports its wake: {st}"
    );

    let flood = NotifyFlood::start(&h.manager, topic);

    // Wait for the demotion, bounded in SECONDS. The streak needs 64
    // fired-but-empty passes and the loop paces past the churn tolerance, so the
    // wall is dominated by 48 coalescible sleeps: MEASURED 1.03 s on an idle
    // desk and 8.30 s under `taskpolicy -b`. The ceiling is set ~14x over the
    // throttled measurement rather than ~2x, because this is a LIVENESS bound —
    // it exists to fail a demotion that never happens, not to time one.
    let iters_before = h.daemon.poll_loop_iterations();
    let started = Instant::now();
    let deadline = started + Duration::from_secs(120);
    while h.daemon.poll_loop_wake_demotions() == 0 && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(10));
    }
    let took = started.elapsed();
    let iterations = h.daemon.poll_loop_iterations() - iters_before;
    let rung = flood.stop();

    println!(
        "[notify storm] demoted after {:.3}s / {iterations} passes \
         ({rung} events rung); paced_passes={} demotions={}",
        took.as_secs_f64(),
        h.daemon.poll_loop_wake_paced_passes(),
        h.daemon.poll_loop_wake_demotions(),
    );

    assert_eq!(
        h.daemon.poll_loop_wake_demotions(),
        1,
        "a wake that fired {} times running without delivering must be DROPPED \
         (it is billing its producer a notify per ring for nothing)",
        64
    );
    assert!(
        h.daemon.poll_loop_wake_paced_passes() > 0,
        "the loop must have PACED itself under the storm — without it a wake \
         that returns instantly never spends its budget and the loop is a spin"
    );
    // THE SPIN BOUND, over the whole storm window. A paced loop runs at roughly
    // the poll interval; an unpaced one runs as fast as the notifier rings.
    let nominal = (took.as_secs_f64() / DEFAULT_POLL_INTERVAL.as_secs_f64()) as u64;
    assert!(
        iterations <= nominal.max(1) * 20 + 200,
        "the loop ran {iterations} passes over {:.3}s of notifier storm against a \
         nominal {nominal} — the pacing is not bounding it",
        took.as_secs_f64()
    );
    // The flood really was a flood (anti-vacuity: a fixture that never rang
    // would make every assertion above about an idle daemon).
    assert!(
        rung > 1_000,
        "the fixture must genuinely have stormed the event service (rang {rung})"
    );

    // THE OPERATOR SURFACE: the row now says its wake is gone, which is the only
    // way to tell a demoted topic from one that never asked for a wake.
    let st = topic_status(&mut client, 3, topic);
    assert_eq!(
        st["wake"].as_bool(),
        Some(false),
        "a DEMOTED topic must report `wake: false` — it asked for one and does \
         not have it, which is exactly what an operator asking 'why is this row \
         slower?' needs to see: {st}"
    );

    // And it still DELIVERS, on the timer.
    let before = h.daemon.poll_loop_frames_drained();
    mirror.publish_raw(&vector3_frame(0)).expect("publish");
    assert!(
        await_frames(&h.daemon, before, Duration::from_secs(5)).is_some(),
        "a demoted topic keeps delivering — the demotion drops a LISTENER, never \
         the tap or a frame"
    );
}

/// **A LOCAL row carries its wake verdict like any other.**
///
/// The INVERSION of `a_local_topic_reports_no_wake_field_at_all`. Before the
/// decision a local row carried NO `wake` field, and the absence was the
/// anti-tautology that made `false` mean "asked and lost". Now every seam asks,
/// so the decision's own operator-facing claim is that NO attached row is absent —
/// and the `false`-means-asked-and-lost discrimination is preserved by the
/// storm arm above, which drives a real demotion and reads `Some(false)`.
///
/// Reverting `attach_local` to `WakeMode::Timer` AND dropping its
/// `wake_requested` insert lands back on `None` and fails here; reverting
/// only the `WakeMode` lands on `Some(false)` and fails the residency arm. The
/// two arms together admit neither half of the revert.
#[test]
fn a_local_topics_row_reports_its_wake_verdict_like_any_other() {
    let h = start("nf");
    let topic = "/wake/plain";
    let _producer = h
        .manager
        .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
        .expect("local producer");

    let mut client = Client::connect(&h.socket);
    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true), "local attach: {att}");

    let st = topic_status(&mut client, 2, topic);
    assert_eq!(
        st["wake"].as_bool(),
        Some(true),
        "a LOCAL row must report a LIVE wake — every attach seam is on \
         the listener, so no attached row is timer-paced and none is absent: {st}"
    );
}

/// **The pacing and demotion bounds cover a LOCAL wake — demonstrably, not by inheritance.**
///
/// `WakeGovernor`/`WakeDemoter` key on wake-source INDICES, so they are blind to
/// which seam opened a tap; that is an argument, and the local wake turns it into a
/// load-bearing one by putting a class of producer we do NOT own behind those
/// bounds. A local topic whose event service is rung without anything being
/// published must therefore be paced and demoted exactly like a remote one — and
/// the demotion is what UN-BILLS the graph publisher, so a bound that failed to
/// fire here would leave a foreign producer paying a notify per ring forever.
///
/// The tap is opened by the LOCAL verb (a plain `attach` with no `robot`), which
/// is the half this arm pins. The producer/ringer keeps the storm fixture's
/// `create_ingress_publisher` shape — the demoter never sees a producer, only
/// wake-source indices, and the fixture needs a SECOND notifier on the topic to
/// ring with.
///
/// That the arm really runs `attach_local` and not `attach_remote` is not taken
/// on faith: dropping `attach_local`'s `wake_requested` insert
/// fails THIS arm (and only this arm) at `None` vs `Some(false)`, which no
/// remote-routed attach could produce.
#[test]
fn a_local_topics_wake_is_paced_and_demoted_by_the_same_bounds() {
    let h = start("lf");
    let topic = "/wake/localflood";
    // The topic exists locally; the LOCAL attach verb is what gives it a wake.
    let mut producer = h
        .manager
        .create_ingress_publisher(topic, MaxSliceLen::const_new(1 << 16))
        .expect("producer");

    let mut client = Client::connect(&h.socket);
    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true), "local attach: {att}");
    let st = topic_status(&mut client, 2, topic);
    assert_eq!(
        st["wake"].as_bool(),
        Some(true),
        "precondition: the LOCAL tap took a wake, so there is something to \
         demote: {st}"
    );

    let flood = NotifyFlood::start(&h.manager, topic);
    let started = Instant::now();
    let deadline = started + Duration::from_secs(120);
    while h.daemon.poll_loop_wake_demotions() == 0 && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(10));
    }
    let took = started.elapsed();
    let rung = flood.stop();
    println!(
        "[LOCAL notify storm] demoted after {:.3}s ({rung} events \
         rung); paced_passes={} demotions={}",
        took.as_secs_f64(),
        h.daemon.poll_loop_wake_paced_passes(),
        h.daemon.poll_loop_wake_demotions(),
    );

    assert_eq!(
        h.daemon.poll_loop_wake_demotions(),
        1,
        "a LOCAL wake that fires without delivering must be DROPPED by the same \
         demotion streak that drops a remote one — otherwise the local wake leaves a \
         graph publisher we do not own billed for nothing, indefinitely"
    );
    assert!(
        h.daemon.poll_loop_wake_paced_passes() > 0,
        "and the loop must have PACED itself under the storm (the observer pacing bound)"
    );
    assert!(
        rung > 1_000,
        "the fixture must genuinely have stormed the event service (rang {rung})"
    );

    // The operator surface says so, and the topic keeps delivering on the timer.
    let st = topic_status(&mut client, 3, topic);
    assert_eq!(
        st["wake"].as_bool(),
        Some(false),
        "a DEMOTED local row reports `wake: false` — asked, and no longer has \
         one: {st}"
    );
    let before = h.daemon.poll_loop_frames_drained();
    producer.publish_raw(&vector3_frame(0)).expect("publish");
    assert!(
        await_frames(&h.daemon, before, Duration::from_secs(5)).is_some(),
        "a demoted local topic keeps delivering — the demotion drops a LISTENER, \
         never the tap or a frame"
    );
}

// ===========================================================================
// The wiring, pinned structurally — the mode is chosen by the SEAM, per class.
// ===========================================================================

/// **EVERY attach seam asks for the wake**, pinned per FUNCTION
/// BODY.
///
/// This arm is the INVERSION of the earlier walk, which required
/// `attach_local`/`attach_for_compose` to contain `WakeMode::Timer` and NOT
/// `Listener`. Both halves of that oracle are now false — so the
/// assertions are flipped and the reason is recorded, rather than the arm being
/// deleted and the seams left unpinned.
///
/// A whole-file `contains("WakeMode::Listener")` is true the moment ANY seam
/// uses it and cannot see a seam that kept the timer, which is exactly the
/// regression this file's behavioural arms would only PARTLY catch: the
/// residency arm sees `attach_local`, the headline sees `attach_remote`, and
/// NOTHING behavioural drives `attach_for_compose`. So this walk is the only
/// pin on that third seam, and a future fourth attach path cannot inherit a
/// timer silently.
#[test]
fn each_attach_seam_asks_for_the_wake_mode_its_topic_class_earns() {
    let src = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/src/daemon.rs"))
        .expect("read the daemon source");
    let code = code_only(&src);

    for seam in [
        "fn attach_remote",
        "fn attach_local",
        "fn attach_for_compose",
    ] {
        let body = fn_body(&code, seam).unwrap_or_else(|| panic!("{seam} exists"));
        assert!(
            body.contains("WakeMode::Listener"),
            "{seam} must ask for a wake. The wake policy puts EVERY topic \
             class on the listener: a remote topic bills only the desk's own netd, \
             and a local one bills its producer for as long as a human is watching \
             (the trade the policy makes, because the alternative left \
             `cerulion viz` on a robot at the ~16 ms poll floor)"
        );
        assert!(
            !body.contains("WakeMode::Timer"),
            "{seam} must not ALSO take the timer arm — one seam, one mode. A tap \
             is put on the timer only by a wake that could not be OPENED or one \
             that was DEMOTED, never by an attach seam"
        );
    }
}

/// **A COMPOSE-attached local row says `wake: false` when its wake was refused
/// — never nothing at all.**
///
/// `attach_for_compose`'s `wake_requested.insert` is the second half of the
/// local wake on that seam, and without this arm it is killed by
/// nothing: deleting it while leaving `WakeMode::Listener` in place survives the
/// whole vizd test suite. The seam's other half is pinned by the structural
/// walk, and the `attach`/`attach_remote` twins are pinned by the storm arms —
/// this is the third instance of the same rule, on the ONE seam that no
/// behavioural arm drove.
///
/// What the missing insert would COST: `status` evaluates
/// `has_wake ? Some(true) : wake_requested.contains ? Some(false) : None`, and
/// `protocol.rs` defines an ABSENT `wake` as **"the
/// daemon predates the local wake"**. So a current daemon would make a false claim
/// about its own version, on exactly the row an operator is reading to answer
/// "why is this topic slower than that one?" — which is the question
/// `wake_requested` was kept (rather than collapsed into `has_wake`) to answer.
///
/// The stimulus is deterministic, not a race: the topic's event-service listener
/// ceiling is genuinely EXHAUSTED before the compose, so the tap's wake cannot
/// be opened and degrades — the same technique
/// `tap_manager_test::a_wake_that_cannot_be_opened_degrades_the_tap_to_the_timer_loudly`
/// uses, driven here through the real `compose_layout` verb. Its sibling
/// discriminator is in the same body: with the hogs RELEASED, a second compose
/// of a fresh topic reports `Some(true)`, so `Some(false)` is a verdict about
/// the wake and not this arm's constant.
#[test]
fn a_compose_attached_local_row_reports_a_refused_wake_as_false_not_absent() {
    let h = start("cw");
    let refused = "/wake/compose_refused";
    let granted = "/wake/compose_granted";

    // Two LOCAL producers. `create_ingress_publisher` so `publish_raw` notifies
    // (the mirror-shaped local producer the storm arm also uses) — the frames
    // matter here because a compose only places a topic whose archetype it can
    // resolve, which takes real Vector3 frames on the wire.
    let mut producers = Vec::new();
    for topic in [refused, granted] {
        producers.push(
            h.manager
                .create_ingress_publisher(topic, MaxSliceLen::const_new(1 << 16))
                .expect("local producer"),
        );
    }

    // The live stream comes up FIRST, because a publisher takes a listener slot
    // of its own (its `SubscriberConnected` channel) — starting it after the
    // exhaustion below fails its creation, not the wake's (measured).
    let pump = FramePump::start(&h.manager, refused);

    // EXHAUST the refused topic's remaining event-service listener slots and HOLD
    // them, so the compose attach's wake cannot be opened. The bound is far above
    // any plausible ceiling, so a raised default fails the precondition loudly
    // rather than looping.
    let mut hogs = Vec::new();
    let mut ceiling_reached = false;
    for _ in 0..512 {
        match h.manager.create_wake_listener(refused) {
            Ok(l) => hogs.push(l),
            Err(_) => {
                ceiling_reached = true;
                break;
            }
        }
    }
    assert!(
        ceiling_reached,
        "precondition: the event service must have a listener ceiling to exhaust"
    );

    let mut client = Client::connect(&h.socket);

    // ARM 1 — the refused wake. `compose_layout` attaches the topic itself
    // (`attach_missing` defaults true) and then PEEKS for frames to resolve an
    // archetype, so the stream has to be LIVE across the call — a pre-seeded
    // burst lands before any tap exists and is never seen (measured: the compose
    // refuses with "still-silent").
    let resp = client.request(&format!(
        r#"{{"id":1,"method":"compose_layout","intent":{{"topics":["{refused}"]}}}}"#
    ));
    assert_eq!(
        resp["ok"].as_bool(),
        Some(true),
        "the compose must SUCCEED — a wake that cannot be opened degrades the tap, \
         it never fails the attach: {resp}"
    );
    assert_eq!(
        resp["attached"],
        serde_json::json!([refused]),
        "compose attached the topic itself (this is the `attach_for_compose` seam, \
         not the `attach` verb): {resp}"
    );

    let st = topic_status(&mut client, 2, refused);
    assert_eq!(
        st["wake"].as_bool(),
        Some(false),
        "a COMPOSE-attached row whose wake was refused must say so. `None` here \
         means 'this daemon predates the local wake' on the wire — a false \
         claim from a daemon that asked and lost: {st}"
    );

    // ARM 2 — the discriminator. Release the hogs, compose a topic whose wake CAN
    // be opened, and the same seam reports `Some(true)`. Without this, `Some(false)`
    // above is satisfied by a daemon that hardcodes it.
    drop(hogs);
    pump.stop();
    let pump = FramePump::start(&h.manager, granted);
    let resp = client.request(&format!(
        r#"{{"id":3,"method":"compose_layout","intent":{{"topics":["{granted}"]}}}}"#
    ));
    assert_eq!(resp["ok"].as_bool(), Some(true), "second compose: {resp}");
    let st = topic_status(&mut client, 4, granted);
    assert_eq!(
        st["wake"].as_bool(),
        Some(true),
        "the same seam reports a LIVE wake when the listener could be opened — so \
         the `false` above is a verdict, not a constant: {st}"
    );
    pump.stop();
    drop(producers);
}

/// A background publisher: a live `geometry_msgs/Vector3` stream on `topic`
/// until stopped.
///
/// `compose_layout` attaches a topic and then PEEKS for frames within a shared
/// budget to resolve its archetype, so a compose arm needs the stream to be live
/// ACROSS the request — frames published before the tap existed are never seen,
/// and the compose refuses with "still-silent" (measured, on this arm's first
/// cut). It creates its own publisher inside the thread, the same shape
/// [`NotifyFlood`] uses.
struct FramePump {
    stop: Arc<std::sync::atomic::AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl FramePump {
    fn start(manager: &Arc<TransportManager>, topic: &str) -> Self {
        let stop = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let stop_t = Arc::clone(&stop);
        let manager_t = Arc::clone(manager);
        let topic_t = topic.to_string();
        let handle = std::thread::spawn(move || {
            let mut publisher = manager_t
                .create_ingress_publisher(&topic_t, MaxSliceLen::const_new(1 << 16))
                .expect("pump publisher");
            let mut seq = 0u32;
            while !stop_t.load(std::sync::atomic::Ordering::Relaxed) {
                let _ = publisher.publish_raw(&vector3_frame(seq));
                seq = seq.wrapping_add(1);
                std::thread::sleep(Duration::from_millis(5));
            }
        });
        Self {
            stop,
            handle: Some(handle),
        }
    }

    fn stop(mut self) {
        self.stop.store(true, std::sync::atomic::Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// **A compose ROLLBACK undoes the wake request it made.**
///
/// `wake_requested`'s own doc states the invariant
/// `status` leans on: "the set equals the tap set", "cleared by `detach`
/// alongside the tap". `rollback_compose_attaches` detaches the tap and drops
/// the stats DIRECTLY — it does not go through the `detach` verb, which is the
/// only other place `wake_requested.remove` is called — so a rolled-back
/// compose left an entry behind that nothing could ever clear again (there is no
/// tap left to detach). The set is keyed by TOPIC NAME and a compose can name a
/// topic a remote robot announced, so a rollback loop grows it without bound on
/// a long-lived daemon.
///
/// STRUCTURAL rather than behavioural, by necessity: with no tap, the leaked entry
/// changes no `status` row and no reply — `status` iterates the taps. So there
/// is no wire observable to assert on, and a walk over the rollback's own body
/// is the pin that exists. Deleting the line fails this arm, and it is checked
/// against the same stripped view, and by the same brace-matched extractor,
/// as the seam walk above.
#[test]
fn a_compose_rollback_undoes_the_wake_request_it_made() {
    let src = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/src/daemon.rs"))
        .expect("read the daemon source");
    let code = code_only(&src);
    let body = fn_body(&code, "fn rollback_compose_attaches").expect("the rollback seam exists");
    for undo in ["taps.detach", "stats.remove", "wake_requested.remove"] {
        assert!(
            body.contains(undo),
            "rollback_compose_attaches must undo `{undo}` — the rollback is the \
             inverse of what the attach ACTUALLY did, and the attach inserts into \
             all three"
        );
    }
}

/// ANTI-TAUTOLOGY for the walk above: the stripper must leave the code it is
/// scanning, and the body extractor must return a body rather than the file.
#[test]
fn the_source_walk_actually_reads_the_bodies_it_claims_to() {
    let src = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/src/daemon.rs"))
        .expect("read the daemon source");
    let code = code_only(&src);
    assert!(
        code.contains("fn attach_remote"),
        "the stripped view must still contain the code being scanned"
    );
    let remote = fn_body(&code, "fn attach_remote").expect("attach_remote body");
    assert!(
        remote.len() < code.len() / 2,
        "fn_body must return ONE body, not the whole file ({} of {} bytes)",
        remote.len(),
        code.len()
    );
    let local = fn_body(&code, "fn attach_local").expect("attach_local body");
    assert!(
        !local.contains("fn attach_remote"),
        "one seam's body must not spill into the next"
    );
    // The stripper's own oracle: comments and string literals go, code stays.
    let sample = "let a = 1; // WakeMode::Listener\nlet b = \"WakeMode::Listener\";\n\
                  /* WakeMode::Listener */ let c = 3;";
    let stripped = code_only(sample);
    assert!(
        !stripped.contains("WakeMode::Listener"),
        "a mention in a comment or a string literal must not satisfy the walk: {stripped}"
    );
    assert!(
        stripped.contains("let a") && stripped.contains("let b") && stripped.contains("let c"),
        "and the code around them must survive: {stripped}"
    );
}

/// Strip `//` line comments, `/* */` block comments (DEPTH-TRACKED — Rust's
/// block comments nest) and string/char literals, so a mention in prose or in a
/// message cannot satisfy a structural assertion.
///
/// Panics on unbalanced block-comment depth: silently returning a truncated view
/// would make every negative assertion above vacuous over a prefix, which is
/// exactly the failure `convergence_adoption_test` documents.
fn code_only(src: &str) -> String {
    let b = src.as_bytes();
    let mut out = String::with_capacity(src.len());
    let mut i = 0usize;
    let mut depth = 0usize;
    while i < b.len() {
        if depth > 0 {
            if b[i] == b'/' && i + 1 < b.len() && b[i + 1] == b'*' {
                depth += 1;
                i += 2;
            } else if b[i] == b'*' && i + 1 < b.len() && b[i + 1] == b'/' {
                depth -= 1;
                i += 2;
            } else {
                i += 1;
            }
            continue;
        }
        if b[i] == b'/' && i + 1 < b.len() && b[i + 1] == b'*' {
            depth += 1;
            i += 2;
            continue;
        }
        if b[i] == b'/' && i + 1 < b.len() && b[i + 1] == b'/' {
            while i < b.len() && b[i] != b'\n' {
                i += 1;
            }
            continue;
        }
        // Raw strings (`r"..."`, `r#"..."#`) — consume to the matching close.
        if b[i] == b'r' && i + 1 < b.len() && (b[i + 1] == b'"' || b[i + 1] == b'#') {
            let mut j = i + 1;
            let mut hashes = 0usize;
            while j < b.len() && b[j] == b'#' {
                hashes += 1;
                j += 1;
            }
            if j < b.len() && b[j] == b'"' {
                j += 1;
                while j < b.len() {
                    if b[j] == b'"' {
                        let mut k = j + 1;
                        let mut seen = 0usize;
                        while k < b.len() && b[k] == b'#' && seen < hashes {
                            seen += 1;
                            k += 1;
                        }
                        if seen == hashes {
                            j = k;
                            break;
                        }
                    }
                    j += 1;
                }
                i = j;
                continue;
            }
        }
        if b[i] == b'"' {
            i += 1;
            while i < b.len() {
                if b[i] == b'\\' {
                    i += 2;
                    continue;
                }
                if b[i] == b'"' {
                    i += 1;
                    break;
                }
                i += 1;
            }
            continue;
        }
        // Char literals, short shapes only, so lifetimes (`'a`) survive.
        if b[i] == b'\'' && i + 2 < b.len() {
            if b[i + 1] == b'\\' {
                let mut j = i + 2;
                while j < b.len() && j < i + 8 && b[j] != b'\'' {
                    j += 1;
                }
                if j < b.len() && b[j] == b'\'' {
                    i = j + 1;
                    continue;
                }
            } else if b[i + 2] == b'\'' {
                i += 3;
                continue;
            }
        }
        out.push(b[i] as char);
        i += 1;
    }
    assert_eq!(
        depth, 0,
        "unbalanced block comment in the scanned source — the stripped view would \
         be a truncated prefix and every negative assertion over it vacuous"
    );
    out
}

/// The brace-matched body of the first `fn` whose signature starts with `sig`.
fn fn_body(code: &str, sig: &str) -> Option<String> {
    let at = code.find(sig)?;
    let open = code[at..].find('{')? + at;
    let bytes = code.as_bytes();
    let mut depth = 0usize;
    for (idx, &c) in bytes.iter().enumerate().skip(open) {
        if c == b'{' {
            depth += 1;
        } else if c == b'}' {
            depth -= 1;
            if depth == 0 {
                return Some(code[open..=idx].to_string());
            }
        }
    }
    None
}
