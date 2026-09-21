// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end pins for the EVENT-DRIVEN `cerulion topic echo` /
//! `topic hz` observer loops over real iceoryx2 shared memory.
//!
//! The observers now block on the subscriber's iceoryx2 event listener
//! (`wait_for_message`) instead of a `try_receive` + sleep poll loop:
//!  - a data event wakes them the instant a frame arrives (µs display
//!    latency, zero idle CPU while data flows);
//!  - on timeout they STILL drain the queue, so a NON-NOTIFYING publisher
//!    (a foreign raw-iceoryx2 writer that puts data on `{topic}/data`
//!    without ringing `{topic}/event`) degrades to heartbeat-cadence
//!    batched display rather than silence (Principle #6: no data loss).
//!
//! These tests open REAL iceoryx2 SHM services on the process-global
//! namespace (the observers call `TransportManager::get_or_init()` /
//! `Config::global_config()`), so they are `#[serial]` and must run with
//! `--test-threads=1`. Run them on their own (the machine's SHM is shared
//! across the workspace). Topic names are pid + nanos
//! unique so a stale service from a prior run cannot collide.
//!
//! Run:
//! ```bash
//! cargo test -p cerulion_cli_engine --test topic_observer_iox2_test -- --test-threads=1
//! ```

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use cerulion_cli_engine::topic_cmd::{
    topic_echo, topic_hz, topic_info, STD_MSGS_STRING_SCHEMA_HASH,
};
use cerulion_core::{PubSubEvent, WireHeader};
use serial_test::serial;

/// The raw iceoryx2 service policy used by the foreign-writer fixtures.
/// The on-SHM service identity is identical across `ipc::Service` and the
/// `ipc_threadsafe::Service` the CLI subscriber uses (only the in-process
/// port-wrapping policy differs), so a service created here opens fine via
/// the CLI's open-only subscriber — see `open_only_subscriber_iox2_test.rs`.
type RawService = iceoryx2::service::ipc::Service;

/// A pid + nanosecond unique canonical (`/`-prefixed) topic name so a stale
/// SHM service from a prior run on the shared Mac cannot collide.
fn unique_topic(tag: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    format!("/{tag}/{}/{}", std::process::id(), nanos)
}

/// Build a wire frame for a `std_msgs/String`-shaped payload that
/// `topic_echo`'s `decode_string_payload` pretty-prints as
/// `std_msgs/String: "<s>"`.
///
/// Layout: 32-byte [`WireHeader`] (schema hash = the pinned
/// `std_msgs/String` hash so echo recognizes it), then the payload — an
/// 8-byte offset table (`offset = 8`, `len = s.len()`) followed by the
/// UTF-8 bytes. `total_size` is the full frame length.
fn build_string_frame(seq: u32, ts_ns: u64, s: &str) -> Vec<u8> {
    let sbytes = s.as_bytes();
    let payload_len = 8 + sbytes.len();
    let total = WireHeader::SIZE + payload_len;
    let mut frame = vec![0u8; total];

    let mut header = WireHeader::new(STD_MSGS_STRING_SCHEMA_HASH, seq, ts_ns);
    header.total_size = total as u32;
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);

    let payload = &mut frame[WireHeader::SIZE..];
    payload[0..4].copy_from_slice(&8u32.to_le_bytes()); // offset
    payload[4..8].copy_from_slice(&(sbytes.len() as u32).to_le_bytes()); // len
    payload[8..8 + sbytes.len()].copy_from_slice(sbytes);
    frame
}

/// Build a bare 32-byte frame (header only, empty payload) carrying an
/// arbitrary `schema_hash` — `topic_info` reads only the header, so this drives
/// the unknown-hash resolution arm without a decodable payload.
fn build_frame_with_hash(hash: u64, seq: u32, ts_ns: u64) -> Vec<u8> {
    let mut frame = vec![0u8; WireHeader::SIZE];
    let mut header = WireHeader::new(hash, seq, ts_ns);
    header.total_size = WireHeader::SIZE as u32;
    header.write_to_buf(&mut frame);
    frame
}

/// RAII env-var guard for the `#[serial]` kill-switch tests (restores
/// the prior value on drop, panic-safe).
struct EnvGuard {
    key: &'static str,
    prev: Option<String>,
}

impl EnvGuard {
    fn set(key: &'static str, val: &str) -> Self {
        let prev = std::env::var(key).ok();
        std::env::set_var(key, val);
        Self { key, prev }
    }
}

impl Drop for EnvGuard {
    fn drop(&mut self) {
        match &self.prev {
            Some(v) => std::env::set_var(self.key, v),
            None => std::env::remove_var(self.key),
        }
    }
}

/// A foreign raw-iceoryx2 writer on `{topic}/data` (+ optionally
/// `{topic}/event`). Holding the node/services/ports alive keeps the
/// topic discoverable by `topic list` for the whole test.
struct RawTopicWriter {
    // Order-independent liveness holders; the publisher is the port we send on.
    _node: iceoryx2::node::Node<RawService>,
    _data_service:
        iceoryx2::service::port_factory::publish_subscribe::PortFactory<RawService, [u8], ()>,
    publisher: iceoryx2::port::publisher::Publisher<RawService, [u8], ()>,
    /// Present only for the NOTIFYING fixture; `None` = pure data writer
    /// (the adversarial timeout-drain case).
    notifier: Option<iceoryx2::port::notifier::Notifier<RawService>>,
    _event_service: Option<iceoryx2::service::port_factory::event::PortFactory<RawService>>,
}

impl RawTopicWriter {
    /// Create the `{topic}/data` publish-subscribe service (+ a publisher)
    /// and, when `notifying`, the `{topic}/event` service (+ a notifier).
    fn new(topic: &str, notifying: bool) -> Self {
        let node = iceoryx2::node::NodeBuilder::new()
            .create::<RawService>()
            .expect("raw iceoryx2 node");

        let data_name: iceoryx2::service::service_name::ServiceName = format!("{topic}/data")
            .as_str()
            .try_into()
            .expect("data name");
        let data_service = node
            .service_builder(&data_name)
            .publish_subscribe::<[u8]>()
            // Room for several frames between the observer's drains.
            .subscriber_max_buffer_size(16)
            .create()
            .expect("raw data service");
        let publisher = data_service
            .publisher_builder()
            .initial_max_slice_len(4096)
            .create()
            .expect("raw publisher");

        let (event_service, notifier) = if notifying {
            let event_name: iceoryx2::service::service_name::ServiceName = format!("{topic}/event")
                .as_str()
                .try_into()
                .expect("event name");
            let event_service = node
                .service_builder(&event_name)
                .event()
                .create()
                .expect("raw event service");
            let notifier = event_service
                .notifier_builder()
                .create()
                .expect("raw notifier");
            (Some(event_service), Some(notifier))
        } else {
            (None, None)
        };

        Self {
            _node: node,
            _data_service: data_service,
            publisher,
            notifier,
            _event_service: event_service,
        }
    }

    /// Publish one wire frame into `{topic}/data`. Rings the `SentSample`
    /// event ONLY if this writer was built notifying — the adversarial
    /// fixture never notifies, forcing the observer's timeout-drain path.
    fn publish(&self, frame: &[u8]) {
        let mut sample = self
            .publisher
            .loan_slice_uninit(frame.len())
            .expect("raw publisher loan");
        let buf = sample.payload_mut();
        for (i, &b) in frame.iter().enumerate() {
            buf[i].write(b);
        }
        // SAFETY: bytes [0, frame.len()) were all initialized above.
        let sample = unsafe { sample.assume_init() };
        sample.send().expect("raw publisher send");

        if let Some(notifier) = &self.notifier {
            notifier
                .notify_with_custom_event_id(PubSubEvent::SentSample.into())
                .expect("raw notify SentSample");
        }
    }
}

/// Run `f` while a background thread rings `{topic}/event` as fast as
/// it can, publishing NOTHING — the progress-free event churn that would
/// otherwise pin a core. The flood stops when the returned guard drops.
struct NotifyFlood {
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<u64>>,
}

impl NotifyFlood {
    fn start(topic: &str) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let topic = topic.to_string();
        let stop_c = stop.clone();
        let handle = std::thread::spawn(move || {
            // A SEPARATE notifier on the same event service — the fixture's own
            // writer stays available to the test for real publishes.
            let node = iceoryx2::node::NodeBuilder::new()
                .create::<RawService>()
                .expect("flood node");
            let event_name: iceoryx2::service::service_name::ServiceName = format!("{topic}/event")
                .as_str()
                .try_into()
                .expect("event name");
            let svc = node
                .service_builder(&event_name)
                .event()
                .open_or_create()
                .expect("flood event service");
            let notifier = svc.notifier_builder().create().expect("flood notifier");
            let mut rung = 0u64;
            while !stop_c.load(Ordering::Relaxed) {
                if notifier
                    .notify_with_custom_event_id(PubSubEvent::SentSample.into())
                    .is_ok()
                {
                    rung += 1;
                }
                // Yield so a low-core runner still schedules the observer: the
                // point is a HIGH event rate, not starving the thread under
                // test. Hundreds of thousands of wakes/s is far more than
                // enough to drive an unpaced loop into a tight spin.
                std::thread::yield_now();
            }
            rung
        });
        Self {
            stop,
            handle: Some(handle),
        }
    }

    /// Stop the flood and return how many notifications it managed to ring.
    fn stop(mut self) -> u64 {
        self.stop.store(true, Ordering::Relaxed);
        self.handle
            .take()
            .expect("flood handle")
            .join()
            .unwrap_or(0)
    }
}

impl Drop for NotifyFlood {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// The anti-vacuity floor for the flood fixture — proof the notifier
/// thread really started and rang REPEATEDLY, so a `paced > 0` assertion is not
/// being satisfied by some other path.
///
/// Deliberately modest, and NOT the load-bearing assertion. The pin is
/// `observer_pacing_engaged_count() > 0`; this only rules out a fixture that
/// silently did nothing. A 10,000-ring floor fails on
/// a contended desk at 1,620 — an absolute event-count floor is a function of
/// the flood thread's scheduler share, i.e. exactly the load-sensitive class
/// these arms avoid everywhere else. 200 is ~25x the eight
/// consecutive progress-free iterations pacing needs, so it still proves a
/// genuine flood, while an idle desk rings 2-3 million.
const MIN_FLOOD_EVENTS: u64 = 200;

/// The CALLING THREAD's own CPU time, for the DIAGNOSTIC print in the
/// flood arm.
///
/// Thread-scoped, not process-scoped, deliberately: the flood fixture spins a
/// thread of its own on purpose, so a process-wide reading is ~a full core no
/// matter how well the observer behaves and would say nothing about the loop
/// under test.
///
/// Never asserted on. A CPU-percentage gate fails OPEN on a loaded runner —
/// a starved spinner accumulates LESS CPU and so passes the very check meant to
/// catch it (the class) — so the enforceable contract lives in the pure
/// `decide_observer_pacing` oracle vectors and this is evidence for a human.
fn thread_cpu_time() -> Duration {
    let mut ts: libc::timespec = unsafe { std::mem::zeroed() };
    // SAFETY: `ts` is a valid, zeroed `timespec` this call fills in.
    if unsafe { libc::clock_gettime(libc::CLOCK_THREAD_CPUTIME_ID, &mut ts) } != 0 {
        return Duration::ZERO;
    }
    Duration::from_secs(ts.tv_sec as u64) + Duration::from_nanos(ts.tv_nsec as u64)
}

/// Spawn `topic_hz` like [`spawn_hz`], additionally reporting how much CPU the
/// OBSERVER THREAD itself burned (the diagnostic).
fn spawn_hz_timed(
    topic: String,
    running: Arc<AtomicBool>,
) -> JoinHandle<(ObserverOutcome, Duration)> {
    std::thread::spawn(move || {
        let mut buf: Vec<u8> = Vec::new();
        let cpu_before = thread_cpu_time();
        let res = topic_hz(&topic, None, running, &mut buf);
        let cpu = thread_cpu_time().saturating_sub(cpu_before);
        ((res, String::from_utf8_lossy(&buf).into_owned()), cpu)
    })
}

/// A finished observer run: its `Result` + everything it wrote to the
/// output sink before `running` was cleared.
type ObserverOutcome = (cerulion_cli_engine::error::CliResult<()>, String);

/// Spawn `topic_echo` on its own thread, returning a handle that yields
/// its `Result` + everything it wrote once `running` is cleared.
fn spawn_echo(topic: String, running: Arc<AtomicBool>) -> JoinHandle<ObserverOutcome> {
    std::thread::spawn(move || {
        let mut buf: Vec<u8> = Vec::new();
        let res = topic_echo(
            &topic,
            None,
            running,
            &mut buf,
            cerulion_cli_engine::topic_cmd::DEFAULT_ECHO_TRUNCATE_LENGTH,
        );
        (res, String::from_utf8_lossy(&buf).into_owned())
    })
}

/// Spawn `topic_hz` on its own thread (same contract as [`spawn_echo`]).
fn spawn_hz(topic: String, running: Arc<AtomicBool>) -> JoinHandle<ObserverOutcome> {
    std::thread::spawn(move || {
        let mut buf: Vec<u8> = Vec::new();
        let res = topic_hz(&topic, None, running, &mut buf);
        (res, String::from_utf8_lossy(&buf).into_owned())
    })
}

// ─────────────────────────────────────────────────────────────────────
// Test 1a (behavioral + ACCUMULATE-ALL): a NOTIFYING publisher's frames
// are ALL echoed — N DISTINCT payloads, each displayed exactly once
// (multiplicity oracle). A drain-to-latest regression (displaying only
// the newest queued frame per wake) fails the exactly-once counts.
// ─────────────────────────────────────────────────────────────────────
#[test]
#[serial]
fn echo_displays_all_distinct_notified_frames_exactly_once() {
    let topic = unique_topic("echo_flow");
    // Create the topic BEFORE the observer so `require_topic_exists` passes.
    let writer = RawTopicWriter::new(&topic, /* notifying = */ true);

    let running = Arc::new(AtomicBool::new(true));
    let handle = spawn_echo(topic.clone(), running.clone());

    // Let the subscriber attach, then publish N DISTINCT payloads as a
    // BURST (back-to-back, one notify each): several frames are queued by
    // the time echo wakes, so accumulate-all vs drain-to-latest is
    // genuinely discriminated.
    const N: u32 = 5;
    std::thread::sleep(Duration::from_millis(500));
    for seq in 0..N {
        writer.publish(&build_string_frame(
            seq,
            1_000 + seq as u64,
            &format!("obs-notify-{seq}"),
        ));
    }
    std::thread::sleep(Duration::from_millis(500));

    running.store(false, Ordering::Relaxed);
    let (res, out) = handle.join().expect("echo thread");
    res.expect("topic_echo must return Ok on a live topic");

    // Hand oracle: EVERY distinct payload appears EXACTLY once.
    for seq in 0..N {
        let needle = format!("std_msgs/String: \"obs-notify-{seq}\"");
        let count = out.matches(&needle).count();
        assert_eq!(
            count, 1,
            "payload {seq} must display exactly once (accumulate-all, no \
             drain-to-latest, no duplicate); saw {count}:\n{out}"
        );
    }
    assert_eq!(
        out.matches("std_msgs/String: \"obs-notify-").count(),
        N as usize,
        "exactly the {N} published frames must display; got:\n{out}"
    );
    assert!(
        out.contains(&format!("schema=0x{STD_MSGS_STRING_SCHEMA_HASH:016x}")),
        "echo must print the wire header schema line; got:\n{out}"
    );
    // Negative gap pin: a CONSECUTIVE stream must emit ZERO gap
    // markers — kills a `missed > 0` → `missed >= 0` mutation that would
    // spam "[gap: 0 frame(s) missed …]" after every frame.
    assert_eq!(
        out.matches("[gap: ").count(),
        0,
        "a gap-free consecutive stream must print NO gap markers; got:\n{out}"
    );
}

// ─────────────────────────────────────────────────────────────────────
// Test 1b (behavioral): a silent topic echoes NOTHING and exits clean —
// AND exits promptly: the join after `running = false`
// completes within 2× the observer heartbeat. Guards BOTH a heartbeat
// bump (the 2s bound is a hardcoded literal, deliberately NOT derived
// from the const — bumping OBSERVER_HEARTBEAT past ~2s fails here
// loudly) and a wait that stops rechecking `running` between waits.
// (echo prints only inside its per-message callback — "no messages" is a
// `hz`-only line; see test 3.)
// ─────────────────────────────────────────────────────────────────────
#[test]
#[serial]
fn echo_silent_topic_prints_nothing_and_exits_clean_and_promptly() {
    let topic = unique_topic("echo_silent");
    // Topic exists (data service) but nothing is ever published.
    let _writer = RawTopicWriter::new(&topic, /* notifying = */ false);

    let running = Arc::new(AtomicBool::new(true));
    let handle = spawn_echo(topic.clone(), running.clone());

    // Span > 1 heartbeat so a spurious wakeup would have surfaced output.
    std::thread::sleep(Duration::from_millis(1500));
    running.store(false, Ordering::Relaxed);
    let stop_at = std::time::Instant::now();

    let (res, out) = handle.join().expect("echo thread");
    let exit_latency = stop_at.elapsed();
    res.expect("topic_echo must return Ok on a silent topic");
    assert!(
        out.trim().is_empty(),
        "a silent topic must produce NO echo output; got:\n{out}"
    );
    // 2s = 2× the 1000ms OBSERVER_HEARTBEAT (worst case: the flag flips just
    // after a wait began → one full heartbeat + generous scheduling slack).
    assert!(
        exit_latency < Duration::from_millis(2000),
        "echo must exit within 2x the observer heartbeat of running=false \
         (the wait must re-check `running` every heartbeat); took {exit_latency:?}"
    );
}

// ─────────────────────────────────────────────────────────────────────
// Test 2 (THE ADVERSARIAL PIN): a NON-NOTIFYING raw publisher still
// displays via the timeout-drain path — data goes onto `{topic}/data`
// with the `{topic}/event` notifier NEVER rung, so the ONLY way echo can
// see it is the on-timeout queue drain.
// ─────────────────────────────────────────────────────────────────────
#[test]
#[serial]
fn echo_displays_non_notifying_publisher_via_timeout_drain() {
    let topic = unique_topic("echo_drain");
    // notifying = false: pure data writer, no event notifier at all.
    let writer = RawTopicWriter::new(&topic, /* notifying = */ false);
    assert!(
        writer.notifier.is_none(),
        "fixture must NOT hold a notifier — the adversarial case rings no event"
    );

    let running = Arc::new(AtomicBool::new(true));
    let handle = spawn_echo(topic.clone(), running.clone());

    // Publish N DISTINCT payloads steadily WITHOUT notifying, across a
    // window spanning several ~1s heartbeats; the observer must
    // drain-on-timeout to display them. 200ms cadence × 12 frames stays
    // far below the 16-deep fixture queue between drains — NO overflow,
    // so accumulate-all means ALL of them display (the overflow/loss case
    // is the gap-marker test below).
    const N: u32 = 12;
    std::thread::sleep(Duration::from_millis(400));
    for seq in 0..N {
        writer.publish(&build_string_frame(
            seq,
            5_000 + seq as u64,
            &format!("obs-drain-{seq}"),
        ));
        std::thread::sleep(Duration::from_millis(200));
    }
    // A full heartbeat after the last publish so the final timeout drain runs.
    std::thread::sleep(Duration::from_millis(1200));

    running.store(false, Ordering::Relaxed);
    let (res, out) = handle.join().expect("echo thread");
    res.expect("topic_echo must return Ok");

    // Hand oracle: EVERY distinct payload displayed EXACTLY once even though
    // NO event was ever fired — the timeout drain accumulates ALL frames
    // (a drain-to-latest regression would show only the newest per drain).
    for seq in 0..N {
        let needle = format!("std_msgs/String: \"obs-drain-{seq}\"");
        let count = out.matches(&needle).count();
        assert_eq!(
            count, 1,
            "non-notified payload {seq} must display exactly once via the \
             timeout drain; saw {count}:\n{out}"
        );
    }
    assert_eq!(
        out.matches("std_msgs/String: \"obs-drain-").count(),
        N as usize,
        "exactly the {N} published frames must display; got:\n{out}"
    );
}

// ─────────────────────────────────────────────────────────────────────
// Test 3 (behavioral): hz's 1s report still TICKS on a silent topic.
// The `min(heartbeat, next-report-deadline)` wait must never block past
// the report deadline, so "no messages received" prints ~once per second.
// ─────────────────────────────────────────────────────────────────────
#[test]
#[serial]
fn hz_reports_tick_on_silent_topic() {
    let topic = unique_topic("hz_silent");
    let _writer = RawTopicWriter::new(&topic, /* notifying = */ false);

    let running = Arc::new(AtomicBool::new(true));
    let handle = spawn_hz(topic.clone(), running.clone());

    // ~2.4s spans two report deadlines (~1s, ~2s).
    std::thread::sleep(Duration::from_millis(2400));
    running.store(false, Ordering::Relaxed);

    let (res, out) = handle.join().expect("hz thread");
    res.expect("topic_hz must return Ok on a silent topic");

    let ticks = out.matches("no messages received").count();
    assert!(
        ticks >= 2,
        "hz must tick its 1s report on a silent topic (expected >= 2 in ~2.4s); \
         saw {ticks}:\n{out}"
    );
}

// ─────────────────────────────────────────────────────────────────────
// Test 4: hz's rate-computation branch against hand-computed
// values. The fixture hand-builds wire headers, so the
// timestamps are exact inputs: ts = {0, 0.1, 0.3, 0.6, 1.0}s gives
//   count 5, span 1.0s          → average = (5−1)/1.0 = 4.00 Hz
//   intervals {0.1,0.2,0.3,0.4} → min 0.1000s, max 0.4000s
//   mean 0.25, var ((.15)²+(.05)²+(.05)²+(.15)²)/4 = 0.0125
//                               → std = √0.0125 = 0.1118…s
// so the report line is EXACTLY (given the {:.2}/{:.4} format specs):
//   "average rate: 4.00 Hz, min: 0.1000s max: 0.4000s std: 0.1118s window: 5"
// Every value is distinct, so a swapped/miscomputed field cannot pass.
// A second phase pins the single-sample branch: exactly ONE frame in a
// report window prints "waiting for more messages... (received 1)".
// ─────────────────────────────────────────────────────────────────────
#[test]
#[serial]
fn hz_rate_line_matches_hand_computed_values_and_single_sample_branch() {
    let topic = unique_topic("hz_rate");
    let writer = RawTopicWriter::new(&topic, /* notifying = */ true);

    let running = Arc::new(AtomicBool::new(true));
    let handle = spawn_hz(topic.clone(), running.clone());

    // Phase A: after attach, burst all 5 frames (one notify each) well
    // inside the FIRST 1s report window. hz wakes on the notify and pushes
    // all 5 wire timestamps before the report fires.
    std::thread::sleep(Duration::from_millis(400));
    const TS_NS: [u64; 5] = [
        0,             // t = 0.0s
        100_000_000,   // t = 0.1s
        300_000_000,   // t = 0.3s
        600_000_000,   // t = 0.6s
        1_000_000_000, // t = 1.0s
    ];
    for (seq, ts) in TS_NS.iter().enumerate() {
        writer.publish(&build_string_frame(seq as u32, *ts, "hz-rate"));
    }
    // Ride past the first report (~1s after hz started) with margin.
    std::thread::sleep(Duration::from_millis(1300));

    // Phase B: the report cleared the window; exactly ONE more frame in the
    // next window → the single-sample branch.
    writer.publish(&build_string_frame(5, 2_000_000_000, "hz-single"));
    std::thread::sleep(Duration::from_millis(1300));

    running.store(false, Ordering::Relaxed);
    let (res, out) = handle.join().expect("hz thread");
    res.expect("topic_hz must return Ok");

    // Hand-computed oracle line (see the header derivation).
    let expected = "average rate: 4.00 Hz, min: 0.1000s max: 0.4000s std: 0.1118s window: 5";
    assert!(
        out.contains(expected),
        "hz must print the hand-computed rate line {expected:?}; got:\n{out}"
    );
    assert!(
        out.contains("waiting for more messages... (received 1)"),
        "a single-sample report window must print the waiting branch; got:\n{out}"
    );
}

// ─────────────────────────────────────────────────────────────────────
// Test 5: the non-notifying degrade window can overflow the
// bounded drop_oldest queue (fixture ceiling: 16) under a fast publisher —
// the loss must be VISIBLE. A 40-frame burst lands entirely between two
// timeout drains, so the queue keeps only the newest 16 and echo's gap
// marker must account for EVERY evicted frame:
//   sum of all reported gap Ns == published − displayed   (hand oracle)
// robust to a drain landing mid-burst (each frame is either displayed or
// inside exactly one gap — the sequences are contiguous 0..=40).
//
// FLAKE-HARDENED: on a preempted macOS CI VM the 40-publish
// burst loop (normally ≪1 ms) can be stretched across ≥3 of echo's 1 s
// timeout-drain windows (`OBSERVER_HEARTBEAT`), so the 16-deep queue is
// emptied mid-burst repeatedly, ALL 41 frames display, and no eviction
// ever happens — the run proves nothing about the gap marker (observed
// on the shared macOS runner: both display anchors passed, gap_total ==
// 0). Same whole-window-stall class the flat_latency/cross_thread_rtt
// gates are hardened against. The fix keeps the feature pin
// UNWEAKENED via attempt semantics:
//   - the EXACT-ACCOUNTING half (published == displayed + sum(gaps)) is
//     asserted on EVERY attempt whose anchors displayed — a wrong or
//     missing marker on a real eviction still fails attempt 1;
//   - an attempt with zero evictions (nothing to account for) is NON-
//     PROBATIVE, not a pass: it is retried on a fresh topic, and only an
//     attempt that REALLY evicted (gap_total > 0) completes the test;
//   - MAX_ATTEMPTS all non-probative fails loudly (needs the µs-scale
//     burst stalled across ≥3 separate 1 s windows, 3 times in a row).
// ─────────────────────────────────────────────────────────────────────
#[test]
#[serial]
fn echo_gap_marker_accounts_for_queue_overflow_loss() {
    const MAX_ATTEMPTS: usize = 3;
    const LAST_SEQ: u32 = 40;

    for attempt in 1..=MAX_ATTEMPTS {
        let topic = unique_topic("echo_gap");
        let writer = RawTopicWriter::new(&topic, /* notifying = */ false);

        let running = Arc::new(AtomicBool::new(true));
        let handle = spawn_echo(topic.clone(), running.clone());

        // Step 1 (baseline): one frame, then a full heartbeat so a timeout
        // drain displays it — the gap accounting needs seq 0 as the anchor
        // (the first-ever displayed frame only baselines, never gaps).
        std::thread::sleep(Duration::from_millis(400));
        writer.publish(&build_string_frame(0, 10_000, "obs-ovf-0"));
        std::thread::sleep(Duration::from_millis(1600));

        // Step 2 (overflow burst): 40 more frames back-to-back (µs–ms), no
        // notify — far above the 16-deep queue, inside one idle window.
        for seq in 1..=LAST_SEQ {
            writer.publish(&build_string_frame(
                seq,
                10_000 + seq as u64,
                &format!("obs-ovf-{seq}"),
            ));
        }
        // A full heartbeat so the post-burst timeout drain runs.
        std::thread::sleep(Duration::from_millis(1600));

        running.store(false, Ordering::Relaxed);
        let (res, out) = handle.join().expect("echo thread");
        res.expect("topic_echo must return Ok");

        // The anchor + the newest frame must have displayed. Absence is the
        // slow-attach flavor of the same VM-stall class (echo subscribed or
        // drained too late) — non-probative, retry on a fresh topic.
        let anchors_displayed = out.contains("std_msgs/String: \"obs-ovf-0\"")
            && out.contains(&format!("std_msgs/String: \"obs-ovf-{LAST_SEQ}\""));
        if !anchors_displayed {
            eprintln!(
                "attempt {attempt}/{MAX_ATTEMPTS}: display anchors missing \
                 (observer attached/drained too late — CI VM stall); retrying"
            );
            continue;
        }

        // Delivery accounting — asserted on EVERY anchored attempt: parse
        // every gap marker's N and every displayed frame; published ==
        // displayed + sum(gaps) EXACTLY. This is the feature-correctness
        // half and is never deferred to a retry.
        let displayed = out.lines().filter(|l| l.starts_with("seq=")).count();
        let gap_total: u64 = out
            .lines()
            .filter_map(|l| {
                l.strip_prefix("[gap: ")
                    .and_then(|rest| rest.split(' ').next())
                    .and_then(|n| n.parse::<u64>().ok())
            })
            .sum();
        let published = u64::from(LAST_SEQ) + 1;
        assert_eq!(
            gap_total + displayed as u64,
            published,
            "gap markers must account for EXACTLY the evicted frames \
             (published {published} = displayed {displayed} + gaps {gap_total}); got:\n{out}"
        );

        if gap_total > 0 {
            // Probative: a real eviction happened and every evicted frame
            // was accounted for by a gap marker. Done.
            return;
        }
        eprintln!(
            "attempt {attempt}/{MAX_ATTEMPTS}: no eviction occurred (echo's \
             1 s timeout drains landed mid-burst — CI VM stretched the \
             µs-scale burst loop); accounting held (all {published} \
             displayed); retrying on a fresh topic"
        );
    }
    panic!(
        "{MAX_ATTEMPTS} attempts produced zero evictions: a 40-frame burst \
         into a 16-deep queue kept being fully drained mid-burst — either \
         the environment is pathologically stalled or the fixture ceiling \
         is no longer 16 (check RawTopicWriter's subscriber_max_buffer_size)"
    );
}

// ─────────────────────────────────────────────────────────────────────
// `topic_info`'s schema-NAME resolution (item 2)
// wired through the live command output. A frame carrying a built-in's hash
// yields `Schema: pkg/Type (0x…)`; a frame carrying an unknown hash yields
// `Schema hash: 0x… (…unresolved…)`. Driven over the real observer harness.
// ─────────────────────────────────────────────────────────────────────
#[test]
#[serial]
fn topic_info_names_known_builtin_schema() {
    let topic = unique_topic("info_known");
    let writer = RawTopicWriter::new(&topic, /* notifying = */ true);

    // topic_info blocks up to 1s for a frame; run it on a thread, then publish
    // a std_msgs/String frame (a built-in the local walker names) once its
    // subscriber has attached.
    let handle = {
        let topic = topic.clone();
        std::thread::spawn(move || topic_info(&topic, None))
    };
    std::thread::sleep(Duration::from_millis(500));
    writer.publish(&build_string_frame(0, 1_000, "info-known"));

    let info = handle
        .join()
        .expect("info thread")
        .expect("topic_info must return Ok on a live topic");
    assert!(
        info.contains(&format!(
            "Schema: std_msgs/String (0x{STD_MSGS_STRING_SCHEMA_HASH:016x})"
        )),
        "topic_info must name the built-in schema, not just the hash: {info}"
    );
    assert!(info.contains(&format!("Topic: {topic}")), "{info}");
}

#[test]
#[serial]
fn topic_info_reports_unknown_hash_with_hint() {
    // CERULION_NETWORK=off makes the unknown-hash REMOTE fallback a fast,
    // hermetic local-only no-op (the LOCAL topic path is unaffected — the
    // kill-switch only gates the remote branch).
    let _guard = EnvGuard::set("CERULION_NETWORK", "off");
    let topic = unique_topic("info_unknown");
    let writer = RawTopicWriter::new(&topic, /* notifying = */ true);

    let handle = {
        let topic = topic.clone();
        std::thread::spawn(move || topic_info(&topic, None))
    };
    std::thread::sleep(Duration::from_millis(500));
    // A frame carrying a bogus schema hash no walker knows.
    let bogus = 0x0123_4567_89AB_CDEFu64;
    writer.publish(&build_frame_with_hash(bogus, 0, 2_000));

    let info = handle
        .join()
        .expect("info thread")
        .expect("topic_info must return Ok on a live topic");
    assert!(
        info.contains(&format!("Schema hash: 0x{bogus:016x}")),
        "an unknown hash is shown verbatim: {info}"
    );
    assert!(
        info.contains("schema name unresolved"),
        "an unknown hash carries the resolution hint: {info}"
    );
}

// ─────────────────────────────────────────────────────────────────────
// Under the CERULION_NETWORK=off kill-switch, a
// topic absent locally errors EXPLICITLY about the disabled remote discovery
// — it must NOT claim robots were searched / promise auto-demand.
// ─────────────────────────────────────────────────────────────────────
#[test]
#[serial]
fn echo_missing_topic_under_kill_switch_says_network_off() {
    let _guard = EnvGuard::set("CERULION_NETWORK", "off");
    // Never created → not a local topic; the kill-switch skips the remote search.
    let topic = unique_topic("kill_switch_missing");
    let running = Arc::new(AtomicBool::new(true));
    let mut buf: Vec<u8> = Vec::new();
    let err = topic_echo(
        &topic,
        None,
        running,
        &mut buf,
        cerulion_cli_engine::topic_cmd::DEFAULT_ECHO_TRUNCATE_LENGTH,
    )
    .expect_err("a missing topic under the kill-switch must error");
    let msg = err.to_string();
    assert!(
        msg.contains("CERULION_NETWORK=off"),
        "the kill-switch not-found error names the switch: {msg}"
    );
    assert!(
        msg.contains("not found locally"),
        "the error states that only the local check ran: {msg}"
    );
    // It must NOT imply robots were searched (the generic not-found wording).
    assert!(
        !msg.contains("on any discovered robot"),
        "the kill-switch path must not claim robots were searched: {msg}"
    );
}

// ═════════════════════════════════════════════════════════════════════
// The observer loops must not pin a core, and "attached but
// nothing arriving" must be VISIBLE.
//
// The enforceable pacing contract is pinned PURELY (the
// `decide_observer_pacing` oracle vectors in `topic_cmd.rs`) rather than
// by a CPU-percentage assertion here: on a loaded runner such a gate
// fails OPEN, because a starved spinner accumulates LESS CPU and so
// passes the very check meant to catch it (the class). What
// these behavioral arms pin is that the pacing did not break anything
// under exactly the pathological condition — the report still ticks and
// frames still flow — plus the silence surface itself.
// ═════════════════════════════════════════════════════════════════════

/// The silence surface: on a topic that exists but never publishes, hz's
/// empty-window line must name WHERE the frames were supposed to come from
/// and HOW LONG none have arrived. A bare "no messages
/// received" leaves an operator unable to tell
/// "about to stream" from "the data plane is dead".
#[test]
#[serial]
fn hz_silent_report_names_the_source_and_the_elapsed_silence() {
    let topic = unique_topic("hz_silence_named");
    let _writer = RawTopicWriter::new(&topic, /* notifying = */ false);

    let running = Arc::new(AtomicBool::new(true));
    let handle = spawn_hz(topic.clone(), running.clone());
    std::thread::sleep(Duration::from_millis(2400));
    running.store(false, Ordering::Relaxed);

    let (res, out) = handle.join().expect("hz thread");
    res.expect("topic_hz must return Ok on a silent topic");

    // The existing cadence contract is untouched…
    assert!(
        out.matches("no messages received").count() >= 2,
        "hz must still tick its 1s report on a silent topic; saw:\n{out}"
    );
    // …and every empty report now carries the two facts the operator needs.
    assert!(
        out.contains("attached to a local producer"),
        "the silent report must name the frame source; saw:\n{out}"
    );
    // DECODABLE, not just "frames": the count includes only frames surviving
    // the wire filter, so claiming "0 frames" during a malformed-frame storm
    // would be an affirmatively wrong claim about the network.
    assert!(
        out.contains("0 decodable frames in"),
        "the silent report must state how long nothing has arrived; saw:\n{out}"
    );
}

/// THE SHAPE: an external event source rings `{topic}/event` as fast
/// as it can while publishing NOTHING. Every one of those events returns the
/// observer's blocking wait instantly with no frame to show, which without
/// pacing sends the loop straight back around — an unbounded tight loop (measured at
/// ~100% of a core on a live robot).
///
/// The pacing floor must bound that WITHOUT changing behaviour: the 1s report
/// must still tick on time, and real frames published during the flood must
/// still be timed and reported. CPU is PRINTED as evidence, never asserted.
#[test]
#[serial]
fn hz_under_a_progress_free_event_flood_still_reports_and_still_times_frames() {
    let topic = unique_topic("hz_flood");
    let writer = RawTopicWriter::new(&topic, /* notifying = */ true);

    let running = Arc::new(AtomicBool::new(true));
    let flood = NotifyFlood::start(&topic);
    let paced_before = cerulion_cli_engine::topic_cmd::observer_pacing_engaged_count();
    let wall_start = std::time::Instant::now();
    let handle = spawn_hz_timed(topic.clone(), running.clone());

    // Publish a REAL ~20 Hz stream through the flood: hand-stamped wire
    // timestamps 50ms apart, so a reported rate is computed from known input.
    let mut ts_ns: u64 = 1_000_000_000;
    for seq in 0..40u32 {
        writer.publish(&build_string_frame(seq, ts_ns, "flood"));
        ts_ns += 50_000_000;
        std::thread::sleep(Duration::from_millis(50));
    }
    running.store(false, Ordering::Relaxed);

    let ((res, out), cpu) = handle.join().expect("hz thread");
    let wall = wall_start.elapsed();
    let rung = flood.stop();
    res.expect("topic_hz must return Ok under an event flood");

    // The flood really did flood (anti-vacuity: without this the whole test
    // could pass on a thread that never rang a single event).
    assert!(
        rung > MIN_FLOOD_EVENTS,
        "the fixture must actually flood the event service; rang only {rung}"
    );
    // THE PACING PIN. Without it the whole fix can go INERT at the call site
    // with every other assertion green: moving `let mut idle_churn = 0;` from
    // outside the `while running` loop to inside it resets the churn budget
    // every iteration, so `decide_observer_pacing` can never reach
    // `OBSERVER_CHURN_TOLERANCE` and the spin is fully restored — while the
    // code compiles clean under lints stricter than this crate's, every
    // structural assertion passes character for character, and the pure oracles
    // stay green because they thread their OWN `&mut churn` and are blind to
    // the call site's state by construction.
    //
    // A LOWER bound, and therefore load-IMMUNE in exactly the way the
    // CPU-percentage gate this file rejects is not: a slow runner DELAYS the
    // eighth consecutive churning iteration, it cannot prevent it, against the
    // >= MIN_FLOOD_EVENTS progress-free events the arm establishes. (An upper
    // bound inverts under load — a starved spinner accumulates less CPU and
    // passes.)
    let paced = cerulion_cli_engine::topic_cmd::observer_pacing_engaged_count() - paced_before;
    assert!(
        paced > 0,
        "the observer must ENGAGE its pacing floor under a progress-free event \
         flood ({rung} events rung, 0 pacings recorded) — the loop is spinning"
    );

    // Frames still get through and still get TIMED — the flood does not
    // starve the data path, and pacing does not swallow arrivals.
    assert!(
        out.contains("average rate:"),
        "frames published during the flood must still be timed and reported; \
         hz said:\n{out}"
    );

    // DIAGNOSTIC ONLY — never an assertion (see the module note above).
    eprintln!(
        "flood arm: wall={:?} observer_thread_cpu={:?} events_rung={} \
         pacings={} (an unpaced observer thread spins at ~100% of a core)",
        wall, cpu, rung, paced
    );
}

/// THE LATENCY CONTRACT, e2e: a topic that is actually FLOWING is never paced.
///
/// `decide_observer_pacing`'s first rule exists so the starvation floor can
/// never sit between a frame and its display, and the pure oracles pin that
/// rule — but they call the decision directly, so they cannot see the CALL
/// SITE passing the wrong argument. Handing `0` where the delivered
/// count belongs makes every wake on a healthy 50 Hz stream look like
/// progress-free churn, and the floor engages on a topic that is streaming
/// perfectly. Lower severity than the spin (it over-paces rather than pinning a
/// core) but wrong in the direction the design most cares about, and it left
/// the suite green before this arm existed.
///
/// `== 0` is the exact bound, not a tolerance: every delivered frame resets the
/// churn budget, so reaching `OBSERVER_CHURN_TOLERANCE` needs 8 CONSECUTIVE
/// progress-free iterations, which a stream delivering ~60 frames over the
/// window cannot produce.
#[test]
#[serial]
fn a_flowing_topic_is_never_paced() {
    let topic = unique_topic("hz_flowing");
    let writer = RawTopicWriter::new(&topic, /* notifying = */ true);

    let running = Arc::new(AtomicBool::new(true));
    let paced_before = cerulion_cli_engine::topic_cmd::observer_pacing_engaged_count();
    let handle = spawn_hz(topic.clone(), running.clone());

    // A steady ~50 Hz stream with hand-stamped wire timestamps 20 ms apart.
    let mut ts_ns: u64 = 1_000_000_000;
    for seq in 0..60u32 {
        writer.publish(&build_string_frame(seq, ts_ns, "flowing"));
        ts_ns += 20_000_000;
        std::thread::sleep(Duration::from_millis(20));
    }
    running.store(false, Ordering::Relaxed);

    let (res, out) = handle.join().expect("hz thread");
    let paced = cerulion_cli_engine::topic_cmd::observer_pacing_engaged_count() - paced_before;
    res.expect("topic_hz must return Ok on a flowing topic");

    // Anti-vacuity: frames really did flow (without this, a silent topic would
    // also record zero pacings and the assertion below would prove nothing).
    assert!(
        out.contains("average rate:"),
        "the stream must actually be timed and reported; hz said:\n{out}"
    );
    assert_eq!(
        paced, 0,
        "a FLOWING topic must never be paced — the starvation floor may not sit \
         between a frame and its display; hz said:\n{out}"
    );
}

/// The same flood against `topic echo`. Two contracts: the observer must not
/// fabricate output from events that carry no frame (an event is not a
/// message), and a REAL frame published during the flood must still display.
#[test]
#[serial]
fn echo_under_a_progress_free_event_flood_shows_no_phantom_frames_but_still_shows_real_ones() {
    let topic = unique_topic("echo_flood");
    let writer = RawTopicWriter::new(&topic, /* notifying = */ true);

    let running = Arc::new(AtomicBool::new(true));
    let flood = NotifyFlood::start(&topic);
    let paced_before = cerulion_cli_engine::topic_cmd::observer_pacing_engaged_count();
    let handle = spawn_echo(topic.clone(), running.clone());

    // Let the flood run alone first — nothing may be displayed for it.
    std::thread::sleep(Duration::from_millis(600));

    writer.publish(&build_string_frame(0, 1_000, "real"));
    std::thread::sleep(Duration::from_millis(600));
    running.store(false, Ordering::Relaxed);

    let (res, out) = handle.join().expect("echo thread");
    let rung = flood.stop();
    res.expect("topic_echo must return Ok under an event flood");

    assert!(
        rung > MIN_FLOOD_EVENTS,
        "the fixture must actually flood the event service; rang only {rung}"
    );
    // THE PACING PIN — echo's own. The counter is wired into BOTH loops, but a
    // counter nothing OBSERVES pins nothing: with this assertion only on the hz
    // arm, moving `let mut idle_churn = 0;` inside echo's `while running` loop
    // restored the full observer spin on the echo path with all 12 tests green
    // (reproduced). Same load-immunity argument as the hz arm: a LOWER bound a
    // slow runner can delay but not prevent, against the >= MIN_FLOOD_EVENTS progress-free
    // events this arm has already established.
    let paced = cerulion_cli_engine::topic_cmd::observer_pacing_engaged_count() - paced_before;
    assert!(
        paced > 0,
        "topic_echo must ENGAGE its pacing floor under a progress-free event \
         flood ({rung} events rung, 0 pacings recorded) — the loop is spinning"
    );
    // Hand oracle: EXACTLY the one real frame, despite tens of thousands of
    // events. An event is not a message.
    assert_eq!(
        out.matches("std_msgs/String: \"real\"").count(),
        1,
        "the one real frame must display exactly once under the flood; saw:\n{out}"
    );
    assert_eq!(
        out.matches("seq=").count(),
        1,
        "no phantom frames may be displayed for events that carried none; saw:\n{out}"
    );
}
