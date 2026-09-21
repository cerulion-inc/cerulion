// SPDX-License-Identifier: AGPL-3.0-only
//! The frame-drain loop's PERIOD: measured, then paced.
//!
//! **Why the observable exists.** A measurement of the desk chain found the
//! frame's SHM residency between netd's re-inject and vizd's drain to be
//! **11.8 ms p50 / 19.4 ms p90** against **78 µs** of actual work. Residency is
//! produced by one loop, so those two order statistics imply its period
//! (`p50 = 0.5T` ⇒ `T ≈ 23.6 ms`, `p90 = 0.9T` ⇒ `T ≈ 21.6 ms`) against a
//! nominal [`DEFAULT_POLL_INTERVAL`] of 16 ms. That is an **inference**, and it
//! has a second candidate explanation (contention on the daemon's state lock
//! with the control plane) which order statistics alone cannot separate from a
//! post-work fixed sleep. The period register measures the quantity directly instead of
//! resting the pacing on the inference.
//!
//! **What the deadline ticker does.** `std::thread::sleep(poll_interval)` taken AFTER
//! the loop's work makes the effective period `interval + work`, never
//! `interval`. A deadline ticker (`sleep` until the next grid point, catch-up
//! clamped) recovers the difference. It changes only WHEN the loop wakes, never
//! what it drains or in what order.
//!
//! **Assertion discipline (the load-robustness lesson).** Contention can only push a
//! period UP, so:
//! * a CEILING on a period is load-UNSAFE in general — every ceiling here is
//!   taken over the **minimum** sample in a window, which a loaded runner can
//!   only invert by stalling *every* sampled iteration;
//! * a FLOOR on a period is load-safe, but the deadline ticker's on-grid resume
//!   can legitimately produce ONE short period after a stall, so floors here are
//!   taken over the **median**, never the minimum;
//! * the anti-hardcode oracle is DISTINCTNESS, not a magic number — a wall
//!   measurement changes every iteration, a constant does not.
//!
//! Hermetic: an isolated per-test SHM root + a unique temp socket + a memory
//! Rerun sink, with the [`DemandPlane`] injected so no `cerulion-netd` is
//! spawned and no LAN state reaches a verdict ⇒ parallel-safe, no `#[serial]`.

#![cfg(unix)]

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::clock::VirtualClock;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_viz::schema_registry::builtin_walker;
use cerulion_viz::sink::SinkState;
use cerulion_viz::worker::VizLogWorker;
use cerulion_vizd::daemon::{start_with_demand_plane, DemandPlane, RunningDaemon};
use cerulion_vizd::DEFAULT_POLL_INTERVAL;

// ── Harness ─────────────────────────────────────────────────────────────────

/// A demand plane that refuses everything: this file measures the LOCAL drain
/// loop, and an inert plane keeps `cerulion-netd` (and whatever robot is on the
/// desk's LAN) out of every number reported here.
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

fn temp_socket(tag: &str) -> (PathBuf, PathBuf) {
    use std::sync::atomic::AtomicU64;
    static N: AtomicU64 = AtomicU64::new(0);
    let n = N.fetch_add(1, Ordering::Relaxed);
    let dir = std::env::temp_dir().join(format!("{tag}_{}_{n}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("mk tempdir");
    (dir.join("vizd.sock"), dir)
}

fn isolated_transport(node_name: &str) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: node_name.to_string(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 16,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init isolated transport")
}

fn memory_worker(recording_id: &str) -> VizLogWorker {
    let (rec, _storage) = rerun::RecordingStreamBuilder::new("vizd")
        .recording_id(recording_id)
        .memory()
        .expect("memory sink");
    VizLogWorker::spawn(rec, builtin_walker(), SinkState::new()).expect("spawn worker")
}

/// A daemon plus the temp dir keeping its socket alive.
struct Harness {
    daemon: RunningDaemon,
    socket: PathBuf,
    _dir: PathBuf,
    manager: Arc<TransportManager>,
}

fn start(tag: &str, poll_interval: Duration) -> Harness {
    let manager = isolated_transport(tag);
    let (socket, dir) = temp_socket(tag);
    let daemon = start_with_demand_plane(
        socket.clone(),
        poll_interval,
        Arc::clone(&manager),
        memory_worker(tag),
        builtin_walker(),
        None, // a memory sink hosts no endpoint
        Arc::new(NoNetdPlane),
    )
    .expect("daemon starts");
    Harness {
        daemon,
        socket,
        _dir: dir,
        manager,
    }
}

/// A background `geometry_msgs/Vector3` publisher at ~100 Hz — a REAL producer
/// on a REAL iceoryx2 service (Principle #13), so the "busy" measurement is of
/// the drain loop doing its actual work, not of an empty pass.
struct Publisher {
    stop: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl Publisher {
    fn spawn(mgr: Arc<TransportManager>, topic: &str) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let stop_c = Arc::clone(&stop);
        let seq = AtomicU32::new(0);
        // The port is created on THIS thread, so `spawn` returning IS the service
        // being discoverable (the `vizd_e2e_test::Publisher::spawn` precedent).
        let mut publisher = mgr
            .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
            .unwrap_or_else(|e| panic!("producer attaches on '{topic}': {e}"));
        let handle = std::thread::spawn(move || {
            while !stop_c.load(Ordering::Relaxed) {
                let s = seq.fetch_add(1, Ordering::Relaxed);
                let _ = publisher.publish_raw(&vector3_frame(s));
                std::thread::sleep(Duration::from_millis(10));
            }
        });
        Publisher {
            stop,
            handle: Some(handle),
        }
    }
}

impl Drop for Publisher {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// A `geometry_msgs/Vector3` wire frame: the 32-byte header + three f64s.
fn vector3_frame(seq: u32) -> Vec<u8> {
    use native_ros2_messages::geometry_msgs::Vector3;
    let mut buf = vec![0u8; WireHeader::SIZE + 24];
    let header = WireHeader {
        schema_hash: <Vector3 as cerulion_core::message::ShmMessage>::SCHEMA_HASH,
        total_size: buf.len() as u32,
        offset_table_offset: 0,
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

/// A minimal control client: read the banner, then line request → line response.
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

// ── Sampling ────────────────────────────────────────────────────────────────

/// Every period the loop REPORTED over `window`, in nanoseconds.
///
/// The observable is a last-value register, so the sampler reads it far faster
/// than the loop writes it and keeps a value only when it CHANGES. Zeros (the
/// pre-first-period state) are skipped.
fn sample_periods(daemon: &RunningDaemon, window: Duration) -> Vec<u64> {
    let deadline = Instant::now() + window;
    let mut seen = Vec::new();
    let mut last = 0u64;
    while Instant::now() < deadline {
        let v = daemon.poll_loop_period_ns();
        if v != 0 && v != last {
            seen.push(v);
            last = v;
        }
        std::thread::sleep(Duration::from_micros(500));
    }
    seen
}

/// Wait (bounded) until the loop has reported at least one period, so a sampling
/// window never opens before the loop's second iteration.
fn await_first_period(daemon: &RunningDaemon, bound: Duration) {
    let deadline = Instant::now() + bound;
    while daemon.poll_loop_period_ns() == 0 {
        assert!(
            Instant::now() < deadline,
            "the drain loop reported NO period within {bound:?} — the poll-period \
             observable is dead (never stored) or the poll thread never ran"
        );
        std::thread::sleep(Duration::from_millis(2));
    }
}

/// min / median / max / mean, in milliseconds, for reporting AND for the
/// direction-aware assertions (see the module header).
struct Stats {
    n: usize,
    min_ns: u64,
    median_ns: u64,
    max_ns: u64,
    mean_ns: u64,
}

impl Stats {
    fn of(mut periods: Vec<u64>) -> Stats {
        assert!(!periods.is_empty(), "no periods sampled");
        periods.sort_unstable();
        let n = periods.len();
        Stats {
            n,
            min_ns: periods[0],
            median_ns: periods[n / 2],
            max_ns: periods[n - 1],
            mean_ns: (periods.iter().map(|v| *v as u128).sum::<u128>() / n as u128) as u64,
        }
    }

    fn report(&self, label: &str) -> String {
        format!(
            "{label}: n={} min={:.3}ms median={:.3}ms mean={:.3}ms max={:.3}ms",
            self.n,
            self.min_ns as f64 / 1e6,
            self.median_ns as f64 / 1e6,
            self.mean_ns as f64 / 1e6,
            self.max_ns as f64 / 1e6,
        )
    }
}

const SAMPLE_WINDOW: Duration = Duration::from_secs(2);

/// The held window for the sustained-overrun arm. At twice the poll interval per
/// pass this nominally contains ~15 iterations; the arm floors at 3, since load
/// can only produce fewer.
const OVERRUN_WINDOW: Duration = Duration::from_millis(500);

// ── The observable ────────────────────────────────────────────────

/// The drain loop reports its OWN measured period, and the
/// number is a measurement rather than an echo of its configuration.
///
/// Three assertions, each killing a different thing:
/// 1. **nonzero** — the observable is live at all (kills "never stored").
/// 2. **at least two DISTINCT values** — it is a wall measurement, not a
///    constant. This is the oracle for the named mutation (*hardcoding the
///    observable to the configured interval*) and, unlike an `!= 16_000_000`
///    check, it kills a hardcode to ANY constant. A real `Instant` delta at
///    nanosecond resolution repeating exactly across a two-second window is not
///    a thing that happens.
/// 3. **the MINIMUM sample is within a generous ceiling of the configured
///    interval** — it is measuring THIS loop and not something unrelated. Taken
///    over the minimum precisely so a loaded runner cannot invert it: every
///    sampled iteration in the window would have to stall past the ceiling.
#[test]
fn the_drain_loop_reports_its_own_measured_period() {
    let h = start("observable", DEFAULT_POLL_INTERVAL);
    await_first_period(&h.daemon, Duration::from_secs(5));

    let periods = sample_periods(&h.daemon, SAMPLE_WINDOW);
    let stats = Stats::of(periods.clone());
    println!(
        "poll period [idle, configured {:?}] {}",
        DEFAULT_POLL_INTERVAL,
        stats.report("period")
    );

    assert!(
        periods.iter().all(|p| *p > 0),
        "every reported period must be nonzero, got {periods:?}"
    );

    let distinct: std::collections::BTreeSet<u64> = periods.iter().copied().collect();
    assert!(
        distinct.len() >= 2,
        "the reported period never CHANGED across {SAMPLE_WINDOW:?} ({} sample(s), \
         value(s) {distinct:?}) — a wall measurement varies every iteration, so a \
         single repeated value means the observable is reporting a CONSTANT (e.g. \
         its own configured interval) rather than measuring anything",
        periods.len()
    );

    let ceiling_ns = DEFAULT_POLL_INTERVAL.as_nanos() as u64 * 20;
    assert!(
        stats.min_ns < ceiling_ns,
        "the FASTEST reported period was {:.3}ms against a configured {:?} \
         (ceiling {:.0}ms) — every sampled iteration stalled, or the observable is \
         not measuring this loop. {}",
        stats.min_ns as f64 / 1e6,
        DEFAULT_POLL_INTERVAL,
        ceiling_ns as f64 / 1e6,
        stats.report("period"),
    );
}

/// The period the loop reports TRACKS its configuration.
///
/// The anti-tautology arm for the arm above: a daemon configured four times
/// slower must report a materially slower period. Without it, an observable
/// wired to some unrelated varying quantity (a frame counter, a timestamp) would
/// satisfy every assertion in the first test.
///
/// Directions, deliberately: a **median floor** on the slow daemon (load can
/// only raise it) and a **minimum ceiling** on the fast one (load would have to
/// stall its every sampled iteration). Both margins are 2×.
#[test]
fn the_reported_period_tracks_the_configured_interval() {
    const FAST: Duration = Duration::from_millis(16);
    const SLOW: Duration = Duration::from_millis(64);

    let fast = start("cfgfast", FAST);
    let slow = start("cfgslow", SLOW);
    await_first_period(&fast.daemon, Duration::from_secs(5));
    await_first_period(&slow.daemon, Duration::from_secs(5));

    let fast_stats = Stats::of(sample_periods(&fast.daemon, SAMPLE_WINDOW));
    let slow_stats = Stats::of(sample_periods(&slow.daemon, SAMPLE_WINDOW));
    println!(
        "poll period [config tracking] {} | {}",
        fast_stats.report("16ms"),
        slow_stats.report("64ms")
    );

    assert!(
        slow_stats.median_ns >= (SLOW.as_nanos() as u64) / 2,
        "a daemon configured at {SLOW:?} reported a MEDIAN period of {:.3}ms — the \
         observable is not tracking its configuration. {}",
        slow_stats.median_ns as f64 / 1e6,
        slow_stats.report("64ms")
    );
    assert!(
        fast_stats.min_ns < (SLOW.as_nanos() as u64) / 2,
        "a daemon configured at {FAST:?} never reported a period below {:.0}ms — \
         the observable is not tracking its configuration. {}",
        (SLOW.as_nanos() as f64 / 2.0) / 1e6,
        fast_stats.report("16ms")
    );
}

/// The same measurement with the loop doing REAL work — a live
/// producer at ~100 Hz on an ATTACHED topic, so every iteration drains frames,
/// folds their headers into per-topic stats and hands a batch to the worker.
///
/// This is the arm whose printed numbers justify the deadline ticker: a fixed sleep taken
/// AFTER the work makes the busy period `interval + work`, so busy must measure
/// materially ABOVE idle. Under a deadline ticker both land on the interval.
///
/// Two assertions: a ceiling over the MINIMUM (load-safe direction), and
/// that frames were actually DRAINED, without which the arm
/// is indistinguishable from the idle one above.
///
/// The idle-vs-busy COMPARISON is printed, not asserted, because the difference
/// the cost model predicts (~6 ms) is inside the range a loaded runner can
/// manufacture on either side.
#[test]
fn the_reported_period_covers_the_loops_real_drain_work() {
    let h = start("busy", DEFAULT_POLL_INTERVAL);
    let topic = "/vel";
    let _pub = Publisher::spawn(Arc::clone(&h.manager), topic);

    let mut client = Client::connect(&h.socket);
    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(
        att["ok"].as_bool(),
        Some(true),
        "the topic must attach so the drain loop has real work: {att}"
    );

    await_first_period(&h.daemon, Duration::from_secs(5));
    let stats = Stats::of(sample_periods(&h.daemon, SAMPLE_WINDOW));

    // ANTI-VACUITY: `attach ok` proves only that the
    // iceoryx2 service existed and a tap was opened — it says nothing about a
    // frame ever crossing. A drain-path regression (the one class that can break
    // this silently; a tap-creation or schema regression cannot, since `attach`
    // errors on the first and `observe_frames` runs before any decode) would
    // leave this arm measuring a SECOND IDLE LOOP while printing a "busy" label,
    // and the arm's stimulus is its entire distinguishing feature versus the
    // idle arm above.
    //
    // `status.frames` is the oracle, and it has to be: the `schema` field of the
    // attach reply this test already holds does NOT serve, because
    // `resolve_for_attach` peeks through its own transient subscriber and is
    // satisfied with the poll loop's drain fully dead. Same one-line oracle as
    // `vizd_e2e_test.rs`'s `assert!(st["frames"] > 0, "frames counted")`.
    let status = client.request(r#"{"id":2,"method":"status"}"#);
    let st = status["topics"]
        .as_array()
        .and_then(|a| {
            a.iter()
                .find(|e| e.get("topic").and_then(|t| t.as_str()) == Some(topic))
        })
        .unwrap_or_else(|| panic!("attached topic must appear in status: {status}"));
    let frames = st["frames"].as_u64().unwrap_or(0);
    assert!(
        frames > 0,
        "the drain loop must have drained REAL frames during the measured window \
         — this arm is only distinguishable from the idle one by its work, and a \
         dead drain would have it print a 'busy' label over an idle loop. \
         status: {st}"
    );

    println!(
        "poll period [busy: 1 attached topic @~100Hz, configured {:?}, \
         {frames} frames drained] {}",
        DEFAULT_POLL_INTERVAL,
        stats.report("period")
    );

    let ceiling_ns = DEFAULT_POLL_INTERVAL.as_nanos() as u64 * 20;
    assert!(
        stats.min_ns < ceiling_ns,
        "the FASTEST reported period on a BUSY loop was {:.3}ms against a configured \
         {:?} (ceiling {:.0}ms). {}",
        stats.min_ns as f64 / 1e6,
        DEFAULT_POLL_INTERVAL,
        ceiling_ns as f64 / 1e6,
        stats.report("period"),
    );
}

// ── The deadline ticker ───────────────────────────────────────────

/// The loop is paced to a GRID, so its period is the configured
/// interval — not the interval PLUS the iteration's work and wake lag.
///
/// **The primary assertion is load-IMMUNE against a false PASS, which is why it
/// is this one.** `std::thread::sleep` guarantees *at least* the requested
/// duration, so a loop that sleeps a fixed `poll_interval` after its work has a
/// period of `>= poll_interval` **by construction** — no machine, however idle,
/// can make it produce a shorter one, and a loaded machine only makes them
/// longer. So "at least one period below the configured interval" cannot be
/// satisfied by the implementation this arm exists to reject.
///
/// The ticker supplies one for a STRUCTURAL reason, not a load-dependent one.
/// The period is measured top-to-top, so with the deadline advancing one grid
/// step it is `I + (lag_k - lag_{k-1})`: for any lag process that is not
/// monotonically increasing, about HALF of all periods fall below `I` by
/// symmetry, and this arm needs one. (It is tempting to assume that
/// sub-interval samples "appear MORE often under load, since bigger overshoots
/// buy shorter compensating sleeps". Measured, that is
/// false: the FRACTION below the interval is flat at 49-56 % under CPU
/// oversubscription up to 512 spinners and FALLS to ~31 % under timer-subsystem
/// pressure. What load improves is the DEPTH — the minimum went 8.25 ms → 0.74 ms
/// — not the frequency.)
///
/// Residual: the assertion is not immune to a false FAILURE. If the loop
/// were persistently late by more than a WHOLE interval the deadline would
/// advance two grid steps and the period would centre on `2I`, where a
/// sub-interval sample needs a lag drop exceeding a full interval. A sweep
/// of that boundary gives: P(fail) is 0 % for mean lag up to and including one
/// interval, 20 % at 17 ms, 100 % at 20 ms — against a measured p50 wake lag of
/// 3.2 ms on a desk machine, and 14 real runs under
/// CPU and timer-pressure load produced no failure. The failure message names both causes so a
/// hypothetical trip is not misread as a code revert.
///
/// The second assertion is the anti-spin floor, and it is not optional: a loop
/// that stopped waiting altogether would satisfy the first assertion trivially.
/// A median FLOOR is the load-safe direction (contention can only raise it).
///
/// MEASURED on a desk machine (`--nocapture` prints them): with a fixed post-work sleep the 16 ms loop
/// ran at a **20.28 ms median / 20.43 ms mean with a 16.05 ms minimum** — no
/// sample below the interval, ever. With the deadline ticker: **15.84 ms median / 16.04 ms
/// mean with a 9.14 ms minimum**.
#[test]
fn the_drain_loop_is_paced_to_a_grid_not_to_a_fixed_sleep_after_its_work() {
    let h = start("ticker", DEFAULT_POLL_INTERVAL);
    await_first_period(&h.daemon, Duration::from_secs(5));

    let stats = Stats::of(sample_periods(&h.daemon, SAMPLE_WINDOW));
    println!(
        "poll period [deadline ticker, configured {:?}] {}",
        DEFAULT_POLL_INTERVAL,
        stats.report("period")
    );

    let interval_ns = DEFAULT_POLL_INTERVAL.as_nanos() as u64;
    assert!(
        stats.min_ns < interval_ns,
        "NO sampled period was below the configured {:?} (fastest {:.3}ms over {} \
         samples). `sleep` never returns early, so the likely cause is that the \
         wait was reverted to a FIXED span AFTER the work, making the period \
         `interval + work + wake lag` — check the printed MEDIAN, which reads \
         ~21ms for that revert. A median near 2x the interval instead means the \
         loop is persistently more than one WHOLE interval late (the deadline \
         advances two grid steps and the period centres on 2I), which is a \
         runner-health problem, not a code revert. {}",
        DEFAULT_POLL_INTERVAL,
        stats.min_ns as f64 / 1e6,
        stats.n,
        stats.report("period"),
    );

    let floor_ns = interval_ns * 4 / 5;
    assert!(
        stats.median_ns >= floor_ns,
        "the MEDIAN period was {:.3}ms against a configured {:?} (floor {:.1}ms) — \
         the loop is running FASTER than its interval, i.e. it is no longer waiting. {}",
        stats.median_ns as f64 / 1e6,
        DEFAULT_POLL_INTERVAL,
        floor_ns as f64 / 1e6,
        stats.report("period"),
    );
}

/// A loop that is PERSISTENTLY LATE still yields
/// the CPU on every pass.
///
/// **The condition.** Waiting to a deadline means the remaining time is negative
/// once a pass's own wall exceeds the interval — and, because the ticker only
/// ever puts the next deadline one interval ahead of `now`, it stays negative on
/// every pass thereafter. A loop that simply slept the remainder therefore takes
/// **zero** waits for as long as the overrun lasts, where a fixed
/// post-work sleep yields ≥ 16 ms unconditionally. That is a duty-cycle
/// regression the "record-only" framing does not cover: the drain re-acquires
/// `ctx.state` microseconds after releasing it, and ~25 controller-verb sites
/// contend for that same lock with no guaranteed gap.
///
/// **Why the work is injected.** A real drain pass was MEASURED (over real
/// iceoryx2, through the exact `drain_owned` + copy + header-parse path) at
/// 0.009 ms for 75 empty taps and 0.230 ms for 75 fully saturated ones, against
/// a 16 ms interval. No arrangement of real topics gets within 50× of the
/// regime, so a test that tried to reach it with publishers would be asserting
/// on the runner's speed rather than on the loop.
///
/// **The oracle is structural, not statistical.** `plan_tick_sleep` floors its
/// already-late arm, so `poll_loop_sleepless_passes()` is 0 by construction — at
/// any load, on any runner, for any overrun.
///
/// **Which is exactly why the arm must also assert its own STIMULUS.**
/// A count that is structurally zero is zero on a *healthy* loop too,
/// so with the injector inert this arm goes green — printing a "sustained
/// overrun" label over an ordinary 16 ms grid — and the seam's plumbing is three
/// lines sitting between two visually identical sibling lines in two struct
/// literals. That is the same class as on the busy arm ("a busy
/// label over an idle loop"), and it is closed the same way: by measuring the
/// stimulus rather than assuming it.
///
/// Three assertions, in the order they must fail:
/// 1. **Iterations floor** — anti-vacuity. A window the loop barely ran in makes
///    everything below trivially true.
/// 2. **Late passes ≥ iterations − 1** — THE stimulus pin, and one-sided by
///    construction: contention can only ADD late passes. (`− 1` because the
///    first pass of the window may have been armed before the injector was.)
/// 3. **Iterations ceiling** — the discriminating check. Each pass
///    burns 2 × 16 ms of CPU by construction, so a 500 ms window admits at most
///    ~15; a healthy loop does ~31. The ceiling sits at 20, above the injected
///    nominal and well below the healthy one. Load can only push it DOWN.
/// 4. **Median period floor** — sampled BEFORE the injector is disarmed, because
///    a sample taken after it reports the RECOVERED loop (the compounding
///    hazard: printed stats that describe a loop that has already
///    healed, under an overrun label).
#[test]
fn a_persistently_late_drain_loop_still_yields_on_every_pass() {
    let h = start("overrun", DEFAULT_POLL_INTERVAL);
    await_first_period(&h.daemon, Duration::from_secs(5));

    // Twice the interval per pass, so the loop is late on EVERY pass and can
    // never recover onto the grid — the sustained regime, not a transient stall
    // (a stall costs exactly one missed deadline: the ticker recomputes `now`
    // after the wait, so the next deadline is strictly ahead again).
    let injected = DEFAULT_POLL_INTERVAL * 2;
    h.daemon
        .inject_poll_work_for_test(injected.as_nanos() as u64);

    let before_iters = h.daemon.poll_loop_iterations();
    let before_sleepless = h.daemon.poll_loop_sleepless_passes();
    let before_late = h.daemon.poll_loop_late_passes();
    // Sample DURING the overrun, not after: the periods are the evidence that
    // the regime was real, and disarming first would sample the recovered loop.
    let periods = sample_periods(&h.daemon, OVERRUN_WINDOW);
    let iters = h.daemon.poll_loop_iterations() - before_iters;
    let sleepless = h.daemon.poll_loop_sleepless_passes() - before_sleepless;
    let late = h.daemon.poll_loop_late_passes() - before_late;
    h.daemon.inject_poll_work_for_test(0);

    println!(
        "poll period [sustained overrun: {injected:?}/pass, configured {:?}] \
         iterations={iters} late={late} sleepless={sleepless} | {}",
        DEFAULT_POLL_INTERVAL,
        if periods.is_empty() {
            "period: (no samples)".to_string()
        } else {
            Stats::of(periods.clone()).report("period")
        }
    );

    // (1) Anti-vacuity, and it must come first: a window in which the loop barely
    // ran would make every count below trivially small. The floor is deliberately
    // far below the ~15 passes the window nominally contains, because a loaded
    // runner can only produce FEWER.
    assert!(
        iters >= 3,
        "the loop went round only {iters} time(s) in {OVERRUN_WINDOW:?} — the \
         overrun window is too short to say anything about its waits"
    );

    // (2) THE STIMULUS PIN. A healthy loop reaches the already-late arm on
    // essentially no pass; an overrunning one reaches it on every pass. This is
    // what fails when the injector is inert, and it is load-SAFE in the only
    // direction that matters: contention makes passes LATER, never earlier.
    //
    // `+ 2` covers the two BOUNDARY passes the window can straddle: the pass
    // already in flight when the injector was armed (counted before the window
    // opened, so it lands in `late` but not `iters`) and the pass whose top the
    // window contains but whose wait it does not (the reverse). Everything in
    // between is late by construction. Against an inert injector this reads
    // late=0 of ~31 iterations, so the margin costs no discrimination at all.
    assert!(
        late + 2 >= iters,
        "only {late} of {iters} passes reached the already-late arm — the \
         overrun regime was NEVER ENTERED, so this arm is measuring an ordinary \
         healthy loop under a 'sustained overrun' label. The injection seam \
         (`inject_poll_work_for_test` → `Ctx::poll_injected_work_ns` → the \
         per-pass read in `poll_loop`) is disconnected."
    );

    // (3) The iteration CEILING — the discriminating check, and an
    // independent check on the same thing: each injected pass burns
    // {injected:?} of CPU, so the window admits at most
    // OVERRUN_WINDOW/injected ≈ 15 passes, against ~31 for a healthy 16 ms loop.
    let ceiling = 20;
    assert!(
        iters <= ceiling,
        "the loop completed {iters} passes in {OVERRUN_WINDOW:?} (ceiling \
         {ceiling}) — at {injected:?} of CPU per pass that is impossible, so the \
         injected work is not being burned"
    );

    // (4) The period the loop ACTUALLY ran at during the overrun — the
    // "compounding" half: samples taken AFTER
    // the injector is disarmed describe the recovered loop instead. Work-bound,
    // so a FLOOR, which load can only raise. Set at 3/4 of the injected span:
    // the window straddles the arming boundary, so a sample or two is a
    // pre-injection ~16 ms period, and 3/4 keeps the median clear of those while
    // still sitting far above the ~16 ms a healthy or recovered loop reports.
    let stats = Stats::of(periods);
    let period_floor_ns = injected.as_nanos() as u64 * 3 / 4;
    assert!(
        stats.median_ns >= period_floor_ns,
        "the MEDIAN period during the overrun was {:.3}ms (floor {:.1}ms against \
         {injected:?} of work per pass) — the samples do not describe the \
         overrun, so either the injection is inert or they were taken after it \
         was disarmed. {}",
        stats.median_ns as f64 / 1e6,
        period_floor_ns as f64 / 1e6,
        stats.report("period")
    );

    // (5) And the floor did its job throughout.
    assert_eq!(
        sleepless, 0,
        "the drain loop took {sleepless} pass(es) with NO wait at all across \
         {iters} iterations of sustained {injected:?}-per-pass overrun. Waiting \
         only the remaining time to a missed deadline waits nothing, so the loop \
         runs back-to-back and re-takes `ctx.state` microseconds after releasing \
         it — where the old fixed sleep yielded unconditionally. \
         `plan_tick_sleep` must floor its already-late arm."
    );
}
