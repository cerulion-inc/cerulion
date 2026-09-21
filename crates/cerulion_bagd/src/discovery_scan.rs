// SPDX-License-Identifier: AGPL-3.0-only
//! The live-topic ENUMERATION, taken OFF the recorder's drive loop.
//!
//! # The defect
//!
//! The discovery rescan called [`TransportManager::list_topics`] **inline on
//! `drive_loop`** every [`DISCOVERY_RESCAN_INTERVAL`] (250 ms). That call walks
//! the iceoryx2 service directory, and its cost scales with the number of live
//! services: MEASURED first-party on the Go2's Jetson at **117.5 ms p50 with 86
//! live topics** (~1.35 ms per topic; ~0.41 ms per topic on an M3 Max desk). So
//! roughly a THIRD of every rescan period was spent not draining any tap, and
//! the fast topics' 16-frame SHM queues overflowed inside that window —
//! **23 % of frames lost** on a full `graph run attach --record`, on exactly
//! those topics at or above 200 Hz, with `staging_full_passes = 0` and
//! `dropped_unwritten = 0` throughout (the writer was never the constraint). `drain_taps` already drains
//! every tap to empty on every pass, so no drain-side change could help: the
//! frames were lost in a window where the drain did not run at all.
//!
//! # The fix, and what it deliberately does NOT change
//!
//! Enumeration moves to its OWN thread, which publishes an immutable snapshot
//! the drive loop consumes in O(1). The drive loop never blocks on a directory
//! walk again.
//!
//! Everything the recorder DECIDES from that snapshot is byte-unchanged and
//! stays where it was (`Recorder::apply_discovery` → `discovery::plan_discovery`)
//! — this module produces scans, it does not judge them. In particular the
//! discovery settle semantics are preserved by CONSTRUCTION rather than by
//! re-derivation:
//!
//! * **One applied scan per enumeration.** The slot holds at most one scan and
//!   [`DiscoveryScanner::next_scan`] TAKES it, so a scan is applied exactly once
//!   and `discovery_quiet_scans` counts exactly what it counted before — the
//!   number of enumerations that added no tap. A scan overwritten before the
//!   loop took it is a scan the loop never saw, which is precisely what happened
//!   earlier when the loop was too busy to reach its 250 ms check.
//! * **The same cadence.** The thread sleeps `interval` AFTER each scan
//!   completes, so the period is `interval + T` — the same period the drive
//!   loop produced by re-arming its timer after the rescan. A fixed grid would
//!   scan slightly more often; that would be a behaviour change to the settle
//!   window's timing for no benefit this fix needs.
//! * **The arm-time scan stays INLINE** (`Recorder::setup`). It runs before the
//!   graph is released to step 0, so it costs no frames, and its result must be
//!   in hand before `discovery_armed` flips.
//!
//! # The failure mode this introduces, and how it is reported
//!
//! A background scanner can die or wedge where an inline call could not, and a
//! recorder whose discovery has silently gone inert is the unrecorded-producer defect
//! wearing a different hat. Two defences, both loud:
//!
//! * If the thread cannot be SPAWNED, the scanner degrades to the earlier
//!   INLINE path at the same cadence — the recording keeps its coverage and
//!   pays the old cost — with one `warn!` naming the degradation.
//! * If the thread was spawned but stops producing, the silence is classified by
//!   the pure [`classify_scanner_silence`] and reported once per regime by the
//!   pure [`ScanSilenceLatch`], with a recovery line when scans resume.
//!
//! # Thread lifetime
//!
//! The worker holds its own `Arc<TransportManager>` clone, so everything it
//! touches outlives it by construction. Sharing a manager across threads is not
//! new territory: `cerulion-netd`'s mirror, egress and query planes each hold
//! one beside their own drive threads, and `TransportManager` carries no manual
//! `unsafe impl Sync` — its thread-safety is structural, derived from its fields
//! (iceoryx2's `ipc_threadsafe` service). The worker is also strictly a READER:
//! `list_topics` walks the service directory and mutates nothing, and it already
//! ran concurrently with every OTHER process's service creation, which is the
//! only interleaving it can now additionally see within this one.
//!
//! Its `JoinHandle` is DROPPED (detached)
//! and `Drop` only sets the stop flag — a recorder must never wait on a
//! diagnostic thread at teardown (the `schema_resolve::SchemaResolver`
//! precedent). The flag is checked between sleep slices and before each scan, so
//! the worker exits within one [`SCANNER_STOP_SLICE`] plus at most one
//! in-flight enumeration.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::TransportManager;

use crate::discovery::DISCOVERY_RESCAN_INTERVAL;

/// How long the worker sleeps between stop-flag checks while waiting out its
/// cadence. Small enough that teardown is prompt, large enough that an idle
/// scanner is not a poll loop (10 wakeups/second).
pub const SCANNER_STOP_SLICE: Duration = Duration::from_millis(100);

/// How many scan cadences of SILENCE mark the scanner as stalled.
///
/// Generous on purpose, and the two errors are asymmetric: reporting late costs
/// a log line, while reporting early would cry wolf on every recorder whose
/// drive loop was simply busy elsewhere for a moment. Eight cadences is 2 s at
/// the shipped interval, against a real period of `interval + T` (~367 ms on the
/// Jetson) — i.e. ~5 missed scans before anything is said.
pub const SCANNER_SILENCE_INTERVALS: u32 = 8;

/// One completed enumeration of the live topic set.
///
/// The `Err` arm is CARRIED rather than swallowed: the coverage manifest
/// counts enumeration failures and refuses to claim `enumerated` on a run whose
/// every scan failed, so a scanner that quietly dropped errors would restore the
/// exact false claim that flag exists to prevent.
#[derive(Debug)]
pub struct TopicScan {
    /// The live topic set, or the enumeration error rendered for the report.
    pub result: Result<Vec<String>, String>,
}

/// PURE: has the scanner been quiet long enough to be worth reporting?
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScannerHealth {
    /// A scan arrived recently enough — nothing to say.
    Healthy,
    /// No scan for [`SCANNER_SILENCE_INTERVALS`] cadences.
    Stalled,
}

/// PURE: classify the scanner's silence against its own cadence.
///
/// A ZERO interval means there is no cadence to be late against, so it can never
/// be stalled — a caller that disabled the scanner must not be told its
/// non-existent thread died.
pub fn classify_scanner_silence(since_last_scan: Duration, interval: Duration) -> ScannerHealth {
    if interval.is_zero() {
        return ScannerHealth::Healthy;
    }
    if since_last_scan >= interval * SCANNER_SILENCE_INTERVALS {
        ScannerHealth::Stalled
    } else {
        ScannerHealth::Healthy
    }
}

/// What the caller should LOG about the scanner's health this pass.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SilenceReport {
    /// Say nothing.
    Quiet,
    /// First observation of a stall — report it loudly.
    Loud,
    /// Scans resumed after a reported stall — say so once.
    Recovered,
}

/// PURE: once-per-regime suppression for the stall report.
///
/// The health check runs on EVERY drive pass (~100/s), so an unlatched warn
/// would flood a log at the exact moment the operator most needs to read it —
/// the disk-fill class. Deliberately a two-state latch rather than a
/// [`FailureRegimeLatch`](cerulion_core::transport::failure_regime_latch::FailureRegimeLatch)
/// consumer: there is no per-occurrence count to suppress here (the condition is
/// a level, not an event stream), so a decade ladder would report a number that
/// means nothing but "how often did we look".
#[derive(Debug, Default)]
pub struct ScanSilenceLatch {
    reported: bool,
}

impl ScanSilenceLatch {
    /// Fold one health observation into the latch and say what to log.
    pub fn observe(&mut self, health: ScannerHealth) -> SilenceReport {
        match (health, self.reported) {
            (ScannerHealth::Stalled, false) => {
                self.reported = true;
                SilenceReport::Loud
            }
            (ScannerHealth::Stalled, true) => SilenceReport::Quiet,
            (ScannerHealth::Healthy, true) => {
                self.reported = false;
                SilenceReport::Recovered
            }
            (ScannerHealth::Healthy, false) => SilenceReport::Quiet,
        }
    }
}

/// The threaded half: a worker publishing into a one-slot mailbox.
struct ThreadedScanner {
    /// The newest completed scan, or `None` when the loop has already taken it.
    latest: Arc<Mutex<Option<TopicScan>>>,
    /// Total enumerations COMPLETED by the worker (Principle #3 observable).
    scans_run: Arc<AtomicU64>,
    stop: Arc<AtomicBool>,
}

impl ThreadedScanner {
    fn start(mgr: Arc<TransportManager>, interval: Duration) -> Option<Self> {
        let latest: Arc<Mutex<Option<TopicScan>>> = Arc::new(Mutex::new(None));
        let scans_run = Arc::new(AtomicU64::new(0));
        let stop = Arc::new(AtomicBool::new(false));
        let (slot, scans, flag) = (
            Arc::clone(&latest),
            Arc::clone(&scans_run),
            Arc::clone(&stop),
        );
        match std::thread::Builder::new()
            .name("bagd-discovery-scan".to_string())
            .spawn(move || scan_loop(mgr, interval, slot, scans, flag))
        {
            // The handle is DROPPED on purpose — see the module docs.
            Ok(_handle) => Some(Self {
                latest,
                scans_run,
                stop,
            }),
            Err(e) => {
                tracing::warn!(
                    error = %e,
                    "bagd could not start the discovery-scan thread — falling back to enumerating \
                     the live topic set INLINE on the recorder's drive loop: on a many-topic \
                     machine that costs the drain roughly a third of \
                     every rescan period and fast topics can overflow their queues. Coverage is \
                     unaffected; throughput is not"
                );
                None
            }
        }
    }

    /// Take the newest completed scan, if the worker has published one since the
    /// last call. A poisoned mailbox is treated as "nothing new": a diagnostic
    /// mutex must never wedge the recording it is only describing.
    fn take(&self) -> Option<TopicScan> {
        self.latest.lock().ok().and_then(|mut slot| slot.take())
    }
}

impl Drop for ThreadedScanner {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

/// The worker body: scan, publish, sleep, repeat until stopped.
fn scan_loop(
    mgr: Arc<TransportManager>,
    interval: Duration,
    latest: Arc<Mutex<Option<TopicScan>>>,
    scans_run: Arc<AtomicU64>,
    stop: Arc<AtomicBool>,
) {
    while !stop.load(Ordering::Relaxed) {
        let result = mgr.list_topics().map_err(|e| e.to_string());
        // Publish BEFORE bumping the counter, so an observer that sees the count
        // move can always take the scan that moved it.
        if let Ok(mut slot) = latest.lock() {
            *slot = Some(TopicScan { result });
        }
        scans_run.fetch_add(1, Ordering::Relaxed);
        sleep_unless_stopped(interval, &stop);
    }
}

/// Sleep `total`, in slices, returning early once `stop` is set.
fn sleep_unless_stopped(total: Duration, stop: &AtomicBool) {
    let deadline = Instant::now() + total;
    loop {
        if stop.load(Ordering::Relaxed) {
            return;
        }
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return;
        }
        std::thread::sleep(remaining.min(SCANNER_STOP_SLICE));
    }
}

/// How the live topic set is being enumerated for this recording.
enum ScanEngine {
    /// Discovery is off — nothing enumerates, and the silence detector is inert.
    Off,
    /// The threaded shape: a worker thread, consumed in O(1).
    Threaded(ThreadedScanner),
    /// The earlier inline shape, reached only when the thread could not be spawned.
    Inline {
        /// When the last inline scan COMPLETED — re-armed after the scan, which
        /// is what `drive_loop` did before the threaded scan existed.
        last: Option<Instant>,
    },
}

/// The drive loop's handle on the live-topic enumeration.
pub struct DiscoveryScanner {
    engine: ScanEngine,
    interval: Duration,
    /// When the loop last took a scan (or when the scanner started) — the clock
    /// [`classify_scanner_silence`] measures against.
    last_scan_at: Instant,
    silence: ScanSilenceLatch,
}

impl DiscoveryScanner {
    /// Start scanning `mgr`'s namespace, unless `discover_live` is off.
    pub fn start(mgr: &Arc<TransportManager>, discover_live: bool) -> Self {
        Self::start_with_interval(mgr, discover_live, DISCOVERY_RESCAN_INTERVAL)
    }

    /// [`Self::start`] with an explicit cadence — the seam tests drive.
    pub fn start_with_interval(
        mgr: &Arc<TransportManager>,
        discover_live: bool,
        interval: Duration,
    ) -> Self {
        let engine = if !discover_live {
            ScanEngine::Off
        } else {
            match ThreadedScanner::start(Arc::clone(mgr), interval) {
                Some(t) => ScanEngine::Threaded(t),
                None => ScanEngine::Inline { last: None },
            }
        };
        Self {
            engine,
            interval,
            last_scan_at: Instant::now(),
            silence: ScanSilenceLatch::default(),
        }
    }

    /// The newest enumeration the recorder has not applied yet, if any.
    ///
    /// O(1) on the threaded path (a mutex take). On the inline fallback it is
    /// the old cadence-gated `list_topics()` call, cost and all.
    pub fn next_scan(&mut self, mgr: &TransportManager) -> Option<TopicScan> {
        let scan = match &mut self.engine {
            ScanEngine::Off => None,
            ScanEngine::Threaded(t) => t.take(),
            ScanEngine::Inline { last } => {
                if last.is_none_or(|t| t.elapsed() >= self.interval) {
                    let scan = TopicScan {
                        result: mgr.list_topics().map_err(|e| e.to_string()),
                    };
                    *last = Some(Instant::now());
                    Some(scan)
                } else {
                    None
                }
            }
        };
        self.report_health(scan.is_some());
        scan
    }

    /// Map this pass's silence verdict onto a log line.
    ///
    /// Only the THREADED engine can go silent independently of the caller: the
    /// inline fallback runs on the drive loop itself, so its silence would be
    /// the loop's own, and `Off` has nothing to be silent about.
    fn report_health(&mut self, took_a_scan: bool) {
        if !matches!(self.engine, ScanEngine::Threaded(_)) {
            return;
        }
        if took_a_scan {
            self.last_scan_at = Instant::now();
        }
        let health = classify_scanner_silence(self.last_scan_at.elapsed(), self.interval);
        match self.silence.observe(health) {
            SilenceReport::Quiet => {}
            SilenceReport::Loud => tracing::warn!(
                silent_for_ms = self.last_scan_at.elapsed().as_millis() as u64,
                cadence_ms = self.interval.as_millis() as u64,
                "bagd's discovery-scan thread has produced NO enumeration for many cadences — \
                 live-topic discovery is not running, so a producer appearing from here on will \
                 be neither recorded nor named in this bag's coverage manifest"
            ),
            SilenceReport::Recovered => tracing::info!(
                "bagd's discovery-scan thread is producing enumerations again — live-topic \
                 discovery has resumed"
            ),
        }
    }

    /// Total enumerations the worker has COMPLETED (Principle #3).
    ///
    /// `0` on the `Off` and inline-fallback engines, which run no worker.
    pub fn scans_run(&self) -> u64 {
        match &self.engine {
            ScanEngine::Threaded(t) => t.scans_run.load(Ordering::Relaxed),
            _ => 0,
        }
    }

    /// Whether a worker thread is actually running (Principle #3) — `false` for
    /// both a disabled scanner and one that degraded to the inline fallback.
    pub fn is_threaded(&self) -> bool {
        matches!(self.engine, ScanEngine::Threaded(_))
    }
}

// ===========================================================================
// Pure oracles for the two decisions above. No transport, no threads.
// ===========================================================================
#[cfg(test)]
mod tests {
    use super::*;

    const CADENCE: Duration = Duration::from_millis(250);

    /// The stall threshold, pinned on BOTH sides at one cadence's resolution.
    ///
    /// A one-sided assertion is satisfied by an always-Stalled classifier, which
    /// would warn on the first drive pass of every healthy recording.
    #[test]
    fn the_stall_threshold_is_exactly_the_declared_number_of_cadences() {
        let threshold = CADENCE * SCANNER_SILENCE_INTERVALS;
        // Hand oracle: (silence, expected verdict).
        let oracle = [
            (Duration::ZERO, ScannerHealth::Healthy),
            (CADENCE, ScannerHealth::Healthy),
            (
                CADENCE * (SCANNER_SILENCE_INTERVALS - 1),
                ScannerHealth::Healthy,
            ),
            (threshold - Duration::from_nanos(1), ScannerHealth::Healthy),
            (threshold, ScannerHealth::Stalled),
            (threshold + Duration::from_nanos(1), ScannerHealth::Stalled),
            (threshold * 100, ScannerHealth::Stalled),
        ];
        for (silence, want) in oracle {
            assert_eq!(
                classify_scanner_silence(silence, CADENCE),
                want,
                "silence {silence:?} against a {CADENCE:?} cadence"
            );
        }
    }

    /// A scanner with no cadence can never be late — the `Off` engine's guard.
    #[test]
    fn a_zero_cadence_is_never_stalled_however_long_the_silence() {
        for silence in [Duration::ZERO, CADENCE, Duration::from_secs(3600)] {
            assert_eq!(
                classify_scanner_silence(silence, Duration::ZERO),
                ScannerHealth::Healthy,
                "a zero cadence has nothing to be late against (silence {silence:?})"
            );
        }
    }

    /// The full latch cycle against a HAND-WRITTEN report vector.
    ///
    /// Written as one sequence rather than four assertions because the property
    /// is the TRANSITIONS: a latch that reports Loud once and never recovers,
    /// and one that recovers without ever having reported, both satisfy any
    /// single-step assertion.
    #[test]
    fn the_silence_latch_reports_once_per_regime_and_recovers_once() {
        use ScannerHealth::{Healthy, Stalled};
        use SilenceReport::{Loud, Quiet, Recovered};
        // (observed health, expected report)
        let oracle = [
            (Healthy, Quiet), // a healthy scanner says nothing...
            (Healthy, Quiet), // ...however often it is asked
            (Stalled, Loud),  // first stall: LOUD
            (Stalled, Quiet), // the regime is open — suppressed
            (Stalled, Quiet),
            (Healthy, Recovered), // scans resumed: exactly one recovery line
            (Healthy, Quiet),     // ...and only one
            (Stalled, Loud),      // a FRESH regime is loud again
            (Healthy, Recovered),
        ];
        let mut latch = ScanSilenceLatch::default();
        for (i, (health, want)) in oracle.into_iter().enumerate() {
            assert_eq!(
                latch.observe(health),
                want,
                "step {i}: observing {health:?} must report {want:?}"
            );
        }
    }

    /// A latch that never sees a stall must never emit a recovery — otherwise
    /// every healthy recording logs "discovery has resumed" for a stall that
    /// never happened.
    #[test]
    fn a_scanner_that_was_never_stalled_never_reports_a_recovery() {
        let mut latch = ScanSilenceLatch::default();
        for _ in 0..1000 {
            assert_eq!(latch.observe(ScannerHealth::Healthy), SilenceReport::Quiet);
        }
    }
}
