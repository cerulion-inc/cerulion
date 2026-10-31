// SPDX-License-Identifier: AGPL-3.0-only
//! The live-topic ENUMERATION: off the recorder's drive loop, and driven by
//! EVENTS rather than by a clock.
//!
//! # The defect, in two halves
//!
//! The recorder learns about producers it was never told about by enumerating
//! the live topic set. `TransportManager::list_topics` is the only way to ask,
//! and it walks the iceoryx2 service directory at a cost that scales with the
//! machine's live service count: MEASURED first-party at **117.5 ms p50 with 86
//! live topics** (~1.35 ms per topic; ~0.41 ms per topic on a desk).
//!
//! The first half of the defect was WHERE that walk happened. It ran inline on
//! `drive_loop` every 250 ms, so roughly a third of every period was spent not
//! draining any tap, and the fast topics' 16-frame SHM queues overflowed inside
//! that window: **23 % of frames lost** on a full recording of a robot's
//! firehose, with `staging_full_passes = 0` and `dropped_unwritten = 0`
//! throughout (the writer was never the constraint). `drain_taps` already drains
//! every tap to empty on every pass, so no drain-side change could help - the
//! frames died in a window where the drain did not run at all.
//!
//! The second half was WHEN. Moving the walk to a worker thread stops it
//! costing frames, but it does not stop it happening: a settled machine still
//! paid four full directory walks a second, forever, to be told nothing had
//! changed. On a robot's compute module that is real, permanent CPU spent
//! answering a question nobody asked.
//!
//! # The fix
//!
//! The worker BLOCKS ON THE KERNEL instead of on a clock. A producer
//! registering is a file appearing in the iceoryx2 service directory (see
//! [`crate::service_dir_watch`]), so `inotify` on Linux and `kqueue` on macOS
//! report it. The worker walks the directory when something happened, and only
//! then.
//!
//! * A machine whose topic set never changes runs **zero** enumerations after
//!   its baseline one. Not a slow poll - none.
//! * A machine that opens a route pays one walk, promptly.
//! * A machine that opens twenty routes at once pays roughly one walk, because
//!   a burst is coalesced: the answer to every event in it is the same walk.
//!
//! The worker still wakes at most ten times a second to check its stop flag and
//! stamp a liveness heartbeat. Those wakes do NO work - they are a syscall
//! returning - and are how a detached diagnostic thread stays both promptly
//! stoppable and observably alive. Discovery itself runs on events alone.
//!
//! # The two things an event does not tell us, and what is done about them
//!
//! An event says "that directory changed", not "a new topic is listable now".
//!
//! * **Bursts.** Twenty routes opening back to back are one logical change.
//!   After the first wake the worker keeps absorbing events for
//!   [`EVENT_COALESCE_WINDOW`] and then walks once.
//! * **Visibility lag.** A service becomes visible to `Service::list` only once
//!   its static-config file carries its final permissions, which is shortly
//!   AFTER the file appears. On Linux that chmod is itself an event; on macOS a
//!   chmod of a file inside a directory is not a change to the directory, so it
//!   is not. Rather than ship two behaviours, every event-driven walk is
//!   followed by a fixed tail of [`CONFIRM_WALKS`] confirmation walks spaced
//!   [`EVENT_CONFIRM_DELAY`] apart, re-armed from scratch if another event
//!   arrives first. It is a bounded tail on a real change, not a cadence:
//!   nothing happening means none of those walks happens.
//!
//! A [`SCAN_RATE_FLOOR`] bounds the worst case from the other side. However
//! chatty the directory becomes, two walks are never started closer together
//! than that - which is a rate LIMIT, and costs an idle recorder nothing
//! because an idle recorder starts none.
//!
//! # What this module deliberately does NOT change
//!
//! Everything the recorder DECIDES from a scan is untouched and stays where it
//! was (`Recorder::apply_discovery` → `discovery::plan_discovery`). This module
//! produces scans; it does not judge them. Two properties are preserved by
//! construction:
//!
//! * **One applied scan per enumeration.** The slot holds at most one scan and
//!   [`DiscoveryScanner::next_scan`] TAKES it, so the drive loop can never be
//!   handed the same enumeration twice.
//! * **The arm-time scan stays INLINE** (`Recorder::setup`). It runs before the
//!   graph is released to step 0, so it costs no frames, and its result must be
//!   in hand before `discovery_armed` flips.
//!
//! The settle window that holds bag creation open no longer counts scans - it
//! measures the wall since the last discovered tap (see
//! `Recorder::discovery_hold_active`). That had to change: an event-driven
//! scanner is silent on a settled machine, so a rule phrased as "N consecutive
//! scans found nothing" would never be satisfied and every plain recording
//! would pay the whole settle cap.
//!
//! # Failure is loud at every tier
//!
//! A recorder whose discovery has gone quiet is the unrecorded-producer defect
//! wearing a different hat, so no degradation is silent:
//!
//! * The watch cannot be ARMED (an unsupported platform, no directory, a
//!   descriptor limit) - the worker falls back to walking on
//!   [`DISCOVERY_RESCAN_INTERVAL`], with one `warn!` naming why.
//! * The watch BREAKS mid-run - same fallback, same loudness. The timed engine
//!   is then owed the walk and confirmations one change is answered with, so a
//!   change the watch saw and never answered still holds the settle window
//!   until the timed engine has looked.
//!   [`DiscoveryScanner::wake_source`] reads `Poll` from then on; in a
//!   recording the degrade is reported by the `warn!`.
//! * The thread cannot be SPAWNED - the scanner degrades to the earlier INLINE
//!   walk on the drive loop at the same cadence, which costs frames but never
//!   coverage, with one `warn!`.
//! * The thread was spawned and stopped heart-beating - classified by the pure
//!   [`classify_scanner_silence`] and reported once per regime by the pure
//!   [`ScanSilenceLatch`], with a recovery line when it resumes.
//!
//! # Thread lifetime
//!
//! The worker holds its own `Arc<TransportManager>` clone, so everything it
//! touches outlives it by construction. It is strictly a READER: `list_topics`
//! walks the service directory and mutates nothing.
//!
//! Its `JoinHandle` is DROPPED (detached) and `Drop` only sets the stop flag - a
//! recorder must never wait on a diagnostic thread at teardown (the
//! `schema_resolve::SchemaResolver` precedent). The flag is checked between
//! waits, so the worker exits within one [`SCANNER_STOP_SLICE`] plus at most one
//! in-flight enumeration.

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicU8, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::TransportManager;

use crate::discovery::DISCOVERY_RESCAN_INTERVAL;
use crate::service_dir_watch::{ServiceDirWatch, WatchWake};

/// The longest the worker blocks in one wait.
///
/// Bounds two things at once: how promptly a detached worker notices its stop
/// flag, and how often it stamps the heartbeat its liveness is read from. NO
/// enumeration happens on a wake that found nothing - this is a syscall
/// returning, not a scan.
pub const SCANNER_STOP_SLICE: Duration = Duration::from_millis(100);

/// How long a burst is absorbed before it is walked.
///
/// A robot's bridge opening seventy routes produces a storm of file events whose
/// correct answer is ONE directory walk. Long enough to swallow a burst of route
/// creations, short enough that a single new topic is discovered promptly.
pub const EVENT_COALESCE_WINDOW: Duration = Duration::from_millis(50);

/// The spacing of the confirmation walks that follow an event-driven one.
///
/// Closes the visibility lag between a service's static-config file appearing
/// and its permissions being finalized, which is when `Service::list` starts
/// reporting it. On a real change only - never a repeating tick.
pub const EVENT_CONFIRM_DELAY: Duration = Duration::from_millis(250);

/// How many confirmation walks follow an event-driven one.
///
/// TWO rather than one, and the reason is a platform asymmetry. On Linux the
/// permissions change is itself an event (`IN_ATTRIB`), so a late one produces
/// its own wake and the design is self-correcting. On macOS `EVFILT_VNODE`
/// watches the DIRECTORY, and a chmod of a file inside it is not a change to
/// the directory - so the confirmation is the entire cover. A single shot that
/// landed before an unusually slow producer finished writing would leave that
/// topic unenumerated with nothing behind it, and the second walk is paid only
/// by a machine that really did change.
///
/// It stays a TAIL, not a cadence: the count is fixed, it is armed only by a
/// real event, and a settled machine runs none of them.
pub const CONFIRM_WALKS: u32 = 2;

/// The walks one change is answered with: the walk itself and its
/// [`CONFIRM_WALKS`] confirmations.
///
/// Also what a watch that BREAKS hands the timed engine. A break can follow a
/// change the watch saw and never walked (it broke while absorbing the burst),
/// and it can be a change itself (the watched directory was replaced and could
/// not be re-opened), so the settle hold stays up until the timed engine has
/// given the full answer.
const WALKS_OWED_PER_CHANGE: u32 = 1 + CONFIRM_WALKS;

/// The closest together two walks are ever STARTED.
///
/// A rate limit, not a cadence: it can only make an event-driven scanner walk
/// LESS, and an idle one walks not at all. It bounds the cost of a pathologically
/// chatty directory at four walks a second, which is what the old unconditional
/// poll cost every recorder all the time.
pub const SCAN_RATE_FLOOR: Duration = DISCOVERY_RESCAN_INTERVAL;

// The confirmation walk must not be held back by the rate floor, or the gap it
// exists to close would be widened by the mechanism meant to bound cost.
const _: () = assert!(EVENT_CONFIRM_DELAY.as_millis() >= SCAN_RATE_FLOOR.as_millis());
// And a burst must be absorbed in well under the confirmation delay, or the two
// collapse into one another and a burst costs a walk per route.
const _: () = assert!(EVENT_COALESCE_WINDOW.as_millis() < EVENT_CONFIRM_DELAY.as_millis());

/// How many cadences of SILENCE mark the worker as stalled.
///
/// Generous on purpose, and the two errors are asymmetric: reporting late costs
/// a log line, while reporting early would cry wolf on every recorder whose
/// worker was simply descheduled for a moment. Eight cadences is 2 s at the
/// shipped interval, against a heartbeat stamped every
/// [`SCANNER_STOP_SLICE`] - i.e. ~20 missed stamps before anything is said.
pub const SCANNER_SILENCE_INTERVALS: u32 = 8;

/// One completed enumeration of the live topic set.
///
/// The `Err` arm is CARRIED rather than swallowed: the coverage manifest counts
/// enumeration failures and refuses to claim `enumerated` on a run whose every
/// scan failed, so a scanner that quietly dropped errors would restore the exact
/// false claim that flag exists to prevent.
#[derive(Debug)]
pub struct TopicScan {
    /// The live topic set, or the enumeration error rendered for the report.
    pub result: Result<Vec<String>, String>,
}

/// What is driving the enumeration for this recording.
///
/// Read back through [`DiscoveryScanner::wake_source`], because the difference
/// is not cosmetic: `Events` costs a settled machine nothing, `Poll` costs it a
/// directory walk four times a second, and `Inline` costs the drive loop
/// itself. A recording does not store it in its summary or coverage manifest;
/// it reports a fall from `Events` with one `warn!` naming the failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum WakeSource {
    /// Discovery is off - nothing enumerates.
    Off = 0,
    /// The shipping shape: a kernel watch on the iceoryx2 service directory.
    Events = 1,
    /// The watch could not be armed, or broke. A worker thread walks on
    /// [`DISCOVERY_RESCAN_INTERVAL`] instead.
    Poll = 2,
    /// The worker thread could not be spawned. The drive loop walks, at the
    /// same cadence, and pays the cost that moved the walk off it.
    Inline = 3,
}

impl WakeSource {
    fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::Events,
            2 => Self::Poll,
            3 => Self::Inline,
            _ => Self::Off,
        }
    }
}

/// PURE: has the worker been quiet long enough to be worth reporting?
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScannerHealth {
    /// A heartbeat arrived recently enough - nothing to say.
    Healthy,
    /// Nothing for [`SCANNER_SILENCE_INTERVALS`] cadences.
    Stalled,
}

/// PURE: classify the worker's silence against its own cadence.
///
/// A ZERO interval means there is no cadence to be late against, so it can never
/// be stalled — a caller that disabled the scanner must not be told its
/// non-existent thread died.
pub fn classify_scanner_silence(
    since_last_sign_of_life: Duration,
    interval: Duration,
) -> ScannerHealth {
    if interval.is_zero() {
        return ScannerHealth::Healthy;
    }
    if since_last_sign_of_life >= interval * SCANNER_SILENCE_INTERVALS {
        ScannerHealth::Stalled
    } else {
        ScannerHealth::Healthy
    }
}

/// What the caller should LOG about the worker's health this pass.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SilenceReport {
    /// Say nothing.
    Quiet,
    /// First observation of a stall — report it loudly.
    Loud,
    /// The worker is alive again after a reported stall - say so once.
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

/// PURE: what a worker that just finished a wait should do next.
///
/// Extracted so the loop's shape is oracle-testable without a kernel watch: the
/// property that matters is "an enumeration happens on a CHANGE, on a pending
/// confirmation, and on nothing else", and that is a decision, not a syscall.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScanStep {
    /// Walk the directory now, then arm the confirmation tail.
    WalkAndConfirm,
    /// Walk the directory now; this is one of the confirmations.
    WalkConfirming,
    /// Do nothing. The overwhelmingly common step on a settled machine.
    Idle,
    /// The watch is gone - degrade to the timed walk and say so.
    Degrade,
}

/// PURE: fold one wait outcome into the worker's next step.
///
/// `confirm_due` is when the pending confirmation walk comes due, if one is
/// armed; `now` is the worker's clock.
pub fn next_step(wake: &WatchWake, confirm_due: Option<Instant>, now: Instant) -> ScanStep {
    match wake {
        WatchWake::Broken { .. } => ScanStep::Degrade,
        WatchWake::Changed => ScanStep::WalkAndConfirm,
        WatchWake::Idle => match confirm_due {
            Some(due) if now >= due => ScanStep::WalkConfirming,
            _ => ScanStep::Idle,
        },
    }
}

/// What the worker still OWES the settle hold, once its baseline walk is in.
///
/// [`DiscoveryScanner::walk_pending`] is [`Owed::pending`] of this, so the
/// rule for when that flag may drop lives in one pure fold ([`owed_after`])
/// rather than in the stores scattered through the worker loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Owed {
    /// Nothing: no change the worker has seen is still unanswered. On the
    /// watch, a probe has found nothing queued since the last owed walk; on the
    /// timed engine, it has run every walk it was owed.
    Nothing,
    /// The last owed walk has run. The next wait is a zero-timeout PROBE: a
    /// change that landed during that walk is queued on the watch, and only an
    /// `Idle` answer here means nothing more is owed.
    Probe,
    /// A change's answer is in flight: its walk, or its confirmation tail.
    Answer,
    /// The watch broke; the timed engine owes this many walks (never zero).
    Timed(u32),
}

impl Owed {
    /// What the worker owes once its BASELINE walk has published. With a watch,
    /// a change that landed during the baseline is queued on it, so the answer
    /// waits on a probe; the timed engine has nothing to probe.
    fn after_baseline(watching: bool) -> Self {
        if watching {
            Self::Probe
        } else {
            Self::Nothing
        }
    }

    /// Whether the settle hold must treat the worker's answer as still in flight.
    fn pending(self) -> bool {
        self != Self::Nothing
    }

    /// How long the worker's next wait on the watch may block: not at all for a
    /// probe, one [`SCANNER_STOP_SLICE`] otherwise.
    fn wait_budget(self) -> Duration {
        if self == Self::Probe {
            Duration::ZERO
        } else {
            SCANNER_STOP_SLICE
        }
    }
}

/// One thing the worker did, as far as the settle hold is concerned.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Happened {
    /// A directory change woke the worker ([`ScanStep::WalkAndConfirm`]).
    Changed,
    /// A wait came back with nothing to do ([`ScanStep::Idle`]).
    Idle,
    /// A confirmation walk published, with `left` more still to run.
    Confirmed {
        /// Confirmations still to run after this one.
        left: u32,
    },
    /// The watch broke, on a wait or while a burst was being absorbed.
    Broke,
    /// The timed engine walked once.
    TimedWalk,
}

/// PURE: fold one thing the worker did into what it still owes.
///
/// The rows that matter: an idle wait clears ONLY a probe, never a tail that is
/// waiting for its next confirmation; the last confirmation hands over to a
/// probe rather than clearing; and a break of any kind owes the timed engine
/// the walks one change is answered with ([`WALKS_OWED_PER_CHANGE`]), because
/// the break may have cut a tail short, may itself be a change, and leaves
/// nothing watching from then on.
fn owed_after(owed: Owed, happened: Happened) -> Owed {
    match happened {
        Happened::Changed => Owed::Answer,
        Happened::Idle => match owed {
            Owed::Probe => Owed::Nothing,
            other => other,
        },
        Happened::Confirmed { left: 0 } => Owed::Probe,
        Happened::Confirmed { .. } => Owed::Answer,
        Happened::Broke => Owed::Timed(WALKS_OWED_PER_CHANGE),
        Happened::TimedWalk => match owed {
            Owed::Timed(n) if n > 1 => Owed::Timed(n - 1),
            _ => Owed::Nothing,
        },
    }
}

/// The threaded half: a worker publishing into a one-slot mailbox.
struct ThreadedScanner {
    /// The newest completed scan, or `None` when the loop has already taken it.
    latest: Arc<Mutex<Option<TopicScan>>>,
    /// Total enumerations COMPLETED by the worker (Principle #3 observable).
    scans_run: Arc<AtomicU64>,
    /// Enumerations the worker ran because it was WOKEN by a directory change,
    /// as opposed to a baseline, a confirmation or a timed fallback walk
    /// (Principle #3). A burst counts once, because a burst is one change.
    /// Zero on every engine but the watch, which is how a caller that owns the
    /// scanner tells a real watch from a timer. The recorder does not store it.
    wakes: Arc<AtomicU64>,
    /// Milliseconds since the worker started, stamped on every wait return. The
    /// liveness signal a silent-but-healthy event-driven worker still produces.
    heartbeat_ms: Arc<AtomicU64>,
    /// Whether the worker OWES a walk it has not finished: `true` until the
    /// baseline has published, then [`Owed::pending`] of the worker's state.
    ///
    /// Stored with `Release` after the scan it covers is published, read with
    /// `Acquire`, so a drive pass that reads `false` takes that scan on its
    /// next `next_scan`.
    ///
    /// The settle hold reads it, because "something changed and we have not
    /// looked yet" is the one state in which freezing the channel set is
    /// knowably premature.
    walk_pending: Arc<AtomicBool>,
    /// The LIVE wake source, so a mid-run degrade reads back through
    /// [`DiscoveryScanner::wake_source`].
    source: Arc<AtomicU8>,
    stop: Arc<AtomicBool>,
}

/// Everything the worker body needs, gathered so the spawn site stays readable.
struct WorkerChannels {
    latest: Arc<Mutex<Option<TopicScan>>>,
    scans_run: Arc<AtomicU64>,
    wakes: Arc<AtomicU64>,
    heartbeat_ms: Arc<AtomicU64>,
    walk_pending: Arc<AtomicBool>,
    source: Arc<AtomicU8>,
    stop: Arc<AtomicBool>,
}

impl ThreadedScanner {
    fn start(
        mgr: Arc<TransportManager>,
        interval: Duration,
        watch_root: Option<PathBuf>,
    ) -> Option<Self> {
        let latest: Arc<Mutex<Option<TopicScan>>> = Arc::new(Mutex::new(None));
        let scans_run = Arc::new(AtomicU64::new(0));
        let wakes = Arc::new(AtomicU64::new(0));
        let heartbeat_ms = Arc::new(AtomicU64::new(0));
        // TRUE from the start: the worker's baseline walk is owed, and the
        // settle hold must not read a worker that has not looked yet as quiet.
        let walk_pending = Arc::new(AtomicBool::new(true));
        let source = Arc::new(AtomicU8::new(WakeSource::Events as u8));
        let stop = Arc::new(AtomicBool::new(false));
        let channels = WorkerChannels {
            latest: Arc::clone(&latest),
            scans_run: Arc::clone(&scans_run),
            wakes: Arc::clone(&wakes),
            heartbeat_ms: Arc::clone(&heartbeat_ms),
            walk_pending: Arc::clone(&walk_pending),
            source: Arc::clone(&source),
            stop: Arc::clone(&stop),
        };
        match std::thread::Builder::new()
            .name("bagd-discovery-scan".to_string())
            .spawn(move || scan_loop(mgr, interval, watch_root, channels))
        {
            // The handle is DROPPED on purpose — see the module docs.
            Ok(_handle) => Some(Self {
                latest,
                scans_run,
                wakes,
                heartbeat_ms,
                walk_pending,
                source,
                stop,
            }),
            Err(e) => {
                tracing::warn!(
                    error = %e,
                    "bagd could not start the discovery-scan thread. Live-topic discovery falls \
                     back to enumerating the service directory INLINE on the recorder's drive \
                     loop: on a many-topic machine that costs the drain roughly a third of every \
                     cadence and fast topics can overflow their queues. Coverage is unaffected; \
                     throughput is not"
                );
                None
            }
        }
    }

    /// Take the newest completed scan, if the worker has published one since the
    /// last call.
    ///
    /// A poisoned mailbox is RECOVERED rather than read as "nothing new". The
    /// data behind it is one `Option` with no invariant a panic could break,
    /// and discarding scans on poison would stop discovery dead while
    /// `scans_run` kept climbing and the heartbeat kept stamping - every
    /// observable reading healthy while nothing was being delivered.
    fn take(&self) -> Option<TopicScan> {
        let mut slot = self
            .latest
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        slot.take()
    }
}

impl Drop for ThreadedScanner {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

/// The worker body: arm a watch if one can be armed, then wait for something to
/// happen.
fn scan_loop(
    mgr: Arc<TransportManager>,
    interval: Duration,
    watch_root: Option<PathBuf>,
    ch: WorkerChannels,
) {
    let origin = Instant::now();

    // ARM FIRST, then walk. The order is load-bearing, not tidiness: the
    // baseline walk is the single most expensive operation this worker performs
    // (117.5 ms at 86 live topics), and a producer that registers DURING it is
    // in neither the walk's result nor - if the watch were armed afterwards -
    // any event, because its file appeared before anything was watching. That
    // topic would then never be enumerated, with every observable reading
    // healthy. Arming first costs a config read and one syscall, and turns that
    // window into a queued event the first wait converts into a second walk.
    let identity = mgr.iox_shm_identity();
    let root = watch_root.unwrap_or_else(|| PathBuf::from(&identity.root_path));
    let mut watch = match ServiceDirWatch::arm(&root, &identity.service_dir) {
        Ok(w) => {
            let watching = w.watched_path();
            // At `info!`, and once per recording: the DEGRADED engines announce
            // themselves loudly, so without this the healthy case is the only
            // one an operator cannot confirm from the log. Which engine ran
            // decides whether a settled machine cost nothing or a directory
            // walk on every cadence, which is the whole point of the change.
            tracing::info!(
                watching = %watching.display(),
                "bagd live-topic discovery is EVENT-driven: it watches the iceoryx2 service \
                 directory and enumerates only when that directory changes"
            );
            Some(w)
        }
        Err(e) => {
            degrade_to_poll(&ch, &e.to_string(), WatchFailure::NotArmed, interval);
            None
        }
    };

    // The BASELINE enumeration. Unconditional and immediate: an event only ever
    // reports a CHANGE, so nothing that was already live when the watch was
    // armed would otherwise be seen. (The arm-time inline scan in
    // `Recorder::setup` covers the tap set; this covers the worker's own view.)
    let mut last_walk_started = Instant::now();
    publish(&ch, &mgr, &origin);

    // `walk_pending` has read true since the scanner was built, because the
    // baseline was owed. From here it is `owed.pending()`, and every change to
    // `owed` goes through `owed_after`.
    let mut owed = Owed::after_baseline(watch.is_some());
    report_owed(&ch, owed);

    let mut confirm_due: Option<Instant> = None;
    let mut confirms_left: u32 = 0;
    while !ch.stop.load(Ordering::Relaxed) {
        let Some(w) = watch.as_mut() else {
            // The degraded engine: the earlier timed walk, unchanged. It walks
            // on every cadence regardless, so once it has run the walks a broken
            // watch handed it, it owes nothing.
            sleep_unless_stopped(interval, &ch.stop, &ch.heartbeat_ms, &origin);
            if ch.stop.load(Ordering::Relaxed) {
                return;
            }
            publish(&ch, &mgr, &origin);
            owed = owed_after(owed, Happened::TimedWalk);
            report_owed(&ch, owed);
            continue;
        };

        let wake = w.wait(owed.wait_budget());
        stamp_heartbeat(&ch.heartbeat_ms, &origin);
        if ch.stop.load(Ordering::Relaxed) {
            return;
        }
        let step = next_step(&wake, confirm_due, Instant::now());
        let mut broke_while_coalescing = None;
        if step == ScanStep::WalkAndConfirm {
            ch.wakes.fetch_add(1, Ordering::Relaxed);
            // Something changed and we have not looked yet. Said BEFORE the
            // absorption and the rate floor, because that delay is exactly the
            // window in which the settle hold must not freeze the channel set.
            owed = owed_after(owed, Happened::Changed);
            report_owed(&ch, owed);
            // Absorb the rest of the burst. Seventy routes opening back to back
            // are one logical change and deserve one walk.
            broke_while_coalescing = coalesce(w, &ch.stop, &ch.heartbeat_ms, &origin);
            if ch.stop.load(Ordering::Relaxed) {
                return;
            }
        }
        // A watch that broke DURING the absorption degrades HERE, not after one
        // more walk. The re-target failure arm cannot reproduce itself on the
        // next wait - it leaves the watch bound to a directory it has already
        // decided is wrong - so a dropped break there is permanent blindness
        // rather than a report one pass late.
        if let Some(reason) = broke_while_coalescing {
            // Owed BEFORE the engine flips, so a reader that sees `Poll` also
            // sees the walks the timed engine now owes.
            owed = owed_after(owed, Happened::Broke);
            report_owed(&ch, owed);
            degrade_to_poll(&ch, &reason, WatchFailure::BrokeMidRun, interval);
            watch = None;
            confirm_due = None;
            confirms_left = 0;
            continue;
        }

        // Everything below is done with the watch borrow, which is what lets
        // the degrade arm replace it.
        match step {
            ScanStep::Idle => {
                owed = owed_after(owed, Happened::Idle);
                report_owed(&ch, owed);
            }
            ScanStep::Degrade => {
                owed = owed_after(owed, Happened::Broke);
                report_owed(&ch, owed);
                degrade_to_poll(
                    &ch,
                    &broken_reason(&wake),
                    WatchFailure::BrokeMidRun,
                    interval,
                );
                watch = None;
                confirm_due = None;
                confirms_left = 0;
            }
            ScanStep::WalkAndConfirm => {
                // Never start two walks closer together than the floor.
                hold_rate_floor(last_walk_started, &ch.stop, &ch.heartbeat_ms, &origin);
                if ch.stop.load(Ordering::Relaxed) {
                    return;
                }
                last_walk_started = Instant::now();
                publish(&ch, &mgr, &origin);
                // A burst arriving while a confirmation is pending supersedes
                // it, and re-arms the full tail behind the fresher walk.
                confirms_left = CONFIRM_WALKS;
                confirm_due = Some(Instant::now() + EVENT_CONFIRM_DELAY);
            }
            ScanStep::WalkConfirming => {
                last_walk_started = Instant::now();
                publish(&ch, &mgr, &origin);
                confirms_left = confirms_left.saturating_sub(1);
                confirm_due = (confirms_left > 0).then(|| Instant::now() + EVENT_CONFIRM_DELAY);
                // The tail is what covers a service that was not listable yet,
                // so the answer is not complete until the tail is spent, and
                // even then a change that landed during this walk is queued on
                // the watch: the last confirmation hands over to a probe.
                owed = owed_after(
                    owed,
                    Happened::Confirmed {
                        left: confirms_left,
                    },
                );
                report_owed(&ch, owed);
            }
        }
    }
}

/// Render a broken watch's reason. [`next_step`] answers `Degrade` for
/// [`WatchWake::Broken`] alone, so the other arms are unreachable and say so
/// rather than inventing a plausible-looking explanation.
fn broken_reason(wake: &WatchWake) -> String {
    match wake {
        WatchWake::Broken { reason } => reason.clone(),
        other => format!("unreachable: degraded on a {other:?} wake"),
    }
}

/// Enumerate and publish, so an observer that sees the count move can always
/// take the scan that moved it.
fn publish(ch: &WorkerChannels, mgr: &TransportManager, origin: &Instant) {
    let result = mgr.list_topics().map_err(|e| e.to_string());
    {
        // Recovered on poison for the reason `take` recovers: a scan discarded
        // here is invisible, because the counter below moves either way.
        let mut slot = ch
            .latest
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        *slot = Some(TopicScan { result });
    }
    ch.scans_run.fetch_add(1, Ordering::Relaxed);
    stamp_heartbeat(&ch.heartbeat_ms, origin);
}

/// Record that the worker is alive. Cheap enough to do on every wait return.
fn stamp_heartbeat(heartbeat_ms: &AtomicU64, origin: &Instant) {
    heartbeat_ms.store(origin.elapsed().as_millis() as u64, Ordering::Relaxed);
}

/// Publish whether the worker owes a walk. `Release`, so a reader that sees
/// `false` also sees the scan published before it.
fn report_owed(ch: &WorkerChannels, owed: Owed) {
    ch.walk_pending.store(owed.pending(), Ordering::Release);
}

/// Which way the watch failed. A FIELD rather than a phrase spliced into the
/// message: an operator greps by key, and a message that changes shape with its
/// data cannot be grepped for at all.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WatchFailure {
    /// The watch could never be established for this recording.
    NotArmed,
    /// The watch was working and stopped.
    BrokeMidRun,
}

impl WatchFailure {
    /// The stable token the log line carries.
    fn as_str(self) -> &'static str {
        match self {
            Self::NotArmed => "could_not_be_armed",
            Self::BrokeMidRun => "broke_mid_recording",
        }
    }
}

/// Report the fall back to a timed walk, and flip the observable engine.
fn degrade_to_poll(ch: &WorkerChannels, reason: &str, failure: WatchFailure, interval: Duration) {
    ch.source.store(WakeSource::Poll as u8, Ordering::Release);
    tracing::warn!(
        watch_failure = failure.as_str(),
        reason = %reason,
        cadence_ms = interval.as_millis() as u64,
        "bagd's watch on the iceoryx2 service directory FAILED - live-topic discovery \
         falls back to RE-ENUMERATING the directory on a timer. Coverage is unaffected and the \
         walk still runs off the recorder's drive loop, so no frames are at risk; the cost is \
         that a settled machine now pays a full service-directory walk on every cadence instead \
         of none at all"
    );
}

/// Keep absorbing events for [`EVENT_COALESCE_WINDOW`] so one burst is one walk.
///
/// Returns the reason the watch broke, if it broke. A break swallowed here
/// would leave the caller walking with a watch that has already stopped
/// working, and on the re-target-failure path it would never be reported at
/// all.
fn coalesce(
    w: &mut ServiceDirWatch,
    stop: &AtomicBool,
    heartbeat_ms: &AtomicU64,
    origin: &Instant,
) -> Option<String> {
    let deadline = Instant::now() + EVENT_COALESCE_WINDOW;
    loop {
        if stop.load(Ordering::Relaxed) {
            return None;
        }
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return None;
        }
        if let WatchWake::Broken { reason } = w.wait(remaining) {
            return Some(reason);
        }
        stamp_heartbeat(heartbeat_ms, origin);
    }
}

/// Wait out [`SCAN_RATE_FLOOR`] since the previous walk STARTED, if any of it is
/// left.
fn hold_rate_floor(
    last_walk_started: Instant,
    stop: &AtomicBool,
    heartbeat_ms: &AtomicU64,
    origin: &Instant,
) {
    let earliest = last_walk_started + SCAN_RATE_FLOOR;
    let remaining = earliest.saturating_duration_since(Instant::now());
    if remaining.is_zero() {
        return;
    }
    sleep_unless_stopped(remaining, stop, heartbeat_ms, origin);
}

/// Sleep `total`, in slices, returning early once `stop` is set.
fn sleep_unless_stopped(
    total: Duration,
    stop: &AtomicBool,
    heartbeat_ms: &AtomicU64,
    origin: &Instant,
) {
    let deadline = Instant::now() + total;
    loop {
        if stop.load(Ordering::Relaxed) {
            return;
        }
        stamp_heartbeat(heartbeat_ms, origin);
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
    /// The threaded shape, consumed in O(1) - event-driven, or degraded to a
    /// timed walk on the same thread.
    Threaded(ThreadedScanner),
    /// The earlier inline shape, reached only when the thread could not be spawned.
    Inline {
        /// When the last inline scan COMPLETED — re-armed after the scan, which
        /// is what `drive_loop` did before the worker existed.
        last: Option<Instant>,
    },
}

/// The drive loop's handle on the live-topic enumeration.
pub struct DiscoveryScanner {
    engine: ScanEngine,
    interval: Duration,
    /// When the worker last showed a sign of life, in its own milliseconds.
    /// Compared against [`Self::interval`] by [`classify_scanner_silence`].
    last_seen_alive: Instant,
    last_heartbeat_ms: u64,
    silence: ScanSilenceLatch,
}

impl DiscoveryScanner {
    /// Start watching `mgr`'s namespace, unless `discover_live` is off.
    pub fn start(mgr: &Arc<TransportManager>, discover_live: bool) -> Self {
        Self::start_with_interval(mgr, discover_live, DISCOVERY_RESCAN_INTERVAL)
    }

    /// [`Self::start`] with an explicit FALLBACK cadence - the seam tests drive.
    ///
    /// On the shipping event-driven engine the interval governs only the stall
    /// threshold; it becomes a walk cadence only if the watch cannot be armed or
    /// breaks. A test that sets it absurdly long and still sees enumerations has
    /// therefore proved the events are real.
    pub fn start_with_interval(
        mgr: &Arc<TransportManager>,
        discover_live: bool,
        interval: Duration,
    ) -> Self {
        Self::start_with_interval_and_root(mgr, discover_live, interval, None)
    }

    /// [`Self::start_with_interval`] with the watched ROOT overridden.
    ///
    /// The seam the fallback arm needs: pointing the watch at a directory that
    /// cannot be watched is the only way to exercise the degraded engine without
    /// an unsupported platform or a descriptor limit.
    pub fn start_with_interval_and_root(
        mgr: &Arc<TransportManager>,
        discover_live: bool,
        interval: Duration,
        watch_root: Option<PathBuf>,
    ) -> Self {
        let engine = if !discover_live {
            ScanEngine::Off
        } else {
            match ThreadedScanner::start(Arc::clone(mgr), interval, watch_root) {
                Some(t) => ScanEngine::Threaded(t),
                None => ScanEngine::Inline { last: None },
            }
        };
        Self {
            engine,
            interval,
            last_seen_alive: Instant::now(),
            last_heartbeat_ms: 0,
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
        self.report_health();
        scan
    }

    /// Map this pass's liveness verdict onto a log line.
    ///
    /// Reads the HEARTBEAT, not the scan count. That distinction is the whole
    /// difference an event-driven engine makes: a settled machine produces no
    /// scans at all and is perfectly healthy, so classifying scan silence would
    /// warn on every well-behaved recording. Only the THREADED engine can go
    /// silent independently of the caller - the inline fallback runs on the
    /// drive loop itself, so its silence would be the loop's own, and `Off` has
    /// nothing to be silent about.
    fn report_health(&mut self) {
        let ScanEngine::Threaded(t) = &self.engine else {
            return;
        };
        let beat = t.heartbeat_ms.load(Ordering::Relaxed);
        if beat != self.last_heartbeat_ms {
            self.last_heartbeat_ms = beat;
            self.last_seen_alive = Instant::now();
        }
        let health = classify_scanner_silence(self.last_seen_alive.elapsed(), self.interval);
        match self.silence.observe(health) {
            SilenceReport::Quiet => {}
            SilenceReport::Loud => tracing::warn!(
                silent_for_ms = self.last_seen_alive.elapsed().as_millis() as u64,
                cadence_ms = self.interval.as_millis() as u64,
                "bagd's discovery-scan thread has shown NO sign of life for many cadences - \
                 live-topic discovery is not running, so a producer appearing from here on will \
                 be neither recorded nor named in this bag's coverage manifest"
            ),
            SilenceReport::Recovered => tracing::info!(
                "bagd's discovery-scan thread is alive again - live-topic discovery has resumed"
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

    /// Enumerations the worker ran because a directory change WOKE it
    /// (Principle #3). A burst counts once.
    ///
    /// The observable that separates a real watch from a timer: it stays at `0`
    /// on every engine but [`WakeSource::Events`], and it moves on that one only
    /// when the machine's service set actually changed.
    pub fn wakes(&self) -> u64 {
        match &self.engine {
            ScanEngine::Threaded(t) => t.wakes.load(Ordering::Relaxed),
            _ => 0,
        }
    }

    /// Whether the worker OWES a walk it has not finished.
    ///
    /// True until the worker's baseline walk has published; from a directory
    /// event until the walk it caused and its confirmation tail have published
    /// and a probe of the watch finds nothing more queued; and, after a watch
    /// breaks, until the timed engine has run the walks a change is owed. The
    /// settle hold reads it, because an event inside the window is positive
    /// evidence that the live topic set is still moving, and freezing the
    /// channel set on the quiet boundary while that answer is in flight omits
    /// exactly the producer the event was about - which is the defect live
    /// discovery exists to prevent, arriving by a new route.
    ///
    /// Read with `Acquire`: a `false` here is ordered after the scan the worker
    /// published before clearing it, so the `next_scan` that follows takes it.
    ///
    /// `false` when discovery is off, and on the inline engine, which walks on
    /// the drive loop itself.
    pub fn walk_pending(&self) -> bool {
        match &self.engine {
            ScanEngine::Threaded(t) => t.walk_pending.load(Ordering::Acquire),
            _ => false,
        }
    }

    /// What is driving the enumeration right now, including a mid-run degrade
    /// from [`WakeSource::Events`] to [`WakeSource::Poll`].
    pub fn wake_source(&self) -> WakeSource {
        match &self.engine {
            ScanEngine::Off => WakeSource::Off,
            ScanEngine::Inline { .. } => WakeSource::Inline,
            ScanEngine::Threaded(t) => WakeSource::from_u8(t.source.load(Ordering::Acquire)),
        }
    }

    /// Whether a worker thread is actually running (Principle #3) — `false` for
    /// both a disabled scanner and one that degraded to the inline fallback.
    pub fn is_threaded(&self) -> bool {
        matches!(self.engine, ScanEngine::Threaded(_))
    }
}

// ===========================================================================
// Pure oracles for the decisions above. No transport, no threads, no kernel.
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
            (Healthy, Quiet), // a healthy worker says nothing...
            (Healthy, Quiet), // ...however often it is asked
            (Stalled, Loud),  // first stall: LOUD
            (Stalled, Quiet), // the regime is open — suppressed
            (Stalled, Quiet),
            (Healthy, Recovered), // it woke up: exactly one recovery line
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

    /// THE ANTI-POLL ORACLE: an idle wait NEVER enumerates.
    ///
    /// The whole fix is in this row of the table. A worker that walked on an
    /// idle tick would be a poll with a watch bolted on, and would look
    /// identical from the outside except in CPU - which is precisely the thing
    /// this change exists to remove. Driven as a hand-written vector rather than
    /// four separate assertions because the property is the MAPPING: a
    /// `next_step` that always returns `WalkAndConfirm` satisfies any
    /// single-row check about changes.
    #[test]
    fn an_idle_wait_never_enumerates_and_a_change_always_does() {
        let now = Instant::now();
        let broken = WatchWake::Broken {
            reason: "gone".into(),
        };
        // (wake, confirmation armed?, expected step)
        let oracle = [
            (&WatchWake::Idle, None, ScanStep::Idle),
            (&WatchWake::Changed, None, ScanStep::WalkAndConfirm),
            // A change during an armed confirmation supersedes it: the walk that
            // follows is fresher than the one the confirmation would have run.
            (
                &WatchWake::Changed,
                Some(now + Duration::from_millis(10)),
                ScanStep::WalkAndConfirm,
            ),
            // A confirmation that is armed but NOT yet due changes nothing.
            (
                &WatchWake::Idle,
                Some(now + Duration::from_millis(10)),
                ScanStep::Idle,
            ),
            // Due: exactly one confirming walk.
            (
                &WatchWake::Idle,
                Some(now - Duration::from_millis(1)),
                ScanStep::WalkConfirming,
            ),
            (&broken, None, ScanStep::Degrade),
            // A broken watch outranks a due confirmation - degrading is the only
            // step that can restore coverage, and walking first would report a
            // healthy-looking scan from a watch that will never fire again.
            (
                &broken,
                Some(now - Duration::from_millis(1)),
                ScanStep::Degrade,
            ),
        ];
        for (i, (wake, due, want)) in oracle.into_iter().enumerate() {
            assert_eq!(
                next_step(wake, due, now),
                want,
                "row {i}: {wake:?} with confirmation {due:?} must step {want:?}"
            );
        }
    }

    /// The confirmation is due at its delay, pinned on BOTH sides.
    ///
    /// A one-sided check is satisfied by a confirmation that fires immediately,
    /// which would make every event cost two back-to-back walks.
    #[test]
    fn the_confirmation_walk_is_due_at_its_delay_and_not_before() {
        let armed = Instant::now();
        let due = armed + EVENT_CONFIRM_DELAY;
        assert_eq!(
            next_step(&WatchWake::Idle, Some(due), due - Duration::from_nanos(1)),
            ScanStep::Idle,
            "a confirmation must not run early"
        );
        assert_eq!(
            next_step(&WatchWake::Idle, Some(due), due),
            ScanStep::WalkConfirming,
            "a confirmation must run at its delay"
        );
    }

    /// What the worker owes, against a HAND-WRITTEN transition table.
    ///
    /// The settle hold reads `walk_pending`, and every value it can read is a
    /// row here. A table rather than separate checks, because the defects this
    /// pins are each one wrong row: an idle wait clearing a confirmation tail
    /// that is waiting for its next walk, the last confirmation clearing
    /// instead of handing over to a probe, and a break that drops what was
    /// owed instead of handing it to the timed engine.
    #[test]
    fn what_the_worker_owes_follows_its_hand_written_transition_table() {
        use Happened::{Broke, Changed, Confirmed, Idle, TimedWalk};
        use Owed::{Answer, Nothing, Probe, Timed};
        // (owed before, what happened, owed after)
        let oracle = [
            // A change is answered whatever was owed before it.
            (Nothing, Changed, Answer),
            (Probe, Changed, Answer),
            (Answer, Changed, Answer),
            // An idle wait clears a PROBE and nothing else.
            (Probe, Idle, Nothing),
            (Answer, Idle, Answer),
            (Nothing, Idle, Nothing),
            // The tail: a confirmation with more to come keeps the answer in
            // flight; the last one hands over to a probe, never straight to
            // nothing.
            (Answer, Confirmed { left: 1 }, Answer),
            (Answer, Confirmed { left: 0 }, Probe),
            // A break owes the timed engine three walks, from every state.
            (Nothing, Broke, Timed(3)),
            (Probe, Broke, Timed(3)),
            (Answer, Broke, Timed(3)),
            // The timed engine pays them off one walk at a time.
            (Timed(3), TimedWalk, Timed(2)),
            (Timed(2), TimedWalk, Timed(1)),
            (Timed(1), TimedWalk, Nothing),
            (Nothing, TimedWalk, Nothing),
        ];
        for (i, (before, happened, want)) in oracle.into_iter().enumerate() {
            assert_eq!(
                owed_after(before, happened),
                want,
                "row {i}: {before:?} then {happened:?} must owe {want:?}"
            );
        }
        // The three walks are the walk and its confirmations.
        assert_eq!(WALKS_OWED_PER_CHANGE, 1 + CONFIRM_WALKS);
    }

    /// Only `Nothing` reads as quiet, only a probe skips the wait, and the
    /// baseline hands over to a probe exactly when there is a watch to probe.
    #[test]
    fn only_nothing_owed_reads_quiet_and_only_a_probe_skips_the_wait() {
        // (owed, pending, wait budget)
        let oracle = [
            (Owed::Nothing, false, SCANNER_STOP_SLICE),
            (Owed::Probe, true, Duration::ZERO),
            (Owed::Answer, true, SCANNER_STOP_SLICE),
            (Owed::Timed(1), true, SCANNER_STOP_SLICE),
        ];
        for (owed, pending, budget) in oracle {
            assert_eq!(owed.pending(), pending, "{owed:?} pending");
            assert_eq!(owed.wait_budget(), budget, "{owed:?} wait budget");
        }
        assert_eq!(Owed::after_baseline(true), Owed::Probe);
        assert_eq!(Owed::after_baseline(false), Owed::Nothing);
    }

    /// The engine byte round-trips, so a mid-run degrade cannot read back as
    /// `Off` and look like a recorder that was never asked to discover.
    #[test]
    fn every_wake_source_round_trips_through_its_observable_byte() {
        for want in [
            WakeSource::Off,
            WakeSource::Events,
            WakeSource::Poll,
            WakeSource::Inline,
        ] {
            assert_eq!(WakeSource::from_u8(want as u8), want);
        }
    }
}
