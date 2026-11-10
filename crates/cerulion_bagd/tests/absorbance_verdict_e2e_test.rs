// SPDX-License-Identifier: AGPL-3.0-only
//! The per-topic ABSORBANCE VERDICT, end to end over real iceoryx2.
//!
//! Isolated per-test SHM roots (the `common` harness), hand-built wire frames,
//! hand-written oracles. Parallel-safe: no `#[serial]`, no shared namespace, no
//! process-global env.
//!
//! # What makes this arm sound, and why both topics ride ONE recorder
//!
//! The verdict compares a tap's `depth / rate` against the drain-gap tail the
//! recorder MEASURED on this very run — two quantities that both move with load,
//! so an arm that drove them on separate recorders would be comparing two
//! different histograms and could pass on a coincidence.
//!
//! So both topics live on ONE recorder, in ONE run, behind ONE drain gate:
//! identical depth (same slice, same budget), identical measured tail, and the
//! ONLY difference between them is how fast they publish. A verdict that always
//! answered `Absorbs` fails on the fast topic; one that always answered `Short`
//! fails on the slow one; and one that read the rate off frames DRAINED rather
//! than off committed sequences fails on the fast topic too, because a queue
//! held shut for most of the run delivers almost nothing.
//!
//! # How the stall is MEASURED rather than declared
//!
//! `fault_inject_tap_drain_gate` holds `drain_taps` shut. The drive loop keeps
//! running and keeps recording its own pass gaps, and with nothing draining its
//! backlog-aware pacing ramps to `WAIT_TICK` — so the histogram fills with real
//! ~10 ms gaps, produced by the shipped pacing rule rather than by a number a
//! test wrote into a histogram. Each arm asserts the tail it got, so a run whose
//! stall did not materialise fails its own precondition instead of quietly
//! reporting a verdict about nothing.

#![cfg(unix)]

mod common;

use std::cell::RefCell;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_bagd::absorbance::{ABSORBANCE_MIN_RATE_SPAN, ABSORBANCE_VERDICT_QUANTILE};
use cerulion_bagd::{
    run_bagd, AbsorbanceVerdict, BagdConfig, BagdSummary, DrainGapHistogram, FlashbackSettings,
    TapSpec, TopicAbsorbance,
};
use cerulion_core::flashback::retention::RetentionCaps;
use cerulion_core::flashback::switch::TriggerPosture;
use cerulion_core::flashback::trigger::TriggerPolicy;

use common::{
    await_bagd_ready, build_frame, join_bagd, make_manager, multi_publisher_with_provisioning,
    publisher_with_provisioning, unique_out, unique_ready_file, unique_topic,
};

/// The slice both topics are provisioned with.
const SLICE: u32 = 64 * 1024;

/// The real iceoryx2 slot at [`SLICE`]: `align(48-byte header + slice, 8)`.
/// The header was 40 bytes before iceoryx2 0.10 added `payload_offset` to it;
/// `transport::tap_depth_tests` reads the size off the type and is where that
/// number is pinned.
const SLOT: u64 = SLICE as u64 + 48;

/// The per-topic byte budget, chosen so the depth rule lands on EXACTLY 2 —
/// `140_000 / 65_584` = 2. Small on purpose: the whole arm is about a queue
/// being too shallow for the stalls its recorder measured.
const BUDGET: u64 = 140_000;

/// The depth [`BUDGET`] buys at [`SLICE`], HAND-COMPUTED rather than re-derived
/// from the rule under test.
const DEPTH: u64 = 2;

/// The topics' provisioned service ceiling — well above [`DEPTH`], so the
/// budget rather than the ceiling is what sets the tap's depth.
const CEILING: usize = 256;

/// The borrow budget both topics are provisioned with.
const BORROW: usize = 4;

/// The FAST topic's publish period. 2.5 ms = 400 Hz, so `DEPTH / rate` is 5 ms —
/// comfortably under the ~10 ms pacing tick the stall produces.
const FAST_PERIOD: Duration = Duration::from_micros(2_500);

/// The SLOW topic's publish period. 200 ms = 5 Hz, so `DEPTH / rate` is 400 ms,
/// OVER the drain-gap ladder's top edge (250 ms). The control arm.
///
/// Over the top edge because the verdict compares the absorbance with the UPPER
/// edge of the bucket the tail lands in, and a host that coalesces timers moves
/// the tail into the top bucket without stalling anything: macOS background QoS
/// lets a timer fire up to 100 ms late (`kern.timer_coalesce_bg_ns_max`), so the
/// drive loop's 10 ms idle sleep can land past 100 ms, which the histogram files
/// in the (100, 250] ms bucket and serves as 250 ms. A tap holding 200 ms reads
/// `Short` against that. At 400 ms every in-ladder tail is covered, so the only
/// tail that can deny the control `Absorbs` is an OVERFLOW one, which the verdict
/// arms report as a stalled runner. They still assert against the row's OWN
/// reported tail rather than against a constant.
const SLOW_PERIOD: Duration = Duration::from_millis(200);

/// The band, `[2/3, 3/2]` of the COMMANDED rate, that an arm priced at that rate
/// needs its publisher's ACHIEVED rate to sit in for the attempt to be probative
/// ([`classify_attempt`]), and then the recorder's MEASURED rate to sit in
/// ([`assert_measured_rate_in_band`]). See [`in_rate_band`].
///
/// The measured rate is the tap's committed sequence delta over the DRAIN-clock
/// span between its first and last drained frames. With the grid-paced
/// [`publish_until`], that span is off from the committed frames' own span by at
/// most one drain gap plus one publish lateness, and the band holds whenever the
/// span is at least three times that error: an in-ladder gap (at most 250 ms)
/// plus a lateness under 400 ms against the two seconds and more every arm here
/// runs for. A publisher committing at half its period's rate falls outside it.
const RATE_BAND_LOW: (u64, u64) = (2, 3);
const RATE_BAND_HIGH: (u64, u64) = (3, 2);

/// How long the drain is held shut. Long enough that the paced idle passes
/// DOMINATE the histogram (a 0.999 quantile has to see them), and long enough
/// that the rate window spans more than `ABSORBANCE_MIN_RATE_SPAN`.
const STALL: Duration = Duration::from_millis(2_500);

/// The STUTTERING drain the loud arm uses: `STUTTER_CYCLES` x (open, closed).
///
/// The open window is kept SHORT on purpose. `quantile_us` takes
/// `rank = ceil(0.999 * total)`, which for any total under 1000 is the run
/// MAXIMUM — so the reading is the worst pass either way — but a long open
/// window adds thousands of microsecond-scale passes and would eventually push
/// the paced ones out of even that. Short opens keep the paced gaps a large
/// fraction of the histogram, which is what the arm's own tail assertion checks.
const STUTTER_OPEN: Duration = Duration::from_millis(250);
const STUTTER_CLOSED: Duration = Duration::from_millis(750);
const STUTTER_CYCLES: usize = 5;

/// The quiet arm's own log line, so its absence guard has something POSITIVE to
/// count over the same capture (an absence guard over an empty capture passes
/// vacuously).
const QUIET_PROBE: &str = "quiet-arm-capture-probe";

fn temp_dir(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "absorb-{tag}-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0)
    ));
    std::fs::create_dir_all(&dir).expect("temp dir");
    dir
}

/// Sweep everything a runtime arm leaves in the system temp dir.
///
/// Each arm creates THREE artifacts, and `common::cleanup(&dir)` alone cleans
/// none of them: that helper sweeps a BAG FILE and its
/// rotation siblings (`remove_file` plus a prefix scan of the parent) and `dir`
/// is a DIRECTORY — `remove_file` refuses it, the prefix scan then finds only
/// the same directory and refuses it again — so every run would leak the Flashback
/// directory, the ready file inside it, AND the output bag the helper is never
/// pointed at.
///
/// Not a new pattern: the two calls on the directory and the ready file are
/// exactly what the sibling `flashback_e2e_test.rs` arms do, and the bag goes
/// through `common::cleanup` because that is this crate's idiom for a bag (it
/// takes the rotation siblings with it). Best-effort throughout — a test that
/// already failed must not be re-reported as a cleanup failure.
fn cleanup_artifacts(out: &Path, dir: &Path, ready: &Path) {
    common::cleanup(out);
    std::fs::remove_dir_all(dir).ok();
    std::fs::remove_file(ready).ok();
}

fn settings(dir: &Path) -> FlashbackSettings {
    FlashbackSettings {
        window_span: Duration::from_secs(30),
        window_max_bytes: 64 * 1024 * 1024,
        anchor_max_bytes: 64 * 1024 * 1024,
        anchor_cap_basis: cerulion_core::flashback::CapBasis::Env,
        exclude_topics: cerulion_core::flashback::ExcludeTopics::default(),
        trace_max_bytes: 64 * 1024 * 1024,
        dir: dir.to_path_buf(),
        label: "absorb".into(),
        caps: RetentionCaps::default(),
        policy: TriggerPolicy::default(),
        posture: TriggerPosture::default(),
        window_only: true,
        tap_budget_bytes: BUDGET,
    }
}

/// The verdict row for `topic`, with the invariants every arm depends on
/// asserted once here rather than repeated per arm.
fn row(summary: &BagdSummary, topic: &str) -> TopicAbsorbance {
    let health = summary
        .record_health
        .topics
        .get(topic)
        .unwrap_or_else(|| panic!("no health row for {topic}"));
    assert_eq!(
        health.tap_buffer_depth,
        Some(DEPTH),
        "{topic}: the budget must have set the depth, or this arm is measuring \
         a queue it did not size"
    );
    let a = health
        .absorbance
        .unwrap_or_else(|| panic!("{topic}: the recorder stamps a verdict on every tap it opens"));
    assert_eq!(
        a.tap_buffer_depth, DEPTH,
        "{topic}: the verdict must price the depth the tap actually got"
    );
    a
}

/// Publish `body` on `pubr` on an ABSOLUTE grid of `period` until `stop` flips,
/// returning how many frames really went out (the arm's own denominator sanity
/// check).
///
/// Slot `k` falls due at `start + k * period`, and every publish attempt takes
/// one slot. The thread sleeps only until the next slot is due, so a wake that
/// lands past one or more due slots publishes them back to back and the commit
/// rate over any window of a few periods is the commanded one, however late
/// each wake lands. A relative `sleep(period)` after each publish adds every
/// wake's lateness to the period instead, and under timer coalescing (macOS
/// background QoS fires a timer up to 100 ms late) a 100 ms period then commits
/// near 5 Hz, which prices the tap at a rate the arm never commanded.
fn publish_until(
    pubr: &mut cerulion_core::transport::publisher::CerulionPublisher,
    period: Duration,
    stop: &AtomicBool,
    body: &[u8],
) -> u32 {
    let start = std::time::Instant::now();
    let mut slot = 0u32;
    let mut seq = 0u32;
    let mut sent = 0u32;
    while !stop.load(Ordering::Relaxed) {
        let due = start + period * slot;
        let now = std::time::Instant::now();
        if now < due {
            std::thread::sleep(due - now);
            continue;
        }
        slot += 1;
        // The wire `sequence` is what the rate window differences, so it
        // advances by exactly one per COMMIT — which is what makes the frames
        // this queue drops still count toward the rate.
        //
        // BOTH counters advance INSIDE the success arm, and the sequence one is
        // the load-bearing half: a production publisher's stream is GAP-FREE at
        // commit (a discarded loan burns no sequence), so a helper
        // that burned one on a FAILED publish would hand the rate window a gap
        // it reads as commits that never happened, inflating the rate this arm's
        // oracle is built on. `sent` is counted the same way for the same
        // reason: a failed publish commits nothing.
        if pubr
            .publish_raw(&build_frame(0xC133, seq, 1_000 + u64::from(seq), body))
            .is_ok()
        {
            sent += 1;
            seq = seq.wrapping_add(1);
        }
    }
    sent
}

/// A rate in millihertz: `frames` committed over `over`, saturating at
/// `u64::MAX` for a span too short to hold a plausible rate.
fn millihertz(frames: u128, over: Duration) -> u64 {
    u64::try_from(frames * 1_000_000_000_000u128 / over.as_nanos().max(1)).unwrap_or(u64::MAX)
}

/// Whether `mhz` sits between [`RATE_BAND_LOW`] and [`RATE_BAND_HIGH`] of
/// `commanded_mhz`, both ends included.
fn in_rate_band(mhz: u64, commanded_mhz: u64) -> bool {
    let (lo_n, lo_d) = RATE_BAND_LOW;
    let (hi_n, hi_d) = RATE_BAND_HIGH;
    mhz.saturating_mul(lo_d) >= commanded_mhz.saturating_mul(lo_n)
        && mhz.saturating_mul(hi_d) <= commanded_mhz.saturating_mul(hi_n)
}

/// PROBATIVE CHECK for an arm priced at its COMMANDED rate: the recorder's
/// measured rate for `row` sits in the band of `1 / period` ([`in_rate_band`]).
///
/// An arm calls it only after [`classify_attempt`] found the attempt probative,
/// which puts the publisher's ACHIEVED rate, `sent` frames over
/// `published_for`, inside that band already. A measured rate outside it, or
/// none at all, is then the rate window disagreeing with what was published,
/// which is the code under test, and it panics.
fn assert_measured_rate_in_band(
    what: &str,
    row: &TopicAbsorbance,
    period: Duration,
    sent: u32,
    published_for: Duration,
) {
    let commanded_mhz = millihertz(1, period);
    if row.rate_mhz.is_some_and(|m| in_rate_band(m, commanded_mhz)) {
        return;
    }
    let (lo_n, lo_d) = RATE_BAND_LOW;
    let (hi_n, hi_d) = RATE_BAND_HIGH;
    let measured = row
        .rate_mhz
        .map_or_else(|| "NO rate".to_string(), |m| format!("{m} mHz"));
    panic!(
        "PRECONDITION: {what} is priced at its commanded {commanded_mhz} mHz, where depth \
         {DEPTH} holds {} us, inside a band of [{lo_n}/{lo_d}, {hi_n}/{hi_d}] of it. The \
         publisher committed {sent} frames in {published_for:?} (~{} mHz), inside the band, \
         and the recorder measured {measured}, so the rate window disagrees with what was \
         published: {row:?}",
        DEPTH * 1_000_000_000 / commanded_mhz.max(1),
        millihertz(u128::from(sent), published_for)
    );
}

/// How many attempts [`rerun_non_probative`] gives an arm's scenario.
const ATTEMPTS: u32 = 3;

/// What one attempt of an arm's scenario showed about itself.
#[derive(Debug)]
enum Attempt {
    /// The run could check the arm, and every assertion on it ran and held.
    Probative,
    /// [`classify_attempt`] found, from this attempt's recorded evidence, that
    /// the run could not check the arm. Carries the reasons and the evidence.
    NonProbative(String),
}

/// One priced tap's PUBLISHER as the fixture measured it: the period the arm
/// commanded, the frames the fixture committed (counted in [`publish_until`]'s
/// success arm) and the wall time it published for (the fixture's own clock).
#[derive(Debug, Clone, Copy)]
struct TapRun {
    what: &'static str,
    period: Duration,
    sent: u32,
    published_for: Duration,
}

/// When the recorder evaluated the answers an arm asserts on, which fixes the
/// histogram those answers were read against. A property of the arm, written
/// in its judge, never of the run.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TailRead {
    /// The arm checks the rows evaluated at finalize against the run's FINAL
    /// histogram, plus what its judge shows an overflow gap mid-run cannot
    /// change: the headline's live frame, polled for until the recorder's
    /// evaluation converges to it, and the quiet arm's silence, which only a
    /// `Short` evaluation breaks.
    AtFinalize,
    /// The arm also checks a line the recorder minted MID-RUN, at the first
    /// evaluation that found its tap short, against the histogram as it stood
    /// then. Counts only grow, so the final histogram holds every gap that
    /// evaluation saw, but not WHEN each gap arrived.
    MidRun,
}

/// Everything [`classify_attempt`] reads: the drain-gap histogram's bucket
/// edges and counts as the run's health document carries them, each priced
/// tap's [`TapRun`], and the arm's [`TailRead`].
///
/// Nothing the recorder DERIVED is here: no verdict, rate estimate, measured
/// tail, live status frame or log field. The type carries none, so no answer
/// the code under test gives can turn an attempt non-probative.
#[derive(Debug, Clone)]
struct AttemptEvidence {
    edges_us: Vec<u64>,
    counts: Vec<u64>,
    taps: Vec<TapRun>,
    read: TailRead,
}

impl AttemptEvidence {
    fn new(gaps: &DrainGapHistogram, read: TailRead, taps: &[TapRun]) -> Self {
        Self {
            edges_us: gaps.edges_us.clone(),
            counts: gaps.counts.clone(),
            taps: taps.to_vec(),
            read,
        }
    }
}

/// Where a histogram's 0.999 reading lands, walked over the bucket counts by
/// this file rather than by `DrainGapHistogram::quantile_us`, with the rank rule
/// that method documents: 1-based, `ceil(q * total)`, at least 1.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct TailWalk {
    /// Gaps recorded.
    total: u64,
    /// The 1-based rank of the 0.999 reading among them.
    rank: u64,
    /// Gaps in the unbounded overflow bucket, past the ladder's top edge.
    overflow: u64,
}

impl TailWalk {
    /// Whether the reading lands in the overflow bucket: the ladder holds fewer
    /// gaps than the rank needs.
    fn overflowed(&self) -> bool {
        self.total - self.overflow < self.rank
    }
}

/// The [`TailWalk`] of a histogram's `counts` over `edges_us`.
///
/// `None` when nothing was recorded, and when the document is malformed (its
/// counts are not one per edge plus the overflow bucket): such a histogram has
/// no overflow bucket to read, so it never makes an attempt non-probative and
/// the arms' own assertions judge it.
fn walk_tail(edges_us: &[u64], counts: &[u64]) -> Option<TailWalk> {
    if counts.len() != edges_us.len() + 1 {
        return None;
    }
    let total: u64 = counts.iter().sum();
    if total == 0 {
        return None;
    }
    let rank = (ABSORBANCE_VERDICT_QUANTILE * total as f64).ceil().max(1.0) as u64;
    Some(TailWalk {
        total,
        rank,
        overflow: counts[edges_us.len()],
    })
}

/// Why [`classify_attempt`] found an attempt unable to check its arm.
#[derive(Debug, Clone, PartialEq, Eq)]
enum NonProbativeReason {
    /// The 0.999 drain-gap reading lands in the unbounded overflow bucket. A
    /// drive-loop gap past the ladder's top edge holds the rank, and against
    /// that tail the verdict contract rules out `Absorbs` (an absorbance over
    /// the edge is `Unrankable`, one at or under it `Short`) and leaves every
    /// row without a measured tail or a required depth.
    OverflowTail(TailWalk),
    /// The final 0.999 reading sits inside the ladder, yet a gap past the top
    /// edge was recorded, and the arm checks a line minted mid-run
    /// ([`TailRead::MidRun`]). An evaluation made while that gap held the rank
    /// (as it does whenever fewer than 1000 gaps are recorded, where the rank is
    /// the run maximum) read an overflow tail, and the contract leaves a line
    /// minted then without a measured tail or a required depth. The histogram
    /// does not say whether the line was minted before the gap or after it.
    OverflowGapMidRun(TailWalk),
    /// A publisher committed outside the band of the rate its arm prices it
    /// at, so the run did not set up the scenario the arm checks.
    PublisherOffBand {
        what: &'static str,
        commanded_mhz: u64,
        achieved_mhz: u64,
    },
}

impl NonProbativeReason {
    fn describe(&self) -> String {
        match self {
            Self::OverflowTail(walk) => format!(
                "the 0.999 drain-gap reading (rank {} of {} gaps) lands in the histogram's \
                 unbounded overflow bucket, which holds {} gap(s) past the ladder's top edge, \
                 so no tap can rank `Absorbs` or carry a measured tail against it",
                walk.rank, walk.total, walk.overflow
            ),
            Self::OverflowGapMidRun(walk) => format!(
                "the final 0.999 drain-gap reading (rank {} of {} gaps) is inside the ladder, \
                 but {} gap(s) past its top edge were recorded and this arm checks a line \
                 minted mid-run, which an evaluation made while such a gap held the rank \
                 leaves without a measured tail or a required depth",
                walk.rank, walk.total, walk.overflow
            ),
            Self::PublisherOffBand {
                what,
                commanded_mhz,
                achieved_mhz,
            } => {
                let (lo_n, lo_d) = RATE_BAND_LOW;
                let (hi_n, hi_d) = RATE_BAND_HIGH;
                let way = if achieved_mhz.saturating_mul(lo_d) < commanded_mhz.saturating_mul(lo_n)
                {
                    "fell behind"
                } else {
                    "ran ahead of"
                };
                format!(
                    "PRECONDITION: {what} committed at ~{achieved_mhz} mHz, outside the band \
                     [{lo_n}/{lo_d}, {hi_n}/{hi_d}] of its commanded {commanded_mhz} mHz. The \
                     publisher {way} its grid, so this run did not set up the scenario the arm \
                     checks"
                )
            }
        }
    }
}

/// An attempt's standing, as [`classify_attempt`] decides it.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Classification {
    /// The run can check its arm: every assertion runs and a failure is final.
    Probative,
    /// The run cannot check its arm, for every reason listed.
    NonProbative(Vec<NonProbativeReason>),
}

/// Decide from an attempt's RECORDED evidence alone whether it can check its
/// arm.
///
/// Non-probative exactly when the 0.999 drain-gap reading, walked here over the
/// recorded counts ([`walk_tail`]), lands in the overflow bucket; when an arm
/// that checks a mid-run line ([`TailRead::MidRun`]) recorded any gap in the
/// overflow bucket at all; or when a priced tap's achieved rate (`sent` over
/// `published_for`) sits outside the band of its commanded rate
/// ([`in_rate_band`]). A pure function of [`AttemptEvidence`], which holds
/// nothing the recorder decided, so a verdict a regression got wrong reaches
/// the arm's assertions and fails there.
fn classify_attempt(evidence: &AttemptEvidence) -> Classification {
    let mut reasons = Vec::new();
    if let Some(walk) = walk_tail(&evidence.edges_us, &evidence.counts) {
        if walk.overflowed() {
            reasons.push(NonProbativeReason::OverflowTail(walk));
        } else if walk.overflow > 0 && evidence.read == TailRead::MidRun {
            reasons.push(NonProbativeReason::OverflowGapMidRun(walk));
        }
    }
    for tap in &evidence.taps {
        let commanded_mhz = millihertz(1, tap.period);
        let achieved_mhz = millihertz(u128::from(tap.sent), tap.published_for);
        if !in_rate_band(achieved_mhz, commanded_mhz) {
            reasons.push(NonProbativeReason::PublisherOffBand {
                what: tap.what,
                commanded_mhz,
                achieved_mhz,
            });
        }
    }
    if reasons.is_empty() {
        Classification::Probative
    } else {
        Classification::NonProbative(reasons)
    }
}

/// The escape every arm takes AHEAD of its assertions: the attempt, marked
/// non-probative with its reasons and the evidence line `shown`, exactly when
/// [`classify_attempt`] finds it so; `None` for a probative attempt.
fn non_probative(evidence: &AttemptEvidence, shown: &str) -> Option<Attempt> {
    match classify_attempt(evidence) {
        Classification::Probative => None,
        Classification::NonProbative(reasons) => Some(Attempt::NonProbative(format!(
            "{}; {shown}",
            reasons
                .iter()
                .map(NonProbativeReason::describe)
                .collect::<Vec<_>>()
                .join("; ")
        ))),
    }
}

/// Run an arm's scenario, and run it again ONLY after an attempt proves itself
/// non-probative, up to [`ATTEMPTS`] attempts in all.
///
/// `attempt(n)` builds every piece of state it uses (transport manager, topics,
/// recorder, Flashback directory, bag) and removes it before it returns, so
/// nothing one attempt did reaches the next. It runs inside an
/// `absorbance_attempt{attempt=n}` span entered on this thread, the thread the
/// log-capturing arms run `run_bagd` on, so [`attempt_lines`] keeps one
/// attempt's captured lines apart from another's.
///
/// Each attempt hands its recorded evidence to [`classify_attempt`] ahead of
/// every assertion, and [`Attempt::NonProbative`] comes back from that call and
/// from nothing else. A probative attempt then runs the arm's assertions, and a
/// wrong answer panics there: that failure is final and nothing re-runs.
///
/// Each non-probative attempt's line goes to the process's stderr outside the
/// test harness's capture, so a run that passed on a later attempt still shows
/// what the earlier ones measured. When every attempt is non-probative the arm
/// fails, naming each one.
fn rerun_non_probative(what: &str, attempt: impl FnMut(u32) -> Attempt) {
    rerun_into(what, &mut std::io::stderr(), attempt);
}

/// [`rerun_non_probative`], writing each non-probative attempt's line to `out`.
fn rerun_into(what: &str, out: &mut dyn std::io::Write, mut attempt: impl FnMut(u32) -> Attempt) {
    let mut seen = Vec::new();
    for n in 1..=ATTEMPTS {
        let span = tracing::info_span!("absorbance_attempt", attempt = n);
        match span.in_scope(|| attempt(n)) {
            Attempt::Probative => return,
            Attempt::NonProbative(evidence) => {
                let line =
                    format!("{what}: attempt {n} of {ATTEMPTS} was NON-PROBATIVE: {evidence}");
                writeln!(out, "{line}").ok();
                seen.push(line);
            }
        }
    }
    panic!(
        "NON-PROBATIVE: all {ATTEMPTS} attempts of {what} ran on a host that invalidated \
         them, so this runner cannot check the arm. That is a stalled runner, not a verdict \
         regression:\n{}",
        seen.join("\n")
    );
}

/// The captured `lines` that attempt `n` logged: those inside its
/// `absorbance_attempt` span (see [`rerun_non_probative`]).
fn attempt_lines(lines: &[String], n: u32) -> Vec<String> {
    let span = format!(":absorbance_attempt{{attempt={n}}}:");
    lines
        .iter()
        .filter(|l| l.contains(&span))
        .cloned()
        .collect()
}

/// One attempt's EVIDENCE line: the drain-gap histogram and where this file's
/// walk puts its 0.999 reading, then each priced tap's commanded and achieved
/// rates beside what the recorder measured and answered for it (`rows`, in the
/// order of `evidence.taps`). The recorder's numbers are printed for the reader
/// and read by nothing.
fn attempt_evidence(evidence: &AttemptEvidence, rows: &[&TopicAbsorbance]) -> String {
    let walk = walk_tail(&evidence.edges_us, &evidence.counts).map_or_else(
        || "no 0.999 reading".to_string(),
        |w| {
            format!(
                "the 0.999 reading (rank {} of {}) lands in the {}",
                w.rank,
                w.total,
                if w.overflowed() {
                    "overflow bucket"
                } else {
                    "ladder"
                }
            )
        },
    );
    let mut out = format!(
        "gaps per bucket {:?} over edges up to {:?} us, {} of them over the top edge, {walk}",
        evidence.counts,
        evidence.edges_us.last(),
        evidence
            .counts
            .get(evidence.edges_us.len())
            .copied()
            .unwrap_or(0)
    );
    for (tap, row) in evidence.taps.iter().zip(rows) {
        out.push_str(&format!(
            "; {}: commanded {} mHz, measured {:?} mHz, achieved ~{} mHz ({} frames in {:?}), \
             verdict {:?}",
            tap.what,
            millihertz(1, tap.period),
            row.rate_mhz,
            millihertz(u128::from(tap.sent), tap.published_for),
            tap.sent,
            tap.published_for,
            row.verdict
        ));
    }
    out
}

/// THE HEADLINE: one recorder, one measured stall, two rates — and two
/// different verdicts.
///
/// The SLOW topic is not decoration. It is the arm that fails if the verdict
/// ever collapses to "everything is short", and it shares the recorder, the
/// depth and the histogram with the fast one, so the comparison isolates the
/// rate exactly.
#[test]
fn a_tap_too_shallow_for_the_measured_stall_is_short_while_its_slower_sibling_absorbs() {
    rerun_non_probative("the headline", headline_attempt);
}

/// One attempt of the headline's scenario, on its own manager, recorder, topics
/// and bag (see [`rerun_non_probative`]).
fn headline_attempt(attempt: u32) -> Attempt {
    let mgr = make_manager(16);
    let dir = temp_dir("headline");
    let fast_topic = unique_topic("absorb_fast");
    let slow_topic = unique_topic("absorb_slow");
    let out = unique_out("absorb");
    let ready = unique_ready_file("absorb");

    // The taps are OPEN-ONLY, so the publishers must exist before the recorder
    // arms — and for the BUDGETED depth they must exist for a second reason: the
    // slice is read off the live publisher's dynamic config.
    let mut fast = publisher_with_provisioning(&mgr, &fast_topic, BORROW, CEILING, SLICE);
    let mut slow = publisher_with_provisioning(&mgr, &slow_topic, BORROW, CEILING, SLICE);

    let gate = Arc::new(AtomicBool::new(true));
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = BagdConfig::new(
        out.clone(),
        vec![TapSpec::attach(&fast_topic), TapSpec::attach(&slow_topic)],
    );
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = Some(Duration::from_millis(50));
    cfg.ready_file = Some(ready.clone());
    cfg.flashback = Some(settings(&dir));
    cfg.fault_inject_tap_drain_gate = Some(gate.clone());

    let handle = {
        let m = Arc::clone(&mgr);
        let c = cfg.clone();
        let s = Arc::clone(&shutdown);
        std::thread::spawn(move || run_bagd(m, c, s))
    };
    await_bagd_ready(&ready, "the absorbance headline");
    // THE LIVE SURFACE. The verdict's whole justification is that the boundary is stated
    // DURING the run, and the terminal summary and the bag both arrive after it
    // — so `/bagd/status` is the one surface that delivers the stated property,
    // and without an assertion here it can be deleted with every other arm green.
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");
    // The window the publishers really run for — the DENOMINATOR of the rate
    // oracle below, measured rather than assumed from the sleeps.
    let started = std::time::Instant::now();

    // Both publishers run for the whole window; only the drain is interrupted.
    let stop = Arc::new(AtomicBool::new(false));
    let body = vec![0xAB; 256];
    let fast_stop = Arc::clone(&stop);
    let slow_stop = Arc::clone(&stop);
    let fast_body = body.clone();
    let fast_thread =
        std::thread::spawn(move || publish_until(&mut fast, FAST_PERIOD, &fast_stop, &fast_body));
    let slow_thread =
        std::thread::spawn(move || publish_until(&mut slow, SLOW_PERIOD, &slow_stop, &body));

    // (1) A short OPEN window so both taps anchor their rate windows on a real
    //     drained frame rather than on the reopen.
    std::thread::sleep(Duration::from_millis(300));
    // (2) THE STALL. The loop keeps passing and keeps recording its gaps; with
    //     nothing draining, its pacing ramps to WAIT_TICK.
    gate.store(false, Ordering::Relaxed);
    std::thread::sleep(STALL);
    // (3) Reopen, so the last drain closes each rate window past the stall.
    gate.store(true, Ordering::Relaxed);
    std::thread::sleep(Duration::from_millis(300));

    // Read the LIVE frames before shutting down — the verdicts must be on the
    // wire while the run is happening, which is the whole claim. The wait is
    // for the CONVERGED pair, not the first both-verdict frame (see the
    // helper's doc): the publishers keep publishing and the drain
    // stays open until it returns, so the run is still live for however long
    // convergence takes on this runner.
    let live = await_status_verdicts(&mut status, &fast_topic, &slow_topic);

    stop.store(true, Ordering::Relaxed);
    let wall = started.elapsed();
    let fast_sent = fast_thread.join().expect("fast publisher");
    let slow_sent = slow_thread.join().expect("slow publisher");
    shutdown.store(true, Ordering::Relaxed);
    let summary = join_bagd(handle, &shutdown, "the absorbance headline").expect("recorder");

    let run = HeadlineRun {
        fast_row: row(&summary, &fast_topic),
        slow_row: row(&summary, &slow_topic),
        gaps: summary.record_health.drain_gaps.clone(),
        fast_sent,
        slow_sent,
        wall,
        live,
        unpriced_taps: summary.unpriced_taps,
        shm_pinned_bytes: summary.shm_pinned_bytes,
    };
    let outcome = judge_headline(attempt, &run);
    cleanup_artifacts(&out, &dir, &ready);
    outcome
}

/// What one headline attempt recorded, gathered before any of it is judged, so
/// [`judge_headline`] runs on a seeded run exactly as it does on a live one.
#[derive(Debug, Clone)]
struct HeadlineRun {
    /// The recorder-wide drain-gap histogram, off the run's health document.
    gaps: DrainGapHistogram,
    fast_row: TopicAbsorbance,
    slow_row: TopicAbsorbance,
    /// Frames each publisher committed, and the wall time both published for.
    fast_sent: u32,
    slow_sent: u32,
    wall: Duration,
    /// What the live wait returned (see [`await_status_verdicts`]).
    live: Result<LiveFrame, Option<LiveFrame>>,
    unpriced_taps: u64,
    shm_pinned_bytes: u64,
}

/// Judge one headline attempt: [`classify_attempt`] on its recorded evidence,
/// then, on a probative attempt, every assertion the arm makes.
fn judge_headline(attempt: u32, run: &HeadlineRun) -> Attempt {
    let (fast_row, slow_row, gaps) = (&run.fast_row, &run.slow_row, &run.gaps);
    let (fast_sent, slow_sent, wall) = (run.fast_sent, run.slow_sent, run.wall);
    // AT FINALIZE: the rows, and the live frame the wait polls for while the run
    // goes on, which the recorder's evaluation reaches once its reading sits
    // inside the ladder.
    let evidence = AttemptEvidence::new(
        gaps,
        TailRead::AtFinalize,
        &[
            TapRun {
                what: "the fast tap",
                period: FAST_PERIOD,
                sent: fast_sent,
                published_for: wall,
            },
            TapRun {
                what: "the slow control",
                period: SLOW_PERIOD,
                sent: slow_sent,
                published_for: wall,
            },
        ],
    );
    let shown = attempt_evidence(&evidence, &[fast_row, slow_row]);
    eprintln!("the headline, attempt {attempt}: {shown}");
    // NON-PROBATIVE, decided by `classify_attempt` from this attempt's recorded
    // histogram and both publishers' own counts, ahead of every assertion below.
    // Neither row's verdict nor the live wait's outcome enters that decision, so
    // on a probative attempt an unrankable control, a fast row without a tail or
    // a live feed that never converged fails below, on this attempt.
    if let Some(non_probative) = non_probative(&evidence, &shown) {
        return non_probative;
    }
    // PRECONDITION: the control is priced at its commanded 5 Hz, which is what
    // puts its absorbance over the ladder's top edge (see `SLOW_PERIOD`), and its
    // publisher committed inside the band, so the measured rate must sit there too.
    assert_measured_rate_in_band("the slow control", slow_row, SLOW_PERIOD, slow_sent, wall);
    // THE LIVE WAIT. On a probative attempt a wait that timed out is the live
    // surface failing to state the boundary, whatever its last frame carried.
    let live = run.live.clone().unwrap_or_else(live_never_converged);

    // PRECONDITION, asserted rather than assumed: the two publishers really did
    // run at different rates over the same window.
    assert!(
        fast_sent > slow_sent * 4,
        "the fixture must really separate the two rates: fast={fast_sent} slow={slow_sent}"
    );

    // PRECONDITION: the stall really was measured. Both rows read the SAME
    // recorder-wide histogram, so one assertion covers both.
    //
    // And the STATISTIC is established rather than assumed. `quantile_us` takes
    // `rank = ceil(q * total)`, and for q = 0.999 that is `total` itself for
    // every total under 1000 — so on a short run the "tail" IS the run maximum,
    // and one preempted pass moves it. This fixture's run is ~3 s, i.e. a few
    // hundred passes, so it is squarely in that regime: the arms below are up
    // against the worst pass of the run, and they say so.
    let total_gaps = gaps.total();
    assert!(
        total_gaps > 0,
        "the recorder must have measured its own cadence, or there is no tail          to be short of"
    );
    // `classify_attempt` walked this run's 0.999 reading inside the ladder above,
    // so it is an in-ladder edge, and a fast row that does not carry it is the
    // code under test.
    let Some(tail) = fast_row.measured_tail_us else {
        panic!(
            "this run's 0.999 drain-gap reading is inside the ladder, so the fast row must \
             carry it as its measured tail: {total_gaps} gaps measured, fast={fast_row:?}"
        );
    };
    assert_eq!(
        slow_row.measured_tail_us,
        Some(tail),
        "both topics ride one recorder's histogram — a difference here means the \
         arm is not isolating the rate"
    );
    assert!(
        tail >= 5_000,
        "the injected stall must have paced the loop into the millisecond rungs; \
         got a {tail} us tail over {total_gaps} gaps, so this run measured no \
         stall to be short of"
    );
    // The SLOW control clears every in-ladder tail with room: the rate band caps
    // its measured rate at 3/2 of 5 Hz, where depth 2 holds about 267 ms, over
    // the ladder's 250 ms top edge, and an overflow tail re-runs the attempt
    // above. A tail at or past its absorbance is one this ladder cannot serve.
    let slow_abs_pre = slow_row
        .absorbance_us
        .expect("the slow row has a rate and a depth");
    assert!(
        slow_abs_pre > tail,
        "the control tap's in-band absorbance ({slow_abs_pre} us) must clear this run's \
         in-ladder tail ({tail} us): {slow_row:?}"
    );

    // THE VERDICTS.
    assert_eq!(
        fast_row.verdict,
        AbsorbanceVerdict::Short,
        "a depth-{DEPTH} queue at ~{} Hz holds {:?} us, against a measured {tail} us \
         stall: {fast_row:?}",
        1_000_000 / FAST_PERIOD.as_micros(),
        fast_row.absorbance_us
    );
    assert_eq!(
        slow_row.verdict,
        AbsorbanceVerdict::Absorbs,
        "the SAME depth at ~{} Hz holds far longer than the same stall — a verdict \
         that cannot tell them apart is not a verdict: {slow_row:?}",
        1_000_000 / SLOW_PERIOD.as_micros()
    );

    // THE ARITHMETIC, checked against the row's own reported numbers so a
    // renderer that printed one thing and decided on another cannot pass.
    let fast_abs = fast_row
        .absorbance_us
        .expect("a short row has an absorbance");
    assert!(
        fast_abs < tail,
        "SHORT must mean the absorbance is under the tail: {fast_abs} vs {tail}"
    );
    assert_eq!(
        fast_row.shortfall_at_least_us,
        Some(tail - fast_abs),
        "the shortfall is the difference, not a re-derivation"
    );
    let required = fast_row
        .required_depth
        .expect("an in-ladder tail yields a required depth");
    assert!(
        required > DEPTH,
        "the DEPTH formulation must agree with the TIME one: needs {required}, has {DEPTH}"
    );
    let slow_abs = slow_row
        .absorbance_us
        .expect("an absorbing row has one too");
    assert!(
        slow_abs >= tail,
        "ABSORBS must mean the absorbance covers the tail: {slow_abs} vs {tail}"
    );
    assert!(
        slow_row.required_depth.is_some_and(|d| d <= DEPTH),
        "…and the depth formulation must agree: {slow_row:?}"
    );

    // THE RATE BASIS, and this is the assertion a frames-counting implementation
    // cannot satisfy: the fast tap DROPPED most of its frames (depth 2, held
    // shut for seconds), yet its rate must still be the rate the PUBLISHER
    // committed at.
    //
    // The oracle is what the publisher ACHIEVED on this run — `fast_sent` frames
    // over the measured `wall` — rather than the nominal period, because a
    // sleeping publisher on a loaded runner is slower than its period says. The
    // band is wide (0.4x - 1.6x) on purpose: the rate window spans first-drained
    // frame to last-drained frame, which is a little shorter than the wall, and
    // no tighter band is needed for the discrimination. A frames-DRAINED basis
    // reports roughly `depth / gap` — measured ~50 Hz here against a published
    // ~265 Hz — which is far outside it.
    assert!(
        !fast_row.rate_is_floor,
        "a single-writer topic with parseable headers must get the exact basis: {fast_row:?}"
    );
    let fast_mhz = fast_row.rate_mhz.expect("a verdict with a rate");
    let published_mhz =
        u64::try_from(u128::from(fast_sent) * 1_000_000_000_000u128 / wall.as_nanos().max(1))
            .expect("a plausible rate");
    assert!(
        fast_mhz * 5 >= published_mhz * 2 && fast_mhz * 5 <= published_mhz * 8,
        "the rate must count the frames the queue DROPPED: the publisher sent \
         {fast_sent} frames in {wall:?} (~{published_mhz} mHz) and the verdict \
         reports {fast_mhz} mHz"
    );
    assert!(
        slow_row.rate_mhz.is_some_and(|m| m < fast_mhz / 4),
        "…and the slow topic's rate must be measurably lower: {slow_row:?}"
    );

    // THE LIVE SURFACE, asserted on ONE frame: the two verdicts and the counting
    // basis must be co-located, because a reader deciding whether a zero
    // elsewhere is good news needs the basis beside the verdict rather than in
    // a different frame. The wait above already converged on this pair, so the
    // two verdict asserts pin the SAME frame against future drift in the
    // helper's predicate rather than doing fresh discrimination.
    let (fast_live, slow_live, basis_live) = live;
    assert_eq!(fast_live, "short", "the live frame's fast verdict");
    assert_eq!(slow_live, "absorbs", "…and its slow control");
    assert_eq!(
        basis_live, "prefix_invisible",
        "the counting basis rides the SAME frame, in the SAME word the bag uses"
    );

    // The recorder's own tap accounting is unaffected by any of this.
    assert_eq!(
        run.unpriced_taps, 0,
        "both taps are budgeted, so both price themselves"
    );
    assert!(
        run.shm_pinned_bytes <= 2 * DEPTH * SLOT,
        "the pinned bound must not exceed what two depth-{DEPTH} queues could hold: {} > {}",
        run.shm_pinned_bytes,
        2 * DEPTH * SLOT
    );

    Attempt::Probative
}

/// THE LOUD LINE: a tap that cannot absorb says so ONCE per regime, naming what
/// an operator needs — including what its loss numbers cannot see.
///
/// INVERSE HARNESS — `run_bagd` on the TEST thread, stimulus on helpers —
/// because `tracing-test` scopes its capture to the test's own span and the
/// recorder's warn would otherwise be invisible (the
/// `flashback_tap_depth_e2e_test::a_tap_the_joined_width_put_over_its_budget_says_so_once`
/// precedent).
///
/// The stimulus is a STUTTERING drain — short open windows separated by longer
/// closed ones — and that shape is what makes the "once per regime" claim
/// testable at all. A single long stall cannot: while the gate is shut nothing
/// drains, so the rate window's span never advances and no verdict can be minted
/// until the reopen — leaving exactly ONE short evaluation, which a reporter
/// with no latch at all would also produce. Stuttering advances the window on
/// every open period while the closed ones fill the histogram, so the regime is
/// genuinely re-evaluated across several `ABSORBANCE_EVAL_INTERVAL`s and the
/// suppressed DEBUG repeats are the proof of it.
#[test]
#[tracing_test::traced_test]
fn a_tap_that_cannot_absorb_says_so_once_per_regime_and_names_the_counting_caveat() {
    let captured = || {
        let all = RefCell::new(Vec::new());
        logs_assert(|lines: &[&str]| {
            *all.borrow_mut() = lines.iter().map(|l| (*l).to_string()).collect();
            Ok(())
        });
        all.into_inner()
    };
    rerun_non_probative("the loud arm", |attempt| loud_attempt(attempt, &captured));
}

/// One attempt of the loud arm's scenario, on its own manager, recorder, topic
/// and bag (see [`rerun_non_probative`]). `captured` returns every line the
/// test's log capture holds, which [`attempt_lines`] narrows to this attempt's.
fn loud_attempt(attempt: u32, captured: &dyn Fn() -> Vec<String>) -> Attempt {
    let mgr = make_manager(16);
    let dir = temp_dir("loud");
    let topic = unique_topic("absorb_loud");
    let out = unique_out("absorb_loud");
    let ready = unique_ready_file("absorb_loud");

    let mut pubr = publisher_with_provisioning(&mgr, &topic, BORROW, CEILING, SLICE);

    let gate = Arc::new(AtomicBool::new(true));
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = BagdConfig::new(out.clone(), vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    cfg.ready_file = Some(ready.clone());
    cfg.flashback = Some(settings(&dir));
    cfg.fault_inject_tap_drain_gate = Some(gate.clone());
    // This recorder attached to a publisher that was ALREADY RUNNING, so it
    // cannot prove a head loss — which is exactly the caveat the line must
    // carry, and it is the shipped default rather than a dial this arm turns.
    assert!(
        !cfg.armed_before_producers,
        "the caveat under test is the UNARMED one"
    );

    let stop = Arc::new(AtomicBool::new(false));
    let stim_stop = Arc::clone(&stop);
    let stim_ready = ready.clone();
    let stim_gate = Arc::clone(&gate);
    let stim_shutdown = Arc::clone(&shutdown);
    let stimulus = std::thread::spawn(move || {
        await_bagd_ready(&stim_ready, "the absorbance loud arm");
        let body = vec![0xAB; 256];
        let pumping = std::time::Instant::now();
        let pump =
            std::thread::spawn(move || publish_until(&mut pubr, FAST_PERIOD, &stim_stop, &body));
        // STUTTER: each OPEN window drains the backlog and advances the rate
        // window's span; each CLOSED window paces the drive loop at `WAIT_TICK`
        // and fills the histogram. After the first cycle the window spans more
        // than `ABSORBANCE_MIN_RATE_SPAN`, so every evaluation from then on has
        // both a rate and a growing tail — i.e. an OPEN regime to re-evaluate.
        for _ in 0..STUTTER_CYCLES {
            stim_gate.store(true, Ordering::Relaxed);
            std::thread::sleep(STUTTER_OPEN);
            stim_gate.store(false, Ordering::Relaxed);
            std::thread::sleep(STUTTER_CLOSED);
        }
        stim_gate.store(true, Ordering::Relaxed);
        std::thread::sleep(Duration::from_millis(300));
        stop.store(true, Ordering::Relaxed);
        let sent = pump.join().expect("publisher");
        let published_for = pumping.elapsed();
        stim_shutdown.store(true, Ordering::Relaxed);
        (sent, published_for)
    });

    let summary = run_bagd(mgr.clone(), cfg, shutdown.clone()).expect("clean finalize");
    let (sent, published_for) = stimulus.join().expect("stimulus");

    let run = LoudRun {
        row: row(&summary, &topic),
        topic: topic.clone(),
        gaps: summary.record_health.drain_gaps.clone(),
        sent,
        published_for,
        lines: attempt_lines(&captured(), attempt),
    };
    let outcome = judge_loud(attempt, &run);
    cleanup_artifacts(&out, &dir, &ready);
    outcome
}

/// What one loud-arm attempt recorded, gathered before any of it is judged, so
/// [`judge_loud`] runs on a seeded run exactly as it does on a live one.
#[derive(Debug, Clone)]
struct LoudRun {
    topic: String,
    /// The drain-gap histogram, off the run's health document.
    gaps: DrainGapHistogram,
    row: TopicAbsorbance,
    /// Frames the publisher committed, and the wall time it published for.
    sent: u32,
    published_for: Duration,
    /// This attempt's captured log lines ([`attempt_lines`]).
    lines: Vec<String>,
}

/// Judge one loud-arm attempt: [`classify_attempt`] on its recorded evidence,
/// then, on a probative attempt, every assertion the arm makes.
fn judge_loud(attempt: u32, run: &LoudRun) -> Attempt {
    let (a, topic) = (&run.row, &run.topic);
    let (sent, published_for) = (run.sent, run.published_for);
    // MID-RUN: the head is minted at the first evaluation that found the tap
    // short, against the histogram as it stood then.
    let evidence = AttemptEvidence::new(
        &run.gaps,
        TailRead::MidRun,
        &[TapRun {
            what: "the loud arm's tap",
            period: FAST_PERIOD,
            sent,
            published_for,
        }],
    );
    let shown = attempt_evidence(&evidence, &[a]);
    eprintln!("the loud arm, attempt {attempt}: {shown}");
    // NON-PROBATIVE, decided by `classify_attempt` from this attempt's recorded
    // histogram and its publisher's own count, ahead of every assertion below.
    // No field of the row or of the head line enters that decision.
    if let Some(non_probative) = non_probative(&evidence, &shown) {
        return non_probative;
    }
    // PRECONDITION: it is short at the rate the arm commanded. A depth-2 queue at
    // 400 Hz holds 5 ms, under the 10 ms the closed drain paces the loop at.
    assert_measured_rate_in_band("the loud arm's tap", a, FAST_PERIOD, sent, published_for);
    // PRECONDITION: the run really did end SHORT, so the log assertions below
    // are about a condition that occurred.
    assert!(sent > 100, "the fixture must really publish: {sent}");
    assert_eq!(a.verdict, AbsorbanceVerdict::Short, "{a:?}");
    // The one STICKY field on the row, and the only thing that keeps a topic
    // which fell short EARLIER in a run on the terminal roll-up once it
    // recovers. Its DERIVATION is production-only — every other assertion on it
    // in this repo reads a hand-constructed row — so it is pinned here, where
    // this same run's log already establishes at least two loud evaluations
    // (one head plus at least one suppressed repeat, asserted below).
    assert!(
        a.short_evaluations >= 2,
        "the recorder must COUNT the evaluations that found this tap short: {a:?}"
    );

    let check = |lines: &[&str]| -> Result<(), String> {
        let heads: Vec<&&str> = lines
            .iter()
            // The TERMINAL roll-up is a WARN on the same subject, so the head
            // is matched by its own opening — `tap CANNOT ABSORB` against the
            // roll-up's `recording carried taps that COULD NOT ABSORB`. The two
            // share no phrase today, so this is belt-and-braces rather than the
            // only thing separating them; it is kept because a future edit to
            // either line must not be able to make this arm count two.
            .filter(|l| l.contains("tap CANNOT ABSORB"))
            .filter(|l| line_is_warn(l))
            .collect();
        // The per-topic regime opens ONCE. The evaluation runs on a 1 s
        // throttle across a >3 s run, so a per-evaluation report would show
        // here as several lines.
        if heads.len() != 1 {
            return Err(format!(
                "the shortfall must be reported ONCE per regime, not per evaluation; got {} \
                 WARN line(s): {heads:?}",
                heads.len()
            ));
        }
        // "Exactly one head" only catches a latchless reporter if the
        // regime was evaluated MORE THAN ONCE while open. It is not: the
        // evaluation runs on a 1 s throttle and this run is ~3 s, so a shift of
        // a hundred milliseconds could leave the finalize evaluation as the only
        // short one — at which point a reporter with no latch at all emits
        // exactly one WARN and passes. The suppressed DEBUG repeat is the proof
        // the latch was asked twice.
        let repeats = lines
            .iter()
            .filter(|l| l.contains("suppressed repeat"))
            .count();
        if repeats < 1 {
            return Err(format!(
                "ANTI-VACUITY: the open regime must be re-evaluated, or 'exactly one head' \
                 is satisfied by a reporter with no latch at all; got {repeats} suppressed \
                 repeat(s)"
            ));
        }
        let head = heads[0];
        for needle in [
            "WINDOW AVERAGE",
            "DRIVE-LOOP gaps",
            "prefix_lost",
            "nothing was COUNTED",
            "loss_counting_basis=\"prefix_invisible\"",
            // The REMEDY is the one that can move THIS tap's depth (the fixture
            // is window-only, so it is budgeted).
            "depth_mode=\"budgeted\"",
            "CERULION_FLASHBACK_TAP_BUDGET_MB",
        ] {
            if !head.contains(needle) {
                return Err(format!("the head must carry `{needle}`: {head}"));
            }
        }
        // The DIAGNOSTIC FIELD SET: an operator who reads only this line must be
        // able to see what the queue holds and what it would need to.
        //
        // WHOLE TOKENS, and every numeric field must carry a MEASURED value
        // rather than merely be present. Both halves were learned the hard way:
        // `contains("tap_buffer_depth=2")` is satisfied by `tap_buffer_depth=24`,
        // and a bare `absorbance_us=` is satisfied by a site that fabricated
        // `absorbance_us=0` on a plainly-streaming topic — which is exactly what
        // these fields' own contract forbids ("None is UNKNOWN, never 0").
        if !has_field(head, "tap_buffer_depth", "2") {
            return Err(format!("the head must carry tap_buffer_depth=2: {head}"));
        }
        let measured = |key: &str| -> Result<(), String> {
            let Some(tok) = field_token(head, key) else {
                return Err(format!("the head must carry `{key}=`: {head}"));
            };
            if !tok.starts_with("Some(") || tok == "Some(0)" {
                return Err(format!(
                    "`{key}` must carry a MEASURED value, not an unknown and not a \
                     fabricated zero; got `{tok}`: {head}"
                ));
            }
            Ok(())
        };
        // The fields a short verdict measures whatever its tail.
        for key in ["absorbance_us", "shortfall_at_least_us", "rate_mhz"] {
            measured(key)?;
        }
        // The two fields an OVERFLOW tail leaves unknown by contract. This
        // attempt is probative under `TailRead::MidRun`, so its final histogram
        // holds no gap past the ladder's top edge, and counts only grow: no
        // evaluation during the run read an overflow tail, the head's included.
        // A head that reaches here without a tail or a required depth is the
        // code under test.
        for key in ["required_depth", "measured_tail_us"] {
            measured(key)?;
        }
        // The TERMINAL roll-up rides the same run, and names the topic.
        let terminal: Vec<&&str> = lines
            .iter()
            .filter(|l| l.contains("recording carried taps that COULD NOT ABSORB"))
            .filter(|l| line_is_warn(l))
            .collect();
        if terminal.len() != 1 {
            return Err(format!(
                "the terminal roll-up must fire once: got {terminal:?}"
            ));
        }
        // …and it must NAME the short topic — a roll-up that fires with the
        // right phrase but an empty or mislabelled topic list would pass a
        // count check while telling the operator nothing actionable.
        if !terminal[0].contains(topic.as_str()) {
            return Err(format!(
                "the terminal roll-up must name the short topic `{topic}`: {}",
                terminal[0]
            ));
        }
        Ok(())
    };
    let lines: Vec<&str> = run.lines.iter().map(String::as_str).collect();
    if let Err(msg) = check(&lines) {
        panic!("the loud arm's log assertions returned an error: {msg}");
    }

    Attempt::Probative
}

/// THE ANTI-TAUTOLOGY CONTROL: a recorder whose taps absorb everything it
/// measured says NOTHING, and its rows still carry a verdict.
///
/// Without this, every "exactly one line" assertion above would be satisfied by
/// a reporter that fired on healthy taps too — and the silence is the promise
/// this feature makes to a robot that is fine.
#[test]
#[tracing_test::traced_test]
fn a_recorder_whose_taps_absorb_is_silent_and_still_carries_its_verdict() {
    let captured = || {
        let all = RefCell::new(Vec::new());
        logs_assert(|lines: &[&str]| {
            *all.borrow_mut() = lines.iter().map(|l| (*l).to_string()).collect();
            Ok(())
        });
        all.into_inner()
    };
    rerun_non_probative("the quiet arm", |attempt| quiet_attempt(attempt, &captured));
}

/// One attempt of the quiet arm's scenario, on its own manager, recorder, topic
/// and bag (see [`rerun_non_probative`]). `captured` returns every line the
/// test's log capture holds, which [`attempt_lines`] narrows to this attempt's.
fn quiet_attempt(attempt: u32, captured: &dyn Fn() -> Vec<String>) -> Attempt {
    let mgr = make_manager(16);
    let dir = temp_dir("quiet");
    let topic = unique_topic("absorb_quiet");
    let out = unique_out("absorb_quiet");
    let ready = unique_ready_file("absorb_quiet");

    let mut pubr = publisher_with_provisioning(&mgr, &topic, BORROW, CEILING, SLICE);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = BagdConfig::new(out.clone(), vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    cfg.ready_file = Some(ready.clone());
    cfg.flashback = Some(settings(&dir));

    let stop = Arc::new(AtomicBool::new(false));
    let stim_stop = Arc::clone(&stop);
    let stim_ready = ready.clone();
    let stim_shutdown = Arc::clone(&shutdown);
    // The stimulus owns its OWN window, because `run_bagd` runs on THIS thread:
    // sleeping here before the call would flip `stop` before the recorder had
    // started and the publisher would send nothing (measured — the arm failed
    // its own `sent > 20` precondition).
    let stimulus = std::thread::spawn(move || {
        await_bagd_ready(&stim_ready, "the absorbance quiet arm");
        let body = vec![0xAB; 256];
        let pumping = std::time::Instant::now();
        let pump = std::thread::spawn(move || {
            // SLOW only, and no stall: a depth-2 queue at 5 Hz holds 400 ms, over
            // the ladder's top edge, so it covers every in-ladder tail an
            // unblocked drive loop can produce (see `SLOW_PERIOD`).
            publish_until(&mut pubr, SLOW_PERIOD, &stim_stop, &body)
        });
        std::thread::sleep(Duration::from_millis(2_500));
        stop.store(true, Ordering::Relaxed);
        let sent = pump.join().expect("publisher");
        let published_for = pumping.elapsed();
        stim_shutdown.store(true, Ordering::Relaxed);
        (sent, published_for)
    });

    let summary = run_bagd(mgr.clone(), cfg, shutdown.clone()).expect("clean finalize");
    let (sent, published_for) = stimulus.join().expect("stimulus");
    // The positive control for the absence guard below, emitted inside this
    // attempt's span so `attempt_lines` keeps it.
    tracing::warn!("{QUIET_PROBE}");

    let run = QuietRun {
        row: row(&summary, &topic),
        gaps: summary.record_health.drain_gaps.clone(),
        sent,
        published_for,
        lines: attempt_lines(&captured(), attempt),
    };
    let outcome = judge_quiet(attempt, &run);
    cleanup_artifacts(&out, &dir, &ready);
    outcome
}

/// What one quiet-arm attempt recorded, gathered before any of it is judged, so
/// [`judge_quiet`] runs on a seeded run exactly as it does on a live one.
#[derive(Debug, Clone)]
struct QuietRun {
    /// The drain-gap histogram, off the run's health document.
    gaps: DrainGapHistogram,
    row: TopicAbsorbance,
    /// Frames the publisher committed, and the wall time it published for.
    sent: u32,
    published_for: Duration,
    /// This attempt's captured log lines ([`attempt_lines`]).
    lines: Vec<String>,
}

/// Judge one quiet-arm attempt: [`classify_attempt`] on its recorded evidence,
/// then, on a probative attempt, every assertion the arm makes.
fn judge_quiet(attempt: u32, run: &QuietRun) -> Attempt {
    // The row EXISTS and it ABSORBS, the half that makes the silence meaningful
    // rather than a recorder that evaluated nothing.
    let a = &run.row;
    let (sent, published_for) = (run.sent, run.published_for);
    // AT FINALIZE: the row. The loud lines this arm forbids come only from
    // `Short` evaluations, and an evaluation against an overflow tail leaves a
    // tap whose absorbance is over the ladder's top edge `Unrankable`, not
    // `Short`.
    let evidence = AttemptEvidence::new(
        &run.gaps,
        TailRead::AtFinalize,
        &[TapRun {
            what: "the quiet arm's tap",
            period: SLOW_PERIOD,
            sent,
            published_for,
        }],
    );
    let shown = attempt_evidence(&evidence, &[a]);
    eprintln!("the quiet arm, attempt {attempt}: {shown}");
    // NON-PROBATIVE, decided by `classify_attempt` from this attempt's recorded
    // histogram and its publisher's own count, ahead of every assertion below.
    // The row's verdict does not enter that decision: against a final 0.999
    // reading inside the ladder a tap that cannot absorb fails below.
    if let Some(non_probative) = non_probative(&evidence, &shown) {
        return non_probative;
    }
    // PRECONDITION: the tap is priced at its commanded 5 Hz, which is what puts
    // its absorbance over the ladder's top edge, and its publisher committed
    // inside the band, so the measured rate must sit there too.
    assert_measured_rate_in_band("the quiet arm's tap", a, SLOW_PERIOD, sent, published_for);
    assert!(
        a.rate_mhz.is_some(),
        "the tap must have measured a rate, or its silence proves nothing: {a:?}"
    );
    // ANTI-VACUITY, bounded by the rate the recorder minted rather than by a
    // throughput this runner had to reach. `RateWindow::estimate` mints a rate
    // only over a drain-clock span of at least `ABSORBANCE_MIN_RATE_SPAN`, and the
    // band above puts it at no less than `RATE_BAND_LOW` of the commanded rate, so
    // the window counted at least `low * span / period` commits past its anchor
    // frame, and the publisher committed that anchor too. A count of grid slots
    // instead (the window holds 13) also loses every slot that falls due after
    // the publisher's last wake before the stop flag, which is the host's
    // scheduling and not the recorder.
    let (lo_n, lo_d) = RATE_BAND_LOW;
    let min_sent = 1
        + (u128::from(lo_n) * ABSORBANCE_MIN_RATE_SPAN.as_nanos())
            .div_ceil(u128::from(lo_d) * SLOW_PERIOD.as_nanos());
    assert!(
        u128::from(sent) >= min_sent,
        "the recorder minted {:?} mHz over at least {ABSORBANCE_MIN_RATE_SPAN:?}, which \
         accounts for at least {min_sent} committed frames, but the fixture committed \
         {sent}: {a:?}",
        a.rate_mhz
    );
    // The row's tail sits under its absorbance. The tail is `quantile_us(0.999)`,
    // which over a run of a few hundred passes is the run MAXIMUM (`rank =
    // ceil(0.999 * total)` IS `total`). The rate band caps the measured rate at
    // 3/2 of 5 Hz, where depth 2 holds about 267 ms, over the ladder's 250 ms top
    // edge, and an overflow tail re-runs the attempt above, so a tail at or past
    // this absorbance is one the ladder cannot serve. Read against the row's OWN
    // numbers rather than a constant.
    if let (Some(tail), Some(absorbance)) = (a.measured_tail_us, a.absorbance_us) {
        assert!(
            tail < absorbance,
            "this run's in-ladder tail ({tail} us) must sit under what this in-band tap \
             can hold ({absorbance} us): {a:?}"
        );
    }
    assert_eq!(
        a.verdict,
        AbsorbanceVerdict::Absorbs,
        "an unstalled recorder's 5 Hz tap must absorb its own gaps: {a:?}"
    );

    let check = |lines: &[&str]| -> Result<(), String> {
        // POSITIVE CONTROL FIRST: an absence guard over an EMPTY
        // capture passes vacuously, so this closure proves the capture is live
        // before it proves anything is missing from it. The probe is emitted by
        // the test body itself, so it cannot be silenced by a change to bagd.
        let probes = lines.iter().filter(|l| l.contains(QUIET_PROBE)).count();
        if probes != 1 {
            return Err(format!(
                "the log capture is not live (expected exactly 1 probe line, got \
                 {probes}), so the absence assertions below would be vacuous"
            ));
        }
        for forbidden in [
            "CANNOT ABSORB this run's own drain stalls",
            "recording carried taps that COULD NOT ABSORB",
        ] {
            if let Some(l) = lines.iter().find(|l| l.contains(forbidden)) {
                return Err(format!("a healthy recorder must say nothing; got: {l}"));
            }
        }
        Ok(())
    };
    let lines: Vec<&str> = run.lines.iter().map(String::as_str).collect();
    if let Err(msg) = check(&lines) {
        panic!("the quiet arm's log assertions returned an error: {msg}");
    }

    Attempt::Probative
}

/// U4: a Flashback CAPTURE carries the recorder's own health document, so a bag
/// from an unknown robot can say what its taps could and could not absorb.
///
/// STRUCTURAL rather than behavioural, and deliberately so: driving a real
/// capture needs the trigger plane, while what U4 asks is whether the health
/// document is in the attachment set the capture writer is handed. The walk is
/// over the COMMENT-STRIPPED source, so a comment naming the attachment does not
/// satisfy it (the `cdylib_iox2_log_level_test` precedent).
#[test]
fn a_capture_carries_the_recorders_own_health_document() {
    let src = std::fs::read_to_string(
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("src")
            .join("lib.rs"),
    )
    .expect("read bagd lib.rs");
    let code = common::code_only(&src);
    let body = close_capture_body(&code);
    assert!(
        body.contains("CAPTURE_RECORDER_HEALTH_ATTACHMENT.to_string()"),
        "a capture must push the health document into its attachment \
         set — without it the drain-gap histogram, the per-topic loss counts and \
         the absorbance verdict reach a `--record` bag and NOTHING else, which is \
         exactly the recorder whose loss boundary D11(a) narrowed"
    );
    // EXPRESSION LINKAGE, not two independent `contains`. The pair
    // above is satisfied by a `close_capture` that pushes stale or empty bytes
    // under the name and calls `build_record_health()` somewhere else entirely —
    // which is the whole failure this pin exists to catch, since a capture
    // carrying an EMPTY health document is worse than one carrying none (it
    // reads as a recorder that measured nothing rather than as a bag with no
    // report). So the assertion is that the bytes pushed under the name are the
    // SERIALIZE OF THE FRESHLY BUILT document, by data path: the push must live
    // inside that `to_vec`'s own `match`, bound to that match's `Ok` binding.
    let serialized = match_expr_on(&body, "serde_json::to_vec(&self.build_record_health())");
    let bound = ok_arm_binding(&serialized);
    assert!(
        serialized.contains(&format!(
            "attachments.push((CAPTURE_RECORDER_HEALTH_ATTACHMENT.to_string(), {bound}))"
        )),
        "the capture must push the SERIALIZE of the document it just built — \
         the `Ok({bound})` arm of `serde_json::to_vec(&self.build_record_health())` must be \
         what pushes under CAPTURE_RECORDER_HEALTH_ATTACHMENT, or the attachment's bytes and \
         the recorder's health are two unrelated values. Got: {serialized}"
    );
    // ANTI-TAUTOLOGY, two ways. (1) the stripped body really is the code, not an
    // empty string that would make every `contains` above vacuous — and
    // `CaptureJob` is near the END of the function, so a truncated body fails
    // here.
    assert!(
        body.contains("CaptureJob"),
        "the walk must have found the real function body"
    );
    // (2) the body did not OVER-extend into a later function. `code_only` strips
    // comments but NOT string literals, so an unbalanced brace inside a literal
    // would silently swallow whatever follows — at which point some other
    // function's `push` satisfies the assertion above.
    assert!(
        !body.contains("\n    fn "),
        "the extracted body ran past `close_capture` into a later function, so \
         the assertions above may be reading somebody else's code"
    );
    // (3) a COMMENT naming the attachment does not satisfy the walk — the
    // stripper is doing real work.
    let commented = common::code_only(
        "fn f() {\n    // attachments.push((CAPTURE_RECORDER_HEALTH_ATTACHMENT.to_string(), b));\n}",
    );
    assert!(
        !commented.contains("CAPTURE_RECORDER_HEALTH_ATTACHMENT"),
        "a commented-out push must not satisfy this walk: {commented}"
    );
}

/// The body of `Recorder::close_capture`, found by its DEFINITION.
///
/// NOT `common::fn_body`: `code_only` strips comments but NOT string literals,
/// and `lib.rs` carries the close-timer structural guard, which holds that
/// function's own signature as a (deliberately split) literal EARLIER in the
/// file. `fn_body` takes the first textual match, so it would brace-match from
/// inside that guard and hand back a body belonging to nothing. The definition
/// is distinguished by sitting at the START of its line — a literal is always
/// preceded by a quote or an argument.
fn close_capture_body(code: &str) -> String {
    fn_body_at_definition(code, "close_capture")
}

/// The `match` EXPRESSION whose scrutinee is exactly `scrutinee`, brace-matched
/// from its own opening brace.
///
/// What makes the U4 pin a LINKAGE assertion rather than two independent
/// substring checks: everything inside the returned text shares one data path
/// with that scrutinee, so a `push` found in here provably carries a value
/// derived from it.
fn match_expr_on(code: &str, scrutinee: &str) -> String {
    let needle = format!("match {scrutinee} {{");
    let start = code.find(&needle).unwrap_or_else(|| {
        panic!(
            "no `match {scrutinee} {{` — the expression this pin is anchored on moved, was \
                reshaped, or was split so that its result no longer reaches the push directly"
        )
    });
    let open = start + needle.len() - 1;
    let bytes = code.as_bytes();
    let mut depth = 0usize;
    for (idx, b) in bytes.iter().enumerate().skip(open) {
        match b {
            b'{' => depth += 1,
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    return code[open..=idx].to_string();
                }
            }
            _ => {}
        }
    }
    panic!("the `match {scrutinee}` expression's brace is never closed");
}

/// The identifier an `Ok(..)` arm binds, read OUT OF THE SOURCE rather than
/// assumed — a rename of the binding must not silently weaken the pin above
/// into one that passes on any arm at all.
fn ok_arm_binding(expr: &str) -> String {
    let at = expr
        .find("Ok(")
        .unwrap_or_else(|| panic!("no `Ok(` arm in: {expr}"));
    let rest = &expr[at + "Ok(".len()..];
    let end = rest
        .find(')')
        .unwrap_or_else(|| panic!("unterminated `Ok(` binding in: {expr}"));
    rest[..end].trim().to_string()
}

/// The body of the method `name`, found by its DEFINITION line.
///
/// The definition is distinguished by sitting at the START of its line at method
/// indentation — a mention inside a string literal is always preceded by a quote
/// or an argument, which is what keeps `code_only`'s (deliberate) literal
/// blindness from handing back a body belonging to nothing.
fn fn_body_at_definition(code: &str, name: &str) -> String {
    let needle = format!("\n    fn {name}(");
    let start = code
        .find(&needle)
        .unwrap_or_else(|| panic!("`fn {name}` not found — did it move or get renamed?"));
    let open = code[start..]
        .find('{')
        .unwrap_or_else(|| panic!("`fn {name}` has no body brace"))
        + start;
    let bytes = code.as_bytes();
    let mut depth = 0usize;
    for (idx, b) in bytes.iter().enumerate().skip(open) {
        match b {
            b'{' => depth += 1,
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    return code[open..=idx].to_string();
                }
            }
            _ => {}
        }
    }
    panic!("`fn {name}`'s body brace is never closed");
}

/// The VALUE of `key=` on a captured line, matched as a whole whitespace token.
///
/// `tracing` renders one field per whitespace-separated `key=value`, so this is
/// exact where `contains("key=")` is a prefix match a longer key satisfies.
fn field_token<'a>(line: &'a str, key: &str) -> Option<&'a str> {
    let want = format!("{key}=");
    line.split_whitespace().find_map(|t| t.strip_prefix(&want))
}

/// Whole-TOKEN field match: `tap_buffer_depth=2` must not be satisfied by
/// `tap_buffer_depth=24`.
fn has_field(line: &str, key: &str, value: &str) -> bool {
    field_token(line, key) == Some(value)
}

/// Whether a captured line's LEVEL is `WARN`.
///
/// Whole-token, not a substring: `tracing-test` renders the SPAN NAME — this
/// test function's own name — into every line, so a bare `contains("WARN")`
/// would be satisfied by a rename or by an uppercase field value
/// (the `line_level` rule).
fn line_is_warn(line: &str) -> bool {
    line.split_whitespace().any(|t| t == "WARN")
}

/// Wait for a `/bagd/status` frame whose `absorbance` row for `topic` satisfies
/// `want`, and return that DECODED row.
///
/// Decodes the whole `TopicAbsorbance` rather than just its verdict string,
/// because the arms that use it assert on the numbers too — the point of a
/// `NoClaim` is as much what it STOPS carrying as what it says.
///
/// The deadline is generous and is not the oracle: what is being waited for is a
/// recorder's own evaluation cadence plus a fixed idle cap, so a loaded runner
/// makes this SLOWER, never wrong.
fn await_status_verdict(
    sub: &mut cerulion_core::transport::subscriber::DataOnlySubscriber,
    topic: &str,
    want: impl Fn(&TopicAbsorbance) -> bool,
) -> TopicAbsorbance {
    const BORROW_BUDGET: usize = 2;
    let mut found: Option<TopicAbsorbance> = None;
    // The LAST row seen for this topic, whether or not it matched. A timeout
    // that only says "nothing matched" cannot distinguish a recorder that never
    // published a row from one that published the WRONG row for thirty seconds
    // — and the second is exactly what a regression here looks like.
    let mut last_seen: Option<TopicAbsorbance> = None;
    common::await_condition(Duration::from_secs(30), || {
        loop {
            let mut batch = Vec::new();
            let drained = sub.drain_owned(BORROW_BUDGET, &mut batch).expect("drain");
            for frame in batch {
                let payload = frame.payload();
                if payload.len() <= cerulion_core::wire::WireHeader::SIZE {
                    continue;
                }
                let body = &payload[cerulion_core::wire::WireHeader::SIZE..];
                let Ok(v) = serde_json::from_slice::<serde_json::Value>(body) else {
                    continue;
                };
                let Some(row) = v.get("absorbance").and_then(|a| a.get(topic)) else {
                    continue;
                };
                let Ok(row) = serde_json::from_value::<TopicAbsorbance>(row.clone()) else {
                    continue;
                };
                if want(&row) {
                    found = Some(row);
                } else {
                    last_seen = Some(row);
                }
            }
            if drained == 0 {
                break;
            }
        }
        found.is_some()
    });
    found.unwrap_or_else(|| match last_seen {
        Some(row) => panic!(
            "no /bagd/status frame carried a MATCHING absorbance row for {topic}; the last \
             one it did carry was {row:?}"
        ),
        None => panic!("no /bagd/status frame carried an absorbance row for {topic} at all"),
    })
}

/// One `/bagd/status` frame's `(fast verdict, slow verdict, loss_counting_basis)`.
type LiveFrame = (String, String, String);

/// Wait for a `/bagd/status` frame whose verdicts have CONVERGED to
/// `(fast = short, slow = absorbs)`, and return
/// `(fast verdict, slow verdict, loss_counting_basis)` off that ONE frame.
///
/// CONVERGENCE-POLLED rather than "the first frame carrying both verdicts":
/// a row enters the status map at its FIRST evaluation (including
/// a `no_claim` minted before either rate window has closed, and a pre-stall
/// `absorbs` minted when a loaded runner stretches the initial open window
/// past `ABSORBANCE_MIN_RATE_SPAN`), and the evaluation re-runs only once per
/// `ABSORBANCE_EVAL_INTERVAL` (1 s), so the newest status frame trails the
/// stall by up to an interval. The one-shot form of this helper therefore
/// raced the evaluation cadence with a ~100 ms margin (reopen sleep 300 ms vs
/// an eval landing at ~t+1.0 s ticks), and a loaded runner inverted it into
/// `fast_live == "no_claim"`/`"absorbs"` at the headline's live assert — the
/// macOS shard-0 flake. Waiting for the converged pair is a bound load can
/// only DELAY, never invert: the recorder keeps evaluating
/// while the wait runs, and the post-stall state IS (short, absorbs).
///
/// The oracle still bites: a live surface that never states the boundary (the
/// status absorbance map deleted), a verdict that always answers `absorbs`,
/// and one that always answers `short` (the slow control flips) all exhaust
/// the deadline and return `Err` with the last both-verdict frame seen, which
/// the headline fails on through [`live_never_converged`] whenever the attempt
/// is probative. Whether it is comes from [`classify_attempt`] on the attempt's
/// recorded histogram and publisher counts, never from the frames this wait
/// read: a run whose final 0.999 reading lands in the overflow bucket is
/// non-probative whatever its feed showed, and one whose reading sits inside
/// the ladder fails here whatever its last frame carried.
///
/// The frames are drained in chunks of the DEFAULT borrow budget: an
/// `OwnedInboundSample` holds its SHM borrow until dropped, so asking for more
/// than the budget is an `ExceedsMaxBorrows` the moment a loaded runner queues a
/// backlog (the `flashback_tap_depth_e2e_test::drain_status` lesson).
fn await_status_verdicts(
    sub: &mut cerulion_core::transport::subscriber::DataOnlySubscriber,
    fast: &str,
    slow: &str,
) -> Result<LiveFrame, Option<LiveFrame>> {
    const BORROW_BUDGET: usize = 2;
    let mut found = None;
    let mut last_seen: Option<LiveFrame> = None;
    common::await_condition(Duration::from_secs(30), || {
        loop {
            let mut batch = Vec::new();
            let drained = sub.drain_owned(BORROW_BUDGET, &mut batch).expect("drain");
            for frame in batch {
                let payload = frame.payload();
                if payload.len() <= cerulion_core::wire::WireHeader::SIZE {
                    continue;
                }
                let body = &payload[cerulion_core::wire::WireHeader::SIZE..];
                let Ok(v) = serde_json::from_slice::<serde_json::Value>(body) else {
                    continue;
                };
                let verdict = |topic: &str| -> Option<String> {
                    Some(
                        v.get("absorbance")?
                            .get(topic)?
                            .get("verdict")?
                            .as_str()?
                            .to_string(),
                    )
                };
                if let (Some(f), Some(s), Some(b)) = (
                    verdict(fast),
                    verdict(slow),
                    v.get("loss_counting_basis")
                        .and_then(|b| b.as_str())
                        .map(str::to_string),
                ) {
                    if f == "short" && s == "absorbs" {
                        found = Some((f, s, b));
                    } else {
                        last_seen = Some((f, s, b));
                    }
                }
            }
            if drained == 0 {
                break;
            }
        }
        found.is_some()
    });
    found.ok_or(last_seen)
}

/// The headline's failure for a live wait that never converged (see
/// [`await_status_verdicts`]): `last_seen` is the last frame that carried a
/// verdict for both topics, if any did.
fn live_never_converged(last_seen: Option<LiveFrame>) -> LiveFrame {
    match last_seen {
        Some((f, s, b)) => panic!(
            "no /bagd/status frame converged to (fast=short, slow=absorbs) while the \
             run was live; the last both-verdict frame carried (fast={f}, slow={s}, \
             basis={b}). This attempt's recorded 0.999 drain-gap reading sits inside \
             the ladder and its publishers ran inside their bands, so this is the live \
             surface failing to state the absorbance boundary DURING the run"
        ),
        None => panic!(
            "no /bagd/status frame carried a verdict for both topics at all: the \
             live status feed is the one surface that states the absorbance boundary \
             DURING the run"
        ),
    }
}

/// A TWO-WRITER topic gets a LABELLED FLOOR, never a confident number — while
/// its single-writer sibling on the SAME recorder stays exact.
///
/// # Why this arm exists, and why it is an e2e rather than a unit
///
/// `sequence` is a PER-PUBLISHER counter, so with two writers the
/// batch maximum HOPS between unrelated ladders and a rate differenced across
/// them is a fabrication — measured at 1550 Hz for a ~101 Hz two-writer `/tf`
/// without the rule. The rule is `RateWindow::note_multi_writer`,
/// which the pure arms drive DIRECTLY; nothing there can see whether production
/// ever CALLS it. This is the wiring: deleting the call site ships an exact-basis
/// rate on a two-writer topic with every pure arm green.
///
/// The single-writer sibling is not decoration. It shares the recorder, the
/// window and the publish period, so "everything is a floor" — the cheapest way
/// to pass the first assertion — fails on it.
///
/// The drain runs FREELY here (no gate): this arm is about the rate BASIS, and a
/// stall would only make both windows fall back to the frames basis, which is
/// the very label the multi-writer arm is trying to earn for the right reason.
#[test]
fn a_two_writer_topic_reports_a_labelled_floor_while_its_solo_sibling_stays_exact() {
    /// Both topics publish at this period. Fast enough that a ~2.5 s window
    /// holds hundreds of frames, so the rate window closes on real drains.
    const PERIOD: Duration = Duration::from_millis(5);
    /// The window the publishers run for — comfortably over
    /// `ABSORBANCE_MIN_RATE_SPAN` (1 s), so a window really does close.
    const WINDOW: Duration = Duration::from_millis(2_500);
    /// The multi-publisher topic's slice. Small: this arm prices no depth.
    const MULTI_SLICE: u32 = 4096;

    let mgr = make_manager(16);
    let dir = temp_dir("floor");
    let multi_topic = unique_topic("absorb_multi");
    let solo_topic = unique_topic("absorb_solo");
    let out = unique_out("floor");
    let ready = unique_ready_file("floor");

    // The FIRST multi writer creates the service; the second opens it by
    // equality (see `multi_publisher_with_provisioning`'s own doc).
    let mut writer_a = multi_publisher_with_provisioning(&mgr, &multi_topic, CEILING, MULTI_SLICE);
    let mut writer_b = multi_publisher_with_provisioning(&mgr, &multi_topic, CEILING, MULTI_SLICE);
    let mut solo = publisher_with_provisioning(&mgr, &solo_topic, BORROW, CEILING, SLICE);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = BagdConfig::new(
        out.clone(),
        // NEITHER tap is declared multi-publisher: the floor must be earned by
        // OBSERVATION, which is the case a declaration-only check cannot see.
        vec![TapSpec::attach(&multi_topic), TapSpec::attach(&solo_topic)],
    );
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.ready_file = Some(ready.clone());
    cfg.flashback = Some(settings(&dir));

    let handle = {
        let m = Arc::clone(&mgr);
        let c = cfg.clone();
        let s = Arc::clone(&shutdown);
        std::thread::spawn(move || run_bagd(m, c, s))
    };
    await_bagd_ready(&ready, "the absorbance floor arm");

    let stop = Arc::new(AtomicBool::new(false));
    let body = vec![0xCD; 128];
    let (a_stop, b_stop, s_stop) = (Arc::clone(&stop), Arc::clone(&stop), Arc::clone(&stop));
    let (a_body, b_body) = (body.clone(), body.clone());
    let a = std::thread::spawn(move || publish_until(&mut writer_a, PERIOD, &a_stop, &a_body));
    let b = std::thread::spawn(move || publish_until(&mut writer_b, PERIOD, &b_stop, &b_body));
    let so = std::thread::spawn(move || publish_until(&mut solo, PERIOD, &s_stop, &body));

    std::thread::sleep(WINDOW);
    stop.store(true, Ordering::Relaxed);
    let sent_a = a.join().expect("writer A");
    let sent_b = b.join().expect("writer B");
    let sent_solo = so.join().expect("solo writer");
    shutdown.store(true, Ordering::Relaxed);
    let summary = join_bagd(handle, &shutdown, "the absorbance floor arm").expect("recorder");

    // PRECONDITION: BOTH writers really published, or plurality was never
    // observable and the floor below would be earned by nothing.
    assert!(
        sent_a > 0 && sent_b > 0 && sent_solo > 0,
        "the fixture must really run three publishers: a={sent_a} b={sent_b} solo={sent_solo}"
    );

    let health = &summary.record_health.topics;
    let multi_row = health
        .get(&multi_topic)
        .and_then(|h| h.absorbance)
        .expect("the multi-writer tap must carry a verdict row");
    let solo_row = health
        .get(&solo_topic)
        .and_then(|h| h.absorbance)
        .expect("the single-writer tap must carry a verdict row");

    // THE CLAIM, asserted FIRST and deliberately not behind the rate
    // precondition below: the basis bit is retired the moment plurality is
    // OBSERVED, whether or not a window has closed since. Ordering it after the
    // precondition made a wiring defect surface as "no rate" rather than as
    // "unlabelled basis" — MEASURED: with the call site neutered this row reads
    // `rate_mhz: None, rate_is_floor: false`, because two interleaved writers
    // make the batch maximum jump BACKWARD often enough that the sequence
    // regression guard re-anchors the window forever. That is a second, quieter
    // consequence of the same defect, and it is not the one this arm is named
    // for.
    assert!(
        multi_row.rate_is_floor,
        "a topic whose plurality the recorder OBSERVED must serve a LABELLED FLOOR — \
         its sequences hop between two unrelated counters: {multi_row:?}"
    );

    // PRECONDITION for the anti-tautology half: the solo window really closed.
    // Without it `!rate_is_floor` is satisfied by a tap that estimated nothing.
    assert!(
        multi_row.rate_mhz.is_some() && solo_row.rate_mhz.is_some(),
        "both taps must have closed a rate window over a {WINDOW:?} run: \
         multi={multi_row:?} solo={solo_row:?}"
    );
    // THE ANTI-TAUTOLOGY HALF: an implementation that labels everything a floor
    // passes the assertion above and fails here.
    assert!(
        !solo_row.rate_is_floor,
        "a single-writer tap's sequence IS a commit counter, so its rate is EXACT \
         and must not be labelled a floor: {solo_row:?}"
    );

    cleanup_artifacts(&out, &dir, &ready);
}

/// BOTH finalize paths roll the shortfall up on the terminal line — the
/// `--record` one as well as the window-only one.
///
/// STRUCTURAL, and the reason is the arm it complements: the loud e2e above
/// runs `window_only: true`, so it drives `finalize_window_only` and is
/// STRUCTURALLY BLIND to the `--record` path, where `finalize_threaded` writes a
/// continuous bag. Driving that behaviourally means a second full recorder run
/// with a real writer thread for one log line, so the cheap half is asserted
/// here and the expensive half is asserted once, on the plane D11(a) narrowed.
///
/// What it buys: the roll-up is the ONLY surface that reports a topic that fell
/// short EARLIER IN THE RUN and recovered before shutdown (nothing on the row is
/// sticky except `short_evaluations`, which nothing else prints), so losing it
/// on the `--record` path would make a whole episode survive only in a log line
/// that has scrolled away.
#[test]
fn both_finalize_paths_roll_the_shortfall_up_on_the_terminal_line() {
    let src = std::fs::read_to_string(
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("src")
            .join("lib.rs"),
    )
    .expect("read bagd lib.rs");
    let code = common::code_only(&src);

    // …and the label tripwires run at the TERMINAL paths ONLY. They
    // used to live inside `build_record_health`, which had exactly those two
    // callers; it now has a third — `close_capture`, mid-run — where
    // running them would put a fleet of synchronous `warn!`s inside the capture
    // budget AND could consume tripwire B's one-shot latch on a state that is
    // transiently true (its two inputs are written at drain time and at flush
    // time), permanently disarming the genuine finalize-time check.
    let capture = close_capture_body(&code);
    assert!(
        !capture.contains("report_label_bookkeeping_tripwires"),
        "a mid-run capture close must not run the terminal label tripwires"
    );
    for path in ["finalize_threaded", "finalize_window_only"] {
        let body = fn_body_at_definition(&code, path);
        assert!(
            body.contains("self.report_label_bookkeeping_tripwires()"),
            "{path} is a TERMINAL path, so it owns the label-bookkeeping tripwires that \
             `build_record_health` no longer runs for it"
        );
        assert!(
            body.contains("self.log_absorbance_terminal()"),
            "{path} must roll the run's shortfall up on its terminal line, or a tap \
             that could not absorb is reported on ONE of the two finalize paths and \
             silently on the other"
        );
        // ANTI-TAUTOLOGY: the extracted body really is that function's, and did
        // not over-extend into a later one (where somebody else's call would
        // satisfy the assertion above).
        assert!(
            body.contains("build_record_health()"),
            "{path}: the walk must have found the real function body"
        );
        assert!(
            !body.contains("\n    fn "),
            "{path}: the extracted body ran past its own closing brace"
        );
    }
}

/// A topic that STREAMS and then goes SILENT past the idle cap
/// reads `NoClaim` on the LIVE status feed AND in the FINAL health document —
/// not the verdict it had while it was live.
///
/// # Why this needs a real recorder
///
/// The staleness rule is asked at EVALUATION time, and the two things it has to
/// protect are the two surfaces a real recorder produces on its own schedule:
/// the `/bagd/status` frame the drive loop publishes every pass, and the
/// `record_health.json` the finalize path builds. `RateWindow::observe`'s own
/// idle cap cannot cover either, because it only fires when a NEXT frame
/// arrives and the case here is precisely the one where none does — so before
/// the fix this topic kept serving its live `Absorbs` for the rest of the run,
/// on both surfaces, however long it stayed quiet. The liveness rate estimate's
/// rule is the one inherited: a stopped stream DROPS its rate rather than
/// decaying one.
///
/// The arm asserts the LIVE verdict first, so the `NoClaim` after it is a
/// TRANSITION rather than a topic that never had a verdict at all.
#[test]
fn a_topic_that_stops_publishing_makes_no_claim_on_status_and_at_finalize() {
    let mgr = make_manager(16);
    let dir = temp_dir("quiet_transition");
    let topic = unique_topic("absorb_stops");
    let out = unique_out("stops");
    let ready = unique_ready_file("stops");

    let mut pubr = publisher_with_provisioning(&mgr, &topic, BORROW, CEILING, SLICE);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = BagdConfig::new(out.clone(), vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = Some(Duration::from_millis(50));
    cfg.ready_file = Some(ready.clone());
    cfg.flashback = Some(settings(&dir));

    let handle = {
        let m = Arc::clone(&mgr);
        let c = cfg.clone();
        let s = Arc::clone(&shutdown);
        std::thread::spawn(move || run_bagd(m, c, s))
    };
    await_bagd_ready(&ready, "the quiet-transition arm");
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");

    // (1) STREAM, long enough to close a rate window and be evaluated.
    let body = vec![0xEE; 256];
    let stop = Arc::new(AtomicBool::new(false));
    let pub_stop = Arc::clone(&stop);
    let stream = std::thread::spawn(move || {
        publish_until(&mut pubr, Duration::from_millis(5), &pub_stop, &body)
    });
    let live = await_status_verdict(&mut status, &topic, |v| v.rate_mhz.is_some());
    assert_ne!(
        live.verdict,
        AbsorbanceVerdict::NoClaim,
        "PRECONDITION: the topic must have a real verdict while it is live, or the \
         `NoClaim` below is not a TRANSITION: {live:?}"
    );

    // (2) STOP, and stay stopped past the cap. Nothing else changes.
    stop.store(true, Ordering::Relaxed);
    let sent = stream.join().expect("publisher");
    // ANTI-VACUITY, bounded by the RECORDER'S OWN condition rather than by a
    // rate this runner had to achieve.
    //
    // This asked for `sent > 100`, which is a THROUGHPUT claim wearing a
    // fixture's clothes: `publish_until` commits one frame per 5 ms grid slot, so
    // 100 frames demands that the publisher thread got the CPU across half a
    // second of slots before the status feed served a rate. The grid publishes
    // the slots a late wake overran, but only once the thread runs again, so a
    // host that keeps the thread off the CPU still holds the count below the
    // slots elapsed at the instant the stop flag flips, while the recorder
    // behaves perfectly. That is exactly the load-inverted class: a bound whose
    // failing side is the healthy behaviour of a slow machine.
    //
    // What the guard is FOR is that the topic genuinely streamed before it went
    // quiet — and the recorder has already said so, far better than a frame
    // count can. `await_status_verdict` returned only once the recorder served
    // `rate_mhz`, which `RateWindow::estimate` mints only from a window spanning
    // at least `ABSORBANCE_MIN_RATE_SPAN` (1 s) of CONTINUOUSLY DRAINED frames
    // with no gap reaching `ABSORBANCE_MAX_IDLE_GAP`; the precondition above then
    // rejected a `NoClaim`. So the floor that claim implies is TWO committed
    // frames a second apart, and that is what is asserted here — a bound load can
    // only DELAY, never invert, and which no run reaching this line can be under.
    assert!(
        sent >= 2,
        "the recorder served a rate, which it mints only by differencing two \
         drained frames at least {:?} apart — so the fixture cannot have published \
         fewer than two: {sent}",
        cerulion_bagd::absorbance::ABSORBANCE_MIN_RATE_SPAN
    );

    // (3) THE LIVE SURFACE: the row must move to NoClaim on its own, with no
    //     further frames — the whole point, since nothing calls `observe` again.
    let quiet = await_status_verdict(&mut status, &topic, |v| {
        v.verdict == AbsorbanceVerdict::NoClaim
    });
    assert_eq!(
        quiet.rate_mhz, None,
        "a stopped stream DROPS its rate: {quiet:?}"
    );
    assert_eq!(
        quiet.measured_tail_us, None,
        "…and compares nothing: {quiet:?}"
    );

    // (4) THE DURABLE SURFACE: finalize AFTER the transition, and the health
    //     document must carry the same answer rather than the live one.
    shutdown.store(true, Ordering::Relaxed);
    let summary = join_bagd(handle, &shutdown, "the quiet-transition arm").expect("recorder");
    let final_row = row(&summary, &topic);
    assert_eq!(
        final_row.verdict,
        AbsorbanceVerdict::NoClaim,
        "the FINAL record_health.json must not carry a verdict about a topic that \
         stopped publishing: {final_row:?}"
    );
    assert_eq!(final_row.rate_mhz, None, "{final_row:?}");

    cleanup_artifacts(&out, &dir, &ready);
}

/// A histogram recorded through `DrainGapHistogram::record`: `in_ladder` gaps of
/// 8 ms, which the ladder files under its 10 ms edge, and `overflow` gaps of
/// 300 ms, past its 250 ms top edge.
fn seeded_gaps(in_ladder: u64, overflow: u64) -> DrainGapHistogram {
    let mut gaps = DrainGapHistogram::new();
    for _ in 0..in_ladder {
        gaps.record(Duration::from_millis(8));
    }
    for _ in 0..overflow {
        gaps.record(Duration::from_millis(300));
    }
    gaps
}

/// The 0.999 reading of a [`seeded_gaps`] histogram whose reading sits inside
/// the ladder: the 10 ms edge its 8 ms gaps fall under.
const SEEDED_TAIL_US: u64 = 10_000;

/// A row for a depth-[`DEPTH`] tap committing at `rate_mhz` against an in-ladder
/// tail of `tail_us`, written out by hand from the verdict contract: the
/// absorbance is `depth / rate`, the required depth `ceil(rate * tail)`, and the
/// tap is `Short` by `tail - absorbance` when it holds less than the tail.
fn seeded_row(rate_mhz: u64, tail_us: u64) -> TopicAbsorbance {
    let absorbance_us = DEPTH * 1_000_000_000 / rate_mhz;
    let required_depth =
        u64::try_from((u128::from(rate_mhz) * u128::from(tail_us)).div_ceil(1_000_000_000))
            .expect("a plausible depth");
    let short = absorbance_us < tail_us;
    TopicAbsorbance {
        tap_buffer_depth: DEPTH,
        rate_mhz: Some(rate_mhz),
        rate_is_floor: false,
        absorbance_us: Some(absorbance_us),
        measured_tail_us: Some(tail_us),
        required_depth: Some(required_depth),
        shortfall_at_least_us: if short {
            Some(tail_us - absorbance_us)
        } else {
            None
        },
        verdict: if short {
            AbsorbanceVerdict::Short
        } else {
            AbsorbanceVerdict::Absorbs
        },
        short_evaluations: if short { 3 } else { 0 },
        // A window-only tap is budgeted, and the fixture's taps sit inside it.
        budget_bytes: Some(BUDGET),
        over_budget: None,
        budget_unpriced: false,
    }
}

/// One `/bagd/status` frame carrying `fast` and `slow` beside the counting basis
/// the recorder serves.
fn live_frame(fast: &str, slow: &str) -> LiveFrame {
    (
        fast.to_string(),
        slow.to_string(),
        "prefix_invisible".to_string(),
    )
}

/// A probative headline run over `gaps`: both publishers on their grids for 4 s,
/// rows that agree with the gaps' in-ladder tail, and a live feed that converged.
fn seeded_headline(gaps: DrainGapHistogram) -> HeadlineRun {
    HeadlineRun {
        gaps,
        fast_row: seeded_row(400_000, SEEDED_TAIL_US),
        slow_row: seeded_row(5_000, SEEDED_TAIL_US),
        fast_sent: 1_600,
        slow_sent: 20,
        wall: Duration::from_secs(4),
        live: Ok(live_frame("short", "absorbs")),
        unpriced_taps: 0,
        shm_pinned_bytes: 2 * DEPTH * SLOT,
    }
}

/// A probative quiet run over `gaps`: a 5 Hz publisher on its grid for 2.6 s, an
/// absorbing row and a capture holding only the arm's own probe.
fn seeded_quiet(gaps: DrainGapHistogram) -> QuietRun {
    QuietRun {
        gaps,
        row: seeded_row(5_000, SEEDED_TAIL_US),
        sent: 13,
        published_for: Duration::from_millis(2_600),
        lines: vec![format!("WARN seeded: {QUIET_PROBE}")],
    }
}

/// The topic a seeded loud run names.
const SEEDED_TOPIC: &str = "/seeded/absorb_loud";

/// The lines a loud run captures, in the recorder's shape: one WARN head
/// carrying `head`'s fields, two suppressed DEBUG repeats, and the WARN terminal
/// roll-up naming [`SEEDED_TOPIC`].
fn seeded_loud_lines(head: &TopicAbsorbance) -> Vec<String> {
    let head = format!(
        "WARN seeded: tap CANNOT ABSORB this run's own drain stalls: the rate is a WINDOW \
         AVERAGE, the gaps are DRIVE-LOOP gaps, and loss before the first drained frame is \
         invisible to frames_lost and prefix_lost, so a zero means nothing was COUNTED. \
         topic={SEEDED_TOPIC} tap_buffer_depth={} depth_mode=\"budgeted\" remedy=\"raise \
         CERULION_FLASHBACK_TAP_BUDGET_MB\" required_depth={:?} absorbance_us={:?} \
         measured_tail_us={:?} shortfall_at_least_us={:?} rate_mhz={:?} \
         loss_counting_basis=\"prefix_invisible\"",
        head.tap_buffer_depth,
        head.required_depth,
        head.absorbance_us,
        head.measured_tail_us,
        head.shortfall_at_least_us,
        head.rate_mhz
    );
    let repeat = |n: u32| {
        format!(
            "DEBUG seeded: bagd tap still cannot absorb this run's drain stalls (suppressed \
             repeat) topic={SEEDED_TOPIC} suppressed={n}"
        )
    };
    let terminal = format!(
        "WARN seeded: recording carried taps that COULD NOT ABSORB this run's own drain \
         stalls absorbance_short_topics=[\"{SEEDED_TOPIC} [short]\"]"
    );
    vec![head, repeat(1), repeat(2), terminal]
}

/// A probative loud run over `gaps`: a 400 Hz publisher on its grid for 7 s, a
/// short row and a capture whose head carries that row's fields.
fn seeded_loud(gaps: DrainGapHistogram) -> LoudRun {
    let row = seeded_row(400_000, SEEDED_TAIL_US);
    LoudRun {
        topic: SEEDED_TOPIC.to_string(),
        gaps,
        lines: seeded_loud_lines(&row),
        row,
        sent: 2_800,
        published_for: Duration::from_secs(7),
    }
}

/// Three copies of `run`, one per attempt the driver can ask for.
fn three<R: Clone>(run: R) -> [R; 3] {
    [run.clone(), run.clone(), run]
}

/// How [`rerun_into`] ended when it judged seeded runs.
struct Driven {
    /// The attempts the driver asked for.
    calls: u32,
    /// The lines it wrote for non-probative attempts.
    printed: String,
    /// The panic it ended in, if it ended in one.
    panic: Option<String>,
}

/// Drive `judge` through [`rerun_into`], handing attempt `n` the run `runs[n - 1]`.
fn drive<R>(what: &str, runs: &[R], judge: fn(u32, &R) -> Attempt) -> Driven {
    let mut calls = 0u32;
    let mut sink = Vec::new();
    let ended = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        rerun_into(what, &mut sink, |n| {
            calls += 1;
            judge(n, &runs[n as usize - 1])
        });
    }));
    let panic = ended.err().map(|payload| {
        payload
            .downcast_ref::<String>()
            .cloned()
            .or_else(|| payload.downcast_ref::<&str>().map(|s| (*s).to_string()))
            .unwrap_or_else(|| "a panic with no message".to_string())
    });
    Driven {
        calls,
        printed: String::from_utf8(sink).expect("the driver writes text"),
        panic,
    }
}

/// `runs[0]` is probative and passes: the driver asks for one attempt and
/// prints nothing.
fn passes_on_attempt_one<R>(what: &str, runs: &[R], judge: fn(u32, &R) -> Attempt) {
    let driven = drive(what, runs, judge);
    assert!(
        driven.panic.is_none() && driven.calls == 1 && driven.printed.is_empty(),
        "{what}: the unseeded control must pass on attempt 1 with nothing printed; it took \
         {} attempt(s), printed {:?} and ended in {:?}",
        driven.calls,
        driven.printed,
        driven.panic
    );
}

/// `runs[0]` is probative and carries a seeded regression: the driver asks for
/// exactly one attempt, which panics with `expect` rather than as non-probative.
fn fails_on_attempt_one<R>(what: &str, runs: &[R], judge: fn(u32, &R) -> Attempt, expect: &str) {
    let driven = drive(what, runs, judge);
    assert_eq!(
        driven.calls, 1,
        "{what}: a probative attempt's failure must be final, yet the driver asked for {} \
         attempt(s), printed {:?} and ended in {:?}",
        driven.calls, driven.printed, driven.panic
    );
    let panic = driven
        .panic
        .unwrap_or_else(|| panic!("{what}: the seeded regression passed"));
    assert!(
        panic.contains(expect) && !panic.contains("NON-PROBATIVE") && driven.printed.is_empty(),
        "{what}: the seeded regression must fail its own assertion (`{expect}`) on attempt 1, \
         not as non-probative; it printed {:?} and ended in: {panic}",
        driven.printed
    );
}

/// What one non-probative attempt's line must and must not carry.
type LineExpect<'a> = (&'a [&'a str], &'a [&'a str]);

/// Every run in `runs` is non-probative: the driver asks for all three attempts,
/// prints each one's line with its evidence, and fails as NON-PROBATIVE naming
/// all three. Line `n` must carry every string in `expect[n - 1].0` and none in
/// `expect[n - 1].1`.
fn fails_non_probative<R>(
    what: &str,
    runs: &[R; 3],
    judge: fn(u32, &R) -> Attempt,
    expect: [LineExpect<'_>; 3],
) {
    let driven = drive(what, runs, judge);
    assert_eq!(
        driven.calls, ATTEMPTS,
        "{what}: every non-probative attempt earns another, up to {ATTEMPTS}; the driver \
         asked for {} and ended in {:?}",
        driven.calls, driven.panic
    );
    let panic = driven
        .panic
        .unwrap_or_else(|| panic!("{what}: three non-probative attempts must FAIL the arm"));
    assert!(
        panic.starts_with(&format!("NON-PROBATIVE: all {ATTEMPTS} attempts of {what}")),
        "{what}: the failure must say NON-PROBATIVE and name the arm: {panic}"
    );
    let lines: Vec<&str> = driven.printed.lines().collect();
    assert_eq!(
        lines.len(),
        3,
        "{what}: every attempt prints its line: {lines:?}"
    );
    for (n, (line, (must, must_not))) in (1..).zip(lines.iter().zip(expect)) {
        assert!(
            line.starts_with(&format!(
                "{what}: attempt {n} of {ATTEMPTS} was NON-PROBATIVE: "
            )),
            "{what}: line {n} must name its attempt: {line}"
        );
        assert!(
            panic.contains(line),
            "{what}: the failure must name attempt {n} with its evidence: {panic}"
        );
        for needle in ["gaps per bucket", "lands in the", "achieved ~", "measured "]
            .iter()
            .chain(must)
        {
            assert!(
                line.contains(needle),
                "{what}: attempt {n} must print `{needle}`: {line}"
            );
        }
        for needle in must_not {
            assert!(
                !line.contains(needle),
                "{what}: attempt {n} must not print `{needle}`: {line}"
            );
        }
    }
}

/// A PROBATIVE attempt's failure is final. Each seeded regression fails on its
/// first attempt with no second one asked for.
///
/// The headline and quiet seeds ride a run whose recorded tail sits inside the
/// ladder beside one stray gap in the overflow bucket: 2000 recorded gaps put
/// the 0.999 rank at 1998, inside the 1999 gaps the ladder holds, so the stray
/// 300 ms gap does not hold the tail and both arms can check their verdicts.
/// The loud arm also checks its mid-run head ([`TailRead::MidRun`]), which a
/// stray gap leaves unknowable, so its seeds ride 2000 gaps all inside the
/// ladder. Each arm's unseeded control on the same histogram passes on attempt
/// 1, so each failure below is its seed's.
#[test]
fn a_seeded_regression_on_a_probative_attempt_fails_on_attempt_one_with_no_rerun() {
    let gaps = || seeded_gaps(1_999, 1);
    let walk = walk_tail(&gaps().edges_us, &gaps().counts).expect("recorded gaps");
    assert_eq!(
        walk,
        TailWalk {
            total: 2_000,
            rank: 1_998,
            overflow: 1
        }
    );
    assert!(
        !walk.overflowed(),
        "the seeds must ride an in-ladder tail: {walk:?}"
    );

    // THE HEADLINE.
    passes_on_attempt_one(
        "the seeded headline",
        &[seeded_headline(gaps())],
        judge_headline,
    );
    // The slow control answers `Unrankable` at finalize, the live feed converged.
    let mut at_finalize = seeded_headline(gaps());
    at_finalize.slow_row.verdict = AbsorbanceVerdict::Unrankable;
    fails_on_attempt_one(
        "the seeded headline",
        &three(at_finalize),
        judge_headline,
        "a verdict that cannot tell them apart is not a verdict",
    );
    // The slow control answers `Unrankable` on both surfaces, so the live wait
    // timed out on `slow=unrankable` beside `fast=short`.
    let mut on_both = seeded_headline(gaps());
    on_both.slow_row.verdict = AbsorbanceVerdict::Unrankable;
    on_both.live = Err(Some(live_frame("short", "unrankable")));
    fails_on_attempt_one(
        "the seeded headline",
        &three(on_both),
        judge_headline,
        "no /bagd/status frame converged",
    );
    // THE LIVE-WAIT TIMEOUT: the final rows are right and the live feed never
    // converged.
    let mut live_only = seeded_headline(gaps());
    live_only.live = Err(Some(live_frame("short", "unrankable")));
    fails_on_attempt_one(
        "the seeded headline",
        &three(live_only),
        judge_headline,
        "no /bagd/status frame converged",
    );

    // THE QUIET ARM: the tap answers `Short` against a tail it covers.
    passes_on_attempt_one("the seeded quiet arm", &[seeded_quiet(gaps())], judge_quiet);
    let mut short = seeded_quiet(gaps());
    short.row.verdict = AbsorbanceVerdict::Short;
    short.row.shortfall_at_least_us = Some(1);
    fails_on_attempt_one(
        "the seeded quiet arm",
        &three(short),
        judge_quiet,
        "an unstalled recorder's 5 Hz tap must absorb its own gaps",
    );

    // THE LOUD ARM, on a run with no gap past the ladder.
    let in_ladder = || seeded_gaps(2_000, 0);
    passes_on_attempt_one(
        "the seeded loud arm",
        &[seeded_loud(in_ladder())],
        judge_loud,
    );
    // The final row answers `Unrankable` against an in-ladder tail.
    let mut unrankable = seeded_loud(in_ladder());
    unrankable.row.verdict = AbsorbanceVerdict::Unrankable;
    fails_on_attempt_one(
        "the seeded loud arm",
        &three(unrankable),
        judge_loud,
        "left: Unrankable",
    );
    // The head drops the in-ladder tail it was evaluated against.
    let mut no_tail = seeded_loud(in_ladder());
    no_tail.lines = seeded_loud_lines(&tailless(&no_tail.row));
    fails_on_attempt_one(
        "the seeded loud arm",
        &three(no_tail),
        judge_loud,
        "`required_depth` must carry a MEASURED value",
    );
}

/// `row` as the recorder answers it against an OVERFLOW tail: no measured tail
/// and no required depth.
fn tailless(row: &TopicAbsorbance) -> TopicAbsorbance {
    TopicAbsorbance {
        measured_tail_us: None,
        required_depth: None,
        ..*row
    }
}

/// A stray gap past the ladder makes the loud arm's attempt NON-PROBATIVE even
/// when the final 0.999 reading comes back inside the ladder: its head may have
/// been minted while that gap held the rank, which leaves the head without a
/// tail or a required depth by contract. The arm runs again and passes on a
/// clean attempt.
///
/// The seeded attempt 1 is what a single host stall before the head does to a
/// run past 1000 gaps: 1999 gaps in the ladder, one past it, a tailless head and
/// a final row that carries the in-ladder tail.
#[test]
fn a_stray_overflow_gap_reruns_the_loud_arm_whose_head_may_have_read_it() {
    let mut stalled = seeded_loud(seeded_gaps(1_999, 1));
    stalled.lines = seeded_loud_lines(&tailless(&stalled.row));
    let clean = seeded_loud(seeded_gaps(2_000, 0));
    let driven = drive(
        "the seeded loud arm",
        &[stalled, clean.clone(), clean],
        judge_loud,
    );
    assert!(
        driven.panic.is_none() && driven.calls == 2,
        "a tailless head beside a stray overflow gap must re-run the loud arm, not fail \
         it: {} attempt(s), printed {:?}, ended in {:?}",
        driven.calls,
        driven.printed,
        driven.panic
    );
    let lines: Vec<&str> = driven.printed.lines().collect();
    assert!(
        lines.len() == 1
            && lines[0].starts_with("the seeded loud arm: attempt 1 of 3 was NON-PROBATIVE: ")
            && lines[0].contains("rank 1998 of 2000 gaps) is inside the ladder")
            && lines[0].contains("minted mid-run")
            && !lines[0].contains("unbounded overflow bucket"),
        "attempt 1 names the stray gap beside an in-ladder reading: {lines:?}"
    );
}

/// Three NON-PROBATIVE attempts fail the arm loudly, naming all three, and every
/// attempt prints its evidence: the gap histogram with where its 0.999 reading
/// lands, each tap's commanded, measured and achieved rates, and its number.
///
/// Per arm, attempt 1 is an overflow tail alone, attempt 2 a publisher off its
/// band alone over an in-ladder tail, and attempt 3 a publisher off its band
/// beside an overflow tail (for the loud arm, beside a stray overflow gap whose
/// final reading is inside the ladder), so each line names its own reasons and
/// only those.
#[test]
fn three_non_probative_attempts_fail_the_arm_naming_all_three() {
    const OVERFLOW: &str = "unbounded overflow bucket";
    const MID_RUN: &str = "minted mid-run";
    const BEHIND: &str = "fell behind its grid";

    let mut slow_behind = seeded_headline(seeded_gaps(297, 0));
    slow_behind.slow_sent = 8;
    let mut fast_behind = seeded_headline(seeded_gaps(296, 1));
    fast_behind.fast_sent = 800;
    fails_non_probative(
        "the seeded headline",
        &[
            seeded_headline(seeded_gaps(296, 1)),
            slow_behind,
            fast_behind,
        ],
        judge_headline,
        [
            (&[OVERFLOW, "rank 297 of 297"], &[BEHIND]),
            (
                &["the slow control committed at ~2000 mHz", BEHIND],
                &[OVERFLOW],
            ),
            (
                &[OVERFLOW, "the fast tap committed at ~200000 mHz", BEHIND],
                &["the slow control committed"],
            ),
        ],
    );

    let mut quiet_behind = seeded_quiet(seeded_gaps(79, 0));
    quiet_behind.sent = 6;
    let mut quiet_both = seeded_quiet(seeded_gaps(78, 1));
    quiet_both.sent = 6;
    fails_non_probative(
        "the seeded quiet arm",
        &[seeded_quiet(seeded_gaps(78, 1)), quiet_behind, quiet_both],
        judge_quiet,
        [
            (&[OVERFLOW, "rank 79 of 79"], &[BEHIND]),
            (
                &["the quiet arm's tap committed at ~2307 mHz", BEHIND],
                &[OVERFLOW],
            ),
            (&[OVERFLOW, BEHIND], &[]),
        ],
    );

    let mut loud_behind = seeded_loud(seeded_gaps(427, 0));
    loud_behind.sent = 700;
    let mut loud_both = seeded_loud(seeded_gaps(1_999, 1));
    loud_both.lines = seeded_loud_lines(&tailless(&loud_both.row));
    loud_both.sent = 700;
    fails_non_probative(
        "the seeded loud arm",
        &[seeded_loud(seeded_gaps(426, 1)), loud_behind, loud_both],
        judge_loud,
        [
            (&[OVERFLOW, "rank 427 of 427"], &[BEHIND, MID_RUN]),
            (
                &["the loud arm's tap committed at ~100000 mHz", BEHIND],
                &[OVERFLOW, MID_RUN],
            ),
            (&[MID_RUN, "rank 1998 of 2000", BEHIND], &[OVERFLOW]),
        ],
    );
}

/// The driver asks for another attempt only after a non-probative one, stops at
/// the first probative attempt, and keeps the earlier attempt's line.
#[test]
fn the_rerun_driver_stops_at_the_first_probative_attempt() {
    let driven = drive(
        "the seeded quiet arm",
        &[
            seeded_quiet(seeded_gaps(78, 1)),
            seeded_quiet(seeded_gaps(79, 0)),
            seeded_quiet(seeded_gaps(79, 0)),
        ],
        judge_quiet,
    );
    assert!(
        driven.panic.is_none() && driven.calls == 2,
        "a probative attempt 2 ends the arm there: {} attempt(s), ended in {:?}",
        driven.calls,
        driven.panic
    );
    let lines: Vec<&str> = driven.printed.lines().collect();
    assert!(
        lines.len() == 1
            && lines[0].starts_with("the seeded quiet arm: attempt 1 of 3 was NON-PROBATIVE: "),
        "only attempt 1 was non-probative, and its line is kept: {lines:?}"
    );
}

/// A tap for the classification tests: `sent` frames of a `period_ms`
/// publisher over `published_for_ms`.
fn tap(what: &'static str, period_ms: u64, sent: u32, published_for_ms: u64) -> TapRun {
    TapRun {
        what,
        period: Duration::from_millis(period_ms),
        sent,
        published_for: Duration::from_millis(published_for_ms),
    }
}

/// A 5 Hz tap that committed on its grid.
const ON_GRID: TapRun = TapRun {
    what: "the on-grid tap",
    period: Duration::from_millis(200),
    sent: 20,
    published_for: Duration::from_secs(4),
};

/// Both ways an arm reads the tail.
const READS: [TailRead; 2] = [TailRead::AtFinalize, TailRead::MidRun];

/// `classify_attempt` over `gaps`, `read` and `taps`.
fn classify(gaps: &DrainGapHistogram, read: TailRead, taps: &[TapRun]) -> Classification {
    classify_attempt(&AttemptEvidence::new(gaps, read, taps))
}

/// `(in_ladder, rank)` for a histogram of `in_ladder` gaps beside one stray
/// overflow gap whose 0.999 reading, at `rank`, the ladder holds: at exactly
/// 1000 gaps rank 999 of the 999 in-ladder gaps, and at 2000 gaps rank 1998 of
/// 1999.
const STRAY: [(u64, u64); 2] = [(999, 999), (1_999, 1_998)];

/// The 0.999 reading inside the ladder, or no reading at all, is PROBATIVE for
/// either read when nothing sits past the ladder, and for an arm that reads at
/// finalize when a stray gap sits in the overflow bucket without holding the
/// rank.
#[test]
fn classify_attempt_finds_an_in_ladder_or_vacant_tail_probative() {
    // Armed and vacant, and never armed: no reading to land anywhere.
    assert_eq!(
        walk_tail(
            &DrainGapHistogram::new().edges_us,
            &DrainGapHistogram::new().counts
        ),
        None
    );
    for read in READS {
        for gaps in [DrainGapHistogram::new(), DrainGapHistogram::default()] {
            assert_eq!(
                classify(&gaps, read, &[ON_GRID]),
                Classification::Probative,
                "{read:?} {gaps:?}"
            );
        }
        // In the ladder, with nothing past it.
        for in_ladder in [297, 2_000] {
            assert_eq!(
                classify(&seeded_gaps(in_ladder, 0), read, &[ON_GRID]),
                Classification::Probative,
                "{read:?} {in_ladder}"
            );
        }
    }
    for (in_ladder, rank) in STRAY {
        let gaps = seeded_gaps(in_ladder, 1);
        let walk = walk_tail(&gaps.edges_us, &gaps.counts).expect("recorded gaps");
        assert_eq!(
            walk,
            TailWalk {
                total: in_ladder + 1,
                rank,
                overflow: 1
            }
        );
        assert!(!walk.overflowed(), "{walk:?}");
        assert_eq!(
            classify(&gaps, TailRead::AtFinalize, &[ON_GRID]),
            Classification::Probative
        );
    }
}

/// An arm that checks a mid-run line is NON-PROBATIVE beside any gap past the
/// ladder, even one that does not hold the final rank, and the reason carries
/// the walk.
#[test]
fn classify_attempt_finds_a_stray_overflow_gap_non_probative_for_a_mid_run_line() {
    for (in_ladder, rank) in STRAY {
        let walk = TailWalk {
            total: in_ladder + 1,
            rank,
            overflow: 1,
        };
        assert!(!walk.overflowed(), "{walk:?}");
        assert_eq!(
            classify(&seeded_gaps(in_ladder, 1), TailRead::MidRun, &[ON_GRID]),
            Classification::NonProbative(vec![NonProbativeReason::OverflowGapMidRun(walk)]),
            "{in_ladder} in the ladder, 1 past it"
        );
    }
    // Two past it at 2500 gaps: rank 2498, held by the 2498 in-ladder gaps.
    let walk = TailWalk {
        total: 2_500,
        rank: 2_498,
        overflow: 2,
    };
    assert!(!walk.overflowed(), "{walk:?}");
    assert_eq!(
        classify(&seeded_gaps(2_498, 2), TailRead::MidRun, &[ON_GRID]),
        Classification::NonProbative(vec![NonProbativeReason::OverflowGapMidRun(walk)])
    );
}

/// The 0.999 reading in the overflow bucket is NON-PROBATIVE for either read,
/// with the one reason that carries the walk that put it there.
#[test]
fn classify_attempt_finds_an_overflow_tail_non_probative() {
    for read in READS {
        for (in_ladder, overflow, rank) in [(296, 1, 297), (998, 2, 999), (0, 1, 1), (0, 3, 3)] {
            let walk = TailWalk {
                total: in_ladder + overflow,
                rank,
                overflow,
            };
            assert!(walk.overflowed(), "{walk:?}");
            assert_eq!(
                classify(&seeded_gaps(in_ladder, overflow), read, &[ON_GRID]),
                Classification::NonProbative(vec![NonProbativeReason::OverflowTail(walk)]),
                "{read:?}: {in_ladder} in the ladder, {overflow} past it"
            );
        }
    }
}

/// A MALFORMED histogram, whose counts are not one per edge plus the overflow
/// bucket, has no overflow bucket to read, so it never makes an attempt
/// non-probative: the arms' own assertions judge it.
#[test]
fn classify_attempt_never_reads_a_malformed_histogram_as_an_overflow_tail() {
    for (edges_us, counts) in [
        (vec![100, 250], vec![0, 9]),
        (vec![100], vec![0, 0, 9]),
        (vec![], vec![9, 9]),
    ] {
        assert_eq!(
            walk_tail(&edges_us, &counts),
            None,
            "{edges_us:?} {counts:?}"
        );
        let gaps = DrainGapHistogram { edges_us, counts };
        for read in READS {
            assert_eq!(
                classify(&gaps, read, &[ON_GRID]),
                Classification::Probative,
                "{read:?} {gaps:?}"
            );
        }
    }
}

/// Each priced publisher's ACHIEVED rate is held to the band of its commanded
/// rate, both ends included, and only the publishers off it are named.
#[test]
fn classify_attempt_holds_each_publisher_to_its_band() {
    let in_ladder = seeded_gaps(297, 0);
    // 5 Hz commanded is 5000 mHz, so the band is [3334, 7500] mHz in whole
    // millihertz; 1000 s of publishing makes `sent` the achieved rate in mHz.
    for sent in [3_334, 5_000, 7_500] {
        assert_eq!(
            classify(
                &in_ladder,
                TailRead::AtFinalize,
                &[tap("the edge tap", 200, sent, 1_000_000)]
            ),
            Classification::Probative,
            "{sent} mHz is inside the band"
        );
    }
    for (sent, way) in [
        (3_333, "fell behind"),
        (0, "fell behind"),
        (7_501, "ran ahead of"),
    ] {
        let off = NonProbativeReason::PublisherOffBand {
            what: "the edge tap",
            commanded_mhz: 5_000,
            achieved_mhz: u64::from(sent),
        };
        assert!(off.describe().contains(way), "{}", off.describe());
        for read in READS {
            assert_eq!(
                classify(
                    &in_ladder,
                    read,
                    &[ON_GRID, tap("the edge tap", 200, sent, 1_000_000)]
                ),
                Classification::NonProbative(vec![off.clone()]),
                "{read:?}: {sent} mHz is outside the band, and the on-grid tap is not named"
            );
        }
    }
    // Both reasons at once: the tail's reason first, then each tap off its band.
    let behind = tap("the behind tap", 200, 8, 4_000);
    let off = NonProbativeReason::PublisherOffBand {
        what: "the behind tap",
        commanded_mhz: 5_000,
        achieved_mhz: 2_000,
    };
    assert_eq!(
        classify(
            &seeded_gaps(296, 1),
            TailRead::AtFinalize,
            &[behind, ON_GRID]
        ),
        Classification::NonProbative(vec![
            NonProbativeReason::OverflowTail(TailWalk {
                total: 297,
                rank: 297,
                overflow: 1
            }),
            off.clone(),
        ])
    );
    assert_eq!(
        classify(&seeded_gaps(1_999, 1), TailRead::MidRun, &[behind, ON_GRID]),
        Classification::NonProbative(vec![
            NonProbativeReason::OverflowGapMidRun(TailWalk {
                total: 2_000,
                rank: 1_998,
                overflow: 1
            }),
            off,
        ])
    );
}

/// This file's walk and the recorder's `quantile_us` agree on which readings land
/// in the overflow bucket for every well-formed histogram up to 2100 gaps with up
/// to three of them past the ladder, so a run the recorder ranked inside the
/// ladder is never classified as an overflow tail, and the reverse.
#[test]
fn the_tail_walk_agrees_with_the_recorders_quantile() {
    for total in 1..=2_100u64 {
        for overflow in 0..=total.min(3) {
            let mut gaps = DrainGapHistogram::new();
            let top = gaps.edges_us.len();
            gaps.counts[6] = total - overflow;
            gaps.counts[top] = overflow;
            let walk = walk_tail(&gaps.edges_us, &gaps.counts).expect("recorded gaps");
            assert_eq!(
                walk.overflowed(),
                gaps.quantile_us(ABSORBANCE_VERDICT_QUANTILE).is_none(),
                "{walk:?} against {gaps:?}"
            );
        }
    }
}
