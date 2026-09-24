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

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_bagd::{
    run_bagd, AbsorbanceVerdict, BagdConfig, BagdSummary, FlashbackSettings, TapSpec,
    TopicAbsorbance,
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

/// The SLOW topic's publish period. 100 ms = 10 Hz, so `DEPTH / rate` is 200 ms
/// — comfortably OVER the same tail. The control arm.
///
/// The margin is deliberate rather than tuned to the observed tail: the stall
/// paces the drive loop at `WAIT_TICK` (10 ms), which lands in the 25 ms rung,
/// and a loaded runner can push it to the 50 ms one. 200 ms clears both by a
/// factor of four, so the control cannot flip on load — and the arms assert
/// against the row's OWN reported tail rather than against a constant, so a
/// runner that stalls harder still fails loudly instead of silently.
const SLOW_PERIOD: Duration = Duration::from_millis(100);

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

/// Publish `body` on `pubr` every `period` until `stop` flips, returning how
/// many frames really went out (the arm's own denominator sanity check).
fn publish_until(
    pubr: &mut cerulion_core::transport::publisher::CerulionPublisher,
    period: Duration,
    stop: &AtomicBool,
    body: &[u8],
) -> u32 {
    let mut seq = 0u32;
    let mut sent = 0u32;
    while !stop.load(Ordering::Relaxed) {
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
        std::thread::sleep(period);
    }
    sent
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

    // PRECONDITION, asserted rather than assumed: the two publishers really did
    // run at different rates over the same window.
    assert!(
        fast_sent > slow_sent * 4,
        "the fixture must really separate the two rates: fast={fast_sent} slow={slow_sent}"
    );

    let fast_row = row(&summary, &fast_topic);
    let slow_row = row(&summary, &slow_topic);

    // PRECONDITION: the stall really was measured. Both rows read the SAME
    // recorder-wide histogram, so one assertion covers both.
    //
    // And the STATISTIC is established rather than assumed. `quantile_us` takes
    // `rank = ceil(q * total)`, and for q = 0.999 that is `total` itself for
    // every total under 1000 — so on a short run the "tail" IS the run maximum,
    // and one preempted pass moves it. This fixture's run is ~3 s, i.e. a few
    // hundred passes, so it is squarely in that regime: the arms below are up
    // against the worst pass of the run, and they say so.
    let gaps = &summary.record_health.drain_gaps;
    let total_gaps = gaps.total();
    assert!(
        total_gaps > 0,
        "the recorder must have measured its own cadence, or there is no tail          to be short of"
    );
    // NON-PROBATIVE ESCAPE (the runner-stall pattern): a runner stalled past the
    // ladder's top edge makes the comparison unrankable rather than wrong, and
    // an unrankable run must fail as an ATTRIBUTABLE stall rather than as a
    // regression in the code under test.
    let Some(tail) = fast_row.measured_tail_us else {
        panic!(
            "NON-PROBATIVE: this run's 0.999 drain-gap reading fell in the              histogram's unbounded overflow bucket (> 250 ms), so neither arm              can be ranked. That is a stalled runner, not a verdict regression              — {total_gaps} gaps measured, fast={fast_row:?}"
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
    // The SLOW control is only meaningful while its absorbance clears the tail
    // with room; if a runner stalled hard enough to close that gap, this arm
    // proves nothing and says so rather than reporting a regression.
    let slow_abs_pre = slow_row
        .absorbance_us
        .expect("the slow row has a rate and a depth");
    assert!(
        slow_abs_pre > tail,
        "NON-PROBATIVE: the control tap's absorbance ({slow_abs_pre} us) no \
         longer clears this run's measured tail ({tail} us) — a stalled runner, \
         not a verdict regression"
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
        summary.unpriced_taps, 0,
        "both taps are budgeted, so both price themselves"
    );
    assert!(
        summary.shm_pinned_bytes <= 2 * DEPTH * SLOT,
        "the pinned bound must not exceed what two depth-{DEPTH} queues could hold: {} > {}",
        summary.shm_pinned_bytes,
        2 * DEPTH * SLOT
    );

    cleanup_artifacts(&out, &dir, &ready);
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
        stim_shutdown.store(true, Ordering::Relaxed);
        sent
    });

    let summary = run_bagd(mgr.clone(), cfg, shutdown.clone()).expect("clean finalize");
    let sent = stimulus.join().expect("stimulus");
    assert!(sent > 100, "the fixture must really publish: {sent}");

    // PRECONDITION: the run really did end SHORT, so the log assertions below
    // are about a condition that occurred.
    let a = row(&summary, &topic);
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

    logs_assert(|lines: &[&str]| {
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
        for key in [
            "required_depth",
            "absorbance_us",
            "measured_tail_us",
            "shortfall_at_least_us",
            "rate_mhz",
        ] {
            let Some(tok) = field_token(head, key) else {
                return Err(format!("the head must carry `{key}=`: {head}"));
            };
            if !tok.starts_with("Some(") || tok == "Some(0)" {
                return Err(format!(
                    "`{key}` must carry a MEASURED value, not an unknown and not a \
                     fabricated zero; got `{tok}`: {head}"
                ));
            }
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
    });

    cleanup_artifacts(&out, &dir, &ready);
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
        let pump = std::thread::spawn(move || {
            // SLOW only, and no stall: a depth-2 queue at 10 Hz holds 200 ms,
            // which covers anything an unblocked drive loop can produce.
            publish_until(&mut pubr, SLOW_PERIOD, &stim_stop, &body)
        });
        std::thread::sleep(Duration::from_millis(2_500));
        stop.store(true, Ordering::Relaxed);
        let sent = pump.join().expect("publisher");
        stim_shutdown.store(true, Ordering::Relaxed);
        sent
    });

    let summary = run_bagd(mgr.clone(), cfg, shutdown.clone()).expect("clean finalize");
    let sent = stimulus.join().expect("stimulus");
    assert!(sent > 10, "the fixture must really publish: {sent}");
    // The positive control for the absence guard below — see there.
    tracing::warn!("{QUIET_PROBE}");

    // The row EXISTS and it ABSORBS — the half that makes the silence meaningful
    // rather than a recorder that evaluated nothing.
    let a = row(&summary, &topic);
    assert!(
        a.rate_mhz.is_some(),
        "the tap must have measured a rate, or its silence proves nothing: {a:?}"
    );
    // NON-PROBATIVE ESCAPE (the runner-stall pattern), and this arm needs one more
    // than its siblings do. The tail is `quantile_us(0.999)` and over a run of a
    // few hundred passes `rank = ceil(0.999 * total)` IS `total`, i.e. the run
    // MAXIMUM — so one preempted drive pass longer than this tap's own
    // absorbance flips the verdict to `Short`, and without an escape a loaded
    // runner reports that as a regression in the code under test.
    //
    // Read against the row's OWN numbers rather than a constant, so the escape
    // tracks whatever depth and rate this run actually achieved.
    if let (Some(tail), Some(absorbance)) = (a.measured_tail_us, a.absorbance_us) {
        assert!(
            tail < absorbance,
            "NON-PROBATIVE: this run's worst drive-loop pass ({tail} us) met or exceeded \
             what this unstalled tap can hold ({absorbance} us), so `Absorbs` is not the \
             correct answer for THIS run and the silence below is about nothing. That is a \
             stalled runner, not a verdict regression: {a:?}"
        );
    }
    assert_eq!(
        a.verdict,
        AbsorbanceVerdict::Absorbs,
        "an unstalled recorder's 10 Hz tap must absorb its own gaps: {a:?}"
    );

    logs_assert(|lines: &[&str]| {
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
    });

    cleanup_artifacts(&out, &dir, &ready);
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
/// the deadline and panic naming the last both-verdict frame seen — which
/// also separates a genuine regression from a runner stalled hard enough that
/// the measured tail exceeded the slow control's 200 ms absorbance.
///
/// The frames are drained in chunks of the DEFAULT borrow budget: an
/// `OwnedInboundSample` holds its SHM borrow until dropped, so asking for more
/// than the budget is an `ExceedsMaxBorrows` the moment a loaded runner queues a
/// backlog (the `flashback_tap_depth_e2e_test::drain_status` lesson).
fn await_status_verdicts(
    sub: &mut cerulion_core::transport::subscriber::DataOnlySubscriber,
    fast: &str,
    slow: &str,
) -> (String, String, String) {
    const BORROW_BUDGET: usize = 2;
    let mut found = None;
    let mut last_seen: Option<(String, String, String)> = None;
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
    found.unwrap_or_else(|| match last_seen {
        Some((f, s, b)) => panic!(
            "no /bagd/status frame converged to (fast=short, slow=absorbs) while the \
             run was live; the last both-verdict frame carried (fast={f}, slow={s}, \
             basis={b}). `slow=short` beside `fast=short` is NON-PROBATIVE (the \
             runner-stall pattern): a runner stalled hard enough that its worst drive \
             pass exceeded the control's 200 ms absorbance, not a verdict \
             regression. Anything else is the live surface failing to state the \
             absorbance boundary DURING the run"
        ),
        None => panic!(
            "no /bagd/status frame carried a verdict for both topics at all — the \
             live status feed is the one surface that states the absorbance boundary \
             DURING the run"
        ),
    })
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
    // fixture's clothes: `publish_until` sleeps 5 ms per frame, so 100 frames
    // demands the loop really ran at ~200 Hz for half a second before the status
    // feed served a rate. Load pushes an achieved publish rate only DOWN — and
    // on macOS CI's background QoS the timer slack is charged PER WAKEUP, so a
    // nominal 5 ms sleep lands near 29 ms — which makes it a gate a loaded
    // runner FAILS while behaving perfectly (macOS shard 2: 70 frames across the
    // same ~2 s window that yields ~400 on an idle desk, an effective ~35 Hz).
    // That is exactly the load-inverted class: a bound whose failing side is
    // the healthy behaviour of a slow machine.
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
