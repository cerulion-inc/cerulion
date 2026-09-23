// SPDX-License-Identifier: AGPL-3.0-only
//! Live-topic discovery is driven by FILESYSTEM EVENTS, not by a clock.
//!
//! # What these arms adjudicate
//!
//! Moving the service-directory walk off the drive loop stopped it costing
//! frames. It did not stop it happening: a settled machine still paid four full
//! `Service::list` walks a second - 117.5 ms each at 86 live topics, MEASURED
//! first-party on a robot's compute module - forever, to be told nothing had
//! changed.
//!
//! The fix is that the worker blocks on a kernel watch of the iceoryx2 service
//! directory (`inotify` / `kqueue`) and walks only when a file appears. The
//! property that matters is therefore a NEGATIVE one - *an idle recorder runs no
//! enumerations at all* - and it is the arm a polling implementation cannot
//! pass however low its cadence is set.
//!
//! The positive arms exist because that negative is trivially satisfiable by a
//! scanner that never enumerates ANYTHING, which would leave discovery inert:
//! the same defect the feature exists to fix, wearing the fix's clothes.
//!
//! # Why the assertions are counts and conditions, never walls
//!
//! A wall tight enough to separate "woke on an event" from "woke on a timer" on
//! this desk is also tight enough for a loaded runner to invert. So the
//! enumeration COUNT is the oracle throughout: load can delay an event-driven
//! walk but cannot manufacture one, and it can only push a polled walk rate
//! DOWN, so every bound here is violated by a design regression and not by a
//! busy machine.

mod common;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_bag::BagReader;
use cerulion_bagd::discovery_scan::{
    DiscoveryScanner, WakeSource, CONFIRM_WALKS, EVENT_CONFIRM_DELAY,
};
use cerulion_bagd::{
    run_bagd, BagdConfig, RecordCoverage, TapSource, TapSpec, RECORD_COVERAGE_ATTACHMENT,
};
use cerulion_core::transport::TransportManager;

use common::*;

/// An arbitrary, stable schema hash for the hand-built frames.
const HASH: u64 = 0x0996_0996_0996_0996;

/// A LIVENESS ceiling for every condition wait here - stated in seconds, never
/// in units of the thing under test.
const DEADLINE: Duration = Duration::from_secs(20);

/// How often a condition wait asks.
const POLL: Duration = Duration::from_millis(5);

/// The FALLBACK cadence the scanner arms drive at.
///
/// Short enough that the POLLING control runs a visible number of walks inside
/// the observation window, which is what makes the contrast below an
/// observation rather than a window too small to see anything.
const FALLBACK_CADENCE: Duration = Duration::from_millis(100);

/// How long an "and then nothing happened" window lasts.
///
/// Fifteen fallback cadences: the polling control runs roughly fifteen uncaused
/// walks inside it, while an event-driven engine runs only what it was woken
/// for - none at all on a machine nobody else is using.
const QUIET_WINDOW: Duration = Duration::from_millis(1500);

/// Block until `cond` holds, or fail LOUDLY with `what`.
fn await_condition(what: &str, mut cond: impl FnMut() -> bool) {
    let start = Instant::now();
    while start.elapsed() < DEADLINE {
        if cond() {
            return;
        }
        std::thread::sleep(POLL);
    }
    panic!("timed out after {DEADLINE:?} waiting for {what}");
}

/// Drive the scanner until its baseline enumeration and any tail behind it have
/// landed, then return the enumeration count that state is worth.
///
/// The baseline walk is unconditional - an event reports only a CHANGE, so
/// nothing already live when the watch was armed would be seen otherwise - so
/// every "how many walks did this cost?" assertion has to be relative to it.
fn settle_baseline(scanner: &mut DiscoveryScanner, mgr: &TransportManager) -> u64 {
    await_condition("the worker's baseline enumeration", || {
        scanner.next_scan(mgr).is_some()
    });
    // Anything the arming itself woke arms a full confirmation tail behind it.
    // Wait the whole tail out, derived from the shipped constants rather than
    // guessed, so the window measured afterwards is genuinely idle.
    std::thread::sleep(EVENT_CONFIRM_DELAY * (CONFIRM_WALKS + 1) + Duration::from_millis(100));
    while scanner.next_scan(mgr).is_some() {}
    scanner.scans_run()
}

/// A scanner forced onto the TIMED engine, for use as a control.
///
/// Reached through the documented fallback (a watch root that cannot exist), so
/// the control is the shipped degraded path rather than a second implementation
/// written for the test.
fn forced_polling_scanner(mgr: &Arc<TransportManager>, cadence: Duration) -> DiscoveryScanner {
    let nowhere = std::env::temp_dir().join(unique_topic("no-such-iox-root").replace('/', "_"));
    let mut scanner =
        DiscoveryScanner::start_with_interval_and_root(mgr, true, cadence, Some(nowhere));
    await_condition("the forced polling engine to report itself", || {
        let _ = scanner.next_scan(mgr);
        scanner.wake_source() == WakeSource::Poll
    });
    scanner
}

// ===========================================================================
// THE HEADLINE: an idle machine costs nothing, and every walk has a cause.
// ===========================================================================

/// Every walk the event-driven engine runs has a directory CHANGE behind it,
/// while the timed engine it replaced walks with nothing behind any of them.
///
/// # Why this is a ratio and not "walks == 0"
///
/// `iceoryx2::testing::generate_isolated_config` isolates a test by PREFIX, not
/// by root path: every test on this machine shares one `services` directory. So
/// a literal zero-walk assertion would be measuring what ELSE is running on the
/// box, and would fail on a busy one for a reason that has nothing to do with
/// this code. `walks <= 2 * wakes` (one walk for the change plus at most one
/// bounded confirmation) is the same property stated so that foreign activity
/// raises BOTH sides. A scanner that walks on a timer has no wakes at all and
/// fails it on its first tick.
///
/// The cost being removed is then stated as a property rather than as a wall: a
/// forced POLLING scanner runs in the SAME process, over the SAME window, at the
/// SAME cadence, and every one of its walks is uncaused. Each is 117.5 ms at 86
/// live topics.
///
/// The anti-inertness half is in the same body: a scanner that never enumerates
/// anything satisfies the ratio trivially while leaving discovery blind, which
/// is the defect this feature exists to fix wearing the fix's clothes.
#[test]
fn every_walk_the_event_engine_runs_has_a_directory_change_behind_it() {
    let mgr: Arc<TransportManager> = make_manager(8);
    // A live publisher, so the service directory exists to be watched at all.
    let first = unique_topic("idle-a");
    let _pub_a = publisher(&mgr, &first, 64);

    let mut events = DiscoveryScanner::start_with_interval(&mgr, true, FALLBACK_CADENCE);
    assert!(
        events.is_threaded(),
        "discovery ON must run a worker - an inline fallback here means the spawn failed, and \
         the whole point of the worker is that the drive loop does not enumerate"
    );
    settle_baseline(&mut events, &mgr);
    assert_eq!(
        events.wake_source(),
        WakeSource::Events,
        "the watch must be armed on a healthy machine with a real service directory - a Poll \
         here means the platform watch could not be established and this arm would be measuring \
         the fallback against itself"
    );

    // The control: the SAME cadence, forced onto the timed engine.
    let mut poll = forced_polling_scanner(&mgr, FALLBACK_CADENCE);

    // Standing up the control takes time, so let any confirmation armed by a
    // wake BEFORE the window land before the window opens - otherwise it is
    // counted against a wake the window never saw.
    std::thread::sleep(EVENT_CONFIRM_DELAY * (CONFIRM_WALKS + 1) + Duration::from_millis(100));
    while events.next_scan(&mgr).is_some() {}
    while poll.next_scan(&mgr).is_some() {}

    let event_base = events.scans_run();
    let poll_base = poll.scans_run();
    let wakes_before = events.wakes();
    let quiet_start = Instant::now();
    while quiet_start.elapsed() < QUIET_WINDOW {
        // Drive both exactly as the drive loop does, so neither is blocked on a
        // full mailbox while the other runs free.
        let _ = events.next_scan(&mgr);
        let _ = poll.next_scan(&mgr);
        std::thread::sleep(Duration::from_millis(2));
    }
    let event_walks = events.scans_run() - event_base;
    let event_wakes = events.wakes() - wakes_before;
    let poll_walks = poll.scans_run() - poll_base;
    println!(
        "idle window {QUIET_WINDOW:?}: event engine {event_walks} walk(s) from {event_wakes} \
         wake(s); polling engine {poll_walks} walk(s)"
    );

    // ANTI-VACUITY: the control must really have walked, or the invariant below
    // is being compared against nothing.
    assert!(
        poll_walks >= 5,
        "the polling control only walked {poll_walks} time(s) in {QUIET_WINDOW:?} at a \
         {FALLBACK_CADENCE:?} cadence - the window is too short (or the machine too starved) \
         for this arm to mean anything"
    );
    // ...and it walked with NO directory change behind a single one of them.
    // That is the cost this change removes, stated as a property rather than as
    // a wall: each of those walks is 117.5 ms at 86 live topics, so a settled
    // recorder paying that rate burns roughly half a core forever.
    assert_eq!(
        poll.wakes(),
        0,
        "the polling control walked {poll_walks} time(s); if any of them had a directory change \
         behind it then this is not the timed engine and the contrast below is not the contrast"
    );

    // THE INVARIANT, and the arm a poll cannot pass: every walk had a cause.
    //
    // Deliberately NOT `event_walks < poll_walks`. Every test on this machine
    // shares one `services` directory (`generate_isolated_config` isolates by
    // PREFIX, not by root), so a foreign service appearing during the window
    // gives this engine real wakes and real walks - which a comparison of raw
    // counts would read as a regression. This ratio raises both sides together,
    // so it cannot be broken by what else is running, while a scanner that walked
    // on a timer has NO wakes at all and fails it on its first tick.
    //
    // The `+ 1` is the one confirmation that can straddle the window's opening
    // edge: a wake microseconds before it arms a walk that lands microseconds
    // after. It forgives exactly that and nothing else - a timed engine over
    // this window produces a dozen or more uncaused walks, not one.
    let per_wake = u64::from(1 + CONFIRM_WALKS);
    assert!(
        event_walks <= per_wake * event_wakes + 1,
        "the event-driven engine ran {event_walks} walk(s) from {event_wakes} wake(s). Each \
         wake is worth one walk plus a tail of {CONFIRM_WALKS} confirmations, so a walk beyond \
         that had no directory change behind it, which is a timer by another name (the polling \
         control ran {poll_walks} such walks in the same window)"
    );

    // THE ANTI-INERTNESS HALF: the same scanner sees a topic that appears now.
    let late = unique_topic("idle-b");
    let _pub_b = publisher(&mgr, &late, 64);

    let mut saw_late = false;
    await_condition("an enumeration containing the newly created topic", || {
        if let Some(scan) = events.next_scan(&mgr) {
            if let Ok(live) = scan.result {
                saw_late = live.contains(&late);
            }
        }
        saw_late
    });
    assert!(
        events.wakes() > wakes_before + event_wakes,
        "the enumeration that found the topic must have been triggered by a directory CHANGE. \
         A scanner that walked on a timer would satisfy every assertion above except this one"
    );
    assert_eq!(
        events.wake_source(),
        WakeSource::Events,
        "the watch must still be the engine - a silent degrade to polling here would restore \
         the cost this change removes"
    );
}

// ===========================================================================
// Bursts: seventy routes opening at once are one question, not seventy.
// ===========================================================================

/// A burst of topics is COALESCED: far fewer wakes than topics, every topic
/// found.
///
/// The motivating shape is a bridge opening every route it discovered on a robot
/// bus within a second or two. The correct answer to every event in that storm
/// is the same directory walk, and a watcher that walked once per event would be
/// strictly worse than the poll it replaced.
///
/// The bound is on WAKES rather than walks, and that is what makes it robust on
/// a shared machine: the walk count is also governed by the rate floor and the
/// confirmation tail, while "how many distinct changes did this burst look
/// like?" is exactly the coalescing claim. Load can only push wakes DOWN (a
/// slower machine piles more events into one absorption window), so this fails
/// on a design regression, not on a contended desk.
#[test]
fn a_burst_of_topics_is_coalesced_into_far_fewer_wakes_than_topics() {
    /// Enough that "one wake per topic" and "a handful of wakes" are not close.
    const BURST: usize = 20;

    let mgr: Arc<TransportManager> = make_manager(8);
    let seed = unique_topic("burst-seed");
    let _seed_pub = publisher(&mgr, &seed, 64);

    let mut scanner = DiscoveryScanner::start_with_interval(&mgr, true, FALLBACK_CADENCE);
    let baseline = settle_baseline(&mut scanner, &mgr);
    if scanner.wake_source() != WakeSource::Events {
        // The degraded engine walks on a timer by design; the coalescing claim
        // is about the watch, so there is nothing here to adjudicate.
        return;
    }
    let wakes_before = scanner.wakes();

    let mut topics = Vec::with_capacity(BURST);
    let mut held = Vec::with_capacity(BURST);
    for i in 0..BURST {
        let t = unique_topic(&format!("burst-{i}"));
        held.push(publisher(&mgr, &t, 64));
        topics.push(t);
    }

    // Every one of them must end up in an enumeration - coalescing may not cost
    // coverage. That is the property a "fewer wakes" bound alone would let a
    // regression trade away.
    let mut seen: std::collections::BTreeSet<String> = std::collections::BTreeSet::new();
    await_condition(
        "every topic of the burst to appear in some enumeration",
        || {
            if let Some(scan) = scanner.next_scan(&mgr) {
                if let Ok(live) = scan.result {
                    for t in &topics {
                        if live.contains(t) {
                            seen.insert(t.clone());
                        }
                    }
                }
            }
            seen.len() == BURST
        },
    );
    assert_eq!(
        seen.len(),
        BURST,
        "coalescing must not lose a topic - a burst is one question, not a reason to skip any \
         of its answers"
    );

    let wakes = scanner.wakes() - wakes_before;
    let walks = scanner.scans_run() - baseline;
    println!("burst of {BURST} topics: {wakes} wake(s), {walks} walk(s)");
    assert!(
        wakes >= 1,
        "a burst of {BURST} topics must have woken the watch at least once - zero wakes means \
         the enumerations came from somewhere else and this arm measured nothing"
    );
    assert!(
        (wakes as usize) < BURST,
        "a burst of {BURST} topics produced {wakes} wake(s), which is not coalescing. One walk \
         per route would be strictly worse than the timed poll this change replaced: each walk \
         is 117.5 ms at 86 live topics"
    );
    assert!(
        walks <= u64::from(1 + CONFIRM_WALKS) * wakes + 1,
        "{walks} walk(s) from {wakes} wake(s): every walk must be a change plus a tail of at \
         most {CONFIRM_WALKS} confirmations, with one allowance for a confirmation armed by \
         the baseline"
    );
}

// ===========================================================================
// The fallback: loud, observable, and still correct.
// ===========================================================================

/// A watch that cannot be ARMED degrades to the timed walk, says so, and keeps
/// discovering.
///
/// The degradation has three distinct ways to go wrong and each would be
/// invisible without its own check: it could be unobservable (`wake_source`
/// still claiming `Events`), it could go silent, or it could simply stop
/// discovering. The last is the one that matters to a recording, so the arm
/// ends by proving a topic created after the degrade is still enumerated.
///
/// # Why the loudness half is split in two
///
/// The warn is emitted on the WORKER thread, and `tracing_test` matches lines
/// by the test's own span prefix - an event with no span is never matched, so
/// `logs_contain` returns false for a line it can be watched printing. Rather
/// than assert something the harness cannot see, the property is decomposed
/// into the two halves that together mean it:
///
/// * `wake_source()` flipping to `Poll` proves `degrade_to_poll` RAN, because
///   that function is the only thing that writes it; and
/// * a source walk proves `degrade_to_poll` reports at `warn!` with the text an
///   operator would grep for.
///
/// Together those are "the degrade was reported, loudly, with that wording",
/// deterministically and without a subprocess.
#[test]
fn a_watch_that_cannot_be_armed_falls_back_to_a_timed_walk_loudly() {
    let mgr: Arc<TransportManager> = make_manager(8);
    let topic = unique_topic("fallback");
    let _pub = publisher(&mgr, &topic, 64);

    // A root that does not exist has no directory to watch and no parent to
    // watch it appear in - the same refusal an unsupported platform or a
    // descriptor limit produces, reached deterministically.
    let nowhere = std::env::temp_dir().join(unique_topic("no-such-iox-root").replace('/', "_"));
    let mut scanner =
        DiscoveryScanner::start_with_interval_and_root(&mgr, true, FALLBACK_CADENCE, Some(nowhere));

    await_condition("the scanner to report the degraded engine", || {
        // Drain so the worker is never blocked on a full mailbox while we look.
        let _ = scanner.next_scan(&mgr);
        scanner.wake_source() == WakeSource::Poll
    });
    assert_eq!(
        scanner.wake_source(),
        WakeSource::Poll,
        "an unarmable watch must be OBSERVABLE as a degrade, not merely logged - an operator \
         reading a recording's CPU cost has to be able to tell which engine ran"
    );
    // The other half of "loudly": the function that just ran reports it at
    // `warn!`, with the wording an operator would grep for.
    let src = code_only(
        &std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/src/discovery_scan.rs"
        ))
        .expect("read cerulion_bagd/src/discovery_scan.rs"),
    );
    let reporter = fn_body(&src, "degrade_to_poll");
    for required in [
        "tracing::warn!",
        "falls back to RE-ENUMERATING the directory on a timer",
        "WakeSource::Poll",
    ] {
        assert!(
            reporter.contains(required),
            "`degrade_to_poll` must contain `{required}`. The degrade must be LOUD AND \
             observable: a recorder that silently stopped watching would keep its coverage and \
             quietly pay a full service-directory walk on every cadence. Body:\n{reporter}"
        );
    }

    // And it still works: the degraded engine is slow, never blind.
    let late = unique_topic("fallback-late");
    let _pub_late = publisher(&mgr, &late, 64);
    let mut saw_late = false;
    await_condition("the degraded engine to enumerate a new topic", || {
        if let Some(scan) = scanner.next_scan(&mgr) {
            if let Ok(live) = scan.result {
                saw_late = live.contains(&late);
            }
        }
        saw_late
    });
    assert_eq!(
        scanner.wakes(),
        0,
        "the degraded engine walks on a timer, so it can never report a directory CHANGE - a \
         nonzero count here means the two engines' observables have been crossed"
    );
}

/// Discovery OFF arms no watch, starts no thread and serves nothing.
///
/// The anti-tautology control for every count above: without it, "the event
/// engine walked far less than the polling control" is also satisfied by a
/// scanner that was never started, and the comparison would be measuring
/// nothing.
#[test]
fn a_disabled_scanner_arms_no_watch_and_serves_no_scans() {
    let mgr: Arc<TransportManager> = make_manager(8);
    let topic = unique_topic("off");
    let _pub = publisher(&mgr, &topic, 64);

    let mut scanner = DiscoveryScanner::start_with_interval(&mgr, false, Duration::from_millis(10));
    assert!(!scanner.is_threaded(), "discovery OFF must start no worker");
    assert_eq!(
        scanner.wake_source(),
        WakeSource::Off,
        "a disabled scanner must report no engine at all"
    );

    // A topic appearing must not move anything either - the watch is the thing
    // being proved absent, so the stimulus has to be one it would react to.
    let _pub_b = publisher(&mgr, &unique_topic("off-late"), 64);
    std::thread::sleep(Duration::from_millis(300));
    assert_eq!(scanner.scans_run(), 0, "a disabled scanner runs no walks");
    assert_eq!(scanner.wakes(), 0, "a disabled scanner arms no watch");
    for i in 0..5 {
        assert!(
            scanner.next_scan(&mgr).is_none(),
            "call {i}: a disabled scanner must serve nothing"
        );
    }
}

// ===========================================================================
// The ordering the worker cannot be tested into behaviourally.
// ===========================================================================

/// `wakes` can only be bumped by a real directory CHANGE.
///
/// The behavioural arms correlate walks with this counter, so a scanner that
/// incremented it on a timer would satisfy every one of them while polling. What
/// the counter actually counts is not something those arms can see, because they
/// read it through the same object they are testing - so it is stated at source:
/// the increment sits in the `WalkAndConfirm` branch, which `next_step` answers
/// for `WatchWake::Changed` and for nothing else.
#[test]
fn the_wake_counter_is_bumped_only_on_a_directory_change() {
    let src = code_only(
        &std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/src/discovery_scan.rs"
        ))
        .expect("read cerulion_bagd/src/discovery_scan.rs"),
    );

    // Exactly one increment in the whole module, and it is guarded by the step
    // that only a change produces.
    let bumps = src.matches("wakes.fetch_add").count();
    assert_eq!(
        bumps, 1,
        "`wakes` must have exactly ONE increment, or the arms that read it cannot say what it \
         counts; found {bumps}"
    );
    let at = src
        .find("wakes.fetch_add")
        .expect("the increment was just counted");
    let guard = &src[..at];
    let step_at = guard
        .rfind("if step == ScanStep::WalkAndConfirm")
        .expect("the increment must sit inside the WalkAndConfirm branch");
    assert!(
        !guard[step_at..].contains('}'),
        "the increment must still be INSIDE the `WalkAndConfirm` branch - a timer tick reaching \
         it would make every behavioural arm in this file vacuous"
    );

    // And that step is answered for a change alone.
    let decide = fn_body(&src, "next_step");
    assert!(
        decide.contains("WatchWake::Changed => ScanStep::WalkAndConfirm"),
        "`next_step` must answer `WalkAndConfirm` for `Changed`. Body:\n{decide}"
    );
    for uncaused in ["WatchWake::Idle => ScanStep::WalkAndConfirm"] {
        assert!(
            !decide.contains(uncaused),
            "`next_step` must never answer `WalkAndConfirm` for `{uncaused}`. Body:\n{decide}"
        );
    }
}

/// The watch is ARMED before the baseline walk, never after it.
///
/// # Why this is a source walk and not a behavioural arm
///
/// The failure it guards is a race against the single most expensive operation
/// in the module: with the walk first, a producer registering DURING that walk
/// (117.5 ms at 86 live topics) is in neither the walk's result nor any event,
/// because its file appeared before anything was watching. On a settled machine
/// nothing ever changes that directory again, so the topic is never enumerated,
/// never tapped, never named in the coverage manifest - and `wakes()` stays 0,
/// the heartbeat keeps stamping and `wake_source()` still reads `Events`, so
/// not one of the three loud degradation paths fires.
///
/// Winning that race deliberately would mean creating a service inside a window
/// the test cannot see the edges of, which is not a test but a coin flip. The
/// ordering IS the property, and the source states it exactly.
#[test]
fn the_watch_is_armed_before_the_workers_baseline_walk() {
    let src = code_only(
        &std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/src/discovery_scan.rs"
        ))
        .expect("read cerulion_bagd/src/discovery_scan.rs"),
    );
    let body = fn_body(&src, "scan_loop");

    let arm_at = body.find("ServiceDirWatch::arm").unwrap_or_else(|| {
        panic!("`scan_loop` must arm the watch - the walk is reading the wrong text. Body:\n{body}")
    });
    let walk_at = body.find("publish(").unwrap_or_else(|| {
        panic!("`scan_loop` must publish a baseline enumeration. Body:\n{body}")
    });
    assert!(
        arm_at < walk_at,
        "the watch must be armed BEFORE the baseline walk. With the walk first, a producer that \
         registers during it is in neither the walk's result nor any event, and is never \
         enumerated for the life of the recording while every observable reads healthy. Body:\n\
         {body}"
    );
}

// ===========================================================================
// End to end: a producer that appears after the recorder armed.
// ===========================================================================

/// A topic created AFTER the recorder armed is recorded, from the first frame
/// its tap can see.
///
/// The flagship shape: the recorder is armed before the graph is released to
/// step 0, so the producers it exists to capture do not exist when it looks.
/// Nothing on the drive loop enumerates - the walk that found this topic
/// happened on the worker, woken by the topic's own service file appearing.
///
/// The ready file proves `Recorder::setup` finished, not that the worker's
/// watch is armed - the scanner is built later, in the drive loop. That is not
/// a race this arm has to close, and the reason is the ordering the sibling
/// source walk pins: the watch is armed BEFORE the baseline walk, so a producer
/// created before arming is found by that walk and one created after it is
/// found by its own event. There is no third case for the stimulus to land in.
///
/// The frame oracle is EXACT (`== FRAMES`), which is only meaningful because
/// the publishing starts after a rendezvous on the tap actually being attached:
/// a data-only tap has no back-fill, so frames published before it attaches are
/// simply gone and an exact count taken without the rendezvous would be a race.
#[test]
#[serial_test::serial]
fn a_topic_created_after_arm_is_recorded_from_its_first_frames() {
    const FRAMES: u32 = 12;

    let mgr = make_manager(16);
    let declared = unique_topic("post-arm-declared");
    let late = unique_topic("post-arm-late");
    let out = unique_out("post-arm");
    let ready = unique_out("post-arm_ready");
    let mut declared_pub = publisher_with_provisioning(&mgr, &declared, 8, 16, 4096);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = BagdConfig::new(out.clone(), vec![TapSpec::attach(&declared)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.ready_file = Some(ready.clone());
    cfg.status_period = None;
    cfg.schema_wait = Duration::from_millis(3000);
    cfg.discover_live = true;
    let mgr_for_bagd = mgr.clone();
    let flag = shutdown.clone();
    let handle = std::thread::spawn(move || run_bagd(mgr_for_bagd, cfg, flag));
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    // The topic does not exist until HERE - strictly after the recorder armed,
    // which is what makes the arm-time enumeration unable to have found it.
    let appeared = Instant::now();
    let mut late_pub = publisher_with_provisioning(&mgr, &late, 8, 16, 4096);

    // Rendezvous on the tap, never a sleep.
    await_condition(
        "discovery to attach a tap to the newly created topic",
        || mgr.topic_subscriber_count(&late) >= 1,
    );
    let discovery_latency = appeared.elapsed();

    for i in 0..FRAMES {
        let frame = build_frame(HASH, i, 1_000 + i as u64, b"post-arm");
        late_pub.publish_raw(&frame).expect("publish late");
        declared_pub.publish_raw(&frame).expect("publish declared");
        settle();
    }
    std::thread::sleep(Duration::from_millis(300));
    shutdown.store(true, Ordering::Relaxed);
    let summary = handle.join().expect("bagd thread").expect("clean finalize");

    let coverage = &summary.record_coverage;
    let entry = coverage.tapped.get(&late).unwrap_or_else(|| {
        panic!("a producer that appeared after arm must be recorded: {coverage:?}")
    });
    assert_eq!(
        entry.source,
        TapSource::Discovered,
        "only discovery could have tapped a topic the caller never named"
    );
    assert!(
        entry.attached_late,
        "the topic did not exist at arm time, so its tap is by definition a late attach - a \
         `false` here would claim coverage from the producer's first frame, which no data-only \
         tap can give"
    );
    assert_eq!(
        entry.frames_recorded, FRAMES as u64,
        "every frame published after the rendezvous must be in the bag; a shortfall means the \
         tap was reported attached before it was"
    );
    assert_eq!(coverage.gap_count(), 0, "nothing live went unrecorded");

    // The same account survives into the bag an operator reads later.
    let reader = BagReader::open(&out).expect("open bag");
    let att = reader
        .attachment(RECORD_COVERAGE_ATTACHMENT)
        .expect("read attachments")
        .expect("record_coverage.json is present in every finalized bag");
    let on_disk: RecordCoverage = serde_json::from_slice(&att.data).expect("coverage parses");
    assert!(
        on_disk.tapped.contains_key(&late),
        "the bag's own manifest must name the discovered topic, not just the in-process summary"
    );

    // Not an assertion - a stopwatch tight enough to separate an event from a
    // timer is also tight enough for a loaded runner to invert. Printed so a
    // human reading a failure has the number in front of them.
    println!("discovery latency (producer created -> tap attached): {discovery_latency:?}");

    cleanup(&out);
}

// ===========================================================================
// The measurement harness. Not a gate.
// ===========================================================================

/// Print what discovery costs at a realistic topic count, BOTH ways.
///
/// Both engines are measured in ONE process, on ONE machine, against ONE
/// hundred live topics, so the comparison carries no cross-build or
/// cross-machine term: the event-driven engine is the shipping one, and the
/// polling engine is reached through the documented fallback seam and is
/// byte-for-byte the behaviour this change replaced.
///
/// Deliberately `#[ignore]`d: it stands up a hundred publishers and then does
/// nothing for ten seconds twice, which is a measurement, not a property. Run it
/// by hand with `--ignored --nocapture`.
#[test]
#[ignore = "measurement harness: run by hand with --ignored --nocapture"]
fn measure_idle_discovery_cost_at_one_hundred_topics() {
    const TOPICS: usize = 100;
    const OBSERVE: Duration = Duration::from_secs(10);
    const SHIPPED_CADENCE: Duration = Duration::from_millis(250);

    let mgr: Arc<TransportManager> = make_manager(8);
    let mut held = Vec::with_capacity(TOPICS);
    let mut names = Vec::with_capacity(TOPICS);
    for i in 0..TOPICS {
        let topic = unique_topic(&format!("measure-{i}"));
        held.push(publisher_with_provisioning(&mgr, &topic, 8, 16, 4096));
        names.push(topic);
    }

    // What ONE walk costs on this machine, at this topic count.
    let mut walk_ns = Vec::with_capacity(21);
    for _ in 0..21 {
        let t = Instant::now();
        let live = mgr.list_topics().expect("enumerate");
        walk_ns.push(t.elapsed().as_nanos() as u64);
        assert!(live.len() >= TOPICS);
    }
    walk_ns.sort_unstable();
    let walk_p50 = Duration::from_nanos(walk_ns[walk_ns.len() / 2]);

    // Idle cost, each engine, same window.
    let mut event_scanner = DiscoveryScanner::start_with_interval(&mgr, true, SHIPPED_CADENCE);
    let event_base = settle_baseline(&mut event_scanner, &mgr);
    let event_engine = event_scanner.wake_source();
    let event_walks = idle_walks(&mut event_scanner, &mgr, OBSERVE) - event_base;

    let mut poll_scanner = forced_polling_scanner(&mgr, SHIPPED_CADENCE);
    let poll_base = poll_scanner.scans_run();
    let poll_engine = poll_scanner.wake_source();
    let poll_walks = idle_walks(&mut poll_scanner, &mgr, OBSERVE) - poll_base;

    // And a REAL recording's drive loop over the same topic set, so the
    // per-pass numbers come from the recorder rather than from a harness.
    let out = unique_out("measure");
    let ready = unique_out("measure_ready");
    let shutdown = Arc::new(AtomicBool::new(false));
    // One DECLARED tap so the recorder has a topic it was asked for; discovery
    // finds the other ninety-nine, which is the shape being measured.
    let mut cfg = BagdConfig::new(out.clone(), vec![TapSpec::attach(&names[0])]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.ready_file = Some(ready.clone());
    cfg.status_period = None;
    cfg.schema_wait = Duration::from_millis(3000);
    cfg.discover_live = true;
    let mgr_for_bagd = mgr.clone();
    let flag = shutdown.clone();
    let handle = std::thread::spawn(move || run_bagd(mgr_for_bagd, cfg, flag));
    assert!(
        wait_for_file(&ready, Duration::from_secs(60)),
        "bagd ready (a hundred taps and a mirror gather take a while to arm)"
    );
    std::thread::sleep(Duration::from_secs(10));
    shutdown.store(true, Ordering::Relaxed);
    let summary = handle.join().expect("bagd thread").expect("clean finalize");
    // Span over passes is the loop's PERIOD, which includes the backlog-aware
    // pacing sleep between passes. It is NOT a mean pass duration, and printing
    // it as one puts a number larger than the measured worst pass next to it.
    let mean_period = summary
        .drive_span
        .checked_div(summary.drive_passes.max(1) as u32)
        .unwrap_or_default();

    println!("topics                   : {TOPICS}");
    println!("one list_topics walk     : {walk_p50:?} p50 over 21 samples");
    println!("observation window       : {OBSERVE:?}, machine idle throughout");
    println!("engine (shipping)        : {event_engine:?}");
    println!("  walks in the window    : {event_walks}");
    println!(
        "  wall spent walking     : {:?}",
        walk_p50 * event_walks as u32
    );
    println!("engine (fallback/before) : {poll_engine:?} at {SHIPPED_CADENCE:?}");
    println!("  walks in the window    : {poll_walks}");
    println!(
        "  wall spent walking     : {:?}",
        walk_p50 * poll_walks as u32
    );
    println!("recorder drive loop over {TOPICS} discovered topics:");
    println!("  passes                 : {}", summary.drive_passes);
    println!("  span                   : {:?}", summary.drive_span);
    println!("  mean period            : {mean_period:?} (work plus the pacing sleep)");
    println!(
        "  worst pass             : {:?} (work only, the high water)",
        summary.max_pass_duration
    );

    cleanup(&out);
}

/// Drive `scanner` exactly as the drive loop does for `window`, and report its
/// total enumeration count afterwards.
fn idle_walks(scanner: &mut DiscoveryScanner, mgr: &TransportManager, window: Duration) -> u64 {
    let t = Instant::now();
    while t.elapsed() < window {
        // Exactly what the drive loop does: take whatever is there, O(1).
        let _ = scanner.next_scan(mgr);
        std::thread::sleep(Duration::from_millis(1));
    }
    scanner.scans_run()
}
