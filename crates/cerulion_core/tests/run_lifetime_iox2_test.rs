// SPDX-License-Identifier: AGPL-3.0-only
//! Binding a consumer's lifetime to ONE run, over REAL
//! iceoryx2.
//!
//! Every test mints its OWN isolated SHM root (`testing::iceoryx_test_config`),
//! so a watcher here sees exactly the runs this test published. Parallel-safe:
//! no process-global state, no `#[serial]`.
//!
//! # What these arms are for
//!
//! The pure oracles in `transport::run_registry`'s `mod lifetime_tests` pin the
//! DECISION — which reading means what, and how many it takes to act. They are
//! blind to the property C3 actually turns on, which is a transport fact:
//!
//! > **`Ending` is published ONCE, and the writer is released microseconds
//! > later.**
//!
//! C0 MEASURED a gatherer racing a graceful exit at 7 of 8 at a hot spin and
//! **0 of 8 at a natural cadence**, because every `gather_runs_on_config` opens
//! a FRESH subscriber and the registry service carries no history — so a frame
//! sent before that subscriber existed is not missed by timing, it is
//! unreachable. C3's answer is a LONG-LIVED subscriber, and
//! [`a_graceful_end_is_heard_long_after_the_run_is_gone`] is where that answer
//! is proved rather than asserted: it observes the run's last word LONG after
//! the run is gone, with a fresh gather at that same instant as the CONTROL
//! showing what a polling consumer would have had.
//!
//! # Load discipline
//!
//! No arm states a wall in units of the republish interval (macOS
//! background-QoS timer coalescing charges sleep slack PER WAKEUP; a nominal
//! 150 ms has been measured as 1100 to 1696 ms). Verdicts are the assertions;
//! every wait is a CONDITION under a generous seconds-scale ceiling, which load
//! can delay but not invert. The one arm that needs silence to MEAN something
//! injects its own short grace rather than spending the shipped one, so its
//! claim is about the rule and not about the clock.
//!
//! **The rewrite of these arms extends that rule: no arm compares two wall-derived COUNTS
//! either.** A verdict index is `ceil(grace / iteration_cost) + 1`, so a
//! comparison between two of them is a wall-clock race wearing a ratio's
//! clothing — as load stretches the iteration both indices collapse into small
//! integers and the discrimination evaporates. Where two routes must be told
//! apart, they are separated so far that only ONE of them can fire inside the
//! arm's ceiling, and the assertion is which one did — see [`SHRINK_GRACES`].

use std::time::{Duration, Instant};

use cerulion_core::transport::run_registry::{
    gather_runs_on_config, RunEnded, RunGraces, RunHandle, RunRecord, RunSignal, RunState,
    RunWatcher, RUN_GATHER_WINDOW,
};

/// A short grace, so an arm that must reach the ABSENT verdict does not spend
/// the shipped five seconds. It is still an order of magnitude above the
/// republish interval, so a healthy run cannot fall through it.
const TEST_GRACE: Duration = Duration::from_millis(1_500);

/// How often the SHRINK arm looks at its two watchers. Fine relative to the
/// 300 ms grace it waits out, and deliberately not finer: `observe` drains a
/// subscriber and reads a dynamic config, and the arm gains nothing from
/// spinning. Nothing is asserted in units of it.
const OBSERVE_SPACING: Duration = Duration::from_millis(20);

/// The grace pair BOTH watchers in the SHRINK arm run under — the
/// SAME pair for both, so a successor is the only variable between them.
///
/// # Why the two are separated by 2000x rather than by the shipped ratio
///
/// The arm used to inject one grace, let `RunGraces::scaled_from` derive the
/// other at the shipped 2 s : 5 s, drive both watchers on one loop and compare
/// the observation INDEX at which each reached its verdict. That is a
/// wall-clock race wearing a ratio's clothing: each index is
/// `ceil(grace / iteration_cost) + 1`, so as load stretches the iteration both
/// collapse into small integers whose ratio no longer carries the graces'.
///
/// MEASURED by moving that one variable and nothing else — at a 500 ms
/// iteration a CORRECT implementation yields `(3, 4)` and the ratio assertion
/// failed; at 1500 ms it yields `(2, 2)` and tied. Both CI firings were exactly
/// that, on a saturated 2-slot runner where macOS background-QoS coalescing
/// was measured charging a nominal wait 7-11x.
///
/// With this pair the arm asserts a CONDITION instead. While the successor
/// publishes, the no-successor watcher's other two routes are BOTH foreclosed —
/// a live writer kills the exact zero-publisher route, and a different
/// `graph_name` kills the successor route — so its only remaining route is a
/// silence grace 10x longer than [`SHRINK_LIVENESS_CEILING`], which nothing in
/// this arm waits out. Load can delay the replaced watcher's verdict; it cannot
/// make the alone watcher's arrive.
///
/// [`vanish`](RunGraces::vanish) is therefore doing two jobs, and the second is
/// easy to miss: it is also the window in which the arm's whole INFERENCE holds.
/// A verdict is attributed to the successor route because no other route can
/// have produced it — which stops being true the moment the observation loop has
/// been silent for longer than this grace, since `quiet_past_the_grace` would
/// then fire for both watchers. That is a HARNESS condition, not a product one,
/// and it is guarded in the loop below.
///
/// The shipped RATIO is not this arm's business and is pinned where it belongs:
/// by the `RUN_REPLACED_GRACE <= RUN_VANISH_GRACE` const-assert and by
/// `run_registry`'s pure `lifetime_tests` (both thresholds pinned on both
/// sides, and `scaled_from` keeping the ratio).
const SHRINK_GRACES: RunGraces = RunGraces {
    vanish: Duration::from_secs(600),
    replaced: Duration::from_millis(300),
};

/// How long the SHRINK arm waits for the replaced watcher's verdict to ARRIVE
/// AT ALL before reporting that it never did.
///
/// **A liveness bound, NOT a response-time bound**, and the distinction is the
/// whole of the rewrite. A verdict that arrives LATE is accepted, deliberately: a
/// starved runner legitimately delivers one late — that is exactly what made
/// this arm flake — so rejecting a late verdict would re-arm the load-inversion
/// class the rewrite exists to have escaped. What this ceiling exists to catch
/// is a verdict that never comes, which is what a deleted or inert
/// `replaced_by_a_successor` looks like, and it is therefore checked on the
/// iterations that produce NO verdict.
///
/// Response time is a function of the constants, and measuring it from a
/// transport arm is a wall race by construction. It is owned where it can be
/// stated exactly: the `RUN_REPLACED_GRACE <= RUN_VANISH_GRACE` const-assert and
/// `run_registry`'s pure `lifetime_tests`, which pin both thresholds on both
/// sides with no clock in the room.
///
/// Sized between two bounds, with room against each: ~170x the ~0.35 s healthy
/// path (and ~6x the slowest run MEASURED in the spacing sweep, 10.4 s
/// at a 5 s iteration — 250x this loop's nominal spacing), while staying at one
/// tenth of [`SHRINK_GRACES`]`.vanish`, which is the separate and much looser
/// bound the arm's inference actually depends on.
const SHRINK_LIVENESS_CEILING: Duration = Duration::from_secs(60);

/// A hand-built run record. Every field DISTINCT and non-default.
fn record(run_id: u128, graph: &str) -> RunRecord {
    RunRecord {
        run_id,
        supervisor_pid: 4711,
        run_started_at_ns: 1_753_000_000_000_000_000,
        state: RunState::Live,
        graph_name: graph.to_string(),
        run_dir: format!("/tmp/{graph}-{run_id:032x}"),
    }
}

/// Observe until the watcher reports a terminal outcome, or fail after a
/// GENEROUS seconds ceiling. Load can only delay a verdict; a broken binding
/// never reaches one at all.
///
/// The ceiling is checked only on iterations that yield NO outcome, so an
/// outcome arriving after it is accepted. Deliberate, and the same rule as
/// [`SHRINK_LIVENESS_CEILING`]: this is a liveness bound, so failing a
/// late-but-arriving verdict would turn a starved runner into a red build
/// without telling anyone anything true.
fn observe_until_ended(watcher: &mut RunWatcher, what: &str) -> RunEnded {
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        if let Some(ended) = watcher.observe().ended {
            return ended;
        }
        assert!(
            Instant::now() < deadline,
            "{what}: the watcher never reached a verdict within 20s"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
}

/// Observe until the watcher has HEARD its run at least once (i.e. reports
/// `Live`), so an arm that then kills the run is testing a binding that was
/// actually established.
fn observe_until_live(watcher: &mut RunWatcher, what: &str) {
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        let observation = watcher.observe();
        if observation.signal == RunSignal::Live {
            return;
        }
        assert!(
            Instant::now() < deadline,
            "{what}: the watcher never heard its run within 20s (last signal {:?})",
            observation.signal
        );
        std::thread::sleep(Duration::from_millis(20));
    }
}

/// **THE headline, and the whole reason C3 uses a long-lived subscriber.**
///
/// A graceful end is published ONCE and the writer is gone microseconds later.
/// Here the watcher does not even LOOK until the run is long dead — and it still
/// reports `Graceful`, because the frame has been sitting in its own SHM queue
/// since the instant it was sent.
///
/// The CONTROL is in the same body and is what makes this a measurement rather
/// than a claim: at that same instant a fresh `gather_runs_on_config` — the
/// polling shape, which opens a new subscriber per call — sees NOTHING. A
/// consumer built that way cannot tell this graceful exit from a crash.
#[test]
fn a_graceful_end_is_heard_long_after_the_run_is_gone() {
    let config = cerulion_core::testing::iceoryx_test_config();
    let run_id = 0x1111_2222_3333_4444_5555_6666_7777_8888;
    let handle = RunHandle::publish_on_config(&config, record(run_id, "go2_attach")).expect("run");

    let mut watcher = RunWatcher::open_on_config(&config, run_id).expect("watcher");
    observe_until_live(&mut watcher, "the graceful arm");

    // A graceful exit: announce, then go. This is exactly `RunDescriptor::drop`.
    handle.set_ending();
    drop(handle);

    // Deliberately look LATE — far past any window a gather could have held
    // open, and past the point where anything is still publishing. A polling
    // consumer has nothing left to see here; this watcher has a queue.
    std::thread::sleep(Duration::from_millis(500));

    // The CONTROL first, so it cannot be accused of running after the watcher
    // somehow revived anything: what a POLLING consumer would have at this
    // instant.
    let polled = gather_runs_on_config(&config, RUN_GATHER_WINDOW).expect("gather");
    assert!(
        !polled.records.iter().any(|r| r.run_id == run_id),
        "PRECONDITION: the run must be gone from a fresh gather, else this arm is not testing \
         late observation at all — got {:?}",
        polled.records
    );

    let observation = watcher.observe();
    assert_eq!(
        observation.signal,
        RunSignal::Ending,
        "the run's LAST WORD was delivered to this subscriber and must still be readable — a \
         polling consumer saw {:?} at this same instant",
        polled.records
    );
    assert_eq!(
        observation.ended,
        Some(RunEnded::Graceful),
        "a graceful end is terminal on its first sighting"
    );
    assert_eq!(watcher.ended(), Some(RunEnded::Graceful));
}

/// A run that DISAPPEARS without announcing anything — a crash, a SIGKILL — is
/// reported `Vanished`. Nothing is published on this path, so absence is the
/// only available signal, and this is the arm that proves the recorder is not
/// wedged waiting for a word that will never come.
///
/// Dropping a `RunHandle` directly is exactly the crash shape: `RunHandle` has
/// no `Drop` that announces (the announcement lives in `cerulion_cli_engine`'s
/// `RunDescriptor::drop`), so the record simply stops.
///
/// SCOPE, because it is easy to over-read: this arm has ONE run on its isolated
/// namespace, so the drop takes the registry's publisher count to zero and the
/// EXACT zero-publisher route fires. That count is machine-WIDE, so on a real
/// machine with any other run live — a successor, a concurrent graph, a second
/// recording — the same crash reaches the same verdict through a grace instead.
/// Which route fires is `classify_run_signal`'s business and is pinned purely;
/// what this arm pins is that a crash reaches `Vanished` at all rather than
/// wedging the recorder waiting for a word that will never come.
#[test]
fn a_run_that_disappears_without_announcing_is_reported_vanished() {
    let config = cerulion_core::testing::iceoryx_test_config();
    let run_id = 0x2222_3333_4444_5555_6666_7777_8888_9999;
    let handle = RunHandle::publish_on_config(&config, record(run_id, "crashy")).expect("run");

    let mut watcher =
        RunWatcher::open_on_config_with_grace(&config, run_id, TEST_GRACE).expect("watcher");
    observe_until_live(&mut watcher, "the vanish arm");

    // No `set_ending` — the run just goes.
    drop(handle);

    assert_eq!(
        observe_until_ended(&mut watcher, "the vanish arm"),
        RunEnded::Vanished,
        "a run that stops publishing without announcing must be reported as vanished, never as \
         graceful"
    );
}

/// The ANTI-TAUTOLOGY for every arm above: a run that keeps running is NEVER
/// reported ended, however long the watcher looks.
///
/// Without this, "the watcher reports an outcome" is satisfied by a watcher that
/// reports one unconditionally — which would finalize every recording on its
/// first pass.
#[test]
fn a_live_run_is_never_reported_ended() {
    let config = cerulion_core::testing::iceoryx_test_config();
    let run_id = 0x3333_4444_5555_6666_7777_8888_9999_aaaa;
    let _handle = RunHandle::publish_on_config(&config, record(run_id, "healthy")).expect("run");

    let mut watcher =
        RunWatcher::open_on_config_with_grace(&config, run_id, TEST_GRACE).expect("watcher");
    observe_until_live(&mut watcher, "the live arm");

    // Watch for several multiples of the injected grace. A watcher that could
    // fabricate an absence would have done it many times over by here.
    //
    // A resolution residual (the same class as the SHRINK arm's, at nothing like the same
    // headroom — recorded because a residual worth closing is still worth
    // naming): the `live_sightings` precondition below asked the WALL
    // window to resolve into enough observations to contain one. It normally
    // resolves into ~200, and reaching zero needs a SINGLE iteration charged
    // the whole 4.5 s window — a 225x stretch of its 20 ms sleep, against the
    // 7-11x measured for macOS coalescing. So the loop now exits on the window AND on the
    // condition, under the same generous failsafe every other arm uses, and
    // the precondition can no longer be lost to resolution.
    let until = Instant::now() + TEST_GRACE * 3;
    let failsafe = Instant::now() + Duration::from_secs(20);
    let mut live_sightings = 0usize;
    while Instant::now() < until || live_sightings == 0 {
        let observation = watcher.observe();
        assert_eq!(
            observation.ended, None,
            "a run that is still publishing must never be reported ended (signal {:?})",
            observation.signal
        );
        if observation.signal == RunSignal::Live {
            live_sightings += 1;
        }
        assert!(
            Instant::now() < failsafe,
            "the watcher never heard the still-publishing run again within 20s — 'never ended' \
             would be vacuous (last signal {:?})",
            observation.signal
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    assert!(
        live_sightings > 0,
        "PRECONDITION: the watcher must actually have been hearing the run, else 'never ended' is \
         vacuous"
    );
    assert_eq!(watcher.ended(), None);
}

/// **The motivating scenario.** A run dies; the consumer is still bound to it;
/// the graph RESTARTS, minting a new `run_id` and publishing on the same
/// registry. The binding must end on the FIRST run — a second run's records
/// must not keep it alive.
///
/// This is what makes one-bag-one-run structural rather than promised: the
/// consumer is already finalizing before the new run's frames could reach it.
///
/// Dropping the `run_id` filter in `RunWatcher::observe` — the
/// watcher would then be kept alive by the successor's records and splice the
/// two runs, which is precisely the reported defect.
#[test]
fn a_restarted_graph_does_not_keep_the_binding_to_the_run_it_replaced() {
    let config = cerulion_core::testing::iceoryx_test_config();
    let first_id = 0x4444_5555_6666_7777_8888_9999_aaaa_bbbb;
    let second_id = 0x5555_6666_7777_8888_9999_aaaa_bbbb_cccc;

    let first = RunHandle::publish_on_config(&config, record(first_id, "go2_attach")).expect("run");
    let mut watcher =
        RunWatcher::open_on_config_with_grace(&config, first_id, TEST_GRACE).expect("watcher");
    observe_until_live(&mut watcher, "the restart arm");

    // The first run dies without announcing, and the graph is restarted at once
    // — the "forgot to stop the recorder" shape. The successor is LIVE and
    // republishing throughout the window in which the watcher must give up on
    // its predecessor.
    drop(first);
    let _second =
        RunHandle::publish_on_config(&config, record(second_id, "go2_attach")).expect("restart");

    let ended = observe_until_ended(&mut watcher, "the restart arm");
    assert_eq!(
        ended,
        RunEnded::Vanished,
        "the binding is to the FIRST run: a successor's records are another run's traffic, not \
         evidence that this one is alive"
    );

    // And the successor really was live and heard throughout — otherwise this
    // arm degenerates into the plain vanish test and catches nothing extra.
    let observation = watcher.observe();
    assert!(
        observation.runs_seen >= 2,
        "PRECONDITION: the watcher must have HEARD the successor run (it is on the same registry) \
         — only then does ending on the first run prove the run_id filter is real; runs_seen={}",
        observation.runs_seen
    );
    assert_eq!(
        watcher.ended(),
        Some(RunEnded::Vanished),
        "and the verdict must not have been walked back by the successor's traffic"
    );
}

/// **The SHRINK, and the proof it is not inert.** A run replaced by a SUCCESSOR
/// reaches its verdict on the shorter replaced-grace route, not on the full
/// silence grace — and the difference is the splice window, because a consumer
/// keeps recording the successor's data onto the predecessor's topics for
/// exactly as long as the verdict takes.
///
/// The pure oracles in `run_registry`'s `lifetime_tests` already pin the RULE
/// (`classify_run_signal`'s precedence, both thresholds pinned on both sides).
/// What only real transport can show is that the route is REACHED IN
/// PRODUCTION: that a successor's record, arriving over the wire, is recognised
/// by `is_successor_record` and really does shorten the wait.
///
/// Two watchers, ONE stimulus, in one body, under the IDENTICAL grace pair —
/// they are bound to two runs that die at the same instant on the same
/// registry, and differ ONLY in whether a successor announces during their
/// silence. The claim is a CONDITION, not a wall: the replaced watcher must
/// reach `Vanished`, and at the instant it does the other must not have, because
/// nothing the other can observe reaches a verdict inside this arm's ceiling.
/// See [`SHRINK_GRACES`] for why that is load-proof where the previous
/// count-ratio formulation was not.
///
/// The two wall bounds in the loop are different in KIND and neither is a
/// response-time claim: [`SHRINK_LIVENESS_CEILING`] catches a verdict that never
/// ARRIVES, and [`SHRINK_GRACES`]`.vanish` guards the arm's own inference (past
/// it, a verdict can no longer be attributed to the successor route) and fails
/// as a harness fault.
///
/// Deleting the `replaced_by_a_successor` route from
/// `classify_run_signal` leaves the replaced watcher no route at all, and the
/// liveness ceiling fires. Dropping that route's `successor_seen` conjunct — or
/// `is_successor_record`'s `graph_name` conjunct — instead ends BOTH watchers,
/// which the per-observation `ended == None` assert catches.
#[test]
fn a_successor_reaches_the_verdict_on_the_shorter_route_than_silence_alone() {
    let config = cerulion_core::testing::iceoryx_test_config();
    let replaced_id = 0x7777_8888_9999_aaaa_bbbb_cccc_dddd_eeee;
    let alone_id = 0x8888_9999_aaaa_bbbb_cccc_dddd_eeee_ffff;

    // Two runs of DIFFERENT graphs, so only one of them can be "replaced": a
    // successor is same-graph by definition, and using one graph name for both
    // would let the restart of one look like a successor of the other.
    let replaced_run =
        RunHandle::publish_on_config(&config, record(replaced_id, "will_restart")).expect("run");
    let alone_run =
        RunHandle::publish_on_config(&config, record(alone_id, "no_successor")).expect("run");

    let mut on_replaced =
        RunWatcher::open_on_config_with_graces(&config, replaced_id, SHRINK_GRACES)
            .expect("watcher");
    let mut on_alone =
        RunWatcher::open_on_config_with_graces(&config, alone_id, SHRINK_GRACES).expect("watcher");
    observe_until_live(&mut on_replaced, "the replaced watcher");
    observe_until_live(&mut on_alone, "the alone watcher");

    // Anchored BEFORE the runs die, so it is a strict OVER-estimate of the
    // silence either watcher has measured (each anchors at its own last drain,
    // which is earlier still only by the microseconds these two drops take).
    // Conservative in the direction that matters for the validity guard below.
    let silence_began = Instant::now();

    // BOTH runs die at the same instant. Only the first is replaced. The
    // successor stays up for the whole comparison, which is also what holds the
    // machine-wide publisher count above zero and so forecloses the exact
    // zero-writer route for BOTH watchers.
    drop(replaced_run);
    drop(alone_run);
    let successor =
        RunHandle::publish_on_config(&config, record(0x1234, "will_restart")).expect("successor");

    // Drive both on ONE loop so they see the same wall. Nothing here counts
    // iterations: the loop ends on the replaced watcher's VERDICT, and what is
    // asserted is that the other watcher has not reached one.
    let (replaced_verdict, alone_meanwhile) = loop {
        let replaced_obs = on_replaced.observe();
        if replaced_obs.ended.is_some() {
            // THE VALIDITY GUARD. The bound checked here is
            // NOT the liveness ceiling: a verdict that arrives late is ACCEPTED,
            // because a starved runner legitimately delivers one late and
            // rejecting it would re-arm the exact load-inversion class this arm
            // was rewritten to escape. What a verdict cannot survive is the
            // observation loop having gone quiet for longer than the SILENCE
            // GRACE, because past that point `quiet_past_the_grace` fires for
            // both watchers and the attribution below — "only the successor
            // route can have produced this" — is no longer sound. So the guard
            // is on the arm's INFERENCE, not on the product's response time, and
            // it fails as the harness fault it is. At 2000x the replaced grace
            // it cannot be tripped by load; it is here so that if it somehow is,
            // the arm says what really happened instead of blaming the product.
            let waited = silence_began.elapsed();
            assert!(
                waited < SHRINK_GRACES.vanish,
                "HARNESS (not a product regression): the verdict arrived after {waited:?}, past \
                 the {vanish:?} silence grace both watchers run under — so it can no longer be \
                 attributed to the successor route, and the comparison below would be \
                 meaningless. This loop was starved for over ten minutes; re-run it",
                vanish = SHRINK_GRACES.vanish,
            );
            // The other watcher is read AFTER the verdict lands, never before:
            // that hands it strictly MORE time to have ended, so the assertions
            // below cannot be an artefact of polling order.
            break (replaced_obs, on_alone.observe());
        }
        let alone_obs = on_alone.observe();
        assert_eq!(
            alone_obs.ended,
            None,
            "the watcher with NO successor must not reach a verdict here: its silence grace \
             ({vanish:?}) is untouched and a live successor holds the publisher count above zero, \
             so the only route left to it is the successor one — which belongs to a run nothing \
             replaced (signal {signal:?}, successor_seen={seen})",
            vanish = SHRINK_GRACES.vanish,
            signal = alone_obs.signal,
            seen = alone_obs.successor_seen,
        );
        // LIVENESS, checked here and deliberately ONLY here — on the iterations
        // that produced no verdict, which is the shape of "it never arrives".
        // The verdict branch above does not repeat it: see
        // [`SHRINK_LIVENESS_CEILING`] for why a late-but-arriving verdict is
        // accepted rather than failed.
        assert!(
            silence_began.elapsed() < SHRINK_LIVENESS_CEILING,
            "the REPLACED watcher never reached a verdict at all within \
             {SHRINK_LIVENESS_CEILING:?}. With a {vanish:?} silence grace and a successor holding \
             the publisher count above zero, the successor route is the ONLY route it has — so \
             this is exactly what a deleted or inert `replaced_by_a_successor` looks like",
            vanish = SHRINK_GRACES.vanish,
        );
        std::thread::sleep(OBSERVE_SPACING);
    };

    assert_eq!(
        replaced_verdict.ended,
        Some(RunEnded::Vanished),
        "the replaced run ended, and under these graces it can only have been the successor route \
         that said so"
    );
    assert!(
        replaced_verdict.successor_seen,
        "PRECONDITION: the successor must have been HEARD over the wire — without that the \
         verdict above came from somewhere else and this arm says nothing about the successor \
         route"
    );
    // THE COMPARISON. Not a wall and not a count: at the instant one watcher
    // gave up, the other — same code, same graces, same loop, same stimulus but
    // for the successor — has not.
    assert_eq!(
        alone_meanwhile.ended,
        None,
        "a SUCCESSOR is positive evidence and must shorten the wait: the replaced watcher reached \
         its verdict while the one with no successor is still inside its {vanish:?} grace. Both \
         ending together means the successor route fired for a run nothing replaced, or is inert \
         and both took the silence route",
        vanish = SHRINK_GRACES.vanish,
    );
    assert_eq!(
        alone_meanwhile.signal,
        RunSignal::Waiting,
        "and it is not merely unconfirmed — with its grace untouched and a live writer on the \
         registry, its reading makes NO claim at all"
    );
    assert!(
        !alone_meanwhile.successor_seen,
        "a run of a DIFFERENT graph is not a successor — `is_successor_record`'s graph_name \
         conjunct, over the wire"
    );

    // ANTI-VACUITY for those `None`s: the alone watcher is a WORKING watcher
    // held only by its grace, not a wedged one. Drop the successor and the
    // machine-wide publisher count reaches zero, which is the exact, grace-free
    // absence route — so it reaches its own verdict at once.
    drop(successor);
    assert_eq!(
        observe_until_ended(&mut on_alone, "the alone watcher"),
        RunEnded::Vanished,
        "the no-successor watcher must reach a verdict once absence becomes EXACT — otherwise \
         'it had not ended yet' above is satisfied by a watcher that never ends anything"
    );

    // Both verdicts are the same OUTCOME: `Replaced` is deliberately not a third
    // terminal variant, because the decision is "my run ended" either way.
    assert_eq!(on_replaced.ended(), Some(RunEnded::Vanished));
    assert_eq!(on_alone.ended(), Some(RunEnded::Vanished));
}

/// The RUN has no relationship to its watchers. A consumer dying
/// — the recorder being SIGKILLed — must not disturb the run at all.
///
/// Structural by design (the registry is one-way: a run publishes and reads
/// nothing back), and this is what keeps it so.
#[test]
fn a_watcher_that_dies_does_not_disturb_the_run_it_was_watching() {
    let config = cerulion_core::testing::iceoryx_test_config();
    let run_id = 0x6666_7777_8888_9999_aaaa_bbbb_cccc_dddd;
    let want = record(run_id, "survivor");
    let _handle = RunHandle::publish_on_config(&config, want.clone()).expect("run");

    {
        let mut watcher =
            RunWatcher::open_on_config_with_grace(&config, run_id, TEST_GRACE).expect("watcher");
        observe_until_live(&mut watcher, "the survivor arm");
        // The watcher dies here — the recorder's process is gone.
    }

    // The run is untouched: still live, still announcing itself, byte for byte.
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let gather = gather_runs_on_config(&config, RUN_GATHER_WINDOW).expect("gather");
        if gather.records == vec![want.clone()] {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "the run must still be announcing itself after its watcher died — got {:?}",
            gather.records
        );
    }

    // A NEW watcher binds to the same run and finds it live — so the run's
    // ability to be watched survived too, not just its record.
    let mut rebound =
        RunWatcher::open_on_config_with_grace(&config, run_id, TEST_GRACE).expect("rebind");
    observe_until_live(&mut rebound, "the rebound watcher");
    assert_eq!(rebound.ended(), None);
}
