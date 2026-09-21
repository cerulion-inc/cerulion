// SPDX-License-Identifier: AGPL-3.0-only
//! The NODE-DEATH trigger producer, driven through a REAL
//! `GraphRuntime`.
//!
//! # Why this file exists beside `scheduler_test`'s three bare-`Scheduler` arms
//!
//! The node-death producer has TWO mint sites, and each is unreachable from the other's test.
//! The scheduler's circuit-breaker edge can only be driven on a BARE `Scheduler`
//! (those arms own it), because through a `GraphRuntime` the FIRST panic poisons
//! the node's entry mutex and `tick()` is never re-entered — so
//! `consecutive_panics` sticks at 1 and the breaker never opens. That poison
//! transition is what a node death actually looks like on a robot, and it is what
//! this file pins.
//!
//! `rayon_fire_iox2_test`'s Test 5 already documents the poison-once-then-inert
//! behaviour in detail and asserts the ISOLATION half (the pool survives). It is
//! deliberately not extended here: this file asserts what the poison MEANS to
//! Flashback, which is a different claim about the same stimulus.
//!
//! # The replay firewall is BEHAVIOURAL here, not a comment
//!
//! `step()` — the seam a replay drives — reaches the same step boundary the live
//! loop does, so a naive producer would publish a live capture request during a
//! replay. It cannot: the ledger records nothing until something ARMS it, and the
//! only thing that arms it is `run_live`. Both halves of that are pinned below,
//! and each on its own would be satisfied by a broken implementation (a
//! never-arming `run_live` passes the firewall arm; an always-armed ledger passes
//! the arming arm).
//!
//! Parallel-safe: `build_for_test` mints a per-test SHM root and every graph
//! carries a unique prefix.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::clock::VirtualClock;
use cerulion_core::flashback::channel::FlashbackResponder;
use cerulion_core::flashback::trigger::TriggerKind;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{ClosureNodeEntry, MacroPolicy, NodeEntry, NodeInfo};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::scheduler::NodeDeathCause;
use indexmap::IndexMap;

/// A process-unique prefix, so parallel tests never collide on a topic name.
fn unique_prefix(tag: &str) -> String {
    static N: AtomicU64 = AtomicU64::new(0);
    format!(
        "{tag}{}_{}",
        std::process::id() % 100_000,
        N.fetch_add(1, Ordering::Relaxed)
    )
}

/// A one-node graph whose single `Period(10)` CLOSURE panics from tick
/// `panic_from` onwards.
///
/// A closure rather than a `#[cerulion_node]` node for the reason Test 5 gives:
/// the macro wraps the body in loan/`try_view` machinery that can skip it, which
/// muddies "did it panic". The closure body runs unconditionally per fire, so the
/// panic lands on exactly the tick this test names.
fn build_panicking_runtime(prefix: &str, panic_from: u64) -> (GraphRuntime, Arc<AtomicU64>) {
    let ticks = Arc::new(AtomicU64::new(0));
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "node_death".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "panicker".to_string(),
            node_type: "panic_node".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    };
    let info = NodeInfo::from_names(vec![], vec!["out".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 10 });
    let ticks_cb = Arc::clone(&ticks);
    let entry = ClosureNodeEntry::new(info, move |_ctx| {
        let this_tick = ticks_cb.fetch_add(1, Ordering::Relaxed) + 1;
        if this_tick >= panic_from {
            panic!("node_death deliberate panic at tick {this_tick}");
        }
        Ok(())
    })
    .with_label("panic_node");
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("panicker".to_string(), Box::new(entry));

    let clock = Arc::new(VirtualClock::new());
    let runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build the panicking graph");
    (runtime, ticks)
}

/// **THE runtime-side mint, end to end.** A node panics once, its entry mutex
/// poisons, and EXACTLY ONE `panic_disable` capture request reaches a recorder —
/// however long the run then continues.
///
/// The oracle is the CHANNEL rather than the ledger, and it has to be: the step
/// boundary DRAINS the ledger and hands the deaths to the publisher, so a
/// ledger-side assertion after the fact reads an empty ledger whether the
/// producer worked or not. Reading the recorder's own end is also the
/// no-inert-shipping proof — everything else in this file would pass against a
/// runtime that minted a death and published nothing.
///
/// The "exactly one" is the load-bearing half. The poison arm runs on EVERY later
/// fire — that is what the mutex poisoning means — so an unlatched report would
/// mint a capture request per fire, which on a 1 kHz graph is a thousand a second
/// and is precisely the spam the one-capture-per-incident rule forbids.
#[test]
fn a_poisoned_node_entry_publishes_one_capture_however_long_the_run_continues() {
    let prefix = unique_prefix("nd_poison");
    let (mut runtime, ticks) = build_panicking_runtime(&prefix, 3);
    // The recorder's end, opened BEFORE anything can publish: this channel keeps
    // no history for a subscriber that attaches later, so a request published
    // first is not missed by timing — it is structurally unreachable. It rides
    // the runtime's OWN per-test manager, which is exactly what
    // `resolve_doorbell_transport` hands the publisher.
    let responder = FlashbackResponder::open_on_manager(
        runtime.test_transport().expect("a per-test manager"),
        "test-recorder",
    )
    .expect("the responder opens");
    let ledger = runtime.node_death_ledger_for_test();
    // The live loop is what arms in production; this test drives the polled seam
    // deliberately, so it arms by hand. Which SEAM arms is pinned separately —
    // see the two firewall arms below.
    ledger.arm();

    // Ticks 1-2 run clean: nothing has died.
    runtime.step(Duration::from_millis(10));
    runtime.step(Duration::from_millis(10));
    assert_eq!(ticks.load(Ordering::Relaxed), 2, "two clean ticks ran");
    assert!(
        !ledger.has_pending(),
        "a healthy node must not be reported dead"
    );
    assert!(
        responder.drain_requests().is_empty(),
        "a healthy graph asks for no capture"
    );

    // Tick 3 panics → the entry mutex poisons.
    runtime.step(Duration::from_millis(10));
    assert_eq!(
        runtime
            .node_handle("panicker")
            .expect("the node handle")
            .panic_count(),
        1,
        "PRECONDITION: exactly one panic reached the scheduler — through a \
         GraphRuntime the poison stops `tick()` being re-entered"
    );

    // …and 20 more steps, every one of which takes the poisoned arm.
    for _ in 0..20 {
        runtime.step(Duration::from_millis(10));
    }
    let (panics, fires) = {
        let handle = runtime.node_handle("panicker").expect("the node handle");
        (handle.panic_count(), handle.fire_count())
    };
    assert_eq!(
        panics, 1,
        "PRECONDITION: still ONE panic — the node is inert, not re-panicking, so \
         the 20 steps above really did exercise the poisoned arm"
    );
    assert_eq!(
        fires, 23,
        "PRECONDITION: the scheduler kept FIRING it — the callback no-ops on the \
         poisoned lock, which is the arm that must not report 23 deaths"
    );
    assert_eq!(
        ticks.load(Ordering::Relaxed),
        3,
        "PRECONDITION: the node body ran three times and never again"
    );

    // The step boundary already TOOK the death and handed it to the publisher, so
    // the ledger is empty by construction — asserted, because a reader who
    // expected to find it there would otherwise conclude nothing was minted.
    assert!(
        !ledger.has_pending(),
        "the boundary drained the ledger — the oracle is the channel below"
    );
    assert_eq!(ledger.dropped(), 0, "nothing hit the ledger's cap");

    // The publish rides a DETACHED thread by design (opening an iceoryx2 node is
    // a cold ~620 ms operation and `request_and_linger` holds its ports for 1.5 s
    // — neither may happen on a live graph's step thread), so the wait for the
    // FIRST frame is a CONDITION wait, never a fixed sleep.
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut got = Vec::new();
    while Instant::now() < deadline && got.is_empty() {
        got.extend(responder.drain_requests());
        if got.is_empty() {
            std::thread::sleep(Duration::from_millis(20));
        }
    }
    assert!(
        !got.is_empty(),
        "the dead node's capture request must reach a recorder"
    );

    // Then keep stepping and draining for LONGER than the linger, so the window
    // covers every frame this one death can ever produce.
    //
    // The oracle is the count of DISTINCT REQUEST IDS, not of frames.
    // `request_and_linger` deliberately RE-PUBLISHES under the SAME id every
    // `FLASHBACK_REQUEST_RETRY_INTERVAL` (300 ms) for `FLASHBACK_UNWATCHED_LINGER`
    // (1.5 s), because this channel keeps no history for a subscriber that
    // attaches late and iceoryx2 reclaims a departing publisher's unread samples.
    // Those retries are ONE ask being delivered — the recorder's gate coalesces
    // them on the id — while a second ask carries a second id. So "reported once"
    // is a claim about ids, and asserting on frames instead would fail against the
    // channel working exactly as designed.
    let window = Instant::now() + Duration::from_secs(3);
    while Instant::now() < window {
        runtime.step(Duration::from_millis(10));
        got.extend(responder.drain_requests());
        std::thread::sleep(Duration::from_millis(25));
    }
    let ids: std::collections::BTreeSet<u64> = got.iter().map(|f| f.request_id).collect();
    assert_eq!(
        ids.len(),
        1,
        "EXACTLY ONE ask for 300+ fires of a dead node — got {} frames carrying \
         {} distinct ids: {ids:?}",
        got.len(),
        ids.len()
    );
    // HAND oracle on the vocabulary, applied to EVERY frame: a retry carrying a
    // different subject would be a different ask wearing one id.
    for frame in &got {
        assert_eq!(frame.request.kind, TriggerKind::PanicDisable);
        assert_eq!(
            frame.request.subject, "panicker",
            "the subject is the NODE — two nodes dying are two conditions, two \
             deaths of one node are one"
        );
        assert!(
            frame
                .request
                .detail
                .contains(NodeDeathCause::EntryPoisoned.as_wire()),
            "the detail names WHICH death was observed: {}",
            frame.request.detail
        );
        assert!(!frame.request.pin, "a node-death capture is not pinned");
        assert_ne!(frame.request_id, 0, "a real minted request id");
    }
}

/// **THE REPLAY FIREWALL.** The polled `step()` seam never arms, so a node that
/// dies during a replay mints NOTHING.
///
/// Not "nothing is drained" — nothing is WRITTEN. That distinction is the whole
/// design: a replay reaches the identical step boundary, and the only thing
/// standing between it and a live capture request on `/__cerulion/flashback` is
/// that `record` returns on an unarmed ledger.
#[test]
fn a_polled_replay_style_run_never_arms_and_records_no_death() {
    let prefix = unique_prefix("nd_replay");
    let (mut runtime, ticks) = build_panicking_runtime(&prefix, 1);
    // The recorder's end, attached first — so "no request arrived" is a real
    // absence rather than a subscriber that was not there to hear one.
    let responder = FlashbackResponder::open_on_manager(
        runtime.test_transport().expect("a per-test manager"),
        "test-recorder",
    )
    .expect("the responder opens");
    let ledger = runtime.node_death_ledger_for_test();

    for _ in 0..10 {
        runtime.step(Duration::from_millis(10));
    }

    // The stimulus really happened — without this, "nothing recorded" is
    // satisfied by a graph whose node never ran.
    let handle = runtime.node_handle("panicker").expect("the node handle");
    assert_eq!(
        ticks.load(Ordering::Relaxed),
        1,
        "the body ran and panicked"
    );
    assert_eq!(handle.panic_count(), 1, "the panic reached the scheduler");
    assert!(handle.fire_count() >= 10, "the node kept being fired");

    assert!(
        !ledger.is_armed(),
        "the polled seam must never arm the ledger — that is the firewall"
    );
    assert!(!ledger.has_pending());
    assert!(ledger.take().is_empty(), "nothing was ever minted");

    // …and the CHANNEL agrees. Given a window an order of magnitude longer than
    // the headline arm needed for its own request to arrive — an "expect nothing"
    // budget, not a convergence wait.
    let deadline = Instant::now() + Duration::from_secs(2);
    while Instant::now() < deadline {
        runtime.step(Duration::from_millis(10));
        assert!(
            responder.drain_requests().is_empty(),
            "a replay must publish NO capture request"
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// The other half of the firewall: `run_live` DOES arm.
///
/// Without this, a `run_live` that forgot to arm would ship the node-death producer fully
/// inert on the one path it exists for, and the firewall arm above would still
/// pass — it asserts an absence.
///
/// The loop is entered with `running` already false, so this is the arm point
/// alone: `run_live` collects its external sources (none here), arms, and returns
/// without executing a step.
#[test]
fn run_live_arms_the_ledger_which_is_what_the_firewall_is_absence_of() {
    let prefix = unique_prefix("nd_live");
    let (mut runtime, _ticks) = build_panicking_runtime(&prefix, 100);
    let ledger = runtime.node_death_ledger_for_test();
    assert!(!ledger.is_armed(), "a freshly built runtime is unarmed");

    let running = std::sync::atomic::AtomicBool::new(false);
    runtime.run_live(&running).expect("run_live entry");

    assert!(
        ledger.is_armed(),
        "run_live is what declares a publisher exists — the node-death producer is inert on \
         every live run without it"
    );
}

/// A node that dies while the ledger is UNARMED is still reported once it arms.
///
/// # The hole this closes
///
/// The poison arm runs at FIRE RATE for the rest of a run, so it is latched on
/// the closure's own `bool`. A latch set UNCONDITIONALLY, before
/// the `record` call, is a hole — `NodeDeathLedger::record` returns `false` without
/// recording anything when the ledger is unarmed (the replay firewall, and the
/// zero-cost path). A node poisoned while unarmed would latch "reported" against
/// a ledger that had taken nothing, and because the node is dead and firing
/// forever by then, NOTHING would ever offer it again. The node-death producer would be
/// silently inert for that node for the whole run.
///
/// It is reachable on the shipping seams, not just in principle: the polled
/// `step()` path shares this callback with the live loop, and `run_live` is
/// documented as resumable — so a host that steps a graph (warm-up,
/// `trigger_external` + `step()`, `run_live_step_once_for_test`) and then enters
/// `run_live` walks straight into it.
///
/// The latch is set only when the ledger could ACTUALLY have taken the death:
/// `record(..) || is_armed()`. The second term preserves the latch's purpose — a
/// `false` from an ARMED ledger is a dedup or cap refusal, which must not be
/// re-offered at fire rate.
///
/// # The oracle is the CHANNEL, not the ledger
///
/// `step()`'s boundary DRAINS the ledger and hands the death to the publisher,
/// so a post-hoc `ledger.take()` reads empty whether the mint worked or not —
/// the trap this file's headline arm documents. The count is of DISTINCT REQUEST
/// IDS, not of frames: `request_and_linger` re-publishes under one id.
#[test]
fn a_death_that_happens_while_unarmed_is_still_reported_once_armed() {
    let prefix = unique_prefix("nd_latearm");
    let (mut runtime, ticks) = build_panicking_runtime(&prefix, 1);
    // Attached before anything can publish — this channel keeps no history for a
    // late subscriber, so "no request" is a real absence.
    let responder = FlashbackResponder::open_on_manager(
        runtime.test_transport().expect("a per-test manager"),
        "test-recorder",
    )
    .expect("the responder opens");
    let ledger = runtime.node_death_ledger_for_test();

    // PHASE 1 — poison the node while the ledger is UNARMED.
    for _ in 0..5 {
        runtime.step(Duration::from_millis(10));
    }
    assert_eq!(
        ticks.load(Ordering::Relaxed),
        1,
        "the body ran and panicked"
    );
    assert_eq!(
        runtime
            .node_handle("panicker")
            .expect("the node handle")
            .panic_count(),
        1,
        "PRECONDITION: the panic reached the scheduler and poisoned the entry"
    );
    assert!(
        !ledger.is_armed(),
        "PRECONDITION: the polled seam has not armed the ledger"
    );
    assert!(
        responder.drain_requests().is_empty(),
        "PRECONDITION: an unarmed ledger asks for nothing — that is the firewall"
    );

    // PHASE 2 — the ledger arms (what `run_live` does at loop entry), and the
    // node keeps firing into its poisoned entry exactly as before.
    ledger.arm();

    // THE PIN. If the closure latched during phase 1 against a ledger that took
    // nothing, NOTHING would ever arrive and the death would be lost for the run.
    let mut ids = std::collections::BTreeSet::new();
    let deadline = Instant::now() + Duration::from_secs(10);
    while Instant::now() < deadline && ids.is_empty() {
        runtime.step(Duration::from_millis(10));
        for frame in responder.drain_requests() {
            assert_eq!(
                frame.request.kind,
                TriggerKind::PanicDisable,
                "a node death asks under the PanicDisable kind"
            );
            assert_eq!(
                frame.request.subject, "panicker",
                "…naming the node that died"
            );
            ids.insert(frame.request_id);
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    assert_eq!(
        ids.len(),
        1,
        "a node poisoned while UNARMED must still be reported once the ledger arms — the latch \
         may only be set when the ledger could actually have taken the death"
    );

    // …and it is reported ONCE, not once per fire: the latch's own job is intact.
    // Anti-tautology for the latch rule — a latch deleted outright would also satisfy
    // the assertion above.
    let settle = Instant::now() + Duration::from_secs(2);
    while Instant::now() < settle {
        runtime.step(Duration::from_millis(10));
        for frame in responder.drain_requests() {
            ids.insert(frame.request_id);
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    assert_eq!(
        ids.len(),
        1,
        "the death is minted once per TRANSITION; a dead node firing forever must not re-mint it. \
         Got {ids:?}"
    );
}

/// **The node this feature exists for: one that dies on its LAST input and is
/// never fired again.**
///
/// # The hole this closes
///
/// A node-death producer that records a death only where a LATER callback finds the entry
/// mutex poisoned misses this shape. That arm runs at fire rate — for a `Period` node, forever —
/// which is why every other arm in this file sees it. It never runs at all for a
/// `Data`-triggered node whose panic CONSUMED its last input: nothing publishes
/// to it again, so the scheduler never fires it again, so no later callback
/// exists to notice the poison. The run would end with a dead node, an operator with
/// no bag, and every assertion in this file still green.
///
/// That is not a corner. A node that dies once, on the frame that killed it, is
/// the shape a black box is FOR — a perception node that panics on the malformed
/// message that kills it, whose upstream then stops
/// because the graph is coming down.
///
/// # The shape
///
/// The producer is `Period` but publishes on its FIRST tick ONLY, so the
/// consumer is fired exactly once, panics, and is never fired again. The arm
/// asserts that ONE `panic_disable` request still reaches the recorder, and — the
/// half that makes it non-vacuous — that the consumer really was fired only once.
#[test]
fn a_node_that_dies_on_its_last_input_is_still_captured() {
    let prefix = unique_prefix("nd_lastinput");
    let publishes = Arc::new(AtomicU64::new(0));
    let consumer_fires = Arc::new(AtomicU64::new(0));

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "node_death_last_input".to_string(),
        prefix: prefix.clone(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "source".to_string(),
                node_type: "one_shot_source".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                ros2: None,
                id: "dier".to_string(),
                node_type: "dies_on_last_input".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "source/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };

    // ONE publish, ever. After it the consumer has no further trigger, which is
    // the whole point — a second publish would restore the poison arm and the
    // arm would pass for the wrong reason.
    let publishes_cb = Arc::clone(&publishes);
    let source_info = NodeInfo::from_names(vec![], vec!["out".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 10 });
    let source = ClosureNodeEntry::new(source_info, move |ctx| {
        if publishes_cb.load(Ordering::Relaxed) > 0 {
            return Ok(());
        }
        let publisher = ctx
            .publisher_mut("out")
            .expect("the source's 'out' publisher must be wired");
        let mut proxy = publisher.loan_proxy::<native_ros2_messages::geometry_msgs::Vector3>()?;
        proxy.x = 1.0;
        proxy.y = 2.0;
        proxy.z = 3.0;
        drop(proxy);
        publishes_cb.fetch_add(1, Ordering::Relaxed);
        Ok(())
    })
    .with_label("one_shot_source");

    let fires_cb = Arc::clone(&consumer_fires);
    let consumer_info = NodeInfo::from_names(vec!["inp".to_string()], vec![]).with_policy(
        MacroPolicy::DataTrigger {
            input_name: "inp".to_string(),
        },
    );
    let consumer = ClosureNodeEntry::new(consumer_info, move |_ctx| {
        fires_cb.fetch_add(1, Ordering::Relaxed);
        panic!("node_death: the frame that killed it");
    })
    .with_label("dies_on_last_input");

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("source".to_string(), Box::new(source));
    factories.insert("dier".to_string(), Box::new(consumer));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build the one-shot graph");

    let responder = FlashbackResponder::open_on_manager(
        runtime.test_transport().expect("a per-test manager"),
        "test-recorder",
    )
    .expect("the responder opens");
    runtime.node_death_ledger_for_test().arm();

    let mut ids = std::collections::BTreeSet::new();
    let deadline = Instant::now() + Duration::from_secs(10);
    while Instant::now() < deadline && ids.is_empty() {
        runtime.step(Duration::from_millis(10));
        for frame in responder.drain_requests() {
            assert_eq!(frame.request.kind, TriggerKind::PanicDisable);
            assert_eq!(frame.request.subject, "dier", "the node that died");
            ids.insert(frame.request_id);
        }
        std::thread::sleep(Duration::from_millis(20));
    }

    // PRECONDITIONS, asserted before the claim so a run that fired the consumer
    // repeatedly cannot pass this arm for the wrong reason.
    assert_eq!(
        publishes.load(Ordering::Relaxed),
        1,
        "PRECONDITION: the source publishes exactly ONCE"
    );
    assert_eq!(
        consumer_fires.load(Ordering::Relaxed),
        1,
        "PRECONDITION: the consumer is fired exactly ONCE — if it were fired again the poison \
         arm would run and this arm would pass for the wrong reason, proving nothing"
    );

    assert_eq!(
        ids.len(),
        1,
        "a node that panics on its LAST input is never fired again, so nothing later can notice \
         its poisoned entry — the death must be recorded at the panic itself"
    );

    // …and it stays ONE. The latch is shared between the panic site and the
    // poison site, so a node that somehow is fired again must not mint a second.
    let settle = Instant::now() + Duration::from_secs(2);
    while Instant::now() < settle {
        runtime.step(Duration::from_millis(10));
        for frame in responder.drain_requests() {
            ids.insert(frame.request_id);
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    assert_eq!(ids.len(), 1, "one death is ONE ask: {ids:?}");
}

/// **The SHIPPING node shape: a cdylib, whose panic comes back as a returned
/// `Err` and poisons nothing on the host.**
///
/// # The hole this closes
///
/// `DylibNodeEntry::tick` converts FFI code 2 (the cdylib's own `catch_unwind`
/// caught a panic) and code 3 (its `NODES` mutex is poisoned by an earlier one)
/// into `TransportError::NodeTickPanicked` — a RETURNED error, not an unwind.
/// Nothing crosses the host's entry guard, so:
///
/// * the ENTRY-POISONED arm never runs (the host mutex is never poisoned), and
/// * the callback returns cleanly, so `fire_node_into` sees `Ok(())` and resets
///   `consecutive_panics` — the circuit breaker never opens either.
///
/// So without host-side handling of that returned error a permanently-dead cdylib
/// node mints NO death by either site. Every node built by `cerulion node build` is
/// a cdylib, so the node-death producer would be inert on the
/// shape that actually ships — while every other arm in this file, all of which
/// use in-process closures that really unwind, stays green.
///
/// # Why a closure returning the variant is the faithful stimulus
///
/// `TransportError::NodeTickPanicked` is exactly what `DylibNodeEntry::tick`
/// hands back; the host cannot tell (and must not care) whether it came from a
/// real `.so` or from here. What is under test is the HOST's handling of a
/// returned panic-class error, and this drives that seam directly rather than
/// through a `.so` build the assertion would not otherwise depend on.
///
/// The node keeps FIRING — a cdylib does, since nothing is poisoned host-side —
/// so this pins the latch as well: forever-failing must still be ONE death.
#[test]
fn a_cdylib_style_panic_class_error_mints_exactly_one_death() {
    let prefix = unique_prefix("nd_cdylib");
    let fires = Arc::new(AtomicU64::new(0));

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "node_death_cdylib".to_string(),
        prefix: prefix.clone(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "dylib_node".to_string(),
            node_type: "panic_class_err".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    };

    let fires_cb = Arc::clone(&fires);
    let info = NodeInfo::from_names(vec![], vec!["out".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 10 });
    let entry = ClosureNodeEntry::new(info, move |_ctx| {
        fires_cb.fetch_add(1, Ordering::Relaxed);
        // EXACTLY what `DylibNodeEntry::tick` returns for FFI codes 2 and 3.
        Err(cerulion_core::TransportError::NodeTickPanicked {
            node_id: "dylib_node".to_string(),
            reason: "cdylib panic (FFI code 2)".to_string(),
        })
    })
    .with_label("panic_class_err");

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("dylib_node".to_string(), Box::new(entry));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build the cdylib-shaped graph");

    let responder = FlashbackResponder::open_on_manager(
        runtime.test_transport().expect("a per-test manager"),
        "test-recorder",
    )
    .expect("the responder opens");
    runtime.node_death_ledger_for_test().arm();

    let mut ids = std::collections::BTreeSet::new();
    let mut details = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(10);
    while Instant::now() < deadline && ids.is_empty() {
        runtime.step(Duration::from_millis(10));
        for frame in responder.drain_requests() {
            assert_eq!(frame.request.kind, TriggerKind::PanicDisable);
            assert_eq!(frame.request.subject, "dylib_node");
            details.push(frame.request.detail.clone());
            ids.insert(frame.request_id);
        }
        std::thread::sleep(Duration::from_millis(20));
    }

    assert!(
        fires.load(Ordering::Relaxed) > 1,
        "PRECONDITION: a cdylib node keeps being FIRED — nothing is poisoned host-side — which \
         is what makes the latch half of this arm meaningful"
    );
    assert_eq!(
        ids.len(),
        1,
        "a cdylib node whose panic comes back as a returned `NodeTickPanicked` must mint a \
         death: neither the entry-poison arm nor the circuit breaker can see it"
    );
    // The DETAIL must name the mechanism that was actually observed. Reporting a
    // host entry poison here would be false: nothing on the host is poisoned.
    assert!(
        details[0].contains(NodeDeathCause::CdylibPanicked.as_wire()),
        "the capture's detail must name the cdylib cause, not a host entry poison: {:?}",
        details[0]
    );

    // …and it stays ONE, though the node fails forever.
    let settle = Instant::now() + Duration::from_secs(2);
    while Instant::now() < settle {
        runtime.step(Duration::from_millis(10));
        for frame in responder.drain_requests() {
            ids.insert(frame.request_id);
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    assert_eq!(ids.len(), 1, "one death is ONE ask: {ids:?}");
}
