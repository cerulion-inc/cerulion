// SPDX-License-Identifier: AGPL-3.0-only
//! The BEHAVIOURAL pin for the node-death producer's three failure
//! branches — no transport, the trigger channel will not open, the request will
//! not publish.
//!
//! # What this file exists to stop
//!
//! A review raised all three from `debug!` to `warn!`, because a robot
//! binary enables `tracing/release_max_level_info`: a `debug!` there does not
//! exist at ANY `RUST_LOG`, so a node died, no capture was made for it, and the
//! operator had no evidence of the second fact. That flip shipped with no test
//! (the class of defect where an unpinned level is free to regress and
//! the regression is invisible exactly where it costs the most).
//!
//! Every predicate here therefore matches the LEVEL TOKEN as well as the
//! message. A text-only filter is not enough: revert a `warn!` to
//! `debug!` and the wording, the fields and the counters are all still there.
//!
//! # Why the branches were unreachable, and what makes them reachable
//!
//! A test process always resolves a transport (the runtime falls back to the
//! process singleton), an isolated iceoryx2 namespace always opens, and a
//! publish onto a live service always succeeds. `flashback::fault_injection` is
//! the seam that makes each branch reachable; it is thread-local and RAII, so
//! these arms need no `#[serial]` for the fault's sake and a panicking arm
//! cannot leak its fault into whatever runs next on the same libtest worker.
//!
//! # Two of the three run through a lifted function, and that is not a shortcut
//!
//! The channel-open and publish branches live in the body of a DETACHED thread
//! in production. `tracing-test`'s `logs_assert` filters the captured buffer by
//! the test function's SPAN NAME, and that span is entered THREAD-LOCALLY — an
//! event emitted on a spawned thread is captured WITHOUT the prefix and every
//! line is filtered out, so the assertion reads `got []` while the warn
//! genuinely fired. The fix is the harness inversion this repo has used before:
//! the thread body is now the named `GraphRuntime::publish_node_death_captures`
//! and these arms drive it, unchanged, on the test thread. It is the SAME
//! function production spawns — not a copy that could drift green.
//!
//! The fourth branch, a failed `std::thread::Builder::spawn`, is deliberately
//! NOT pinned: reaching it needs the process to be out of threads, which is a
//! process-global condition no parallel-safe test may create. It is one `warn!`
//! beside three that are pinned.
//!
//! # The kill switch is process-global, so these arms pin it
//!
//! `request_node_death_captures` reads `CERULION_FLASHBACK` before it does
//! anything else — an operator who turned Flashback off must not have a dead
//! node open a channel on their behalf — so an ambient `off` would make the
//! no-transport arm assert against a function that returned before logging at
//! all, reporting `got 0 — all lines: []`. Each arm pins the switch on for its
//! body, under a FILE-LOCAL lock because the pin itself is a process-global
//! write.
//!
//! The CLI-side twin (`graph_cmd::flashback_fault_reporting_tests`) takes the
//! CRATE-WIDE `test_env` lock instead, because its binary also contains a test
//! that sets the switch to `off`. The two scopes differ for a real reason and
//! it is spelled out here so a reader does not re-derive it — or, worse, copy
//! the narrower one into a lib-test context, which is exactly the bug that put
//! this note here (CI Linux shard 0, one arm, `all lines: []`).
//!
//! Otherwise parallel-safe: `build_for_test` mints a per-test SHM root, every
//! graph carries a unique prefix, and the fault switch is thread-local.

#![cfg(unix)]

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{mpsc, Arc};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::flashback::channel::FlashbackResponder;
use cerulion_core::flashback::fault_injection::{self, FlashbackFault};
use cerulion_core::flashback::trigger::CaptureRequest;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::{ClosureNodeEntry, MacroPolicy, NodeEntry, NodeInfo};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::transport::{TransportConfig, TransportManager};
use indexmap::IndexMap;
use tracing_test::traced_test;

/// Pins `CERULION_FLASHBACK` ON for the body, restoring the prior value on drop,
/// and holds the file-local lock that makes that write safe.
///
/// Returned as `(switch, lock)` — tuple elements drop FRONT TO BACK, so the
/// restore runs while the lock is still held. Reversed, the restore would write
/// the variable after releasing the lock, i.e. while another arm is already
/// reading it.
fn pin_flashback_on() -> (SwitchGuard, std::sync::MutexGuard<'static, ()>) {
    static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
    // Poison-tolerant: a panicking arm must fail THAT arm, never convert every
    // later one into a poisoned-lock panic naming the wrong test.
    let lock = LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let prior = std::env::var(cerulion_core::flashback::FLASHBACK_ENV).ok();
    std::env::set_var(cerulion_core::flashback::FLASHBACK_ENV, "on");
    (SwitchGuard(prior), lock)
}

/// Restores whatever `CERULION_FLASHBACK` held before [`pin_flashback_on`].
struct SwitchGuard(Option<String>);

impl Drop for SwitchGuard {
    fn drop(&mut self) {
        match self.0.take() {
            Some(v) => std::env::set_var(cerulion_core::flashback::FLASHBACK_ENV, v),
            None => std::env::remove_var(cerulion_core::flashback::FLASHBACK_ENV),
        }
    }
}

/// A process-unique prefix, so parallel tests never collide on a topic name.
fn unique_prefix(tag: &str) -> String {
    static N: AtomicU64 = AtomicU64::new(0);
    format!(
        "{tag}{}_{}",
        std::process::id() % 100_000,
        N.fetch_add(1, Ordering::Relaxed)
    )
}

// ---------------------------------------------------------------------------
// Level + field predicates
//
// Both read WHOLE WHITESPACE TOKENS rather than substrings, which is the
// whole-token discipline and it is load-bearing twice over: `tracing-test` renders
// the test function's own name into every line, so a bare `line.contains("WARN")`
// would be satisfied by a test called `..._warns_...` no matter what level the
// event carried; and a bare `line.contains("subject=")` is satisfied by prose
// that merely names the field.
// ---------------------------------------------------------------------------

/// The level token a captured line carries, or `None` if it carries none.
///
/// Fails CLOSED — an unparseable line matches no level, which ZEROES a count.
/// That is safe for every PRESENCE and COUNT oracle here (they fail rather than
/// pass), and every ABSENCE guard in this file is paired, in the same test, with
/// a positive count over the same capture that would fail first.
fn level_of(line: &str) -> Option<&str> {
    line.split_whitespace()
        .find(|t| matches!(*t, "TRACE" | "DEBUG" | "INFO" | "WARN" | "ERROR"))
}

/// Lines at `level` whose message contains `marker`.
fn lines_at<'a>(lines: &'a [&'a str], level: &str, marker: &str) -> Vec<&'a str> {
    lines
        .iter()
        .filter(|l| level_of(l) == Some(level) && l.contains(marker))
        .copied()
        .collect()
}

/// `true` when `line` carries `key=value` as a whole whitespace token.
fn has_field(line: &str, key: &str, value: &str) -> bool {
    let want = format!("{key}={value}");
    line.split_whitespace().any(|t| t == want)
}

/// The marker of the "no transport" branch.
const NO_TRANSPORT: &str = "no transport for the node-death trigger";
/// The marker of the "channel will not open" branch.
const NO_CHANNEL: &str = "could not open the trigger channel for a node death";
/// The marker of the "request will not publish" branch.
const NO_PUBLISH: &str = "could not publish a node-death capture request";

/// A one-node graph whose single `Period(10)` CLOSURE panics from tick
/// `panic_from` onwards.
///
/// A closure rather than a `#[cerulion_node]` node for the reason
/// `node_death_trigger_iox2_test` gives: the macro wraps the body in
/// loan/`try_view` machinery that can skip it, which muddies "did it panic".
fn build_panicking_runtime(prefix: &str, panic_from: u64) -> GraphRuntime {
    let ticks = Arc::new(AtomicU64::new(0));
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "flashback_fault".to_string(),
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
    let entry = ClosureNodeEntry::new(info, move |_ctx| {
        let this_tick = ticks.fetch_add(1, Ordering::Relaxed) + 1;
        if this_tick >= panic_from {
            panic!("flashback_fault deliberate panic at tick {this_tick}");
        }
        Ok(())
    })
    .with_label("panic_node");
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("panicker".to_string(), Box::new(entry));

    GraphRuntime::build_for_test(config, factories, Arc::new(VirtualClock::new()), 8)
        .expect("build the panicking graph")
}

/// A per-test transport, for the arms that drive the publisher directly.
fn isolated_manager() -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig::default(),
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("an isolated transport manager")
}

/// Two node-death requests, so the per-request arms can tell them apart.
fn two_requests() -> Vec<CaptureRequest> {
    vec![
        CaptureRequest::panic_disable("alpha", "deliberate (panic)"),
        CaptureRequest::panic_disable("beta", "deliberate (panic)"),
    ]
}

// ---------------------------------------------------------------------------
// Branch 1: no transport
// ---------------------------------------------------------------------------

/// **A node dies with no transport for the trigger → exactly one WARN.**
///
/// Driven through the REAL production chain: a real graph, a real panic, a real
/// entry-mutex poison, the real step boundary calling the real
/// `report_node_deaths`. The fault only removes the transport the branch tests
/// for — nothing else about the run is synthetic.
///
/// The paired `error!` assertion is not decoration: it pins the review
/// SPLIT, where the diagnosis ("a node has STOPPED") was hoisted ABOVE the
/// transport gate so that losing the capture cannot also lose the finding. A
/// regression that moved it back under the gate would leave this arm's warn
/// intact and silently delete the only line that names the dead node.
#[test]
#[traced_test]
fn a_node_death_with_no_transport_warns_that_no_capture_will_be_made() {
    let _switch = pin_flashback_on();
    let _fault = fault_injection::arm(FlashbackFault::NoTransport);
    let mut runtime = build_panicking_runtime(&unique_prefix("fbf_notrans"), 3);
    runtime.node_death_ledger_for_test().arm();

    for _ in 0..5 {
        runtime.step(Duration::from_millis(10));
    }
    assert_eq!(
        runtime
            .node_handle("panicker")
            .expect("the node handle")
            .panic_count(),
        1,
        "PRECONDITION: the node really died"
    );

    logs_assert(|lines: &[&str]| {
        let warns = lines_at(lines, "WARN", NO_TRANSPORT);
        if warns.len() != 1 {
            return Err(format!(
                "expected EXACTLY one WARN naming the missing transport, got {} — all lines: {:?}",
                warns.len(),
                lines
            ));
        }
        if !warns[0].contains("no capture will be made") {
            return Err(format!(
                "the warn must say what was LOST, not only that a lookup failed: {}",
                warns[0]
            ));
        }
        let diagnosis = lines_at(lines, "ERROR", "a node has STOPPED");
        if diagnosis.len() != 1 {
            return Err(format!(
                "the DIAGNOSIS must survive the lost capture: expected one ERROR naming the \
                 stopped node, got {}",
                diagnosis.len()
            ));
        }
        Ok(())
    });
}

// ---------------------------------------------------------------------------
// Branch 2: the trigger channel will not open
// ---------------------------------------------------------------------------

/// **The trigger channel will not open → exactly one WARN for the whole batch.**
///
/// "Exactly one" is the load-bearing half and it is a property of the design: a
/// batch shares ONE open, so a channel that will not open costs one line however
/// many nodes died. A regression that opened per request would turn a cascading
/// graph's death storm into a line storm — the flood class this producer's
/// batching exists to avoid.
#[test]
#[traced_test]
fn a_trigger_channel_that_will_not_open_warns_once_for_the_whole_batch() {
    let _switch = pin_flashback_on();
    let manager = isolated_manager();
    let _fault = fault_injection::arm(FlashbackFault::ChannelOpen);
    GraphRuntime::publish_node_death_captures_for_test(&manager, &two_requests());

    logs_assert(|lines: &[&str]| {
        let warns = lines_at(lines, "WARN", NO_CHANNEL);
        if warns.len() != 1 {
            return Err(format!(
                "expected EXACTLY one WARN for a two-request batch that shares one open, got {} \
                 — all lines: {:?}",
                warns.len(),
                lines
            ));
        }
        if !warns[0].contains("no capture will be made") {
            return Err(format!("the warn must say what was LOST: {}", warns[0]));
        }
        // Nothing published, so the publish branch must be silent — otherwise
        // "exactly one" above could be satisfied by the wrong line.
        let published = lines_at(lines, "WARN", NO_PUBLISH);
        if !published.is_empty() {
            return Err(format!(
                "a channel that never opened cannot have failed to publish: {published:?}"
            ));
        }
        Ok(())
    });
}

// ---------------------------------------------------------------------------
// Branch 3: the request will not publish
// ---------------------------------------------------------------------------

/// **EVERY unpublishable request warns, under its OWN subject.**
///
/// The channel opens for real here — only the publish is injected — so this arm
/// also proves the open path is untouched by the seam.
///
/// Two requests, two warns, each carrying its own `subject=`. That is the pin
/// for a review correction which has no other test: the batch used to
/// report only its LAST entry, and the loss was invisible because the recorder's
/// gate coalesces a burst into one capture — an operator would read one node
/// named where five had died. A per-request assertion is the only thing that
/// separates "reported the batch" from "reported the batch's last member".
#[test]
#[traced_test]
fn every_unpublishable_node_death_request_warns_under_its_own_subject() {
    let _switch = pin_flashback_on();
    let manager = isolated_manager();
    let _fault = fault_injection::arm(FlashbackFault::Publish);
    GraphRuntime::publish_node_death_captures_for_test(&manager, &two_requests());

    logs_assert(|lines: &[&str]| {
        let warns = lines_at(lines, "WARN", NO_PUBLISH);
        if warns.len() != 2 {
            return Err(format!(
                "expected one WARN PER REQUEST (2), got {} — all lines: {:?}",
                warns.len(),
                lines
            ));
        }
        for subject in ["alpha", "beta"] {
            if !warns.iter().any(|l| has_field(l, "subject", subject)) {
                return Err(format!(
                    "no warn carried `subject={subject}` as a field — a batch must not report \
                     only its last member: {warns:?}"
                ));
            }
        }
        // The channel really did open: the open branch stayed silent.
        let opened = lines_at(lines, "WARN", NO_CHANNEL);
        if !opened.is_empty() {
            return Err(format!(
                "the seam must inject only the publish, not the open: {opened:?}"
            ));
        }
        Ok(())
    });
}

// ---------------------------------------------------------------------------
// The anti-tautology control
// ---------------------------------------------------------------------------

/// **A healthy publish writes NONE of the three lines.**
///
/// Without this the three arms above would pass a reporter that warned
/// unconditionally. Its scope is exactly that and no wider: it cannot rescue
/// the "exactly N" counts, which each drive their own branch.
///
/// The positive half — a real recorder really receives both requests — is what
/// keeps the absence guard from being vacuous, and it makes this arm a
/// no-inert-shipping proof for the lifted function as well: the code these tests
/// drive is the code that delivers a capture request on a healthy robot.
///
/// # The recorder drains from a HELPER thread, and it has to
///
/// The emitting code must stay on the TEST thread or `logs_assert` sees
/// nothing, so the observer is what moves. It also cannot simply drain after
/// the publisher returns: iceoryx2 reclaims a departing publisher's unread
/// samples, so a requester dropped before its recorder's next drive pass takes
/// its own request back out of the queue. That is the MEASURED hazard
/// `request_batch_and_linger` exists for, and draining afterwards models a
/// recorder that was never there. So the helper drains on a ~10 ms loop for the
/// whole linger — which is exactly what a real recorder does.
#[test]
#[traced_test]
fn a_healthy_node_death_publish_writes_none_of_the_three_warns() {
    let _switch = pin_flashback_on();
    let manager = isolated_manager();
    let stop = Arc::new(AtomicBool::new(false));
    let (ready_tx, ready_rx) = mpsc::channel::<()>();
    let (got_tx, got_rx) = mpsc::channel::<Vec<String>>();

    let recorder_manager = Arc::clone(&manager);
    let recorder_stop = Arc::clone(&stop);
    let recorder = std::thread::spawn(move || {
        // Opened BEFORE the publish is allowed to start (the `ready` handshake
        // below): this channel keeps no history for a late subscriber, so a
        // request published first is not missed by timing, it is structurally
        // unreachable.
        let responder = FlashbackResponder::open_on_manager(&recorder_manager, "recorder")
            .expect("the responder opens");
        ready_tx.send(()).expect("announce readiness");
        let mut got = Vec::new();
        let drain = |got: &mut Vec<String>| {
            got.extend(
                responder
                    .drain_requests()
                    .into_iter()
                    .map(|f| f.request.subject),
            );
        };
        while !recorder_stop.load(Ordering::Relaxed) {
            drain(&mut got);
            std::thread::sleep(Duration::from_millis(10));
        }
        drain(&mut got);
        got_tx.send(got).expect("hand the drained subjects back");
    });

    ready_rx
        .recv_timeout(Duration::from_secs(30))
        .expect("the recorder came up");
    GraphRuntime::publish_node_death_captures_for_test(&manager, &two_requests());
    stop.store(true, Ordering::Relaxed);
    let mut subjects = got_rx
        .recv_timeout(Duration::from_secs(30))
        .expect("the recorder handed its drain back");
    recorder.join().expect("the recorder thread finished");

    subjects.sort();
    subjects.dedup();
    assert_eq!(
        subjects,
        vec!["alpha".to_string(), "beta".to_string()],
        "PRECONDITION (and the no-inert-shipping proof): a healthy publish really delivers \
         BOTH requests to a recorder — without this the silence below proves nothing"
    );

    logs_assert(|lines: &[&str]| {
        for marker in [NO_TRANSPORT, NO_CHANNEL, NO_PUBLISH] {
            let noisy = lines_at(lines, "WARN", marker);
            if !noisy.is_empty() {
                return Err(format!(
                    "a healthy publish must write no failure warn, got {noisy:?}"
                ));
            }
        }
        Ok(())
    });
}
