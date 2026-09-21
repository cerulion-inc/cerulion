// SPDX-License-Identifier: AGPL-3.0-only
//! The E-STOP capture trigger (Flashback producer 1).
//!
//! Two halves, and each one covers what the other structurally cannot.
//!
//! **The BEHAVIOURAL half** drives `EngageEstopVerb::execute_with_caller` — the
//! real handler, reached the way `cerud`'s ops server reaches it — against
//! recording doubles. It answers "is it asked, exactly once, for the right
//! caller" and, in the arm that matters most, "does an ask that FAILS still leave
//! the e-stop successful". A double is the only way to ask the second question:
//! a real transport failure is not something a test can reliably conjure, and the
//! guarantee ("a publish failure can never fail the e-stop") is about the verb's
//! control flow, not about iceoryx2.
//!
//! **The REAL-TRANSPORT half** drives the PRODUCTION ask (`TransportAsk`) over a
//! per-test SHM root and reads the request back off `/__cerulion/flashback` with
//! a real `FlashbackResponder` — the recorder's own end of the channel. This is
//! the no-inert-shipping proof: without it, every assertion in this file could
//! pass while the shipped producer published nothing at all.
//!
//! # What holds the two halves together
//!
//! A double could be substituted for the real ask in production and the
//! behavioural half would not notice. That is closed by the TYPE SYSTEM rather
//! than by a test: `EngageEstopVerb::with_capture_ask` — the only way to give the
//! verb an ask other than the production one — is `#[cfg(any(test, feature =
//! "test-seam"))]`, so a plain `cargo build -p cerulion_remoted` cannot construct
//! a silently-capture-less e-stop verb.
//!
//! # The ONE thing in this file that is not parallel-safe, and why it is a
//! MUTEX rather than a comment
//!
//! Transport isolation is per-test (`init_for_test` SHM roots, unique node
//! names), so no `#[serial]` and no `--test-threads=1`. The plane KILL SWITCH is
//! different in kind: `CERULION_FLASHBACK` is process-global, and the production
//! `TransportAsk::ask` READS it synchronously on the calling thread before it
//! spawns anything. So the kill-switch arm does not merely WRITE a global — it
//! writes one that a sibling arm's production code reads, and it holds the write
//! for a two-second "expect nothing" window.
//!
//! That is not hypothetical. Without the lock it is the failure this file shows: run
//! alone, `the_production_ask_publishes_a_real_estop_request_a_recorder_can_read`
//! passes in 0.04 s; run in the binary, the kill switch is `off` when its ask
//! reaches `plane_off()`, so it publishes NOTHING and spends its whole 20 s
//! deadline reading an empty channel — a failure that looks exactly like a broken
//! producer and points every diagnosis at the transport.
//!
//! [`ENV_LOCK`] is the guard: the two arms whose behaviour depends on that variable
//! take it for their whole body. A comment saying "this is the only test that
//! touches the env" is no substitute: it would be TRUE of the write and false of
//! the read.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::time::{Duration, Instant};

use cerud::lease::ControlLease;
use cerud::transport::CallerIdentity;
use cerud::verbs::VerbHandler;

use cerulion_core::flashback::channel::{FlashbackResponder, FLASHBACK_UNWATCHED_LINGER};
use cerulion_core::flashback::trigger::TriggerKind;
use cerulion_core::{TransportConfig, TransportManager};

use cerulion_remoted::flashback::{EstopCaptureAsk, TransportAsk};
use cerulion_remoted::pairing_verbs::EngageEstopVerb;

/// A process-unique suffix, so parallel tests never share an SHM root or a node
/// name.
fn unique_id() -> String {
    use std::sync::atomic::AtomicU64;
    static N: AtomicU64 = AtomicU64::new(0);
    format!(
        "{}_{}",
        std::process::id(),
        N.fetch_add(1, Ordering::Relaxed)
    )
}

/// A fresh per-test transport manager on its own SHM root (parallel-safe) —
/// the same helper shape `wire_plane_test.rs` uses.
fn test_manager(tag: &str) -> Arc<TransportManager> {
    let ix = cerulion_core::testing::iceoryx_test_config();
    TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("estopfb_{tag}_{}", unique_id()),
            ..Default::default()
        },
        ix,
    )
    .expect("init_for_test")
}

fn lease() -> Arc<Mutex<ControlLease>> {
    Arc::new(Mutex::new(ControlLease::with_default_window()))
}

/// Serializes the arms whose outcome depends on `CERULION_FLASHBACK` — one that
/// WRITES it and one whose production code READS it.
///
/// A file-local mutex rather than `#[serial]`: the serialization needed here is
/// between two named arms over one named variable, and `serial_test` is not a
/// dev-dep of this crate (the repo's other single-variable cases —
/// `completion_wiring_tests`, `client_e2e_test` — take the same shape).
///
/// POISON-TOLERANT (`into_inner`): a panic in one arm must fail THAT arm, never
/// convert every later arm into a poisoned-lock panic that names the wrong test.
fn env_lock() -> MutexGuard<'static, ()> {
    static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
    LOCK.get_or_init(|| Mutex::new(()))
        .lock()
        .unwrap_or_else(|e| e.into_inner())
}

/// A recording double: counts asks and remembers every caller it was asked for.
#[derive(Default)]
struct SpyAsk {
    asks: AtomicUsize,
    callers: Mutex<Vec<String>>,
}

impl SpyAsk {
    fn count(&self) -> usize {
        self.asks.load(Ordering::SeqCst)
    }
    fn callers(&self) -> Vec<String> {
        self.callers.lock().expect("spy").clone()
    }
}

impl EstopCaptureAsk for SpyAsk {
    fn ask(&self, by: &str) {
        self.asks.fetch_add(1, Ordering::SeqCst);
        self.callers.lock().expect("spy").push(by.to_string());
    }
}

/// A double that models the ask FAILING as badly as the trait permits.
///
/// The trait returns `()`, so the only failure a production ask can express
/// internally is one it swallows — which is the point. This double PANICS if
/// anything ever tries to read a result from it, and more usefully it simply does
/// nothing at all, which is exactly what a real `TransportAsk` does when the
/// channel cannot be opened.
#[derive(Default)]
struct SilentlyFailingAsk {
    asks: AtomicUsize,
}

impl EstopCaptureAsk for SilentlyFailingAsk {
    fn ask(&self, _by: &str) {
        self.asks.fetch_add(1, Ordering::SeqCst);
        // ... and reports nothing. A real failed publish logs and returns.
    }
}

// ── the behavioural half ────────────────────────────────────────────────────

#[test]
fn engaging_the_estop_asks_for_exactly_one_capture_naming_the_caller() {
    let spy = Arc::new(SpyAsk::default());
    let verb = EngageEstopVerb::with_capture_ask(lease(), spy.clone());

    // ANTI-TAUTOLOGY: nothing has been asked before the verb runs, so the
    // "exactly one" below is capable of failing in both directions.
    assert_eq!(spy.count(), 0, "nothing asked before the engage");

    let out = verb
        .execute_with_caller(
            &CallerIdentity::verified("operator-alpha"),
            &serde_json::json!({}),
        )
        .expect("the e-stop succeeds");

    // HAND oracle on the verb's own answer — unchanged by this producer.
    assert_eq!(out["engaged"], serde_json::json!(true));
    assert_eq!(out["by"], serde_json::json!("operator-alpha"));

    assert_eq!(spy.count(), 1, "exactly one capture asked per engage");
    assert_eq!(spy.callers(), vec!["operator-alpha".to_string()]);
}

#[test]
fn a_capture_ask_that_reports_nothing_still_leaves_the_estop_successful() {
    // THE safety pin. The e-stop is safety-critical and the capture is
    // best-effort, so the failure of the second must be invisible to the first.
    let failing = Arc::new(SilentlyFailingAsk::default());
    let verb = EngageEstopVerb::with_capture_ask(lease(), failing.clone());

    let out = verb
        .execute_with_caller(
            &CallerIdentity::verified("operator-beta"),
            &serde_json::json!({}),
        )
        .expect("a failed capture ask must not fail the e-stop");

    assert_eq!(out["engaged"], serde_json::json!(true));
    assert!(
        out["safe_frame"].is_string(),
        "the safe frame is still reported: {out}"
    );
    // ANTI-TAUTOLOGY: the failing ask really was reached — otherwise this test
    // would pass against a verb that had no producer wired at all, which is the
    // exact regression the whole file exists to prevent.
    assert_eq!(failing.asks.load(Ordering::SeqCst), 1);
}

#[test]
fn each_engage_asks_again_because_de_duplication_is_the_recorder_gates_job() {
    // `ControlLease::engage_estop` is idempotent and cannot tell a first engage
    // from a re-engage, so this producer deliberately does NOT try to. Three
    // engages ⇒ three asks; the recorder's trigger gate latches the regime and
    // applies the refractory floor, and `CaptureRequest::estop` pins ONE subject
    // for the whole robot so they coalesce into one bag there.
    //
    // Pinned rather than left implicit: a future edit that "helpfully" added an
    // edge check here would put a second, divergent copy of the anti-spam policy
    // in a crate that cannot see the gate's state.
    let spy = Arc::new(SpyAsk::default());
    let verb = EngageEstopVerb::with_capture_ask(lease(), spy.clone());

    for who in ["op-1", "op-2", "op-1"] {
        verb.execute_with_caller(&CallerIdentity::verified(who), &serde_json::json!({}))
            .expect("the e-stop succeeds");
    }

    assert_eq!(spy.count(), 3);
    assert_eq!(
        spy.callers(),
        vec!["op-1".to_string(), "op-2".to_string(), "op-1".to_string()],
        "every engage is reported with ITS OWN caller"
    );
}

// ── the real-transport half ─────────────────────────────────────────────────

#[test]
fn the_production_ask_publishes_a_real_estop_request_a_recorder_can_read() {
    // NO-INERT-SHIPPING. Everything above would pass against a producer that
    // published nothing; this drives the PRODUCTION `TransportAsk` and reads the
    // request off the channel with the recorder's own end.
    //
    // The lock is taken FIRST and held for the whole body: `TransportAsk::ask`
    // consults `CERULION_FLASHBACK` synchronously, so the kill-switch arm's
    // two-second `off` window would make this publish nothing at all.
    let _env = env_lock();
    // …and the PRECONDITION is asserted rather than assumed. Without it a
    // re-introduced race fails as a twenty-second empty-channel timeout that
    // reads exactly like a broken producer; with it, it fails in milliseconds
    // naming the real cause.
    assert!(
        !matches!(
            std::env::var("CERULION_FLASHBACK").ok().as_deref(),
            Some("off" | "0" | "false" | "no")
        ),
        "PRECONDITION: the capture plane must be ON for this arm — a sibling test \
         left CERULION_FLASHBACK={:?}, which makes the production ask return \
         before it publishes anything",
        std::env::var("CERULION_FLASHBACK").ok()
    );
    let mgr = test_manager("prod");
    // The responder is opened BEFORE the ask, and the order is the contract:
    // iceoryx2 pub/sub keeps no history for a subscriber that attaches later, so
    // a request published first is not merely missed by timing — it is
    // structurally unreachable.
    let responder =
        FlashbackResponder::open_on_manager(&mgr, "test-recorder").expect("responder opens");

    let ask = TransportAsk::on_manager(mgr.clone());
    let verb = EngageEstopVerb::with_capture_ask(lease(), Arc::new(ask));

    // Drain once first: the channel must be empty, or "exactly one request"
    // below could be satisfied by somebody else's frame.
    assert!(
        responder.drain_requests().is_empty(),
        "the channel starts empty"
    );

    verb.execute_with_caller(
        &CallerIdentity::verified("operator-gamma"),
        &serde_json::json!({}),
    )
    .expect("the e-stop succeeds");

    // The publish is on a DETACHED THREAD by design (the e-stop must not wait on
    // an iceoryx2 open), so this is a CONDITION wait, never a fixed sleep.
    //
    // It also keeps draining PAST the first non-empty read, through the
    // whole linger window. Stopping at the first frame made the "exactly one
    // request" assertion below unable to see a SECOND, differently-id'd ask
    // arriving later — the arm would have passed a producer that published two
    // asks for one e-stop, which is exactly what the coalescing arm added below
    // exists to forbid. The wait is still a CONDITION (a first frame must
    // arrive); only the settle is fixed, and it is spent once on a passing run.
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut got = Vec::new();
    while Instant::now() < deadline && got.is_empty() {
        got.extend(responder.drain_requests());
        if got.is_empty() {
            std::thread::sleep(Duration::from_millis(20));
        }
    }
    // …then drain through the rest of the linger (plus a margin), so a second
    // ask under a different id has somewhere to show up.
    let settle = Instant::now() + FLASHBACK_UNWATCHED_LINGER + Duration::from_millis(500);
    while Instant::now() < settle {
        got.extend(responder.drain_requests());
        std::thread::sleep(Duration::from_millis(20));
    }

    // DISTINCT REQUEST IDS, not frames. `request_and_linger` re-publishes under
    // ONE id every 300 ms for 1.5 s into a depth-64 queue, and the loop above
    // breaks on the first NON-EMPTY drain — so a stall longer than the republish
    // interval puts two frames in that first drain and an exact FRAME count
    // fails against the channel working exactly as designed. This is the trap
    // `50d1ba37e` documents and fixed the same way in
    // `node_death_trigger_iox2_test`.
    let ids: std::collections::BTreeSet<u64> = got.iter().map(|f| f.request_id).collect();
    assert_eq!(
        ids.len(),
        1,
        "exactly one e-stop REQUEST reached the recorder (it may arrive as several republished \
         frames under that one id): {ids:?}"
    );
    let frame = &got[0];
    // HAND oracle on the vocabulary — the kind, the robot-wide subject, and the
    // caller riding the DETAIL rather than the regime key.
    assert_eq!(frame.request.kind, TriggerKind::EStop);
    assert_eq!(frame.request.subject, "estop");
    assert!(
        frame.request.detail.contains("operator-gamma"),
        "the detail names the operator: {}",
        frame.request.detail
    );
    assert!(
        !frame.request.subject.contains("operator-gamma"),
        "the caller must never reach the regime key"
    );
    assert!(!frame.request.pin, "an e-stop capture is not pinned");
    assert_ne!(frame.request_id, 0, "a real minted request id");
}

#[test]
fn the_plane_kill_switch_stops_the_production_ask_before_it_opens_transport() {
    // `CERULION_FLASHBACK=off` means the operator turned the capture plane off.
    // An e-stop must then publish NOTHING — and, more importantly, must not open
    // an iceoryx2 node on their behalf.
    //
    // The env is process-global AND read by a sibling arm's production code, so
    // this test owns it for its body under BOTH `ENV_LOCK` (against the reader)
    // and an RAII guard (against a panic leaving it set). The lock is declared
    // FIRST so it drops LAST: locals drop in reverse declaration order, which is
    // what puts the env RESTORE inside the critical section.
    let _env_lock = env_lock();
    struct EnvGuard(Option<String>);
    impl Drop for EnvGuard {
        fn drop(&mut self) {
            match self.0.take() {
                Some(v) => std::env::set_var("CERULION_FLASHBACK", v),
                None => std::env::remove_var("CERULION_FLASHBACK"),
            }
        }
    }

    let mgr = test_manager("off");
    let responder =
        FlashbackResponder::open_on_manager(&mgr, "test-recorder").expect("responder opens");

    let _guard = EnvGuard(std::env::var("CERULION_FLASHBACK").ok());
    std::env::set_var("CERULION_FLASHBACK", "off");

    let verb =
        EngageEstopVerb::with_capture_ask(lease(), Arc::new(TransportAsk::on_manager(mgr.clone())));
    verb.execute_with_caller(
        &CallerIdentity::verified("operator-delta"),
        &serde_json::json!({}),
    )
    .expect("the e-stop succeeds with the plane off");

    // Give a publish that SHOULD NOT happen ample room to happen anyway. This
    // window is an "expect nothing" budget, not a convergence wait — the
    // positive arm above proves a real publish lands well inside it.
    let deadline = Instant::now() + Duration::from_secs(2);
    while Instant::now() < deadline {
        assert!(
            responder.drain_requests().is_empty(),
            "the kill switch must publish nothing"
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// A SECOND e-stop while the first ask is still lingering COALESCES into it
/// rather than spawning a second publisher thread.
///
/// # The hazard
///
/// Every ask spawns a detached thread that lives for
/// `FLASHBACK_UNWATCHED_LINGER` (1.5 s). Unbounded, a caller that engages the
/// e-stop in a loop — an automated safety loop re-arming, a bouncing bumper, the
/// paired caller the ops plane explicitly supports — spawns one thread per call
/// on the one daemon that must not fall over.
///
/// # Why COALESCING is the right bound, and not a cap
///
/// `CaptureRequest::estop` pins ONE robot-wide subject by design, so every
/// concurrent ask is the same regime key and the recorder's gate already folds
/// them into one capture. A second thread would re-publish the identical subject
/// the in-flight linger is already re-publishing. A fixed cap would admit N
/// identical asks before refusing — the same waste with a number in front of it.
///
/// The oracle is BOTH halves: the counter says the suppression happened, and the
/// CHANNEL says exactly one ask (one request id) reached the recorder. Without
/// the second half, an implementation that coalesced the thread but published
/// twice would pass.
#[test]
fn a_second_estop_inside_the_linger_coalesces_instead_of_spawning_another_publisher() {
    // See the production arm: `ask` reads `CERULION_FLASHBACK` synchronously.
    let _env = env_lock();
    assert!(
        !matches!(
            std::env::var("CERULION_FLASHBACK").ok().as_deref(),
            Some("off" | "0" | "false" | "no")
        ),
        "PRECONDITION: the capture plane must be ON for this arm"
    );
    let mgr = test_manager("coalesce");
    let responder =
        FlashbackResponder::open_on_manager(&mgr, "test-recorder").expect("responder opens");
    assert!(
        responder.drain_requests().is_empty(),
        "the channel starts empty"
    );

    let ask = TransportAsk::on_manager(mgr.clone());
    assert_eq!(
        ask.coalesced_asks(),
        0,
        "PRECONDITION: nothing has coalesced yet"
    );

    // THREE asks, back to back and well inside the 1.5 s linger of the first.
    ask.ask("operator-one");
    ask.ask("operator-two");
    ask.ask("operator-three");

    assert_eq!(
        ask.coalesced_asks(),
        2,
        "the second and third asks must COALESCE into the first's in-flight linger rather than \
         each spawning a 1.5 s publisher thread"
    );

    // …and the CHANNEL agrees: ONE ask reached the recorder, not three. Drained
    // through the whole linger so a later, differently-id'd ask has somewhere to
    // appear.
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut got = Vec::new();
    while Instant::now() < deadline && got.is_empty() {
        got.extend(responder.drain_requests());
        if got.is_empty() {
            std::thread::sleep(Duration::from_millis(20));
        }
    }
    let settle = Instant::now() + FLASHBACK_UNWATCHED_LINGER + Duration::from_millis(500);
    while Instant::now() < settle {
        got.extend(responder.drain_requests());
        std::thread::sleep(Duration::from_millis(20));
    }
    let ids: std::collections::BTreeSet<u64> = got.iter().map(|f| f.request_id).collect();
    assert_eq!(
        ids.len(),
        1,
        "three concurrent e-stops are ONE robot-wide regime and must reach the recorder as one \
         ask: {ids:?}"
    );
    assert_eq!(got[0].request.kind, TriggerKind::EStop);

    // ANTI-TAUTOLOGY: once the linger has ended the latch is CLEARED, so a later
    // e-stop is a fresh ask rather than being suppressed forever. Without this, a
    // latch that never cleared would pass every assertion above while silently
    // disabling e-stop captures for the life of the process.
    let cleared = Instant::now() + FLASHBACK_UNWATCHED_LINGER + Duration::from_secs(2);
    let mut fresh = std::collections::BTreeSet::new();
    while Instant::now() < cleared && fresh.is_empty() {
        ask.ask("operator-later");
        for frame in responder.drain_requests() {
            if !ids.contains(&frame.request_id) {
                fresh.insert(frame.request_id);
            }
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    assert_eq!(
        fresh.len(),
        1,
        "the in-flight latch must CLEAR when the linger ends — a later e-stop is a new ask, not \
         a suppressed one: {fresh:?}"
    );
}

/// A leader publisher that FAILS must not silently swallow the asks that
/// coalesced behind it.
///
/// # The hazard
///
/// Coalescing tells every follower, in effect, "the in-flight publisher has you
/// covered". If that leader then fails — a transient transport fault, an open
/// that loses a race for one of `FLASHBACK_MAX_REQUESTERS` slots — the followers
/// were absorbed and served by nobody: no capture for any of them, and not even
/// the leader's own `warn!` to explain it, because that line is about the
/// leader's request. On a safety path that is silent loss (Principle #6).
///
/// # Why the seam, and what it does NOT weaken
///
/// The failure has to be both DETERMINISTIC and TRANSIENT. A real transport
/// fault is neither: it would fail the retry too, so the arm could never tell
/// "retried and served them" from "retried and failed again" — which is the only
/// distinction that matters. `fail_next_publish_for_test` is one-shot, so the
/// RETRY takes the ordinary production path and the assertion below is a genuine
/// publish over the real channel, read off the recorder's own end.
#[test]
fn a_failed_leader_retries_for_the_asks_it_absorbed_rather_than_discarding_them() {
    // See the production arm: `ask` reads `CERULION_FLASHBACK` synchronously.
    let _env = env_lock();
    assert!(
        !matches!(
            std::env::var("CERULION_FLASHBACK").ok().as_deref(),
            Some("off" | "0" | "false" | "no")
        ),
        "PRECONDITION: the capture plane must be ON for this arm"
    );
    let mgr = test_manager("leaderfail");
    let responder =
        FlashbackResponder::open_on_manager(&mgr, "test-recorder").expect("responder opens");
    assert!(
        responder.drain_requests().is_empty(),
        "the channel starts empty"
    );

    let ask = TransportAsk::on_manager(mgr.clone());
    assert_eq!(
        ask.leader_failure_retries(),
        0,
        "PRECONDITION: no retry yet"
    );

    // The LEADER's first publish attempt will fail.
    TransportAsk::fail_next_publish_for_test();
    ask.ask("operator-leader");
    // …and two more arrive while it holds the latch, so they COALESCE into a
    // publisher that is about to fail.
    ask.ask("operator-follower-1");
    ask.ask("operator-follower-2");
    assert_eq!(
        ask.coalesced_asks(),
        2,
        "PRECONDITION: two asks must have been ABSORBED by the failing leader — otherwise this \
         arm is testing a leader that owed nobody anything"
    );

    // THE PIN: a real request reaches the recorder, published by the retry.
    // Without the retry nothing arrives at all — the leader failed and the
    // followers are discarded with it.
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut got = Vec::new();
    while Instant::now() < deadline && got.is_empty() {
        got.extend(responder.drain_requests());
        if got.is_empty() {
            std::thread::sleep(Duration::from_millis(20));
        }
    }
    let settle = Instant::now() + FLASHBACK_UNWATCHED_LINGER + Duration::from_millis(500);
    while Instant::now() < settle {
        got.extend(responder.drain_requests());
        std::thread::sleep(Duration::from_millis(20));
    }

    let ids: std::collections::BTreeSet<u64> = got.iter().map(|f| f.request_id).collect();
    assert_eq!(
        ids.len(),
        1,
        "the failed leader must retry ONCE for the asks it absorbed — exactly one request must \
         reach the recorder, not zero (discarded) and not three (one per ask): {ids:?}"
    );
    assert_eq!(got[0].request.kind, TriggerKind::EStop);
    assert_eq!(
        ask.leader_failure_retries(),
        1,
        "…and the recovery is COUNTED, so it is a measurement rather than a claim"
    );

    // ANTI-TAUTOLOGY: a leader that SUCCEEDS never retries, however many asks
    // coalesce behind it. Without this, an implementation that retried
    // unconditionally would pass everything above while doubling the publish work
    // of every ordinary e-stop.
    let ask2 = TransportAsk::on_manager(mgr.clone());
    ask2.ask("operator-healthy");
    ask2.ask("operator-healthy-follower");
    assert_eq!(
        ask2.coalesced_asks(),
        1,
        "PRECONDITION: the healthy leader also absorbed an ask"
    );
    let settle = Instant::now() + FLASHBACK_UNWATCHED_LINGER + Duration::from_millis(500);
    while Instant::now() < settle {
        std::thread::sleep(Duration::from_millis(50));
    }
    assert_eq!(
        ask2.leader_failure_retries(),
        0,
        "a leader that SUCCEEDED owes its followers nothing — it already served them"
    );
}

/// A follower that arrives in the window between ADMISSION and the baseline load
/// is still owned by the leader.
///
/// # The race
///
/// The retry fires when `coalesced` GREW during the leader's tenure. If the
/// baseline is read after the CAS that admits the leader, a follower can
/// `fetch_add` in between — and the leader then reads a baseline that already
/// contains it, sees no growth, and declines to retry. That absorbed ask gets no
/// capture: the same silent loss the retry exists to prevent, one race narrower.
///
/// Fixed by reading the baseline BEFORE the CAS. The asymmetry is what makes that
/// safe: a follower increments only when its own CAS fails, which requires
/// `in_flight` to be true, so every increment after a successful CAS belongs to
/// this leader. An increment between the read and the CAS belonged to the
/// PREVIOUS leader and can only cause a spurious retry — one extra publish,
/// bounded and idempotent at the recorder's gate. Never a missed one.
///
/// # Deterministic, not a stress loop
///
/// `set_after_admit_hook_for_test` runs exactly in that window and bumps
/// `coalesced`, which is precisely what a follower's failed CAS does. So the arm
/// reproduces the race on every run rather than hoping to hit it, and a
/// baseline read after admission fails it every run too.
#[test]
fn a_follower_arriving_between_admission_and_the_baseline_is_still_owned_by_the_leader() {
    let _env = env_lock();
    assert!(
        !matches!(
            std::env::var("CERULION_FLASHBACK").ok().as_deref(),
            Some("off" | "0" | "false" | "no")
        ),
        "PRECONDITION: the capture plane must be ON for this arm"
    );
    let mgr = test_manager("admitrace");
    let responder =
        FlashbackResponder::open_on_manager(&mgr, "test-recorder").expect("responder opens");
    assert!(
        responder.drain_requests().is_empty(),
        "the channel starts empty"
    );

    let ask = TransportAsk::on_manager(mgr.clone());
    // A follower lands in the admission→baseline window, exactly once.
    TransportAsk::set_after_admit_hook_for_test(Some(|coalesced| {
        coalesced.fetch_add(1, std::sync::atomic::Ordering::AcqRel);
    }));
    // …and the leader's publish fails, so the retry is what must rescue it.
    TransportAsk::fail_next_publish_for_test();
    ask.ask("operator-leader");
    TransportAsk::set_after_admit_hook_for_test(None);

    assert_eq!(
        ask.coalesced_asks(),
        1,
        "PRECONDITION: exactly one follower must have landed in the window"
    );

    // THE PIN: the leader still owes that follower, so the retry publishes.
    // Under a baseline read AFTER admission the growth is invisible and NOTHING
    // reaches the recorder.
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut got = Vec::new();
    while Instant::now() < deadline && got.is_empty() {
        got.extend(responder.drain_requests());
        if got.is_empty() {
            std::thread::sleep(Duration::from_millis(20));
        }
    }
    let settle = Instant::now() + FLASHBACK_UNWATCHED_LINGER + Duration::from_millis(500);
    while Instant::now() < settle {
        got.extend(responder.drain_requests());
        std::thread::sleep(Duration::from_millis(20));
    }

    let ids: std::collections::BTreeSet<u64> = got.iter().map(|f| f.request_id).collect();
    assert_eq!(
        ids.len(),
        1,
        "a follower admitted into the baseline window is still the leader's to serve — the retry \
         must publish exactly one request, not zero: {ids:?}"
    );
    assert_eq!(got[0].request.kind, TriggerKind::EStop);
    assert_eq!(
        ask.leader_failure_retries(),
        1,
        "…and the recovery is counted"
    );
}

/// Strip Rust comments so a source-order walk cannot be satisfied by PROSE.
///
/// Load-bearing here: the seam's own comment block names both
/// `compare_exchange` and the baseline load, in that order, purely to explain
/// itself — so a walk over the raw text would happily "prove" the invariant
/// from a paragraph describing it. Block comments are depth-tracked because
/// Rust's nest. String literals are deliberately not modelled; this file holds
/// its needles as literals, which is why it never walks ITSELF.
fn code_only(src: &str) -> String {
    let b = src.as_bytes();
    let (mut out, mut i, mut depth) = (String::with_capacity(src.len()), 0usize, 0usize);
    while i < b.len() {
        if depth == 0 && b[i] == b'/' && i + 1 < b.len() && b[i + 1] == b'/' {
            while i < b.len() && b[i] != b'\n' {
                i += 1;
            }
        } else if b[i] == b'/' && i + 1 < b.len() && b[i + 1] == b'*' {
            depth += 1;
            i += 2;
        } else if depth > 0 && b[i] == b'*' && i + 1 < b.len() && b[i + 1] == b'/' {
            depth -= 1;
            i += 2;
        } else {
            if depth == 0 {
                out.push(b[i] as char);
            }
            i += 1;
        }
    }
    out
}

/// The baseline load must PRECEDE the admitting CAS in the source itself.
///
/// # Why the behavioural pin is not enough on its own
///
/// `AFTER_ADMIT_HOOK` models a follower arriving at one instant, and a hook can
/// never be seen by a load placed ABOVE it. So an implementation that moved the
/// load after the CAS but above the hook would read a baseline WITHOUT the
/// injected follower, see growth, retry — and pass
/// `a_follower_arriving_between_admission_and_the_baseline_is_still_owned_by_the_leader`
/// vacuously. Moving the hook to fire at admission shrinks that blind slot to
/// nothing a realistic edit would occupy, but it cannot close it by
/// construction.
///
/// This closes it directly, by asserting the invariant the hook only models:
/// the read happens before the compare-exchange. Cheap, total, and immune to
/// where the seam sits.
#[test]
fn the_baseline_load_precedes_the_admitting_cas_in_source_order() {
    let src = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/src/flashback.rs"))
        .expect("read the ops-plane ask's own source");
    let code = code_only(&src);

    let ask = code
        .find("fn ask(&self, by: &str)")
        .expect("`TransportAsk` must still implement `ask`");
    let body = &code[ask..];

    let load = body
        .find("let coalesced_before = self.coalesced.load(")
        .expect("the leader must still read a coalesce baseline");
    let cas = body
        .find(".compare_exchange(false, true,")
        .expect("admission must still be a compare_exchange on `in_flight`");

    assert!(
        load < cas,
        "the coalesce baseline must be read BEFORE the CAS that admits the leader. Read after \
         it, a follower can `fetch_add` in between, the leader reads a baseline that already \
         contains it, sees no growth, declines to retry — and that absorbed ask gets no capture \
         (Principle #6). Found the load at {load} and the CAS at {cas} within `ask`."
    );

    // ANTI-TAUTOLOGY: the stripper must leave real code (or both `find`s above
    // would be searching a husk), and must remove the comment prose that names
    // the same two tokens in the opposite order.
    assert!(
        code.contains("compare_exchange(false, true,"),
        "the stripped view must still hold the CAS itself"
    );
    assert!(
        !code.contains("baseline load precedes"),
        "the stripped view must NOT hold comment prose — otherwise this walk could be satisfied \
         by a paragraph rather than by the code"
    );
}
