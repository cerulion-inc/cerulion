// SPDX-License-Identifier: AGPL-3.0-only
//! The E-STOP capture producer.
//!
//! # What this is
//!
//! When a human engages the ops-plane e-stop, somebody has DECLARED an incident.
//! That is the highest-specificity trigger in the whole Flashback space — higher
//! than a worker death, higher than any monitor verdict — and until this module
//! shipped, nothing published it: [`TriggerKind::EStop`] had a wire byte, a
//! posture switch and a [`CaptureRequest`] constructor, and zero producers.
//!
//! This module is the producer. It publishes ONE
//! [`CaptureRequest::estop`] onto the machine-local
//! `/__cerulion/flashback` channel, following the
//! `publish_process_faults` pattern the graph supervisor already uses for a dying
//! worker: a requester built PER CALL, best-effort, and not kept BEYOND the call.
//!
//! "Not kept beyond the call" is the accurate form. The publish now
//! LINGERS — `request_and_linger` holds the requester's ports for
//! `FLASHBACK_UNWATCHED_LINGER` after the frame goes out, because a request
//! whose publisher is reclaimed before any recorder drains it is a request
//! nobody receives. What is true is that no requester outlives its call;
//! "never held" would be wrong, because the linger contradicts it.
//!
//! # SCOPE IS NARROW, and the narrowness is not a bug to be papered over
//!
//! What fires this is `cerud`'s `engage-estop` verb, reached over the
//! pairing-gated `cerulion/ops/1` plane — so it sees a WAN-paired operator's
//! e-stop and NOTHING ELSE. A hardwired Cat-0 / STO stop (motor power cut,
//! compute alive) announces nowhere in the middleware: no verb runs, no request
//! is published, and no capture happens. Nothing in this module's logs or docs
//! may imply otherwise, because an operator who reads "the e-stop is captured"
//! and then pulls the physical mushroom would be owed a bag that does not exist.
//!
//! # The e-stop is SAFETY-CRITICAL; the capture is BEST-EFFORT. The asymmetry is
//! the whole design
//!
//! Two rules follow, and both are structural rather than a matter of care:
//!
//! * **A publish failure can never fail the e-stop.** [`EstopCaptureAsk::ask`]
//!   returns `()`. There is no error to propagate, so no future edit can
//!   accidentally wire one into the verb's `Result`.
//! * **A publish can never SLOW the e-stop.** Opening an iceoryx2 node is a
//!   cold, unbounded-ish operation (measured elsewhere in this repo at ~620 ms on
//!   an idle desk), and the ops plane's e-stop floor is the one path in this
//!   daemon that must not wait behind anything. So [`TransportAsk`] does the
//!   whole publish on a DETACHED THREAD: the verb pays one `thread::spawn` and
//!   returns. This is the one deliberate deviation from `publish_process_faults`,
//!   which publishes inline — that caller is a supervisor handling a dead worker,
//!   with no safety floor to protect.
//!
//! # FIRE-AND-FORGET, deliberately
//!
//! Nothing here opens the outcome subscriber or waits on a verdict. That keeps
//! the channel's documented back-compat condition UNBOUND: `channel.rs`'s
//! verdict-9 note says the compat story is owed the first time something
//! "publishes a switchable kind AND WAITS ON THE ANSWER", and this producer does
//! not wait. It also means an operator who engages the e-stop is never made to
//! wait on a recorder that may not exist.
//!
//! # There is no de-duplication here, ON PURPOSE
//!
//! `ControlLease::engage_estop` is idempotent and cannot distinguish a first
//! engage from a re-engage, so this module asks on EVERY engage. That is correct
//! because the de-duplication already exists one layer down and is the reason
//! that layer exists: [`TriggerKind::EStop`] is automatic, so the recorder's gate
//! latches its regime and applies the refractory floor, and
//! [`CaptureRequest::estop`] pins ONE subject for the whole robot precisely so
//! two engagements inside the floor coalesce into one bag. Re-implementing that
//! here would be a second copy of a policy that has to agree with the first.
//!
//! [`TriggerKind::EStop`]: cerulion_core::flashback::trigger::TriggerKind::EStop
//! [`CaptureRequest`]: cerulion_core::flashback::trigger::CaptureRequest
//! [`CaptureRequest::estop`]: cerulion_core::flashback::trigger::CaptureRequest::estop

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;

use cerulion_core::flashback::channel::{FlashbackRequester, FLASHBACK_UNWATCHED_LINGER};
use cerulion_core::flashback::trigger::CaptureRequest;
use cerulion_core::flashback::{parse_plane_switch, PlaneSwitch, FLASHBACK_ENV};
use cerulion_core::TransportManager;

/// What the e-stop verb asks for when it engages.
///
/// A trait rather than a concrete call so the verb can be driven without
/// transport — the same dependency-injection seam `cerulion_netd`'s `MirrorPlane`
/// uses, and for the same reason: the interesting assertions are "was it asked,
/// exactly once, with the right cause" and "does a failing ask still leave the
/// e-stop successful", neither of which needs an iceoryx2 node to be true.
///
/// `Send + Sync` because the ops server serves sessions CONCURRENTLY over a
/// shared `&self`, so two operators can engage at once.
pub trait EstopCaptureAsk: Send + Sync {
    /// Ask the machine's recorder to capture the moment. Never fails, by design
    /// — see the module docs.
    ///
    /// `by` is the authenticated caller id the verb engaged on behalf of; it
    /// rides the request's DETAIL, never its subject (the subject is the regime
    /// key, and keying an e-stop on the operator would give every operator their
    /// own regime).
    fn ask(&self, by: &str);
}

/// The PRODUCTION ask: publish onto `/__cerulion/flashback`.
///
/// Holds no transport of its own. A `cerulion_remoted` that never serves a wire
/// peer and is never e-stopped therefore still never touches iceoryx2 — the same
/// property `wire::ManagerSource::Lazy` exists to preserve, restated here because
/// this is a second path that could have broken it.
pub struct TransportAsk {
    /// The manager to publish through, when a caller has one.
    ///
    /// `None` on every production path, which resolves to
    /// [`TransportManager::get_or_init`] — the DEFAULT `iox2_` namespace, which
    /// is the data plane an unqualified `cerulion bagd` (and therefore the
    /// always-on window recorder a `graph run` spawns) listens on.
    ///
    /// This is the same NAMESPACE TRAP `publish_process_faults` carries a whole
    /// paragraph about, and `remoted` lands on the right side of it for a reason
    /// worth stating rather than assuming: unlike a multi-process supervisor —
    /// whose own singleton lives on a throwaway `cer_p_{hex}` PLANNING namespace,
    /// so requests published through it reach nothing at all — `remoted` is a
    /// robot-level daemon whose own transport is `TransportConfig::default()`.
    /// Its singleton IS the data plane.
    manager: Option<Arc<TransportManager>>,
    /// Is a capture ask already IN FLIGHT for this plane?
    ///
    /// Each ask spawns a detached thread that lives for
    /// `FLASHBACK_UNWATCHED_LINGER` (1.5 s). Unbounded, a caller that engages the
    /// e-stop in a loop — an automated safety loop re-arming, a bouncing bumper,
    /// the paired-caller case the ops plane explicitly supports — spawns one
    /// thread per call and can exhaust the process's thread budget on the one
    /// daemon that must not fall over.
    ///
    /// COALESCING rather than a cap, because coalescing is what is actually true
    /// here: `CaptureRequest::estop` pins ONE robot-wide subject by design decision,
    /// so every concurrent e-stop ask is the SAME regime key, and the recorder's
    /// gate already folds a burst into one capture. A second thread would
    /// re-publish the identical subject the in-flight linger is already
    /// re-publishing — it buys nothing and costs a thread. A fixed cap would
    /// admit N identical asks before refusing, which is the same waste with an
    /// arbitrary number in front of it.
    ///
    /// The cost, stated: a coalesced ask's DETAIL (`engaged by {by}`) is not
    /// published. The subject is robot-wide, so the capture is the same capture;
    /// what is lost is the second operator's name in the cause list. Counted and
    /// logged rather than silent — see [`coalesced_asks`](Self::coalesced_asks).
    in_flight: Arc<AtomicBool>,
    /// How many asks COALESCED into an in-flight one. Observable so the
    /// suppression is a measurement rather than a claim (Principle #3).
    coalesced: Arc<AtomicU64>,
    /// How many times a FAILED leader retried on behalf of asks it had absorbed.
    ///
    /// Observable so the recovery is a measurement rather than a claim: zero on
    /// every healthy robot (it takes a leader failure with at least one ask
    /// coalesced behind it to move), and the one number that separates "the
    /// followers were served by the retry" from "the followers were dropped".
    leader_failure_retries: Arc<AtomicU64>,
}

/// TEST SEAM: run immediately after a leader is ADMITTED (its CAS succeeded),
/// before the baseline would have been loaded under the old ordering.
///
/// A plain `fn(&AtomicU64)` rather than a closure so it needs no capture and can
/// live in a `static`. A test installs one that bumps `coalesced`, which is
/// precisely what a follower's failed CAS does — so this reproduces the race
/// exactly, and deterministically, instead of hoping a stress loop hits it.
#[cfg(any(test, feature = "test-seam"))]
pub(crate) static AFTER_ADMIT_HOOK: std::sync::Mutex<Option<fn(&AtomicU64)>> =
    std::sync::Mutex::new(None);

/// TEST SEAM: force the NEXT publish attempt to fail, once.
///
/// A leader that fails is the hazard the retry below exists for, and it cannot
/// be produced from outside without a real transport fault — which is neither
/// deterministic nor transient, so a retry could never be observed SUCCEEDING
/// against one. One-shot (`swap(false)`), so the retry takes the real path and
/// the arm asserts a genuine publish rather than a second failure.
#[cfg(any(test, feature = "test-seam"))]
pub(crate) static FAIL_NEXT_PUBLISH: AtomicBool = AtomicBool::new(false);

/// Clears the in-flight latch when the publish thread ends — INCLUDING on an
/// unwind, which is why this is a guard and not a store at the end of the body.
/// A panic that left the latch set would suppress every later e-stop capture for
/// the life of the process.
struct InFlightGuard(Arc<AtomicBool>);

impl Drop for InFlightGuard {
    fn drop(&mut self) {
        self.0.store(false, Ordering::Release);
    }
}

impl TransportAsk {
    /// The production ask.
    pub fn new() -> Self {
        Self {
            manager: None,
            in_flight: Arc::new(AtomicBool::new(false)),
            coalesced: Arc::new(AtomicU64::new(0)),
            leader_failure_retries: Arc::new(AtomicU64::new(0)),
        }
    }

    /// How many e-stop asks COALESCED into an already-in-flight one.
    ///
    /// Zero on every ordinary robot: it takes a second e-stop inside the 1.5 s
    /// linger of the first to move it.
    pub fn coalesced_asks(&self) -> u64 {
        self.coalesced.load(Ordering::Acquire)
    }

    /// TEST SEAM: make the NEXT publish attempt fail, once.
    ///
    /// `#[doc(hidden)]` and feature-gated, so it never compiles into the
    /// production daemon. It exists because a leader failure cannot otherwise be
    /// produced deterministically AND transiently — a real transport fault would
    /// fail the retry too, which is the one thing the recovery arm must be able
    /// to distinguish.
    #[doc(hidden)]
    #[cfg(any(test, feature = "test-seam"))]
    pub fn fail_next_publish_for_test() {
        FAIL_NEXT_PUBLISH.store(true, Ordering::Release);
    }

    /// TEST SEAM: install (or clear, with `None`) the after-admission hook.
    ///
    /// See [`AFTER_ADMIT_HOOK`]. Feature-gated, so it never compiles into the
    /// production daemon.
    #[doc(hidden)]
    #[cfg(any(test, feature = "test-seam"))]
    pub fn set_after_admit_hook_for_test(hook: Option<fn(&AtomicU64)>) {
        *AFTER_ADMIT_HOOK.lock().unwrap_or_else(|e| e.into_inner()) = hook;
    }

    /// How many times a failed leader RETRIED for asks it had absorbed.
    ///
    /// Zero on every healthy robot; see [`leader_failure_retries`](Self::leader_failure_retries).
    pub fn leader_failure_retries(&self) -> u64 {
        self.leader_failure_retries.load(Ordering::Acquire)
    }

    /// The production ask, publishing through an EXPLICIT manager.
    ///
    /// Test-only, and it is the PRODUCTION publish path — not a stand-in. It
    /// exists so a test can drive the real publish over an isolated per-test SHM
    /// root instead of the shared default namespace; everything below the
    /// resolution of this one field is the code a robot runs.
    #[doc(hidden)]
    #[cfg(any(test, feature = "test-seam"))]
    pub fn on_manager(manager: Arc<TransportManager>) -> Self {
        Self {
            manager: Some(manager),
            in_flight: Arc::new(AtomicBool::new(false)),
            coalesced: Arc::new(AtomicU64::new(0)),
            leader_failure_retries: Arc::new(AtomicU64::new(0)),
        }
    }

    /// PURE: is the capture plane switched off for this robot?
    ///
    /// Read through the SAME parser the recorder's own arm word and the
    /// supervisor's `flashback_switched_off` use. An operator who turned
    /// Flashback off must not have an e-stop open an iceoryx2 node on their
    /// behalf.
    fn plane_off() -> bool {
        matches!(
            parse_plane_switch(std::env::var(FLASHBACK_ENV).ok().as_deref()),
            PlaneSwitch::Off
        )
    }

    /// Build a requester and publish `request`. Runs on the detached thread.
    ///
    /// Returns whether a request really reached the channel. The caller needs
    /// that answer rather than fire-and-forget, because this publisher is the
    /// LEADER for every ask that coalesced behind it: if it fails, those asks
    /// were absorbed and served by nobody (Principle #6).
    #[must_use]
    fn publish(manager: Option<Arc<TransportManager>>, request: &CaptureRequest) -> bool {
        let opened = match manager {
            Some(m) => FlashbackRequester::open_on_manager(&m),
            None => match TransportManager::get_or_init() {
                Ok(m) => FlashbackRequester::open_on_manager(&m),
                Err(e) => {
                    // WARN, not DEBUG. `cerulion_remoted` sets
                    // `release_max_level_info`, so a `debug!` here does not exist
                    // in a robot's shipping binary at ANY `RUST_LOG` — an e-stop
                    // would produce no capture and no evidence that it had not.
                    // The sibling publish-failure branch below has always been a
                    // `warn!`; these two open-failure branches were the
                    // inconsistency.
                    tracing::warn!(
                        error = %e,
                        "flashback: no transport for the e-stop trigger — no capture \
                         will be made for this e-stop (the e-stop itself is unaffected)"
                    );
                    return false;
                }
            },
        };
        let requester = match opened {
            Ok(r) => r,
            Err(e) => {
                // WARN for the same reason as the branch above.
                tracing::warn!(
                    error = %e,
                    "flashback: could not open the trigger channel for the e-stop — no \
                     capture will be made for this e-stop (the e-stop itself is unaffected)"
                );
                return false;
            }
        };
        // LINGER, never a bare `request()`. A requester dropped before the
        // recorder's next drive pass takes its own request back out of the queue
        // (iceoryx2 reclaims a departing publisher's unread samples) — measured,
        // and documented at `FLASHBACK_UNWATCHED_LINGER`. The linger costs this
        // detached thread and nothing else; the e-stop returned long ago.
        //
        // Still FIRE-AND-FORGET: no outcome is read, so the channel's verdict-9
        // back-compat condition stays unbound.
        // TEST SEAM: force this attempt to fail without a real transport fault,
        // so the leader-failure recovery below is drivable deterministically.
        // Consumed (one-shot), so the RETRY takes the real path — which is what
        // makes the arm assert a genuine publish rather than a second failure.
        #[cfg(any(test, feature = "test-seam"))]
        if FAIL_NEXT_PUBLISH.swap(false, Ordering::AcqRel) {
            tracing::warn!("flashback: test seam forced the e-stop publish to fail");
            return false;
        }
        match requester.request_and_linger(request, FLASHBACK_UNWATCHED_LINGER, &|| false) {
            // INFO, not DEBUG: `cerulion_remoted` caps release logging at info
            // (`release_max_level_info`), so a `debug!` here would not exist in a
            // robot's shipping binary at ANY `RUST_LOG` — and this line is the
            // operator's only evidence that the incident they declared was also
            // asked to be recorded.
            Ok(_) => {
                tracing::info!(
                    subject = %request.subject,
                    "flashback: e-stop engaged — asked the recorder to capture the moment"
                );
                true
            }
            Err(e) => {
                tracing::warn!(
                    error = %e,
                    "flashback: could not publish the e-stop capture request — the \
                     e-stop itself is unaffected"
                );
                false
            }
        }
    }
}

impl Default for TransportAsk {
    fn default() -> Self {
        Self::new()
    }
}

impl EstopCaptureAsk for TransportAsk {
    fn ask(&self, by: &str) {
        if Self::plane_off() {
            return;
        }
        // The DETAIL, never the subject: `CaptureRequest::estop` pins one subject
        // for the whole robot by design decision.
        // The leader's coalesce baseline, read BEFORE the CAS that admits it.
        //
        // Reading it AFTER left a window — several statements wide — in which a
        // follower on another thread could `fetch_add` between admission and the
        // load. The leader would then miss that growth, decline to retry, and the
        // absorbed ask would get no capture: the exact loss the retry exists to
        // prevent, one race narrower than the version before it.
        //
        // Before-the-CAS is safe in the direction that matters, and the asymmetry
        // is the whole argument. A follower increments ONLY when its own CAS
        // fails, which can only happen while `in_flight` is true. If this CAS
        // succeeds, `in_flight` was false at that instant — so every increment
        // after it is a follower of THIS leader and is counted. An increment
        // between the read and the CAS belonged to the PREVIOUS leader's tenure,
        // and counting it can only produce a spurious retry: one extra publish
        // attempt, bounded and idempotent at the recorder's gate. Never a missed
        // one. Losing an ask is unacceptable (Principle #6); an occasional extra
        // publish is not.
        let coalesced_before = self.coalesced.load(Ordering::Acquire);
        // COALESCE. One ask at a time may hold a detached thread; a second while
        // that one lingers is the SAME robot-wide subject the in-flight linger is
        // already re-publishing, so it would buy nothing and cost a thread. See
        // `in_flight` for why this is coalescing rather than a cap.
        if self
            .in_flight
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            let n = self.coalesced.fetch_add(1, Ordering::AcqRel) + 1;
            // INFO, not DEBUG: `release_max_level_info` again — an operator whose
            // second e-stop produced no separate ask must be able to see that in
            // a shipping binary.
            tracing::info!(
                by = %by,
                coalesced_asks = n,
                "flashback: an e-stop capture ask is already in flight — this one \
                 coalesces into it (same robot-wide subject, one capture) rather than spawning \
                 a second publisher"
            );
            return;
        }
        // TEST SEAM: a follower landing the instant AFTER admission.
        //
        // A review note: this used to sit four statements lower, after `guard`,
        // `request` and `manager`. That made the pin VACUOUS: an implementation
        // that moved the baseline load after the CAS but ABOVE this line would
        // load before the hook fired, see the follower as growth, retry, and
        // pass. Firing it as the FIRST
        // thing after a successful CAS is also the faithful model: the window
        // this seam represents opens at admission, not four statements later.
        //
        // A hook can still never catch a load placed ABOVE it, which is why the
        // behavioural pin is PAIRED with a source-order assertion that the
        // baseline load precedes the `compare_exchange` (see
        // `the_baseline_load_precedes_the_admitting_cas_in_source_order`).
        //
        // It precedes the guard deliberately, and only in test builds: the
        // production binary has no hook, so the guard is still constructed
        // immediately after the CAS there.
        #[cfg(any(test, feature = "test-seam"))]
        if let Some(hook) = *AFTER_ADMIT_HOOK.lock().unwrap_or_else(|e| e.into_inner()) {
            hook(&self.coalesced);
        }
        // Constructed IMMEDIATELY after the CAS (after the cfg-gated seam above,
        // which does not exist in a production build), before anything that could
        // conceivably unwind: the guard's whole justification is that a panic
        // must not strand the latch, and leaving even two statements outside it
        // would leave a window where that is untrue.
        let guard = InFlightGuard(Arc::clone(&self.in_flight));
        let request = CaptureRequest::estop(format!("ops-plane e-stop engaged by {by}"));
        let manager = self.manager.clone();
        let coalesced = Arc::clone(&self.coalesced);
        let retries = Arc::clone(&self.leader_failure_retries);
        // DETACHED, and the reason is the safety floor — see the module docs.
        // Failing to spawn is itself best-effort: the e-stop has already taken
        // effect on the lease by the time this runs.
        if let Err(e) = std::thread::Builder::new()
            .name("cer-estop-capture".to_string())
            .spawn(move || {
                // Moved IN, so the latch clears when the thread ends — including
                // on an unwind. Clearing it here rather than after `spawn`
                // returns is the whole point: the latch must span the LINGER.
                let _guard = guard;
                let served = Self::publish(manager.clone(), &request);
                // THE LEADER OWES ITS FOLLOWERS (Principle #6).
                //
                // Every ask that arrived while this thread held the latch was
                // COALESCED — told, in effect, "the in-flight publisher has you
                // covered". If that publisher then failed, those asks were
                // absorbed and served by nobody: an e-stop with no capture and,
                // for the followers, not even the leader's own warn to explain
                // it. Under a TRANSIENT transport fault that is silent data loss
                // on a safety path.
                //
                // So a failed leader that absorbed anyone retries ONCE, on this
                // same thread — no second spawn, so the thread bound this latch
                // exists to enforce is untouched, and the latch is still held, so
                // asks arriving during the retry keep coalescing correctly onto
                // it. Bounded at one: a second failure is reported and the
                // condition is left to the next e-stop rather than looped over.
                let coalesced_after = coalesced.load(Ordering::Acquire);
                if !served && coalesced_after > coalesced_before {
                    let absorbed = coalesced_after - coalesced_before;
                    retries.fetch_add(1, Ordering::AcqRel);
                    // The count rides `absorbed=` and NOT the prose: a datum
                    // spliced into the message is one opaque string to every
                    // consumer (the AGENTS.md logging convention), and it was already in the
                    // field, so the interpolation only made the line
                    // ungreppable.
                    tracing::warn!(
                        absorbed,
                        "flashback: the in-flight e-stop publisher FAILED after further \
                         e-stop(s) coalesced into it — retrying once so those asks are not \
                         silently discarded"
                    );
                    if !Self::publish(manager, &request) {
                        // `absorbed=` carries the count; see the warn above.
                        tracing::error!(
                            absorbed,
                            "flashback: the e-stop capture retry ALSO failed — no \
                             capture was made for this e-stop or for the e-stop(s) that \
                             coalesced into it (the e-stops themselves are unaffected)"
                        );
                    }
                }
            })
        {
            // The thread never started. The latch still clears, and by
            // construction rather than by an action here: `Builder::spawn` takes
            // the closure BY VALUE and drops it on failure, which drops the
            // `InFlightGuard` it captured. Stated because the alternative — a
            // guard created after a successful spawn — would leave a failed spawn
            // suppressing every later e-stop capture for the life of the process,
            // and that is not visible from this arm alone.
            tracing::warn!(
                error = %e,
                "flashback: could not spawn the e-stop capture publisher — the e-stop \
                 itself is unaffected"
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    /// A recording double — the shape the verb tests use.
    #[derive(Default)]
    struct SpyAsk {
        asks: AtomicUsize,
        last: Mutex<Option<String>>,
    }
    use std::sync::Mutex;

    impl EstopCaptureAsk for SpyAsk {
        fn ask(&self, by: &str) {
            self.asks.fetch_add(1, Ordering::SeqCst);
            *self.last.lock().expect("spy") = Some(by.to_string());
        }
    }

    #[test]
    fn the_request_carries_the_estop_kind_and_the_one_robot_wide_subject() {
        // HAND oracle: the vocabulary this producer must mint, restated here
        // rather than read back off the constructor — a producer that started
        // minting `Manual` (inheriting the latch and floor EXEMPTIONS a human
        // watching a verb earns) would otherwise pass.
        let request = CaptureRequest::estop("ops-plane e-stop engaged by abc");
        assert_eq!(
            request.kind,
            cerulion_core::flashback::trigger::TriggerKind::EStop
        );
        assert_eq!(request.subject, "estop");
        assert!(request.detail.contains("abc"), "{}", request.detail);
        // The caller id rides the DETAIL and never the subject: two operators
        // engaging must be ONE regime, not two.
        assert!(!request.subject.contains("abc"));
    }

    #[test]
    fn a_spy_records_the_caller_it_was_asked_for() {
        // Anti-tautology floor for the verb tests: the double really does
        // observe, so a "was asked exactly once" assertion elsewhere is capable
        // of failing.
        let spy = SpyAsk::default();
        assert_eq!(spy.asks.load(Ordering::SeqCst), 0);
        spy.ask("operator-7");
        assert_eq!(spy.asks.load(Ordering::SeqCst), 1);
        assert_eq!(spy.last.lock().expect("spy").as_deref(), Some("operator-7"));
    }

    #[test]
    fn the_plane_kill_switch_is_read_through_the_shared_parser() {
        // Oracle vectors for the one env read this module does, driven through
        // the parser rather than a second copy of its spelling. `None` is ON —
        // the feature ships on, and an operator who never heard of the variable
        // gets it (the `parse_plane_switch` contract).
        assert!(matches!(parse_plane_switch(None), PlaneSwitch::On));
        assert!(matches!(parse_plane_switch(Some("off")), PlaneSwitch::Off));
        assert!(matches!(parse_plane_switch(Some("on")), PlaneSwitch::On));
    }
}
