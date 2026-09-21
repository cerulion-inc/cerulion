// SPDX-License-Identifier: AGPL-3.0-only
//! Topic bridge manager — the demand-driven egress SCOPING filter.
//!
//! Access decision: the egress allow-list is a per-graph scoping /
//! SAFETY filter (limiting which produced topics a graph exports), NOT an
//! AUTHORIZATION boundary. Authorization is the account / pairing grant, enforced by
//! the [`crate::transport::demand_authorizer::DemandAuthorizer`] seam BOTH demand
//! planes consult. This list narrows a graph's own egress surface; it never
//! decides WHO may access the machine.
//!
//! A per-topic bridge flag no longer gates a publisher's dual-publish
//! (publishers are network-free now). It signals the network GATEWAY's egress
//! TAP SET: when a remote subscriber appears (detected via zenoh liveliness),
//! the demand watch calls [`TopicBridgeManager::enable_bridge`], which flips the
//! topic's flag ON; the [`crate::transport::gateway::GatewayRuntime`] reads that
//! flag and attaches a listener-less SHM tap to forward the produced topic to
//! zenoh. A remote departure flips it OFF and the gateway drops the tap.
//!
//! The coordination primitive is `Arc<AtomicBool>` shared between the bridge
//! manager (writer, flipped by the demand watch) and the gateway (reader). An
//! `EgressPolicy` SCOPING filter (private) guards every flip — only ANNOUNCED
//! (allow-listed, or any registered under `AllowAll`) topics may ever egress.
//! The DEFAULT posture is deny-all, so a manager that installed no policy
//! egresses NOTHING. This is a per-graph EXPORT scope, not an access decision:
//! the demand-authorization gate (account/pairing grant) is what refuses an
//! unauthorized demander.

use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

use crate::error::{TransportError, TransportResult};

/// The DEMAND-TRANSITION wake signal — the thing a gateway with
/// ZERO demand blocks on instead of re-scanning its flag map a thousand times a
/// second.
///
/// It is a GENERATION counter, not a flag, and that is the whole of its
/// correctness. A gateway's idle wait is a two-step sequence — "are any flags ON?"
/// then "block" — and a demand landing BETWEEN those two steps is exactly the
/// lost-wakeup class Principle #6 forbids. A boolean "something changed" flag
/// would have to be cleared by the waiter, re-opening the same window one layer
/// down. A monotonic generation closes it by construction: the caller snapshots
/// the generation BEFORE it reads the flags, and [`Self::wait_for_change`] returns
/// IMMEDIATELY if the counter has moved past that snapshot, whether the bump
/// happened during the scan, during the lock acquisition, or a microsecond before
/// the wait.
///
/// WHO bumps it is deliberately narrow — the two events that can give a
/// zero-demand gateway something to do:
///
/// * [`TopicBridgeManager::enable_bridge`] on a genuine `false → true` demand
///   transition (a remote subscriber appeared), and
/// * [`TopicBridgeManager::register_topic`] when the registered set GROWS (a new
///   egress topic joined, so the next pass has a new flag to watch and a new
///   announce to make).
///
/// Plus [`crate::transport::TransportManager::register_dynamic_egress_topic`],
/// which is a runtime registration originating IN THIS PROCESS. A registration
/// arriving from ANOTHER process rides the iceoryx2 control channel and cannot
/// bump an in-process condvar, so it is served by the wait's TIMEOUT — which is
/// why the timeout is a real fallback and not a formality (see
/// [`crate::transport::gateway::GATEWAY_ZERO_DEMAND_IDLE`]).
///
/// A `disable_bridge` does NOT bump: demand going away can only give the loop
/// LESS to do, and the pass that observes it is already running (the loop only
/// blocks once nothing is ON).
pub struct DemandSignal {
    /// The monotonic generation. Bumped under the mutex by [`Self::signal`];
    /// read by [`Self::generation`] and compared by [`Self::wait_for_change`].
    generation: Mutex<u64>,
    /// Woken (`notify_all` — a machine may run more than one gateway in-process,
    /// e.g. a test harness) on every bump.
    changed: Condvar,
}

impl Default for DemandSignal {
    fn default() -> Self {
        Self::new()
    }
}

impl DemandSignal {
    /// A fresh signal at generation 0.
    pub fn new() -> Self {
        Self {
            generation: Mutex::new(0),
            changed: Condvar::new(),
        }
    }

    /// The current generation — the value a waiter must snapshot BEFORE it makes
    /// the decision it is about to block on. Poison-tolerant: this is a WAKE
    /// primitive on the observability path, and a poisoned lock must never wedge
    /// the plane it exists to pace (the `lock_regime_latch` precedent).
    pub fn generation(&self) -> u64 {
        *self
            .generation
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    /// Record that something a zero-demand gateway cares about happened, and wake
    /// every waiter. Cheap (one mutex + a notify) and called only on TRANSITIONS —
    /// never per frame, never per drive pass.
    pub fn signal(&self) {
        let mut gen_guard = self
            .generation
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        *gen_guard = gen_guard.wrapping_add(1);
        drop(gen_guard);
        self.changed.notify_all();
    }

    /// Block until the generation moves past `seen`, or `timeout` elapses.
    /// Returns `true` iff the generation MOVED (a real signal), `false` on the
    /// timeout — the caller's only way to tell a wake from a fallback tick, and
    /// the observable the no-inert-shipping pins key on.
    ///
    /// Returns `true` IMMEDIATELY (never blocking) when the generation already
    /// differs from `seen` — that is the race-window close: a transition that
    /// landed after the caller's snapshot but before this call cannot be lost.
    pub fn wait_for_change(&self, seen: u64, timeout: Duration) -> bool {
        let gen_guard = self
            .generation
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        // `wait_timeout_while` re-checks the predicate on every (possibly
        // spurious) wake, so a spurious wake is a no-op rather than a fabricated
        // signal.
        let (gen_guard, _timed_out) = self
            .changed
            .wait_timeout_while(gen_guard, timeout, |g| *g == seen)
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        *gen_guard != seen
    }
}

/// The egress bridge POSTURE — which produced topics a remote token
/// may switch ON for network egress. Replaces the earlier
/// `Option<HashSet<String>>` ("no list installed = permissive"): there is NO
/// permissive back-compat arm. The DEFAULT is deny-all (`AllowList(empty)`), so
/// a raw manager that never installed a policy egresses NOTHING — strictest by
/// construction. Nothing leaves the machine unless a graph build (or an
/// embedder) EXPLICITLY installs a posture.
enum EgressPolicy {
    /// Only member topics (canonical names) may ever flip —
    /// the graph's declared `network: egress:` list. A non-member is REFUSED
    /// (logged once per topic). The DEFAULT posture is `AllowList(empty)` =
    /// deny-all: an un-installed gate refuses every topic (the same
    /// once-per-topic refusal path as any allow-list miss), so a bare library
    /// exports nothing until a policy is installed.
    AllowList(HashSet<String>),
    /// The explicit permissive-egress posture — any REGISTERED topic's
    /// flag may flip, with NO per-topic refusal log. Installed by the
    /// permissive-default `graph run` arm for a graph with no `network:` block,
    /// so every produced topic is network-viewable until pairing lands.
    /// Tests/embedders opt into it LOUDLY via
    /// [`TopicBridgeManager::set_egress_allow_all`]; observable via
    /// [`TopicBridgeManager::egress_is_allow_all`].
    AllowAll,
}

impl Default for EgressPolicy {
    /// Deny-all: nothing egresses until a policy is explicitly installed.
    fn default() -> Self {
        EgressPolicy::AllowList(HashSet::new())
    }
}

/// The egress POSTURE + its once-per-topic refusal
/// breadcrumb latch, guarded by ONE mutex (they change together: installing a
/// new posture resets the latch — a fresh `AllowList` may change verdicts, and
/// a stale latch would silently swallow its first refusals).
#[derive(Default)]
struct EgressGate {
    /// The active egress posture (default: deny-all [`EgressPolicy::AllowList`]
    /// with an empty set — nothing egresses until a policy is installed).
    policy: EgressPolicy,
    /// Topics whose refusal has already been logged (once per topic — a
    /// busy LAN can re-deliver the same foreign token on every liveliness
    /// churn, and per-event logging would flood). Only ever populated under
    /// [`EgressPolicy::AllowList`].
    refused_logged: HashSet<String>,
}

/// Tracks which topics have an active egress bridge (a demand-driven tap).
///
/// Driven by liveliness events from the zenoh session. The gateway
/// registers a flag per announced topic ([`Self::register_topic`]); the demand
/// watch flips it when remote subscribers appear/disappear, and the gateway
/// reads it to attach/detach its egress tap.
pub struct TopicBridgeManager {
    /// topic_name → egress-active flag (shared with the gateway)
    bridges: Mutex<HashMap<String, Arc<AtomicBool>>>,
    /// The egress allow-list gate consulted by
    /// [`Self::enable_bridge`] before any flag flips true.
    egress: Mutex<EgressGate>,
    /// The demand-transition wake a zero-demand gateway blocks
    /// on. Bumped by [`Self::enable_bridge`] on a genuine `false → true` edge and
    /// by [`Self::register_topic`] when the registered set grows. Shared out via
    /// [`Self::demand_signal`].
    demand_signal: Arc<DemandSignal>,
}

impl Default for TopicBridgeManager {
    fn default() -> Self {
        Self::new()
    }
}

impl TopicBridgeManager {
    /// Create a new bridge manager with no registered topics.
    pub fn new() -> Self {
        Self {
            bridges: Mutex::new(HashMap::new()),
            egress: Mutex::new(EgressGate::default()),
            demand_signal: Arc::new(DemandSignal::new()),
        }
    }

    /// The shared [`DemandSignal`] — the wake a gateway with ZERO
    /// demand blocks on instead of polling its flag map. Handed out as an `Arc` so
    /// a caller can hold it across a `GatewayRuntime` being moved onto a drive
    /// thread (netd's egress plane does exactly that, and pokes it on teardown so
    /// the join is prompt rather than bounded by the idle timeout).
    pub fn demand_signal(&self) -> Arc<DemandSignal> {
        Arc::clone(&self.demand_signal)
    }

    /// Install the egress SCOPING list — after this call,
    /// ONLY the given topics (canonical-keyed) can ever have their bridge
    /// flag flipped true by [`Self::enable_bridge`]. The graph build hook
    /// installs the graph's declared `network: egress:` list here.
    ///
    /// This is a per-graph safety filter on which produced
    /// topics THIS graph exports — NOT an authorization boundary. Authorization
    /// (WHO may demand this machine's topics) is the account/pairing grant,
    /// enforced by the
    /// [`crate::transport::demand_authorizer::DemandAuthorizer`] seam the demand
    /// planes consult. The list still STRUCTURALLY narrows egress to
    /// the declared set (a graph never leaks a topic it did not list), but a
    /// topic being listed no longer implies the demander is authorized.
    ///
    /// The scoping is STRUCTURAL, not advisory: [`Self::enable_bridge`] is
    /// the only writer of `true` into any bridge flag (the gateway only READs
    /// its flags; [`Self::disable_bridge`] only writes `false`), so a
    /// non-member flag can never read `true`.
    ///
    /// An EMPTY `topics` installs a deny-all list — the right state for an
    /// ingress-only graph (its watch task runs for ingress liveliness, but
    /// nothing it produces may leave the machine).
    ///
    /// REPLACE semantics: a second call overwrites the previous posture (one
    /// graph build per process on the production path; the once-per-topic
    /// refusal latch resets so the new list's refusals are logged afresh).
    /// Never calling this leaves the manager at the DEFAULT deny-all posture
    /// (`AllowList(empty)`) — a raw transport user or test egresses NOTHING
    /// until it installs a list here or opts into [`Self::set_egress_allow_all`].
    pub fn set_egress_allowlist(&self, topics: &[String]) -> TransportResult<()> {
        let canonical: HashSet<String> = topics
            .iter()
            .map(|t| crate::transport::network::canonical_topic(t).into_owned())
            .collect();
        let mut gate = self.egress.lock().map_err(|_| TransportError::Internal {
            reason: "bridge manager egress-gate mutex poisoned".to_string(),
        })?;
        gate.policy = EgressPolicy::AllowList(canonical);
        gate.refused_logged.clear();
        Ok(())
    }

    /// Install the explicit permissive-egress posture
    /// (`EgressPolicy::AllowAll`) — after this call ANY registered topic's
    /// bridge flag may flip true, with no per-topic refusal log. The
    /// permissive-default `graph run` arm calls this for a graph with NO
    /// `network:` block (every produced topic is network-viewable until
    /// pairing lands); tests and embedders that want a raw manager to egress opt
    /// in HERE (the strict default is deny-all). A graph build (or embedder) can
    /// be proven to have EXPLICITLY opted in via [`Self::egress_is_allow_all`].
    /// REPLACE semantics like [`Self::set_egress_allowlist`]: resets the refusal
    /// latch (harmless — `AllowAll` never refuses).
    pub fn set_egress_allow_all(&self) -> TransportResult<()> {
        let mut gate = self.egress.lock().map_err(|_| TransportError::Internal {
            reason: "bridge manager egress-gate mutex poisoned".to_string(),
        })?;
        gate.policy = EgressPolicy::AllowAll;
        gate.refused_logged.clear();
        Ok(())
    }

    /// Whether the explicit permissive-egress posture
    /// (`EgressPolicy::AllowAll`) is installed (Principle #3: observable
    /// state). Distinguishes an EXPLICIT permissive-egress opt-in from the
    /// default deny-all `AllowList(empty)` — the default refuses every topic,
    /// `AllowAll` admits every registered one, so this is the only way to tell
    /// them apart from outside. Poison-mapped like the other gate readers.
    pub fn egress_is_allow_all(&self) -> TransportResult<bool> {
        let gate = self.egress.lock().map_err(|_| TransportError::Internal {
            reason: "bridge manager egress-gate mutex poisoned".to_string(),
        })?;
        Ok(matches!(gate.policy, EgressPolicy::AllowAll))
    }

    /// Register a TOPIC's egress bridge flag (topic-keyed — no
    /// publisher state). Returns the `Arc<AtomicBool>` the demand watch flips
    /// and the gateway reads. The gateway calls this at boot for every announced
    /// topic; the flag starts `false` (no demand yet).
    ///
    /// Idempotent: a repeat call for the same canonical topic returns the
    /// existing flag.
    pub fn register_topic(&self, topic: &str) -> TransportResult<Arc<AtomicBool>> {
        // An empty name would canonicalize
        // to "/" — a flag keyed by a malformed name no liveliness key can
        // ever match (a silently-dead bridge). The production path is
        // guarded at a distance (validate_topic_name), but this is a pub
        // API.
        if topic.is_empty() {
            return Err(TransportError::Internal {
                reason: "bridge registration requires a non-empty topic name".to_string(),
            });
        }
        // Key the map by the CANONICAL name so a flag
        // registered under a raw no-slash name still matches the
        // liveliness watcher's recovered canonical name.
        let topic = crate::transport::network::canonical_topic(topic);
        let mut bridges = self.bridges.lock().map_err(|_| TransportError::Internal {
            reason: "bridge manager mutex poisoned".to_string(),
        })?;
        let before = bridges.len();
        let flag = bridges
            .entry(topic.into_owned())
            .or_insert_with(|| Arc::new(AtomicBool::new(false)))
            .clone();
        let grew = bridges.len() != before;
        drop(bridges);
        // A NEW egress topic gives a zero-demand gateway
        // something to do (reconcile its drive view, announce the topic), so wake
        // it. Gated on a genuine growth because this call is IDEMPOTENT and the
        // runtime-registration republish pump re-registers the whole set
        // periodically — an unconditional bump would wake the idle loop on every
        // republish for nothing.
        if grew {
            self.demand_signal.signal();
        }
        Ok(flag)
    }

    /// Enable network bridge for a topic (called when remote subscriber appears).
    ///
    /// No-op if the topic has no registered flag.
    ///
    /// An egress ALLOW-LIST gates every flip. A non-member
    /// topic is REFUSED here: its flag is never flipped, so nothing it
    /// produces can leave the machine no matter what a remote token asks for.
    /// The refusal logs once per topic (`warn!` — on an auth-less robot LAN a
    /// remote asking for an undeclared topic is a signal the operator should
    /// SEE at the default filter; the once-per-topic latch keeps liveliness
    /// churn from flooding). The DEFAULT posture is deny-all
    /// (`AllowList(empty)`), so a manager that installed no policy (a raw
    /// transport user, or a test that never opted in) refuses every topic via
    /// this SAME once-per-topic path — nothing egresses by construction. Only
    /// the explicit `EgressPolicy::AllowAll` posture
    /// ([`Self::set_egress_allow_all`]) admits any registered topic.
    ///
    /// Returns whether this call genuinely TRANSITIONED the flag false→true.
    /// The demand reconciler calls this every ~1 s for every
    /// demanded topic; an unconditional `info!` after the store logged once per
    /// second per topic forever in steady state. The log is now transition-gated
    /// (fires only on the false→true edge), and the returned bool lets the
    /// reconciler attribute a flip to itself ONLY when it actually caused the
    /// transition (0 on a healthy link where the liveliness subscriber won the
    /// race). A gate-refused or already-true call returns `false`.
    pub fn enable_bridge(&self, topic: &str) -> TransportResult<bool> {
        let topic = crate::transport::network::canonical_topic(topic);
        // Egress-posture gate FIRST (own lock scope — never held across the
        // bridges lock below). Only AllowAll admits unconditionally; an
        // AllowList (including the DEFAULT empty deny-all) refuses non-members.
        {
            let mut gate = self.egress.lock().map_err(|_| TransportError::Internal {
                reason: "bridge manager egress-gate mutex poisoned".to_string(),
            })?;
            if let EgressPolicy::AllowList(list) = &gate.policy {
                if !list.contains(topic.as_ref()) {
                    if gate.refused_logged.insert(topic.as_ref().to_owned()) {
                        tracing::warn!(
                            topic = %topic,
                            "bridge-enable refused: topic is not in this graph's `network: \
                             egress:` SCOPING list — a per-graph SAFETY FILTER limiting \
                             which produced topics THIS graph exports, NOT an authorization \
                             boundary (the account/pairing grant authorizes network access). Add \
                             the topic to `egress:` to export \
                             it (logged once per topic)"
                        );
                    }
                    return Ok(false);
                }
            }
        }
        let bridges = self.bridges.lock().map_err(|_| TransportError::Internal {
            reason: "bridge manager mutex poisoned".to_string(),
        })?;
        if let Some(flag) = bridges.get(topic.as_ref()) {
            // Transition-guard the log (F-C0): only the false→true EDGE is a
            // lifecycle event worth an `info!`; a re-affirm by the per-second
            // reconciler is silent.
            let was = flag.swap(true, Ordering::Relaxed);
            if !was {
                tracing::info!(topic = %topic, "network bridge enabled — remote demand observed");
                // THE demand transition. Wake any gateway parked
                // on the zero-demand idle wait so it attaches this topic's tap
                // now rather than at its next fallback tick. Transition-gated for
                // the same reason the `info!` above is: the reconciler
                // re-affirms every demanded topic ~1 Hz, and an unconditional
                // bump would wake a busy gateway's idle wait for a no-op.
                //
                // Bumping while the `bridges` lock is held is deliberate and
                // safe: `DemandSignal::signal` takes only its OWN mutex and never
                // calls back into this manager, so there is no lock-order edge to
                // invert. Doing it here also means the generation moves BEFORE
                // this function returns, so a waiter that snapshotted the
                // generation before its flag scan cannot observe the flag as OFF
                // and then miss the bump.
                self.demand_signal.signal();
            }
            return Ok(!was);
        }
        Ok(false)
    }

    /// The number of registered egress bridge flags (Principle #3:
    /// observable state). The `bridges` map is ADD-ONLY — [`Self::register_topic`]
    /// inserts, and no path ever removes a key ([`Self::enable_bridge`] /
    /// [`Self::disable_bridge`] only flip existing flags) — so this count is
    /// MONOTONIC. The gateway's [`crate::transport::gateway::GatewayRuntime`] uses
    /// that monotonicity as a cheap dirty-check: a count change is a reliable
    /// "a new egress topic was registered" signal driving its per-pass tap
    /// reconciliation, without snapshotting the whole map every pass.
    pub fn registered_topic_count(&self) -> TransportResult<usize> {
        let bridges = self.bridges.lock().map_err(|_| TransportError::Internal {
            reason: "bridge manager mutex poisoned".to_string(),
        })?;
        Ok(bridges.len())
    }

    /// A snapshot of every registered egress topic + its shared flag,
    /// SORTED by canonical topic name for deterministic iteration (Principle #3).
    /// This map is THE source of truth for which topics may egress — the gateway
    /// reconciles its tap set + lazily-minted per-topic counters against this
    /// snapshot (a runtime-registered topic appears here exactly like a
    /// boot-plan-declared one). Each returned `Arc<AtomicBool>` is the SAME flag
    /// the demand watch / queryable flip via [`Self::enable_bridge`], so a clone
    /// held by the gateway observes demand transitions.
    pub fn registered_topics(&self) -> TransportResult<Vec<(String, Arc<AtomicBool>)>> {
        let bridges = self.bridges.lock().map_err(|_| TransportError::Internal {
            reason: "bridge manager mutex poisoned".to_string(),
        })?;
        let mut out: Vec<(String, Arc<AtomicBool>)> = bridges
            .iter()
            .map(|(topic, flag)| (topic.clone(), Arc::clone(flag)))
            .collect();
        out.sort_by(|a, b| a.0.cmp(&b.0));
        Ok(out)
    }

    /// Whether `topic` (canonical-keyed) has ever hit the egress
    /// allow-list REFUSAL path in [`Self::enable_bridge`] — i.e. a demand token
    /// asked to export it and the posture (a Strict allow-list, or the default
    /// deny-all) refused. Reads the once-per-topic `refused_logged` latch, which
    /// is the durable record of that decision (Principle #3: observable state).
    /// `AllowAll` never refuses, so this is always `false` under a permissive
    /// posture. Used by the self-ingress e2e as the DETERMINISTIC signal
    /// (a background-thread `warn!` cannot be captured by `tracing_test`): a
    /// foreign demand IS refused (`true`), while the machine's OWN ingress demand
    /// is SKIPPED before `enable_bridge` (stays `false`). Poison-mapped like the
    /// other gate readers.
    pub fn was_egress_refused(&self, topic: &str) -> TransportResult<bool> {
        let topic = crate::transport::network::canonical_topic(topic);
        let gate = self.egress.lock().map_err(|_| TransportError::Internal {
            reason: "bridge manager egress-gate mutex poisoned".to_string(),
        })?;
        Ok(gate.refused_logged.contains(topic.as_ref()))
    }

    /// Whether an egress bridge flag has been registered for
    /// this topic (canonical-keyed, so a raw no-slash query matches a flag
    /// registered under the canonical name and vice versa). A gateway registers
    /// these for its ANNOUNCED (egress) topics via [`Self::register_topic`].
    ///
    /// The network→local ingress path ([`crate::transport::TransportManager::create_ingress_publisher`])
    /// reads this to REFUSE injecting remote frames into a topic this gateway
    /// already taps OUT to the network — otherwise a re-injected frame would be
    /// tapped → egressed → re-injected, an infinite loop (Principle #6:
    /// pathological state must not be silent).
    pub fn is_registered(&self, topic: &str) -> TransportResult<bool> {
        let topic = crate::transport::network::canonical_topic(topic);
        let bridges = self.bridges.lock().map_err(|_| TransportError::Internal {
            reason: "bridge manager mutex poisoned".to_string(),
        })?;
        Ok(bridges.contains_key(topic.as_ref()))
    }

    /// Whether `topic`'s egress bridge flag is
    /// currently ON (registered AND flipped true). The demand QUERYABLE handler
    /// reads this AFTER [`Self::enable_bridge`] to decide the SYNCHRONOUS ack it
    /// returns to the demander — `true` ⇒ the topic is egressing (allowed),
    /// `false` ⇒ refused (allow-list miss) or unregistered. Distinct from
    /// [`Self::enable_bridge`]'s TRANSITION bool (which returns `false` for an
    /// already-on topic that IS egressing), so an ack built off the transition
    /// alone would mislabel an already-enabled re-demand as refused. Canonical-
    /// keyed; poison-mapped like the other readers.
    pub fn is_enabled(&self, topic: &str) -> TransportResult<bool> {
        let topic = crate::transport::network::canonical_topic(topic);
        let bridges = self.bridges.lock().map_err(|_| TransportError::Internal {
            reason: "bridge manager mutex poisoned".to_string(),
        })?;
        Ok(bridges
            .get(topic.as_ref())
            .map(|flag| flag.load(Ordering::Relaxed))
            .unwrap_or(false))
    }

    /// Disable network bridge for a topic (called when the topic's last remote
    /// demand goes away — the demand reconciler after `REMOVE_CONFIRM` consecutive
    /// absences, or the liveliness subscriber on a `Delete`).
    ///
    /// No-op if the topic has no registered flag. Returns whether this call
    /// genuinely TRANSITIONED the flag true→false. This function
    /// does not know WHY it was called, so it does NOT log the reason — the
    /// CALLER logs its own precise cause on a genuine transition (F-C1/S1: the
    /// old shared `"no remote subscribers"` message implied real-time knowledge
    /// the periodic, hysteresis-gated reconciler does not have). The transition
    /// bool lets each caller gate its log to the true→false edge.
    pub fn disable_bridge(&self, topic: &str) -> TransportResult<bool> {
        let topic = crate::transport::network::canonical_topic(topic);
        let bridges = self.bridges.lock().map_err(|_| TransportError::Internal {
            reason: "bridge manager mutex poisoned".to_string(),
        })?;
        if let Some(flag) = bridges.get(topic.as_ref()) {
            return Ok(flag.swap(false, Ordering::Relaxed));
        }
        Ok(false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_register_topic_returns_bridge_flag() {
        let mgr = TopicBridgeManager::new();
        let flag = mgr.register_topic("sensor/lidar").unwrap();
        assert!(!flag.load(Ordering::Relaxed));
    }

    #[test]
    fn test_register_same_topic_returns_same_flag() {
        let mgr = TopicBridgeManager::new();
        let flag1 = mgr.register_topic("sensor/lidar").unwrap();
        let flag2 = mgr.register_topic("sensor/lidar").unwrap();
        assert!(Arc::ptr_eq(&flag1, &flag2));
    }

    #[test]
    fn test_register_different_topics_returns_different_flags() {
        let mgr = TopicBridgeManager::new();
        let a = mgr.register_topic("sensor/lidar").unwrap();
        let b = mgr.register_topic("camera/image").unwrap();
        assert!(!Arc::ptr_eq(&a, &b));
        assert!(!a.load(Ordering::Relaxed));
        assert!(!b.load(Ordering::Relaxed));
    }

    #[test]
    fn test_enable_bridge_sets_flag_true() {
        let mgr = TopicBridgeManager::new();
        let flag = mgr.register_topic("sensor/lidar").unwrap();
        // The default posture is deny-all — opt into permissive egress
        // to exercise the enable MECHANICS (the deny-all default is pinned by
        // `fresh_manager_default_denies_egress`).
        mgr.set_egress_allow_all().unwrap();
        mgr.enable_bridge("sensor/lidar").unwrap();
        assert!(flag.load(Ordering::Relaxed));
    }

    #[test]
    fn test_disable_bridge_sets_flag_false() {
        let mgr = TopicBridgeManager::new();
        let flag = mgr.register_topic("sensor/lidar").unwrap();
        mgr.set_egress_allow_all().unwrap(); // Explicit opt-in (deny-all default).
        mgr.enable_bridge("sensor/lidar").unwrap();
        assert!(flag.load(Ordering::Relaxed));
        mgr.disable_bridge("sensor/lidar").unwrap();
        assert!(!flag.load(Ordering::Relaxed));
    }

    #[test]
    fn test_enable_bridge_unregistered_topic_is_noop() {
        let mgr = TopicBridgeManager::new();
        // Opt into permissive egress so the gate ADMITS — proving the
        // no-op is due to the topic being UNREGISTERED, not the deny-all gate.
        mgr.set_egress_allow_all().unwrap();
        // Should not panic
        mgr.enable_bridge("nonexistent/topic").unwrap();
        // Subsequent registration should start as false (no ghost entry)
        let flag = mgr.register_topic("nonexistent/topic").unwrap();
        assert!(!flag.load(Ordering::Relaxed));
    }

    #[test]
    fn test_disable_bridge_unregistered_topic_is_noop() {
        let mgr = TopicBridgeManager::new();
        // Should not panic
        mgr.disable_bridge("nonexistent/topic").unwrap();
    }

    #[test]
    fn test_enable_disable_cycle() {
        let mgr = TopicBridgeManager::new();
        let flag = mgr.register_topic("sensor/lidar").unwrap();
        mgr.set_egress_allow_all().unwrap(); // Explicit opt-in (deny-all default).

        mgr.enable_bridge("sensor/lidar").unwrap();
        assert!(flag.load(Ordering::Relaxed));

        mgr.disable_bridge("sensor/lidar").unwrap();
        assert!(!flag.load(Ordering::Relaxed));

        mgr.enable_bridge("sensor/lidar").unwrap();
        assert!(flag.load(Ordering::Relaxed));
    }

    #[test]
    fn test_is_registered_reflects_registration_canonically() {
        let mgr = TopicBridgeManager::new();
        // Unregistered topic → false.
        assert!(!mgr.is_registered("sensor/lidar").unwrap());
        // After registration → true, keyed canonically (raw query matches).
        let _flag = mgr.register_topic("sensor/lidar").unwrap();
        assert!(mgr.is_registered("sensor/lidar").unwrap());
        // Canonical query for the same topic matches the raw-registered flag.
        assert!(mgr.is_registered("/sensor/lidar").unwrap());
        // A different topic stays unregistered.
        assert!(!mgr.is_registered("camera/image").unwrap());
    }

    #[test]
    fn test_register_canonical_is_registered_via_raw_query() {
        // Register under the canonical (leading-slash) name; a raw no-slash
        // query must still report it registered (both canonicalize identically).
        let mgr = TopicBridgeManager::new();
        let _flag = mgr.register_topic("/perception/depth").unwrap();
        assert!(mgr.is_registered("perception/depth").unwrap());
        assert!(mgr.is_registered("/perception/depth").unwrap());
    }

    #[test]
    fn test_bridge_flag_observable_from_cloned_arc() {
        let mgr = TopicBridgeManager::new();
        let flag = mgr.register_topic("sensor/lidar").unwrap();
        let cloned = Arc::clone(&flag);
        mgr.set_egress_allow_all().unwrap(); // Explicit opt-in (deny-all default).

        // Simulate: the gateway holds `cloned`, bridge manager toggles via `enable_bridge`
        mgr.enable_bridge("sensor/lidar").unwrap();
        assert!(cloned.load(Ordering::Relaxed));
    }

    // ---- The registered-topic snapshot surface (Principle #3) — the
    // source of truth the gateway reconciles its tap set from. ----

    /// `registered_topic_count` is the monotonic add-only count the gateway uses
    /// as a cheap dirty-check. Empty → 0; each DISTINCT canonical registration
    /// bumps it by one; an idempotent re-register does NOT; enable/disable never
    /// change it (they flip existing flags, never add/remove keys). Hand oracle.
    #[test]
    fn registered_topic_count_is_monotonic_and_dedup_exact() {
        let mgr = TopicBridgeManager::new();
        assert_eq!(
            mgr.registered_topic_count().unwrap(),
            0,
            "empty starts at 0"
        );
        mgr.register_topic("/a").unwrap();
        assert_eq!(mgr.registered_topic_count().unwrap(), 1);
        mgr.register_topic("/b").unwrap();
        assert_eq!(mgr.registered_topic_count().unwrap(), 2);
        // Idempotent re-register (raw spelling of /a) → no bump.
        mgr.register_topic("a").unwrap();
        assert_eq!(
            mgr.registered_topic_count().unwrap(),
            2,
            "a canonical-duplicate registration must not bump the count"
        );
        // enable/disable flip flags, never touch the key set.
        mgr.set_egress_allow_all().unwrap();
        mgr.enable_bridge("/a").unwrap();
        mgr.disable_bridge("/a").unwrap();
        assert_eq!(
            mgr.registered_topic_count().unwrap(),
            2,
            "enable/disable must never change the registered count"
        );
    }

    /// `registered_topics` snapshots (canonical, flag) SORTED, deduped, with the
    /// SAME flag `Arc` the manager stores (so a gateway clone observes an
    /// enable). Hand oracle for names + order; `Arc::ptr_eq` for flag identity.
    #[test]
    fn registered_topics_snapshot_is_sorted_deduped_and_shares_flags() {
        let mgr = TopicBridgeManager::new();
        // Register out of order + a raw duplicate of /alpha.
        let gamma = mgr.register_topic("/gamma").unwrap();
        let _alpha = mgr.register_topic("/alpha").unwrap();
        let _alpha_dup = mgr.register_topic("alpha").unwrap(); // canonical dup
        let snap = mgr.registered_topics().unwrap();
        let names: Vec<String> = snap.iter().map(|(t, _)| t.clone()).collect();
        assert_eq!(
            names,
            vec!["/alpha".to_string(), "/gamma".to_string()],
            "snapshot must be sorted + deduped canonically"
        );
        // The snapshot's /gamma flag is the SAME Arc the manager stores: an
        // enable via the manager is visible through the snapshot clone.
        let snap_gamma = &snap.iter().find(|(t, _)| t == "/gamma").unwrap().1;
        assert!(
            Arc::ptr_eq(snap_gamma, &gamma),
            "snapshot shares the flag Arc"
        );
        mgr.set_egress_allow_all().unwrap();
        mgr.enable_bridge("/gamma").unwrap();
        assert!(
            snap_gamma.load(Ordering::Relaxed),
            "an enable through the manager is observable via the snapshot's shared flag"
        );
    }

    // ---- The egress POSTURE gate (pure, no
    // transport). The graph-build wiring + the once-per-topic refusal-log pin
    // live in `tests/network_graph_wiring_test.rs`. The mechanics tests ABOVE
    // opt into `AllowAll` explicitly — the DEFAULT is deny-all, pinned next. ----

    /// The default posture is deny-all
    /// (`AllowList(empty)`) — a raw manager that never installed a policy
    /// egresses NOTHING. A registered topic's flag is REFUSED (the flag stays
    /// false) via the SAME once-per-topic path as any allow-list miss, and the
    /// manager is NOT `AllowAll`. Opting into `AllowAll` then admits it — the
    /// anti-tautology that the apparatus can flip.
    #[test]
    fn fresh_manager_default_denies_egress() {
        let mgr = TopicBridgeManager::new();
        let flag = mgr.register_topic("/nw/topic").unwrap();
        assert!(
            !mgr.egress_is_allow_all().unwrap(),
            "a fresh manager is not AllowAll — the default is deny-all"
        );
        // Deny-all: an un-installed gate refuses a registered topic.
        mgr.enable_bridge("/nw/topic").unwrap();
        assert!(
            !flag.load(Ordering::Relaxed),
            "the default deny-all posture must refuse every topic"
        );
        // Explicit opt-in admits it (the apparatus is load-bearing).
        mgr.set_egress_allow_all().unwrap();
        mgr.enable_bridge("/nw/topic").unwrap();
        assert!(
            flag.load(Ordering::Relaxed),
            "AllowAll must admit the topic the deny-all default refused"
        );
    }

    /// `is_enabled` reports the flag state the demand queryable acks
    /// on — false when unregistered, false when registered-but-refused (deny),
    /// true after an allowed enable, false again after disable. The anti-
    /// tautology vs `enable_bridge`'s transition bool: after a SECOND allowed
    /// enable (already on), `enable_bridge` returns `false` (no transition) yet
    /// `is_enabled` stays `true` — so an ack must read `is_enabled`, not the
    /// transition, to avoid mislabeling an already-egressing re-demand.
    #[test]
    fn is_enabled_tracks_flag_state_for_the_ack() {
        let mgr = TopicBridgeManager::new();
        // Unregistered → not enabled.
        assert!(!mgr.is_enabled("/nw/topic").unwrap());
        let _flag = mgr.register_topic("/nw/topic").unwrap();
        // Registered but deny-all default → refused → not enabled.
        mgr.enable_bridge("/nw/topic").unwrap();
        assert!(!mgr.is_enabled("/nw/topic").unwrap());
        // Allowed enable → enabled.
        mgr.set_egress_allow_all().unwrap();
        assert!(
            mgr.enable_bridge("/nw/topic").unwrap(),
            "first enable transitions"
        );
        assert!(mgr.is_enabled("/nw/topic").unwrap());
        // Second enable: no transition, still enabled (the ack must not flip to
        // refused on an already-egressing re-demand).
        assert!(
            !mgr.enable_bridge("/nw/topic").unwrap(),
            "second enable is not a transition"
        );
        assert!(mgr.is_enabled("/nw/topic").unwrap());
        // Disable → not enabled.
        mgr.disable_bridge("/nw/topic").unwrap();
        assert!(!mgr.is_enabled("/nw/topic").unwrap());
    }

    /// A member topic's flag flips; a non-member's NEVER does — the
    /// structural refusal (enable_bridge is the only true-writer).
    #[test]
    fn allowlist_member_flips_non_member_never_does() {
        let mgr = TopicBridgeManager::new();
        let member = mgr.register_topic("/nw/declared").unwrap();
        let outsider = mgr.register_topic("/nw/undeclared").unwrap();
        mgr.set_egress_allowlist(&["/nw/declared".to_string()])
            .unwrap();

        mgr.enable_bridge("/nw/declared").unwrap();
        mgr.enable_bridge("/nw/undeclared").unwrap();
        assert!(member.load(Ordering::Relaxed), "member must flip");
        assert!(
            !outsider.load(Ordering::Relaxed),
            "a non-member flag must NEVER read true"
        );
        // Repeated foreign tokens change nothing.
        mgr.enable_bridge("/nw/undeclared").unwrap();
        assert!(!outsider.load(Ordering::Relaxed));
    }

    /// An EMPTY list is deny-all — the ingress-only-graph state (its watch
    /// runs for ingress liveliness, but nothing produced may leave).
    #[test]
    fn allowlist_empty_denies_all() {
        let mgr = TopicBridgeManager::new();
        let flag = mgr.register_topic("/nw/topic").unwrap();
        mgr.set_egress_allowlist(&[]).unwrap();
        mgr.enable_bridge("/nw/topic").unwrap();
        assert!(
            !flag.load(Ordering::Relaxed),
            "an empty allow-list must refuse every topic"
        );
    }

    /// REPLACE semantics: a second install overwrites the first (and the
    /// new list's verdicts apply immediately).
    #[test]
    fn allowlist_replace_changes_verdict() {
        let mgr = TopicBridgeManager::new();
        let flag_b = mgr.register_topic("/nw/b").unwrap();
        mgr.set_egress_allowlist(&["/nw/a".to_string()]).unwrap();
        mgr.enable_bridge("/nw/b").unwrap();
        assert!(!flag_b.load(Ordering::Relaxed), "b refused under list [a]");

        mgr.set_egress_allowlist(&["/nw/b".to_string()]).unwrap();
        mgr.enable_bridge("/nw/b").unwrap();
        assert!(
            flag_b.load(Ordering::Relaxed),
            "b must flip after the replacing list admits it"
        );
    }

    /// The list is canonical-keyed both ways: a raw no-slash entry matches
    /// a canonical enable and vice versa (same aliasing contract as
    /// registration).
    #[test]
    fn allowlist_is_canonical_keyed_both_ways() {
        let mgr = TopicBridgeManager::new();
        let raw_listed = mgr.register_topic("raw/listed").unwrap();
        let canon_listed = mgr.register_topic("/canon/listed").unwrap();
        // One entry raw, one canonical.
        mgr.set_egress_allowlist(&["raw/listed".to_string(), "/canon/listed".to_string()])
            .unwrap();

        // Enable via the OPPOSITE spelling of each.
        mgr.enable_bridge("/raw/listed").unwrap();
        mgr.enable_bridge("canon/listed").unwrap();
        assert!(raw_listed.load(Ordering::Relaxed));
        assert!(canon_listed.load(Ordering::Relaxed));
    }

    /// `disable_bridge` is NOT gated: disabling (storing false) is always
    /// safe, including for a topic a replaced list no longer admits.
    #[test]
    fn allowlist_never_blocks_disable() {
        let mgr = TopicBridgeManager::new();
        let flag = mgr.register_topic("/nw/was_allowed").unwrap();
        mgr.set_egress_allowlist(&["/nw/was_allowed".to_string()])
            .unwrap();
        mgr.enable_bridge("/nw/was_allowed").unwrap();
        assert!(flag.load(Ordering::Relaxed));

        // Replace with a list that no longer admits it; the LOWER edge
        // (disable) must still work — a lingering true flag would keep
        // exporting a topic the new list forbids.
        mgr.set_egress_allowlist(&[]).unwrap();
        mgr.disable_bridge("/nw/was_allowed").unwrap();
        assert!(
            !flag.load(Ordering::Relaxed),
            "disable must always be able to lower a flag"
        );
        // And it can never come back up under the deny-all list.
        mgr.enable_bridge("/nw/was_allowed").unwrap();
        assert!(!flag.load(Ordering::Relaxed));
    }

    // ---- The explicit default-permissive posture (AllowAll). ----

    /// `AllowAll` admits ANY registered topic (no allow-list membership) — the
    /// permissive-default posture. And it is observable as such.
    #[test]
    fn allow_all_flips_any_registered_topic() {
        let mgr = TopicBridgeManager::new();
        let a = mgr.register_topic("/nw/a").unwrap();
        let b = mgr.register_topic("/nw/b").unwrap();
        // Fresh manager is NOT AllowAll (the deny-all AllowList default).
        assert!(!mgr.egress_is_allow_all().unwrap());

        mgr.set_egress_allow_all().unwrap();
        assert!(mgr.egress_is_allow_all().unwrap());
        mgr.enable_bridge("/nw/a").unwrap();
        mgr.enable_bridge("/nw/b").unwrap();
        assert!(
            a.load(Ordering::Relaxed),
            "AllowAll admits any registered topic"
        );
        assert!(
            b.load(Ordering::Relaxed),
            "AllowAll admits any registered topic"
        );
    }

    /// REPLACE semantics both ways: an `AllowList` REPLACED by `AllowAll` admits
    /// a formerly-refused topic; and `egress_is_allow_all` tracks the posture (a
    /// bare `AllowList` is NOT AllowAll — the anti-tautology for the observable).
    #[test]
    fn allow_all_replaces_allowlist_and_is_distinct_from_it() {
        let mgr = TopicBridgeManager::new();
        let b = mgr.register_topic("/nw/b").unwrap();
        mgr.set_egress_allowlist(&["/nw/a".to_string()]).unwrap();
        assert!(
            !mgr.egress_is_allow_all().unwrap(),
            "an AllowList is not the AllowAll posture"
        );
        mgr.enable_bridge("/nw/b").unwrap();
        assert!(!b.load(Ordering::Relaxed), "b refused under the list [a]");

        // Replace the list with the permissive-default posture.
        mgr.set_egress_allow_all().unwrap();
        assert!(mgr.egress_is_allow_all().unwrap());
        mgr.enable_bridge("/nw/b").unwrap();
        assert!(
            b.load(Ordering::Relaxed),
            "AllowAll must admit a topic the replaced list refused"
        );
    }
}

/// Oracle-vector tests for [`DemandSignal`] — the demand-transition
/// wake a zero-demand gateway blocks on — and for the two [`TopicBridgeManager`]
/// edges that bump it.
///
/// The race-window arm here is the PURE half of the lost-wakeup pin: it fixes the
/// primitive's contract ("a bump that lands after the caller's snapshot but before
/// the wait returns at once"). The half that proves the GATEWAY takes its snapshot
/// in the right order is behavioural and lives in
/// `crates/cerulion_netd/tests/egress_plane_iox2_test.rs`, driven through the
/// `set_idle_wait_gate_for_test` seam.
#[cfg(test)]
mod demand_signal_tests {
    use super::*;
    use std::sync::atomic::AtomicU64;
    use std::time::Instant;

    /// A generous liveness ceiling. Never a wall in units of anything under test —
    /// load can delay a thread but cannot make a `wait_for_change` that returned
    /// `true` have blocked.
    const LIVENESS_CEILING: Duration = Duration::from_secs(10);

    #[test]
    fn a_fresh_signal_starts_at_zero_and_each_bump_advances_it_by_one() {
        let sig = DemandSignal::new();
        assert_eq!(sig.generation(), 0, "a fresh signal starts at generation 0");
        for expected in 1..=5u64 {
            sig.signal();
            assert_eq!(
                sig.generation(),
                expected,
                "each signal advances the generation by exactly one"
            );
        }
    }

    /// THE race-window oracle. A bump that lands between the caller's snapshot and
    /// the wait must return IMMEDIATELY — the whole reason this is a generation
    /// counter and not a flag. Asserted on the RETURN VALUE (`true` = the
    /// generation moved) plus a wall CEILING far under the timeout: an
    /// implementation that blocked would spend the full 10 s.
    #[test]
    fn a_bump_between_the_snapshot_and_the_wait_returns_at_once() {
        let sig = DemandSignal::new();
        // The caller's snapshot — taken BEFORE the decision it will block on.
        let seen = sig.generation();
        // ... the demand lands HERE, inside the window.
        sig.signal();
        let started = Instant::now();
        assert!(
            sig.wait_for_change(seen, LIVENESS_CEILING),
            "a generation already past the snapshot must report a real signal"
        );
        assert!(
            started.elapsed() < LIVENESS_CEILING,
            "it must not have blocked at all, let alone waited out the timeout"
        );
    }

    /// The complement: with NOTHING signalled, the wait SPENDS its timeout and
    /// reports `false`. Without this arm, an implementation that always returned
    /// `true` immediately would pass every positive arm above while fabricating a
    /// wake on every idle tick.
    #[test]
    fn a_quiet_signal_spends_its_timeout_and_reports_no_change() {
        let sig = DemandSignal::new();
        let seen = sig.generation();
        let budget = Duration::from_millis(120);
        let started = Instant::now();
        assert!(
            !sig.wait_for_change(seen, budget),
            "a timeout is not a signal"
        );
        assert!(
            started.elapsed() >= budget,
            "the wait must really block for its budget, not return instantly (got {:?})",
            started.elapsed()
        );
    }

    /// A bump from ANOTHER thread while the waiter is parked wakes it — the real
    /// cross-thread path (`enable_bridge` runs on the zenoh callback / demand
    /// reconciler thread, the wait on the gateway drive thread). The waiter's
    /// budget is a generous CEILING; the load-bearing assertion is the returned
    /// `true`, which a timeout cannot produce.
    #[test]
    fn a_bump_from_another_thread_wakes_a_parked_waiter() {
        let sig = Arc::new(DemandSignal::new());
        let seen = sig.generation();
        let writer = Arc::clone(&sig);
        let handle = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(30));
            writer.signal();
        });
        assert!(
            sig.wait_for_change(seen, LIVENESS_CEILING),
            "a cross-thread bump must wake the parked waiter, not time out"
        );
        handle.join().expect("writer thread");
    }

    /// Every bump wakes EVERY waiter (`notify_all`) — a machine may run more than
    /// one gateway in-process (a test harness routinely does), and a `notify_one`
    /// would park the others for the whole fallback timeout. Hand oracle: all
    /// `WAITERS` report a real signal.
    #[test]
    fn one_bump_wakes_every_parked_waiter() {
        const WAITERS: usize = 4;
        let sig = Arc::new(DemandSignal::new());
        let seen = sig.generation();
        let woken = Arc::new(AtomicU64::new(0));
        let handles: Vec<_> = (0..WAITERS)
            .map(|_| {
                let sig = Arc::clone(&sig);
                let woken = Arc::clone(&woken);
                std::thread::spawn(move || {
                    if sig.wait_for_change(seen, LIVENESS_CEILING) {
                        woken.fetch_add(1, Ordering::Relaxed);
                    }
                })
            })
            .collect();
        // Give the waiters time to park; even if they have not yet, the generation
        // snapshot they hold is `seen`, so the bump below is never lost.
        std::thread::sleep(Duration::from_millis(20));
        sig.signal();
        for h in handles {
            h.join().expect("waiter thread");
        }
        assert_eq!(
            woken.load(Ordering::Relaxed),
            WAITERS as u64,
            "one bump must wake every waiter"
        );
    }

    /// A genuine `false → true` demand transition bumps the signal; a RE-AFFIRM of
    /// an already-demanded topic does NOT. Both halves matter: the demand
    /// reconciler re-affirms every demanded topic ~1 Hz, so an unconditional bump
    /// would wake an idle gateway forever for a no-op.
    #[test]
    fn only_a_genuine_demand_transition_bumps_the_signal() {
        let mgr = TopicBridgeManager::new();
        mgr.set_egress_allow_all().unwrap();
        mgr.register_topic("/nw/t").unwrap();
        let sig = mgr.demand_signal();

        let before = sig.generation();
        assert!(mgr.enable_bridge("/nw/t").unwrap(), "the false→true edge");
        assert_eq!(
            sig.generation(),
            before + 1,
            "a genuine demand transition bumps exactly once"
        );

        let after_edge = sig.generation();
        assert!(
            !mgr.enable_bridge("/nw/t").unwrap(),
            "a re-affirm is not a transition"
        );
        assert_eq!(
            sig.generation(),
            after_edge,
            "a re-affirm must NOT bump — the reconciler re-affirms ~1 Hz forever"
        );
    }

    /// A REFUSED enable (allow-list miss) never bumps — the flag did not flip, so
    /// there is nothing for a zero-demand gateway to wake up and do.
    #[test]
    fn a_refused_enable_never_bumps_the_signal() {
        let mgr = TopicBridgeManager::new();
        mgr.register_topic("/nw/refused").unwrap();
        mgr.set_egress_allowlist(&["/nw/other".to_string()])
            .unwrap();
        let sig = mgr.demand_signal();
        let before = sig.generation();
        assert!(!mgr.enable_bridge("/nw/refused").unwrap());
        assert_eq!(
            sig.generation(),
            before,
            "a refused enable flips nothing, so it must wake nothing"
        );
    }

    /// Registering a NEW topic bumps (the gateway has a new flag to watch and a new
    /// announce to make); re-registering the SAME topic does not. The second half
    /// is load-bearing: `register_topic` is idempotent and the runtime-registration
    /// republish pump re-registers the whole set periodically.
    #[test]
    fn only_a_growing_registered_set_bumps_the_signal() {
        let mgr = TopicBridgeManager::new();
        let sig = mgr.demand_signal();

        let before = sig.generation();
        mgr.register_topic("/nw/new").unwrap();
        assert_eq!(
            sig.generation(),
            before + 1,
            "a NEW egress topic wakes the idle gateway"
        );

        let after_new = sig.generation();
        mgr.register_topic("/nw/new").unwrap();
        assert_eq!(
            sig.generation(),
            after_new,
            "an idempotent re-register must NOT bump — the republish pump does this forever"
        );
    }

    /// `disable_bridge` deliberately does NOT bump: demand going away can only give
    /// the loop LESS to do, and the pass that observes it is already running (the
    /// loop only parks once nothing is ON). Pinned so a future edit that adds a
    /// bump there is a deliberate act rather than an accident.
    #[test]
    fn losing_demand_does_not_bump_the_signal() {
        let mgr = TopicBridgeManager::new();
        mgr.set_egress_allow_all().unwrap();
        mgr.register_topic("/nw/t").unwrap();
        mgr.enable_bridge("/nw/t").unwrap();
        let sig = mgr.demand_signal();
        let before = sig.generation();
        assert!(mgr.disable_bridge("/nw/t").unwrap(), "the true→false edge");
        assert_eq!(
            sig.generation(),
            before,
            "losing demand gives an idle loop nothing to do"
        );
    }

    /// The signal handed out is THE one the manager bumps (an `Arc` clone, not a
    /// fresh instance) — the anti-tautology for every arm above, all of which read
    /// through `demand_signal()`.
    #[test]
    fn the_handed_out_signal_is_the_one_the_manager_bumps() {
        let mgr = TopicBridgeManager::new();
        let a = mgr.demand_signal();
        let b = mgr.demand_signal();
        assert!(
            Arc::ptr_eq(&a, &b),
            "every caller must observe the SAME signal"
        );
        let before = a.generation();
        mgr.register_topic("/nw/ptr").unwrap();
        assert_eq!(b.generation(), before + 1, "both handles see the same bump");
    }
}
