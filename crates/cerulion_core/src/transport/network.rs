// SPDX-License-Identifier: AGPL-3.0-only
//! Zenoh network transport session and task lifecycle.
//!
//! Manages the zenoh session and a background network task for cross-machine
//! communication. Both are lazy: the session is created on first access, and the
//! network task starts only when the first network subscriber is added.
//!
//! # Design
//!
//! One zenoh session per process (Principle #8), lazy-initialized via `OnceCell`.
//! Multicast scouting is disabled by default to prevent accidental discovery
//! of unrelated zenoh nodes on the same network.
//!
//! The network task runs on a dedicated OS thread with a single-threaded tokio
//! runtime (Principle #9: use tokio). It starts when the first network subscriber
//! is registered and stops when the last one is removed.
//!
//! A gateway process ALSO runs a second dedicated OS thread — the demand
//! RECONCILER ([`NetworkManager::start_demand_reconciler`]) — the query-shaped
//! belt to the subscriber watch's suspenders: it runs one bounded liveliness
//! demand gather every ~1 s and reconciles the announced topics' egress flags,
//! so egress still starts when subscriber-interest propagation fails on a strict
//! connect-only link.

use std::collections::{BTreeSet, HashMap, HashSet};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, Weak};
use std::time::{Duration, Instant};

use once_cell::sync::OnceCell;
use zenoh::Wait;

use super::bridge::TopicBridgeManager;
use super::cerulion_q::{self, CatalogEntry, CatalogProvenance, QueryVerb, SchemaDoc};
use super::demand_authorizer::{
    AllowAllAuthorizer, DemandAuthorizer, DemandDecision, DemandSubject,
};
use super::discovery::{
    liveliness_prefix, query_announce_entries, query_live_topics_by_prefix, TopicToken,
};
use super::gateway::is_reserved_topic;
use super::publisher::CerulionPublisher;
use super::run_artifacts;
use super::run_registry;
use super::TransportManager;
use crate::error::{TransportError, TransportResult};
use crate::wire::WireHeader;

/// Zenoh session mode (node role in the network topology).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum ZenohMode {
    /// Peer mode: participates in mesh topology (default).
    #[default]
    Peer,
    /// Client mode: connects to a router, does not route traffic.
    Client,
    /// Router mode: accepts connections from clients and peers.
    Router,
}

impl ZenohMode {
    fn as_str(self) -> &'static str {
        match self {
            ZenohMode::Peer => "\"peer\"",
            ZenohMode::Client => "\"client\"",
            ZenohMode::Router => "\"router\"",
        }
    }
}

/// Configuration for the zenoh network transport.
///
/// Defaults are safe for local development: scouting disabled, no endpoints.
/// For cross-machine communication, configure `connect_endpoints` or
/// `listen_endpoints` to reach remote peers/routers.
#[derive(Debug, Clone)]
pub struct NetworkConfig {
    /// Zenoh mode (peer, client, or router).
    pub mode: ZenohMode,
    /// Remote endpoints to connect to (e.g., `["tcp/192.168.1.10:7447"]`).
    pub connect_endpoints: Vec<String>,
    /// Local endpoints to listen on (e.g., `["tcp/0.0.0.0:7447"]`).
    pub listen_endpoints: Vec<String>,
    /// Enable multicast scouting (disabled by default).
    pub multicast_scouting: bool,
    /// Enable gossip scouting (disabled by default).
    pub gossip_scouting: bool,
    /// HARD-BOUND the session's zenoh CONNECT phase at 1 s so a
    /// black-hole (dead / unroutable / SYN-dropping) connect locator cannot stall
    /// the caller past that bound. When `true`, `NetworkManager::build_zenoh_config`
    /// sets `connect/timeout_ms: 1000` (a POSITIVE global timeout — zenoh 1.8
    /// wraps the whole connect phase in `tokio::time::timeout` ONLY when this is
    /// non-zero; a `0` would run the attempts inline and unbounded, each paying
    /// up to the 10 s transport open_timeout), `connect/retry/period_init_ms: 0`
    /// (single inline attempt per endpoint — a reachable robot connects BEFORE
    /// the open returns, so a gather immediately after cannot race the TCP
    /// connect), and `connect/exit_on_failure: false` (a fast per-endpoint
    /// connect failure like ENETUNREACH does not fail the open).
    ///
    /// IMPORTANT — the 1 s bound is FATAL, not merely slow: zenoh 1.8's
    /// `start_peer` propagates the global-timeout Elapsed error UNCONDITIONALLY
    /// (`exit_on_failure` is NOT consulted on that arm) and tries endpoints
    /// SEQUENTIALLY, so ONE candidate that HANGS the connect (SYN-drop, or a TCP
    /// tarpit that accepts but never speaks zenoh) can consume the whole 1 s
    /// budget and make `zenoh::open` FAIL, losing every OTHER reachable robot.
    /// This flag is therefore only HALF the stall defense — the CALLER
    /// (`topic_cmd::query_remote_topics_with_candidates`) TCP-PRE-FILTERS EVERY
    /// connect endpoint before folding it in: ladder candidates at a short
    /// budget, and EXPLICIT `--connect` locators at a budget equal to this 1 s
    /// bound. An endpoint that cannot TCP-connect within its budget could not
    /// have contributed to the gather anyway, so it is SKIPPED FOR THE RUN (a
    /// loud warn for an explicit locator) rather than folded in — and the caller
    /// retries the open once with the reachable-explicit set on failure. Do NOT
    /// rely on `bounded_connect` alone to survive a hanging endpoint.
    ///
    /// Default `false` — gateways and graph runs keep zenoh's robust default
    /// retry semantics (`-1` = dead endpoints retry on a background connector).
    /// ONLY the EPHEMERAL `topic list` discovery query session sets it (that
    /// session is a one-shot bounded gather, so its connect phase must be
    /// hard-bounded — the 11 s stall fix).
    pub bounded_connect: bool,
    /// This machine's robot identity — the resolved hostname
    /// or `CERULION_ROBOT_IDENTITY` override, DECOUPLED from the graph prefix
    /// (identity is network-only, resolved at runtime). Rides as the first
    /// chunk of every ANNOUNCE key (`cerulion_ann/{robot}{canonical topic}`)
    /// and the bare
    /// identity token (`cerulion_ann/{robot}`), so `topic list` groups
    /// announced topics by PRODUCER (absolute mirror topics included) and the
    /// mDNS TXT `robot=` name matches by construction. REQUIRED for
    /// [`NetworkManager::announce_egress_topic`] /
    /// [`NetworkManager::announce_gateway_identity`] — announcing without it is
    /// a loud `Err` naming the fix. Must be a single clean key chunk (no `/`).
    /// `None` (the default) is fine for sessions that never announce
    /// (discovery queries, ingress-only bridges).
    pub robot_identity: Option<String>,
}

impl Default for NetworkConfig {
    fn default() -> Self {
        Self {
            mode: ZenohMode::Peer,
            connect_endpoints: vec![],
            listen_endpoints: vec![],
            multicast_scouting: false,
            gossip_scouting: false,
            bounded_connect: false,
            robot_identity: None,
        }
    }
}

/// Mutable state for the background network task.
struct TaskState {
    handle: Option<std::thread::JoinHandle<()>>,
    shutdown_tx: Option<tokio::sync::broadcast::Sender<()>>,
    subscriber_count: usize,
}

/// Mutable lifecycle state for one background helper thread — the
/// producer-side DEMAND-RECONCILER + expiry thread
/// ([`NetworkManager::start_demand_reconciler`]) AND the demander-side DEMAND-GET
/// loop ([`NetworkManager::start_demand_get_loop`]) each own one. Separate from
/// [`TaskState`] (the subscriber watch) because the threads have independent
/// lifecycles; each is stopped by [`NetworkManager::stop_bg_thread`] at Drop with
/// the SAME broadcast-channel-then-join pattern [`NetworkManager::stop_task`] uses.
#[derive(Default)]
struct BgThreadState {
    handle: Option<std::thread::JoinHandle<()>>,
    shutdown_tx: Option<tokio::sync::broadcast::Sender<()>>,
    /// Mid-pass shutdown: the demander DEMAND-GET loop runs a
    /// harvest + topics × selectors of SEQUENTIAL blocking GETs per pass, so a
    /// `Drop`-join could block for the whole pass (the SIGINT-reap path).
    /// [`NetworkManager::stop_bg_thread`] raises this flag BEFORE the shutdown
    /// broadcast; the pass polls it between GETs and returns early. Broadcast
    /// alone can only wake the ≤50 ms sleep, not interrupt a blocking GET —
    /// hence a separate `Fn`-pollable flag (`AtomicBool::load` needs no `&mut`,
    /// so the pass's `should_abort` stays a plain `Fn() -> bool`) that never
    /// touches the sleep receiver, keeping the loop's own shutdown detection
    /// untouched. `None` for the reconciler thread (its gather is one bounded
    /// window, not a sequential GET loop).
    abort: Option<Arc<AtomicBool>>,
}

/// A drop-guard that logs one loud `error!` if a demand helper
/// thread (the producer reconciler/expiry OR the demander GET loop) unwinds.
/// Armed (`clean = false`) at thread entry and disarmed (`clean = true`) only on
/// the clean shutdown-broadcast exit; a panic in a pass drops it with
/// `clean == false`, so the silent death of the belt (demand going
/// undiscovered with zero signal) becomes VISIBLE. No auto-restart machinery —
/// simple-is-better; the operator restarts the gateway. `msg` names the offender.
struct BgThreadDeathGuard {
    clean: bool,
    msg: &'static str,
}

impl Drop for BgThreadDeathGuard {
    fn drop(&mut self) {
        if !self.clean {
            tracing::error!(
                thread = self.msg,
                "a network background thread died unexpectedly — remote demand may no longer \
                 be discovered on links with broken liveliness propagation; restart \
                 the gateway"
            );
        }
    }
}

/// How often the demand reconciler runs one bounded liveliness gather.
///
/// WHY 1 s: the reconciler is a COARSE safety BELT, not the fast path. The
/// liveliness SUBSCRIBER ([`NetworkManager::start_task`]) stays the sub-ms
/// reaction to demand changes on a healthy link; the reconciler only matters
/// when subscriber-interest propagation FAILS (the strict connect-only
/// robot↔Mac link — bounded liveliness QUERIES still work there while the
/// subscriber never wakes). A 1 s cadence adds one lightweight bounded query
/// per second in steady state while bounding worst-case egress-start latency to
/// ~1 s when the subscriber path is broken.
const RECONCILE_INTERVAL: Duration = Duration::from_secs(1);

/// The bounded gather window for the reconciler's single liveliness
/// query per pass.
///
/// WHY 300 ms (and NEVER 0): this is a POSITIVE upper bound — zenoh 1.8 treats a
/// `0` timeout as inline-UNBOUNDED, which would let one pass block indefinitely.
/// The gather returns EARLY when the reply channel closes (query complete), so
/// on an established link a pass typically finishes in a few ms; 300 ms is the
/// generous ceiling sized for an already-connected connect-only link, not
/// initial discovery.
const RECONCILE_GATHER_WINDOW: Duration = Duration::from_millis(300);

/// Consecutive absences (missed gathers) before the reconciler DISABLES
/// a topic's egress flag.
///
/// WHY 3-pass hysteresis: it kills flapping AND makes REMOVAL robust. On the
/// broken link a consumer's departure `Delete` fails to propagate to the
/// producer's subscriber exactly as its `Put` did, so the reconciler is also the
/// removal authority; requiring 3 consecutive absences (~3 s) before disabling
/// avoids dropping live egress on a single transient query blip (a false
/// disable self-heals on the very next pass that re-sees the demand token).
const REMOVE_CONFIRM: u32 = 3;

/// Queryable inversion: how often the DEMANDER re-issues its demand
/// GETs. Each pass pulls every registered ingress topic's egress ON at the
/// producer (idempotent — a repeat GET re-affirms). A coarse pull, not a hot
/// path; it also serves as the producer's keepalive heartbeat (the last GET's
/// arrival time gates the producer expiry below).
const DEMAND_GET_INTERVAL: Duration = Duration::from_secs(2);

/// Bounded (POSITIVE — never 0) reply-gather window for ONE demand GET.
/// zenoh 1.8 treats a `0` timeout as inline-UNBOUNDED (which would let one dead
/// GET block a whole pass); 300 ms is a generous ceiling for an already-connected
/// link (the GET returns early when the reply channel closes).
const DEMAND_GET_TIMEOUT: Duration = Duration::from_millis(300);

/// Bounded window for the demander's per-pass identity harvest (a
/// `cerulion_ann/**` announce gather — a dialer→accepter query, the working
/// direction). POSITIVE, same rationale as [`DEMAND_GET_TIMEOUT`].
const DEMAND_IDENTITY_WINDOW: Duration = Duration::from_millis(300);

/// Producer-side GET-demand TTL. A GET-granted topic whose LAST demand
/// GET is older than this is EXPIRED; the producer's expiry sweep disables it
/// only when liveliness is ALSO absent (the composition rule). `3× DEMAND_GET_INTERVAL`
/// = hysteresis-equivalent (survives 2 missed GETs before releasing egress).
const DEMAND_TTL: Duration = Duration::from_secs(6);

/// The demand-queryable ACK payloads — the SYNCHRONOUS verdict a
/// producer returns to a demand GET (a tiny control ack, raw bytes, not a wire
/// message). The demander tallies these (Principle #3) and they are the
/// allow-list-refusal signal the demander sees directly.
const ACK_ENABLED: &[u8] = b"enabled";
/// See [`ACK_ENABLED`]. Returned for an allow-list miss / unregistered / a
/// suppressed producer.
const ACK_REFUSED: &[u8] = b"refused";

/// The synthetic RESOURCE label the `catalog` verb passes to the
/// [`DemandAuthorizer`] gate. The catalog enumerates the WHOLE machine (no single
/// topic), and an account gate authorizes on the SUBJECT (the demanding party's
/// account), not the topic — so this descriptor is informational (it names the
/// resource in a refusal log / a future per-resource policy), never a real topic.
const CATALOG_QUERY_RESOURCE: &str = "<catalog: all topics>";

/// The synthetic RESOURCE label the `runs` verb passes to the
/// [`DemandAuthorizer`] gate — the [`CATALOG_QUERY_RESOURCE`] precedent, and a
/// DISTINCT label for the same reason the two verbs are distinct: `runs` serves
/// the machine's live-run set plus each run's whole effective `graph.yaml`, which
/// is strictly more than the topic enumeration `catalog` serves. A future
/// per-resource policy must be able to tell them apart, and a refusal log must
/// name which one was refused.
const RUNS_QUERY_RESOURCE: &str = "<runs: live runs + graphs>";

/// Canonical Cerulion topic names carry a leading `/`
/// (`/{prefix}/{node}/{output}`). The NETWORK layer normalizes every name
/// to canonical form at its boundaries so the wire key-space has exactly
/// one name per topic — a raw no-slash name (tests, tooling) aliases its
/// canonical twin on the network, while locally the two remain distinct
/// iceoryx2 services (only canonical graph topics are bridged in
/// production).
pub(crate) fn canonical_topic(topic: &str) -> std::borrow::Cow<'_, str> {
    if topic.starts_with('/') {
        std::borrow::Cow::Borrowed(topic)
    } else {
        std::borrow::Cow::Owned(format!("/{topic}"))
    }
}

/// Run ONE bounded DEMAND gather — the reconciler's live-query half. A
/// query failure is a transient (returned as `Err`); the caller SKIPS the pass
/// leaving every flag untouched (never disable on a failed gather — that would
/// drop egress on a blip). The returned set is canonical-keyed to match the
/// announce flags.
fn gather_demand_set(
    session: &zenoh::Session,
    window: Duration,
) -> TransportResult<HashSet<String>> {
    let topics = query_live_topics_by_prefix(session, liveliness_prefix(), window)?;
    Ok(topics
        .into_iter()
        .map(|t| canonical_topic(&t).into_owned())
        .collect())
}

/// Apply a gathered DEMAND set to the announced topics with 3-pass
/// hysteresis — the reconciler's PURE decision half (no zenoh, deterministically
/// oracle-testable via [`apply_demand_reconcile_for_test`]).
///
/// For each announced topic: if it is demanded, RESET its absence counter (which
/// also begins TRACKING it) and idempotently [`enable`](TopicBridgeManager::enable_bridge)
/// it (the egress allow-list gate INSIDE `enable_bridge` still enforces posture —
/// the reconciler can never WIDEN egress past the graph's declared list). If it
/// is NOT demanded, increment its absence counter ONLY WHEN THE RECONCILER IS
/// ALREADY TRACKING IT (it enabled it earlier); at [`REMOVE_CONFIRM`] consecutive
/// absences, [`disable`](TopicBridgeManager::disable_bridge) it and stop tracking.
///
/// The tracking guard is load-bearing but PRECISELY scoped: it
/// protects a topic whose demand the reconciler has NEVER observed (a flag
/// flipped by some OTHER path — a direct `enable_bridge`, or the liveliness
/// subscriber — for a topic never in `absence`), leaving it untouched. Once the
/// reconciler OBSERVES a topic's demand it ADOPTS lifecycle ownership of that
/// topic (enable now, absence-disable later) even if another path enabled it
/// first — an observed topic is lifecycle-managed, not guarded.
fn apply_demand_reconcile(
    demand: &HashSet<String>,
    bridge_mgr: &TopicBridgeManager,
    announced: &[String],
    absence: &mut HashMap<String, u32>,
    pass_counter: &AtomicU64,
    enabled_counter: &AtomicU64,
) {
    pass_counter.fetch_add(1, Ordering::Relaxed);
    for topic in announced {
        let canonical = canonical_topic(topic).into_owned();
        if demand.contains(&canonical) {
            // Demand present: (re)start tracking + affirm egress. `enable_bridge`
            // is idempotent and the allow-list gate inside it may still refuse a
            // non-member (the flag stays down). The counter attributes a
            // flip to the RECONCILER only when THIS call genuinely transitioned
            // the flag false→true — a re-affirm of an already-on flag (or a
            // gate-refused non-member) returns `false` and is NOT counted, so on
            // a healthy link (where the subscriber flipped it first) the count
            // stays 0.
            absence.insert(canonical.clone(), 0);
            match bridge_mgr.enable_bridge(&canonical) {
                Ok(true) => {
                    enabled_counter.fetch_add(1, Ordering::Relaxed);
                }
                Ok(false) => {}
                Err(e) => tracing::error!(
                    topic = %canonical,
                    error = %e,
                    "reconciler: enable_bridge failed (bridge manager poisoned)"
                ),
            }
        } else if let Some(count) = absence.get_mut(&canonical) {
            // Only a TRACKED topic (one the reconciler previously enabled from
            // observed demand) is subject to absence-driven removal.
            *count += 1;
            if *count >= REMOVE_CONFIRM {
                match bridge_mgr.disable_bridge(&canonical) {
                    // Log the PRECISE mechanism at the call site —
                    // `disable_bridge` does not imply real-time subscriber
                    // knowledge; only the true→false edge is logged.
                    Ok(true) => tracing::info!(
                        topic = %canonical,
                        remove_confirm = REMOVE_CONFIRM,
                        "network bridge disabled — demand absent across REMOVE_CONFIRM \
                         consecutive reconcile passes"
                    ),
                    Ok(false) => {}
                    Err(e) => tracing::error!(
                        topic = %canonical,
                        error = %e,
                        "reconciler: disable_bridge failed (bridge manager poisoned)"
                    ),
                }
                // Stop tracking after removal — a fresh demand token re-tracks it.
                absence.remove(&canonical);
            }
        }
    }
}

/// The belt feed: refresh `topics` (the reconciler belt's
/// live iteration set) from the bridge manager's registered-topic snapshot (THE
/// source of truth for which topics may egress), using the SAME monotonic-count
/// dirty-check discipline [`crate::transport::gateway::GatewayRuntime::drive_once`]
/// uses.
///
/// # Why the belt needs this
///
/// The demand reconciler's ~1 s liveliness-query BELT re-affirms egress
/// (and absence-disables) for the topics it iterates. If that set were
/// FROZEN at the BOOT announce list, a topic REGISTERED AT RUNTIME (a
/// `cerulion ros2 attach` dds_bridge's raw route, fed via the control
/// channel or the permissive SHM probe) would ride the queryable + liveliness-
/// subscriber paths but NEVER be re-affirmed by the belt — on exactly the
/// broken-subscriber link the belt exists for, a runtime topic's demand would go
/// un-re-affirmed. Feeding the belt the LIVE registered set closes that: a
/// runtime topic is re-affirmed by the belt the moment it registers.
///
/// # Cost discipline
///
/// The bridge `bridges` map is ADD-ONLY (`register_topic` inserts; nothing
/// removes a key), so `registered_topic_count()` is MONOTONIC and a bare count
/// read (one uncontended mutex lock, no alloc) is a reliable "a new egress topic
/// registered" dirty-check. The per-pass steady-state cost is exactly that one
/// count read; only a GROWN count takes the full (locked, allocating) snapshot +
/// rebuilds `topics`. The seed at thread spawn IS the boot announce set (whose
/// length equals the registered count at boot — both derived from the same
/// snapshot), so the dirty-check is stable from the first pass and never
/// re-snapshots spuriously.
///
/// Robust: a poisoned bridge mutex leaves `topics` UNCHANGED (the belt keeps its
/// last-known set + logs at `debug!` — never wedges, never silently drops a
/// topic). `registered_topics()` returns CANONICAL, SORTED names, so the refreshed
/// `topics` needs no re-canonicalization ([`apply_demand_reconcile`] canonicalizes
/// each entry again anyway, a no-op on an already-canonical name).
fn refresh_reconcile_topics(bridge_mgr: &TopicBridgeManager, topics: &mut Vec<String>) {
    match bridge_mgr.registered_topic_count() {
        // Grew since the last pass — a runtime registration. Re-snapshot the live
        // registered set (canonical + sorted) into `topics`.
        Ok(count) if count > topics.len() => match bridge_mgr.registered_topics() {
            Ok(registered) => {
                topics.clear();
                topics.extend(registered.into_iter().map(|(topic, _)| topic));
            }
            Err(e) => tracing::debug!(
                error = %e,
                "reconciler belt: could not snapshot the registered egress set \
                 (bridge manager poisoned) — the belt keeps its last-known iteration set this pass"
            ),
        },
        // Unchanged (steady state) — the cheap path, no snapshot.
        Ok(_) => {}
        Err(e) => tracing::debug!(
            error = %e,
            "reconciler belt: could not read the registered egress count \
             (bridge manager poisoned) — the belt keeps its last-known iteration set this pass"
        ),
    }
}

/// Once-per-regime latch for the reconciler's bounded demand
/// GATHER — the query half's belt-must-not-be-silent guard. A permanently broken
/// query session freezes every egress flag this regime (no gather ⇒ no
/// reconcile), and at the default log level a bare `debug!` made that
/// indistinguishable from healthy. Mirroring [`IngressWarnLatch`]'s discipline:
/// the FIRST failure of a regime `warn!`s (naming the consequence — flags
/// frozen), repeats downgrade to `debug!` carrying the running count (a
/// per-second failing gather must not flood), and the first SUCCESS after a
/// failure run logs a recovery `info!` with the suppressed count. Owned per
/// reconciler (the background thread and the sync seam each own one), mirroring
/// the `absence` map's lifecycle. `consecutive_failures == 0` is healthy;
/// `== 1` is the loud first, `> 1` the suppressed repeats.
#[derive(Default)]
struct GatherFailureLatch {
    consecutive_failures: u32,
}

/// Run ONE bounded demand gather, applying the gather-failure latch discipline —
/// the shared gather half of every reconcile path. On success (incl. an empty
/// result — the strict connect-only accepter-GET returns Ok(empty), NOT Err),
/// logs a recovery `info!` once if it ends a failure run and returns
/// `Some(demand)`. On failure, latches loud-once (first `warn!` naming the
/// frozen-flags consequence, repeats `debug!`) and returns `None` (the caller
/// leaves every flag untouched — never disable on a query blip).
fn gather_with_latch(
    session: &zenoh::Session,
    gather_latch: &mut GatherFailureLatch,
) -> Option<HashSet<String>> {
    match gather_demand_set(session, RECONCILE_GATHER_WINDOW) {
        Ok(demand) => {
            // Recovery: a success that ends a failure run logs `info!` once with
            // the suppressed count, then re-arms the latch (mirrors
            // `IngressWarnLatch::reset` / `DrainWarnLatch::on_success`).
            if gather_latch.consecutive_failures > 0 {
                tracing::info!(
                    suppressed = gather_latch.consecutive_failures,
                    "demand reconcile query recovered — the reconciler can query \
                     again; egress flags reconcile this pass"
                );
                gather_latch.consecutive_failures = 0;
            }
            Some(demand)
        }
        Err(e) => {
            gather_latch.consecutive_failures += 1;
            if gather_latch.consecutive_failures == 1 {
                tracing::warn!(
                    error = %e,
                    "demand reconcile query FAILED — the demand reconciler cannot \
                     query, so egress flags are FROZEN this regime (no topic can start or \
                     stop egressing until the query recovers); repeats log at debug"
                );
            } else {
                tracing::debug!(
                    error = %e,
                    consecutive_failures = gather_latch.consecutive_failures,
                    "demand reconcile query still failing — egress flags frozen \
                     (repeat — see the first warning for this regime)"
                );
            }
            None
        }
    }
}

/// THE shared reconcile-pass body — run by BOTH the production
/// background thread (see [`NetworkManager::start_demand_reconciler`]) and the
/// synchronous test seam (`reconcile_demand_pass`). One pass: refresh the belt's
/// iteration set from the LIVE registered set (the belt feed —
/// UNCONDITIONAL, so the set is fresh the moment a frozen query recovers), then
/// gather demand once and, on success, apply the liveliness reconcile AND — when
/// `get_state` is `Some` — the GET-expiry sweep against the SAME gathered
/// demand set (the composition rule needs the liveliness view to gate the expiry).
/// A gather failure skips the apply + expiry (flags untouched) and does NOT bump
/// `pass_counter`; it is latched via `gather_latch` so a broken query
/// session is LOUD once per regime instead of a silent `debug!` forever.
///
/// The production thread and the sync seam call
/// THIS ONE function instead of each inlining the pass body, so the belt's
/// `refresh_reconcile_topics` call has EXACTLY ONE call site reachable from both.
/// That unification IS the coverage guarantee — deleting the refresh line here
/// breaks the deterministic sync-seam test
/// (`belt_reaffirms_a_runtime_registered_topic_via_live_reconcile_seam`, which
/// drives this path and trips its 20 s deadline if the refresh is reverted), which
/// therefore covers the production thread BY CONSTRUCTION. There is no
/// second, separately inlined production refresh line a test can leave green while
/// deleting.
///
/// `get_state` is the only place the two callers diverge, and it does NOT fork the
/// shared body: the production thread passes `Some(get_state)` (it owns the
/// GET-keepalive map and runs the expiry sweep every pass); the sync reconcile
/// seam passes `None` (its GET-expiry sweep has a dedicated seam, `get_expiry_pass`,
/// so the reconcile seam must not disable a GET-granted topic as a side effect).
// The shared pass body legitimately threads the reconciler's whole per-pass state
// (session, bridge, iteration set, absence map, two observability counters, the
// gather latch, and the optional GET-keepalive state); bundling any subset would
// obscure the mirror to the production thread's locals rather than clarify it.
#[allow(clippy::too_many_arguments)]
fn run_reconcile_pass_on_session(
    session: &zenoh::Session,
    bridge_mgr: &TopicBridgeManager,
    topics: &mut Vec<String>,
    absence: &mut HashMap<String, u32>,
    pass_counter: &AtomicU64,
    enabled_counter: &AtomicU64,
    gather_latch: &mut GatherFailureLatch,
    get_state: Option<&GetDemandState>,
) {
    // Refresh the belt's iteration set from the LIVE registered
    // set BEFORE gathering — UNCONDITIONAL (runs whether or not the gather
    // succeeds) so the set is fresh the moment a frozen query recovers. This is
    // the ONE refresh call site both production and the sync seam reach.
    refresh_reconcile_topics(bridge_mgr, topics);
    // Gather once; apply the liveliness reconcile AND (production only) the
    // GET-expiry sweep against the SAME demand set. A failed/frozen gather skips
    // BOTH (never disable on an unknown liveliness view — the conservative arm).
    if let Some(demand) = gather_with_latch(session, gather_latch) {
        apply_demand_reconcile(
            &demand,
            bridge_mgr,
            topics,
            absence,
            pass_counter,
            enabled_counter,
        );
        if let Some(get_state) = get_state {
            run_get_expiry(bridge_mgr, get_state, &demand, Instant::now());
        }
    }
}

/// Sleep `dur`, waking early (returning `true`) if the reconciler's
/// broadcast shutdown signal arrives. Polls the signal every ≤50 ms so a Drop
/// join is prompt instead of waiting up to a full [`RECONCILE_INTERVAL`]. Sync
/// (the reconciler thread has no tokio runtime), so it polls `try_recv` rather
/// than `.await` — the sync-appropriate shape of `stop_task`'s broadcast pattern.
fn sleep_with_shutdown(rx: &mut tokio::sync::broadcast::Receiver<()>, dur: Duration) -> bool {
    use tokio::sync::broadcast::error::TryRecvError;
    let deadline = Instant::now() + dur;
    loop {
        match rx.try_recv() {
            // A send, a closed sender (manager dropped), or a lag (we only ever
            // send once, so this is unreachable) — all mean "shut down".
            Ok(()) | Err(TryRecvError::Closed) | Err(TryRecvError::Lagged(_)) => return true,
            Err(TryRecvError::Empty) => {}
        }
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return false;
        }
        std::thread::sleep(remaining.min(Duration::from_millis(50)));
    }
}

/// TEST SEAM: apply a HAND-BUILT demand set to the announced topics with
/// the production hysteresis logic — no zenoh, fully deterministic. Delegates to
/// the exact [`apply_demand_reconcile`] the background thread runs, so the
/// hysteresis + allow-list + tracking-guard oracle tests exercise the real code
/// path (not a re-implementation).
#[cfg(any(test, feature = "test-helpers"))]
pub fn apply_demand_reconcile_for_test(
    demand: &HashSet<String>,
    bridge_mgr: &TopicBridgeManager,
    announced: &[String],
    absence: &mut HashMap<String, u32>,
    pass_counter: &AtomicU64,
    enabled_counter: &AtomicU64,
) {
    apply_demand_reconcile(
        demand,
        bridge_mgr,
        announced,
        absence,
        pass_counter,
        enabled_counter,
    );
}

/// TEST SEAM: refresh `topics` from the bridge manager's live
/// registered set via the EXACT production [`refresh_reconcile_topics`] the belt
/// runs — no zenoh, fully deterministic. The oracle-vector driver for the belt
/// feed's monotonic-count dirty-check (a runtime registration grows `topics`; an
/// unchanged count leaves it byte-identical).
#[cfg(any(test, feature = "test-helpers"))]
pub fn refresh_reconcile_topics_for_test(
    bridge_mgr: &TopicBridgeManager,
    topics: &mut Vec<String>,
) {
    refresh_reconcile_topics(bridge_mgr, topics);
}

/// TEST SEAM: run the refresh-then-apply CORE of one belt pass on a
/// HAND-BUILT demand set — refresh `topics` from the live registered set THEN apply
/// the reconcile. Fully deterministic, no zenoh: it exercises the same
/// `refresh_reconcile_topics` + `apply_demand_reconcile` steps the shared pass body
/// (`run_reconcile_pass_on_session`) runs, but WITHOUT the zenoh gather (the caller
/// supplies `demand` directly) so the belt feed is testable off-wire. This is the
/// belt-feed integration oracle: a topic registered AT RUNTIME (not in the seed
/// `topics`) is picked up by the refresh and re-affirmed by the apply — the mutation
/// kill is the refresh: without it `topics` stays frozen and a runtime topic is
/// never enabled. (The refresh call INSIDE the shared production pass body is
/// covered separately by the LIVE sync-seam test — see
/// `run_reconcile_pass_on_session`.)
#[cfg(any(test, feature = "test-helpers"))]
pub fn refresh_and_apply_demand_reconcile_for_test(
    demand: &HashSet<String>,
    bridge_mgr: &TopicBridgeManager,
    topics: &mut Vec<String>,
    absence: &mut HashMap<String, u32>,
    pass_counter: &AtomicU64,
    enabled_counter: &AtomicU64,
) {
    refresh_reconcile_topics(bridge_mgr, topics);
    apply_demand_reconcile(
        demand,
        bridge_mgr,
        topics,
        absence,
        pass_counter,
        enabled_counter,
    );
}

// ===========================================================================
// Queryable inversion — the wire-proven demand path.
//
// A liveliness-token demand path uses the TWO BROKEN directions of a strict
// connect-only, listen-less, scouting-off zenoh 1.8 peer link (the dialer
// declares a `cerulion_lv` token; the accepter GETs `cerulion_lv/**`). This
// inverts onto the two WORKING directions: the PRODUCER (accepter) declares the
// verb-dispatched QUERYABLE at `cerulion_q/{robot}/**` (an accepter
// declaration — reaches the dialer), and the DEMANDER (dialer) GETs its `demand`
// verb (a dialer query — reaches the accepter) to pull egress ON with a
// synchronous ack. Each granted GET stamps a
// last-seen instant; the producer's reconciler thread expires a topic (disables
// egress) when its last GET aged past DEMAND_TTL AND liveliness is also absent
// (the composition rule — a topic disables only when BOTH paths release it).
// ===========================================================================

/// PRODUCER-side GET-demand tracking. The demand QUERYABLE's zenoh
/// callback records the last-GET instant per ALLOWED canonical topic here; the
/// reconciler thread's EXPIRY sweep reads it and disables a topic whose last GET
/// aged past `DEMAND_TTL` AND which liveliness is not holding. Shared (`Arc`)
/// between the queryable callback (writer) and the reconciler thread + the
/// gateway's Principle-#3 accessors (readers). All counters are `Relaxed` — a
/// monotone tally, never a synchronization point.
#[derive(Default)]
pub struct GetDemandState {
    /// canonical topic → last GET `Instant` (PRESENT ⇒ GET-granted / holding
    /// egress via the queryable path).
    last_get: Mutex<HashMap<String, Instant>>,
    /// cumulative ALLOWED grants applied by the queryable (enable + record).
    grants: AtomicU64,
    /// cumulative GET refusals (allow-list miss / unregistered / suppressed).
    refusals: AtomicU64,
    /// cumulative GET-expiry disables applied by the reconciler thread.
    expiries: AtomicU64,
}

impl GetDemandState {
    /// Grant/expiry atomicity: record one ALLOWED grant under an
    /// ALREADY-HELD `last_get` guard — bump the grant tally and stamp the topic's
    /// last-GET instant (which BEGINS/refreshes its keepalive — the producer
    /// expiry sweep releases egress `DEMAND_TTL` after this). The queryable
    /// callback ([`handle_demand_verb`]) holds `last_get` across
    /// `enable_bridge` + `is_enabled` + this stamp so the expiry sweep — which
    /// holds `last_get` across its WHOLE sweep — cannot race a fresh grant into a
    /// spurious disable (enable → sweep-disable+remove → stamp would leave egress
    /// OFF but the ack lying ENABLED with a fresh stamp). Counter + stamp
    /// semantics are identical to the pre-split one-shot grant; only the lock is
    /// now the caller's to hold across the enable+ack+stamp critical section.
    fn record_grant_locked(&self, last: &mut HashMap<String, Instant>, canonical: &str) {
        self.grants.fetch_add(1, Ordering::Relaxed);
        last.insert(canonical.to_string(), Instant::now());
    }

    /// Record one refusal (no keepalive stamp — a refused topic is NOT holding
    /// egress, so it must not keep any GET-granted state alive).
    fn record_refusal(&self) {
        self.refusals.fetch_add(1, Ordering::Relaxed);
    }

    /// Observable state (Principle #3): cumulative allowed GET grants applied.
    pub fn grant_count(&self) -> u64 {
        self.grants.load(Ordering::Relaxed)
    }

    /// Observable state (Principle #3): cumulative GET refusals.
    pub fn refusal_count(&self) -> u64 {
        self.refusals.load(Ordering::Relaxed)
    }

    /// Observable state (Principle #3): cumulative GET-expiry disables.
    pub fn expiry_count(&self) -> u64 {
        self.expiries.load(Ordering::Relaxed)
    }

    /// Observable state (Principle #3): the canonical topics currently GET-granted
    /// (holding egress via a fresh-enough GET), SORTED for deterministic
    /// assertions. Poison-recovering.
    pub fn granted_topics(&self) -> Vec<String> {
        let last = self
            .last_get
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let mut topics: Vec<String> = last.keys().cloned().collect();
        topics.sort();
        topics
    }
}

/// DEMANDER-side demand-GET tracking. The demand GET loop reads the
/// ingress topics to demand + the harvested producer identities and records the
/// acks it saw. Shared (`Arc`) between the GET loop thread + the gateway's
/// Principle-#3 accessors.
#[derive(Default)]
pub struct DemandGetState {
    /// canonical ingress topics to demand each pass (fixed at gateway boot).
    topics: Mutex<Vec<String>>,
    /// producer robot identities harvested from the announce space — targets the
    /// EXPLICIT GETs. Empty ⇒ the WILDCARD fallback (`cerulion_q/*/demand{topic}`,
    /// which intersects every producer's `cerulion_q/{robot}/**` queryable).
    identities: Mutex<BTreeSet<String>>,
    /// cumulative demand-GET passes run.
    passes: AtomicU64,
    /// cumulative replies acking "enabled".
    enabled_acks: AtomicU64,
    /// cumulative replies acking "refused".
    refused_acks: AtomicU64,
    /// once-per-regime GET-failure latch (loud-once; a dead link must not flood).
    fail: Mutex<GatherFailureLatch>,
}

impl DemandGetState {
    /// Build a demander state for a fixed set of canonical ingress topics.
    pub fn from_topics(topics: Vec<String>) -> Self {
        Self {
            topics: Mutex::new(topics),
            ..Default::default()
        }
    }

    /// Add a canonical ingress topic to the DYNAMIC demand set (idempotent
    /// — a duplicate is a no-op). The background demand-GET loop re-reads the topic
    /// set every pass, so a topic added while the loop runs is demanded on the very
    /// next pass. This is the seam that lets a PURE-INGRESS demander
    /// (`cerulion-netd`) grow its keepalive set as consumers demand new mirrors,
    /// versus the gateway's fixed boot set ([`Self::from_topics`]).
    pub fn add_topic(&self, canonical: String) {
        let mut topics = self
            .topics
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if !topics.contains(&canonical) {
            topics.push(canonical);
        }
    }

    /// Remove a canonical ingress topic from the DYNAMIC demand set (a
    /// mirror's last consumer left, the bridge is torn down). The loop stops
    /// GETting it on the next pass. A topic that was never present is a no-op.
    pub fn remove_topic(&self, canonical: &str) {
        let mut topics = self
            .topics
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        topics.retain(|t| t != canonical);
    }

    /// Observable state (Principle #3): the number of canonical topics currently demanded.
    pub fn topic_count(&self) -> usize {
        self.topics
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .len()
    }

    /// Observable state (Principle #3): cumulative demand-GET passes run.
    pub fn pass_count(&self) -> u64 {
        self.passes.load(Ordering::Relaxed)
    }

    /// Observable state (Principle #3): cumulative replies acking "enabled".
    pub fn enabled_ack_count(&self) -> u64 {
        self.enabled_acks.load(Ordering::Relaxed)
    }

    /// Observable state (Principle #3): cumulative replies acking "refused".
    pub fn refused_ack_count(&self) -> u64 {
        self.refused_acks.load(Ordering::Relaxed)
    }

    /// Observable state (Principle #3): the harvested producer identities, SORTED. Empty
    /// until the first announce harvest lands (until then GETs use the wildcard).
    pub fn harvested_identities(&self) -> Vec<String> {
        let ids = self
            .identities
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        ids.iter().cloned().collect()
    }

    /// Observable state (Principle #3): number of harvested producer identities.
    pub fn harvested_identity_count(&self) -> usize {
        self.identities
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .len()
    }

    /// A per-pass GET failure: latch loud-once (first `warn!`, repeats `debug!`).
    fn record_get_failure(&self, error: &TransportError) {
        let mut latch = self
            .fail
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        latch.consecutive_failures += 1;
        if latch.consecutive_failures == 1 {
            tracing::warn!(
                error = %error,
                "demand GET FAILED — the demander cannot query the producer's demand \
                 queryable, so its ingress topics may never start egressing on the remote; \
                 repeats log at debug"
            );
        } else {
            tracing::debug!(
                error = %error,
                consecutive_failures = latch.consecutive_failures,
                "demand GET still failing (repeat — see the first warning for this regime)"
            );
        }
    }

    /// A pass with no GET failures ends any failure run: recovery `info!` once.
    fn record_get_success(&self) {
        let mut latch = self
            .fail
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if latch.consecutive_failures > 0 {
            tracing::info!(
                suppressed = latch.consecutive_failures,
                "demand GET recovered — the demander can query the producer again"
            );
            latch.consecutive_failures = 0;
        }
    }
}

/// PURE: build the demand-GET selectors for ONE canonical ingress
/// topic under the `cerulion_q` query surface. With harvested identities, target
/// each EXPLICITLY (`cerulion_q/{robot}/demand{canonical}`) — precise, no wildcard
/// fan-out; otherwise fall back to the single-chunk WILDCARD
/// (`cerulion_q/*/demand{canonical}`), which intersects every producer's
/// `cerulion_q/{robot}/**` queryable. Deterministic order (the `BTreeSet` iterates
/// sorted). Oracle-tested (no zenoh).
fn demand_get_selectors(canonical_topic: &str, identities: &BTreeSet<String>) -> Vec<String> {
    if identities.is_empty() {
        vec![cerulion_q::demand_selector("*", canonical_topic)]
    } else {
        identities
            .iter()
            .map(|robot| cerulion_q::demand_selector(robot, canonical_topic))
            .collect()
    }
}

/// PURE: given the GET last-seen map, the liveliness demand set (topics
/// the subscriber/reconciler path is holding this pass), and `now`, return the
/// canonical topics whose GET-grant has EXPIRED (older than `ttl`) AND which
/// liveliness is NOT holding — the composition rule: a topic disables only when
/// BOTH paths have released it. SORTED (deterministic). Oracle-tested (no zenoh,
/// no clock — the caller passes `now`).
fn get_expired_topics(
    last_get: &HashMap<String, Instant>,
    liveliness_demand: &HashSet<String>,
    now: Instant,
    ttl: Duration,
) -> Vec<String> {
    let mut expired: Vec<String> = last_get
        .iter()
        .filter(|(topic, &stamped)| {
            now.saturating_duration_since(stamped) > ttl
                && !liveliness_demand.contains(topic.as_str())
        })
        .map(|(topic, _)| topic.clone())
        .collect();
    expired.sort();
    expired
}

/// The AllowAll-only permissive SHM PROBE — the automagic that closes
/// the runtime-raw-route demand gap for a `cerulion ros2 attach` robot.
///
/// # The problem it solves (decision: option 3's automagic)
///
/// A generic `ros2 attach` robot's dds_bridge creates ~90 SHM publishers at
/// RUNTIME (raw routes like `/utlidar/cloud`). None are in the boot
/// [`GatewayPlan`](super::gateway::GatewayPlan), so a remote DEMAND for one names
/// a topic the gateway has NOT registered → the demand paths refuse it
/// ("unregistered"). The control channel closes the gap when a worker PUSHES a
/// registration over it; this probe closes it for the automagic case where
/// no worker registered but the service is LIVE in local SHM: when a demand names
/// an UNregistered topic AND the egress posture is permissive (`AllowAll`), the
/// gateway probes local SHM for a live service of that name and, if found,
/// registers + announces it on the spot — the approved announce-on-first-serve
/// semantics — so the demand GRANTS exactly like a plan-declared producer.
///
/// # Safety by construction
///
/// - **AllowAll ONLY.** Under a Strict (explicit `network:` block → allow-list)
///   posture the probe never runs (by design — an explicit block is a
///   TIGHTENING, so unknown topics stay refused). [`Self::try_serve`] returns
///   [`ProbeVerdict::NotPermissive`] without touching a counter.
/// - **Open-only, TOCTOU-safe.** The SHM check is
///   [`TransportManager::create_data_only_subscriber`](super::TransportManager::create_data_only_subscriber),
///   which opens the data service `.open()` (never `open_or_create`) — a MISS
///   creates NOTHING (no phantom service), and the probe drops its subscriber
///   immediately (the real egress tap is attached later by the drive loop).
/// - **Loop-safe.** A topic this gateway INGRESSES (its local SHM service is the
///   gateway's OWN re-injection) is NEVER probe-served — serving it back out is
///   the echo loop the design refuses. Excluded with a loud refusal
///   ([`ProbeVerdict::LoopRefused`]), as is the reserved control-plane namespace.
/// - **Bounded state under a hostile flood.** A MISS keeps NO per-name state —
///   just one [`Self::misses`] tally + a single warn-once-per-regime latch
///   ([`Self::flood_warned`], re-armed by a HIT), so a flood of distinct missing
///   names cannot grow unbounded memory or flood the log (the `DrainWarnLatch`
///   discipline).
///
/// Both demand paths — the queryable ([`demand_grant_decision`]) AND the
/// liveliness watch ([`apply_live_put_demand`]) — funnel their unregistered case
/// through [`Self::try_serve`], so a probe-hit via either path converges to the
/// SAME registered + announced + grantable state (`try_serve` is idempotent, so
/// double-hits are safe). Held by the gateway (via the `Arc` it stores for the
/// Principle-#3 accessors) and by the [`NetworkManager`] (via
/// [`NetworkManager::set_permissive_probe`], read by the two demand threads).
pub(crate) struct PermissiveShmProbe {
    /// Weak back-ref to the owning manager — the SHM node ([`TransportManager::create_data_only_subscriber`](super::TransportManager::create_data_only_subscriber)),
    /// the bridge gate + registry, the ingress registry, and the egress announce.
    /// WEAK (never `Arc`) to break the cycle: the manager owns the
    /// [`NetworkManager`], which owns this probe's `Arc`; a strong ref back would
    /// leak the manager (Drop never runs → the demand threads never stop). The
    /// demand-thread callers `upgrade()` per call — a `None` (manager mid-Drop)
    /// degrades to a debug no-op, never UB.
    manager: Weak<TransportManager>,
    /// Principle #3: cumulative probe HITS (a live local service registered +
    /// announced on demand).
    hits: AtomicU64,
    /// Principle #3: cumulative probe MISSES (a demand for a topic with no live
    /// local SHM service — refused, nothing created).
    misses: AtomicU64,
    /// Principle #3: cumulative LOOP refusals (a demand for a topic this gateway
    /// ingresses, or a reserved control-plane name — never probe-served).
    loop_refusals: AtomicU64,
    /// Warn-once-per-regime latch for the MISS log: the first miss of a regime
    /// `warn!`s, repeats `debug!` (so a hostile flood of missing names logs ONCE,
    /// not per demand); a HIT re-arms it. ONE flag for ALL names (not a per-name
    /// map) so state stays O(1) under a flood.
    flood_warned: AtomicBool,
    /// Warn-once-per-regime latch for the LOOP-refusal log (ingress topic /
    /// reserved name) — the DrainWarnLatch sibling of `flood_warned`. The counter
    /// bumps every time (Principle #3), but the WARN fires once per regime so a
    /// hostile flood of distinct reserved/loop names (or a flappy peer re-demanding
    /// an ingress topic, or the gateway's OWN ingress-token history-replay
    /// self-Put) logs ONCE, not per demand; a HIT re-arms it. ONE flag for both
    /// classes (O(1) state under a flood).
    loop_flood_warned: AtomicBool,
    /// Cumulative probe HITS whose network announce FAILED
    /// (registered + demand-grantable but not yet discoverable — the flag is
    /// RETAINED). Mirrors the gateway's `reg_announce_deferred` semantics: a pure
    /// cumulative tally (never decremented). Bumped when a hit's initial announce
    /// fails AND when a later demand's deferred-announce RE-ATTEMPT
    /// ([`Self::heal_deferred_announce`]) fails again. A `> 0` value is a
    /// transient-blip signal, not a permanent refusal — the topic still egresses on
    /// demand and heals (announces) on the next demand. Read via
    /// [`Self::announce_deferred_count`].
    announce_deferred: AtomicU64,
    /// Boot TOCTOU close: the STATIC declared-ingress set (canonical
    /// names) handed at probe construction from the [`GatewayPlan`](super::gateway::GatewayPlan)
    /// `ingress` list — known BEFORE any ingress SHM service or the dynamic ingress
    /// map exists. The ingress SHM service goes live at `create_ingress_publisher`
    /// EARLIER in boot than the dynamic map is populated (at the END of
    /// `register_ingress`), so a demand in that window would otherwise probe-serve
    /// the gateway's OWN ingress topic → a permanent echo loop. `try_serve`
    /// refuses a topic in EITHER this static set OR the dynamic map (the latter still
    /// covers ingress ADDED at runtime).
    declared_ingress: HashSet<String>,
}

/// The outcome of one [`PermissiveShmProbe::try_serve`] — an observable
/// classification of what the probe did (or why it did nothing). The demand paths
/// ignore it (they proceed to `enable_bridge` regardless — a served topic has
/// a flag to flip, an unserved one still refuses); the deterministic test seams
/// read it. Only [`Self::Served`]/[`Self::Missing`]/[`Self::LoopRefused`] move a
/// counter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ProbeVerdict {
    /// Egress posture is NOT `AllowAll` — the probe never runs under Strict
    /// (by design). No counter touched.
    NotPermissive,
    /// The topic was ALREADY registered (a race, or the enable path already owns
    /// it) — the probe is a no-op. No counter touched.
    AlreadyRegistered,
    /// The topic is an INGRESS topic or a reserved control-plane name — refused
    /// for loop safety ([`PermissiveShmProbe::loop_refusals`] bumped).
    LoopRefused,
    /// A live local SHM service was found — the topic was registered + announced
    /// ([`PermissiveShmProbe::hits`] bumped). Now demand-grantable. NOTE: an
    /// announce failure on a hit does NOT downgrade to [`Self::Missing`] — the flag
    /// is retained (grantable) so it stays `Served`, and the deferred announce is
    /// counted separately ([`PermissiveShmProbe::announce_deferred`]).
    Served,
    /// Nothing was served ([`PermissiveShmProbe::misses`] bumped), for one of two
    /// reasons: (1) the common case — NO live local SHM service (the open-only
    /// probe found nothing and created nothing); or (2) the rare register-poison
    /// arm — a live service WAS found but registering its bridge flag failed (a
    /// poisoned bridge mutex), so the topic is neither registered nor announced.
    /// Both are counted as a miss because nothing was made grantable.
    Missing,
    /// The owning manager could not be upgraded (mid-Drop) — a debug no-op. No
    /// counter touched.
    ManagerGone,
}

impl ProbeVerdict {
    /// A STABLE classification label for the deterministic test seam
    /// ([`crate::transport::gateway::GatewayRuntime::probe_verdict_for_test`]) — a
    /// crisp per-case oracle without exposing the `pub(crate)` enum across the
    /// integration-test boundary. Test-gated (its only caller is the seam).
    #[cfg(any(test, feature = "test-helpers"))]
    pub(crate) fn label(&self) -> &'static str {
        match self {
            ProbeVerdict::NotPermissive => "not_permissive",
            ProbeVerdict::AlreadyRegistered => "already_registered",
            ProbeVerdict::LoopRefused => "loop_refused",
            ProbeVerdict::Served => "served",
            ProbeVerdict::Missing => "missing",
            ProbeVerdict::ManagerGone => "manager_gone",
        }
    }
}

impl PermissiveShmProbe {
    /// Create a probe for `manager` (held WEAK — see the field doc) with the
    /// STATIC `declared_ingress` set (canonical names — the boot-plan
    /// ingress topics known before any ingress SHM service or dynamic-map entry
    /// exists; see the `declared_ingress` field doc).
    pub(crate) fn new(manager: Weak<TransportManager>, declared_ingress: HashSet<String>) -> Self {
        Self {
            manager,
            hits: AtomicU64::new(0),
            misses: AtomicU64::new(0),
            loop_refusals: AtomicU64::new(0),
            flood_warned: AtomicBool::new(false),
            loop_flood_warned: AtomicBool::new(false),
            announce_deferred: AtomicU64::new(0),
            declared_ingress,
        }
    }

    /// Record + LOG one LOOP refusal (an ingress topic or a reserved control-plane
    /// name). The `loop_refusals` counter bumps UNCONDITIONALLY (Principle #3); the
    /// WARN fires once per regime (`loop_flood_warned`, re-armed by a HIT) so a
    /// hostile flood — or the gateway's own ingress-token history-replay self-Put —
    /// logs ONCE, then rides `debug!`. `detail` names the hazard and appears in
    /// BOTH the warn and the debug line.
    fn record_loop_refusal(&self, canonical: &str, detail: &str) {
        self.loop_refusals.fetch_add(1, Ordering::Relaxed);
        if self.loop_flood_warned.swap(true, Ordering::Relaxed) {
            tracing::debug!(
                topic = %canonical,
                detail = %detail,
                "permissive SHM probe REFUSED (repeat — see the first warning; \
                 loop_refusal_count keeps counting)"
            );
        } else {
            tracing::warn!(
                topic = %canonical,
                detail = %detail,
                "permissive SHM probe REFUSED — this gateway must never serve \
                 it to the network (repeats log at debug; loop_refusal_count keeps counting)"
            );
        }
    }

    /// Re-attempt the deferred announce for an ALREADY-REGISTERED
    /// probe-served `canonical` topic whose earlier announce FAILED
    /// (registered-but-unannounced). The `AlreadyRegistered` heal path a later
    /// demand reaches (`manager` upgraded, `canonical` known-registered). Behaviour:
    ///
    /// - Already announced ⇒ NO-OP (the normal idempotent re-demand — no deferred
    ///   state to heal).
    /// - Registered but NOT announced ⇒ re-attempt [`TransportManager::announce_egress_topic`]
    ///   (itself idempotent): success emits the loud recovery `info!` (the deferred
    ///   state clears — the topic is now discoverable); a repeat failure bumps
    ///   [`Self::announce_deferred`] and rides `debug!` (NO warn flood — the initial
    ///   serve already warned once about the retained-but-unannounced state, and the
    ///   flag is RETAINED / grantable throughout).
    ///
    /// A network-less manager (unreachable for a real gateway) is a silent no-op.
    fn heal_deferred_announce(&self, manager: &TransportManager, canonical: &str) {
        let Some(network) = manager.network() else {
            return;
        };
        if network.is_announced(canonical) {
            // Already announced — no deferred state, nothing to heal.
            return;
        }
        // Registered but NOT announced: an earlier hit's announce failed (deferred).
        match manager.announce_egress_topic(canonical) {
            Ok(()) => {
                tracing::info!(
                    topic = %canonical,
                    "permissive SHM probe: deferred announce HEALED — the network announce \
                     that failed on an earlier probe hit succeeded on this demand's re-attempt; \
                     remote discovery now lists this topic (the flag was grantable throughout)"
                );
            }
            Err(e) => {
                self.announce_deferred.fetch_add(1, Ordering::Relaxed);
                tracing::debug!(
                    topic = %canonical,
                    error = %e,
                    "permissive SHM probe: deferred announce re-attempt FAILED again \
                     (registered + demand-grantable; the flag is RETAINED and re-attempts on the \
                     next demand — see the first serve warning; announce_deferred keeps counting)"
                );
            }
        }
    }

    /// The probe body both demand paths funnel an unregistered-topic
    /// demand through. See the type doc for the full contract; the gates run in
    /// this order (cheapest / most-refusing first):
    ///
    /// 1. `upgrade()` the manager (→ [`ProbeVerdict::ManagerGone`] if mid-Drop).
    /// 2. Posture: `AllowAll` ONLY (→ [`ProbeVerdict::NotPermissive`] under Strict
    ///    — no probe, no counter; the "Strict never probes" decision).
    /// 3. Already registered? (→ [`ProbeVerdict::AlreadyRegistered`] — the enable
    ///    path owns it; makes double-demand idempotent). This arm is ALSO
    ///    the deferred-announce HEAL path — if the topic is registered but NOT yet
    ///    announced (an earlier hit's announce failed), it re-attempts the announce
    ///    idempotently ([`Self::heal_deferred_announce`]).
    /// 4. Reserved control-plane name OR ingress topic (STATIC declared set OR the
    ///    dynamic map)? (→ [`ProbeVerdict::LoopRefused`] — never serve
    ///    framework plumbing / an echo loop).
    /// 5. SHM open-only probe → [`ProbeVerdict::Served`] (register + announce) or
    ///    [`ProbeVerdict::Missing`] (refused, nothing created).
    ///
    /// Idempotent: a second call for an already-served topic returns
    /// `AlreadyRegistered` (step 3) — one registration, one announce (and, if the
    /// first announce failed, the heal happens here on the retry).
    pub(crate) fn try_serve(&self, topic: &str) -> ProbeVerdict {
        let Some(manager) = self.manager.upgrade() else {
            tracing::debug!(
                topic = %topic,
                "permissive SHM probe skipped — owning manager is gone (mid-Drop)"
            );
            return ProbeVerdict::ManagerGone;
        };
        let canonical = canonical_topic(topic).into_owned();

        // 2. Posture gate: AllowAll ONLY. Under a Strict (allow-list / deny-all)
        //    posture the probe NEVER runs — an explicit `network:` block is a
        //    tightening, so unknown topics stay refused (by design). A
        //    poisoned gate reads conservatively (do not probe).
        if !matches!(manager.bridge_manager().egress_is_allow_all(), Ok(true)) {
            return ProbeVerdict::NotPermissive;
        }

        // 3. Already registered (boot-plan, control channel, or an earlier
        //    probe) → the normal enable path owns it. No-op for the register — makes
        //    an idempotent double-demand register ONCE. This arm IS the
        //    deferred-announce HEAL path. An earlier probe hit whose announce FAILED
        //    left the topic registered (grantable) but UNannounced; the warn there
        //    promised it heals on the next demand — this is that demand, so
        //    re-attempt the announce idempotently. A topic already announced is a
        //    no-op (the normal re-demand); nothing else re-announces a probe-served
        //    topic, so without this the deferred state would be permanent.
        if manager
            .bridge_manager()
            .is_registered(&canonical)
            .unwrap_or(false)
        {
            self.heal_deferred_announce(&manager, &canonical);
            return ProbeVerdict::AlreadyRegistered;
        }

        // 4a. Reserved control-plane namespace: the `/__cerulion/*` services ARE
        //     live in local SHM (e.g. the runtime-registration control service),
        //     so an unguarded probe would find + serve one — leaking the control
        //     plane onto the network. Refuse it as a loop-class hazard (the same
        //     predicate `register_runtime_topic` / `register_dynamic_egress_topic`
        //     use — ONE source of truth).
        if is_reserved_topic(&canonical) {
            self.record_loop_refusal(&canonical, "reserved control-plane namespace");
            return ProbeVerdict::LoopRefused;
        }

        // 4b. Loop safety: a topic this gateway INGRESSES has a live local SHM
        //     service that is the gateway's OWN network re-injection. Serving it
        //     back out is the re-inject → tap → egress → re-inject echo loop
        //     the design refuses. NEVER probe-serve it.
        //
        //     Boot TOCTOU close: check the STATIC declared-ingress set
        //     (known at probe construction) FIRST, then the DYNAMIC ingress map.
        //     The dynamic map is populated only at the END of `register_ingress`,
        //     but the ingress SHM service goes live EARLIER at
        //     `create_ingress_publisher` — so a demand landing in that boot window
        //     would find a live service the dynamic check has not yet recorded and
        //     probe-serve the gateway's OWN ingress topic (a permanent echo loop).
        //     The static set closes that window structurally; the dynamic check
        //     stays to cover ingress ADDED at runtime.
        if self.declared_ingress.contains(&canonical)
            || manager
                .network()
                .map(|n| n.is_ingress_registered(&canonical))
                .unwrap_or(false)
        {
            // Its local SHM service is this gateway's OWN re-injection — serving it
            // back out would echo-loop.
            self.record_loop_refusal(&canonical, "network INGRESS topic — echo-loop");
            return ProbeVerdict::LoopRefused;
        }

        // 5. Probe local SHM open-only (`.open()`, never creates — TOCTOU-safe).
        //    A live service ⇒ HIT; any error ⇒ MISS (absent, or the rare
        //    slot-exhaustion false-miss — conservative: refuse now, heal on the
        //    next demand once a slot frees). The probe subscriber is DROPPED
        //    immediately (existence check only; the drive loop attaches the real
        //    egress tap).
        match manager.create_data_only_subscriber(&canonical) {
            Ok(_probe_sub) => {
                drop(_probe_sub);
                // Register + announce = register_runtime_topic semantics (the
                // approved announce-on-first-serve). Both idempotent. The bridge
                // map is ADD-ONLY, so the register COMMITS the grantable state
                // before the (reachably-fallible) announce.
                if let Err(e) = manager.bridge_manager().register_topic(&canonical) {
                    // A poisoned bridge mutex — the topic is neither registered nor
                    // announced. Count as a miss (nothing served) and move on.
                    self.misses.fetch_add(1, Ordering::Relaxed);
                    tracing::warn!(
                        topic = %canonical,
                        error = %e,
                        "permissive SHM probe: live service found but registration FAILED \
                         (bridge manager poisoned) — not served"
                    );
                    return ProbeVerdict::Missing;
                }
                if let Err(e) = manager.announce_egress_topic(&canonical) {
                    // The flag is COMMITTED (grantable), only DISCOVERY is
                    // degraded — the register_runtime_topic precedent: warn, RETAIN
                    // the flag (never roll back). Count the deferred announce
                    // (Principle #3) and warn ONCE (the loud first signal). It heals
                    // on the NEXT demand: that demand hits the AlreadyRegistered arm
                    // (step 3), which re-attempts the announce idempotently
                    // (`heal_deferred_announce`) — the ONLY path that re-announces a
                    // probe-served topic. Still a HIT (the topic IS served /
                    // grantable).
                    self.announce_deferred.fetch_add(1, Ordering::Relaxed);
                    tracing::warn!(
                        topic = %canonical,
                        error = %e,
                        "permissive SHM probe HIT registered + demand-grantable but its \
                         network announce FAILED — remote discovery will not list it yet (a demand \
                         token still grants egress; the flag is RETAINED). Heals on the next demand: \
                         its AlreadyRegistered arm re-attempts the announce idempotently"
                    );
                }
                self.hits.fetch_add(1, Ordering::Relaxed);
                // A genuine serve ends any refusal regime — re-arm BOTH latches.
                self.flood_warned.store(false, Ordering::Relaxed);
                self.loop_flood_warned.store(false, Ordering::Relaxed);
                tracing::info!(
                    topic = %canonical,
                    "permissive SHM probe HIT — live local service registered + announced \
                     on demand (announce-on-first-serve)"
                );
                ProbeVerdict::Served
            }
            Err(_e) => {
                self.misses.fetch_add(1, Ordering::Relaxed);
                // Warn ONCE per miss regime (a hostile flood of distinct missing
                // names logs once, not per demand); repeats ride debug. Re-armed by
                // a HIT above.
                if !self.flood_warned.swap(true, Ordering::Relaxed) {
                    tracing::warn!(
                        topic = %canonical,
                        "permissive SHM probe MISS — remote demand for a topic with no \
                         live local SHM service; refused (nothing created). Repeats log at debug; \
                         miss_count keeps counting"
                    );
                } else {
                    tracing::debug!(
                        topic = %canonical,
                        "permissive SHM probe MISS (repeat — see the first warning)"
                    );
                }
                ProbeVerdict::Missing
            }
        }
    }

    /// Principle #3: cumulative probe HITS (live services registered + announced).
    pub(crate) fn hit_count(&self) -> u64 {
        self.hits.load(Ordering::Relaxed)
    }

    /// Principle #3: cumulative probe MISSES (demands for absent local services).
    pub(crate) fn miss_count(&self) -> u64 {
        self.misses.load(Ordering::Relaxed)
    }

    /// Principle #3: cumulative LOOP refusals (ingress topic / reserved name).
    pub(crate) fn loop_refusal_count(&self) -> u64 {
        self.loop_refusals.load(Ordering::Relaxed)
    }

    /// Observable state (Principle #3): cumulative probe HITS whose network announce
    /// FAILED (registered + demand-grantable but not yet discoverable — the flag is
    /// RETAINED). A pure cumulative tally (never decremented): a `> 0` value is a
    /// transient-blip signal, not a permanent refusal — the topic still egresses on
    /// demand and its announce re-attempts (heals) on the next demand's
    /// `AlreadyRegistered` arm.
    pub(crate) fn announce_deferred_count(&self) -> u64 {
        self.announce_deferred.load(Ordering::Relaxed)
    }
}

/// The gateway's SINGLE query-surface CALLBACK — verb dispatch. Runs on
/// the zenoh query thread for every GET landing on `cerulion_q/{robot}/**`. Parses
/// the verb ([`cerulion_q::parse_query_verb`]) and routes it: `demand{topic}` to
/// the demand path (behavior preserved), `catalog` to the catalog
/// path, `schema/{pkg}/{Type}` to the schema-serving path, and
/// `runs` to the live-run path. An unrecognized/malformed key is a
/// `debug!` no-op (a future verb from a newer peer must not error).
///
/// The `authorizer` gate covers the WHOLE query surface — `demand`
/// (frame-serving), `catalog`, `schema` (the discovery plane) AND `runs`
/// all consult it before serving, so an unauthorized party can neither
/// pull frames nor ENUMERATE the robot's topic catalog / schemas / running graphs.
/// The default [`AllowAllAuthorizer`]
/// admits everything (deny-nothing plumbing, byte-identical); an account-grant authorizer
/// installs the `is_allowed(account)` predicate once and the entire surface is gated.
#[allow(clippy::too_many_arguments)]
fn handle_query(
    query: &zenoh::query::Query,
    bridge_mgr: &TopicBridgeManager,
    get_state: &GetDemandState,
    suppress: &AtomicBool,
    robot: &str,
    probe: Option<&PermissiveShmProbe>,
    self_ingress: &Mutex<HashSet<String>>,
    authorizer: &dyn DemandAuthorizer,
    catalog: &CatalogSource,
    schema: &SchemaSource,
    runs: &RunSource,
) {
    match cerulion_q::parse_query_verb(query.key_expr().as_str()) {
        Some(QueryVerb::Demand { topic }) => handle_demand_verb(
            query,
            bridge_mgr,
            get_state,
            suppress,
            robot,
            probe,
            self_ingress,
            authorizer,
            &topic,
        ),
        Some(QueryVerb::Catalog) => {
            handle_catalog_verb(query, bridge_mgr, robot, catalog, authorizer)
        }
        Some(QueryVerb::Schema { requested }) => {
            handle_schema_verb(query, robot, schema, &requested, authorizer)
        }
        Some(QueryVerb::Runs) => handle_runs_verb(query, robot, runs, authorizer),
        None => {
            tracing::debug!(
                key = %query.key_expr().as_str(),
                "query surface: unrecognized or malformed key — ignored"
            );
        }
    }
}

/// The `demand` verb — enable + ack. Recovers the canonical topic from
/// the parsed key, enables its egress bridge (the allow-list gate INSIDE
/// `enable_bridge` still enforces posture — the queryable can never widen egress
/// past the declared list), records the grant (keepalive stamp) on success, and
/// replies the synchronous ack.
///
/// The non-suppress decision runs the permissive SHM probe FIRST (see
/// [`demand_grant_decision`]) so a demand for an unregistered-but-live topic is
/// served + granted, not refused.
///
/// When `suppress` is set (the `suppress_live_demand` test seam — see its field
/// doc), this is a NON-flipping demand path exactly like the suppressed liveliness
/// subscriber: it replies REFUSED without ever calling `enable_bridge`, stamping a
/// keepalive, OR probing, so a test can make the reconciler the sole flag-flipper.
#[allow(clippy::too_many_arguments)]
fn handle_demand_verb(
    query: &zenoh::query::Query,
    bridge_mgr: &TopicBridgeManager,
    get_state: &GetDemandState,
    suppress: &AtomicBool,
    robot: &str,
    probe: Option<&PermissiveShmProbe>,
    self_ingress: &Mutex<HashSet<String>>,
    authorizer: &dyn DemandAuthorizer,
    canonical: &str,
) {
    let ack: &[u8] = if suppress.load(Ordering::Relaxed) {
        // Suppress arm (test seam): a NON-flipping demand path — reply REFUSED
        // without `enable_bridge`, a keepalive stamp, OR the probe. It touches no
        // shared state, so it stays OUTSIDE the `last_get` lock.
        tracing::debug!(
            topic = %canonical,
            "demand GET received but queryable enable suppressed (test seam)"
        );
        get_state.record_refusal();
        ACK_REFUSED
    } else {
        demand_grant_decision(
            bridge_mgr,
            get_state,
            probe,
            self_ingress,
            authorizer,
            canonical,
        )
    };
    // Reply on the CONCRETE key (our real identity + the recovered topic) — it
    // intersects both the wildcard and the explicit GET selector, so zenoh
    // accepts it either way, and the demander can attribute the replier.
    let reply_key = cerulion_q::demand_reply_key(robot, canonical);
    if let Err(e) = query.reply(reply_key.as_str(), ack.to_vec()).wait() {
        tracing::debug!(topic = %canonical, error = %e, "demand queryable: reply failed");
    }
}

/// The `catalog` verb — reply with the robot's full topic catalog
/// (every registered/announced topic + its schema hash the gateway holds). Reads
/// the live bridge manager + the shared [`CatalogSource`] (runtime hashes +
/// boot-topic provenance) and replies the versioned JSON payload on the concrete
/// `cerulion_q/{robot}/catalog` key. A reply failure is a `debug!` no-op — the
/// demander falls back to the announce listing.
///
/// Gated by the demand [`DemandAuthorizer`] BEFORE building the
/// catalog (see [`catalog_serve_decision`]) — an unauthorized party gets an
/// EXPLICIT refusal reply (empty entries + `error`), never the enumeration.
fn handle_catalog_verb(
    query: &zenoh::query::Query,
    bridge_mgr: &TopicBridgeManager,
    robot: &str,
    catalog: &CatalogSource,
    authorizer: &dyn DemandAuthorizer,
) {
    let reply = catalog_serve_decision(bridge_mgr, catalog, authorizer, robot);
    let entries = reply.entries.len();
    let refused = reply.error.is_some();
    let bytes = cerulion_q::encode_catalog_reply(&reply);
    let reply_key = cerulion_q::catalog_selector(robot);
    match query.reply(reply_key.as_str(), bytes).wait() {
        Ok(()) => {
            tracing::debug!(robot = %robot, entries, refused, "catalog reply served")
        }
        Err(e) => {
            tracing::debug!(robot = %robot, error = %e, "catalog queryable: reply failed")
        }
    }
}

/// The catalog-serve DECISION — the demand-authorization gate FIRST
/// (the access boundary), then the catalog build. An unauthorized
/// catalog GET serves NOTHING: a LOUD `warn!` plus an explicit
/// [`cerulion_q::CatalogReply::refused`] (empty entries + `error`), so a denied desk
/// cannot enumerate the robot's topics and gets an explicit refusal (never a silent
/// empty catalog). The LAN plane
/// carries no authenticated identity yet, so the subject is `Lan { locator: None }`
/// (no account yet). The DEFAULT authorizer admits everything (deny-nothing
/// — byte-identical to an un-gated plane); an account-grant authorizer swaps in
/// `is_allowed(account)`. Extracted so the
/// sync test seam runs the EXACT decision the real callback runs, minus the reply.
fn catalog_serve_decision(
    bridge_mgr: &TopicBridgeManager,
    catalog: &CatalogSource,
    authorizer: &dyn DemandAuthorizer,
    robot: &str,
) -> cerulion_q::CatalogReply {
    if let DemandDecision::Deny { reason } = authorizer.authorize_demand(
        &DemandSubject::Lan { locator: None },
        CATALOG_QUERY_RESOURCE,
    ) {
        tracing::warn!(
            robot = %robot,
            plane = "zenoh-lan",
            reason = %reason,
            "catalog GET REFUSED by the demand-authorization gate — the \
             demanding party is not authorized for this machine (account/pairing grant\
             ); enumerating nothing"
        );
        return cerulion_q::CatalogReply::refused(robot, reason);
    }
    build_catalog_reply_from_bridge(bridge_mgr, catalog, robot)
}

/// The `schema` verb — reply with the `.msg`/YAML TEXT of the
/// requested type plus its full nested-custom closure. `requested` is the
/// qualified `pkg/Type` OR a package-less bare `Name`,
/// keyed directly into the pre-built [`SchemaSource`] (handed to the gateway in
/// its plan). Computes the transitive closure ([`cerulion_q::collect_schema_closure`])
/// and replies the versioned JSON on the concrete `cerulion_q/{robot}/schema/{requested}`
/// key. An unknown type replies an EXPLICIT structured NOT-FOUND (never silence,
/// never empty-200); a reply-send failure is a `debug!` no-op (the desk falls
/// back to hash-only).
///
/// Gated by the demand [`DemandAuthorizer`] BEFORE resolving the
/// closure (see [`schema_serve_decision`]) — an unauthorized party gets an EXPLICIT
/// refusal reply (empty `docs` + `error`), never the served `.msg`/YAML text.
fn handle_schema_verb(
    query: &zenoh::query::Query,
    robot: &str,
    schema: &SchemaSource,
    requested: &str,
    authorizer: &dyn DemandAuthorizer,
) {
    let reply = schema_serve_decision(schema, authorizer, robot, requested);
    let served = reply.docs.len();
    let bytes = cerulion_q::encode_schema_reply(&reply);
    let reply_key = cerulion_q::schema_selector(robot, requested);
    match query.reply(reply_key.as_str(), bytes).wait() {
        Ok(()) => tracing::debug!(
            robot = %robot, requested = %requested, docs = served,
            "schema reply served"
        ),
        Err(e) => tracing::debug!(
            robot = %robot, requested = %requested, error = %e,
            "schema queryable: reply failed"
        ),
    }
}

/// The schema-serve DECISION — the demand-authorization gate FIRST
/// (the same access boundary the `demand`/`catalog` verbs consult), then the
/// closure resolution. An unauthorized schema GET serves NO `docs`: a LOUD `warn!`
/// plus an explicit [`cerulion_q::SchemaReply::refused`] (`error` set), so a denied
/// desk cannot pull the robot's `.msg`/YAML text and gets an explicit refusal.
/// Subject `Lan { locator: None }` (no account yet); the `requested` type
/// name is the resource. The DEFAULT authorizer admits everything (deny-nothing —
/// byte-identical); an account-grant authorizer swaps in `is_allowed(account)`.
/// Extracted so the sync test seam runs the EXACT decision the real callback runs,
/// minus the reply.
fn schema_serve_decision(
    schema: &SchemaSource,
    authorizer: &dyn DemandAuthorizer,
    robot: &str,
    requested: &str,
) -> cerulion_q::SchemaReply {
    if let DemandDecision::Deny { reason } =
        authorizer.authorize_demand(&DemandSubject::Lan { locator: None }, requested)
    {
        tracing::warn!(
            robot = %robot,
            requested = %requested,
            plane = "zenoh-lan",
            reason = %reason,
            "schema GET REFUSED by the demand-authorization gate — the \
             demanding party is not authorized for this machine (account/pairing grant\
             ); serving nothing"
        );
        return cerulion_q::SchemaReply::refused(robot, requested, reason);
    }
    // The docs map is merge-mutable now (the netd backfill seam) —
    // lock it for the closure walk (a control-plane GET, never the data plane).
    let docs = schema
        .docs
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    match cerulion_q::collect_schema_closure(requested, &docs) {
        Some(docs) => cerulion_q::SchemaReply::found(robot, requested, docs),
        None => cerulion_q::SchemaReply::not_found(
            robot,
            requested,
            format!(
                "robot '{robot}' does not have schema '{requested}' (not a workspace .msg / \
                 YAML type it serves); it may be a built-in type your desk already has, or a \
                 type this robot does not produce"
            ),
        ),
    }
}

/// The `runs` verb: reply with the runs LIVE on this machine right
/// now, each carrying its effective `graph.yaml` + `run.json` VERBATIM. Replies
/// the versioned JSON payload on the concrete `cerulion_q/{robot}/runs` key. A
/// reply failure is a `debug!` no-op (the desk reports "no answer", which its
/// decode side already renders as "could not establish", never "no runs").
///
/// Gated by the demand [`DemandAuthorizer`] BEFORE any gather (see
/// [`runs_serve_decision`]) — an unauthorized party gets an EXPLICIT refusal
/// reply, never the machine's run set.
fn handle_runs_verb(
    query: &zenoh::query::Query,
    robot: &str,
    runs: &RunSource,
    authorizer: &dyn DemandAuthorizer,
) {
    let reply = runs_serve_decision(runs, authorizer, robot);
    let served = reply.runs.len();
    let refused = reply.error.is_some();
    let settled = reply.completeness.is_settled();
    let bytes = cerulion_q::encode_runs_reply(&reply);
    let reply_key = cerulion_q::runs_selector(robot);
    match query.reply(reply_key.as_str(), bytes).wait() {
        Ok(()) => tracing::debug!(
            robot = %robot, runs = served, refused, settled,
            "runs reply served"
        ),
        Err(e) => tracing::debug!(
            robot = %robot, error = %e,
            "runs queryable: reply failed"
        ),
    }
}

/// The runs-serve DECISION: the demand-authorization gate FIRST
/// (the same access boundary `demand`/`catalog`/`schema` consult), then the
/// serve-time gather + run-directory read. Extracted so the sync test seam runs
/// the EXACT decision the real callback runs, minus the reply.
///
/// # Three answers, and none of them is a confident empty
///
/// 1. **Refused** — [`cerulion_q::RunsReply::refused`]: no runs, an `error`
///    reason, and a NON-settled verdict, so a denied desk cannot read the empty
///    list as "this robot is running nothing".
/// 2. **Gather failed** — empty runs +
///    [`cerulion_q::RunsCompleteness::not_established`] and NO `error`. A failed
///    gather is not a refusal (the `error` field means exactly "the robot refused
///    you"), so it is reported as the non-claim it is: the desk renders "could not
///    establish the running graph". A LOUD `warn!` carries the cause, because a
///    gateway that cannot reach its own machine's registry is a real fault.
/// 3. **Gathered** — whatever the registry established, with the gather's OWN
///    completeness verdict carried through unchanged. That passthrough is the
///    whole reason [`cerulion_q::RunsCompleteness`] exists: a windowed LISTEN that
///    expired with live writers unheard produces the SAME empty list a genuinely
///    idle machine produces, and only the verdict tells them apart (the same
///    lesson the discovery marker and the checked mirror gather taught).
///
/// Runs whose directory could not be read are SKIPPED with a named reason (see
/// [`run_artifacts::collect_run_entries`]) rather than served as entries carrying
/// empty documents — and each skip is logged, because a run present in the
/// registry and absent from the answer is otherwise an unexplained hole.
fn runs_serve_decision(
    runs: &RunSource,
    authorizer: &dyn DemandAuthorizer,
    robot: &str,
) -> cerulion_q::RunsReply {
    if let DemandDecision::Deny { reason } =
        authorizer.authorize_demand(&DemandSubject::Lan { locator: None }, RUNS_QUERY_RESOURCE)
    {
        tracing::warn!(
            robot = %robot,
            plane = "zenoh-lan",
            reason = %reason,
            "runs GET REFUSED by the demand-authorization gate — the \
             demanding party is not authorized for this machine (account/pairing grant\
             ); serving no runs"
        );
        return cerulion_q::RunsReply::refused(robot, reason);
    }
    let gather = match run_registry::gather_runs_on_config(&runs.iox_config, runs.gather_window) {
        Ok(gather) => gather,
        Err(e) => {
            tracing::warn!(
                robot = %robot,
                error = %e,
                "runs GET: the run registry could not be gathered — answering \
                 'could not establish', never 'no runs'"
            );
            return cerulion_q::build_runs_reply(
                robot,
                Vec::new(),
                Vec::new(),
                cerulion_q::RunsCompleteness::not_established(),
            );
        }
    };
    let fold = run_artifacts::collect_run_entries(&gather.records);
    for skip in &fold.skipped {
        tracing::warn!(
            robot = %robot,
            run_id = %cerulion_q::format_run_id(skip.run_id),
            run_dir = %skip.run_dir,
            reason = %skip.reason,
            "runs GET: skipping a live run whose directory could not be read — \
             an entry carrying an empty document would render desk-side as a graph with \
             no nodes"
        );
    }
    cerulion_q::build_runs_reply(
        robot,
        fold.entries,
        fold.skipped
            .iter()
            .map(run_artifacts::SkippedRun::to_wire)
            .collect(),
        gather.completeness.into(),
    )
}

/// The shared, thread-safe source the `catalog` verb reads to build a
/// [`cerulion_q::CatalogReply`]. The GATEWAY owns the writers — it drains the
/// runtime-registration control channel into `hashes`, and `boot_topics` is fixed
/// at boot — while the query-surface callback (on the zenoh callback thread) reads
/// them + the live bridge manager to answer a catalog GET. Cheap to clone (three
/// `Arc`s).
#[derive(Clone)]
pub struct CatalogSource {
    /// Canonical runtime-registered topic → the `schema_hash` the producing
    /// process advertised (shared with the gateway's reg-channel drain — the sole
    /// writer). A topic absent here (a boot-plan hashless announce) catalogs with
    /// `schema_hash: None`.
    pub hashes: Arc<Mutex<HashMap<String, u64>>>,
    /// Canonical boot-plan announce topics — provenance `Boot`; every other
    /// registered topic is provenance `Runtime`.
    pub boot_topics: Arc<HashSet<String>>,
    /// Canonical topic → its qualified schema NAME (`pkg/Type`),
    /// resolved by the CLI from the graph's node OUTPUTS and handed to the gateway
    /// in its plan. A topic absent here falls through to [`Self::hash_names`]
    /// (below); absent from both catalogs with `schema_name: None`. SEEDED at
    /// boot; a later registration's serving MERGES in through the
    /// gateway's `SchemaServingHandles` (the netd backfill seam), hence the
    /// `Mutex` — the same discipline as [`Self::hashes`] beside it.
    pub topic_schemas: Arc<Mutex<HashMap<String, String>>>,
    /// The hash→qualified-name REVERSE map — the CLI
    /// computes each served type's recipe-3 `schema_hash` (it holds the parsed IR
    /// for both `.msg` and YAML encodings) and hands the bindings to the gateway.
    /// A RUNTIME-registered topic (a `ros2 attach` raw route) arrives over the
    /// reg-channel carrying only a `(topic, schema_hash)` — NO name — so it is
    /// absent from [`Self::topic_schemas`]; the catalog builder names it by
    /// resolving ITS reg-channel hash through this map. A hash with no matching
    /// served doc stays `None` (never a guess). SEEDED at boot with the CLI-handed
    /// bindings (built-in corpus included); a later registration's
    /// serving MERGES in through the gateway's `SchemaServingHandles`.
    pub hash_names: Arc<Mutex<HashMap<u64, String>>>,
    /// A WEAK handle to the gateway's [`TransportManager`], used to probe
    /// each catalogued topic's LIVE producer count
    /// ([`TransportManager::topic_publisher_count`](super::TransportManager::topic_publisher_count))
    /// at serve time — the per-row LIVENESS affordance the desk sidebar needs
    /// (`Some(0)` = a registered-but-dead route, the Go2 `/uslam/cloud_map`
    /// case, `Some(n)` = a live producer). WEAK to avoid a manager → NetworkManager →
    /// query-surface → `CatalogSource` → manager reference cycle (mirroring
    /// `PermissiveShmProbe`'s weak manager handle). `None` on a serve path with no
    /// transport handle (the test seams) — the catalog then serves `producer_count:
    /// None` (byte-identical to a wire without the field). The probe is a CHEAP no-wait
    /// `.open()` + dynamic-config read per topic, paid once per catalog GET (a
    /// control-plane op per sidebar refresh), never on the data plane.
    pub producer_probe: Option<Weak<TransportManager>>,
    /// The shared DATA-FLOW liveness table
    /// ([`TopicLivenessObserver::table`](super::liveness::TopicLivenessObserver::table))
    /// the gateway's observer writes on its drive thread and this serve reads on
    /// the zenoh callback thread. Stamps each entry's
    /// [`CatalogEntry::liveness`](super::cerulion_q::CatalogEntry::liveness).
    ///
    /// This exists because [`Self::producer_probe`] answers the WRONG question
    /// for the desk affordance: it counts registered publisher ports, and a
    /// `ros2 attach` bridge registers one per discovered DDS topic at graph-build
    /// time whether or not the route ever carries a sample — so it reads
    /// `Some(1)` for a dead route and a streaming one alike. Frames must be
    /// OBSERVED, which requires a tap attached BEFORE the frame is published
    /// (a serve-time peek is provably empty on iceoryx2 — see the
    /// [`liveness`](super::liveness) module docs), hence a table fed by a
    /// long-lived observer rather than a probe called here.
    ///
    /// A plain (non-`Weak`) `Arc` is safe: the table holds only records, so it
    /// closes no cycle back to the manager. `None` on a serve path with no
    /// observer (the test seams) ⇒ every `liveness: None`, byte-identical to a
    /// wire without the field.
    pub liveness: Option<super::liveness::LivenessTable>,
    /// The clock the liveness AGES are computed against — the observer's
    /// own clock, so `last_frame_age_ms` is `robot_now - observed_at` with no
    /// cross-machine (or cross-clock) subtraction anywhere in the chain. Shares
    /// the gateway manager's clock handle. `None` ⇒ no liveness is stamped.
    pub liveness_clock: Option<Arc<dyn crate::clock::Clock>>,
}

/// The shared, thread-safe source the `schema` verb reads — the
/// pre-built map of served schema docs (qualified `pkg/Type` → [`SchemaDoc`]),
/// closure-complete over the robot's CUSTOM types (built-ins omitted — the desk
/// has them). The CLI builds it from the workspace and hands it to the gateway in
/// its plan; the callback thread reads it. SEEDED at boot; a later
/// registration's serving MERGES in through the gateway's
/// `SchemaServingHandles` (hence the `Mutex`). Cheap to clone (one `Arc`).
#[derive(Clone, Default)]
pub struct SchemaSource {
    /// Qualified `pkg/Type` → its served doc. Every doc's `deps` name OTHER keys
    /// in this map, so [`cerulion_q::collect_schema_closure`] never dangles
    /// (each merged serving is itself closure-complete, and the merge is
    /// key-additive — it removes nothing).
    pub docs: Arc<Mutex<std::collections::BTreeMap<String, SchemaDoc>>>,
}

/// What the `runs` verb needs to answer: the gateway host's OWN
/// iceoryx2 namespace, plus the window a serve-time gather is allowed to spend.
///
/// # Why a config and not a pre-built answer
///
/// The run set is not pushed to the gateway and is not cached: it is GATHERED at
/// serve time, because that is what makes ABSENCE exact for free. A dead run has
/// no registry writer *and* `RunDescriptor::drop` removes its directory, so there
/// is no retraction protocol to get wrong, no staleness window to tune, and no
/// way for the gateway to keep serving a run that ended (the
/// defect that killed the `register_egress` piggyback, whose `release_egress`
/// LINGERS, is precisely a lingering-positive).
///
/// # Why the manager's config, never the global one
///
/// The registry is keyed by iceoryx2 NAMESPACE, so a gather threading the wrong
/// one answers a confident empty about a machine that is running something —
/// which is the one answer this verb must never give. Both
/// shipping shapes were measured: the netd-hosted shared gateway and the per-run gateway CHILD
/// both resolve the namespace the registry writer publishes on, and the arm that
/// discriminates a manager-config gather from a global-config one is a host on a
/// NON-global namespace — exactly the per-run-child-after-a-real-mismatch shape.
/// Threading [`TransportManager::iox_config`] is what makes this correct on both.
///
/// # Cost, stated
///
/// `gather_runs_on_config` builds a FRESH iceoryx2 reader node per call (its
/// documented contract), so a runs GET is not free — it is the same shape
/// `gather_mirror_provenance` pays, which was MEASURED at ~620 ms on an idle
/// desk, plus up to [`Self::gather_window`] when a live writer has to be heard
/// from. That is affordable only because the DAG is immutable for the life of a
/// `run_id` and the desk therefore fetches it ONCE per run, and it is why
/// this verb must never be folded into the ~2 s catalog poll. RESIDUAL, stated
/// rather than implied: the serve runs on the zenoh callback thread that also
/// serves `demand`, so a runs GET delays a concurrent demand GET by its own
/// duration.
///
/// Cheap to clone (one `Arc` + a `Duration`), mirroring [`CatalogSource`].
#[derive(Clone)]
pub struct RunSource {
    /// The gateway host's own iceoryx2 namespace — the one its
    /// [`TransportManager`] was initialized with.
    pub iox_config: Arc<iceoryx2::config::Config>,
    /// The ceiling ONE serve-time gather may spend listening. A machine with no
    /// live run answers immediately regardless (the registry's zero-publisher
    /// fast path), so this is paid only when a writer is live and has not yet
    /// been heard from.
    pub gather_window: Duration,
}

/// Assemble a catalog reply from the LIVE gateway state — the bridge
/// manager's registered topics (the authoritative produced/announced set) joined
/// with the shared runtime hashes + boot-topic provenance. Pure sort/dedup +
/// version stamping is delegated to [`cerulion_q::build_catalog_reply`].
fn build_catalog_reply_from_bridge(
    bridge_mgr: &TopicBridgeManager,
    catalog: &CatalogSource,
    robot: &str,
) -> cerulion_q::CatalogReply {
    let registered = bridge_mgr.registered_topics().unwrap_or_default();
    let hashes = catalog
        .hashes
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    // The name maps are merge-mutable now (the netd backfill seam) —
    // lock each ONCE for the whole sweep, exactly like `hashes` above.
    let topic_schemas = catalog
        .topic_schemas
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    let hash_names = catalog
        .hash_names
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    // Upgrade the WEAK manager handle ONCE for the whole sweep (a live
    // gateway upgrade always succeeds — the manager owns this query surface). `None`
    // on a probe-less serve path (a test seam) ⇒ every `producer_count: None` (the
    // field-less wire).
    let producer_mgr = catalog.producer_probe.as_ref().and_then(Weak::upgrade);
    // Read the observer's clock ONCE for the whole sweep, so every entry
    // in one reply is aged against the same instant (two topics observed in the
    // same sweep must not disagree about "now").
    let liveness_now_ns = catalog.liveness_clock.as_ref().map(|c| c.now_ns());
    let entries = registered.into_iter().map(|(topic, _flag)| {
        let schema_hash = hashes.get(&topic).copied();
        // The topic's LIVE producer count — the desk's per-row liveness
        // affordance (`Some(0)` = a registered-but-dead route, `Some(n)` = live). A
        // cheap no-wait `.open()` + dynamic-config read. `None` when there is no manager
        // (a test seam) OR when the probe itself FAILS (a transient
        // iceoryx2 error, e.g. fd pressure): a failed probe reports UNKNOWN, never a
        // fabricated `Some(0)` that would dim a healthy topic (the user-visible
        // inverse bug). `topic_publisher_count_checked` owns that dead-vs-unknown split.
        let producer_count = producer_mgr
            .as_ref()
            .and_then(|mgr| mgr.topic_publisher_count_checked(&topic));
        // The qualified schema NAME the CLI resolved for this topic (so
        // the desk knows WHICH type to `schema`-fetch). Precedence: the explicit
        // CLI topic→name binding FIRST (a graph node OUTPUT), then
        // the hash→name reverse map for a RUNTIME-registered topic (a
        // `ros2 attach` raw route, absent from `topic_schemas`, whose reg-channel
        // hash names it). Absent from BOTH ⇒ `None` (never a guess).
        let schema_name = topic_schemas
            .get(&topic)
            .cloned()
            .or_else(|| schema_hash.and_then(|h| hash_names.get(&h).cloned()));
        let provenance = if catalog.boot_topics.contains(&topic) {
            CatalogProvenance::Boot
        } else {
            CatalogProvenance::Runtime
        };
        // The DATA-FLOW half. `None` whenever the robot genuinely
        // observed nothing (no observer wired, observation disabled, or a tap
        // that never attached) — a genuine UNKNOWN that the desk renders exactly
        // like an old robot's, NEVER as "dead".
        let liveness = match (&catalog.liveness, liveness_now_ns) {
            (Some(table), Some(now_ns)) => super::liveness::read_liveness(table, &topic, now_ns),
            _ => None,
        };
        CatalogEntry {
            topic,
            schema_hash,
            schema_name,
            provenance,
            producer_count,
            liveness,
        }
    });
    cerulion_q::build_catalog_reply(robot, entries)
}

/// The non-suppress demand-grant DECISION — the permissive SHM probe
/// (register-on-first-serve for a live unregistered AllowAll topic) followed by
/// the enable + ack. Extracted from [`handle_demand_verb`] so the sync
/// test seam ([`demand_grant_decision_for_test`]) exercises the EXACT queryable
/// decision (probe + enable + ack) the real zenoh callback runs, minus the reply.
/// Returns the ack bytes ([`ACK_ENABLED`] granted / [`ACK_REFUSED`]).
#[allow(clippy::too_many_arguments)]
fn demand_grant_decision(
    bridge_mgr: &TopicBridgeManager,
    get_state: &GetDemandState,
    probe: Option<&PermissiveShmProbe>,
    self_ingress: &Mutex<HashSet<String>>,
    authorizer: &dyn DemandAuthorizer,
    canonical: &str,
) -> &'static [u8] {
    // The demand-authorization gate — the access boundary, consulted
    // BEFORE any probe/enable so an unauthorized demand serves NOTHING (no SHM open, no
    // announce, no egress flag). The DEFAULT authorizer admits everything (deny-nothing
    // plumbing, byte-identical); an account-grant authorizer swaps in `is_allowed(account)`. The
    // LAN subject carries no authenticated identity yet (the zenoh demand callback does not
    // thread the querier's identity), so it is `Lan { locator: None }`.
    // A refusal is LOUD (never a silent drop) and counted as a refusal, exactly like an
    // allow-list miss.
    if let DemandDecision::Deny { reason } =
        authorizer.authorize_demand(&DemandSubject::Lan { locator: None }, canonical)
    {
        tracing::warn!(
            topic = %canonical,
            plane = "zenoh-lan",
            reason = %reason,
            "demand REFUSED by the demand-authorization gate — the demanding \
             party is not authorized for this machine (account/pairing grant); \
             serving nothing"
        );
        get_state.record_refusal();
        return ACK_REFUSED;
    }
    // Permissive SHM probe FIRST — OUTSIDE the `last_get` critical
    // section (the SHM open + zenoh announce must not extend the FINDING-1 lock
    // scope). A demand for an UNregistered topic under AllowAll egress that is LIVE
    // in local SHM is registered + announced here, so the enable below GRANTS it
    // exactly like a plan-declared producer. `try_serve` self-gates (AllowAll-only,
    // ingress-loop-safe, idempotent) — a no-op for a registered topic or a Strict
    // posture, so a healthy grant pays nothing new.
    if let Some(probe) = probe {
        probe.try_serve(canonical);
    }
    // A demand GET for a topic THIS gateway INGRESSES is never an egress
    // candidate — enabling its bridge would be the
    // re-inject→tap→egress→re-inject loop, and (the asymmetric-guard case) a
    // Strict gateway that declares BOTH egress and ingress topics runs its OWN
    // demand-GET loop over its ingress topics, self-querying this queryable every
    // pass; without this skip `enable_bridge` refuses the ingress topic
    // (GatewayPlan::validate guarantees ingress∩egress=∅) and emits the
    // wrong-audience WARN about the machine's OWN ingress. Refuse it BEFORE the
    // enable — AFTER the probe (whose loop-refusal accounting is a distinct,
    // still-exercised path) — using the SAME predicate the liveliness watch's Put
    // arm consults. The genuine egress refusal (a produced-but-unlisted topic) is
    // untouched: a produced topic never enters `self_ingress`.
    if demand_put_targets_self_ingress(self_ingress, canonical) {
        tracing::debug!(
            topic = %canonical,
            "demand GET for our own network-ingress topic — not an egress candidate (loop exclusion); refused without enable_bridge"
        );
        get_state.record_refusal();
        return ACK_REFUSED;
    }
    // Grant/expiry atomicity: hold `last_get` across `enable_bridge`
    // + `is_enabled` + the keepalive stamp so the expiry sweep — which holds
    // `last_get` across its WHOLE sweep — cannot race a fresh grant into a spurious
    // disable (enable → sweep disable+remove → stamp would leave egress OFF but the
    // ack lying ENABLED with a fresh stamp).
    //
    // LOCK ORDER: both this decision and `run_get_expiry` take `last_get` FIRST,
    // then `bridges` (inside `enable_bridge`/`is_enabled`/`disable_bridge`, each
    // its own scoped lock) — one consistent order, no deadlock.
    let mut last = get_state
        .last_get
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    let enabled = match bridge_mgr.enable_bridge(canonical) {
        // `enable_bridge`'s bool is the TRANSITION, not the state — read
        // `is_enabled` for the ack so an already-egressing re-demand is not
        // mislabeled refused.
        Ok(_transitioned) => bridge_mgr.is_enabled(canonical).unwrap_or(false),
        Err(e) => {
            tracing::error!(
                topic = %canonical,
                error = %e,
                "demand queryable: enable_bridge failed (bridge manager poisoned)"
            );
            false
        }
    };
    if enabled {
        // Stamp under the SAME held guard — grant + keepalive are atomic with the
        // enable above (the FINDING-1 critical section).
        get_state.record_grant_locked(&mut last, canonical);
        ACK_ENABLED
    } else {
        // Refusal touches neither map, so release `last_get` before the bump (the
        // refusal path needs no lock — matches the suppress arm).
        drop(last);
        get_state.record_refusal();
        ACK_REFUSED
    }
}

/// The liveliness watch's Put-demand ACTION — the permissive SHM probe
/// (register-on-first-serve for a live unregistered AllowAll topic) followed by
/// the `enable_bridge`. Extracted so the liveliness watch loop AND the
/// sync test seam ([`apply_live_put_demand_for_test`]) run the SAME body — so a
/// probe-hit via the liveliness path converges to the same registered + announced
/// + enabled state as the queryable path. Returns the `enable_bridge` transition.
fn apply_live_put_demand(
    bridge_mgr: &TopicBridgeManager,
    probe: Option<&PermissiveShmProbe>,
    authorizer: &dyn DemandAuthorizer,
    topic: &str,
) -> TransportResult<bool> {
    // The SAME demand-authorization gate the queryable path consults, so
    // the liveliness Put-demand path cannot widen access past it. An unauthorized
    // demand serves NOTHING (no probe, no enable) — loud, never silent. The default
    // authorizer admits everything (byte-identical); an account-grant authorizer installs
    // `is_allowed(account)`.
    if let DemandDecision::Deny { reason } =
        authorizer.authorize_demand(&DemandSubject::Lan { locator: None }, topic)
    {
        tracing::warn!(
            topic = %topic,
            plane = "zenoh-lan",
            reason = %reason,
            "liveliness demand REFUSED by the demand-authorization gate — \
             the demanding party is not authorized for this machine (account/pairing \
             grant); egress not enabled"
        );
        return Ok(false);
    }
    // Same permissive SHM probe as the queryable path — a live unregistered
    // AllowAll topic is registered + announced here so the enable below flips its
    // (now-existing) egress flag. Self-gating (see `PermissiveShmProbe::try_serve`).
    if let Some(probe) = probe {
        probe.try_serve(topic);
    }
    bridge_mgr.enable_bridge(topic)
}

/// Whether a demand Put for `topic` targets a topic THIS manager
/// INGRESSES — i.e. our OWN network re-injection's demand echo. The liveliness
/// watch subscribes to the demand key-space and hears EVERY demand token,
/// including the one [`NetworkManager::register_ingress`] declared for a topic
/// this machine merely consumes (a `topic hz`/`echo` desk, or any ingress-only
/// peer). An ingressed topic is NEVER an egress candidate — enabling its bridge
/// would be the re-inject → tap → egress → re-inject loop — so the watch
/// SKIPS [`apply_live_put_demand`] for it. Without the skip, on a deny-all /
/// ingress-only manager `enable_bridge` refuses the topic (allow-list miss) and
/// emits the "bridge-enable refused: … egress allow-list …" WARN about the
/// machine's own demand token — wrong-audience noise on any
/// desk that only consumes. The load-bearing genuine refusal (a topic the machine PRODUCES
/// but did not egress-list under a Strict allow-list) is UNAFFECTED: a produced
/// topic never enters `self_ingress`, so it still reaches `enable_bridge` and
/// warns. Poison-recovering: a panic while some thread held the set must not
/// silently flip the verdict to "not ingress" and re-open the false warn.
fn demand_put_targets_self_ingress(self_ingress: &Mutex<HashSet<String>>, topic: &str) -> bool {
    self_ingress
        .lock()
        .map(|s| s.contains(topic))
        .unwrap_or_else(|poison| poison.into_inner().contains(topic))
}

/// SYNC SEAM (queryable path): run the EXACT queryable grant decision
/// ([`demand_grant_decision`], probe + enable + ack) inline and report whether it
/// GRANTED (ack == [`ACK_ENABLED`]) — the deterministic, zenoh-timing-free driver
/// the gateway's `demand_query_grants_for_test` delegates to.
#[cfg(any(test, feature = "test-helpers"))]
#[allow(clippy::too_many_arguments)]
pub(crate) fn demand_grant_decision_for_test(
    bridge_mgr: &TopicBridgeManager,
    get_state: &GetDemandState,
    probe: &PermissiveShmProbe,
    self_ingress: &Mutex<HashSet<String>>,
    authorizer: &dyn DemandAuthorizer,
    topic: &str,
) -> bool {
    let canonical = canonical_topic(topic).into_owned();
    demand_grant_decision(
        bridge_mgr,
        get_state,
        Some(probe),
        self_ingress,
        authorizer,
        &canonical,
    ) == ACK_ENABLED
}

/// SYNC SEAM (liveliness path): run the EXACT liveliness Put action
/// ([`apply_live_put_demand`], probe + `enable_bridge`) inline and report whether
/// the topic is now ENABLED (grantable) — the deterministic driver the gateway's
/// `liveliness_demand_grants_for_test` delegates to. Proves the liveliness path
/// converges to the same served + grantable state as the queryable path.
#[cfg(any(test, feature = "test-helpers"))]
pub(crate) fn apply_live_put_demand_for_test(
    bridge_mgr: &TopicBridgeManager,
    probe: &PermissiveShmProbe,
    authorizer: &dyn DemandAuthorizer,
    topic: &str,
) -> bool {
    let canonical = canonical_topic(topic).into_owned();
    let _ = apply_live_put_demand(bridge_mgr, Some(probe), authorizer, &canonical);
    bridge_mgr.is_enabled(&canonical).unwrap_or(false)
}

/// SYNC SEAM (catalog verb): run the EXACT catalog-serve DECISION
/// ([`catalog_serve_decision`], authorizer gate → build) inline and return the
/// reply — the deterministic, zenoh-timing-free driver the gateway's
/// `catalog_serve_for_test` delegates to. A REFUSED catalog carries `error: Some(..)`
/// + empty entries; an authorized one carries the live catalog.
#[cfg(any(test, feature = "test-helpers"))]
pub(crate) fn catalog_serve_decision_for_test(
    bridge_mgr: &TopicBridgeManager,
    catalog: &CatalogSource,
    authorizer: &dyn DemandAuthorizer,
    robot: &str,
) -> cerulion_q::CatalogReply {
    catalog_serve_decision(bridge_mgr, catalog, authorizer, robot)
}

/// SYNC SEAM (schema verb): run the EXACT schema-serve DECISION
/// ([`schema_serve_decision`], authorizer gate → closure) inline and return the
/// reply — the deterministic driver the gateway's `schema_serve_for_test`
/// delegates to. A REFUSED schema carries `error: Some(..)` + empty docs.
#[cfg(any(test, feature = "test-helpers"))]
pub(crate) fn schema_serve_decision_for_test(
    schema: &SchemaSource,
    authorizer: &dyn DemandAuthorizer,
    robot: &str,
    requested: &str,
) -> cerulion_q::SchemaReply {
    schema_serve_decision(schema, authorizer, robot, requested)
}

/// SYNC SEAM (runs verb): run the EXACT runs-serve DECISION
/// ([`runs_serve_decision`], authorizer gate → gather → run-directory fold)
/// inline and return the reply — the deterministic, zenoh-timing-free driver the
/// gateway's `runs_serve_for_test` delegates to. A REFUSED runs GET carries
/// `error: Some(..)` + no runs + a non-settled verdict.
#[cfg(any(test, feature = "test-helpers"))]
pub(crate) fn runs_serve_decision_for_test(
    runs: &RunSource,
    authorizer: &dyn DemandAuthorizer,
    robot: &str,
) -> cerulion_q::RunsReply {
    runs_serve_decision(runs, authorizer, robot)
}

/// The producer EXPIRY sweep — disable GET-granted topics whose last GET
/// aged past `DEMAND_TTL` AND which liveliness is not holding this pass (the
/// composition rule). Holds the `last_get` lock across the WHOLE sweep
/// (`get_expired_topics` → `disable_bridge` → `remove`) so a fresh GET landing
/// mid-sweep cannot be raced into a spurious disable: the queryable callback
/// ([`handle_demand_verb`]) now ALSO holds `last_get` across its
/// `enable_bridge`/`is_enabled`/keepalive-stamp critical section, so
/// a grant either completes fully BEFORE this sweep reads the map (its fresh
/// stamp is then non-expired) or blocks until AFTER the sweep removed+disabled
/// the topic (then re-enables AND re-stamps atomically) — the "egress OFF but ack
/// ENABLED" interleave is gone. LOCK ORDER: both the callback and this sweep take
/// `last_get` FIRST, then `bridges` (inside `enable_bridge`/`is_enabled`/
/// `disable_bridge`, each its own scoped lock) — one consistent order, no
/// deadlock. A genuine true→false disable is logged (the caller-owned edge
/// log) + counted.
fn run_get_expiry(
    bridge_mgr: &TopicBridgeManager,
    get_state: &GetDemandState,
    liveliness_demand: &HashSet<String>,
    now: Instant,
) {
    let mut last_get = get_state
        .last_get
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    let expired = get_expired_topics(&last_get, liveliness_demand, now, DEMAND_TTL);
    for topic in &expired {
        match bridge_mgr.disable_bridge(topic) {
            Ok(true) => {
                get_state.expiries.fetch_add(1, Ordering::Relaxed);
                tracing::info!(
                    topic = %topic,
                    ttl_secs = DEMAND_TTL.as_secs(),
                    "network bridge disabled — no demand GET within DEMAND_TTL and \
                     liveliness absent (both demand paths released this topic)"
                );
            }
            // Already off (another path lowered it first) — still stop tracking.
            Ok(false) => {}
            Err(e) => tracing::error!(
                topic = %topic,
                error = %e,
                "expiry: disable_bridge failed (bridge manager poisoned)"
            ),
        }
        last_get.remove(topic);
    }
}

/// One full demand-GET pass on an owned session (the DEMANDER side) —
/// the shared body of the background GET loop
/// ([`NetworkManager::start_demand_get_loop`]) and the sync seam
/// ([`NetworkManager::demand_get_pass`]). Harvests producer identities from the
/// announce space, then GETs every registered ingress topic (explicitly per
/// identity, else the wildcard) and tallies the acks. Per-pass failure latched
/// loud-once; a harvest failure is a debug (the wildcard still works without it).
///
/// Mid-pass shutdown: `should_abort` is polled BEFORE the pass starts
/// and BETWEEN selector GETs so a `Drop`-triggered teardown returns after the
/// current blocking GET rather than the whole harvest + topics × selectors sweep.
/// An aborted pass returns WITHOUT recording a pass failure — a teardown is NOT a
/// link failure, so it must not trip the loud-once GET-failure latch. The sync
/// seam passes a never-abort closure.
fn run_demand_get_pass_on_session(
    session: &zenoh::Session,
    demand_state: &DemandGetState,
    self_identity: Option<&str>,
    should_abort: &dyn Fn() -> bool,
) {
    // Abort before ANY work (incl. bumping the pass counter) so a Drop
    // landing at the very top of a pass is a clean no-op teardown, not a counted
    // partial pass.
    if should_abort() {
        return;
    }
    demand_state.passes.fetch_add(1, Ordering::Relaxed);

    // 1. Identity harvest from the ANNOUNCE space (a dialer→accepter query — the
    //    working direction). The producer announces its egress topics + a bare
    //    identity token, so this learns the producer's robot chunk. A failure is
    //    non-fatal — the wildcard fallback still reaches every producer.
    match query_announce_entries(session, DEMAND_IDENTITY_WINDOW) {
        Ok(entries) => {
            let mut ids = demand_state
                .identities
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            for (robot, _topic) in entries {
                // Belt and suspenders: NEVER demand from OURSELVES. A
                // Strict gateway announces its own identity, so the harvest would
                // otherwise learn self and emit a `cerulion_q/{self}/demand{...}`
                // self-query every pass — including for its own INGRESS topics,
                // which the grant-side self-ingress guard already refuses without
                // warning. Dropping self here avoids the wasted self round-trip
                // entirely (the wildcard-`*` fallback residual stays covered by
                // that grant-side guard).
                if self_identity != Some(robot.as_str()) {
                    ids.insert(robot);
                }
            }
        }
        Err(e) => tracing::debug!(
            error = %e,
            "demand GET: announce identity harvest failed — using the wildcard selector"
        ),
    }

    // 2. Snapshot the topics + identities, then GET each (explicit or wildcard).
    let topics: Vec<String> = demand_state
        .topics
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .clone();
    let identities: BTreeSet<String> = demand_state
        .identities
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .clone();

    let mut pass_failure: Option<TransportError> = None;
    for topic in &topics {
        for selector in demand_get_selectors(topic, &identities) {
            // Abort between GETs — a mid-teardown pass returns after the
            // current blocking GET rather than the whole topics × selectors sweep.
            // Return WITHOUT recording a pass failure (shutdown is not a link
            // failure — it must not trip the loud-once GET-failure latch).
            if should_abort() {
                return;
            }
            match session
                .get(selector.as_str())
                .timeout(DEMAND_GET_TIMEOUT)
                .wait()
            {
                Ok(replies) => {
                    // Bounded by the query timeout — the channel closes when the
                    // GET completes or the timeout elapses.
                    while let Ok(reply) = replies.recv() {
                        if let Ok(sample) = reply.into_result() {
                            let payload = sample.payload().to_bytes();
                            let bytes = payload.as_ref();
                            if bytes == ACK_ENABLED {
                                demand_state.enabled_acks.fetch_add(1, Ordering::Relaxed);
                            } else if bytes == ACK_REFUSED {
                                demand_state.refused_acks.fetch_add(1, Ordering::Relaxed);
                            }
                        }
                    }
                }
                Err(e) => {
                    pass_failure = Some(TransportError::Receive {
                        topic: selector.clone(),
                        reason: format!("demand GET failed: {e}"),
                    })
                }
            }
        }
    }

    match pass_failure {
        Some(e) => demand_state.record_get_failure(&e),
        None => demand_state.record_get_success(),
    }
}

/// Why an inbound network frame was rejected by
/// `validate_ingress_frame`. A frame is only re-injected into local
/// iceoryx2 once it passes ALL of these checks — a rejection is a loud,
/// COUNTED drop, never a crash and never a silent one (Principle #6).
///
/// The remote-plane work made this `pub` so the zenoh-free
/// [`IngressInjector::reinject_raw`] return value ([`ReinjectOutcome::Rejected`])
/// can carry the exact rejection class to a desk / `cerulion_remoted` caller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IngressRejection {
    /// Fewer than `WireHeader::SIZE` (32) bytes — cannot contain a header.
    TooSmall,
    /// The 32 header bytes did not parse into a [`WireHeader`]. Defensive:
    /// `WireHeader::read_from_buf` fails only on a short buffer (already caught
    /// by `TooSmall`), so this arm is unreachable today, but it makes the
    /// header-parse failure an EXPLICIT rejection class if that contract ever
    /// changes.
    BadHeader,
    /// `header.total_size` disagrees with the received byte length — the frame
    /// is truncated or corrupt on the wire.
    SizeMismatch,
    /// `header.schema_hash` did not match the topic's expected schema hash —
    /// a publisher/subscriber schema skew (the most common cross-machine
    /// misconfiguration).
    SchemaMismatch,
}

impl std::fmt::Display for IngressRejection {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            IngressRejection::TooSmall => "frame smaller than the 32-byte wire header",
            IngressRejection::BadHeader => "wire header failed to parse",
            IngressRejection::SizeMismatch => "header total_size does not match the frame length",
            IngressRejection::SchemaMismatch => {
                "schema_hash does not match the topic's expected schema"
            }
        })
    }
}

/// Validate a raw network frame IN PLACE against the topic's expected
/// schema hash. STRONGER than the deleted prost envelope's implicit checks:
/// length ≥ header, header parses, `total_size == bytes.len()`, and
/// `schema_hash == expected_hash`. On success the parsed [`WireHeader`] is
/// returned (callers ignore it; unit tests inspect it). Pure — no transport,
/// no allocation — so it is exhaustively oracle-tested with hand-built frames.
fn validate_ingress_frame(
    expected_hash: u64,
    bytes: &[u8],
) -> Result<WireHeader, IngressRejection> {
    if bytes.len() < WireHeader::SIZE {
        return Err(IngressRejection::TooSmall);
    }
    let header = WireHeader::read_from_buf(bytes).ok_or(IngressRejection::BadHeader)?;
    if header.total_size as usize != bytes.len() {
        return Err(IngressRejection::SizeMismatch);
    }
    if header.schema_hash != expected_hash {
        return Err(IngressRejection::SchemaMismatch);
    }
    Ok(header)
}

/// Observable ingress counters for ONE bridged topic (Principle #3:
/// state observable independently of execution). Shared (`Arc`) between the
/// zenoh receive callback (writer) and [`NetworkManager::ingress_stats`]
/// (reader). All fields are `Relaxed` atomics — a monotone tally, never a
/// synchronization point.
#[derive(Debug, Default)]
struct IngressCounters {
    /// Frames validated AND re-injected into the local iceoryx2 publisher
    /// (the success count — a frame that validated but failed re-injection is
    /// NOT counted here; it lands in `reinject_failure_drops`).
    frames: AtomicU64,
    /// Frames dropped because the wire `schema_hash` did not match the topic's
    /// expected schema hash.
    schema_mismatch_drops: AtomicU64,
    /// Frames dropped because they were structurally undecodable as a Cerulion
    /// wire frame (too small / unparseable header / `total_size` ≠ byte length).
    decode_errors: AtomicU64,
    /// Frames that passed validation but were
    /// dropped because the local re-injection failed (`publish_raw` Err — e.g.
    /// loan-pool exhaustion — or a poisoned publisher mutex). Distinct from the
    /// two validation buckets: this is LOCAL transport pressure / a code bug,
    /// not a malformed or mis-schemed remote.
    reinject_failure_drops: AtomicU64,
}

/// Snapshot of a bridged topic's ingress counters (Principle #3).
/// Returned by [`NetworkManager::ingress_stats`]; `Copy` so callers can freely
/// pass it around and diff two snapshots.
///
/// # Accounting contract
///
/// `frames` counts frames validated AND re-injected (successes). The THREE
/// drop buckets — `schema_mismatch_drops` + `decode_errors` +
/// `reinject_failure_drops` — sum to the total loss: every received frame
/// lands in exactly one of the four counters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct IngressStats {
    /// Frames validated AND re-injected into local iceoryx2 (successes only).
    pub frames: u64,
    /// Frames dropped because the wire `schema_hash` did not match the topic's
    /// expected schema hash.
    pub schema_mismatch_drops: u64,
    /// Frames dropped because they were structurally undecodable as a Cerulion
    /// wire frame (too small / unparseable header / `total_size` ≠ byte length).
    pub decode_errors: u64,
    /// Frames that PASSED validation but were dropped because the local
    /// re-injection failed (`publish_raw` Err such as loan-pool exhaustion, or
    /// a poisoned publisher mutex). Local transport pressure / code-bug
    /// signal — investigate THIS machine, not the remote publisher.
    pub reinject_failure_drops: u64,
}

/// First-warn-then-debug rejection-log latch, one per bridged topic
/// (rate-conscious logging modeled on [`crate::graph::drain_latch::DrainWarnLatch`]).
/// The FIRST failure of a class since the last fully-successful frame logs at
/// `warn` (`error` for the poisoned-mutex class — see
/// [`record_reinject_failure`]); repeats downgrade to `debug` so a
/// persistently failing regime at line rate does not flood the log. A
/// successful re-injection [`reset`](Self::reset)s every bit — the caller
/// logs the recovery at `info` exactly once (the `DrainWarnLatch::on_success`
/// shape) — so a fresh failure regime warns again. Interior-mutable
/// (`AtomicBool`) so the `Fn` zenoh callback can flip it.
#[derive(Debug, Default)]
struct IngressWarnLatch {
    /// True once a schema-mismatch has warned since the last reset.
    schema_mismatch_warned: AtomicBool,
    /// True once a decode-error has warned since the last reset.
    decode_error_warned: AtomicBool,
    /// True once a re-injection failure (publish_raw Err / poisoned mutex)
    /// has warned since the last reset.
    reinject_failure_warned: AtomicBool,
}

impl IngressWarnLatch {
    /// Clear every latch bit — called after a fully-successful re-injection so
    /// the NEXT failure of any class warns loudly again (regime detection).
    /// Returns `true` exactly when this success ends a failure run (any bit
    /// was set) — the caller logs the recovery at `info` once, mirroring
    /// `DrainWarnLatch::on_success`. Steady-state successes return `false`
    /// so the callback stays log-free.
    fn reset(&self) -> bool {
        let schema = self.schema_mismatch_warned.swap(false, Ordering::Relaxed);
        let decode = self.decode_error_warned.swap(false, Ordering::Relaxed);
        let reinject = self.reinject_failure_warned.swap(false, Ordering::Relaxed);
        schema || decode || reinject
    }
}

/// Bump the right counter for a rejected ingress frame and log it
/// rate-consciously (first of a class → `warn!`, repeats → `debug!`). Never
/// crashes, never silently drops. `SchemaMismatch` counts as a schema drop;
/// every other class is a structural decode error.
fn record_ingress_rejection(
    counters: &IngressCounters,
    warn: &IngressWarnLatch,
    topic: &str,
    rejection: IngressRejection,
) {
    let (counter, already_warned) = match rejection {
        IngressRejection::SchemaMismatch => (
            &counters.schema_mismatch_drops,
            warn.schema_mismatch_warned.swap(true, Ordering::Relaxed),
        ),
        IngressRejection::TooSmall
        | IngressRejection::BadHeader
        | IngressRejection::SizeMismatch => (
            &counters.decode_errors,
            warn.decode_error_warned.swap(true, Ordering::Relaxed),
        ),
    };
    counter.fetch_add(1, Ordering::Relaxed);
    if already_warned {
        tracing::debug!(
            topic = %topic,
            reason = %rejection,
            "ingress: dropped network frame (repeat — see the first warning for this class)"
        );
    } else {
        tracing::warn!(
            topic = %topic,
            reason = %rejection,
            "ingress: dropped network frame"
        );
    }
}

/// Count + rate-latch a re-injection failure:
/// a frame that PASSED [`validate_ingress_frame`] but could not be published
/// into local iceoryx2. Same latch discipline as [`record_ingress_rejection`]:
/// the first failure of the class logs loudly, sustained failures downgrade
/// to `debug` (a line-rate remote must not flood the log), and every failure
/// bumps `reinject_failure_drops` — never an uncounted drop.
///
/// `error` selects the loud level: `Some(e)` is a `publish_raw` failure
/// (transport pressure — first occurrence logs `warn!` with the error);
/// `None` is a poisoned publisher mutex (a code-bug signal — first occurrence
/// logs `error!`). Never panics on the zenoh
/// thread.
fn record_reinject_failure(
    counters: &IngressCounters,
    warn: &IngressWarnLatch,
    topic: &str,
    error: Option<&TransportError>,
) {
    counters
        .reinject_failure_drops
        .fetch_add(1, Ordering::Relaxed);
    let already_warned = warn.reinject_failure_warned.swap(true, Ordering::Relaxed);
    match (already_warned, error) {
        (true, Some(e)) => tracing::debug!(
            topic = %topic,
            error = %e,
            "ingress: re-injection into local iceoryx2 failed — validated frame dropped \
             (repeat — see the first warning for this class)"
        ),
        (true, None) => tracing::debug!(
            topic = %topic,
            "ingress: local publisher mutex poisoned — validated frame dropped \
             (repeat — see the first error for this class)"
        ),
        (false, Some(e)) => tracing::warn!(
            topic = %topic,
            error = %e,
            "ingress: re-injection into local iceoryx2 failed — validated frame dropped \
             (reinject_failure_drops counts these; repeats log at debug)"
        ),
        (false, None) => tracing::error!(
            topic = %topic,
            "ingress: local publisher mutex poisoned — validated frame dropped \
             (reinject_failure_drops counts these; repeats log at debug)"
        ),
    }
}

/// The outcome of an [`IngressInjector::reinject_raw`]
/// call. A rejected or reinject-failed frame is NOT an `Err` the caller must
/// handle — it is a normal, COUNTED drop (mirrored into [`IngressStats`]);
/// callers may branch on this for their own metrics, or ignore it and read the
/// stats snapshot. Never a panic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReinjectOutcome {
    /// The frame validated and was re-injected into local iceoryx2 SHM,
    /// delivered to `recipients` subscribers. Bumps `IngressStats::frames`.
    Injected {
        /// Number of local subscribers the frame was delivered to.
        recipients: usize,
    },
    /// The frame failed ingress validation (schema/size/decode). Bumps
    /// `IngressStats::schema_mismatch_drops` (for
    /// [`IngressRejection::SchemaMismatch`]) or `decode_errors` (structural).
    Rejected(IngressRejection),
    /// The frame validated but the local `publish_raw` failed (loan-pool
    /// exhaustion, or a poisoned injector mutex). Bumps
    /// `IngressStats::reinject_failure_drops`. Investigate THIS machine.
    ReinjectFailed,
}

/// A **zenoh-free** local-SHM re-injection handle
/// — the factored-out Step 2 of [`TransportManager::register_ingress_topic`],
/// usable on a `network: None` [`TransportManager`].
///
/// It owns the egress-suppressed injection publisher for ONE topic (created by
/// [`TransportManager::create_ingress_publisher`], so the same loop-exclusion +
/// single-writer safety checks apply — see
/// [`TransportManager::create_ingress_injector`]), plus the topic's expected
/// schema hash and the [`IngressStats`] counters. [`Self::reinject_raw`]
/// validates each raw wire frame's `total_size` + `schema_hash`
/// BEFORE `publish_raw`-ing it into local SHM — an invalid frame is refused,
/// COUNTED, and rate-latch-logged, never silently dropped and never a panic.
///
/// Two production consumers share this ONE implementation:
/// - the zenoh ingress bridge ([`NetworkManager::register_ingress`]) wraps an
///   injector in an `Arc` and calls `reinject_raw` from its receive callback —
///   so the full zenoh path stays byte-identical;
/// - the iroh-sourced desk re-inject client + `cerulion_remoted` (the
///   remote plane) hold an injector directly and call `reinject_raw` per frame
///   read off the tunnel — WITHOUT any zenoh session (Steps 1/3 skipped).
///
/// Interior-mutable (the publisher lives behind a `Mutex`) so `reinject_raw`
/// takes `&self` and one injector can be shared across tasks via `Arc`; it is
/// `Send + Sync`.
pub struct IngressInjector {
    /// Canonical topic name, used only for log identification and [`Self::topic`].
    topic: String,
    /// The topic's expected wire `schema_hash`; every frame must match.
    expected_schema_hash: u64,
    /// The egress-suppressed local injection publisher. Behind a `Mutex` for
    /// interior mutability (`publish_raw` needs `&mut`; the zenoh callback is
    /// `Fn`); a poisoned lock maps to [`ReinjectOutcome::ReinjectFailed`].
    publisher: Mutex<CerulionPublisher>,
    /// Shared ingress counters (Principle #3). `Arc` so the zenoh
    /// [`IngressEntry`] can read the SAME counters via [`Self::stats`] while the
    /// callback writes them.
    counters: Arc<IngressCounters>,
    /// First-of-regime warn latch (flood-suppression), shared with the
    /// rate-conscious rejection/re-inject-failure loggers.
    warn: IngressWarnLatch,
}

impl IngressInjector {
    /// Construct from a ready egress-suppressed publisher + the topic's expected
    /// schema hash. `topic` is used only for log lines and [`Self::topic`] — it
    /// should name the same topic `publisher` was created for.
    ///
    /// The public constructor is [`TransportManager::create_ingress_injector`],
    /// which creates `publisher` (with the loop-exclusion + single-writer safety
    /// checks) and calls this.
    pub(crate) fn new(
        topic: String,
        expected_schema_hash: u64,
        publisher: CerulionPublisher,
    ) -> Self {
        Self {
            topic,
            expected_schema_hash,
            publisher: Mutex::new(publisher),
            counters: Arc::new(IngressCounters::default()),
            warn: IngressWarnLatch::default(),
        }
    }

    /// The canonical topic this injector re-injects into.
    pub fn topic(&self) -> &str {
        &self.topic
    }

    /// The wire `schema_hash` every re-injected frame must carry.
    pub fn expected_schema_hash(&self) -> u64 {
        self.expected_schema_hash
    }

    /// Validate `frame` against the topic's expected schema (checks:
    /// length ≥ header, header parses, `total_size == frame.len()`,
    /// `schema_hash == expected`) THEN re-inject it VERBATIM into local iceoryx2
    /// via `publish_raw`. This is the SAME validate → publish → count → warn
    /// logic the zenoh ingress callback runs, so the full zenoh path routes
    /// through here (one implementation).
    ///
    /// Never panics and never silently drops: an invalid frame is refused +
    /// counted ([`ReinjectOutcome::Rejected`]); a validated-but-unpublishable
    /// frame is counted ([`ReinjectOutcome::ReinjectFailed`]); a successful
    /// re-injection wakes the event-driven subscribers CURRENTLY attached,
    /// bumps `frames`, and — if it ends a failure regime — logs the recovery
    /// once. Observe the tallies with [`Self::stats`].
    ///
    /// NO late-joiner history is retained at this seam: the injection
    /// publisher is created with `history_size = 0` (see
    /// `TransportManager::create_ingress_publisher`), so `deliver_history`
    /// delivers ZERO frames. A subscriber that attaches AFTER a re-injection
    /// sees nothing until the NEXT frame is injected. Operational consequence:
    /// for a one-shot or static stream (e.g. a lone `/tf_static` frame), a
    /// subscriber that joins late never sees that frame — attach the subscriber
    /// before injecting, or re-inject on demand.
    pub fn reinject_raw(&self, frame: &[u8]) -> ReinjectOutcome {
        // The netd-ingest latency probe: the clock read happens ONLY when the probe
        // is enabled — one `OnceLock` load otherwise. The span it opens covers
        // validate + lock + SHM publish, i.e. everything this desk does between the
        // network handing the bytes over and the frame being visible to vizd's tap.
        // What it deliberately does NOT cover: the zenoh callback's own
        // `payload().to_bytes()` copy, which happens one frame up the stack.
        let probe_enter = crate::lat_probe::probe_enabled().then(std::time::Instant::now);
        match validate_ingress_frame(self.expected_schema_hash, frame) {
            Ok(header) => match self.publisher.lock() {
                Ok(mut pubr) => match pubr.publish_raw(frame) {
                    Ok(recipients) => {
                        // The SentSample wake is fired INSIDE
                        // `publish_raw` (ingress publishers are armed via
                        // `arm_publish_raw_notify`), so an explicit
                        // `notify_sent_sample()` here would be redundant — folding it in
                        // makes the wake structural for EVERY raw-ingress caller
                        // (a caller such as the DDS-bridge `RawIngressRoute`
                        // cannot forget it). Best-effort either way (data already in the SHM
                        // queue; a notify failure loses only the wake, Principle
                        // #6).
                        //
                        // `publish_raw` itself drains the publisher's own
                        // listener too, so this is not the only drain —
                        // but it is deliberately KEPT and is not a no-op.
                        // `publish_raw` drains BEFORE its notify, and iceoryx2
                        // self-delivers that notify to the notifying
                        // publisher's OWN listener, so without this call every
                        // publish would leave exactly one self-delivered
                        // `SentSample` queued until the NEXT publish's drain. It
                        // also keeps the drain unconditional here, independent
                        // of whether this publisher is
                        // `arm_publish_raw_notify`-armed. Either way the queue
                        // stays bounded at ≤1 event. (The late-joiner history
                        // drain is not the distinguishing reason for this call:
                        // `publish_raw` makes the same call.)
                        pubr.check_subscriber_events();
                        // A fully-successful re-injection ends any failure run:
                        // report the recovery at info EXACTLY once.
                        if self.warn.reset() {
                            tracing::info!(
                                topic = %self.topic,
                                "ingress: recovered — frames validating and re-injecting again"
                            );
                        }
                        self.counters.frames.fetch_add(1, Ordering::Relaxed);
                        // One line per SAMPLED frame. `t_shm_ns`
                        // is the cross-process join stamp vizd's drain line is
                        // subtracted from; `ingest_us` is this desk's own ingress
                        // cost. Keyed on the wire `sequence`, which both daemons
                        // sample identically without coordination.
                        if let Some(t0) = probe_enter {
                            if crate::lat_probe::should_sample(header.sequence) {
                                tracing::info!(
                                    stage = "s1_netd_ingest",
                                    topic = %self.topic,
                                    seq = header.sequence,
                                    bytes = frame.len(),
                                    ingest_us = t0.elapsed().as_micros() as u64,
                                    t_shm_ns = crate::lat_probe::wall_ns() as u64,
                                    "latency probe stage"
                                );
                            }
                        }
                        ReinjectOutcome::Injected { recipients }
                    }
                    Err(e) => {
                        record_reinject_failure(&self.counters, &self.warn, &self.topic, Some(&e));
                        ReinjectOutcome::ReinjectFailed
                    }
                },
                Err(_) => {
                    record_reinject_failure(&self.counters, &self.warn, &self.topic, None);
                    ReinjectOutcome::ReinjectFailed
                }
            },
            Err(rejection) => {
                record_ingress_rejection(&self.counters, &self.warn, &self.topic, rejection);
                ReinjectOutcome::Rejected(rejection)
            }
        }
    }

    /// Snapshot this injector's ingress counters (Principle #3) — the SINGLE
    /// implementation behind both this accessor and
    /// [`NetworkManager::ingress_stats`].
    pub fn stats(&self) -> IngressStats {
        IngressStats {
            frames: self.counters.frames.load(Ordering::Relaxed),
            schema_mismatch_drops: self.counters.schema_mismatch_drops.load(Ordering::Relaxed),
            decode_errors: self.counters.decode_errors.load(Ordering::Relaxed),
            reinject_failure_drops: self.counters.reinject_failure_drops.load(Ordering::Relaxed),
        }
    }

    /// Testing-only: arm the underlying publisher's fire-once `publish_raw`
    /// fault-injection seam (mirrors [`CerulionPublisher::fault_inject_publish_raw_after`])
    /// so the `network: None` seam's `ReinjectFailed` path can be exercised
    /// without real iceoryx2 pool exhaustion.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn fault_inject_publish_raw_after(&self, n: u32) {
        if let Ok(mut pubr) = self.publisher.lock() {
            pubr.fault_inject_publish_raw_after(n);
        }
    }
}

/// One active network→local ingress bridge. Holds the zenoh
/// subscriber (dropping it undeclares the subscription and stops the
/// callback), the liveliness token (declaring it flips the REMOTE egress
/// publisher's bridge flag on — [`crate::transport::discovery::TopicToken`]),
/// and the shared [`IngressInjector`] the callback re-injects through and
/// [`NetworkManager::ingress_stats`] reads counters from. The injector (and its
/// re-injection publisher) lives INSIDE the subscriber's callback closure via
/// an `Arc` clone (kept alive by `_sub`); this entry holds the other `Arc`.
struct IngressEntry {
    /// Zenoh subscriber handle — its Drop undeclares the subscription.
    _sub: zenoh::pubsub::Subscriber<()>,
    /// Liveliness DEMAND token advertising a live subscriber for this topic to
    /// remote egress publishers (its Drop undeclares the token → remote bridge
    /// off on a healthy link). `None` only under the test seam
    /// [`NetworkManager::set_suppress_ingress_token_for_test`] (proving the
    /// queryable-inversion GET path alone drives egress). Production always
    /// declares it — it is the belt-and-suspenders healthy-link demand path
    /// alongside the demand GET.
    _token: Option<TopicToken>,
    /// Shared with the receive callback; its [`IngressInjector::stats`] backs
    /// [`NetworkManager::ingress_stats`].
    injector: Arc<IngressInjector>,
}

/// Manages the zenoh session and background network task lifecycle.
///
/// The session is lazily created on first call to [`session()`](Self::session).
/// The network task is lazily started when the first network subscriber is
/// registered via [`add_network_subscriber()`](Self::add_network_subscriber)
/// and stopped when the last one is removed.
pub struct NetworkManager {
    config: NetworkConfig,
    session: OnceCell<zenoh::Session>,
    task: Mutex<TaskState>,
    /// Canonical-topic-keyed registry of active network→local ingress
    /// bridges. Each entry owns a zenoh subscriber whose callback validates and
    /// re-injects received wire frames into that topic's local iceoryx2
    /// publisher. Registered via [`Self::register_ingress`]; snapshotted via
    /// [`Self::ingress_stats`].
    ingress: Mutex<HashMap<String, IngressEntry>>,
    /// The canonical topics THIS manager network-INGRESSES (re-injects
    /// from the network into local SHM). Populated at the TOP of
    /// [`Self::register_ingress`] — BEFORE the demand token is declared — so the
    /// liveliness watch (which hears this manager's OWN demand token) can tell
    /// its own ingress topic from a produced-and-egressable one. An ingress topic
    /// is NEVER an egress candidate: enabling its bridge would be the
    /// re-inject → tap → egress → re-inject echo loop, and on an ingress-only
    /// desk (deny-all posture) `enable_bridge` would REFUSE + WARN about the
    /// machine's OWN demand — wrong-audience noise for the `topic hz`/`echo`
    /// user. Shared with the watch thread (an `Arc` clone),
    /// and DISTINCT from the late-populated `ingress` map above (populated only at
    /// the END of `register_ingress`, which the [`PermissiveShmProbe`]'s
    /// `is_ingress_registered` check reads at a different, TOCTOU-guarded seam):
    /// this set is the watch's RACE-FREE exclusion, populated before ANY zenoh
    /// declaration so the watch's Put arm never mis-fires the refusal.
    self_ingress: Arc<Mutex<HashSet<String>>>,
    /// Canonical-topic-keyed registry of retained
    /// EGRESS-ANNOUNCE liveliness tokens
    /// (`cerulion_ann/{robot}{canonical topic}` — discovery presence, distinct
    /// from the ingress DEMAND key-space; the robot chunk is
    /// `config.robot_identity`). One token per produced topic makes it visible
    /// to `topic list`. Registered via [`Self::announce_egress_topic`]
    /// (idempotent — a re-announce is a no-op, never a duplicate token);
    /// observable via [`Self::announced_topics`]. Dropping a token undeclares
    /// it (the topic goes off the network listing).
    announced: Mutex<HashMap<String, TopicToken>>,
    /// The retained BARE identity announce token
    /// (`cerulion_ann/{robot}` — robot presence with no topic suffix). At most
    /// ONE per gateway process; declared via [`Self::announce_gateway_identity`]
    /// (idempotent); observable via [`Self::has_identity_token`]. It keeps an
    /// ingress-only / zero-egress gateway visible in `topic list` ROBOTS even
    /// where mDNS cannot reach. Dropping it undeclares the presence.
    identity_token: Mutex<Option<TopicToken>>,
    /// The background DEMAND-RECONCILER + expiry thread's lifecycle
    /// state (the query-shaped belt to the subscriber watch; it also runs the
    /// queryable-inversion GET-expiry sweep). Started by
    /// [`Self::start_demand_reconciler`] (once, from
    /// [`crate::transport::gateway::GatewayRuntime::new`]); stopped by
    /// `stop_bg_thread` in [`Drop`].
    reconciler: Mutex<BgThreadState>,
    /// Queryable inversion: the DEMANDER-side background DEMAND-GET loop
    /// thread's lifecycle state — re-issues demand GETs every
    /// `DEMAND_GET_INTERVAL` for every registered ingress topic. Started by
    /// [`Self::start_demand_get_loop`] (once, from the gateway); stopped by
    /// `stop_bg_thread` in [`Drop`].
    demand_get: Mutex<BgThreadState>,
    /// The unified query surface: the PRODUCER-side retained verb-dispatched
    /// QUERYABLE (`cerulion_q/{robot}/**`) — serves both the `demand` verb
    /// and the `catalog` verb. At most one per gateway; declared by
    /// [`Self::start_query_surface`]. Its Drop undeclares the queryable.
    query_surface: Mutex<Option<zenoh::query::Queryable<()>>>,
    /// The schema-serving maps the LATCHED query surface's callback
    /// actually captured — recorded in the SAME critical section that commits
    /// the queryable, so they can never name a different construction's maps.
    /// The surface outlives the gateway construction that started it (a
    /// partial boot can start the surface and then fail; the retry's
    /// `start_query_surface` no-ops), so a caller that wants to MERGE serving
    /// into what the surface reads must take THESE handles
    /// ([`Self::query_surface_serving_handles`]), never the handles of whichever
    /// gateway object later succeeded.
    query_surface_serving: Mutex<Option<super::gateway::SchemaServingHandles>>,
    /// TEST SEAM: when `true`, the two LIVE demand-driven flag-flip paths
    /// on the PRODUCER — the liveliness SUBSCRIBER ([`Self::start_task`]) AND the
    /// demand QUERYABLE (`handle_demand_verb`) — still DRAIN / respond but SKIP
    /// flipping any egress flag, so the periodic RECONCILER is the sole flipper.
    /// Deterministically simulates the strict-link failure mode where live
    /// subscriber-interest propagation is broken while bounded liveliness QUERIES
    /// still work. Always present (an `Arc` so it threads into the `move`
    /// closures); in production it stays `false` — the only writer,
    /// [`Self::set_suppress_live_demand_for_test`], is test-gated. Reading it
    /// costs one relaxed load on the cold demand paths, so production behavior is
    /// byte-identical.
    suppress_live_demand: Arc<AtomicBool>,
    /// TEST SEAM: when `true`, [`Self::register_ingress`] SKIPS declaring
    /// its liveliness DEMAND token — so the producer's subscriber sees no `Put`
    /// and its reconciler gathers empty, isolating the queryable-inversion GET
    /// path as the sole enabler in the decisive e2e. Always present; the only
    /// writer, [`Self::set_suppress_ingress_token_for_test`], is test-gated.
    /// Reading it costs one relaxed load on the cold ingress-registration path.
    suppress_ingress_token: Arc<AtomicBool>,
    /// TEST SEAM (fire-once): when armed via
    /// [`Self::fault_inject_announce_egress_once`] (test-gated), the NEXT
    /// [`Self::announce_egress_topic`] call returns `TransportError::Publish`
    /// (the exact error a real liveliness-declare failure surfaces) instead of
    /// declaring the token, then disarms itself. Mirrors the
    /// `fault_inject_publish_raw_after` precedent (publisher.rs): an UNCONDITIONAL
    /// field (never cfg-gated — per the FFI struct-layout rule), read on the cold
    /// announce path, always `false` in production (the only writer is the
    /// test-gated setter). Lets the gateway's `register_runtime_topic`
    /// announce-failure path be pinned WITHOUT a broken zenoh session.
    fail_next_announce_egress: AtomicBool,
    /// The AllowAll-only permissive SHM probe (see [`PermissiveShmProbe`]),
    /// set ONCE at gateway boot via [`Self::set_permissive_probe`] BEFORE the
    /// liveliness watch / demand queryable start, and read by BOTH demand paths
    /// (the liveliness watch loop in [`Self::start_task`] + the queryable callback
    /// [`handle_demand_verb`]) to serve a live unregistered runtime topic on
    /// demand. `None` on a manager that is not a gateway (no probe installed) —
    /// both paths then serve registered topics only.
    permissive_probe: Mutex<Option<Arc<PermissiveShmProbe>>>,
    /// The DYNAMIC demand set for a PURE-INGRESS demander's self-heal
    /// keepalive loop (`cerulion-netd`). A gateway builds its OWN
    /// [`DemandGetState`] from its fixed boot ingress list and passes it to
    /// [`Self::start_demand_get_loop`]; a pure-ingress demander instead calls
    /// [`Self::ensure_ingress_demand_keepalive`] per demanded topic, which grows
    /// THIS shared set and start-once-drives the SAME loop over it. Both share the
    /// manager's single `demand_get` thread slot (start-once, idempotent), and a
    /// manager is only ever ONE role — a gateway (fixed set) or a pure-ingress
    /// demander (this dynamic set) — so they never contend. The self-heal is the
    /// dialer→accepter demand GET (the wire direction that survives a
    /// strict connect-only link AND re-crosses after a restarted robot gateway
    /// reconnects), which the gateway-only wiring never started for the
    /// pure-ingress netd path (the silent-black-hole root cause).
    ingress_demand_state: Arc<DemandGetState>,
    /// The ONE demand-authorization seam (`is_allowed`-shaped) the LAN
    /// (zenoh) demand-grant consults before serving a topic — the access decision's
    /// gate on the LAN plane. A LIVE slot (an `Arc<Mutex<..>>` so
    /// the boot-captured demand callbacks re-read it per demand): the DEFAULT is the
    /// deny-nothing [`AllowAllAuthorizer`] (byte-identical to an un-gated plane), swappable at any
    /// time via [`Self::set_demand_authorizer`] (an account-grant authorizer installs
    /// here; a stub deny-authorizer drives the refusal tests). Reading it costs
    /// one relaxed lock+clone on the COLD demand path, so a healthy grant pays nothing
    /// new; production behavior with the default is unchanged.
    demand_authorizer: Arc<Mutex<Arc<dyn DemandAuthorizer>>>,
}

impl NetworkManager {
    /// Create a new network manager with the given configuration.
    ///
    /// Does NOT open the zenoh session or start the network task — both
    /// happen lazily on demand.
    pub fn new(config: NetworkConfig) -> Self {
        Self {
            config,
            session: OnceCell::new(),
            task: Mutex::new(TaskState {
                handle: None,
                shutdown_tx: None,
                subscriber_count: 0,
            }),
            ingress: Mutex::new(HashMap::new()),
            self_ingress: Arc::new(Mutex::new(HashSet::new())),
            announced: Mutex::new(HashMap::new()),
            identity_token: Mutex::new(None),
            reconciler: Mutex::new(BgThreadState::default()),
            demand_get: Mutex::new(BgThreadState::default()),
            query_surface: Mutex::new(None),
            query_surface_serving: Mutex::new(None),
            suppress_live_demand: Arc::new(AtomicBool::new(false)),
            suppress_ingress_token: Arc::new(AtomicBool::new(false)),
            fail_next_announce_egress: AtomicBool::new(false),
            permissive_probe: Mutex::new(None),
            ingress_demand_state: Arc::new(DemandGetState::default()),
            demand_authorizer: Arc::new(Mutex::new(Arc::new(AllowAllAuthorizer))),
        }
    }

    /// Install the demand-authorization gate consulted by the LAN
    /// (zenoh) demand-grant. Swaps the live slot the demand callbacks re-read, so it
    /// takes effect immediately regardless of whether the demand paths have started —
    /// an account-grant authorizer installs here; the default is the
    /// deny-nothing [`AllowAllAuthorizer`]. Poison-recovering (the demand path must not
    /// wedge on a poisoned lock).
    pub fn set_demand_authorizer(&self, authorizer: Arc<dyn DemandAuthorizer>) {
        let mut guard = self
            .demand_authorizer
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        *guard = authorizer;
    }

    /// TEST SEAM: the current authorizer, read the SAME way the demand
    /// callbacks read it (lock + clone the live slot) — so the gateway test seams
    /// consult exactly what a [`Self::set_demand_authorizer`] installed.
    /// `#[cfg(test/test-helpers)]`.
    #[cfg(any(test, feature = "test-helpers"))]
    pub(crate) fn demand_authorizer_for_test(&self) -> Arc<dyn DemandAuthorizer> {
        self.demand_authorizer
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .clone()
    }

    /// Install the AllowAll-only permissive SHM probe (see
    /// [`PermissiveShmProbe`]). The gateway calls this ONCE at boot BEFORE
    /// starting the liveliness watch + demand queryable, so both demand paths
    /// capture the probe. Set-once (a repeat overwrites — one gateway per manager).
    pub(crate) fn set_permissive_probe(&self, probe: Arc<PermissiveShmProbe>) {
        let mut guard = self
            .permissive_probe
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        *guard = Some(probe);
    }

    /// The installed permissive SHM probe, if any — an OWNED clone so it
    /// threads into the demand-path closures. `None` on a non-gateway manager.
    fn permissive_probe(&self) -> Option<Arc<PermissiveShmProbe>> {
        self.permissive_probe
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .clone()
    }

    /// Whether the PRODUCER query surface (the verb-dispatched queryable)
    /// is currently declared (Principle #3: observable state). `true` after
    /// [`Self::start_query_surface`] retains it. The gateway's
    /// `query_surface_active` reads this — the DETERMINISTIC observable that the
    /// empty-boot-plan gateway (Part B) started its demand/catalog surface (an
    /// empty deny-all/ingress-only gateway leaves it `false`).
    pub fn has_query_surface(&self) -> bool {
        self.query_surface
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .is_some()
    }

    /// Whether `topic` (canonical-keyed) has a registered network→local
    /// INGRESS bridge. The permissive SHM probe reads this for loop safety — a
    /// topic this gateway ingresses has a local SHM service that is the gateway's
    /// OWN re-injection, so serving it back out would echo-loop. Lean
    /// predicate (no stats clone); poison-recovering like [`Self::ingress_stats`].
    pub fn is_ingress_registered(&self, topic: &str) -> bool {
        let canonical = canonical_topic(topic).into_owned();
        self.ingress
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .contains_key(&canonical)
    }

    /// Whether `topic` (canonical-keyed) is in the race-free
    /// SELF-INGRESS exclusion set — the topics THIS manager network-ingresses and
    /// which the liveliness watch must therefore NEVER treat as egress candidates
    /// (the echo-loop / wrong-audience refusal). The public sibling of
    /// [`Self::is_ingress_registered`] over the `self_ingress` set (a DISTINCT set,
    /// populated at the TOP of `register_ingress` before any zenoh declaration and
    /// CLEARED on [`Self::unregister_ingress`]). Exposed
    /// (Principle #3) so a released topic's exit from the exclusion set is directly
    /// observable. Poison-recovering like [`Self::is_ingress_registered`].
    pub fn is_self_ingress(&self, topic: &str) -> bool {
        let canonical = canonical_topic(topic).into_owned();
        self.self_ingress
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .contains(&canonical)
    }

    /// TEST SEAM: the race-free self-ingress exclusion set (the topics
    /// this manager network-ingresses), for the gateway's
    /// `demand_query_grants_for_test` to feed the queryable grant decision — so a
    /// self-ingress demand GET is proven refused WITHOUT the wrong-audience
    /// `enable_bridge` warn. Production reads `self.self_ingress` directly.
    #[cfg(any(test, feature = "test-helpers"))]
    pub(crate) fn self_ingress_set_for_test(&self) -> &Arc<Mutex<HashSet<String>>> {
        &self.self_ingress
    }

    /// Get or lazily create the zenoh session.
    ///
    /// The first call opens the session (blocking). Subsequent calls return
    /// the same session. Thread-safe: concurrent callers will block until
    /// the first caller finishes initialization.
    pub fn session(&self) -> TransportResult<&zenoh::Session> {
        self.session.get_or_try_init(|| {
            let config = self.build_zenoh_config()?;
            let session =
                zenoh::open(config)
                    .wait()
                    .map_err(|e| TransportError::SessionCreation {
                        reason: format!("{e}"),
                    })?;
            // Operational plumbing, not an INFO lifecycle event: a plain
            // `topic list` opens/closes several ephemeral discovery + verify
            // sessions, so this rides at DEBUG (surfaced under `--verbose`) to
            // keep the `cerulion=info` default quiet. `graph run`'s genuine
            // lifecycle INFOs are unaffected.
            tracing::debug!("zenoh session opened");
            Ok(session)
        })
    }

    /// Check if the zenoh session has been created.
    pub fn is_active(&self) -> bool {
        self.session.get().is_some()
    }

    /// Orderly-close the zenoh session if one was opened.
    ///
    /// The SHORT-LIVED discovery sessions (`topic list`'s remote-topics gather
    /// and its verify-before-trust probes) drop their `NetworkManager` moments
    /// after a liveliness query — ORDERLY RESOURCE RELEASE: it tears down the
    /// session's background tasks + sockets deterministically at end of use
    /// rather than at an arbitrary `Drop` point.
    ///
    /// NOTE: this does NOT by itself silence the cosmetic `ERROR ... Unable to
    /// publish transport event: session closed` that `zenoh::api::admin` races
    /// out during teardown (verified: it still fires WITH an explicit close —
    /// it is emitted from inside zenoh's own close path). That line is
    /// suppressed at the log layer instead (the `zenoh::api::admin=off`
    /// directive in `crate::init_logging`).
    ///
    /// SCOPED TO EPHEMERAL DISCOVERY SESSIONS: the gateway's long-lived session
    /// must stay open for the whole run, so the gateway NEVER calls this — only
    /// the discovery callers do. A manager whose session was never opened is a
    /// no-op.
    pub fn close(&self) {
        if let Some(session) = self.session.get() {
            tracing::debug!("closing ephemeral discovery zenoh session (orderly teardown)");
            if let Err(e) = session.close().wait() {
                // A close error on teardown is not actionable — the session is
                // going away regardless; record it at debug and move on.
                tracing::debug!(error = %e, "zenoh session close returned an error (ignored)");
            }
        }
    }

    /// Check if the background network task is running.
    pub fn is_task_running(&self) -> bool {
        self.task.lock().unwrap().handle.is_some()
    }

    /// Number of registered network subscribers.
    pub fn network_subscriber_count(&self) -> usize {
        self.task.lock().unwrap().subscriber_count
    }

    /// Register a network subscriber. Starts the network task if this is the first.
    ///
    /// The zenoh session is lazily created if not already open.
    /// The bridge manager is passed to the network task so liveliness events
    /// can drive automatic bridge enable/disable.
    pub fn add_network_subscriber(
        &self,
        bridge_mgr: Arc<TopicBridgeManager>,
    ) -> TransportResult<()> {
        // Ensure session exists before starting the task
        let session = self.session()?;

        let mut state = self.task.lock().unwrap();
        state.subscriber_count += 1;
        if state.subscriber_count == 1 && state.handle.is_none() {
            Self::start_task(
                &mut state,
                session.clone(),
                bridge_mgr,
                Arc::clone(&self.suppress_live_demand),
                // The permissive SHM probe (set at gateway boot BEFORE
                // this runs); the liveliness watch loop serves a live unregistered
                // topic through it. `None` on a non-gateway manager.
                self.permissive_probe(),
                // The self-ingress exclusion set — the watch skips
                // enable_bridge (and its refusal WARN) for a topic THIS manager
                // re-injects from the network (our own demand token echo).
                Arc::clone(&self.self_ingress),
                // The demand-authorization slot the Put arm consults.
                Arc::clone(&self.demand_authorizer),
            )?;
        }
        Ok(())
    }

    /// Unregister a network subscriber. Stops the network task if this was the last.
    pub fn remove_network_subscriber(&self) {
        let mut state = self.task.lock().unwrap();
        state.subscriber_count = state.subscriber_count.saturating_sub(1);
        if state.subscriber_count == 0 {
            Self::stop_task(&mut state);
        }
    }

    /// Access the network configuration.
    pub fn config(&self) -> &NetworkConfig {
        &self.config
    }

    /// Publish a raw Cerulion wire frame to the network via zenoh.
    ///
    /// The zenoh payload IS the wire frame VERBATIM — the 32-byte
    /// [`WireHeader`] (explicitly little-endian) already carries
    /// `schema_hash` / `sequence` / `timestamp_ns`, and the topic is the zenoh
    /// key expression (`cerulion{canonical topic}`, e.g. `/p/n/o` →
    /// `cerulion/p/n/o`). There is NO prost envelope; taking
    /// `frame` BY VALUE keeps the `Vec<u8>`→`ZBytes` hand-off zero-copy (zenoh
    /// 1.x moves an owned `Vec` into `ZBytes` without copying).
    ///
    /// This is called by publishers bridging local messages to remote
    /// subscribers, and by tests exercising the ingress path.
    ///
    /// # Errors
    ///
    /// - `TransportError::Publish` if the frame is smaller than the 32-byte
    ///   wire header (cannot be a valid Cerulion frame) or the zenoh put fails
    /// - `TransportError::SessionCreation` if the session cannot be opened
    pub fn publish_to_network(&self, topic: &str, frame: Vec<u8>) -> TransportResult<()> {
        // Minimal outbound validation: a frame smaller than the wire header
        // cannot be a valid Cerulion frame. Matches the size shape the deleted
        // prost `from_wire_frame` enforced.
        if frame.len() < WireHeader::SIZE {
            return Err(TransportError::Publish {
                topic: topic.to_string(),
                reason: format!(
                    "wire data too small for header: {} bytes, need {}",
                    frame.len(),
                    WireHeader::SIZE
                ),
            });
        }
        let session = self.session()?;
        let key = Self::data_key(topic);
        let size_bytes = frame.len();

        session
            .put(&key, frame)
            .wait()
            .map_err(|e| TransportError::Publish {
                topic: topic.to_string(),
                reason: format!("zenoh put failed: {e}"),
            })?;

        tracing::trace!(topic = %topic, size_bytes, "published raw wire frame to network");
        Ok(())
    }

    /// Register a network→local INGRESS bridge for `topic`.
    ///
    /// Declares a persistent zenoh subscriber on `data_key(topic)` whose
    /// callback validates every received raw wire frame against
    /// `expected_schema_hash` and (on success) re-injects it VERBATIM into
    /// `local_publisher` (preserving the wire `sequence` + `timestamp_ns`
    /// byte-identically — Principle #7). Also declares a liveliness token so a
    /// remote EGRESS publisher's bridge flag flips on (a live subscriber now
    /// exists for this topic — see [`crate::transport::discovery::TopicToken`]).
    ///
    /// The `local_publisher` should be minted via
    /// [`crate::transport::TransportManager::create_ingress_publisher`], which
    /// keeps it OUT of the egress bridge map (the structural loop exclusion —
    /// an ingress publisher must never itself bridge back out).
    ///
    /// # Errors
    ///
    /// - `TransportError::Internal` for an empty topic name, a poisoned
    ///   registry mutex, or a double registration (a topic already has an
    ///   ingress bridge — refused rather than leaving two silent subscribers)
    /// - `TransportError::Receive` if the zenoh subscriber cannot be declared
    /// - `TransportError::SessionCreation` if the session cannot be opened
    pub fn register_ingress(
        &self,
        topic: &str,
        expected_schema_hash: u64,
        local_publisher: CerulionPublisher,
    ) -> TransportResult<()> {
        // Empty name would canonicalize to "/" — a bridge keyed by a malformed
        // name no data key can ever match (crib the bridge-registration guard).
        if topic.is_empty() {
            return Err(TransportError::Internal {
                reason: "ingress registration requires a non-empty topic name".to_string(),
            });
        }
        let canonical = canonical_topic(topic).into_owned();

        // Record this topic as one we INGRESS *before* declaring the
        // demand token below — so the liveliness watch (which hears our OWN
        // demand token) recognizes it as our re-injection and NEVER runs the
        // egress `enable_bridge` refusal on it (the wrong-audience WARN a
        // `topic hz`/`echo` desk would otherwise trigger). Race-free by
        // ordering: the insert is synchronous and precedes ANY zenoh
        // declaration, so the async Put the watch later hears always finds it
        // here. A stray entry on a later-failed registration is harmless (it can
        // only SUPPRESS an egress warn for a topic this machine does not
        // produce). Idempotent (HashSet) for a double-registration.
        self.self_ingress
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .insert(canonical.clone());

        // Reject a double-registration BEFORE opening the zenoh subscriber, so
        // a second call is a pure no-op on the transport (no half-registered
        // state — Principle #6).
        {
            let ingress = self.ingress.lock().map_err(|_| TransportError::Internal {
                reason: "ingress registry mutex poisoned".to_string(),
            })?;
            if ingress.contains_key(&canonical) {
                return Err(TransportError::Internal {
                    reason: format!(
                        "topic '{canonical}' already has a registered network ingress bridge"
                    ),
                });
            }
        }

        let session = self.session()?;
        let data_key = Self::data_key(&canonical);

        // The validate → publish_raw → count → warn logic lives in
        // the shared `IngressInjector` (one implementation for the zenoh path
        // AND the zenoh-free desk / `cerulion_remoted` re-inject seam). The
        // injector owns the injection publisher (behind its own Mutex); an `Arc`
        // clone lives inside the callback (kept alive by the subscriber handle),
        // and the entry keeps the other `Arc` for `ingress_stats`.
        let injector = Arc::new(IngressInjector::new(
            canonical.clone(),
            expected_schema_hash,
            local_publisher,
        ));
        let cb_injector = Arc::clone(&injector);

        let subscriber = session
            .declare_subscriber(&data_key)
            .callback(move |sample| {
                // Re-inject VERBATIM into local iceoryx2 (Principle #7). The
                // return value is observed via `ingress_stats`; a rejected /
                // reinject-failed frame is counted + rate-latch-logged inside
                // `reinject_raw`, never a panic on the zenoh thread.
                let bytes = sample.payload().to_bytes();
                cb_injector.reinject_raw(&bytes);
            })
            .wait()
            .map_err(|e| TransportError::Receive {
                topic: canonical.clone(),
                reason: format!("zenoh ingress subscriber creation failed: {e}"),
            })?;

        // Declare the liveliness DEMAND token AFTER the data subscriber (ordering
        // is load-bearing): the token advertises "a live subscriber exists for
        // this topic" to remote egress publishers (discovery.rs), and zenoh
        // propagates declarations in order per session — so by the time a
        // remote's bridge flag flips on, this side's data-subscriber interest
        // has already propagated and the remote's FIRST bridged frame routes
        // here (no first-frame loss window). Dropped with the entry → remote
        // bridge disables.
        //
        // Queryable inversion: this token is now the healthy-link
        // BELT alongside the demand GET (the gateway's [`Self::start_demand_get_loop`]
        // pulls the same egress ON via the producer's queryable — the direction
        // that survives a strict connect-only link where this DIALER declaration
        // never forwards). It is SUPPRESSED only under the test seam
        // (`suppress_ingress_token`), which isolates the GET path in the decisive
        // e2e; production always declares it.
        let token = if self.suppress_ingress_token.load(Ordering::Relaxed) {
            tracing::debug!(
                topic = %canonical,
                "test seam: ingress demand token declaration suppressed"
            );
            None
        } else {
            Some(TopicToken::declare(session, &canonical)?)
        };

        let mut ingress = self.ingress.lock().map_err(|_| TransportError::Internal {
            reason: "ingress registry mutex poisoned".to_string(),
        })?;
        // Re-check under the lock (TOCTOU): if another thread registered the
        // same topic between the pre-check and here, refuse — the just-created
        // `subscriber` + `token` drop on this early return (undeclaring both),
        // so no orphaned zenoh state survives.
        if ingress.contains_key(&canonical) {
            return Err(TransportError::Internal {
                reason: format!(
                    "topic '{canonical}' already has a registered network ingress bridge"
                ),
            });
        }
        ingress.insert(
            canonical,
            IngressEntry {
                _sub: subscriber,
                _token: token,
                injector,
            },
        );
        Ok(())
    }

    /// Tear down the network→local INGRESS bridge for
    /// `topic` — the inverse of [`Self::register_ingress`].
    ///
    /// Removes the topic's `IngressEntry` from the registry and drops it, which:
    ///
    /// - **undeclares the zenoh data subscriber** (`_sub` Drop) — no more remote
    ///   frames re-inject for this topic;
    /// - **undeclares the liveliness DEMAND token** (`_token` Drop) — a remote
    ///   egress publisher sees this side's subscriber-interest go away and (on a
    ///   healthy link) disables its bridge, so the frame stops crossing the network;
    /// - **releases the egress-suppressed local injection publisher** (the
    ///   `IngressInjector`'s `CerulionPublisher`, shared into the receive callback,
    ///   drops with the entry) — freeing the mirror topic's iceoryx2 publisher slot
    ///   so a later re-register can cleanly re-create the mirror.
    ///
    /// It ALSO removes the topic from the `self_ingress` exclusion
    /// set: the topic is no longer network-ingressed, so it is once again a
    /// legitimate egress candidate (a produced topic of the same name would
    /// otherwise be wrongly skipped by the liveliness watch's egress path).
    ///
    /// # NOT reversed here (the asymmetry vs `register_ingress_topic`)
    ///
    /// The manager's single liveliness WATCH task
    /// ([`crate::transport::TransportManager::start_network_bridge_watch`], started
    /// once as step 1 of `register_ingress_topic`) is deliberately **NOT** stopped:
    /// it is a latched, per-MANAGER task SHARED by every ingress topic AND the
    /// egress side (it flips remote bridge flags for produced topics too), so
    /// tearing it down on a single-topic unregister would break the machine's other
    /// bridges. The watch is stopped exactly once, by `NetworkManager::Drop`.
    ///
    /// The entry is `remove`d under the registry lock but DROPPED outside it (a
    /// zenoh undeclare can block briefly on the network thread — holding the lock
    /// across it would stall a concurrent `register_ingress` / `ingress_stats`).
    ///
    /// # Errors
    ///
    /// - [`TransportError::Internal`] for a poisoned registry mutex, or — LOUD, not
    ///   silent — if `topic` has NO registered ingress bridge (never registered, or
    ///   already unregistered): the caller asked to release something that does not
    ///   exist.
    #[must_use = "ingress unregistration result must be checked"]
    pub fn unregister_ingress(&self, topic: &str) -> TransportResult<()> {
        let canonical = canonical_topic(topic).into_owned();

        // Remove under the lock; drop the heavy entry (undeclare subscriber +
        // demand token, release the injection publisher) OUTSIDE the lock so a
        // blocking zenoh undeclare never stalls a concurrent register / stats call.
        let removed = {
            let mut ingress = self.ingress.lock().map_err(|_| TransportError::Internal {
                reason: "ingress registry mutex poisoned".to_string(),
            })?;
            ingress.remove(&canonical)
        };

        let Some(entry) = removed else {
            return Err(TransportError::Internal {
                reason: format!(
                    "topic '{canonical}' has no registered network ingress bridge to unregister \
                     — it was never registered, or it was already unregistered"
                ),
            });
        };

        // Drop OUTSIDE the ingress lock: undeclares the zenoh subscriber + demand
        // token and releases the local injection publisher (freeing the mirror
        // topic's single iceoryx2 publisher slot for a clean re-register).
        drop(entry);

        // The topic is no longer ingressed, so remove it from the
        // self-ingress exclusion set — a produced topic of this name is once again a
        // legitimate egress candidate (leaving a stale entry would wrongly suppress
        // its egress). Idempotent (HashSet::remove).
        self.self_ingress
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .remove(&canonical);

        tracing::info!(
            topic = %canonical,
            "network ingress bridge unregistered — subscriber + demand token undeclared, \
             injection publisher released"
        );
        Ok(())
    }

    /// Snapshot the ingress counters for a bridged topic (Principle
    /// #3: observable state). Returns `None` if `topic` has no registered
    /// ingress bridge. Poison-recovering: a panic while some thread held the
    /// registry lock must not turn every stats read into `None`.
    pub fn ingress_stats(&self, topic: &str) -> Option<IngressStats> {
        let canonical = canonical_topic(topic).into_owned();
        let ingress = self
            .ingress
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let entry = ingress.get(&canonical)?;
        // The counters live in the shared `IngressInjector` now — the
        // ONE `stats()` implementation backs both this and the seam's accessor.
        Some(entry.injector.stats())
    }

    /// This manager's configured robot identity, or a LOUD `Err`
    /// naming the fix — the shared gate for every announce path (a gateway
    /// announcing without an identity would mint unparseable keys, so it is
    /// refused up front, never silently degraded).
    fn require_robot_identity(&self) -> TransportResult<&str> {
        match self.config.robot_identity.as_deref() {
            Some(robot) => Ok(robot),
            None => Err(TransportError::InvalidTransportConfig {
                reason: "announcing on the network requires a robot identity, but this \
                         NetworkConfig has none — set `NetworkConfig.robot_identity` \
                         (both the gateway and `GraphConfig::network_transport_config` \
                         stamp it from the resolved hostname or the \
                         `CERULION_ROBOT_IDENTITY` override, never the graph prefix)"
                    .to_string(),
            }),
        }
    }

    /// ANNOUNCE `topic` on the network — declare + RETAIN a
    /// `cerulion_ann/{robot}{canonical topic}` liveliness token (the robot
    /// chunk is `config.robot_identity`) so a remote `topic list` query can
    /// list this machine's produced topic AND attribute it to this robot.
    ///
    /// IDEMPOTENT per canonical topic: a second call for the same topic is a
    /// no-op `Ok(())` (no duplicate token). The token is retained for the
    /// manager's lifetime and undeclared on drop (see the `announced` registry).
    /// Opens the zenoh session lazily (Principle #8) but does NOT start the
    /// network task — an announce is pure discovery presence, not a subscriber.
    ///
    /// # Errors
    ///
    /// - [`TransportError::Internal`] for an empty topic name or a poisoned
    ///   registry mutex.
    /// - [`TransportError::InvalidTransportConfig`] when `config.robot_identity`
    ///   is unset (the loud no-identity refusal — never a silent bad key).
    /// - [`TransportError::Publish`] for an invalid robot chunk (empty / `/`)
    ///   or if the liveliness token cannot be declared.
    /// - [`TransportError::SessionCreation`] if the zenoh session cannot open.
    pub fn announce_egress_topic(&self, topic: &str) -> TransportResult<()> {
        // Empty name would canonicalize to "/" — a token no query can match
        // (crib the ingress-registration guard).
        if topic.is_empty() {
            return Err(TransportError::Internal {
                reason: "egress announce requires a non-empty topic name".to_string(),
            });
        }
        // TEST SEAM (fire-once): when armed via
        // `fault_inject_announce_egress_once`, the NEXT announce fails — simulating
        // a broken zenoh session / liveliness declare so the gateway
        // runtime-topic announce-failure path is pinnable. Read-then-disarm
        // (mirrors the `fault_inject_publish_raw_after` shape): one relaxed load
        // on the cold announce path, always `false` in production.
        if self.fail_next_announce_egress.load(Ordering::Relaxed) {
            self.fail_next_announce_egress
                .store(false, Ordering::Relaxed);
            return Err(TransportError::Publish {
                topic: canonical_topic(topic).into_owned(),
                reason: "test seam: injected announce_egress_topic failure \
                         (fault_inject_announce_egress_once)"
                    .to_string(),
            });
        }
        let robot = self.require_robot_identity()?.to_string();
        let canonical = canonical_topic(topic).into_owned();

        // Idempotent pre-check BEFORE opening the session / declaring a token —
        // a re-announce touches nothing on the transport.
        {
            let announced = self
                .announced
                .lock()
                .map_err(|_| TransportError::Internal {
                    reason: "announce registry mutex poisoned".to_string(),
                })?;
            if announced.contains_key(&canonical) {
                return Ok(());
            }
        }

        let session = self.session()?;
        let token = TopicToken::announce(session, &robot, &canonical)?;

        let mut announced = self
            .announced
            .lock()
            .map_err(|_| TransportError::Internal {
                reason: "announce registry mutex poisoned".to_string(),
            })?;
        // Re-check under the lock (TOCTOU): if another thread announced the
        // same topic in the gap, drop the just-declared duplicate token here.
        announced.entry(canonical).or_insert(token);
        Ok(())
    }

    /// Declare + RETAIN the BARE identity announce token
    /// (`cerulion_ann/{robot}` — robot presence, no topic suffix) so this
    /// gateway surfaces a `topic list` ROBOTS row even with ZERO egress topics
    /// (an ingress-only robot) and on mDNS-unreachable networks. Called once at
    /// gateway network boot ([`crate::transport::gateway::GatewayRuntime`]).
    ///
    /// IDEMPOTENT: at most ONE identity token per manager — a second call once
    /// a token is retained is a no-op `Ok(())` (mirrors
    /// [`Self::announce_egress_topic`]'s fast path, including the re-check
    /// under the lock). The token is retained for the manager's lifetime and
    /// undeclared on drop. Opens the zenoh session lazily; never starts the
    /// network task.
    ///
    /// # Errors
    ///
    /// - [`TransportError::Internal`] for a poisoned token mutex.
    /// - [`TransportError::InvalidTransportConfig`] when `config.robot_identity`
    ///   is unset.
    /// - [`TransportError::Publish`] for an invalid robot chunk (empty / `/`)
    ///   or if the liveliness token cannot be declared.
    /// - [`TransportError::SessionCreation`] if the zenoh session cannot open.
    pub fn announce_gateway_identity(&self) -> TransportResult<()> {
        let robot = self.require_robot_identity()?.to_string();

        // Idempotent pre-check BEFORE opening the session / declaring a token.
        {
            let identity = self
                .identity_token
                .lock()
                .map_err(|_| TransportError::Internal {
                    reason: "identity token mutex poisoned".to_string(),
                })?;
            if identity.is_some() {
                return Ok(());
            }
        }

        let session = self.session()?;
        let token = TopicToken::announce_identity(session, &robot)?;

        let mut identity = self
            .identity_token
            .lock()
            .map_err(|_| TransportError::Internal {
                reason: "identity token mutex poisoned".to_string(),
            })?;
        // Re-check under the lock (TOCTOU): a racing declarer keeps the
        // incumbent; the duplicate token drops (undeclares) at end of scope.
        if identity.is_none() {
            tracing::info!(robot = %robot, "gateway identity announce declared");
            *identity = Some(token);
        }
        Ok(())
    }

    /// Whether the bare identity announce token has been declared
    /// (Principle #3: observable state). Poison-recovering.
    pub fn has_identity_token(&self) -> bool {
        self.identity_token
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .is_some()
    }

    /// Number of retained egress-announce tokens (Principle #3:
    /// observable state). Poison-recovering. Used by the graph-build wiring
    /// pins to assert one announce per produced topic.
    pub fn announced_topic_count(&self) -> usize {
        self.announced
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .len()
    }

    /// Whether `topic` (canonical-keyed) currently has a retained
    /// egress-announce token (Principle #3). The permissive SHM probe reads this to
    /// detect a HIT whose announce previously FAILED (registered but not announced)
    /// so it can re-attempt the announce idempotently on the next demand (the probe's
    /// `heal_deferred_announce` deferred-announce heal path). Poison-recovering (crib
    /// [`Self::announced_topic_count`]).
    pub fn is_announced(&self, topic: &str) -> bool {
        let canonical = canonical_topic(topic).into_owned();
        self.announced
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .contains_key(&canonical)
    }

    /// The canonical topic names currently announced on the network,
    /// SORTED for deterministic assertions (Principle #3). Poison-recovering.
    pub fn announced_topics(&self) -> Vec<String> {
        let announced = self
            .announced
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let mut topics: Vec<String> = announced.keys().cloned().collect();
        topics.sort();
        topics
    }

    /// Returns the zenoh key expression for topic data.
    ///
    /// Format: `cerulion{canonical topic}` — e.g. `/p/n/o` →
    /// `cerulion/p/n/o`. Canonical topic names carry a
    /// leading slash, which serves as the join separator (zenoh key
    /// expressions forbid EMPTY chunks, so the previous
    /// `format!("cerulion/data/{topic}")` shape would be an invalid key
    /// for canonical names). A raw no-slash topic is canonicalized first
    /// and therefore ALIASES its slashed twin on the wire — locally they
    /// remain distinct iceoryx2 services; only canonical graph topics are
    /// bridged in production.
    ///
    /// Wire-format version boundary: the `cerulion` key namespace IS
    /// the wire-frame-format version boundary. The zenoh payload is the raw
    /// Cerulion wire frame verbatim (32-byte little-endian [`WireHeader`] +
    /// payload — no envelope), so a BREAKING change to that frame format MUST
    /// bump this namespace (e.g. `cerulion2`) to keep an old peer from
    /// mis-parsing a new frame. Additive, non-payload metadata (e.g.
    /// host/skew stamps) rides zenoh ATTACHMENTS, never the payload, so it does
    /// not touch this boundary.
    pub fn data_key(topic: &str) -> String {
        format!("cerulion{}", canonical_topic(topic))
    }

    /// Start the background network task on a dedicated thread.
    ///
    /// The task subscribes to zenoh liveliness events on `cerulion_lv/**`
    /// and drives bridge enable/disable via the `TopicBridgeManager`:
    /// - `Put` event → `enable_bridge(topic)` (remote subscriber appeared)
    /// - `Delete` event → `disable_bridge(topic)` (remote subscriber left)
    ///
    /// Contract: demand tokens PRESENT at watch start are replayed via
    /// liveliness history (`.history(true)` on the subscriber — they arrive as
    /// the same `Put` samples), and tokens declared later arrive live. Without
    /// the history replay, a gateway (re)starting while a remote consumer's
    /// demand token is already routable would never see it — the topic would
    /// silently never egress.
    fn start_task(
        state: &mut TaskState,
        session: zenoh::Session,
        bridge_mgr: Arc<TopicBridgeManager>,
        suppress_live_demand: Arc<AtomicBool>,
        // The AllowAll-only permissive SHM probe (`None` on a non-gateway
        // manager) — the liveliness watch's Put arm serves a live unregistered
        // runtime topic through it (see `apply_live_put_demand`).
        probe: Option<Arc<PermissiveShmProbe>>,
        // Canonical topics THIS manager network-INGRESSES — a demand
        // Put for one of these is our OWN re-injection's echo, never an egress
        // candidate, so the Put arm SKIPS enable_bridge (whose deny-all/allow-list
        // refusal would otherwise WARN about the machine's own demand token).
        self_ingress: Arc<Mutex<HashSet<String>>>,
        // The demand-authorization SLOT the liveliness Put-demand arm
        // consults (locked + cloned per Put — a cold-path lock). Captured as the slot so
        // a later `set_demand_authorizer` (the account authorizer) is honored
        // live by this already-running watch. Default admits everything (byte-identical).
        demand_authorizer: Arc<Mutex<Arc<dyn DemandAuthorizer>>>,
    ) -> TransportResult<()> {
        let (tx, mut rx) = tokio::sync::broadcast::channel::<()>(1);
        state.shutdown_tx = Some(tx);

        let handle = std::thread::Builder::new()
            .name("cerulion-network".to_string())
            .spawn(move || {
                let rt = tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                    .expect("failed to create network runtime");

                rt.block_on(async move {
                    tracing::info!("network task started");

                    // Subscribe to liveliness events for topic discovery.
                    // `.history(true)`: replay demand tokens ALREADY live at
                    // declaration (the gateway-restart-with-a-live-viewer case)
                    // as ordinary Put samples; later tokens arrive live.
                    let liveliness_key =
                        format!("{}/**", crate::transport::discovery::liveliness_prefix());
                    let liveliness_sub = match session
                        .liveliness()
                        .declare_subscriber(&liveliness_key)
                        .history(true)
                        .await
                    {
                        Ok(sub) => sub,
                        Err(e) => {
                            tracing::error!(
                                error = %e,
                                "failed to create liveliness subscriber — bridge disabled"
                            );
                            // Fall back to just waiting for shutdown
                            let _ = rx.recv().await;
                            return;
                        }
                    };

                    tracing::debug!(
                        key = %liveliness_key,
                        "liveliness watcher active"
                    );

                    // Strip
                    // ONLY the namespace chunk — the remainder keeps its
                    // leading slash (canonical form). NOTE: `**` matches
                    // ZERO chunks, so the BARE `cerulion_lv` key (a
                    // foreign token at exactly the namespace name) DOES
                    // arrive — the starts_with('/') guard below rejects
                    // it loudly instead of feeding "" into the bridge.
                    let prefix = crate::transport::discovery::liveliness_prefix().to_string();

                    loop {
                        tokio::select! {
                            sample = liveliness_sub.recv_async() => {
                                match sample {
                                    Ok(sample) => {
                                        let key = sample.key_expr().as_str();
                                        let recovered = key
                                            .strip_prefix(&prefix)
                                            .filter(|t| t.starts_with('/'));
                                        if recovered.is_none() {
                                            tracing::warn!(
                                                key = %key,
                                                "liveliness key matched the namespace but is \
                                                 not a canonical topic — ignored"
                                            );
                                        }
                                        if let Some(topic) = recovered {
                                            // Test seam: when suppressed,
                                            // the sample is still DRAINED (received
                                            // above) but the flag flip is skipped —
                                            // simulating the strict-link failure
                                            // where subscriber interest never wakes
                                            // while the reconciler's query still
                                            // works. Production: always false.
                                            if suppress_live_demand.load(Ordering::Relaxed) {
                                                tracing::debug!(topic = %topic, kind = ?sample.kind(), "demand sample received but subscriber flip suppressed (test seam)");
                                            } else {
                                                match sample.kind() {
                                                    zenoh::sample::SampleKind::Put => {
                                                        // A demand Put for a topic THIS
                                                        // manager INGRESSES is our OWN re-injection's
                                                        // echo (register_ingress declared the demand
                                                        // token this same-session watch hears). It is
                                                        // never an egress candidate — enabling its
                                                        // bridge would be the
                                                        // re-inject→tap→egress→re-inject loop, and on a
                                                        // deny-all/ingress-only desk enable_bridge would
                                                        // REFUSE + WARN about the machine's OWN demand
                                                        // (the wrong-audience noise a `topic hz`/`echo` desk
                                                        // would otherwise trigger). Skip it. The
                                                        // load-bearing robot-side refusal — a topic the
                                                        // robot PRODUCES but did not egress-list under
                                                        // Strict — is untouched: a produced topic is
                                                        // never in this ingress set, so it still reaches
                                                        // enable_bridge and warns.
                                                        let is_self_ingress = demand_put_targets_self_ingress(&self_ingress, topic);
                                                        if is_self_ingress {
                                                            tracing::debug!(topic = %topic, "demand for our own network-ingress topic — not an egress candidate (loop exclusion); enable_bridge skipped");
                                                        } else if let Err(e) = apply_live_put_demand(&bridge_mgr, probe.as_deref(), demand_authorizer.lock().unwrap_or_else(std::sync::PoisonError::into_inner).clone().as_ref(), topic) {
                                                            tracing::error!(topic = %topic, error = %e, "bridge manager poisoned — stopping liveliness watcher");
                                                            break;
                                                        }
                                                    }
                                                    zenoh::sample::SampleKind::Delete => {
                                                        match bridge_mgr.disable_bridge(topic) {
                                                            // Caller-owned log on the true→false edge:
                                                            // a `Delete` is a real-time subscriber departure.
                                                            Ok(true) => tracing::info!(topic = %topic, "network bridge disabled — remote subscriber left"),
                                                            Ok(false) => {}
                                                            Err(e) => {
                                                                tracing::error!(topic = %topic, error = %e, "bridge manager poisoned — stopping liveliness watcher");
                                                                break;
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    Err(e) => {
                                        tracing::error!(error = %e, "liveliness subscriber channel closed unexpectedly — network bridge state is now frozen");
                                        break;
                                    }
                                }
                            }
                            _ = rx.recv() => {
                                tracing::info!("network task shutting down");
                                break;
                            }
                        }
                    }
                });
            })
            .map_err(|e| TransportError::SessionCreation {
                reason: format!("failed to spawn network thread: {e}"),
            })?;

        state.handle = Some(handle);
        tracing::info!("network task thread spawned");
        Ok(())
    }

    /// Stop the background network task and join the thread.
    fn stop_task(state: &mut TaskState) {
        if let Some(tx) = state.shutdown_tx.take() {
            let _ = tx.send(());
        }
        if let Some(handle) = state.handle.take() {
            let _ = handle.join();
            tracing::info!("network task stopped");
        }
    }

    /// Start the background DEMAND-RECONCILER thread — the query-shaped
    /// BELT to the liveliness subscriber's SUSPENDERS.
    ///
    /// The subscriber watch (`start_task`) is the fast path: it flips egress
    /// flags sub-ms on a healthy link. This reconciler is the BELT against a
    /// MISSED subscriber wake: it runs one bounded wildcard demand gather every
    /// `RECONCILE_INTERVAL` and reconciles the announced topics' flags against it
    /// (with `REMOVE_CONFIRM`-pass hysteresis), starting egress within ~1 s of
    /// demand on any link where the demand token crosses. It never widens egress
    /// past the allow-list (the gate lives inside
    /// [`TopicBridgeManager::enable_bridge`]).
    ///
    /// SCOPE (a property of the wire): its gather is an ACCEPTER→DIALER GET of
    /// `cerulion_lv/**` — one of the two directions that do NOT route on a strict
    /// connect-only, listen-less peer link (the real robot↔Mac case), so on THAT
    /// link the reconciler cannot observe demand and nothing egresses through it.
    /// The reconciler is therefore the belt for links where the demand token DOES
    /// cross (loopback, listener-ful peers, scouting-on meshes) plus the
    /// flap/absence-hysteresis + enable-flood guard everywhere; the strict
    /// connect-only real-link demand path is the QUERYABLE INVERSION
    /// (`start_query_surface` + `start_demand_get_loop`), which moves demand
    /// onto the two WORKING directions.
    ///
    /// The tracking guard in
    /// `apply_demand_reconcile` is PRECISELY scoped: it never touches a
    /// flag for a topic whose demand it has NEVER observed (a flag another path
    /// set stays untouched), but once it OBSERVES a topic's demand it ADOPTS
    /// lifecycle ownership of that topic — enabling it now and absence-disabling
    /// it later — even if another path enabled it first.
    ///
    /// Lifecycle mirrors `start_task`: it clones the (already-open, from the
    /// caller's prior `start_network_bridge_watch`) session into a dedicated
    /// `"cerulion-demand-reconciler"` thread and is stopped by `stop_reconciler`
    /// at [`Drop`]. IDEMPOTENT — a second call while a thread is running is a
    /// no-op `Ok(())` (one reconciler per gateway).
    ///
    /// The `pass_counter`/`enabled_counter` are the caller-owned (gateway-owned,
    /// Principle #3) observability counters the thread bumps; `announced` is the
    /// BOOT canonical announce list — the SEED of the belt's iteration set.
    ///
    /// The belt feed: the belt does not iterate a FROZEN
    /// `announced` list — each pass it refreshes that set from the bridge
    /// manager's LIVE registered set (`refresh_reconcile_topics`, monotonic-count
    /// dirty-checked), so a topic REGISTERED AT RUNTIME (the control channel
    /// / the SHM probe) is re-affirmed by the belt, not only the boot
    /// topics. `announced` is thus the SEED, not the whole story: an empty seed on
    /// a permissive gateway grows to the runtime-registered set as raw routes
    /// register.
    ///
    /// Queryable inversion: the thread ALSO runs the GET-EXPIRY sweep
    /// each pass (against `get_state`, shared with the demand queryable). After
    /// applying the liveliness reconcile it disables any GET-granted topic whose
    /// last demand GET aged past `DEMAND_TTL` AND which the liveliness gather
    /// this pass did NOT hold — the composition rule (a topic disables only when
    /// BOTH the GET path and the liveliness path release it). Expiry runs ONLY on
    /// a successful gather (a failed/frozen gather leaves the liveliness view
    /// unknown, so it never disables — the conservative arm).
    ///
    /// `start_when_empty`: when `false`, an EMPTY `announced` list skips
    /// the reconciler entirely (an ingress-only / zero-egress gateway).
    /// That is right for a NON-permissive empty gateway (it can never
    /// egress), and the gateway keeps that skip by passing `false`. But a
    /// PERMISSIVE (`AllowAll`) empty-boot-plan gateway (a generic `ros2 attach`
    /// robot whose ~90 topics are all runtime raw routes) fills its egress set at
    /// runtime and its runtime topics get demand-GET-granted through the queryable,
    /// so its GET-expiry sweep IS meaningful even with an empty announce list — the
    /// gateway passes `true` to start the thread anyway. When `true`, the thread
    /// starts with an empty `announced` SEED; the belt feed then
    /// GROWS the iteration set from the live registered set as raw routes register,
    /// so the liveliness reconcile re-affirms them too (not just the GET-expiry
    /// sweep) — the belt covers runtime topics on any link where the demand token
    /// crosses.
    ///
    /// # Errors
    ///
    /// - [`TransportError::SessionCreation`] if the session cannot open or the
    ///   reconciler thread cannot spawn.
    /// - [`TransportError::Internal`] if the reconciler state mutex is poisoned.
    pub fn start_demand_reconciler(
        &self,
        bridge_mgr: Arc<TopicBridgeManager>,
        announced: Vec<String>,
        pass_counter: Arc<AtomicU64>,
        enabled_counter: Arc<AtomicU64>,
        get_state: Arc<GetDemandState>,
        start_when_empty: bool,
    ) -> TransportResult<()> {
        // Nothing to reconcile AND not permissive (an ingress-only / zero-egress
        // gateway that can never egress) — skip the thread AND its per-second query
        // entirely rather than spin a loop over an empty announce list.
        // A permissive empty-plan gateway (`start_when_empty`) starts it for
        // the GET-expiry sweep of its runtime-granted topics.
        if announced.is_empty() && !start_when_empty {
            tracing::debug!(
                "demand reconciler: no announced topics (non-permissive) — reconciler \
                 not started"
            );
            return Ok(());
        }
        // Session must be reachable (the caller starts the watch first, opening
        // it); clone it into the thread like `add_network_subscriber` does.
        let session = self.session()?.clone();

        let mut state = self
            .reconciler
            .lock()
            .map_err(|_| TransportError::Internal {
                reason: "demand reconciler state mutex poisoned".to_string(),
            })?;
        if state.handle.is_some() {
            // Already running — one reconciler per gateway (idempotent).
            return Ok(());
        }

        let (tx, mut rx) = tokio::sync::broadcast::channel::<()>(1);
        state.shutdown_tx = Some(tx);

        // Canonicalize once so the thread compares against canonical demand keys.
        let announced: Vec<String> = announced
            .iter()
            .map(|t| canonical_topic(t).into_owned())
            .collect();
        let topic_count = announced.len();

        let handle = std::thread::Builder::new()
            .name("cerulion-demand-reconciler".to_string())
            .spawn(move || {
                // A panicking reconcile pass would kill this thread SILENTLY
                // — demand would go undiscovered on a broken-liveliness
                // link with zero signal. Arm a drop-guard at entry,
                // disarm on the clean shutdown path; on an unwind it logs one loud
                // `error!`. No auto-restart (simple-is-better) — the operator
                // restarts the gateway.
                let mut death_guard = BgThreadDeathGuard {
                    clean: false,
                    msg: "demand reconciler thread",
                };
                // Per-thread absence tracking (the sync seam owns its own).
                let mut absence: HashMap<String, u32> = HashMap::new();
                // Per-thread gather-failure latch.
                let mut gather_latch = GatherFailureLatch::default();
                // The belt's LIVE iteration set — SEEDED from the
                // boot announce list, then refreshed from the bridge manager's
                // registered set each pass (`refresh_reconcile_topics`) so a
                // RUNTIME-registered topic (control channel / SHM
                // probe) is re-affirmed by the belt, not only the boot topics.
                let mut topics: Vec<String> = announced;
                loop {
                    // Sleep FIRST (coarse belt cadence; the subscriber is the
                    // fast path), waking early on shutdown so Drop joins promptly.
                    if sleep_with_shutdown(&mut rx, RECONCILE_INTERVAL) {
                        break;
                    }
                    // Run the ONE shared pass body
                    // the sync seam also runs — refresh the belt's iteration set
                    // from the live registered set (dirty-checked), then gather +
                    // apply the liveliness reconcile + the GET-expiry sweep
                    // against the SAME demand set. Calling the shared fn rather than
                    // re-inlining the body is what makes the sync-seam
                    // tests cover THIS production line by construction (see
                    // `run_reconcile_pass_on_session`). The thread passes
                    // `Some(get_state)` so its expiry sweep runs each pass.
                    run_reconcile_pass_on_session(
                        &session,
                        &bridge_mgr,
                        &mut topics,
                        &mut absence,
                        &pass_counter,
                        &enabled_counter,
                        &mut gather_latch,
                        Some(get_state.as_ref()),
                    );
                }
                death_guard.clean = true;
                tracing::debug!("demand reconciler thread exiting");
            })
            .map_err(|e| TransportError::SessionCreation {
                reason: format!("failed to spawn demand reconciler thread: {e}"),
            })?;

        state.handle = Some(handle);
        tracing::info!(topics = topic_count, "demand reconciler thread spawned");
        Ok(())
    }

    /// Stop one background helper thread (reconciler OR demand-get loop)
    /// and join it — the [`Self::stop_task`] shape (broadcast the shutdown, then
    /// join). The thread polls the signal every ≤50 ms, so the join is prompt.
    /// `label` names the thread in the stop breadcrumb.
    fn stop_bg_thread(state: &mut BgThreadState, label: &str) {
        // Raise the mid-pass abort BEFORE the broadcast send so a
        // thread stuck between the demand-GET pass's sequential blocking GETs
        // returns after the CURRENT GET (≤ DEMAND_GET_TIMEOUT) instead of running
        // the whole pass — the join below then completes promptly. No-op for the
        // reconciler (its `abort` is `None`).
        if let Some(flag) = &state.abort {
            flag.store(true, Ordering::Relaxed);
        }
        if let Some(tx) = state.shutdown_tx.take() {
            let _ = tx.send(());
        }
        if let Some(handle) = state.handle.take() {
            let _ = handle.join();
            tracing::debug!(thread = label, "background thread stopped");
        }
    }

    /// The unified query surface: declare + retain the gateway's SINGLE
    /// verb-dispatched QUERYABLE on `cerulion_q/{robot}/**`. A remote GET is routed
    /// by verb (`handle_query`): `demand{topic}` runs the demand path
    /// (enables the bridge — the allow-list gate INSIDE `enable_bridge` still
    /// enforces posture — records the grant, replies a synchronous ack;
    /// `get_state` is shared with the reconciler thread's expiry sweep),
    /// `catalog` replies the robot's full topic catalog built from `catalog` + the
    /// live bridge manager, `schema/{pkg}/{Type}` replies
    /// the requested type's `.msg`/YAML closure built from `schema`, and `runs`
    /// replies the machine's live runs, each carrying its effective
    /// `graph.yaml` + `run.json`, gathered at serve time through `runs`.
    ///
    /// IDEMPOTENT — at most ONE queryable per gateway (a second call once one is
    /// retained is a no-op `Ok(())`). Opens the session lazily. The demand verb's
    /// enable is gated by the SAME `suppress_live_demand` seam as the liveliness
    /// subscriber (both are LIVE demand paths — see the field doc).
    ///
    /// # Errors
    ///
    /// - [`TransportError::InvalidTransportConfig`] when `robot_identity` is unset.
    /// - [`TransportError::Internal`] for a poisoned queryable mutex.
    /// - [`TransportError::Receive`] if the queryable cannot be declared.
    /// - [`TransportError::SessionCreation`] if the session cannot open.
    pub fn start_query_surface(
        &self,
        bridge_mgr: Arc<TopicBridgeManager>,
        get_state: Arc<GetDemandState>,
        catalog: CatalogSource,
        schema: SchemaSource,
        runs: RunSource,
    ) -> TransportResult<()> {
        let robot = self.require_robot_identity()?.to_string();
        // Idempotent pre-check BEFORE opening the session / declaring.
        {
            let q = self
                .query_surface
                .lock()
                .map_err(|_| TransportError::Internal {
                    reason: "query surface mutex poisoned".to_string(),
                })?;
            if q.is_some() {
                return Ok(());
            }
        }

        let session = self.session()?;
        let key = cerulion_q::queryable_key(&robot);
        let suppress = Arc::clone(&self.suppress_live_demand);
        let cb_robot = robot.clone();
        // The permissive SHM probe (set at gateway boot before this runs);
        // the demand verb serves a live unregistered topic through it. `None` on a
        // non-gateway manager (the callback then serves registered topics only).
        let probe = self.permissive_probe();
        // The self-ingress exclusion set — the demand verb refuses a GET
        // for a topic THIS gateway ingresses (its own re-injection) WITHOUT
        // enable_bridge, so a Strict egress+ingress gateway self-querying its own
        // ingress topics never fires the wrong-audience refusal WARN. Same set the
        // liveliness watch consults.
        let self_ingress = Arc::clone(&self.self_ingress);
        // Capture the demand-authorization SLOT (not a snapshot) so a
        // later `set_demand_authorizer` — the account authorizer — is honored
        // LIVE by this already-declared queryable. The demand callback locks + clones
        // the current authorizer per GET (a cold-path lock, no cost to a healthy grant).
        let demand_authorizer = Arc::clone(&self.demand_authorizer);
        // The handles onto the EXACT maps the callback below captures —
        // cloned BEFORE the move, recorded (under the latch lock) iff THIS
        // declarer's queryable is the one kept, so `query_surface_serving_handles`
        // can never diverge from what the callback reads.
        let captured_serving = super::gateway::SchemaServingHandles::from_parts(
            Arc::clone(&catalog.topic_schemas),
            Arc::clone(&catalog.hash_names),
            Arc::clone(&schema.docs),
        );
        let queryable = session
            .declare_queryable(&key)
            .callback(move |query| {
                let authorizer = demand_authorizer
                    .lock()
                    .unwrap_or_else(std::sync::PoisonError::into_inner)
                    .clone();
                handle_query(
                    &query,
                    &bridge_mgr,
                    &get_state,
                    &suppress,
                    &cb_robot,
                    probe.as_deref(),
                    &self_ingress,
                    authorizer.as_ref(),
                    &catalog,
                    &schema,
                    &runs,
                );
            })
            .wait()
            .map_err(|e| TransportError::Receive {
                topic: key.clone(),
                reason: format!("query surface declaration failed: {e}"),
            })?;

        let mut q = self
            .query_surface
            .lock()
            .map_err(|_| TransportError::Internal {
                reason: "query surface mutex poisoned".to_string(),
            })?;
        // Re-check under the lock (TOCTOU): a racing declarer keeps the incumbent;
        // the duplicate queryable drops (undeclares) at end of scope.
        if q.is_none() {
            tracing::info!(robot = %robot, key = %key, "query surface declared");
            *q = Some(queryable);
            // Record the kept queryable's serving captures in the SAME
            // critical section — an incumbent keeps ITS captures (this declarer's
            // duplicate drops, and so must its handles).
            *self
                .query_surface_serving
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner) = Some(captured_serving);
        }
        Ok(())
    }

    /// The schema-serving handles the LATCHED query surface's callback
    /// actually reads — `None` until [`Self::start_query_surface`] commits a
    /// queryable. The surface outlives the gateway construction that started it
    /// (a partial boot can start it and then fail; a later construction's
    /// `start_query_surface` no-ops on the latch), so a caller merging serving
    /// into "what the catalog/schema verbs serve" MUST merge through these —
    /// handles taken from a later gateway object name maps the callback never
    /// reads. Cheap (three `Arc`s).
    pub fn query_surface_serving_handles(&self) -> Option<super::gateway::SchemaServingHandles> {
        self.query_surface_serving
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .clone()
    }

    /// Queryable inversion: start the DEMANDER's background demand-GET
    /// loop — every `DEMAND_GET_INTERVAL` it harvests producer identities from
    /// the announce space and GETs every registered ingress topic (explicitly per
    /// identity, else the wildcard) to pull that topic's egress ON at the
    /// producer. The GET (a dialer→accepter query) is the direction that survives
    /// a strict connect-only link where the ingress DEMAND token (a dialer
    /// declaration) never forwards.
    ///
    /// Lifecycle mirrors [`Self::start_demand_reconciler`]: a dedicated
    /// `"cerulion-demand-get"` thread stopped by `stop_bg_thread` at
    /// [`Drop`]. IDEMPOTENT — one loop per gateway. An empty `topics` set skips
    /// the thread (a pure-egress gateway has no ingress to demand).
    ///
    /// # Errors
    ///
    /// - [`TransportError::SessionCreation`] if the session cannot open or the
    ///   thread cannot spawn.
    /// - [`TransportError::Internal`] if the loop state mutex is poisoned.
    pub fn start_demand_get_loop(&self, demand_state: Arc<DemandGetState>) -> TransportResult<()> {
        // Nothing to demand (a pure-egress gateway) — skip the thread entirely.
        {
            let topics = demand_state
                .topics
                .lock()
                .map_err(|_| TransportError::Internal {
                    reason: "demand-get state topics mutex poisoned".to_string(),
                })?;
            if topics.is_empty() {
                tracing::debug!("demand-get loop: no ingress topics — loop not started");
                return Ok(());
            }
        }
        let session = self.session()?.clone();
        // Our own robot identity, so the harvest never self-demands.
        let self_identity = self.config.robot_identity.clone();

        let mut state = self
            .demand_get
            .lock()
            .map_err(|_| TransportError::Internal {
                reason: "demand-get loop state mutex poisoned".to_string(),
            })?;
        if state.handle.is_some() {
            // Already running — one loop per gateway (idempotent).
            return Ok(());
        }

        let (tx, mut rx) = tokio::sync::broadcast::channel::<()>(1);
        state.shutdown_tx = Some(tx);
        // The mid-pass abort flag `stop_bg_thread` raises before the
        // broadcast, so a Drop mid-pass returns after the current blocking GET.
        let abort = Arc::new(AtomicBool::new(false));
        state.abort = Some(Arc::clone(&abort));

        let handle = std::thread::Builder::new()
            .name("cerulion-demand-get".to_string())
            .spawn(move || {
                let mut death_guard = BgThreadDeathGuard {
                    clean: false,
                    msg: "demand-get loop thread",
                };
                loop {
                    // Sleep FIRST (coarse pull cadence), waking early on shutdown.
                    if sleep_with_shutdown(&mut rx, DEMAND_GET_INTERVAL) {
                        break;
                    }
                    run_demand_get_pass_on_session(
                        &session,
                        &demand_state,
                        self_identity.as_deref(),
                        &|| abort.load(Ordering::Relaxed),
                    );
                }
                death_guard.clean = true;
                tracing::debug!("demand-get loop thread exiting");
            })
            .map_err(|e| TransportError::SessionCreation {
                reason: format!("failed to spawn demand-get loop thread: {e}"),
            })?;

        state.handle = Some(handle);
        tracing::info!("demand-get loop thread spawned");
        Ok(())
    }

    /// Register `topic` in the DYNAMIC demand-keepalive set and ensure the
    /// background demand-GET loop is running over it — the PURE-INGRESS demander's
    /// self-heal wiring (`cerulion-netd`). A gateway starts its own loop from a
    /// fixed boot set ([`Self::start_demand_get_loop`]); a pure-ingress demander
    /// (which only ever calls [`crate::transport::TransportManager::register_ingress_topic`],
    /// never boots a gateway) instead calls THIS per demanded topic so the
    /// re-affirming GET keepalive runs — the dialer→accepter query that
    /// (a) crosses a strict connect-only link the demand-token declaration cannot,
    /// and (b) RE-AFFIRMS egress within one pass after a restarted robot gateway's
    /// transport reconnects (peer-mode default `connect/timeout_ms=-1` retries the
    /// endpoint forever), which is exactly the self-heal the silent black
    /// hole was missing.
    ///
    /// Idempotent: a repeat call for the same topic is a set no-op, and the loop
    /// start is start-once (a running loop reads the grown set on its next pass).
    /// The topic is canonicalized to match the demand key-space.
    ///
    /// # Errors
    ///
    /// [`TransportError::SessionCreation`] if the loop thread cannot spawn or the
    /// session cannot open (the first call opens the lazy session).
    #[must_use = "ingress demand keepalive result must be checked"]
    pub fn ensure_ingress_demand_keepalive(&self, topic: &str) -> TransportResult<()> {
        let canonical = canonical_topic(topic).into_owned();
        self.ingress_demand_state.add_topic(canonical);
        // Start-once over the SHARED dynamic set. `start_demand_get_loop` short-
        // circuits `Ok` when the loop is already running (one loop per manager),
        // and the running loop re-reads the grown topic set every pass — so the
        // newly-added topic is demanded on the very next pass without a restart.
        self.start_demand_get_loop(Arc::clone(&self.ingress_demand_state))
    }

    /// Remove `topic` from the DYNAMIC demand-keepalive set (its mirror's
    /// last consumer left and the ingress bridge was torn down). The running loop
    /// stops GETting it on its next pass; the loop thread itself keeps running for
    /// the manager's lifetime (stopped once at Drop) so a later demand re-affirms
    /// cheaply. A topic that was never registered is a no-op.
    pub fn remove_ingress_demand_keepalive(&self, topic: &str) {
        let canonical = canonical_topic(topic).into_owned();
        self.ingress_demand_state.remove_topic(&canonical);
    }

    /// Run ONE demand-GET pass INLINE over the dynamic keepalive set — the
    /// synchronous belt the netd health watch drives when a mirror goes stale, so a
    /// restarted robot gateway is re-demanded PROMPTLY (as soon as the transport
    /// reconnects) rather than waiting up to `DEMAND_GET_INTERVAL` for the next
    /// background pass. Unlike the test-gated `demand_get_pass` seam this is a
    /// PRODUCTION seam: it delegates to the SAME pass body the background loop runs,
    /// over the SAME shared state. A no-topic set is a cheap no-op pass.
    ///
    /// `should_abort` is polled BEFORE the pass and BETWEEN GETs (the
    /// mid-pass-abort mechanism the background loop uses): the netd health watch
    /// passes its own shutdown flag so a plane drop DURING a belt pass returns after
    /// the current GET instead of blocking `HealthWatch::drop`'s join for the whole
    /// pass — a degraded regime is exactly when every GET times out FULLY (no
    /// producer answers), so an un-abortable pass would stall teardown for
    /// `≈ 300ms × topics × selectors`.
    ///
    /// # Errors
    ///
    /// [`TransportError::SessionCreation`] if the session cannot open.
    #[must_use = "ingress demand-get pass result must be checked"]
    pub fn run_ingress_demand_get_pass(
        &self,
        should_abort: &dyn Fn() -> bool,
    ) -> TransportResult<()> {
        let session = self.session()?;
        run_demand_get_pass_on_session(
            session,
            &self.ingress_demand_state,
            self.config.robot_identity.as_deref(),
            should_abort,
        );
        Ok(())
    }

    /// Observable state (Principle #3): the number of topics in the dynamic demand-keepalive
    /// set — observable state for tests + operators.
    pub fn ingress_demand_keepalive_topic_count(&self) -> usize {
        self.ingress_demand_state.topic_count()
    }

    /// SYNC SEAM: run exactly ONE reconcile pass inline (gather + apply)
    /// against caller-owned state — the deterministic, no-thread driver used by
    /// [`crate::transport::gateway::GatewayRuntime::reconcile_demand_once`]. Grabs
    /// the session and delegates to the SAME pass body the background thread runs.
    ///
    /// # Errors
    ///
    /// [`TransportError::SessionCreation`] if the session cannot open.
    #[cfg(any(test, feature = "test-helpers"))]
    pub(crate) fn reconcile_demand_pass(
        &self,
        bridge_mgr: &TopicBridgeManager,
        topics: &mut Vec<String>,
        absence: &mut HashMap<String, u32>,
        pass_counter: &AtomicU64,
        enabled_counter: &AtomicU64,
    ) -> TransportResult<()> {
        let session = self.session()?;
        // The sync seam is a deterministic single-pass test driver whose gather
        // succeeds against a live session, so it uses a per-call gather latch —
        // the persistent, regime-spanning latch
        // lives on the production background thread (`start_demand_reconciler`).
        // `run_reconcile_pass_on_session` refreshes `topics` from
        // the live registered set first (the belt feed), so this seam re-affirms a
        // runtime-registered topic exactly as the thread does.
        let mut gather_latch = GatherFailureLatch::default();
        // `None` get_state: the sync reconcile seam does NOT run the GET-expiry
        // sweep (its dedicated `get_expiry_pass` seam owns that), so it must not
        // disable a GET-granted topic as a side effect of a reconcile pass.
        run_reconcile_pass_on_session(
            session,
            bridge_mgr,
            topics,
            absence,
            pass_counter,
            enabled_counter,
            &mut gather_latch,
            None,
        );
        Ok(())
    }

    /// TEST SEAM: set the suppress-live-demand flag (see
    /// [`Self::suppress_live_demand`]). When `true`, BOTH live producer demand
    /// paths — the liveliness subscriber AND the demand queryable — drain/respond
    /// but skip flipping flags, so a test can prove the reconciler is the sole
    /// flag-flipper on a link where live demand propagation is broken.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn set_suppress_live_demand_for_test(&self, suppress: bool) {
        self.suppress_live_demand.store(suppress, Ordering::Relaxed);
    }

    /// TEST SEAM: set the suppress-ingress-token flag (see
    /// [`Self::suppress_ingress_token`]). When `true`, [`Self::register_ingress`]
    /// skips declaring its liveliness DEMAND token, so the producer's subscriber
    /// sees no `Put` and its reconciler gathers empty — isolating the queryable-
    /// inversion GET path as the sole enabler in the decisive e2e.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn set_suppress_ingress_token_for_test(&self, suppress: bool) {
        self.suppress_ingress_token
            .store(suppress, Ordering::Relaxed);
    }

    /// TEST SEAM: arm fire-once fault injection on
    /// [`Self::announce_egress_topic`] (see [`Self::fail_next_announce_egress`]).
    /// The NEXT announce call returns `TransportError::Publish` and disarms the
    /// flag. Mirrors the `fault_inject_publish_raw_after` precedent — lets the
    /// gateway `register_runtime_topic` announce-failure path be pinned WITHOUT a
    /// broken zenoh session.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn fault_inject_announce_egress_once(&self) {
        self.fail_next_announce_egress
            .store(true, Ordering::Relaxed);
    }

    /// SYNC SEAM: run exactly ONE demand-GET pass inline (the DEMANDER
    /// side) — the deterministic, no-thread driver used by
    /// [`crate::transport::gateway::GatewayRuntime::demand_get_once`]. Delegates
    /// to the SAME pass body the background loop runs.
    ///
    /// # Errors
    ///
    /// [`TransportError::SessionCreation`] if the session cannot open.
    #[cfg(any(test, feature = "test-helpers"))]
    pub(crate) fn demand_get_pass(&self, demand_state: &DemandGetState) -> TransportResult<()> {
        let session = self.session()?;
        // The deterministic single-pass seam never aborts (no teardown
        // races a synchronous inline pass) — a never-abort closure.
        run_demand_get_pass_on_session(
            session,
            demand_state,
            self.config.robot_identity.as_deref(),
            &|| false,
        );
        Ok(())
    }

    /// SYNC SEAM: run exactly ONE GET-expiry sweep inline (the PRODUCER
    /// side) at `now` — the deterministic driver for the expiry test used by
    /// [`crate::transport::gateway::GatewayRuntime::run_get_expiry_once`]. Gathers
    /// the liveliness demand set once (like the background thread) so the
    /// composition rule can gate the expiry, then delegates to [`run_get_expiry`].
    /// A failed gather is treated as an EMPTY liveliness view (the demander is
    /// gone in the expiry scenario), so the expiry proceeds.
    ///
    /// # Errors
    ///
    /// [`TransportError::SessionCreation`] if the session cannot open.
    #[cfg(any(test, feature = "test-helpers"))]
    pub(crate) fn get_expiry_pass(
        &self,
        bridge_mgr: &TopicBridgeManager,
        get_state: &GetDemandState,
        now: Instant,
    ) -> TransportResult<()> {
        let session = self.session()?;
        let liveliness_demand =
            gather_demand_set(session, RECONCILE_GATHER_WINDOW).unwrap_or_default();
        run_get_expiry(bridge_mgr, get_state, &liveliness_demand, now);
        Ok(())
    }

    /// Build the zenoh `Config` from our `NetworkConfig`.
    fn build_zenoh_config(&self) -> TransportResult<zenoh::Config> {
        let mut config = zenoh::Config::default();
        let map_err = |key: &str, e: zenoh::Error| TransportError::SessionCreation {
            reason: format!("failed to set {key}: {e}"),
        };

        config
            .insert_json5("mode", self.config.mode.as_str())
            .map_err(|e| map_err("mode", e))?;
        config
            .insert_json5(
                "scouting/multicast/enabled",
                &self.config.multicast_scouting.to_string(),
            )
            .map_err(|e| map_err("scouting/multicast/enabled", e))?;
        config
            .insert_json5(
                "scouting/gossip/enabled",
                &self.config.gossip_scouting.to_string(),
            )
            .map_err(|e| map_err("scouting/gossip/enabled", e))?;

        // Explicitly set endpoints to prevent zenoh defaults from listening on random ports.
        let connect_json = serde_json::to_string(&self.config.connect_endpoints).map_err(|e| {
            TransportError::SessionCreation {
                reason: format!("failed to serialize connect endpoints: {e}"),
            }
        })?;
        config
            .insert_json5("connect/endpoints", &connect_json)
            .map_err(|e| map_err("connect/endpoints", e))?;

        // HARD-BOUND the connect phase at 1 s so a black-hole connect
        // locator never stalls an ephemeral discovery query (the 11 s `topic
        // list` stall fix). Mechanics (verified against zenoh 1.8's
        // `Runtime::connect_peers`): a POSITIVE `connect/timeout_ms` is the
        // real bound — it wraps the whole connect phase in
        // `tokio::time::timeout`; `0` would take the inline-unbounded branch
        // where each endpoint pays up to the 10 s transport open_timeout
        // SEQUENTIALLY (worse than the `-1` default, which retries dead
        // endpoints on a background connector). `connect/retry/period_init_ms:
        // 0` keeps the inline SINGLE-ATTEMPT branch per endpoint, so reachable
        // robots connect BEFORE the open returns and the gather cannot race
        // the TCP connect; `exit_on_failure: false` keeps an unreachable
        // endpoint from failing the open. Only set when the caller opted in
        // (the discovery query session); gateways/graph runs keep zenoh's
        // robust default retry semantics.
        if self.config.bounded_connect {
            config
                .insert_json5("connect/timeout_ms", "1000")
                .map_err(|e| map_err("connect/timeout_ms", e))?;
            // `connect/retry` defaults to a NONE Option, so set the whole
            // sub-object (unambiguously supported — deep-pathing into a None
            // Option is not) with `period_init_ms: 0`; the other retry fields
            // stay at their defaults.
            config
                .insert_json5("connect/retry", "{ period_init_ms: 0 }")
                .map_err(|e| map_err("connect/retry", e))?;
            config
                .insert_json5("connect/exit_on_failure", "false")
                .map_err(|e| map_err("connect/exit_on_failure", e))?;
        }

        let listen_json = serde_json::to_string(&self.config.listen_endpoints).map_err(|e| {
            TransportError::SessionCreation {
                reason: format!("failed to serialize listen endpoints: {e}"),
            }
        })?;
        config
            .insert_json5("listen/endpoints", &listen_json)
            .map_err(|e| map_err("listen/endpoints", e))?;

        Ok(config)
    }
}

impl Drop for NetworkManager {
    fn drop(&mut self) {
        // Tear down the ingress bridges FIRST, while the zenoh session
        // (a sibling field — it drops only after this body returns) is still
        // open: each entry's zenoh subscriber and liveliness token undeclare
        // against the live session, and the entry's LOCAL iceoryx2 injection
        // publisher drops here (releasing its ports/services) — never after
        // the session or (via `TransportManager`'s network-before-node field
        // order) the iceoryx2 node handle. Poison-recovering: a panicked
        // registrant must not abort teardown.
        {
            let ingress = self
                .ingress
                .get_mut()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            let count = ingress.len();
            ingress.clear();
            if count > 0 {
                tracing::debug!(count, "network ingress bridges torn down");
            }
        }

        // Undeclare the egress-announce tokens against the live session
        // too (same ordering rationale as ingress above — the session field
        // drops only after this Drop body returns).
        {
            let announced = self
                .announced
                .get_mut()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            let count = announced.len();
            announced.clear();
            if count > 0 {
                tracing::debug!(count, "network egress announces undeclared");
            }
        }

        // Undeclare the bare identity announce token against the live
        // session too (same ordering rationale as the announces above).
        {
            let identity = self
                .identity_token
                .get_mut()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            if identity.take().is_some() {
                tracing::debug!("gateway identity announce undeclared");
            }
        }

        // Stop the demand reconciler + demand-get loop threads BEFORE
        // the session field drops (a leaked thread must not outlive teardown and
        // touch a closing session) — same join-before-session ordering as
        // `stop_task` below. Poison-recovering (a panicked pass must not abort
        // teardown).
        {
            let reconciler = self
                .reconciler
                .get_mut()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            Self::stop_bg_thread(reconciler, "demand-reconciler");
        }
        {
            let demand_get = self
                .demand_get
                .get_mut()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            Self::stop_bg_thread(demand_get, "demand-get");
        }

        // Undeclare the query surface against the live session (same
        // ordering rationale as the announces above).
        {
            let queryable = self
                .query_surface
                .get_mut()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            if queryable.take().is_some() {
                tracing::debug!("query surface undeclared");
            }
        }

        // &mut self in Drop: access Mutex internals directly without locking
        let state = self.task.get_mut().unwrap();
        Self::stop_task(state);

        if self.session.get().is_some() {
            // Operational plumbing (see the open-side note): DEBUG, not INFO,
            // so an ephemeral discovery session's teardown is silent at the
            // `cerulion=info` CLI default.
            tracing::debug!("zenoh session closing (NetworkManager dropped)");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The DYNAMIC demand-keepalive set (`DemandGetState::add_topic` /
    /// `remove_topic` / `topic_count`) grows + shrinks correctly and is idempotent.
    /// The demand-GET pass re-reads this set every pass, so this is the seam that
    /// lets a pure-ingress netd re-affirm demand for a live, growing set of mirrors.
    /// Hand oracle, no transport.
    #[test]
    fn demand_get_state_dynamic_topic_set_grows_shrinks_and_is_idempotent() {
        let state = DemandGetState::default();
        assert_eq!(state.topic_count(), 0, "fresh state is empty");

        state.add_topic("/a".to_string());
        state.add_topic("/b".to_string());
        assert_eq!(state.topic_count(), 2);

        // Idempotent add — a duplicate is a no-op (a re-demand of a live mirror).
        state.add_topic("/a".to_string());
        assert_eq!(state.topic_count(), 2, "duplicate add is a no-op");

        // Remove one; the sibling survives.
        state.remove_topic("/a");
        assert_eq!(state.topic_count(), 1);
        {
            let topics = state.topics.lock().unwrap();
            assert_eq!(&*topics, &vec!["/b".to_string()], "only /b remains");
        }

        // Removing a never-present topic is a no-op (teardown must never fail).
        state.remove_topic("/never");
        assert_eq!(state.topic_count(), 1);

        // Remove the last; empty again (a demand-GET pass over it is a cheap no-op).
        state.remove_topic("/b");
        assert_eq!(state.topic_count(), 0);
    }

    /// The netd health belt's inline
    /// demand-GET pass (`run_ingress_demand_get_pass`) THREADS its caller's abort
    /// closure through to `run_demand_get_pass_on_session` — so a plane drop
    /// mid-pass returns after the current GET instead of blocking the health
    /// thread's join for the whole (all-GETs-time-out-while-the-robot-is-down)
    /// pass. `run_demand_get_pass_on_session` checks `should_abort()` at the TOP
    /// (before the pass counter bump), so an
    /// abort-immediately closure short-circuits with ZERO passes run, while a
    /// never-abort closure runs the pass. Reverting `run_ingress_demand_get_pass`'s
    /// `should_abort` argument back to a hardcoded `&|| false`
    /// makes the abort=true case run the pass → this test FAILS.
    #[test]
    fn run_ingress_demand_get_pass_honors_the_abort_closure() {
        // Scouting-off, no-endpoint local session (hermetic, no peer). The pass's
        // top abort check runs before any GET, so no remote is needed.
        let mgr = NetworkManager::new(NetworkConfig::default());
        mgr.ingress_demand_state.add_topic("/abort".to_string());

        // Abort IMMEDIATELY → the pass body returns at its top, before the counter
        // bump. Zero passes run.
        mgr.run_ingress_demand_get_pass(&|| true)
            .expect("abort pass is Ok");
        assert_eq!(
            mgr.ingress_demand_state.pass_count(),
            0,
            "abort=true short-circuits the pass (no GETs, no counter bump)"
        );

        // Never abort → the pass runs (harvest + the topic GET; no peer ⇒ fast).
        mgr.run_ingress_demand_get_pass(&|| false)
            .expect("run pass is Ok");
        assert_eq!(
            mgr.ingress_demand_state.pass_count(),
            1,
            "abort=false runs the pass exactly once (anti-tautology — the closure is honored)"
        );
    }

    /// Oracle-vector for the liveliness-watch self-ingress skip
    /// decision ([`demand_put_targets_self_ingress`]). A demand Put for a topic
    /// THIS manager ingresses is our own re-injection's demand echo → SKIP
    /// enable_bridge (no wrong-audience refusal WARN); any other topic → do NOT
    /// skip (the genuine egress refusal path is untouched). Poison-recovering.
    /// Pure — no transport. Hand oracle, NOT a self-compare. What breaks this test:
    /// reverting the watch's `if is_self_ingress { skip }` to always calling
    /// `apply_live_put_demand` re-opens the false WARN (pinned e2e by
    /// `network_ingress_test::self_ingress_demand_is_not_refused_but_foreign_is`).
    #[test]
    fn self_ingress_skip_decision_oracle() {
        use std::collections::HashSet;
        use std::sync::Mutex;

        let set: Mutex<HashSet<String>> = Mutex::new(HashSet::new());

        // Fresh set: nothing is self-ingress (a produced topic demanded by a
        // remote must still reach enable_bridge → warn).
        assert!(
            !demand_put_targets_self_ingress(&set, "/mine"),
            "empty set: no topic is our ingress"
        );

        // After registering /mine as an ingress topic: a demand Put for /mine is
        // OUR echo → skip.
        set.lock().unwrap().insert("/mine".to_string());
        assert!(
            demand_put_targets_self_ingress(&set, "/mine"),
            "an ingressed topic's own demand echo is skipped"
        );
        // A DISTINCT foreign topic is NEVER skipped (the load-bearing genuine
        // refusal path stays live — a produced-but-unlisted topic warns).
        assert!(
            !demand_put_targets_self_ingress(&set, "/foreign"),
            "a topic we do not ingress is not skipped"
        );
        // Exact-name keyed (no prefix confusion): /mine/sub is not /mine.
        assert!(
            !demand_put_targets_self_ingress(&set, "/mine/sub"),
            "the check is exact-name, not prefix"
        );

        // Poison recovery: a poisoned set must NOT silently flip a real
        // self-ingress topic to "not ingress" (which would re-open the false
        // WARN). Poison the mutex, then re-assert the verdict survives.
        let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _guard = set.lock().unwrap();
            panic!("poison the self_ingress mutex");
        }));
        assert!(set.is_poisoned(), "the set is now poisoned");
        assert!(
            demand_put_targets_self_ingress(&set, "/mine"),
            "poison-recovering: an ingressed topic is STILL skipped after poison"
        );
    }

    /// The flagship gap: a RUNTIME-registered topic (a
    /// `ros2 attach` raw route) arrives over the reg-channel carrying only a
    /// `(topic, schema_hash)` — NO name — so it is absent from `topic_schemas`.
    /// The catalog must NAME it by resolving ITS hash through the `hash_names`
    /// reverse map; an unknown hash stays `None` (never a wrong name).
    /// Pure — no transport. Hand oracle.
    ///
    /// Reverting the `.or_else(|| schema_hash.and_then(..))` fold in
    /// `build_catalog_reply_from_bridge` (to a bare `topic_schemas.get(..)` lookup)
    /// leaves `/rt/widget` unnamed and fails the first assertion.
    #[test]
    fn catalog_names_runtime_topic_via_hash_reverse_map() {
        use std::collections::{HashMap, HashSet};
        use std::sync::{Arc, Mutex};

        const WIDGET_HASH: u64 = 0xC823_0001_ABCD_0001;
        let bridge = TopicBridgeManager::new();
        // Two registered topics: one runtime raw route (hash known, NO topic→name
        // binding), one whose hash is NOT in the reverse map.
        bridge
            .register_topic("/rt/widget")
            .expect("register widget");
        bridge
            .register_topic("/rt/unknown")
            .expect("register unknown");

        let catalog = CatalogSource {
            hashes: Arc::new(Mutex::new(HashMap::from([
                ("/rt/widget".to_string(), WIDGET_HASH),
                ("/rt/unknown".to_string(), 0xDEAD_BEEF_u64),
            ]))),
            boot_topics: Arc::new(HashSet::new()),
            // No explicit topic→name binding for either runtime topic.
            topic_schemas: Arc::new(Mutex::new(HashMap::new())),
            // The reverse map names the widget hash; the unknown hash is absent.
            hash_names: Arc::new(Mutex::new(HashMap::from([(
                WIDGET_HASH,
                "probe_msgs/Widget".to_string(),
            )]))),
            // No transport handle in this pure serve-decision seam →
            // producer_count serves None (byte-identical to a wire without the field).
            producer_probe: None,
            liveness: None,
            liveness_clock: None,
        };

        let reply = build_catalog_reply_from_bridge(&bridge, &catalog, "go2");
        let widget = reply
            .entries
            .iter()
            .find(|e| e.topic == "/rt/widget")
            .expect("widget entry");
        // Named via the hash reverse map (its reg-channel hash resolved).
        assert_eq!(widget.schema_name.as_deref(), Some("probe_msgs/Widget"));
        assert_eq!(widget.schema_hash, Some(WIDGET_HASH));
        assert_eq!(widget.provenance, CatalogProvenance::Runtime);
        // An unknown hash → still None (never a wrong name).
        let unknown = reply
            .entries
            .iter()
            .find(|e| e.topic == "/rt/unknown")
            .expect("unknown entry");
        assert_eq!(unknown.schema_name, None);
    }

    /// The EXPLICIT topic→name binding (a graph node
    /// OUTPUT) WINS over the hash reverse map — precedence pin. Pure. Hand oracle.
    #[test]
    fn catalog_explicit_topic_binding_wins_over_hash_reverse_map() {
        use std::collections::{HashMap, HashSet};
        use std::sync::{Arc, Mutex};

        const HASH: u64 = 0xC823_0002_0000_0002;
        let bridge = TopicBridgeManager::new();
        bridge.register_topic("/g/cam/image").expect("register");
        let catalog = CatalogSource {
            hashes: Arc::new(Mutex::new(HashMap::from([(
                "/g/cam/image".to_string(),
                HASH,
            )]))),
            boot_topics: Arc::new(HashSet::new()),
            topic_schemas: Arc::new(Mutex::new(HashMap::from([(
                "/g/cam/image".to_string(),
                "sensor_msgs/Image".to_string(),
            )]))),
            // A (deliberately wrong) reverse-map entry the explicit binding must beat.
            hash_names: Arc::new(Mutex::new(HashMap::from([(
                HASH,
                "wrong/ByHash".to_string(),
            )]))),
            producer_probe: None,
            liveness: None,
            liveness_clock: None,
        };
        let reply = build_catalog_reply_from_bridge(&bridge, &catalog, "go2");
        let entry = reply
            .entries
            .iter()
            .find(|e| e.topic == "/g/cam/image")
            .expect("image entry");
        assert_eq!(entry.schema_name.as_deref(), Some("sensor_msgs/Image"));
    }

    #[test]
    fn test_default_config_disables_scouting() {
        let config = NetworkConfig::default();
        assert!(!config.multicast_scouting);
        assert!(!config.gossip_scouting);
        assert!(config.connect_endpoints.is_empty());
        assert!(config.listen_endpoints.is_empty());
        assert_eq!(config.mode, ZenohMode::Peer);
        // Bounded-connect is OFF by default — only the ephemeral
        // discovery query session opts in — and no robot identity is assumed.
        assert!(!config.bounded_connect);
        assert!(config.robot_identity.is_none());
    }

    /// `bounded_connect: true` carries the
    /// THREE connect-bounding zenoh keys — `connect/timeout_ms == 1000` (the
    /// POSITIVE global timeout is the real tokio hard bound in zenoh 1.8; `0`
    /// would take the inline-unbounded branch), `connect/retry/period_init_ms
    /// == 0` (single inline attempt so reachable robots connect BEFORE the
    /// open returns), and `connect/exit_on_failure == false` — so an
    /// unroutable connect locator can never stall the session open past the
    /// bound; `false` carries NONE of them (zenoh's robust default retry).
    /// Asserted via the built zenoh `Config`'s `get_json` getters — hand
    /// oracles, never a self-compare.
    #[test]
    fn bounded_connect_sets_the_three_connect_keys_only_when_opted_in() {
        // Opted in: all three keys present with the exact bounded values (a
        // `ModeDependentValue::Unique(..)` serializes bare).
        let bounded = NetworkManager::new(NetworkConfig {
            bounded_connect: true,
            ..NetworkConfig::default()
        });
        let cfg = bounded
            .build_zenoh_config()
            .expect("bounded config must build");
        // `.ok().as_deref()` yields `Option<&str>` (the boxed zenoh error is not
        // `PartialEq`, so a bare `Result` compare would not compile).
        assert_eq!(
            cfg.get_json("connect/timeout_ms").ok().as_deref(),
            Some("1000"),
            "bounded_connect must set connect/timeout_ms=1000 (the POSITIVE \
             global timeout is the real hard bound; 0 is inline-unbounded)"
        );
        // zenoh's deep `get_json` cannot traverse into the `Option`-wrapped
        // mode-dependent retry struct (NoMatchingKey even when set) — read the
        // whole `connect/retry` object and assert on parsed JSON instead.
        let retry: serde_json::Value = serde_json::from_str(
            &cfg.get_json("connect/retry")
                .expect("bounded_connect must set the connect/retry object"),
        )
        .expect("connect/retry must be valid JSON");
        assert_eq!(
            retry["period_init_ms"],
            serde_json::json!(0),
            "bounded_connect must set connect/retry.period_init_ms=0 (single \
             inline attempt — reachable robots connect before the open returns)"
        );
        assert_eq!(
            cfg.get_json("connect/exit_on_failure").ok().as_deref(),
            Some("false"),
            "bounded_connect must set connect/exit_on_failure=false"
        );

        // Opted out (default): none of the keys is forced to the bounded value.
        // The fields are unset (schema default `null` or absent), so `get_json`
        // is Err (→ None) or `"null"` — in either case NOT the bounded value.
        let unbounded = NetworkManager::new(NetworkConfig::default());
        let cfg = unbounded
            .build_zenoh_config()
            .expect("unbounded config must build");
        assert_ne!(
            cfg.get_json("connect/timeout_ms").ok().as_deref(),
            Some("1000"),
            "the default (unbounded) config must NOT force connect/timeout_ms=1000"
        );
        // Same deep-path caveat: read the whole retry object (absent/None on
        // the default config) — it must not carry the forced zero.
        let default_retry_zeroed = cfg
            .get_json("connect/retry")
            .ok()
            .and_then(|j| serde_json::from_str::<serde_json::Value>(&j).ok())
            .is_some_and(|v| v["period_init_ms"] == serde_json::json!(0));
        assert!(
            !default_retry_zeroed,
            "the default (unbounded) config must NOT force connect/retry.period_init_ms=0"
        );
        assert_ne!(
            cfg.get_json("connect/exit_on_failure").ok().as_deref(),
            Some("false"),
            "the default (unbounded) config must NOT force connect/exit_on_failure=false"
        );
    }

    #[test]
    fn test_zenoh_mode_as_str() {
        assert_eq!(ZenohMode::Peer.as_str(), "\"peer\"");
        assert_eq!(ZenohMode::Client.as_str(), "\"client\"");
        assert_eq!(ZenohMode::Router.as_str(), "\"router\"");
    }

    #[test]
    fn test_build_zenoh_config_succeeds() {
        let mgr = NetworkManager::new(NetworkConfig::default());
        let config = mgr.build_zenoh_config();
        assert!(config.is_ok());
    }

    #[test]
    fn test_network_manager_not_active_before_session() {
        let mgr = NetworkManager::new(NetworkConfig::default());
        assert!(!mgr.is_active());
    }

    #[test]
    fn test_task_not_running_initially() {
        let mgr = NetworkManager::new(NetworkConfig::default());
        assert!(!mgr.is_task_running());
        assert_eq!(mgr.network_subscriber_count(), 0);
    }

    // -----------------------------------------------------------------------
    // Ingress-frame validation (pure oracle tests, no transport)
    // -----------------------------------------------------------------------

    /// Hand-build a raw wire frame (32-byte little-endian header + payload)
    /// exactly as an egress publisher would put it on the wire.
    fn make_frame(schema_hash: u64, seq: u32, timestamp_ns: u64, payload: &[u8]) -> Vec<u8> {
        let header = WireHeader {
            schema_hash,
            total_size: (WireHeader::SIZE + payload.len()) as u32,
            offset_table_offset: 0,
            offset_table_count: 0,
            sequence: seq,
            timestamp_ns,
        };
        let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
        header.write_to_buf(&mut frame[..WireHeader::SIZE]);
        frame[WireHeader::SIZE..].copy_from_slice(payload);
        frame
    }

    #[test]
    fn validate_ingress_frame_accepts_well_formed_frame() {
        let frame = make_frame(0xCAFE_BABE, 42, 5_000_000_000, &[1, 2, 3, 4]);
        let header = validate_ingress_frame(0xCAFE_BABE, &frame).expect("well-formed frame");
        // Hand oracle: the parsed header carries exactly what we stamped.
        assert_eq!(header.schema_hash, 0xCAFE_BABE);
        assert_eq!(header.sequence, 42);
        assert_eq!(header.timestamp_ns, 5_000_000_000);
        assert_eq!(header.total_size as usize, frame.len());
    }

    #[test]
    fn validate_ingress_frame_accepts_empty_payload_at_header_boundary() {
        // A header-only (32-byte) frame is valid — an empty-payload schema.
        let frame = make_frame(0x1234, 0, 0, &[]);
        assert_eq!(frame.len(), WireHeader::SIZE);
        assert!(validate_ingress_frame(0x1234, &frame).is_ok());
    }

    #[test]
    fn validate_ingress_frame_rejects_truncated_below_header() {
        assert_eq!(
            validate_ingress_frame(0x1, &[0u8; 10]),
            Err(IngressRejection::TooSmall)
        );
        // Exactly one byte short of the header is still TooSmall.
        assert_eq!(
            validate_ingress_frame(0x1, &[0u8; WireHeader::SIZE - 1]),
            Err(IngressRejection::TooSmall)
        );
    }

    #[test]
    fn validate_ingress_frame_rejects_total_size_mismatch() {
        let mut frame = make_frame(0xABCD, 7, 100, &[9, 9, 9]);
        // Corrupt total_size (header bytes 8..12) to disagree with the length.
        frame[8] = frame[8].wrapping_add(1);
        assert_eq!(
            validate_ingress_frame(0xABCD, &frame),
            Err(IngressRejection::SizeMismatch)
        );
    }

    #[test]
    fn validate_ingress_frame_rejects_schema_mismatch() {
        let frame = make_frame(0xAAAA_AAAA, 1, 1, &[5, 6, 7]);
        assert_eq!(
            validate_ingress_frame(0xBBBB_BBBB, &frame),
            Err(IngressRejection::SchemaMismatch)
        );
    }

    #[test]
    fn ingress_warn_latch_first_warn_then_debug_then_reset() {
        // The latch bit is `false` until the first failure of a class warns,
        // then stays `true` (repeats debug) until a successful frame resets it.
        let latch = IngressWarnLatch::default();
        // Steady state: reset with no failure run ends nothing (no recovery
        // report — the callback stays log-free).
        assert!(!latch.reset(), "steady-state reset must report no recovery");
        // Pre-warn: class fresh (swap returns the OLD value = false).
        assert!(!latch.schema_mismatch_warned.swap(true, Ordering::Relaxed));
        // Repeat: now already warned (swap returns true).
        assert!(latch.schema_mismatch_warned.swap(true, Ordering::Relaxed));
        // Reset (a successful frame) ENDS the failure run: reports recovery
        // exactly once, clears every bit → next failure warns again.
        assert!(
            latch.reset(),
            "reset after a failure run must report recovery"
        );
        assert!(!latch.reset(), "second reset must not re-report recovery");
        assert!(!latch.schema_mismatch_warned.swap(true, Ordering::Relaxed));
        assert!(!latch.decode_error_warned.swap(true, Ordering::Relaxed));
        assert!(!latch.reinject_failure_warned.swap(true, Ordering::Relaxed));
        // Any single set bit (here: reinject) makes the next reset a recovery.
        assert!(latch.reset());
    }

    #[test]
    fn record_ingress_rejection_bumps_the_right_counter() {
        let counters = IngressCounters::default();
        let latch = IngressWarnLatch::default();
        // Schema mismatch → schema_mismatch_drops.
        record_ingress_rejection(&counters, &latch, "/t", IngressRejection::SchemaMismatch);
        // Every structural class → decode_errors.
        record_ingress_rejection(&counters, &latch, "/t", IngressRejection::TooSmall);
        record_ingress_rejection(&counters, &latch, "/t", IngressRejection::BadHeader);
        record_ingress_rejection(&counters, &latch, "/t", IngressRejection::SizeMismatch);
        assert_eq!(counters.schema_mismatch_drops.load(Ordering::Relaxed), 1);
        assert_eq!(counters.decode_errors.load(Ordering::Relaxed), 3);
        assert_eq!(counters.frames.load(Ordering::Relaxed), 0);
        assert_eq!(counters.reinject_failure_drops.load(Ordering::Relaxed), 0);
    }

    // -----------------------------------------------------------------------
    // Queryable inversion — pure oracle tests (no transport, no clock)
    // -----------------------------------------------------------------------

    /// `demand_get_selectors` — WILDCARD when no identity is
    /// harvested (reaches every producer's `cerulion_q/{robot}/**` queryable),
    /// EXPLICIT per identity once harvested (precise, deterministic sorted order),
    /// under the unified `cerulion_q/{robot}/demand{topic}` grammar. Hand oracles.
    #[test]
    fn demand_get_selectors_wildcard_then_explicit() {
        // No identities → the single-chunk wildcard fallback.
        let empty = BTreeSet::new();
        assert_eq!(
            demand_get_selectors("/utlidar/cloud", &empty),
            vec!["cerulion_q/*/demand/utlidar/cloud".to_string()]
        );
        // Harvested identities → one explicit selector each, SORTED.
        let ids: BTreeSet<String> = ["go2", "alpha"].iter().map(|s| s.to_string()).collect();
        assert_eq!(
            demand_get_selectors("/imu", &ids),
            vec![
                "cerulion_q/alpha/demand/imu".to_string(),
                "cerulion_q/go2/demand/imu".to_string(),
            ]
        );
        // A single harvested identity.
        let one: BTreeSet<String> = ["go2"].iter().map(|s| s.to_string()).collect();
        assert_eq!(
            demand_get_selectors("/t", &one),
            vec!["cerulion_q/go2/demand/t".to_string()]
        );
    }

    /// `get_expired_topics` — the composition rule. A GET-granted topic
    /// EXPIRES (is returned for disable) only when it aged past the TTL AND
    /// liveliness is NOT holding it. Fresh grants, and expired-but-liveliness-held
    /// grants, are NEVER returned. Deterministic sorted output. Hand oracle
    /// `Instant`s built by subtracting from a fixed `now`.
    #[test]
    fn get_expired_topics_composition_rule() {
        let ttl = Duration::from_secs(6);
        let now = Instant::now();
        let old = now.checked_sub(Duration::from_secs(10)).unwrap_or(now); // > ttl
        let fresh = now.checked_sub(Duration::from_secs(1)).unwrap_or(now); // < ttl

        let mut last_get: HashMap<String, Instant> = HashMap::new();
        last_get.insert("/expired_alone".to_string(), old); // expired + no liveliness → disable
        last_get.insert("/expired_held".to_string(), old); // expired BUT liveliness holds → keep
        last_get.insert("/fresh".to_string(), fresh); // within TTL → keep

        let mut liveliness: HashSet<String> = HashSet::new();
        liveliness.insert("/expired_held".to_string());

        // Only the expired-AND-liveliness-absent topic is released.
        assert_eq!(
            get_expired_topics(&last_get, &liveliness, now, ttl),
            vec!["/expired_alone".to_string()]
        );

        // With NO liveliness holding, BOTH expired topics release (sorted).
        assert_eq!(
            get_expired_topics(&last_get, &HashSet::new(), now, ttl),
            vec!["/expired_alone".to_string(), "/expired_held".to_string()]
        );

        // A wholly-fresh map releases nothing.
        let mut fresh_only: HashMap<String, Instant> = HashMap::new();
        fresh_only.insert("/a".to_string(), fresh);
        assert!(get_expired_topics(&fresh_only, &HashSet::new(), now, ttl).is_empty());
    }

    /// `GetDemandState` counters + keepalive map — a grant stamps the
    /// topic (granted_topics reflects it) and bumps the grant tally; a refusal
    /// bumps only the refusal tally (never a keepalive — a refused topic must not
    /// hold egress state).
    #[test]
    fn get_demand_state_records_grants_and_refusals() {
        let state = GetDemandState::default();
        assert_eq!(state.grant_count(), 0);
        assert!(state.granted_topics().is_empty());

        // Grants are stamped under a caller-held `last_get` guard via
        // `record_grant_locked`. Counter + keepalive semantics hold as for a
        // one-shot grant (3 grants, 2 distinct topics, re-grant
        // refreshes the stamp without adding a topic).
        {
            let mut last = state
                .last_get
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            state.record_grant_locked(&mut last, "/a");
            state.record_grant_locked(&mut last, "/b");
            state.record_grant_locked(&mut last, "/a"); // re-grant refreshes, no new topic
        }
        assert_eq!(state.grant_count(), 3);
        assert_eq!(
            state.granted_topics(),
            vec!["/a".to_string(), "/b".to_string()]
        );

        state.record_refusal();
        assert_eq!(state.refusal_count(), 1);
        // A refusal never adds a keepalive.
        assert_eq!(
            state.granted_topics(),
            vec!["/a".to_string(), "/b".to_string()]
        );
    }

    /// ACK payloads are the two distinct control tokens the demander
    /// distinguishes.
    #[test]
    fn demand_ack_constants_are_distinct() {
        assert_eq!(ACK_ENABLED, b"enabled");
        assert_eq!(ACK_REFUSED, b"refused");
        assert_ne!(ACK_ENABLED, ACK_REFUSED);
    }

    /// A validated frame whose local re-injection fails is
    /// a COUNTED, LATCHED drop — both the `publish_raw`-Err arm (`Some(e)`) and
    /// the poisoned-mutex arm (`None`) bump `reinject_failure_drops` and share
    /// ONE latch bit (first failure loud, repeats debug), reset on success.
    #[test]
    fn record_reinject_failure_counts_and_latches_once() {
        let counters = IngressCounters::default();
        let latch = IngressWarnLatch::default();
        let e = TransportError::LoanCapacity {
            topic: "/t".to_string(),
        };

        // First failure: counter 1, latch was fresh and is now set.
        record_reinject_failure(&counters, &latch, "/t", Some(&e));
        assert_eq!(counters.reinject_failure_drops.load(Ordering::Relaxed), 1);
        assert!(latch.reinject_failure_warned.load(Ordering::Relaxed));

        // Sustained failures (including the poisoned arm) keep counting under
        // the SAME already-latched bit — the debug-downgrade discipline.
        record_reinject_failure(&counters, &latch, "/t", Some(&e));
        record_reinject_failure(&counters, &latch, "/t", None);
        assert_eq!(counters.reinject_failure_drops.load(Ordering::Relaxed), 3);
        assert!(latch.reinject_failure_warned.load(Ordering::Relaxed));

        // The validation buckets are untouched — the four counters partition
        // received frames (the IngressStats accounting contract).
        assert_eq!(counters.schema_mismatch_drops.load(Ordering::Relaxed), 0);
        assert_eq!(counters.decode_errors.load(Ordering::Relaxed), 0);
        assert_eq!(counters.frames.load(Ordering::Relaxed), 0);

        // Recovery (a successful re-injection) ends the run: reported once,
        // and the NEXT failure re-latches from fresh (warns loudly again).
        assert!(latch.reset());
        record_reinject_failure(&counters, &latch, "/t", None);
        assert_eq!(counters.reinject_failure_drops.load(Ordering::Relaxed), 4);
        assert!(latch.reinject_failure_warned.load(Ordering::Relaxed));
    }
}
