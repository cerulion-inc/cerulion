// SPDX-License-Identifier: AGPL-3.0-only
//! The mirror-plane seam — the boundary between netd's pure
//! refcount plane ([`crate::registry`]) and the actual network mirror.
//!
//! netd's [`MirrorPlane`] does exactly two things: `ensure_mirror` (on the FIRST
//! demand for a `(robot, topic)`) registers ONE shared desk-side ingress bridge —
//! the mirror publisher + the zenoh demand token — so a remote robot's topic
//! re-injects into desk SHM at its CANONICAL name, and `release_mirror` (on the
//! LAST release) tears it down. The production [`GatewayMirrorPlane`] owns the
//! machine's ONE zenoh session (Principle #8) and delegates to
//! [`TransportManager::register_ingress_topic`] — the SAME desk-mirror primitive
//! `cerulion-vizd`'s remote arm already ships. The refcount plane guarantees
//! `ensure_mirror` is called exactly once per stream, which is precisely what
//! dissolves today's two-consumers-collide-on-the-single-writer-slot bug.
//!
//! `ensure_mirror` also registers the mirror's PROVENANCE (via
//! `TransportManager::register_mirror_provenance`) at that same re-injection point
//! — netd IS a re-injector, exactly like connectd/vizd — so `topic list` / Studio
//! fold the netd-mirrored topic into the REMOTE section as `● streaming {robot}`
//! (the "one data source = one topic" decision) rather than surfacing it as a
//! phantom LOCAL topic. Best-effort: a provenance failure never fails the demand.
//!
//! # Mirror identity: one topic, one mirror
//!
//! The mirror is keyed by the CANONICAL TOPIC (not `(robot, topic)`): a consumer
//! subscribing to `/utlidar/robot_odom` must find the mirror at that exact SHM
//! name (the "one wire name per topic, no per-machine aliasing" decision). The
//! `robot` half of a demand is PROVENANCE — it rides the refcount key (for
//! `status` + the C0 `/__cerulion/mirrors` registry) but not the SHM name. If two
//! DIFFERENT robots ever claim one topic name (the "one data source = one topic"
//! decision says a topic name IS the stream, so this shouldn't happen), the second
//! `register_ingress_topic` refuses the already-bridged single-writer slot LOUDLY
//! and the daemon rolls the refcount back — a loud failure, never silent
//! corruption. A dedicated provenance-conflict guard is a C0/C3 refinement.
//!
//! # Per-topic teardown (C2 — this chunk)
//!
//! [`MirrorPlane::release_mirror`] now performs the REAL refcount-0 teardown:
//! [`GatewayMirrorPlane`] calls
//! [`TransportManager::unregister_ingress_topic`](cerulion_core::transport::TransportManager::unregister_ingress_topic)
//! (undeclare the zenoh subscriber + demand token, release the mirror
//! topic's iceoryx2 publisher slot) then best-effort
//! [`unregister_mirror_provenance`](cerulion_core::transport::TransportManager::unregister_mirror_provenance)
//! (C0 — drop the `topic list` REMOTE fold), and returns [`MirrorRelease::Retired`]
//! so the daemon RETIRES the registry entry. A later demand for the same key
//! classifies as a fresh `FirstDemand` and cleanly RE-CREATES the mirror (the full
//! demand→release→re-demand cycle over a real teardown). On the rare teardown
//! FAILURE it returns [`MirrorRelease::Lingering`] (loud) so the bridge is kept
//! reusable rather than half-torn-down.
//!
//! # What C2 still does NOT build
//!
//! - **The driven `GatewayRuntime`** (egress taps + the demand/catalog query
//!   surface) is not started here: a `GatewayRuntime` mandates a `robot_identity`
//!   announce that is wrong for a pure-ingress desk (netd CONSUMES; it is not a
//!   data source — the vizd precedent uses `robot_identity: None`). The egress /
//!   producing role plugs into THIS seam in **C5/C6** (egress + robot convergence).

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use cerulion_core::transport::TransportManager;
use cerulion_core::wire::MaxSliceLen;

use crate::health::{
    BeltConfig, BeltSchedule, HealthConfig, HealthState, HealthTransition, MirrorHealth,
};
use crate::registry::TopicKey;

/// The per-mirror ingress buffer ceiling handed to
/// [`TransportManager::register_ingress_topic`]. DERIVED DIRECTLY from
/// cerulion_core's `graph::config::DEFAULT_MAX_SLICE_LEN` (128 MiB) — the same
/// value the graph build + `cerulion-connectd` use for a re-inject bridge — so it
/// CANNOT drift (it IS that constant, narrowed to the `MaxSliceLen` u32 width in a
/// const context; 128 MiB < 2^32).
const MIRROR_MAX_SLICE_LEN: MaxSliceLen =
    MaxSliceLen::const_new(cerulion_core::graph::config::DEFAULT_MAX_SLICE_LEN as u32);

/// An error from a [`MirrorPlane`] operation. The daemon maps it to a structured
/// protocol error and (for `ensure_mirror`) rolls the refcount back.
#[derive(Debug)]
pub enum MirrorError {
    /// The underlying transport refused the mirror registration — no network
    /// configured, the zenoh session could not open, the schema/single-writer slot
    /// is taken (e.g. a different robot already mirrors this topic), or a
    /// double-registration. Carries the offending key + the transport cause.
    ///
    /// The `TransportError` is BOXED: it is a large enum, and a bare
    /// `Result<(), MirrorError>` would trip clippy's `result_large_err` and bloat
    /// every mirror-op stack frame.
    Register {
        /// The `(robot, topic)` whose mirror failed.
        key: TopicKey,
        /// The underlying transport error.
        source: Box<cerulion_core::TransportError>,
    },
    /// The iroh WAN plane could not establish the mirror: the robot
    /// REFUSED the wire plane (unpaired desk key / unclaimed robot / missing
    /// `CAP_OBSERVE`), was UNREACHABLE (dial timeout / no route), rejected the
    /// topic demand, or the desk has NO WAN endpoint configured for the robot.
    /// Carries the offending key + an actionable reason (the robot's own refusal
    /// text where applicable). A distinct variant from [`Self::Register`] because a
    /// WAN failure is NOT a zenoh `TransportError` — it is a dial/demand outcome.
    #[cfg(feature = "wan")]
    Iroh {
        /// The `(robot, topic)` whose iroh WAN mirror failed.
        key: TopicKey,
        /// The actionable failure reason (dial/refusal/demand detail).
        reason: String,
    },
}

impl MirrorError {
    /// The `(robot, topic)` this error concerns.
    pub fn key(&self) -> &TopicKey {
        match self {
            MirrorError::Register { key, .. } => key,
            #[cfg(feature = "wan")]
            MirrorError::Iroh { key, .. } => key,
        }
    }
}

impl fmt::Display for MirrorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MirrorError::Register { key, source } => write!(
                f,
                "failed to register the network mirror for robot='{}' topic='{}': {source}",
                key.robot, key.topic
            ),
            #[cfg(feature = "wan")]
            MirrorError::Iroh { key, reason } => write!(
                f,
                "failed to establish the iroh WAN mirror for robot='{}' topic='{}': {reason}",
                key.robot, key.topic
            ),
        }
    }
}

impl std::error::Error for MirrorError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            MirrorError::Register { source, .. } => Some(source.as_ref()),
            // `Iroh` carries a flattened reason string (no nested std::error source).
            #[cfg(feature = "wan")]
            MirrorError::Iroh { .. } => None,
        }
    }
}

/// Whether a [`MirrorPlane::release_mirror`] actually tore the physical bridge
/// down — the daemon uses this to keep the registry TRUTHFUL.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MirrorRelease {
    /// The physical bridge was TORN DOWN — the daemon retires the registry entry
    /// (`DemandRegistry::retire`). C2's real teardown returns this on success; a
    /// later demand for the same key is a fresh `FirstDemand` that re-creates the
    /// mirror.
    Retired,
    /// The physical bridge was KEPT — the registry entry LINGERS (`mirror_present`,
    /// refcount 0) so a re-demand reuses the existing bridge WITHOUT re-ensuring,
    /// and the bridge is reclaimed at the process idle self-exit. In C2 this is the
    /// teardown-FAILURE fallback: `unregister_ingress_topic` errored, so the bridge
    /// may still be live and re-ensuring it would be refused — keeping it reusable
    /// is the safe path. (A spy/test plane with no real transport also returns this
    /// to exercise the daemon's lingering path.)
    Lingering,
}

/// The seam between netd's refcount plane and the network mirror. The daemon calls
/// `ensure_mirror` on the FIRST demand for a `(robot, topic)` and `release_mirror`
/// on the LAST release. `Send + Sync`: shared across the daemon's per-connection +
/// idle-watch threads.
pub trait MirrorPlane: Send + Sync {
    /// Ensure the shared desk mirror for `key` exists, validating inbound frames
    /// against `schema_hash`. Called ONLY on a genuine first ensure — the refcount
    /// plane guarantees this fires exactly ONCE per live mirror (a re-demand of a
    /// LINGERING mirror reuses it WITHOUT re-ensuring — see [`crate::registry`]), so
    /// an implementation need NOT be idempotent. **In fact the production
    /// [`GatewayMirrorPlane`]'s `register_ingress_topic` REFUSES a re-registration
    /// with a hard error**, which is precisely why the refcount plane's
    /// single-ensure + lingering-reuse guarantee is load-bearing: a double-ensure
    /// would surface as a `MirrorError`, not be silently absorbed.
    fn ensure_mirror(&self, key: &TopicKey, schema_hash: u64) -> Result<(), MirrorError>;

    /// Release the shared mirror for `key`, called on the LAST release. Returns
    /// whether the physical bridge was actually retired ([`MirrorRelease::Retired`])
    /// or KEPT ([`MirrorRelease::Lingering`]). **C2**: the production
    /// [`GatewayMirrorPlane`] tears the real bridge down and returns `Retired` on
    /// success (`Lingering` only on the rare teardown failure).
    fn release_mirror(&self, key: &TopicKey) -> MirrorRelease;
}

/// A running mirror-health watch thread. Dropping it stops the thread
/// (flip the flag, join). Owned by [`GatewayMirrorPlane`] behind a `Mutex<Option>`
/// and started LAZILY on the first `ensure_mirror`, so a spawned-but-never-demanded
/// netd starts no health thread.
struct HealthWatch {
    shutdown: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl Drop for HealthWatch {
    fn drop(&mut self) {
        self.shutdown.store(true, Ordering::SeqCst);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// The production mirror plane: owns the machine's ONE network-configured
/// [`TransportManager`] (the single zenoh session — Principle #8) and registers a
/// shared ingress bridge per demanded topic.
///
/// `main.rs` builds the `manager` via [`TransportManager::init`]; the iox2 boot
/// test via `TransportManager::init_for_test`. The session opens LAZILY inside the
/// first `register_ingress_topic`, so a netd that is spawned but never demanded
/// opens no zenoh session at all (then self-exits on idle).
///
/// # Self-heal
///
/// Each `ensure_mirror` additionally arms the demand-GET KEEPALIVE for the
/// topic ([`TransportManager::ensure_ingress_demand_keepalive`]) — the always-on
/// re-affirmation that re-crosses a restarted robot gateway's fresh link so the
/// mirror resumes streaming INVISIBLY (no client re-demand). It also runs a HEALTH
/// WATCH thread (started lazily on the first demand) that samples each mirror's
/// re-injected frame count: a mirror gone stale is LOUD (`warn!` degraded →
/// `info!` recovered) and the belt re-affirms demand PROMPTLY. Together they close
/// the silent black hole a restarted gateway's fresh link would otherwise leave open.
pub struct GatewayMirrorPlane {
    manager: Arc<TransportManager>,
    /// The canonical topics currently mirrored — the health watch's iteration set,
    /// grown on `ensure_mirror` and shrunk on `release_mirror` (Principle #3).
    active: Arc<Mutex<BTreeSet<String>>>,
    /// Per-topic OBSERVABLE health (Principle #3) — the health thread publishes the
    /// current [`HealthState`] here each poll; [`Self::mirror_health`] reads it. The
    /// restart e2e asserts the DEGRADED→HEALTHY transition off this, cross-thread
    /// (the `warn!`/`info!` logging is pinned separately, on the test thread, by
    /// `log_transition`'s `#[traced_test]` unit).
    health_states: Arc<Mutex<BTreeMap<String, HealthState>>>,
    /// Principle #3: the count of PROMPT demand-reaffirm belt
    /// passes the health thread has run — observable so a test can pin that the
    /// EDGE-TRIGGERED belt actually fired (and, mutation-style, did not fire per
    /// poll). Distinct from the always-on background keepalive loop's passes.
    belt_passes: Arc<AtomicU64>,
    /// Health-watch timings (production defaults; short in tests).
    health_config: HealthConfig,
    /// The edge-triggered belt backoff bounds (finding B; default burst).
    belt_config: BeltConfig,
    /// The lazily-started health-watch thread (started on the first demand).
    watch: Mutex<Option<HealthWatch>>,
}

impl GatewayMirrorPlane {
    /// Wrap an already-initialized (network-configured) transport manager. The
    /// manager MUST carry a network transport for `ensure_mirror` to succeed; a
    /// local-only (`CERULION_NETD_NETWORK=off`) manager surfaces the explicit
    /// "not network-configured" error per demand. Production health-watch timings.
    pub fn new(manager: Arc<TransportManager>) -> Self {
        Self::with_health_config(manager, HealthConfig::default())
    }

    /// Like [`Self::new`] but with explicit health-watch timings — the restart e2e
    /// uses a short `degrade_after` + `poll` so the degrade/recover cycle is fast.
    pub fn with_health_config(manager: Arc<TransportManager>, health_config: HealthConfig) -> Self {
        Self {
            manager,
            active: Arc::new(Mutex::new(BTreeSet::new())),
            health_states: Arc::new(Mutex::new(BTreeMap::new())),
            belt_passes: Arc::new(AtomicU64::new(0)),
            health_config,
            belt_config: BeltConfig::default(),
            watch: Mutex::new(None),
        }
    }

    /// The wrapped transport manager (diagnostics / the daemon's shutdown drop).
    pub fn manager(&self) -> &Arc<TransportManager> {
        &self.manager
    }

    /// Principle #3: the last-sampled [`HealthState`] of `topic`'s mirror,
    /// or `None` before the health thread has sampled it (or after teardown). The
    /// restart e2e polls this for the DEGRADED→HEALTHY self-heal transition —
    /// cross-thread-observable state, no log scraping.
    pub fn mirror_health(&self, topic: &str) -> Option<HealthState> {
        self.health_states
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .get(topic)
            .copied()
    }

    /// Principle #3: the number of PROMPT demand-reaffirm belt
    /// passes the health thread has run — the observable that pins the edge-triggered
    /// belt as load-bearing (it fires on a degraded edge, not per poll).
    pub fn belt_pass_count(&self) -> u64 {
        self.belt_passes.load(Ordering::Relaxed)
    }

    /// Lock the active-topic set (poison-tolerant — a panicked caller must never
    /// wedge the plane; the set under the lock is plain data).
    fn lock_active(&self) -> std::sync::MutexGuard<'_, BTreeSet<String>> {
        self.active.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Ensure the health-watch thread is running (started lazily on the first
    /// demand). Idempotent — a second call while a thread runs is a no-op.
    fn ensure_health_watch(&self) {
        let mut guard = self.watch.lock().unwrap_or_else(PoisonError::into_inner);
        if guard.is_some() {
            return;
        }
        let shutdown = Arc::new(AtomicBool::new(false));
        let manager = Arc::clone(&self.manager);
        let active = Arc::clone(&self.active);
        let states = Arc::clone(&self.health_states);
        let belt_passes = Arc::clone(&self.belt_passes);
        let config = self.health_config;
        let belt_config = self.belt_config;
        let stop = Arc::clone(&shutdown);
        let handle = std::thread::Builder::new()
            .name("netd-mirror-health".to_string())
            .spawn(move || {
                health_watch_loop(HealthLoopArgs {
                    manager,
                    active,
                    states,
                    belt_passes,
                    config,
                    belt_config,
                    shutdown: stop,
                })
            })
            .ok(); // a spawn failure is non-fatal — the keepalive loop still self-heals.
        if handle.is_none() {
            tracing::warn!(
                "netd: could not spawn the mirror health-watch thread — mirrors still self-heal \
                 via the demand keepalive, but degraded/recovered transitions will not be logged"
            );
        }
        *guard = Some(HealthWatch { shutdown, handle });
    }
}

impl MirrorPlane for GatewayMirrorPlane {
    fn ensure_mirror(&self, key: &TopicKey, schema_hash: u64) -> Result<(), MirrorError> {
        // Register the shared ingress bridge at the CANONICAL topic name (mirror
        // by topic — the `robot` is provenance, see the module docs). This creates
        // the single mirror publisher + declares the zenoh demand token; the
        // session opens lazily on the first call.
        self.manager
            .register_ingress_topic(&key.topic, schema_hash, MIRROR_MAX_SLICE_LEN)
            .map_err(|source| MirrorError::Register {
                key: key.clone(),
                source: Box::new(source),
            })?;
        tracing::info!(
            robot = %key.robot,
            topic = %key.topic,
            "netd: shared network mirror registered (first demand)"
        );

        // Register mirror PROVENANCE at the re-injection point (right
        // after the ingress publisher is created — the connectd/vizd re-injector
        // pattern), so `topic list` / Studio FOLD this netd-mirrored topic into the
        // REMOTE section as `● streaming {robot}` (the "one data source = one topic"
        // decision) instead of surfacing the re-injected SHM service as a phantom
        // LOCAL topic. netd IS the re-injector, so the provenance is its job — not a
        // consumer-migration (C3) concern. Pass the SAME canonical `key.topic` the
        // ingress publisher used for its `{topic}/data` service (raw == canonical
        // for netd — it always mirrors under `/…`). Best-effort: a provenance
        // failure logs a warn but NEVER fails the demand — the mirror still streams;
        // it just would not fold into REMOTE.
        if let Err(e) = self
            .manager
            .register_mirror_provenance(&key.topic, &key.robot)
        {
            tracing::warn!(
                robot = %key.robot, topic = %key.topic, error = %e,
                "netd: could not register mirror provenance — the mirror still streams, but \
                 `topic list` will show it as LOCAL rather than REMOTE from this robot"
            );
        }

        // Arm the SELF-HEAL demand-GET keepalive for this topic. Without
        // it, `register_ingress_topic` above declares only a ONE-SHOT liveliness
        // demand token — which does NOT reliably re-cross a restarted robot
        // gateway's fresh link (zenoh 1.8 interest asymmetry), the silent black
        // hole this fixes. The keepalive re-affirms egress via the dialer→accepter
        // demand GET (the wire direction that DOES cross a reconnected link) so the
        // mirror resumes INVISIBLY when the robot returns. Best-effort: a keepalive
        // start failure warns but never fails the demand (the mirror still works
        // now; it just would not auto-heal after a restart).
        if let Err(e) = self.manager.ensure_ingress_demand_keepalive(&key.topic) {
            tracing::warn!(
                robot = %key.robot, topic = %key.topic, error = %e,
                "netd: could not arm the demand-GET keepalive — the mirror streams now but may \
                 NOT self-heal if the robot's gateway restarts"
            );
        }

        // Track the topic for the health watch + start the watch thread (lazy).
        self.lock_active().insert(key.topic.clone());
        self.ensure_health_watch();
        Ok(())
    }

    fn release_mirror(&self, key: &TopicKey) -> MirrorRelease {
        // The REAL refcount-0 teardown. Undeclare the zenoh data
        // subscriber + demand token and release the mirror topic's single iceoryx2
        // publisher slot (so a re-demand cleanly RE-CREATES the mirror), via the
        // shared `unregister_ingress_topic` primitive. The daemon calls this
        // OFF the registry lock (the zenoh undeclare can BLOCK, and holding the
        // registry lock across it wedged the daemon) — same-key teardown
        // serialization is now provided by the registry's per-entry `tearing` state
        // (`claim_tearing`), NOT the lock: exactly one caller wins the claim and
        // runs this, and while it runs every demand for the key returns `Retiring`
        // (waits). So a re-register of the just-torn-down slot cannot race a demand,
        // and this never runs concurrently for one key.
        match self.manager.unregister_ingress_topic(&key.topic) {
            Ok(()) => {
                // Best-effort C0 provenance cleanup: drop the mirror from this
                // process's provenance set so `topic list` / Studio stop folding it
                // into REMOTE. A failure here is non-fatal — the mirror IS torn
                // down; a stale REMOTE row would at worst persist until process
                // exit (the republish stops when the manager drops).
                if let Err(e) = self.manager.unregister_mirror_provenance(&key.topic) {
                    tracing::warn!(
                        robot = %key.robot, topic = %key.topic, error = %e,
                        "netd: mirror torn down but could not remove its provenance — `topic list` \
                         may still show it as REMOTE from this robot until netd exits"
                    );
                }
                // The topic is genuinely GONE — drop it from the demand
                // keepalive set (the loop stops GETting it next pass) and the
                // health-watch iteration set. Only on Retired: a Lingering
                // (teardown-failure) bridge is KEPT reusable, so its self-heal must
                // keep running.
                self.manager.remove_ingress_demand_keepalive(&key.topic);
                self.lock_active().remove(&key.topic);
                tracing::info!(
                    robot = %key.robot,
                    topic = %key.topic,
                    "netd: shared network mirror torn down (last demand released)"
                );
                MirrorRelease::Retired
            }
            Err(e) => {
                // Teardown FAILED — the physical bridge may still be live, so a
                // re-ensure would be refused (a topic already has a registered
                // ingress bridge). Keep the entry LINGERING so a re-demand REUSES
                // the bridge instead. Loud (Principle #3): the un-retired bridge is
                // reclaimed only at idle self-exit.
                tracing::error!(
                    robot = %key.robot,
                    topic = %key.topic,
                    error = %e,
                    "netd: network mirror teardown FAILED — the bridge LINGERS (reused on re-demand) \
                     until idle self-exit drops the whole session"
                );
                MirrorRelease::Lingering
            }
        }
    }
}

/// Arguments to [`health_watch_loop`] — grouped into a struct so the loop stays
/// under clippy's argument-count bound and the shared handles read by name.
struct HealthLoopArgs {
    manager: Arc<TransportManager>,
    active: Arc<Mutex<BTreeSet<String>>>,
    states: Arc<Mutex<BTreeMap<String, HealthState>>>,
    belt_passes: Arc<AtomicU64>,
    config: HealthConfig,
    belt_config: BeltConfig,
    shutdown: Arc<AtomicBool>,
}

/// The mirror health-watch loop (runs on the `netd-mirror-health` thread
/// until the plane drops). Every `config.poll` it samples each active mirror's
/// re-injected frame count ([`TransportManager::ingress_stats`]), feeds the pure
/// [`MirrorHealth`] machine, and LOUDLY logs the DEGRADED (`warn!`) → RECOVERED
/// (`info!`) edges so a stale mirror is never a silent black hole.
///
/// On a fresh Healthy→Degraded EDGE it drives a bounded backoff BURST of prompt
/// demand re-affirm passes (a [`BeltSchedule`], finding B) — NOT one pass per poll
/// (which stormed forever for a by-design-quiet latched mirror); the always-on 2 s
/// background keepalive loop is the sustained re-affirm. Each belt pass is
/// SHUTDOWN-ABORTABLE (finding A): it threads the loop's `shutdown` flag as its
/// abort closure so a plane drop mid-pass returns after the current GET.
fn health_watch_loop(args: HealthLoopArgs) {
    let HealthLoopArgs {
        manager,
        active,
        states,
        belt_passes,
        config,
        belt_config,
        shutdown,
    } = args;
    // Per-topic health, pruned when a topic leaves the active set. `dampened`
    // records whether each topic's CURRENT degrade regime is a flap (finding C).
    let mut health: BTreeMap<String, MirrorHealth> = BTreeMap::new();
    let mut dampened: BTreeMap<String, bool> = BTreeMap::new();
    let mut belt = BeltSchedule::new(Instant::now(), belt_config);
    while !shutdown.load(Ordering::Acquire) {
        if sleep_with_shutdown(config.poll, &shutdown) {
            return;
        }
        let now = Instant::now();
        let topics: Vec<String> = active
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .iter()
            .cloned()
            .collect();
        // Prune per-topic maps + observable state for topics no longer mirrored.
        health.retain(|t, _| topics.contains(t));
        dampened.retain(|t, _| topics.contains(t));
        {
            let mut st = states.lock().unwrap_or_else(PoisonError::into_inner);
            st.retain(|t, _| topics.contains(t));
        }

        let mut new_degraded_edge = false;
        for topic in &topics {
            // A torn-down / not-yet-registered stat reads as 0 frames — the machine
            // treats "no stats" the same as "no progress" (both are the absence of
            // fresh data, which is exactly what DEGRADED means).
            let frames = manager
                .network()
                .and_then(|n| n.ingress_stats(topic))
                .map(|s| s.frames)
                .unwrap_or(0);
            let entry = health
                .entry(topic.clone())
                .or_insert_with(|| MirrorHealth::new(frames, now, config.degrade_after));
            let transition = entry.observe(frames, now);
            // Classify a fresh degrade as a FLAP (dampened) at the edge,
            // and remember it so the matching RECOVERED is dampened symmetrically.
            let is_flap = match transition {
                HealthTransition::Degraded => {
                    let flap = entry.degrade_is_flap(now, config.flap_window);
                    dampened.insert(topic.clone(), flap);
                    new_degraded_edge = true; // a genuine new degrade drives the belt
                    flap
                }
                HealthTransition::Recovered => dampened.get(topic).copied().unwrap_or(false),
                _ => dampened.get(topic).copied().unwrap_or(false),
            };
            log_transition(topic, transition, entry.stalled_for(now), frames, is_flap);
            // Publish the OBSERVABLE current state (Principle #3) for `mirror_health`.
            states
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .insert(topic.clone(), entry.state());
        }

        // The belt is EDGE-TRIGGERED with bounded backoff, NOT per-poll.
        // A fresh Degraded edge (re)arms a small burst; the schedule then fires at
        // growing intervals and goes quiet (the 2 s background keepalive sustains).
        if belt.poll(new_degraded_edge, now) {
            belt_passes.fetch_add(1, Ordering::Relaxed);
            // Pass the loop's shutdown flag as the abort closure so a
            // plane drop mid-pass returns after the current GET (each GET times out
            // FULLY while the robot is down, so an un-abortable pass would stall the
            // Drop join for seconds at Go2 topic scale).
            let abort = || shutdown.load(Ordering::Acquire);
            if let Err(e) = manager.run_ingress_demand_get_pass(&abort) {
                tracing::debug!(error = %e, "netd: prompt demand re-affirm pass failed (transient)");
            }
        }
    }
}

/// The health-transition LOG-LEVEL mapping — split out so it is pinnable
/// on the calling thread by a scoped-subscriber unit (the health loop runs on the
/// `netd-mirror-health` thread, whose events a thread-local subscriber cannot
/// capture). DEGRADED is a loud `warn!` (the robot is likely down), RECOVERED a
/// loud `info!` (the self-heal was invisible to consumers), sustained degradation a
/// `debug!` (no flood while a robot stays down), and healthy-steady is silent.
///
/// `dampened` downgrades the DEGRADED/RECOVERED loud logs to `debug!`
/// for a FLAPPING topic (a healthy low-rate publisher whose gap exceeds
/// `degrade_after`) — the FIRST degrade stays loud, sustained flaps go quiet.
fn log_transition(
    topic: &str,
    transition: HealthTransition,
    stalled: Duration,
    frames: u64,
    dampened: bool,
) {
    let stalled_ms = stalled.as_millis() as u64;
    match transition {
        HealthTransition::None => {}
        HealthTransition::Degraded if dampened => tracing::debug!(
            topic = %topic,
            stalled_ms,
            "netd: mirror degraded again (flapping — a low-rate/bursty topic; re-affirming demand)"
        ),
        HealthTransition::Degraded => tracing::warn!(
            topic = %topic,
            stalled_ms,
            frames,
            "netd: mirror DEGRADED — no new frames; the robot's gateway is likely down/restarting. \
             Re-affirming demand (the mirror self-heals when it returns, no client re-demand needed)"
        ),
        HealthTransition::StillDegraded => tracing::debug!(
            topic = %topic,
            stalled_ms,
            "netd: mirror still degraded — belt burst re-affirming demand"
        ),
        HealthTransition::Recovered if dampened => tracing::debug!(
            topic = %topic,
            frames,
            "netd: mirror recovered (flapping low-rate topic)"
        ),
        HealthTransition::Recovered => tracing::info!(
            topic = %topic,
            frames,
            "netd: mirror RECOVERED — frames resumed (self-healed; consumers saw no re-demand)"
        ),
    }
}

/// Sleep `dur` in ≤50 ms slices, returning `true` early if `shutdown` flips — so
/// the health thread joins PROMPTLY at plane drop rather than after a full poll.
fn sleep_with_shutdown(dur: Duration, shutdown: &AtomicBool) -> bool {
    const SLICE: Duration = Duration::from_millis(50);
    let deadline = Instant::now() + dur;
    loop {
        if shutdown.load(Ordering::Acquire) {
            return true;
        }
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return shutdown.load(Ordering::Acquire);
        }
        std::thread::sleep(SLICE.min(remaining));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A thread-safe in-memory writer for a SCOPED `tracing_subscriber::fmt`
    /// subscriber — captures log lines synchronously into a shared buffer. Used to
    /// pin `log_transition`'s LOG-LEVEL mapping WITHOUT a process-global subscriber
    /// (`log_transition` is called on THIS thread, so a thread-local `with_default`
    /// dispatcher captures it deterministically). Crib of the netd integration
    /// test's `SharedBuf`.
    #[derive(Clone)]
    struct SharedBuf(Arc<Mutex<Vec<u8>>>);
    impl std::io::Write for SharedBuf {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.0.lock().unwrap().extend_from_slice(buf);
            Ok(buf.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }
    impl tracing_subscriber::fmt::MakeWriter<'_> for SharedBuf {
        type Writer = SharedBuf;
        fn make_writer(&self) -> Self::Writer {
            self.clone()
        }
    }

    /// `log_transition` maps each health transition to the RIGHT level —
    /// DEGRADED → loud `WARN`, RECOVERED → loud `INFO`, StillDegraded → `DEBUG`
    /// (no flood), None → silent. Captured on this thread via a scoped subscriber
    /// (the health thread's events are uncapturable by a thread-local, hence the
    /// factored fn). Hand oracle on the level + marker string.
    #[test]
    fn log_transition_maps_each_health_edge_to_the_right_level() {
        let buf = SharedBuf(Arc::new(Mutex::new(Vec::new())));
        let subscriber = tracing_subscriber::fmt()
            .with_writer(buf.clone())
            .with_max_level(tracing::Level::TRACE)
            .without_time()
            .finish();
        tracing::subscriber::with_default(subscriber, || {
            log_transition("/tf", HealthTransition::None, Duration::ZERO, 10, false);
            log_transition(
                "/tf",
                HealthTransition::Degraded,
                Duration::from_millis(5000),
                10,
                false,
            );
            log_transition(
                "/tf",
                HealthTransition::StillDegraded,
                Duration::from_millis(7000),
                10,
                false,
            );
            log_transition(
                "/tf",
                HealthTransition::Recovered,
                Duration::ZERO,
                11,
                false,
            );
        });
        let out = String::from_utf8_lossy(&buf.0.lock().unwrap()).to_string();

        // DEGRADED is a WARN carrying the loud marker.
        assert!(
            out.lines()
                .any(|l| l.contains("WARN") && l.contains("mirror DEGRADED")),
            "degraded is a loud WARN; captured:\n{out}"
        );
        // RECOVERED is an INFO carrying the loud marker.
        assert!(
            out.lines()
                .any(|l| l.contains("INFO") && l.contains("mirror RECOVERED")),
            "recovered is a loud INFO; captured:\n{out}"
        );
        // Sustained degradation is a DEBUG (flood-suppressed), NOT a second WARN.
        assert!(
            out.lines()
                .any(|l| l.contains("DEBUG") && l.contains("still degraded")),
            "sustained degradation is a DEBUG; captured:\n{out}"
        );
        assert_eq!(
            out.matches("mirror DEGRADED").count(),
            1,
            "the loud DEGRADED edge fires exactly ONCE (not per sustained poll); captured:\n{out}"
        );
        // None is silent.
        assert!(
            !out.contains("mirror HEALTHY"),
            "the healthy-steady None arm emits nothing"
        );
    }

    /// A FLAPPING topic's degrade/recover downgrade to DEBUG
    /// (dampened), so a healthy low-rate publisher never emits recurring loud noise.
    #[test]
    fn log_transition_dampens_a_flapping_topic_to_debug() {
        let buf = SharedBuf(Arc::new(Mutex::new(Vec::new())));
        let subscriber = tracing_subscriber::fmt()
            .with_writer(buf.clone())
            .with_max_level(tracing::Level::TRACE)
            .without_time()
            .finish();
        tracing::subscriber::with_default(subscriber, || {
            // dampened = true → the loud markers must NOT appear at WARN/INFO.
            log_transition(
                "/tf_static",
                HealthTransition::Degraded,
                Duration::from_millis(5000),
                3,
                true,
            );
            log_transition(
                "/tf_static",
                HealthTransition::Recovered,
                Duration::ZERO,
                4,
                true,
            );
        });
        let out = String::from_utf8_lossy(&buf.0.lock().unwrap()).to_string();
        assert!(
            !out.contains("WARN"),
            "a flapping degrade is dampened to DEBUG, never WARN; captured:\n{out}"
        );
        assert!(
            !out.contains("INFO"),
            "a flapping recover is dampened to DEBUG, never INFO; captured:\n{out}"
        );
        assert!(
            out.contains("flapping"),
            "the dampened lines name the flap; captured:\n{out}"
        );
    }

    #[test]
    fn mirror_error_display_names_the_key_and_cause() {
        let err = MirrorError::Register {
            key: TopicKey::new("ubuntu", "/utlidar/robot_odom"),
            source: Box::new(cerulion_core::TransportError::InvalidTransportConfig {
                reason: "no network".to_string(),
            }),
        };
        let s = err.to_string();
        assert!(s.contains("ubuntu"), "names the robot: {s}");
        assert!(s.contains("/utlidar/robot_odom"), "names the topic: {s}");
        assert!(s.contains("no network"), "carries the cause: {s}");
        // The accessor points at the offending key.
        assert_eq!(err.key().topic, "/utlidar/robot_odom");
        // std::error::Error::source is wired.
        assert!(std::error::Error::source(&err).is_some());
    }
}
