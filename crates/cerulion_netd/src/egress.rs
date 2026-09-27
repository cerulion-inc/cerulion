// SPDX-License-Identifier: AGPL-3.0-only
//! The egress-plane seam — the boundary between netd's pure
//! refcount registry ([`crate::registry`]) and the machine's ONE network egress
//! path.
//!
//! This is "egress convergence": a desk graph pushes its produced-topic egress plan
//! into netd (over the [`crate::protocol`] `register_egress` verb) so ONE zenoh
//! session serves the machine's WHOLE network plane — the ingress mirrors netd's
//! [`crate::mirror::MirrorPlane`] already owns AND its graphs' egress — instead of
//! every producing graph spawning its own per-run gateway (a Principle-#8
//! violation). This module holds
//! the seam + the shared embedded gateway.
//!
//! # The production plane owns ONE embedded gateway
//!
//! [`GatewayEgressPlane`] drives a single embedded
//! [`GatewayRuntime`] over the SAME
//! [`TransportManager`] the mirror plane owns (Principle #8). It boots LAZILY on the
//! FIRST egress registration (via the additive
//! [`GatewayRuntime::new_embedded`] —
//! empty announce, permissive `AllowAll` desk posture) and then GROWS the egress set
//! at runtime: each registered plan's produced topics are pushed onto the running
//! gateway via [`TransportManager::register_dynamic_egress_topic`], which the
//! gateway's runtime-registration reader picks up on its next drive pass. So the
//! desk announces nothing until its first graph produces (the requirement:
//! "activated when the first egress plan registers"), and the pure-ingress desk is
//! byte-identical (no gateway thread, no announce) when no egress plan is registered.
//!
//! On a LISTEN-configured machine (`CERULION_NETD_LISTEN` set, i.e. a
//! robot serving the network) the daemon ADDITIONALLY boots this same gateway AT
//! DAEMON START via [`GatewayEgressPlane::boot_standing_gateway`], with an EMPTY
//! announce set under the same `AllowAll` posture. Without it, the only production
//! boots would be `graph run`'s per-run child and this plane's first `register_egress`
//! — so a pure-ROS robot (rmw publishers + netd, no graph run ever) would have no zenoh
//! session, no announce, no catalog, and no mDNS beacon, and its runtime-registered
//! topics would be unreachable from every networked Cerulion host. The start boot opens
//! the session (binding the LISTEN locator, so the beacon's port-bound precondition
//! holds) and starts the query surface + reconciler for the empty permissive plan;
//! its reg-channel reader then picks up runtime registrations (rmw publishers, raw
//! routes) exactly as it does registered ones. A DESK (no LISTEN) gets
//! no standing boot: nothing boots until the first egress plan registers.
//!
//! # Identity
//!
//! The embedded gateway announces under this MACHINE's identity (the hostname — the
//! standing decision that the hostname is the robot identity), which
//! [`crate::net`] resolves into the shared manager's `NetworkConfig.robot_identity`
//! at init. (Because that identity is immutable at `TransportManager::init` and the
//! session is shared, it is carried from init rather than truly minted on first
//! plan; the ANNOUNCE-level activation IS lazy — nothing announces until this plane
//! boots the gateway. See [`crate::net`] for the design tension.)
//!
//! # The cross-plan loop guard is netd's, not the plane's
//!
//! On the ONE shared session a topic mirrored IN by an ingress demand must never be
//! announced OUT by egress (the re-inject → tap → egress → re-inject echo
//! loop), and vice versa. cerulion_core already prevents the PHYSICAL loop (the
//! permissive probe + the live demand-grant path exclude any `is_ingress_registered`
//! / `self_ingress` topic), but netd ADDS a LOUD, EARLY refusal at the control seam:
//! [`DemandRegistry::register_egress`](crate::registry::DemandRegistry::register_egress)
//! refuses a plan whose topic is demanded, and `demand` refuses a topic that is
//! egress-registered — the two sets are mutually exclusive by construction. So this
//! plane never even sees a conflicting topic.
//!
//! # What this plane does NOT do
//!
//! - **Physical egress RETRACTION on release.** The reg-channel + gateway announce
//!   set are ADD-ONLY (no removal primitive exists, the egress twin of the ingress side's
//!   `unregister_ingress_topic`). So on release the CONNECTION-SCOPED accounting is
//!   dropped (liveness + the loop guard stay truthful), but the physical announce
//!   LINGERS on the shared gateway until netd's idle self-exit drops the whole
//!   session — the same lingering a mirror shows when its teardown fails. Physical
//!   retraction is not implemented.
//! - **Per-client / per-computer egress SCOPING (the AllowAll exposure).** The
//!   embedded gateway boots PERMISSIVE (`AllowAll`), which installs a permissive SHM
//!   probe. That probe serves a remote demand-by-name for ANY live local SHM topic
//!   (except one the shared manager has registered as ingress — the loop exclusion),
//!   NOT only the topics a plan registered. So a LAN peer can demand a live
//!   local topic that NO graph pushed an egress plan for. That
//!   is exactly the permissive-desk default (`graph run` with no
//!   `network:` block already exposes every produced topic) — but per-graph
//!   isolation (graph A's demand must not reach graph B's topic) is NOT provided.
//!   Constraining the embedded gateway to the union of plan-registered topics is
//!   STRUCTURALLY INFEASIBLE in the current [`GatewayRuntime`] without a multi-part
//!   core change: (a) an empty-boot `AllowList` gateway starts NO demand surface
//!   (the gateway's `!egress_flags.is_empty() || permissive` gate), so runtime
//!   registrations would never become demandable; (b) `register_runtime_topic` adds
//!   the bridge FLAG + announce but does NOT add the topic to the egress allow-list,
//!   so a runtime topic under `AllowList` would not egress; and (c) the permissive
//!   probe (the only path that serves an unregistered live topic) exists only under
//!   `AllowAll`. Closing this needs a GatewayRuntime boot-gate + probe inclusion-set
//!   change. The **account-scoped pairing gate** bounds it from the other side (the
//!   LAN plane trends to existence-beacon-only and topic DATA rides the paired
//!   identity plane — so a remote demand is account-gated before any topic, scoped or
//!   not, is served). The exposure is the documented desk default, chosen by design.
//! - **The CLI push side** (`graph run` registering its plan, and its per-run
//!   gateway fallback) lives in `cerulion_cli_engine`, not here.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::thread::JoinHandle;

use cerulion_core::transport::TransportManager;
use cerulion_core::{
    GatewayPlan, GatewayRuntime, IceoryxShmIdentity, SchemaServing, TransportError,
};

use crate::net::LISTEN_ENV;
use crate::registry::{canonical_topic, ConnId};

/// The per-client identity of an egress registration — the netd connection that
/// pushed the plan (also the [`crate::registry`] scope key). Passed to the plane so
/// a spy can count per client and a future retraction can key by it.
pub type EgressId = ConnId;

/// An error from an [`EgressPlane`] operation. The daemon maps it to a structured
/// protocol error and rolls the registry's egress accounting back.
#[derive(Debug)]
pub enum EgressError {
    /// The shared embedded gateway could not boot, or a produced topic could not be
    /// pushed onto it — carries the underlying transport cause. Boxed because
    /// `TransportError` is a large enum (clippy `result_large_err`).
    Gateway {
        /// The underlying transport error.
        source: Box<TransportError>,
    },
    /// The producing run forwarded an iceoryx2 SHM-discovery identity
    /// (root/prefix + the service & node directories + the per-file service
    /// suffixes) that does NOT match netd's shared session, so netd's embedded
    /// gateway — which taps SHM on ITS namespace — could not see the run's topics.
    /// The registration is refused so the run falls back to a per-run gateway child
    /// on its OWN namespace rather than being silently tapped on the wrong one
    /// (which would egress nothing). Carries both identities for the LOUD
    /// diagnostic — boxed because [`IceoryxShmIdentity`] is a wide struct (nine
    /// `String`s) and `EgressError` is a `Result`-carried error (clippy
    /// `result_large_err`), matching the boxed [`TransportError`] above.
    NamespaceMismatch {
        /// The forwarded run's full SHM-discovery identity.
        run: Box<IceoryxShmIdentity>,
        /// netd's shared-session full SHM-discovery identity.
        netd: Box<IceoryxShmIdentity>,
    },
}

impl std::fmt::Display for EgressError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EgressError::Gateway { source } => {
                write!(f, "failed to register the network egress plan: {source}")
            }
            EgressError::NamespaceMismatch { run, netd } => write!(
                f,
                "the producing run's iceoryx2 SHM-discovery identity [{run}] does NOT match this \
                 cerulion-netd's shared session [{netd}], so netd's gateway cannot tap the run's \
                 SHM. Egress is refused so the run uses a per-run gateway child on its own \
                 namespace. (Do netd and the run resolve the SAME global_config()? A divergent \
                 IOX2_CONFIG_FILE — or a custom service/node directory — between them is the usual \
                 cause.)",
            ),
        }
    }
}

impl std::error::Error for EgressError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            EgressError::Gateway { source } => Some(source.as_ref()),
            EgressError::NamespaceMismatch { .. } => None,
        }
    }
}

/// The seam between netd's egress refcount plane and the machine's ONE network
/// egress path. The daemon calls [`Self::register_egress`] when a connection's
/// `register_egress` verb is accepted by the registry, and [`Self::release_egress`]
/// on release / disconnect. `Send + Sync`: shared across the daemon's per-connection
/// threads.
pub trait EgressPlane: Send + Sync {
    /// Read only this plane's accumulated metadata. No gateway boot, network query,
    /// mirror registration or discovery occurs here. A plane without metadata
    /// reports an error instead of a positive empty snapshot.
    fn serving_schema_snapshot(&self) -> Result<crate::protocol::ServingSchemaSnapshot, String> {
        Err("this daemon has no local serving-schema provider".to_string())
    }

    /// Ensure `plan`'s produced topics are ANNOUNCED + egress-on-demand on the
    /// machine's shared session. On the FIRST call this BOOTS the embedded gateway;
    /// subsequent calls push more topics onto the running one. Returns whether THIS
    /// call booted the gateway (`Ok(true)` = first egress plan on this daemon;
    /// `Ok(false)` = joined a running one). The daemon has ALREADY recorded the
    /// registry accounting + enforced the cross-plan loop guard, so an
    /// implementation never sees a conflicting topic; a failure here rolls that
    /// accounting back.
    ///
    /// `ix_config_json` is the producing run's serialized iceoryx2
    /// namespace (`Some` for a multi-process run, `None` for a monolith that shares
    /// netd's default namespace). The production plane VERIFIES a `Some` config's
    /// full SHM-discovery identity (root/prefix + the service & node directories +
    /// the per-file service suffixes — see [`cerulion_core::IceoryxShmIdentity`])
    /// matches netd's shared session before tapping — a mismatch returns
    /// [`EgressError::NamespaceMismatch`] so the caller falls back to a per-run
    /// gateway child. `None` (or a matching config) taps normally.
    fn register_egress(
        &self,
        id: EgressId,
        plan: &GatewayPlan,
        serving: &SchemaServing,
        ix_config_json: Option<&str>,
    ) -> Result<bool, EgressError>;

    /// Release the egress registration for `id`. The physical announce LINGERS
    /// on the shared gateway past release (the reg-channel is add-only; retraction is
    /// not implemented) — the registry drops the connection-scoped accounting; the
    /// lingering announce is reclaimed at netd's idle self-exit.
    fn release_egress(&self, id: EgressId);
}

/// A no-op egress plane for a mirror-ONLY daemon (no network egress configured) —
/// [`crate::daemon::start`] injects it so a mirror-only daemon still answers the
/// egress verbs (with an explicit refusal) rather than crashing. Every
/// `register_egress` is refused LOUDLY; `release_egress` is a no-op. The production
/// [`GatewayEgressPlane`] is what `main.rs` builds via
/// [`crate::daemon::start_with_egress`].
pub struct NoopEgressPlane;

impl EgressPlane for NoopEgressPlane {
    fn register_egress(
        &self,
        _id: EgressId,
        _plan: &GatewayPlan,
        _serving: &SchemaServing,
        _ix_config_json: Option<&str>,
    ) -> Result<bool, EgressError> {
        Err(EgressError::Gateway {
            source: Box::new(TransportError::InvalidTransportConfig {
                reason: "this cerulion-netd instance has no egress plane configured — it serves \
                         remote-topic demands (ingress) only. Egress convergence requires the \
                         production GatewayEgressPlane (built by the netd binary's main.rs over a \
                         network-configured shared manager)"
                    .to_string(),
            }),
        })
    }

    fn release_egress(&self, _id: EgressId) {}
}

/// A running embedded gateway drive thread. Dropping it stops the thread (flip the
/// exit flag, join). Owned by [`GatewayEgressPlane`] behind a `Mutex<Option>` and
/// started LAZILY on the first `register_egress`, so a spawned-but-never-produced
/// desk starts no gateway thread and announces nothing.
struct RunningGateway {
    exit: Arc<AtomicBool>,
    /// The shared demand-transition wake the drive thread's
    /// zero-demand idle wait blocks on. Held here for ONE reason —
    /// [`Drop`] pokes it after flipping `exit`, so a teardown of an IDLE gateway
    /// joins in microseconds instead of waiting out
    /// [`cerulion_core::GATEWAY_ZERO_DEMAND_IDLE`]. `exit` is an external flag the
    /// wait cannot be woken by, so without this poke every netd shutdown (and
    /// every `ensure_gateway_booted` re-boot of a dead slot) would pay that whole
    /// window (200 ms, derived from the liveness sweep interval).
    wake: Arc<cerulion_core::DemandSignal>,
    /// Principle #3: the drive loop's pacing counters, cloned
    /// BEFORE the gateway is moved onto the thread — the only way to observe the
    /// zero-demand wait from outside, since the gateway itself is owned by the
    /// thread.
    stats: Arc<cerulion_core::GatewayDriveStats>,
    /// Set by the drive thread IF [`GatewayRuntime::run`] returned an error (an
    /// abnormal drive-loop death, distinct from the clean exit-flag teardown). While
    /// set, [`GatewayEgressPlane::ensure_gateway_booted`] treats this slot as dead and
    /// RE-BOOTS a fresh gateway on the next register — otherwise a crashed drive
    /// thread would leave `inner` Some forever, silently pushing every later topic
    /// into a dead reg-channel. Shared with the thread.
    dead: Arc<AtomicBool>,
    /// A `#[doc(hidden)]` test-only fault seam: when set (only ever by
    /// [`GatewayEgressPlane::fault_inject_drive_failure_for_test`]), the drive loop
    /// returns an error on its next pass so the crash → dead → re-boot path is
    /// exercisable from the integration test (netd has no `test-helpers` feature, so
    /// the seam is `#[doc(hidden)] pub` rather than feature-gated). Never set in
    /// production — the drive loop's relaxed load per ~1 ms idle poll is negligible.
    fault: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl Drop for RunningGateway {
    fn drop(&mut self) {
        self.exit.store(true, Ordering::SeqCst);
        // Wake a drive thread parked on the zero-demand idle
        // wait. It re-reads `exit` at the top of its next pass, so the store
        // ABOVE must happen first — otherwise the woken pass could observe `exit`
        // still false, loop, and park again for the full fallback timeout.
        self.wake.signal();
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// What caused an embedded-gateway boot — chooses the one `info!` the boot logs,
/// so the standing start boot and the classic first-registration
/// boot each name their own reason (an operator reading the robot's log must be
/// able to tell "a graph registered egress" from "LISTEN made this a standing
/// gateway").
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GatewayBootTrigger {
    /// The first `register_egress` plan (the original lazy boot).
    EgressRegistration,
    /// Daemon start on a LISTEN-configured machine
    /// ([`GatewayEgressPlane::boot_standing_gateway`]).
    StandingStart,
}

/// The egress gateway drive loop (mirrors [`GatewayRuntime::run`]) with an added
/// test-only fault seam so the run-error → dead → re-boot path is testable. Returns
/// `Err` on a genuine drive-once failure OR a set fault flag; the caller thread
/// records the death so the plane re-boots on the next demand.
fn run_gateway_drive(
    gateway: &mut GatewayRuntime,
    exit: &AtomicBool,
    fault: &AtomicBool,
) -> Result<(), cerulion_core::TransportError> {
    while !exit.load(Ordering::Relaxed) {
        if fault.load(Ordering::Relaxed) {
            return Err(TransportError::Internal {
                reason: "netd egress drive fault (test seam)".to_string(),
            });
        }
        let forwarded = gateway.drive_once()?;
        if forwarded == 0 {
            // Backlog-aware pacing (a forwarded pass loops again immediately).
            // The idle tick is the gateway's OWN `idle_wait` —
            // the 1 ms `GATEWAY_IDLE_POLL` while any demand is ON,
            // a block on the demand signal when none is. A desk with
            // registered-but-undemanded egress topics is exactly that second
            // state.
            gateway.idle_wait();
        }
    }
    Ok(())
}

/// The production egress plane: drives ONE embedded [`GatewayRuntime`] over the
/// SAME network-configured [`TransportManager`] the mirror plane owns (Principle #8
/// — one zenoh session for the whole machine). `main.rs` builds this beside the
/// [`GatewayMirrorPlane`](crate::mirror::GatewayMirrorPlane) from the SAME manager.
pub struct GatewayEgressPlane {
    manager: Arc<TransportManager>,
    /// The lazily-booted gateway drive thread (started on the FIRST egress plan).
    /// `None` until then — a pure-ingress desk runs no egress gateway.
    inner: Mutex<Option<RunningGateway>>,
    /// This machine's `_cerulion._tcp` mDNS advertisement, raised at
    /// the FIRST gateway boot — the beacon lost when the
    /// permissive plane stopped spawning a `run-gateway` child.
    ///
    /// It lives on the PLANE, not on [`RunningGateway`], deliberately: a drive
    /// thread that dies and re-boots did not change the zenoh session or its
    /// bound port, so the beacon must survive that (a flapping mDNS record makes
    /// a robot appear and vanish on every desk browsing the LAN). It is withdrawn
    /// when the plane drops — i.e. when netd exits.
    beacon: crate::beacon::GatewayBeacon,
    /// A `#[doc(hidden)]` test seam, NEVER installed in production: a hook
    /// installed on every gateway this plane boots, which
    /// [`GatewayRuntime::idle_wait`] runs BETWEEN its flag scan and its blocking
    /// wait — i.e. inside the LOST-WAKEUP WINDOW that wait's ordering argument
    /// exists to close.
    ///
    /// The rule is a GATE, not prose:
    /// `tests/idle_wait_gate_confinement_test.rs` walks `src/**/*.rs` over a
    /// comment- and literal-stripped view and REFUSES any mention of this field
    /// that is not one of four shapes — this declaration, the `Mutex::new(None)`
    /// initialisation, a READ, or `ensure_gateway_booted`'s guarded forward onto
    /// the freshly built runtime (a no-op unless a test already installed a hook).
    /// An INSTALL — a `set_idle_wait_gate_for_test` call, a `= Some(..)` write —
    /// may appear only in a `#[cfg(test)] mod` or another `fn …_for_test` seam.
    ///
    /// It exists because that window is otherwise unreachable from a test, and a
    /// stimulus landing in it is counter-INDISTINGUISHABLE from a broken signal:
    /// the scan then observes demand as already ON, so the loop takes the paced
    /// arm and NEITHER `zero_demand_waits` NOR `demand_wakes` moves — exactly what
    /// a gateway whose transition never bumped the signal produces. A test that
    /// merely flips demand and hopes the drive thread was parked is therefore
    /// betting on a race it cannot even score, which is how
    /// `a_demand_transition_wakes_the_parked_drive_thread` went red twice on
    /// `Test (macOS)`, the second time on a PR containing no Rust at all.
    ///
    /// Stopping the thread here turns that bet into a GATE: the hook fires after
    /// the generation was snapshotted and after the scan found no demand, so a
    /// transition raised while it is held is one `wait_for_change` cannot miss, on
    /// every scheduler.
    idle_wait_gate: Mutex<Option<Arc<dyn Fn() + Send + Sync>>>,
    /// Every serving any caller ever handed
    /// this plane, folded FIRST-WINS (`SchemaServing::merge_from`) UNDER the
    /// [`Self::inner`] slot lock. Two consumers: a gateway (RE-)boot seeds its
    /// OWN maps from a clone (self-consistency — a replacement gateway's maps
    /// describe everything ever registered, not just the racing caller's
    /// serving), and the re-boot arm FOLDS it into [`Self::callback_serving`]
    /// (the maps that actually serve — see that field), which is how a
    /// registration that CAUSES a re-boot reaches the latched callback at all.
    ///
    /// LOCK ORDER (documented, and nowhere inverted): `inner` (the slot) →
    /// `accumulated_serving` → `callback_serving` → the per-map serving mutexes
    /// (taken one at a time inside `SchemaServingHandles::merge`). The zenoh
    /// callback takes only the per-map mutexes; the drive thread takes none of
    /// these; no path takes a later lock and then an earlier one.
    ///
    /// RESIDUAL (accepted, additive-only): a registration whose LATER step fails
    /// (e.g. a topic push error rolls the registry back) leaves its serving
    /// folded here. The names/docs are inert without the announce, first-wins
    /// keeps them from displacing anything, and a retry re-folds idempotently.
    accumulated_serving: Mutex<cerulion_core::SchemaServing>,
    /// The serving maps the query-surface CALLBACK actually reads —
    /// the FIRST boot's [`cerulion_core::SchemaServingHandles`], held for the
    /// PLANE's lifetime. `start_query_surface` is LATCHED on the shared
    /// `NetworkManager` (a second call is a no-op), so a RE-booted gateway's
    /// freshly-materialized maps are never read by the callback: every merge —
    /// and, on a re-boot, a fold of the whole accumulator — must land in THESE
    /// handles or it serves nothing. `None` until the first boot succeeds; never
    /// replaced afterwards (mirrors the latched queryable it feeds).
    ///
    /// KNOWN RESIDUAL of the latched surface (pre-existing, orthogonal to the
    /// serving merge): the callback's reg-channel HASH table is likewise the
    /// first construction's, so a topic registered AFTER a re-boot catalogs with
    /// `schema_hash: None` (its hash drains into the replacement gateway's
    /// table). Its NAME + doc resolve through these merged handles, which is
    /// what a desk consumer needs (`resolve_remote_ingress_target` derives the
    /// wire hash from the walker, not the catalog row). Un-latching the shared
    /// surface's sources would close the hash column too.
    callback_serving: Mutex<Option<cerulion_core::SchemaServingHandles>>,
    /// A `#[doc(hidden)]` test seam, never set in production: a one-shot
    /// flag that fails the NEXT gateway boot AFTER `new_embedded` succeeded,
    /// i.e. after the construction started (and latched) the query surface — the
    /// PARTIAL-BOOT shape whose retry could strand the callback on dropped
    /// maps. Set only by
    /// [`Self::fault_inject_boot_failure_after_surface_for_test`].
    boot_fault_after_surface: AtomicBool,
}

impl GatewayEgressPlane {
    /// Wrap the shared (network-configured) transport manager. The gateway is NOT
    /// booted here — it boots on the first [`EgressPlane::register_egress`] (or,
    /// on a LISTEN-configured machine, at daemon start via
    /// [`Self::boot_standing_gateway`]), so a desk that never
    /// produces stays byte-identical to a mirror-only one.
    pub fn new(manager: Arc<TransportManager>) -> Self {
        Self::with_beacon(manager, crate::beacon::GatewayBeacon::new())
    }

    /// TEST ONLY: the same plane, with its beacon's advertise step SUPPRESSED.
    ///
    /// A test that boots this plane over a robot-shaped `NetworkConfig` (an
    /// identity plus a `tcp/` listen endpoint) makes the beacon decide
    /// `Advertise`, and the production advertiser then publishes a genuine,
    /// resolvable `_cerulion._tcp` record on whatever LAN the machine running
    /// `cargo test` is on — MEASURED as `egpdesk` on two interfaces for the
    /// length of the run, where a concurrent `cerulion topic list` renders it as
    /// a live ROBOT and caches a dead ephemeral port in `peers.json`.
    ///
    /// The DECISION path is byte-identical, so every `mdns_beacon_decision()`
    /// oracle still binds; only the socket is not opened. Nothing in `src/` may
    /// call this — see [`crate::beacon::GatewayBeacon::without_mdns_for_test`],
    /// whose `advertiser_is_real` observable is what makes a substitution
    /// detectable.
    #[doc(hidden)]
    pub fn new_without_mdns_for_test(manager: Arc<TransportManager>) -> Self {
        Self::with_beacon(
            manager,
            crate::beacon::GatewayBeacon::without_mdns_for_test(),
        )
    }

    fn with_beacon(manager: Arc<TransportManager>, beacon: crate::beacon::GatewayBeacon) -> Self {
        Self {
            manager,
            inner: Mutex::new(None),
            beacon,
            idle_wait_gate: Mutex::new(None),
            accumulated_serving: Mutex::new(cerulion_core::SchemaServing::default()),
            callback_serving: Mutex::new(None),
            boot_fault_after_surface: AtomicBool::new(false),
        }
    }

    /// Test seam (`#[doc(hidden)]` — never called in production): make
    /// the NEXT boot fail AFTER its construction started (and latched) the query
    /// surface, exercising the partial-boot → retry path deterministically. The
    /// flag is one-shot (the failing boot consumes it), so the retry succeeds.
    #[doc(hidden)]
    pub fn fault_inject_boot_failure_after_surface_for_test(&self) {
        self.boot_fault_after_surface.store(true, Ordering::SeqCst);
    }

    /// TEST ONLY: install the [`idle_wait_gate`](Self::idle_wait_gate)
    /// hook every gateway booted after this call runs inside its zero-demand
    /// idle wait's lost-wakeup window. See that field for why the window needs a
    /// gate rather than a bet.
    ///
    /// Must be called BEFORE the first `register_egress` (which boots the
    /// gateway); a hook installed afterwards reaches the NEXT boot only. The hook
    /// runs ON the drive thread and blocks it for as long as it takes, so a
    /// caller that holds it must bound that hold itself — a hook that never
    /// returns wedges the drive thread and, with it, the plane's teardown join.
    #[doc(hidden)]
    pub fn set_idle_wait_gate_for_test(&self, gate: Arc<dyn Fn() + Send + Sync>) {
        *self
            .idle_wait_gate
            .lock()
            .unwrap_or_else(PoisonError::into_inner) = Some(gate);
    }

    /// Principle #3: whether this plane's beacon reaches the REAL
    /// mDNS socket. `true` on every production path; `false` only for a plane
    /// built by [`new_without_mdns_for_test`](Self::new_without_mdns_for_test).
    ///
    /// The pin that stops the test constructor being silently substituted for
    /// [`new`](Self::new): a desk declines before the advertiser is reached, so
    /// a desk-shaped arm can assert this over the PRODUCTION constructor and
    /// still publish nothing.
    pub fn mdns_advertiser_is_real(&self) -> bool {
        self.beacon.advertiser_is_real()
    }

    /// Principle #3: whether this netd is advertising the machine over
    /// `_cerulion._tcp`. `false` on a desk (no dialable listen endpoint), on a
    /// LOCAL-ONLY netd, and before the first egress registration boots the
    /// gateway. The observable that makes the beacon checkable without a LAN.
    pub fn is_advertising_mdns(&self) -> bool {
        self.beacon.is_advertising()
    }

    /// Principle #3: the beacon decision this plane took, `None`
    /// until the first gateway boot CONSULTS it.
    ///
    /// This — not `is_advertising_mdns` — is what proves the plane wires the
    /// beacon at all: a desk correctly declines and reports `false`, which is
    /// indistinguishable from a plane that never asked. See
    /// [`crate::beacon::GatewayBeacon::decision`].
    pub fn mdns_beacon_decision(&self) -> Option<crate::beacon::BeaconDecision> {
        self.beacon.decision()
    }

    /// Refresh existing mDNS endpoint facts without restarting the LAN gateway.
    pub fn refresh_robot_beacon(
        &self,
        expected_eid: &str,
    ) -> Result<bool, cerulion_mdns::MdnsError> {
        self.beacon.refresh_robot_facts(expected_eid)
    }

    /// The shared transport manager (diagnostics / the daemon's shutdown drop).
    pub fn manager(&self) -> &Arc<TransportManager> {
        &self.manager
    }

    /// Principle #3: the running embedded gateway's drive-loop
    /// pacing counters — drive passes, zero-demand waits, and the demand wakes that
    /// ended them. `None` until the gateway boots (a pure-ingress desk drives
    /// nothing, so there is nothing to report and no zero to misread as one).
    ///
    /// This is the ONLY window onto the drive loop from outside: the gateway itself
    /// is owned by its thread. The counters — not a wall — are what make the
    /// zero-demand wait checkable, since no elapsed time can distinguish a loop
    /// that BLOCKED on a wake from one that spun and got lucky.
    pub fn drive_stats(&self) -> Option<Arc<cerulion_core::GatewayDriveStats>> {
        self.inner
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .as_ref()
            .map(|g| Arc::clone(&g.stats))
    }

    /// Principle #3: whether the embedded egress gateway has booted
    /// (the first egress plan registered). `false` on a pure-ingress desk.
    pub fn gateway_running(&self) -> bool {
        self.inner
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .is_some()
    }

    /// Boot the shared embedded gateway on the first demand (idempotent — a call
    /// while a HEALTHY one runs is `Ok(false)`). Returns whether THIS call booted it.
    /// A previous gateway whose drive thread DIED (its `dead` flag set) is cleared and
    /// RE-BOOTED here rather than left in place — otherwise a crashed thread would
    /// leave `inner` Some forever and every later topic would be pushed into a dead
    /// reg-channel (so the "re-boots on next demand" promise holds).
    ///
    /// This fn also owns the serving fold, and
    /// it happens ATOMICALLY with the slot resolution, under the SAME `inner`
    /// lock: the caller's serving is folded into [`Self::accumulated_serving`],
    /// then EITHER merged into the live gateway's handles (already-running arm)
    /// or seeded into the (re-)boot as the ACCUMULATED clone. Were the merge to
    /// re-acquire the slot lock after this fn returned, a concurrent
    /// registration could replace a DEAD gateway in between — the merge would then
    /// write into the OLD gateway's dropped handles and the caller's types would
    /// resolve `schema_name: None`. The accumulator additionally makes the
    /// RE-boot itself retain every serving merged before the death, which no
    /// merge ordering alone could (the replacement's constructor would see
    /// only the racing caller's serving).
    fn ensure_gateway_booted(
        &self,
        serving: &SchemaServing,
        trigger: GatewayBootTrigger,
    ) -> Result<bool, EgressError> {
        let mut guard = self.inner.lock().unwrap_or_else(PoisonError::into_inner);
        // Fold the caller's serving into the plane-lifetime accumulator UNDER the
        // slot lock (lock order: `inner` → `accumulated_serving` — see the field
        // doc). Every (re-)boot below seeds from the accumulated clone.
        let accumulated = {
            let mut acc = self
                .accumulated_serving
                .lock()
                .unwrap_or_else(PoisonError::into_inner);
            acc.merge_from(serving);
            acc.clone()
        };
        // A dead gateway (drive thread crashed) is torn down so we re-boot below.
        // Dropping the slot joins the already-exited thread (cheap) — safe here on the
        // register caller thread (NEVER on the drive thread, which would self-join).
        if guard
            .as_ref()
            .is_some_and(|g| g.dead.load(Ordering::SeqCst))
        {
            tracing::warn!(
                "netd egress plane: the shared gateway drive thread had DIED — re-booting it (a \
                 later egress registration must not push topics into a dead reg-channel)"
            );
            *guard = None;
        }
        if guard.is_some() {
            // The already-running arm: merge THIS caller's serving into the maps
            // the query-surface callback reads, while still holding the slot lock
            // — atomic with the resolution, so a concurrent dead-gateway
            // replacement can never strand this merge in a dropped gateway's
            // handles (the race). Prior servings are already there
            // (merged when they registered, or folded at a re-boot below).
            if let Some(cb) = self
                .callback_serving
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .as_ref()
            {
                cb.merge(serving);
            }
            return Ok(false);
        }
        // The additive shared-manager constructor: empty announce, permissive desk
        // posture (topics grow at runtime via register_dynamic_egress_topic). The
        // serving seed is the ACCUMULATED one — a re-boot after a drive-thread
        // death must retain every previously-registered serving, not just this
        // caller's.
        let mut gateway =
            GatewayRuntime::new_embedded(Arc::clone(&self.manager), accumulated.clone()).map_err(
                |e| EgressError::Gateway {
                    source: Box::new(e),
                },
            )?;
        // A `#[doc(hidden)]` test seam, never set in production: fail
        // the boot AFTER `new_embedded` succeeded, i.e. after the construction
        // STARTED (and latched) the query surface. This is the PARTIAL-BOOT
        // shape: the gateway drops here, but the surface
        // keeps reading the dropped construction's maps — which is exactly why
        // `callback_serving` below is recorded from the SURFACE's own captures
        // and never from whichever gateway object a later retry succeeds with.
        if self.boot_fault_after_surface.swap(false, Ordering::SeqCst) {
            return Err(EgressError::Gateway {
                source: Box::new(TransportError::Internal {
                    reason: "netd egress boot fault after the query surface started (test seam)"
                        .to_string(),
                }),
            });
        }
        // The test-only lost-wakeup-window hook, if one was installed.
        // `None` on every production path, so the drive loop is byte-unchanged.
        if let Some(gate) = self
            .idle_wait_gate
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
        {
            gateway.set_idle_wait_gate_for_test(gate);
        }
        // Wire the serving maps the query-surface callback reads. The
        // handles are taken from the SURFACE's own recorded captures
        // (`query_surface_serving_handles`) — NEVER from `gateway`, because the
        // surface is LATCHED and outlives constructions: a PARTIAL boot can start
        // it and then fail, and this successful retry's construction then holds
        // maps the callback never reads. Recording the retry gateway's handles
        // here would be exactly that divergence (later
        // merges land in maps nobody serves). Then fold the whole ACCUMULATOR in
        // on EVERY boot: a no-op on a clean first boot (the constructor seeded
        // the same maps), and the ONLY path a re-boot-causing or post-partial-
        // boot registration's serving has into what actually serves.
        {
            let mut cb = self
                .callback_serving
                .lock()
                .unwrap_or_else(PoisonError::into_inner);
            if cb.is_none() {
                *cb = self
                    .manager
                    .network()
                    .and_then(|n| n.query_surface_serving_handles());
                if cb.is_none() {
                    // Structurally unreachable after a successful boot (the
                    // construction starts the surface, which records its
                    // captures) — but a silent fallback to the gateway's handles
                    // would mask exactly the divergence this exists to prevent,
                    // so it is LOUD.
                    tracing::warn!(
                        "netd egress plane: the query surface recorded no serving captures after \
                         a successful gateway boot — falling back to the booted gateway's own \
                         handles (schema serving may not reach the catalog if the surface was \
                         latched by an earlier construction)"
                    );
                    *cb = Some(gateway.schema_serving_handles());
                }
            }
            if let Some(cbh) = cb.as_ref() {
                cbh.merge(&accumulated);
            }
        }
        // Clone the pacing observables + the wake BEFORE the
        // gateway is moved onto the drive thread — after the move the plane has no
        // other handle on either.
        let stats = gateway.drive_stats();
        let wake = self.manager.bridge_manager().demand_signal();
        let exit = Arc::new(AtomicBool::new(false));
        let dead = Arc::new(AtomicBool::new(false));
        let fault = Arc::new(AtomicBool::new(false));
        let exit_thread = Arc::clone(&exit);
        let dead_thread = Arc::clone(&dead);
        let fault_thread = Arc::clone(&fault);
        let handle = std::thread::Builder::new()
            .name("netd-egress-gateway".to_string())
            .spawn(move || {
                let mut gateway = gateway;
                if let Err(e) = run_gateway_drive(&mut gateway, &exit_thread, &fault_thread) {
                    tracing::error!(
                        error = %e,
                        "netd egress gateway drive loop exited with an error — egress stops until \
                         the next demand RE-BOOTS it (marking the slot dead)"
                    );
                    // Record the death so `ensure_gateway_booted` re-boots. Never clear
                    // `inner` from this thread — its RunningGateway holds THIS thread's
                    // JoinHandle, so a self-drop would self-join (deadlock).
                    dead_thread.store(true, Ordering::SeqCst);
                }
            })
            .map_err(|e| EgressError::Gateway {
                source: Box::new(TransportError::Internal {
                    reason: format!("could not spawn the netd egress gateway drive thread: {e}"),
                }),
            })?;
        *guard = Some(RunningGateway {
            exit,
            wake,
            stats,
            dead,
            fault,
            handle: Some(handle),
        });
        // ONE `info!` per boot, naming its reason (see `GatewayBootTrigger`).
        match trigger {
            GatewayBootTrigger::EgressRegistration => tracing::info!(
                "netd egress plane: shared embedded gateway booted (first egress registration) — \
                 the desk now announces its produced topics under this machine's identity"
            ),
            GatewayBootTrigger::StandingStart => tracing::info!(
                "netd egress plane: STANDING gateway booted at daemon start ({LISTEN_ENV} is set \
                 — this machine serves the network): runtime-registered topics (rmw publishers, \
                 raw routes) are announced + demandable with no graph run, and the mDNS beacon \
                 rises on the bound listen port"
            ),
        }
        // This machine is now a PRODUCER on the LAN, so raise the
        // `_cerulion._tcp` beacon — the advertise nothing else raises once
        // the permissive plane spawns no `run-gateway` child (without it
        // a robot serving 86 topics answers nothing to `dns-sd -B`).
        //
        // HERE and not at daemon start, for a reason that is not stylistic: the
        // beacon's SRV record says "dial me at this port", and netd's zenoh
        // session is LAZY, so at daemon start the listener is not bound and a
        // beacon raised then resolves to a closed port. By this line the gateway
        // has opened the session. Idempotent + never-fatal (see `GatewayBeacon`):
        // a re-boot after a drive-thread death does not re-register, and every
        // refusal (desk shape, local-only, a failed register) is a log line, not
        // an error — the gateway keeps serving either way.
        self.beacon
            .ensure_raised(self.manager.network().map(|n| n.config()));
        Ok(true)
    }

    /// Boot the embedded gateway AT DAEMON START: the STANDING
    /// gateway a LISTEN-configured machine (a robot serving the network) needs so
    /// its runtime-registered topics (rmw publishers, raw routes) are announced,
    /// catalogued and demandable with NO graph run, and so the mDNS beacon rises
    /// at boot on the freshly-bound listen port. Same boot as the first
    /// `register_egress` (empty announce, `AllowAll`; the reg-channel reader grows
    /// the set at runtime) — only the trigger, and therefore the logged reason,
    /// differs. Idempotent (`Ok(false)` if something already booted it).
    ///
    /// The caller decides WHETHER (the [`crate::net::standing_gateway_requested`]
    /// gate — the LISTEN endpoint, exactly the beacon's gate, never identity) and
    /// owns the failure posture: `main.rs` warns loudly and keeps the daemon
    /// serving (a later `register_egress` retries the boot).
    pub fn boot_standing_gateway(&self, serving: &SchemaServing) -> Result<bool, EgressError> {
        self.ensure_gateway_booted(serving, GatewayBootTrigger::StandingStart)
    }

    /// Test seam (`#[doc(hidden)]` — never called in production): force the running
    /// gateway's drive loop to fail on its next pass (so it marks the slot dead),
    /// exercising the crash → re-boot path. No-op if no gateway is running.
    #[doc(hidden)]
    pub fn fault_inject_drive_failure_for_test(&self) {
        if let Some(g) = self
            .inner
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .as_ref()
        {
            g.fault.store(true, Ordering::SeqCst);
        }
    }

    /// Test seam (`#[doc(hidden)]`): whether the running gateway's drive thread has
    /// recorded its death (the `dead` flag) — the observable a test polls after a
    /// fault before it re-registers. `false` when no gateway is running or it is
    /// healthy.
    #[doc(hidden)]
    pub fn gateway_drive_died_for_test(&self) -> bool {
        self.inner
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .as_ref()
            .is_some_and(|g| g.dead.load(Ordering::SeqCst))
    }
}

impl EgressPlane for GatewayEgressPlane {
    fn serving_schema_snapshot(&self) -> Result<crate::protocol::ServingSchemaSnapshot, String> {
        let schema_serving = self
            .accumulated_serving
            .try_lock()
            .map_err(|error| format!("local serving metadata unavailable: {error}"))?
            .clone();
        let ix_config_json = serde_json::to_string(&self.manager.iox_config())
            .map_err(|error| format!("cannot serialize serving namespace: {error}"))?;
        Ok(crate::protocol::ServingSchemaSnapshot {
            schema_serving,
            ix_config_json,
        })
    }

    fn register_egress(
        &self,
        _id: EgressId,
        plan: &GatewayPlan,
        serving: &SchemaServing,
        ix_config_json: Option<&str>,
    ) -> Result<bool, EgressError> {
        // 0. VERIFY the forwarded run namespace matches netd's shared
        //    session BEFORE booting anything. The gateway taps SHM on THIS manager's
        //    namespace (Principle #8 — one node per process), so a run that resolved a
        //    DIFFERENT SHM-discovery identity (root/prefix + the service & node
        //    directories + the per-file service suffixes — anything that changes WHICH
        //    SHM files a topic resolves to) is un-tappable here; refuse it so the run
        //    falls back to a per-run gateway child on its own namespace rather than
        //    being silently tapped on the wrong one (egressing nothing). A `None`
        //    config (the monolith path) shares netd's default namespace by
        //    construction — nothing to verify. The check is SCOPED per registration:
        //    each `register_egress` carries its own config, so two concurrent runs on
        //    different namespaces cannot cross (the matching one is served here; the
        //    other is refused → child).
        if let Some(json) = ix_config_json {
            let run = cerulion_core::ix_config_shm_identity_from_json(json).map_err(|e| {
                EgressError::Gateway {
                    source: Box::new(e),
                }
            })?;
            let netd = self.manager.iox_shm_identity();
            if run != netd {
                return Err(EgressError::NamespaceMismatch {
                    run: Box::new(run),
                    netd: Box::new(netd),
                });
            }
        }
        // 1. Boot the shared gateway lazily on the FIRST egress plan — which ALSO
        //    folds + merges this registration's serving ATOMICALLY under the slot
        //    lock (the backfill seam; see `ensure_gateway_booted`): a
        //    gateway the standing start boot brought up with the built-in corpus
        //    only still catalogs + serves this plan's CUSTOM types, and a
        //    concurrent dead-gateway replacement can never strand the merge in a
        //    dropped gateway's handles.
        let booted = self.ensure_gateway_booted(serving, GatewayBootTrigger::EgressRegistration)?;
        // 2. Push each of the plan's ANNOUNCE topics onto the running gateway over
        //    the reg-channel — the gateway's runtime-registration reader picks them
        //    up on its next drive pass and announces + enables egress-on-demand. The
        //    schema hash is informational for egress (frames forward VERBATIM), so it
        //    is resolved best-effort from `serving` (a 0 fallback never gates
        //    delivery). Only the egress half of the plan is consumed — a graph's
        //    ingress converges via the demand plane.
        for topic in &plan.announce {
            let hash = hash_for_topic(topic, serving);
            self.manager
                .register_dynamic_egress_topic(topic, hash)
                .map_err(|e| EgressError::Gateway {
                    source: Box::new(e),
                })?;
        }
        Ok(booted)
    }

    fn release_egress(&self, _id: EgressId) {
        // The physical announce LINGERS on the shared gateway (the reg-channel +
        // announce set are add-only; retraction is not implemented). The registry has
        // already dropped the connection-scoped accounting (liveness + the loop guard
        // stay truthful); the lingering announce is reclaimed when netd idle-self-exits
        // and this plane drops (the gateway thread stops + the session drops). A gone
        // producer's tap simply fails to attach (counted, non-fatal) meanwhile.
        tracing::debug!(
            "netd egress plane: egress released — the physical announce lingers on the shared \
             gateway until idle self-exit (retraction is not implemented)"
        );
    }
}

/// Resolve a produced topic's wire `schema_hash` from the handed [`SchemaServing`]
/// (topic → schema name → hash), else `0`. Egress forwards frames VERBATIM, so the
/// hash is informational (catalog/diagnostics) and never gates delivery — a `0`
/// fallback is safe when the desk did not resolve the topic's schema. Pure.
fn hash_for_topic(topic: &str, serving: &SchemaServing) -> u64 {
    let canon = canonical_topic(topic);
    let name = serving
        .topic_schemas
        .iter()
        .find(|ts| canonical_topic(&ts.topic) == canon)
        .map(|ts| ts.schema_name.as_str());
    let Some(name) = name else {
        return 0;
    };
    serving
        .schema_hashes
        .iter()
        .find(|b| b.qualified == name)
        .map(|b| b.schema_hash)
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::{SchemaHashName, TopicSchema};

    #[test]
    fn noop_egress_plane_refuses_loudly_and_release_is_a_noop() {
        let plane = NoopEgressPlane;
        let plan = GatewayPlan {
            egress_policy: cerulion_core::GatewayEgressPolicy::AllowAll,
            announce: vec!["/t".to_string()],
            ingress: vec![],
        };
        let err = plane
            .register_egress(0, &plan, &SchemaServing::default(), None)
            .expect_err("no-op plane refuses egress");
        assert!(
            err.to_string().contains("no egress plane configured"),
            "the refusal is explicit + loud: {err}"
        );
        // release is a safe no-op.
        plane.release_egress(0);
    }

    #[test]
    fn hash_for_topic_resolves_from_serving_else_zero() {
        let serving = SchemaServing {
            topic_schemas: vec![TopicSchema {
                topic: "/robot/odom".to_string(),
                schema_name: "nav_msgs/Odometry".to_string(),
            }],
            schema_docs: vec![],
            schema_hashes: vec![SchemaHashName {
                schema_hash: 0xDEAD_BEEF,
                qualified: "nav_msgs/Odometry".to_string(),
            }],
        };
        // Resolves through topic → name → hash (canonicalized topic match).
        assert_eq!(hash_for_topic("robot/odom", &serving), 0xDEAD_BEEF);
        assert_eq!(hash_for_topic("/robot/odom", &serving), 0xDEAD_BEEF);
        // Unknown topic → 0 (informational; egress forwards verbatim).
        assert_eq!(hash_for_topic("/unknown", &serving), 0);
        // A topic with a name but no matching hash binding → 0.
        let partial = SchemaServing {
            topic_schemas: vec![TopicSchema {
                topic: "/t".to_string(),
                schema_name: "pkg/Type".to_string(),
            }],
            ..Default::default()
        };
        assert_eq!(hash_for_topic("/t", &partial), 0);
    }
}
