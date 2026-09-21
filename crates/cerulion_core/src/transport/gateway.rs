// SPDX-License-Identifier: AGPL-3.0-only
//! The single network-GATEWAY process for a `graph run`.
//!
//! One gateway per running graph owns the robot's entire network plane: the
//! single zenoh session (one robot = one network peer — Principle #8), every
//! egress announce, the demand watch, egress forwarding, and the ingress
//! re-injection door for ALL declared ingress topics. Graph / worker processes
//! carry NO network session — they publish into and read from shared memory
//! only. The gateway taps their SHM zero-copy and moves network serialization
//! off the control processes.
//!
//! # Egress is demand-driven, tap-based
//!
//! The gateway does NOT dual-publish from inside a publisher. Instead each
//! produced topic it announces has a bridge flag ([`super::bridge`]); a REMOTE
//! subscriber's DEMAND liveliness token flips that flag ON (the demand watch),
//! and the gateway then attaches a listener-less [`DataOnlySubscriber`] SHM tap
//! to the topic and forwards each drained wire frame to zenoh VERBATIM. When
//! the demand token goes away the flag flips OFF and the gateway drops the tap —
//! an unwanted topic costs nothing. The single heap copy to zenoh
//! (`OwnedInboundSample::payload().to_vec()`) exists identically in every
//! architecture; the tap adds none of its own. The tap shares the topic's
//! introspection subscriber headroom with bagd / `topic echo` taps — an attach
//! failure (slot exhaustion) is counted, warned once per regime, and retried
//! every pass ([`GatewayRuntime::attach_failure_count`]).
//!
//! # Demand reconciliation — the belt to the subscriber's suspenders
//!
//! The demand watch above is a liveliness SUBSCRIBER: it flips egress flags
//! sub-ms on a healthy link. But on a strict connect-only, listen-less demander
//! link (the real robot↔Mac case) subscriber-interest propagation can FAIL while
//! bounded liveliness QUERIES still work — the producing gateway never wakes and
//! the demanded topic silently never egresses. So [`GatewayRuntime::new`] ALSO
//! starts a query-shaped BELT: the demand RECONCILER
//! ([`super::network::NetworkManager::start_demand_reconciler`]) runs one bounded
//! `cerulion_lv/**` demand gather every ~1 s and reconciles the announce flags
//! against it (with 3-pass hysteresis), so egress starts within ~1 s of demand
//! even when the subscriber path is dead. It never widens egress past the
//! allow-list (that gate lives in [`super::bridge::TopicBridgeManager::enable_bridge`]).
//! Its tracking guard is precisely scoped: a topic whose demand it has never
//! observed is left untouched, but once it observes a topic's demand it adopts
//! that topic's egress lifecycle (enable + later absence-disable). Observable via
//! [`GatewayRuntime::reconcile_pass_count`] / [`GatewayRuntime::reconciler_enabled_count`].
//!
//! # Ingress
//!
//! The gateway registers one network→local ingress bridge per declared ingress
//! topic ([`super::TransportManager::register_ingress_topic`]) — it validates
//! every inbound frame against the topic's expected schema hash and re-injects
//! it into local SHM for the graph process to read.
//!
//! # Loop safety
//!
//! A gateway NEVER taps a topic it also re-injects: [`GatewayPlan::validate`]
//! refuses a plan whose `announce` and `ingress` sets intersect (the
//! local-publish → egress → ingress → re-inject → egress loop), and the
//! ingress registration's own [`super::bridge::TopicBridgeManager::is_registered`]
//! guard is the structural backstop.

use std::collections::{HashMap, HashSet};
use std::fmt;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use serde::{Deserialize, Serialize};

use super::liveness::{DrainObservation, TopicLiveness, TopicLivenessObserver};
use super::network::{
    canonical_topic, CatalogSource, DemandGetState, GetDemandState, PermissiveShmProbe, RunSource,
    SchemaSource,
};
use super::reg_channel::{RegRecord, RegistrationReader};
use super::run_registry::RUN_GATHER_WINDOW;
use super::subscriber::DataOnlySubscriber;
use super::TransportManager;
use crate::error::{TransportError, TransportResult};
use crate::wire::MaxSliceLen;

/// How long the gateway [`GatewayRuntime::run`] loop sleeps when a drive pass
/// forwarded nothing. Backlog-aware pacing (cribbed from `cerulion_bagd`'s
/// `drive_loop`): a pass that forwarded ≥1 frame loops again immediately
/// (drain-to-empty under load), an idle pass yields the CPU for one tick so the
/// loop is not a busy-spin. A forwarder wants low added latency, so the tick is
/// tighter (1 ms) than bagd's 10 ms recorder tick.
const GATEWAY_IDLE_POLL: Duration = Duration::from_millis(1);

/// The FALLBACK tick for a gateway with ZERO demand — the one
/// state in which the drive loop has provably nothing to forward.
///
/// While ANY egress flag is ON the loop is byte-identical to the polling loop: it paces
/// at `GATEWAY_IDLE_POLL` (1 ms), so the one-budget-per-pass drain, the liveness
/// `queue_emptied` observation, the sustained-regression bands and the
/// rate window all see exactly the cadence they were derived against. None of
/// those timings moves.
///
/// With NO flag ON, a pass loads every flag, forwards zero and sleeps 1 ms — on a
/// `ros2 attach` Go2 that is ~71 atomic loads a thousand times a second for nothing,
/// which on a Jetson is both CPU and a denial of deep C-states. Such a gateway
/// BLOCKS on the [`DemandSignal`](super::bridge::DemandSignal) instead, so the
/// first demand wakes it immediately and an idle robot ticks ~5 Hz rather than
/// ~1000 Hz.
///
/// The timeout is a REAL fallback, not a formality — it is what keeps this loop a
/// strict SUPERSET of the polling one. Two events reach a gateway without touching
/// the in-process signal: a runtime egress registration published from ANOTHER
/// process over the iceoryx2 control channel (`ros2 attach`'s `dds_bridge` worker is
/// exactly that), and the drive loop's own exit flag. So the real cost of this
/// park is: on a ZERO-DEMAND gateway, a cross-process runtime registration's
/// announce, and the drive loop's teardown, are each delayed by up to this value.
///
/// **The value is DERIVED from [`LIVENESS_SWEEP_INTERVAL_NS`](super::liveness::LIVENESS_SWEEP_INTERVAL_NS), not chosen.** At zero
/// demand this park is the ONLY thing that calls `drive_once`, and `drive_once` is
/// the only production caller of `TopicLivenessObserver::sweep` — whose own throttle
/// is a FLOOR with no catch-up. So the park IS the observer's grid in exactly the
/// state the observer exists for (topics nobody has demanded, whose liveness
/// nothing else can see). A 250 ms park stretches the grid
/// to a MEASURED 256-282 ms, quietly moving every band derived from the sweep
/// interval — `REGRESSION_RESET_MIN_GAP_NS`, `SUSTAINED_REGRESSION_MIN_SPAN_NS`,
/// `RATE_ESTIMATE_MIN_WINDOW_NS` and the labelled `RATE_FLOOR_BASIS_CEILING_MHZ`
/// ceiling — for the zero-demand case only.
/// Deriving it, plus the const-assert below, makes that
/// coupling explicit and un-driftable: raising this constant cannot silently
/// coarsen the liveness plane, and re-tuning it deliberately means re-deriving the
/// bands too.
///
/// The CPU argument is untouched by the derivation: the point of this park is dropping
/// a zero-demand gateway off a ~1000 Hz flag scan, and 5 Hz versus 4 Hz is not a
/// number anyone can measure against that. `GATEWAY_IDLE_POLL` is still 200x tighter,
/// so the change still buys what it was for, and 200 ms remains far below a
/// human-noticeable delay for the two events above.
pub const GATEWAY_ZERO_DEMAND_IDLE: Duration =
    Duration::from_nanos(super::liveness::LIVENESS_SWEEP_INTERVAL_NS);

/// The zero-demand park must never be COARSER than the liveness sweep grid, or the
/// park silently becomes the observer's cadence and every band derived from
/// [`LIVENESS_SWEEP_INTERVAL_NS`](super::liveness::LIVENESS_SWEEP_INTERVAL_NS) stops describing a real zero-demand gateway.
///
/// Equality is the tight end and is SAFE on the correct side: `sweep`'s throttle is a
/// strict `<` (`now - last_sweep < interval` returns early), and a drive pass costs a
/// nonzero amount of time on top of the park, so `elapsed >= interval` holds on every
/// pass. `DemandSignal::wait_for_change` cannot return early on a timeout either —
/// `wait_timeout_while` re-checks its predicate and re-arms the remaining timeout, so
/// a spurious wake does not shorten the park.
const _: () = assert!(
    GATEWAY_ZERO_DEMAND_IDLE.as_nanos() as u64 <= super::liveness::LIVENESS_SWEEP_INTERVAL_NS,
    "GATEWAY_ZERO_DEMAND_IDLE must not exceed LIVENESS_SWEEP_INTERVAL_NS — at zero \
     demand the park paces TopicLivenessObserver::sweep, so a coarser park stretches \
     the liveness and rate-estimate bands"
);
/// And it must stay far ABOVE the demanded-path tick, or parking buys nothing.
const _: () = assert!(
    GATEWAY_ZERO_DEMAND_IDLE.as_nanos() > 10 * GATEWAY_IDLE_POLL.as_nanos(),
    "GATEWAY_ZERO_DEMAND_IDLE must stay well above GATEWAY_IDLE_POLL, or the \
     zero-demand park is not a fallback tick but a second busy poll"
);

/// The drive loop's own pacing counters (Principle #3), shared out
/// of the [`GatewayRuntime`] as an `Arc` so a caller can keep observing after the
/// gateway is MOVED onto a drive thread (netd's egress plane does exactly that).
///
/// These exist because no latency measurement can tell a loop that BLOCKED on a
/// wake from one that got lucky with a timer — the counters are the only
/// log-independent, load-safe evidence that the zero-demand wait is real and that a
/// demand transition is what ended it.
#[derive(Debug, Default)]
pub struct GatewayDriveStats {
    passes: AtomicU64,
    zero_demand_waits: AtomicU64,
    demand_wakes: AtomicU64,
}

impl GatewayDriveStats {
    /// Cumulative [`GatewayRuntime::drive_once`] passes. On an idle gateway this is
    /// the number a ceiling assertion bounds: ~1000/s before the zero-demand wait,
    /// ~`1s / GATEWAY_ZERO_DEMAND_IDLE` after.
    pub fn passes(&self) -> u64 {
        self.passes.load(Ordering::Relaxed)
    }

    /// Cumulative times the loop ENTERED the zero-demand blocking wait (i.e. found
    /// no egress flag ON). Stays 0 for a gateway that always has demand, which is
    /// the "the ON path is byte-unchanged" pin.
    pub fn zero_demand_waits(&self) -> u64 {
        self.zero_demand_waits.load(Ordering::Relaxed)
    }

    /// Cumulative times a zero-demand wait ended because the
    /// [`DemandSignal`](super::bridge::DemandSignal) FIRED rather than timing out.
    /// A LOWER bound load can delay but never fake: a timeout cannot increment it.
    pub fn demand_wakes(&self) -> u64 {
        self.demand_wakes.load(Ordering::Relaxed)
    }
}

/// The reserved control-plane topic namespace. Everything under this
/// prefix is the framework's own control plane — the runtime-registration
/// control service lives at `__cerulion/gateway_topics` (canonical
/// `/__cerulion/gateway_topics`). A runtime egress registration for any topic in
/// this namespace is REFUSED loudly by
/// [`GatewayRuntime::register_runtime_topic`]: registering (and thus announcing +
/// egress-tapping) a control-plane service would let robot data plumbing collide
/// with the plumbing that carries it. The bare namespace token `/__cerulion` and
/// any `/__cerulion/…` child are reserved; a distinct name like `/__cerulionx`
/// is NOT (it is a different topic, not a child of the namespace).
pub const RESERVED_TOPIC_PREFIX: &str = "/__cerulion";

/// Whether `canonical` (a leading-slash canonical topic name) is in the
/// reserved control-plane namespace ([`RESERVED_TOPIC_PREFIX`]) — the bare token
/// itself or any child under it. `pub(crate)` so the registration-channel entry
/// point ([`super::TransportManager::register_dynamic_egress_topic`]) reuses the
/// EXACT same reserved-namespace predicate (defense in depth, ONE source of
/// truth — the gateway-side guard is the backstop).
pub(crate) fn is_reserved_topic(canonical: &str) -> bool {
    canonical == RESERVED_TOPIC_PREFIX
        || canonical.starts_with(&format!("{RESERVED_TOPIC_PREFIX}/"))
}

/// The egress POSTURE half of a [`GatewayPlan`] — which produced topics
/// a remote demand token may switch ON for network egress. Serde-clean (plain
/// strings) so a CLI can hand a plan across the graph→gateway process boundary.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum GatewayEgressPolicy {
    /// Permissive-default egress: any ANNOUNCED topic's flag may flip.
    /// Selected for a real-clock single-process `graph run` with no `network:`
    /// block — every produced topic is network-viewable until pairing-based access gating lands.
    AllowAll,
    /// Only the listed canonical topics may ever flip (the graph's declared
    /// `network: egress:` list). A non-member demand token is refused; an EMPTY
    /// list is deny-all (an ingress-only gateway).
    AllowList(Vec<String>),
}

/// One declared network→local INGRESS topic + the schema hash every
/// inbound frame is validated against before re-injection (resolved from the
/// consuming input's macro metadata — see
/// [`crate::graph::runtime::compute_gateway_plan`]).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GatewayIngressEntry {
    /// Canonical absolute topic the gateway re-injects inbound frames into.
    pub topic: String,
    /// Expected wire `schema_hash` (recipe-3) — a mismatch is a counted drop.
    pub schema_hash: u64,
}

/// The complete network plan for ONE gateway process — the serde-clean
/// value a CLI computes from a graph's `network:` block (or the permissive
/// default) and hands to a separate gateway process. Holds only `String`s /
/// `u64`s so it round-trips through JSON with no transport types.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GatewayPlan {
    /// Which produced topics a remote demand token may egress.
    pub egress_policy: GatewayEgressPolicy,
    /// Canonical topics this gateway ANNOUNCES on the network (declares an
    /// egress-presence token for + registers a bridge flag for, so a remote
    /// demand token can flip it and `topic list --network` can list it).
    pub announce: Vec<String>,
    /// Network→local ingress bridges this gateway opens.
    pub ingress: Vec<GatewayIngressEntry>,
}

/// One topic→schema-name binding for catalog enrichment. The
/// CLI resolves it from the graph (a produced/ingress topic's declared schema)
/// and hands it to the gateway so the `catalog` verb can carry the qualified
/// name per topic. Serde-clean.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TopicSchema {
    /// Canonical absolute topic name (leading `/`).
    pub topic: String,
    /// The topic's qualified schema name (`pkg/Type`).
    pub schema_name: String,
}

/// One hash→qualified-name binding — a reverse-map entry
/// the gateway uses to NAME a RUNTIME-registered topic (a `ros2 attach` raw route
/// that arrives over the reg-channel carrying only a `(topic, schema_hash)` — no
/// name). The CLI computes each served type's recipe-3 `schema_hash` (it holds
/// the parsed IR for BOTH `.msg` and YAML encodings, and resolves nested targets
/// against the built-in corpus so the hash matches the wire) and hands the
/// bindings across; the gateway materializes them into a hash→name map read at
/// catalog-build time. Serde-clean.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SchemaHashName {
    /// The served type's recipe-3 wire `schema_hash`.
    pub schema_hash: u64,
    /// Its qualified `pkg/Type` (or package-less bare) name.
    pub qualified: String,
}

/// The schema-SERVING half of a gateway's plan — the topic→name
/// bindings (catalog enrichment) + the pre-built served schema docs (the `schema`
/// verb's source, closure-complete over the robot's CUSTOM types; built-ins are
/// omitted — the desk has them) + the hash→name reverse bindings (naming the
/// runtime-registered raw routes). The CLI builds this from the workspace
/// (`.msg` store + workspace YAML + built-in classification), which a network-free
/// cerulion_core gateway process cannot reach, and hands it across the process
/// boundary. Serde-clean + `Default` (an empty serving = catalog without names +
/// no schema serving, so every existing gateway
/// caller keeps working unchanged via [`GatewayRuntime::new`]).
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct SchemaServing {
    /// Topic→schema-name bindings for catalog enrichment.
    pub topic_schemas: Vec<TopicSchema>,
    /// The served schema docs (requested-and-nested-custom closure members). Each
    /// doc's `deps` name OTHER docs in this list — closure-complete, no dangles.
    pub schema_docs: Vec<super::cerulion_q::SchemaDoc>,
    /// Hash→qualified-name bindings for the catalog's
    /// hash-reverse resolution of runtime-registered topics. `#[serde(default)]`
    /// so an older handoff (no field) boots naming only `topic_schemas` topics.
    ///
    /// Carries the BUILT-IN corpus's bindings (~254 entries) as well
    /// as the custom types' — a runtime-registered topic of a built-in type (an
    /// rmw `std_msgs/String` publisher) otherwise catalogs `schema_name: None`,
    /// which every desk consumer refuses. The gateway itself cannot compute them
    /// (`cerulion_core` has no message corpus); the handing side does.
    #[serde(default)]
    pub schema_hashes: Vec<SchemaHashName>,
}

/// Live handles onto a RUNNING gateway's schema-serving maps — the
/// topic→name bindings, the hash→name reverse map, and the served docs the
/// query-surface callback reads. Cloned off the [`GatewayRuntime`] BEFORE it
/// moves onto its drive thread ([`GatewayRuntime::schema_serving_handles`]), so
/// a LATER `register_egress` can [`merge`](Self::merge) its own
/// [`SchemaServing`] into the live plane.
///
/// This exists because a FROZEN boot serving would be wrong: `cerulion-netd`'s
/// standing gateway boots at daemon start with the built-in corpus only, a
/// later registration never re-boots, and its custom types would catalogue
/// `schema_name: None` with zero served docs — the desk's `SchemaUnavailable`.
/// The merge is the backfill seam that makes the standing boot and per-graph
/// registrations compose in BOTH directions.
///
/// Cheap to clone (three `Arc`s); `Send + Sync` (the maps are `Mutex`-guarded —
/// the same discipline as the reg-channel hash table beside them).
#[derive(Clone)]
pub struct SchemaServingHandles {
    /// Canonical topic → qualified schema name (the catalog's first resolution).
    topic_schemas: Arc<Mutex<HashMap<String, String>>>,
    /// Recipe-3 `schema_hash` → qualified name (names runtime-registered topics).
    hash_names: Arc<Mutex<HashMap<u64, String>>>,
    /// Qualified name → served doc (the `schema` verb's source).
    docs: Arc<Mutex<std::collections::BTreeMap<String, super::cerulion_q::SchemaDoc>>>,
}

impl SchemaServingHandles {
    /// Build handles over the given maps — used by `start_query_surface` to
    /// record the EXACT maps its callback captured (the latched query
    /// surface outlives the gateway construction that started it, so the merge
    /// target must be bound to the surface's own captures, never to whichever
    /// gateway object a caller happens to hold — see
    /// [`super::network::NetworkManager::query_surface_serving_handles`]).
    pub(crate) fn from_parts(
        topic_schemas: Arc<Mutex<HashMap<String, String>>>,
        hash_names: Arc<Mutex<HashMap<u64, String>>>,
        docs: Arc<Mutex<std::collections::BTreeMap<String, super::cerulion_q::SchemaDoc>>>,
    ) -> Self {
        Self {
            topic_schemas,
            hash_names,
            docs,
        }
    }

    /// Fold `serving` into the live maps. FIRST-WINS per key, which makes the
    /// merge IDEMPOTENT and keeps the boot seed authoritative (the boot serving's
    /// customs-first order is preserved: an already-bound topic, hash, or doc is
    /// never overwritten — and a duplicate hash implies a duplicate name anyway,
    /// since the qualified name folds into the recipe-3 digest). Topics are
    /// canonicalized exactly as the boot materialization canonicalizes them, so a
    /// merged binding and a boot binding for one topic land on ONE key.
    ///
    /// Locks are taken one map at a time and never nested (no ordering hazard
    /// with the catalog serve, which locks the same maps one at a time).
    pub fn merge(&self, serving: &SchemaServing) {
        {
            let mut ts = self
                .topic_schemas
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            for b in &serving.topic_schemas {
                ts.entry(canonical_topic(&b.topic).into_owned())
                    .or_insert_with(|| b.schema_name.clone());
            }
        }
        {
            let mut hn = self
                .hash_names
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            for b in &serving.schema_hashes {
                hn.entry(b.schema_hash)
                    .or_insert_with(|| b.qualified.clone());
            }
        }
        {
            let mut docs = self
                .docs
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            for doc in &serving.schema_docs {
                docs.entry(doc.qualified.clone())
                    .or_insert_with(|| doc.clone());
            }
        }
    }
}

impl SchemaServing {
    /// Fold `other` into `self`, FIRST-WINS per key — the SAME
    /// precedence as [`SchemaServingHandles::merge`], stated once: an existing
    /// topic binding (canonical name), hash binding, or doc (qualified name) is
    /// never overwritten, which makes repeated folds idempotent and keeps the
    /// earliest registration authoritative.
    ///
    /// This is the ACCUMULATOR half of the backfill seam: `cerulion-netd`'s
    /// egress plane folds every registration's serving into one plane-lifetime
    /// `SchemaServing` and seeds any gateway (RE-)boot from it — so a gateway
    /// replaced after a drive-thread death retains every serving merged before
    /// the death, instead of forgetting them (the race this closes: a
    /// replacement between a caller's boot-resolution and its merge dropped the
    /// caller's types back to `schema_name: None`).
    pub fn merge_from(&mut self, other: &Self) {
        for ts in &other.topic_schemas {
            let canon = canonical_topic(&ts.topic);
            if !self
                .topic_schemas
                .iter()
                .any(|have| canonical_topic(&have.topic) == canon)
            {
                self.topic_schemas.push(ts.clone());
            }
        }
        for b in &other.schema_hashes {
            if !self
                .schema_hashes
                .iter()
                .any(|have| have.schema_hash == b.schema_hash)
            {
                self.schema_hashes.push(b.clone());
            }
        }
        for doc in &other.schema_docs {
            if !self
                .schema_docs
                .iter()
                .any(|have| have.qualified == doc.qualified)
            {
                self.schema_docs.push(doc.clone());
            }
        }
    }
}

impl GatewayPlan {
    /// Loop safety: refuse a plan whose `announce` and `ingress` sets
    /// intersect. A gateway that both tapped a topic for egress AND re-injected
    /// remote frames into it would loop (re-injected frame → tapped → egressed →
    /// a remote re-injects → …). Compared on CANONICAL names so `/t` and `t`
    /// cannot slip past each other. Called at [`GatewayRuntime::new`]; a CLI can
    /// also call it before serializing a plan across the process boundary.
    pub fn validate(&self) -> TransportResult<()> {
        let announce: std::collections::HashSet<String> = self
            .announce
            .iter()
            .map(|t| canonical_topic(t).into_owned())
            .collect();
        for entry in &self.ingress {
            let canonical = canonical_topic(&entry.topic).into_owned();
            if announce.contains(&canonical) {
                return Err(TransportError::InvalidTransportConfig {
                    reason: format!(
                        "gateway plan is a network loop: topic '{canonical}' is BOTH \
                         announced for egress and registered for ingress — a gateway must \
                         never re-inject a topic it also taps out. Remove '{canonical}' \
                         from one of `egress`/`announce` or `ingress`"
                    ),
                });
            }
        }
        Ok(())
    }
}

/// The running network gateway. Owns the network-configured
/// [`TransportManager`] (the single zenoh session), the egress bridge flags for
/// every announced topic, and the lazily-attached egress taps. Drive it with
/// [`Self::run`] (loop until an exit flag) or [`Self::drive_once`] (one pass,
/// for tests).
pub struct GatewayRuntime {
    manager: Arc<TransportManager>,
    /// Canonical egress topic → its bridge flag — the drive loop's iteration
    /// view of the registered egress set (boot-plan topics AND
    /// runtime-registered ones). Held in an `Arc<[..]>` so [`Self::drive_once`]
    /// can iterate it while mutating the tap map.
    /// [`Self::drive_once`] RECONCILES it against the bridge
    /// manager's registered-flag map (the source of truth) whenever a runtime
    /// registration grows that map — so a topic registered after boot joins the
    /// tap set with no restart.
    egress_flags: Arc<[(String, Arc<AtomicBool>)]>,
    /// The canonical topics declared in the boot `GatewayPlan.announce`.
    /// Immutable after construction; used only to distinguish RUNTIME-registered
    /// topics (registered set MINUS this) for the Principle-#3 accessors
    /// ([`Self::runtime_registered_topics`]).
    boot_topics: HashSet<String>,
    /// Canonical announce topic → its active egress tap (present only while the
    /// topic's bridge flag is ON). The observable [`Self::active_tap_topics`].
    taps: HashMap<String, DataOnlySubscriber>,
    /// Canonical announce topic → cumulative SUCCESSFULLY-forwarded frame count
    /// (Principle #3). Interior-mutable so the forward path holds only `&self`.
    forwarded: HashMap<String, AtomicU64>,
    /// Canonical announce topic → first-warn-then-debug latch for repeated zenoh
    /// put failures (a persistently unreachable peer must not flood the log).
    forward_warn: HashMap<String, AtomicBool>,
    /// Canonical announce topic → cumulative tap ATTACH failure count
    /// (Principle #3: a topic that cannot egress because its tap never attaches
    /// — e.g. subscriber slots exhausted — must be visible, never
    /// silent-forever).
    attach_failures: HashMap<String, AtomicU64>,
    /// Canonical announce topic → once-per-regime latch for the attach-failure
    /// warn (re-armed by a successful attach, so a healed-then-broken topic
    /// warns loudly again).
    attach_warn: HashMap<String, AtomicBool>,
    /// Reusable SHM drain buffer — one alloc, reused every drive pass.
    drain_buf: Vec<super::subscriber::OwnedInboundSample>,
    /// The READER on the `/__cerulion/gateway_topics` control service —
    /// [`Self::drive_once`] drains it and feeds every record through
    /// [`Self::register_runtime_topic`] so a network-free worker's runtime raw
    /// routes join the announce + demand set. `None` iff the reader could not be
    /// opened at boot (a loud warn was emitted; boot-plan topics still work).
    reg_reader: Option<RegistrationReader>,
    /// Reusable buffer the reg-channel drain fills each pass — one alloc,
    /// reused (mirrors `drain_buf`).
    reg_drain_buf: Vec<RegRecord>,
    /// Canonical runtime-registered topic → the `schema_hash` the
    /// producing process advertised for it (item 5 — stored observably for
    /// future probe/diagnostics; egress forwards frames verbatim, so it never
    /// gates delivery). Read via [`Self::runtime_topic_schema_hash`].
    ///
    /// Shared (`Arc<Mutex<..>>`) with the query surface's
    /// [`CatalogSource`], which reads it on the zenoh callback thread to stamp the
    /// `catalog` verb's per-topic `schema_hash`. This gateway's drive thread is the
    /// sole writer ([`Self::drain_registration_channel`]).
    reg_hashes: Arc<Mutex<HashMap<String, u64>>>,
    /// Live handles onto the schema-serving maps the query surface
    /// reads — cloned out via [`Self::schema_serving_handles`] BEFORE the gateway
    /// moves onto its drive thread, so a later `register_egress` can merge its
    /// serving into the running plane (the netd backfill seam).
    serving_handles: SchemaServingHandles,
    /// Cumulative control records the gateway REFUSED to apply
    /// (`register_runtime_topic` returned `Err` — e.g. a hostile reserved-name
    /// record that slipped past the writer-side guard). Counted (Principle #3);
    /// the drain never wedges on one.
    reg_rejected: u64,
    /// First rejected record logs `warn!`, repeats `debug!` (flood
    /// suppression — a persistent flooder must not spam the log).
    reg_rejected_warned: bool,
    /// Cumulative LEGITIMATE control records whose apply hit a
    /// transient ANNOUNCE failure (`register_runtime_topic` returned a non-
    /// `InvalidTransportConfig` `Err` — e.g. a zenoh session / liveliness-declare
    /// blip). Distinct from [`Self::reg_rejected`]: the topic was NOT refused —
    /// its demand flag is RETAINED (grantable) and `register_runtime_topic` already
    /// warned once about the retained-but-unannounced state; it self-HEALS on the
    /// next drain/republish (the announce is re-attempted idempotently). Counted
    /// (Principle #3); the drain never wedges on one.
    reg_announce_deferred: u64,
    /// The AllowAll-only permissive SHM probe (see [`PermissiveShmProbe`]).
    /// Created + installed on the [`NetworkManager`] at boot (BEFORE the liveliness
    /// watch + demand queryable start, so both demand paths capture it); the
    /// gateway keeps this `Arc` for the Principle-#3 accessors
    /// ([`Self::probe_hit_count`] / [`Self::probe_miss_count`] /
    /// [`Self::probe_loop_refusal_count`]).
    probe: Arc<PermissiveShmProbe>,
    /// The demand-transition wake this gateway's zero-demand idle
    /// wait blocks on, cloned from the manager's [`TopicBridgeManager`] at boot so
    /// the wait never re-acquires it. The SAME `Arc` `enable_bridge` /
    /// `register_topic` bump.
    demand_signal: Arc<super::bridge::DemandSignal>,
    /// Pacing counters (Principle #3), shared so a caller can keep
    /// observing after this gateway is moved onto a drive thread.
    drive_stats: Arc<GatewayDriveStats>,
    /// Test seam (`#[doc(hidden)]`, never set in production): a
    /// hook run INSIDE [`Self::idle_wait`] between the flag scan and the blocking
    /// wait — i.e. exactly in the lost-wakeup race window. It is the only way to
    /// make that window DETERMINISTIC from a test: a hook that flips demand ON
    /// must still be served immediately, which holds iff the generation was
    /// snapshotted BEFORE the scan.
    #[allow(clippy::type_complexity)]
    idle_wait_gate: Option<Arc<dyn Fn() + Send + Sync>>,
    /// The DATA-FLOW liveness observer whose shared table stamps every
    /// catalog entry's `liveness`. It watches every REGISTERED topic — including
    /// the ones no one has demanded, which is the whole point: those are exactly
    /// the rows whose `producer_count` reads `Some(1)` on a `ros2 attach` robot
    /// whether or not the DDS route ever carries a sample.
    ///
    /// It costs at most ONE gateway subscriber port per topic, never two: while a
    /// topic is being forwarded, [`Self::drive_once`] hands observation to the
    /// EGRESS tap ([`TopicLivenessObserver::set_externally_observed`]) and feeds
    /// the frames that drain already produced
    /// ([`TopicLivenessObserver::note_frames`]), so a demanded topic's liveness is
    /// free.
    liveness: TopicLivenessObserver,
    /// Number of ingress bridges this gateway opened (for [`Self::ingress_count`]).
    ingress_count: usize,
    /// Cumulative demand-reconcile passes applied (Principle #3), bumped
    /// by BOTH the background reconciler thread and the sync seam (they share
    /// these `Arc` counters). Read via [`Self::reconcile_pass_count`].
    reconcile_pass_count: Arc<AtomicU64>,
    /// Observable state (Principle #3): cumulative reconciler-CAUSED egress-flag
    /// TRANSITIONS (false→true) for demanded announced topics — bumped only when
    /// the reconciler's `enable_bridge` genuinely flipped the flag, NOT
    /// on a re-affirm of an already-on flag or a gate-refused non-member. So on a
    /// healthy link where the liveliness subscriber won the race the count stays
    /// 0; a `> 0` value attributes egress to the RECONCILER, not the subscriber.
    /// See [`Self::reconciler_enabled_count`].
    reconciler_enabled_count: Arc<AtomicU64>,
    /// Queryable inversion: PRODUCER-side GET-demand tracking shared
    /// with the demand queryable (writer) and the reconciler thread's expiry
    /// sweep. Its Principle-#3 counters surface via [`Self::demand_grant_count`]
    /// / [`Self::demand_refusal_count`] / [`Self::demand_expiry_count`]. Empty on
    /// a pure-ingress gateway (no queryable declared).
    get_demand_state: Arc<GetDemandState>,
    /// Queryable inversion: DEMANDER-side demand-GET tracking, present
    /// iff this gateway has ingress topics (the demand-GET loop is running). Its
    /// counters surface via [`Self::demand_get_pass_count`] etc.
    demand_get_state: Option<Arc<DemandGetState>>,
    /// The belt's LIVE iteration set for the
    /// synchronous [`Self::reconcile_demand_once`] seam (the background thread owns
    /// its own copy). SEEDED from the boot canonical announce list; the sync seam
    /// refreshes it from the bridge manager's registered set each pass (the belt
    /// feed), so a runtime-registered topic is re-affirmed here exactly as the
    /// production thread re-affirms it. Test-seam-only.
    #[cfg(any(test, feature = "test-helpers"))]
    reconcile_topics: Vec<String>,
    /// The sync seam's absence tracking (the background thread owns its
    /// own). Only touched by [`Self::reconcile_demand_once`].
    #[cfg(any(test, feature = "test-helpers"))]
    reconcile_absence: HashMap<String, u32>,
    /// A CLONE of the [`CatalogSource`] handed to the query surface,
    /// retained so the sync seam [`Self::catalog_serve_for_test`] runs the EXACT
    /// catalog-serve decision (authorizer gate + build) the real `catalog` callback
    /// runs. Cheap (three `Arc`s). Test-only.
    #[cfg(any(test, feature = "test-helpers"))]
    catalog_source: CatalogSource,
    /// A CLONE of the [`SchemaSource`] handed to the query surface, for
    /// the sync seam [`Self::schema_serve_for_test`]. Cheap (one `Arc`). Test-only.
    #[cfg(any(test, feature = "test-helpers"))]
    schema_source: SchemaSource,
    /// A CLONE of the [`RunSource`] handed to the query surface, for
    /// the sync seam [`Self::runs_serve_for_test`]. Cheap (one `Arc` + a
    /// `Duration`). Test-only.
    #[cfg(any(test, feature = "test-helpers"))]
    run_source: RunSource,
}

/// What ONE drive pass's drain of ONE egress tap saw. Split out of
/// [`GatewayRuntime::drive_once`] because determining
/// [`DrainObservation::queue_emptied`] correctly needs a second question put to the
/// tap (see [`GatewayRuntime::drain_egress_tap`]), and returning four loose values
/// from the middle of the forwarding loop read worse than one named struct.
#[derive(Debug, Default)]
struct EgressDrain {
    /// Frames drained — and therefore moved into the caller's `pending`.
    frames: u64,
    /// The largest publisher loan-time `timestamp_ns` across them, or `None` when
    /// none carried a parseable header.
    newest_stamp_ns: Option<u64>,
    /// The largest publisher commit `sequence` across them, or `None`
    /// when none carried a parseable header — the rate estimate's numerator, read
    /// from the same header parse as `newest_stamp_ns`.
    ///
    /// A DEMANDED topic is observed through this drain rather than the observer's
    /// own tap, so without it the very topics a desk is watching would be the only
    /// ones with no rate.
    newest_sequence: Option<u32>,
    /// Publishers attached to the topic at drain time — the
    /// single-writer evidence the rate's sequence basis requires (a multi-writer
    /// topic's `newest_sequence` maximum hops between per-publisher counters).
    writers_seen: Option<u32>,
    /// Whether the drain left the tap's queue EMPTY. OBSERVED (a short read, or a
    /// non-consuming `has_samples()` query after a saturated one), never inferred
    /// from arithmetic.
    queue_emptied: bool,
    /// The drain error, if any. The partial fill is kept and forwarded.
    error: Option<TransportError>,
}

impl GatewayRuntime {
    /// Boot a gateway on `manager` for `plan`.
    ///
    /// `manager` MUST carry a network transport (a loud `Err` naming the
    /// `network:` fix otherwise) with a `robot_identity` set (announce keys
    /// carry the robot chunk). Boot, in order: validate the plan
    /// (loop safety), install the egress posture on the bridge gate, register
    /// every announce topic's bridge flag (so a remote demand token can flip
    /// it), start the demand watch, declare the BARE identity announce token
    /// (`cerulion_ann/{robot}` — a zero-egress gateway still surfaces a ROBOTS
    /// row), declare an egress-presence announce token per announce topic, and
    /// register every ingress bridge.
    ///
    /// The egress posture is installed BEFORE the watch starts, so no demand
    /// token can flip a flag before the allow-list gate is in place.
    ///
    /// This boots with NO schema serving (catalog carries no schema names, the
    /// `schema` verb serves nothing), exactly the behavior before schema serving existed.
    /// The production `graph run` path uses [`Self::new_with_schema_serving`],
    /// handing the CLI-computed [`SchemaServing`] across the process boundary.
    pub fn new(manager: Arc<TransportManager>, plan: GatewayPlan) -> TransportResult<Self> {
        Self::new_with_schema_serving(manager, plan, SchemaServing::default())
    }

    /// Boot a gateway on `manager` for `plan`, ALSO serving the
    /// `catalog` verb's per-topic schema NAMES and the `schema` verb's `.msg`/YAML
    /// closure from `serving` (built by the CLI from the workspace — a network-free
    /// cerulion_core gateway cannot reach the `.msg` store / built-in corpus, so it
    /// is handed the pre-computed docs). An empty `serving` is identical to
    /// [`Self::new`].
    pub fn new_with_schema_serving(
        manager: Arc<TransportManager>,
        plan: GatewayPlan,
        serving: SchemaServing,
    ) -> TransportResult<Self> {
        plan.validate()?;
        if manager.network().is_none() {
            return Err(TransportError::InvalidTransportConfig {
                reason: "a network gateway requires a configured network transport, but this \
                         TransportManager has none — add the graph's `network:` block \
                         or set `TransportConfig.network = Some(NetworkConfig { .. })` before \
                         initializing the transport"
                    .to_string(),
            });
        }

        // 1. Install the egress posture FIRST (before the watch can flip a flag).
        match &plan.egress_policy {
            GatewayEgressPolicy::AllowAll => manager.bridge_manager().set_egress_allow_all()?,
            GatewayEgressPolicy::AllowList(list) => {
                manager.bridge_manager().set_egress_allowlist(list)?
            }
        }

        // 1b. Create + install the AllowAll-only permissive SHM probe
        //     (see `PermissiveShmProbe`) BEFORE the liveliness watch (step 3) + the
        //     demand queryable (step 3b) start, so BOTH demand paths capture it and
        //     can serve a live unregistered runtime topic on demand. The probe
        //     holds a WEAK ref to `manager` (breaking the manager → NetworkManager
        //     → probe cycle that would leak the manager); the gateway keeps this
        //     `Arc` for the Principle-#3 accessors. It NO-OPS under a Strict posture
        //     (installed above) — the "Strict never probes" decision.
        // Boot TOCTOU close: hand the probe the plan's DECLARED ingress set
        // (canonical) NOW — before step 4 opens any ingress SHM service. The dynamic
        // ingress map is populated only at the END of `register_ingress`, but the
        // ingress SHM service goes live EARLIER at `create_ingress_publisher`; a
        // demand landing in that window would otherwise probe-serve the gateway's own
        // ingress topic (a permanent echo loop). The static set refuses it
        // structurally regardless of dynamic-map timing.
        // hot-path-alloc-ok: boot — one set per gateway.
        let declared_ingress: HashSet<String> = plan
            .ingress
            .iter()
            .map(|e| canonical_topic(&e.topic).into_owned())
            .collect();
        // hot-path-alloc-ok: boot — one Arc per gateway.
        let probe = Arc::new(PermissiveShmProbe::new(
            Arc::downgrade(&manager),
            declared_ingress,
        ));
        manager
            .network()
            .expect("network present (checked above)")
            .set_permissive_probe(Arc::clone(&probe));

        // 2. Register a bridge flag per announce topic (topic-keyed — the demand
        //    watch flips these; there are no publishers in a gateway process).
        //    Every alloc in this file is boot / network-forward: the gateway is
        //    the network-forwarder cold path (sibling of the excluded
        //    network.rs), never the zero-copy message hot path. The counter maps
        //    + `egress_flags` view are built from the bridge manager's
        //    registered snapshot BELOW (step 2b) so a canonical duplicate in
        //    `plan.announce` is deduped exactly as the source of truth is —
        //    keeping the drive-loop dirty-check (registered_count vs
        //    egress_flags.len()) stable from the first pass.
        let mut boot_topics: HashSet<String> = HashSet::new(); // hot-path-alloc-ok: boot
        for topic in &plan.announce {
            let canonical = canonical_topic(topic).into_owned();
            manager.bridge_manager().register_topic(&canonical)?;
            boot_topics.insert(canonical);
        }

        // 3. Start the demand watch (latched-idempotent) so remote demand tokens
        //    flip the announce flags; then declare the presence tokens: the BARE
        //    identity token FIRST (`cerulion_ann/{robot}`, so an
        //    ingress-only / zero-egress gateway still surfaces a `topic list`
        //    ROBOTS row even where mDNS cannot reach), then one egress-presence
        //    token per announced topic.
        manager.start_network_bridge_watch()?;
        manager.announce_gateway_identity()?;
        for topic in &plan.announce {
            manager.announce_egress_topic(topic)?;
        }

        // 2b. Build the drive-loop's `egress_flags` view + the per-topic counter
        //     maps from the bridge manager's registered SNAPSHOT (the source of
        //     truth) — deduped + sorted, so `egress_flags.len()` equals
        //     `registered_topic_count()` from the first pass (the drive-loop
        //     dirty-check stays stable) even if `plan.announce` carried a
        //     canonical duplicate.
        let registered = manager.bridge_manager().registered_topics()?;
        let mut forwarded = HashMap::new();
        let mut forward_warn = HashMap::new();
        let mut attach_failures = HashMap::new();
        let mut attach_warn = HashMap::new();
        for (canonical, _) in &registered {
            forwarded.insert(canonical.clone(), AtomicU64::new(0)); // hot-path-alloc-ok: boot
            forward_warn.insert(canonical.clone(), AtomicBool::new(false)); // hot-path-alloc-ok: boot
            attach_failures.insert(canonical.clone(), AtomicU64::new(0)); // hot-path-alloc-ok: boot
            attach_warn.insert(canonical.clone(), AtomicBool::new(false)); // hot-path-alloc-ok: boot
        }
        let egress_flags: Arc<[(String, Arc<AtomicBool>)]> = registered.into();

        // 2c. The DATA-FLOW liveness observer. Created BEFORE the query
        //     surface (step 3b) so no catalog GET can observe a missing table,
        //     and seeded with the boot announce set so observation of those
        //     topics starts at the first drive pass rather than at the first
        //     runtime registration. Runtime-registered topics join it in
        //     `reconcile_egress_topics`, exactly as they join the drive view.
        let mut liveness = TopicLivenessObserver::new(Arc::clone(&manager));
        liveness.track_all(egress_flags.iter().map(|(t, _)| t.as_str()));

        // 3b. Start the demand path (BOTH halves of the queryable
        //     inversion), sharing the Principle-#3 counters below; all threads /
        //     the queryable are stopped in `NetworkManager::Drop`.
        //
        //     PRODUCER half (this gateway ANNOUNCES): declare the verb-dispatched
        //     QUERY SURFACE (`cerulion_q/{robot}/**`; an accepter
        //     DECLARATION that reaches a connect-only dialer, serving both the
        //     `demand` and `catalog` verbs) so a remote demand GET pulls egress ON
        //     (and a catalog GET is answered), and
        //     start the demand RECONCILER + EXPIRY thread (the ~1 s liveliness-
        //     query belt to the subscriber watch's suspenders + the GET-expiry
        //     sweep that releases a topic once BOTH demand paths go absent).
        //
        //     Empty boot plan: gating this whole demand
        //     surface on `!egress_flags.is_empty()` would be wrong — a gateway with ZERO
        //     boot-announce topics would start NO queryable, so no demand would ever arrive
        //     and the control-channel registrations + the AllowAll SHM probe
        //     would be unreachable. A GENERIC `ros2 attach` robot has ZERO
        //     YAML-declared egress topics (all ~90 are runtime raw routes) → an
        //     EMPTY boot plan is a LEGITIMATE state that FILLS at runtime. So the
        //     surface starts whenever the gateway CAN gain egress topics at
        //     runtime — i.e. under a PERMISSIVE (`AllowAll`) posture — OR already
        //     has boot topics. The queryable must run so remote demand arrives (and
        //     triggers the probe); the reconciler must run so its GET-expiry sweep
        //     RELEASES runtime-granted topics when demand ends (the sweep operates
        //     on `get_state`, which fills at runtime). The real constraint such a
        //     gate would protect — don't spin a demand surface for a gateway that can
        //     NEVER egress (an ingress-only DENY-ALL gateway; Strict never probes) —
        //     is PRESERVED: an empty NON-permissive plan still starts nothing
        //     (`start_when_empty = false`).
        //
        //     NOTE (the belt feed): the reconciler thread's
        //     LIVENESS reconcile is SEEDED with the boot announce set but REFRESHES
        //     it from the bridge manager's LIVE registered set each pass
        //     (`refresh_reconcile_topics`, monotonic-count dirty-checked), so a
        //     topic REGISTERED AT RUNTIME is made demandable via the queryable +
        //     liveliness-subscriber paths AND re-affirmed by the ~1 s reconciler
        //     liveliness BELT (not only the boot topics), and released by the
        //     GET-expiry sweep. The belt now covers runtime topics on any link
        //     where the demand token crosses.
        let announced_canonical: Arc<[String]> = egress_flags
            .iter()
            .map(|(canonical, _)| canonical.clone())
            .collect();
        let reconcile_pass_count = Arc::new(AtomicU64::new(0));
        let reconciler_enabled_count = Arc::new(AtomicU64::new(0));
        let get_demand_state = Arc::new(GetDemandState::default());
        // The shared runtime-hash table + boot-topic provenance the
        // `catalog` verb reads. Created HERE (before the query surface starts, step
        // 3b) so its `Arc`s thread into BOTH the catalog source (read on the zenoh
        // callback thread) and this gateway's drive thread (the sole hash writer).
        // The live TOPIC set for a catalog still comes from the bridge manager's
        // registered set, so runtime registrations catalog correctly even before
        // the drive thread records their hash.
        let reg_hashes: Arc<Mutex<HashMap<String, u64>>> = Arc::new(Mutex::new(HashMap::new()));
        let catalog_boot_topics: Arc<HashSet<String>> = Arc::new(boot_topics.clone());
        // Materialize the CLI-handed schema serving into the
        // sources the query-surface callback reads. `topic_schemas` is
        // canonical-topic-keyed (mirrors the announce/hash keying); `schema_docs`
        // is qualified-name-keyed for the closure walk. SEEDED at boot;
        // Not frozen — a later registration's serving MERGES in
        // through [`SchemaServingHandles`] (the netd backfill seam), which is why
        // the maps are `Mutex`-guarded like the reg-channel hash table above.
        let catalog_topic_schemas: Arc<Mutex<HashMap<String, String>>> = Arc::new(Mutex::new(
            serving
                .topic_schemas
                .iter()
                .map(|ts| {
                    (
                        canonical_topic(&ts.topic).into_owned(),
                        ts.schema_name.clone(),
                    )
                })
                .collect(),
        ));
        // Materialize the CLI-computed hash→name bindings
        // into the reverse map the catalog reads to NAME a runtime-registered topic
        // (a `ros2 attach` raw route) by ITS reg-channel hash.
        let catalog_hash_names: Arc<Mutex<HashMap<u64, String>>> = Arc::new(Mutex::new(
            serving
                .schema_hashes
                .iter()
                .map(|b| (b.schema_hash, b.qualified.clone()))
                .collect(),
        ));
        let schema_source = SchemaSource {
            docs: Arc::new(Mutex::new(
                serving
                    .schema_docs
                    .iter()
                    .map(|d| (d.qualified.clone(), d.clone()))
                    .collect(),
            )),
        };
        // The live-merge handles a later `register_egress` backfills
        // through — cloned Arcs onto the SAME maps the query surface reads.
        let serving_handles = SchemaServingHandles {
            topic_schemas: Arc::clone(&catalog_topic_schemas),
            hash_names: Arc::clone(&catalog_hash_names),
            docs: Arc::clone(&schema_source.docs),
        };
        // The catalog source the query surface reads. The gateway retains a
        // CLONE (below, test-gated) so the `catalog_serve_for_test` seam runs the
        // EXACT gated decision — a cheap three-`Arc` clone.
        let catalog_source = CatalogSource {
            hashes: Arc::clone(&reg_hashes),
            boot_topics: Arc::clone(&catalog_boot_topics),
            topic_schemas: Arc::clone(&catalog_topic_schemas),
            hash_names: Arc::clone(&catalog_hash_names),
            // A WEAK handle to THIS gateway's manager so the catalog serve
            // probes each topic's live producer count (the desk's per-row liveness
            // affordance). Weak breaks the manager → NetworkManager → query-surface →
            // CatalogSource → manager cycle (the PermissiveShmProbe precedent).
            producer_probe: Some(Arc::downgrade(&manager)),
            // The DATA-FLOW liveness table the observer below feeds. The
            // observer is created here (before the query surface starts) so no
            // catalog GET can ever race ahead of the table existing.
            liveness: Some(liveness.table()),
            liveness_clock: Some(manager.clock_arc()),
        };
        // The source the `runs` verb gathers through. It carries THIS
        // gateway's OWN namespace, never the process-global one — the run registry
        // is keyed by iceoryx2 namespace, and a gather on the wrong one answers a
        // confident empty about a machine that is running something.
        // Measured: both shipping gateway hosts (netd's shared gateway and the
        // per-run child) share the namespace the registry writer publishes on, and
        // threading the manager's config is what stays correct when they do
        // NOT (a Strict posture / a divergent `IOX2_CONFIG_FILE`).
        let run_source = RunSource {
            iox_config: Arc::new(manager.iox_config()),
            gather_window: RUN_GATHER_WINDOW,
        };
        // Keep test-only clones BEFORE the sources move into the query
        // surface (the seams reconstruct the exact serve decision the callback runs).
        #[cfg(any(test, feature = "test-helpers"))]
        let catalog_source_for_seam = catalog_source.clone();
        #[cfg(any(test, feature = "test-helpers"))]
        let schema_source_for_seam = schema_source.clone();
        #[cfg(any(test, feature = "test-helpers"))]
        let run_source_for_seam = run_source.clone();
        let permissive = matches!(plan.egress_policy, GatewayEgressPolicy::AllowAll);
        if !egress_flags.is_empty() || permissive {
            let network = manager.network().expect("network present (checked above)");
            network.start_query_surface(
                manager.bridge_manager_arc(),
                Arc::clone(&get_demand_state),
                catalog_source,
                schema_source,
                run_source,
            )?;
            network.start_demand_reconciler(
                manager.bridge_manager_arc(),
                announced_canonical.to_vec(),
                Arc::clone(&reconcile_pass_count),
                Arc::clone(&reconciler_enabled_count),
                Arc::clone(&get_demand_state),
                // Start the reconciler even on an EMPTY announce set when permissive
                // (for the GET-expiry sweep of runtime-granted topics); a
                // non-empty set starts it regardless of this flag.
                permissive,
            )?;
        }

        // 4. Open each ingress bridge (validate + re-inject; the loop-exclusion
        //    guard rejects a topic already announced — belt-and-suspenders with
        //    `plan.validate` above).
        for entry in &plan.ingress {
            manager.register_ingress_topic(
                &entry.topic,
                entry.schema_hash,
                MaxSliceLen::const_new(crate::graph::config::DEFAULT_MAX_SLICE_LEN as u32),
            )?;
        }

        // 4b. DEMANDER half (this gateway has INGRESS): start the demand-GET loop
        //     — every ~2 s it GETs each ingress topic's egress ON at the producer
        //     (a dialer QUERY that reaches a listening accepter — the direction
        //     that survives a strict connect-only link where the ingress DEMAND
        //     token declaration above never forwards).
        let demand_get_state = if plan.ingress.is_empty() {
            None
        } else {
            let ingress_topics: Vec<String> = plan
                .ingress
                .iter()
                .map(|e| canonical_topic(&e.topic).into_owned())
                .collect();
            let state = Arc::new(DemandGetState::from_topics(ingress_topics));
            manager
                .network()
                .expect("network present (checked above)")
                .start_demand_get_loop(Arc::clone(&state))?;
            Some(state)
        };

        // Open the READER on the runtime-registration control service.
        // Non-fatal on failure: a gateway with only boot-plan topics must still
        // run, so a control-service open failure degrades to `None` with a LOUD
        // warn (observable via `runtime_registration_active`) rather than aborting
        // the whole network plane.
        let reg_reader = match manager.create_registration_reader() {
            Ok(reader) => Some(reader),
            Err(e) => {
                tracing::warn!(
                    error = %e,
                    "gateway could not open the runtime-registration control service — \
                     runtime raw-route registrations (e.g. a `ros2 attach` dds_bridge's) will NOT \
                     be picked up; boot-plan announce/ingress topics are unaffected"
                );
                None
            }
        };

        tracing::info!(
            announce = plan.announce.len(),
            ingress = plan.ingress.len(),
            allow_all = matches!(plan.egress_policy, GatewayEgressPolicy::AllowAll),
            runtime_registration = reg_reader.is_some(),
            "network gateway booted"
        );

        Ok(Self {
            demand_signal: manager.bridge_manager().demand_signal(),
            drive_stats: Arc::new(GatewayDriveStats::default()),
            idle_wait_gate: None,
            manager,
            egress_flags,
            boot_topics,
            taps: HashMap::new(),
            forwarded,
            forward_warn,
            attach_failures,
            attach_warn,
            // hot-path-alloc-ok: boot — the reusable drain buffer, allocated once.
            drain_buf: Vec::new(),
            reg_reader,
            // hot-path-alloc-ok: boot — reusable reg-channel drain buffer.
            reg_drain_buf: Vec::new(),
            reg_hashes,
            serving_handles,
            reg_rejected: 0,
            reg_rejected_warned: false,
            reg_announce_deferred: 0,
            probe,
            liveness,
            ingress_count: plan.ingress.len(),
            reconcile_pass_count,
            reconciler_enabled_count,
            get_demand_state,
            demand_get_state,
            #[cfg(any(test, feature = "test-helpers"))]
            reconcile_topics: announced_canonical.to_vec(),
            #[cfg(any(test, feature = "test-helpers"))]
            reconcile_absence: HashMap::new(),
            #[cfg(any(test, feature = "test-helpers"))]
            catalog_source: catalog_source_for_seam,
            #[cfg(any(test, feature = "test-helpers"))]
            schema_source: schema_source_for_seam,
            #[cfg(any(test, feature = "test-helpers"))]
            run_source: run_source_for_seam,
        })
    }

    /// Boot a gateway EMBEDDED in an already-init'd, SHARED
    /// [`TransportManager`] — the seam `cerulion-netd`'s egress plane uses so ONE
    /// zenoh session serves BOTH the desk's ingress mirrors AND its graphs' egress
    /// (Principle #8: one session per process). UNLIKE the per-run
    /// `graph run-gateway` path — which builds its OWN detached manager (via
    /// `TransportManager::detached_with_config`) and boots with the graph's FULL
    /// plan — the desk daemon owns the singleton manager and grows the egress set at
    /// RUNTIME: each desk graph pushes its produced topics via
    /// [`TransportManager::register_dynamic_egress_topic`], which this gateway's
    /// runtime-registration reader ([`Self::register_runtime_topic`]) picks up on
    /// its next drive pass. This additive constructor changes NOTHING about the
    /// existing [`Self::new`] / [`Self::new_with_schema_serving`] paths — it is a
    /// thin, intention-revealing wrapper that pins the desk-embedding boot shape.
    ///
    /// The boot plan is therefore an EMPTY announce set under the PERMISSIVE
    /// (`AllowAll`) posture — the desk default (topic egress allow-lists are not
    /// authorization; by the account-scoped access decision the account
    /// layer gates access). `AllowAll` is also exactly the posture that starts the
    /// demand query surface + reconciler on an empty boot plan, so
    /// a topic registered at runtime becomes demandable with no restart; an empty
    /// non-permissive plan would start no demand surface and runtime egress would
    /// never fire.
    ///
    /// The desk mirror-loop is prevented WITHOUT any change here: on the ONE shared
    /// session the permissive SHM probe and the live demand-grant path both EXCLUDE
    /// any topic the shared manager has registered as INGRESS
    /// (`is_ingress_registered` / the `self_ingress` set), so a topic
    /// netd mirrors IN is never re-egressed. netd ADDITIONALLY refuses a conflicting
    /// registration LOUDLY at its control seam (the cross-plan loop guard in
    /// `cerulion_netd`'s registry) — belt to the core's structural suspenders.
    ///
    /// `serving` carries the catalog/schema payload the desk resolved for its first
    /// egress graph; an empty [`SchemaServing`] is identical to a bare boot.
    pub fn new_embedded(
        manager: Arc<TransportManager>,
        serving: SchemaServing,
    ) -> TransportResult<Self> {
        Self::new_with_schema_serving(
            manager,
            GatewayPlan {
                egress_policy: GatewayEgressPolicy::AllowAll,
                announce: Vec::new(),
                ingress: Vec::new(),
            },
            serving,
        )
    }

    /// The gateway's network-configured transport manager.
    pub fn manager(&self) -> &TransportManager {
        &self.manager
    }

    /// Clone the live schema-serving handles — the merge seam a caller
    /// keeps AFTER moving this gateway onto its drive thread, so a later
    /// registration's [`SchemaServing`] (custom hash→name bindings + served docs)
    /// reaches the running query surface instead of being frozen out by the boot
    /// snapshot. Cheap (three `Arc`s).
    pub fn schema_serving_handles(&self) -> SchemaServingHandles {
        self.serving_handles.clone()
    }

    /// Install the demand-authorization gate the LAN (zenoh) demand-grant
    /// consults before serving a topic (the access boundary). Forwards to the
    /// [`NetworkManager`](super::network::NetworkManager); takes effect LIVE (the demand
    /// callbacks re-read the slot). The LAN plane carries no authenticated identity yet;
    /// the default is the deny-nothing
    /// [`AllowAllAuthorizer`](super::demand_authorizer::AllowAllAuthorizer).
    pub fn set_demand_authorizer(
        &self,
        authorizer: Arc<dyn super::demand_authorizer::DemandAuthorizer>,
    ) {
        if let Some(net) = self.manager.network() {
            net.set_demand_authorizer(authorizer);
        }
    }

    /// Run the gateway until `exit` flips true. Each pass drives egress
    /// once; a pass that forwarded nothing yields the CPU for one
    /// `GATEWAY_IDLE_POLL` tick (backlog-aware pacing), one that forwarded
    /// loops again immediately to drain a backlog. Ingress runs on the zenoh
    /// callback thread and needs no drive.
    /// A pass that forwarded nothing now calls [`Self::idle_wait`],
    /// which paces at `GATEWAY_IDLE_POLL` while any demand is ON and BLOCKS on the
    /// demand signal when none is. Consequence for teardown, stated because it is
    /// real: `exit` is an external flag this loop cannot be woken by, so a
    /// zero-demand gateway observes it up to [`GATEWAY_ZERO_DEMAND_IDLE`] late.
    /// A caller that wants a prompt join pokes
    /// `manager.bridge_manager().demand_signal().signal()` after setting `exit`
    /// (netd's egress plane does).
    pub fn run(&mut self, exit: &AtomicBool) -> TransportResult<()> {
        while !exit.load(Ordering::Relaxed) {
            let forwarded = self.drive_once()?;
            if forwarded == 0 {
                self.idle_wait();
            }
        }
        Ok(())
    }

    /// Pace one idle drive pass — the ONE place the gateway
    /// sleeps, shared by [`Self::run`], `cerulion-netd`'s egress drive thread and
    /// `cerulion graph run-gateway`'s inline loop (three copies of the same loop
    /// body; one pacing rule).
    ///
    /// * **Any egress flag ON** ⇒ `sleep(GATEWAY_IDLE_POLL)`, byte-identical to
    ///   the polling loop. This is the state every liveness band was
    ///   derived against, so none of them moves.
    /// * **No flag ON** ⇒ block on the demand signal for up to
    ///   [`GATEWAY_ZERO_DEMAND_IDLE`].
    ///
    /// The zero-demand arm paces the liveness sweep as a side effect (this
    /// park is the only thing calling `drive_once` there, and `drive_once` is the
    /// only production caller of `sweep`, whose own throttle is a floor with no
    /// catch-up). That is why [`GATEWAY_ZERO_DEMAND_IDLE`] is DERIVED from
    /// `LIVENESS_SWEEP_INTERVAL_NS` and const-asserted `<=` it, rather than chosen:
    /// so the bands stay true in BOTH arms instead of only the demanded one.
    ///
    /// **The ORDERING here is the whole correctness argument (Principle #6).** The
    /// generation is snapshotted BEFORE the flag scan, never after. A demand
    /// landing anywhere from the first atomic load to the instant the condvar
    /// mutex is taken bumps the generation past `seen`, and
    /// [`DemandSignal::wait_for_change`](super::bridge::DemandSignal::wait_for_change)
    /// then returns without blocking. Snapshotting after the scan would leave
    /// exactly that window open, and the demand would wait out the whole fallback
    /// timeout — the lost-wakeup class this repo refuses to ship.
    pub fn idle_wait(&self) {
        // Snapshot FIRST — see the ordering note above.
        let seen = self.demand_signal.generation();
        if self.any_demand_on() {
            std::thread::sleep(GATEWAY_IDLE_POLL);
            return;
        }
        // Test seam: run INSIDE the race window (between the scan and the wait).
        if let Some(gate) = &self.idle_wait_gate {
            gate();
        }
        self.drive_stats
            .zero_demand_waits
            .fetch_add(1, Ordering::Relaxed);
        if self
            .demand_signal
            .wait_for_change(seen, GATEWAY_ZERO_DEMAND_IDLE)
        {
            self.drive_stats
                .demand_wakes
                .fetch_add(1, Ordering::Relaxed);
        }
    }

    /// Whether ANY registered egress topic currently has demand.
    /// One relaxed load per registered flag — the same scan `drive_once` runs, paid
    /// once per idle wait (≈5 Hz at zero demand) instead of ~1000 Hz.
    pub fn any_demand_on(&self) -> bool {
        self.egress_flags
            .iter()
            .any(|(_, flag)| flag.load(Ordering::Relaxed))
    }

    /// Observable state (Principle #3): the drive loop's pacing counters. Cloneable
    /// so a caller can keep observing after this gateway is moved onto a thread.
    pub fn drive_stats(&self) -> Arc<GatewayDriveStats> {
        Arc::clone(&self.drive_stats)
    }

    /// Test seam (`#[doc(hidden)]`, never called in production):
    /// install a hook run inside [`Self::idle_wait`] BETWEEN the flag scan and the
    /// blocking wait, i.e. exactly in the lost-wakeup race window. A hook that
    /// flips demand ON must still be served immediately.
    #[doc(hidden)]
    pub fn set_idle_wait_gate_for_test(&mut self, gate: Arc<dyn Fn() + Send + Sync>) {
        self.idle_wait_gate = Some(gate);
    }

    /// One egress drive pass — the single-iteration test seam of
    /// [`Self::run`]. For every announce topic: attach a tap if its bridge flag
    /// is ON and none exists, drain the tap and forward each frame to zenoh, or
    /// drop the tap if the flag is OFF. Returns the number of frames
    /// SUCCESSFULLY forwarded this pass.
    ///
    /// Per-topic failures are NON-FATAL: a
    /// routine per-topic transient — a producer worker restart re-creating its
    /// service, subscriber-slot exhaustion — must never kill the process-wide
    /// network plane. An ATTACH failure is counted + warn-latched and retried
    /// next pass; a DRAIN failure forwards the partial fill, is counted +
    /// warn-latched, and DROPS the tap (the canonical cause is a torn-down/
    /// re-created producer service a stale tap can never see) so the next pass
    /// re-attaches a fresh one. The flag stays ON throughout.
    ///
    /// Each pass first RECONCILES the egress view against the bridge
    /// manager's registered-flag map (the source of truth) — via the private
    /// `reconcile_egress_topics` — so a topic registered at RUNTIME
    /// ([`Self::register_runtime_topic`], fed by the control service /
    /// the SHM probe) joins the tap set here with no restart.
    pub fn drive_once(&mut self) -> TransportResult<usize> {
        // One bump per pass (Principle #3): the observable a
        // zero-demand CEILING is asserted on, and the only way to tell "paced by
        // the demand wait" from "spinning" without a profiler (the
        // `conn_read_loop_iterations` precedent).
        self.drive_stats.passes.fetch_add(1, Ordering::Relaxed);
        // Drain the runtime-registration control service FIRST so any
        // topics a network-free worker pushed since the last pass are registered
        // (via `register_runtime_topic`) BEFORE the reconcile below pulls them
        // into the drive view — a same-pass join, no extra pass of latency.
        self.drain_registration_channel();
        // Pull any runtime-registered topics into the drive view (cheap
        // count-dirty-check; a snapshot + lazy counter mint only when it grew).
        self.reconcile_egress_topics()?;
        // Iterate a clone of the current flag list so the loop body is free to
        // mutate `self.taps` / `self.drain_buf` (reconcile above is the only
        // writer of `self.egress_flags`, and it already ran this pass).
        let flags = Arc::clone(&self.egress_flags);
        let mut forwarded_total = 0usize;
        for (topic, flag) in flags.iter() {
            if flag.load(Ordering::Relaxed) {
                // Attach a tap lazily on the first ON pass. A failure here is
                // non-fatal (the flag stays on, so the next pass retries) —
                // slot exhaustion or a not-yet-created producer service — and
                // once-per-regime latched + counted so it is loud exactly once
                // per regime and never silent (Principle #3).
                if !self.taps.contains_key(topic) {
                    // ORDERING IS LOAD-BEARING: release the liveness
                    // observer's own tap BEFORE attempting the egress attach.
                    // Both are subscriber ports on the same topic, and a topic
                    // provisions only INTROSPECTION_SUBSCRIBER_HEADROOM spare
                    // slots — shared with bagd, `topic echo`, and vizd. Attaching
                    // egress first would make the two COMPETE for the last slot,
                    // and a failed attach `continue`s before the hand-off, so the
                    // observer would keep the slot forever and egress would be
                    // starved PERMANENTLY. Releasing first guarantees egress gets
                    // first refusal on the slot the observer was holding, so this
                    // feature can never cost a topic its ability to egress.
                    //
                    // This is deliberately NOT `set_externally_observed(true)`:
                    // observation may only be declared external once a drainer
                    // actually EXISTS (below, on the success arm). Releasing is
                    // also free when the observer holds nothing — one hash lookup,
                    // no allocation, no table lock — which is what keeps a
                    // SUSTAINED attach-failure regime cheap: this runs on every
                    // ~1 kHz drive pass while the observer only re-attaches once
                    // per 200 ms sweep.
                    self.liveness.release_own_tap(topic);
                    match self.manager.create_data_only_subscriber(topic) {
                        Ok(tap) => {
                            self.record_attach_success(topic);
                            tracing::debug!(topic = %topic, "gateway egress tap attached");
                            // NOW there is a real drainer — hand
                            // observation to it (one gateway port per topic, and
                            // the interval opens on this freshly attached port).
                            self.liveness.set_externally_observed(topic, true);
                            // hot-path-alloc-ok: once per demand transition (a
                            // remote (un)subscribing), not per forwarded frame.
                            self.taps.insert(topic.clone(), tap);
                        }
                        Err(e) => {
                            self.record_attach_failure(topic, &e);
                            // Nothing is draining this topic. External observation was
                            // never claimed above, so in the steady
                            // failure regime this is a hash lookup that early-
                            // returns; it is load-bearing only on the pass after a
                            // DRAIN failure dropped a tap that WAS external, where
                            // it hands observation back rather than banking time
                            // nothing is doing. Either way the observer re-attaches
                            // on its next sweep and the release above still gives
                            // egress first refusal on the slot next pass.
                            self.liveness.set_externally_observed(topic, false);
                            continue;
                        }
                    }
                }
                // Drain the tap into a local vec of frame bytes, so the SHM
                // sample borrows are released before forwarding (which holds
                // &self). The drain also OBSERVES whether it left the
                // queue empty, which is what closes the advancement baseline —
                // see `drain_egress_tap`.
                let mut pending: Vec<Vec<u8>> = Vec::new();
                let drain = self.drain_egress_tap(topic, &mut pending);
                let drain_err = drain.error;
                let newest_ts = drain.newest_stamp_ns;
                // The rate estimate's numerator for a DEMANDED topic.
                let newest_seq = drain.newest_sequence;
                // How many frames the tap OBSERVED this pass — captured
                // before `pending` is consumed by the forward loop. Deliberately
                // the DRAINED count, not the FORWARDED one: a frame that reached
                // SHM but failed to reach zenoh is still proof the topic is alive,
                // and conflating the two would report an unreachable peer as a dead
                // producer.
                let drained = drain.frames;
                for frame in pending {
                    if self.forward_frame(topic, frame) {
                        forwarded_total += 1;
                    }
                }
                // The egress tap IS this topic's liveness observer while
                // demand lasts — the frames it just drained are the observation, so
                // a demanded topic pays nothing extra (no second port, no second
                // drain). The `true` affirmation is a hash lookup that early-returns
                // in the steady state (the attach arm above already declared it);
                // it exists so the invariant "an attached egress tap is the topic's
                // observer" cannot drift. A pass that drained NOTHING needs no
                // `note_frames` at all — the interval is already open, so a zero
                // report would only cost a table lock + a clock read per pass per
                // demanded topic, on a loop that spins as fast as backlog demands.
                self.liveness.set_externally_observed(topic, true);
                if drained > 0 {
                    // `queue_emptied` ENDS the advancement baseline, so it
                    // must be true only when this pass really did empty the tap —
                    // and it must be REACHABLE, or the topic never dates at all.
                    // `drain_egress_tap` OBSERVES it (a short read, or a
                    // non-consuming `has_samples()` query after a saturated one)
                    // rather than inferring it from arithmetic; an
                    // Err still leaves an unknown remainder and reports `false`,
                    // but that path also drops the tap below, so the re-attach
                    // re-opens the baseline anyway.
                    self.liveness.note_frames(
                        topic,
                        DrainObservation {
                            frames: drained,
                            newest_stamp_ns: newest_ts,
                            newest_sequence: newest_seq,
                            writers_seen: drain.writers_seen,
                            queue_emptied: drain.queue_emptied,
                        },
                    );
                }
                if let Some(e) = drain_err {
                    // Counted + once-per-regime latched (a wedged tap must not
                    // warn-spam; the latch re-arms on the next successful
                    // forward). DROP the tap: the canonical cause is a
                    // re-created producer service (worker crash+respawn under
                    // peer-loss continue) that a stale tap can never see — the
                    // flag stays ON, so the next pass re-attaches a fresh tap.
                    // Frames still queued in the broken tap are unrecoverable.
                    self.record_forward_failure(topic, Some(&e));
                    self.taps.remove(topic);
                    // The drainer this topic's observation was riding is
                    // GONE, so end the interval — exactly as the observer does when
                    // its own tap drain fails. Without this the interval stays OPEN
                    // with nothing attached, and the topic keeps serving a verdict
                    // (`no_data` after long enough, or an ever-staler `idle` age)
                    // that no longer rests on anything watching it. The re-attach
                    // next pass resumes with the banked history intact.
                    self.liveness.set_externally_observed(topic, false);
                }
            } else {
                // Demand is OFF, so the egress tap is not observing this
                // topic (whether or not one existed a moment ago) — hand observation
                // back to the observer, which re-attaches its own tap on its next
                // sweep. Idempotent + allocation-free when nothing changed.
                self.liveness.set_externally_observed(topic, false);
                if self.taps.remove(topic).is_some() {
                    tracing::debug!(topic = %topic, "gateway egress tap detached — demand ended");
                }
            }
        }
        // Sweep the observer's own taps. The throttle is a FLOOR, not a
        // cadence — it can only run when CALLED — so this is a clock read on most
        // passes of a DEMANDED loop (1 ms tick) and a real sweep on most passes of a
        // zero-demand one, whose park is derived from that same interval.
        // Runs AFTER the egress loop so this pass's external-observation hand-offs
        // are already reflected and the sweep never double-taps a demanded topic.
        self.liveness.sweep();
        Ok(forwarded_total)
    }

    /// Drain ONE egress tap into `pending` and report what was seen —
    /// including, as OBSERVED, whether the drain left the tap's queue empty.
    ///
    /// The gateway takes ONE borrow budget per drive pass by design (backlog-aware
    /// pacing: [`Self::run`] loops again immediately after a pass that forwarded),
    /// so a read that comes back FULL is ambiguous: the queue may hold more, or
    /// may be exactly empty. That ambiguity is not cosmetic.
    /// [`DrainObservation::queue_emptied`] is the ONLY thing that closes the
    /// advancement baseline, so a drain that can never report `true` leaves its
    /// topic permanently undatable — [`LivenessState::Idle`](super::liveness::LivenessState::Idle)
    /// with no age, never `Streaming`, for as long as demand lasts. Inferring it
    /// as `drained < budget` was additionally UNREACHABLE at `budget == 1` (the
    /// `.max(1)` floor, or a service created at 1): at a site that already knows a
    /// frame arrived, "fewer frames than asked for" is arithmetically impossible.
    ///
    /// So emptiness is OBSERVED, not inferred: after a SATURATED read the tap's
    /// port is ASKED, via the non-consuming
    /// [`DataOnlySubscriber::has_samples`](super::subscriber::DataOnlySubscriber::has_samples).
    /// Non-consuming is the whole point — draining one extra frame would answer
    /// the same question, but it would also change WHAT this pass forwards and
    /// WHEN (a 3-frame burst that crosses as 2+1 would cross as 3 at once,
    /// which a shallow downstream queue can turn into an eviction), and an
    /// emptiness QUERY must not move the data plane. The cost is one
    /// `update_connections()`-shaped check on passes whose read actually
    /// saturated, and nothing at all on every other pass.
    fn drain_egress_tap(&mut self, topic: &str, pending: &mut Vec<Vec<u8>>) -> EgressDrain {
        let Some(tap) = self.taps.get_mut(topic) else {
            return EgressDrain::default();
        };
        let budget = tap.max_borrowed_samples().max(1);
        // Read BEFORE the drain, off the tap's own service handle.
        let writers_seen = Some(tap.publisher_count());
        self.drain_buf.clear();
        // NON-FATAL drain: the error is
        // captured, not `?`-propagated — otherwise one topic's transient receive
        // error (a producer worker restart re-creating its service) would kill run()
        // and with it the robot's ENTIRE network plane. The partial fill is KEPT
        // on Err (`drain_owned` contract) and forwarded below — those frames were
        // already popped off the SHM queue, so dropping them would lose data
        // (Principle #6).
        let mut error = tap.drain_owned(budget, &mut self.drain_buf).err();
        // A SHORT read proves the queue was left empty. A read that FILLED the
        // budget proves nothing either way — that is what the query resolves. An
        // Err leaves an unknown remainder and can never claim emptiness.
        let mut queue_emptied = error.is_none() && self.drain_buf.len() < budget;
        if error.is_none() && self.drain_buf.len() >= budget {
            match tap.has_samples() {
                Ok(more) => queue_emptied = !more,
                Err(e) => error = Some(e),
            }
        }
        // The gateway is the network-forwarder cold path (NOT the zero-copy
        // message hot path). The one heap copy to zenoh (`to_vec`) exists
        // identically in every architecture — it moves network serialization OFF
        // the graph process. Draining `drain_buf` here also releases the SHM
        // sample borrows before the caller's `&self` forward.
        let mut newest_stamp_ns: Option<u64> = None;
        let mut newest_sequence: Option<u32> = None;
        let mut frames = 0u64;
        // hot-path-alloc-ok: network forward.
        pending.reserve(self.drain_buf.len());
        for owned in self.drain_buf.drain(..) {
            if let Some(header) = owned.wire_header() {
                let ts = header.timestamp_ns;
                newest_stamp_ns = Some(newest_stamp_ns.map_or(ts, |seen: u64| seen.max(ts)));
                let seq = header.sequence;
                newest_sequence = Some(newest_sequence.map_or(seq, |seen: u32| seen.max(seq)));
            }
            frames += 1;
            // hot-path-alloc-ok: the single copy to zenoh (network forward).
            pending.push(owned.payload().to_vec());
        }
        EgressDrain {
            frames,
            newest_stamp_ns,
            newest_sequence,
            writers_seen,
            queue_emptied,
            error,
        }
    }

    /// Reconcile the drive view (`self.egress_flags` + the per-topic
    /// counter maps) against the bridge manager's registered-flag map — the ONE
    /// source of truth for which topics may egress. A runtime registration
    /// ([`Self::register_runtime_topic`]) adds a flag to that map; this pulls it
    /// into the drive view so [`Self::drive_once`] taps it.
    ///
    /// CHEAP by construction: the bridge map is ADD-ONLY (register_topic inserts,
    /// nothing removes), so `registered_topic_count()` is monotonic and a bare
    /// count read (one uncontended mutex lock) is a reliable dirty-check. Only
    /// when it exceeds the current view does this take the full snapshot + lazily
    /// mint the four per-topic counters for any newly-seen topic (a snapshot per
    /// registration event, NOT per drive pass). The per-pass steady-state cost is
    /// exactly one count read.
    fn reconcile_egress_topics(&mut self) -> TransportResult<()> {
        let registered_count = self.manager.bridge_manager().registered_topic_count()?;
        if registered_count == self.egress_flags.len() {
            return Ok(());
        }
        // The registered set grew — re-snapshot (deduped + sorted) and lazily
        // mint counter state for any topic we have not seen before.
        let registered = self.manager.bridge_manager().registered_topics()?;
        for (canonical, _) in &registered {
            // hot-path-alloc-ok: once per newly-registered topic (a runtime
            // registration), never per forwarded frame.
            self.forwarded
                .entry(canonical.clone())
                .or_insert_with(|| AtomicU64::new(0));
            self.forward_warn
                .entry(canonical.clone())
                .or_insert_with(|| AtomicBool::new(false));
            self.attach_failures
                .entry(canonical.clone())
                .or_insert_with(|| AtomicU64::new(0));
            self.attach_warn
                .entry(canonical.clone())
                .or_insert_with(|| AtomicBool::new(false));
            // A runtime-registered topic joins the liveness observer at
            // the same moment it joins the drive view — so a `ros2 attach` robot,
            // whose topics are ALL runtime raw routes, observes every one of them.
            self.liveness.track(canonical);
        }
        self.egress_flags = registered.into();
        Ok(())
    }

    /// Drain the `/__cerulion/gateway_topics` control service and feed
    /// every VALID record through [`Self::register_runtime_topic`] (so the
    /// register + announce happen exactly as for a boot-plan topic), storing each
    /// record's advertised `schema_hash` in the topic→hash table (item 5).
    ///
    /// Robust by construction: malformed records are dropped inside
    /// [`RegistrationReader::drain`] (counted + warn-once there), and a record
    /// whose `register_runtime_topic` returns `Err` is counted + SKIPPED here —
    /// one bad record can NEVER wedge the drain or the drive loop. This splits
    /// that `Err` into two DISTINCT classes so the counter + the log tell the truth.
    ///
    /// A REFUSAL ([`TransportError::InvalidTransportConfig`] — a reserved-name /
    /// empty record that slipped past the writer-side guard) routes to
    /// [`Self::reg_rejected`] plus the loud "will NOT be announced or made
    /// demandable" warn (accurate: a refused topic is neither). An ANNOUNCE FAILURE
    /// (any other `Err` — a transient zenoh / liveliness blip on a LEGITIMATE
    /// topic) routes to [`Self::reg_announce_deferred`] plus at most a `debug!` (NO
    /// false "will NOT be announced" warn: the demand flag is RETAINED so the topic
    /// IS demand-grantable, `register_runtime_topic` already warned once about the
    /// retained-but-unannounced state, and it self-heals on the next
    /// drain/republish).
    ///
    /// The reader borrow is released (records moved into `reg_drain_buf`) BEFORE
    /// the `register_runtime_topic` calls, since that takes `&self` on the same
    /// gateway.
    fn drain_registration_channel(&mut self) {
        if self.reg_reader.is_none() {
            return;
        }
        // First: drain into the reusable buffer (scoped &mut borrow of the
        // reader), then release the borrow before the &self register calls.
        self.reg_drain_buf.clear();
        {
            let reader = self.reg_reader.as_mut().expect("reader present (checked)");
            // hot-path-alloc-ok: the reusable reg drain buffer (cold control path).
            reader.drain(&mut self.reg_drain_buf);
        }
        // Then: apply each record. `std::mem::take` moves the records out so we
        // do not hold a borrow of `self.reg_drain_buf` across `&self` register
        // calls; the emptied Vec keeps its capacity for the next pass.
        let records = std::mem::take(&mut self.reg_drain_buf);
        for record in &records {
            match self.register_runtime_topic(&record.topic) {
                Ok(_newly) => {
                    // Store the advertised hash (canonical-keyed) regardless of
                    // new-vs-dup — an idempotent re-register may carry an updated
                    // hash for the same topic.
                    let canonical = canonical_topic(&record.topic).into_owned();
                    self.reg_hashes
                        .lock()
                        .unwrap_or_else(std::sync::PoisonError::into_inner)
                        .insert(canonical, record.schema_hash);
                }
                // Discriminate a genuine REFUSAL (InvalidTransportConfig —
                // reserved/empty) from a transient ANNOUNCE failure of a legit
                // topic. Only the former is "will NOT be announced or made
                // demandable"; the latter retains the (grantable) flag and heals.
                Err(e @ TransportError::InvalidTransportConfig { .. }) => {
                    self.record_registration_rejected(&record.topic, &e)
                }
                Err(e) => self.record_registration_deferred(&record.topic, &e),
            }
        }
        // Return the (now-empty) buffer to reuse its capacity.
        self.reg_drain_buf = records;
        self.reg_drain_buf.clear();
    }

    /// A control record `register_runtime_topic` refused — counted
    /// (Principle #3) + loud once, then the drain continues.
    fn record_registration_rejected(&mut self, topic: &str, error: &TransportError) {
        self.reg_rejected += 1;
        if self.reg_rejected_warned {
            tracing::debug!(
                topic = %topic,
                error = %error,
                "gateway refused a runtime-registration control record \
                 (repeat — see the first warning; the drain continues)"
            );
        } else {
            self.reg_rejected_warned = true;
            tracing::warn!(
                topic = %topic,
                error = %error,
                "gateway refused a runtime-registration control record — it will NOT be \
                 announced or made demandable (counted; the drain continues; repeats log at debug). \
                 A reserved-namespace record is the usual cause (a control-plane name must never \
                 be robot egress)"
            );
        }
    }

    /// A LEGITIMATE control record whose apply hit a transient
    /// ANNOUNCE failure — counted (Principle #3) at `debug!` only. Deliberately
    /// does NOT emit the "will NOT be announced or made demandable" warn: that
    /// claim is FALSE here — the demand flag is retained (grantable),
    /// `register_runtime_topic` already emitted its own once-per-topic
    /// registered-but-unannounced warn, and the announce self-heals on the next
    /// drain/republish. A `debug!` (not a warn) avoids re-flooding for the same
    /// self-healing condition `register_runtime_topic` already surfaced.
    fn record_registration_deferred(&mut self, topic: &str, error: &TransportError) {
        self.reg_announce_deferred += 1;
        tracing::debug!(
            topic = %topic,
            error = %error,
            "gateway: a runtime-registration control record is registered + \
             demand-grantable but its announce is deferred (transient failure; the flag is \
             retained and the announce re-attempts on the next drain — see register_runtime_topic's \
             own warn; counted)"
        );
    }

    /// Observable state (Principle #3): whether the gateway opened the runtime-registration
    /// control-service reader at boot (`false` iff the open failed — a loud warn
    /// was emitted then; runtime raw-route registrations will not be picked up).
    pub fn runtime_registration_active(&self) -> bool {
        self.reg_reader.is_some()
    }

    /// Observable state (Principle #3): the `schema_hash` a producing process advertised
    /// for a RUNTIME-registered `topic` (canonical-keyed), or `None` if the topic
    /// was not runtime-registered via the control channel. Item 5: stored for
    /// future probe/diagnostics; egress forwards frames verbatim, so this never
    /// gates delivery.
    pub fn runtime_topic_schema_hash(&self, topic: &str) -> Option<u64> {
        let canonical = canonical_topic(topic).into_owned();
        self.reg_hashes
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .get(&canonical)
            .copied()
    }

    /// Observable state (Principle #3): cumulative MALFORMED control records dropped at the
    /// drain (bad magic/version, truncated, length-mismatch, invalid UTF-8). `0`
    /// if no reader is open.
    pub fn registration_malformed_count(&self) -> u64 {
        self.reg_reader
            .as_ref()
            .map(|r| r.malformed_count())
            .unwrap_or(0)
    }

    /// Observable state (Principle #3): cumulative control records the gateway REFUSED to
    /// apply (`register_runtime_topic` returned [`TransportError::InvalidTransportConfig`]
    /// — e.g. a hostile reserved-name record on the wire). Distinct from
    /// [`Self::registration_announce_deferred_count`]: a rejected record is
    /// neither announced nor demand-grantable.
    pub fn registration_rejected_count(&self) -> u64 {
        self.reg_rejected
    }

    /// Observable state (Principle #3): cumulative LEGITIMATE control records whose
    /// apply hit a transient ANNOUNCE failure (`register_runtime_topic` returned a
    /// non-`InvalidTransportConfig` `Err`). Unlike a
    /// [`Self::registration_rejected_count`] record, the topic's demand flag is
    /// RETAINED (grantable) and the announce self-heals on the next
    /// drain/republish — so a `> 0` value is a transient-blip signal, not a
    /// permanent refusal.
    pub fn registration_announce_deferred_count(&self) -> u64 {
        self.reg_announce_deferred
    }

    // ---- The AllowAll-only permissive SHM probe (Principle #3) ----

    /// Cumulative permissive SHM probe HITS (Principle #3): remote
    /// demands for an UNregistered topic that the probe found LIVE in local SHM and
    /// registered + announced on the spot (announce-on-first-serve).
    pub fn probe_hit_count(&self) -> u64 {
        self.probe.hit_count()
    }

    /// Cumulative permissive SHM probe MISSES (Principle #3): remote
    /// demands for a topic with NO live local SHM service (refused, nothing
    /// created). Bounded state: a flood of distinct missing names bumps this
    /// counter but warns only once per regime.
    pub fn probe_miss_count(&self) -> u64 {
        self.probe.miss_count()
    }

    /// Cumulative permissive SHM probe LOOP refusals (Principle #3): a
    /// demand for a topic this gateway INGRESSES (its local SHM service is the
    /// gateway's own re-injection — serving it back out is the echo loop)
    /// or a reserved control-plane name. Never probe-served.
    pub fn probe_loop_refusal_count(&self) -> u64 {
        self.probe.loop_refusal_count()
    }

    /// Observable state (Principle #3): cumulative permissive SHM probe HITS whose
    /// network announce FAILED (registered + demand-grantable but not yet
    /// discoverable — the flag is RETAINED). Mirrors
    /// [`Self::registration_announce_deferred_count`]: a `> 0` value is a
    /// transient-blip signal, not a permanent refusal — the topic still egresses on
    /// demand and its announce self-heals on the next demand (the `AlreadyRegistered`
    /// arm re-attempts it idempotently).
    pub fn probe_announce_deferred_count(&self) -> u64 {
        self.probe.announce_deferred_count()
    }

    /// Observable state (Principle #3): whether this gateway's PRODUCER query surface (the
    /// verb-dispatched queryable serving both `demand` and `catalog`) is declared —
    /// the DETERMINISTIC observable that the demand/catalog surface started. The
    /// Part-B pin: an EMPTY-boot-plan gateway under a PERMISSIVE (`AllowAll`)
    /// posture now starts it (so runtime demand can arrive + trigger the probe, and
    /// a `topic list` catalog GET is answered), while an empty NON-permissive
    /// (deny-all / ingress-only) gateway leaves it `false` (it can
    /// never egress).
    pub fn query_surface_active(&self) -> bool {
        self.manager
            .network()
            .map(|n| n.has_query_surface())
            .unwrap_or(false)
    }

    /// SYNC SEAM (queryable path): run the EXACT queryable grant DECISION
    /// (`demand_grant_decision` — permissive SHM probe + `enable_bridge` + ack) for
    /// `topic` inline and report whether it GRANTED (ack ENABLED). The
    /// deterministic, zenoh-timing-free driver for the probe tests: a probe HIT
    /// registers + announces + grants; a MISS / loop-refusal / Strict posture
    /// refuses. Observable side effects (registration, announce, counters) are read
    /// via the Principle-#3 accessors. Exercises the SAME body the real zenoh
    /// queryable callback runs.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn demand_query_grants_for_test(&self, topic: &str) -> bool {
        // Feed the SAME self-ingress exclusion set the production
        // queryable consults, so this seam exercises the self-ingress skip too.
        let net = self
            .manager
            .network()
            .expect("a gateway always has a network manager");
        // Consult the SAME demand authorizer the production queryable
        // reads (installed via `set_demand_authorizer`), so this seam exercises the
        // authorization gate too — a stub deny-authorizer refuses here.
        let authorizer = net.demand_authorizer_for_test();
        super::network::demand_grant_decision_for_test(
            self.manager.bridge_manager(),
            &self.get_demand_state,
            &self.probe,
            net.self_ingress_set_for_test(),
            authorizer.as_ref(),
            topic,
        )
    }

    /// SYNC SEAM (liveliness path): run the EXACT liveliness Put action
    /// (`apply_live_put_demand` — permissive SHM probe + `enable_bridge`) for
    /// `topic` inline and report whether the topic is now ENABLED (grantable). The
    /// deterministic driver proving the liveliness path converges to the SAME
    /// served + grantable state as [`Self::demand_query_grants_for_test`].
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn liveliness_demand_grants_for_test(&self, topic: &str) -> bool {
        // The liveliness path consults the SAME demand authorizer.
        let net = self
            .manager
            .network()
            .expect("a gateway always has a network manager");
        let authorizer = net.demand_authorizer_for_test();
        super::network::apply_live_put_demand_for_test(
            self.manager.bridge_manager(),
            &self.probe,
            authorizer.as_ref(),
            topic,
        )
    }

    /// SYNC SEAM (catalog verb): run the EXACT catalog-serve DECISION
    /// the real `catalog` callback runs — the SAME demand authorizer gate
    /// (`set_demand_authorizer`), then the catalog build — and return the reply. A
    /// REFUSED catalog carries `error: Some(..)` + EMPTY entries (an unauthorized
    /// party enumerates NOTHING); an authorized one carries the live catalog.
    /// Zenoh-timing-free (the reply is built, not sent).
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn catalog_serve_for_test(&self, robot: &str) -> super::cerulion_q::CatalogReply {
        let net = self
            .manager
            .network()
            .expect("a gateway always has a network manager");
        let authorizer = net.demand_authorizer_for_test();
        super::network::catalog_serve_decision_for_test(
            self.manager.bridge_manager(),
            &self.catalog_source,
            authorizer.as_ref(),
            robot,
        )
    }

    /// SYNC SEAM (schema verb): run the EXACT schema-serve DECISION the
    /// real `schema` callback runs — the SAME demand authorizer gate, then the
    /// closure resolution — and return the reply. A REFUSED schema carries
    /// `error: Some(..)` + EMPTY docs (an unauthorized party gets NO `.msg`/YAML
    /// text). Zenoh-timing-free.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn schema_serve_for_test(
        &self,
        robot: &str,
        requested: &str,
    ) -> super::cerulion_q::SchemaReply {
        let net = self
            .manager
            .network()
            .expect("a gateway always has a network manager");
        let authorizer = net.demand_authorizer_for_test();
        super::network::schema_serve_decision_for_test(
            &self.schema_source,
            authorizer.as_ref(),
            robot,
            requested,
        )
    }

    /// SYNC SEAM (runs verb): run the EXACT runs-serve DECISION the
    /// real `runs` callback runs — the SAME demand authorizer gate, then the
    /// serve-time registry gather + run-directory fold — and return the reply. A
    /// REFUSED runs GET carries `error: Some(..)` + NO runs + a NON-settled
    /// verdict (an unauthorized party learns nothing, and cannot read the empty
    /// list as "this robot is running nothing"). Zenoh-timing-free (the reply is
    /// built, not sent).
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn runs_serve_for_test(&self, robot: &str) -> super::cerulion_q::RunsReply {
        let net = self
            .manager
            .network()
            .expect("a gateway always has a network manager");
        let authorizer = net.demand_authorizer_for_test();
        super::network::runs_serve_decision_for_test(&self.run_source, authorizer.as_ref(), robot)
    }

    /// The iceoryx2 namespace this gateway's `runs` verb gathers on:
    /// the Principle-#3 observable that makes the NAMESPACE claim checkable.
    ///
    /// Without it, "the serve threads the manager's config" is asserted by no
    /// test that does not also depend on a live registry writer: a gather on the
    /// WRONG namespace and a gather on the right one with nothing published are
    /// the same empty answer.
    #[cfg(any(test, feature = "test-helpers"))]
    #[must_use]
    pub fn runs_gather_namespace_for_test(&self) -> &iceoryx2::config::Config {
        &self.run_source.iox_config
    }

    /// SYNC SEAM: run the raw permissive SHM probe for `topic` and return
    /// its verdict LABEL (the crisp per-case classification —
    /// `served`/`missing`/`loop_refused`/`not_permissive`/`already_registered`).
    /// Drives the SAME `try_serve` body both demand paths call (with its register +
    /// announce side effects on a hit), for the oracle-based verdict-taxonomy pin.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn probe_verdict_for_test(&self, topic: &str) -> &'static str {
        self.probe.try_serve(topic).label()
    }

    /// SYNC SEAM (boot-TOCTOU pin): run `try_serve` for `topic`
    /// against a FRESH probe built with an explicit STATIC `declared`-ingress set
    /// (canonicalized) over this gateway's manager — WITHOUT registering any ingress
    /// bridge, so the DYNAMIC ingress map stays empty for `topic`. Proves the boot
    /// window is closed structurally: a topic in the DECLARED set is `loop_refused`
    /// even before its ingress SHM service / dynamic-map entry exists. Pass an EMPTY
    /// `declared` for the anti-tautology control (the same live topic then classifies
    /// `served`, proving the static set is what refuses). Returns the verdict LABEL.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn probe_verdict_with_declared_ingress_for_test(
        &self,
        topic: &str,
        declared: &[String],
    ) -> &'static str {
        let declared_set: HashSet<String> = declared
            .iter()
            .map(|t| canonical_topic(t).into_owned())
            .collect();
        let probe = PermissiveShmProbe::new(Arc::downgrade(&self.manager), declared_set);
        probe.try_serve(topic).label()
    }

    /// Register a topic for network egress AT RUNTIME — the gateway-core
    /// entry point the `__cerulion/gateway_topics` control service and
    /// the permissive SHM probe both feed. The decision: a
    /// `cerulion ros2 attach` robot's dds_bridge creates its SHM publishers at
    /// runtime (raw routes like `/utlidar/cloud`), and those must be BOTH
    /// discoverable (announced) AND demandable exactly like a YAML-declared
    /// producer — discovery and demand must both tell the truth.
    ///
    /// Registers the topic's bridge flag in the manager's
    /// [`TopicBridgeManager`](super::bridge::TopicBridgeManager) (the source of
    /// truth: this is what makes the topic demand-GRANTABLE — the
    /// demand queryable and the liveliness demand watch both flip
    /// whatever registered flag a remote token names) and ANNOUNCES it on the
    /// network (a hashless liveliness token — the announce carries no schema
    /// hash; egress forwards frames verbatim and the viewer validates on ingress
    /// against its own metadata). The next [`Self::drive_once`] reconciles the
    /// topic into the tap set.
    ///
    /// Empty boot plan: a PERMISSIVE (`AllowAll`) gateway booted
    /// with an EMPTY announce plan — the generic `ros2 attach` case, ~90 runtime
    /// raw routes and zero YAML egress — now DOES start the demand
    /// queryable AND reconciler (see [`Self::new`]'s empty-boot-plan block), so a
    /// runtime topic is demand-grantable via BOTH the queryable and the liveliness
    /// watch, and the GET-expiry sweep releases it when demand ends. An empty
    /// NON-permissive (deny-all / ingress-only) gateway still starts NEITHER
    /// (it can never egress; Strict never probes).
    ///
    /// Takes `&self` (interior mutability via the shared bridge manager +
    /// idempotent network announce; the drive-view state is reconciled by
    /// `drive_once`, never touched here) so a feeder may call it from the drive
    /// thread OR a separate control thread.
    ///
    /// IDEMPOTENT: returns `Ok(true)` on a genuinely NEW registration, `Ok(false)`
    /// when the topic was already registered (no duplicate announce/tap/counter).
    ///
    /// # Errors
    ///
    /// - [`TransportError::InvalidTransportConfig`] if `topic` is in the reserved
    ///   control-plane namespace ([`RESERVED_TOPIC_PREFIX`]) — refused LOUDLY,
    ///   naming the topic + reason (a control-plane service must never be
    ///   registered as robot egress) — or if `topic` is empty.
    /// - Whatever
    ///   [`TopicBridgeManager::register_topic`](super::bridge::TopicBridgeManager::register_topic)
    ///   / [`TransportManager::announce_egress_topic`] surface (a poisoned bridge
    ///   mutex; a network-less manager — unreachable after [`Self::new`]'s guard).
    ///   On an ANNOUNCE failure specifically (zenoh session open / liveliness
    ///   declare), the demand flag is ALREADY committed and is RETAINED (not
    ///   rolled back): the fn emits ONE loud `warn!` naming the retained
    ///   registered-but-unannounced state, then propagates the Err. The topic
    ///   stays demand-grantable and a later call re-attempts the announce
    ///   (idempotent — it heals).
    pub fn register_runtime_topic(&self, topic: &str) -> TransportResult<bool> {
        if topic.is_empty() {
            return Err(TransportError::InvalidTransportConfig {
                reason: "gateway runtime topic registration requires a non-empty topic name"
                    .to_string(),
            });
        }
        let canonical = canonical_topic(topic).into_owned();
        if is_reserved_topic(&canonical) {
            return Err(TransportError::InvalidTransportConfig {
                reason: format!(
                    "gateway runtime topic registration REFUSED: '{canonical}' is in the \
                     reserved control-plane namespace '{RESERVED_TOPIC_PREFIX}' — that \
                     namespace carries Cerulion's own control services (e.g. the runtime \
                     registration channel), so registering it as robot egress would let \
                     data plumbing collide with the plumbing that carries it. Use a topic \
                     outside '{RESERVED_TOPIC_PREFIX}'"
                ),
            });
        }
        // A genuinely-new registration iff the source of truth did not already
        // hold the flag (register_topic itself is idempotent).
        let newly = !self.manager.bridge_manager().is_registered(&canonical)?;
        // Source of truth FIRST: registering the flag is what makes the topic
        // demand-grantable (the queryable + liveliness watch flip it). This
        // COMMITS before the announce, which is reachably fallible (zenoh session
        // open; liveliness declare).
        self.manager.bridge_manager().register_topic(&canonical)?;
        // Discovery truth: announce it (hashless liveliness token; idempotent).
        // On announce failure the demand flag is ALREADY committed but the topic
        // is NOT on the network — a partial commit we make LOUD, never silent.
        if let Err(e) = self.manager.announce_egress_topic(&canonical) {
            // Deliberately NO rollback: the bridge map is ADD-ONLY (register_topic
            // inserts, nothing removes), and a grantable-but-unannounced topic is
            // the CONSERVATIVE state — demand still works; only remote DISCOVERY
            // is degraded, and it heals on retry (see the warn + the idempotent
            // re-announce path). Rolling the flag back would trade a recoverable
            // discovery gap for an unrecoverable demand gap.
            tracing::warn!(
                topic = %canonical,
                error = %e,
                "gateway runtime topic registered + demand-grantable but its network \
                 announce FAILED — remote discovery will NOT list this topic (a demand token can \
                 still grant egress; the flag is RETAINED, not rolled back). Heals on retry: a \
                 later register_runtime_topic for the same topic re-attempts the announce \
                 (registration is idempotent)."
            );
            return Err(e);
        }
        if newly {
            tracing::info!(
                topic = %canonical,
                "gateway runtime egress topic registered — announced + demandable"
            );
        }
        Ok(newly)
    }

    /// Forward one drained wire frame to zenoh under `topic`. Returns `true` on
    /// success (and bumps the per-topic counter + clears the failure latch);
    /// a put failure is COUNTED-and-latched, never a panic (`&self` — the
    /// counters + latch are interior-mutable). The `network()` is `Some` by
    /// [`Self::new`]'s guarantee.
    fn forward_frame(&self, topic: &str, frame: Vec<u8>) -> bool {
        let Some(network) = self.manager.network() else {
            return false;
        };
        match network.publish_to_network(topic, frame) {
            Ok(()) => {
                if let Some(counter) = self.forwarded.get(topic) {
                    counter.fetch_add(1, Ordering::Relaxed);
                }
                self.record_forward_success(topic);
                true
            }
            Err(e) => {
                self.record_forward_failure(topic, Some(&e));
                false
            }
        }
    }

    /// First failure of a run logs `warn!`, repeats `debug!` (flood suppression,
    /// the [`super::network`] ingress-latch discipline). Covers the zenoh-put
    /// and tap-DRAIN failure classes (attach failures have their own latch —
    /// [`Self::attach_failure_count`]). `error` is `Some` for a transport
    /// failure; `None` reserved for a future poison class.
    fn record_forward_failure(&self, topic: &str, error: Option<&TransportError>) {
        let already = self
            .forward_warn
            .get(topic)
            .is_some_and(|w| w.swap(true, Ordering::Relaxed));
        match (already, error) {
            (false, Some(e)) => tracing::warn!(
                topic = %topic,
                error = %e,
                "gateway egress forward failed — dropped this frame (repeats log at debug)"
            ),
            (true, Some(e)) => tracing::debug!(
                topic = %topic,
                error = %e,
                "gateway egress forward failed (repeat — see the first warning)"
            ),
            (_, None) => tracing::warn!(topic = %topic, "gateway egress forward failed"),
        }
    }

    /// A successful forward ends any failure run: clear the latch and report the
    /// recovery once (the `DrainWarnLatch::on_success` shape).
    fn record_forward_success(&self, topic: &str) {
        if let Some(w) = self.forward_warn.get(topic) {
            if w.swap(false, Ordering::Relaxed) {
                tracing::info!(topic = %topic, "gateway egress recovered — forwarding again");
            }
        }
    }

    /// Record a tap ATTACH failure, counted always
    /// (Principle #3), and LOUD once per regime: the first failure `warn!`s
    /// naming the topic, the probable cause, and the remedy; repeats downgrade
    /// to `debug!` (the attach is retried every drive pass, so an
    /// unlatched warn would flood). Re-armed by [`Self::record_attach_success`].
    fn record_attach_failure(&self, topic: &str, error: &TransportError) {
        if let Some(c) = self.attach_failures.get(topic) {
            c.fetch_add(1, Ordering::Relaxed);
        }
        let already = self
            .attach_warn
            .get(topic)
            .is_some_and(|w| w.swap(true, Ordering::Relaxed));
        if already {
            tracing::debug!(
                topic = %topic,
                error = %error,
                "gateway egress tap attach still failing (repeat — see the first warning; \
                 retried every drive pass)"
            );
        } else {
            tracing::warn!(
                topic = %topic,
                error = %error,
                "gateway egress tap attach FAILED — this topic will NOT egress until the tap \
                 attaches. Probable cause: the topic's subscriber slots are exhausted (the \
                 gateway tap shares the introspection headroom with bagd / `cerulion topic \
                 echo`/`hz` consumers) or the producer's service is not up yet. Remedy: stop a \
                 concurrent introspection tap, or raise the topic's subscriber provisioning via \
                 the graph's buffer config. Retried every drive pass (logged once per failure \
                 regime; attach_failure_count keeps counting)"
            );
        }
    }

    /// A successful attach ends an attach-failure regime: re-arm the latch and
    /// report the recovery once.
    fn record_attach_success(&self, topic: &str) {
        if let Some(w) = self.attach_warn.get(topic) {
            if w.swap(false, Ordering::Relaxed) {
                tracing::info!(
                    topic = %topic,
                    "gateway egress tap attached after earlier failures — topic egressing again"
                );
            }
        }
    }

    /// This topic's OBSERVED data-flow liveness (Principle #3): exactly
    /// the value the `catalog` verb serves for it. `None` = the robot is not
    /// observing it right now (untracked, tap never attached, tap lost, or
    /// observation disabled), which is UNKNOWN and must never be rendered as
    /// "dead".
    pub fn topic_liveness(&self, topic: &str) -> Option<TopicLiveness> {
        self.liveness.liveness(topic)
    }

    /// Observable state (Principle #3): liveness taps the observer currently OWNS. Excludes
    /// topics observed through the egress tap, so this is the count of EXTRA
    /// subscriber ports the feature costs right now.
    pub fn liveness_tap_count(&self) -> usize {
        self.liveness.active_tap_count()
    }

    /// Whether data-flow liveness observation is enabled at all
    /// (`CERULION_TOPIC_LIVENESS=off` disables it).
    pub fn liveness_enabled(&self) -> bool {
        self.liveness.is_enabled()
    }

    /// Test seam: run the next [`Self::drive_once`] observer sweep
    /// regardless of the self-throttle.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn force_liveness_sweep_for_test(&mut self) {
        self.liveness.force_next_sweep_for_test();
    }

    /// Test seam: make `topic`'s EGRESS tap fail its next drain. The
    /// production stimulus is a producer service torn down and re-created (a
    /// worker crash+respawn under `--peer-loss continue`), which a stale tap can
    /// never see and which is not deterministically stageable. Returns whether an
    /// egress tap was there to poison.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn fail_next_egress_drain_for_test(&mut self, topic: &str) -> bool {
        match self.taps.get_mut(topic) {
            Some(tap) => {
                tap.fault_inject_receive_after_for_test(0);
                true
            }
            None => false,
        }
    }

    /// Observable state (Principle #3): the canonical topics with an ACTIVE egress tap
    /// right now, SORTED for deterministic assertions.
    pub fn active_tap_topics(&self) -> Vec<String> {
        let mut topics: Vec<String> = self.taps.keys().cloned().collect();
        topics.sort();
        topics
    }

    /// Observable state (Principle #3): cumulative frames SUCCESSFULLY forwarded for
    /// `topic` (canonical-keyed; `0` for an unannounced topic).
    pub fn forwarded_count(&self, topic: &str) -> u64 {
        let canonical = canonical_topic(topic).into_owned();
        self.forwarded
            .get(&canonical)
            .map(|c| c.load(Ordering::Relaxed))
            .unwrap_or(0)
    }

    /// Observable state (Principle #3): cumulative tap ATTACH failures for `topic`
    /// (canonical-keyed; `0` for an unannounced topic). A demanded topic whose
    /// count keeps growing while [`Self::active_tap_topics`] omits it is the
    /// slot-exhaustion signature (see the attach-failure warn for the remedy).
    pub fn attach_failure_count(&self, topic: &str) -> u64 {
        let canonical = canonical_topic(topic).into_owned();
        self.attach_failures
            .get(&canonical)
            .map(|c| c.load(Ordering::Relaxed))
            .unwrap_or(0)
    }

    /// The DRIVE VIEW's egress topic count (Principle #3):
    /// boot-plan topics plus any runtime-registered ones the last
    /// [`Self::drive_once`] reconciled in. This is the reconciled DRIVE view, NOT
    /// the announce/registration source of truth: a [`Self::register_runtime_topic`]
    /// (which announces IMMEDIATELY) is NOT reflected here until the next
    /// `drive_once` reconciles it in, so this UNDER-reports a fresh runtime
    /// registration until then.
    ///
    /// For the FRESH source-of-truth count (updated the instant
    /// `register_runtime_topic` returns, before any drive pass) use
    /// [`Self::registered_topic_count`]; for the ANNOUNCE truth (the tokens
    /// actually declared on the network, updated immediately) use
    /// [`NetworkManager::announced_topic_count`](super::network::NetworkManager::announced_topic_count).
    pub fn egress_view_count(&self) -> usize {
        self.egress_flags.len()
    }

    /// Observable state (Principle #3): the FRESH total of registered egress topics
    /// (boot-plan + runtime), read straight from the bridge manager (the source
    /// of truth) — reflects a [`Self::register_runtime_topic`] immediately, with
    /// no drive pass required.
    pub fn registered_topic_count(&self) -> TransportResult<usize> {
        self.manager.bridge_manager().registered_topic_count()
    }

    /// Observable state (Principle #3): the topics registered AT RUNTIME
    /// ([`Self::register_runtime_topic`]) — the current registered set MINUS the
    /// boot-plan announce set — SORTED. Fresh (reads the source of truth), so a
    /// test or operator can observe a runtime registration WITHOUT driving the
    /// gateway.
    pub fn runtime_registered_topics(&self) -> TransportResult<Vec<String>> {
        let registered = self.manager.bridge_manager().registered_topics()?;
        let mut out: Vec<String> = registered
            .into_iter()
            .map(|(topic, _)| topic)
            .filter(|topic| !self.boot_topics.contains(topic))
            .collect();
        out.sort();
        Ok(out)
    }

    /// Observable state (Principle #3): the number of runtime-registered egress topics
    /// (see [`Self::runtime_registered_topics`]).
    pub fn runtime_registered_count(&self) -> TransportResult<usize> {
        Ok(self.registered_topic_count()? - self.boot_topics.len())
    }

    /// Observable state (Principle #3): whether `topic` was registered AT RUNTIME
    /// (registered in the source of truth AND not a boot-plan announce topic).
    /// Canonical-keyed. `false` for a boot-plan topic or an unregistered one.
    pub fn is_runtime_registered(&self, topic: &str) -> TransportResult<bool> {
        let canonical = canonical_topic(topic).into_owned();
        if self.boot_topics.contains(&canonical) {
            return Ok(false);
        }
        self.manager.bridge_manager().is_registered(&canonical)
    }

    /// Observable state (Principle #3): number of ingress bridges this gateway opened.
    pub fn ingress_count(&self) -> usize {
        self.ingress_count
    }

    /// Cumulative demand-reconcile passes applied (Principle #3): the
    /// sum across the background reconciler thread and any sync
    /// `reconcile_demand_once` calls (they share the counter).
    pub fn reconcile_pass_count(&self) -> u64 {
        self.reconcile_pass_count.load(Ordering::Relaxed)
    }

    /// Observable state (Principle #3): cumulative reconciler-CAUSED egress-flag
    /// transitions (false→true) for demanded announced topics. `> 0` attributes
    /// an egress flag flip to the RECONCILER rather than the liveliness
    /// subscriber. It counts only GENUINE transitions the reconciler caused,
    /// so it stays 0 on a healthy link where the subscriber flipped the
    /// flag first (a reconciler re-affirm returns "already on" and is not
    /// counted) and on a gate-refused non-member.
    pub fn reconciler_enabled_count(&self) -> u64 {
        self.reconciler_enabled_count.load(Ordering::Relaxed)
    }

    // ---- Queryable inversion — Principle-#3 counters ----

    /// Observable state (Principle #3): cumulative ALLOWED demand-GET grants the PRODUCER
    /// queryable applied (enable + keepalive stamp). `> 0` attributes egress to a
    /// remote demand GET (the inversion path) rather than the liveliness paths.
    pub fn demand_grant_count(&self) -> u64 {
        self.get_demand_state.grant_count()
    }

    /// Observable state (Principle #3): cumulative demand-GET REFUSALS the producer
    /// queryable returned (allow-list miss / unregistered / suppressed).
    pub fn demand_refusal_count(&self) -> u64 {
        self.get_demand_state.refusal_count()
    }

    /// Observable state (Principle #3): cumulative GET-expiry disables the reconciler
    /// thread applied (a topic released because its last GET aged past the TTL
    /// AND liveliness was also absent — the composition rule).
    pub fn demand_expiry_count(&self) -> u64 {
        self.get_demand_state.expiry_count()
    }

    /// Observable state (Principle #3): the canonical topics the PRODUCER queryable is
    /// currently holding egress for via a fresh-enough GET grant (a keepalive
    /// stamp is present), SORTED. Empty ⇒ no topic is GET-granted — e.g. under
    /// the `suppress_live_demand` seam a demand GET is refused WITHOUT a stamp.
    pub fn granted_topics(&self) -> Vec<String> {
        self.get_demand_state.granted_topics()
    }

    /// Observable state (Principle #3): cumulative demand-GET passes the DEMANDER loop ran
    /// (`0` on a pure-egress gateway with no ingress).
    pub fn demand_get_pass_count(&self) -> u64 {
        self.demand_get_state
            .as_ref()
            .map(|s| s.pass_count())
            .unwrap_or(0)
    }

    /// Cumulative "enabled" acks the DEMANDER received (Principle #3):
    /// a producer accepted its demand and egress is (or already was) ON.
    pub fn demand_enabled_ack_count(&self) -> u64 {
        self.demand_get_state
            .as_ref()
            .map(|s| s.enabled_ack_count())
            .unwrap_or(0)
    }

    /// Cumulative "refused" acks the DEMANDER received (Principle #3):
    /// a producer refused its demand (topic not in the egress allow-list).
    pub fn demand_refused_ack_count(&self) -> u64 {
        self.demand_get_state
            .as_ref()
            .map(|s| s.refused_ack_count())
            .unwrap_or(0)
    }

    /// Observable state (Principle #3): number of producer identities the DEMANDER
    /// harvested from the announce space (`0` ⇒ its GETs still use the wildcard).
    pub fn harvested_identity_count(&self) -> usize {
        self.demand_get_state
            .as_ref()
            .map(|s| s.harvested_identity_count())
            .unwrap_or(0)
    }

    /// Observable state (Principle #3): the harvested producer identities, SORTED (empty
    /// on a pure-egress gateway or before the first harvest lands).
    pub fn harvested_identities(&self) -> Vec<String> {
        self.demand_get_state
            .as_ref()
            .map(|s| s.harvested_identities())
            .unwrap_or_default()
    }

    /// SYNC SEAM (DEMANDER): run exactly ONE demand-GET pass inline (no
    /// thread) — the deterministic driver for the inversion e2e. Harvests
    /// identities + GETs every ingress topic once. No-op `Ok(())` on a
    /// pure-egress gateway (no demand-get state).
    ///
    /// # Errors
    ///
    /// [`TransportError::InvalidTransportConfig`] if the manager has no network;
    /// [`TransportError::SessionCreation`] if the session cannot open.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn demand_get_once(&mut self) -> TransportResult<()> {
        let Some(state) = self.demand_get_state.as_ref() else {
            return Ok(());
        };
        let network =
            self.manager
                .network()
                .ok_or_else(|| TransportError::InvalidTransportConfig {
                    reason: "demand_get_once requires a configured network transport".to_string(),
                })?;
        network.demand_get_pass(state)
    }

    /// SYNC SEAM (PRODUCER): run exactly ONE GET-expiry sweep inline at
    /// `now` — the deterministic driver for the expiry e2e. Disables GET-granted
    /// topics whose last GET aged past `DEMAND_TTL` AND which liveliness is not
    /// holding (the composition rule).
    ///
    /// # Errors
    ///
    /// [`TransportError::InvalidTransportConfig`] if the manager has no network;
    /// [`TransportError::SessionCreation`] if the session cannot open.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn run_get_expiry_once(&mut self, now: std::time::Instant) -> TransportResult<()> {
        let network =
            self.manager
                .network()
                .ok_or_else(|| TransportError::InvalidTransportConfig {
                    reason: "run_get_expiry_once requires a configured network transport"
                        .to_string(),
                })?;
        network.get_expiry_pass(self.manager.bridge_manager(), &self.get_demand_state, now)
    }

    /// SYNC SEAM: run exactly ONE demand-reconcile pass inline (no
    /// thread) against this gateway's own absence tracking + shared counters —
    /// the deterministic driver for the reconciler tests. Delegates to the SAME
    /// gather+apply body [`NetworkManager::start_demand_reconciler`]'s thread runs.
    ///
    /// # Errors
    ///
    /// [`TransportError::InvalidTransportConfig`] if the manager has no network
    /// (unreachable after [`Self::new`]'s guard); [`TransportError::SessionCreation`]
    /// if the session cannot open.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn reconcile_demand_once(&mut self) -> TransportResult<()> {
        let network =
            self.manager
                .network()
                .ok_or_else(|| TransportError::InvalidTransportConfig {
                    reason: "reconcile_demand_once requires a configured network transport"
                        .to_string(),
                })?;
        // `reconcile_demand_pass` refreshes `reconcile_topics`
        // from the live registered set FIRST (the belt feed), so this seam
        // re-affirms a runtime-registered topic — the sync analogue of the
        // production thread growing its iteration set.
        network.reconcile_demand_pass(
            self.manager.bridge_manager(),
            &mut self.reconcile_topics,
            &mut self.reconcile_absence,
            &self.reconcile_pass_count,
            &self.reconciler_enabled_count,
        )
    }

    /// Test seam: arm a fire-once drain receive fault on the ACTIVE tap for
    /// `topic` (delegates to the tap's `fault_inject_receive_after_for_test`).
    /// Returns `false` when no tap is currently attached — the caller must
    /// have driven the gateway to attach first.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn fault_inject_tap_drain_for_test(&mut self, topic: &str, after: u32) -> bool {
        let canonical = canonical_topic(topic).into_owned();
        match self.taps.get_mut(&canonical) {
            Some(tap) => {
                tap.fault_inject_receive_after_for_test(after);
                true
            }
            None => false,
        }
    }
}

impl fmt::Debug for GatewayRuntime {
    /// Concise operational summary — the observable surface (announce/ingress
    /// counts + the live tap set), never the manager or tap handles (neither is
    /// `Debug`, and dumping them would be noise, not signal).
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("GatewayRuntime")
            .field("egress_view_count", &self.egress_flags.len())
            .field("ingress_count", &self.ingress_count)
            .field("active_taps", &self.active_tap_topics())
            .finish_non_exhaustive()
    }
}
// Pure GatewayPlan tests (validate + serde round-trip) live in
// `tests/gateway_iox2_test.rs` alongside the transport e2e — keeping this file
// free of a `#[cfg(test)] mod tests` avoids annotating test-only allocations for
// the hot-path alloc lint (the whole file is the network-forwarder cold path).
