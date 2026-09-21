// SPDX-License-Identifier: AGPL-3.0-only
//! Process-wide runtime state for rmw_cerulion.
//!
//! One Cerulion `TransportManager` per process (Principle #8: ONE
//! iceoryx2 node), plus deterministic registries backing the rmw graph
//! API (`rmw_count_publishers`, `rmw_get_topic_names_and_types`, …).
//!
//! Registries are `BTreeMap`s — graph queries enumerate in stable order
//! (Principle #7: introspection output is reproducible).

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock, RwLock};

use cerulion_core::transport::{TransportConfig, TransportManager};

/// A compact snapshot of an endpoint's QoS: what the caller REQUESTED,
/// plus the queue depth Cerulion actually PROVISIONED for it.
///
/// The record carries BOTH depths, under names a call site cannot mix up
/// by accident, because they are not the same number and reporting the
/// wrong one is the defect this exists to close.
/// A `TRANSIENT_LOCAL` publisher's depth is clamped
/// and its subscriber queue ceiling raised; an rmw SUBSCRIPTION's
/// requested depth is not applied to its queue at all. Echoing the
/// REQUEST back through `rmw_get_{publishers,subscriptions}_info_by_topic`
/// would report a number no transport ever provisioned, so that surface
/// serves [`Self::provisioned_depth`] and a create whose request differs
/// from what it got says so LOUDLY at create time, naming both numbers.
///
/// The other three axes are the values the caller REQUESTED —
/// the six `*_get_actual_qos` getters (and with them the
/// actual durability / reliability / history story) do not serve provisioned
/// values; this record carries a provisioned value for the depth axis only.
///
/// Plain `u32`/`usize` (not the ffi `rmw_qos_*_policy_t` aliases) keep
/// this module decoupled from the C bindings; the aliases are `c_uint`,
/// so the values map through without a cast at the fill site.
#[derive(Clone)]
pub struct QosSnapshot {
    /// `rmw_qos_reliability_policy_t` value, as REQUESTED.
    pub reliability: u32,
    /// `rmw_qos_durability_policy_t` value, as REQUESTED.
    pub durability: u32,
    /// `rmw_qos_history_policy_t` value, as REQUESTED.
    pub history: u32,
    /// The queue depth the caller ASKED for.
    ///
    /// NEVER served to a ROS query — it is kept so the create-time warn
    /// can name both numbers.
    pub requested_depth: usize,
    /// The queue depth Cerulion actually PROVISIONED for this endpoint —
    /// its RETENTION, the frames it really keeps:
    ///
    ///
    /// - PUBLISHER: the clamped history. `TRANSIENT_LOCAL` depth 1000 ⇒ 16,
    ///   depth 5 ⇒ 5, and VOLATILE ⇒ 0 (it retains nothing for a late
    ///   joiner). NOT the topic's `subscriber_max_buffer_size` ceiling,
    ///   which is a subscriber-SLOT budget rather than a depth anything
    ///   retains — reporting it would make a depth-5 publisher claim 16.
    /// - SUBSCRIPTION: the transport's default subscriber buffer, which IS
    ///   that endpoint's own queue depth (an rmw subscription's requested
    ///   depth is not applied).
    ///
    /// This is what `rmw_get_{publishers,subscriptions}_info_by_topic`
    /// reports. `*_get_actual_qos` does not report it: that getter
    /// answers a constant, not the `depth = history` rule.
    pub provisioned_depth: usize,
}

/// One publisher or subscription endpoint on a topic, backing
/// `rmw_get_{publishers,subscriptions}_info_by_topic`.
///
/// Process-LOCAL: only endpoints created in THIS process are recorded —
/// cross-process endpoint discovery is not implemented
/// (same limitation as the `nodes` map).
#[derive(Clone)]
pub struct EndpointRecord {
    /// Owning node's name.
    pub node_name: String,
    /// Owning node's namespace.
    pub node_namespace: String,
    /// The topic's ROS graph type name ("pkg/msg/Name").
    pub type_name: String,
    /// Deterministic entity gid (== `rmw_get_gid_for_publisher`).
    pub endpoint_gid: [u8; 16],
    /// QoS the endpoint was created with.
    pub qos: QosSnapshot,
}

/// Entity registries for graph introspection. Keyed by Cerulion topic
/// name; values carry the ROS-visible metadata.
#[derive(Default)]
pub struct GraphRegistry {
    /// topic → publisher endpoint records. The `Vec` is the SINGLE
    /// source of truth for this process's publishers on a topic: the
    /// publisher count is `Vec::len` and the type name is the records'
    /// `type_name` — there is no parallel counter that could drift
    /// out of step.
    pub publishers: BTreeMap<String, Vec<EndpointRecord>>,
    /// topic → subscription endpoint records (same single-source-of-truth
    /// contract as `publishers`).
    pub subscriptions: BTreeMap<String, Vec<EndpointRecord>>,
    /// service → (type name "pkg/srv/Name", server count)
    pub services: BTreeMap<String, (String, usize)>,
    /// service → (type name, client count)
    pub clients: BTreeMap<String, (String, usize)>,
    /// (namespace, node name) → () for this process's nodes —
    /// keyed by the FULLY-QUALIFIED pair so same-named nodes in
    /// different namespaces don't collide. Cross-process
    /// node discovery is not implemented.
    pub nodes: BTreeMap<(String, String), ()>,
}

impl GraphRegistry {
    /// (type, count) add — services and clients (no per-endpoint info
    /// query in scope for them).
    pub fn add(map: &mut BTreeMap<String, (String, usize)>, key: &str, type_name: &str) {
        let entry = map
            .entry(key.to_string())
            .or_insert_with(|| (type_name.to_string(), 0));
        entry.1 += 1;
    }

    /// (type, count) remove — services and clients.
    pub fn remove(map: &mut BTreeMap<String, (String, usize)>, key: &str) {
        if let Some(entry) = map.get_mut(key) {
            entry.1 = entry.1.saturating_sub(1);
            if entry.1 == 0 {
                map.remove(key);
            }
        }
    }

    /// Register a publisher/subscription endpoint on a topic. The `Vec`
    /// length is the count, so there is no separate counter to keep in
    /// step.
    pub fn add_endpoint(
        map: &mut BTreeMap<String, Vec<EndpointRecord>>,
        topic: &str,
        record: EndpointRecord,
    ) {
        map.entry(topic.to_string()).or_default().push(record);
    }

    /// Deregister the endpoint with `gid`; drop the topic key once the
    /// last endpoint leaves (keeps empty topics absent, matching the
    /// tuple `remove`). Removing by gid is exact even with multiple
    /// same-topic endpoints in this process.
    pub fn remove_endpoint(
        map: &mut BTreeMap<String, Vec<EndpointRecord>>,
        topic: &str,
        gid: &[u8; 16],
    ) {
        if let Some(records) = map.get_mut(topic) {
            if let Some(pos) = records.iter().position(|r| &r.endpoint_gid == gid) {
                // swap_remove: order among a topic's endpoints is not
                // observable (rmw returns an unordered set), and this
                // avoids an O(n) shift.
                records.swap_remove(pos);
            }
            if records.is_empty() {
                map.remove(topic);
            }
        }
    }
}

/// Global runtime singleton.
pub struct Runtime {
    pub transport: Arc<TransportManager>,
    pub graph: RwLock<GraphRegistry>,
    /// Monotonic counter for deterministic per-process entity ids (gids,
    /// client instance numbers). Never wall-clock, never random.
    pub entity_counter: AtomicU64,
    /// Every live node's graph guard condition state. Registry
    /// mutations trigger ALL of them so executors blocked in `rmw_wait`
    /// re-run their graph queries (without this,
    /// `wait_for_service`-style loops only progress via timeout).
    pub graph_guards: Mutex<Vec<GraphGuardPtr>>,
    /// Every live publisher (raw PublisherData pointers, same lifecycle
    /// discipline as `graph_guards`). `rmw_wait` pumps their iceoryx2
    /// events so TRANSIENT_LOCAL history reaches late joiners even when
    /// the publisher never publishes again (without the pump, ros2_control
    /// waits forever for the latched `robot_description`).
    pub publishers: Mutex<Vec<PublisherPtr>>,
    /// Throttle for the event pump (ms since runtime start of the last
    /// pump; one pump per PUMP_INTERVAL_MS across all waiting threads).
    pub last_pump_ms: AtomicU64,
    /// Monotonic base for `last_pump_ms`.
    pub started: std::time::Instant,
}

/// Raw pointer to a live `PublisherData`.
///
/// # Soundness
/// The pointee is owned by the publisher `Box` created in
/// `rmw_create_publisher` and lives until `rmw_destroy_publisher`, which
/// UNREGISTERS it from `Runtime::publishers` (the "UNREGISTER FIRST" step
/// there) before `Box::from_raw` frees it. The only access through this
/// pointer is `pump_publisher_events`, which derefs it while holding the
/// `publishers` registry lock and then takes the per-publisher
/// `inner.try_lock()` — never blocking, never forming a `&mut` alias.
/// The private field + `unsafe fn new` keep that invariant constructor-
/// gated rather than mintable from an arbitrary pointer.
pub struct PublisherPtr(*const PublisherData);
unsafe impl Send for PublisherPtr {}

impl PublisherPtr {
    /// # Safety
    /// `ptr` must point at a `PublisherData` that stays live for as long
    /// as this `PublisherPtr` remains registered in `Runtime::publishers`
    /// (unregister-before-free is enforced in `rmw_destroy_publisher`).
    pub unsafe fn new(ptr: *const PublisherData) -> Self {
        Self(ptr)
    }

    /// The registered pointer. Sound to deref only under the discipline
    /// documented on the type (registry lock + `inner.try_lock()`); the
    /// unregister path uses it for identity comparison only.
    pub(crate) fn as_ptr(&self) -> *const PublisherData {
        self.0
    }
}

/// Event-pump throttle. Also the cap on any single `rmw_wait` kernel
/// block: a parked waiter must keep pumping at this cadence or
/// TRANSIENT_LOCAL history stalls behind an indefinite block.
pub(crate) const PUMP_INTERVAL_MS: u64 = 20;

/// Process the iceoryx2 events (SubscriberJoined → history delivery)
/// of every live publisher in THIS process. Called from `rmw_wait`'s
/// poll loop; throttled; `try_lock` so an application thread holding a
/// publisher lock is never blocked (it will pump on its next publish
/// anyway).
pub fn pump_publisher_events() -> bool {
    let Some(rt) = RUNTIME.get() else {
        return false;
    };
    let now_ms = rt.started.elapsed().as_millis() as u64;
    let last = rt.last_pump_ms.load(Ordering::Relaxed);
    if now_ms.saturating_sub(last) < PUMP_INTERVAL_MS {
        return false;
    }
    if rt
        .last_pump_ms
        .compare_exchange(last, now_ms, Ordering::Relaxed, Ordering::Relaxed)
        .is_err()
    {
        return false; // another thread is pumping this interval
    }
    let publishers = rt.publishers.lock().unwrap_or_else(|e| e.into_inner());
    for p in publishers.iter() {
        // SAFETY: pointers are live while registered (PublisherPtr).
        let data = unsafe { &*p.as_ptr() };
        if let Ok(mut inner) = data.inner.try_lock() {
            inner.publisher.check_subscriber_events();
        }
    }
    true
}

/// Raw pointer to a node's graph `GuardConditionState`.
///
/// # Soundness
/// The pointee is owned by the guard-condition Box created in
/// `rmw_create_node` and lives until `rmw_destroy_node`, which
/// UNREGISTERS it from `graph_guards` before destroying it. The only
/// access through this pointer is the atomic `triggered` flag. The
/// private field + `unsafe fn new` keep that invariant constructor-gated
/// (matching the sibling [`PublisherPtr`]).
pub struct GraphGuardPtr(*const GuardConditionState);
unsafe impl Send for GraphGuardPtr {}

impl GraphGuardPtr {
    /// # Safety
    /// `ptr` must point at a `GuardConditionState` that stays live for as
    /// long as this `GraphGuardPtr` remains registered in
    /// `Runtime::graph_guards` (unregister-before-free is enforced in
    /// `rmw_destroy_node`).
    pub unsafe fn new(ptr: *const GuardConditionState) -> Self {
        Self(ptr)
    }

    /// The registered pointer. Sound to deref only under the discipline
    /// documented on the type (registry lock; reads the atomic flag); the
    /// unregister path uses it for identity comparison only.
    pub(crate) fn as_ptr(&self) -> *const GuardConditionState {
        self.0
    }
}

/// Lock an entity mutex guarding `!Send` iceoryx2 state. Returns
/// `None` if the mutex is POISONED: a panic mid-operation may have torn
/// the Rc-backed slot-map state, and re-entering it risks SHM
/// bookkeeping use-after-free — strictly worse than failing the
/// call. The entity stays wedged (every later call errors)
/// until destroyed. Plain-DATA locks (registries, guard lists) keep
/// `unwrap_or_else(|e| e.into_inner())` — a torn BTreeMap is still a
/// valid BTreeMap.
pub fn lock_unpoisoned<T>(m: &Mutex<T>) -> Option<std::sync::MutexGuard<'_, T>> {
    m.lock().ok()
}

/// Trigger every registered graph guard (call after any registry
/// mutation, AFTER releasing the registry write lock).
pub fn notify_graph_change() {
    if let Some(rt) = RUNTIME.get() {
        let guards = rt.graph_guards.lock().unwrap_or_else(|e| e.into_inner());
        for g in guards.iter() {
            // SAFETY: registered pointers are live (see GraphGuardPtr).
            // `trigger` = flag + doorbell ring: without the ring
            // an executor parked in an event-driven `rmw_wait` would only
            // notice the graph change at its 20ms pump cap.
            unsafe { (*g.as_ptr()).trigger() };
        }
    }
}

static RUNTIME: OnceLock<Runtime> = OnceLock::new();

/// Install the crate's stderr tracing subscriber (idempotent).
///
/// Host processes (move_group, rviz, ros2cli) have no Rust tracing
/// subscriber — without one, every diagnostic in this crate is
/// silently dropped. try_init: a host that DID install one wins.
/// Split out of [`runtime`] so the load-time era guard in
/// the init-options exports and `rmw_init` — and `ffi_guard`'s panic
/// arm — can be LOUD before the runtime exists: `runtime()`, the usual
/// installer, is never reached on those paths.
///
/// A NO-OP in the crate's own `--lib` test build: a
/// successful `try_init` here would take the process-global subscriber slot,
/// after which `tracing_test`'s installer hits its own `.expect(..)` and
/// PANICS for every later `#[traced_test]` in the binary. That would make the
/// lib suite pass by ALPHABETICAL ACCIDENT — whichever traced test sorted
/// first happened to claim the slot before any arm drove an entry point.
/// Integration tests link the NON-test rlib and are unaffected: they get
/// the real installer, which is the surface a host actually sees.
#[cfg(test)]
pub(crate) fn install_tracing() {}

/// See the `cfg(test)` twin above for why the lib test build no-ops.
#[cfg(not(test))]
pub(crate) fn install_tracing() {
    let _ = tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                // cerulion_core must be in the default filter: the
                // root-cause diagnostics (deliver_history failures,
                // malformed-frame drops, publish errors) are emitted on
                // cerulion_core targets — a filter naming only
                // rmw_cerulion turns every other target OFF and
                // silently drops them.
                .unwrap_or_else(|_| {
                    tracing_subscriber::EnvFilter::new("rmw_cerulion=warn,cerulion_core=warn")
                }),
        )
        // ANSI colour only on a terminal: a host's
        // stderr is usually a pipe, a log file or journald, and the
        // default subscriber colours regardless — the era refusal would then
        // reach the operator wrapped in escape codes, and a whole-token
        // reader could not even find its level.
        .with_ansi(std::io::IsTerminal::is_terminal(&std::io::stderr()))
        .with_writer(std::io::stderr)
        .try_init();
}

/// Initialize (or fetch) the process runtime. Called from `rmw_init`
/// and every entry point that needs transport access.
pub fn runtime() -> Result<&'static Runtime, String> {
    if let Some(rt) = RUNTIME.get() {
        return Ok(rt);
    }
    install_tracing();
    let transport = TransportManager::init(TransportConfig {
        node_name: format!("rmw_cerulion_{}", std::process::id()),
        ..Default::default()
    })
    .or_else(|_| TransportManager::get())
    .map_err(|e| format!("transport init failed: {e}"))?;

    let rt = Runtime {
        transport,
        graph: RwLock::new(GraphRegistry::default()),
        entity_counter: AtomicU64::new(1),
        graph_guards: Mutex::new(Vec::new()),
        publishers: Mutex::new(Vec::new()),
        last_pump_ms: AtomicU64::new(0),
        started: std::time::Instant::now(),
    };
    // Two threads racing rmw_init: first one wins; both get the same
    // singleton (TransportManager::init is itself OnceLock-backed).
    let _ = RUNTIME.set(rt);
    RUNTIME
        .get()
        .ok_or_else(|| "runtime registration failed".to_string())
}

/// Next deterministic entity id.
pub fn next_entity_id() -> u64 {
    match RUNTIME.get() {
        Some(rt) => rt.entity_counter.fetch_add(1, Ordering::Relaxed),
        None => 0,
    }
}

/// Why a ROS name was refused as a Cerulion topic / service name by
/// [`ros_topic_to_cerulion`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RosNameError {
    /// The name is empty.
    Empty,
    /// The name has no leading `/`. Every name rcl hands an rmw is FULLY
    /// QUALIFIED (rcl expands relative names + validates them before the rmw
    /// call), so this only ever means a caller bypassed rcl — never a name
    /// to repair.
    NoLeadingSlash,
    /// The name has an EMPTY segment — a lone `/`, a trailing `/`, or a `//`
    /// inside. rcl's full-name grammar forbids all three, and the transport
    /// does NOT catch them (its topic validation refuses only an empty or an
    /// over-long name), so without this check `/` would proceed to a
    /// malformed `//data` iceoryx2 service instead of the documented refusal.
    EmptySegment,
}

impl std::fmt::Display for RosNameError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RosNameError::Empty => write!(f, "ROS name is empty"),
            RosNameError::NoLeadingSlash => write!(
                f,
                "ROS name has no leading '/' (not fully qualified); the rmw uses the ROS \
                 name verbatim as the Cerulion name and never prefixes one"
            ),
            RosNameError::EmptySegment => write!(
                f,
                "ROS name has an empty segment (a lone '/', a trailing '/', or '//'); a \
                 fully-qualified name is '/seg' or '/seg/…/seg' with every segment \
                 non-empty — the rmw never repairs a name, only refuses it"
            ),
        }
    }
}

impl std::error::Error for RosNameError {}

/// Map a ROS topic / service name to its Cerulion name: the IDENTITY.
///
/// A fully-qualified ROS name (`/chatter`, `/ns/cmd_vel`) IS the Cerulion
/// canonical name — canonical names carry the leading slash and the slash is
/// part of the iceoryx2 service name (`/chatter/data`). So an rmw publisher on
/// ROS `/chatter` is what `cerulion topic echo /chatter` reads, what a
/// network gateway's canonical-name probe / egress tap opens, and what a
/// desk-side mirror re-injects — one data source, one name, on both sides.
///
/// A spelling that strips the leading `/` (`chatter/data`) does not
/// interoperate with the network plane's `/topic` canonical names: every gateway
/// path canonicalizes to `/chatter` and opens `/chatter/data`, a DISTINCT
/// iceoryx2 service from `chatter/data`, so such rmw topics would never reach a
/// networked host in either direction. There is deliberately NO alias and NO
/// fallback to the stripped spelling — one name.
///
/// Validation is loud, never repairing, and enforces the fully-qualified
/// grammar the transport does NOT: a leading `/` and NO empty segment (so
/// `/`, `//`, `/a/` and `/a//b` are refused). None of these can arrive
/// through rcl (it expands + validates names first), so one reaching this
/// seam is a direct C-ABI caller's bug and is REFUSED rather than silently
/// prefixed or trimmed. The transport's own topic validation refuses only an
/// empty or an over-long name, so a lone `/` let through here would become a
/// malformed `//data` iceoryx2 service — the check lives here on purpose.
/// Every refusing call site logs the offending name under `topic=` /
/// `service=`.
///
/// # Errors
///
/// [`RosNameError::Empty`] for an empty name; [`RosNameError::NoLeadingSlash`]
/// for a name not starting with `/`; [`RosNameError::EmptySegment`] for a
/// lone `/`, a trailing `/`, or a `//` inside.
pub fn ros_topic_to_cerulion(ros_name: &str) -> Result<String, RosNameError> {
    if ros_name.is_empty() {
        return Err(RosNameError::Empty);
    }
    let Some(segments) = ros_name.strip_prefix('/') else {
        return Err(RosNameError::NoLeadingSlash);
    };
    // `split` over the rest yields "" for a lone "/", for a trailing "/",
    // and for each "//" — exactly the three empty-segment shapes.
    if segments.split('/').any(str::is_empty) {
        return Err(RosNameError::EmptySegment);
    }
    Ok(ros_name.to_string())
}

/// Guard condition state: a flag the waitset probes, set by
/// `rmw_trigger_guard_condition` from any thread — plus
/// an fd doorbell so a trigger can WAKE a waiter parked in the
/// event-driven `rmw_wait` (Linux: eventfd; other unix: pipe).
///
/// The FLAG is the truth; the doorbell is only WHEN to look. `trigger`
/// stores the flag BEFORE ringing, so a waiter that drains the ring and
/// then probes the flag never loses a trigger (drain-then-probe is
/// race-free by that ordering), and a doorbell-less guard (creation
/// failure — warned loudly, never fatal) still works: a parked waiter
/// notices the flag at its ≤`PUMP_INTERVAL_MS` (20ms) block cap instead
/// of instantly.
pub struct GuardConditionState {
    pub triggered: AtomicBool,
    /// `None` only when doorbell creation failed at guard creation
    /// (fd exhaustion) — the degrade documented above.
    pub doorbell: Option<cerulion_core::wake::Doorbell>,
}

impl GuardConditionState {
    pub fn new() -> Self {
        let doorbell = match cerulion_core::wake::Doorbell::new() {
            Ok(d) => Some(d),
            Err(e) => {
                tracing::warn!(
                    error = %e,
                    "guard-condition doorbell creation failed — triggers on this \
                     guard will wake a blocked rmw_wait at its 20ms pump cap \
                     instead of immediately (check the process fd limit)"
                );
                None
            }
        };
        Self {
            triggered: AtomicBool::new(false),
            doorbell,
        }
    }

    /// Trigger the guard: flag first (the truth), then ring (the wake) —
    /// see the type docs for why that order is load-bearing.
    pub fn trigger(&self) {
        self.triggered.store(true, Ordering::Release);
        if let Some(d) = &self.doorbell {
            d.ring();
        }
    }
}

impl Default for GuardConditionState {
    fn default() -> Self {
        Self::new()
    }
}

/// Type-erased payloads hung off the rmw entity structs' `data`
/// pointers. Each is a `Box` owned by the entity; destroy fns reclaim.
pub struct NodeData {
    pub name: String,
    pub namespace: String,
    /// The graph guard condition rcl wires into executors (we trigger it
    /// on local graph changes).
    pub graph_guard: *mut crate::ffi::rmw_guard_condition_t,
}

// SAFETY: graph_guard points at a Box we own for the node's lifetime;
// rcl accesses it from executor threads — the pointee's state is atomic.
unsafe impl Send for NodeData {}
unsafe impl Sync for NodeData {}

/// Publisher state — port AND outstanding loans behind ONE mutex.
///
/// # Soundness
///
/// The Cerulion transport is instantiated over
/// `iceoryx2::service::ipc_threadsafe::Service` (MutexProtected policy), so the
/// publisher port and its samples are `Send + Sync` and the rmw
/// port-structs carry no manual `Send + Sync` impls — these types are
/// `Send + Sync` by construction. The single `inner` mutex remains for a
/// CORRECTNESS reason independent of `Send`: rcl shares each publisher across
/// executor threads through type-erased `void*` pointers, and keeping the port
/// and every outstanding `RawShmLoan`'s sample behind ONE mutex serializes all
/// slot-map mutations. Splitting loans into a second mutex
/// would admit a concrete race between `rmw_publish` and
/// `rmw_return_loaned_message_from_publisher`.
pub struct PublisherInner {
    pub publisher: cerulion_core::transport::publisher::CerulionPublisher,
    /// Outstanding zero-copy loans (rmw_borrow_loaned_message →
    /// rmw_publish_loaned_message), keyed by the payload pointer handed
    /// to rclcpp. Each loan OWNS its iceoryx2 sample — and mutates the
    /// publisher's slot state, hence same-mutex confinement (see the
    /// soundness note on `PublisherInner` above).
    ///
    /// Held UNINIT-typed. The borrow path zeroes the header
    /// region and runs the typesupport `init_function` on the payload
    /// (fields), but repr(C) padding stays unwritten until the publish
    /// path completes the byte coverage (a FIXED loan zeroes the
    /// layout's pad ranges then promotes via `assume_init`; a WINDOWED
    /// loan is sealed and promoted via `assume_init_shm_defined` — see
    /// that method's contract).
    pub pending_loans: Vec<PendingLoan>,
    /// Windowed borrow: window-tail base addresses whose heap-hook
    /// QUARANTINE extents are outstanding (registered when the borrow
    /// armed the window; the entry must outlive the frame — an
    /// incidental allocation the fill bump-allocated into the slot may
    /// be freed long after publish). Retired via the hook's
    /// `retire_slot` when the transport hands the SAME slot out again
    /// (the only reclamation signal an rmw can observe; the
    /// immediately-following re-arm re-establishes coverage under
    /// either hook disposition — delete, or
    /// tombstone-with-revive). Bounded by the publisher's pool slot
    /// count.
    pub quarantined_tails: Vec<usize>,
    /// Windowed borrow: loans held UNSENT because their borrow
    /// thread's window is still armed and cannot be disarmed from the
    /// thread that finished the loan (the wrong-thread publish/return
    /// shape). Holding the loan pins the slot so iceoryx2 can never
    /// recycle it under a live window — an armed window over a recycled
    /// slot would bump a later fill into a shipped frame. Released when
    /// the owning thread next borrows (it disarms its stale window
    /// first). Bounded by the pool slot count; a thread that never
    /// borrows again leaks its one slot, counted through the degrade
    /// latch's `wrong_thread` total.
    pub orphaned_loans: Vec<OrphanedLoan>,
    /// Windowed borrow: per-publisher reusable seal buffers — the windowed
    /// publish path's plan/commit storage, cleared (never dropped) per
    /// publish so a steady-state windowed publish performs no heap
    /// allocation in the seal machinery (pinned by
    /// `rmw_borrow_zero_alloc_test`). Same-mutex confinement with the
    /// loans: the seal runs under the publisher lock.
    pub seal_scratch: crate::type_bridge::SealScratch,
}

/// One outstanding publisher-side loan (`rmw_borrow_loaned_message` →
/// publish/return), keyed by the payload pointer handed to rclcpp.
pub struct PendingLoan {
    /// The pointer rclcpp holds (`slot + WireHeader::SIZE`).
    pub key: usize,
    /// The held iceoryx2 sample.
    pub loan: cerulion_core::transport::publisher::RawShmLoanUninit,
    /// Which publish path finalizes this loan.
    pub kind: LoanKind,
}

/// How a pending publisher loan is finalized.
pub enum LoanKind {
    /// Fixed-type loan: the struct IS the wire fixed section; the
    /// publish stamps the header and sends.
    Fixed,
    /// Windowed-borrow loan: the publish seals the filled
    /// struct into a wire frame (adopt/copy walk) — see
    /// `crate::type_bridge::BridgedMessage::seal_borrowed_frame`.
    Windowed(WindowedLoan),
}

/// The window bookkeeping of one windowed-borrow loan.
pub struct WindowedLoan {
    /// Absolute address the window was armed at (`payload + tail_off`).
    pub tail_base: usize,
    /// Loaned payload length past the `WireHeader`.
    pub payload_len: usize,
    /// The thread whose window THIS loan armed — `None` for a windowless
    /// borrow (the thread's window already belonged to another loan, or
    /// the hook refused the arm). Only the owning thread can disarm.
    pub window_owner: Option<std::thread::ThreadId>,
}

/// A windowed loan held unsent because its borrow thread's window is
/// still armed — see [`PublisherInner::orphaned_loans`].
pub struct OrphanedLoan {
    /// The thread whose window still covers this slot.
    pub thread: std::thread::ThreadId,
    /// The slot's window base (quarantine key).
    pub tail_base: usize,
    /// The held sample pinning the slot.
    pub loan: cerulion_core::transport::publisher::RawShmLoanUninit,
}

pub struct PublisherData {
    pub topic: String,
    pub type_name: String,
    pub bridge: Arc<crate::bridge::AnyBridge>,
    pub inner: Mutex<PublisherInner>,
    pub gid: [u8; 16],
    pub sequence: AtomicU64,
    /// Flood-suppressed loudness + an unconditional counter for
    /// serialized frames REJECTED by `rmw_publish_serialized_message`
    /// because their buffer cannot hold a wire header.
    pub malformed_header_rejects:
        Mutex<cerulion_core::transport::failure_regime_latch::FailureRegimeLatch>,
    /// The same, for serialized frames rejected because their
    /// `schema_hash` disagrees with this publisher's type. SEPARATE from
    /// `malformed_header_rejects` on purpose — "your bytes are not a
    /// frame" and "your frame is the wrong type" are different problems
    /// with different remedies, and sharing one latch would let either
    /// condition's open regime swallow the other's loud head (the same
    /// rule as [`SubscriptionData`]'s hash-vs-decode split).
    pub schema_hash_rejects:
        Mutex<cerulion_core::transport::failure_regime_latch::FailureRegimeLatch>,
    /// Windowed borrow: `Some(geometry)` when this publisher serves
    /// `rmw_borrow_loaned_message` through the borrow WINDOW — set at
    /// create iff the type is windowed-borrowable
    /// (`AnyBridge::can_borrow_windowed`) AND the heap-hook handshake was
    /// Active (`crate::heaphook::active_hook`; the preload set cannot
    /// change after process load, so the create-time answer is the
    /// process answer). `None` means the plain loan surface only.
    pub windowed_borrow: Option<crate::type_bridge::BorrowSlotGeometry>,
    /// Windowed borrow: flood-suppressed loudness + an unconditional
    /// counter for windowed publishes that paid a COPY
    /// (`crate::borrow_degrade_latch` — `kind=` names why). A degrade,
    /// never a loss: the frame still ships correct.
    pub borrow_degrades: Mutex<cerulion_core::transport::failure_regime_latch::FailureRegimeLatch>,
    /// Windowed borrow: wrong-thread loan RETURNS — a slot-hold lifecycle
    /// event, deliberately NOT on [`Self::borrow_degrades`]: a return
    /// publishes and copies nothing, so routing it through the publish
    /// latch would falsely inflate the publish-degrade diagnostic and
    /// [`Self::borrow_degrade_count`]. Recovered by the owner thread's
    /// self-heal sweep.
    pub wrong_thread_returns:
        Mutex<cerulion_core::transport::failure_regime_latch::FailureRegimeLatch>,
    /// Windowed publishes whose every forgeable member
    /// adopted (fully zero-copy). Principle #3 — with
    /// [`Self::borrow_copied_frames`] this is the adoption RATE.
    pub borrow_adopted_frames: AtomicU64,
    /// Windowed publishes that paid a copy on at least
    /// one member (escapees, windowless borrows, seal fallbacks).
    pub borrow_copied_frames: AtomicU64,
}

impl PublisherData {
    /// Unconditional running total of serialized frames rejected for a
    /// malformed wire header — independent of log level and never reset by
    /// recovery (Principle #3).
    ///
    /// Reachable only from Rust: the rmw C ABI is STANDARDIZED, so rclcpp /
    /// rclpy see this entity as an opaque `*mut c_void` and no accessor can be
    /// added for them. That is precisely why the shared latch re-announces an
    /// open regime at each decade of the running total — the LOG is a ROS
    /// user's whole window onto the condition.
    pub fn malformed_header_reject_count(&self) -> u64 {
        cerulion_core::transport::failure_regime_latch::lock_regime_latch(
            &self.malformed_header_rejects,
        )
        .total_failures()
    }

    /// Unconditional running total of serialized frames rejected on the
    /// schema-hash gate. Same reachability caveat as
    /// [`malformed_header_reject_count`](Self::malformed_header_reject_count).
    pub fn schema_hash_reject_count(&self) -> u64 {
        cerulion_core::transport::failure_regime_latch::lock_regime_latch(&self.schema_hash_rejects)
            .total_failures()
    }

    /// Unconditional running total of windowed publishes
    /// that paid a copy — independent of log level, never reset by
    /// recovery (Principle #3; same reachability caveat as the reject
    /// counters — the rmw C ABI is standardized, so the latch's log lines
    /// are a ROS user's only window).
    pub fn borrow_degrade_count(&self) -> u64 {
        cerulion_core::transport::failure_regime_latch::lock_regime_latch(&self.borrow_degrades)
            .total_failures()
    }

    /// Fully zero-copy windowed publishes.
    pub fn borrow_adopted_count(&self) -> u64 {
        self.borrow_adopted_frames.load(Ordering::Relaxed)
    }

    /// Windowed publishes that paid a copy on at least
    /// one member.
    pub fn borrow_copied_count(&self) -> u64 {
        self.borrow_copied_frames.load(Ordering::Relaxed)
    }

    /// Unconditional running total of wrong-thread loan
    /// RETURNS (slot-hold lifecycle events — deliberately separate from
    /// [`Self::borrow_degrade_count`]: a return publishes nothing).
    pub fn borrow_wrong_thread_return_count(&self) -> u64 {
        cerulion_core::transport::failure_regime_latch::lock_regime_latch(
            &self.wrong_thread_returns,
        )
        .total_failures()
    }

    /// Test seam: the loan-bookkeeping vector capacities
    /// `(pending_loans, quarantined_tails, orphaned_loans)` — the
    /// structural pin that publisher CREATE reserved the loan budget, so
    /// an in-budget borrow's push never allocates on the loan path (the
    /// borrow path itself cannot carry a zero-alloc counter pin: the
    /// typesupport `init` allocates on the heap by design).
    ///
    /// Poison-safe via `into_inner` (the documented exception to the
    /// `lock_unpoisoned` rule): this is a read-only DIAGNOSTIC of
    /// structurally-valid capacity fields — poison is a logical flag, a
    /// `Vec`'s capacity cannot be torn, and no slot decision rides the
    /// answer — so it must keep answering after a publisher panic
    /// (asserting diagnostics on a poisoned publisher is exactly what
    /// the poisoned-destroy arms do) rather than aborting the reader.
    #[cfg(feature = "test-seams")]
    pub fn loan_bookkeeping_capacities(&self) -> (usize, usize, usize) {
        let inner = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        (
            inner.pending_loans.capacity(),
            inner.quarantined_tails.capacity(),
            inner.orphaned_loans.capacity(),
        )
    }
}

// No manual Send/Sync impls — `PublisherInner`'s transport
// state is `ipc_threadsafe::Service` (Send+Sync), so `Mutex<PublisherInner>`
// and therefore `PublisherData` are Send+Sync by construction.

/// Subscription state — port AND outstanding take loans behind ONE mutex.
///
/// # Soundness (mirrors [`PublisherInner`])
///
/// The subscriber port and its held samples are `Send + Sync` by construction
/// (`ipc_threadsafe::Service`, MutexProtected policy — see the note under
/// [`PublisherInner`]), but the single `inner` mutex remains for the same
/// CORRECTNESS reason: rcl shares each subscription across executor threads
/// through type-erased `void*` pointers, and keeping the port and every
/// outstanding take loan's `OwnedInboundSample` behind ONE mutex
/// serializes all slot/borrow-accounting mutations. Splitting the loans into a
/// second mutex would admit the same class of race as a split on the
/// publisher side (`rmw_take_loaned_message`'s receive vs.
/// `rmw_return_loaned_message_from_subscription`'s sample drop, both of which
/// mutate the subscriber connection's borrow bookkeeping).
pub struct SubscriptionInner {
    pub subscriber: cerulion_core::transport::subscriber::CerulionSubscriber,
    /// Outstanding zero-copy take loans (`rmw_take_loaned_message` →
    /// `rmw_return_loaned_message_from_subscription`), keyed by the pointer
    /// handed to rclcpp. Each entry OWNS its iceoryx2 sample: while it lives,
    /// the publisher-pool slot is pinned (iceoryx2 borrow accounting — the
    /// slot cannot be reclaimed or overwritten until the `Sample` drops) and
    /// one unit of the service's `subscriber_max_borrowed_samples` budget is
    /// consumed. Dropping the entry (return, or subscription destroy)
    /// releases both — and, for a forged take, un-forges + destroys the
    /// shadow FIRST (field order below is load-bearing).
    pub pending_takes: Vec<PendingTake>,
    /// The recyclable shadow messages the forged take hands
    /// out, sized to the loan budget. Empty (nothing built) until the first
    /// forged take; never touched by a fixed type's take.
    pub shadows: TakeShadowPool,
    /// Zero-alloc reusable scratch for the ADOPTION branch:
    /// the per-take forged-range walk and the registration rollback ledger.
    /// Cleared and refilled per adopted take under this same mutex;
    /// RESERVED at subscription create to the type's forgeable-member count
    /// (the bound both ever reach), so the FIRST adopted take grows
    /// neither, not only the steady state. Never touched off the adopt path.
    pub adopt_ranges: Vec<(usize, usize)>,
    /// The `(range start, cookie)` pairs registered so far in the CURRENT
    /// adopted take — the all-or-nothing rollback ledger (see
    /// `adopt_ranges`).
    pub adopt_registered: Vec<(usize, usize)>,
}

/// One outstanding loaned take.
///
/// `key` is what rclcpp holds and later returns: the SHM payload pointer
/// for a fixed type, the shadow's address for a forged one. Declaration
/// order is the DROP order and it matters: the shadow un-forges (and its
/// `fini` runs over empty sequences) before the sample's SHM borrow is
/// released, so no destructor ever sees a shared-memory address.
pub struct PendingTake {
    /// The loaned pointer as handed to the caller.
    pub key: usize,
    /// The shadow serving a forged take; `None` for a fixed type.
    pub shadow: Option<TakeShadow>,
    /// The held sample — the SHM bytes behind the loan.
    pub sample: cerulion_core::transport::subscriber::OwnedInboundSample,
}

/// One rmw-owned SHADOW message — the object a forged
/// loaned take hands to rclcpp in place of a copy. Its fixed fields,
/// strings and nested containers are real, rmw-filled members on the
/// process heap; its forgeable primitive sequences are aimed at the held
/// SHM sample for the loan's lifetime and re-pointed at nothing before the
/// sample is released or the object is destroyed.
///
/// Built through the typesupport's own `init_function`, destroyed through
/// its `fini_function` (`AnyBridge::new_shadow` / `destroy_shadow`), and
/// only ever destroyed AFTER an un-forge — the rmw owns the whole lifecycle,
/// so no allocator (libc or C++) is ever handed a forged buffer to free.
pub struct TakeShadow {
    ptr: std::ptr::NonNull<std::os::raw::c_void>,
    bridge: Arc<crate::bridge::AnyBridge>,
    /// The mask of members currently FORGED into a held sample
    /// (`ForgeOutcome::forged` of the take being served; `0` while idle).
    forged: u64,
    /// True once a take COPIED into a forgeable member (an entry below the
    /// data floor): that member now owns a heap buffer only the typesupport's
    /// `fini` can free, so this shadow is DESTROYED on release rather than
    /// recycled — the next forge would otherwise overwrite the header and
    /// leak the buffer.
    tainted: bool,
}

// SAFETY: the shadow's bytes are reached only under the subscription's
// `inner` mutex (pool + pending-take table) or through the pointer the rmw
// handed the caller under the loan contract; the bridge is `Send + Sync`.
unsafe impl Send for TakeShadow {}

impl TakeShadow {
    fn build(bridge: &Arc<crate::bridge::AnyBridge>) -> Option<Self> {
        // SAFETY: the pool only builds shadows for a bridge whose
        // `can_loan_take()` holds with a nonzero forged-sequence count (the
        // forged-take arm is the sole caller).
        let ptr = unsafe { bridge.new_shadow() }?;
        Some(Self {
            ptr: std::ptr::NonNull::new(ptr)?,
            bridge: Arc::clone(bridge),
            forged: 0,
            tainted: false,
        })
    }

    /// The object's address — the loaned pointer for a forged take.
    pub fn as_ptr(&self) -> *mut std::os::raw::c_void {
        self.ptr.as_ptr()
    }

    /// Record what a successful `unflatten_forged` did to this shadow: the
    /// forged mask (what [`Self::unforge`] must re-point) and whether any
    /// forgeable member was copied instead (which taints the shadow).
    pub(crate) fn record(&mut self, outcome: &crate::type_bridge::ForgeOutcome) {
        self.forged = outcome.forged;
        self.tainted |= outcome.below_floor > 0;
    }

    /// Re-point every currently-forged member at nothing (idempotent; the
    /// mask is cleared so a second call is a no-op).
    pub(crate) fn unforge(&mut self) {
        // SAFETY: `ptr` came from this bridge's `new_shadow`.
        unsafe { self.bridge.unforge(self.ptr.as_ptr(), self.forged) }
        self.forged = 0;
    }

    /// True once a take copied into a forgeable member; such a shadow is
    /// destroyed on release, never recycled.
    pub fn is_tainted(&self) -> bool {
        self.tainted
    }
}

impl Drop for TakeShadow {
    fn drop(&mut self) {
        // SAFETY: `ptr` came from this bridge's `new_shadow` and is not used
        // after this. `destroy_shadow` un-forges the mask before `fini`, so
        // the destructor frees copies only, never a shared-memory address.
        unsafe { self.bridge.destroy_shadow(self.ptr.as_ptr(), self.forged) }
    }
}

/// The per-subscription pool of recyclable shadows.
///
/// Sized to the loan budget: a consumer can hold at most `capacity` forged
/// loans at once, and the (`capacity + 1`)th take is REFUSED before any
/// frame is consumed — the loud, latched budget arm (see
/// [`crate::loan_refusal_latch`]). Shadows are built LAZILY (a subscription
/// that never takes a loan builds none) and then RECYCLED, so a steady-state
/// forged take allocates nothing on the rmw side.
pub struct TakeShadowPool {
    free: Vec<TakeShadow>,
    built: usize,
    capacity: usize,
}

impl TakeShadowPool {
    /// An empty pool that will build at most `capacity` shadows.
    pub(crate) fn new(capacity: usize) -> Self {
        Self {
            free: Vec::with_capacity(capacity),
            built: 0,
            capacity,
        }
    }

    /// Take a free shadow, or build one while under capacity.
    pub(crate) fn acquire(
        &mut self,
        bridge: &Arc<crate::bridge::AnyBridge>,
    ) -> Result<TakeShadow, crate::loan_refusal_latch::LoanRefusal> {
        if let Some(shadow) = self.free.pop() {
            return Ok(shadow);
        }
        if self.built >= self.capacity {
            return Err(crate::loan_refusal_latch::LoanRefusal::ShadowPoolExhausted);
        }
        let shadow = TakeShadow::build(bridge)
            .ok_or(crate::loan_refusal_latch::LoanRefusal::ShadowAllocationFailed)?;
        self.built += 1;
        Ok(shadow)
    }

    /// Return a shadow. It is UN-FORGED here, unconditionally, so a recycled
    /// shadow can never carry a dangling sequence into its next loan — the
    /// one place the un-forge-before-release rule is enforced for the
    /// recycle path (the destroy path enforces it in `Drop`). A TAINTED
    /// shadow (one that copied into a forgeable member) is destroyed instead
    /// of recycled — its `fini` frees the copy — and the pool's build count
    /// gives the slot back, so the next take builds a fresh one.
    pub(crate) fn release(&mut self, mut shadow: TakeShadow) {
        shadow.unforge();
        if shadow.is_tainted() {
            self.built = self.built.saturating_sub(1);
            drop(shadow);
        } else {
            self.free.push(shadow);
        }
    }

    /// The most shadows this pool will ever build (== the loan budget).
    pub fn capacity(&self) -> usize {
        self.capacity
    }

    /// Shadows built so far (never exceeds [`Self::capacity`]).
    pub fn built(&self) -> usize {
        self.built
    }

    /// Shadows currently idle in the pool.
    pub fn free_count(&self) -> usize {
        self.free.len()
    }
}

pub struct SubscriptionData {
    pub topic: String,
    pub type_name: String,
    pub bridge: Arc<crate::bridge::AnyBridge>,
    pub inner: Mutex<SubscriptionInner>,
    /// Deterministic per-process entity gid (entity counter + schema
    /// hash, never random) — the endpoint id reported by
    /// `rmw_get_subscriptions_info_by_topic` and the key the graph
    /// registry deregisters by on destroy.
    pub gid: [u8; 16],
    /// Flood-suppressed loudness for a frame that passes the
    /// schema-hash gate but that the bridge cannot DECODE. Without the latch
    /// such a frame would drop silently (no log, no counter),
    /// indistinguishable from an empty queue.
    pub decode_failures: Mutex<crate::decode_failure_latch::DecodeFailureLatch>,
    /// Flood-suppressed loudness + an unconditional counter for
    /// frames dropped on the schema-hash gate. SEPARATE from
    /// `decode_failures` on purpose — a hash mismatch (the two ends
    /// disagree on the type) and a decode failure (they agree on the
    /// type and disagree on the framing) are different conditions with
    /// different remedies, and sharing one latch would let either
    /// condition's regime mask the other's loud head.
    pub hash_mismatches: Mutex<cerulion_core::transport::failure_regime_latch::FailureRegimeLatch>,
    /// Forged loaned take: flood-suppressed loudness + an unconditional counter
    /// for loaned takes REFUSED on a budget (the shadow pool, or the
    /// transport's borrow budget) — see [`crate::loan_refusal_latch`].
    /// Its own latch: a refused take is the CONSUMER retaining loans, a
    /// different condition with a different remedy from a bad frame
    /// (`decode_failures`) or a skewed type (`hash_mismatches`).
    pub loan_refusals: Mutex<cerulion_core::transport::failure_regime_latch::FailureRegimeLatch>,
    /// Forged loaned take: flood-suppressed loudness + an unconditional counter
    /// for forged takes that had to COPY a sequence whose entry sat below the
    /// frame's data floor (`loan_refusal_latch::report_forge_fallback`). Its
    /// own latch: the take SUCCEEDED (a degrade, not a refusal) and the
    /// remedy is on the PRODUCER's side, so it must not share the refusal
    /// regime.
    pub forge_fallbacks: Mutex<cerulion_core::transport::failure_regime_latch::FailureRegimeLatch>,
    /// Adopt-take (`--adopt-take`): `Some` iff the env gate
    /// was armed at CREATE, the heap-hook grant was constructible, AND the
    /// type carries at least one forgeable sequence — the witness plus the
    /// `Arc`-shared counters and the adopt borrow budget. `None` keeps the
    /// plain copying take in place (see `crate::adopt_take`).
    pub adopt: Option<crate::adopt_take::AdoptTakeState>,
}

impl SubscriptionData {
    /// Unconditional running total of forged takes that copied a
    /// below-the-floor sequence — independent of log level, never reset by
    /// recovery (Principle #3).
    pub fn forge_fallback_count(&self) -> u64 {
        cerulion_core::transport::failure_regime_latch::lock_regime_latch(&self.forge_fallbacks)
            .total_failures()
    }

    /// Unconditional running total of loaned takes refused on a budget —
    /// independent of log level and never reset by recovery (Principle #3).
    /// Reachable only from Rust (the rmw C ABI is standardized — see
    /// [`PublisherData::malformed_header_reject_count`]).
    pub fn loan_refusal_count(&self) -> u64 {
        cerulion_core::transport::failure_regime_latch::lock_regime_latch(&self.loan_refusals)
            .total_failures()
    }
}

// No manual Send/Sync impls — the subscriber is over
// `ipc_threadsafe::Service` (Send+Sync), so `Mutex<SubscriptionInner>` and
// therefore `SubscriptionData` are Send+Sync by construction. (the
// held `OwnedInboundSample`s are `Send` too — compile-time-pinned in
// cerulion_core `transport/subscriber.rs`, the `assert_send::<OwnedInboundSample>`
// const closure — which is what lets rclcpp return a loan from a different
// executor thread than the one that took it.)

pub struct ServiceData {
    pub service_name: String,
    pub type_name: String,
    pub request_bridge: Arc<crate::bridge::AnyBridge>,
    pub response_bridge: Arc<crate::bridge::AnyBridge>,
    pub server: Mutex<cerulion_core::transport::service::ServiceServer>,
    /// See [`SubscriptionData::decode_failures`] — a request the
    /// bridge cannot decode must not be dropped silently.
    pub decode_failures: Mutex<crate::decode_failure_latch::DecodeFailureLatch>,
}

// No manual Send/Sync impls — the server is over
// `ipc_threadsafe::Service` (Send+Sync), so `Mutex<ServiceServer>` and
// therefore `ServiceData` are Send+Sync by construction.

pub struct ClientData {
    pub service_name: String,
    pub type_name: String,
    pub request_bridge: Arc<crate::bridge::AnyBridge>,
    pub response_bridge: Arc<crate::bridge::AnyBridge>,
    pub client: Mutex<cerulion_core::transport::service::ServiceClient>,
    pub gid: [u8; 16],
    /// See [`SubscriptionData::decode_failures`] — a response the
    /// bridge cannot decode must not be dropped silently.
    pub decode_failures: Mutex<crate::decode_failure_latch::DecodeFailureLatch>,
}

// No manual Send/Sync impls — the client is over
// `ipc_threadsafe::Service` (Send+Sync), so `Mutex<ServiceClient>` and
// therefore `ClientData` are Send+Sync by construction.

#[cfg(test)]
mod tests {
    use super::{ros_topic_to_cerulion, RosNameError};

    /// The mapping is the IDENTITY on every fully-qualified ROS name — hand
    /// oracle per input (never a self-compare): plain, namespaced, a name a
    /// ROS namespace convention would mangle (`_` / digits), a one-segment
    /// and a two-segment minimal name, and a deep one.
    #[test]
    fn fully_qualified_names_map_to_themselves_verbatim() {
        let cases: &[(&str, &str)] = &[
            ("/chatter", "/chatter"),
            ("/cmd_vel", "/cmd_vel"),
            ("/ns/sub/topic_1", "/ns/sub/topic_1"),
            ("/add_two_ints", "/add_two_ints"),
            ("/a", "/a"),
            ("/a/b", "/a/b"),
        ];
        for &(input, want) in cases {
            assert_eq!(
                ros_topic_to_cerulion(input).as_deref(),
                Ok(want),
                "{input:?} must map to itself"
            );
        }
    }

    /// A relative name is REFUSED, never prefixed: a slash-stripped
    /// spelling must not silently become the canonical one (no alias). Each
    /// arm pins the exact reason.
    #[test]
    fn relative_and_empty_names_are_refused_never_prefixed() {
        assert_eq!(
            ros_topic_to_cerulion("chatter"),
            Err(RosNameError::NoLeadingSlash)
        );
        assert_eq!(
            ros_topic_to_cerulion("ns/topic"),
            Err(RosNameError::NoLeadingSlash)
        );
        assert_eq!(ros_topic_to_cerulion(""), Err(RosNameError::Empty));
        // A leading space is not a leading slash.
        assert_eq!(
            ros_topic_to_cerulion(" /chatter"),
            Err(RosNameError::NoLeadingSlash)
        );
    }

    /// An EMPTY SEGMENT is refused, never trimmed: a lone `/` (which the
    /// transport would otherwise accept and turn into a malformed `//data`
    /// service), `//`, `///`, a trailing `/`, and a `//` inside — each pinned
    /// to the exact reason, with the accepted minimal shapes as the control
    /// so the check cannot be satisfied by refusing everything.
    #[test]
    fn empty_segments_are_refused_never_trimmed() {
        for bad in ["/", "//", "///", "/a/", "/a//b", "/a/b/", "//a"] {
            assert_eq!(
                ros_topic_to_cerulion(bad),
                Err(RosNameError::EmptySegment),
                "{bad:?} has an empty segment and must be refused"
            );
        }
        for good in ["/a", "/a/b", "/a_1/b2/c"] {
            assert_eq!(
                ros_topic_to_cerulion(good).as_deref(),
                Ok(good),
                "{good:?} is fully qualified with no empty segment"
            );
        }
    }

    /// The refusal reasons render distinctly (an operator reads the log
    /// line, not the enum) and the empty check runs before the slash check.
    #[test]
    fn refusal_reasons_render_distinctly() {
        let empty = RosNameError::Empty.to_string();
        let relative = RosNameError::NoLeadingSlash.to_string();
        let segment = RosNameError::EmptySegment.to_string();
        assert_ne!(empty, relative);
        assert_ne!(relative, segment);
        assert_ne!(empty, segment);
        assert!(empty.contains("empty"), "{empty}");
        assert!(relative.contains("leading '/'"), "{relative}");
        assert!(
            relative.contains("never prefixes"),
            "the message must state the no-repair rule: {relative}"
        );
        assert!(segment.contains("empty segment"), "{segment}");
        assert!(
            segment.contains("never repairs"),
            "the message must state the no-repair rule: {segment}"
        );
    }
}
