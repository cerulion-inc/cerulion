// SPDX-License-Identifier: AGPL-3.0-only
//! rmw publisher / subscription surface: create, destroy, publish,
//! take, and the loaned-message zero-copy path.

use std::collections::HashMap;
use std::os::raw::{c_char, c_void};
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex, OnceLock};

use cerulion_core::codegen::{
    slice_ceiling_for_type, SliceCeilingOverrides, RMW_SLICE_CEILING_ENV,
};
use cerulion_core::transport::failure_regime_latch::{
    lock_regime_latch, FailureRegimeLatch, RegimeDecision,
};
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::TransportError;

use crate::borrow_degrade_latch::{
    report_borrow_adopted, report_borrow_degraded, report_wrong_thread_return,
    report_wrong_thread_return_healed, BorrowDegrade,
};
use crate::bridge::AnyBridge;
use crate::ffi::{
    self, rmw_ret_t, RMW_RET_BAD_ALLOC, RMW_RET_ERROR, RMW_RET_INVALID_ARGUMENT, RMW_RET_OK,
    RMW_RET_UNSUPPORTED,
};
use crate::runtime::{
    self, GraphRegistry, PublisherData, PublisherInner, SubscriptionData, SubscriptionInner,
};
use crate::type_bridge::{BorrowSlotGeometry, BridgedMessage, SealRefusal, WindowExtent};
use crate::type_bridge_cpp::CppBridgedMessage;

/// The blanket ceiling for bridged VARIABLE messages, reachable
/// only as [`slice_len_with_overrides`]'s fixed-arm `unwrap_or` (provably
/// untaken: the exact bound is floored at `WireHeader::SIZE`, which is
/// `MaxSliceLen`'s own floor). The VARIABLE arm resolves per type through
/// [`slice_ceiling_for_type`] instead — the same five-tier table that sizes
/// graph-declared outputs of the same schemas — whose catch-all for unlisted
/// types is this SAME 128 MiB (`cerulion_core`'s tier catch-all /
/// `graph::config::DEFAULT_MAX_SLICE_LEN`), so a type
/// neither the table nor the env names keeps this ceiling byte-identically.
const BRIDGED_VARIABLE_SLICE_LEN: MaxSliceLen = MaxSliceLen::const_new(128 * 1024 * 1024);

/// The `subscriber_max_borrowed_samples` a loanable (plain,
/// recursively-fixed) rmw topic's iceoryx2 service is CREATED with, so a
/// loaned take (`rmw_take_loaned_message`) can HOLD samples across the C ABI.
///
/// Each outstanding take loan is one live borrow on the subscription's
/// connection. The budget is sized for the real rclcpp shapes:
///
/// - `SingleThreadedExecutor`: at most 1 loan held at a time (the loaned
///   callback returns before the next take);
/// - `MultiThreadedExecutor` with a reentrant callback group: one loan per
///   concurrently-executing callback — bounded in practice by the executor's
///   thread count, but not by the rmw;
/// - plus 1 transient for the `receive()` in progress on a plain `rmw_take`
///   racing held loans.
///
/// 4 = 3 concurrently-held loans + 1 transient headroom — enough for the
/// STE (1) and a small MTE, without inflating every rmw topic's publisher
/// pool (the pool grows by `max_subscribers × borrow` slots at create;
/// Unused slots are demand-paged, so the cost is address space).
///
/// Exhaustion is LOUD and bounded, never silent loss and never a hang: a take
/// past the budget fails with iceoryx2's `ExceedsMaxBorrows`, surfaced as
/// `RMW_RET_ERROR` plus an `error!` naming the outstanding-loan count and the
/// remedy (`rmw_return_loaned_message_from_subscription`); returning any one
/// loan recovers the next take.
///
/// Applied as [`TopicServiceConfig::create_borrow_floor`] — a CREATE-leg-only
/// floor (create provisions, open tolerates), so an rmw create never
/// refuses a pre-existing smaller service (e.g. one minted first by a native
/// `cerulion topic echo` at the iceoryx2 default of 2, or by a graph at the
/// HOLD floor of 3); against such a service the loan budget is simply
/// the smaller created value, and the loud `ExceedsMaxBorrows` arm binds
/// earlier.
///
/// [`TopicServiceConfig::create_borrow_floor`]: cerulion_core::transport::TopicServiceConfig::create_borrow_floor
pub(crate) const RMW_TAKE_LOAN_BORROW_BUDGET: usize = 4;

/// Windowed borrow: the
/// iceoryx2 `publisher_max_loaned_samples` a LOANABLE rmw topic's publisher
/// port is created with (fixed-type loans and windowed borrows alike; the
/// iceoryx2 default is 2). The windowed publish holds the borrow loan while
/// a refusal fallback takes a second, exact-size loan, and rclcpp callers
/// can hold several borrows at once — 4 = 3 concurrently-held borrows + 1
/// fallback/transient, mirroring [`RMW_TAKE_LOAN_BORROW_BUDGET`]'s shape.
/// Cost is APPARENT bytes only: each unit is one more slice-ceiling-sized
/// slot in the demand-paged pool.
pub(crate) const RMW_LOANED_SAMPLES_BUDGET: usize = 4;

/// The fill TAIL a windowed borrow reserves past the
/// message struct — the borrow window's capacity, i.e. how much unbounded
/// payload a fill can place before it overflows to the heap (escape ⇒
/// copy). 32 MiB covers a 4K RGB `sensor_msgs/Image` (~24 MiB) with room;
/// an 8K frame escapes and publishes through the exact-size copy loan —
/// degraded, loud, never failed. Apparent bytes only (the loan is sliced
/// from the topic's demand-paged pool and consumers slice on
/// `total_size`).
pub(crate) const RMW_BORROW_TAIL_BYTES: usize = 32 * 1024 * 1024;

/// The most dead gap bytes a zero-copy windowed frame may
/// ship (`seal_borrowed_frame`'s budget — the compaction threshold). Gaps
/// come from abandoned earlier fills (a grown vector's first allocation)
/// and incidental same-thread allocations; below the budget the zero-copy
/// win outweighs the slack, above it a tight copy frame is cheaper than
/// shipping (and recording) megabytes of dead bytes — and the copy also
/// bounds how much incidental process memory can ride a frame's gaps.
pub(crate) const RMW_BORROW_MAX_GAP_BYTES: usize = 4 * 1024 * 1024;

/// Principle #3: windowed slots deliberately LEAKED at
/// publisher destroy — because another thread's borrow window was still
/// armed over them, or because a panic POISONED the loan bookkeeping and
/// nothing about it can be trusted for a release decision (see the
/// teardown block in `rmw_destroy_publisher`). Each leaked slot also
/// retains its message's heap-side containers (`fini` cannot run — it
/// would race the owner thread's live fill). Process-wide and never
/// reset; readable via [`borrow_destroy_leak_count`] — the observable
/// the destroy-lifecycle regression arms pin.
///
/// A destroy that leaks ANY slot also leaks the whole
/// `PublisherData`, so the `CerulionPublisher` inside it is never dropped
/// and all THREE of its iceoryx2 ports — the data publisher plus the event
/// notifier and listener — stay REGISTERED for the life of this process:
/// one of the topic's publisher slots and one notifier + one listener slot
/// on its event service are held, against every process on the machine. And
/// because `CerulionPublisher::drop` is the only sender of
/// `PubSubEvent::PublisherDisconnected`, no disconnect edge is ever sent;
/// the transport's liveliness is COUNT-based (a native subscriber's `Lost`
/// edge comes from the runtime sweep reading the live publisher count), so
/// subscribers of the topic see this publisher as ALIVE until the process
/// exits. A leaked sample keeps the port's on-disk tag alive, and a
/// DEREGISTERED port with a live tag wedges every later dead-node sweep of
/// this process's node (the teardown block cites the iceoryx2 file:line
/// facts). The slot cost is stated there too: rmw topics carry iceoryx2's default
/// `max_publishers = 2`, so a re-create on the same ROS topic still
/// succeeds only while the topic's OTHER slot is free (it takes that slot)
/// and the next returns NULL — with any other live publisher already on
/// the topic (the `/rosout` shape) the first re-create is the refused one.
static BORROW_DESTROY_LEAKS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Unconditional running total of windowed slots leaked at publisher
/// destroy (cross-thread armed windows; poisoned loan bookkeeping) — see
/// the teardown block in `rmw_destroy_publisher`. Process-wide, never
/// reset (Principle #3).
pub fn borrow_destroy_leak_count() -> u64 {
    BORROW_DESTROY_LEAKS.load(Ordering::Relaxed)
}

pub(crate) unsafe fn bridge_for(
    type_support: *const ffi::rosidl_message_type_support_t,
) -> Result<Arc<AnyBridge>, rmw_ret_t> {
    let handle = super::resolve_introspection(type_support).ok_or(RMW_RET_INVALID_ARGUMENT)?;
    let bridge = match handle {
        super::IntrospectionHandle::C(members) => BridgedMessage::new(members).map(AnyBridge::C),
        super::IntrospectionHandle::Cpp(members) => {
            CppBridgedMessage::new(members).map(AnyBridge::Cpp)
        }
    };
    match bridge {
        Ok(b) => Ok(Arc::new(b)),
        Err(e) => {
            tracing::error!(error = %e, "type bridge registration failed");
            Err(RMW_RET_ERROR)
        }
    }
}

/// The rmw BRIDGE half of the slice-ceiling override: the ceiling for a bridged topic's
/// SHM slots, resolved through the core per-type lookup.
///
/// The lookup key is [`AnyBridge::qualified_name`] — `pkg/Type`, which is
/// ALREADY the tier table's canonical key form: `qualified_name_of` keeps
/// only the package half of the rosidl `pkg__msg` introspection namespace
/// (`tf2_msgs__msg` + `TFMessage` ⇒ `tf2_msgs/TFMessage`). The ROS graph
/// spelling [`ros_graph_type_name`] derives (`pkg/msg/Type`) is NOT the key —
/// feeding it would trip the core lookup's rosidl-spelling warn and land
/// every type on the catch-all (pinned by
/// `slice_ceiling_tests::the_lookup_key_is_the_bridges_pkg_type_name_not_the_ros_graph_spelling`).
///
/// Cold path: called once per `rmw_create_publisher`, never per message.
fn slice_len_for(bridge: &AnyBridge) -> MaxSliceLen {
    slice_len_with_overrides(bridge, rmw_slice_ceiling_overrides())
}

/// The pure decision under [`slice_len_for`], parameterized on the parsed
/// override set (oracle-testable without env mutation).
///
/// * FIXED layout: the provably-exact `WireHeader + fixed section` bound,
///   never widened or narrowed — a bigger slot buys nothing and a
///   smaller one would refuse every publish, so a
///   [`CERULION_RMW_SLICE_CEILING`][RMW_SLICE_CEILING_ENV] entry naming a
///   fixed bridged type is IGNORED with a `warn!` at create time (per
///   creation — bounded and cold) rather than left silently inert.
/// * VARIABLE layout: the env override wins OUTRIGHT (widen or narrow — the
///   operator's explicit word, per the core resolver's contract); otherwise
///   the five-tier table via [`slice_ceiling_for_type`], whose catch-all for
///   a type neither names is the blanket 128 MiB.
fn slice_len_with_overrides(bridge: &AnyBridge, overrides: &SliceCeilingOverrides) -> MaxSliceLen {
    let qualified = bridge.qualified_name();
    if bridge.layout().is_fixed() {
        if overrides.get(qualified).is_some() {
            report_fixed_override_ignored(qualified);
        }
        let exact = (WireHeader::SIZE + bridge.layout().fixed_size) as u32;
        MaxSliceLen::try_new(exact.max(WireHeader::SIZE as u32))
            .unwrap_or(BRIDGED_VARIABLE_SLICE_LEN)
    } else if let Some(overridden) = overrides.get(qualified) {
        overridden
    } else {
        slice_ceiling_for_type(qualified)
    }
}

/// The ignored-override warn above is flood-suppressed per type,
/// because the condition repeats identically on every publisher
/// creation of that fixed type — the env is parsed once, so the regime can
/// never heal in-process — and a ROS graph can create many publishers of one
/// type, so a bare per-creation `warn!` floods stderr despite parse-once. One
/// [`FailureRegimeLatch`] per TYPE NAME (a keyed registry, not a
/// `PublisherData` field — `slice_len_for` runs before any handle exists), so
/// a second fixed type still gets its own loud head. There is deliberately no
/// `on_success` call: no in-process event can end the regime, and the decade
/// re-announcements bound the total at `log10(creations)` lines.
fn report_fixed_override_ignored(qualified: &str) {
    static LATCHES: OnceLock<Mutex<HashMap<String, FailureRegimeLatch>>> = OnceLock::new();
    let mut latches = LATCHES
        .get_or_init(|| Mutex::new(HashMap::new()))
        .lock()
        // A DIAGNOSTIC latch must never wedge the path it observes (the
        // `lock_regime_latch` rule, applied to the keyed map).
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    let decision = latches
        .entry(qualified.to_string())
        .or_default()
        .on_failure();
    match decision {
        RegimeDecision::Loud => tracing::warn!(
            type_name = %qualified,
            "{RMW_SLICE_CEILING_ENV} names a FIXED-layout bridged type — the \
             override is IGNORED: fixed types are sized exactly at wire header \
             + fixed section (a larger slot buys nothing; a smaller one would \
             refuse every publish). Repeats for this type log at debug"
        ),
        RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
            type_name = %qualified,
            total_failures = total,
            suppressed,
            "{RMW_SLICE_CEILING_ENV} STILL names this FIXED-layout bridged type — \
             the override is IGNORED on every creation"
        ),
        RegimeDecision::Suppressed { suppressed } => tracing::debug!(
            type_name = %qualified,
            suppressed,
            "{RMW_SLICE_CEILING_ENV} names a FIXED-layout bridged type \
             (suppressed repeat) — override IGNORED"
        ),
    }
}

/// The parsed [`RMW_SLICE_CEILING_ENV`] set, read ONCE per process at the
/// first publisher creation. Parse-once IS this seam's flood regime: the core
/// parser warns per malformed entry, so a per-creation re-parse would repeat
/// every malformed-entry warn once per publisher — an rclcpp process creating
/// hundreds of publishers must not pay a hundred identical lines for one typo
/// (the same once-per-regime discipline as the crate's failure latches, on a
/// path too cold to need one).
static RMW_SLICE_CEILING_OVERRIDES: OnceLock<SliceCeilingOverrides> = OnceLock::new();

fn rmw_slice_ceiling_overrides() -> &'static SliceCeilingOverrides {
    RMW_SLICE_CEILING_OVERRIDES.get_or_init(|| match std::env::var(RMW_SLICE_CEILING_ENV) {
        Ok(value) => SliceCeilingOverrides::parse(&value),
        Err(std::env::VarError::NotPresent) => SliceCeilingOverrides::default(),
        Err(std::env::VarError::NotUnicode(raw)) => {
            // Loud, never a silent drop: the operator SET the variable; a
            // non-UTF-8 value dropping every override quietly would be the
            // silently-inert class the core parser exists to kill.
            tracing::warn!(
                raw = ?raw,
                "{RMW_SLICE_CEILING_ENV} is not valid UTF-8; ignoring every \
                 slice-ceiling override"
            );
            SliceCeilingOverrides::default()
        }
    })
}

/// Register `topic` for NETWORK EGRESS at runtime — the same best-effort,
/// network-free side-call the `ros2 attach` bridge makes for each of its raw
/// routes (`examples/go2/nodes/dds_bridge/src/generic.rs`). Pushes `(canonical
/// topic, schema_hash)` over the reserved `/__cerulion/gateway_topics` control
/// service; a gateway on this machine drains it and ANNOUNCES the topic +
/// serves remote demand for it, so an rmw publisher's frames reach networked
/// Cerulion hosts exactly like a YAML producer's. Without this call NOTHING
/// registers rmw topics: a gateway learns a runtime topic only from this
/// channel, and the desk refuses to demand anything a robot does not catalog.
///
/// Idempotent, and the channel's own pump periodically republishes the whole
/// set, so a gateway that boots AFTER this publisher still converges. The rmw
/// itself opens no network session — with no gateway on the machine the
/// registration is simply inert.
///
/// BEST-EFFORT by design: local ROS pub/sub must never fail because the
/// registration could not be queued. Two failure classes, each reported at
/// its real cadence:
/// - a registration-hostile NAME (the reserved `/__cerulion/` control-plane
///   namespace, or a name over the channel's byte bound) is PER-TOPIC and
///   independently actionable — loud every time;
/// - a PROCESS-WIDE local control-plane failure (the control service cannot
///   be opened — a broken SHM environment, never a network condition) is the
///   same for every publisher this process ever creates, so it rides ONE
///   process-wide [`FailureRegimeLatch`]: loud head, `debug!` repeats, a loud
///   re-announcement per decade of the running total, and an `info!` on
///   recovery (a rclcpp process creating hundreds of publishers must not
///   flood a log with one identical line per publisher).
///
/// Panic-CONTAINED: the transport's registration can descend into iceoryx2
/// (loan + send on the control service — panic-capable on corrupt SHM), and
/// this runs LAST in `rmw_create_publisher`, on a handle that is already fully
/// constructed and registered. An escaping panic there would reach
/// `ffi_guard`, which returns null to rcl — orphaning a live, registered
/// handle. The registration is best-effort by contract, so a panic inside it
/// is caught HERE and reported as the process-wide control-plane failure it
/// is; the publisher stays local-only and the caller keeps its handle.
///
/// There is NO unregister — see the note in [`rmw_destroy_publisher`].
fn register_dynamic_egress(transport: &TransportManager, topic: &str, schema_hash: u64) {
    /// One regime for the whole process: the control-plane failure is
    /// process-wide, so the latch is too (not per publisher).
    static CONTROL_PLANE: Mutex<FailureRegimeLatch> = Mutex::new(FailureRegimeLatch::new());
    // `AssertUnwindSafe`: the transport holds NO lock across its panic-capable
    // work — the lazy-slot guard covers only an `Arc` clone/install, the
    // record map's lock is released before the send, and the record is
    // retained + the pump armed BEFORE the one step that can panic (otherwise
    // a panic would poison the slot and turn
    // every later publisher in the process local-only). So a caught panic
    // here leaves the plane fully usable: the next publisher registers
    // normally, and this topic's record is already in the set for the pump to
    // republish. This catch is the last line of defense for the HANDLE only.
    let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        transport.register_dynamic_egress_topic(topic, schema_hash)
    }));
    let result = match outcome {
        Ok(result) => result,
        Err(payload) => Err(TransportError::Internal {
            reason: format!(
                "dynamic egress registration panicked: {}",
                panic_message(payload.as_ref())
            ),
        }),
    };
    match result {
        Ok(_) => {
            let recovered = lock_regime_latch(&CONTROL_PLANE).on_success();
            if let Some(suppressed_count) = recovered {
                tracing::info!(
                    topic = %topic,
                    suppressed_count,
                    "dynamic egress registration recovered — the local control plane is \
                     reachable again; this and later publishers register normally"
                );
            }
        }
        Err(error @ TransportError::InvalidTransportConfig { .. }) => {
            tracing::warn!(
                topic = %topic,
                error = %error,
                "dynamic egress registration REFUSED (registration-hostile name) — the topic \
                 stays LOCAL-ONLY (remote hosts cannot discover or demand it); local ROS \
                 pub/sub is unaffected"
            );
        }
        Err(error) => {
            let decision = lock_regime_latch(&CONTROL_PLANE).on_failure();
            match decision {
                RegimeDecision::Loud => tracing::warn!(
                    topic = %topic,
                    error = %error,
                    "dynamic egress registration failed (LOCAL control plane — the \
                     __cerulion/gateway_topics service could not be opened; NOT a network \
                     condition). This and every later publisher in this process stay \
                     LOCAL-ONLY; local ROS pub/sub is unaffected. Repeats log at debug"
                ),
                RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
                    topic = %topic,
                    error = %error,
                    total_failures = total,
                    suppressed,
                    "dynamic egress registration STILL failing (local control plane) — \
                     every publisher this process created stays LOCAL-ONLY"
                ),
                RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                    topic = %topic,
                    error = %error,
                    suppressed,
                    "dynamic egress registration failed (control plane, suppressed repeat) — \
                     topic LOCAL-ONLY"
                ),
            }
        }
    }
}

/// The human-readable text of a caught panic payload (`panic!("…")` carries a
/// `&str` or a `String`; anything else is named as such rather than dropped).
fn panic_message(payload: &(dyn std::any::Any + Send)) -> &str {
    payload
        .downcast_ref::<&str>()
        .copied()
        .or_else(|| payload.downcast_ref::<String>().map(String::as_str))
        .unwrap_or("non-string panic payload")
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_create_publisher(
    node: *const ffi::rmw_node_t,
    type_support: *const ffi::rosidl_message_type_support_t,
    topic_name: *const c_char,
    qos: *const ffi::rmw_qos_profile_t,
    publisher_options: *const ffi::rmw_publisher_options_t,
) -> *mut ffi::rmw_publisher_t {
    ffi::ffi_guard(std::ptr::null_mut(), || unsafe {
        if node.is_null() || qos.is_null() || publisher_options.is_null() {
            return std::ptr::null_mut();
        }
        if !ffi::is_our_identifier((*node).implementation_identifier) {
            return std::ptr::null_mut();
        }
        let Some(ros_topic) = ffi::cstr(topic_name) else {
            return std::ptr::null_mut();
        };
        let Ok(rt) = runtime::runtime() else {
            return std::ptr::null_mut();
        };
        let Ok(bridge) = bridge_for(type_support) else {
            return std::ptr::null_mut();
        };

        // The ROS name IS the Cerulion canonical name (identity mapping — see
        // `runtime::ros_topic_to_cerulion`). A relative name cannot arrive
        // through rcl; one reaching here is refused LOUDLY, never prefixed.
        let topic = match runtime::ros_topic_to_cerulion(ros_topic) {
            Ok(topic) => topic,
            Err(error) => {
                tracing::error!(
                    topic = %ros_topic,
                    error = %error,
                    "publisher creation refused: the ROS name is not a fully-qualified topic name"
                );
                return std::ptr::null_mut();
            }
        };
        // Transient-local durability maps to Cerulion history (late-joiner
        // replay); the requested depth is clamped to [1, 16] (floor 1 so a
        // TRANSIENT_LOCAL request always retains at least one frame; cap 16 is
        // an rmw policy choice — Cerulion core accepts a larger `history_size`,
        // it just warns when history exceeds 75% of the subscriber buffer,
        // which the ceiling raise below neutralizes by provisioning that buffer up).
        // VOLATILE (and any non-TRANSIENT_LOCAL durability) retains nothing
        // (history = 0).
        //
        // `depth_governs_retention` rides the SAME branch on purpose (the
        // retention-depth reporting rule): only a TRANSIENT_LOCAL
        // ask participates in the retention at all. Every other durability
        // retains 0 BECAUSE OF THE POLICY the caller chose — that is what
        // VOLATILE MEANS, not a divergence from the ask — so its create must
        // not warn. Deriving the flag here rather than re-testing the
        // durability at the warn site is what keeps the two from drifting: a
        // future retaining durability cannot be added without visiting this
        // tuple, and the clamp constant can move without touching the gate.
        let (history, depth_governs_retention) =
            if (*qos).durability == ffi::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL {
                ((*qos).depth.clamp(1, 16), true)
            } else {
                (0, false)
            };
        // The retention-depth reporting rule:
        // the provisioned depth endpoint info reports is HISTORY / RETENTION —
        // the frames this publisher actually keeps — NOT the subscriber-slot
        // ceiling `topic_config.subscriber_max_buffer_size` computed below.
        //
        // A TRANSIENT_LOCAL depth-1000 publisher therefore reports 16 (the
        // clamp), a depth-5 one reports 5, and a VOLATILE one reports 0. The
        // ceiling (22 for a clamped history 16) is a slot BUDGET, not a depth
        // anything retains: reporting it would make a depth-5 publisher claim
        // 16 while retaining 5. The 22 ceiling stays pinned where
        // it belongs, by the require-N probes in
        // `rmw_transient_local_ceiling_test.rs`.
        let requested_depth = (*qos).depth;
        // A TRANSIENT_LOCAL publisher makes its own history request SATISFIABLE
        // by provisioning the
        // subscriber buffer ceiling UP (this option was chosen over clamping
        // history down or touching the core warn). cerulion_core warns when
        //   history_size > subscriber_max_buffer_size * 3 / 4
        // (transport/mod.rs — CORRECT and left untouched), so a latched
        // /rosout-class topic (rclpy asks TRANSIENT_LOCAL depth 1000 → clamped
        // to 16) would trip it on EVERY rclpy process. Raise the ceiling to the
        // smallest `c` with `history <= c*3/4` in that integer expression:
        // `c*3/4 >= history` ⇔ `c >= 4*history/3` ⇔ `c = ceil(4*history/3) =
        // (history*4).div_ceil(3)`. Then late-joiners CAN hold the full
        // history AND the core warn stays quiet — the ROS-faithful outcome
        // (a TRANSIENT_LOCAL depth-N request really retains N). `.max` never
        // LOWERS the stock default (a depth-1 /tf_static topic needs no raise:
        // 1 <= default*3/4).
        let mut topic_config = rt.transport.default_topic_config();
        topic_config.history_size = history;
        if history > 0 {
            topic_config.subscriber_max_buffer_size = topic_config
                .subscriber_max_buffer_size
                .max((history * 4).div_ceil(3));
        }
        // A loanable (plain) type's service must be born with borrow
        // headroom for held take loans. The rmw PUBLISHER usually creates the
        // service (create provisions, open tolerates), so the floor
        // rides the CREATE leg here — and only the create leg: it never arms
        // an open requirement, so a pre-existing smaller service still
        // attaches (degraded — the loan budget is then whatever that service
        // was created with).
        // Keyed on the TAKE-side gate — a forged type's
        // subscribers hold samples across the C ABI exactly like a fixed
        // type's, so its service needs the same borrow headroom.
        if bridge.can_loan_take() {
            topic_config.create_borrow_floor = Some(RMW_TAKE_LOAN_BORROW_BUDGET);
        }
        // The publish-side WINDOWED borrow — unbounded
        // primitive-sequence types (Image.data, PointCloud2.data, …) get
        // `rmw_borrow_loaned_message` when the type qualifies AND the heap
        // hook's versioned handshake is Active in this process (the
        // `cerulion ros2` launcher preloads it; absent/skewed/foreign ⇒
        // `None` here and the surface is the plain loan path) AND the
        // topic's slice ceiling can hold the slot geometry (the gate in
        // `windowed_borrow_geometry` — a per-type/env ceiling below
        // `WireHeader + tail_off` would let the slot construction write
        // past the loan). The ceiling is PORT-level (the ask IS the port's
        // value — see `CerulionPublisher::max_slice_len`), so the
        // create-time answer is the port's answer for its whole life. The
        // geometry is cached on the handle so the per-call paths never
        // recompute eligibility.
        let slice_len = slice_len_for(&bridge);
        let windowed_geo = windowed_borrow_geometry(
            &bridge,
            slice_len.get() as usize,
            crate::heaphook::active_hook().is_some(),
        );
        if windowed_geo.is_none()
            && crate::heaphook::active_hook().is_some()
            && bridge.can_borrow_windowed()
        {
            // The TYPE qualifies and the hook is live — only the ceiling
            // refused. Say so: the operator set (or inherited) a ceiling
            // that turns this topic's zero-copy off.
            tracing::warn!(
                topic = %topic,
                type_name = %bridge.qualified_name(),
                slice_ceiling = slice_len.get(),
                "windowed borrow DISABLED for this topic: its slice ceiling \
                 cannot hold the borrow slot geometry (WireHeader + message \
                 struct); publishes use the copy path — raise the per-type \
                 ceiling (CERULION_RMW_SLICE_CEILING) to restore zero-copy"
            );
        }
        // Loanable topics (fixed loans and windowed borrows) hold
        // samples across the C ABI and the windowed publish can take a
        // second, exact-size fallback loan while the borrow loan is still
        // held — raise the port's simultaneous-loan budget above the
        // iceoryx2 default of 2. Port-level: no open-time requirement on
        // any other opener; apparent-bytes-only cost (demand-paged pool).
        if bridge.can_loan() || windowed_geo.is_some() {
            topic_config.publisher_max_loaned_samples = Some(RMW_LOANED_SAMPLES_BUDGET);
        }
        // Degraded case: if the data service ALREADY exists at a lower ceiling
        // (a subscriber created it first at defaults), this create-side config
        // cannot re-provision an already-open service — the core's existing
        // degraded-open behavior governs and a resulting warn is then GENUINE
        // (the create-side-floor precedent: create provisions the
        // ceiling, a later open merely tolerates it).
        let publisher = match rt.transport.create_publisher_with_topic_config(
            &topic,
            slice_len,
            history,
            topic_config,
        ) {
            Ok(p) => p,
            Err(e) => {
                tracing::error!(topic = %topic, error = %e, "publisher creation failed");
                return std::ptr::null_mut();
            }
        };

        // The depth this endpoint really
        // retains, READ BACK from the service the publisher just opened rather
        // than assumed equal to `history`. The two diverge whenever this create
        // ATTACHED to a service someone else made deeper — iceoryx2's
        // `.history_size(N)` open verification is at-least, and the port is
        // sized from the service's static config, so a depth-5 request on a
        // depth-16 topic really retains 16 (MEASURED by a late joiner in
        // `rmw_transient_local_ceiling_test.rs`). Reporting the ask there would
        // understate what late joiners receive — the exact inversion of this
        // decision. On a fresh CREATE the read-back equals `history`, so every
        // decided value is unchanged: TL-1000 ⇒ 16, TL-5 ⇒ 5, VOLATILE ⇒ 0
        // (iceoryx2's own default `publisher_history_size` is 0, and Cerulion
        // only calls `.history_size()` when the request is nonzero).
        let provisioned_depth = publisher.provisioned_history_size();

        // Say LOUDLY, at create time, that the ask was not what the
        // endpoint got — naming BOTH numbers, so a ROS user asking for
        // `depth=1000` sees the clamp instead of having the request echoed
        // back at them by the introspection APIs.
        //
        // The line carries no separate `retained_frames` field. Under
        // retention-depth reporting `provisioned_depth` IS the answer to "how
        // many old frames will a late joiner get?", so a second field naming the
        // same quantity would be two spellings of one value on one line —
        // exactly the kind of surface an operator has to be told to ignore.
        // The line has one shape: requested vs provisioned.
        //
        // SCOPED TO REAL CLAMPS (retention-depth reporting rule). A VOLATILE
        // publisher retains 0 because retention 0 IS WHAT VOLATILE MEANS (the
        // DDS contract: a late joiner gets only post-match samples) — that is
        // the policy the caller chose, not a divergence from their ask, and
        // there is nothing for them to act on. Warning there would put a line
        // on every create of the ordinary ROS path (rclcpp's default QoS is
        // KeepLast(10) + VOLATILE) that says only "you chose VOLATILE". So the
        // gate is `depth_governs_retention`: it fires only where the CALLER'S
        // DEPTH really was clamped — TRANSIENT_LOCAL above the ceiling
        // (depth 1000 ⇒ 16) or below the floor (the degenerate depth 0 ⇒ 1).
        if depth_governs_retention && requested_depth != provisioned_depth {
            tracing::warn!(
                topic = %topic,
                requested_depth,
                provisioned_depth,
                "requested QoS depth differs from the depth Cerulion provisioned — \
                 endpoint info reports the provisioned depth"
            );
        }

        // Fault-injection seam (test builds only — `test-seams`): a panic
        // HERE models any later construction step failing after the transport
        // publisher exists, the exact shape the register-LAST ordering at the
        // end of this function defends (see `register_dynamic_egress`).
        #[cfg(feature = "test-seams")]
        crate::test_seams::maybe_panic_after_publisher_transport_create();

        // Deterministic gid: entity counter + schema hash (never random).
        let entity = runtime::next_entity_id();
        let mut gid = [0u8; 16];
        gid[..8].copy_from_slice(&entity.to_le_bytes());
        gid[8..].copy_from_slice(&bridge.schema_hash().to_le_bytes());

        let type_name = ros_graph_type_name(bridge.qualified_name());
        let can_loan = bridge.can_loan();
        // Parity with the native graph build: arm the producer-side SHM
        // doorbell (rings inside `notify_sent_sample`, which every rmw
        // publish path already calls), so a consumer parked on this topic's
        // line — the event-driven `rmw_wait`'s park tier, or a native
        // monitor-wait loop in the same namespace — wakes the instant the
        // frame is committed. No-op stub off Linux; a failed open degrades
        // to the fd/timer wakes with the warn `enable_doorbell_shared` logs.
        // UNOWNED, never unlinked: a ROS topic is provisioned at TWO
        // publishers (the `/rosout` shape), so an owned bell would let the
        // first publisher to die pull the page from under the survivor — it
        // would keep ringing its old inode while a wait set created
        // afterwards mapped a fresh one and never heard it. The residual is
        // one 64-byte page per topic outliving every publisher on the machine
        // until the next creator; a re-created publisher joins the same
        // page. Pinned by the survivor test in `rmw_wait_event_test`.
        let mut publisher = publisher;
        publisher.enable_doorbell_shared(cerulion_core::doorbell::default_namespace().as_str());
        // The ROS-visible loan gate: a fixed type's plain loan OR the
        // windowed borrow. rclcpp routes `borrow_loaned_message` through
        // the rmw exactly when this is set.
        let can_loan_messages = can_loan || windowed_geo.is_some();
        let data = Box::new(PublisherData {
            topic: topic.clone(),
            type_name,
            bridge,
            // Loan bookkeeping is reserved to the loan budget at CREATE so
            // the publish-side loan path never allocates for an in-budget
            // borrow (the subscription path's discipline).
            // `quarantined_tails`/`orphaned_loans` can outgrow the budget
            // over a pool's slot rotation — the budget reserve is the
            // FLOOR, later growth is amortized and warmup-visible (see
            // `rmw_borrow_zero_alloc_test`).
            inner: std::sync::Mutex::new(PublisherInner {
                publisher,
                pending_loans: Vec::with_capacity(RMW_LOANED_SAMPLES_BUDGET),
                quarantined_tails: Vec::with_capacity(RMW_LOANED_SAMPLES_BUDGET),
                orphaned_loans: Vec::with_capacity(RMW_LOANED_SAMPLES_BUDGET),
                seal_scratch: crate::type_bridge::SealScratch::new(),
            }),
            gid,
            sequence: std::sync::atomic::AtomicU64::new(0),
            malformed_header_rejects: std::sync::Mutex::new(Default::default()),
            schema_hash_rejects: std::sync::Mutex::new(Default::default()),
            windowed_borrow: windowed_geo,
            borrow_degrades: std::sync::Mutex::new(Default::default()),
            wrong_thread_returns: std::sync::Mutex::new(Default::default()),
            borrow_adopted_frames: std::sync::atomic::AtomicU64::new(0),
            borrow_copied_frames: std::sync::atomic::AtomicU64::new(0),
        });

        // topic_name must outlive the publisher and rcl expects the ROS
        // name (with leading slash) — keep the ROS form in a NUL-terminated
        // owned buffer. Computed BEFORE the `Box::into_raw` calls below so
        // that if `leak_ros_name`'s allocation OOM-panics, the unwind drops
        // `data`/`rmw_pub` while they are still owned Boxes — instead of
        // orphaning the already-raw'd boxes (ffi_guard → null return would
        // leak both; Principle #11).
        let topic_name_ptr = leak_ros_name(ros_topic);

        // Zeroed + field stores — see the subscription mirror below for why
        // (cross-distro field-set drift).
        let mut rmw_pub: Box<ffi::rmw_publisher_t> = Box::new(std::mem::zeroed());
        rmw_pub.implementation_identifier = ffi::implementation_identifier_ptr();
        rmw_pub.data = Box::into_raw(data) as *mut c_void;
        rmw_pub.topic_name = topic_name_ptr;
        rmw_pub.options = *publisher_options;
        rmw_pub.can_loan_messages = can_loan_messages;
        let ptr = Box::into_raw(rmw_pub);
        let data_ref = &*((*ptr).data as *const PublisherData);
        // Loud zero-copy reporting: rclcpp silently downgrades when
        // can_loan_messages is false — say which path this topic got.
        tracing::info!(
            topic = %data_ref.topic,
            type_name = %data_ref.type_name,
            zero_copy = can_loan,
            windowed_zero_copy = data_ref.windowed_borrow.is_some(),
            "publisher created (zero_copy ⇒ the loaned pointer IS the SHM slot; \
             windowed_zero_copy ⇒ unbounded fills adopt through the heap hook's \
             borrow window, escapes copy loudly; both false ⇒ flatten + memcpy \
             into an SHM loan, no serialization)"
        );
        // Endpoint record for rmw_get_publishers_info_by_topic:
        // node name/namespace from the owning node, gid + type from the
        // publisher just built, QoS carrying BOTH the requested profile and
        // the depth this create really provisioned (the depth
        // served to ROS is the provisioned one; the other three axes are
        // the request).
        // Process-LOCAL only.
        let node_data = &*((*node).data as *const runtime::NodeData);
        let record = runtime::EndpointRecord {
            node_name: node_data.name.clone(),
            node_namespace: node_data.namespace.clone(),
            type_name: data_ref.type_name.clone(),
            endpoint_gid: data_ref.gid,
            qos: runtime::QosSnapshot {
                reliability: (*qos).reliability,
                durability: (*qos).durability,
                history: (*qos).history,
                requested_depth,
                provisioned_depth,
            },
        };
        // Registry mutation is the last step of HANDLE CONSTRUCTION: if
        // anything above panics (caught by ffi_guard → null return), the
        // registry must not carry a phantom publisher forever.
        // The network-egress registration below is later still, for the
        // same reason.
        {
            let mut graph = rt.graph.write().unwrap_or_else(|e| e.into_inner());
            GraphRegistry::add_endpoint(&mut graph.publishers, &data_ref.topic, record);
        }
        // Register for the rmw_wait event pump (TRANSIENT_LOCAL history
        // to late joiners on idle publishers).
        {
            let mut pubs = rt.publishers.lock().unwrap_or_else(|e| e.into_inner());
            // SAFETY: `(*ptr).data` is the Box we just `into_raw`'d; it is
            // unregistered here before being freed in rmw_destroy_publisher.
            pubs.push(runtime::PublisherPtr::new(
                (*ptr).data as *const PublisherData,
            ));
        }
        runtime::notify_graph_change();

        // Make the topic REMOTELY DEMANDABLE: register it for network egress
        // with the bridge's schema hash (the wire `schema_hash` every frame of
        // this publisher carries — for a variable type that is still the
        // introspection-derived hash). This comes LAST, after every fallible /
        // panic-capable step of handle construction has succeeded (the
        // transport publisher, the boxes, the ROS-name buffer, the registry
        // and pump entries): the registration channel is ADDITIVE with no
        // removal primitive, so a record pushed before a later step panicked
        // (ffi_guard → null) would have been a PHANTOM — a topic the gateway
        // announces and accepts demand for with zero producers, for the life
        // of the process (pinned
        // by `rmw_canonical_names_test`'s injected-failure arm). Registering
        // last means a record exists only for a handle rcl actually received.
        // The transport publisher STILL precedes it — its create-time egress
        // loop-check must run before any egress flag can exist — and the call
        // is panic-CONTAINED, so it can neither null this already-constructed
        // handle nor leave it orphaned.
        register_dynamic_egress(
            &rt.transport,
            &data_ref.topic,
            data_ref.bridge.schema_hash(),
        );
        ptr
    })
}

/// # Safety
/// `publisher` must be a valid publisher from this implementation.
#[no_mangle]
pub unsafe extern "C" fn rmw_destroy_publisher(
    node: *mut ffi::rmw_node_t,
    publisher: *mut ffi::rmw_publisher_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if node.is_null() || publisher.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*publisher).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        // All fallible/registry work happens through a BORROW first; the
        // Box::from_raw + frees are the final infallible block, so a
        // caught panic can never leave the rmw handle dangling.
        {
            let data = &*((*publisher).data as *const PublisherData);
            if let Ok(rt) = runtime::runtime() {
                // UNREGISTER from the event pump FIRST — the pump must
                // never touch a freed PublisherData.
                {
                    let mut pubs = rt.publishers.lock().unwrap_or_else(|e| e.into_inner());
                    pubs.retain(|p| !std::ptr::eq(p.as_ptr(), data as *const PublisherData));
                }
                // The topic's network-egress registration (see
                // `register_dynamic_egress`) is deliberately NOT withdrawn
                // here, because nothing CAN withdraw it: the registration
                // channel is an ADDITIVE per-process set — the writer side has
                // insert + periodic republish and no remove, and the gateway
                // side has `register_runtime_topic` with no inverse and no
                // expiry. So the registration outlives the publisher: this
                // process keeps republishing the topic until it exits, and a
                // gateway that heard it keeps it announced until the gateway
                // restarts. What a remote host then sees is an announced topic
                // with NO live producer (a zero producer count / no data
                // flowing), and a re-created publisher on the same ROS name —
                // the common shape — lands straight back on the standing
                // announce. This is stated in the transport's registration
                // channel docs and pinned by
                // `rmw_canonical_names_test`; a removal primitive added to the
                // channel must be wired here and update both.
                let mut graph = rt.graph.write().unwrap_or_else(|e| e.into_inner());
                GraphRegistry::remove_endpoint(&mut graph.publishers, &data.topic, &data.gid);
            }
            runtime::notify_graph_change();
            // Principle #3: the adopted/copied totals'
            // ONLY window onto ROS code — the rmw C ABI is standardized, so
            // rclcpp/rclpy hold this entity as an opaque pointer and no
            // accessor can reach them (the same argument that earned the
            // decade re-announcement in the shared latch). Say at destroy
            // how the windowed publishes split, so an operator (or a bench
            // harness) can verify the zero-copy path actually engaged
            // rather than silently copying. Gated to windowed publishers:
            // for every other publisher both totals are structurally zero
            // and the line would be per-destroy noise.
            if data.windowed_borrow.is_some() {
                tracing::info!(
                    topic = %data.topic,
                    adopted = data.borrow_adopted_count(),
                    copied = data.borrow_copied_count(),
                    "windowed borrow summary at destroy (adopted = fully \
                     zero-copy publishes; copied = paid a copy on at least \
                     one member; both count only frames the transport \
                     ACCEPTED — a send that failed is in neither total)"
                );
            }
        }
        // A windowed loan whose ARMED window lives on
        // ANOTHER thread must not release its slot at destroy — the TLS
        // window cannot be disarmed from here, and a released slot is
        // recycled by the pool while that window keeps bump-allocating
        // into it (a later allocation on the owner thread would write
        // into a NEW frame's memory: the stale-window use-after-free
        // class). Such slots are deliberately LEAKED (`mem::forget` — a
        // sample that never drops is never reclaimed, and the forgotten
        // sample's refs keep the mapping alive), bounded by the pool's
        // slot count: the bounded-leak-over-corruption direction. Loans
        // whose window THIS thread owns (or that never armed) are
        // disarmed and released normally, `fini` freeing their heap
        // containers. The block yields the leaked-slot count: a nonzero
        // count also decides the fate of the `PublisherData` box below
        // (leaked samples pin the port, so the port must stay).
        let leaked_slots: usize = {
            let data = &*((*publisher).data as *const PublisherData);
            // Destroy MUST run its leak discipline even when an earlier
            // panic POISONED the bookkeeping: the `lock_unpoisoned`
            // refusal shape would skip this block, fall through to the
            // `Box` drop below, and return every held slot to the pool
            // while a borrow window may still be armed — exactly the
            // stale-window use-after-free class this block exists to
            // prevent. `into_inner` here is the documented
            // exception to the `lock_unpoisoned` rule: the guarded data
            // is structurally valid (poison is a logical flag), and the
            // poisoned arm TRUSTS NOTHING logically — every windowed loan
            // and every orphan is leaked (counted, warned), only Fixed
            // loans (which never arm a window) are released.
            let (mut guard, poisoned) = match data.inner.lock() {
                Ok(g) => (g, false),
                Err(e) => (e.into_inner(), true),
            };
            let inner = &mut *guard;
            let me = std::thread::current().id();
            // Disarm this thread's window ONLY on the clean arm. The
            // inference (`owns_here`) reads the loan tables, and on the
            // poisoned arm those are exactly what cannot be trusted: a
            // panic can leave an armed TLS window ABSENT from the tables
            // (mid-borrow, between arm and push) or an entry stale
            // (mid-publish, after the pending removal), so "trust nothing
            // logically" includes the ownership inference — the poisoned
            // arm preserves EVERY armed window untouched, consistent with
            // leaking every windowed slot below. Cost: that thread's
            // later borrows degrade windowless (latched, loud) — a
            // degrade, never corruption.
            if !poisoned {
                let owns_here = inner.orphaned_loans.iter().any(|o| o.thread == me)
                    || inner.pending_loans.iter().any(|p| {
                        matches!(&p.kind,
                            runtime::LoanKind::Windowed(w) if w.window_owner == Some(me))
                    });
                if owns_here {
                    if let Some(api) = crate::heaphook::active_hook() {
                        let _ = (api.disarm_window)();
                    }
                }
            }
            let mut leaked = 0usize;
            for pending in inner.pending_loans.drain(..) {
                match pending.kind {
                    runtime::LoanKind::Windowed(w)
                        if poisoned || w.window_owner.is_some_and(|t| t != me) =>
                    {
                        // No fini either — the leak is WHOLE-LOAN by
                        // construction: the heap containers' headers are
                        // reachable ONLY through the struct in the slot,
                        // which the owner thread's live fill may be
                        // concurrently rewriting (a bump-backed `resize`
                        // rewrites the sequence header, a heap `assign`
                        // the string header), so no partial "free the
                        // heap half" fini exists — reading the headers
                        // races the writer, and freeing a block a
                        // concurrent assign still touches is the UAF
                        // class this branch prevents (on the poisoned
                        // arm, additionally: trust no torn bookkeeping).
                        // The message's HEAP-side containers therefore
                        // leak WITH the slot — bounded to one message's
                        // containers per leaked slot, slots per publisher
                        // by the loan budget; process-life growth needs
                        // repeated create → cross-thread-arm → destroy
                        // incidents, each counted + warned below.
                        std::mem::forget(pending.loan);
                        leaked += 1;
                    }
                    runtime::LoanKind::Windowed(w) => {
                        data.bridge
                            .fini_borrowed_payload(pending.key as *mut c_void, w.payload_len);
                        drop(pending.loan);
                    }
                    runtime::LoanKind::Fixed => drop(pending.loan),
                }
            }
            for orphan in inner.orphaned_loans.drain(..) {
                if poisoned || orphan.thread != me {
                    std::mem::forget(orphan.loan);
                    leaked += 1;
                } else {
                    drop(orphan.loan);
                }
            }
            if leaked > 0 {
                BORROW_DESTROY_LEAKS.fetch_add(leaked as u64, Ordering::Relaxed);
                tracing::warn!(
                    topic = %data.topic,
                    leaked,
                    poisoned,
                    "publisher destroyed while borrow window state cannot be \
                     safely released (another thread's window still armed, or \
                     the bookkeeping was poisoned by a panic) — the slot(s) and \
                     their messages' heap containers are deliberately LEAKED \
                     (never returned to the pool), so a live window can never \
                     bump into recycled memory; the publisher's iceoryx2 ports \
                     (data publisher + event notifier + listener) stay REGISTERED \
                     and hold their slots for the life of this process, and NO \
                     PublisherDisconnected edge is sent — subscribers see this \
                     publisher as alive until the process exits (a port whose \
                     samples outlive it must not be deregistered); \
                     publish or return loans on their borrowing thread before \
                     destroying the publisher"
                );
            }
            leaked
        };
        unleak_ros_name((*publisher).topic_name);
        let data = Box::from_raw((*publisher).data as *mut PublisherData);
        if leaked_slots > 0 {
            // A leaked slot means the iceoryx2 `Publisher` inside
            // this `PublisherData` must NOT be dropped either — the whole box
            // is leaked, so the port stays REGISTERED on the topic.
            //
            // Why (iceoryx2 0.9.1, verified in source): `Publisher::drop`
            // (`src/port/publisher.rs:380-395`) only `release_publisher_handle`s
            // — it DEREGISTERS the port from the service's dynamic config and
            // nothing else. The port's on-disk tag
            // (`<root>/nodes/<node_id>/<prefix><port_id>.port_tag`) is the
            // LAST field of `PublisherSharedState` (`:194`), an `Arc` every
            // `SampleMut` shares, so it is deleted only when the last sample
            // drops — and the samples forgotten above never drop. Dropping the
            // publisher here would therefore leave a deregistered port with a LIVE
            // tag. At process exit the dead-node sweep (`src/node/mod.rs:
            // 668-730`) runs its service-tags pass, which calls
            // `remove_port_tag` ONLY for ports still registered
            // (`src/service/mod.rs:795`); the port-tags pass then meets the
            // orphan tag, `remove_stale_port_resources` succeeds but never
            // deletes the tag itself, and `remove_node` deletes `.details` and
            // `rmdir`s the node directory (`:821-855`) → ENOTEMPTY →
            // `NodeCleanupFailure::InternalError` on EVERY sweep, forever:
            // `cerulion clean` never converges and the `.shm_state`
            // reclamation is skipped for good.
            //
            // With the port still registered, this process's exit is the
            // ordinary "exited with live publishers" shape: the service-tags
            // pass removes the port AND its tag, the node directory empties,
            // and `remove_node` succeeds. The other way round — deleting the
            // tag by path — is wrong in the opposite direction: the port-tags
            // pass is what reclaims a dead port's data segment, so a tagless
            // port strands its segment instead.
            //
            // Cost. Forgetting the `Box` also skips
            // `CerulionPublisher::drop`, and that type owns THREE iceoryx2
            // ports — the data publisher plus the event notifier and listener
            // (`cerulion_core/src/transport/publisher.rs:113-115`) — so all
            // three stay registered for the life of this process: one of the
            // topic's publisher slots AND one notifier + one listener slot on
            // its event service are held, against every process on the machine.
            // `CerulionPublisher::drop` (`publisher.rs:2468`) is also the ONLY
            // sender of `PubSubEvent::PublisherDisconnected`, so no disconnect
            // edge is ever sent — and the transport's liveliness is
            // COUNT-based (`ExternalPublisherProbe::live_publishers`,
            // `transport/mod.rs:1796`, consumed by the runtime's liveliness
            // sweep, `graph/runtime.rs:12668-12684`), so a native subscriber
            // of the topic never sees a `Lost`/`PublisherDisconnected` edge
            // for this publisher until the process exits and the dead-node
            // sweep reclaims it. Sending the disconnect notify before
            // forgetting is deliberately NOT done: the sweep keys on the
            // count, not the event, so it would change nothing there, while
            // any consumer that keys on the event would be told
            // "disconnected" about a port that still counts as attached — an
            // event/count contradiction, not a remedy. Slot cost: the port
            // holds one of the topic's publisher slots for the life of this
            // process. rmw topics are provisioned at iceoryx2's
            // default `max_publishers = 2` (`default_topic_config` leaves
            // `max_publishers: None` and declares
            // `PublisherProvisioning::External`, so no single-writer pre-check
            // fires), so a re-create on the same ROS topic still succeeds
            // ONLY while the topic's other slot is free — it takes that slot —
            // and the next is refused (with any other live publisher already
            // on the topic, the `/rosout` shape, the FIRST re-create is the
            // refused one):
            // `create_publisher_with_topic_config` fails with
            // `ExceedsMaxSupportedPublishers` ("all 2 of the topic's publisher
            // slots are attached"), `rmw_create_publisher` logs `publisher
            // creation failed` and returns NULL. Pinned in-process by
            // `rmw_borrow_publish_test` (port count + the 2-slot consequence)
            // and cross-process by `rmw_leak_at_destroy_registry_test` (a
            // dead node with a leaked port is swept clean).
            std::mem::forget(data);
        } else {
            drop(data);
        }
        drop(Box::from_raw(publisher));
        RMW_RET_OK
    })
}

/// # Safety
/// rmw ABI contract: `ros_message` points at a valid message of the
/// publisher's type.
#[no_mangle]
pub unsafe extern "C" fn rmw_publish(
    publisher: *const ffi::rmw_publisher_t,
    ros_message: *const c_void,
    _allocation: *mut ffi::rmw_publisher_allocation_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if publisher.is_null() || ros_message.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*publisher).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        let data = &*((*publisher).data as *const PublisherData);
        let now = match runtime::runtime() {
            Ok(rt) => rt.transport.clock().now_ns(),
            Err(_) => 0,
        };
        // Sequence minted AND sent under the same lock — two concurrent
        // publishes must not put seq N+1 on the wire before seq N
        // (replay-relevant observability).
        // A poisoned lock means wedged: never re-enter torn iceoryx2 state.
        let Some(mut inner) = runtime::lock_unpoisoned(&data.inner) else {
            tracing::error!("entity wedged by an earlier panic; failing call");
            return RMW_RET_ERROR;
        };
        if inner.publisher.has_history() {
            // TRANSIENT_LOCAL: keep the owned-Vec path — the history
            // ring needs the frame bytes AFTER the send, so the loan
            // fast path would pay the copy back anyway. Latched topics
            // (robot_description, /tf_static) are small + rare; sensor
            // topics are VOLATILE and take the loan path below.
            //
            // Flatten BEFORE minting the sequence — an encode error must
            // not burn a number (replay tooling reads gaps as dropped
            // frames). Flatten with a placeholder, then stamp the real
            // value: the header is fixed-offset, so re-stamping is two
            // field writes.
            let mut frame = match data.bridge.flatten(ros_message, 0, now) {
                Ok(f) => f,
                Err(e) => {
                    tracing::error!(topic = %data.topic, error = %e, "flatten failed");
                    return RMW_RET_ERROR;
                }
            };
            let seq = data.sequence.fetch_add(1, Ordering::Relaxed) as u32;
            match WireHeader::read_from_buf(&frame) {
                Some(mut header) => {
                    header.sequence = seq;
                    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
                }
                None => {
                    // Unreachable: `frame` is the header we just flattened, so
                    // it is >= WireHeader::SIZE. But never SHIP a placeholder
                    // seq=0 frame on a None — replay tooling reads seq=0 as a
                    // burned/duplicate. Drop unsent + loud rather than silently
                    // corrupt the sequence stream.
                    tracing::error!(topic = %data.topic, "header re-read failed; dropping frame unsent");
                    return RMW_RET_ERROR;
                }
            }
            match inner.publisher.publish_raw(&frame) {
                Ok(_recipients) => {
                    // Native history: TRANSIENT_LOCAL
                    // late-joiner retention is native + zero-copy — the
                    // `publish_raw` above retained this frame in the iceoryx2
                    // publisher port's history queue (sized at creation by
                    // `history`). No Cerulion-side ring to push to. Native
                    // delivery is TARGETED to the new connection only (correct
                    // DDS latch semantics — no broadcast re-publish to existing
                    // subscribers, so there is no cross-publisher
                    // sequence-collision dedup problem).
                    //
                    // Wake event-driven subscribers + drain SubscriberConnected
                    // so a late joiner gets the retained history via
                    // `update_connections` (publish_raw itself doesn't notify).
                    inner.publisher.check_subscriber_events();
                    if let Err(e) = inner.publisher.notify_sent_sample() {
                        tracing::debug!(topic = %data.topic, error = ?e, "notify failed");
                    }
                    RMW_RET_OK
                }
                Err(e) => {
                    tracing::error!(topic = %data.topic, error = %e, "publish failed");
                    RMW_RET_ERROR
                }
            }
        } else {
            // VOLATILE hot path (flatten-into-loan): size
            // pre-pass → exact-size UNINIT loan → bounds-checked
            // flatten straight into the SHM slot → send. ONE
            // full-payload copy (scattered C message → SHM), ZERO
            // payload-sized heap allocations — vs. a
            // flatten-to-Vec + publish_raw-memcpy (2 copies + 1 alloc).
            let frame_size = match data.bridge.frame_size(ros_message) {
                Ok(n) => n,
                Err(e) => {
                    tracing::error!(topic = %data.topic, error = %e, "frame sizing failed");
                    return RMW_RET_ERROR;
                }
            };
            let mut loan = match inner.publisher.loan_raw_uninit(frame_size) {
                Ok(l) => l,
                Err(e) => {
                    tracing::error!(topic = %data.topic, error = %e, "SHM loan failed");
                    return RMW_RET_ERROR;
                }
            };
            // Placeholder sequence (0) for the same no-burned-numbers
            // reason as the history path; re-stamped below on success.
            let written =
                match data
                    .bridge
                    .flatten_into_uninit(ros_message, 0, now, loan.bytes_uninit_mut())
                {
                    Ok(n) => n,
                    Err(e) => {
                        // Drop the loan — a partially-filled slot must NEVER
                        // be sent (torn/uninitialized bytes; Principle #7).
                        drop(loan);
                        tracing::error!(topic = %data.topic, error = %e, "flatten failed");
                        return RMW_RET_ERROR;
                    }
                };
            // Belt-and-suspenders for the unsafe promotion below: the
            // cursor already enforces Ok ⇒ written == slot len, but
            // re-checking the integer HERE makes the safety argument
            // local and checked at run time instead of cross-module
            // trust. One compare on a cold branch that is provably
            // never taken.
            if written != frame_size {
                drop(loan);
                tracing::error!(
                    topic = %data.topic,
                    written,
                    frame_size,
                    "encode length mismatch; dropping loan unsent"
                );
                return RMW_RET_ERROR;
            }
            // SAFETY — proof chain that every byte of the slot is
            // initialized (iceoryx2's `assume_init` contract):
            // 1. the slot is exactly `frame_size` bytes
            //    (`loan_raw_uninit(frame_size)` is exact-size);
            // 2. `FrameCursor::new` refuses buffers smaller than the
            //    head and pre-zeroes `[0, head_len)` before any field
            //    op runs;
            // 3. every cursor write is capacity-checked (`Err` on
            //    overflow — never out-of-bounds) and advances a
            //    CONTIGUOUS watermark: alignment gaps and repr(C)
            //    padding are explicitly zero-filled, so no byte below
            //    the watermark is ever skipped;
            // 4. `Ok` requires the watermark to end EXACTLY at the
            //    slot length (`require_full`), re-checked by the
            //    integer compare above.
            // Proven by the dual-poison byte-identity tests
            // (bridge_test/cpp_bridge_test) and the 1 MiB e2e.
            let mut loan = loan.assume_init();
            let seq = data.sequence.fetch_add(1, Ordering::Relaxed) as u32;
            match WireHeader::read_from_buf(loan.bytes_mut()) {
                Some(mut header) => {
                    header.sequence = seq;
                    header.write_to_buf(&mut loan.bytes_mut()[..WireHeader::SIZE]);
                }
                None => {
                    // Same loud-over-silent guard as the TRANSIENT_LOCAL path:
                    // the slot is >= head_len >= WireHeader::SIZE and
                    // flatten_into_uninit just wrote a full header, so this is
                    // unreachable — but a None must NOT ship a placeholder
                    // seq=0 frame. Drop the loan unsent (releases the slot).
                    drop(loan);
                    tracing::error!(topic = %data.topic, "header re-read failed after flatten; dropping loan unsent");
                    return RMW_RET_ERROR;
                }
            }
            match inner.publisher.send_raw_loan(loan) {
                Ok(_recipients) => {
                    inner.publisher.check_subscriber_events();
                    if let Err(e) = inner.publisher.notify_sent_sample() {
                        tracing::debug!(topic = %data.topic, error = ?e, "notify failed");
                    }
                    RMW_RET_OK
                }
                Err(e) => {
                    tracing::error!(topic = %data.topic, error = %e, "publish failed");
                    RMW_RET_ERROR
                }
            }
        }
    })
}

/// Zero-copy loan: hand rclcpp a pointer DIRECTLY into a loaned SHM
/// slot (post-WireHeader). Enabled only for verified fixed-layout types
/// (`can_loan` — C struct ≡ wire fixed section, byte-for-byte).
///
/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_borrow_loaned_message(
    publisher: *const ffi::rmw_publisher_t,
    type_support: *const ffi::rosidl_message_type_support_t,
    ros_message: *mut *mut c_void,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        // rmw.h (Jazzy): INVALID_ARGUMENT if `publisher`, `type_support` or
        // `ros_message` is NULL.
        if publisher.is_null() || type_support.is_null() || ros_message.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*publisher).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        let data = &*((*publisher).data as *const PublisherData);
        // rmw.h (Jazzy, `rmw_borrow_loaned_message`): "RMW_RET_INVALID_ARGUMENT
        // if `*ros_message` is not NULL (to prevent leaks)". A caller reusing
        // an out-slot that still holds an outstanding loan would overwrite
        // the only handle that can publish or return the prior loan, pinning
        // its SHM slot for the publisher's life. Refused BEFORE any loan is
        // taken, so no slot is consumed and `*ros_message` stays untouched.
        // Mirrors the take-side guard in `take_loaned_impl`.
        if !(*ros_message).is_null() {
            tracing::error!(
                topic = %data.topic,
                ptr = *ros_message as usize,
                "loaned borrow refused: *ros_message is not NULL (rmw.h: INVALID_ARGUMENT \
                 to prevent leaks) — publish or return the outstanding loan first, then \
                 pass a NULL slot"
            );
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !data.bridge.can_loan() {
            // An unbounded type with an Active heap hook
            // borrows through the WINDOW.
            if let Some(geo) = data.windowed_borrow {
                return borrow_windowed(data, &geo, ros_message);
            }
            // rclcpp falls back to heap-message + rmw_publish — permitted
            // and loud (the zero_copy=false log at creation).
            return RMW_RET_UNSUPPORTED;
        }
        // Safety invariant for the `init_loaned_payload` call
        // below: `can_loan` (checked above) is computed as
        // `layout.is_fixed() && fixed_size == c_size` with per-field
        // C==wire offset equality (type_bridge.rs) — a recursively-FIXED,
        // all-primitive layout with no strings/sequences/variable nesting
        // anywhere. That is what makes running the rosidl `init_function`
        // on SHM sound: with no pointer-bearing members it writes
        // primitive defaults in place and allocates NOTHING. A future
        // `can_loan` widening that admits a pointer-bearing type would
        // placement-new heap-backed containers INTO SHARED MEMORY (their
        // heap pointers meaningless in every other process) — this
        // tripwire catches it before that ships.
        debug_assert!(
            data.bridge.layout().is_fixed(),
            "can_loan admitted a variable layout — init_function on SHM would embed heap pointers"
        );
        let frame_len = WireHeader::SIZE + data.bridge.layout().fixed_size;
        // ≤ 8 by the codegen static assert (see `WireLayout::fixed_align`); the
        // `.max(1)` is pure defense against a zero from a degenerate layout.
        let fixed_align = data.bridge.layout().fixed_align.max(1);
        // Loan + registration under ONE lock — the loan's sample shares the
        // publisher's Rc-backed iceoryx2 state (the confinement
        // invariant: all !Send state behind a single mutex).
        // A poisoned lock means wedged: never re-enter torn iceoryx2 state.
        let Some(mut inner) = runtime::lock_unpoisoned(&data.inner) else {
            tracing::error!("entity wedged by an earlier panic; failing call");
            return RMW_RET_ERROR;
        };
        // UNINIT loan — a whole-slot zero-fill would be both
        // wrong and wasted. rclcpp's `LoanedMessage` never placement-news
        // `MessageT` on the loaned branch (jazzy loaned_message.hpp:
        // `static_cast` only; only the heap fallback constructs) and its
        // dtor never runs `~MessageT`, so the slot state IS the message's
        // initial state — and zeros are NOT rosidl defaults (a Quaternion
        // declares `float64 w 1`). Zero ONLY the 32-byte header region
        // (rewritten at publish; deterministic meanwhile), then run the
        // typesupport's `init_function` on the payload — the loaned
        // message's REAL construction. repr(C) padding stays unwritten
        // until the publish path zeroes the layout's pad ranges.
        let mut loan = match inner.publisher.loan_raw_uninit(frame_len) {
            Ok(l) => l,
            Err(e) => {
                tracing::error!(topic = %data.topic, error = %e, "SHM loan failed");
                return RMW_RET_BAD_ALLOC;
            }
        };
        let bytes = loan.bytes_uninit_mut();
        bytes[..WireHeader::SIZE].fill(std::mem::MaybeUninit::new(0));
        let payload_ptr = bytes.as_mut_ptr().add(WireHeader::SIZE).cast::<u8>();
        // Alignment gate (fail closed — never hand out, and never run a
        // typed `init_function` on, a misaligned struct pointer). Expected
        // to ALWAYS hold on iceoryx2 0.9.1: the sample header is 40 B @
        // align 8 (measured — see `IOX2_SAMPLE_HEADER_BYTES` in
        // cerulion_core) and a `[u8]` payload has align 1, so the payload
        // starts at chunk+40 of an 8-aligned chunk ⇒ 8-aligned; +32
        // (WireHeader) keeps it, and `fixed_align <= 8` by the codegen
        // static assert. But that is DE FACTO, not an iceoryx2 API
        // guarantee, and the service builder's `.payload_alignment()`
        // override is deliberately NOT used (it would change the slot
        // layout the `iceoryx2_slot_bytes` closed form documents
        // as exact for exactly-this builder shape) — so THIS check is the
        // guard: an iceoryx2 bump that moves the payload offset fails
        // loudly here instead of handing the typesupport's init, and then
        // rclcpp, UB. Mirrors the take-side gate (`take_loaned_impl`).
        if !(payload_ptr as usize).is_multiple_of(fixed_align) {
            // Release the slot unused — nothing was initialized in it.
            drop(loan);
            tracing::error!(
                topic = %data.topic,
                ptr = payload_ptr as usize,
                fixed_align,
                "loaned borrow: SHM payload pointer is misaligned for the message \
                 struct — releasing the slot unused (the iceoryx2 payload offset \
                 moved?)"
            );
            return RMW_RET_ERROR;
        }
        data.bridge.init_loaned_payload(payload_ptr as *mut c_void);
        inner.pending_loans.push(runtime::PendingLoan {
            key: payload_ptr as usize,
            loan,
            kind: runtime::LoanKind::Fixed,
        });
        *ros_message = payload_ptr as *mut c_void;
        RMW_RET_OK
    })
}

/// May this publisher serve the WINDOWED borrow? The
/// TYPE half (`AnyBridge::can_borrow_windowed`), the PROCESS half (the
/// heap hook handshake), and the CEILING half: the topic's slice ceiling
/// must hold at least `WireHeader + tail_off` — the region the borrow
/// UNCONDITIONALLY writes (the header zero + the typesupport's
/// `init_function` over the struct). A per-type / `CERULION_RMW_SLICE_CEILING`
/// ceiling below that would let the slot construction write past the
/// loan into neighboring pool memory, so such a topic keeps the copy
/// path (loudly, at create). A ceiling that fits the geometry but leaves
/// a small/zero fill TAIL is deliberately allowed: a zero-capacity
/// window is sound (every fill escapes ⇒ copies), just never zero-copy.
///
/// Pure over its three inputs so the boundary is oracle-testable; the
/// create path is the only production caller.
pub fn windowed_borrow_geometry(
    bridge: &AnyBridge,
    slice_ceiling: usize,
    hook_active: bool,
) -> Option<BorrowSlotGeometry> {
    if !hook_active {
        return None;
    }
    bridge
        .borrow_geometry()
        .filter(|geo| slice_ceiling >= WireHeader::SIZE + geo.tail_off)
}

/// The WINDOWED borrow — loan `WireHeader + struct +
/// fill tail`, construct the message in the slot, arm the heap
/// hook's borrow window over the tail on the calling thread, and hand
/// rclcpp the struct pointer. Every hook refusal degrades to a WINDOWLESS
/// borrow (the fill goes to the heap; the publish copies it into the same
/// loan) — never a failed borrow, never silence (latched).
///
/// # Safety
/// Called from `rmw_borrow_loaned_message` under its argument checks;
/// `ros_message` is a valid, NULL-holding out-slot.
unsafe fn borrow_windowed(
    data: &PublisherData,
    geo: &BorrowSlotGeometry,
    ros_message: *mut *mut c_void,
) -> rmw_ret_t {
    // `windowed_borrow` is set only when the handshake was Active, and the
    // preload set cannot change after load — defensive downgrade only.
    let Some(hook) = crate::heaphook::active_hook() else {
        return RMW_RET_UNSUPPORTED;
    };
    // Loan + all window bookkeeping under the one publisher lock (the
    // confinement invariant). A poisoned lock means wedged.
    let Some(mut inner) = runtime::lock_unpoisoned(&data.inner) else {
        tracing::error!("entity wedged by an earlier panic; failing call");
        return RMW_RET_ERROR;
    };
    let inner = &mut *inner;
    let me = std::thread::current().id();

    // (a) A window THIS thread orphaned earlier (a loan finished on the
    //     wrong thread) is disarmed now, and its held slots are released —
    //     their quarantine extents persist until slot reuse, exactly like a
    //     published slot's. This sweep runs BEFORE the loan below: an
    //     orphan pins a slot against the publisher's loan budget, so with
    //     the budget otherwise exhausted the self-heal is exactly what
    //     makes the next loan possible — below the loan it could never run
    //     when it was needed (a budget-exhausted borrow would fail
    //     before the heal).
    if inner.orphaned_loans.iter().any(|o| o.thread == me) {
        // rc is the stale window's latched escape (or NOT_ARMED if the
        // thread's TLS was recycled) — nothing to act on here.
        let _ = (hook.disarm_window)();
        let mut i = 0;
        while i < inner.orphaned_loans.len() {
            if inner.orphaned_loans[i].thread == me {
                let o = inner.orphaned_loans.swap_remove(i);
                inner.quarantined_tails.push(o.tail_base);
                drop(o.loan);
            } else {
                i += 1;
            }
        }
        // Close the wrong-thread-return regime: the held slot(s) this
        // sweep just released are what those returns latched about.
        report_wrong_thread_return_healed(&data.wrong_thread_returns, &data.topic);
    }

    // Slot size: head + struct/remnant + the fill tail, clamped to the
    // topic's slice ceiling (the loan itself would refuse past it).
    let ceiling = inner.publisher.max_slice_len().get() as usize;
    let frame_len = (WireHeader::SIZE + geo.tail_off + RMW_BORROW_TAIL_BYTES).min(ceiling);
    let mut loan = match inner.publisher.loan_raw_uninit(frame_len) {
        Ok(l) => l,
        Err(e) => {
            tracing::error!(topic = %data.topic, error = %e, "SHM loan failed");
            return RMW_RET_BAD_ALLOC;
        }
    };
    // Belt for the create-time ceiling gate (`windowed_borrow_geometry`):
    // everything below `WireHeader + tail_off` is written unconditionally
    // (header zero + the typesupport init over the struct), so a loan that
    // cannot hold it must NEVER be constructed in. Unreachable — the
    // ceiling is port-level and fixed at create — but an out-of-bounds
    // write is the one failure this path may not risk on a drifted
    // invariant.
    if frame_len < WireHeader::SIZE + geo.tail_off {
        drop(loan);
        tracing::error!(
            topic = %data.topic,
            frame_len,
            tail_off = geo.tail_off,
            "windowed borrow: loan smaller than the slot geometry (ceiling \
             drifted below the create-time gate?) — refusing to construct"
        );
        return RMW_RET_ERROR;
    }
    let bytes = loan.bytes_uninit_mut();
    // Deterministic header region until publish rewrites it.
    bytes[..WireHeader::SIZE].fill(std::mem::MaybeUninit::new(0));
    let payload_ptr = bytes.as_mut_ptr().add(WireHeader::SIZE).cast::<u8>();
    // Alignment gate (fail closed — mirror of the fixed-type borrow's; a
    // rosidl/std container struct member needs at most align 8 on LP64).
    if !(payload_ptr as usize).is_multiple_of(8) {
        drop(loan);
        tracing::error!(
            topic = %data.topic,
            ptr = payload_ptr as usize,
            "windowed borrow: SHM payload pointer is misaligned for the message \
             struct — releasing the slot unused (the iceoryx2 payload offset moved?)"
        );
        return RMW_RET_ERROR;
    }
    let payload_len = frame_len - WireHeader::SIZE;
    let tail_base = payload_ptr as usize + geo.tail_off;
    let tail_limit = payload_ptr as usize + payload_len;

    // (b) Retire-on-reuse: the transport handed this SLOT out again, which
    //     is the only reclamation signal an rmw can observe — the previous
    //     window's quarantine extent is now reclaimable (nothing can hold
    //     a pointer into a slot the pool re-issued). `retire_slot` ENDS
    //     this rmw's coverage obligation for the previous extent, and
    //     `arm_window` below RE-ESTABLISHES coverage for the fresh one —
    //     correct under BOTH hook dispositions (retire
    //     deletes the quarantine entry, or tombstones it
    //     with revive), which is exactly why the rc is ignored on purpose
    //     and nothing here reads hook quarantine state back.
    if let Some(pos) = inner.quarantined_tails.iter().position(|&t| t == tail_base) {
        inner.quarantined_tails.swap_remove(pos);
        let _ = (hook.retire_slot)(tail_base as *mut c_void);
    }

    // Construct the message in the slot (the rule: never hand out
    // a zeroed never-constructed message), THEN arm — init's own small
    // allocations belong on the heap where `fini` frees them, and the
    // window stays purely the caller's fill.
    data.bridge.init_loaned_payload(payload_ptr as *mut c_void);
    let arm_rc = (hook.arm_window)(tail_base as *mut c_void, tail_limit as *mut c_void);
    let window_owner = if arm_rc == crate::heaphook::RC_OK {
        Some(me)
    } else {
        // Nested arm (a second outstanding borrow on this thread) or a
        // torn-down thread: proceed WINDOWLESS — the fill lands on the
        // heap and the publish copies it into this same loan.
        report_borrow_degraded(
            &data.borrow_degrades,
            &data.topic,
            BorrowDegrade::WindowUnavailable,
            0,
            0,
        );
        None
    };
    inner.pending_loans.push(runtime::PendingLoan {
        key: payload_ptr as usize,
        loan,
        kind: runtime::LoanKind::Windowed(runtime::WindowedLoan {
            tail_base,
            payload_len,
            window_owner,
        }),
    });
    *ros_message = payload_ptr as *mut c_void;
    RMW_RET_OK
}

/// Finalize a windowed loan — recover the window extent
/// (cursor bisection), disarm, SEAL the filled struct in place
/// (adopt/copy), and send; every refusal falls back to the exact-size
/// copy loan (the always-sound spine). Runs under the publisher lock.
///
/// # Safety
/// `key` is the pending loan's payload pointer; `loan`/`w` are its held
/// sample and window bookkeeping; the caller validated the handle.
unsafe fn publish_windowed_loan(
    data: &PublisherData,
    inner: &mut runtime::PublisherInner,
    mut loan: cerulion_core::transport::publisher::RawShmLoanUninit,
    w: runtime::WindowedLoan,
    key: usize,
    now: u64,
) -> rmw_ret_t {
    let geo = data
        .windowed_borrow
        .expect("a windowed loan exists only on a windowed-borrow publisher");
    let hook = crate::heaphook::active_hook();
    let me = std::thread::current().id();

    // Wrong thread: the borrow thread's window cannot be disarmed from
    // here, and sealing under a still-armed window is unsound in BOTH
    // directions (its future bumps could land in this slot; our copy
    // placements could collide with them). Copy the frame out, release the
    // struct's heap containers, and HOLD the slot until the owning thread
    // borrows again (see `PublisherInner::orphaned_loans`).
    if let Some(owner) = w.window_owner {
        if owner != me {
            report_borrow_degraded(
                &data.borrow_degrades,
                &data.topic,
                BorrowDegrade::WrongThread,
                0,
                0,
            );
            let ret = publish_copy_of_slot_struct(data, inner, key as *const c_void, now);
            // The destroy-time summary calls
            // these totals PUBLISHES, so a frame the transport refused is
            // neither an adopted nor a copied publish. Count AFTER the send
            // reports success. The degrade LATCH above is deliberately
            // unconditional — it answers "did the borrow path degrade?",
            // which is true whatever the send then did.
            if ret == RMW_RET_OK {
                data.borrow_copied_frames.fetch_add(1, Ordering::Relaxed);
            }
            data.bridge
                .fini_borrowed_payload(key as *mut c_void, w.payload_len);
            inner.orphaned_loans.push(runtime::OrphanedLoan {
                thread: owner,
                tail_base: w.tail_base,
                loan,
            });
            return ret;
        }
    }

    // Recover the window extent while it is still armed (the range test
    // answers only then), then DISARM — the seal runs disarmed, so its own
    // allocations can never bump into a live window.
    let window = match (w.window_owner, hook) {
        (Some(_), Some(api)) => {
            let cursor =
                crate::heaphook::bisect_window_cursor(&api, w.tail_base, key + w.payload_len);
            // The latched escape code is diagnostics; adoption is decided
            // by the exact address test per member.
            let _escape = (api.disarm_window)();
            cursor.map(|c| WindowExtent {
                base: w.tail_base,
                cursor: c,
            })
        }
        _ => None,
    };
    let armed = w.window_owner.is_some();

    match data.bridge.seal_borrowed_frame(
        key as *mut u8,
        w.payload_len,
        &geo,
        window,
        RMW_BORROW_MAX_GAP_BYTES,
        &mut inner.seal_scratch,
    ) {
        Ok(seal) => {
            let seq = data.sequence.fetch_add(1, Ordering::Relaxed) as u32;
            let header = WireHeader {
                schema_hash: data.bridge.schema_hash(),
                total_size: (WireHeader::SIZE + seal.total_payload) as u32,
                offset_table_offset: data.bridge.layout().fixed_size as u32,
                offset_table_count: data.bridge.layout().variable_fields.len() as u32,
                sequence: seq,
                timestamp_ns: now,
            };
            {
                let bytes = loan.bytes_uninit_mut();
                let mut head = [0u8; WireHeader::SIZE];
                header.write_to_buf(&mut head);
                for (i, b) in head.iter().enumerate() {
                    bytes[i] = std::mem::MaybeUninit::new(*b);
                }
            }
            // SAFETY (`assume_init_shm_defined`'s contract): the header was
            // just written; the seal wrote the wire head + every copied
            // range and left every offset-table-referenced adopted range
            // where the fill placed it; the remainder is documented
            // gap/slack over OS-defined SHM bytes, and consumers slice on
            // `total_size`.
            let sealed = loan.assume_init_shm_defined();
            if armed {
                // The window's quarantine extent outlives the frame until
                // the pool re-issues this slot (retire-on-reuse in
                // `borrow_windowed`).
                inner.quarantined_tails.push(w.tail_base);
            }
            // The seal's verdict is decided here and REPORTED here (the
            // latch describes the seal, which happened regardless of the
            // send), but the COUNTER is withheld until the frame is on the
            // wire — see the WrongThread site above. `adopted` is the
            // number the bench harness cites as proof the zero-copy path
            // engaged; a send that returned Err never reached a subscriber
            // and must not appear in it.
            let adopted_frame = if seal.escaped > 0 {
                report_borrow_degraded(
                    &data.borrow_degrades,
                    &data.topic,
                    BorrowDegrade::Escaped,
                    seal.escaped,
                    seal.adopted,
                );
                false
            } else if window.is_some() {
                report_borrow_adopted(&data.borrow_degrades, &data.topic);
                true
            } else {
                // Windowless borrow: the degrade was latched at borrow time.
                false
            };
            match inner.publisher.send_raw_loan(sealed) {
                Ok(_recipients) => {
                    if adopted_frame {
                        data.borrow_adopted_frames.fetch_add(1, Ordering::Relaxed);
                    } else {
                        data.borrow_copied_frames.fetch_add(1, Ordering::Relaxed);
                    }
                    inner.publisher.check_subscriber_events();
                    if let Err(e) = inner.publisher.notify_sent_sample() {
                        tracing::debug!(topic = %data.topic, error = ?e, "notify failed");
                    }
                    RMW_RET_OK
                }
                Err(e) => {
                    tracing::error!(topic = %data.topic, error = %e, "windowed publish failed");
                    RMW_RET_ERROR
                }
            }
        }
        Err(refusal) => {
            // Pre-mutation refusal: the struct is INTACT — the copy loan
            // publishes the same bytes a plain copying publish would have.
            let kind = match &refusal {
                SealRefusal::Overflow { .. } => Some(BorrowDegrade::Overflow),
                SealRefusal::ExcessiveGaps { .. } => Some(BorrowDegrade::GapExcess),
                SealRefusal::Encode(_) => None,
            };
            let ret = if let Some(kind) = kind {
                report_borrow_degraded(&data.borrow_degrades, &data.topic, kind, 0, 0);
                let ret = publish_copy_of_slot_struct(data, inner, key as *const c_void, now);
                if ret == RMW_RET_OK {
                    data.borrow_copied_frames.fetch_add(1, Ordering::Relaxed);
                }
                ret
            } else {
                // A corrupt container header — the same failure class (and
                // outcome) as a flatten failure on the plain publish path.
                tracing::error!(
                    topic = %data.topic,
                    error = %refusal,
                    "windowed publish: message could not be encoded; dropping frame unsent"
                );
                RMW_RET_ERROR
            };
            data.bridge
                .fini_borrowed_payload(key as *mut c_void, w.payload_len);
            drop(loan);
            if armed {
                inner.quarantined_tails.push(w.tail_base);
            }
            ret
        }
    }
}

/// The exact-size COPY publish of a slot-resident (or any readable)
/// message struct — the windowed publish's always-sound fallback, byte-
/// for-byte the plain `rmw_publish` VOLATILE hot path (size pre-pass →
/// exact uninit loan → bounds-checked flatten → re-stamp → send).
///
/// # Safety
/// `msg` must point at a valid, initialized message of `data`'s bridged
/// type that stays readable for the duration of the call.
unsafe fn publish_copy_of_slot_struct(
    data: &PublisherData,
    inner: &mut runtime::PublisherInner,
    msg: *const c_void,
    now: u64,
) -> rmw_ret_t {
    let frame_size = match data.bridge.frame_size(msg) {
        Ok(n) => n,
        Err(e) => {
            tracing::error!(topic = %data.topic, error = %e, "frame sizing failed");
            return RMW_RET_ERROR;
        }
    };
    let mut copy_loan = match inner.publisher.loan_raw_uninit(frame_size) {
        Ok(l) => l,
        Err(e) => {
            tracing::error!(topic = %data.topic, error = %e, "SHM loan failed");
            return RMW_RET_ERROR;
        }
    };
    let written = match data
        .bridge
        .flatten_into_uninit(msg, 0, now, copy_loan.bytes_uninit_mut())
    {
        Ok(n) => n,
        Err(e) => {
            drop(copy_loan);
            tracing::error!(topic = %data.topic, error = %e, "flatten failed");
            return RMW_RET_ERROR;
        }
    };
    if written != frame_size {
        drop(copy_loan);
        tracing::error!(
            topic = %data.topic,
            written,
            frame_size,
            "encode length mismatch; dropping loan unsent"
        );
        return RMW_RET_ERROR;
    }
    // SAFETY: the cursor proof chain — Ok(n) with n == frame_size means
    // every byte initialized (see the rmw_publish call site's chain).
    let mut copy_loan = copy_loan.assume_init();
    let seq = data.sequence.fetch_add(1, Ordering::Relaxed) as u32;
    match WireHeader::read_from_buf(copy_loan.bytes_mut()) {
        Some(mut header) => {
            header.sequence = seq;
            header.write_to_buf(&mut copy_loan.bytes_mut()[..WireHeader::SIZE]);
        }
        None => {
            drop(copy_loan);
            tracing::error!(topic = %data.topic, "header re-read failed after flatten; dropping loan unsent");
            return RMW_RET_ERROR;
        }
    }
    match inner.publisher.send_raw_loan(copy_loan) {
        Ok(_recipients) => {
            inner.publisher.check_subscriber_events();
            if let Err(e) = inner.publisher.notify_sent_sample() {
                tracing::debug!(topic = %data.topic, error = ?e, "notify failed");
            }
            RMW_RET_OK
        }
        Err(e) => {
            tracing::error!(topic = %data.topic, error = %e, "publish failed");
            RMW_RET_ERROR
        }
    }
}

/// Finalize a zero-copy loan: write the WireHeader in place and send
/// the SAME SHM slot rclcpp just wrote the message into. No copies.
///
/// # Safety
/// `ros_message` must be a loan from `rmw_borrow_loaned_message` on
/// this publisher.
#[no_mangle]
pub unsafe extern "C" fn rmw_publish_loaned_message(
    publisher: *const ffi::rmw_publisher_t,
    ros_message: *mut c_void,
    _allocation: *mut ffi::rmw_publisher_allocation_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if publisher.is_null() || ros_message.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*publisher).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        let data = &*((*publisher).data as *const PublisherData);
        let key = ros_message as usize;
        let now = match runtime::runtime() {
            Ok(rt) => rt.transport.clock().now_ns(),
            Err(_) => 0,
        };
        // Loan extraction + sequence + send all under the one lock.
        // A poisoned lock means wedged: never re-enter torn iceoryx2 state.
        let Some(mut inner) = runtime::lock_unpoisoned(&data.inner) else {
            tracing::error!("entity wedged by an earlier panic; failing call");
            return RMW_RET_ERROR;
        };
        let Some(pos) = inner.pending_loans.iter().position(|p| p.key == key) else {
            tracing::error!(topic = %data.topic, "publish of unknown loaned message");
            return RMW_RET_INVALID_ARGUMENT;
        };
        let pending = inner.pending_loans.swap_remove(pos);
        let mut loan = match pending.kind {
            runtime::LoanKind::Fixed => pending.loan,
            runtime::LoanKind::Windowed(w) => {
                // Seal the filled struct in place (or fall
                // back to the exact-size copy loan) — see the helper.
                return publish_windowed_loan(data, &mut inner, pending.loan, w, key, now);
            }
        };

        // Zero the layout's repr(C) padding ranges before send.
        // The typesupport `init_function` writes FIELDS, not padding, and
        // the caller's writes can deposit process-memory garbage there (a
        // whole-struct assignment copies padding bytes from a stack
        // temporary) — frames must be deterministic (Principle #7) and
        // must not leak process memory into SHM. Field bytes are
        // untouched: they carry the caller's writes or the rosidl
        // defaults from borrow time. Ranges are registration-computed
        // from the same introspection data that sized the slot, so the
        // indexing is in-bounds by construction.
        {
            let bytes = loan.bytes_uninit_mut();
            for &(off, len) in data.bridge.loan_pad_ranges() {
                let start = WireHeader::SIZE + off;
                bytes[start..start + len].fill(std::mem::MaybeUninit::new(0));
            }
        }
        // SAFETY (byte coverage for `assume_init`): bytes [0, 32) were
        // zeroed at borrow (and are rewritten below); the payload's FIELD
        // bytes were written by the typesupport `init_function` at borrow
        // (rosidl ALL-initialization writes every member — the C bridge
        // additionally zero-fills first because rosidl's C `__init` skips
        // default-less members, rosidl#477) and/or by the caller through
        // the loaned pointer; the payload's PADDING bytes were zeroed
        // just above. No byte of the slot is unwritten.
        let mut loan = loan.assume_init();

        let seq = data.sequence.fetch_add(1, Ordering::Relaxed) as u32;
        let header = WireHeader {
            schema_hash: data.bridge.schema_hash(),
            total_size: (WireHeader::SIZE + data.bridge.layout().fixed_size) as u32,
            offset_table_offset: data.bridge.layout().fixed_size as u32,
            offset_table_count: 0,
            sequence: seq,
            timestamp_ns: now,
        };
        header.write_to_buf(&mut loan.bytes_mut()[..WireHeader::SIZE]);

        // Native history: no Cerulion-side ring push.
        // `send_raw_loan` retains the frame natively in the iceoryx2
        // publisher port's history queue (zero-copy, by SHM offset) on
        // success, and a failed send retains nothing (native history only
        // keeps frames that actually went out) — so no two-phase
        // push/finish/rollback machinery is needed.
        match inner.publisher.send_raw_loan(loan) {
            Ok(_recipients) => {
                inner.publisher.check_subscriber_events();
                if let Err(e) = inner.publisher.notify_sent_sample() {
                    tracing::debug!(topic = %data.topic, error = ?e, "notify failed");
                }
                RMW_RET_OK
            }
            Err(e) => {
                tracing::error!(topic = %data.topic, error = %e, "loaned publish failed");
                RMW_RET_ERROR
            }
        }
    })
}

/// Cancel path: drop the loan, releasing the SHM slot.
///
/// # Safety
/// `loaned_message` must come from `rmw_borrow_loaned_message`.
#[no_mangle]
pub unsafe extern "C" fn rmw_return_loaned_message_from_publisher(
    publisher: *const ffi::rmw_publisher_t,
    loaned_message: *mut c_void,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if publisher.is_null() || loaned_message.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*publisher).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        let data = &*((*publisher).data as *const PublisherData);
        let key = loaned_message as usize;
        // A poisoned lock means wedged: never re-enter torn iceoryx2 state.
        let Some(mut inner) = runtime::lock_unpoisoned(&data.inner) else {
            tracing::error!("entity wedged by an earlier panic; failing call");
            return RMW_RET_ERROR;
        };
        match inner.pending_loans.iter().position(|p| p.key == key) {
            Some(pos) => {
                let pending = inner.pending_loans.swap_remove(pos);
                match pending.kind {
                    runtime::LoanKind::Fixed => drop(pending.loan),
                    runtime::LoanKind::Windowed(w) => {
                        let inner = &mut *inner;
                        // Release the struct's heap containers while its
                        // headers are readable (in-slot storage is emptied
                        // by the walk / covered by the hook's quarantine).
                        data.bridge
                            .fini_borrowed_payload(key as *mut c_void, w.payload_len);
                        let me = std::thread::current().id();
                        match w.window_owner {
                            Some(owner) if owner == me => {
                                if let Some(api) = crate::heaphook::active_hook() {
                                    let _ = (api.disarm_window)();
                                }
                                // The armed window's quarantine extent
                                // outlives the loan until slot reuse.
                                inner.quarantined_tails.push(w.tail_base);
                                drop(pending.loan);
                            }
                            Some(owner) => {
                                // Wrong-thread return: the owner's window is
                                // still armed over this slot — HOLD it (see
                                // `PublisherInner::orphaned_loans`). A return
                                // publishes and copies NOTHING, so this is a
                                // slot-hold lifecycle event on its OWN latch,
                                // never a publish degrade (routing
                                // it through the publish reporter would falsely
                                // inflate `borrow_degrade_count` with a
                                // "publish copied" headline).
                                report_wrong_thread_return(&data.wrong_thread_returns, &data.topic);
                                inner.orphaned_loans.push(runtime::OrphanedLoan {
                                    thread: owner,
                                    tail_base: w.tail_base,
                                    loan: pending.loan,
                                });
                            }
                            None => drop(pending.loan),
                        }
                    }
                }
                RMW_RET_OK
            }
            None => RMW_RET_INVALID_ARGUMENT,
        }
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_publisher_count_matched_subscriptions(
    publisher: *const ffi::rmw_publisher_t,
    subscription_count: *mut usize,
) -> rmw_ret_t {
    // ffi_guard: the count probe descends into iceoryx2's service
    // builder (panic-capable on corrupt SHM); rclcpp wait-for-matched
    // loops hammer this entry point, and a panic crossing the C ABI
    // aborts the host process.
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if publisher.is_null() || subscription_count.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*publisher).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        let data = &*((*publisher).data as *const PublisherData);
        // Transport-level (iceoryx2 dynamic config): counts subscribers in
        // ALL processes. The local registry only sees this process — which
        // breaks cross-process readiness checks (rclcpp wait-for-matched,
        // rcl_action availability).
        let count = match runtime::runtime() {
            Ok(rt) => rt.transport.topic_subscriber_count(&data.topic),
            Err(_) => 0,
        };
        *subscription_count = count;
        RMW_RET_OK
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_publisher_get_actual_qos(
    publisher: *const ffi::rmw_publisher_t,
    qos: *mut ffi::rmw_qos_profile_t,
) -> rmw_ret_t {
    if publisher.is_null() || qos.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    *qos = default_actual_qos();
    RMW_RET_OK
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_init_publisher_allocation(
    _type_support: *const ffi::rosidl_message_type_support_t,
    _message_bounds: *const ffi::rosidl_runtime_c__Sequence__bound,
    _allocation: *mut ffi::rmw_publisher_allocation_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_fini_publisher_allocation(
    _allocation: *mut ffi::rmw_publisher_allocation_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED
}

/// # Safety
/// Caller frees with `rmw_publisher_free`.
#[no_mangle]
pub unsafe extern "C" fn rmw_publisher_allocate() -> *mut ffi::rmw_publisher_t {
    Box::into_raw(Box::new(std::mem::zeroed::<ffi::rmw_publisher_t>()))
}

/// # Safety
/// `publisher` must come from `rmw_publisher_allocate`.
#[no_mangle]
pub unsafe extern "C" fn rmw_publisher_free(publisher: *mut ffi::rmw_publisher_t) {
    if !publisher.is_null() {
        drop(Box::from_raw(publisher));
    }
}

// =====================================================================
// Subscription
// =====================================================================

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_create_subscription(
    node: *const ffi::rmw_node_t,
    type_support: *const ffi::rosidl_message_type_support_t,
    topic_name: *const c_char,
    qos: *const ffi::rmw_qos_profile_t,
    subscription_options: *const ffi::rmw_subscription_options_t,
) -> *mut ffi::rmw_subscription_t {
    ffi::ffi_guard(std::ptr::null_mut(), || unsafe {
        if node.is_null() || qos.is_null() || subscription_options.is_null() {
            return std::ptr::null_mut();
        }
        if !ffi::is_our_identifier((*node).implementation_identifier) {
            return std::ptr::null_mut();
        }
        let Some(ros_topic) = ffi::cstr(topic_name) else {
            return std::ptr::null_mut();
        };
        let Ok(rt) = runtime::runtime() else {
            return std::ptr::null_mut();
        };
        let Ok(bridge) = bridge_for(type_support) else {
            return std::ptr::null_mut();
        };

        // Identity mapping — see `rmw_create_publisher`. A subscription on ROS
        // `/chatter` reads the canonical `/chatter`, which is ALSO where a desk
        // mirror of a remote `/chatter` is re-injected — the reverse direction
        // a slash-stripped spelling would break.
        let topic = match runtime::ros_topic_to_cerulion(ros_topic) {
            Ok(topic) => topic,
            Err(error) => {
                tracing::error!(
                    topic = %ros_topic,
                    error = %error,
                    "subscription creation refused: the ROS name is not a fully-qualified topic name"
                );
                return std::ptr::null_mut();
            }
        };
        // The subscriber-slot reporting rule: `create_subscriber` forwards
        // the transport DEFAULTS — the caller's `depth`/`history` are read
        // exactly once, below, to fill an introspection record, and are never
        // applied to the iceoryx2 queue (the depth-honoring
        // `create_subscriber_with_buffers` is not used on this path). So the
        // depth this endpoint really got is the transport's default subscriber
        // buffer, and that is what endpoint info reports instead of
        // echoing an ask that changed nothing.
        let requested_depth = (*qos).depth;
        let provisioned_depth = rt.transport.subscriber_buffer_size();
        // Same create-config shape as `create_subscriber` (the
        // transport defaults — see the note above), with ONE
        // addition for loanable (plain) types: the create-leg borrow floor,
        // so a subscription-first create also mints the service loan-ready
        // (order-independent with the publisher-side floor; the core keeps
        // the OPEN leg requirement-free, so a pre-existing smaller service
        // still attaches).
        let mut topic_config = rt.transport.default_topic_config();
        // The take-side gate (fixed types AND forgeable-
        // sequence types) — distinct from the publisher's `can_loan`.
        let can_loan_take = bridge.can_loan_take();
        // The adopt-take gate — read at CREATE
        // because it keys the borrow floor (adopted samples pin borrow
        // units for as long as the APP retains messages, so the loaned-take
        // sizing of 4 is wrong for this mode) and because per-subscription
        // latching avoids mid-stream mode flips.
        let mut adopt = crate::adopt_take::arm_for_create(&topic, &bridge);
        if let Some(state) = &adopt {
            // COMPOSE the two floors,
            // never replace one with the other. Arming adopt-take requires a
            // forgeable sequence, and `can_loan_take` is `can_loan ||
            // forge_count > 0`, so an adopt-armed subscription is ALWAYS
            // loan-capable too — and it stays advertised as such
            // (`can_loan_messages = can_loan_take`) with its shadow pool
            // sized `RMW_TAKE_LOAN_BORROW_BUDGET`. An `else if` would therefore let
            // a small `CERULION_RMW_ADOPT_TAKE_BUDGET` SHRINK the floor the
            // loaned-take path depends on: a budget of 1-3 mints the service
            // at 2-3 borrows (1 and 2 are discarded by
            // `effective_create_borrow_floor` as not exceeding the iceoryx2
            // default), and the independent `rmw_take_loaned_message` path on
            // that same subscription then fails `ExceedsMaxBorrows` one or
            // two loans early. The cost of the max is APPARENT bytes only
            // (demand-paged slots), which is the wrong thing to save.
            topic_config.create_borrow_floor = Some(if can_loan_take {
                state.budget.max(RMW_TAKE_LOAN_BORROW_BUDGET)
            } else {
                state.budget
            });
        } else if can_loan_take {
            topic_config.create_borrow_floor = Some(RMW_TAKE_LOAN_BORROW_BUDGET);
        }
        let subscriber = match rt.transport.create_subscriber_with_buffers(
            &topic,
            topic_config,
            provisioned_depth,
        ) {
            Ok(s) => s,
            Err(e) => {
                tracing::error!(topic = %topic, error = %e, "subscription creation failed");
                return std::ptr::null_mut();
            }
        };
        // The effective borrow budget is the
        // service's real capacity — smaller than the requested floor when the
        // service already existed (the floor is CREATE-leg-only). The adopt
        // refusal diagnostics report this value.
        if let Some(state) = adopt.as_mut() {
            state.effective_budget = subscriber.max_borrowed_samples();
        }
        // A service whose borrow ceiling cannot carry a
        // held sample AND an incoming receive cannot carry adoption either —
        // a caller reusing one message WEDGES on it permanently, because the
        // release that would free its borrow runs inside the take the borrow
        // blocks. The wedge cannot be closed from inside the take,
        // so it is closed by not arming:
        // the copying take holds nothing and cannot wedge. Reachable only on
        // a PRE-EXISTING service, since the OPEN leg is requirement-free by
        // design and nothing this crate creates goes below the floor.
        if let Some(state) = adopt.as_ref() {
            if crate::adopt_take::adopt_viability(state.effective_budget)
                == crate::adopt_take::AdoptViability::BorrowCeilingTooSmall
            {
                tracing::warn!(
                    topic = %crate::era_check::escape_control_chars(&topic),
                    effective_budget = state.effective_budget,
                    minimum = crate::adopt_take::MIN_ADOPT_EFFECTIVE_BUDGET,
                    "adopt-take NOT armed for this subscription — the service's \
                     subscriber_max_borrowed_samples is below the minimum adoption \
                     needs (one unit for the message the app still holds, one for \
                     the incoming receive), so an adopting take would refuse every \
                     receive a reusing caller makes and never run the release that \
                     would recover; this subscription's PLAIN takes are served by the \
                     COPYING take, which holds nothing (rmw_take_loaned_message is \
                     unaffected: a loan is returned by its own call, so it cannot wedge \
                     the way an adopted reuse does). The ceiling comes from whoever created the \
                     service first — stop that creator and restart this one, or raise \
                     its subscriber_max_borrowed_samples"
                );
                adopt = None;
            }
        }
        // The ONE affirmative claim, made only once every gate has passed —
        // an operator greps this line to confirm the zero-copy path is on,
        // so it must never be printed for a subscription that then degrades.
        if let Some(state) = adopt.as_ref() {
            tracing::info!(
                topic = %crate::era_check::escape_control_chars(&topic),
                budget = state.budget,
                effective_budget = state.effective_budget,
                "adopt-take ARMED for this subscription — plain takes of its forgeable \
                 sequences serve SHM in place; the app's free() releases each sample"
            );
        }
        // The ask is DROPPED on this path, so say so at create time
        // rather than letting the introspection API report the request back as
        // though it had been honoured.
        if requested_depth != provisioned_depth {
            tracing::warn!(
                topic = %topic,
                requested_depth,
                provisioned_depth,
                "requested QoS depth differs from the depth Cerulion provisioned — \
                 an rmw subscription's requested depth is NOT applied to its queue, \
                 so the queue is the transport default; endpoint info reports \
                 the provisioned depth"
            );
        }

        // Deterministic gid: entity counter + schema hash (never random) —
        // mirrors the publisher; the endpoint id reported by
        // rmw_get_subscriptions_info_by_topic and the registry deregister
        // key. Read the bridge BEFORE it moves into `data`.
        let entity = runtime::next_entity_id();
        let mut gid = [0u8; 16];
        gid[..8].copy_from_slice(&entity.to_le_bytes());
        gid[8..].copy_from_slice(&bridge.schema_hash().to_le_bytes());

        let type_name = ros_graph_type_name(bridge.qualified_name());
        // Adopt-take hot path: the
        // adoption scratch is reserved UP FRONT to the type's forgeable-
        // member count — the high-water bound both Vecs ever reach — so the
        // FIRST adopted take allocates only the structural Arc. A
        // `Vec::new()` here would make the first adopted receive grow both
        // scratch Vecs before the one allocation the contract allows. Zero
        // for a type with no forgeable member (no reservation at all).
        let forgeable_members = bridge.forged_sequence_count();
        let data = Box::new(SubscriptionData {
            topic: topic.clone(),
            type_name,
            bridge,
            inner: std::sync::Mutex::new(SubscriptionInner {
                subscriber,
                // No outstanding take loans yet. Reserved UP FRONT to
                // the borrow budget so every in-budget loaned take is
                // allocation-free on the receive path — a `Vec::new()` here
                // would make the FIRST successful take allocate. The budget
                // already bounds outstanding loans on an rmw-created service
                // (iceoryx2 refuses the next receive past it); only a
                // pre-existing service created with a LARGER budget (a graph's
                // `--record` raise) could ever push past this reservation,
                // and that is a one-time cold growth, not a per-take cost.
                pending_takes: Vec::with_capacity(RMW_TAKE_LOAN_BORROW_BUDGET),
                // Shadows for the forged take, built lazily
                // up to the loan budget and then recycled (see
                // `TakeShadowPool`); a fixed type never touches it.
                shadows: runtime::TakeShadowPool::new(RMW_TAKE_LOAN_BORROW_BUDGET),
                // The adoption branch's reusable scratch:
                // reserved to the forgeable-member count at create, so even
                // the first adopted take grows neither (see above).
                adopt_ranges: Vec::with_capacity(forgeable_members),
                adopt_registered: Vec::with_capacity(forgeable_members),
            }),
            gid,
            decode_failures: std::sync::Mutex::new(
                crate::decode_failure_latch::DecodeFailureLatch::new(),
            ),
            hash_mismatches: std::sync::Mutex::new(
                cerulion_core::transport::failure_regime_latch::FailureRegimeLatch::new(),
            ),
            loan_refusals: std::sync::Mutex::new(
                cerulion_core::transport::failure_regime_latch::FailureRegimeLatch::new(),
            ),
            forge_fallbacks: std::sync::Mutex::new(
                cerulion_core::transport::failure_regime_latch::FailureRegimeLatch::new(),
            ),
            adopt,
        });

        // Computed BEFORE `Box::into_raw(data)` so an OOM panic here drops
        // `data`/`rmw_sub` while still owned, instead of orphaning the
        // raw'd box (Principle #11 — mirrors the
        // publisher).
        let topic_name_ptr = leak_ros_name(ros_topic);

        // Zeroed + field stores instead of a struct literal: distros ADD
        // fields to rmw_subscription_t over time (rolling has
        // `is_cft_supported`; Jazzy does not) — naming every field would
        // make the source compile against exactly one distro's bindings.
        // Unnamed fields stay zero (false/null), which is the correct
        // "not supported" value for all of them.
        let mut rmw_sub: Box<ffi::rmw_subscription_t> = Box::new(std::mem::zeroed());
        rmw_sub.implementation_identifier = ffi::implementation_identifier_ptr();
        rmw_sub.data = Box::into_raw(data) as *mut c_void;
        rmw_sub.topic_name = topic_name_ptr;
        rmw_sub.options = *subscription_options;
        // Loaned takes: for plain (recursively-
        // fixed) types AND for types carrying a forgeable primitive sequence
        // (`Image.data`, `LaserScan.ranges`, …). NOT the publisher's gate:
        // a borrow must hand out a whole SHM-resident struct, a take only has
        // to hand out a readable one. rclcpp's executor routes EVERY
        // subscription of a `can_loan_messages` topic through
        // `rmw_take_loaned_message` — the mutable `SharedPtr` callback shape
        // included, which is why the read-only loan contract is documented
        // beside the fixed-type rule — and Jazzy additionally gates on
        // `ROS_DISABLE_LOANED_MESSAGES=0`, above the rmw.
        rmw_sub.can_loan_messages = can_loan_take;
        let ptr = Box::into_raw(rmw_sub);
        // Registry mutation last (see rmw_create_publisher).
        {
            let data_ref = &*((*ptr).data as *const SubscriptionData);
            // The adopt stats join the
            // process-global shutdown registry HERE — only a create that
            // already succeeded reaches this block, so a failed create can
            // never leak a registry Arc.
            if let Some(state) = &data_ref.adopt {
                crate::adopt_take::register_stats(Arc::clone(&state.stats));
            }
            // Loud zero-copy reporting — the subscription-side mirror of the
            // publisher's create log: rclcpp silently downgrades to the
            // copying take when can_loan_messages is false.
            tracing::info!(
                topic = %data_ref.topic,
                type_name = %data_ref.type_name,
                zero_copy = can_loan_take,
                forged_sequences = data_ref.bridge.forged_sequence_count(),
                "subscription created (zero_copy=false ⇒ copying take/unflatten; \
                 zero_copy=true ⇒ loaned take holds the SHM sample across the C ABI, \
                 forged_sequences>0 ⇒ served through an rmw-owned shadow whose primitive \
                 sequences aim at that sample)"
            );
            // Endpoint record for rmw_get_subscriptions_info_by_topic
            // — mirrors the publisher side.
            let node_data = &*((*node).data as *const runtime::NodeData);
            let record = runtime::EndpointRecord {
                node_name: node_data.name.clone(),
                node_namespace: node_data.namespace.clone(),
                type_name: data_ref.type_name.clone(),
                endpoint_gid: data_ref.gid,
                qos: runtime::QosSnapshot {
                    reliability: (*qos).reliability,
                    durability: (*qos).durability,
                    history: (*qos).history,
                    requested_depth,
                    provisioned_depth,
                },
            };
            let mut graph = rt.graph.write().unwrap_or_else(|e| e.into_inner());
            GraphRegistry::add_endpoint(&mut graph.subscriptions, &data_ref.topic, record);
        }
        runtime::notify_graph_change();
        ptr
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_destroy_subscription(
    node: *mut ffi::rmw_node_t,
    subscription: *mut ffi::rmw_subscription_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if node.is_null() || subscription.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*subscription).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        // Borrow first, free last (see rmw_destroy_publisher).
        {
            let data = &*((*subscription).data as *const SubscriptionData);
            // A destroy with outstanding take loans RELEASES them
            // (the Box drop below drops the pending_takes samples, freeing
            // the SHM borrows — and un-forges + destroys
            // each loan's shadow FIRST, so no destructor sees a shared-memory
            // address) — never a leak, never an abort. Loud, because the
            // caller's loaned pointers are dangling from here on and a later
            // "return" will be INVALID_ARGUMENT.
            //
            // Counted under the lock; a POISONED mutex skips the count (we
            // will not re-enter torn state just to log a number) but the Box
            // drop still releases every held sample — Mutex::drop drops its
            // contents whether or not it is poisoned.
            if let Some(inner) = runtime::lock_unpoisoned(&data.inner) {
                let outstanding = inner.pending_takes.len();
                if outstanding > 0 {
                    tracing::warn!(
                        topic = %data.topic,
                        outstanding,
                        "subscription destroyed with outstanding loaned takes — \
                         releasing their SHM borrows now; the loaned pointers \
                         are dangling"
                    );
                }
            }
            // The per-subscription adopt-take
            // proof line, and — with adopted samples outstanding — the
            // warn-and-LEAVE arm: the app legitimately holds forged
            // pointers into those samples, so their registrations are left
            // in place (reclaiming would dangle them; the release callback
            // and the Arc refcounts reference nothing subscription-scoped,
            // so frees after destroy still release correctly).
            if let Some(adopt) = &data.adopt {
                crate::adopt_take::log_destroy_summary(&data.topic, &adopt.stats);
                // Fold this subscription's
                // counts into the retired total and drop its registry
                // entry, after the proof line above, which is unaffected.
                // Without this the registry keeps a strong `Arc` per successful
                // create forever, so a process that cycles
                // subscriptions grows it without bound.
                crate::adopt_take::retire_stats(&adopt.stats);
            }
            if let Ok(rt) = runtime::runtime() {
                let mut graph = rt.graph.write().unwrap_or_else(|e| e.into_inner());
                GraphRegistry::remove_endpoint(&mut graph.subscriptions, &data.topic, &data.gid);
            }
            runtime::notify_graph_change();
        }
        unleak_ros_name((*subscription).topic_name);
        drop(Box::from_raw((*subscription).data as *mut SubscriptionData));
        drop(Box::from_raw(subscription));
        RMW_RET_OK
    })
}

unsafe fn take_impl(
    subscription: *const ffi::rmw_subscription_t,
    ros_message: *mut c_void,
    taken: *mut bool,
    message_info: *mut ffi::rmw_message_info_t,
) -> rmw_ret_t {
    if subscription.is_null() || ros_message.is_null() || taken.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    if !ffi::is_our_identifier((*subscription).implementation_identifier) {
        return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
    }
    *taken = false;
    let data = &*((*subscription).data as *const SubscriptionData);
    let expected_hash = data.bridge.schema_hash();

    // A poisoned lock means wedged: never re-enter torn iceoryx2 state.
    let Some(inner) = runtime::lock_unpoisoned(&data.inner) else {
        tracing::error!("entity wedged by an earlier panic; failing call");
        return RMW_RET_ERROR;
    };
    // Under the adopt grant, the plain take is
    // served by ADOPTION — forgeable sequences aim at the held sample,
    // whose byte ranges are registered with the preloaded heap hook so the
    // app's eventual free() releases it. No grant (the `adopt` field is
    // `None`), and everything below is the plain copying take.
    if let Some(adopt) = data.adopt.as_ref() {
        // The grant proves the hook
        // was live at subscription CREATE; it does not prove the release
        // callback is still installed NOW. The final context's `rmw_shutdown`
        // clears it, and nothing in rmw or rcl guarantees every subscription
        // is destroyed first — an executor still spinning, or node destructors
        // running after `rclcpp::shutdown()`, both take afterwards. Adopting
        // then registers ranges with a hook that has no callback to call, so
        // the app's `free()` takes the hook's no-callback path and LEAKS the
        // range by design: the cookie is never reclaimed, `outstanding` never
        // drains, and the SHM borrow plus its pool slot are pinned for the
        // life of the process. The copying take is the correct fallback —
        // it serves the same bytes and holds nothing.
        let callback_epoch = crate::adopt_take::release_callback_epoch();
        if callback_epoch != 0 {
            return take_adopted(
                data,
                adopt,
                inner,
                ros_message,
                taken,
                message_info,
                callback_epoch,
            );
        }
        warn_adopt_degraded_after_shutdown(&data.topic);
    }
    let mut result = RMW_RET_OK;
    let mut took = false;
    let mut info_out: Option<(u64, u64)> = None;
    // ONE message per call (the rmw take contract): try_receive_one,
    // never the drain-everything path — extra drained frames would be
    // silently lost (Principle #6).
    let receive = inner.subscriber.try_receive_one(|msg| {
        let header = msg.header();
        // The DECISION lives in `take_gate` — one pure
        // function, one legal mutation target, shared by all three take
        // paths; the CONSEQUENCE (which latch, whether the frame is
        // consumed, what the caller is told) stays here where it differs.
        if crate::take_gate::schema_hash_verdict(header.schema_hash, expected_hash).is_mismatch() {
            // A BARE per-frame `warn!` would flood here. A hash
            // mismatch is a type SKEW, so it fails every frame until
            // somebody redeploys — on a 100 Hz topic that is ~100
            // lines/s forever, the disk-fill class. Loud once
            // per regime, counted always.
            let mut latch = cerulion_core::transport::failure_regime_latch::lock_regime_latch(
                &data.hash_mismatches,
            );
            cerulion_core::transport::frame_drop_latch::report_schema_hash_mismatch(
                &mut latch,
                cerulion_core::transport::frame_drop_latch::FrameDropSite::Message,
                &data.topic,
                expected_hash,
                header.schema_hash,
            );
            return;
        }
        {
            // Recovery is observed HERE — the hash gate passing is the
            // whole condition; whether the bridge can then DECODE the
            // frame is a different condition with its own latch.
            let mut latch = cerulion_core::transport::failure_regime_latch::lock_regime_latch(
                &data.hash_mismatches,
            );
            cerulion_core::transport::frame_drop_latch::report_schema_hash_match(
                &mut latch,
                cerulion_core::transport::frame_drop_latch::FrameDropSite::Message,
                &data.topic,
            );
        }
        if data.bridge.unflatten(msg.payload(), ros_message) {
            took = true;
            info_out = Some((header.timestamp_ns, header.sequence as u64));
            crate::decode_failure_latch::report_decode_success(
                &data.decode_failures,
                crate::decode_failure_latch::DecodeSite::Subscription,
                &data.topic,
            );
        } else {
            // Without this arm the frame vanishes
            // with no log and no counter, and `taken = false` is exactly
            // what an EMPTY queue returns, so a subscriber dropping 100% of
            // a live topic looks idle. Loud once per regime.
            crate::decode_failure_latch::report_decode_failure(
                &data.decode_failures,
                crate::decode_failure_latch::DecodeSite::Subscription,
                &data.topic,
                &data.type_name,
                msg.payload().len(),
            );
        }
    });
    drop(inner);

    match receive {
        Ok(_) => {}
        Err(e) => {
            tracing::error!(topic = %data.topic, error = %e, "take failed");
            result = RMW_RET_ERROR;
        }
    }
    if took {
        *taken = true;
        if !message_info.is_null() {
            let mut info: ffi::rmw_message_info_t = std::mem::zeroed();
            if let Some((ts, seq)) = info_out {
                info.source_timestamp = ts as i64;
                info.received_timestamp = match runtime::runtime() {
                    Ok(rt) => rt.transport.clock().now_ns() as i64,
                    Err(_) => 0,
                };
                info.publication_sequence_number = seq;
                info.reception_sequence_number = u64::MAX;
            }
            info.publisher_gid.implementation_identifier = ffi::implementation_identifier_ptr();
            info.from_intra_process = false;
            *message_info = info;
        }
    }
    result
}

/// Withdraw a set of adopt-take registrations and reclaim their refcount
/// clones — the caller-side rollback, which never fires the release
/// callback.
///
/// This is one routine,
/// because there are TWO ways the adopting take can end with registrations
/// the caller will never free. The planned one is a registration FAILURE
/// mid-loop. The other is a PANIC after the loop completed: the forged
/// guard empties the caller's headers on unwind — which is what makes the
/// message `fini`-safe — and that is exactly what leaves the app with no
/// pointer to any registered range. "Each later free still releases its
/// sample" is true on the success path and FALSE here: nothing would ever
/// free them, the cookies keep their clones alive, and the sample's SHM
/// borrow slot stays pinned for the life of the process.
///
/// # Safety
/// Every `cookie` must be a live `Arc::into_raw(Arc<AdoptedSample>)` that
/// this process registered and has not yet reclaimed.
unsafe fn withdraw_adopt_registrations(
    api: &crate::heaphook::HookApi,
    registered: &[(usize, usize)],
    topic: &str,
) {
    use crate::adopt_take::AdoptedSample;
    use crate::heaphook::RC_OK;
    for &(ptr, cookie) in registered {
        let urc = (api.unregister_segment)(ptr as *mut c_void);
        if urc == RC_OK {
            drop(unsafe { std::sync::Arc::from_raw(cookie as *const AdoptedSample) });
        } else {
            // "Cannot happen" (nothing else can remove a range the app has
            // never seen) — if it does, the hook may still own the cookie:
            // leak the clone (the sample stays pinned) rather than risk a
            // double release.
            tracing::error!(
                topic = %topic,
                ptr,
                urc,
                "adopt-take rollback: unregister failed — leaking its refcount clone \
                 rather than risking a double release"
            );
        }
    }
}

/// The one unwind path for a caller message
/// whose forgeable members currently aim into a held sample. Armed
/// immediately after a successful `unflatten_forged`; every fallible step
/// between the forge and the completed registrations (latch reporting, the
/// range Vec, `Arc::new`, the registration loop) runs under it. Three ways
/// out, all leaving the message fini-safe:
///
/// - SUCCESS (registrations complete): [`Self::disarm`] — the forged
///   pointers are live and registered, exactly the contract;
/// - the planned registration-failure rollback: [`Self::unforge_now`] —
///   the SAME un-forge the panic path runs, then the caller copies the
///   masked members (one cleanup routine, two entry points);
/// - a PANIC anywhere in the window: `Drop` un-forges (masked members go
///   to the EMPTY header — rosidl `fini` / `~vector` free nothing), the
///   sample's `Arc` unwinds and releases, and `ffi_guard` converts the
///   panic to `RMW_RET_ERROR` — the caller's later `fini` touches no
///   dangling SHM pointer.
struct ForgedMessageGuard<'a> {
    bridge: &'a crate::bridge::AnyBridge,
    msg: *mut c_void,
    mask: u64,
    armed: bool,
}

impl ForgedMessageGuard<'_> {
    /// Success: the forged pointers are registered and MEANT to outlive
    /// this call — nothing to clean. Called
    /// LAST, after every panic-capable reporter in the function's tail, so
    /// a panic anywhere before it still runs the `Drop` un-forge (`&mut`
    /// rather than consuming, because the registration-failure arm flips
    /// the guard mid-function and the tail disarm must still be reachable
    /// on that path — idempotent by construction).
    fn disarm(&mut self) {
        self.armed = false;
    }

    /// The rollback's un-forge — the same routine the panic path runs,
    /// executed NOW so the caller can copy the masked members over the
    /// emptied headers.
    ///
    /// This is gated on `armed`, exactly as
    /// `Drop` is. Un-forging is idempotent only while the members still aim
    /// at the sample. Once the registration-failure rollback has stood the
    /// guard down and `copy_forged_members` has refilled those members with
    /// REAL heap buffers, a second un-forge overwrites their headers with
    /// the EMPTY header — it does not free — so the only pointer to each
    /// freshly-allocated buffer is dropped. A tail-panic path
    /// that called this unconditionally would leak one buffer set per panic on
    /// the copy-fallback path.
    fn unforge_now(&mut self) {
        if !self.armed {
            return;
        }
        self.armed = false;
        if self.mask != 0 {
            // SAFETY: `msg` is the caller's initialized message of `bridge`'s
            // type whose `mask` members were forged by `unflatten_forged`
            // (the arming site's contract).
            unsafe { self.bridge.unforge(self.msg, self.mask) };
        }
    }
}

impl Drop for ForgedMessageGuard<'_> {
    fn drop(&mut self) {
        if self.armed && self.mask != 0 {
            // SAFETY: as in `unforge_now` — the unwind arm of the one
            // cleanup routine.
            unsafe { self.bridge.unforge(self.msg, self.mask) };
        }
    }
}

/// One line per PROCESS when a still-alive subscription takes after
/// the release callback was cleared. Loud inference (the subscription silently
/// degrades to copying), bounded to one line because the condition is
/// permanent once it starts and the process is already shutting down.
fn warn_adopt_degraded_after_shutdown(topic: &str) {
    static WARNED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
    if WARNED.swap(true, Ordering::Relaxed) {
        return;
    }
    tracing::warn!(
        topic = %topic,
        "adopt-take degraded to the copying take: the heap hook's release callback was \
         cleared at the final rmw_shutdown while this subscription is still taking. \
         Adopting now would register ranges nothing can release, pinning one shared-memory \
         borrow per take for the life of the process. Destroy subscriptions before \
         rmw_shutdown to keep the zero-copy path."
    );
}

/// Adopt-take: the ADOPTING plain take — `take_impl`'s
/// serve path when the subscription holds an
/// [`crate::adopt_take::AdoptTakeGrant`] (env gate armed at create + Active
/// heap-hook handshake + a forgeable type; the witness parameter is the
/// structural gate — no grant, no branch).
///
/// One frame is received OWNED (`try_receive_one_owned` — the same queue
/// receive as the copying take, the one-read-path rule), decoded through
/// `unflatten_forged` against the CALLER-owned message (fixed fields /
/// strings / nested members copy exactly as `unflatten` copies them; each
/// forgeable sequence's container header aims at the held sample's bytes,
/// `capacity == size` so growth reallocates), and each forged member's
/// exact byte range is registered with the hook, cookie =
/// `Arc::into_raw(Arc<AdoptedSample>)`. The take returns with the sample
/// HELD; the app's `fini`/destructor frees each forged pointer, the hook's
/// interposed `free` classifies it as a Release and drops one clone; the
/// LAST drop releases the SHM borrow + pool slot.
///
/// Fallbacks, all serving the same bytes the copying take serves:
/// - `forged == 0` (every forgeable entry below the data floor / past the
///   mask): the sample drops immediately — behavior byte-identical to
///   the copy path;
/// - any registration failure ("should be never"): siblings unregistered,
///   the message un-forged, the masked members copied
///   (`copy_forged_members` — only the `PrimSeq` copy arm, never a re-run
///   of the whole `unflatten` over a partially-populated message), the
///   sample dropped, latched;
/// - decode `Err`: the plain path's decode-failure handling (frame
///   consumed + dropped through the latch; below-floor copies made
///   before the failure are owned by the caller's `fini`, the copying
///   take's own convention).
///
/// `ExceedsMaxBorrows` here is app-induced retention (adopted messages pin
/// borrow units until freed): `taken = false`, the frame stays queued, a
/// latched warn names the budget knob — NEVER `RMW_RET_ERROR`; the refusal
/// self-heals when the app frees any adopted message.
///
/// # Known residual: a take racing the FINAL `rmw_shutdown`
///
/// Such a take can return a sample whose release path is already gone — the
/// registrations are live, the callback that would release them is not, and
/// the app's `free()` takes the hook's no-callback path, pinning one borrow
/// and its pool slot. This limit is DELIBERATE.
///
/// It cannot be closed by a check here. The application takes delivery when
/// `rmw_take` RETURNS, so any window a check closes simply moves past the
/// check — which is what the dispatch gate and the post-registration epoch
/// re-read each do: they narrow the window and cannot close it.
/// The two designs that DO close it are both rejected: mutual exclusion
/// between shutdown and the take (a lock on the path this mode exists to make
/// fast), and deferring the callback teardown until adopted samples drain —
/// which reverses the deliberate contract that a post-shutdown free must never
/// call into a possibly-unmapped module, pinned by three tests.
///
/// Why it is acceptable: rcl stops executors before `rmw_shutdown`, so a
/// well-formed shutdown has no take in flight. And the window is
/// SHUTDOWN-ONLY and BOUNDED — a take that completes while the context is
/// alive never loses its registrations, a clear landing DURING a take is
/// caught by the epoch re-read and rolled back, and once the epoch is zero the
/// dispatch gate degrades every later take to a copy, so the loss cannot grow.
/// Pinned by `the_lost_registration_window_is_shutdown_only_and_bounded`.
unsafe fn take_adopted(
    data: &SubscriptionData,
    adopt: &crate::adopt_take::AdoptTakeState,
    mut inner: std::sync::MutexGuard<'_, SubscriptionInner>,
    ros_message: *mut c_void,
    taken: *mut bool,
    message_info: *mut ffi::rmw_message_info_t,
    // The release-callback epoch the DISPATCH observed. Re-read
    // after the registrations land; a different value means a shutdown
    // cleared (or cleared and re-installed) the callback in between, so the
    // registrations belong to a generation that is gone and must be
    // withdrawn.
    callback_epoch: u64,
) -> rmw_ret_t {
    use crate::adopt_take::AdoptedSample;
    use crate::heaphook::RC_OK;
    use crate::loan_refusal_latch::LoanServed;

    let expected_hash = data.bridge.schema_hash();
    let api = *adopt.grant.api();

    let owned = match inner.subscriber.try_receive_one_owned() {
        Ok(o) => {
            // The receive SUCCEEDED, so close any open
            // receive-failure regime. A latch with no success side is worse
            // than no latch: opening
            // `adopt.receive_failures` and closing it nowhere would let ONE
            // transport fault suppress every later fault on this
            // subscription for the life of the process.
            //
            // Here rather than in the tail, because the condition this latch
            // tracks is whether the TRANSPORT accepted the call, not whether
            // a frame was served: an empty queue is a successful receive and
            // recovers it, as does a frame that then fails its hash gate, its
            // entry gate or its decode — each of those has its own latch for
            // its own condition. Costs one uncontended lock and a branch on
            // the healthy path, the price the sibling reporters already pay.
            crate::loan_refusal_latch::report_adopt_receive_recovered(
                &adopt.receive_failures,
                &data.topic,
            );
            o
        }
        Err(e) => {
            let reason = e.to_string();
            if reason.contains("ExceedsMaxBorrows") {
                adopt.stats.budget_refusals.fetch_add(1, Ordering::Relaxed);
                // An adopt-armed type is ALWAYS
                // `can_loan_take` too, so `rmw_take_loaned_message`'s
                // outstanding loans consume the SAME
                // `subscriber_max_borrowed_samples` budget as adopted
                // samples. Reporting
                // `kind=adopted_budget_exhausted` and advising to free adopted
                // messages whichever side holds the borrows would be advice that does
                // nothing for a consumer whose borrows are all LOANS, on a
                // line an operator greps by kind. Both counts are already in
                // hand here, so the attribution is a pure classification of
                // them (`BorrowHolders::classify`) rather than an assumption.
                let adopted_outstanding = adopt.stats.outstanding.load(Ordering::Relaxed) as usize;
                let loaned_outstanding = inner.pending_takes.len();
                crate::loan_refusal_latch::report_adopt_borrow_refused(
                    &data.loan_refusals,
                    &data.topic,
                    adopted_outstanding,
                    loaned_outstanding,
                    // The effective service
                    // capacity — what ExceedsMaxBorrows really bound at —
                    // never the requested create floor.
                    adopt.effective_budget,
                    &reason,
                );
                return RMW_RET_OK;
            }
            // The else-arm of the branch above: a transport fault that is not
            // the borrow budget. A BARE per-take `error!` here is, on a
            // 100 Hz topic with a persistent fault, ~100 lines/s (the disk-fill
            // class) — so it rides a latch. The return
            // stays `RMW_RET_ERROR`: only the log volume is bounded.
            //
            // Its OWN latch and its OWN wording, not the loaned path's.
            // Routing it through `LoanRefusal::ReceiveFailed` on the shared
            // `loan_refusals` latch would bound the volume and buy two bugs:
            // it would print "loaned take refused — return outstanding loans" on
            // an ADOPTED take that holds no loan and returns an error, and —
            // worse — the shared regime is kind-agnostic and closes only on
            // a SUCCESS, so a consumer retaining adopted messages holds it
            // open forever and a genuine transport fault would be suppressed
            // to `debug!`, below the shipped filter, while still failing the
            // call. Different conditions, different remedies, different
            // latches.
            crate::loan_refusal_latch::report_adopt_receive_failed(
                &adopt.receive_failures,
                &data.topic,
                &reason,
            );
            return RMW_RET_ERROR;
        }
    };
    let Some(owned) = owned else {
        // Empty queue — `taken` stays false.
        return RMW_RET_OK;
    };
    // `try_receive_one_owned` only returns frames whose header parses and
    // whose total_size is in bounds — a None here means the frame changed
    // under us; fail loudly, never fabricate (mirror of take_loaned_impl).
    let Some(header) = owned.wire_header() else {
        tracing::error!(topic = %data.topic, "adopted take: header re-read failed; dropping frame");
        return RMW_RET_ERROR;
    };
    // The pure gate (see `take_gate`), not a fourth
    // hand-spelled comparison.
    if crate::take_gate::schema_hash_verdict(header.schema_hash, expected_hash).is_mismatch() {
        // Same regime latch + semantics as the copying take: the
        // frame is consumed and dropped, `taken` stays false.
        drop(owned);
        let mut latch = cerulion_core::transport::failure_regime_latch::lock_regime_latch(
            &data.hash_mismatches,
        );
        cerulion_core::transport::frame_drop_latch::report_schema_hash_mismatch(
            &mut latch,
            cerulion_core::transport::frame_drop_latch::FrameDropSite::Message,
            &data.topic,
            expected_hash,
            header.schema_hash,
        );
        return RMW_RET_OK;
    }
    {
        let mut latch = cerulion_core::transport::failure_regime_latch::lock_regime_latch(
            &data.hash_mismatches,
        );
        cerulion_core::transport::frame_drop_latch::report_schema_hash_match(
            &mut latch,
            cerulion_core::transport::frame_drop_latch::FrameDropSite::Message,
            &data.topic,
        );
    }
    let ts = header.timestamp_ns;
    let seq = header.sequence as u64;

    // The PRE-WRITE frame gate. Both decodes below walk
    // the type's members writing as they go and bail at the first entry they
    // cannot resolve, so without this gate a malformed frame leaves a caller that
    // legally reuses ONE message across takes holding a CHIMERA — some
    // members from the new frame, the rest from the one it had — on a call
    // that reports nothing taken. Asking the same question first,
    // READ-ONLY, means such a frame is refused with the caller's message
    // BYTE-UNTOUCHED: this runs BEFORE `release_forgeable_members` below, so
    // not even the previous take's samples are given back on a frame that
    // was never delivered.
    //
    // The alternative, a scratch decode plus a member
    // swap, is UNSOUND for the C++ bridge and is deliberately not
    // used: a rosidl C++ message holds real `std::string` objects, which
    // libstdc++ makes non-trivially-relocatable (its short-string form
    // stores a pointer into the object's OWN buffer), so moving a decoded
    // message's bytes into the caller's would corrupt every string member on
    // the platform ROS 2 ships. `type_bridge_cpp` already records the same
    // constraint from the other side ("a zeroed `std::string` is not even a
    // valid object on libstdc++") and reaches those members only through the
    // compiled shim. rosidl introspection exposes no per-member move, so a
    // "member swap" has nothing to call.
    //
    // SCOPE: this gate is on the ADOPTED path only. `take_impl`'s copying
    // take decodes member-by-member into the caller's message and can
    // leave a chimera — including on a subscription the adopt
    // viability gate routed there. The plain take
    // path every rmw user rides does not carry this gate.
    //
    // What this therefore closes and what it does not is stated on
    // `AnyBridge::frame_entries_readable`; the residual classes (an
    // allocation failure in a copy arm, a malformed body inside a NESTED
    // member, and an unaligned forged entry) still fail mid-decode — exactly
    // as the plain copying take does.
    {
        let body = &owned.payload()[WireHeader::SIZE..];
        if let Err((var_idx, verdict)) = data.bridge.frame_entries_readable(body) {
            let body_len = body.len();
            // The frame is consumed and dropped through the same decode-failure
            // latch the decode's own failure arms use — one condition, one
            // regime, whichever of them detected it.
            drop(owned);
            // The member attribution the decode's own arm prints through
            // `warn_bad_entry` rides the LATCHED line, not a second bare one.
            // A stale or hand-rolled producer fails EVERY frame, so an
            // unlatched line here is one warning per frame at frame rate —
            // the disk-fill class, and it would duplicate the
            // latched report's own loud head and decade re-announcements.
            // Same latch, same regime, same counter as the decode's own
            // failure arms; what this reporter adds is `var_idx=`/`reason=`
            // and a headline whose cause is actually THIS one.
            crate::decode_failure_latch::report_decode_entry_refused(
                &data.decode_failures,
                crate::decode_failure_latch::DecodeSite::Subscription,
                &data.topic,
                &data.type_name,
                body_len,
                var_idx,
                verdict.as_str(),
            );
            return RMW_RET_OK;
        }
    }

    // Reuse safety: the plain `unflatten` frees a reused message's previous
    // allocations inside its copy arms; the forge arm OVERWRITES headers.
    // Release existing forgeable storage first, so a caller legally reusing
    // one initialized message across takes leaks nothing — and a
    // previously-FORGED pointer routes through the hook's interposed free
    // to a release instead of losing the only handle to its sample.
    data.bridge.release_forgeable_members(ros_message);

    // The ADOPTION RETENTION limit is
    // enforced HERE — after the reused message's own release above, and
    // before the forge — and it degrades to a COPY rather than refusing.
    //
    // Putting this check before the receive would wedge the very
    // pattern the release above exists to support. For a caller reusing one
    // message buffer, the sample it is holding is released BY THIS CALL; a
    // gate that refuses first therefore never lets that release run, so
    // `outstanding` never falls and every later take is refused too —
    // permanently, and deterministically at `BUDGET=1`. It would also contradict
    // the documented self-heal, because for such a caller the free IS the
    // refused call.
    //
    // SCOPE: this gate runs
    // AFTER the borrow-consuming receive, so it can only choose adopt-vs-copy
    // for a borrow iceoryx2 has ALREADY granted; it cannot pre-empt an
    // `ExceedsMaxBorrows` on the receive itself. A reusing caller therefore
    // still wedges when the SERVICE's own ceiling is what blocks the receive
    // that would have run the release above — reachable only on a PRE-EXISTING
    // service whose `max_borrowed_samples` is 1, since `effective_budget` is
    // read straight off the subscriber there and the create-leg floor
    // (`max(budget, 4)`) never applied. Not reachable through any service this
    // crate creates; left as a known limitation, because every candidate fix
    // (pre-flighting the borrow, or releasing before the receive) re-opens one
    // of the holes this ordering closes.
    //
    // Note also that at the DEFAULT budget this gate is unreachable:
    // `retention_limit == effective_budget`, and the receive already fails at
    // `outstanding == effective_budget`, so the copy-degrade arm below is live
    // only when `budget < effective_budget` (a small
    // `CERULION_RMW_ADOPT_TAKE_BUDGET`, or a larger pre-existing service).
    //
    // Checking after the release fixes the wedge (the reused sample is
    // already back). Serving by COPY rather than refusing fixes the other
    // half: we hold a frame at this point, so refusing would mean dropping
    // it (data loss) and emptying the caller's message on a call that
    // reports nothing taken. A copy honours the budget exactly — nothing new
    // is retained — delivers the frame, and is a path this function already
    // has: `forged == 0` is the "nothing adoptable" arm, which drops the
    // sample, counts a fallback and serves `Copied`.
    let retention_limit = adopt.budget.min(adopt.effective_budget);
    let over_retention =
        adopt.stats.outstanding.load(Ordering::Relaxed) as usize >= retention_limit;
    let outcome = if over_retention {
        let body = &owned.payload()[WireHeader::SIZE..];
        if !data.bridge.unflatten(body, ros_message) {
            let body_len = body.len();
            crate::decode_failure_latch::report_decode_failure(
                &data.decode_failures,
                crate::decode_failure_latch::DecodeSite::Subscription,
                &data.topic,
                &data.type_name,
                body_len,
            );
            return RMW_RET_OK;
        }
        // Nothing forged, nothing retained: the arm below drops the sample,
        // counts the fallback and labels the take `Copied`.
        crate::type_bridge::ForgeOutcome::default()
    } else {
        let body = &owned.payload()[WireHeader::SIZE..];
        match data.bridge.unflatten_forged(body, ros_message) {
            Ok(outcome) => outcome,
            Err(_partial) => {
                // Mirror the plain path's decode-failure handling:
                // frame consumed + dropped, `taken` stays false. Below-floor
                // copies made before the failure live in the caller's
                // message and its `fini` owns them.
                let body_len = body.len();
                crate::decode_failure_latch::report_decode_failure(
                    &data.decode_failures,
                    crate::decode_failure_latch::DecodeSite::Subscription,
                    &data.topic,
                    &data.type_name,
                    body_len,
                );
                return RMW_RET_OK;
            }
        }
    };
    // From here until the registrations
    // complete, the caller's message carries pointers into the held sample
    // — any panic (latch reporting, Vec growth, Arc::new, the registration
    // loop) must un-forge before `ffi_guard` reports the error, or the
    // caller's later `fini` frees dangling SHM. Armed even for a zero mask
    // (its Drop is then a no-op) so the flow has exactly one shape.
    //
    // On the UNWIND path the sample is released BEFORE this
    // un-forge (locals drop in reverse declaration order, and `adopted`
    // below is declared later) — the inverse of the planned rollback's
    // order. That is safe, and only because `unforge` WRITES empty headers
    // and never dereferences the sample; do not add a read there.
    let mut forged_guard = ForgedMessageGuard {
        bridge: &data.bridge,
        msg: ros_message,
        mask: outcome.forged,
        armed: true,
    };
    // The forge
    // reporters speak ONLY for a take that ATTEMPTED a forge. The
    // over-retention arm decodes with the plain copying `unflatten` and never
    // looks at any entry's placement, so its `below_floor == 0` means NOT
    // MEASURED, not "measured clean" — and `report_forge_clean` is
    // `observe_success` on the below-floor latch: it CLOSES an open regime,
    // re-arms it, and prints "frames place their sequences at or above the
    // data floor again", a claim about the PRODUCER's wire layout. On a topic
    // whose publisher really does place sequences below the floor, every
    // over-retention take would emit that false recovery and re-arm, so the
    // next genuinely-forging take prints a fresh LOUD head instead of the
    // suppressed repeat it is — a flood in exactly the regime the latch
    // exists to suppress, alternating at frame rate while the app sits at its
    // retention limit. The same rule applies to
    // `report_adopt_registration_clean` (a recovery is reported only for a
    // condition this call actually tested) and to the ordering of
    // the fallback report; the over-retention arm is routed past both.
    if !over_retention {
        if outcome.below_floor > 0 {
            crate::loan_refusal_latch::report_forge_fallback(
                &data.forge_fallbacks,
                &data.topic,
                outcome.below_floor,
                data.bridge.forged_sequence_count(),
                data.bridge.layout().data_floor(),
            );
        } else {
            crate::loan_refusal_latch::report_forge_clean(&data.forge_fallbacks, &data.topic);
        }
    }

    // The outcome → label table is decided by
    // the branch that actually ran rather than by "we are on the adopt
    // path". Reporting `Adopted` for every successful take, including
    // the two that serve by COPY, would tell an operator the zero-copy
    // path was working on frames that paid a full copy.
    //
    // The rule this table is keyed on: the counters and the label agree.
    // Labelling the zero-ranges case `Copied` while still
    // counting it in `adopted_takes` and NOT in `fallbacks` would let the
    // summary read `adopted == takes` while the served label said a
    // copy happened. The counters and the label answer ONE question,
    // the one `AdoptStats::adopted_takes` is documented by and the bench
    // citability gate rests on: DID THIS TAKE PAY A PAYLOAD COPY?
    //
    //   registrations succeeded, >= 1 range  -> Adopted  (adopted_takes++)
    //   forged mask set, ZERO ranges         -> Adopted  (adopted_takes++)
    //        nothing to copy and nothing to hold: zero copies paid, which
    //        is what the counter means and what the label must agree with
    //   registration failed, copy served     -> Copied   (fallbacks++)
    //   nothing adoptable (forged == 0)      -> Copied   (fallbacks++)
    //   copy failed                          -> no report (not served)
    let served: LoanServed;
    // Set on the adopted path, acted on inside the guarded tail.
    let mut report_registration_clean = false;
    if outcome.forged != 0 {
        // Per-forged-member exact ranges (the hook's granularity contract —
        // never the enclosing sample), walked into the subscription's
        // reusable scratch (the steady-state
        // adopted take is a hot path — cleared + refilled under the one
        // mutex, capacity high-water, so no per-take Vec growth).
        let inner_state = &mut *inner;
        inner_state.adopt_ranges.clear();
        data.bridge
            .forged_entry_ranges(ros_message, outcome.forged, &mut inner_state.adopt_ranges);
        // hot-path-alloc-ok: ONE allocation per ADOPTED take,
        // and it is STRUCTURAL — the sample needs shared ownership whose
        // clones (the registration cookies) outlive this call on whatever
        // thread the app frees from, which is exactly `Arc`'s contract; a
        // pooled refcount block would re-implement Arc's cross-thread
        // release race. Pinned at EXACTLY 1 by `rmw_adopt_zero_alloc_test`;
        // amortizable via a slab if measured to matter.
        let adopted = Arc::new(AdoptedSample::new(owned, Arc::clone(&adopt.stats)));
        // The injected-panic seam sits INSIDE the guarded
        // window (post-forge, pre-registration) — a regression test drives
        // the unwind path through it.
        #[cfg(feature = "test-seams")]
        crate::test_seams::maybe_panic_in_adopt_forge_window();
        // The shutdown-RACE seam — it CLEARS the release
        // callback right here, between the dispatch's epoch check and the
        // registrations below, which is exactly the interleaving a concurrent
        // `rmw_shutdown` produces.
        #[cfg(feature = "test-seams")]
        crate::test_seams::maybe_clear_release_callback_in_adopt_forge_window();
        // Its opposite — a concurrent CREATE re-installing the
        // same callback here must be invisible to the epoch re-check below.
        #[cfg(feature = "test-seams")]
        crate::test_seams::maybe_reinstall_release_callback_in_adopt_forge_window();
        inner_state.adopt_registered.clear();
        let mut failed_rc: Option<i32> = None;
        for &(ptr, len) in &inner_state.adopt_ranges {
            // One Arc clone per registration — the cookie IS the refcount
            // unit, and the hook fires the callback at most once per
            // registration, by cookie identity.
            let cookie = Arc::into_raw(Arc::clone(&adopted)) as usize;
            let rc = (api.register_segment)(ptr as *mut c_void, len, cookie);
            if rc == RC_OK {
                inner_state.adopt_registered.push((ptr, cookie));
            } else {
                // Reclaim THIS registration's clone; roll back below.
                drop(Arc::from_raw(cookie as *const AdoptedSample));
                failed_rc = Some(rc);
                break;
            }
        }
        // A race: the registrations are only
        // good if the callback that was there when we checked is STILL the one
        // installed. A `rmw_shutdown` between the dispatch's check and this
        // point would leave these ranges registered with a hook that has
        // nothing to call, so the app's free would leak every cookie and pin
        // the sample for the life of the process — the shutdown leak, in its
        // residual window. Re-reading the epoch closes that window without a
        // lock: the withdrawal below needs no callback (`unregister_segment`
        // is the caller-side half), and the app cannot have freed anything
        // yet, because this take has not returned its message.
        let epoch_lost =
            failed_rc.is_none() && crate::adopt_take::release_callback_epoch() != callback_epoch;
        if failed_rc.is_some() || epoch_lost {
            // All-or-nothing at the registration stage: unregister the
            // siblings already made (unregister never fires the callback —
            // it is the caller-side withdrawal), reclaim their clones, then
            // serve the take by copy.
            // Read the count BEFORE
            // the withdrawal clears the list. Reading `.len()` after the
            // `clear()` below would make
            // `registered_before_failure` ALWAYS 0 — which erases exactly
            // the distinction the field exists for: a PARTIAL failure (some
            // siblings registered, then a break, i.e. a stale leaked
            // registration) versus a wholly bad range set. The same test
            // reports `=1` with the read here and `=0` with the read after
            // the `clear()`.
            let registered_before_failure = inner_state.adopt_registered.len();
            withdraw_adopt_registrations(&api, &inner_state.adopt_registered, &data.topic);
            inner_state.adopt_registered.clear();
            // Un-forge the caller's message BEFORE the sample can be
            // released (the last Arc drop below), then copy the masked
            // members — the same bytes the copying take serves. Consumes
            // the unwind guard: the planned rollback and a panic run the
            // SAME cleanup routine.
            forged_guard.unforge_now();
            let copied = {
                let body = &adopted.payload()[WireHeader::SIZE..];
                data.bridge
                    .copy_forged_members(body, ros_message, outcome.forged)
            };
            drop(adopted); // last clone — releases the sample (after the un-forge above)
            if !copied {
                // Allocation failure on the copy — the frame cannot be
                // served: the plain path's decode-failure handling.
                // The registration-fallback
                // report must not fire ABOVE this branch, else a take that
                // could not be served at all would still be announced as
                // "served by copy instead". A report is made AFTER the
                // outcome it describes is known, never before.
                crate::decode_failure_latch::report_decode_failure(
                    &data.decode_failures,
                    crate::decode_failure_latch::DecodeSite::Subscription,
                    &data.topic,
                    &data.type_name,
                    0,
                );
                return RMW_RET_OK;
            }
            match failed_rc {
                Some(rc) => crate::loan_refusal_latch::report_adopt_registration_fallback(
                    &adopt.registration_failures,
                    &data.topic,
                    rc,
                    registered_before_failure,
                    inner_state.adopt_ranges.len(),
                ),
                // The hook refused nothing — a shutdown landed mid
                // take. Same remedy, same once-per-process line as the
                // dispatch-level degrade, because it is the same condition
                // seen one step later.
                None => warn_adopt_degraded_after_shutdown(&data.topic),
            }
            served = LoanServed::Copied;
        } else {
            // Registrations complete — but the guard stays ARMED through
            // every reporter below: a panic
            // in a recovery log must still un-forge the caller's message.
            // The tail disarm is the one stand-down point.
            // A recovery is only meaningful
            // if a registration ACTUALLY happened. An all-empty forged
            // mask registers zero ranges, so it never asked the hook for
            // anything — reporting it clean would close a registration
            // -failure regime that nothing tested, re-arm the latch, and
            // make the next genuine failure a fresh loud head instead of
            // the suppressed repeat it is. An empty `/scan` on a topic
            // whose registrations are failing would do exactly that, once
            // per frame.
            // `Adopted` either way — this arm increments
            // `adopted_takes` and no fallback, and the label answers the
            // same question those counters do (zero payload copies paid).
            // The registration REPORT below still keys on a real
            // registration, which is a different question.
            served = LoanServed::Adopted;
            // The REPORT is deferred into the tail's guarded window rather
            // than made here. At this point the registrations are LIVE and
            // their `Arc` cookies hold the sample, so a panic in this
            // reporter — a host `tracing` subscriber's `on_event`, the same
            // hazard class the report-window seam exists to exercise — would
            // un-forge the caller's message through `ForgedMessageGuard::drop`
            // and never reach `withdraw_adopt_registrations`. The sample
            // would never be released and one borrow-budget slot would be
            // lost for the life of the process; `budget` such panics exhaust
            // the subscription. Every panic-capable reporter on the adopted
            // path sits inside the ONE window whose `Err` arm pairs the
            // un-forge with the withdrawal.
            report_registration_clean = !inner_state.adopt_registered.is_empty();
            // Drop the construction reference; the registrations hold
            // theirs. With ZERO ranges registered (every forged entry
            // empty) this releases the sample immediately — correct:
            // nothing aliases it, and the app's frees are rosidl no-ops.
            drop(adopted);
        }
    } else {
        // Nothing adoptable this frame — every forgeable entry was copied.
        // The sample drops here, byte-identical to the copy path. The
        // guard holds a zero mask (its Drop and the tail disarm are both
        // no-op-safe); it stands down at the ONE tail point with everyone
        // else.
        drop(owned);
        served = LoanServed::Copied;
    }

    // The tail's
    // panic-capable reporters run inside a catch so the rollback can be
    // COMPLETE. `ForgedMessageGuard` alone empties the caller's headers,
    // which is what makes the message `fini`-safe — and is exactly what
    // leaves the app unable to free the registered ranges, so on the
    // adopted path the sample would stay pinned for the process's life.
    // The two halves are paired HERE, in the order the planned rollback
    // already uses: un-forge first, release second.
    let tail = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        // The residual's seam — a clear landing HERE is past every
        // check this function makes, which is the whole point of the residual.
        #[cfg(feature = "test-seams")]
        crate::test_seams::maybe_clear_release_callback_in_adopt_report_window();
        // The registration-recovery report, deferred from the
        // adopted branch above so it is covered by this window's rollback.
        // The rule is the same — a recovery is only reported when a
        // registration ACTUALLY happened, which is what the flag carries.
        if report_registration_clean {
            crate::loan_refusal_latch::report_adopt_registration_clean(
                &adopt.registration_failures,
                &data.topic,
            );
            // The seam travels WITH this reporter — move the block
            // back outside the window and the pin moves with it and fails.
            #[cfg(feature = "test-seams")]
            crate::test_seams::maybe_panic_in_adopt_registration_report();
        }
        crate::loan_refusal_latch::report_loan_served(&data.loan_refusals, &data.topic, served);
        crate::decode_failure_latch::report_decode_success(
            &data.decode_failures,
            crate::decode_failure_latch::DecodeSite::Subscription,
            &data.topic,
        );
        // The injected-panic seam sits inside the guarded
        // window, so the regression drives the unwind path through it.
        #[cfg(feature = "test-seams")]
        crate::test_seams::maybe_panic_in_adopt_report_window();
    }));
    if let Err(payload) = tail {
        forged_guard.unforge_now();
        if matches!(served, LoanServed::Adopted) {
            let inner_state = &mut *inner;
            withdraw_adopt_registrations(&api, &inner_state.adopt_registered, &data.topic);
            inner_state.adopt_registered.clear();
        }
        std::panic::resume_unwind(payload);
    }
    // The guard stands down last, since every
    // panic-capable reporter above ran while it was still armed, so a panic
    // anywhere in this function un-forges the caller's message (fini-safe)
    // rather than leaving forged pointers behind an error return. The window
    // above pairs that un-forge with the registration withdrawal, so the
    // panic path leaves the caller holding nothing of ours AND us holding
    // nothing of the caller's — the sample is released, not pinned.
    forged_guard.disarm();
    // The adopt counters are
    // TRANSACTIONAL with delivery — every one of them moves HERE, once the
    // report window has succeeded and the take is certain to return
    // `taken = true`, and none of them moves anywhere else.
    //
    // Moving any of them elsewhere breaks one of two invariants.
    // Bumping `adopted_takes`/`fallbacks` in the branch that
    // decides the outcome and `takes` inside the report window means a panic in
    // a reporter (a hostile `tracing::Subscriber::on_event` — the hazard the
    // report deferral and this window's seam exist for) leaves `adopted_takes`
    // incremented and `takes` not: `adopted > takes`, inverting the invariant
    // the adopt-take summary and the bench citability gate rest on, for the
    // life of the subscription and into the retired aggregate. Bumping
    // `takes` first instead makes the three consistent — and consistently
    // WRONG, because the unwind returns `RMW_RET_ERROR` with `taken = false`,
    // so all three then count a delivery that never happened against a
    // counter documented as "successful takes".
    //
    // Counting after the window satisfies both readings at once: a take that
    // unwinds contributes NOTHING anywhere, and every take that is counted was
    // served. `served` already carries which arm ran, so the outcome split
    // stays exactly where it was decided.
    adopt.stats.takes.fetch_add(1, Ordering::Relaxed);
    match served {
        LoanServed::Adopted => adopt.stats.adopted_takes.fetch_add(1, Ordering::Relaxed),
        // Both copy arms — a registration failure that fell back, and a frame
        // with nothing adoptable in it.
        _ => adopt.stats.fallbacks.fetch_add(1, Ordering::Relaxed),
    };
    drop(inner);
    *taken = true;
    if !message_info.is_null() {
        // Identical fill to `take_impl`.
        let mut info: ffi::rmw_message_info_t = std::mem::zeroed();
        info.source_timestamp = ts as i64;
        info.received_timestamp = match runtime::runtime() {
            Ok(rt) => rt.transport.clock().now_ns() as i64,
            Err(_) => 0,
        };
        info.publication_sequence_number = seq;
        info.reception_sequence_number = u64::MAX;
        info.publisher_gid.implementation_identifier = ffi::implementation_identifier_ptr();
        info.from_intra_process = false;
        *message_info = info;
    }
    RMW_RET_OK
}

/// # Safety
/// rmw ABI contract: `ros_message` is an initialized message of the
/// subscription's type.
#[no_mangle]
pub unsafe extern "C" fn rmw_take(
    subscription: *const ffi::rmw_subscription_t,
    ros_message: *mut c_void,
    taken: *mut bool,
    _allocation: *mut ffi::rmw_subscription_allocation_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        take_impl(subscription, ros_message, taken, std::ptr::null_mut())
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_take_with_info(
    subscription: *const ffi::rmw_subscription_t,
    ros_message: *mut c_void,
    taken: *mut bool,
    message_info: *mut ffi::rmw_message_info_t,
    _allocation: *mut ffi::rmw_subscription_allocation_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        take_impl(subscription, ros_message, taken, message_info)
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_take_sequence(
    subscription: *const ffi::rmw_subscription_t,
    count: usize,
    message_sequence: *mut ffi::rmw_message_sequence_t,
    message_info_sequence: *mut ffi::rmw_message_info_sequence_t,
    taken: *mut usize,
    _allocation: *mut ffi::rmw_subscription_allocation_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if subscription.is_null()
            || message_sequence.is_null()
            || message_info_sequence.is_null()
            || taken.is_null()
        {
            return RMW_RET_INVALID_ARGUMENT;
        }
        *taken = 0;
        let seq = &mut *message_sequence;
        let info_seq = &mut *message_info_sequence;
        if seq.capacity < count || info_seq.capacity < count {
            return RMW_RET_INVALID_ARGUMENT;
        }
        for i in 0..count {
            let msg = *seq.data.add(i);
            let mut took = false;
            let ret = take_impl(subscription, msg, &mut took, info_seq.data.add(i));
            if ret != RMW_RET_OK {
                // Earlier iterations already unflattened into the
                // caller's slots — sizes must stay consistent with
                // *taken even on error.
                seq.size = *taken;
                info_seq.size = *taken;
                return ret;
            }
            if !took {
                break;
            }
            *taken += 1;
        }
        seq.size = *taken;
        info_seq.size = *taken;
        RMW_RET_OK
    })
}

/// The zero-copy loaned TAKE — the read-side mirror of
/// `rmw_borrow_loaned_message`. Receives ONE owned iceoryx2 sample off the
/// SAME queue receive as `take_impl` (`try_receive_one_owned` — the
/// one-read-path decision: no bypass read plane), validates it BEFORE any
/// pointer escapes, then holds the sample in the subscription's
/// `pending_takes` table and hands rclcpp a pointer DIRECTLY into the SHM
/// payload (past the 32-byte `WireHeader`). Zero copies, zero payload allocs.
///
/// The held sample is what makes the pointer sound: iceoryx2's borrow
/// accounting pins the publisher-pool slot until the `Sample` drops (the pool
/// is sized with a `subscriber_max_borrowed_samples` term per subscriber slot
/// for exactly this), so the bytes behind the loaned pointer can never be
/// reclaimed or overwritten by a later publish while the loan is
/// outstanding. `rmw_return_loaned_message_from_subscription` drops it.
///
/// Two shapes are served, gated by `bridge.can_loan_take()`:
///
/// - a PLAIN (recursively-fixed) type — `bridge.can_loan()`, the same gate as
///   the publish-side loan: the C struct IS the wire fixed section,
///   byte-for-byte (pinned by `layout_equivalence_test` over all 254 vendored
///   types), so the SHM payload pointer itself is handed out;
/// - a type carrying at least one FORGEABLE primitive
///   sequence (`Image.data`, `LaserScan.ranges`, …): an rmw-owned SHADOW
///   message from the subscription's pool receives the small copied
///   remainder (fixed fields, strings, nested messages) while each forgeable
///   sequence's container header is aimed at the held sample's bytes with
///   capacity == size. The shadow's address is the loaned pointer; the
///   return un-forges it before the sample is released and recycles it.
///
/// Everything else returns `RMW_RET_UNSUPPORTED` and rclcpp falls back to
/// the copying take. Both shapes share one budget arm: a take refused by
/// the shadow pool or by the transport's borrow budget is loud once per
/// regime (`loan_refusal_latch`), consumes no frame, and recovers on the
/// first served take after a return.
unsafe fn take_loaned_impl(
    subscription: *const ffi::rmw_subscription_t,
    loaned_message: *mut *mut c_void,
    taken: *mut bool,
    message_info: *mut ffi::rmw_message_info_t,
) -> rmw_ret_t {
    if subscription.is_null() || loaned_message.is_null() || taken.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    if !ffi::is_our_identifier((*subscription).implementation_identifier) {
        return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
    }
    // rmw.h (Jazzy, `rmw_take_loaned_message`): "RMW_RET_INVALID_ARGUMENT if
    // `*loaned_message` is not NULL (to prevent leaks)". A caller reusing an
    // out-slot that still holds an outstanding loan would overwrite the only
    // handle that can return the prior sample, pinning its SHM borrow for the
    // subscription's life. Refused BEFORE any receive, so no frame is
    // consumed and `*loaned_message` stays untouched (the rmw.h \post).
    if !(*loaned_message).is_null() {
        let data = &*((*subscription).data as *const SubscriptionData);
        tracing::error!(
            topic = %data.topic,
            ptr = *loaned_message as usize,
            "loaned take refused: *loaned_message is not NULL (rmw.h: INVALID_ARGUMENT \
             to prevent leaks) — return the outstanding loan first, then pass a NULL slot"
        );
        return RMW_RET_INVALID_ARGUMENT;
    }
    *taken = false;
    let data = &*((*subscription).data as *const SubscriptionData);
    if !data.bridge.can_loan_take() {
        // Neither served shape: the copying take stays (rclcpp falls back —
        // can_loan_messages was false at create, loudly logged).
        return RMW_RET_UNSUPPORTED;
    }
    // A FIXED type hands out the SHM payload pointer itself;
    // every other eligible type is served through a shadow.
    let forged = !data.bridge.can_loan();
    let expected_hash = data.bridge.schema_hash();
    let fixed_size = data.bridge.layout().fixed_size;
    // ≤ 8 by the codegen static assert (see `WireLayout::fixed_align`); the
    // `.max(1)` is pure defense against a zero from a degenerate layout.
    let fixed_align = data.bridge.layout().fixed_align.max(1);

    // Receive + validate + register under the ONE subscription lock (the
    // same confinement discipline as the publisher's pending_loans — see
    // `SubscriptionInner`). A poisoned lock means wedged.
    let Some(mut inner) = runtime::lock_unpoisoned(&data.inner) else {
        tracing::error!("entity wedged by an earlier panic; failing call");
        return RMW_RET_ERROR;
    };
    // A forged take needs its shadow BEFORE the receive: a refusal here
    // consumes NO frame, so the queued frame is served by the take that
    // follows a return. Pool exhaustion is the shadow-side face of the same
    // budget the transport enforces below — one latch, one regime.
    let mut shadow = None;
    if forged {
        match inner.shadows.acquire(&data.bridge) {
            Ok(s) => shadow = Some(s),
            Err(kind) => {
                crate::loan_refusal_latch::report_loan_refused(
                    &data.loan_refusals,
                    &data.topic,
                    kind,
                    inner.pending_takes.len(),
                    inner.shadows.capacity(),
                    "",
                );
                return RMW_RET_ERROR;
            }
        }
    }
    let owned = match inner.subscriber.try_receive_one_owned() {
        Ok(o) => o,
        Err(e) => {
            // The transport-side budget arm lands here: with
            // RMW_TAKE_LOAN_BORROW_BUDGET loans outstanding, iceoryx2 refuses
            // the next receive with `ExceedsMaxBorrows` — loud once per
            // regime and bounded (never silent loss, never a hang); returning
            // any one loan recovers. Other receive failures share the arm;
            // the reason carries the iceoryx2 cause verbatim.
            if let Some(s) = shadow.take() {
                inner.shadows.release(s);
            }
            crate::loan_refusal_latch::report_loan_refused(
                &data.loan_refusals,
                &data.topic,
                crate::loan_refusal_latch::LoanRefusal::ReceiveFailed,
                inner.pending_takes.len(),
                RMW_TAKE_LOAN_BORROW_BUDGET,
                &e.to_string(),
            );
            return RMW_RET_ERROR;
        }
    };
    let Some(owned) = owned else {
        // Empty queue — `taken` stays false; the shadow goes back unused.
        if let Some(s) = shadow.take() {
            inner.shadows.release(s);
        }
        return RMW_RET_OK;
    };
    // `try_receive_one_owned` only returns frames whose header parses and
    // whose total_size is in bounds, so this re-read cannot fail; a None
    // here would mean the frame changed under us — fail loudly, never
    // fabricate.
    let Some(header) = owned.wire_header() else {
        if let Some(s) = shadow.take() {
            inner.shadows.release(s);
        }
        tracing::error!(topic = %data.topic, "loaned take: header re-read failed; dropping frame");
        return RMW_RET_ERROR;
    };
    // The pure gate (see `take_gate`).
    if crate::take_gate::schema_hash_verdict(header.schema_hash, expected_hash).is_mismatch() {
        // Same regime latch as `take_impl` — a hash mismatch is a type SKEW
        // and fails every frame until somebody redeploys. The
        // frame is consumed and dropped; `taken` stays false (mirroring the
        // copying take's semantics exactly).
        if let Some(s) = shadow.take() {
            inner.shadows.release(s);
        }
        let mut latch = cerulion_core::transport::failure_regime_latch::lock_regime_latch(
            &data.hash_mismatches,
        );
        cerulion_core::transport::frame_drop_latch::report_schema_hash_mismatch(
            &mut latch,
            cerulion_core::transport::frame_drop_latch::FrameDropSite::Message,
            &data.topic,
            expected_hash,
            header.schema_hash,
        );
        return RMW_RET_OK;
    }
    {
        // Recovery is observed HERE — the hash gate passing is the whole
        // condition (same split as `take_impl`: framing is a different
        // condition with its own latch).
        let mut latch = cerulion_core::transport::failure_regime_latch::lock_regime_latch(
            &data.hash_mismatches,
        );
        cerulion_core::transport::frame_drop_latch::report_schema_hash_match(
            &mut latch,
            cerulion_core::transport::frame_drop_latch::FrameDropSite::Message,
            &data.topic,
        );
    }
    let ts = header.timestamp_ns;
    let seq = header.sequence as u64;
    let loaned_ptr: *mut c_void = if let Some(mut shadow) = shadow {
        // FORGED shape. The bridge validates the frame (entries in bounds,
        // element-multiple lengths, element-aligned forge targets) and
        // fills the shadow all-or-nothing; a malformed frame is a framing
        // disagreement — consumed + dropped through the copying take's
        // undecodable-frame latch, `taken` false, never a pointer, and the
        // shadow (left un-forged by the bridge) goes back to the pool. An
        // entry BELOW the data floor is not malformed but is never forged:
        // that member is copied (the same bytes the copying take serves),
        // the take succeeds, and the fallback latch tells the operator the
        // producer places entries where the wire forbids them.
        let body = &owned.payload()[WireHeader::SIZE..];
        let outcome = match data.bridge.unflatten_forged(body, shadow.as_ptr()) {
            Ok(outcome) => outcome,
            Err(partial) => {
                let body_len = body.len();
                crate::decode_failure_latch::report_decode_failure(
                    &data.decode_failures,
                    crate::decode_failure_latch::DecodeSite::Subscription,
                    &data.topic,
                    &data.type_name,
                    body_len,
                );
                // Nothing forged survives a failed decode, but COPIES made
                // before the failure do — recording the partial outcome
                // taints the shadow so `release` RETIRES it (its `fini` frees
                // the copies) instead of recycling it under the next forge.
                shadow.record(&partial);
                inner.shadows.release(shadow);
                return RMW_RET_OK;
            }
        };
        shadow.record(&outcome);
        if outcome.below_floor > 0 {
            crate::loan_refusal_latch::report_forge_fallback(
                &data.forge_fallbacks,
                &data.topic,
                outcome.below_floor,
                data.bridge.forged_sequence_count(),
                data.bridge.layout().data_floor(),
            );
        } else {
            crate::loan_refusal_latch::report_forge_clean(&data.forge_fallbacks, &data.topic);
        }
        // Hand-out: the SHADOW is the loaned message. Its forged sequences
        // alias the held sample, whose SHM mapping is READ-ONLY on the
        // subscriber side (iceoryx2 opens data segments `AccessMode::Read`):
        // a caller writing through a forged `data()` faults LOUDLY, never
        // silently and never into another subscriber's view — the documented
        // loaned-take contract (message valid only inside the callback,
        // read-only). The shadow's own fields are ordinary heap memory.
        let ptr = shadow.as_ptr();
        inner.pending_takes.push(runtime::PendingTake {
            key: ptr as usize,
            shadow: Some(shadow),
            sample: owned,
        });
        ptr
    } else {
        // FIXED shape. Framing gate, BEFORE the pointer escapes. A plain
        // type's frame is EXACTLY WireHeader + fixed section, and its header
        // carries NO offset-table entries: `offset_table_count == 0` is the
        // invariant that makes the body a bare C struct at all (a nonzero
        // count claims variable fields, i.e. a body that is NOT the struct —
        // the copying take would decode it as such, and rclcpp reading it as
        // `T` is garbage past the fixed section). `offset_table_offset` is
        // inert with an empty table, but an offset pointing PAST the frame
        // is malformed regardless, so it is bounds-checked; it is NOT pinned
        // to one value because the two producers spell an empty table
        // differently (the native publisher writes `WireHeader::SIZE +
        // fixed_size`, frame-relative; the rmw flatten writes `fixed_size`)
        // and both are legitimate fixed frames.
        //
        // This gate is REACHABLE: `rmw_publish_serialized_message` accepts
        // any frame whose header parses and whose hash matches, so a bag
        // player can put a same-hash frame with nonzero offset metadata on
        // the wire. Anything failing here is a framing disagreement —
        // consumed + dropped through the copying take's undecodable-frame
        // latch, `taken` false, never a pointer.
        let frame = owned.payload();
        let body_len = frame.len() - WireHeader::SIZE; // >= 0: total_size >= SIZE validated
        if body_len != fixed_size
            || header.offset_table_count != 0
            || header.offset_table_offset as usize > frame.len()
        {
            crate::decode_failure_latch::report_decode_failure(
                &data.decode_failures,
                crate::decode_failure_latch::DecodeSite::Subscription,
                &data.topic,
                &data.type_name,
                body_len,
            );
            return RMW_RET_OK;
        }
        // Alignment gate (fail closed — never hand out a misaligned struct
        // pointer). Expected to ALWAYS hold on iceoryx2 0.9.1: the sample
        // header is 40 B @ align 8 (measured — see `IOX2_SAMPLE_HEADER_BYTES`
        // in cerulion_core) and a `[u8]` payload has align 1, so the payload
        // starts at chunk+40 of an 8-aligned chunk ⇒ 8-aligned; +32
        // (WireHeader) keeps it, and `fixed_align <= 8` by the codegen static
        // assert. But that is DE FACTO, not an iceoryx2 API guarantee, and
        // the service builder's `.payload_alignment()` override is
        // deliberately NOT used (it would change the slot layout the
        // `iceoryx2_slot_bytes` closed form documents as exact for
        // exactly-this builder shape) — so THIS assert is the guard: an
        // iceoryx2 bump that moves the payload offset fails loudly here
        // instead of handing rclcpp UB.
        let ros_ptr = frame.as_ptr().add(WireHeader::SIZE);
        if !(ros_ptr as usize).is_multiple_of(fixed_align) {
            tracing::error!(
                topic = %data.topic,
                ptr = ros_ptr as usize,
                fixed_align,
                "loaned take: SHM payload pointer is misaligned for the message \
                 struct — refusing to hand it out (the iceoryx2 payload offset \
                 moved?)"
            );
            return RMW_RET_ERROR;
        }
        // Hand-out. The pointer is `*mut` because the rmw ABI is non-const,
        // but the subscriber-side loan contract is read-only: the mapping IS
        // read-only (iceoryx2 opens subscriber data segments
        // `AccessMode::Read`), so a caller writing through it faults loudly
        // rather than corrupting the frame every other subscriber reads —
        // inherent to the rmw loan ABI, not specific to this implementation.
        inner.pending_takes.push(runtime::PendingTake {
            key: ros_ptr as usize,
            shadow: None,
            sample: owned,
        });
        ros_ptr as *mut c_void
    };
    // A served take closes any open refusal regime (recovery is logged once,
    // only if something was suppressed).
    //
    // This reporter runs
    // with the loan ALREADY tracked in `pending_takes` — both arms above
    // pushed it — but BEFORE `*loaned_message` and `*taken` are written. A
    // panic here (a host `tracing` subscriber's `on_event`) would therefore leave
    // the sample borrowed and the slot consumed while `ffi_guard` returns
    // `RMW_RET_ERROR` and the caller never receives the pointer: with no
    // pointer, it can never call `rmw_return_loaned_message_from_subscription`
    // for it, so that loan would be pinned for the life of the process — one slot
    // per panic. This is the same shape as on the adopted
    // path.
    //
    // The rollback keeps the same invariant as that path: on failure
    // the caller holds nothing of ours and we hold nothing of the caller's.
    // Dropping the `PendingTake` releases the borrow (and its shadow).
    let served = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        crate::loan_refusal_latch::report_loan_served(
            &data.loan_refusals,
            &data.topic,
            crate::loan_refusal_latch::LoanServed::Loaned,
        );
        // The seam travels WITH this reporter, inside the guarded window.
        #[cfg(feature = "test-seams")]
        crate::test_seams::maybe_panic_in_loaned_serve_report();
    }));
    if let Err(payload) = served {
        let key = loaned_ptr as usize;
        inner.pending_takes.retain(|p| p.key != key);
        std::panic::resume_unwind(payload);
    }
    *loaned_message = loaned_ptr;
    *taken = true;
    if !message_info.is_null() {
        // Identical fill to `take_impl`.
        let mut info: ffi::rmw_message_info_t = std::mem::zeroed();
        info.source_timestamp = ts as i64;
        info.received_timestamp = match runtime::runtime() {
            Ok(rt) => rt.transport.clock().now_ns() as i64,
            Err(_) => 0,
        };
        info.publication_sequence_number = seq;
        info.reception_sequence_number = u64::MAX;
        info.publisher_gid.implementation_identifier = ffi::implementation_identifier_ptr();
        info.from_intra_process = false;
        *message_info = info;
    }
    RMW_RET_OK
}

/// Zero-copy loaned take — see `take_loaned_impl` (private; not
/// linked to keep the rustdoc gate green).
///
/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_take_loaned_message(
    subscription: *const ffi::rmw_subscription_t,
    loaned_message: *mut *mut c_void,
    taken: *mut bool,
    _allocation: *mut ffi::rmw_subscription_allocation_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        take_loaned_impl(subscription, loaned_message, taken, std::ptr::null_mut())
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_take_loaned_message_with_info(
    subscription: *const ffi::rmw_subscription_t,
    loaned_message: *mut *mut c_void,
    taken: *mut bool,
    message_info: *mut ffi::rmw_message_info_t,
    _allocation: *mut ffi::rmw_subscription_allocation_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        take_loaned_impl(subscription, loaned_message, taken, message_info)
    })
}

/// Return a loaned take — pop the held sample by its loaned pointer
/// and drop it, releasing the iceoryx2 borrow (the publisher-pool slot and
/// the `subscriber_max_borrowed_samples` unit).
///
/// # Safety
/// `loaned_message` must come from `rmw_take_loaned_message[_with_info]` on
/// this subscription.
#[no_mangle]
pub unsafe extern "C" fn rmw_return_loaned_message_from_subscription(
    subscription: *const ffi::rmw_subscription_t,
    loaned_message: *mut c_void,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if subscription.is_null() || loaned_message.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*subscription).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        let data = &*((*subscription).data as *const SubscriptionData);
        if !data.bridge.can_loan_take() {
            // A loan can never exist for a non-loanable subscription —
            // mirror the take side's gate.
            return RMW_RET_UNSUPPORTED;
        }
        let key = loaned_message as usize;
        // A poisoned lock means wedged: never re-enter torn iceoryx2 state.
        let Some(mut inner) = runtime::lock_unpoisoned(&data.inner) else {
            tracing::error!("entity wedged by an earlier panic; failing call");
            return RMW_RET_ERROR;
        };
        match inner.pending_takes.iter().position(|t| t.key == key) {
            Some(pos) => {
                let runtime::PendingTake { shadow, sample, .. } =
                    inner.pending_takes.swap_remove(pos);
                // Ordering, load-bearing: the shadow is
                // UN-FORGED (inside `release`) and back in the pool BEFORE
                // the sample's SHM borrow is released — nothing in the rmw
                // references the frame once the transport may reclaim it.
                if let Some(shadow) = shadow {
                    inner.shadows.release(shadow);
                }
                drop(sample);
                RMW_RET_OK
            }
            None => {
                tracing::error!(
                    topic = %data.topic,
                    ptr = key,
                    "return of unknown loaned message — not an \
                     outstanding take loan on this subscription"
                );
                RMW_RET_INVALID_ARGUMENT
            }
        }
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_subscription_count_matched_publishers(
    subscription: *const ffi::rmw_subscription_t,
    publisher_count: *mut usize,
) -> rmw_ret_t {
    // ffi_guard: same panic-capable iceoryx2 probe as
    // rmw_publisher_count_matched_subscriptions; rcl_action
    // availability polls this in a loop.
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if subscription.is_null() || publisher_count.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*subscription).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        let data = &*((*subscription).data as *const SubscriptionData);
        // Transport-level — see rmw_publisher_count_matched_subscriptions.
        // rcl_action_server_is_available gates on THIS count for the
        // action's feedback/status topics (with a
        // process-local registry alone, cross-process action clients would never see
        // the server).
        let count = match runtime::runtime() {
            Ok(rt) => rt.transport.topic_publisher_count(&data.topic),
            Err(_) => 0,
        };
        *publisher_count = count;
        RMW_RET_OK
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_subscription_get_actual_qos(
    subscription: *const ffi::rmw_subscription_t,
    qos: *mut ffi::rmw_qos_profile_t,
) -> rmw_ret_t {
    if subscription.is_null() || qos.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    *qos = default_actual_qos();
    RMW_RET_OK
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_subscription_set_content_filter(
    _subscription: *mut ffi::rmw_subscription_t,
    _options: *const ffi::rmw_subscription_content_filter_options_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_subscription_get_content_filter(
    _subscription: *const ffi::rmw_subscription_t,
    _allocator: *mut ffi::rcutils_allocator_t,
    _options: *mut ffi::rmw_subscription_content_filter_options_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_init_subscription_allocation(
    _type_support: *const ffi::rosidl_message_type_support_t,
    _message_bounds: *const ffi::rosidl_runtime_c__Sequence__bound,
    _allocation: *mut ffi::rmw_subscription_allocation_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_fini_subscription_allocation(
    _allocation: *mut ffi::rmw_subscription_allocation_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED
}

/// # Safety
/// Caller frees with `rmw_subscription_free`.
#[no_mangle]
pub unsafe extern "C" fn rmw_subscription_allocate() -> *mut ffi::rmw_subscription_t {
    Box::into_raw(Box::new(std::mem::zeroed::<ffi::rmw_subscription_t>()))
}

/// # Safety
/// `subscription` must come from `rmw_subscription_allocate`.
#[no_mangle]
pub unsafe extern "C" fn rmw_subscription_free(subscription: *mut ffi::rmw_subscription_t) {
    if !subscription.is_null() {
        drop(Box::from_raw(subscription));
    }
}

// =====================================================================
// Shared helpers
// =====================================================================

pub(crate) fn ros_graph_type_name(qualified: &str) -> String {
    match qualified.split_once('/') {
        Some((pkg, name)) => format!("{pkg}/msg/{name}"),
        None => qualified.to_string(),
    }
}

/// Own a NUL-terminated copy of a ROS name for an entity-lifetime C
/// string; reclaimed by [`unleak_ros_name`] in the destroy fn.
pub(crate) fn leak_ros_name(name: &str) -> *const c_char {
    let mut buf = Vec::with_capacity(name.len() + 1);
    buf.extend_from_slice(name.as_bytes());
    buf.push(0);
    Box::into_raw(buf.into_boxed_slice()) as *const c_char
}

/// # Safety
/// `ptr` must come from [`leak_ros_name`] (or be null) and must not be
/// used afterwards.
pub(crate) unsafe fn unleak_ros_name(ptr: *const c_char) {
    if ptr.is_null() {
        return;
    }
    // Reconstruct the `Box`: length = strlen + 1.
    let len = std::ffi::CStr::from_ptr(ptr).to_bytes().len() + 1;
    drop(Box::from_raw(std::ptr::slice_from_raw_parts_mut(
        ptr as *mut u8,
        len,
    )));
}

pub(crate) fn default_actual_qos() -> ffi::rmw_qos_profile_t {
    ffi::rmw_qos_profile_t {
        history: ffi::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth: 16,
        reliability: ffi::RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        durability: ffi::RMW_QOS_POLICY_DURABILITY_VOLATILE,
        deadline: ffi::rmw_time_t { sec: 0, nsec: 0 },
        lifespan: ffi::rmw_time_t { sec: 0, nsec: 0 },
        liveliness: ffi::RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
        liveliness_lease_duration: ffi::rmw_time_t { sec: 0, nsec: 0 },
        avoid_ros_namespace_conventions: false,
    }
}

// =====================================================================
// Slice-ceiling override bridge-half unit pins (pure decision — no transport, no env
// mutation; the parsed override set is handed in directly)
// =====================================================================

#[cfg(test)]
mod slice_ceiling_tests {
    use super::*;
    use crate::ffi::introspection_cpp::{CppMessageMember, CppMessageMembers};
    use crate::type_bridge::{ros_type, BridgedMessage};
    use crate::type_bridge_cpp::CppBridgedMessage;
    use cerulion_core::testing::{count_at_exclusively, debug_lines_expected, line_level};
    use std::ffi::CString;
    use tracing_test::traced_test;

    const CATCH_ALL: u32 = 128 * 1024 * 1024;
    const SMALL_TIER: u32 = 256 * 1024;

    fn leaked_cstr(s: &str) -> *const c_char {
        CString::new(s).expect("cstr").into_raw()
    }

    fn fixture_member(
        name: &str,
        type_id: u8,
        offset: u32,
    ) -> ffi::rosidl_typesupport_introspection_c__MessageMember {
        ffi::rosidl_typesupport_introspection_c__MessageMember {
            name_: leaked_cstr(name),
            type_id_: type_id,
            offset_: offset,
            ..Default::default()
        }
    }

    /// A hand-built introspection bridge (the `bridge_test.rs` fixture shape,
    /// process-lifetime leaks) — `namespace` is the rosidl `pkg__msg` form,
    /// exactly what a real typesupport carries.
    fn fixture_bridge(
        namespace: &str,
        name: &str,
        size_of: usize,
        members: Vec<ffi::rosidl_typesupport_introspection_c__MessageMember>,
    ) -> AnyBridge {
        let members = Box::leak(members.into_boxed_slice());
        let mm = Box::leak(Box::new(
            ffi::rosidl_typesupport_introspection_c__MessageMembers {
                message_namespace_: leaked_cstr(namespace),
                message_name_: leaked_cstr(name),
                member_count_: members.len() as u32,
                size_of_: size_of,
                members_: members.as_ptr(),
                ..Default::default()
            },
        ));
        // SAFETY: `mm` points at valid, leaked (process-lifetime)
        // introspection data — the rosidl typesupport contract.
        AnyBridge::C(unsafe { BridgedMessage::new(mm).expect("fixture bridge") })
    }

    /// One string member ⇒ a VARIABLE layout; 24 = the rosidl C string
    /// struct (`{data, size, capacity}`) on 64-bit.
    fn variable_bridge(namespace: &str, name: &str) -> AnyBridge {
        fixture_bridge(
            namespace,
            name,
            24,
            vec![fixture_member("label", ros_type::STRING, 0)],
        )
    }

    /// Three doubles ⇒ a recursively-FIXED layout, `fixed_size` 24.
    fn fixed_bridge(namespace: &str, name: &str) -> AnyBridge {
        fixture_bridge(
            namespace,
            name,
            24,
            vec![
                fixture_member("x", ros_type::DOUBLE, 0),
                fixture_member("y", ros_type::DOUBLE, 8),
                fixture_member("z", ros_type::DOUBLE, 16),
            ],
        )
    }

    /// A hand-built C++ introspection bridge (the `cpp_bridge_test.rs`
    /// fixture shape): one unbounded `f64` sequence ⇒ VARIABLE layout. The
    /// container function pointers are consulted only at encode time, never
    /// at construction, so the decision pins need none.
    fn cpp_variable_bridge(namespace: &str, name: &str) -> AnyBridge {
        let members = Box::leak(Box::new([CppMessageMember {
            name_: leaked_cstr("data"),
            type_id_: ros_type::DOUBLE,
            string_upper_bound_: 0,
            members_: std::ptr::null(),
            #[cfg(cerulion_has_is_key)]
            is_key_: false,
            is_array_: true,
            array_size_: 0,
            is_upper_bound_: false,
            offset_: 0,
            default_value_: std::ptr::null(),
            size_function: None,
            get_const_function: None,
            get_function: None,
            fetch_function: None,
            assign_function: None,
            resize_function: None,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer_: false,
        }]));
        let mm = Box::leak(Box::new(CppMessageMembers {
            message_namespace_: leaked_cstr(namespace),
            message_name_: leaked_cstr(name),
            member_count_: 1,
            size_of_: 24,
            #[cfg(cerulion_has_is_key)]
            has_any_key_member_: false,
            members_: members.as_ptr(),
            init_function: None,
            fini_function: None,
        }));
        // SAFETY: `mm` points at valid, leaked (process-lifetime)
        // introspection data — the rosidl typesupport contract.
        AnyBridge::Cpp(unsafe { CppBridgedMessage::new(mm).expect("cpp fixture bridge") })
    }

    /// THE KEY-FORM CONVERSION PIN: the identity fed to the core lookup is
    /// the bridge's `pkg/Type` qualified name — ALREADY the tier table's
    /// canonical key (`qualified_name_of` keeps only the package half of the
    /// rosidl `pkg__msg` namespace) — and NOT the ROS graph spelling
    /// `pkg/msg/Type` that `ros_graph_type_name` derives from it. The
    /// discriminator: `tf2_msgs/TFMessage` is SMALL-tier (256 KiB), while the
    /// graph spelling would trip the core lookup's rosidl-spelling warn and
    /// land on the 128 MiB catch-all — so feeding
    /// `ros_graph_type_name(..)` into the lookup fails the tier assert.
    #[test]
    fn the_lookup_key_is_the_bridges_pkg_type_name_not_the_ros_graph_spelling() {
        let bridge = variable_bridge("tf2_msgs__msg", "TFMessage");
        assert_eq!(bridge.qualified_name(), "tf2_msgs/TFMessage");
        assert_eq!(
            ros_graph_type_name(bridge.qualified_name()),
            "tf2_msgs/msg/TFMessage",
            "the graph spelling exists and differs — the lookup must not use it"
        );
        assert!(!bridge.layout().is_fixed(), "fixture must be variable");
        assert_eq!(
            slice_len_with_overrides(&bridge, &SliceCeilingOverrides::default()).get(),
            SMALL_TIER,
            "a table-listed type must resolve its REAL tier through the bridge \
             (the catch-all here means the lookup was fed the wrong spelling \
             or never consulted)"
        );
    }

    /// HAPPY + PRECEDENCE + DETERMINISM: the env override beats the table in
    /// both directions on a table-LISTED type (the arm that discriminates an
    /// inverted precedence: a table-first order serves SMALL here), and two
    /// identical resolutions are identical.
    #[test]
    fn an_env_override_beats_the_table_in_both_directions() {
        let bridge = variable_bridge("tf2_msgs__msg", "TFMessage");
        let widened = SliceCeilingOverrides::parse("tf2_msgs/TFMessage:1048576");
        assert_eq!(
            slice_len_with_overrides(&bridge, &widened).get(),
            1_048_576,
            "override must WIDEN past the SMALL tier"
        );
        let narrowed = SliceCeilingOverrides::parse("tf2_msgs/TFMessage:1024");
        assert_eq!(
            slice_len_with_overrides(&bridge, &narrowed).get(),
            1024,
            "override must NARROW below the SMALL tier"
        );
        // Determinism: same inputs, same answer.
        assert_eq!(
            slice_len_with_overrides(&bridge, &narrowed),
            slice_len_with_overrides(&bridge, &narrowed),
        );
    }

    /// EDGE: an empty env and a type absent from BOTH the env and the table
    /// fall back byte-identically to the blanket (the table's
    /// own catch-all IS that blanket).
    #[test]
    fn a_type_absent_from_env_and_table_keeps_the_blanket_byte_identically() {
        let bridge = variable_bridge("my_pkg__msg", "Big");
        assert_eq!(bridge.qualified_name(), "my_pkg/Big");
        let empty = SliceCeilingOverrides::parse("");
        assert!(empty.is_empty());
        assert_eq!(
            slice_len_with_overrides(&bridge, &empty),
            BRIDGED_VARIABLE_SLICE_LEN,
            "unlisted type + empty env must equal the pre-override constant"
        );
        assert_eq!(
            slice_len_with_overrides(&bridge, &SliceCeilingOverrides::default()).get(),
            CATCH_ALL
        );
    }

    /// Adversarial case: a malformed env entry does not disturb a valid sibling's
    /// resolution through the rmw decision — the sibling's override still
    /// applies, and an unrelated type still gets its table answer.
    #[test]
    fn a_malformed_entry_does_not_disturb_a_valid_siblings_resolution() {
        let overrides = SliceCeilingOverrides::parse("nonsense,tf2_msgs/TFMessage:1048576");
        let listed = variable_bridge("tf2_msgs__msg", "TFMessage");
        assert_eq!(
            slice_len_with_overrides(&listed, &overrides).get(),
            1_048_576,
            "the valid sibling override must survive the malformed neighbor"
        );
        let unrelated = variable_bridge("my_pkg__msg", "Big");
        assert_eq!(
            slice_len_with_overrides(&unrelated, &overrides).get(),
            CATCH_ALL,
            "an unrelated type must keep its table answer"
        );
    }

    /// A FIXED bridged type keeps the provably-exact `WireHeader + fixed
    /// section` bound, and an env entry naming it is IGNORED — loudly (one
    /// WARN per creation naming the type), never silently inert and never
    /// applied (an applied narrow override would refuse every publish of a
    /// frame that is ALWAYS exactly 56 bytes here).
    #[traced_test]
    #[test]
    fn a_fixed_type_keeps_its_exact_bound_and_an_override_naming_it_warns_ignored() {
        let bridge = fixed_bridge("rmw_slc__msg", "Pt");
        assert!(bridge.layout().is_fixed(), "fixture must be fixed");
        let exact = (WireHeader::SIZE + 24) as u32; // 3 doubles
        assert_eq!(
            slice_len_with_overrides(&bridge, &SliceCeilingOverrides::default()).get(),
            exact,
            "no override: exact bound, byte-identical to the pre-override bound"
        );
        let named = SliceCeilingOverrides::parse("rmw_slc/Pt:32");
        assert_eq!(
            slice_len_with_overrides(&bridge, &named).get(),
            exact,
            "an override naming a fixed type must be IGNORED, not applied"
        );
        logs_assert(|lines: &[&str]| {
            let warns =
                count_at_exclusively(lines, "WARN", &["override is IGNORED", "rmw_slc/Pt"])?;
            match warns {
                1 => Ok(()),
                n => Err(format!(
                    "expected exactly ONE ignored-override WARN naming the type \
                     (zero before the named override, one after), got {n}"
                )),
            }
        });
    }

    /// The ignored-override warn is
    /// flood-suppressed PER TYPE — N resolutions of one fixed overridden
    /// type get ONE loud head + N-1 debug repeats (a bare per-creation
    /// `warn!` fails this with N loud lines), and a second fixed type still
    /// gets its OWN loud head. Type names are unique to this test because
    /// the latch registry is process-global across the lib binary's tests.
    #[traced_test]
    #[test]
    fn the_ignored_override_warn_is_once_per_type_not_per_creation() {
        let exact = (WireHeader::SIZE + 24) as u32;
        let overrides = SliceCeilingOverrides::parse("rmw_slcflood/A:32,rmw_slcflood/B:32");
        let a = fixed_bridge("rmw_slcflood__msg", "A");
        for _ in 0..5 {
            assert_eq!(
                slice_len_with_overrides(&a, &overrides).get(),
                exact,
                "suppression must not change the sizing decision"
            );
        }
        let b = fixed_bridge("rmw_slcflood__msg", "B");
        assert_eq!(slice_len_with_overrides(&b, &overrides).get(), exact);
        logs_assert(|lines: &[&str]| {
            // The ONE shared body for the level match (read from the line
            // HEADER, never anywhere on the line). The ABSENCE sweep below
            // keeps this non-exclusive form: the exclusive one would REFUSE at
            // a level this marker never uses rather than read 0. Every positive
            // count uses `count_at_exclusively`, which also pairs the
            // level-free total.
            let count = |level: &str, needle: &str, ty: &str| {
                lines
                    .iter()
                    .filter(|l| {
                        line_level(l) == Some(level) && l.contains(needle) && l.contains(ty)
                    })
                    .count()
            };
            let a_heads =
                count_at_exclusively(lines, "WARN", &["override is IGNORED", "rmw_slcflood/A"])?;
            if a_heads != 1 {
                return Err(format!(
                    "5 resolutions of one type must produce EXACTLY 1 loud head \
                     (a bare per-creation warn produces 5), got {a_heads}"
                ));
            }
            // Level-free, and FIRST: a suppressed repeat must never be LOUD.
            // The exclusive DEBUG count below refuses a loud copy too, but with
            // a generic message; this arm names the condition.
            for level in ["WARN", "INFO", "ERROR"] {
                let n = count(level, "suppressed repeat", "rmw_slcflood/A");
                if n != 0 {
                    return Err(format!(
                        "a suppressed repeat for the repeated type was emitted at {level} \
                         ({n} line(s))"
                    ));
                }
            }
            let a_suppressed =
                count_at_exclusively(lines, "DEBUG", &["suppressed repeat", "rmw_slcflood/A"])?;
            let want_a_suppressed = debug_lines_expected(4);
            if a_suppressed != want_a_suppressed {
                return Err(format!(
                    "expected {want_a_suppressed} DEBUG suppressed repeats for the repeated type, \
                     got {a_suppressed}"
                ));
            }
            let b_heads =
                count_at_exclusively(lines, "WARN", &["override is IGNORED", "rmw_slcflood/B"])?;
            if b_heads != 1 {
                return Err(format!(
                    "a second fixed type must get its OWN loud head, got {b_heads}"
                ));
            }
            Ok(())
        });
    }

    /// A C++ typesupport carrying the C-style
    /// `pkg__msg` namespace — accepted verbatim, nothing validates the
    /// spelling — must normalize to the SAME canonical `pkg/Type` as its
    /// `pkg::msg` twin, so the tier table, the env override, AND the schema
    /// hash all agree across the two spellings. Unnormalized, `tf2_msgs__msg`
    /// flows through a `::`-only split as `tf2_msgs__msg/TFMessage`:
    /// both the table and the override MISS (the type falls to the 128 MiB
    /// blanket) and its schema hash can interop with nothing.
    #[test]
    fn a_cpp_dunder_msg_namespace_resolves_like_its_colon_twin() {
        let dunder = cpp_variable_bridge("tf2_msgs__msg", "TFMessage");
        let colons = cpp_variable_bridge("tf2_msgs::msg", "TFMessage");
        assert_eq!(dunder.qualified_name(), "tf2_msgs/TFMessage");
        assert_eq!(colons.qualified_name(), "tf2_msgs/TFMessage");
        // The interop guarantee: the two spellings hash identically.
        assert_eq!(dunder.schema_hash(), colons.schema_hash());
        // Table tier honored under the dunder spelling (not the blanket).
        assert_eq!(
            slice_len_with_overrides(&dunder, &SliceCeilingOverrides::default()).get(),
            SMALL_TIER,
            "the dunder spelling must resolve its REAL tier"
        );
        // Env override honored under the dunder spelling (not missed).
        let narrowed = SliceCeilingOverrides::parse("tf2_msgs/TFMessage:1024");
        assert_eq!(slice_len_with_overrides(&dunder, &narrowed).get(), 1024);
        // The `::` control resolves identically.
        assert_eq!(
            slice_len_with_overrides(&colons, &SliceCeilingOverrides::default()).get(),
            SMALL_TIER
        );
        assert_eq!(slice_len_with_overrides(&colons, &narrowed).get(), 1024);
    }
}
