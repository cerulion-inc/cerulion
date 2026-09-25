// SPDX-License-Identifier: AGPL-3.0-only
//! Node entry trait and implementations for graph runtime.
//!
//! Defines `NodeEntry` — the interface between the graph runtime and node logic.
//! Two implementations:
//! - `ClosureNodeEntry` for testing (test-only)
//! - `DylibNodeEntry` for production cdylib loading via `libloading`
//!
//! The graph runtime is agnostic to the backing implementation.
//!
//! A node author implements neither and never names `NodeEntry`:
//! `#[cerulion_node]` generates the impl and the exported symbols
//! `DylibNodeEntry` loads. What a node author uses from this module is
//! [`NodeContext`] (handed to `init`) and [`ExternalSource`] (returned by an
//! `external` node), both re-exported by the prelude.
//!
//! `NodeInfo` carries the node's input/output port metadata and an
//! optional `MacroPolicy`; the node *type* is the folder name
//! (`nodes/<type>/`), resolved at graph-load time, never carried by
//! the node itself or its FFI export.

use crate::scheduler::{
    BackpressureEvent, ExpectWithinEvent, LivelinessEvent, PromiseWithinEvent, QosEventStore,
    SyncHeadOp, SyncOpAnswer,
};
use crate::wire::MaxSliceLen;
use std::ffi::CStr;
use std::str::FromStr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use indexmap::IndexMap;

use crate::clock::Clock;
// RealClock used only behind `#[cfg(any(test, feature = "test-helpers"))]`
// in the `for_tests` constructor — gate the import the same way to
// avoid a non-test "unused import" lint.
#[cfg(any(test, feature = "test-helpers"))]
use crate::clock::RealClock;
use crate::error::{TransportError, TransportResult};
use crate::message::ShmMessage;
use crate::transport::input_view::InputView;
use crate::transport::output_proxy::OutputProxy;
use crate::transport::publisher::CerulionPublisher;
use crate::transport::subscriber::{CerulionSubscriber, RawInputView, ReceivedMessage};
use crate::transport::TransportManager;

/// Cooperative shutdown signal shared between `GraphRuntime` and every node
/// in the graph.
///
/// Wraps `Arc<AtomicBool>`; cheap to clone, lock-free to read. The runtime's
/// main loop polls `is_requested()` between ticks and exits cleanly when set.
/// Nodes flip the bool by calling `NodeContext::request_shutdown()` (or the
/// macro-injected `self.request_shutdown()` shim method).
///
/// Idempotent: requesting twice is safe and observable as a single request.
#[derive(Clone, Debug, Default)]
pub struct ShutdownSignal {
    inner: Arc<AtomicBool>,
}

impl ShutdownSignal {
    /// Construct a fresh, un-requested shutdown signal.
    pub fn new() -> Self {
        Self::default()
    }

    /// Request graceful shutdown of the graph. Idempotent.
    ///
    /// Uses `Release` ordering so the runtime's `Acquire` poll observes
    /// the request along with all prior writes (e.g. the node's last
    /// state mutation before deciding to shut down).
    pub fn request(&self) {
        self.inner.store(true, Ordering::Release);
    }

    /// Returns true if shutdown has been requested.
    ///
    /// Uses `Acquire` ordering — pairs with `request()`'s `Release`.
    pub fn is_requested(&self) -> bool {
        self.inner.load(Ordering::Acquire)
    }
}

/// Hidden runtime-context fields the `#[cerulion_node]` macro injects into
/// every user struct.
///
/// Bundled into a single typed wrapper so the user can `#[derive(Default)]`
/// freely without colliding with the macro's hidden state. The user never
/// touches this type directly — it's accessed through the macro-injected
/// `self.now_ns()` and `self.request_shutdown()` shim methods, and
/// overwritten in the wrapper's `init` from the runtime's clock + shared
/// shutdown signal.
///
/// Public for the proc-macro to reference; `#[doc(hidden)]` so it doesn't
/// pollute the user-facing prelude.
#[doc(hidden)]
#[derive(Clone)]
pub struct CerNodeRuntimeFields {
    pub clock: Arc<dyn Clock>,
    pub shutdown_signal: ShutdownSignal,
}

impl std::fmt::Debug for CerNodeRuntimeFields {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // `dyn Clock` isn't `Debug` by design (unit clock + simulated
        // clock keep the trait object slim). Print the field set without
        // the inner clock state — the signal carries enough state to be
        // useful in test failure messages.
        f.debug_struct("CerNodeRuntimeFields")
            .field("clock", &"<dyn Clock>")
            .field("shutdown_signal", &self.shutdown_signal)
            .finish()
    }
}

/// Sentinel pre-init clock. Replaces the silent
/// `RealClock` default of `CerNodeRuntimeFields`. Any clock method
/// called on this type panics with a clear diagnostic — surfacing the
/// "node was constructed but not initialized via the runtime wrapper"
/// misuse loudly, instead of silently returning `RealClock` time and
/// letting tests using `VirtualClock` see two different timelines.
///
/// In normal flow the macro's `Entry::init` overwrites the clock with
/// the runtime's real clock BEFORE any tick fires, so user code never
/// reaches `UninitClock`. If it does, the panic identifies the bug.
///
/// Marked `#[doc(hidden)]` so it doesn't pollute the user-facing
/// prelude; users of `#[cerulion_node]` never see it.
#[doc(hidden)]
pub struct UninitClock;

impl Clock for UninitClock {
    fn now_ns(&self) -> u64 {
        panic!(
            "UninitClock::now_ns called — node was not initialized via the runtime wrapper. \
             This is a bug in the caller (a `#[cerulion_node]` user should never construct \
             the user struct directly and call clock methods on it pre-init)."
        );
    }
    fn virt_ns(&self) -> Option<u64> {
        panic!(
            "UninitClock::virt_ns called — node was not initialized via the runtime wrapper. \
             This is a bug in the caller."
        );
    }
    fn ext_ns(&self) -> Option<u64> {
        panic!(
            "UninitClock::ext_ns called — node was not initialized via the runtime wrapper. \
             This is a bug in the caller."
        );
    }
}

// `Default` is preserved BUT routed to `UninitClock`
// instead of `RealClock`. The macro's `#[derive(Default)]`-flow still
// compiles — user nodes that derive `Default` get `__cer_rt` initialized
// via `CerNodeRuntimeFields::default()`. The semantic difference: any
// pre-init clock access PANICS instead of silently returning
// `RealClock` time, which is what the determinism contract
// demands. The runtime wrapper's `init()` overwrites the clock with the
// real clock before any tick fires, so normal flow never reaches
// `UninitClock`. This is INVISIBLE to correctly-implemented
// node code; it only fires for misuse (constructing the user struct
// outside the runtime wrapper and reading a clock method pre-init).
impl Default for CerNodeRuntimeFields {
    fn default() -> Self {
        Self {
            clock: Arc::new(UninitClock),
            shutdown_signal: ShutdownSignal::default(),
        }
    }
}

/// Backpressure policy when an input buffer is full.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum BackpressurePolicy {
    /// Discard oldest message, enqueue new one (default).
    #[default]
    DropOldest,
    /// The producer's tick is DEFERRED by the scheduler while any
    /// block consumer's queue is full (zero-copy, no data loss, no deadlock).
    /// The producer's publisher increments a shared `outstanding` mirror on
    /// each send; the consumer's subscriber decrements it on drain; the
    /// producer's scheduler pre-fire reads the mirror and skips the tick the
    /// moment `outstanding >= depth` (the declared depth IS the input's
    /// real iceoryx2 queue, no clamp). Requires an
    /// all-block topic — a MIXED topic (block + non-block consumers) degrades
    /// its block consumers to `DropOldest` (deferring would starve the
    /// non-block sibling).
    Block,
    /// Accept at most one message per N ms, drop extras.
    Sample(u64),
}

/// Metadata describing a declarative input port.
///
/// Produced by `#[input(...)]` field attributes in `#[cerulion_node]`.
#[derive(Debug, Clone)]
pub struct InputMeta {
    pub name: String,
    /// The input port type's layout hash (`<T as ShmMessage>::SCHEMA_HASH`).
    /// In-process macro nodes always populate it. Cdylib
    /// (`DylibNodeEntry`) info JSON carries it too (ABI v11, a STRICT bump —
    /// a pre-v11 cdylib is refused at load, so on this host `0` can only
    /// mean a raw-FFI info block that omits the key). The network
    /// ingress-hash resolver reads it to validate ingress frames against
    /// the consuming input's schema.
    pub schema_hash: u64,
    pub trigger: bool,
    pub depth: usize,
    pub backpressure: BackpressurePolicy,
    /// Per-input expected interval (ms). When
    /// set, the scheduler tracks the last-data time on this input and
    /// increments `NodeHandle::expect_within_missed_count` if `N`
    /// ms elapse without new data. Independent of trigger policy
    /// (`#[input(trigger)]` and `#[input(expect_within_ms = N)]` are
    /// orthogonal).
    pub expect_within_ms: Option<u64>,
}

/// Metadata describing a declarative output port.
///
/// Produced by `#[output]` field attributes in `#[cerulion_node]`.
///
/// `#[non_exhaustive]` so future fields can be added without breaking
/// downstream callers. Construct via direct struct-literal syntax (which
/// requires a feature opt-in per Rust's stability rules), or use a
/// builder helper if one is added.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct OutputMeta {
    pub name: String,
    pub schema_hash: u64,
    /// Per-output committed interval (ms). When
    /// set, the publisher tracks the last-publish time on this output
    /// and increments the node's `promise_within_missed_count` if N
    /// ms elapse without a publish. Independent of trigger policy.
    pub promise_within_ms: Option<u64>,
    /// Schema-default `max_slice_len` from `<T as ShmMessage>::MAX_SLICE_LEN`
    /// (tier-2 of the 3-tier resolution
    /// ladder).
    ///
    /// `None` means the macro had no field type to consult (e.g. the
    /// info came from `NodeInfo::from_names` without
    /// per-port-type plumbing). The runtime tier-2 lookup treats
    /// `None` as "skip; fall through to tier-3 (`DEFAULT_MAX_SLICE_LEN`)".
    /// `Some(n)` is the per-schema budget the macro inferred from the
    /// `#[output]` field's type.
    ///
    /// Typed `Option<MaxSliceLen>`, which is tighter than
    /// `Option<NonZero…>`: `Some(0)` is unrepresentable, and
    /// more. The `MaxSliceLen` newtype encodes
    /// BOTH the upper bound (`u32::MAX`, the wire format's
    /// `WireHeader::total_size` ceiling) AND the lower bound
    /// (`>= WireHeader::SIZE = 32`). Construction is fallible (via
    /// `MaxSliceLen::try_new`), so the field stays `pub` safely:
    /// `meta.max_slice_len_default = MaxSliceLen::try_new(8)` returns
    /// `None` — no pathological state ever stored, even on direct
    /// mutation. Codegen emits via `MaxSliceLen::const_new` (panic-at-
    /// const-eval on invalid). The runtime tier-2 lookup unwraps
    /// via `.get()` for a `u32` directly; no runtime floor/ceiling
    /// check needed.
    pub max_slice_len_default: Option<MaxSliceLen>,
    /// The output port type's own fixed wire-section size
    /// (`<T as ShmMessage>::WIRE_FIXED_SIZE`) — the layout fact the
    /// RECORDER needs, from the one vantage that cannot be stale.
    ///
    /// `schema_hash` above is already the authoritative wire identity a
    /// recorded channel is stamped with; this is its missing other half.
    /// Before this field existed the recorder took the size from the workspace
    /// `schemas/` file the graph's `schema:` names, so a workspace whose
    /// node publishes a layout the file does not describe (or does not
    /// describe at all) recorded a channel whose descriptor disagreed with
    /// every frame on it.
    ///
    /// **`None` is "no claim", and `Some(0)` is a real answer.** A purely
    /// VARIABLE schema (`tf2_msgs/TFMessage`) has a genuinely zero-byte
    /// fixed section, so `0` cannot double as the unknown sentinel — hence
    /// the `Option`. `None` means the node did not declare one at all: a
    /// closure-based entry (no per-port type), a raw-FFI cdylib whose info
    /// block omits the key, or a macro cdylib built before this field
    /// existed. Consumers must fall back, never fabricate.
    ///
    /// **The ABI was NOT bumped, and here is what that costs.** An OLD
    /// cdylib on a NEW host is clean: the key is absent, the field defaults
    /// to `None`, and the recorder sizes from the workspace file exactly as
    /// it did before this field existed. The reverse — a NEW cdylib on an OLD host —
    /// is not "exactly as before": that host's legal-key set does not carry
    /// `wire_fixed_size`, so its unknown-key walk warns once per output
    /// port on every node load, naming a "nearest legal key" and reporting a
    /// correctly-built cdylib as a typo. A bump would not have made that
    /// pair work either — `DylibNodeEntry::load` compares ABI versions with
    /// exact equality, so it would REFUSE the load in both directions — so
    /// the choice is between a spurious warn and a refusal, not between a
    /// warn and correctness. The warn was judged the better failure for a
    /// key whose absence degrades safely.
    ///
    /// `u32`, not `usize`, because that is what the wire and the bag's
    /// `SchemaDescriptor` carry, and the narrowing DROPS an out-of-range
    /// value to `None` (no claim → the file) rather than saturating it —
    /// mirroring the recorder's workspace fold, which OMITS an
    /// over-ceiling schema instead of recording `u32::MAX`.
    ///
    /// Why it narrows at runtime rather than asserting. For a type that is
    /// actually published the bound IS compile-time:
    /// [`ShmMessage::_SHM_INVARIANTS`](crate::message::ShmMessage) const-asserts
    /// `WireHeader::SIZE + WIRE_FIXED_SIZE + 8 * VARIABLE_FIELD_COUNT <=
    /// u32::MAX` for EVERY `T`, fixed and variable alike, and `loan_proxy::<T>`
    /// forces it — so an unrepresentable port cannot publish. (Codegen adds a
    /// second, fixed-schema-only guard: an explicit `assert!(sum <= u32::MAX)`
    /// before the `as u32` cast that feeds `MaxSliceLen::const_new`. Note that
    /// `const_new` itself takes a `u32` and therefore cannot see the overflow —
    /// the `assert!` is the load-bearing line, not the constructor.)
    ///
    /// What that leaves is a const item, and a const item is only evaluated
    /// when something references it. This builder is `pub` and reachable
    /// without ever loaning: a hand-written `NodeEntry` may call it with any
    /// `usize`, and the sibling JSON arm takes its value from an info block
    /// nothing type-checks at all. Those are the callers the runtime drop is
    /// for.
    pub wire_fixed_size: Option<u32>,
}

impl OutputMeta {
    /// Construct an `OutputMeta` with all required fields.
    ///
    /// The struct is `#[non_exhaustive]`, so external callers can't
    /// build it via struct-literal syntax; this constructor is the
    /// stable construction surface for `cerulion_macros`'
    /// generated code (which mechanically passes
    /// `<T as ShmMessage>::MAX_SLICE_LEN`) and any hand-written
    /// `NodeEntry` impls. Hand-written callers should think
    /// carefully about `max_slice_len_default` — `None` means tier-2
    /// resolution is skipped and tier-3's `DEFAULT_MAX_SLICE_LEN`
    /// fires (with a `tracing::warn!`).
    pub fn new(name: String, schema_hash: u64, max_slice_len_default: Option<MaxSliceLen>) -> Self {
        Self {
            name,
            schema_hash,
            max_slice_len_default,
            promise_within_ms: None,
            wire_fixed_size: None,
        }
    }

    /// Builder-style: set the per-output committed interval
    /// (`promise_within_ms`).
    pub fn with_promise_within_ms(mut self, promise_within_ms: u64) -> Self {
        self.promise_within_ms = Some(promise_within_ms);
        self
    }

    /// Builder-style: declare the per-output fixed wire-section
    /// size from the port type's `<T as ShmMessage>::WIRE_FIXED_SIZE`.
    ///
    /// A builder rather than a fourth `new` argument for the same reason
    /// [`Self::with_promise_within_ms`] is one: `new` is the stable
    /// construction surface a `#[non_exhaustive]` struct leaves to
    /// out-of-crate callers, and widening it would churn every hand-written
    /// `NodeEntry` impl for a fact most of them cannot supply.
    ///
    /// Takes `usize` (the trait const's type) and narrows: a value past
    /// `u32::MAX` leaves the field `None` — "no claim", so a consumer falls
    /// back to the workspace schema file — never a truncated or saturated
    /// size, which would be an affirmatively wrong descriptor. See the
    /// field docs for why that is only half unreachable at compile time.
    ///
    /// The drop is REPORTED, at the same level and with the same explanation
    /// as the identical condition on the cdylib JSON path
    /// (`parse_info_json`) — minus that path's cdylib-only "rebuild the
    /// cdylib" line, which names nothing an in-process caller can act on.
    /// Same condition, one loudness: a silent drop here would be the
    /// shipping path for every in-process macro node, and this crate's rule
    /// is a loud warn at the inference site rather than silent inference.
    pub fn with_wire_fixed_size(mut self, wire_fixed_size: usize) -> Self {
        self.wire_fixed_size = u32::try_from(wire_fixed_size).ok();
        if self.wire_fixed_size.is_none() {
            tracing::warn!(
                output = %self.name,
                schema_hash = self.schema_hash,
                raw_value = wire_fixed_size,
                "output port declared a wire_fixed_size above u32::MAX, which no Cerulion \
                 frame can carry (WireHeader::total_size is u32); treating the port as \
                 declaring no size, so a recording falls back to the workspace schema file"
            );
        }
        self
    }
}

/// Metadata describing a node's I/O ports.
///
/// Describes a node's ports and optional trigger policy. The node's
/// *type* is the folder name (`nodes/<type>/`), resolved at
/// graph-load time, never stored here.
///
/// `input_names` and `output_names` are precomputed mirrors of
/// `input_meta` / `output_meta` (kept as fields rather than methods so
/// existing call sites can borrow them as `&[String]`). Fields are
/// `pub(crate)` to enforce the
/// `input_names == input_meta.iter().map(|m| m.name).collect()`
/// invariant (and the same for outputs); external callers must
/// construct via [`NodeInfo::from_names`] / [`NodeInfo::with_meta`]
/// and read via the `pub` accessors below.
#[derive(Debug, Clone, Default)]
pub struct NodeInfo {
    pub(crate) input_names: Vec<String>,
    pub(crate) output_names: Vec<String>,
    /// Declarative input metadata. Empty for nodes without `#[input]` attrs.
    pub(crate) input_meta: Vec<InputMeta>,
    /// Declarative output metadata. Empty for nodes without `#[output]` attrs.
    pub(crate) output_meta: Vec<OutputMeta>,
    /// Macro→runtime policy
    /// plumbing: trigger policy declared by the `#[cerulion_node(...)]`
    /// macro (`period_ms = N`, `sync_window_ms = N`,
    /// or `external`). Surfaced from the cdylib's info JSON. The
    /// runtime uses this when the graph YAML doesn't override with
    /// its own `policy:` block.
    pub(crate) policy: Option<MacroPolicy>,
    /// Per-node tick-execution deadline (ms).
    /// When set, the scheduler wraps the `fire_node` callback with
    /// timing; tick callbacks taking longer than N ms increment
    /// `NodeHandle::tick_within_missed_count`. Orthogonal to
    /// the trigger policy.
    pub(crate) tick_within_ms: Option<u64>,
    /// Node-level producer rate cap (ms). When set, the graph
    /// runtime installs a scheduler pre-fire time-gate that defers the
    /// node's tick while `now - last_fire < N ms` — a "fire no faster than
    /// once per N ms" cap. Composes with the trigger policy and the `block`
    /// pre-fire (defer if EITHER fires); mutually exclusive with `period_ms`
    /// (rejected at macro-expansion). ABI v6 plumbed this
    /// across the cdylib info-JSON FFI alongside `tick_within_ms`, so a
    /// cdylib node carries the cap exactly as an in-crate one does: the
    /// macro writes `"throttle_ms":N` into the info JSON, `parse_info_json`
    /// reads it back into this field (absent on a pre-v6 cdylib, or on a
    /// node without the attr, ⇒ `None` via serde default), and the graph
    /// runtime consumes it through `NodeInfo::throttle_ms` to install the
    /// pre-fire time-gate.
    pub(crate) throttle_ms: Option<u64>,
}

/// Macro-declared trigger policy. Mirrors
/// `cerulion_macros::parse::NodeLevelAttrs`; re-encoded here in
/// `cerulion_core` because the runtime can't depend on the macro
/// crate. The cdylib serialises this into its info JSON; the
/// loader (and the CLI's `parse_node_metadata`) parses it back.
///
/// `DataTrigger { input_name }` carries the macro-declared
/// `#[input(trigger)]` field name so the runtime can synthesize a
/// `DataTriggerBinding` against the YAML-wired source for the
/// input named here.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MacroPolicy {
    Period {
        period_ms: u64,
    },
    /// Bounded sync: fire ONCE PER COMPLETE ALIGNED SET, in set order. A set is
    /// one unconsumed message from every `#[input(trigger)]` port whose
    /// timestamps lie within `window_ms` of each other. Each trigger message is
    /// consumed by at most one set, so a burst holding k complete sets gives k
    /// fires and the k-th tick reads the k-th set's members, not the freshest
    /// frame on each topic. A message a partner has run more than `window_ms`
    /// past is in no set: it is discarded and counted
    /// (`NodeHandle::sync_unmatched_discard_count`).
    ///
    /// That is the semantic a `#[cerulion_node]` node gets. A node the per-set
    /// path cannot serve (for example a raw-FFI library, or any node under
    /// `CERULION_DRAIN_DISCIPLINE=separate`) keeps the earlier latest-per-set
    /// behaviour, and the graph build says so with a `warn`.
    Sync {
        window_ms: u64,
    },
    /// Unbounded sync: fire ONCE PER COMPLETE SET, as soon as every
    /// `#[input(trigger)]` port has an unconsumed message, with no timing
    /// bound. Each trigger message is consumed by at most one set, and a
    /// backlog is served in order, set by set, each set formed from the oldest
    /// unconsumed message on every port. The matcher trades a set's oldest
    /// member for that port's next message only when some other port has no
    /// later message waiting and the trade strictly tightens the set
    /// (`scheduler::sync_match`); otherwise it fires at once rather than
    /// waiting for a possibly nearer message. **Not recommended for control
    /// loops:** worst-case fire latency is the slowest publisher's
    /// inter-arrival interval, unbounded if it stops.
    UnboundedSync,
    External,
    DataTrigger {
        input_name: String,
    },
}

/// Wire-shape mirror of [`MacroPolicy`] used by the cdylib FFI's
/// `cerulion_node_info()` JSON and by raw-FFI nodes' `INFO_BYTES`
/// JSON. A single struct here keeps the macro emitter
/// (`cerulion_macros::codegen::gen_cdylib`), the runtime parser
/// (`parse_info_json` below), the CLI emitter
/// (`cerulion_cli_engine::templates::generate_info_fn`), and the
/// CLI parser (`cerulion_cli_engine::node_metadata::try_parse_raw_ffi_node`)
/// in lock-step on one shape.
///
/// Wire shape (each variant produces exactly one of):
///
/// ```json
/// {"period_ms": 100}
/// {"sync_window_ms": 25}
/// {"unbounded_sync": true}
/// {"external": true}
/// {"data_trigger": {"input_name": "count"}}
/// ```
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PolicyJson {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub period_ms: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sync_window_ms: Option<u64>,
    #[serde(default, skip_serializing_if = "is_false")]
    pub unbounded_sync: bool,
    #[serde(default, skip_serializing_if = "is_false")]
    pub external: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub data_trigger: Option<DataTriggerPayload>,
}

/// Payload of the `data_trigger` variant in [`PolicyJson`].
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DataTriggerPayload {
    pub input_name: String,
}

#[doc(hidden)]
fn is_false(b: &bool) -> bool {
    !*b
}

impl From<&MacroPolicy> for PolicyJson {
    fn from(p: &MacroPolicy) -> Self {
        match p {
            MacroPolicy::Period { period_ms } => PolicyJson {
                period_ms: Some(*period_ms),
                ..Default::default()
            },
            MacroPolicy::Sync { window_ms } => PolicyJson {
                sync_window_ms: Some(*window_ms),
                ..Default::default()
            },
            MacroPolicy::UnboundedSync => PolicyJson {
                unbounded_sync: true,
                ..Default::default()
            },
            MacroPolicy::External => PolicyJson {
                external: true,
                ..Default::default()
            },
            MacroPolicy::DataTrigger { input_name } => PolicyJson {
                data_trigger: Some(DataTriggerPayload {
                    input_name: input_name.clone(),
                }),
                ..Default::default()
            },
        }
    }
}

impl PolicyJson {
    /// Inverse of `From<&MacroPolicy>`. Returns the highest-
    /// priority variant present (Period > Sync > UnboundedSync >
    /// DataTrigger > External). All variants are mutually
    /// exclusive in macro-emitted JSON.
    pub fn into_macro_policy(self) -> Option<MacroPolicy> {
        if let Some(period_ms) = self.period_ms {
            return Some(MacroPolicy::Period { period_ms });
        }
        if let Some(window_ms) = self.sync_window_ms {
            return Some(MacroPolicy::Sync { window_ms });
        }
        if self.unbounded_sync {
            return Some(MacroPolicy::UnboundedSync);
        }
        if let Some(payload) = self.data_trigger {
            return Some(MacroPolicy::DataTrigger {
                input_name: payload.input_name,
            });
        }
        if self.external {
            return Some(MacroPolicy::External);
        }
        None
    }
}

/// Construction-time port-name uniqueness (inputs first, later extended to
/// outputs) for the meta-carrying
/// `NodeInfo` constructors — the closest boundary to a duplicate's author.
/// Hard assert in ALL builds (this is a cold path):
/// a duplicate means conflicting per-port declarations
/// (policy/depth on inputs, schema/deadline on outputs) and downstream
/// readers would silently resolve first-wins (`.find()`) or last-wins
/// (`HashMap` collect) depending on the call site. Macro structs cannot
/// hit this (Rust rejects duplicate fields) and the dylib FFI ingress
/// (`parse_info_json`) pre-validates and returns `Err` — only a direct
/// embedder call reaches the panic, and that's a programming error worth
/// one. `GraphTopology::build` re-checks inputs as defense-in-depth for
/// in-crate literal construction (fields are `pub(crate)`).
fn assert_unique_port_names<'a>(names: impl Iterator<Item = &'a str>, ctor: &str, kind: &str) {
    let mut seen: Vec<&str> = Vec::new();
    for name in names {
        assert!(
            !seen.contains(&name),
            "NodeInfo::{ctor}: duplicate {kind} port name '{name}' — each \
             port may carry exactly one meta entry (conflicting \
             declarations are ambiguous)"
        );
        seen.push(name);
    }
}

impl NodeInfo {
    /// Construct a `NodeInfo` from raw input/output port name lists.
    ///
    /// Used by `ClosureNodeEntry` (test-only) where only port names are
    /// available; the cdylib JSON parser constructs via
    /// [`Self::with_input_names_and_output_meta`]. `input_meta` and
    /// `output_meta` are empty (the names-only constructor).
    pub fn from_names(input_names: Vec<String>, output_names: Vec<String>) -> Self {
        Self {
            input_names,
            output_names,
            input_meta: Vec::new(),
            output_meta: Vec::new(),
            policy: None,
            tick_within_ms: None,
            throttle_ms: None,
        }
    }

    /// Construct a `NodeInfo` from input/output
    /// metadata. Names are DERIVED from the meta entries' `name`
    /// fields, so the `*_names` ↔ `*_meta` invariant is preserved
    /// by construction.
    ///
    /// Use this when you have full metadata available (e.g. macro
    /// codegen with `#[input(trigger, depth = 1)]` attributes)
    /// rather than just port names.
    ///
    /// # Panics
    ///
    /// Panics if two `InputMeta` entries — or two `OutputMeta` entries —
    /// share a name: each port carries exactly one meta (conflicting
    /// declarations are ambiguous). See `assert_unique_port_names`.
    pub fn with_meta(input_meta: Vec<InputMeta>, output_meta: Vec<OutputMeta>) -> Self {
        assert_unique_port_names(
            input_meta.iter().map(|m| m.name.as_str()),
            "with_meta",
            "input",
        );
        assert_unique_port_names(
            output_meta.iter().map(|m| m.name.as_str()),
            "with_meta",
            "output",
        );
        let input_names = input_meta.iter().map(|m| m.name.clone()).collect();
        let output_names = output_meta.iter().map(|m| m.name.clone()).collect();
        Self {
            input_names,
            output_names,
            input_meta,
            output_meta,
            policy: None,
            tick_within_ms: None,
            throttle_ms: None,
        }
    }

    /// Construct a `NodeInfo` from raw input
    /// names + populated output meta.
    ///
    /// Used by the `#[cerulion_node]` declarative-mode emission, which
    /// knows full output metadata (per-port schema marker types, hence
    /// `<T as ShmMessage>::SCHEMA_HASH` and `MAX_SLICE_LEN`) but only
    /// has names for inputs (the trigger/depth metadata flows
    /// through a different path). Output names are DERIVED from
    /// `output_meta` to preserve the `output_names` ↔ `output_meta`
    /// invariant; input meta is left empty.
    ///
    /// # Panics
    ///
    /// Panics on duplicate input names or duplicate `OutputMeta` names —
    /// see `assert_unique_port_names`. The dylib FFI ingress
    /// (`parse_info_json`) pre-validates and returns `Err` instead, so a
    /// buggy cdylib cannot reach these panics.
    pub fn with_input_names_and_output_meta(
        input_names: Vec<String>,
        output_meta: Vec<OutputMeta>,
    ) -> Self {
        assert_unique_port_names(
            input_names.iter().map(String::as_str),
            "with_input_names_and_output_meta",
            "input",
        );
        assert_unique_port_names(
            output_meta.iter().map(|m| m.name.as_str()),
            "with_input_names_and_output_meta",
            "output",
        );
        let output_names = output_meta.iter().map(|m| m.name.clone()).collect();
        Self {
            input_names,
            output_names,
            input_meta: Vec::new(),
            output_meta,
            policy: None,
            tick_within_ms: None,
            throttle_ms: None,
        }
    }

    /// Builder that attaches a [`MacroPolicy`] to
    /// an existing `NodeInfo`. Chains naturally with `from_names` /
    /// `with_meta`.
    pub fn with_policy(mut self, policy: MacroPolicy) -> Self {
        self.policy = Some(policy);
        self
    }

    /// Builder that attaches a per-node
    /// `tick_within_ms` to an existing `NodeInfo`. Chains naturally
    /// with `from_names` / `with_meta` / `with_policy`.
    pub fn with_tick_within_ms(mut self, tick_within_ms: u64) -> Self {
        self.tick_within_ms = Some(tick_within_ms);
        self
    }

    /// Builder-style setter for the node-level `throttle_ms`
    /// producer rate cap. Emitted by `#[cerulion_node(throttle_ms = N)]`.
    pub fn with_throttle_ms(mut self, throttle_ms: u64) -> Self {
        self.throttle_ms = Some(throttle_ms);
        self
    }

    /// The node-level producer rate cap (ms), if declared.
    pub fn throttle_ms(&self) -> Option<u64> {
        self.throttle_ms
    }

    /// Read-only access to the per-node
    /// tick-execution deadline (ms), if set. Used by the scheduler at
    /// graph-build time to wire up the per-tick timing wrapper.
    pub fn tick_within_ms(&self) -> Option<u64> {
        self.tick_within_ms
    }

    /// Read-only access to declared input port names. Order matches
    /// declaration order on the node struct (insertion-stable).
    pub fn input_names(&self) -> &[String] {
        &self.input_names
    }

    /// Read-only access to declared output port names. Order matches
    /// declaration order.
    pub fn output_names(&self) -> &[String] {
        &self.output_names
    }

    /// Read-only access to input metadata (depth, backpressure,
    /// trigger flag, etc.). Empty for nodes without `#[input]` attrs.
    pub fn input_meta(&self) -> &[InputMeta] {
        &self.input_meta
    }

    /// Read-only access to output metadata. Empty for nodes without
    /// `#[output]` attrs.
    pub fn output_meta(&self) -> &[OutputMeta] {
        &self.output_meta
    }

    /// Read-only access to the macro-declared trigger policy, if any.
    ///
    /// Returns an owned clone of the policy. The
    /// `MacroPolicy::DataTrigger { input_name: String }` variant makes
    /// the enum no longer `Copy`, so this clones the inner String for
    /// that one variant; the four older variants clone for free
    /// (struct copies, no allocations). The callers that already exist
    /// (`macro_cdylib_policy_round_trip_test.rs`, etc.) compare via
    /// `assert_eq!(info.policy(), Some(MacroPolicy::X { ... }))` —
    /// returning an owned value keeps that ergonomics intact.
    ///
    /// `info.policy()` is called at graph-build time (once per node);
    /// it is NOT on any hot path. The runtime's per-node-build code
    /// in `runtime.rs` reads `entry_info.policy.as_ref()` directly
    /// off the snapshot field instead of going through this accessor,
    /// so the clone here only fires for callers that explicitly want
    /// an owned value.
    pub fn policy(&self) -> Option<MacroPolicy> {
        self.policy.clone()
    }

    /// SINGLE source of truth for whether a macro
    /// `DataTrigger` input is eligible to UNIFY onto the node's BODY
    /// subscriber — eliminating the separate trigger-drain
    /// subscriber and so contributing +1 subscriber (body only) instead of
    /// +2.
    ///
    /// Called at BOTH the count pass (`max_subscribers` provisioning) AND the
    /// per-node build loop (where the unification actually happens), so the
    /// two can NEVER drift — an eligible input that the count pass treats as
    /// non-unified would over-provision a subscriber slot, and a build loop
    /// that unified an input the count pass treated as non-unified would
    /// under-provision the standalone listener's event slot.
    ///
    /// Eligibility requires BOTH:
    ///   1. the ENTRY declares the drain capability
    ///      (`supports_unified_drain == true`, from
    ///      `NodeEntry::unifies_trigger_drain()`, a capability decoupled
    ///      from `performs_input_snapshot`, the rayon-eligibility flag
    ///      the unified drain originally piggybacked on): its `drain_trigger_input`
    ///      drives the BODY subscriber, so eliding the separate trigger-drain
    ///      subscriber cannot mute the node. Macro nodes and
    ///      `ClosureNodeEntry` declare it; entries whose
    ///      `drain_trigger_input` is the `(0, None)` default (a cdylib without
    ///      the drain export) would NEVER fire if unified, so they keep the dual
    ///      subscriber. The caller supplies this bool (looked up from the
    ///      runtime's `unifies_trigger_drain` map by node id); AND
    ///   2. the trigger input's backpressure policy is the default
    ///      `DropOldest`. `Sample(N)` and `Block` keep the dual subscriber —
    ///      the analysis below, from the verified drain code:
    ///
    ///      **`Sample(N)`** — its original exclusion reason ("unified
    ///      read-decimation would gate the FIRE rate") is REFUTED by the
    ///      code: `DrainOutcome.popped` is the RAW pop count
    ///      (`removed_in_drain`, set by `drain_with_block_accounting` per
    ///      frame removed from the iceoryx2 queue), and the sample gate runs
    ///      AFTER the drain on the single surviving frame — the decimate
    ///      early-return still carries `popped: removed_in_drain.get()`, so
    ///      fire-per-arrival WOULD be preserved. The REAL blocker is the
    ///      `expect_within_ms` watchdog: on a decimated drain
    ///      `DrainOutcome.latest_ts` is `None`, so the Unified arm would
    ///      skip the `signal_input_received` reset — while the Separate
    ///      arm resets it for EVERY raw frame ts (its trigger-drain
    ///      subscriber is ungated). A sample-gated trigger input carrying
    ///      `expect_within_ms` would silently start tripping its watchdog
    ///      during decimation regimes under Unified. Until that reset
    ///      semantic is reconciled (candidate: carry the decimated frame's
    ///      wire ts in `DrainOutcome` for the trigger-reset path only),
    ///      `Sample(N)` stays Separate.
    ///
    ///      **`Block`** — the pre-fire mirror analysis: block's producer
    ///      defer reads the consumer-queue `outstanding` mirror, and the
    ///      block probe's event fires off the PRE-drain outstanding depth —
    ///      both calibrated to the body drain running at BODY-READ time
    ///      (inside the tick). Unifying moves the drain to the level
    ///      boundary BEFORE decide, draining the queue before the pre-fire
    ///      gate evaluates the mirror for this step's producer-defer
    ///      decision — changing producer pacing. Block stays Separate.
    ///
    ///      (Since ABI v8 the info JSON carries each input's declared
    ///      backpressure verbatim, and cdylib `input_meta` is no longer
    ///      synthesized as `DropOldest` for every input, so
    ///      this exclusion applies to cdylib nodes exactly as it does to
    ///      macro and closure ones.)
    ///
    ///      **This gate is DATA-shaped, and it was deliberately not
    ///      extended to Sync.** A per-set Sync trigger input is eligible on
    ///      CAPABILITY alone (`per_set_capable` in `graph/runtime.rs` reads
    ///      `unifies_trigger_drain`, never a policy), and both reasons above
    ///      were answered rather than inherited: `Sample(N)`'s watchdog
    ///      blocker is closed by the per-set backpressure contract that a
    ///      DECIMATED arrival resets a per-set Sync trigger's
    ///      `expect_within` anchor (see `per_set_sync_trigger` on
    ///      `CerulionSubscriber`), and `Block`'s pacing blocker is closed by
    ///      the slot-debt occupancy model (`block_slot_debt`), which keeps
    ///      the mirror equal to UNSERVED frames once a pop and its serve
    ///      stop coinciding. Neither answer has been applied to the Data
    ///      path — doing so would be a separate change with its own
    ///      calibration — so do NOT unify a Data `block`/`sample` trigger on
    ///      the strength of the Sync precedent without moving the
    ///      occupancy model and the reset semantics across with it.
    ///
    /// The lookup key `trigger_input_name == InputMeta.name == yaml_input.name`
    /// (the macro field name = the body subscriber map key).
    ///
    /// Fail-safe: a MISSING input (no `InputMeta` with that name) returns
    /// `false` (ineligible → keep the dual subscriber), NOT the default policy.
    /// For a macro node the trigger name always resolves to a declared input
    /// (both come from the same `field_attrs`), so a miss is unreachable there;
    /// a CLOSURE built via `NodeInfo::from_names` (empty `input_meta`) DOES
    /// miss — deliberately: such a closure declared nothing about its trigger
    /// input's policy, so it keeps the dual subscriber. `is_some_and` makes
    /// the safe outcome explicit rather than letting `unwrap_or_default()`
    /// silently expand a miss to `DropOldest` == eligible.
    pub fn unifies_data_trigger(
        &self,
        trigger_input_name: &str,
        supports_unified_drain: bool,
    ) -> bool {
        supports_unified_drain
            && self
                .input_meta
                .iter()
                .find(|m| m.name == trigger_input_name)
                .is_some_and(|m| m.backpressure == BackpressurePolicy::DropOldest)
    }
}

/// A single publisher's teardown reconciliation snapshot — the
/// producer-side terms of the identity
///
/// ```text
/// committed_frames <= frames_recorded + frames_lost + headerless
///                     + dropped_unwritten
///                     + frames_dropped_send_fail + frames_dropped_overflow
/// ```
///
/// The relation is `<=` (an upper-bound cross-check), NOT unconditional
/// equality: `frames_dropped_{send_fail,overflow}` and the bag-side
/// `frames_lost` are DISJOINT only when the producer-side drops were the
/// topic's TAIL (highest committed seq, where bagd's forward-only gap detector
/// never sees a surviving newer seq to reveal the hole). A MID-STREAM
/// producer-side drop is ALSO counted by bagd as a `frames_lost` gap, so that
/// frame is double-attributed and the RHS over-counts by the overlap. An exact
/// match therefore confirms tail-only drops. Note also that the LHS is derived
/// from a u32 wire counter (wraps at 2^32, ~49.7 days at 1 kHz): on a
/// single-topic run past the wrap it resets while the u64 bag-side terms keep
/// accumulating, so the identity is bounded by the wrap window.
///
/// **The LHS is split from the wire counter.** It used to be
/// `next_sequence` outright, which is the committed count only while the
/// counter starts at 0 — true of every live run and FALSE of a restored replay,
/// whose publisher is seeded with the recorded stream's next sequence. One
/// frame committed by a publisher seeded at 4242 leaves `next_sequence` at
/// 4243, and reporting that as the producer term mints 4242 frames of phantom
/// loss against bag-side counts that only ever saw one. All three quantities
/// are therefore carried — the raw counter, its origin, and their difference —
/// so nothing has to be re-derived and none of them is conflated (Principle
/// #3). They are read straight off the port, so they cannot disagree.
///
/// Harvested once at graph teardown (before publishers drop) and LOGGED so the
/// reconciliation can be computed by cross-referencing the run log against the
/// bag's `record_health.json`. Record-only / diagnostics — never read on any
/// firing path (determinism firewall).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PublisherReconStat {
    /// The publisher's topic name.
    pub topic: String,
    /// The next sequence the publisher would assign — so `final_seq` (the
    /// highest COMMITTED seq) is `next_sequence.wrapping_sub(1)`. NOT the
    /// committed count on a restored replay; see [`Self::committed_frames`].
    pub next_sequence: u32,
    /// The sequence this publisher STARTED at — 0 on every
    /// live path, the recorded stream's next sequence on a restored replay.
    pub initial_sequence: u32,
    /// Frames THIS publisher committed
    /// (`next_sequence - initial_sequence`) — the identity's producer term.
    /// Read from
    /// [`crate::transport::publisher::CerulionPublisher::committed_frames`],
    /// which owns the subtraction.
    pub committed_frames: u32,
    /// Frames lost at the overflow re-loan arm of `OutputProxy::Drop`.
    pub frames_dropped_overflow: u64,
    /// Frames lost at the steady-state commit-then-fail arms.
    pub frames_dropped_send_fail: u64,
}

/// Publisher over the iceoryx2 IPC transport.
///
/// Single-variant enum (one `Ipc` arm) kept for call-site stability:
/// every `AnyPublisher::Ipc(x)` construction and `match` arm in the
/// codebase continues to work unchanged. Enum dispatch avoids vtable
/// overhead and `Box<dyn>` heap allocation (see
/// `docs/internals/core-transport.md`).
pub enum AnyPublisher {
    /// iceoryx2 shared memory publisher (cross-process IPC).
    Ipc(CerulionPublisher),
}

impl AnyPublisher {
    /// This publisher's raw iceoryx2 publisher id — the value a
    /// served sample's origin reports (see
    /// [`CerulionPublisher::publisher_id`]). Read once at WIRING time to
    /// build the trace-ring manifest's publisher section.
    pub fn publisher_id(&self) -> u128 {
        match self {
            Self::Ipc(p) => p.publisher_id(),
        }
    }

    /// Publish raw wire bytes directly (raw-FFI re-publish path + the
    /// service layer; late-joiner history delivery has since moved to
    /// native/zero-copy).
    ///
    /// Returns the number of subscribers the frame was delivered to.
    #[must_use = "publish result must be checked"]
    pub fn publish_raw(&mut self, data: &[u8]) -> TransportResult<usize> {
        match self {
            Self::Ipc(p) => p.publish_raw(data),
        }
    }

    /// Loan a zero-copy SHM-backed write proxy for schema `T`.
    ///
    /// Dispatches to `CerulionPublisher::loan_proxy`. On `Drop` the proxy
    /// finalizes the WireHeader and publishes; see `OutputProxy::Drop`.
    #[must_use = "loan_proxy result must be checked"]
    pub fn loan_proxy<T: ShmMessage>(&mut self) -> TransportResult<OutputProxy<'_, T>> {
        match self {
            Self::Ipc(p) => p.loan_proxy::<T>(),
        }
    }

    /// Returns the topic name.
    pub fn topic(&self) -> &str {
        match self {
            Self::Ipc(p) => p.topic(),
        }
    }

    /// Returns the current sequence number (the next value a COMMIT assigns).
    ///
    /// This is the WIRE counter, and it no longer starts
    /// at 0 on a restored replay — so it is the count of committed frames only
    /// when [`Self::initial_sequence`] is 0. For that count, read
    /// [`Self::committed_frames`].
    pub fn sequence(&self) -> u32 {
        match self {
            Self::Ipc(p) => p.sequence(),
        }
    }

    /// The wire sequence this publisher STARTED at — 0 on
    /// every live path, the recorded stream's next sequence on a restored
    /// replay. Delegates to [`CerulionPublisher::initial_sequence`].
    pub fn initial_sequence(&self) -> u32 {
        match self {
            Self::Ipc(p) => p.initial_sequence(),
        }
    }

    /// Frames THIS publisher has committed (`sequence()`
    /// minus its seed) — the quantity the reconciliation identity's
    /// producer term means. Delegates to
    /// [`CerulionPublisher::committed_frames`].
    pub fn committed_frames(&self) -> u32 {
        match self {
            Self::Ipc(p) => p.committed_frames(),
        }
    }

    /// Unconditional running total of output DISCARDS on this
    /// port (incomplete-output ticks that skipped publish). Delegates to
    /// [`CerulionPublisher::output_discard_count`]. Reachable from the node's own
    /// tick code (which holds `AnyPublisher`); the off-thread OPERATOR surface is
    /// the per-output `NodeHandle::output_discard_count` accessor, which reads the
    /// same count via the runtime-shared anchor.
    pub fn output_discard_count(&self) -> u64 {
        match self {
            Self::Ipc(p) => p.output_discard_count(),
        }
    }

    /// Running total of `SentSample` notifies on this port's topic
    /// that did not reach every live listener (a saturated consumer event
    /// socket, or a dead-but-unreaped listener registration) — log-level
    /// independent, never reset on recovery, and zero on a healthy graph.
    /// Delegates to [`CerulionPublisher::notify_undelivered_count`]. Reachable
    /// from the node's own tick code (which holds `AnyPublisher`); the
    /// off-thread OPERATOR surface is the per-output
    /// `NodeHandle::notify_undelivered_count` accessor, which reads the same
    /// count via the runtime-shared anchor.
    pub fn notify_undelivered_count(&self) -> u64 {
        match self {
            Self::Ipc(p) => p.notify_undelivered_count(),
        }
    }

    /// Frames lost at the overflow re-loan arm of `OutputProxy::Drop`
    /// (send-side loss, iceoryx2 pool pressure). Harvested at graph teardown for
    /// the producer-vs-recorder reconciliation identity.
    pub fn frames_dropped_overflow(&self) -> u64 {
        match self {
            Self::Ipc(p) => p.frames_dropped_overflow(),
        }
    }

    /// Frames lost at the steady-state commit-then-fail arms of
    /// `OutputProxy::Drop` (the previously counter-less silent-loss hole).
    /// Harvested at graph teardown for the reconciliation identity.
    pub fn frames_dropped_send_fail(&self) -> u64 {
        match self {
            Self::Ipc(p) => p.frames_dropped_send_fail(),
        }
    }

    /// Returns the publisher's configured maximum slot size.
    ///
    /// `MaxSliceLen` — encodes both the wire-format
    /// ceiling and the floor `>= WireHeader::SIZE` at the type level.
    ///
    /// Useful for tests asserting that the 3-tier
    /// `max_slice_len` resolution selected the expected value
    /// (tier-1 YAML / tier-2 schema-default / tier-3 fallback).
    pub fn max_slice_len(&self) -> MaxSliceLen {
        match self {
            Self::Ipc(p) => p.max_slice_len(),
        }
    }

    /// Drain pending subscriber events to deliver native
    /// history to a freshly-connected late joiner WITHOUT publishing. Forwards
    /// to [`CerulionPublisher::pump_history`]; off the deterministic firing
    /// path (runtime cadence for quiescent publishers).
    pub fn pump_history(&mut self) {
        match self {
            Self::Ipc(p) => p.pump_history(),
        }
    }

    /// The live-loop boundary re-check for the notify-elision
    /// self-heal. Forwards to `CerulionPublisher::resweep_notify_elision`; a
    /// no-op unless elision is armed AND a foreign listener is present. Returns
    /// the number of listeners a boundary notify triggered.
    pub fn resweep_notify_elision(&mut self) -> usize {
        match self {
            Self::Ipc(p) => p.resweep_notify_elision(),
        }
    }
}

/// The per-output LAZY-LOAN slot the `#[cerulion_node_impl]` tick
/// frame binds for each `#[output]` port.
///
/// # Why lazy
///
/// Before lazy loans the generated tick preamble pre-LOANED an [`OutputProxy`] for
/// EVERY declared output and armed publishing on every fully-Ok tick. A
/// SPARSE writer — a node whose tick leaves some outputs untouched on a given
/// tick (a DDS bridge whose empty-queue tick writes nothing; a multi-output
/// node writing a different subset each tick) — therefore Drop-DISCARDED the
/// untouched proxies every tick. Alternating write/empty ticks re-armed the
/// discard flood latch on each complete publish, so every empty-tick discard
/// became a fresh regime head = one loud `error!` per empty tick (observed:
/// 3118 errors in ~4 min on a live Unitree Go2).
///
/// The fix: the `__cer_assign_*` / `__cer_fill_from_*` /
/// `__cer_with_nested_*` write shims (and the Deref method calls) route
/// through [`__cer_loan`](Self::__cer_loan), which loans the proxy on the
/// FIRST write. An output the tick never writes never loans → nothing to
/// discard → a zero-traffic non-event (a per-skipped-port `trace!` gives
/// on-demand visibility; there is deliberately NO counter — it would grow
/// unboundedly on a healthy sparse writer). A loaned-but-incomplete proxy
/// (a partial write) keeps today's loud path EXACTLY: the loan defers publish,
/// the tick tail arms it on Ok, and `OutputProxy::Drop`'s
/// all-variables gate fires the loud discard error + flood latch.
///
/// # Not self-referential
///
/// The `proxy`'s `'loan` borrow points at the underlying
/// [`CerulionPublisher`] behind the `&mut AnyPublisher` (owned by the
/// `NodeContext` publisher map), NOT at the `pubr` field — so holding both
/// fields with the same `'loan` param is sound. At most one field is
/// populated with the live borrow at a time: `pubr` is `take`n exactly once
/// (the first loan), after which `proxy` owns the borrow.
///
/// # Wire timestamp
///
/// Because the loan now happens at first-write time, `OutputProxy`'s wire
/// `timestamp_ns` (stamped by `loan_proxy`) moves from tick-start to
/// first-write time. This is replay-safe: in deterministic runs the gating
/// clock does not advance mid-step, so every write in a tick reads the same
/// `now_ns()`.
///
/// Doc-hidden — this is macro plumbing bound as `__cer_<port>` locals /
/// helper params; user code never names it.
#[doc(hidden)]
pub struct LazyOutput<'loan, T: ShmMessage + 'loan> {
    /// The publisher borrow, present until the first loan `take`s it. `None`
    /// thereafter (the borrow lives in `proxy`).
    pubr: Option<&'loan mut AnyPublisher>,
    /// The live proxy, populated on the first write (`None` while the port is
    /// untouched). Drops at the end of the tick frame — publishing iff the
    /// tail armed it, discarding otherwise.
    proxy: Option<OutputProxy<'loan, T>>,
}

impl<'loan, T: ShmMessage + 'loan> LazyOutput<'loan, T> {
    /// Bind a not-yet-loaned output over its publisher borrow.
    #[doc(hidden)]
    #[inline]
    pub fn new(pubr: &'loan mut AnyPublisher) -> Self {
        Self {
            pubr: Some(pubr),
            proxy: None,
        }
    }

    /// Get-or-loan: return a `&mut OutputProxy` for a write, loaning on the
    /// FIRST call. The freshly-loaned proxy is DEFERRED immediately
    /// (the publish-on-success inversion) so discard is the default and
    /// only the tick tail's [`__cer_arm_if_loaned`](Self::__cer_arm_if_loaned)
    /// — on a fully-Ok outcome — arms the publish. A loan failure `?`-exits
    /// and the tick bails, so a port whose loan failed is never re-loaned.
    #[doc(hidden)]
    #[inline]
    pub fn __cer_loan(&mut self) -> TransportResult<&mut OutputProxy<'loan, T>> {
        if self.proxy.is_none() {
            if let Some(pubr) = self.pubr.take() {
                let mut proxy = pubr.loan_proxy::<T>()?;
                // Defer at loan time — the tail arms on Ok, so every
                // early exit (a later sibling loan failure, a transport error,
                // a user-body `Err`) discards by construction.
                proxy.__cer_defer_publish();
                self.proxy = Some(proxy);
            }
        }
        // Present unless a prior loan already failed (in which case the tick
        // already bailed and this is never reached) — a loud `Err`, never a
        // panic, keeps the illegal state unrepresentable.
        self.proxy
            .as_mut()
            .ok_or_else(|| TransportError::NodeError {
                node_id: String::from("lazy_output"),
                reason: String::from(
                    "lazy-loan: output publisher unavailable (a prior loan on this port failed)",
                ),
            })
    }

    /// Arm publishing IFF this port was loaned this tick. Called by
    /// the generated tick tail on a fully-Ok outcome; a no-op for an untouched
    /// (never-loaned) port. The owning proxy then publishes on Drop through the
    /// normal gates (staged flush + all-variables).
    #[doc(hidden)]
    #[inline]
    pub fn __cer_arm_if_loaned(&mut self) {
        if let Some(p) = &mut self.proxy {
            p.__cer_arm_publish();
        }
    }

    /// Emit a per-skipped-port `trace!` iff this port was NOT written this tick.
    /// Off by default; `RUST_LOG=…=trace` surfaces which outputs a
    /// sparse writer skipped, without a counter (which would grow unboundedly).
    #[doc(hidden)]
    #[inline]
    pub fn __cer_trace_if_unloaned(&self) {
        if self.proxy.is_none() {
            if let Some(pubr) = &self.pubr {
                tracing::trace!(
                    topic = pubr.topic(),
                    schema_hash = T::SCHEMA_HASH,
                    "lazy-loan: output not written this tick — no loan, publish, or discard"
                );
            }
        }
    }
}

/// Subscriber over the iceoryx2 IPC transport.
///
/// Single-variant enum (one `Ipc` arm) kept for call-site stability —
/// see [`AnyPublisher`] for the rationale.
pub enum AnySubscriber {
    /// iceoryx2 shared memory subscriber (cross-process IPC).
    Ipc(CerulionSubscriber),
}

impl AnySubscriber {
    /// Wait for messages and invoke callback for each one received.
    #[must_use = "receive result must be checked"]
    pub fn wait_for_message<F>(&self, timeout: Duration, callback: F) -> TransportResult<usize>
    where
        F: FnMut(ReceivedMessage<'_>),
    {
        match self {
            Self::Ipc(s) => s.wait_for_message(timeout, callback),
        }
    }

    /// Try to receive messages without blocking.
    #[must_use = "receive result must be checked"]
    pub fn try_receive<F>(&self, callback: F) -> TransportResult<usize>
    where
        F: FnMut(ReceivedMessage<'_>),
    {
        match self {
            Self::Ipc(s) => s.try_receive(callback),
        }
    }

    /// Drain to the latest sample and view it as schema `T` via a SHM-backed
    /// reader.
    ///
    /// Dispatches to `CerulionSubscriber::try_view`. Returns `Ok(None)` when
    /// no sample is available and propagates schema-mismatch / receive errors.
    #[must_use = "try_view result must be checked"]
    pub fn try_view<T: ShmMessage, R>(
        &mut self,
        f: impl FnOnce(InputView<'_, T>) -> R,
    ) -> TransportResult<Option<R>> {
        match self {
            Self::Ipc(s) => s.try_view::<T, R>(f),
        }
    }

    /// Dispatches to [`CerulionSubscriber::view_raw`].
    pub fn view_raw(&mut self) -> TransportResult<Option<RawInputView<'_>>> {
        match self {
            Self::Ipc(s) => s.view_raw(),
        }
    }

    /// Dispatches to [`CerulionSubscriber::view_raw_expecting`].
    pub fn view_raw_expecting(
        &mut self,
        schema_hash: u64,
    ) -> TransportResult<Option<RawInputView<'_>>> {
        match self {
            Self::Ipc(s) => s.view_raw_expecting(schema_hash),
        }
    }

    /// Returns the topic name.
    pub fn topic(&self) -> &str {
        match self {
            Self::Ipc(s) => s.topic(),
        }
    }

    /// Take this input's edge-triggered
    /// [`BackpressureEvent`], if any — queued by whichever policy probe the
    /// runtime wired (`sample(N)` decimation, `drop_oldest` eviction, or
    /// `block` at-threshold). Surfaced to `#[on_event]` generated
    /// code and to user code polling manually via `ctx.take_backpressure_event("input_name")`.
    #[must_use = "BackpressureEvent is a user-visible data-loss signal; discarding it silently loses observability"]
    pub fn try_take_backpressure_event(&mut self) -> Option<BackpressureEvent> {
        match self {
            Self::Ipc(s) => s.try_take_backpressure_event(),
        }
    }

    /// Freeze a step-boundary snapshot of the latest
    /// sample. See `CerulionSubscriber::snapshot_latest`.
    pub fn snapshot_latest(&mut self) {
        match self {
            Self::Ipc(s) => s.snapshot_latest(),
        }
    }

    /// Drain a data-trigger input's body subscriber once at the level
    /// boundary, freezing the surviving sample and returning (popped,
    /// latest_ts) for the runtime's `signal_data` / `signal_input_received`.
    /// See `CerulionSubscriber::snapshot_latest_for_trigger`.
    ///
    /// Deliberately `pub(crate)`, mirroring [`Self::sync_next_arrived`]: this
    /// mints [`crate::read_outcome::ReadSiteRole::Drain`], and `NodeContext`
    /// hands out `&mut AnySubscriber` (`subscriber_mut`) while `AnySubscriber`
    /// is in the prelude — so a `pub` here is a route for NODE CODE to record a
    /// drain-SITE read from inside its own tick, which offline is believed over
    /// the kind. No in-repo caller does; every real caller is scheduler-side or
    /// the cdylib FFI export path, both in-crate.
    pub(crate) fn snapshot_latest_for_trigger(&mut self) -> (u64, Option<u64>) {
        match self {
            Self::Ipc(s) => s.snapshot_latest_for_trigger(),
        }
    }

    /// The Data burst loop's between-fires refill of a data-trigger
    /// input — the boundary drain's sibling, differing ONLY in that an unserved
    /// frozen head reports `(0, None)` instead of being re-offered. See
    /// `CerulionSubscriber::refill_for_trigger`.
    ///
    /// `pub(crate)` for the same reason as
    /// [`Self::snapshot_latest_for_trigger`]: it mints a DRAIN-site role, and
    /// this is the type a node's own tick can reach.
    pub(crate) fn refill_for_trigger(&mut self) -> (u64, Option<u64>) {
        match self {
            Self::Ipc(s) => s.refill_for_trigger(),
        }
    }

    /// `NeedNext`: the NON-CONSUMING "has a second frame ARRIVED
    /// behind the head?" probe the per-set Sync matcher's gate and argmin ask
    /// for. See `CerulionSubscriber::sync_next_arrived`.
    pub(crate) fn sync_next_arrived(&mut self) -> TransportResult<bool> {
        match self {
            Self::Ipc(s) => s.sync_next_arrived(),
        }
    }

    /// `NeedStamp`: pop this input's next frame into the staged slot
    /// and report its wire stamp (idempotent — a second peek returns the
    /// RETAINED stamp and pops nothing). See
    /// `CerulionSubscriber::sync_peek_next_stamp`.
    pub(crate) fn sync_peek_next_stamp(&mut self) -> TransportResult<Option<u64>> {
        match self {
            Self::Ipc(s) => s.sync_peek_next_stamp(),
        }
    }

    /// `Advance` / `DiscardTie`: DROP this input's head and refill
    /// from the staged next (or, failing that, from the queue), reporting the
    /// new head's stamp. See `CerulionSubscriber::sync_discard_head`.
    pub(crate) fn sync_discard_head(&mut self) -> TransportResult<Option<u64>> {
        match self {
            Self::Ipc(s) => s.sync_discard_head(),
        }
    }

    /// `Void`: serve a RESTORED (stamp-only, unbacked) head's read as
    /// "no frame", in place and WITHOUT draining. See
    /// `CerulionSubscriber::sync_void_head`.
    pub(crate) fn sync_void_head(&mut self) {
        match self {
            Self::Ipc(s) => s.sync_void_head(),
        }
    }

    /// Mark this input's body subscriber as the
    /// unified half of a `DrainSource::Unified` data-trigger binding, arming
    /// the warn-once `try_receive`-misuse enforcement. Called by the graph
    /// runtime at wiring time (Unified arm only). See
    /// `CerulionSubscriber::mark_unified_bound`.
    pub(crate) fn mark_unified_bound(&mut self, node_id: Arc<str>, input: Arc<str>) {
        match self {
            Self::Ipc(s) => s.mark_unified_bound(node_id, input),
        }
    }

    /// Mark this input's body subscriber as a per-message FIFO consumer (a
    /// Data-policy trigger input). Called by the graph runtime at wiring
    /// time for every data-trigger binding, on BOTH drain disciplines. See
    /// `CerulionSubscriber::mark_fifo_consume`.
    pub(crate) fn mark_fifo_consume(&mut self) {
        match self {
            Self::Ipc(s) => s.mark_fifo_consume(),
        }
    }

    /// Forward `CerulionSubscriber::mark_per_set_sync_trigger`.
    pub(crate) fn mark_per_set_sync_trigger(&mut self) {
        match self {
            Self::Ipc(s) => s.mark_per_set_sync_trigger(),
        }
    }

    /// Install this input's SERVICE CURSOR. Called by the graph
    /// runtime at wiring time, beside [`Self::mark_fifo_consume`], so exactly
    /// the per-message FIFO inputs carry one. See
    /// `CerulionSubscriber::register_service_cursor`.
    pub(crate) fn register_service_cursor(&mut self, cursor: Arc<std::sync::atomic::AtomicU64>) {
        match self {
            Self::Ipc(s) => s.register_service_cursor(cursor),
        }
    }
}

/// Runtime context provided to a node during initialization.
///
/// Contains pre-created publishers and subscribers for the node's declared
/// inputs and outputs, keyed by their port name (not the full topic path).
///
/// Runtime additions:
/// - `clock` — shared `Arc<dyn Clock>` so nodes get the same time source
///   the scheduler uses (`VirtualClock` in tests, `RealClock` in
///   production). Read via `clock()`.
/// - `shutdown_signal` — cooperative shutdown trigger; set via
///   `request_shutdown()`, polled by the runtime between ticks.
///
/// Replay-determinism addition:
/// - `env_snapshot` — frozen-at-build environment-variable snapshot.
///   `env()` / `env_str()` read from this rather than `std::env::var`
///   so per-tick env reads are replay-deterministic (Principle #7:
///   Replay = Live). The runtime's `build()`
///   captures the live env once at construction; subsequent shell
///   `setenv` calls don't leak into the running graph.
///
/// # Framework test harnesses only: `for_tests` uses `RealClock`
///
/// A node author never builds a `NodeContext`; the runtime hands one to
/// `init`. This note is for the framework's own tests.
///
/// **WARNING**: tests that build a `NodeContext` via the test-only
/// `NodeContext::for_tests(pubs, subs)` get a `RealClock`
/// (CLOCK_MONOTONIC). If the surrounding test harness uses
/// `VirtualClock` elsewhere (e.g. the scheduler), the node will
/// see DIFFERENT clock values from the scheduler — silently breaking
/// deterministic-time tests and replay equivalence. Use
/// `with_runtime_env(pubs, subs, clock, signal, env_snapshot)` and
/// pass the same `Arc<dyn Clock>` your scheduler holds.
///
/// `for_tests` is the right choice ONLY for tests that don't touch
/// the clock at all (no `real_ns` / `virt_ns` calls). If your node
/// uses `self.virt_ns()`, use `with_runtime_env`.
pub struct NodeContext {
    publishers: IndexMap<String, AnyPublisher>,
    subscribers: IndexMap<String, AnySubscriber>,
    clock: Arc<dyn Clock>,
    shutdown_signal: ShutdownSignal,
    /// Frozen env-var snapshot.
    /// `Arc` so multiple node contexts
    /// in the same graph share one allocation. Per-tick `env()` reads
    /// consult this map rather than `std::env::var` so replay is
    /// deterministic even when the shell environment changes between
    /// runs (Principle #7).
    ///
    /// This is `Arc<HashMap>`, not `Option<Arc<HashMap>>`.
    /// A `None` variant would have to fall back to live
    /// `std::env::var`, which silently leaks non-determinism into
    /// hand-rolled callers. No `default` / `new` /
    /// `with_runtime` / `from_ipc` constructor exists to
    /// produce such a state; the only
    /// constructors are `with_runtime_env` (production canonical,
    /// fully-explicit) and `for_tests` (test-only, empty snapshot).
    /// Code that genuinely needs live env must call
    /// `with_runtime_env(.., Arc::new(std::env::vars().collect()))`.
    /// The runtime does this automatically; only hand-rolled
    /// NodeContext callers need to supply it.
    env_snapshot: Arc<std::collections::HashMap<String, String>>,
    /// Shared per-node store for the edge-triggered
    /// QoS watchdog events (`ExpectWithinEvent` / `PromiseWithinEvent`)
    /// and the liveliness-transition events (`LivelinessEvent`,
    /// drained via `take_liveliness_event`).
    /// The scheduler's `step()` `push_*`es into this on a watchdog miss;
    /// the node drains it from its tick body via
    /// `take_{expect,promise}_within_event`. The runtime mints one
    /// `Arc<QosEventStore>` per node, hands the same `Arc` to the
    /// scheduler (`register_qos_event_store`), and injects this clone via
    /// `set_qos_event_store` BEFORE `init()` moves the context.
    ///
    /// **Unconditional field (NOT cfg-gated)** — `NodeContext` crosses the
    /// cdylib `init()` FFI boundary as a raw `Box`, so host and cdylib
    /// MUST agree on the layout. A cfg-gated field here would desync the
    /// struct between a test-built host and a release-built cdylib and
    /// corrupt the Box (the struct-layout lesson). Default-minted by
    /// the constructor so the ~21 `with_runtime_env` / 39 `for_tests`
    /// callers need no change; the runtime overrides it with the shared
    /// `Arc` for nodes that actually have QoS windows.
    qos_events: Arc<QosEventStore>,
    /// This node's graph id, injected by the runtime via
    /// [`Self::set_node_id_for_recon`] BEFORE `init()` moves the context (mirrors
    /// `set_qos_event_store`). Travels with the `Box` across the cdylib `init()`
    /// FFI, so the cdylib's own context carries it and can name the node in its
    /// teardown reconciliation log. `Some` ⇒ a real runtime-built node; `None` ⇒
    /// a hand-rolled/test context (its teardown log is deliberately suppressed —
    /// see `log_reconciliation_stats_at_teardown`).
    ///
    /// **Unconditional field (NOT cfg-gated)** — same FFI-layout invariant as
    /// `qos_events`: `NodeContext` crosses the cdylib `init()` boundary as a raw
    /// `Box`, so host and cdylib must agree on the layout.
    node_id: Option<String>,
    /// At-most-once guard for the per-publisher teardown
    /// reconciliation log. Set true either when the HOST harvest surfaces these
    /// stats (in-process nodes — [`Self::collect_publisher_recon_stats`]) OR when
    /// the node-side teardown log fires first (cdylib nodes —
    /// `log_reconciliation_stats_at_teardown`, reached from
    /// `impl Drop`). Whichever runs first wins, so a publisher's producer terms
    /// are logged EXACTLY once (host per-topic OR node teardown, never both).
    /// `AtomicBool` (not `Cell`) keeps `NodeContext: Send + Sync` and is writable
    /// through the `&self` harvest path. Also set at BUILD, before `init()`
    /// moves the context, for a planning-only runtime (see
    /// [`Self::silence_teardown_reconciliation`]): a build that will never
    /// run has nothing to reconcile, and its teardown stays silent.
    /// Unconditional field (same FFI-layout invariant as above).
    recon_logged: std::sync::atomic::AtomicBool,
    /// The HOST's process `TransportManager`, carried in the
    /// context so CDYLIB nodes never resolve cross-linkage-unit statics —
    /// the cdylib links its OWN copy of cerulion_core, whose `INSTANCE`
    /// static the host never initialized (`get()` fails there forever, and
    /// `get_or_init()` would mint a SECOND manager on the DEFAULT SHM root,
    /// silently splitting namespaces). Ports already cross the FFI through
    /// this context for exactly this reason; the manager rides the same
    /// path. Injected by the runtime via [`Self::set_transport`] BEFORE
    /// `init()` moves the context (mirrors `set_qos_event_store`); `None`
    /// only for hand-rolled/test contexts that never set it.
    ///
    /// **Unconditional field (NOT cfg-gated)** — same FFI-layout invariant
    /// as `qos_events`: `NodeContext` crosses the cdylib `init()`
    /// boundary as a raw `Box`, so host and cdylib must agree on the layout.
    transport: Option<Arc<TransportManager>>,
}

// The manual `unsafe impl Send for NodeContext` was deleted —
// the transport ports are now `iceoryx2::service::ipc_threadsafe::Service`
// (MutexProtected, Send+Sync), so `NodeContext` is `Send` by construction.

impl Drop for NodeContext {
    /// Surface this node's producer reconciliation terms from INSIDE
    /// the process that owns the publishers.
    ///
    /// For a CDYLIB node the context is dropped inside the cdylib when
    /// `cerulion_node_shutdown` reclaims + drops the boxed entry, so this
    /// `info!` reaches the run log via the cdylib-local stderr
    /// subscriber — closing the host-harvest parity gap for `graph run`
    /// (all-cdylib) graphs. For an in-process node it runs host-side at graph
    /// teardown; the `recon_logged` guard makes it a no-op when the host harvest
    /// already surfaced these stats (the record path calls
    /// `log_publisher_reconciliation_stats` first), so each publisher is logged
    /// exactly once. `node_id`-less (`for_tests`) contexts are skipped, so
    /// dropping a unit-test context is silent.
    fn drop(&mut self) {
        self.log_reconciliation_stats_at_teardown();
    }
}

// There are deliberately NO `Default` / `new` / `with_runtime` /
// `from_ipc` constructors. The two-argument `for_tests`
// (test-only helper) and the fully-explicit `with_runtime_env`
// (production canonical) are the only public constructors. A
// convenience constructor would silently pick `RealClock` + empty env,
// leaking non-determinism into any caller who didn't explicitly opt
// into `with_runtime_env`. Forcing callers through one of the two
// remaining shapes makes the determinism contract type-enforced.

impl NodeContext {
    /// Create a context with explicit clock + signal + env snapshot.
    ///
    /// This is the canonical production
    /// constructor; there is no `new` / `with_runtime` / `from_ipc`
    /// shorthand. For test ergonomics use
    /// `NodeContext::for_tests` (test-only helper, gated behind
    /// `#[cfg(any(test, feature = "test-helpers"))]`, that constructs
    /// a minimal context with `RealClock`, fresh `ShutdownSignal`,
    /// and empty env snapshot — link omitted because the cfg-gate
    /// hides it from the default rustdoc build).
    ///
    /// `env_snapshot` is shared by Arc across every node in the same
    /// graph (the runtime captures live env once at build time and
    /// hands the same Arc to every NodeContext). Per-tick `env()` /
    /// `env_str()` reads consult this map rather than the live
    /// process env, so replay is deterministic.
    pub fn with_runtime_env(
        publishers: IndexMap<String, AnyPublisher>,
        subscribers: IndexMap<String, AnySubscriber>,
        clock: Arc<dyn Clock>,
        shutdown_signal: ShutdownSignal,
        env_snapshot: Arc<std::collections::HashMap<String, String>>,
    ) -> Self {
        Self {
            publishers,
            subscribers,
            clock,
            shutdown_signal,
            env_snapshot,
            // A fresh, isolated store by default. The
            // runtime overrides it with the shared per-node `Arc` via
            // `set_qos_event_store` for nodes with QoS windows; callers
            // that never wire the scheduler (tests) keep this empty store,
            // and the `take_*_within_event` accessors simply return `None`.
            qos_events: Arc::new(QosEventStore::default()),
            // Unset until the runtime injects it (before `init()`).
            // A hand-rolled/test context keeps `None` and stays silent at
            // teardown (see `log_reconciliation_stats_at_teardown`).
            node_id: None,
            recon_logged: std::sync::atomic::AtomicBool::new(false),
            // Unset until the runtime injects it (before `init()`)
            // — see the field doc; `None` for hand-rolled/test contexts.
            transport: None,
        }
    }

    /// Test-only ergonomic constructor. Builds a
    /// `NodeContext` with a `RealClock`, a fresh `ShutdownSignal`, and
    /// an empty env snapshot. Replaces the 30+ test sites that
    /// previously called `NodeContext::for_tests(pubs, subs)`.
    ///
    /// Production code MUST go through [`Self::with_runtime_env`].
    /// This helper is gated behind `#[cfg(any(test, feature = "test-helpers"))]`
    /// so it cannot accidentally be used in production builds.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn for_tests(
        publishers: IndexMap<String, AnyPublisher>,
        subscribers: IndexMap<String, AnySubscriber>,
    ) -> Self {
        Self::with_runtime_env(
            publishers,
            subscribers,
            Arc::new(RealClock),
            ShutdownSignal::new(),
            Arc::new(std::collections::HashMap::new()),
        )
    }

    /// Get a shared reference to a publisher by output name.
    pub fn publisher(&self, name: &str) -> Option<&AnyPublisher> {
        self.publishers.get(name)
    }

    /// Get a mutable reference to a publisher by output name.
    pub fn publisher_mut(&mut self, name: &str) -> Option<&mut AnyPublisher> {
        self.publishers.get_mut(name)
    }

    /// Drive `pump_history` on every publisher this node owns
    /// (runtime cadence for quiescent-publisher late-joiner history delivery).
    /// Off the firing path.
    pub fn pump_history(&mut self) {
        for p in self.publishers.values_mut() {
            p.pump_history();
            // Same boundary, re-check the notify-elision gate.
            // A foreign LISTENER-full subscriber that attached while the producer
            // was quiescent — or off the `notify_sent_sample` gate (e.g. a
            // raw-frame `publish_raw`) — gets notifies flowing within one
            // heartbeat instead of blocking forever on the topic's event
            // listener. A no-op per publisher unless elision is armed AND a
            // foreign listener is present. Rides this path (not a separate FFI
            // export) so the ABI-v7 cdylib `cerulion_node_pump_history` call
            // heals a cdylib's publishers too, no ABI bump.
            p.resweep_notify_elision();
        }
    }

    /// Append this node's per-publisher teardown reconciliation
    /// snapshot ([`PublisherReconStat`]) — one entry per owned publisher — to
    /// `out`. Diagnostics only; iterates in `IndexMap` (graph declaration)
    /// order. Called by the sole [`NodeEntry::collect_publisher_recon_stats`]
    /// forwarder — `ClosureNodeEntry` (the host harvest cannot reach a cdylib's
    /// transferred context, and in-process macro nodes surface via the
    /// `NodeContext` Drop path instead of this forwarder).
    ///
    /// Marks `recon_logged` on exit: once the host harvest has surfaced these
    /// stats to the run log, the node-side teardown fallback
    /// (`log_reconciliation_stats_at_teardown`) MUST NOT re-log them —
    /// this is the at-most-once coordination between the two paths.
    pub fn collect_publisher_recon_stats(&self, out: &mut Vec<PublisherReconStat>) {
        // Guard hardening: IDEMPOTENT. Claim the run-log surface with a
        // test-and-set BEFORE pushing. A second harvest — or a harvest after the
        // node-side teardown fallback already logged — returns WITHOUT pushing,
        // so a publisher can never be double-counted in `out`. (Unreachable in
        // the current tree: the runtime harvests each entry once; this is
        // defense-in-depth against a future double-harvest.) The same swap
        // suppresses the node-side teardown fallback
        // (`log_reconciliation_stats_at_teardown`), so each publisher is
        // surfaced exactly once across the two paths.
        if self
            .recon_logged
            .swap(true, std::sync::atomic::Ordering::Relaxed)
        {
            return;
        }
        for p in self.publishers.values() {
            out.push(PublisherReconStat {
                topic: p.topic().to_string(),
                next_sequence: p.sequence(),
                initial_sequence: p.initial_sequence(),
                committed_frames: p.committed_frames(),
                frames_dropped_overflow: p.frames_dropped_overflow(),
                frames_dropped_send_fail: p.frames_dropped_send_fail(),
            });
        }
    }

    /// Log this node's per-publisher producer reconciliation snapshot
    /// at graph teardown, from INSIDE the process that owns the publishers, at
    /// most once.
    ///
    /// This closes the host-harvest parity gap: a `graph run` node is a CDYLIB
    /// that takes ownership of its `NodeContext` at `init()`
    /// (`Box::into_raw` through `cerulion_node_init`), so the host-side harvest
    /// ([`crate::graph::runtime::GraphRuntime::log_publisher_reconciliation_stats`])
    /// cannot reach it. But `cerulion_core` runs INSIDE the cdylib, which
    /// installs a cdylib-local stderr `tracing` subscriber at
    /// `cerulion_node_init` — so an `info!` emitted here on the cdylib-side
    /// teardown path (reached via `impl Drop for NodeContext` when
    /// `cerulion_node_shutdown` drops the boxed context) is HOST-VISIBLE in the
    /// run log, exactly where the decision matrix reads producer terms.
    ///
    /// The emitted message shares the marker
    /// `"producer reconciliation (per topic)"` with the host harvest, so
    /// ONE grep collects producer terms from both in-process and cdylib graphs.
    ///
    /// ONE compact line per topic, carrying only the structured counters: it
    /// prints at the default filter of every `graph run`, so the line must be
    /// cheap to read. How the counters relate to the bag-side terms (the loss
    /// identity, the replay-seed rule, the u32 wrap) is documented once, in
    /// `docs/internals/recording.md` under "Producer reconciliation lines",
    /// rather than restated on every line of every run.
    ///
    /// At-most-once: the `recon_logged` swap makes this a no-op if the host
    /// harvest already surfaced these stats (in-process record path), if this
    /// method already ran, or if the runtime was built planning-only (the
    /// context was marked at build, see [`Self::silence_teardown_reconciliation`]:
    /// a graph that never ran has nothing to reconcile, and its all-zero lines
    /// would read as a run's result). A `node_id`-less context (hand-rolled /
    /// `for_tests`) is skipped entirely, so unit-test contexts add no teardown
    /// noise.
    ///
    /// Diagnostics only: never read on any firing path (determinism firewall).
    fn log_reconciliation_stats_at_teardown(&self) {
        use std::sync::atomic::Ordering;
        // A producer-less node (no outputs) has nothing to reconcile.
        if self.publishers.is_empty() {
            return;
        }
        // Only real runtime-built nodes carry an id; `for_tests`/hand-rolled
        // contexts stay silent (return BEFORE touching the guard so they never
        // interfere with a real node's at-most-once bookkeeping).
        let node_id = match &self.node_id {
            Some(id) => id.as_str(),
            None => return,
        };
        // At-most-once test-and-set: skip if already surfaced (host harvest) or
        // already logged here.
        if self.recon_logged.swap(true, Ordering::Relaxed) {
            return;
        }
        for p in self.publishers.values() {
            // `sequence()` is the NEXT seq to assign and the highest committed
            // seq is `sequence() - 1` (undefined if the topic never published).
            // The COMMITTED COUNT is `committed_frames()`, which subtracts the
            // seed — equal to `sequence()` on every live run (seed 0) and NOT
            // equal on a restored replay, where reporting
            // the raw counter would mint the whole seed as phantom loss.
            tracing::info!(
                node_id = %node_id,
                topic = %p.topic(),
                producer_next_sequence = p.sequence(),
                producer_initial_sequence = p.initial_sequence(),
                producer_committed_frames = p.committed_frames(),
                frames_dropped_send_fail = p.frames_dropped_send_fail(),
                frames_dropped_overflow = p.frames_dropped_overflow(),
                "producer reconciliation (per topic)"
            );
        }
    }

    /// Get a shared reference to a subscriber by input name.
    pub fn subscriber(&self, name: &str) -> Option<&AnySubscriber> {
        self.subscribers.get(name)
    }

    /// Get a mutable reference to a subscriber by input name.
    ///
    /// Required for `AnySubscriber::try_view`, which mutably drains the
    /// inbound queue before constructing the SHM-backed `InputView`.
    pub fn subscriber_mut(&mut self, name: &str) -> Option<&mut AnySubscriber> {
        self.subscribers.get_mut(name)
    }

    /// Freeze a step-boundary snapshot of each listed
    /// input's latest sample (the runtime-supplied NON-triggering inputs). The
    /// node's next `try_view` of each serves the frozen sample without
    /// re-draining (the accounting-once contract — see
    /// [`crate::transport::subscriber::CerulionSubscriber::snapshot_latest`]). Called by the
    /// `#[cerulion_node]`-generated `NodeEntry::snapshot_inputs` (in-process) and
    /// by the cdylib `cerulion_node_snapshot_inputs` FFI export on a
    /// cdylib node's `NodeContext` (so cdylib nodes hold non-trigger inputs too).
    ///
    /// The runtime supplies these names
    /// from `build_snapshot_input_names`, the policy-mirror of the wiring that
    /// populated `self.subscribers` — so every name SHOULD be a wired
    /// subscriber. An unknown name is therefore a classifier/wiring desync, not
    /// a benign mismatch: it `debug_assert!(false)` (fails loud in debug) and
    /// `tracing::warn!`s + skips in release rather than failing silently.
    pub fn snapshot_inputs(&mut self, inputs: &[String]) {
        for name in inputs {
            match self.subscribers.get_mut(name) {
                Some(sub) => sub.snapshot_latest(),
                None => {
                    debug_assert!(
                        false,
                        "snapshot_inputs: input '{name}' not a wired subscriber — classifier/wiring desync (build_snapshot_input_names emitted a name absent from NodeContext.subscribers)"
                    );
                    tracing::warn!(
                        input = %name,
                        "snapshot_inputs: unknown input name skipped — classifier/wiring desync"
                    );
                }
            }
        }
    }

    /// Drain a single data-trigger input's body subscriber at the
    /// level boundary. The runtime calls this (through the node lock, BEFORE
    /// decide) so the one receive serves BOTH the trigger — via the returned
    /// `(popped, latest_ts)` (`popped` = arrival count for `signal_data`,
    /// `latest_ts` = the `signal_input_received` watchdog reset) — AND the
    /// tick's later `try_view`, which serves the now-frozen sample without a
    /// second receive (the accounting-once contract — see
    /// [`crate::transport::subscriber::CerulionSubscriber::snapshot_latest_for_trigger`]).
    ///
    /// Like [`Self::snapshot_inputs`], `input_name` SHOULD be a wired
    /// subscriber (the runtime supplies it from the trigger-edge wiring); an
    /// unknown name is a classifier/wiring desync, so it `debug_assert!(false)`
    /// (fails loud in debug), `tracing::warn!`s, and returns `(0, None)` in
    /// release rather than failing silently.
    pub fn drain_trigger_input(&mut self, input_name: &str) -> (u64, Option<u64>) {
        match self.subscribers.get_mut(input_name) {
            Some(sub) => sub.snapshot_latest_for_trigger(),
            None => {
                debug_assert!(
                    false,
                    "drain_trigger_input: input '{input_name}' not a wired subscriber — classifier/wiring desync (trigger edge emitted a name absent from NodeContext.subscribers)"
                );
                tracing::warn!(
                    input = %input_name,
                    "drain_trigger_input: unknown input name skipped — classifier/wiring desync"
                );
                (0, None)
            }
        }
    }

    /// Refill a data-trigger input BETWEEN two fires of one step —
    /// the scheduler's Data burst loop asking for the next queued frame after
    /// the fire it just ran.
    ///
    /// Identical to [`Self::drain_trigger_input`] except on one state: a head
    /// this input froze that the tick never read. The boundary drain RE-OFFERS
    /// such a head (it is still owed a fire); a refill must report `(0, None)`,
    /// because re-offering it here would fire the node again on a frame it has
    /// already been fired for — see
    /// [`crate::transport::subscriber::CerulionSubscriber::refill_for_trigger`].
    ///
    /// Same desync posture as its sibling: an unknown name is loud in debug and
    /// `(0, None)` in release (which simply ends the burst — never a fabricated
    /// fire).
    pub fn refill_trigger_input(&mut self, input_name: &str) -> (u64, Option<u64>) {
        match self.subscribers.get_mut(input_name) {
            Some(sub) => sub.refill_for_trigger(),
            None => {
                debug_assert!(
                    false,
                    "refill_trigger_input: input '{input_name}' not a wired subscriber — classifier/wiring desync (trigger edge emitted a name absent from NodeContext.subscribers)"
                );
                tracing::warn!(
                    input = %input_name,
                    "refill_trigger_input: unknown input name skipped — classifier/wiring desync"
                );
                (0, None)
            }
        }
    }

    /// Perform ONE per-set Sync head op on `input_name` and answer
    /// what the matcher's verdict asked for.
    ///
    /// The matcher is a PURE function over `(heads, next_info, window)`; it
    /// cannot touch transport, so it asks for facts by returning a verdict and
    /// the align driver performs the op through this seam and re-runs it. ONE
    /// multiplexed entry point rather than six, mirroring the ONE FFI symbol it
    /// crosses on a cdylib — the six are transitions of one state machine, and
    /// separate hooks would admit partial-capability nodes (advance-without-void)
    /// the degrade logic would then have to enumerate.
    ///
    /// Same desync posture as its two siblings above ([`Self::drain_trigger_input`]
    /// / [`Self::refill_trigger_input`]): an unknown name is a classifier/wiring
    /// desync, so it `debug_assert!(false)` (loud in debug) and `tracing::warn!`s.
    /// The safe release answer here is [`SyncOpAnswer::Failed`] rather than a
    /// fabricated fact, because `Failed` is the ONE answer the align driver's
    /// R-Fail policy maps FAIL-CLOSED at every site — `None` at the argmin (fire
    /// the tuple we can already prove) and `Present` at the gate (this input
    /// REFUSES the descent). A `Nothing` here would instead VOUCH FOR SCARCITY on
    /// an input nothing observed, which is exactly what destroys an arrived
    /// complete in-window set.
    pub fn sync_head_op(&mut self, input_name: &str, op: SyncHeadOp) -> SyncOpAnswer {
        let Some(sub) = self.subscribers.get_mut(input_name) else {
            debug_assert!(
                false,
                "sync_head_op: input '{input_name}' not a wired subscriber — classifier/wiring desync (a Sync op list emitted a name absent from NodeContext.subscribers)"
            );
            tracing::warn!(
                input = %input_name,
                op = ?op,
                "sync_head_op: unknown input name — classifier/wiring desync; answering \
                 Failed, which DISABLES the descent rather than vouching for a fact \
                 nothing observed"
            );
            return SyncOpAnswer::Failed;
        };

        match op {
            SyncHeadOp::FillBoundary => {
                let (popped, latest_ts) = sub.snapshot_latest_for_trigger();
                sync_fill_answer(popped, latest_ts)
            }
            SyncHeadOp::FillRefill => {
                let (popped, latest_ts) = sub.refill_for_trigger();
                sync_fill_answer(popped, latest_ts)
            }
            SyncHeadOp::ProbeNext => {
                let outcome = sub.sync_next_arrived();
                match outcome {
                    Ok(true) => SyncOpAnswer::Present,
                    Ok(false) => SyncOpAnswer::Nothing,
                    Err(e) => {
                        // The align driver flood-latches the FAILURE but cannot
                        // see the CAUSE — an operator debugging a node stuck on
                        // greedy membership needs it, so the error never vanishes.
                        tracing::debug!(
                            input = %input_name,
                            op = ?op,
                            error = ?e,
                            "sync_head_op: the non-consuming next-arrived probe errored"
                        );
                        SyncOpAnswer::Failed
                    }
                }
            }
            SyncHeadOp::PeekNext => {
                let outcome = sub.sync_peek_next_stamp();
                match outcome {
                    Ok(Some(ts)) => SyncOpAnswer::Stamp(ts),
                    Ok(None) => SyncOpAnswer::Nothing,
                    Err(e) => {
                        tracing::debug!(
                            input = %input_name,
                            op = ?op,
                            error = ?e,
                            "sync_head_op: the next-stamp peek errored"
                        );
                        SyncOpAnswer::Failed
                    }
                }
            }
            SyncHeadOp::Advance => {
                let outcome = sub.sync_discard_head();
                match outcome {
                    Ok(Some(ts)) => SyncOpAnswer::Head(ts),
                    Ok(None) => SyncOpAnswer::Nothing,
                    Err(e) => {
                        tracing::debug!(
                            input = %input_name,
                            op = ?op,
                            error = ?e,
                            "sync_head_op: the head advance errored"
                        );
                        SyncOpAnswer::Failed
                    }
                }
            }
            SyncHeadOp::Void => {
                sub.sync_void_head();
                // A void SERVES a read; it produces no head, and the driver
                // reads it as "this input contributes nothing further".
                SyncOpAnswer::Nothing
            }
        }
    }

    /// Take the edge-triggered [`BackpressureEvent`] for
    /// input `name`, if one is pending. All three policies queue events,
    /// one per regime: the first `sample(N)` decimate of a regime, the
    /// first `drop_oldest` eviction of a regime (`dropped > 0`), or the
    /// first at-threshold `block` drain of a regime (`dropped == 0` —
    /// lossless). The manual escape hatch behind `#[on_event]` (with a
    /// `BackpressureEvent` parameter); returns `None` for an unknown input
    /// or no pending event. Metadata only — no data path.
    #[must_use = "BackpressureEvent is a user-visible data-loss signal; discarding it silently loses observability"]
    pub fn take_backpressure_event(&mut self, name: &str) -> Option<BackpressureEvent> {
        self.subscribers
            .get_mut(name)
            .and_then(|s| s.try_take_backpressure_event())
    }

    /// Install the shared per-node `QosEventStore`.
    /// Called by `GraphRuntime::build` BEFORE `init()` moves the context,
    /// handing it the SAME `Arc` that the scheduler holds — so a watchdog
    /// miss pushed by `step()` is drained here. The default-minted store
    /// from the constructor is replaced. Not part of the user API.
    pub(crate) fn set_qos_event_store(&mut self, store: Arc<QosEventStore>) {
        self.qos_events = store;
    }

    /// Stamp this node's graph id, so its teardown reconciliation log
    /// (and, for a cdylib node, the cdylib-side one) can name the node. Called
    /// by `GraphRuntime::build_with_scheduler` BEFORE `init()` moves the context
    /// (mirrors [`Self::set_qos_event_store`]); the value travels with the `Box`
    /// across the cdylib `init()` FFI. Not part of the user API.
    pub(crate) fn set_node_id_for_recon(&mut self, node_id: String) {
        self.node_id = Some(node_id);
    }

    /// Declare this context's teardown reconciliation line already surfaced,
    /// so a runtime that will never run prints none.
    ///
    /// Called by `GraphRuntime::build_with_scheduler` for a
    /// [`super::runtime::BuildPurpose::PlanningOnly`] build (the multi-process
    /// supervisor's planning build, which runs every node's
    /// `init()` and is torn down before any worker spawns), BEFORE `init()`
    /// moves the context: for a cdylib node that is the last moment the host
    /// can reach it. Reuses the `recon_logged` at-most-once guard rather than
    /// adding a field, because `NodeContext` crosses the cdylib `init()` FFI
    /// as a raw `Box` and a new field is a layout change. Not part of the user
    /// API.
    pub(crate) fn silence_teardown_reconciliation(&self) {
        self.recon_logged
            .store(true, std::sync::atomic::Ordering::Relaxed);
    }

    /// Install the HOST's process [`TransportManager`] (see the
    /// `transport` field doc — the context-carried transport contract).
    /// Called by the runtime BEFORE `init()` moves the context (mirrors
    /// `Self::set_qos_event_store`); the `Arc` travels with the `Box`
    /// across the cdylib `init()` FFI. `pub` (not `pub(crate)`) because
    /// host EMBEDDINGS that hand-roll contexts via
    /// [`Self::with_runtime_env`] must be able to carry their manager the
    /// same way (and tests exercise the cdylib contract through it).
    pub fn set_transport(&mut self, transport: Arc<TransportManager>) {
        self.transport = Some(transport);
    }

    /// The HOST's process [`TransportManager`], carried in this
    /// context so cdylib nodes never resolve cross-linkage-unit statics (the
    /// cdylib's own `TransportManager::get()` can NEVER see the host's
    /// singleton — and `get_or_init()` there would mint a second manager on
    /// the wrong SHM namespace). `None` only for hand-rolled/test contexts
    /// that never called [`Self::set_transport`]; every runtime-built
    /// context carries `Some`.
    pub fn transport(&self) -> Option<&Arc<TransportManager>> {
        self.transport.as_ref()
    }

    /// Take the edge-triggered [`ExpectWithinEvent`]
    /// for input `name`, if one is pending. Fires once per silence regime
    /// (the first `step()` miss after data goes quiet on a
    /// `#[input(expect_within_ms = N)]` window) and rearms when fresh data
    /// arrives. The counter twin (`NodeHandle::expect_within_missed_count`)
    /// bumps on EVERY missed window regardless of drains. Metadata only —
    /// no data path. Returns `None` for an unknown input or no pending
    /// event.
    ///
    /// Unlike [`Self::take_backpressure_event`] (which delegates to the
    /// subscriber's receive-path slot), this drains a NodeContext-local
    /// store written by the scheduler — the watchdog miss is detected in
    /// `step()`, not on the subscriber's hot path. The escape hatch the
    /// `#[on_event]` macro calls.
    #[must_use = "ExpectWithinEvent is a user-visible liveliness signal; discarding it silently loses observability"]
    pub fn take_expect_within_event(&mut self, name: &str) -> Option<ExpectWithinEvent> {
        self.qos_events.take_expect(name)
    }

    /// Take the edge-triggered [`PromiseWithinEvent`]
    /// for output `name`, if one is pending. Output twin of
    /// [`Self::take_expect_within_event`] — fires once per silence regime
    /// on a `#[output(promise_within_ms = N)]` window and rearms on the
    /// next publish.
    #[must_use = "PromiseWithinEvent is a user-visible liveliness signal; discarding it silently loses observability"]
    pub fn take_promise_within_event(&mut self, name: &str) -> Option<PromiseWithinEvent> {
        self.qos_events.take_promise(name)
    }

    /// Take the edge-triggered [`LivelinessEvent`] for
    /// input `name`, if one is pending. Fires once per liveliness transition
    /// (a publisher (re)connected, or the last one disconnected) on the
    /// input's topic. Metadata only — no data path. Returns `None` for an
    /// unknown input or no pending event.
    ///
    /// Like [`Self::take_expect_within_event`] (and unlike
    /// [`Self::take_backpressure_event`], which delegates to the subscriber's
    /// receive-path slot), this drains a NodeContext-local store written by
    /// the producer side — the transition is detected off the subscriber's
    /// hot path. The escape hatch the `#[on_event]` macro calls for a
    /// `LivelinessEvent` handler.
    #[must_use = "LivelinessEvent is a user-visible liveliness signal; discarding it silently loses observability"]
    pub fn take_liveliness_event(&mut self, name: &str) -> Option<LivelinessEvent> {
        self.qos_events.take_liveliness(name)
    }

    /// Split into disjoint `&mut` views of the publishers and subscribers
    /// indexmaps.
    ///
    /// Required by the zero-copy tick wrapper that
    /// `#[cerulion_node_impl]` generates when both inputs and outputs
    /// are present: the wrapper holds `OutputProxy` values (borrowing
    /// publishers) and `InputView` values (borrowing subscribers)
    /// concurrently. With `publisher_mut`/`subscriber_mut` going through
    /// `&mut self`, the second call would conflict with the first borrow
    /// extending through the proxy's lifetime. This helper hands out a
    /// single pair of disjoint field borrows so the caller can take what
    /// it needs from each map (typically via `IndexMap::get_disjoint_mut`)
    /// without re-entering `&mut self`.
    pub fn split_publishers_subscribers_mut(
        &mut self,
    ) -> (
        &mut IndexMap<String, AnyPublisher>,
        &mut IndexMap<String, AnySubscriber>,
    ) {
        (&mut self.publishers, &mut self.subscribers)
    }

    /// Returns an iterator over all publisher names.
    pub fn publisher_names(&self) -> impl Iterator<Item = &str> {
        self.publishers.keys().map(|s| s.as_str())
    }

    /// Returns an iterator over all subscriber names.
    pub fn subscriber_names(&self) -> impl Iterator<Item = &str> {
        self.subscribers.keys().map(|s| s.as_str())
    }

    /// Read an env var and parse it as `T`; fall back to `default` on
    /// missing or unparseable values.
    ///
    /// Parse failures emit a `tracing::warn!` so misconfigured values are
    /// visible in logs without crashing the node. For string-valued env
    /// vars, prefer [`Self::env_str`] — it skips the `FromStr` round-trip.
    ///
    /// Reads the snapshot of the environment taken when the graph was built,
    /// never live `std::env`, so a replay reads the same value. A key that was
    /// unset at build time returns `default` for the whole run.
    ///
    /// # Example
    ///
    /// Call it from a node's `init`:
    ///
    /// ```rust
    /// use cerulion_core::prelude::*;
    /// use native_ros2_messages::geometry_msgs::Vector3;
    ///
    /// #[cerulion_node(period_ms = 100)]
    /// #[derive(Default)]
    /// struct GeneratorNode {
    ///     #[output]
    ///     reading: Vector3,
    ///     amplitude: f64,
    ///     label: String,
    /// }
    ///
    /// #[cerulion_node_impl]
    /// impl GeneratorNode {
    ///     fn init(&mut self, ctx: &mut NodeContext) -> Result<(), NodeError> {
    ///         self.amplitude = ctx.env("GENERATOR_AMPLITUDE", 1.0);
    ///         self.label = ctx.env_str("GENERATOR_LABEL", "unnamed");
    ///         Ok(())
    ///     }
    ///
    ///     fn tick(&mut self) -> Result<(), NodeError> {
    ///         self.reading.x = self.amplitude;
    ///         Ok(())
    ///     }
    /// }
    /// # fn main() {}
    /// ```
    pub fn env<T>(&self, key: &str, default: T) -> T
    where
        T: FromStr,
    {
        match self.env_lookup(key) {
            Some(raw) => match raw.parse::<T>() {
                Ok(v) => v,
                Err(_) => {
                    tracing::warn!(
                        env_var = %key,
                        raw = %raw,
                        target_type = std::any::type_name::<T>(),
                        "env var present but failed to parse; using default"
                    );
                    default
                }
            },
            None => default,
        }
    }

    /// String-typed env var lookup.
    ///
    /// Reads from the runtime's frozen env snapshot
    /// (for replay determinism).
    /// There is no live `std::env::var` fallback;
    /// keys absent from the snapshot return `default`.
    ///
    /// See [`Self::env`] for an example in a node's `init`.
    pub fn env_str(&self, key: &str, default: &str) -> String {
        self.env_lookup(key).unwrap_or_else(|| default.to_string())
    }

    /// String-typed env var lookup that reports absence as `None`, for
    /// callers that must tell an unset key from one set to `""`. Reads the
    /// same frozen snapshot as [`Self::env_str`].
    pub fn env_opt(&self, key: &str) -> Option<String> {
        self.env_lookup(key)
    }

    /// Env-var read backend. Always reads from the
    /// frozen snapshot — there is no live `std::env::var` fallback.
    ///
    /// A fallback to `std::env::var(key)` when `env_snapshot` is `None`
    /// would break determinism, so there is none: contexts
    /// constructed via `default()` / `new()` / `with_runtime()` /
    /// `from_ipc()` carry an EMPTY snapshot, and `env_str(key, default)`
    /// returns `default` for any key. Determinism is type-enforced —
    /// you cannot accidentally read live env without going through
    /// `with_runtime_env(.., snapshot)`.
    fn env_lookup(&self, key: &str) -> Option<String> {
        self.env_snapshot.get(key).cloned()
    }

    /// Returns the runtime clock.
    ///
    /// Same instance as the scheduler's clock — `VirtualClock` in tests,
    /// `RealClock` in production. Use this when a node needs a monotonic
    /// timestamp that's identical to the one driving scheduler decisions.
    pub fn clock(&self) -> &Arc<dyn Clock> {
        &self.clock
    }

    /// Request graceful shutdown of the entire graph.
    ///
    /// Idempotent; the runtime polls the signal between ticks and exits
    /// cleanly. Nodes typically call this from inside `tick` after some
    /// completion condition (e.g. enough samples collected).
    pub fn request_shutdown(&self) {
        self.shutdown_signal.request();
    }

    /// Returns the underlying `ShutdownSignal`.
    ///
    /// Mostly useful for the runtime's own poll loop. Nodes should prefer
    /// `request_shutdown()` directly.
    pub fn shutdown_signal(&self) -> &ShutdownSignal {
        &self.shutdown_signal
    }
}

/// What an `#[cerulion_node(external)]` ingress node watches so the
/// live loop can SELF-TRIGGER it when the source has data.
///
/// An `external`-policy node is a driver: it hands the runtime this value ONCE
/// (queried by the live loop's `collect_external_sources` after `init()`; NEVER
/// under the polled `step()` / replay path — see [`NodeEntry::external_source`]).
/// The runtime watches the source and calls `Scheduler::trigger_external` on the
/// node when the source is ready; the EXISTING deterministic
/// `TriggerPolicy::External` arm inside `step()` makes the actual fire decision,
/// so the wake is RECORD-ONLY (it changes WHEN `step()` runs, never WHAT fires —
/// Principle #7).
/// `#[non_exhaustive]` (matches the `handle.rs` convention): variants may grow —
/// `Fd` is unix-only, so a future Windows `Handle` tier must not be a breaking
/// change. Downstream matches need a wildcard arm; constructing the existing
/// variants is unaffected.
#[non_exhaustive]
pub enum ExternalSource {
    /// Tier 1: a pollable device fd (v4l2, socket, evdev, serial). Attached
    /// NON-owning to the live WaitSet: the runtime never reads it and never
    /// closes it; the node owns the device lifetime and drains it in `tick()`.
    /// (EXCEPTION: a cdylib tier-2 `Blocking`-collapse doorbell pipe ALSO arrives
    /// as an `Fd`, but is flagged via the `#[doc(hidden)]`
    /// [`NodeEntry::external_source_is_drained_doorbell_fd`] marker so the runtime
    /// OWNS it — draining AND closing it. That is an internal cdylib mechanism,
    /// NOT part of this device-fd contract.)
    /// Contract: the fd must remain valid (not closed/reopened) for the life
    /// of the live run; close it only in `shutdown()` (or in `tick()` if you
    /// also stop returning readiness).
    ///
    /// # Footguns
    ///
    /// - **Level-triggered:** readiness is re-derived from the CURRENT fd state
    ///   every live step, so an fd your `tick()` leaves readable re-fires the
    ///   node EVERY step — continuous fire until drained. Drain the device in
    ///   `tick()`.
    /// - **`tick()` runs on the single live-loop thread:** a blocking `read(2)`
    ///   inside `tick()` stalls the WHOLE graph. Set `O_NONBLOCK` (or use
    ///   bounded reads) and drain to `EAGAIN`.
    /// - **`POLLHUP`/`POLLERR` count as ready** — deliberately NOT auto-unbound,
    ///   because data buffered before a peer close must still be drainable
    ///   (Principle #6). A hung-up peer therefore leaves the fd permanently
    ///   "ready" and the node fires every step until `tick()` handles the EOF
    ///   (stop returning readiness: reopen the device, request shutdown, or
    ///   close the fd in `shutdown()`).
    Fd(std::os::unix::io::RawFd),
    /// Tier 2: for fd-less SDKs whose only wait API is a blocking call. The
    /// runtime drives this closure on a helper thread; each `true` return
    /// rings the node's doorbell (N rings before a step coalesce to one
    /// fire — buffer per-event data in your source if you need every event).
    /// Return `false` for a spurious/timeout wake. Use a bounded internal
    /// timeout so the thread can observe shutdown between calls (the helper is
    /// signalled to stop when the `GraphRuntime` drops — a runtime that is
    /// never dropped, e.g. `mem::forget` or a `'static` leak, never stops its
    /// helper: the thread's lifetime is tied to Drop). Panics are caught:
    /// the source is poisoned LOUDLY and stops waking the node.
    Blocking(Box<dyn FnMut() -> bool + Send + 'static>),
    /// No self-source: nothing fires this node on the live path.
    /// `cerulion graph run` REFUSES, at launch, a graph that contains such a
    /// node, naming the node and the reason (`host-driven`). This is the value
    /// `cerulion node create --policy external` scaffolds so that the new
    /// crate compiles; replace it with [`Fd`](Self::Fd) or
    /// [`Blocking`](Self::Blocking) before running the graph. (The framework's
    /// own tests fire such a node by calling `trigger_external()` and `step()`
    /// on a runtime they drive themselves.)
    HostDriven,
}

/// Manual impl — the `Blocking` closure blocks `#[derive(Debug)]`; its sibling
/// types all expose `Debug`, so this keeps the sacred user surface consistent.
impl std::fmt::Debug for ExternalSource {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Fd(raw) => f.debug_tuple("Fd").field(raw).finish(),
            Self::Blocking(_) => f.write_str("Blocking(<closure>)"),
            Self::HostDriven => f.write_str("HostDriven"),
        }
    }
}

// ---------------------------------------------------------------------------
// Cdylib external-source FFI kind codes + tier-2 collapse.
// ---------------------------------------------------------------------------

/// C-ABI kind codes returned by a cdylib's OPTIONAL
/// `cerulion_node_external_source(handle, out_fd) -> i32` export, telling the
/// host ([`DylibNodeEntry::external_source`]) how to interpret `*out_fd`. SHARED
/// between the `#[cerulion_node]` codegen (which emits the export for
/// `external` nodes) and the host resolver so the two never drift.
///
/// Tier 1: `*out_fd` holds a device [`RawFd`](std::os::unix::io::RawFd) the host
/// binds as [`ExternalSource::Fd`] (poll-only).
pub const EXTERNAL_SOURCE_KIND_DEVICE_FD: i32 = 1;
/// Tier 2 collapsed at the C ABI: the cdylib materialized an
/// [`ExternalSource::Blocking`] closure into a pipe-backed doorbell (a closure
/// can't cross the FFI) and returned the READ end via `*out_fd`. The host binds
/// it as a crate-internal drained-doorbell fd and DRAINS it to EAGAIN each sweep
/// (distinct from the poll-only device-fd tier). See
/// [`spawn_cdylib_blocking_doorbell`] and
/// [`NodeEntry::external_source_is_drained_doorbell_fd`].
pub const EXTERNAL_SOURCE_KIND_DOORBELL_FD: i32 = 2;
/// [`ExternalSource::HostDriven`] — `*out_fd` is UNTOUCHED; the node fires only
/// via a host `trigger_external`.
pub const EXTERNAL_SOURCE_KIND_HOST_DRIVEN: i32 = 3;
/// The export failed (its LAST_ERROR is set) — the host treats the node as
/// having no external source (`None`).
pub const EXTERNAL_SOURCE_KIND_ERROR: i32 = -1;

// ---------------------------------------------------------------------------
// Cdylib per-set Sync head-op FFI codes.
//
// SHARED between the `#[cerulion_node]` codegen (which emits the OPTIONAL
// `cerulion_node_sync_head_op` export for a Sync node) and the host resolver
// ([`DylibNodeEntry::sync_head_op`]), so the two spellings of one wire cannot
// drift. The op code goes IN, the answer kind comes OUT beside a `u64` stamp
// slot the host reads only for the two kinds that carry one.
//
// ONLY the four ops with no existing entry point cross this symbol: the two
// FILL ops ride `cerulion_node_{drain,refill}_trigger_input`, which already
// exist and already carry the boundary-vs-refill distinction the fills need.
// ---------------------------------------------------------------------------

/// [`SyncHeadOp::ProbeNext`] — the NON-CONSUMING "has a second frame arrived?"
/// probe.
pub const SYNC_HEAD_OP_PROBE_NEXT: u32 = 0;
/// [`SyncHeadOp::PeekNext`] — pop the next frame into the staged slot and
/// report its stamp.
pub const SYNC_HEAD_OP_PEEK_NEXT: u32 = 1;
/// [`SyncHeadOp::Advance`] — drop the head and refill from the staged next (or
/// the queue).
pub const SYNC_HEAD_OP_ADVANCE: u32 = 2;
/// [`SyncHeadOp::Void`] — serve a restored (unbacked) head's read as "no frame".
pub const SYNC_HEAD_OP_VOID: u32 = 3;

/// [`SyncOpAnswer::Nothing`] — the stamp slot is UNTOUCHED.
pub const SYNC_OP_ANSWER_NOTHING: i32 = 0;
/// [`SyncOpAnswer::Present`] — a second frame has arrived, stamp not yet known;
/// the stamp slot is UNTOUCHED.
pub const SYNC_OP_ANSWER_PRESENT: i32 = 1;
/// [`SyncOpAnswer::Head`] — the stamp slot carries the new head's stamp.
pub const SYNC_OP_ANSWER_HEAD: i32 = 2;
/// [`SyncOpAnswer::Stamp`] — the stamp slot carries the STAGED next frame's
/// stamp (nothing was promoted).
pub const SYNC_OP_ANSWER_STAMP: i32 = 3;

/// The ONE mapping every FILL site shares — `(popped, latest_ts)`
/// from a trigger drain into the matcher's answer vocabulary.
///
/// Shared rather than written twice because BOTH fill paths reach it: the
/// in-process [`NodeContext::sync_head_op`] (which calls the subscriber drains
/// directly) and [`DylibNodeEntry::sync_head_op`] (which calls the same drains
/// through `cerulion_node_{drain,refill}_trigger_input`). Two copies of one
/// mapping is how the two surfaces would come to disagree about what a pop
/// means.
///
/// `popped > 0` with NO readable stamp is DEFENSIVE, not a live case: the
/// pop-one drain skips any payload shorter than a `WireHeader` as junk, so
/// every `Sample` it yields carries a readable stamp. If one ever did not it
/// could not be a set member either — a frame with no stamp has no position on
/// the span the window bounds — so `Nothing` (which leaves this input unfilled
/// and refuses the descent) is the sound answer rather than a head at a
/// fabricated time.
pub(crate) fn sync_fill_answer(popped: u64, latest_ts: Option<u64>) -> SyncOpAnswer {
    if popped == 0 {
        return SyncOpAnswer::Nothing;
    }
    match latest_ts {
        Some(ts) => SyncOpAnswer::Head(ts),
        None => SyncOpAnswer::Nothing,
    }
}

/// Cdylib-side tier-2 [`ExternalSource::Blocking`] collapse.
///
/// A closure cannot cross the C ABI, so when a cdylib node returns
/// `ExternalSource::Blocking`, its generated `cerulion_node_external_source`
/// export calls THIS to materialize the closure into a `pipe(2)` + a DETACHED
/// helper thread, and returns the READ end to the host. The helper drives
/// `closure` under `catch_unwind`; each `true` return writes one byte to the
/// write end (waking the host's live-loop WaitSet, which watches the read end);
/// `false` is a spurious/timeout wake (no byte). The host sets `O_NONBLOCK` on
/// the read end at collect time and DRAINS it to EAGAIN each sweep, so the sweep
/// NEVER blocks.
///
/// Returns `Some(read_end`[`RawFd`](std::os::unix::io::RawFd)`)`, or `None` on
/// failure (pipe creation error, logged) — the caller collapses `None` to the
/// error kind code so the host treats the node as having no external source.
///
/// # Lifecycle (no explicit stop flag)
///
/// The helper self-terminates and needs no `cerulion_node_shutdown` wiring:
/// - a caught closure PANIC breaks the loop and closes the write end. The
///   helper's `tracing::error!` dispatches against the CDYLIB's own tracing
///   subscriber — the macro-generated `cerulion_node_init`
///   installs the stderr subscriber (see [`install_cdylib_stderr_tracing`])
///   BEFORE `external_source` is queried, so the log line lands on stderr
///   (subject to RUST_LOG filtering). The poison remains observable HOST-side
///   regardless of log level: the closed write end makes the host's next drain
///   read EOF, which UNBINDS the doorbell with a loud host `tracing::error!`
///   (see [`super::runtime`]'s `DoorbellFdSource`) — the unbind DEFERS one extra
///   probe when that read still drained final buffered rings (they are served
///   first; the following EOF probe unbinds), and the panic hook still writes to
///   the process stderr;
/// - once the host CLOSES the read end (the drained-doorbell binding's `Drop` —
///   teardown or unbind; after an `EBADF` probe the `Drop` deliberately does NOT
///   close, since the fd number may have been reused), the helper's next
///   `write(2)` fails and it breaks, closing the write end;
/// - a bounded closure (per the [`ExternalSource::Blocking`] contract: "use a
///   bounded internal timeout") lets the helper observe those conditions; a
///   closure that blocks forever in a foreign SDK keeps the helper alive until
///   process exit — the same caveat the in-process tier-2 helper documents. The
///   helper is DETACHED (never joined): a closure blocked in a foreign SDK is
///   un-interruptible.
pub fn spawn_cdylib_blocking_doorbell(
    mut closure: Box<dyn FnMut() -> bool + Send + 'static>,
) -> Option<std::os::unix::io::RawFd> {
    let mut fds = [0 as libc::c_int; 2];
    // SAFETY: `fds` is a valid 2-element array; `pipe(2)` fills [read, write].
    let rc = unsafe { libc::pipe(fds.as_mut_ptr()) };
    if rc != 0 {
        let err = std::io::Error::last_os_error();
        tracing::error!(
            error = %err,
            "spawn_cdylib_blocking_doorbell: pipe(2) failed; external Blocking source \
             will not wake its node"
        );
        return None;
    }
    let read_fd = fds[0];
    let write_fd = fds[1];
    // Multi-process hardening: set FD_CLOEXEC on both pipe ends
    // so an exec'd child (e.g. the multi-process spawner) does NOT inherit them.
    // An inherited WRITE end would keep the pipe open after the helper thread dies,
    // suppressing the EOF-unbind contract (the host would never observe EOF, so the
    // dead doorbell would busy-loop the live WaitSet); an inherited READ end simply
    // leaks. Portable `fcntl(F_SETFD)` (NOT `pipe2(O_CLOEXEC)`, which is Linux-only
    // — this crate builds on macOS too). A failing fcntl is non-fatal: warn and
    // proceed (the fd still works; only the no-inherit-on-exec guarantee is lost).
    for (fd, which) in [(read_fd, "read"), (write_fd, "write")] {
        // SAFETY: `fd` is a live pipe end just returned by pipe(2); F_GETFD/F_SETFD
        // only read/toggle the close-on-exec flag on that descriptor.
        let ok = unsafe {
            let flags = libc::fcntl(fd, libc::F_GETFD);
            flags >= 0 && libc::fcntl(fd, libc::F_SETFD, flags | libc::FD_CLOEXEC) == 0
        };
        if !ok {
            tracing::warn!(
                error = %std::io::Error::last_os_error(),
                pipe_end = which,
                "spawn_cdylib_blocking_doorbell: could not set FD_CLOEXEC on the doorbell \
                 pipe end; proceeding — an exec'd child could inherit it (a leaked read end, \
                 or a write end that suppresses the EOF-unbind shutdown contract)"
            );
        }
    }
    // Set O_NONBLOCK on the WRITE end so the helper's ring write NEVER
    // blocks. Before this the write was blocking: if the host-side reader stalls
    // (e.g. the live-loop park limping at the sweep cadence — the very bug
    // the park-fd-poll change fixes), a full 64 KiB pipe pinned the helper thread
    // in pipe_write FOREVER, amplifying the wedge AND suppressing the EOF-unbind
    // shutdown contract (a blocked writer never observes the read-end close). A
    // full pipe already holds a pending byte, and the doorbell is LEVEL-TRIGGERED
    // (one pending byte suffices to ring it), so a non-blocking write that hits
    // EAGAIN is NOT a lost wake — `doorbell_write_ring` counts the coalesced ring
    // and carries on. A failing fcntl is non-fatal (warn + proceed): the fd still
    // works blocking, only the no-wedge guarantee is lost.
    // SAFETY: `write_fd` is the live pipe write end just returned by pipe(2);
    // F_GETFL/F_SETFL read/modify/write its flags.
    let nonblock_ok = unsafe {
        let flags = libc::fcntl(write_fd, libc::F_GETFL);
        flags >= 0 && libc::fcntl(write_fd, libc::F_SETFL, flags | libc::O_NONBLOCK) == 0
    };
    if !nonblock_ok {
        tracing::warn!(
            error = %std::io::Error::last_os_error(),
            "spawn_cdylib_blocking_doorbell: could not set O_NONBLOCK on the doorbell \
             write end; proceeding — a stalled reader could block the helper thread in a \
             full-pipe write"
        );
    }
    std::thread::spawn(move || {
        // Block SIGPIPE for this helper thread.
        // Once the host closes the read end, the next `write(2)` below would
        // otherwise deliver SIGPIPE; a Rust binary already `SIG_IGN`s it
        // process-wide (so the write returns EPIPE), but a NON-Rust host
        // embedding cerulion_core (e.g. a C++ process) may keep the default
        // fatal disposition and be KILLED on teardown. Blocking it per-thread
        // makes the write return EPIPE (handled below) regardless of the host's
        // global disposition, so the read-end-close shutdown contract holds
        // everywhere. The generated pending signal is discarded when the thread
        // exits (never unblocked).
        // SAFETY: `set` is a freshly-zeroed, sigemptyset-initialized sigset; all
        // three libc calls take valid pointers and touch only this thread's mask.
        unsafe {
            let mut set: libc::sigset_t = std::mem::zeroed();
            libc::sigemptyset(&mut set);
            libc::sigaddset(&mut set, libc::SIGPIPE);
            libc::pthread_sigmask(libc::SIG_BLOCK, &set, std::ptr::null_mut());
        }
        'helper: loop {
            // AssertUnwindSafe: `closure` is `&mut`-driven; a panic is caught and
            // terminally poisons this source (break), so no torn state escapes.
            // `&mut Box<dyn FnMut>` is itself `FnMut`, so it drives the closure
            // directly (no wrapper closure).
            match std::panic::catch_unwind(std::panic::AssertUnwindSafe(&mut closure)) {
                Ok(true) => {
                    // One NON-BLOCKING doorbell ring (EINTR retries;
                    // EAGAIN on a full pipe counts a coalesced overflow ring and
                    // carries on — level-triggered, one pending byte suffices, so a
                    // full pipe is NOT a lost wake). A real error (EPIPE once the
                    // host closed the read end) returns `false` → stop the helper.
                    if !doorbell_write_ring(write_fd) {
                        break 'helper;
                    }
                }
                Ok(false) => {}
                Err(_) => {
                    // NOTE: this dispatches
                    // against the CDYLIB's own tracing subscriber — the macro-
                    // generated init installs the stderr subscriber before
                    // external_source is queried, so it lands on stderr (subject
                    // to RUST_LOG). The poison stays observable HOST-side
                    // regardless of log level: the write-end close below drives
                    // the host's drain to EOF → loud host unbind (see
                    // `DoorbellFdSource`), and the panic hook writes to stderr.
                    tracing::error!(
                        "external Blocking source (cdylib) panicked; POISONED — it will no \
                         longer wake the node (restart the graph to recover)"
                    );
                    break;
                }
            }
        }
        // SAFETY: the helper owns the write end; close it on exit so the host's
        // read end sees EOF.
        unsafe {
            libc::close(write_fd);
        }
    });
    Some(read_fd)
}

/// Per-process count of doorbell writes that hit a FULL pipe (EAGAIN) —
/// the coalesced-ring overflow signal. A full pipe is NOT a lost wake (the
/// doorbell is LEVEL-TRIGGERED: one pending byte already rings it), so the helper
/// counts the overflow and carries on WITHOUT blocking (the earlier blocking
/// write pinned the helper thread in pipe_write, amplifying a stalled-reader
/// wedge). Process-cumulative across all helpers in the process — a diagnostic,
/// not a per-node user API. Read via [`doorbell_write_overflow_count`].
static DOORBELL_WRITE_OVERFLOWS: std::sync::atomic::AtomicU64 =
    std::sync::atomic::AtomicU64::new(0);

/// Read the `DOORBELL_WRITE_OVERFLOWS` counter (see its doc). The
/// LIGHTEST host-visible surface for the doorbell-write overflow signal —
/// `spawn_cdylib_blocking_doorbell` hands back only a `RawFd`, so there is no
/// per-helper handle to carry a counter; a process-cumulative `#[doc(hidden)]`
/// accessor is the "no new user API" fallback. Used by the full-pipe
/// regression pin. (Plain code spans, not intra-doc links, so this `pub` item's
/// docs never trip `private_intra_doc_links` under the `-D warnings` docs gate.)
#[doc(hidden)]
pub fn doorbell_write_overflow_count() -> u64 {
    DOORBELL_WRITE_OVERFLOWS.load(std::sync::atomic::Ordering::Relaxed)
}

/// Write one doorbell ring to `write_fd` (which the caller set
/// `O_NONBLOCK`), retrying `EINTR`. Returns `true` if the helper should keep
/// running — either the ring was written, OR the pipe is full (`EAGAIN`) and a
/// pending byte already rings the doorbell (level-triggered, one pending byte
/// suffices, so a full pipe is NOT a lost ring — bump
/// [`DOORBELL_WRITE_OVERFLOWS`] and carry on WITHOUT blocking). Returns `false`
/// on a fatal error (`EPIPE`/`EBADF` once the host closed the read end — the
/// helper must stop). The write end is non-blocking, so this NEVER
/// blocks: a blocking write on a full 64 KiB pipe would pin the helper
/// thread forever under a stalled reader.
fn doorbell_write_ring(write_fd: libc::c_int) -> bool {
    let byte: u8 = 1;
    loop {
        // SAFETY: write one byte from a live local to the owned write end.
        let n = unsafe { libc::write(write_fd, (&byte as *const u8).cast::<libc::c_void>(), 1) };
        if n >= 0 {
            return true; // wrote the ring (or 0) — done for this `true`
        }
        match std::io::Error::last_os_error().raw_os_error() {
            Some(libc::EINTR) => continue, // interrupted mid-write — retry
            // EWOULDBLOCK == EAGAIN on Linux + macOS (matching both would be an
            // unreachable-pattern error). Full pipe ⇒ a pending byte already rings
            // the doorbell; count the coalesced overflow and carry on.
            Some(libc::EAGAIN) => {
                DOORBELL_WRITE_OVERFLOWS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                return true;
            }
            // Real error (read end closed → EPIPE/EBADF): stop the helper.
            _ => return false,
        }
    }
}

// ---------------------------------------------------------------------------
// Cdylib-local stderr tracing subscriber (the STOPGAP).
// ---------------------------------------------------------------------------

/// Whether a `RUST_LOG` spec counts as "absent" for the cdylib subscriber:
/// `None` or the empty string both mean "no spec supplied" ⇒ default to `info`.
///
/// Extracted as a pure predicate so the empty-string-is-unset rule is
/// unit-testable WITHOUT installing a subscriber (calling the installer inside a
/// `cerulion_core` test binary would set that binary's process-global tracing
/// default and break unrelated `#[traced_test]` pins).
fn cdylib_rust_log_is_absent(spec: Option<&str>) -> bool {
    match spec {
        None => true,
        Some(s) => s.is_empty(),
    }
}

/// STOPGAP: install a cdylib-local stderr `fmt` tracing subscriber so
/// node-side `tracing` events become host-visible.
///
/// A cdylib statically links its OWN copy of `cerulion_core` + `tracing`, so its
/// `tracing` global dispatcher is a distinct static from the host binary's. The
/// host initializing its subscriber does nothing for the cdylib's static, so
/// without this every `tracing::error!`/`warn!`/`info!` emitted by node-side code
/// (including `cerulion_core`'s loud-by-design
/// [`OutputProxy`]
/// discard error) dispatches to a no-op — production dylib graphs run silently
/// broken. This installs an `fmt` subscriber writing to `stderr` on the CALLING
/// linked copy's `tracing` static, making those events land on the process's
/// stderr. (The post-launch (a) design bridges node-side events into the host's
/// own subscriber for unified filtering/formatting; this is the interim fix.)
///
/// Contract:
/// - Called from CDYLIB-side code only. A `#[cerulion_node]` node never calls
///   it: the macro-generated `cerulion_node_init` does, before any node code
///   runs. A hand-written raw-FFI node library gets the subscriber only if
///   its own init calls this function; the `cerulion node create --raw-ffi`
///   template does not, so such a node's `tracing` output stays silent until
///   it does. See "Node (cdylib) logs" in `docs/user-api.md`. Because the symbol
///   resolves to the cdylib's statically-linked `cerulion_core`, it operates on
///   that linked copy's `tracing` static. HOST processes never call it.
/// - Guarded by a function-local [`Once`](std::sync::Once). That static is
///   per-linked-copy — each loaded cdylib has its own copy, which is exactly the
///   right scope: one install per cdylib, idempotent across a node type's many
///   instances.
/// - `rust_log_spec` comes from the node's [`NodeContext`] env snapshot
///   (`ctx.env_str("RUST_LOG", "")`), NEVER live `std::env` — the snapshot is
///   frozen at graph build for replay determinism (see [`NodeContext`]'s
///   `env_lookup` docs). An empty/absent spec defaults to `info` (keeps
///   error/warn/info visible — loud by design); a non-empty spec is applied via
///   `tracing_subscriber::EnvFilter`. A spec that fails to parse emits a loud
///   one-line `eprintln!` (tracing is not up yet, so `eprintln!` is the correct
///   channel) and falls back to `info`.
/// - The `fmt` output disables ANSI escapes (`with_ansi(false)`): production
///   dylib-graph stderr is routinely piped/redirected, and escape codes would
///   corrupt log files.
/// - A SUCCESSFUL install under a SET `RUST_LOG` (parseable or not) emits
///   exactly one `eprintln!` BREADCRUMB naming the effective filter:
///   `cerulion cdylib tracing: node-side logs -> stderr (filter: <F>)`. A
///   typo'd-but-parseable RUST_LOG silently suppresses everything (`EnvFilter`
///   semantics), so when the user supplied a spec the effective filter must be
///   observable once per cdylib; the breadcrumb is also the behavioral pin for
///   the single-install guarantee (`Once`). Under the DEFAULT (absent or empty
///   `RUST_LOG`) nothing is printed: the filter is the documented `info`, and
///   a line per cdylib on every run is noise a first user learns to skip.
/// - If a global default already exists in this linked copy (e.g. installed by
///   a link-time constructor before any node code ran),
///   `tracing::subscriber::set_global_default` returns `Err`; we do nothing and
///   stay quiet (NO breadcrumb) — respecting an existing subscriber is correct
///   behavior, not a failure.
// Logging-rule exception (Principle 12), per this function's docs above: both `eprintln!`s below run
// where `tracing::` cannot — the first BEFORE any subscriber exists in this
// linked copy, the second announcing the filter the just-installed subscriber
// took (which that filter could itself suppress). Function-scoped because a
// `#[allow]` on a macro STATEMENT does not reach the lint emitted inside the
// expansion.
#[allow(clippy::print_stderr)]
pub fn install_cdylib_stderr_tracing(rust_log_spec: Option<&str>) {
    static INSTALL_ONCE: std::sync::Once = std::sync::Once::new();
    INSTALL_ONCE.call_once(|| {
        let (filter, filter_desc): (tracing_subscriber::EnvFilter, &str) =
            if cdylib_rust_log_is_absent(rust_log_spec) {
                (
                    tracing_subscriber::EnvFilter::new("info"),
                    "info (default: RUST_LOG absent)",
                )
            } else {
                // `is_absent` is false, so `rust_log_spec` is `Some(non-empty)`;
                // the `unwrap_or` default is unreachable but keeps this panic-free.
                let spec = rust_log_spec.unwrap_or("info");
                match tracing_subscriber::EnvFilter::try_new(spec) {
                    Ok(f) => (f, spec),
                    Err(e) => {
                        // Name WHICH directive failed (multi-directive specs) and
                        // promise only what is certain here: the fallback filter
                        // takes effect only if OUR subscriber installs below (a
                        // pre-existing global default in this linked copy wins).
                        eprintln!(
                            "cerulion cdylib tracing: RUST_LOG spec {spec:?} failed to parse \
                             ({e}); falling back to \"info\" if the cerulion subscriber installs"
                        );
                        (
                            tracing_subscriber::EnvFilter::new("info"),
                            "info (fallback: RUST_LOG spec unparseable)",
                        )
                    }
                }
            };
        let subscriber = tracing_subscriber::fmt()
            .with_env_filter(filter)
            .with_ansi(false)
            .with_writer(std::io::stderr)
            .finish();
        // A pre-existing global default in this linked copy wins (respected —
        // not an error): stay quiet, no breadcrumb. On a successful install,
        // the breadcrumb makes the effective filter observable (a parseable
        // typo'd spec would otherwise silently suppress everything).
        //
        // The breadcrumb prints only when the user CHANGED something: a
        // `RUST_LOG` that is set (parseable or not) is the one case in which
        // the effective filter could differ from what they expect, so that is
        // when it is named. Under the default (absent or empty spec) the
        // filter is the documented `info`, and one line per cdylib on every
        // run would only teach a first user to skim past output.
        let announce = !cdylib_rust_log_is_absent(rust_log_spec);
        if tracing::subscriber::set_global_default(subscriber).is_ok() && announce {
            eprintln!("cerulion cdylib tracing: node-side logs -> stderr (filter: {filter_desc})");
        }
    });
}

/// Interface between the graph runtime and node logic.
///
/// Implementations must be `Send` for use with `Arc<Mutex<Box<dyn NodeEntry>>>`.
/// `NodeContext` is `Send` by construction: its transport ports
/// are `iceoryx2::service::ipc_threadsafe::Service` (MutexProtected, Send+Sync).
pub trait NodeEntry: Send {
    /// Return node port metadata.
    ///
    /// Fallible: implementations that derive metadata from an
    /// external source (e.g. `DylibNodeEntry` parsing the cdylib's
    /// `cerulion_node_info()` JSON) return `Err` when the metadata is
    /// corrupted, so `GraphRuntime::build` / `build_in_process` refuse
    /// to construct a runtime containing the node (load-time failure)
    /// instead of silently degrading to `NodeInfo::default()` (empty
    /// wiring → node never fires). In-memory implementations
    /// (`ClosureNodeEntry`, macro-generated entries) always return `Ok`.
    fn info(&self) -> TransportResult<NodeInfo>;

    /// Initialize the node with its transport context.
    ///
    /// Called once before any `tick()`. The context provides pre-created
    /// publishers and subscribers.
    fn init(&mut self, context: NodeContext) -> TransportResult<()>;

    /// Execute one tick of the node.
    ///
    /// Called by the scheduler according to the node's trigger policy.
    fn tick(&mut self) -> TransportResult<()>;

    /// Clean shutdown. Default implementation does nothing.
    fn shutdown(&mut self) -> TransportResult<()> {
        Ok(())
    }

    /// Drive late-joiner history delivery for this node's publishers without
    /// publishing (runtime cadence). Default no-op; entries that own publishers
    /// override it. NOT on the deterministic firing path.
    ///
    /// This ALSO re-checks the notify-elision gate on every
    /// owned publisher (see [`NodeContext::pump_history`]) — the two are the same
    /// "boundary maintenance for late joiners" class, so they ride one path
    /// (in-process AND the ABI-v7 cdylib `cerulion_node_pump_history` FFI, no ABI
    /// bump).
    fn pump_history(&mut self) {}

    /// Append this node's per-publisher teardown reconciliation
    /// snapshots ([`PublisherReconStat`]) to `out`. Default no-op; **ONLY
    /// `ClosureNodeEntry` overrides it** to forward to
    /// [`NodeContext::collect_publisher_recon_stats`]. Diagnostics only —
    /// called once at graph teardown, never on any firing path.
    ///
    /// **In-process `#[cerulion_node]` macro entries do NOT override this**
    /// (`cerulion_macros` emits no override) — they use the default no-op here
    /// and instead surface their producer terms via `impl Drop for NodeContext`
    /// (`NodeContext::log_reconciliation_stats_at_teardown`), which runs when the
    /// macro `shutdown()` drops the host-side context. The `recon_logged` guard
    /// keeps that exactly-once. So the harvest reaches ONLY `ClosureNodeEntry`
    /// publishers; macro AND cdylib nodes both surface via the Drop path (grep
    /// `"producer reconciliation (per topic)"`).
    ///
    /// **`DylibNodeEntry` uses the default no-op ON PURPOSE:** a cdylib node
    /// TRANSFERS its `NodeContext` into the cdylib at `init()` (`Box::into_raw`
    /// through `cerulion_node_init`), so its publishers live in the cdylib's
    /// `NODES` registry, unreachable from the host — the host harvest yields
    /// nothing for it. The parity gap is instead closed WITHOUT an FFI export
    /// (no ABI bump): the cdylib's own `NodeContext` logs its producer
    /// reconciliation terms from inside the cdylib at teardown
    /// (`NodeContext::log_reconciliation_stats_at_teardown`, reached via
    /// `impl Drop for NodeContext` when `cerulion_node_shutdown` drops the boxed
    /// context), host-visible in the run log via the stderr subscriber.
    /// So a cdylib graph (e.g. the humanoid repro) DOES surface producer terms —
    /// grep `"producer reconciliation (per topic)"`.
    fn collect_publisher_recon_stats(&self, _out: &mut Vec<PublisherReconStat>) {}

    /// This node's `CerulionState::STATE_SHAPE`, or `None`
    /// if it declares no restorable state.
    ///
    /// `None` is the DEFAULT and it is correct for two different populations,
    /// which is why the answer is an `Option` rather than a bool plus a
    /// number. A genuinely stateless node (a relay, a pure function of its
    /// inputs) has nothing to restore and never will; a node whose type has no
    /// `CerulionState` yet has state that this build cannot reach. The restore
    /// treats both the same by default — execute from the constructor — and
    /// `--strict-state` is the switch for an operator who wants the second
    /// population reported rather than tolerated.
    ///
    /// It is the SHAPE and not a bool because the caller must compare it
    /// against the recorded anchor's before applying a byte
    /// ([`crate::state_restore::classify_shape`]), and a second accessor for
    /// "does it have one" would be a second thing that can disagree.
    fn state_shape(&self) -> Option<u64> {
        None
    }

    /// Apply a recorded anchor's PAYLOAD to this node.
    ///
    /// Called by [`crate::graph::GraphRuntime::restore_node_states`] AFTER
    /// [`Self::init`] and BEFORE the first [`Self::tick`] — this ordering
    /// matters, because `init()` opens handles from fields whose values are
    /// still `Default` until the state lands. An implementation that has a
    /// `restored()` hook runs it HERE, after the fields are back and before it
    /// returns.
    ///
    /// `payload` is the anchor blob with its framing already stripped and its
    /// shape already checked, so an implementation decodes it straight through
    /// [`crate::state::StateCursor`]. The default REFUSES rather than
    /// succeeding silently: a runtime that hands state to a node with no
    /// restore path must hear about it, since the alternative is a replay that
    /// reports a confident divergence against state it quietly dropped.
    fn restore_state(&mut self, _payload: &[u8]) -> TransportResult<()> {
        Err(TransportError::GraphError {
            reason: "this node declares no restorable state: it has no \
                     `state_shape()`, so a recorded anchor cannot be applied to it"
                .to_string(),
        })
    }

    /// Encode this node's state into `out` — the inverse of
    /// [`Self::restore_state`], and the CAPTURE half of the pair.
    ///
    /// It writes the PAYLOAD only. The anchor framing (magic + shape) is the
    /// carrier's, written through
    /// [`crate::state_restore::AnchorBlob::write_header`], because the fork
    /// carrier walks nodes through an erased view and holds the shape as a
    /// `u64` rather than as a type parameter. Keeping the two apart is also
    /// what makes `capture_state` the exact inverse of `restore_state`, which
    /// receives an already-unframed payload.
    ///
    /// `out` owns the bound: a bounded sink refuses past its capacity and the
    /// error surfaces here, so a caller with a fixed arena learns its buffer
    /// was too small rather than receiving a partial encoding that looks whole.
    ///
    /// The default REFUSES for the same reason [`Self::restore_state`]'s does:
    /// a node with no capture path must not report an empty anchor as a
    /// successful one, because a resumed run would then diverge against state
    /// nobody recorded.
    fn capture_state(&self, _out: &mut dyn crate::state::StateSink) -> TransportResult<()> {
        Err(TransportError::GraphError {
            reason: "this node declares no capturable state: it has no \
                     `state_shape()`, so no anchor can be taken for it"
                .to_string(),
        })
    }

    /// This node's
    /// [`CerulionState::INLINE_SAFE`](crate::state::CerulionState::INLINE_SAFE),
    /// surfaced per node so the boundary walk can read it without naming the
    /// node's type.
    ///
    /// `true` means capturing this node runs ONLY framework-generated code over
    /// data that cannot block the caller — no lock, no interior mutability, no
    /// user-written encoder — which is what turns the arena's BYTE bound into a
    /// TIME bound with no clock. The carrier tests it FIRST, before building a
    /// sink, so an ineligible node costs zero arena bytes and zero user-code
    /// execution on the node thread.
    ///
    /// **Defaults to `false`**, the same direction the trait's own const
    /// defaults, and for the same reason: a node that has not proved the
    /// property must not silently claim it. Being wrong in this direction costs
    /// only a different carrier — the node joins the fork set, which captures it
    /// just as completely and emits identical bytes.
    fn inline_safe(&self) -> bool {
        false
    }

    /// The pre-fork lock probe:
    /// [`CerulionState::cer_probe`](crate::state::CerulionState::cer_probe)
    /// surfaced per node. Non-blocking, total over the node's DECLARED state.
    ///
    /// `false` means some lock in this node's state graph is held right now, so
    /// the boundary skips the whole anchor rather than forking into a lock the
    /// child would hold forever (a fork child has one thread, so a lock another
    /// thread held at the fork instant is held by nobody in the child's image).
    ///
    /// **Defaults to `true`**, which is sound for the same reason the trait's
    /// default is: the only implementations that can answer `false` are ones
    /// this crate writes. A node reached through a hand-written impl is covered
    /// by the fork carrier's progress watchdog, not by a claim it makes here.
    fn cer_probe(&self) -> bool {
        true
    }

    /// Freeze step-boundary snapshots for the listed
    /// NON-triggering (latest-value) inputs. Called by `GraphRuntime::step` per
    /// level (the fire-gated runtime pass), AFTER deciding the
    /// level's fire-set and draining its trigger inputs and BEFORE firing the
    /// level's nodes, so a same-level producer's same-step publish is not
    /// observed by a same-level latest-value reader (replay = live under
    /// within-level parallelism). `inputs` are the wired input names the
    /// RUNTIME classified as non-triggering (the complement of
    /// `build_trigger_edges` — the macro does not decide).
    ///
    /// Default NO-OP — a no-op-snapshot node's non-trigger
    /// inputs are NOT step-boundary-frozen, so a same-level producer's same-step
    /// publish IS visible to them (HARMLESS while such nodes fire SERIALLY: there
    /// is no same-level race for the snapshot to guard against; the level
    /// executor routes them off the rayon path — see
    /// [`Self::performs_input_snapshot`]).
    ///
    /// **`ClosureNodeEntry` (test-only) inherits this no-op.** In-process
    /// `#[cerulion_node]` macro nodes OVERRIDE it to forward to
    /// [`NodeContext::snapshot_inputs`].
    ///
    /// **Cdylib:** `DylibNodeEntry` now OVERRIDES this too: when the
    /// cdylib exports the optional `cerulion_node_{set_,}snapshot_inputs` FFI
    /// symbols, the override marshals the names ONCE and forwards the per-step
    /// freeze across the FFI to the cdylib's `NodeContext`, so a cdylib's
    /// non-trigger latest-value `#[input]` HOLDS its last-delivered value across
    /// steps (instead of collapsing the tick to a no-op on a silent step). An
    /// older cdylib lacking the symbols keeps this inherited no-op
    /// (back-compat). The override does NOT make a cdylib rayon-eligible —
    /// `performs_input_snapshot` stays `false` (cdylib fires serially); the
    /// hold-vs-rayon distinction is [`Self::holds_input_snapshot`].
    fn snapshot_inputs(&mut self, _inputs: &[String]) {}

    /// Drain a single data-trigger input's body subscriber at the level
    /// boundary (the runtime calls this through the node lock, BEFORE decide), so
    /// the one receive serves both the trigger (returned count/ts) and the tick's
    /// later try_view (frozen). Returns (popped, latest_ts). Default: (0, None) —
    /// nodes without this wiring (a cdylib without the drain export) keep the
    /// separate trigger-drain.
    fn drain_trigger_input(&mut self, _input_name: &str) -> (u64, Option<u64>) {
        (0, None)
    }

    /// Refill a data-trigger input BETWEEN two fires of one step (the
    /// scheduler's Data burst loop). Same drain as [`Self::drain_trigger_input`]
    /// except that an UNSERVED frozen head reports `(0, None)` rather than being
    /// re-offered — see
    /// [`crate::transport::subscriber::CerulionSubscriber::refill_for_trigger`].
    ///
    /// Default `(0, None)`, which is the SAFE answer in both directions: the
    /// burst loop's `popped == 0` break simply ends the burst, so an entry that
    /// cannot refill serves the arrivals its boundary drain signalled and no
    /// more (the earlier one-fire-per-step throughput for a Unified
    /// binding) — degraded, never a same-frame re-fire. Declare
    /// [`Self::refills_trigger_input`] `true` ONLY together with a real
    /// implementation.
    fn refill_trigger_input(&mut self, _input_name: &str) -> (u64, Option<u64>) {
        (0, None)
    }

    /// Does this entry's [`Self::refill_trigger_input`] really drive
    /// its BODY subscriber (vs being the `(0, None)` default)?
    ///
    /// Read ONCE per node at graph build, beside [`Self::unifies_trigger_drain`],
    /// to decide whether to install the scheduler's between-fires refill hook at
    /// all: installing it on an entry that always answers "nothing" costs a node
    /// lock per step and buys nothing, and the accurate `false` is what lets the
    /// build breadcrumb name the nodes whose bursts will be served one frame per
    /// step.
    ///
    /// `false` is the DEGRADED-but-correct answer (see
    /// [`Self::refill_trigger_input`]); the answer must be static per entry.
    fn refills_trigger_input(&self) -> bool {
        false
    }

    /// Does this entry's [`Self::drain_trigger_input`]
    /// drive its BODY subscriber (drain-through-the-node-lock), so the runtime
    /// may elide the separate trigger-drain subscriber and wire the binding
    /// `DrainSource::Unified` (one iceoryx2 receive per data-trigger hop)?
    ///
    /// Decoupled from [`Self::performs_input_snapshot`] — the RAYON-eligibility
    /// flag the unified drain originally piggybacked on. Rayon-safety and
    /// drain-through-the-lock are independent capabilities: a
    /// `ClosureNodeEntry` fires serially (`performs_input_snapshot == false`)
    /// yet drains its own body subscriber just fine.
    ///
    /// Default `false`: an entry whose [`Self::drain_trigger_input`] is the
    /// `(0, None)` default would NEVER fire if unified (the separate drain it
    /// depends on is gone), so it keeps the legacy dual-subscriber path.
    /// Override to `true` ONLY together with a real `drain_trigger_input`.
    ///
    /// READ-PATH CONTRACT for implementors: a Unified drain consumes the
    /// iceoryx2 queue into the subscriber's frozen slot, which is served by
    /// `try_view` (`&mut`) ONLY — the queue-draining `try_receive` (`&self`)
    /// bypasses the slot and would observe an EMPTY queue. An entry whose tick
    /// reads its trigger input accumulate-all via `try_receive` must NOT
    /// declare this capability (see `ClosureNodeEntry::with_unified_drain`).
    ///
    /// Queried ONCE per node at graph build (the capability-map capture,
    /// BEFORE [`Self::init`] injects the context) — so the answer must be
    /// static per entry, never derived from post-init state.
    fn unifies_trigger_drain(&self) -> bool {
        false
    }

    /// Perform ONE per-set Sync head op on this node's `input_name`
    /// and answer what the matcher's verdict asked for.
    ///
    /// The matcher is PURE — it holds no transport — so it asks for a fact by
    /// returning a verdict, and the align driver performs the op through this
    /// seam and re-runs it. ONE multiplexed method rather than six, mirroring
    /// the ONE FFI symbol it crosses on a cdylib (`cerulion_node_sync_head_op`):
    /// the six ops are transitions of one state machine, and separate hooks
    /// would admit partial-capability nodes (advance-without-void) the degrade
    /// logic would then have to enumerate.
    ///
    /// Default [`SyncOpAnswer::Failed`] — FAIL-CLOSED, and correct in both
    /// directions. An entry that does not declare
    /// [`Self::supports_sync_head_ops`] is never GIVEN ops to run (the graph
    /// build checks the capability first and degrades LOUDLY to the legacy
    /// latest-per-set Sync), so this default is unreachable by construction;
    /// and if one somehow were, `Failed` is the answer the driver's R-Fail
    /// policy maps to the DESCENT-DISABLING reading at every site, never to a
    /// fabricated set member.
    fn sync_head_op(&mut self, _input_name: &str, _op: SyncHeadOp) -> SyncOpAnswer {
        SyncOpAnswer::Failed
    }

    /// Does this entry's [`Self::sync_head_op`] really drive its
    /// transport (vs being the [`SyncOpAnswer::Failed`] default)?
    ///
    /// Read ONCE per Sync node at graph build, beside
    /// [`Self::unifies_trigger_drain`] and [`Self::refills_trigger_input`], to
    /// decide whether to install the per-set ops at all. `false` (the default)
    /// keeps the node on the legacy latest-per-set Sync semantics, which is
    /// DEGRADED-but-correct — and the accurate `false` is what lets the build
    /// breadcrumb name the nodes that will run that way instead of silently
    /// serving them a matcher every one of whose questions answers `Failed`.
    ///
    /// Same pairing rule [`Self::unifies_trigger_drain`] documents: declare
    /// `true` ONLY together with a real [`Self::sync_head_op`]. The answer must
    /// be static per entry — the capability map is captured BEFORE
    /// [`Self::init`] injects the context, so it can never be derived from
    /// post-init state.
    fn supports_sync_head_ops(&self) -> bool {
        false
    }

    /// Does [`Self::snapshot_inputs`] REALLY freeze this
    /// node's non-trigger (latest-value) inputs, vs being the no-op default?
    ///
    /// `true` ⇒ a `#[cerulion_node]` macro node whose generated
    /// `snapshot_inputs` forwards to [`NodeContext::snapshot_inputs`] AND whose
    /// tick is thread-safe to fire in parallel. `false` (the default) ⇒ a
    /// cdylib (`DylibNodeEntry`) / closure (`ClosureNodeEntry`) node, fired
    /// SERIALLY.
    ///
    /// The level executor consults this to keep a node that has non-trigger
    /// inputs OFF the within-level rayon parallel fire path: without a real
    /// freeze, a same-level producer's PARALLEL publish could be observed by such
    /// a node mid-level, breaking replay = live (Principle 7). Macro nodes
    /// (`true`) get the real freeze and fire in parallel; cdylib / closure nodes
    /// (`false`) fire serially on the calling thread.
    ///
    /// `DylibNodeEntry` keeps returning `false` here EVEN WHEN it
    /// now performs a real cross-step input freeze over the FFI — a cdylib HOLDS
    /// (see [`Self::holds_input_snapshot`]) but stays serial, because making FFI
    /// ticks thread-safe is out of scope. So this method is the RAYON-eligibility
    /// gate specifically, NOT "does `snapshot_inputs` do anything" (that is
    /// `holds_input_snapshot`).
    ///
    /// Named to pair with [`Self::snapshot_inputs`]: it answers "is this node's
    /// freeze rayon-parallel-safe?" — NOT "can this node take a full state
    /// snapshot".
    fn performs_input_snapshot(&self) -> bool {
        false
    }

    /// Does this node's [`Self::snapshot_inputs`] HOLD borrowed iceoryx2
    /// samples across steps — i.e. does it ACTUALLY freeze + replay non-trigger
    /// latest-value inputs — so that its non-trigger source topics must be
    /// provisioned at `SUBSCRIBER_MAX_BORROWED_HELD` (3) borrows rather than the
    /// iceoryx2 default of 2?
    ///
    /// Distinct from [`Self::performs_input_snapshot`], which ALSO gates
    /// within-level RAYON eligibility. They COINCIDE for macro nodes (hold +
    /// rayon-eligible) and closures (neither). They DIVERGE for a
    /// `DylibNodeEntry` (cdylib) that exports the optional snapshot FFI
    /// symbols: its `snapshot_inputs` forwards the freeze across the FFI, so it
    /// HOLDS (`true` here), yet it stays OFF the rayon path
    /// (`performs_input_snapshot == false`, fired serially). An older cdylib
    /// lacking the symbols returns `false` (no hold — back-compat).
    ///
    /// Default: delegate to [`Self::performs_input_snapshot`] — a node that
    /// really freezes also holds; a no-op-snapshot node holds nothing.
    fn holds_input_snapshot(&self) -> bool {
        self.performs_input_snapshot()
    }

    /// The [`ExternalSource`] an `#[cerulion_node(external)]` ingress
    /// (driver) node watches so the live loop self-triggers it. Queried ONCE by
    /// [`crate::graph::GraphRuntime`]'s `collect_external_sources` at `run_live`
    /// entry (after [`Self::init`]); NEVER under the polled `step()` / replay
    /// path (Principle #7 — the wake is record-only; the fire decision stays in
    /// `step()`'s deterministic `TriggerPolicy::External` arm).
    ///
    /// `None` (the default) means "no self-source": non-external nodes and
    /// older nodes. The macro layer REQUIRES an
    /// `#[cerulion_node(external)]` node to override this — the `Option` is
    /// internal plumbing so a non-external / older `NodeEntry` needs no change.
    /// `Some(`[`ExternalSource::HostDriven`]`)` is the explicit "I have no
    /// self-source; fire me via `trigger_external`" answer (distinct from `None`
    /// only in intent — both leave the node host-driven).
    ///
    /// `&mut self` because a driver typically creates/opens its device here (it
    /// may lazily open the fd or capture SDK state); the runtime calls it exactly
    /// once, so this is not a hot path.
    fn external_source(&mut self) -> Option<ExternalSource> {
        None
    }

    /// Internal plumbing — did the [`ExternalSource::Fd`] this
    /// node last returned from [`Self::external_source`] actually come from a
    /// cdylib's tier-2 [`ExternalSource::Blocking`] collapse (a
    /// [`EXTERNAL_SOURCE_KIND_DOORBELL_FD`] pipe read end), rather than a real
    /// device fd ([`EXTERNAL_SOURCE_KIND_DEVICE_FD`])?
    ///
    /// `#[doc(hidden)]` — this is NOT a user-facing seam. It exists ONLY so
    /// `DylibNodeEntry` can tell the runtime that an `Fd` it returned must be
    /// bound as a DRAINED doorbell (the runtime reads the pipe to EAGAIN each
    /// sweep) instead of a poll-only device fd — WITHOUT widening the public
    /// 3-variant [`ExternalSource`] enum with a cdylib-only variant. The pattern
    /// mirrors [`Self::holds_input_snapshot`] (a `DylibNodeEntry`-only marker the
    /// runtime consults alongside the primary method, under the same lock).
    ///
    /// Default `false`: in-process `#[cerulion_node]` nodes return `Blocking`
    /// directly (the runtime spawns the doorbell in-process — no fd collapse), so
    /// any `Fd` they return is always a real device fd. Only `DylibNodeEntry`
    /// overrides this.
    #[doc(hidden)]
    fn external_source_is_drained_doorbell_fd(&self) -> bool {
        false
    }
}

/// Callback type for node operations that receive a mutable context.
///
/// Used by `ClosureNodeEntry` (test-only).
#[cfg(any(test, feature = "test-helpers"))]
type NodeCallback = Box<dyn FnMut(&mut NodeContext) -> TransportResult<()> + Send>;

/// Closure-based node for tests (test-only).
///
/// Wraps closures for `init` and `tick` operations. Useful for deterministic
/// testing with `VirtualClock` where no FFI overhead is desired and the
/// caller wants to avoid the boilerplate of a `#[cerulion_node]` fixture.
///
/// **Production code should use `#[cerulion_node]` + `#[cerulion_node_impl]`.**
/// Kept available only behind `#[cfg(any(test, feature = "test-helpers"))]`
/// so it cannot leak into production node code.
///
/// A `ClosureNodeEntry` uses the default
/// no-op [`NodeEntry::snapshot_inputs`], so its non-trigger inputs are NOT
/// step-boundary-frozen — harmless because such an entry fires serially; see
/// [`NodeEntry::snapshot_inputs`].
#[cfg(any(test, feature = "test-helpers"))]
pub struct ClosureNodeEntry {
    info: NodeInfo,
    diag_label: String,
    context: Option<NodeContext>,
    init_fn: Option<NodeCallback>,
    tick_fn: NodeCallback,
    shutdown_fn: Option<Box<dyn FnMut() -> TransportResult<()> + Send>>,
    /// Reported via `NodeEntry::unifies_trigger_drain`.
    /// Default `true` — a closure's `drain_trigger_input` forwards to the
    /// `NodeContext` exactly like a macro node's, so an eligible (DropOldest
    /// `input_meta`) data-trigger input unifies onto the body subscriber.
    /// Opt OUT via [`Self::with_unified_drain`] for ticks that read the
    /// trigger input accumulate-all via `try_receive` (the frozen slot only
    /// serves `try_view` — see the trait method's READ-PATH CONTRACT).
    unified_drain: bool,
}

#[cfg(any(test, feature = "test-helpers"))]
impl ClosureNodeEntry {
    /// Create a new closure-based node.
    ///
    /// # Arguments
    ///
    /// - `info` — Node port metadata
    /// - `tick_fn` — Called on each scheduler fire with mutable context access
    pub fn new(
        info: NodeInfo,
        tick_fn: impl FnMut(&mut NodeContext) -> TransportResult<()> + Send + 'static,
    ) -> Self {
        Self {
            info,
            diag_label: "closure_node".to_string(),
            context: None,
            init_fn: None,
            tick_fn: Box::new(tick_fn),
            shutdown_fn: None,
            unified_drain: true,
        }
    }

    /// Opt this closure OUT of the unified trigger drain
    /// (`with_unified_drain(false)`), keeping the legacy dual-subscriber
    /// `DrainSource::Separate` wiring for its data-trigger input.
    ///
    /// Needed when the tick reads the trigger input ACCUMULATE-ALL via
    /// `AnySubscriber::try_receive` (e.g. the /tf model): the unified drain
    /// consumes the queue into the subscriber's frozen slot (latest-wins),
    /// which only `try_view` serves — `try_receive` would observe an empty
    /// queue and silently lose every frame.
    pub fn with_unified_drain(mut self, unified: bool) -> Self {
        self.unified_drain = unified;
        self
    }

    /// Set a diagnostic label used in error messages from this entry.
    pub fn with_label(mut self, label: impl Into<String>) -> Self {
        self.diag_label = label.into();
        self
    }

    /// Set an init callback.
    pub fn with_init(
        mut self,
        init_fn: impl FnMut(&mut NodeContext) -> TransportResult<()> + Send + 'static,
    ) -> Self {
        self.init_fn = Some(Box::new(init_fn));
        self
    }

    /// Set a shutdown callback.
    pub fn with_shutdown(
        mut self,
        shutdown_fn: impl FnMut() -> TransportResult<()> + Send + 'static,
    ) -> Self {
        self.shutdown_fn = Some(Box::new(shutdown_fn));
        self
    }
}

#[cfg(any(test, feature = "test-helpers"))]
impl NodeEntry for ClosureNodeEntry {
    fn info(&self) -> TransportResult<NodeInfo> {
        Ok(self.info.clone())
    }

    fn init(&mut self, mut context: NodeContext) -> TransportResult<()> {
        if let Some(init_fn) = &mut self.init_fn {
            init_fn(&mut context)?;
        }
        self.context = Some(context);
        Ok(())
    }

    fn tick(&mut self) -> TransportResult<()> {
        if let Some(ctx) = &mut self.context {
            (self.tick_fn)(ctx)
        } else {
            Err(TransportError::NodeError {
                node_id: self.diag_label.clone(),
                reason: "node not initialized".to_string(),
            })
        }
    }

    fn shutdown(&mut self) -> TransportResult<()> {
        if let Some(shutdown_fn) = &mut self.shutdown_fn {
            shutdown_fn()?;
        }
        self.context = None;
        Ok(())
    }

    fn pump_history(&mut self) {
        if let Some(ctx) = self.context.as_mut() {
            ctx.pump_history();
        }
    }

    fn collect_publisher_recon_stats(&self, out: &mut Vec<PublisherReconStat>) {
        if let Some(ctx) = self.context.as_ref() {
            ctx.collect_publisher_recon_stats(out);
        }
    }

    // Forward the unified trigger drain to the NodeContext,
    // mirroring the macro-generated impl. Pre-context (0, None) is the graceful
    // no-op: the capability map is captured BEFORE init() injects the context
    // (see `unifies_trigger_drain` below), but drain_level only runs from
    // step()/live_step — by then init() (called inside the build loop) has
    // installed the context, so the fallback is defensively unreachable.
    fn drain_trigger_input(&mut self, input_name: &str) -> (u64, Option<u64>) {
        match self.context.as_mut() {
            Some(ctx) => ctx.drain_trigger_input(input_name),
            None => (0, None),
        }
    }

    // The between-fires refill twin, forwarded the same way.
    fn refill_trigger_input(&mut self, input_name: &str) -> (u64, Option<u64>) {
        match self.context.as_mut() {
            Some(ctx) => ctx.refill_trigger_input(input_name),
            None => (0, None),
        }
    }

    // Gated on the SAME static field as `unifies_trigger_drain`, not
    // on the forwarding above being present: an accumulate-all `try_receive`
    // tick (`with_unified_drain(false)`) is never Unified-wired, so it never
    // reaches the refill hook, and declaring a capability whose read-path
    // contract the entry does not honour would be a claim nothing checks.
    fn refills_trigger_input(&self) -> bool {
        self.unified_drain
    }

    // STATIC capability (a plain field, not
    // `self.context.is_some()`) because the runtime captures the capability map
    // from the factories BEFORE the per-node build loop calls `init()` — a
    // context-presence answer would always read `false` at capture time and no
    // closure could ever unify. `with_unified_drain(false)` opts out for
    // accumulate-all `try_receive` ticks (frozen slot serves `try_view` only).
    fn unifies_trigger_drain(&self) -> bool {
        self.unified_drain
    }

    // Forward the per-set Sync head ops to the NodeContext, mirroring
    // the drain/refill forwarding above. Pre-context `Failed` is the same
    // graceful no-op with the same reachability: the capability map is captured
    // BEFORE `init()` injects the context, but an op only ever runs from the
    // align driver inside `step()`/`live_step`, by which time the build loop's
    // `init()` has installed it. `Failed` is also the FAIL-CLOSED answer, so
    // even that unreachable arm can only refuse a descent, never invent a
    // member.
    fn sync_head_op(&mut self, input_name: &str, op: SyncHeadOp) -> SyncOpAnswer {
        match self.context.as_mut() {
            Some(ctx) => ctx.sync_head_op(input_name, op),
            None => SyncOpAnswer::Failed,
        }
    }

    // Gated on the SAME static field as `unifies_trigger_drain` and
    // `refills_trigger_input`, and the three conditions really are ONE
    // condition: `unified_drain` says "this entry's trigger-input reads go
    // through the frozen-slot path", which is precisely what a per-set head is.
    // Every head op ends in a head the tick must later read via `try_view` (a
    // fill freezes one, an advance replaces one, a void empties one), and the
    // frozen slot is served by `try_view` ALONE — a closure that reads its
    // trigger accumulate-all via `try_receive` (`with_unified_drain(false)`)
    // bypasses the slot entirely, so it would observe an empty queue whether it
    // was the unified drain or the matcher that filled the head. It is the
    // warned-misuse path either way, which is why one flag governs both rather
    // than a second flag restating it.
    fn supports_sync_head_ops(&self) -> bool {
        self.unified_drain
    }
}

/// cdylib-loaded node for production use.
///
/// Loads a dynamic library (.so/.dylib/.dll) and looks up the standard
/// Cerulion node entry point symbols via `extern "C"` ABI.
///
/// # cdylib Entry Point Convention (ABI v3)
///
/// The cdylib must export these `extern "C"` functions:
///
/// - `cerulion_abi_version() -> u32` — Must return `CERULION_ABI_VERSION`
/// - `cerulion_rustc_fingerprint() -> *const c_char`: Must return
///   `RUSTC_FINGERPRINT` (ABI v22). Checked immediately after
///   `cerulion_abi_version`; a mismatch refuses the load even when the ABI
///   version matches, because two different rustc releases can encode the
///   same struct layout's `Option::None` niche differently.
/// - `cerulion_node_info() -> *const c_char` — Returns port metadata as JSON
/// - `cerulion_node_init(ctx: *mut NodeContext) -> u64` — Takes ownership of context, returns handle (0 = error)
/// - `cerulion_node_tick(handle: u64) -> i32` — Execute one tick for the given handle
/// - `cerulion_node_pump_history(handle: u64) -> i32`: Service quiescent late joiners for the given handle (ABI v7). Best-effort liveness; a non-zero return is logged, never propagated.
/// - `cerulion_node_shutdown(handle: u64) -> i32` — Clean shutdown for the given handle
/// - `cerulion_take_last_error() -> *mut c_char`: Pulls the most recent error message off the cdylib's per-thread buffer; null if no error.
/// - `cerulion_free_error(*mut c_char)` — Pairs with `cerulion_take_last_error`; allocator-paired free.
///
/// `cerulion_node_info()` carries no `node_type` field — the node
/// type is the folder name and is resolved by the loader, not by
/// the cdylib. The JSON shape is
/// `{"inputs": [...], "outputs": [...], "policy": {...}?}`.
///
/// Handle-based design allows multiple instances from the same cdylib.
///
/// A `DylibNodeEntry` OVERRIDES [`NodeEntry::snapshot_inputs`] to
/// forward the per-step non-trigger latest-value freeze across the FFI via the
/// optional `cerulion_node_set_snapshot_inputs` / `cerulion_node_snapshot_inputs`
/// symbols — so a cdylib node HOLDS its non-trigger `#[input]`s across steps,
/// exactly like an in-process macro node. An older cdylib that doesn't
/// export the symbols keeps the inherited no-op (back-compat; the symbols are
/// additive and do NOT bump the ABI version). It still fires SERIALLY off the
/// rayon path — [`NodeEntry::performs_input_snapshot`] stays `false`; the hold is
/// reported via [`NodeEntry::holds_input_snapshot`] (which drives the host's
/// borrow-3 provisioning of its snapshot-source topics). See
/// [`NodeEntry::snapshot_inputs`].
///
/// # Safety
///
/// DylibNodeEntry uses `unsafe` for:
/// 1. Loading the library (`libloading::Library::new`)
/// 2. Looking up symbols (`lib.get::<fn>`)
/// 3. Calling `extern "C"` functions
/// 4. Passing `NodeContext` ownership across FFI boundary
///
/// The ABI contract is enforced by convention, not by the type system.
pub struct DylibNodeEntry {
    info_fn: unsafe extern "C" fn() -> *const std::ffi::c_char,
    init_fn: unsafe extern "C" fn(*mut NodeContext) -> u64,
    tick_fn: unsafe extern "C" fn(u64) -> i32,
    /// ABI v7: service quiescent late joiners by
    /// re-publishing this handle's history over the FFI. The in-process
    /// `pump_history` path cannot reach a cdylib node — the
    /// host only holds an opaque handle — so the loader calls this symbol
    /// on every live step instead. Best-effort: a non-zero return is
    /// logged, never propagated (pump is never the firing path). Cdylib
    /// symbol: `cerulion_node_pump_history`.
    pump_history_fn: unsafe extern "C" fn(u64) -> i32,
    shutdown_fn: unsafe extern "C" fn(u64) -> i32,
    /// Take the most recent error
    /// message off the cdylib's per-thread buffer. Returns null if no
    /// error is buffered. The host MUST pair every non-null return
    /// with `free_error_fn`. Cdylib symbol: `cerulion_take_last_error`.
    take_last_error_fn: unsafe extern "C" fn() -> *mut std::ffi::c_char,
    /// Free a string returned by
    /// `take_last_error_fn`. The free MUST happen via the SAME cdylib
    /// (CString allocator pairing). Cdylib symbol: `cerulion_free_error`.
    free_error_fn: unsafe extern "C" fn(*mut std::ffi::c_char),
    /// OPTIONAL cdylib snapshot FFI, resolved best-effort by `load` as a
    /// both-or-neither PAIR ([`SnapshotFns`]). `None` for an older cdylib
    /// that exports NEITHER symbol (back-compat — it still loads, since the
    /// symbols are additive and do NOT bump the ABI version, and its non-trigger
    /// inputs read live) OR — a build mistake — that exports exactly ONE (load
    /// normalizes that to `None` with a warn). When `Some`, `snapshot_inputs`
    /// marshals the names once via `SnapshotFns::set` and forwards the per-step
    /// freeze via `SnapshotFns::snapshot`, and `holds_input_snapshot` reports
    /// `true` so the host provisions the source topics at
    /// `SUBSCRIBER_MAX_BORROWED_HELD`. Cdylib symbols:
    /// `cerulion_node_set_snapshot_inputs` / `cerulion_node_snapshot_inputs`.
    snapshot_ffi: Option<SnapshotFns>,
    /// Lifecycle of the one-time name registration — see
    /// [`SnapshotState`]. Init `Pending`; `Active` after the first SUCCESSFUL
    /// `set` (so the build-fixed names are marshalled to the cdylib exactly ONCE,
    /// and every later `snapshot_inputs` is a zero-alloc `SnapshotFns::snapshot`
    /// call); terminal `Failed` if that `set` fails — the node then runs UNHELD,
    /// loudly, never silently masked.
    snapshot_state: SnapshotState,
    /// Caches the parsed `NodeInfo` so the FFI
    /// `cerulion_node_info()` symbol is invoked AT MOST ONCE per node on
    /// the production load path. `OnceCell` (interior mutability) lets
    /// `info()` — which takes `&self` — populate the cache on its FIRST
    /// successful call, so the second `info()` call inside `init()`
    /// reuses the cached value rather than re-entering the FFI. This
    /// closes the double-FFI hazard documented in `info()`: a
    /// non-idempotent `cerulion_node_info()` (valid on call 1, null/
    /// corrupt on call 2) can no longer wire publishers from one
    /// `NodeInfo` while caching a different one. Failed parses are NOT
    /// cached (the `OnceCell` stays empty), so a corrupt node still
    /// fails on every `info()` call.
    info_cache: std::cell::OnceCell<NodeInfo>,
    handle: Option<u64>,
    /// Flood-control latch for `pump_history` logging.
    /// The pump runs every live-loop iteration, so a PERSISTENT non-zero FFI
    /// return (e.g. a poisoned cdylib `NODES` mutex) would re-log at `error!`
    /// on every iteration. Latched like the runtime's `signal_sync_input`
    /// guard: first failure loud, subsequent quiet (debug) until a success
    /// clears it (recovery info). See [`flood_log_action`].
    pump_history_error_latched: bool,
    /// The twin of `pump_history_error_latched` for the
    /// per-step `snapshot_inputs` FFI call. A holding cdylib's `snapshot` runs
    /// every firing step, so a PERSISTENT mid-run failure (e.g. a sibling cdylib
    /// poisoned the `NODES` mutex AFTER a successful `set` — 870×/s in the WBC
    /// benchmark) would `error!`-flood the live loop. Same latch policy via
    /// [`flood_log_action`]: first failure loud, then `debug!` until recovery.
    snapshot_error_latched: bool,
    /// OPTIONAL unified-trigger-drain FFI symbol
    /// (`cerulion_node_drain_trigger_input(handle, name_ptr, name_len,
    /// out_popped, out_latest_ts, out_has_ts) -> i32`), resolved best-effort by
    /// `load` (single symbol, mirrors `external_source_fn`). Symbol PRESENCE is
    /// the capability: `unifies_trigger_drain()` reports `is_some()`, so an
    /// eligible cdylib data-trigger input is wired `DrainSource::Unified` (one
    /// iceoryx2 receive per hop) and `drain_trigger_input` forwards the
    /// level-boundary drain across the FFI. `None` (a raw-FFI or older
    /// cdylib) keeps today's dual-subscriber path byte-identical (ADDITIVE, no
    /// ABI bump). Only macro-generated cdylibs export it, and their reads are
    /// generated `try_view` (the frozen-slot-served path), so symbol presence
    /// also structurally enforces the trait's READ-PATH CONTRACT for the
    /// production surface.
    drain_trigger_ffi:
        Option<unsafe extern "C" fn(u64, *const u8, usize, *mut u64, *mut u64, *mut i32) -> i32>,
    /// The OPTIONAL between-fires REFILL FFI symbol
    /// (`cerulion_node_refill_trigger_input`, same signature as
    /// `drain_trigger_ffi`), resolved best-effort by `load` exactly like its
    /// sibling. ADDITIVE — symbol PRESENCE is the capability
    /// ([`NodeEntry::refills_trigger_input`]), so a raw-FFI or older
    /// cdylib simply lacks it and its Unified bursts are served one frame per
    /// step (the pre-refill throughput), never a same-frame re-fire. NO ABI
    /// bump, on the snapshot-pair / drain-export precedent.
    ///
    /// It cannot ride the drain symbol: the two sites disagree about what an
    /// unserved frozen head means, and that disagreement is the whole fix (see
    /// [`crate::transport::subscriber::CerulionSubscriber::refill_for_trigger`]).
    refill_trigger_ffi:
        Option<unsafe extern "C" fn(u64, *const u8, usize, *mut u64, *mut u64, *mut i32) -> i32>,
    /// The OPTIONAL per-set Sync head-op FFI symbol
    /// (`cerulion_node_sync_head_op(handle, name_ptr, name_len, op, out_ts,
    /// out_kind) -> i32`), resolved best-effort by `load` exactly like the drain
    /// and refill symbols above. Symbol PRESENCE is the capability
    /// ([`NodeEntry::supports_sync_head_ops`]), so a raw-FFI or older
    /// cdylib simply lacks it and its Sync node keeps the legacy
    /// latest-per-set semantics. ADDITIVE, with NO ABI bump of its own — the
    /// ONE bump this feature takes is `next_head`'s (v15), on the
    /// snapshot-pair / drain-export precedent.
    ///
    /// It carries only the FOUR ops that have no existing home: the two FILL
    /// ops ride this entry's own [`NodeEntry::drain_trigger_input`] /
    /// [`NodeEntry::refill_trigger_input`], which already cross
    /// `cerulion_node_{drain,refill}_trigger_input`.
    sync_head_op_ffi:
        Option<unsafe extern "C" fn(u64, *const u8, usize, u32, *mut u64, *mut i32) -> i32>,
    /// The OPTIONAL state capture/restore FFI pair, resolved
    /// all-three-or-none by `load` (mirrors [`SnapshotFns`]). `None` for a
    /// raw-FFI cdylib or one built without the state exports, which is what makes them ADDITIVE with
    /// no ABI bump.
    state_ffi: Option<StateFns>,
    /// This cdylib's `CerulionState::STATE_SHAPE`, read ONCE at
    /// load through `cerulion_node_state_shape`.
    ///
    /// Cached rather than queried per call because the shape is a compile-time
    /// const of the node TYPE, and one cdylib carries one type. That is not a
    /// micro-optimization: querying it through a handle would make
    /// [`NodeEntry::state_shape`] answer `None` for a node that simply has not
    /// been `init`'d yet — a node that DOES declare state reporting that it
    /// does not, which is precisely the silent Default-fabrication the restore
    /// path exists to refuse.
    state_shape: Option<u64>,
    /// This cdylib's `CerulionState::INLINE_SAFE`, read
    /// ONCE at load through `cerulion_node_inline_safe`.
    ///
    /// `false` for a raw-FFI cdylib or one built without the symbol, which is the SAFE default in
    /// both directions: such a node takes the fork carrier, whose bytes are
    /// identical, so the cost of being wrong is a `fork(2)` and never a wrong
    /// capture. Cached for the same reason as [`Self::state_shape`] — it is a
    /// compile-time const of the node TYPE — and additionally because the
    /// boundary reads it once per node per anchor on the node thread, where an
    /// FFI call and a lock acquisition are exactly what must not be
    /// there.
    inline_safe: bool,
    /// The OPTIONAL pre-fork lock probe. `None` for a
    /// cdylib that exports no probe, which answers the trait default (`true`).
    ///
    /// NOT cached, because unlike the two above it is a question about the
    /// INSTANCE and its answer changes every time a lock in the node's declared
    /// state is taken or released.
    cer_probe_ffi: Option<unsafe extern "C" fn(u64) -> i32>,
    /// Flood-control latch for the per-step
    /// `drain_trigger_input` FFI call — the drain runs at EVERY level boundary
    /// for the node's level, so a PERSISTENT failure (e.g. a sibling cdylib
    /// poisoned the `NODES` mutex) would `error!`-flood exactly like the
    /// `snapshot_error_latched` twin above. Same [`flood_log_action`] policy:
    /// first failure loud, then `debug!` until recovery.
    drain_trigger_error_latched: bool,
    /// The `drain_trigger_error_latched` twin for the between-fires
    /// REFILL FFI call. Its own latch, not a shared one: the two calls fail for
    /// different reasons at different rates (a refill runs once per fire of a
    /// burst, a drain once per level pass), and one open regime must not swallow
    /// the other's loud head (the per-site rule).
    refill_trigger_error_latched: bool,
    /// The same twin for the per-set Sync head-op FFI call. Its OWN
    /// latch for the same reason: the align driver issues up to a handful of
    /// ops per input per pass, so a persistent failure here fails at a
    /// different rate from a drain or a refill, and one open regime must not
    /// swallow another's loud head.
    sync_head_op_error_latched: bool,
    /// OPTIONAL external-source FFI symbol
    /// (`cerulion_node_external_source(handle, out_fd) -> i32`), resolved
    /// best-effort by `load` (mirrors [`SnapshotFns`]). `None` for a non-external
    /// node (the macro emits the export only for `#[cerulion_node(external)]`) OR
    /// an older cdylib; in both cases [`NodeEntry::external_source`] returns
    /// `None` (back-compat), so this is ADDITIVE and does NOT bump the ABI
    /// version. When `Some`, `external_source` invokes it and maps the returned
    /// [`EXTERNAL_SOURCE_KIND_DEVICE_FD`] / `_DOORBELL_FD` / `_HOST_DRIVEN` / `_ERROR`
    /// code to an [`ExternalSource`].
    external_source_fn: Option<unsafe extern "C" fn(u64, *mut i64) -> i32>,
    /// Set by [`NodeEntry::external_source`] when the FFI
    /// returned [`EXTERNAL_SOURCE_KIND_DOORBELL_FD`] (a tier-2 `Blocking` collapse
    /// pipe read end) rather than [`EXTERNAL_SOURCE_KIND_DEVICE_FD`]. Read back by
    /// [`NodeEntry::external_source_is_drained_doorbell_fd`] so the runtime binds
    /// the returned `Fd` as a DRAINED doorbell (read to EAGAIN each sweep) instead
    /// of a poll-only device fd — the public 3-variant `ExternalSource` stays
    /// unchanged. `external_source` is queried once, so this is written once.
    external_is_doorbell_fd: bool,
    /// Set alongside `external_is_doorbell_fd` when the FFI
    /// reported [`EXTERNAL_SOURCE_KIND_DOORBELL_FD`] — the cdylib spawned a
    /// DETACHED helper thread (see [`spawn_cdylib_blocking_doorbell`]) whose
    /// loop and closure both execute the CDYLIB's own code. Unloading the
    /// library (`Drop` → dlclose) while that thread may still run is a
    /// use-after-unmap. glibc happens to keep the mapping alive (the cdylib's
    /// TLS forces `DF_1_NODELETE`), but that is a libc implementation detail,
    /// not a guarantee (musl/macOS differ). When set, `Drop` LEAKS the library
    /// handle instead of dlclosing — the standard libloading remedy for
    /// libraries with threads that can outlive the handle. Bounded: one mapping
    /// per loaded Blocking cdylib, only when its doorbell was materialized.
    leak_library_on_drop: bool,
    /// Diagnostic label for error messages (typically the cdylib path).
    /// Replaces the role that `NodeInfo::node_type` used to play.
    diag_label: String,
    // _library must be last: drop order ensures FFI pointers remain valid
    // when Drop calls shutdown_fn. `Option` only so Drop can `take()` + leak it
    // when `leak_library_on_drop` is set; it is `Some` for the entry's whole
    // life otherwise.
    _library: Option<libloading::Library>,
}

// The manual `unsafe impl Send for DylibNodeEntry` was deleted —
// its `NodeContext` is now `Send` by construction (ipc_threadsafe ports), the
// FFI fn pointers are `Send`, and `libloading::Library` is `Send + Sync`, so
// `DylibNodeEntry` is `Send` by auto-derivation.

/// The pair of optional cdylib snapshot FFI symbols. Resolved TOGETHER
/// in [`DylibNodeEntry::load`] so a partial (exactly-one-symbol) state is
/// unrepresentable past load — a cdylib that exports exactly ONE symbol is
/// normalized to `None` (with a warn; a one-symbol export is a build mistake,
/// since the macro always emits both), and an older cdylib that exports
/// NEITHER is `None` (back-compat no-hold). Both fn pointers are `Copy`.
#[derive(Clone, Copy)]
struct SnapshotFns {
    /// `cerulion_node_set_snapshot_inputs(handle, names_ptr, names_len) -> i32`.
    set: unsafe extern "C" fn(u64, *const u8, usize) -> i32,
    /// `cerulion_node_snapshot_inputs(handle) -> i32`.
    snapshot: unsafe extern "C" fn(u64) -> i32,
}

/// The cdylib state FFI's "the HOST's sink refused" return code.
///
/// It is read as well as the [`StateSinkBridge`]'s own `full` flag, and that is
/// not belt-and-braces: a hash-like container consults `remaining_hint` and
/// calls `StateSink::refuse` **without ever attempting a write**, so the
/// refusal never reaches the trampoline and the bridge's flag stays clear. The
/// code is the only evidence in that case — and it is exactly the case
/// `capacity_hint` exists to produce.
const STATE_FFI_HOST_SINK_REFUSED: i32 = -6;

/// The host's side of the cdylib state sink callback.
///
/// `user` in the C ABI is a pointer to one of these. It carries the caller's
/// own sink plus the two things the callback must report back through a bare
/// `i32`: whether the sink refused, and whether the sink PANICKED.
struct StateSinkBridge<'a> {
    sink: &'a mut dyn crate::state::StateSink,
    /// The sink returned `SinkFull`.
    full: bool,
    /// The sink UNWOUND. Distinguished from `full` because they mean opposite
    /// things to an operator — one says the buffer was too small, the other
    /// says the caller's sink is broken — and both must reach the log, since
    /// the cdylib sees only a non-zero return either way.
    panicked: bool,
}

/// The trampoline the cdylib calls for every chunk of a capture.
///
/// `extern "C"` and therefore ABORT-on-unwind since Rust 1.71, so the
/// `catch_unwind` is not defensive tidiness: a panicking sink would otherwise
/// take the whole robot down mid-capture, where the correct outcome is a
/// refused capture and a loud error.
extern "C" fn state_sink_trampoline(
    user: *mut std::ffi::c_void,
    chunk: *const u8,
    len: usize,
) -> i32 {
    if user.is_null() {
        return -1;
    }
    // Safety: `user` is the `&mut StateSinkBridge` this process passed to
    // `cerulion_node_capture_state` moments ago; the cdylib only calls this
    // during that call, on this thread, and never retains the pointer.
    let bridge = unsafe { &mut *(user as *mut StateSinkBridge) };
    let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        // `from_raw_parts` over a null pointer is UB even at length zero,
        // and a node whose every field is escaped really
        // does write zero bytes, so the empty slice is built by hand.
        let bytes: &[u8] = if len == 0 {
            &[]
        } else if chunk.is_null() {
            return Err(());
        } else {
            // Safety: non-null (checked) and the cdylib passes a live slice of
            // exactly `len` bytes, valid for this call; we only read it.
            unsafe { std::slice::from_raw_parts(chunk, len) }
        };
        bridge.sink.write(bytes).map_err(|_| ())
    }));
    match outcome {
        Ok(Ok(())) => 0,
        Ok(Err(())) => {
            bridge.full = true;
            -1
        }
        Err(_) => {
            bridge.panicked = true;
            -2
        }
    }
}

/// The three optional cdylib state symbols, resolved TOGETHER in
/// [`DylibNodeEntry::load`] so a partial export is unrepresentable past load.
///
/// All-three-or-none for the same reason [`SnapshotFns`] is both-or-neither:
/// the macro emits all three inside one `#[cfg(feature = "cdylib")]` block, so
/// a subset is a build mistake, and the SAFE degradation is to declare no state
/// rather than to half-restore a node.
#[derive(Clone, Copy)]
struct StateFns {
    /// `cerulion_node_state_shape(out_shape) -> i32`. Handle-less: the shape is
    /// a const of the type.
    shape: unsafe extern "C" fn(*mut u64) -> i32,
    /// `cerulion_node_capture_state(handle, sink, user, capacity_hint) -> i32`.
    capture: unsafe extern "C" fn(
        u64,
        Option<extern "C" fn(*mut std::ffi::c_void, *const u8, usize) -> i32>,
        *mut std::ffi::c_void,
        u64,
    ) -> i32,
    /// `cerulion_node_restore_state(handle, payload_ptr, payload_len) -> i32`.
    restore: unsafe extern "C" fn(u64, *const u8, usize) -> i32,
}

/// Lifecycle of a holding cdylib's snapshot-input registration. The
/// build-fixed non-trigger input names are marshalled to the cdylib exactly ONCE
/// (`Pending` → `Active`); every later step is a zero-alloc `snapshot` call.
///
/// `Failed` is the terminal state: if the one-time
/// `set_snapshot_inputs` FFI call fails (realistically a sibling cdylib tick
/// poisoned the process-global `NODES` mutex → `set` returns `-1` and the names
/// were never stored), the node can no longer hold its inputs. It transitions to
/// `Failed` and STOPS calling `snapshot` — rather than latching as if
/// `set` had succeeded and then calling `snapshot` every step, which would freeze
/// nothing yet return `0` (success), SILENTLY masking the lost
/// hold. `Failed` is terminal: a
/// failed registration NEVER later masquerades as `Active`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum SnapshotState {
    /// Names not yet registered with the cdylib.
    Pending,
    /// Names registered (`set` succeeded); each step is a zero-alloc snapshot.
    Active,
    /// `set` failed terminally; the node runs UNHELD and `snapshot` is NOT
    /// called (the lost hold is loud — logged once — never silently masked).
    Failed,
}

impl SnapshotState {
    /// Fold the result of the one-time `set_snapshot_inputs` FFI call into the
    /// state. `Pending` + success (`ret == 0`) → `Active`; `Pending` + failure →
    /// `Failed`. `Active`/`Failed` are terminal (`set` is invoked only while
    /// `Pending`, so this is a no-op for them — pinned for the invariant that a
    /// `Failed` registration can NEVER become `Active`, even on a later `0`).
    fn after_set(self, ret: i32) -> SnapshotState {
        match self {
            SnapshotState::Pending if ret == 0 => SnapshotState::Active,
            SnapshotState::Pending => SnapshotState::Failed,
            terminal => terminal,
        }
    }

    /// Whether the per-step `snapshot` FFI call should run. ONLY in `Active` —
    /// crucially `false` in `Failed`, so a failed `set` never falls through to a
    /// `snapshot` that freezes nothing yet reports success.
    fn should_invoke_snapshot(self) -> bool {
        matches!(self, SnapshotState::Active)
    }
}

impl DylibNodeEntry {
    /// Test seam: whether this entry will LEAK its `Library` handle on drop
    /// (latched by [`Self::external_source`] when a `Blocking` source was
    /// collapsed to a doorbell — a detached helper thread makes `dlclose` a
    /// use-after-unmap). The `debug!` breadcrumb that reports the leak does not
    /// exist in a release build, so this flag is the level-free complement a
    /// test can assert in every profile.
    #[cfg(any(test, feature = "test-helpers"))]
    #[doc(hidden)]
    pub fn leaks_library_on_drop(&self) -> bool {
        self.leak_library_on_drop
    }

    /// Load a node from a dynamic library path.
    ///
    /// # Errors
    ///
    /// Returns `NodeError` if the library cannot be loaded or required
    /// symbols are missing.
    pub fn load(path: &std::path::Path) -> TransportResult<Self> {
        // Safety: We trust the library at the given path to export the expected symbols.
        let library =
            unsafe { libloading::Library::new(path) }.map_err(|e| TransportError::NodeError {
                node_id: path.display().to_string(),
                reason: format!("failed to load library: {}", e),
            })?;

        let path_str = path.display().to_string();

        // ABI version check — must match before loading any other symbols.
        // This prevents undefined behavior from calling functions with changed signatures.
        unsafe {
            let abi_sym: Result<libloading::Symbol<unsafe extern "C" fn() -> u32>, _> =
                library.get(b"cerulion_abi_version\0");
            match abi_sym {
                Ok(abi_fn) => {
                    let version = abi_fn();
                    if version != crate::CERULION_ABI_VERSION {
                        return Err(TransportError::NodeError {
                            node_id: path_str,
                            reason: format!(
                                "ABI version mismatch: library exports version {}, expected {} \
                                 — rebuild the node crate against this core (`cerulion node \
                                 build <type>`, or `cargo build` in the node's crate). A \
                                 cdylib compiled against a different core lays out the \
                                 structs it shares with the host at different offsets, so \
                                 loading it would CORRUPT state rather than merely miss a \
                                 feature — which is why this refuses instead of degrading.",
                                version,
                                crate::CERULION_ABI_VERSION
                            ),
                        });
                    }
                }
                Err(e) => {
                    return Err(TransportError::NodeError {
                        node_id: path_str,
                        reason: format!(
                            "missing symbol cerulion_abi_version (required for ABI compatibility): {}",
                            e
                        ),
                    });
                }
            }
        }

        // The RUSTC FINGERPRINT check (ABI v22) must run right after the
        // ABI-version check and before any other symbol is called.
        // `NodeContext` crosses this FFI as a raw `Box`, and a `repr(Rust)`
        // type reachable through it (`CerulionSubscriber.frozen:
        // Option<FrozenSlot>`) is laid out however THIS cdylib's rustc
        // decided, which can differ from the host's decision even when every
        // struct size and offset (what the ABI-version check above, and the
        // `abi_layout` pin behind it, actually prove) agrees: rustc 1.97.0
        // changed the `None` bit pattern for a niche-holding 4-variant
        // `Option` without moving anything. Loading such a cdylib reads a live
        // `Some(..)` where the writer meant `None`, and its drop glue frees
        // uninitialised bytes, so `libmalloc` calls `abort()` and the process
        // dies on SIGABRT with no panic text and no backtrace. Refuse here,
        // before that state has a chance to reach `drain_for_trigger`.
        let fingerprint_fn = unsafe {
            let sym: libloading::Symbol<unsafe extern "C" fn() -> *const std::ffi::c_char> =
                library.get(b"cerulion_rustc_fingerprint\0").map_err(|e| {
                    TransportError::NodeError {
                        node_id: path_str.clone(),
                        reason: format!(
                            "missing symbol cerulion_rustc_fingerprint (required at ABI {}): \
                             {}. This node crate predates the toolchain guard; rebuild it \
                             (`cerulion node build <type>`)",
                            crate::CERULION_ABI_VERSION,
                            e
                        ),
                    }
                })?;
            *sym
        };
        let node_fingerprint = unsafe {
            let ptr = fingerprint_fn();
            if ptr.is_null() {
                return Err(TransportError::NodeError {
                    node_id: path_str,
                    reason: "cerulion_rustc_fingerprint returned a null pointer".to_string(),
                });
            }
            std::ffi::CStr::from_ptr(ptr).to_string_lossy().into_owned()
        };
        if node_fingerprint != crate::RUSTC_FINGERPRINT {
            let host_release = crate::RUSTC_RELEASE;
            return Err(TransportError::NodeError {
                node_id: path_str,
                reason: format!(
                    "rustc mismatch: this node was built by rustc {node_fingerprint}, but this \
                     host (`cerulion`) was built by rustc {host}. A `repr(Rust)` type crossing \
                     the node FFI boundary (`Option<FrozenSlot>` inside `NodeContext`) can be \
                     laid out differently by different rustc releases even when every struct \
                     size and offset agrees, so loading this node would corrupt state rather \
                     than merely miss a feature, and this refuses instead of risking that. Fix \
                     it one of two ways: (1) `rustup toolchain install {host_release}`, then \
                     `RUSTUP_TOOLCHAIN={host_release} cerulion node build <type>` to rebuild \
                     this node with the host's compiler; or (2) `cargo install --locked \
                     cerulion_cli` to rebuild the `cerulion` host itself with your current \
                     default compiler, so host and node share one toolchain again.",
                    host = crate::RUSTC_FINGERPRINT,
                ),
            });
        }

        // Safety: We look up symbols by their expected names and signatures.
        // If the library doesn't export them or uses different signatures,
        // this is undefined behavior — enforced by convention.
        let info_fn = unsafe {
            let sym: libloading::Symbol<unsafe extern "C" fn() -> *const std::ffi::c_char> =
                library
                    .get(b"cerulion_node_info\0")
                    .map_err(|e| TransportError::NodeError {
                        node_id: path_str.clone(),
                        reason: format!("missing symbol cerulion_node_info: {}", e),
                    })?;
            *sym
        };

        let init_fn = unsafe {
            let sym: libloading::Symbol<unsafe extern "C" fn(*mut NodeContext) -> u64> = library
                .get(b"cerulion_node_init\0")
                .map_err(|e| TransportError::NodeError {
                    node_id: path_str.clone(),
                    reason: format!("missing symbol cerulion_node_init: {}", e),
                })?;
            *sym
        };

        let tick_fn = unsafe {
            let sym: libloading::Symbol<unsafe extern "C" fn(u64) -> i32> = library
                .get(b"cerulion_node_tick\0")
                .map_err(|e| TransportError::NodeError {
                    node_id: path_str.clone(),
                    reason: format!("missing symbol cerulion_node_tick: {}", e),
                })?;
            *sym
        };

        // ABI v7: required symbol. A pre-v7 cdylib
        // missing it loud-fails at load here (the abi_version check above
        // also catches that, but defending here keeps the diagnostic
        // specific) rather than silently never servicing late joiners.
        let pump_history_fn = unsafe {
            let sym: libloading::Symbol<unsafe extern "C" fn(u64) -> i32> = library
                .get(b"cerulion_node_pump_history\0")
                .map_err(|e| TransportError::NodeError {
                    node_id: path_str.clone(),
                    reason: format!("missing symbol cerulion_node_pump_history: {}", e),
                })?;
            *sym
        };

        let shutdown_fn = unsafe {
            let sym: libloading::Symbol<unsafe extern "C" fn(u64) -> i32> = library
                .get(b"cerulion_node_shutdown\0")
                .map_err(|e| TransportError::NodeError {
                    node_id: path_str.clone(),
                    reason: format!("missing symbol cerulion_node_shutdown: {}", e),
                })?;
            *sym
        };

        // Load the new error-message
        // accessor symbols. Required at ABI v3; loading fails cleanly
        // with a "missing symbol" error if a stale v2 cdylib is present
        // (the abi_version check above also catches that, but defending
        // here too means the diagnostic stays specific even if someone
        // ever forgets to bump the constant).
        let take_last_error_fn = unsafe {
            let sym: libloading::Symbol<unsafe extern "C" fn() -> *mut std::ffi::c_char> = library
                .get(b"cerulion_take_last_error\0")
                .map_err(|e| TransportError::NodeError {
                    node_id: path_str.clone(),
                    reason: format!("missing symbol cerulion_take_last_error: {}", e),
                })?;
            *sym
        };
        let free_error_fn = unsafe {
            let sym: libloading::Symbol<unsafe extern "C" fn(*mut std::ffi::c_char)> = library
                .get(b"cerulion_free_error\0")
                .map_err(|e| TransportError::NodeError {
                    node_id: path_str.clone(),
                    reason: format!("missing symbol cerulion_free_error: {}", e),
                })?;
            *sym
        };

        // OPTIONAL snapshot symbols, resolved best-effort. An
        // older cdylib does NOT export them, and that MUST still load (the
        // ABI version is deliberately NOT bumped for these additive symbols);
        // when absent, `snapshot_inputs` is the inherited no-op (back-compat).
        // `*sym` copies the raw fn pointer out, detaching it from the `library`
        // borrow exactly like the required symbols above.
        // Safety: same convention as the required symbol lookups — we resolve by
        // name + signature; a present symbol is trusted to match the ABI.
        let set_snapshot_inputs_fn = unsafe {
            library
                .get::<unsafe extern "C" fn(u64, *const u8, usize) -> i32>(
                    b"cerulion_node_set_snapshot_inputs\0",
                )
                .ok()
                .map(|sym| *sym)
        };
        let snapshot_inputs_fn = unsafe {
            library
                .get::<unsafe extern "C" fn(u64) -> i32>(b"cerulion_node_snapshot_inputs\0")
                .ok()
                .map(|sym| *sym)
        };
        // Combine into the both-or-neither `SnapshotFns` so a partial state is
        // unrepresentable past this point (make the invariant a type,
        // decide it once here). Exactly ONE symbol is a build mistake — the macro
        // always emits BOTH inside one `#[cfg(feature="cdylib")]` block — so
        // disable the hold AND warn, rather than silently degrading to no-hold.
        let snapshot_ffi = match (set_snapshot_inputs_fn, snapshot_inputs_fn) {
            (Some(set), Some(snapshot)) => Some(SnapshotFns { set, snapshot }),
            (None, None) => None,
            _ => {
                tracing::warn!(
                    node = %path_str,
                    "cdylib exports only one of cerulion_node_set_snapshot_inputs / \
                     cerulion_node_snapshot_inputs; cross-step input hold DISABLED — \
                     export BOTH or NEITHER"
                );
                None
            }
        };

        // OPTIONAL external-source symbol — resolved best-effort,
        // exactly like the snapshot symbols above. The macro emits it ONLY for
        // `#[cerulion_node(external)]` nodes, so a non-external node (or an
        // older cdylib) simply lacks it → `external_source()` returns `None`
        // (back-compat; the ABI version is deliberately NOT bumped).
        // Safety: same convention as the other symbol lookups — resolve by name +
        // signature; a present symbol is trusted to match the ABI.
        let external_source_fn = unsafe {
            library
                .get::<unsafe extern "C" fn(u64, *mut i64) -> i32>(
                    b"cerulion_node_external_source\0",
                )
                .ok()
                .map(|sym| *sym)
        };

        // OPTIONAL unified-trigger-drain symbol — resolved
        // best-effort, exactly like the snapshot / external-source symbols
        // above. PRESENCE is the capability (`unifies_trigger_drain()`); a
        // symbol-less (raw-FFI / older) cdylib keeps the dual-subscriber
        // drain (back-compat; the ABI version is deliberately NOT bumped).
        // Safety: same convention as the other symbol lookups — resolve by
        // name + signature; a present symbol is trusted to match the ABI.
        let drain_trigger_ffi = unsafe {
            library
                .get::<unsafe extern "C" fn(
                    u64,
                    *const u8,
                    usize,
                    *mut u64,
                    *mut u64,
                    *mut i32,
                ) -> i32>(b"cerulion_node_drain_trigger_input\0")
                .ok()
                .map(|sym| *sym)
        };

        // OPTIONAL between-fires REFILL symbol — resolved
        // best-effort, exactly like the drain symbol above and INDEPENDENTLY of
        // it (an older macro cdylib exports the drain and not this one,
        // and must keep working: it stays Unified and its bursts are served one
        // frame per step). ADDITIVE; the ABI version is deliberately NOT bumped.
        // Safety: same convention as every other symbol lookup — resolve by
        // name + signature; a present symbol is trusted to match the ABI.
        let refill_trigger_ffi = unsafe {
            library
                .get::<unsafe extern "C" fn(
                    u64,
                    *const u8,
                    usize,
                    *mut u64,
                    *mut u64,
                    *mut i32,
                ) -> i32>(b"cerulion_node_refill_trigger_input\0")
                .ok()
                .map(|sym| *sym)
        };

        // OPTIONAL per-set Sync head-op symbol — resolved
        // best-effort, exactly like the drain / refill symbols above and
        // INDEPENDENTLY of both (an older macro cdylib exports those and
        // not this one, and must keep working: its Sync node keeps the legacy
        // latest-per-set semantics). PRESENCE is the capability
        // (`supports_sync_head_ops()`). ADDITIVE; the ABI version is
        // deliberately NOT bumped here.
        // Safety: same convention as every other symbol lookup — resolve by
        // name + signature; a present symbol is trusted to match the ABI.
        let sync_head_op_ffi = unsafe {
            library
                .get::<unsafe extern "C" fn(u64, *const u8, usize, u32, *mut u64, *mut i32) -> i32>(
                    b"cerulion_node_sync_head_op\0",
                )
                .ok()
                .map(|sym| *sym)
        };

        // OPTIONAL state symbols — resolved best-effort, exactly
        // like the snapshot / external-source / drain symbols above. A raw-FFI
        // cdylib or an older build exports none, declares no state, and loads unchanged
        // (ADDITIVE; the ABI version is deliberately NOT bumped).
        // Safety: same convention as every other symbol lookup — resolve by
        // name + signature; a present symbol is trusted to match the ABI.
        let state_shape_fn = unsafe {
            library
                .get::<unsafe extern "C" fn(*mut u64) -> i32>(b"cerulion_node_state_shape\0")
                .ok()
                .map(|sym| *sym)
        };
        let capture_state_fn = unsafe {
            library
                .get::<unsafe extern "C" fn(
                    u64,
                    Option<extern "C" fn(*mut std::ffi::c_void, *const u8, usize) -> i32>,
                    *mut std::ffi::c_void,
                    u64,
                ) -> i32>(b"cerulion_node_capture_state\0")
                .ok()
                .map(|sym| *sym)
        };
        let restore_state_fn = unsafe {
            library
                .get::<unsafe extern "C" fn(u64, *const u8, usize) -> i32>(
                    b"cerulion_node_restore_state\0",
                )
                .ok()
                .map(|sym| *sym)
        };
        // The two CARRIER symbols, resolved independently of
        // the state trio above and of each other. They are not part of that
        // all-three-or-none group on purpose: the trio decides whether a node
        // has restorable state AT ALL (a partial export there is a node that
        // could be captured and never restored, which is a trap), while these
        // two only decide WHICH CARRIER encodes it. Missing either costs a
        // `fork(2)`, never a wrong capture — so refusing the whole capability
        // over one absent symbol would trade a real anchor for tidiness.
        let inline_safe_fn = unsafe {
            library
                .get::<unsafe extern "C" fn() -> i32>(b"cerulion_node_inline_safe\0")
                .ok()
                .map(|sym| *sym)
        };
        let cer_probe_ffi = unsafe {
            library
                .get::<unsafe extern "C" fn(u64) -> i32>(b"cerulion_node_cer_probe\0")
                .ok()
                .map(|sym| *sym)
        };
        // Read ONCE, here, while the library is known-good — the same reason
        // `state_shape` is read here. A negative return is the exporter saying
        // it could not answer, and the safe reading of that is the same as the
        // absent symbol's: NOT inline-safe, so the node forks.
        let inline_safe = match inline_safe_fn {
            None => false,
            Some(f) => {
                // SAFETY: resolved by name and signature; a present symbol is
                // trusted to match the ABI, exactly as every other lookup here.
                let ret = unsafe { f() };
                if ret < 0 {
                    tracing::warn!(
                        node = %path_str,
                        ret,
                        "cdylib `cerulion_node_inline_safe` failed; this node will take \
                         the FORK carrier at every anchor. Its bytes are unaffected — \
                         both carriers run the same encoder — but it pays a fork(2) per \
                         cadence that an inline-safe node does not"
                    );
                    false
                } else {
                    ret != 0
                }
            }
        };
        let state_ffi = match (state_shape_fn, capture_state_fn, restore_state_fn) {
            (Some(shape), Some(capture), Some(restore)) => Some(StateFns {
                shape,
                capture,
                restore,
            }),
            (None, None, None) => None,
            _ => {
                tracing::warn!(
                    node = %path_str,
                    "cdylib exports only some of cerulion_node_state_shape / \
                     cerulion_node_capture_state / cerulion_node_restore_state; state \
                     capture and restore DISABLED for it — export ALL THREE or NONE"
                );
                None
            }
        };
        // Read the shape ONCE, here, while the library is known-good. It is a
        // compile-time const of the node type, so it needs no handle and no
        // lock — and asking now is what lets `state_shape()` answer correctly
        // before `init()` has run.
        //
        // A cdylib that exports the symbol and then refuses to answer takes the
        // capability away ENTIRELY (`state_ffi` is cleared with it), so
        // `state_shape.is_some()` and `state_ffi.is_some()` can never disagree.
        // Keeping the capture/restore pointers alive beside a `None` shape
        // would leave two answers to "can this node take a recorded anchor?"
        // and only one of them reachable from the restore path.
        let (state_ffi, state_shape) = match state_ffi {
            None => (None, None),
            Some(fns) => {
                let mut shape: u64 = 0;
                // Safety: `shape` is our own stack slot, writable for this call.
                let ret = unsafe { (fns.shape)(&mut shape) };
                if ret == 0 {
                    (Some(fns), Some(shape))
                } else {
                    // DRAIN the cdylib's `LAST_ERROR` here, for the two
                    // independent reasons `pump_history` already drains its own
                    // (see `FloodLogAction::FirstFailure` below):
                    //
                    // 1. It is the exporter's OWN account of why it refused,
                    //    and this log is the only place it can still be told —
                    //    the capability is being switched off on the next line,
                    //    so no later call will ever ask again.
                    // 2. An undrained message does not evaporate. `LAST_ERROR`
                    //    is a per-cdylib slot that persists until something
                    //    takes it, and `init`/`tick`/`shutdown` all read it as
                    //    `take_last_error().unwrap_or_else(|| "...no detail
                    //    provided")`. So a message left here is served as the
                    //    detail of the NEXT failure that sets none of its own —
                    //    a shape complaint reported against an unrelated tick.
                    //
                    // `self` does not exist yet, so this cannot use the
                    // `take_last_error` method; the two symbols are already
                    // resolved as locals and are used directly under the same
                    // contract the method documents (free through the SAME
                    // cdylib that allocated, pointer never reused).
                    let detail = take_last_error_via(take_last_error_fn, free_error_fn);
                    tracing::error!(
                        node = %path_str,
                        code = ret,
                        error = %detail.unwrap_or_else(|| {
                            format!(
                                "cerulion_node_state_shape returned {} ({}, no detail provided)",
                                ret,
                                ffi_error_description(ret),
                            )
                        }),
                        "cdylib cerulion_node_state_shape failed; this node will declare NO \
                         restorable state and a recorded anchor for it cannot be applied"
                    );
                    (None, None)
                }
            }
        };

        Ok(Self {
            info_fn,
            init_fn,
            tick_fn,
            pump_history_fn,
            shutdown_fn,
            take_last_error_fn,
            free_error_fn,
            snapshot_ffi,
            snapshot_state: SnapshotState::Pending,
            state_ffi,
            state_shape,
            inline_safe,
            cer_probe_ffi,
            info_cache: std::cell::OnceCell::new(),
            handle: None,
            pump_history_error_latched: false,
            snapshot_error_latched: false,
            drain_trigger_ffi,
            drain_trigger_error_latched: false,
            refill_trigger_ffi,
            refill_trigger_error_latched: false,
            sync_head_op_ffi,
            sync_head_op_error_latched: false,
            external_source_fn,
            external_is_doorbell_fd: false,
            leak_library_on_drop: false,
            diag_label: path_str,
            _library: Some(library),
        })
    }

    /// Pull the most recent error
    /// message off the cdylib's per-thread buffer (if any), copy it into
    /// an owned Rust String, and free the cdylib's allocation. Returns
    /// `None` only when the cdylib reports no error (null pointer).
    ///
    /// Invalid UTF-8 does NOT return `None` (that would
    /// silently drop the actual diagnostic). This uses
    /// `from_utf8_lossy` which substitutes U+FFFD for invalid sequences
    /// rather than discarding the message — operators see a partial
    /// message instead of the upstream's "no detail provided" fallback.
    fn take_last_error(&self) -> Option<String> {
        take_last_error_via(self.take_last_error_fn, self.free_error_fn)
    }

    /// Parse the JSON string returned by `cerulion_node_info()` into `NodeInfo`.
    ///
    /// Expected shape: `{"inputs": ["..."], "outputs": [...], "policy": {...}?}`.
    /// Missing `inputs` / `outputs` arrays default to empty with a
    /// warning. `policy` is optional; when absent, the node falls
    /// back to the default-Data trigger at runtime.
    ///
    /// `outputs` accepts BOTH the legacy
    /// array-of-strings shape (`["name1", "name2"]`) AND the new
    /// array-of-objects shape (`[{"name", "schema_hash",
    /// "max_slice_len_default"}, ...]`). The new shape carries
    /// per-output `OutputMeta` fields populated from
    /// `<T as ShmMessage>::SCHEMA_HASH` and `MAX_SLICE_LEN`. Old
    /// cdylibs ship the strings shape; the loader accepts
    /// both so a graph can mix old and new node libraries.
    ///
    /// The error type is a plain `String` (the reason
    /// text), NOT a `TransportError`. `parse_info_json` has no
    /// `diag_label` context — it cannot identify *which* cdylib produced
    /// the bytes. Its single meaningful caller, `info()`, owns that
    /// context and is responsible for constructing the operator-facing
    /// `TransportError::NodeInfoParse { diag_label, json_len, prefix,
    /// reason }`. Returning a bare reason string (rather than a
    /// `TransportError::NodeError { node_id: "unknown", .. }`) avoids
    /// double-nesting the placeholder `"Node 'unknown' error:"` prose
    /// inside the outer `NodeInfoParse.reason` field.
    ///
    /// This thin wrapper (kept for the fuzz target +
    /// the parse-shape unit tests, which have no node identity) delegates
    /// to [`parse_info_json_labeled`](Self::parse_info_json_labeled) with a
    /// placeholder label. The ERROR path is label-free as documented above;
    /// the label feeds only the non-propagating unknown-key WARN. The
    /// production path (`info()`) calls the labeled variant directly, so
    /// this wrapper is cfg-gated to its two remaining consumers (plain
    /// builds would otherwise flag it dead under `dead_code = deny`).
    #[cfg(any(test, feature = "fuzz-helpers"))]
    fn parse_info_json(json_str: &str) -> Result<NodeInfo, String> {
        Self::parse_info_json_labeled(json_str, "<unlabeled cdylib>")
    }

    /// The labeled body of `parse_info_json` (that wrapper is cfg-gated to
    /// tests + fuzz, so no intra-doc link here) — `info()` passes
    /// `self.diag_label` so the unknown-key warn can name the
    /// offending node.
    pub fn parse_info_json_labeled(json_str: &str, diag_label: &str) -> Result<NodeInfo, String> {
        if json_str.trim().is_empty() {
            return Err("empty JSON string from cerulion_node_info()".to_string());
        }

        #[derive(serde::Deserialize)]
        struct NodeInfoJson {
            // ABI v6: inputs may be objects carrying
            // `expect_within_ms` OR legacy bare strings — see `InputJson`.
            #[serde(default)]
            inputs: Option<Vec<InputJson>>,
            #[serde(default)]
            outputs: Option<Vec<OutputJson>>,
            // The macro-declared
            // policy. Optional: older cdylibs (or nodes without
            // any policy macro attr) omit the field entirely.
            #[serde(default)]
            policy: Option<PolicyJson>,
            // ABI v6: node-level QoS knobs. Absent on
            // pre-v6 cdylibs and on nodes without the attrs → `None` via
            // serde default. Carried only on the in-crate macro path
            // before ABI v6; now plumbed across the cdylib FFI.
            #[serde(default)]
            tick_within_ms: Option<u64>,
            #[serde(default)]
            throttle_ms: Option<u64>,
        }

        // ABI v6: each `inputs` entry is either a
        // bare string (legacy, pre-v6: name only) or a struct (new:
        // name + optional `expect_within_ms`). Mirrors the `OutputJson`
        // untagged shape: `serde(untagged)` tries the variants in order,
        // so a JSON string matches `Name` and a JSON object matches
        // `Full`. The legacy bare-string form MUST still parse (raw-FFI
        // CLI-templated nodes emit it, and the backward-compat contract
        // holds independent of the ABI gate).
        // ABI v8: the wire encoding of a declared
        // `#[input(backpressure = ...)]`. serde EXTERNAL tagging gives the
        // documented stable shape for free: `"drop_oldest"` / `"block"`
        // (unit variants from plain strings) and `{"sample":N}` (newtype
        // variant from a single-key map).
        // ABI v9: the `Full` object also carries the declared
        // `#[input(trigger)]` mark as `"trigger":true` (present only for a
        // trigger input; absent = a non-trigger latest-value read).
        #[derive(serde::Deserialize)]
        #[serde(rename_all = "snake_case")]
        enum BackpressureJson {
            DropOldest,
            Block,
            Sample(u64),
        }

        impl BackpressureJson {
            fn into_policy(self) -> BackpressurePolicy {
                match self {
                    Self::DropOldest => BackpressurePolicy::DropOldest,
                    Self::Block => BackpressurePolicy::Block,
                    Self::Sample(n) => BackpressurePolicy::Sample(n),
                }
            }
        }

        #[derive(serde::Deserialize)]
        #[serde(untagged)]
        enum InputJson {
            Name(String),
            Full {
                name: String,
                /// Per-input
                /// expected interval (ms). Absent / null = no watchdog.
                #[serde(default)]
                expect_within_ms: Option<u64>,
                /// ABI v8: the declared `#[input(depth = N)]`.
                /// Emitted by the macro only when EXPLICITLY declared;
                /// absent / null = not declared → the host resolves
                /// `DEFAULT_CONSUMER_DEPTH` (same value the in-process
                /// path resolves, so test/live parity holds). Optional
                /// even on v8 payloads — the strict ABI gate (a v7
                /// cdylib is refused at load) means an absent key can
                /// only mean "undeclared", never "stale build".
                #[serde(default)]
                depth: Option<usize>,
                /// ABI v8 (same bump): the declared
                /// backpressure. Same present-vs-absent contract as
                /// `depth`; absent = the `DropOldest` default. Pre-v8 a
                /// dylib `block`/`sample(N)` input silently degraded to
                /// `DropOldest` in production.
                #[serde(default)]
                backpressure: Option<BackpressureJson>,
                /// ABI v9: the declared `#[input(trigger)]`
                /// mark. The macro emits `"trigger":true` only for a
                /// trigger input; absent = a non-trigger latest-value
                /// read → serde default `false`. Pre-v9 the host
                /// hardcoded `false` here (trigger truth was derived
                /// from `MacroPolicy`), which lost the per-input
                /// identity a `Sync`/`UnboundedSync` node needs — see
                /// `CERULION_ABI_VERSION`.
                #[serde(default)]
                trigger: bool,
                /// The input port type's layout hash
                /// (`<T as ShmMessage>::SCHEMA_HASH`). The macro emits it
                /// on every input object (mirroring the output objects) —
                /// an ABI v11 STRICT-bump key, so a pre-v11 cdylib never
                /// reaches this parser. Absent = a raw-FFI info block that
                /// omits it → serde default `0`, the "no declared schema"
                /// sentinel.
                /// The network ingress-hash resolver
                /// (`resolve_ingress_schema_hash`) reads this to validate
                /// ingress frames against the consuming input's schema;
                /// before ABI v11 it was hardcoded `0`, so it refused every
                /// `DylibNodeEntry`-loaded consumer.
                #[serde(default)]
                schema_hash: u64,
            },
        }

        // Each `outputs` entry is either a
        // bare string (legacy: name only) or a struct (new: name +
        // schema_hash + max_slice_len_default). `serde(untagged)`
        // tries the variants in order; the bare-string variant
        // matches first when the wire form is a JSON string and the
        // struct variant matches when it's a JSON object.
        #[derive(serde::Deserialize)]
        #[serde(untagged)]
        enum OutputJson {
            Name(String),
            Full {
                name: String,
                #[serde(default)]
                schema_hash: u64,
                /// `null` in JSON → `None` here. JSON wire shape is
                /// `Option<usize>` (NOT `Option<MaxSliceLen>`) for
                /// forward-compat with older cdylibs that may emit
                /// `0`, values below `WireHeader::SIZE`, or values
                /// above `u32::MAX`. The conversion to the typed
                /// `Option<MaxSliceLen>` below uses
                /// `u32::try_from(...).ok().and_then(MaxSliceLen::try_new)`
                /// which silently promotes `Some(0)` to `None` (treated
                /// as "unset") and emits a `tracing::warn!` for the
                /// other invalid bands (`Some(1..32)` or `Some(> u32::MAX)`)
                /// before falling through to None.
                /// The typed representation is `Option<MaxSliceLen>`
                /// so the trait surface and wire surface match, with
                /// both bounds (lower `>= WireHeader::SIZE`, upper
                /// `<= u32::MAX`) encoded as type-system properties.
                #[serde(default)]
                max_slice_len_default: Option<usize>,
                /// Per-output committed
                /// interval (ms). Absent / null = no commitment.
                #[serde(default)]
                promise_within_ms: Option<u64>,
                /// The output port type's fixed wire-section
                /// size (`<T as ShmMessage>::WIRE_FIXED_SIZE`) — the
                /// layout fact the recorder stamps into a bag channel's
                /// `SchemaDescriptor`, taken from the node instead of from
                /// the workspace `schemas/` file the graph's `schema:`
                /// names.
                ///
                /// ADDITIVE, and deliberately NOT an ABI bump: absent (a
                /// raw-FFI info block, or a macro cdylib built before this
                /// key existed) → `None`, and the recorder falls back to
                /// the workspace file exactly as it did before this key existed.
                /// A strict bump would buy nothing over that degrade and
                /// would collide with the bumps two other in-flight
                /// changes are already claiming.
                ///
                /// `u64` on the wire so an OVER-RANGE value from a
                /// hand-written info block PARSES (and is then dropped to
                /// `None` below) rather than failing the whole entry as
                /// "no variant matched" — the untagged-enum hazard the
                /// unknown-key warn below exists for. Scope, stated: this
                /// widens the accepted RANGE, not the accepted TYPE. A
                /// value of the wrong JSON type (`-1`, `"64"`) still fails
                /// `Option<u64>`, and under `serde(untagged)` that fails
                /// the whole `outputs` array — and with it `parse_info_json`
                /// and the NODE LOAD — exactly as a wrong-typed
                /// `promise_within_ms` or `max_slice_len_default` does. A
                /// key-by-key salvage would need a `serde_json::Value`
                /// field and a hand-rolled coercion per key; not worth it
                /// for a shape no emitter in this repo can produce.
                #[serde(default)]
                wire_fixed_size: Option<u64>,
            },
        }

        let parsed: NodeInfoJson = serde_json::from_str(json_str)
            .map_err(|e| format!("invalid JSON from cerulion_node_info(): {}", e))?;

        if parsed.inputs.is_none() {
            tracing::warn!("inputs field missing in node info JSON, defaulting to empty");
        }
        if parsed.outputs.is_none() {
            tracing::warn!("outputs field missing in node info JSON, defaulting to empty");
        }

        // DX: unknown-key warn on Full-object port
        // entries. `serde(untagged)` cannot carry `deny_unknown_fields`
        // (an unknown key would make the `Full` variant unmatchable and
        // fail the WHOLE entry as "no variant matched" — worse than
        // ignoring), so a typo'd optional key (`"bakpressure"`) parses fine
        // and the declared intent is SILENTLY dropped (the field defaults).
        // Post-parse, re-walk the raw JSON as a `serde_json::Value` and warn
        // once per unknown key, naming the node (diag_label), the port, the
        // key, and the nearest legal key. Bare-string (legacy Name) entries
        // are untouched — nothing to scan. Cold path (node load), so the
        // second parse is free. Back-compat unchanged: warn-only, the entry
        // still loads with its defaults.
        //
        // Legal sets mirror what the macro EMITS + serde READS. `trigger`
        // is a real parsed key since ABI v9: the macro emits
        // `"trigger":true` for a `#[input(trigger)]` field and the parser
        // reads it into `InputMeta::trigger` (a hand-written raw-FFI info
        // block may carry it too).
        const LEGAL_INPUT_KEYS: &[&str] = &[
            "name",
            "trigger",
            "depth",
            "backpressure",
            "expect_within_ms",
            // The per-input port-type layout hash.
            "schema_hash",
        ];
        const LEGAL_OUTPUT_KEYS: &[&str] = &[
            "name",
            "schema_hash",
            "max_slice_len_default",
            "promise_within_ms",
            // The per-output port-type fixed wire-section size.
            "wire_fixed_size",
        ];
        // The TOP-LEVEL envelope keys — one entry per field of
        // `NodeInfoJson` above, in declaration order. Every one of those fields
        // is `#[serde(default)] Option<..>`, which is what makes a typo here
        // silent: `"polciy"` parses fine, `policy` takes its default `None`,
        // and the node runs with its declared trigger policy absent. The port
        // entries below are reported; without this check the envelope is not,
        // so a misspelled key is dropped in total silence at the one level a
        // hand-written raw-FFI info block is most likely to get wrong.
        //
        // WARN, not deny, for the reason the untagged variants below cannot
        // deny either — but a DIFFERENT one, and it is the load-bearing half:
        // this JSON crosses the cdylib ABI, so a NEWER cdylib emitting a key
        // an OLDER host has not learned must still load (the `#[serde(default)]`
        // on every field is that contract). Denying here would turn a
        // forward-compatible deploy into a node that refuses to start.
        const LEGAL_ENVELOPE_KEYS: &[&str] = &[
            "inputs",
            "outputs",
            "policy",
            "tick_within_ms",
            "throttle_ms",
        ];
        if let Ok(raw) = serde_json::from_str::<serde_json::Value>(json_str) {
            // Same `unknown_key` field name and the same did-you-mean as the
            // port-entry arm below, so ONE grep finds every unknown key this
            // parse dropped, at either level.
            if let Some(obj) = raw.as_object() {
                for key in obj.keys() {
                    if !LEGAL_ENVELOPE_KEYS.contains(&key.as_str()) {
                        let nearest = nearest_legal_key(key, LEGAL_ENVELOPE_KEYS);
                        tracing::warn!(
                            diag_label = %diag_label,
                            section = "<top-level>",
                            unknown_key = %key,
                            nearest = %nearest,
                            "unknown key in cerulion_node_info() JSON — ignored; \
                             the closest legal key is carried in the `nearest` \
                             field, and the node loads with the default for the \
                             intended field"
                        );
                    }
                }
            }
            for (section, legal) in [("inputs", LEGAL_INPUT_KEYS), ("outputs", LEGAL_OUTPUT_KEYS)] {
                let Some(entries) = raw.get(section).and_then(|v| v.as_array()) else {
                    continue;
                };
                for entry in entries {
                    let Some(obj) = entry.as_object() else {
                        continue; // bare-string Name form — untouched
                    };
                    let port = obj
                        .get("name")
                        .and_then(|v| v.as_str())
                        .unwrap_or("<unnamed>");
                    for key in obj.keys() {
                        if !legal.contains(&key.as_str()) {
                            let nearest = nearest_legal_key(key, legal);
                            tracing::warn!(
                                diag_label = %diag_label,
                                section,
                                port = %port,
                                unknown_key = %key,
                                nearest = %nearest,
                                "unknown key in cerulion_node_info() JSON — ignored; \
                                 the closest legal key is carried in the `nearest` \
                                 field, and the port loads with the default for the \
                                 intended field"
                            );
                        }
                    }
                }
            }
        }

        // The variants are mutually exclusive in macro-emitted
        // JSON (the macro's compile-time validator rejects combos
        // like `period_ms` + `#[input(trigger)]`). `into_macro_policy`
        // applies the documented variant priority (Period > Sync >
        // UnboundedSync > DataTrigger > External).
        let policy = parsed.policy.and_then(PolicyJson::into_macro_policy);

        // ABI v6 + ABI v8: normalise each
        // input entry to a named record. The legacy bare-string variant
        // (`InputJson::Name`) carries no QoS → all-`None` (raw-FFI
        // CLI-templated nodes emit it; their depth/backpressure are the
        // host defaults, matching their pre-v8 behavior).
        struct NormalizedInput {
            name: String,
            expect_within_ms: Option<u64>,
            depth: Option<usize>,
            backpressure: Option<BackpressureJson>,
            trigger: bool,
            // The declared port-type schema hash (0 = legacy /
            // undeclared sentinel).
            schema_hash: u64,
        }
        let input_entries: Vec<NormalizedInput> = parsed
            .inputs
            .unwrap_or_default()
            .into_iter()
            .map(|entry| match entry {
                InputJson::Name(name) => NormalizedInput {
                    name,
                    expect_within_ms: None,
                    depth: None,
                    backpressure: None,
                    // Legacy bare-string form carries no per-input QoS and
                    // no trigger mark (raw-FFI CLI-templated nodes emit it;
                    // their trigger membership rides `MacroPolicy`).
                    trigger: false,
                    // No declared schema on the bare-string form → 0 sentinel.
                    schema_hash: 0,
                },
                InputJson::Full {
                    name,
                    expect_within_ms,
                    depth,
                    backpressure,
                    trigger,
                    schema_hash,
                } => NormalizedInput {
                    name,
                    expect_within_ms,
                    depth,
                    backpressure,
                    trigger,
                    schema_hash,
                },
            })
            .collect();
        let input_names: Vec<String> = input_entries.iter().map(|e| e.name.clone()).collect();
        let output_meta: Vec<OutputMeta> = parsed
            .outputs
            .unwrap_or_default()
            .into_iter()
            .map(|entry| match entry {
                OutputJson::Name(name) => OutputMeta {
                    name,
                    schema_hash: 0,
                    max_slice_len_default: None,
                    promise_within_ms: None,
                    // The bare-string form declares no layout at
                    // all — "no claim", so the recorder sizes the channel
                    // from the workspace schema file, as it always has.
                    wire_fixed_size: None,
                },
                OutputJson::Full {
                    name,
                    schema_hash,
                    max_slice_len_default,
                    promise_within_ms,
                    wire_fixed_size,
                } => OutputMeta {
                    name: name.clone(),
                    schema_hash,
                    // Convert `Option<usize>` JSON
                    // wire shape to typed `Option<MaxSliceLen>`.
                    // `MaxSliceLen::try_new` enforces BOTH bounds:
                    // values `> u32::MAX` (theoretically possible
                    // from an older cdylib) become `None`;
                    // values `< WireHeader::SIZE` (i.e., 0-31)
                    // become `None`. `n == WireHeader::SIZE` (32) IS
                    // valid (header-only payload). `n == 0` is
                    // dropped silently (interpreted as "unset");
                    // `n in [1, 31]` and `n > u32::MAX` fire a
                    // `tracing::warn!` since they are clearly user-
                    // intent values that the type system rejects.
                    max_slice_len_default: max_slice_len_default.and_then(|n| {
                        u32::try_from(n)
                            .ok()
                            .and_then(MaxSliceLen::try_new)
                            .or_else(|| {
                                if n > 0 {
                                    tracing::warn!(
                                        output = %name,
                                        schema_hash,
                                        raw_value = n,
                                        "cdylib info JSON declared a max_slice_len_default \
                                         outside [WireHeader::SIZE, u32::MAX]; treating as \
                                         None (tier-3 DEFAULT_MAX_SLICE_LEN will fire). \
                                         Rebuild the cdylib with a value in range."
                                    );
                                }
                                None
                            })
                    }),
                    promise_within_ms,
                    // Narrow the `u64` wire shape to the `u32`
                    // the wire and the bag descriptor carry. `Some(0)` is
                    // a REAL answer (a purely-variable schema has a
                    // zero-byte fixed section), so it survives — unlike
                    // `max_slice_len_default` above, where 0 means unset.
                    // An out-of-range value is dropped to `None` — "no
                    // claim", so the recorder falls back to the workspace
                    // file — with a loud warn, never truncated: a
                    // descriptor that is confidently wrong is worse than
                    // one that defers.
                    wire_fixed_size: wire_fixed_size.and_then(|n| {
                        u32::try_from(n).ok().or_else(|| {
                            tracing::warn!(
                                output = %name,
                                schema_hash,
                                raw_value = n,
                                "cdylib info JSON declared a wire_fixed_size above u32::MAX, \
                                 which no Cerulion frame can carry (WireHeader::total_size is \
                                 u32); treating the port as declaring no size, so a recording \
                                 falls back to the workspace schema file. Rebuild the cdylib."
                            );
                            None
                        })
                    }),
                },
            })
            .collect();
        // The FFI half of the duplicate-port-name
        // enforcement: a buggy cdylib emitting two entries with the same
        // name would otherwise silently last-win in
        // `build_output_meta_lookup` (conflicting schema_hash /
        // max_slice_len) or first-win in input wiring. FFI input is
        // runtime-bound, so this is an `Err` (caught by `info()`'s loud
        // error!-and-default fallback) — the constructors' asserts guard
        // the in-crate paths and must not be reachable from FFI data.
        for (idx, name) in input_names.iter().enumerate() {
            if input_names[..idx].contains(name) {
                // `parse_info_json` returns `Err(String)`; the caller wraps it
                // into the structured `TransportError::NodeInfoParse`.
                return Err(format!(
                    "duplicate input port name '{name}' in cerulion_node_info() \
                     JSON — each port may be declared once"
                ));
            }
        }
        for (idx, m) in output_meta.iter().enumerate() {
            if output_meta[..idx].iter().any(|p| p.name == m.name) {
                return Err(format!(
                    "duplicate output port name '{}' in cerulion_node_info() \
                     JSON — each port may be declared once",
                    m.name
                ));
            }
        }
        // ABI v6: build `InputMeta` for each cdylib
        // input so the per-input `expect_within_ms` watchdog
        // reaches the runtime across the FFI. Pre-v6, the cdylib
        // path used `with_input_names_and_output_meta`, which left
        // `input_meta` EMPTY; `expect_within_ms` was therefore
        // registered "from nowhere" for dylib-loaded nodes.
        //
        // ABI v8: `depth` AND (same bump) `backpressure` are
        // now REAL across the FFI — the declared `#[input(depth = N,
        // backpressure = ...)]` arrives in the input object and feeds
        // `GraphTopology::build` exactly like the in-process path (pre-v8
        // depth was HARDCODED to `DEFAULT_CONSUMER_DEPTH` and backpressure
        // to `DropOldest` here — the silent test/live divergence class:
        // depth mis-sized buffers; a degraded `block` DROPPED DATA live
        // while tests passed). Absent keys = not declared → the same
        // defaults the in-process macro emission resolves. No range check
        // here: `GraphTopology` validates depth ≥ 1 and ≤
        // MAX_CONSUMER_DEPTH at graph load, loudly, for
        // hand-written/hostile payloads too.
        //
        // ABI v9: the declared `#[input(trigger)]` mark now rides
        // the wire, so `m.trigger` on the FFI path carries the REAL per-input
        // truth (pre-v9 it was hardcoded `false` and trigger membership was
        // derived from `MacroPolicy` alone — which lost the input identity a
        // `Sync`/`UnboundedSync` node needs). This reaches
        // `validate_no_silent_data_trigger` (which counts `m.trigger`) so the
        // FFI path warns/validates exactly like the in-process macro path — a
        // parity fix, not a fire change: the trigger-EDGE / snapshot / fire
        // classification still flows from `MacroPolicy` (`build_trigger_edges`
        // et al. read `info.policy`, not `m.trigger`), so nothing fires
        // differently. (Trigger-scoped Sync classification later flipped to consume
        // this per-input truth.)
        // `schema_hash` now rides the FFI input object (the macro
        // emits `<T as ShmMessage>::SCHEMA_HASH` per input, mirroring the
        // output objects). The network ingress-hash resolver
        // (`resolve_ingress_schema_hash`) reads it to validate ingress frames
        // against the consuming input's schema. Before ABI v11 this was HARDCODED
        // `0`, so the resolver refused EVERY DylibNodeEntry-loaded consumer
        // ("no consumer with a declared schema"). ABI v11 STRICT bump: a
        // pre-v11 cdylib is refused at load, so an omitted key here can only
        // be a raw-FFI info block — serde-defaults to `0`, the "undeclared"
        // sentinel the resolver fail-opens on. (The port-write shim rework deleted the never-executed `queue` / `max_age_ms` /
        // `filter_fn` members outright.)
        let input_meta: Vec<InputMeta> = input_entries
            .into_iter()
            .map(|entry| InputMeta {
                name: entry.name,
                schema_hash: entry.schema_hash,
                trigger: entry.trigger,
                depth: entry
                    .depth
                    .unwrap_or(crate::graph::topology::DEFAULT_CONSUMER_DEPTH),
                backpressure: entry
                    .backpressure
                    .map(BackpressureJson::into_policy)
                    .unwrap_or_default(),
                expect_within_ms: entry.expect_within_ms,
            })
            .collect();
        // Names are already proven unique by the explicit `Err`-returning
        // checks above, so `with_meta`'s unique-name assert cannot fire
        // here (it remains defense-in-depth for in-crate literal callers).
        let mut info = NodeInfo::with_meta(input_meta, output_meta);
        info.policy = policy;
        info.tick_within_ms = parsed.tick_within_ms;
        info.throttle_ms = parsed.throttle_ms;
        Ok(info)
    }
}

impl DylibNodeEntry {
    /// The RAW JSON document the library's `cerulion_node_info()` returns —
    /// exactly what [`NodeEntry::info`] parses, as text. A process that
    /// inspects a library on another's behalf (`cerulion-wsd --inspect-node`)
    /// prints this, and the caller parses it with
    /// [`DylibNodeEntry::parse_info_json_labeled`], so the two sides share one
    /// parser and the library is only ever loaded in the child.
    pub fn info_json(&self) -> TransportResult<String> {
        // Safety: cerulion_node_info returns a pointer to a null-terminated C string.
        let ptr = unsafe { (self.info_fn)() };
        if ptr.is_null() {
            // Null pointer refuses to load, same as malformed
            // JSON — the metadata is unusable either way. json_len 0 +
            // empty prefix make the "nothing was returned" case
            // distinguishable from a parse failure in the Display.
            // Principle #12: an unrecoverable node-load failure logs at
            // error level so a log aggregator watching tracing ERROR
            // events still fires.
            tracing::error!(
                diag_label = %self.diag_label,
                json_len = 0,
                "cerulion_node_info() returned a null pointer — refusing to load node"
            );
            return Err(TransportError::NodeInfoParse {
                diag_label: self.diag_label.clone(),
                json_len: 0,
                prefix: String::new(),
                reason: "cerulion_node_info() returned a null pointer".to_string(),
            });
        }
        // Safety: The pointer is expected to be valid and null-terminated.
        let c_str = unsafe { CStr::from_ptr(ptr) };
        Ok(c_str.to_string_lossy().into_owned())
    }
}

impl NodeEntry for DylibNodeEntry {
    /// Read the cdylib's port + policy metadata.
    ///
    /// **Caching note:** `info()` populates
    /// `info_cache` (a `OnceCell`, interior mutability) on its FIRST
    /// successful call. On the production node-load path `info()` is
    /// called twice — once by `runtime.rs` to read `macro_policy` for
    /// trigger-policy synthesis, and once by `init()` to seed the cache —
    /// but only the FIRST call re-enters the FFI; the second returns the
    /// cached value. This closes the prior double-FFI hazard where a
    /// non-idempotent `cerulion_node_info()` could return different bytes
    /// across the two calls, wiring publishers from one `NodeInfo` while
    /// caching another. Failed parses are NOT cached, so a corrupt node
    /// fails on every call.
    ///
    /// **Failure semantics:** if `cerulion_node_info()`
    /// returns null OR returns malformed JSON, this method returns
    /// `Err(TransportError::NodeInfoParse)` carrying the diagnostic
    /// label (cdylib path), the raw JSON byte length, and the first
    /// 80 bytes of the offending payload. `GraphRuntime::build` /
    /// `build_in_process` propagate this as a load-time failure —
    /// earlier the method silently fell back to
    /// `NodeInfo::default()` (empty wiring) and the node loaded but
    /// never fired.
    fn info(&self) -> TransportResult<NodeInfo> {
        if let Some(cached) = self.info_cache.get() {
            return Ok(cached.clone());
        }
        let json_str = self.info_json()?;

        // The labeled variant, so the unknown-key WARN names this
        // cdylib; the ERROR path stays label-free (wrapped below).
        let parsed =
            Self::parse_info_json_labeled(&json_str, &self.diag_label).map_err(|reason| {
                // Propagate instead of falling back to
                // NodeInfo::default(). json_len + first-80-bytes prefix +
                // diag_label ride along in the error
                // so operators can identify which cdylib is corrupted.
                // `reason` is a plain String from parse_info_json (no nested
                // "Node 'unknown' error:" prose) — info() owns the
                // diag_label context that parse_info_json lacks.
                let prefix = safe_utf8_prefix(&json_str, 80).to_string();
                // Principle #12: log the unrecoverable parse failure at error
                // level (exactly once) before propagating, so structured
                // alerting on tracing ERROR events still fires.
                tracing::error!(
                    diag_label = %self.diag_label,
                    json_len = json_str.len(),
                    prefix = %prefix,
                    reason = %reason,
                    "cerulion_node_info() returned unparseable JSON — refusing to load node"
                );
                TransportError::NodeInfoParse {
                    diag_label: self.diag_label.clone(),
                    json_len: json_str.len(),
                    prefix,
                    reason,
                }
            })?;

        // Populate the cache on the FIRST successful
        // parse so a later `info()` (e.g. the one inside `init()`) reuses
        // this exact value instead of re-entering the FFI. `set` only
        // fails if the cell is already populated (a benign race we can't
        // hit here — `info()` is single-threaded per node); in that case
        // the already-cached value is authoritative, so ignore the Err.
        let _ = self.info_cache.set(parsed.clone());
        Ok(parsed)
    }

    fn init(&mut self, context: NodeContext) -> TransportResult<()> {
        // `info()` now populates `info_cache` on its
        // first successful call (interior mutability via `OnceCell`), so
        // calling it here reuses the value cached at graph-build time
        // (`runtime.rs` calls `entry.info()?` before `init()`) instead of
        // making a SECOND FFI `cerulion_node_info()` call. Behavior is
        // identical when `init()` is reached without a prior `info()`:
        // the cache is empty, so this call performs the one-and-only FFI
        // read and caches it. A corrupted info JSON still fails init (and
        // therefore graph build) — failed parses are never cached.
        self.info()?;

        if self.handle.is_some() {
            return Err(TransportError::NodeError {
                node_id: self.diag_label.clone(),
                reason: "already initialized (double init)".to_string(),
            });
        }

        // Transfer ownership of NodeContext to the cdylib via raw pointer.
        // Safety: The cdylib always consumes the pointer via Box::from_raw()
        // as its first action after null check, on both success and failure paths.
        let ctx_ptr = Box::into_raw(Box::new(context));
        let handle = unsafe { (self.init_fn)(ctx_ptr) };
        if handle == 0 {
            // Pull the cdylib's
            // detailed error message instead of returning the bare
            // sentinel. Falls back to the sentinel text if the cdylib
            // didn't stash anything (shouldn't happen post-v3 but
            // defensive).
            let detail = self.take_last_error().unwrap_or_else(|| {
                "cerulion_node_init returned handle 0 (no detail provided)".to_string()
            });
            return Err(TransportError::NodeError {
                node_id: self.diag_label.clone(),
                reason: format!("init failed: {}", detail),
            });
        }
        self.handle = Some(handle);
        Ok(())
    }

    fn tick(&mut self) -> TransportResult<()> {
        let handle = self.handle.ok_or_else(|| TransportError::NodeError {
            node_id: self.diag_label.clone(),
            reason: "node not initialized".to_string(),
        })?;
        let ret = unsafe { (self.tick_fn)(handle) };
        if ret != 0 {
            // Prefer the cdylib's
            // detailed error string over the generic class label.
            // `ffi_error_description(ret)` is kept as the fallback so
            // the user still sees the class even if the cdylib failed
            // to produce a message (panic during set_last_error etc.).
            let detail = self.take_last_error().unwrap_or_else(|| {
                format!(
                    "cerulion_node_tick returned {} ({}, no detail provided)",
                    ret,
                    ffi_error_description(ret),
                )
            });
            // Replay exit-3 widening: FFI codes 2 (panic caught by the
            // cdylib's catch_unwind) and 3 (NODES mutex poisoned — the
            // poisoned-dead state a tick panic leaves behind) are
            // PANIC-CLASS failures, surfaced via the dedicated structural
            // variant so callers never have to string-match the reason.
            // Every other nonzero code (1 = user Err, 4 = handle not
            // found, …) stays the plain `NodeError` — a deterministic
            // tick Err is normal execution.
            if ret == 2 || ret == 3 {
                return Err(TransportError::NodeTickPanicked {
                    node_id: self.diag_label.clone(),
                    reason: detail,
                });
            }
            return Err(TransportError::NodeError {
                node_id: self.diag_label.clone(),
                reason: detail,
            });
        }
        Ok(())
    }

    /// ABI v7: service quiescent late joiners by
    /// re-publishing this node's history over the FFI. Best-effort
    /// liveness — NOT the firing path — so a non-zero FFI return is
    /// LOGGED (with the cdylib's detailed message, if any) and NEVER
    /// propagated or panicked on. A `None` handle (not yet `init`'d) is a
    /// silent no-op, mirroring a pre-init pump in the in-process path.
    fn pump_history(&mut self) {
        let handle = match self.handle {
            Some(h) => h,
            None => return, // not initialized yet — nothing to pump
        };
        let ret = unsafe { (self.pump_history_fn)(handle) };
        match flood_log_action(&mut self.pump_history_error_latched, ret != 0) {
            FloodLogAction::FirstFailure => {
                // Drain the cdylib's LAST_ERROR (so it can't bleed into a later
                // unrelated log) and report loudly — once per failure regime.
                let detail = self.take_last_error().unwrap_or_else(|| {
                    format!(
                        "cerulion_node_pump_history returned {} ({}, no detail provided)",
                        ret,
                        ffi_error_description(ret),
                    )
                });
                tracing::error!(
                    node = %self.diag_label,
                    code = ret,
                    error = %detail,
                    "cdylib pump_history returned non-zero (best-effort; late joiners may \
                     miss history; suppressing further to debug until it recovers)",
                );
            }
            FloodLogAction::SuppressedFailure => {
                // Still draining LAST_ERROR each time (allocator-pairing hygiene),
                // but at debug so a stuck cdylib can't flood the live loop.
                let _ = self.take_last_error();
                tracing::debug!(
                    node = %self.diag_label,
                    code = ret,
                    "cdylib pump_history still returning non-zero (suppressed)",
                );
            }
            FloodLogAction::Recovered => {
                tracing::info!(
                    node = %self.diag_label,
                    "cdylib pump_history recovered",
                );
            }
            FloodLogAction::Quiet => {}
        }
    }

    fn shutdown(&mut self) -> TransportResult<()> {
        let handle = match self.handle.take() {
            Some(h) => h,
            None => return Ok(()), // already shut down or never initialized
        };
        let ret = unsafe { (self.shutdown_fn)(handle) };
        if ret != 0 {
            // See tick() above.
            let detail = self.take_last_error().unwrap_or_else(|| {
                format!(
                    "cerulion_node_shutdown returned {} ({}, no detail provided)",
                    ret,
                    ffi_error_description(ret),
                )
            });
            return Err(TransportError::NodeError {
                node_id: self.diag_label.clone(),
                reason: detail,
            });
        }
        Ok(())
    }

    /// Forward the per-step non-trigger latest-value freeze across the
    /// FFI when this cdylib exports the optional snapshot symbols. The runtime
    /// calls this (through the node lock, at the level boundary BEFORE firing)
    /// for every cdylib node with non-trigger inputs, exactly as it does for an
    /// in-process macro node — so a held input REPLAYS its last-delivered value
    /// on a step with no fresh sample instead of the macro's nested `try_view`
    /// collapsing the whole tick to a no-op.
    ///
    /// Set-once: `inputs` are the BUILD-FIXED non-trigger names (the runtime
    /// passes the same slice every call), so they are marshalled to the cdylib
    /// exactly once (`\n`-joined) and every later call is a zero-alloc
    /// `SnapshotFns::snapshot` call (`(fns.snapshot)(handle)`).
    ///
    /// Back-compat: an older cdylib lacks the symbols (`None`) → this is the
    /// inherited no-op (its non-trigger inputs read live — today's behavior). A
    /// `None` handle (not yet `init`'d) is a silent no-op.
    fn snapshot_inputs(&mut self, inputs: &[String]) {
        // `SnapshotFns` is `Copy` (fn pointers are `Copy`), so this copies the
        // symbols out — no borrow held against the `self.take_last_error()` /
        // `self.snapshot_state` mutations below.
        let Some(fns) = self.snapshot_ffi else {
            return; // old / partial-export cdylib (no symbols) → no-op (back-compat)
        };
        let Some(handle) = self.handle else {
            return; // not initialized yet
        };
        if self.snapshot_state == SnapshotState::Pending {
            // Marshal the build-fixed names exactly ONCE — only while `Pending`
            // (the first call: the `Pending→{Active|Failed}` edge, so the marshal
            // also happens when the set FAILS); every later step is the zero-alloc
            // `snapshot` call below.
            let joined = inputs.join("\n");
            // Safety: `joined` is a live byte slice valid for this call; the
            // cdylib copies it and never retains the pointer. `as_ptr()` is
            // non-null and aligned even for an empty string (len 0 is sound).
            let ret = unsafe { (fns.set)(handle, joined.as_ptr(), joined.len()) };
            self.snapshot_state = self.snapshot_state.after_set(ret);
            if ret != 0 {
                let detail = self.take_last_error().unwrap_or_else(|| {
                    format!("cerulion_node_set_snapshot_inputs returned {ret} (no detail provided)")
                });
                // Do not latch as success and keep calling
                // `snapshot` (which would freeze nothing yet return 0, silently
                // masking the lost hold — the exact earlier throttle bug).
                // `after_set` moved us to `Failed`, so `snapshot` is SKIPPED below
                // and the node runs UNHELD — loudly, exactly once.
                tracing::error!(
                    node = %self.diag_label,
                    code = ret,
                    error = %detail,
                    "cdylib set_snapshot_inputs failed; non-trigger inputs will NOT be held \
                     for the remainder of this run (hold disabled, not silently masked)"
                );
            }
        }
        if !self.snapshot_state.should_invoke_snapshot() {
            // `Failed` (set never succeeded) → skip the snapshot FFI call entirely
            // rather than report a phantom-healthy 0 while holding nothing.
            return;
        }
        // Safety: `handle` is a live registered node handle.
        let ret = unsafe { (fns.snapshot)(handle) };
        // Flood-control the per-step failure log exactly like `pump_history`: a
        // PERSISTENT snapshot failure (e.g. a mid-run `NODES` poison after a
        // successful `set`) fires every firing step — 870×/s in the WBC
        // benchmark — so latch to one loud `error!` per failure regime, `debug!`
        // while it persists, `info!` on recovery.
        match flood_log_action(&mut self.snapshot_error_latched, ret != 0) {
            FloodLogAction::FirstFailure => {
                // Drain LAST_ERROR (so it can't bleed into a later unrelated log)
                // and report loudly — once per failure regime.
                let detail = self.take_last_error().unwrap_or_else(|| {
                    format!("cerulion_node_snapshot_inputs returned {ret} (no detail provided)")
                });
                tracing::error!(
                    node = %self.diag_label,
                    code = ret,
                    error = %detail,
                    "cdylib snapshot_inputs FFI call failed (suppressing further to debug \
                     until it recovers)"
                );
            }
            FloodLogAction::SuppressedFailure => {
                // Still drain LAST_ERROR each time (allocator-pairing hygiene), at
                // debug so a stuck cdylib can't flood the live loop.
                let _ = self.take_last_error();
                tracing::debug!(
                    node = %self.diag_label,
                    code = ret,
                    "cdylib snapshot_inputs still failing (suppressed)"
                );
            }
            FloodLogAction::Recovered => {
                tracing::info!(
                    node = %self.diag_label,
                    "cdylib snapshot_inputs recovered"
                );
            }
            FloodLogAction::Quiet => {}
        }
    }

    /// This cdylib's `CerulionState::STATE_SHAPE`, read once at
    /// load. `None` for a raw-FFI cdylib or an older build — one that exports no state
    /// symbols genuinely declares no restorable state.
    fn state_shape(&self) -> Option<u64> {
        self.state_shape
    }

    /// This cdylib's `CerulionState::INLINE_SAFE`, read
    /// once at load.
    ///
    /// A cdylib that exports no `cerulion_node_inline_safe` reports `false` and
    /// takes the FORK carrier — the same answer the trait's own default gives,
    /// and the safe one: both carriers run the same generated encoder and emit
    /// identical bytes, so being wrong here costs a `fork(2)` per cadence and
    /// never a wrong capture. That is why the symbol is ADDITIVE and no ABI
    /// version moved for it.
    fn inline_safe(&self) -> bool {
        self.inline_safe
    }

    /// Ask this cdylib whether any lock in its node's
    /// DECLARED state is held right now.
    ///
    /// # Every failure answers "not quiescent", and that direction is the point
    ///
    /// An un-`init`'d node (no handle), a cdylib that exports no probe but whose
    /// `NODES` cannot be reached, a probe that returns an error — all of them
    /// mean this process cannot show the node is safe to fork from, and the cost
    /// of saying so is ONE skipped anchor. The cost of the other answer is a
    /// child that inherits a held lock: a fork child has exactly one thread, so
    /// nothing can ever release it, and the child wedges until the watchdog
    /// SIGKILLs it five seconds later — reported as a stalled encoder that never
    /// ran.
    ///
    /// The one exception is an ABSENT symbol, which answers the trait default
    /// `true`. A raw-FFI cdylib, or one built without the symbol, declares no state graph at all, so
    /// there is nothing for a probe to be true OF, and refusing on its behalf
    /// would skip every anchor of every OTHER node in the process for the life of
    /// the run — an anchor-wide refusal earned by a node that contributes none.
    /// The residual is the design's documented one: such a node is covered by the
    /// child's progress watchdog rather than by the probe.
    fn cer_probe(&self) -> bool {
        let Some(probe) = self.cer_probe_ffi else {
            return true;
        };
        let Some(handle) = self.handle else {
            return false;
        };
        // SAFETY: the symbol was resolved by name and signature at load, and
        // `handle` is this entry's own, minted by `cerulion_node_new`.
        unsafe { probe(handle) != 0 }
    }

    /// Encode this cdylib node's state through `out`.
    ///
    /// The encode runs INSIDE the library, on its own data, through the same
    /// generated `cer_capture` the in-process form runs — so the bytes are
    /// identical and which form produced an anchor is invisible in the bag.
    ///
    /// An UNCOVERED cdylib REFUSES here rather than reporting an empty anchor:
    /// an anchor that silently captured nothing is worse than no anchor at all,
    /// because a resumed run would restore that emptiness over real state and
    /// then report a confident divergence about an execution that never
    /// happened.
    fn capture_state(&self, out: &mut dyn crate::state::StateSink) -> TransportResult<()> {
        let Some(fns) = self.state_ffi else {
            return Err(TransportError::GraphError {
                reason: format!(
                    "node '{}' cannot capture state: this cdylib exports no \
                     `cerulion_node_capture_state`, so it declares no capturable state. \
                     Fix: build the node with `#[cerulion_node]`, which emits the state FFI \
                     for every node it generates; a hand-written raw-FFI cdylib must export \
                     the three `cerulion_node_{{state_shape,capture_state,restore_state}}` \
                     symbols itself.",
                    self.diag_label
                ),
            });
        };
        let Some(handle) = self.handle else {
            return Err(TransportError::GraphError {
                reason: format!(
                    "node '{}' cannot capture state: it has not been \
                     initialized, so the cdylib holds no instance to encode",
                    self.diag_label
                ),
            });
        };
        // The caller's bound travels as a scalar because a callback ABI has
        // nowhere else to put it, and without it a hash-like container inside
        // the cdylib would build its whole sort index before discovering the
        // sink cannot take it. `u64::MAX` mirrors `remaining_hint`'s
        // `None`.
        let capacity_hint = out.remaining_hint().map(|n| n as u64).unwrap_or(u64::MAX);
        let mut bridge = StateSinkBridge {
            // Reborrowed, not moved: the host has to reach its own sink again
            // after the call to latch it (below).
            sink: &mut *out,
            full: false,
            panicked: false,
        };
        let user = (&mut bridge) as *mut StateSinkBridge as *mut std::ffi::c_void;
        // Safety: `handle` is a live registered node handle; the trampoline and
        // `user` are valid for exactly this call and the cdylib never retains
        // either.
        let ret =
            unsafe { (fns.capture)(handle, Some(state_sink_trampoline), user, capacity_hint) };
        let (full, panicked) = (bridge.full, bridge.panicked);

        // THE HOST'S OWN OBSERVATION OUTRANKS THE CDYLIB'S RETURN CODE.
        //
        // `ret` is a CLAIM. The three state symbols are ADDITIVE and resolved
        // BY NAME, so nothing here can know the exporter behind them came out
        // of `#[cerulion_node]` — a hand-written raw-FFI one is free to ignore
        // the callback's non-zero return, keep streaming, and still report 0.
        // Believing that 0 would accept whatever PREFIX this sink managed to
        // take as the node's whole state, and a truncated anchor is not a loud
        // failure: a later run restores it as if it were complete, so the
        // resumed node runs on state nobody captured while the run reports it
        // `restored` (Principle #6, and Principle #13 at the other end).
        //
        // The bridge is the only witness that cannot be forged: it is THIS
        // process's own record of what its own sink did.
        if ret == 0 && !full && !panicked {
            return Ok(());
        }

        // Anything the host's sink REFUSED is charged to the host's sink. A
        // refusal that rode `remaining_hint` (a hash-like container
        // refuses before building its sort index, having called `write` zero
        // times) never reaches the trampoline, so nothing else can latch it,
        // and an unlatched `BoundedSink` reports `consumed()` as bytes-written
        // — usually zero — handing the next node a budget this one already
        // spent. A panic or an encoder error is deliberately NOT latched here:
        // the in-process `walk_inline` charges those bytes-written too, and the
        // two forms must classify identically.
        //
        // `&& !panicked` is what makes the sentence above TRUE of the code, and
        // it is load-bearing rather than defensive. The generated
        // `__CerFfiStateSink::write` maps EVERY non-zero callback return onto
        // `SinkFull`, panic included, so a cdylib reports -6 — "your sink
        // refused" — for a sink that refused nothing and merely unwound. Only
        // `bridge.panicked` can tell the two apart, which is the whole reason
        // `StateSinkBridge` carries both flags: exhausted and BROKEN are
        // opposite diagnoses, and `refuse()` asserts the first. Latching on a
        // panic charges the boundary budget a sink's entire capacity for bytes
        // nobody consumed, and presents a broken caller sink to the carrier as
        // an arena too small to retry into.
        //
        // The genuinely-full case is untouched: the trampoline sets `full`
        // itself, so a refusal it observed latches through the first disjunct
        // no matter what the exporter returned, and a hint refusal — which
        // calls `write` zero times, so neither flag is set — still latches on
        // the code alone.
        if full || (ret == STATE_FFI_HOST_SINK_REFUSED && !panicked) {
            out.refuse();
        }

        if ret == 0 {
            // The exporter is broken, and that is the headline: this is not a
            // too-small buffer to retry with and not a broken node. Whatever it
            // left in its error slot belongs to THIS call, so it is DRAINED —
            // an undrained message resurfaces against a later, unrelated
            // failure — and SURFACED rather than thrown away, because it is the
            // only thing the cdylib had to say about a capture it got wrong.
            let said = self
                .take_last_error()
                .map(|d| format!(" The cdylib's error slot held: {d}."))
                .unwrap_or_default();
            let condition = if panicked {
                "the host's sink PANICKED while accepting a chunk"
            } else {
                "the host's sink REFUSED a chunk"
            };
            tracing::error!(
                node = %self.diag_label,
                sink_refused = full,
                sink_panicked = panicked,
                "cdylib cerulion_node_capture_state reported success after the host's sink \
                 rejected a chunk; REFUSING the capture — this cdylib's state exporter is \
                 broken and would otherwise have recorded a truncated anchor as whole state"
            );
            return Err(TransportError::GraphError {
                reason: format!(
                    "node '{}' state capture REFUSED: {condition}, yet its \
                     `cerulion_node_capture_state` returned 0 (success). Only a PREFIX of \
                     this node's state reached the sink, and accepting it would record a \
                     truncated anchor that a later run restores as if it were complete.\
                     {said} This cdylib's state exporter is BROKEN: it must stop at the \
                     sink callback's first non-zero return and report \
                     {STATE_FFI_HOST_SINK_REFUSED}. Fix: build the node with \
                     `#[cerulion_node]`, whose generated exporter does exactly that; a \
                     hand-written raw-FFI exporter must propagate the sink's refusal itself.",
                    self.diag_label
                ),
            });
        }

        let detail = self
            .take_last_error()
            .unwrap_or_else(|| format!("cerulion_node_capture_state returned {ret}"));
        let cause = if panicked {
            "the host sink panicked while accepting a chunk"
        } else if full || ret == STATE_FFI_HOST_SINK_REFUSED {
            "the host sink refused the bytes (its buffer is too small for this node's state)"
        } else {
            "the cdylib could not encode this node's state"
        };
        Err(TransportError::GraphError {
            reason: format!(
                "node '{}' state capture failed (code {ret}): {cause} — {detail}",
                self.diag_label
            ),
        })
    }

    /// Apply a recorded anchor's payload to this cdylib node.
    ///
    /// **THE REFUSE-ON-UNCOVERED PATH.** A cdylib without the exports — every
    /// hand-written raw-FFI node, every build older than the exports — cannot take the bytes,
    /// and the only two things it may do are refuse or lie. It refuses, naming
    /// the node and the fix. Returning `Ok(())` here would leave the node at
    /// its constructor's `Default` while the runtime recorded it as `restored`,
    /// which is fabricated state (Principle #13) and produces a divergence
    /// report about an execution that never happened.
    ///
    /// The refusal is reachable in two ways and they are complementary rather
    /// than redundant: `state_shape()` is `None` for such a node, so
    /// `GraphRuntime::restore_node_states` REPORTS it under `declared_no_state`
    /// (the surface `--strict-state` escalates) instead of applying anything;
    /// and any caller that reaches `restore_state` directly gets this error.
    fn restore_state(&mut self, payload: &[u8]) -> TransportResult<()> {
        let Some(fns) = self.state_ffi else {
            return Err(TransportError::GraphError {
                reason: format!(
                    "node '{}' declares no restorable state: this cdylib \
                     exports no `cerulion_node_restore_state`, so a recorded anchor cannot \
                     be applied to it and it would resume from its constructor's defaults. \
                     Fix: build the node with `#[cerulion_node]`, which emits the state FFI \
                     for every node it generates; a hand-written raw-FFI cdylib must export \
                     the three `cerulion_node_{{state_shape,capture_state,restore_state}}` \
                     symbols itself.",
                    self.diag_label
                ),
            });
        };
        let Some(handle) = self.handle else {
            return Err(TransportError::GraphError {
                reason: format!(
                    "node '{}' cannot restore state: it has not been \
                     initialized, so the cdylib holds no instance to restore into. The \
                     runtime applies anchors AFTER `init()` and before the first `tick()` \
                     (§3.5)",
                    self.diag_label
                ),
            });
        };
        // Safety: `handle` is a live registered node handle; `payload` is a
        // live borrowed slice valid for this call which the cdylib only reads.
        // `as_ptr()` is non-null and aligned even for an empty payload, and the
        // cdylib guards a zero length before building a slice.
        let ret = unsafe { (fns.restore)(handle, payload.as_ptr(), payload.len()) };
        if ret == 0 {
            return Ok(());
        }
        let detail = self
            .take_last_error()
            .unwrap_or_else(|| format!("cerulion_node_restore_state returned {ret}"));
        Err(TransportError::GraphError {
            reason: format!(
                "node '{}' state restore failed (code {ret}): {detail}. The node may be \
                 PARTIALLY restored — the run must fail rather than continue (§2.8)",
                self.diag_label
            ),
        })
    }

    /// A cdylib HOLDS its non-trigger inputs across steps iff it exports
    /// BOTH optional snapshot symbols (so `snapshot_inputs` really forwards the
    /// freeze). The host provisions such source topics at the (`pub(crate)`)
    /// `SUBSCRIBER_MAX_BORROWED_HELD` ceiling. `performs_input_snapshot` stays
    /// `false` (cdylib fires serially — not rayon-eligible); see that method.
    fn holds_input_snapshot(&self) -> bool {
        // Both-or-neither already decided in `load` → a single `is_some()`.
        self.snapshot_ffi.is_some()
    }

    /// Forward the level-boundary unified trigger drain
    /// across the FFI. The runtime calls this (through the node lock, in
    /// `drain_level` BEFORE decide) for a `DrainSource::Unified` cdylib
    /// binding; the cdylib-side export drains the BODY subscriber once
    /// (freezing the surviving sample for the tick's later generated
    /// `try_view`) and reports `(popped, latest_ts)` back through out-params.
    ///
    /// FAIL-SAFE: every nonzero FFI code maps to the safe `(0, None)` — a
    /// failed drain must NEVER fabricate a fire signal (a phantom `popped > 0`
    /// would fire the node against a queue that was not drained). Returning
    /// `(0, None)` means "nothing arrived", so the node simply does not fire
    /// this step; queued frames stay in the body subscriber for the next
    /// drain (no data loss — at worst iceoryx2's `drop_oldest` reclaim).
    /// Failure logging is flood-latched like the `snapshot_inputs` twin
    /// (first loud `error!`, then `debug!` until recovery) — this runs every
    /// level pass.
    fn drain_trigger_input(&mut self, input_name: &str) -> (u64, Option<u64>) {
        let Some(drain_fn) = self.drain_trigger_ffi else {
            return (0, None); // no symbol → capability false → never Unified-wired
        };
        let Some(handle) = self.handle else {
            return (0, None); // not initialized yet
        };
        let mut popped: u64 = 0;
        let mut latest_ts: u64 = 0;
        let mut has_ts: i32 = 0;
        // Safety: `handle` is a live registered node handle; the name pointer
        // is a live borrowed `&str` valid for this call (the cdylib only reads
        // it, never retains it); the out-pointers are our own stack slots.
        let ret = unsafe {
            drain_fn(
                handle,
                input_name.as_ptr(),
                input_name.len(),
                &mut popped,
                &mut latest_ts,
                &mut has_ts,
            )
        };
        match flood_log_action(&mut self.drain_trigger_error_latched, ret != 0) {
            FloodLogAction::FirstFailure => {
                let detail = self.take_last_error().unwrap_or_else(|| {
                    format!("cerulion_node_drain_trigger_input returned {ret} (no detail provided)")
                });
                tracing::error!(
                    node = %self.diag_label,
                    input = %input_name,
                    code = ret,
                    error = %detail,
                    "cdylib drain_trigger_input FFI call failed; unified trigger drain \
                     skipped this step — no fire signal fabricated (suppressing further \
                     to debug until it recovers)"
                );
            }
            FloodLogAction::SuppressedFailure => {
                // Drain LAST_ERROR each time (allocator-pairing hygiene), quietly.
                let _ = self.take_last_error();
                tracing::debug!(
                    node = %self.diag_label,
                    input = %input_name,
                    code = ret,
                    "cdylib drain_trigger_input still failing (suppressed)"
                );
            }
            FloodLogAction::Recovered => {
                tracing::info!(
                    node = %self.diag_label,
                    input = %input_name,
                    "cdylib drain_trigger_input recovered"
                );
            }
            FloodLogAction::Quiet => {}
        }
        if ret != 0 {
            return (0, None);
        }
        (popped, if has_ts != 0 { Some(latest_ts) } else { None })
    }

    /// Symbol presence IS the capability — only
    /// macro-generated cdylibs export `cerulion_node_drain_trigger_input`, and
    /// their input reads are generated `try_view` (the one read path the
    /// unified drain's frozen slot serves), so this gate structurally enforces
    /// the trait's READ-PATH CONTRACT for the production surface. A raw-FFI /
    /// older cdylib lacks the symbol → `false` → the binding stays
    /// `DrainSource::Separate` (today's behavior, byte-identical). Static per
    /// entry (resolved at `load`, before the runtime's pre-init capability-map
    /// capture).
    fn unifies_trigger_drain(&self) -> bool {
        self.drain_trigger_ffi.is_some()
    }

    /// Forward the Data burst loop's between-fires REFILL across the
    /// OPTIONAL `cerulion_node_refill_trigger_input` export — the boundary
    /// drain's twin, differing only in that an unserved frozen head answers
    /// "nothing new" instead of being re-offered.
    ///
    /// FAIL-SAFE, and doubly so here: every nonzero FFI code AND an absent
    /// symbol map to `(0, None)`, which the burst loop reads as an empty queue
    /// and stops on. A refill can therefore only ever cost THROUGHPUT (frames
    /// served on later steps instead of this one) — never a fabricated fire,
    /// never a lost frame (the queue keeps them for the next boundary drain).
    /// Failure logging rides its OWN flood latch, for the reason the field's
    /// doc gives.
    fn refill_trigger_input(&mut self, input_name: &str) -> (u64, Option<u64>) {
        let Some(refill_fn) = self.refill_trigger_ffi else {
            // No symbol → capability false → the hook is not even installed.
            return (0, None);
        };
        let Some(handle) = self.handle else {
            return (0, None); // not initialized yet
        };
        let mut popped: u64 = 0;
        let mut latest_ts: u64 = 0;
        let mut has_ts: i32 = 0;
        // Safety: identical contract to `drain_trigger_input` above — a live
        // registered handle, a borrowed `&str` valid for this call, and our own
        // stack slots as out-pointers.
        let ret = unsafe {
            refill_fn(
                handle,
                input_name.as_ptr(),
                input_name.len(),
                &mut popped,
                &mut latest_ts,
                &mut has_ts,
            )
        };
        match flood_log_action(&mut self.refill_trigger_error_latched, ret != 0) {
            FloodLogAction::FirstFailure => {
                let detail = self.take_last_error().unwrap_or_else(|| {
                    format!(
                        "cerulion_node_refill_trigger_input returned {ret} (no detail provided)"
                    )
                });
                tracing::error!(
                    node = %self.diag_label,
                    input = %input_name,
                    code = ret,
                    error = %detail,
                    "cdylib refill_trigger_input FFI call failed; the data burst ends \
                     here — no fire signal fabricated, queued frames served by later \
                     steps (suppressing further to debug until it recovers)"
                );
            }
            FloodLogAction::SuppressedFailure => {
                // Drain LAST_ERROR each time (allocator-pairing hygiene), quietly.
                let _ = self.take_last_error();
                tracing::debug!(
                    node = %self.diag_label,
                    input = %input_name,
                    code = ret,
                    "cdylib refill_trigger_input still failing (suppressed)"
                );
            }
            FloodLogAction::Recovered => {
                tracing::info!(
                    node = %self.diag_label,
                    input = %input_name,
                    "cdylib refill_trigger_input recovered"
                );
            }
            FloodLogAction::Quiet => {}
        }
        if ret != 0 {
            return (0, None);
        }
        (popped, if has_ts != 0 { Some(latest_ts) } else { None })
    }

    /// Symbol presence IS the capability, exactly as for the drain.
    /// Resolved INDEPENDENTLY of `unifies_trigger_drain`, because an
    /// older macro cdylib exports the drain and not the refill: it stays
    /// Unified (its hops keep the unified drain's one receive) and its bursts are served
    /// one frame per step until it is rebuilt.
    fn refills_trigger_input(&self) -> bool {
        self.refill_trigger_ffi.is_some()
    }

    /// Perform one per-set Sync head op on this cdylib node.
    ///
    /// The two FILL ops deliberately do NOT cross the new symbol: PROMOTION and
    /// the head fill ride the EXISTING drain entry points
    /// (`cerulion_node_{drain,refill}_trigger_input`), which already exist,
    /// already carry the boundary-vs-refill distinction the fills turn on, and
    /// already freeze the surviving sample for the tick's generated `try_view`.
    /// So the new symbol carries only the four ops that have no existing home.
    ///
    /// A `(0, None)` from a FAILING drain FFI is indistinguishable from an
    /// empty queue here — that is the pre-existing drain contract
    /// ([`Self::drain_trigger_input`] maps every nonzero code to the safe
    /// `(0, None)`, and logs it on its own latch), and it degrades to
    /// [`SyncOpAnswer::Nothing`], i.e. "no head": descent-refusing, never a
    /// fabricated set member.
    ///
    /// FAIL-SAFE on the symbol path too: an ABSENT symbol, an unset handle, a
    /// nonzero return and an UNKNOWN answer kind all map to
    /// [`SyncOpAnswer::Failed`], which the align driver's R-Fail policy reads as
    /// the descent-DISABLING answer at every site. The absent-symbol arm is
    /// unreachable by construction — [`Self::supports_sync_head_ops`] reports
    /// `false` without it, so the graph build never installs the ops — and it is
    /// answered rather than asserted for the same reason every other arm here is.
    fn sync_head_op(&mut self, input_name: &str, op: SyncHeadOp) -> SyncOpAnswer {
        let op_code = match op {
            SyncHeadOp::FillBoundary => {
                let (popped, latest_ts) = self.drain_trigger_input(input_name);
                return sync_fill_answer(popped, latest_ts);
            }
            SyncHeadOp::FillRefill => {
                let (popped, latest_ts) = self.refill_trigger_input(input_name);
                return sync_fill_answer(popped, latest_ts);
            }
            SyncHeadOp::ProbeNext => SYNC_HEAD_OP_PROBE_NEXT,
            SyncHeadOp::PeekNext => SYNC_HEAD_OP_PEEK_NEXT,
            SyncHeadOp::Advance => SYNC_HEAD_OP_ADVANCE,
            SyncHeadOp::Void => SYNC_HEAD_OP_VOID,
        };
        let Some(sync_fn) = self.sync_head_op_ffi else {
            return SyncOpAnswer::Failed; // no symbol → capability false → never installed
        };
        let Some(handle) = self.handle else {
            return SyncOpAnswer::Failed; // not initialized yet
        };
        let mut out_ts: u64 = 0;
        let mut out_kind: i32 = -1;
        // Safety: identical contract to `drain_trigger_input` above — a live
        // registered handle, a borrowed `&str` valid for this call (the cdylib
        // only reads it, never retains it), and our own stack slots as
        // out-pointers.
        let ret = unsafe {
            sync_fn(
                handle,
                input_name.as_ptr(),
                input_name.len(),
                op_code,
                &mut out_ts,
                &mut out_kind,
            )
        };
        // Decode BEFORE reporting. A nonzero return and an unknown answer kind
        // are two ways for ONE call to produce no usable answer, with one
        // remedy (rebuild/redeploy the cdylib), so they share ONE regime — the
        // flood-latch rule is per CALL SITE, exactly as the drain and refill
        // latches are, and the loud head names which of the two it was.
        let decoded = if ret == 0 {
            match out_kind {
                SYNC_OP_ANSWER_NOTHING => Some(SyncOpAnswer::Nothing),
                SYNC_OP_ANSWER_PRESENT => Some(SyncOpAnswer::Present),
                SYNC_OP_ANSWER_HEAD => Some(SyncOpAnswer::Head(out_ts)),
                SYNC_OP_ANSWER_STAMP => Some(SyncOpAnswer::Stamp(out_ts)),
                // A cdylib speaking an answer kind this host does not know is a
                // VERSION SKEW. Guessing would fabricate a set member, so it is
                // a failure like any other.
                _ => None,
            }
        } else {
            None
        };
        match flood_log_action(&mut self.sync_head_op_error_latched, decoded.is_none()) {
            FloodLogAction::FirstFailure => {
                let detail = self.take_last_error().unwrap_or_else(|| {
                    format!(
                        "cerulion_node_sync_head_op returned code {ret}, kind {out_kind} \
                         (no detail provided)"
                    )
                });
                if ret != 0 {
                    tracing::error!(
                        node = %self.diag_label,
                        input = %input_name,
                        op = ?op,
                        code = ret,
                        error = %detail,
                        "cdylib sync_head_op FFI call failed; the per-set Sync alignment \
                         pass ends here — no set member fabricated (suppressing further \
                         to debug until it recovers)"
                    );
                } else {
                    tracing::error!(
                        node = %self.diag_label,
                        input = %input_name,
                        op = ?op,
                        kind = out_kind,
                        error = %detail,
                        "cdylib sync_head_op answered an UNKNOWN kind — a version skew \
                         between this host and the node cdylib. Refusing to guess (a guess \
                         would fabricate a set member); rebuild the node against this \
                         Cerulion (suppressing further to debug until it recovers)"
                    );
                }
            }
            FloodLogAction::SuppressedFailure => {
                // Drain LAST_ERROR each time (allocator-pairing hygiene), quietly.
                let _ = self.take_last_error();
                tracing::debug!(
                    node = %self.diag_label,
                    input = %input_name,
                    op = ?op,
                    code = ret,
                    kind = out_kind,
                    "cdylib sync_head_op still failing (suppressed)"
                );
            }
            FloodLogAction::Recovered => {
                tracing::info!(
                    node = %self.diag_label,
                    input = %input_name,
                    "cdylib sync_head_op recovered"
                );
            }
            FloodLogAction::Quiet => {}
        }
        decoded.unwrap_or(SyncOpAnswer::Failed)
    }

    /// Symbol presence IS the capability, exactly as for the drain
    /// and the refill. Resolved INDEPENDENTLY of both, because an older
    /// macro cdylib exports those and not this one: its Sync node keeps the
    /// legacy latest-per-set semantics until it is rebuilt, and the graph build
    /// says so LOUDLY rather than installing ops every one of whose questions
    /// would answer `Failed`. Static per entry (resolved at `load`, before the
    /// runtime's pre-init capability-map capture).
    fn supports_sync_head_ops(&self) -> bool {
        self.sync_head_op_ffi.is_some()
    }

    /// Query the cdylib's OPTIONAL external-source FFI export and
    /// map its kind code to an [`ExternalSource`]. Called ONCE by the runtime's
    /// `collect_external_sources` at `run_live` entry (after [`Self::init`]).
    ///
    /// - No symbol (`None`) → `None` (non-external node / older cdylib:
    ///   back-compat, no behavior change).
    /// - Not yet `init`'d (`handle == None`) → `None` (no handle to query).
    /// - [`EXTERNAL_SOURCE_KIND_DEVICE_FD`] → `Some(Fd(fd))` (poll-only device fd).
    /// - [`EXTERNAL_SOURCE_KIND_DOORBELL_FD`] → `Some(Fd(fd))` AND latches
    ///   `external_is_doorbell_fd` so [`Self::external_source_is_drained_doorbell_fd`]
    ///   tells the runtime to DRAIN this fd (a tier-2 `Blocking` collapse pipe).
    /// - [`EXTERNAL_SOURCE_KIND_HOST_DRIVEN`] → `Some(HostDriven)`.
    /// - [`EXTERNAL_SOURCE_KIND_ERROR`] / any other code → `None` (logged loudly).
    fn external_source(&mut self) -> Option<ExternalSource> {
        let external_source_fn = self.external_source_fn?;
        let Some(handle) = self.handle else {
            // Make the pre-init query attributable here — the
            // collect-side generic warn would otherwise be the only (misattributed)
            // signal that this node had no source only because it wasn't init'd yet.
            tracing::debug!(
                node = %self.diag_label,
                "external_source queried before init; no handle yet"
            );
            return None; // not initialized yet — nothing to query
        };
        let mut out_fd: i64 = -1;
        // Safety: `handle` is a live registered node handle; `&mut out_fd` is a
        // valid, aligned i64 slot the FFI writes only for the Fd/DoorbellFd codes.
        let kind = unsafe { external_source_fn(handle, &mut out_fd as *mut i64) };
        match kind {
            EXTERNAL_SOURCE_KIND_DEVICE_FD => {
                self.external_is_doorbell_fd = false;
                Some(ExternalSource::Fd(out_fd as std::os::unix::io::RawFd))
            }
            EXTERNAL_SOURCE_KIND_DOORBELL_FD => {
                self.external_is_doorbell_fd = true;
                // The cdylib just spawned a detached helper thread running its
                // own code — from here on, dlclosing this library is a
                // use-after-unmap hazard. Leak the handle at Drop instead (see
                // the `leak_library_on_drop` field docs).
                self.leak_library_on_drop = true;
                Some(ExternalSource::Fd(out_fd as std::os::unix::io::RawFd))
            }
            EXTERNAL_SOURCE_KIND_HOST_DRIVEN => {
                self.external_is_doorbell_fd = false;
                Some(ExternalSource::HostDriven)
            }
            other => {
                // EXTERNAL_SOURCE_KIND_ERROR (or an unknown future code): drain
                // LAST_ERROR (allocator-pairing hygiene) and report — the node
                // gets no external source (stays inert), never silently mis-bound.
                let detail = self.take_last_error().unwrap_or_else(|| {
                    format!("cerulion_node_external_source returned {other} (no detail provided)")
                });
                tracing::error!(
                    node = %self.diag_label,
                    code = other,
                    error = %detail,
                    "cdylib external_source FFI call failed or returned an unknown kind; \
                     node has no external source (stays inert)"
                );
                self.external_is_doorbell_fd = false;
                None
            }
        }
    }

    /// Report whether the [`ExternalSource::Fd`] last returned by
    /// [`Self::external_source`] is a tier-2 `Blocking`-collapse doorbell pipe
    /// (kind [`EXTERNAL_SOURCE_KIND_DOORBELL_FD`]) — see the trait method docs.
    fn external_source_is_drained_doorbell_fd(&self) -> bool {
        self.external_is_doorbell_fd
    }
}

impl Drop for DylibNodeEntry {
    /// Log any cdylib shutdown error
    /// instead of silently discarding the FFI return code. With
    /// `GraphRuntime` calling `NodeEntry::shutdown()` explicitly,
    /// this Drop path is mostly a belt-and-
    /// suspenders for direct DylibNodeEntry consumers — but when it
    /// does run after a missed explicit shutdown, errors used to vanish
    /// entirely. Pull `LAST_ERROR` and surface via tracing::error so
    /// debug info from the cdylib doesn't
    /// disappear.
    fn drop(&mut self) {
        if let Some(handle) = self.handle.take() {
            // Safety: handle was obtained from the cdylib's
            // `cerulion_node_init`; the FFI fn is part of the same
            // library whose lifetime this struct extends via _library.
            let ret = unsafe { (self.shutdown_fn)(handle) };
            if ret != 0 {
                let detail = self.take_last_error().unwrap_or_else(|| {
                    format!(
                        "cerulion_node_shutdown returned {} ({}, no detail provided)",
                        ret,
                        ffi_error_description(ret),
                    )
                });
                tracing::error!(
                    node = %self.diag_label,
                    code = ret,
                    error = %detail,
                    "cdylib shutdown returned non-zero during Drop",
                );
            }
        }
        // If this cdylib spawned a detached Blocking doorbell
        // helper (kind DOORBELL_FD), its thread executes cdylib code that
        // cannot be joined — dlclosing would be a use-after-unmap. Leak the handle instead
        // (see the `leak_library_on_drop` field docs for why glibc's NODELETE
        // behavior is not a portable substitute).
        if self.leak_library_on_drop {
            if let Some(library) = self._library.take() {
                std::mem::forget(library);
                tracing::debug!(
                    node = %self.diag_label,
                    "leaking cdylib library handle: a detached Blocking doorbell \
                     helper thread may still execute its code"
                );
            }
        }
    }
}

/// The logging action for one repeatedly-called
/// `DylibNodeEntry` FFI result (`pump_history` and `snapshot_inputs`),
/// given the prior latch state. Pure + total so it is unit-tested without a
/// failing-cdylib fixture. See [`flood_log_action`].
///
/// Also reused by `GraphRuntime::sweep_external_sources` to flood-limit
/// its per-live-step external `poll(2)` / `trigger_external` degrade logs (the
/// live loop runs at ~kHz, so a per-cycle `error!` would flood the journal).
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum FloodLogAction {
    /// First failure in a regime → log loudly (`error!`) and latch.
    FirstFailure,
    /// Continuing failure while latched → log quietly (`debug!`) to avoid
    /// flooding the live loop.
    SuppressedFailure,
    /// First success after a failure regime → clear the latch, log recovery
    /// (`info!`).
    Recovered,
    /// Success with no prior failure → log nothing.
    Quiet,
}

/// Flood-control state machine for a per-iteration `DylibNodeEntry` FFI log
/// (`pump_history` every live-loop iteration; `snapshot_inputs` every
/// firing step). A PERSISTENT failure (e.g. a poisoned cdylib `NODES` mutex)
/// must not re-log at `error!` on every call. Mirrors the runtime's
/// `signal_sync_input` latch: first failure loud, subsequent quiet, success
/// clears it. Mutates `latched` in place and returns the action.
pub(crate) fn flood_log_action(latched: &mut bool, failed: bool) -> FloodLogAction {
    match (failed, *latched) {
        (true, false) => {
            *latched = true;
            FloodLogAction::FirstFailure
        }
        (true, true) => FloodLogAction::SuppressedFailure,
        (false, true) => {
            *latched = false;
            FloodLogAction::Recovered
        }
        (false, false) => FloodLogAction::Quiet,
    }
}

/// Decode an FFI error code into a human-readable description.
///
/// Error codes follow the convention established by `#[cerulion_node]` codegen:
/// - 0: success
/// - 1: logic error (init/tick/shutdown returned `Err`)
/// - 2: panic (caught by `catch_unwind`)
/// - 3: mutex poisoned
fn ffi_error_description(code: i32) -> &'static str {
    match code {
        1 => "logic error",
        2 => "panic",
        3 => "mutex poisoned",
        4 => "handle not found",
        _ => "unknown",
    }
}

/// Lossily decode a C string from the cdylib's
/// LAST_ERROR buffer. Invalid UTF-8 sequences become U+FFFD rather than
/// dropping the entire message (which would make
/// `take_last_error` return `None` and the upstream caller fall
/// back to a generic "no detail provided" string).
///
/// Returns an owned String because the caller frees the source pointer
/// immediately after, so a borrowing `Cow` would dangle.
fn cstr_to_lossy_string(c_str: &CStr) -> String {
    String::from_utf8_lossy(c_str.to_bytes()).into_owned()
}

/// The body of [`DylibNodeEntry::take_last_error`], taking the two symbols
/// rather than a constructed `self`.
///
/// It is split out because `DylibNodeEntry::load` must drain `LAST_ERROR` too
/// — the load-time `cerulion_node_state_shape` failure disables state capture,
/// and the exporter's reason for refusing is only readable there — and at that
/// point `self` does not exist yet. Copying the eight lines instead would put a
/// second `into_raw`/free pairing in the tree for the same slot, which is the
/// kind of duplication the allocator-pairing contract cannot afford.
///
/// Invalid UTF-8 becomes U+FFFD rather than dropping the
/// whole message, so a mangled diagnostic is still served instead of the
/// caller's generic "no detail provided" fallback. Returns `None` only when the
/// cdylib reports no error (null pointer).
fn take_last_error_via(
    take: unsafe extern "C" fn() -> *mut std::ffi::c_char,
    free: unsafe extern "C" fn(*mut std::ffi::c_char),
) -> Option<String> {
    // Safety: `take` returns either null or a pointer produced by
    // `CString::into_raw` inside the cdylib. Null is never dereferenced, and
    // the pointer goes back to the cdylib's own free function, satisfying the
    // allocator-pairing contract.
    let raw = unsafe { take() };
    if raw.is_null() {
        return None;
    }
    // Safety: pointer is non-null and points to a NUL-terminated C string owned
    // by the cdylib for as long as we don't free it.
    let owned = cstr_to_lossy_string(unsafe { CStr::from_ptr(raw) });
    // Safety: we received the pointer from the cdylib's own take_last_error and
    // pair the free immediately. `from_utf8_lossy` already copied the bytes.
    unsafe { free(raw) };
    Some(owned)
}

/// Truncate `s` to at most
/// `max_bytes` bytes, walking down to the nearest UTF-8 char boundary so
/// the returned slice is always valid `&str`. Used by `info()` to
/// log a JSON prefix without panicking on multi-byte char cutoffs.
///
/// Properties:
/// - For `s.len() <= max_bytes`: returns the whole string (no-op loop).
/// - For `max_bytes == 0`: returns `""`.
/// - For all other cases: result length is `<= max_bytes` and the
///   slice is a valid prefix at a char boundary.
fn safe_utf8_prefix(s: &str, max_bytes: usize) -> &str {
    let mut end = max_bytes.min(s.len());
    while end > 0 && !s.is_char_boundary(end) {
        end -= 1;
    }
    &s[..end]
}

/// The legal key closest to `unknown` by Levenshtein
/// edit distance — the "did you mean" half of the info-JSON unknown-key
/// warn. `legal` is a compile-time non-empty set; ties resolve to the
/// earlier entry (stable, deterministic). Cold path (node load only).
fn nearest_legal_key<'a>(unknown: &str, legal: &[&'a str]) -> &'a str {
    fn levenshtein(a: &str, b: &str) -> usize {
        let a: Vec<char> = a.chars().collect();
        let b: Vec<char> = b.chars().collect();
        // Single-row DP; row[j] = distance(a[..i], b[..j]).
        let mut row: Vec<usize> = (0..=b.len()).collect();
        for (i, ca) in a.iter().enumerate() {
            let mut prev = row[0]; // distance(a[..i], b[..0])
            row[0] = i + 1;
            for (j, cb) in b.iter().enumerate() {
                let substitute = prev + usize::from(ca != cb);
                prev = row[j + 1];
                row[j + 1] = substitute.min(prev + 1).min(row[j] + 1);
            }
        }
        row[b.len()]
    }
    legal
        .iter()
        .copied()
        .min_by_key(|l| levenshtein(unknown, l))
        .unwrap_or("name")
}

/// Public wrapper for fuzz testing `parse_info_json`.
///
/// Only available when the `fuzz-helpers` feature is enabled.
///
/// `parse_info_json` returns `Result<NodeInfo, String>` internally (a
/// bare reason string — it has no `diag_label` context). The wrapper
/// re-wraps the error into the same `TransportError::NodeInfoParse`
/// variant the production `info()` path emits, so a direct caller (the
/// fuzz target) observes the documented error variant rather
/// than a raw String — keeping the variant consistent across all
/// callers of the parse path.
#[cfg(feature = "fuzz-helpers")]
pub fn fuzz_parse_info_json(json_str: &str) -> TransportResult<NodeInfo> {
    DylibNodeEntry::parse_info_json(json_str).map_err(|reason| TransportError::NodeInfoParse {
        diag_label: "<fuzz>".to_string(),
        json_len: json_str.len(),
        prefix: safe_utf8_prefix(json_str, 80).to_string(),
        reason,
    })
}

// ---------------------------------------------------------------------------
// This module's half of the ABI LAYOUT PIN (see `crate::abi_layout`).
//
// `abi_pin_struct!` expands to an exhaustive destructuring pattern with no `..`
// rest pattern, so adding or removing a field of one of these structs is a
// COMPILE ERROR naming the struct and the field; it also measures
// size/align/`offset_of!`, which `crate::abi_layout` compares against the
// snapshot table keyed to `CERULION_ABI_VERSION`. `abi_pin_enum!` does the
// same for a variant set (an enum carries no stable field offsets).
// ---------------------------------------------------------------------------
#[cfg(test)]
pub(crate) fn abi_layout_pins() -> Vec<crate::abi_layout::MeasuredStruct> {
    use crate::abi_layout::{abi_pin_enum, abi_pin_struct};
    vec![
        abi_pin_struct!(NodeContext {
            publishers,
            subscribers,
            clock,
            shutdown_signal,
            env_snapshot,
            qos_events,
            node_id,
            recon_logged,
            transport
        }),
        abi_pin_struct!(ShutdownSignal { inner }),
        abi_pin_enum!(AnyPublisher { AnyPublisher::Ipc(_) }),
        abi_pin_enum!(AnySubscriber { AnySubscriber::Ipc(_) }),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wire::MaxSliceLen;

    /// The non-blocking doorbell-write overflow pin. A FULL pipe
    /// must NOT block the helper (a blocking write would pin the thread in
    /// pipe_write under a stalled reader); instead each write to a full pipe
    /// returns `true` (keep running — the pending byte already rings the
    /// LEVEL-TRIGGERED doorbell) and bumps the process overflow counter. Hand
    /// oracle: N hammered writes on a still-full pipe ⇒ overflow delta == N; a
    /// drain + one write then rings a FRESH byte with no overflow bump (re-arm).
    /// Only this test touches `DOORBELL_WRITE_OVERFLOWS`, so DELTAS are robust
    /// without `#[serial]`. Bounded loops — a bug can never hang the test.
    #[test]
    fn doorbell_write_ring_is_nonblocking_and_counts_overflow() {
        // A pipe with O_NONBLOCK on BOTH ends (write end mirrors what
        // `spawn_cdylib_blocking_doorbell` sets; read end so the drain never
        // blocks). RAII-closed at the end.
        let mut fds = [0 as libc::c_int; 2];
        // SAFETY: `fds` is a 2-element array; pipe(2) fills [read, write].
        assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0, "pipe(2)");
        let (read_fd, write_fd) = (fds[0], fds[1]);
        for fd in [read_fd, write_fd] {
            // SAFETY: live pipe end; set O_NONBLOCK on its flags.
            let ok = unsafe {
                let f = libc::fcntl(fd, libc::F_GETFL);
                f >= 0 && libc::fcntl(fd, libc::F_SETFL, f | libc::O_NONBLOCK) == 0
            };
            assert!(ok, "set O_NONBLOCK");
        }

        // FILL the pipe: write via the primitive until it detects EAGAIN (the
        // first overflow bump). Bounded so a bug can never hang the test.
        let before_fill = doorbell_write_overflow_count();
        let mut filled = false;
        for _ in 0..2_000_000u32 {
            assert!(
                doorbell_write_ring(write_fd),
                "a writable/full pipe never returns fatal here"
            );
            if doorbell_write_overflow_count() > before_fill {
                filled = true;
                break;
            }
        }
        assert!(
            filled,
            "the pipe must fill and report EAGAIN within the bound"
        );

        // HAMMER N more writes on the still-full pipe: each is a coalesced
        // overflow ring (returns true, bumps the counter) — the helper stays
        // ALIVE with no wedge (this returns immediately, never blocks).
        const N: u64 = 32;
        let c0 = doorbell_write_overflow_count();
        for _ in 0..N {
            assert!(
                doorbell_write_ring(write_fd),
                "a full-pipe write is a coalesced ring, not a fatal error"
            );
        }
        assert_eq!(
            doorbell_write_overflow_count() - c0,
            N,
            "hand oracle: N hammered full-pipe writes == N overflow rings"
        );

        // RE-ARM: drain some bytes, then one write rings a FRESH byte with NO
        // overflow bump (the pipe now has room).
        let mut buf = [0u8; 4096];
        // SAFETY: read into a live buffer from our non-blocking read end.
        let n = unsafe { libc::read(read_fd, buf.as_mut_ptr().cast::<libc::c_void>(), buf.len()) };
        assert!(n > 0, "drain must free pipe space");
        let c1 = doorbell_write_overflow_count();
        assert!(doorbell_write_ring(write_fd), "post-drain write succeeds");
        assert_eq!(
            doorbell_write_overflow_count(),
            c1,
            "re-arm: a write into a drained pipe is a real ring, not an overflow"
        );

        // SAFETY: close both owned ends exactly once.
        unsafe {
            libc::close(read_fd);
            libc::close(write_fd);
        }
    }

    /// The cdylib snapshot lifecycle state
    /// machine (`SnapshotState`). The bug it guards against: latching as if `set`
    /// succeeded, then calling `snapshot` every step (which froze nothing yet
    /// returned 0, silently masking the lost hold). Oracle-vector (NOT a
    /// self-compare) over the full transition table; the load-bearing assertions
    /// are (a) a `Failed` registration NEVER becomes `Active`, even on a later 0,
    /// and (b) `Failed` does NOT invoke `snapshot`.
    #[test]
    fn snapshot_state_after_set_and_invoke_oracle() {
        use SnapshotState::{Active, Failed, Pending};

        // after_set: (start, ret) -> expected next state. Hand-written oracle.
        let cases = [
            (Pending, 0, Active),  // first set succeeds → hold active
            (Pending, -1, Failed), // NODES-poison / handle / utf8 / panic → terminal Failed
            (Pending, -4, Failed), // any non-zero code fails the same way
            (Pending, 7, Failed),  // sign-agnostic: any non-zero is failure
            (Active, 0, Active),   // terminal: re-applying a success is a no-op
            (Active, -1, Active),  // terminal: a stray non-zero can't un-Active it
            (Failed, 0, Failed),   // KEY: a failed registration NEVER becomes Active
            (Failed, -1, Failed),  // terminal stays Failed
        ];
        for (start, ret, expected) in cases {
            assert_eq!(
                start.after_set(ret),
                expected,
                "after_set({start:?}, {ret}) should be {expected:?}"
            );
        }

        // should_invoke_snapshot: ONLY Active calls the per-step snapshot FFI.
        assert!(
            !Pending.should_invoke_snapshot(),
            "Pending must not snapshot yet"
        );
        assert!(
            Active.should_invoke_snapshot(),
            "Active snapshots each step"
        );
        assert!(
            !Failed.should_invoke_snapshot(),
            "KEY: Failed must NOT snapshot — that is the silent-mask bug"
        );

        // The composed contract walked as the runtime drives it:
        //  - happy path: Pending --set ok--> Active --snapshots-->...
        let happy = Pending.after_set(0);
        assert_eq!(happy, Active);
        assert!(happy.should_invoke_snapshot());
        //  - failure path: Pending --set fails--> Failed --never snapshots-->...
        let failed = Pending.after_set(-1);
        assert_eq!(failed, Failed);
        assert!(!failed.should_invoke_snapshot());
        // ...and a later spurious success cannot resurrect it.
        assert_eq!(failed.after_set(0), Failed);
        assert!(!failed.after_set(0).should_invoke_snapshot());
    }

    /// The cdylib-subscriber "RUST_LOG spec is absent" rule
    /// (`None` or empty ⇒ default to `info`). Oracle-vector over the pure
    /// predicate — testing it directly avoids installing a subscriber (which
    /// would clobber this test binary's tracing default). The install path
    /// itself is covered by the subprocess e2e (`cdylib_tracing_stopgap_test.rs`).
    #[test]
    fn cdylib_rust_log_absence_rule() {
        // Absent: no spec at all, or an empty string (empty RUST_LOG == unset).
        assert!(
            cdylib_rust_log_is_absent(None),
            "None ⇒ absent ⇒ default info"
        );
        assert!(
            cdylib_rust_log_is_absent(Some("")),
            "empty string ⇒ absent ⇒ default info"
        );
        // Present: any non-empty directive is applied verbatim by the installer.
        assert!(!cdylib_rust_log_is_absent(Some("info")));
        assert!(!cdylib_rust_log_is_absent(Some("off")));
        assert!(!cdylib_rust_log_is_absent(Some(
            "cerulion=debug,iceoryx2=warn"
        )));
    }

    /// The `flood_log_action` latch transitions (shared by
    /// `pump_history` and `snapshot_inputs`).
    /// Pins the contract the live-loop-cadence logging relies on: first failure
    /// loud, subsequent failures suppressed, success after failure recovers,
    /// success-without-failure is silent — and the latch flag tracks it.
    #[test]
    fn test_flood_log_action_latch_transitions() {
        let mut latched = false;

        // success while clean → nothing, stays unlatched
        assert_eq!(flood_log_action(&mut latched, false), FloodLogAction::Quiet);
        assert!(!latched);

        // first failure → loud, latches
        assert_eq!(
            flood_log_action(&mut latched, true),
            FloodLogAction::FirstFailure
        );
        assert!(latched);

        // continuing failures → suppressed, stays latched
        assert_eq!(
            flood_log_action(&mut latched, true),
            FloodLogAction::SuppressedFailure
        );
        assert_eq!(
            flood_log_action(&mut latched, true),
            FloodLogAction::SuppressedFailure
        );
        assert!(latched);

        // first success after a failure regime → recovers, clears latch
        assert_eq!(
            flood_log_action(&mut latched, false),
            FloodLogAction::Recovered
        );
        assert!(!latched);

        // back to quiet on continued success
        assert_eq!(flood_log_action(&mut latched, false), FloodLogAction::Quiet);
        assert!(!latched);

        // a NEW failure regime latches loudly again (proves it re-arms)
        assert_eq!(
            flood_log_action(&mut latched, true),
            FloodLogAction::FirstFailure
        );
        assert!(latched);
    }

    #[test]
    fn test_closure_node_info() {
        let info = NodeInfo::from_names(vec!["in1".to_string()], vec!["out1".to_string()]);
        let node = ClosureNodeEntry::new(info.clone(), |_ctx| Ok(()));
        let retrieved = node.info().expect("closure node info is infallible");
        assert_eq!(retrieved.input_names, vec!["in1"]);
        assert_eq!(retrieved.output_names, vec!["out1"]);
    }

    #[test]
    fn test_closure_node_tick_without_init() {
        let info = NodeInfo::default();
        let mut node = ClosureNodeEntry::new(info, |_ctx| Ok(()));
        let result = node.tick();
        assert!(result.is_err());
    }

    #[test]
    fn test_closure_node_init_and_tick() {
        use std::sync::atomic::{AtomicU32, Ordering};
        use std::sync::Arc;

        let counter = Arc::new(AtomicU32::new(0));
        let counter_clone = Arc::clone(&counter);

        let info = NodeInfo::default();
        let mut node = ClosureNodeEntry::new(info, move |_ctx| {
            counter_clone.fetch_add(1, Ordering::Relaxed);
            Ok(())
        });

        let ctx = NodeContext::for_tests(IndexMap::new(), IndexMap::new());
        node.init(ctx).unwrap();
        node.tick().unwrap();
        node.tick().unwrap();

        assert_eq!(counter.load(Ordering::Relaxed), 2);
    }

    // =======================================================================
    // Producer-side reconciliation counters + the teardown harvest
    // primitive. Real iceoryx2 over a per-test isolated SHM root
    // (`init_for_test` — parallel-safe, no `#[serial]`).
    // =======================================================================

    fn test_publisher(topic: &str) -> crate::transport::publisher::CerulionPublisher {
        use crate::transport::{TransportConfig, TransportManager};
        let mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "recon_test".into(),
                clock: std::sync::Arc::new(crate::clock::RealClock),
                subscriber_buffer_size: 4,
                network: None,
            },
            crate::testing::iceoryx_test_config(),
        )
        .expect("per-test SHM transport");
        mgr.create_publisher(
            topic,
            MaxSliceLen::try_new(1024).expect("1024 >= header"),
            0,
        )
        .expect("create publisher")
    }

    /// The steady-state send-fail counter starts at 0 and
    /// increments via the direct-call seam (the `OutputProxy::Drop` arms call
    /// `record_send_fail_drop`; code-read pins the call sites, this pins the
    /// counter+accessor). `frames_dropped_overflow` is independent.
    #[test]
    fn frames_dropped_send_fail_counter_increments() {
        let pubr = test_publisher("send_fail");
        assert_eq!(pubr.frames_dropped_send_fail(), 0, "starts at 0");
        assert_eq!(
            pubr.frames_dropped_overflow(),
            0,
            "independent counter at 0"
        );
        pubr.record_send_fail_drop();
        pubr.record_send_fail_drop();
        assert_eq!(
            pubr.frames_dropped_send_fail(),
            2,
            "two send-fail drops counted"
        );
        assert_eq!(
            pubr.frames_dropped_overflow(),
            0,
            "overflow counter untouched by send-fail drops"
        );

        // The AnyPublisher enum dispatch surfaces the same counters.
        let any = AnyPublisher::Ipc(pubr);
        assert_eq!(any.frames_dropped_send_fail(), 2);
        assert_eq!(any.frames_dropped_overflow(), 0);
    }

    /// The teardown harvest primitive. `NodeContext` yields one
    /// `PublisherReconStat` per owned publisher, and a `ClosureNodeEntry`
    /// override forwards the SAME snapshots (the in-process harvest path). Oracle
    /// on the emitted fields — never a self-compare.
    #[test]
    fn publisher_recon_stats_harvest_via_context_and_closure_entry() {
        let topic = "harvest";
        let pubr = test_publisher(topic);
        let mut pubs: IndexMap<String, AnyPublisher> = IndexMap::new();
        pubs.insert("out".to_string(), AnyPublisher::Ipc(pubr));

        // (a) NodeContext-level harvest.
        let ctx = NodeContext::for_tests(pubs, IndexMap::new());
        let mut stats = Vec::new();
        ctx.collect_publisher_recon_stats(&mut stats);
        assert_eq!(stats.len(), 1, "one publisher → one recon stat");
        let s = &stats[0];
        assert_eq!(s.topic, topic);
        assert_eq!(s.next_sequence, 0, "no committed frames yet");
        // An UNSEEDED (live-path) publisher starts at 0, so
        // the counter and the committed count agree — which is exactly why the
        // conflation went unnoticed until a replay seeded one.
        assert_eq!(s.initial_sequence, 0, "the live path seeds nothing");
        assert_eq!(s.committed_frames, 0, "no committed frames yet");
        assert_eq!(s.frames_dropped_send_fail, 0);
        assert_eq!(s.frames_dropped_overflow, 0);

        // (a') Idempotent (guard hardening): a SECOND harvest of the
        //      SAME context pushes NOTHING — the run-log surface is claimed
        //      exactly once, so a stray double-harvest cannot double-count.
        let mut again = Vec::new();
        ctx.collect_publisher_recon_stats(&mut again);
        assert!(again.is_empty(), "second harvest is a no-op (idempotent)");

        // (b) NodeEntry override forwards the SAME snapshot shape through a
        //     ClosureNodeEntry that OWNS a FRESH (never-harvested) context — a
        //     once-harvested context would correctly yield nothing (see (a')).
        let pubr2 = test_publisher(topic);
        let mut pubs2: IndexMap<String, AnyPublisher> = IndexMap::new();
        pubs2.insert("out".to_string(), AnyPublisher::Ipc(pubr2));
        let ctx2 = NodeContext::for_tests(pubs2, IndexMap::new());
        let mut node = ClosureNodeEntry::new(NodeInfo::default(), |_c| Ok(()));
        node.init(ctx2).expect("init");
        let mut via_entry = Vec::new();
        node.collect_publisher_recon_stats(&mut via_entry);
        assert_eq!(via_entry, stats, "entry harvest == context harvest");
    }

    /// A `DylibNodeEntry` transfers its context into the cdylib at
    /// init, so it correctly yields NOTHING host-side (the default no-op) — the
    /// documented cdylib gap. Pinned via a NOT-yet-init'd default no-op path:
    /// the trait default returns empty (a raw entry with no context).
    #[test]
    fn recon_stats_default_is_empty_for_contextless_entry() {
        // A closure entry WITHOUT init (no context) harvests nothing.
        let node = ClosureNodeEntry::new(NodeInfo::default(), |_c| Ok(()));
        let mut out = Vec::new();
        node.collect_publisher_recon_stats(&mut out);
        assert!(out.is_empty(), "no context → no publisher stats");
    }

    #[test]
    fn test_parse_info_json() {
        let json = r#"{"inputs":[],"outputs":[]}"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        assert!(info.input_names.is_empty());
        assert!(info.output_names.is_empty());
    }

    #[test]
    fn test_parse_info_json_with_ports() {
        let json = r#"{"inputs":["a","b"],"outputs":["x"]}"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        assert_eq!(info.input_names, vec!["a", "b"]);
        assert_eq!(info.output_names, vec!["x"]);
    }

    #[test]
    fn test_parse_info_json_v6_input_objects_carry_expect_within_ms() {
        // ABI v6: inputs are JSON OBJECTS carrying an
        // optional `expect_within_ms`, and the top-level JSON carries
        // `tick_within_ms` / `throttle_ms`. Outputs carry an always-
        // present `promise_within_ms`. Assert all four QoS knobs land on
        // the parsed `NodeInfo`.
        let json = r#"{
            "inputs":[{"name":"img","expect_within_ms":20},{"name":"imu"}],
            "outputs":[{"name":"out","schema_hash":123,"max_slice_len_default":null,"promise_within_ms":30}],
            "tick_within_ms":10,
            "throttle_ms":5
        }"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();

        // Names + order preserved.
        assert_eq!(info.input_names, vec!["img", "imu"]);
        assert_eq!(info.output_names, vec!["out"]);

        // Per-input expect_within_ms: Some on the first, None on the second.
        assert_eq!(info.input_meta.len(), 2);
        assert_eq!(info.input_meta[0].name, "img");
        assert_eq!(info.input_meta[0].expect_within_ms, Some(20));
        assert_eq!(info.input_meta[1].name, "imu");
        assert_eq!(info.input_meta[1].expect_within_ms, None);

        // Per-output promise_within_ms.
        assert_eq!(info.output_meta.len(), 1);
        assert_eq!(info.output_meta[0].name, "out");
        assert_eq!(info.output_meta[0].promise_within_ms, Some(30));
        assert_eq!(info.output_meta[0].schema_hash, 123);
        assert_eq!(info.output_meta[0].max_slice_len_default, None);

        // Node-level QoS.
        assert_eq!(info.tick_within_ms(), Some(10));
        assert_eq!(info.throttle_ms(), Some(5));

        // SAFETY: with NO depth keys in the payload (v6-shaped input
        // objects), the fallback resolves DEFAULT_CONSUMER_DEPTH —
        // and backpressure/trigger keep the topology-safe defaults — so
        // topology wiring is byte-identical to the pre-v6
        // empty-meta path.
        for m in &info.input_meta {
            assert_eq!(m.depth, crate::graph::topology::DEFAULT_CONSUMER_DEPTH);
            assert_eq!(m.backpressure, BackpressurePolicy::default());
            assert!(!m.trigger);
        }
    }

    #[test]
    fn test_parse_info_json_v8_inputs_carry_declared_depth() {
        // ABI v8: input objects may carry the declared
        // `#[input(depth = N)]`. Present → the parsed InputMeta carries
        // EXACTLY the declared value (pre-v8 this was hardcoded to
        // DEFAULT_CONSUMER_DEPTH — the silent test/live divergence this
        // bump kills). Absent → the host fallback DEFAULT_CONSUMER_DEPTH.
        // Coexists with expect_within_ms in either order.
        let json = r#"{
            "inputs":[
                {"name":"deep","depth":32},
                {"name":"both","expect_within_ms":20,"depth":1},
                {"name":"undeclared"}
            ],
            "outputs":[]
        }"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        assert_eq!(info.input_meta.len(), 3);
        assert_eq!(info.input_meta[0].name, "deep");
        assert_eq!(
            info.input_meta[0].depth, 32,
            "declared depth must survive the FFI parse, not the default"
        );
        assert_eq!(info.input_meta[1].depth, 1);
        assert_eq!(
            info.input_meta[1].expect_within_ms,
            Some(20),
            "depth must coexist with expect_within_ms on one object"
        );
        assert_eq!(
            info.input_meta[2].depth,
            crate::graph::topology::DEFAULT_CONSUMER_DEPTH,
            "absent depth key = not declared = host default"
        );
    }

    #[test]
    fn test_parse_info_json_v9_inputs_carry_trigger() {
        // ABI v9: an input object may carry `"trigger":true`
        // (the declared `#[input(trigger)]` mark). Present → the parsed
        // InputMeta carries `trigger == true`; a plain object (no key) and
        // a legacy bare string both default to `false`. Pre-v9 this was
        // HARDCODED false on the FFI path (the gap this bump closes).
        // Hand oracle over all three input forms; coexists with the other
        // v6/v8 QoS keys in any order.
        let json = r#"{
            "inputs":[
                {"name":"fire","trigger":true,"depth":4},
                {"name":"ctx","depth":2},
                "legacy"
            ],
            "outputs":[]
        }"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        assert_eq!(info.input_meta.len(), 3);

        // Trigger input: mark parsed, coexists with `depth`.
        assert_eq!(info.input_meta[0].name, "fire");
        assert!(
            info.input_meta[0].trigger,
            "`trigger:true` must survive the FFI parse (pre-v9 it was dropped)"
        );
        assert_eq!(
            info.input_meta[0].depth, 4,
            "trigger must coexist with depth on one object"
        );

        // Plain object with no trigger key → false.
        assert_eq!(info.input_meta[1].name, "ctx");
        assert!(
            !info.input_meta[1].trigger,
            "an object with no trigger key defaults to false (non-trigger read)"
        );

        // Legacy bare string → false.
        assert_eq!(info.input_meta[2].name, "legacy");
        assert!(
            !info.input_meta[2].trigger,
            "a legacy bare-string input defaults to false"
        );
    }

    #[test]
    fn test_parse_info_json_input_schema_hash_optional_defaults_zero() {
        // An input object may carry `"schema_hash":N` (the port
        // type's layout hash the macro now emits). Present → the parsed
        // InputMeta carries EXACTLY N, so the network ingress-hash
        // resolver can validate ingress frames against a cdylib consumer's
        // schema. ABSENT (a raw-FFI info block that omits the key; pre-v11
        // cdylibs are refused at the ABI gate before parsing) → serde
        // default `0`, the "no declared schema" sentinel — the raw-FFI
        // back-compat contract. A legacy bare-string input is `0` too. Hand
        // oracle over all three forms; coexists with the other QoS keys.
        let json = r#"{
            "inputs":[
                {"name":"typed","schema_hash":18446744073709551615,"trigger":true},
                {"name":"hashless","depth":2},
                "legacy"
            ],
            "outputs":[]
        }"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        assert_eq!(info.input_meta.len(), 3);

        // Declared hash parses through verbatim (u64::MAX exercises the full
        // width — no truncation).
        assert_eq!(info.input_meta[0].name, "typed");
        assert_eq!(
            info.input_meta[0].schema_hash,
            u64::MAX,
            "a declared input schema_hash must round-trip the FFI parse"
        );
        assert!(
            info.input_meta[0].trigger,
            "schema_hash must coexist with trigger on one object"
        );

        // A Full object with NO schema_hash key → 0 (raw-FFI info block).
        assert_eq!(info.input_meta[1].name, "hashless");
        assert_eq!(
            info.input_meta[1].schema_hash, 0,
            "an object with no schema_hash key defaults to the 0 sentinel \
             (back-compat: a cdylib built before the key existed)"
        );

        // A legacy bare-string input → 0.
        assert_eq!(info.input_meta[2].name, "legacy");
        assert_eq!(
            info.input_meta[2].schema_hash, 0,
            "a legacy bare-string input carries the 0 sentinel"
        );
    }

    #[test]
    #[tracing_test::traced_test]
    fn unknown_input_key_warns_names_nearest_and_still_loads_with_default() {
        // A typo'd optional key (`bakpressure`) on a
        // Full input object. serde-untagged ignores it (deny_unknown_fields
        // is impossible on untagged variants), so without the warn the declared
        // `block` intent is dropped in TOTAL SILENCE. With it: loud warn naming
        // the node, port, key, and the nearest legal key — while back-compat
        // is unchanged (the entry still loads, the field defaults).
        let json = r#"{"inputs":[{"name":"scan","bakpressure":"block"}],"outputs":[]}"#;
        let info = DylibNodeEntry::parse_info_json_labeled(json, "typo_node").expect("still loads");
        assert_eq!(info.input_meta.len(), 1);
        assert_eq!(
            info.input_meta[0].backpressure,
            BackpressurePolicy::default(),
            "the typo'd intent falls back to the default (back-compat unchanged)"
        );
        assert!(logs_contain("unknown key"), "the warn must fire");
        assert!(logs_contain("bakpressure"), "names the offending key");
        // The suggestion moved OUT of the message prose and into
        // the `nearest=` structured field (the message must stay constant), so
        // the oracle now pins the field an operator would actually grep.
        assert!(
            logs_contain("nearest=backpressure"),
            "names the nearest legal key in the `nearest` field"
        );
        assert!(logs_contain("typo_node"), "names the node (diag_label)");
    }

    #[test]
    #[tracing_test::traced_test]
    fn unknown_envelope_key_warns_names_nearest_and_still_loads_with_default() {
        // The TOP-LEVEL twin of the arm above. Every field of
        // `NodeInfoJson` is `#[serde(default)] Option<..>`, so `"polciy"`
        // parsed fine, `policy` took its default `None`, and the node ran with
        // its declared trigger policy absent — in total silence, at the one
        // level a hand-written raw-FFI info block is most likely to get wrong.
        //
        // The payload declares NOTHING wrong below the envelope, so a warn
        // here can only have come from the new top-level scan.
        let json = r#"{"inputs":[],"outputs":[],"polciy":{"period_ms":16}}"#;
        let info = DylibNodeEntry::parse_info_json_labeled(json, "envelope_typo_node")
            .expect("still loads");

        // Back-compat is unchanged: the entry still loads and the misspelled
        // field takes its default. That is what makes the warn the ONLY signal
        // the operator gets, and therefore what makes its absence a defect.
        assert!(
            info.policy.is_none(),
            "the typo'd intent falls back to the default (back-compat unchanged)"
        );

        // The KEY as a whole `key=value` token: a bare
        // `logs_contain("polciy")` would also be satisfied by the did-you-mean
        // clause of some other line.
        assert!(
            logs_contain("unknown_key=polciy"),
            "the warn must name the offending key under `unknown_key`"
        );
        // The nearest key rides the `nearest` FIELD (the structured-logging
        // rule), so assert the field rather than prose — which is the stronger
        // pin anyway: a message can mention a key it did not compute.
        assert!(
            logs_contain("nearest=policy"),
            "the warn must carry the nearest legal key under `nearest`"
        );
        assert!(
            logs_contain("envelope_typo_node"),
            "names the node (diag_label)"
        );
        // The DISCRIMINATOR: this is the envelope scan, not the port-entry one.
        // Without it the arm is satisfied by any unknown-key warn at all.
        assert!(
            logs_contain("<top-level>"),
            "the warn must say it is about the envelope, not a port entry"
        );
        assert!(
            logs_contain("WARN"),
            "an ignored declaration is a warn, not a debug line"
        );
    }

    #[test]
    #[tracing_test::traced_test]
    fn exact_key_payload_warns_nothing() {
        // Negative control: a payload using EXACTLY the legal keys
        // (inputs incl. `trigger`, outputs with the full v8 surface) must
        // load silently — no unknown-key warn.
        //
        // The ENVELOPE is now exercised at its full width too —
        // every key of `LEGAL_ENVELOPE_KEYS`, i.e. every field of
        // `NodeInfoJson`. That is what makes the legal set's COMPLETENESS a
        // checked claim: drop `policy` (or either QoS key) from it and every
        // real macro-emitted cdylib that declares one starts warning about a
        // key it was right to emit, which this arm now fails on. A control
        // that only carried `inputs`/`outputs` could not see that.
        let json = r#"{
            "inputs":[{"name":"scan","trigger":true,"depth":4,"backpressure":"block","expect_within_ms":50}],
            "outputs":[{"name":"out","schema_hash":123,"max_slice_len_default":null,"promise_within_ms":null}],
            "policy":{"period_ms":16},
            "tick_within_ms":5,
            "throttle_ms":7
        }"#;
        let info = DylibNodeEntry::parse_info_json(json).expect("loads");
        assert_eq!(info.input_meta.len(), 1);
        assert_eq!(info.output_meta.len(), 1);
        // The envelope keys are not merely tolerated, they PARSE — otherwise
        // this arm would still pass against a build that had stopped reading
        // them and was silently defaulting all three.
        assert_eq!(info.policy, Some(MacroPolicy::Period { period_ms: 16 }));
        assert_eq!(info.tick_within_ms, Some(5));
        assert_eq!(info.throttle_ms, Some(7));
        // The `trigger` key is a REAL parsed value now (not just a
        // silently-accepted legal key) — the payload's `"trigger":true` lands.
        assert!(
            info.input_meta[0].trigger,
            "the exact-key payload's `trigger:true` must parse through"
        );
        assert!(
            !logs_contain("unknown key"),
            "exact-key payloads must not warn"
        );
    }

    #[test]
    #[tracing_test::traced_test]
    fn unknown_output_key_warns_with_output_legal_set() {
        // The outputs section uses ITS OWN legal set — a typo of
        // `promise_within_ms` resolves to the output-side nearest key.
        let json = r#"{"inputs":[],"outputs":[{"name":"out","promise_witin_ms":30}]}"#;
        let info = DylibNodeEntry::parse_info_json_labeled(json, "typo_out_node").expect("loads");
        assert_eq!(info.output_meta.len(), 1);
        assert_eq!(
            info.output_meta[0].promise_within_ms, None,
            "typo'd promise key falls back to None"
        );
        assert!(logs_contain("promise_witin_ms"));
        // The suggestion is carried by the `nearest=` field, not the prose.
        assert!(logs_contain("nearest=promise_within_ms"));
    }

    #[test]
    fn nearest_legal_key_picks_minimum_edit_distance() {
        // Oracle vectors for the levenshtein helper (deterministic ties →
        // earlier entry).
        let legal = &[
            "name",
            "trigger",
            "depth",
            "backpressure",
            "expect_within_ms",
        ];
        assert_eq!(nearest_legal_key("bakpressure", legal), "backpressure");
        assert_eq!(nearest_legal_key("dept", legal), "depth");
        assert_eq!(
            nearest_legal_key("expect_within", legal),
            "expect_within_ms"
        );
        assert_eq!(nearest_legal_key("nmae", legal), "name");
    }

    #[test]
    fn test_parse_info_json_v8_inputs_carry_declared_backpressure() {
        // ABI v8: input objects may carry the declared
        // backpressure — wire shape `"drop_oldest"` / `"block"` /
        // `{"sample":N}`. Present → the parsed InputMeta carries EXACTLY
        // the declared policy (pre-v8 this was hardcoded to DropOldest:
        // a dylib `block` input silently dropped data live). Absent → the
        // DropOldest default. Coexists with depth + expect_within_ms.
        let json = r#"{
            "inputs":[
                {"name":"b","backpressure":"block","depth":32},
                {"name":"s","backpressure":{"sample":7},"expect_within_ms":20},
                {"name":"d","backpressure":"drop_oldest"},
                {"name":"undeclared"}
            ],
            "outputs":[]
        }"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        assert_eq!(info.input_meta.len(), 4);
        assert_eq!(
            info.input_meta[0].backpressure,
            BackpressurePolicy::Block,
            "declared `block` must survive the FFI parse, not the default"
        );
        assert_eq!(
            info.input_meta[0].depth, 32,
            "backpressure must coexist with depth on one object"
        );
        assert_eq!(
            info.input_meta[1].backpressure,
            BackpressurePolicy::Sample(7),
            "the {{\"sample\":N}} shape must parse to Sample(N)"
        );
        assert_eq!(
            info.input_meta[1].expect_within_ms,
            Some(20),
            "backpressure must coexist with expect_within_ms"
        );
        assert_eq!(
            info.input_meta[2].backpressure,
            BackpressurePolicy::DropOldest,
            "an EXPLICIT drop_oldest declaration parses too"
        );
        assert_eq!(
            info.input_meta[3].backpressure,
            BackpressurePolicy::DropOldest,
            "absent backpressure key = not declared = DropOldest default"
        );
    }

    #[test]
    fn test_parse_info_json_v6_legacy_bare_string_inputs_still_parse() {
        // Backward-compat: pre-v6 cdylibs AND raw-FFI
        // (CLI-templated) nodes emit inputs as a bare-string array. The
        // untagged `InputJson::Name` variant MUST still accept this. The
        // legacy inputs carry no QoS → `expect_within_ms` is None, and the
        // node-level QoS knobs are absent → None.
        let json = r#"{"inputs":["a","b"],"outputs":["x"]}"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        assert_eq!(info.input_names, vec!["a", "b"]);
        assert_eq!(info.output_names, vec!["x"]);
        assert_eq!(info.input_meta.len(), 2);
        for m in &info.input_meta {
            assert_eq!(
                m.expect_within_ms, None,
                "legacy bare-string inputs carry no expect_within_ms"
            );
            // Still topology-safe defaults.
            assert_eq!(m.depth, crate::graph::topology::DEFAULT_CONSUMER_DEPTH);
            assert_eq!(m.backpressure, BackpressurePolicy::default());
            assert!(!m.trigger);
        }
        // Absent node-level QoS → None.
        assert_eq!(info.tick_within_ms(), None);
        assert_eq!(info.throttle_ms(), None);
    }

    #[test]
    fn test_parse_info_json_v6_input_object_missing_name_is_rejected() {
        // Silent-failure pin (symmetric with the output-side
        // wrong-type rejection): a malformed input OBJECT lacking the required
        // `name` matches NEITHER untagged `InputJson` variant (`Name` wants a
        // string, `Full` requires `name`), so the whole parse must Err LOUDLY
        // — never silently drop the input or ghost-wire an empty node.
        let json = r#"{"inputs":[{"expect_within_ms":20}],"outputs":[]}"#;
        assert!(
            DylibNodeEntry::parse_info_json(json).is_err(),
            "input object missing `name` must be rejected, not silently dropped"
        );
    }

    #[test]
    fn test_parse_info_json_v6_input_object_wrong_typed_expect_within_ms_is_rejected() {
        // A present-but-wrong-typed `expect_within_ms` (string, not u64) fails
        // the `Full` variant; `Name` also fails (object != string) → whole
        // parse Errs. Pins that serde-untagged REJECTS malformed QoS rather
        // than defaulting it to None and wiring a subtly-wrong node.
        let json = r#"{"inputs":[{"name":"x","expect_within_ms":"abc"}],"outputs":[]}"#;
        assert!(
            DylibNodeEntry::parse_info_json(json).is_err(),
            "wrong-typed expect_within_ms must be rejected, not defaulted to None"
        );
    }

    #[test]
    fn test_parse_info_json_legacy_string_outputs_yields_empty_meta() {
        // Backward-compat: older cdylibs ship
        // outputs as bare-string arrays. The loader must accept this
        // shape and produce `OutputMeta` with `schema_hash = 0` and
        // `max_slice_len_default = None` so the runtime tier-2 lookup
        // skips and falls through to tier-3.
        let json = r#"{"inputs":["in1"],"outputs":["x","y"]}"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        assert_eq!(info.input_names, vec!["in1"]);
        assert_eq!(info.output_names, vec!["x", "y"]);
        assert_eq!(info.output_meta.len(), 2);
        for meta in &info.output_meta {
            assert_eq!(meta.schema_hash, 0);
            assert_eq!(meta.max_slice_len_default, None);
        }
    }

    #[test]
    fn test_parse_info_json_object_outputs_carries_schema_hash_and_max_slice_len() {
        // New shape: per-output JSON object
        // with `name`, `schema_hash`, and `max_slice_len_default`.
        let json = r#"{"inputs":["in1"],"outputs":[
            {"name":"image","schema_hash":1234,"max_slice_len_default":16777216},
            {"name":"twist","schema_hash":5678,"max_slice_len_default":4096}
        ]}"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        assert_eq!(info.input_names, vec!["in1"]);
        assert_eq!(info.output_names, vec!["image", "twist"]);
        assert_eq!(info.output_meta.len(), 2);
        assert_eq!(info.output_meta[0].name, "image");
        assert_eq!(info.output_meta[0].schema_hash, 1234);
        assert_eq!(
            info.output_meta[0].max_slice_len_default,
            MaxSliceLen::try_new(16777216)
        );
        assert_eq!(info.output_meta[1].name, "twist");
        assert_eq!(info.output_meta[1].schema_hash, 5678);
        assert_eq!(
            info.output_meta[1].max_slice_len_default,
            MaxSliceLen::try_new(4096)
        );
    }

    #[test]
    fn test_parse_info_json_object_outputs_with_null_max_slice_len_default() {
        // `max_slice_len_default: null` round-trips to `None`. This is
        // what the cdylib emits when the `<T as ShmMessage>::MAX_SLICE_LEN`
        // const is `None` (hand-written impls without a populated
        // schema-default).
        let json = r#"{"inputs":[],"outputs":[
            {"name":"custom","schema_hash":99,"max_slice_len_default":null}
        ]}"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        assert_eq!(info.output_meta.len(), 1);
        assert_eq!(info.output_meta[0].name, "custom");
        assert_eq!(info.output_meta[0].schema_hash, 99);
        assert_eq!(info.output_meta[0].max_slice_len_default, None);
    }

    #[test]
    fn test_parse_info_json_mixed_legacy_and_new_outputs_within_same_array() {
        // Untagged enum dispatch: the same array can contain both bare
        // strings and full objects. This is unusual but useful for
        // partial migrations and well-defined under serde's
        // `untagged` shape — each element is matched individually.
        let json = r#"{"inputs":[],"outputs":[
            "legacy_name",
            {"name":"new","schema_hash":42,"max_slice_len_default":8192}
        ]}"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        assert_eq!(info.output_meta.len(), 2);
        assert_eq!(info.output_meta[0].name, "legacy_name");
        assert_eq!(info.output_meta[0].schema_hash, 0);
        assert_eq!(info.output_meta[0].max_slice_len_default, None);
        assert_eq!(info.output_meta[1].name, "new");
        assert_eq!(info.output_meta[1].schema_hash, 42);
        assert_eq!(
            info.output_meta[1].max_slice_len_default,
            MaxSliceLen::try_new(8192)
        );
    }

    #[test]
    fn test_parse_info_json_object_outputs_missing_schema_hash_defaults_to_zero() {
        // Edge case:
        // an object missing `schema_hash` falls through `serde(default)`
        // and produces `schema_hash: 0`. The output is still
        // recognized as the `Full` variant because the `name` field
        // is present.
        let json = r#"{"inputs":[],"outputs":[
            {"name":"partial","max_slice_len_default":2048}
        ]}"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        assert_eq!(info.output_meta.len(), 1);
        assert_eq!(info.output_meta[0].name, "partial");
        assert_eq!(info.output_meta[0].schema_hash, 0);
        assert_eq!(
            info.output_meta[0].max_slice_len_default,
            MaxSliceLen::try_new(2048)
        );
    }

    #[test]
    fn test_parse_info_json_object_outputs_missing_max_slice_len_default_is_none() {
        // Object without `max_slice_len_default` defaults to `None`
        // — same shape a hand-written `impl ShmMessage` (default
        // trait const) would produce.
        let json = r#"{"inputs":[],"outputs":[
            {"name":"x","schema_hash":7}
        ]}"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        assert_eq!(info.output_meta.len(), 1);
        assert_eq!(info.output_meta[0].name, "x");
        assert_eq!(info.output_meta[0].schema_hash, 7);
        assert_eq!(info.output_meta[0].max_slice_len_default, None);
    }

    #[test]
    fn test_parse_info_json_object_outputs_wrong_type_for_schema_hash_is_rejected() {
        // Edge case:
        // a value of the wrong type for `schema_hash` (e.g. string
        // instead of u64) makes the `Full` variant fail to
        // deserialize. Under `serde(untagged)` the deserializer
        // tries the next variant — `Name(String)` — which expects a
        // bare string, not an object. Both variants fail → the array
        // element fails → the entire `outputs` array fails → the
        // top-level parse returns Err.
        let json = r#"{"inputs":[],"outputs":[
            {"name":"x","schema_hash":"oops"}
        ]}"#;
        let result = DylibNodeEntry::parse_info_json(json);
        assert!(
            result.is_err(),
            "wrong-type schema_hash must be rejected, got: {:?}",
            result
        );
    }

    #[test]
    fn test_parse_info_json_outputs_with_bare_number_element_is_rejected() {
        // Edge case:
        // a bare numeric element like `42` matches neither variant
        // (Name expects string, Full expects object) and the parse
        // fails cleanly.
        let json = r#"{"inputs":[],"outputs":[42]}"#;
        let result = DylibNodeEntry::parse_info_json(json);
        assert!(
            result.is_err(),
            "bare-number output element must be rejected, got: {:?}",
            result
        );
    }

    #[test]
    fn test_parse_info_json_empty_string() {
        let result = DylibNodeEntry::parse_info_json("");
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("empty JSON string"), "got: {}", err);
    }

    #[test]
    fn test_parse_info_json_whitespace_only() {
        let result = DylibNodeEntry::parse_info_json("   \n\t  ");
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("empty JSON string"), "got: {}", err);
    }

    #[test]
    fn test_parse_info_json_missing_braces() {
        // No valid JSON structure — serde_json will reject this
        let result = DylibNodeEntry::parse_info_json("not json at all");
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("JSON"), "got: {}", err);
    }

    #[test]
    fn test_parse_info_json_truncated_json() {
        // Valid start but truncated — serde_json rejects this as invalid JSON
        let result = DylibNodeEntry::parse_info_json(r#"{"inputs":["a"#);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("JSON"), "unexpected error: {}", err);
    }

    #[test]
    fn test_parse_info_json_unicode() {
        // Unicode characters in port names — must not panic
        let json = r#"{"inputs":["入力"],"outputs":["出力"]}"#;
        let result = DylibNodeEntry::parse_info_json(json);
        let info = result.expect("unicode JSON should parse");
        assert_eq!(info.input_names, vec!["入力"]);
        assert_eq!(info.output_names, vec!["出力"]);
    }

    #[test]
    fn test_parse_info_json_missing_inputs_outputs() {
        // Empty object — inputs/outputs should default to empty
        let json = r#"{}"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        assert!(info.input_names.is_empty());
        assert!(info.output_names.is_empty());
    }

    #[test]
    fn test_parse_info_json_ignores_extra_fields() {
        // Old-format JSON with `node_type` should still parse — the field is
        // simply ignored by the new schema (forward-compat).
        let json = r#"{"node_type":"legacy","inputs":["a"],"outputs":["b"]}"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        assert_eq!(info.input_names, vec!["a"]);
        assert_eq!(info.output_names, vec!["b"]);
    }

    // ============================================================
    // The per-output `wire_fixed_size` — the layout fact the
    // RECORDER stamps into a bag channel's `SchemaDescriptor`, carried
    // from the node instead of from the workspace `schemas/` file the
    // graph's `schema:` names.
    //
    // The whole contract is the three-way distinction between "the node
    // says N", "the node says ZERO" and "the node says NOTHING". Hand
    // oracles, never a round-trip against the emitter: this is the READ
    // half, and the emitter half is pinned in `cerulion_macros`.
    // ============================================================

    #[test]
    #[tracing_test::traced_test]
    fn a_declared_output_wire_fixed_size_reaches_the_meta() {
        let json = r#"{"inputs":[],"outputs":[{"name":"o","schema_hash":7,"max_slice_len_default":null,"promise_within_ms":null,"wire_fixed_size":1048576}]}"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        assert_eq!(
            info.output_meta()[0].wire_fixed_size,
            Some(1_048_576),
            "the node's declared fixed section must reach the recorder verbatim"
        );
        // The anti-tautology half of the over-range arm below: a REPRESENTABLE
        // size says nothing. Without it, hoisting that warn out of its
        // `or_else` would fire on every healthy port and no test would notice.
        logs_assert(|lines: &[&str]| {
            if lines
                .iter()
                .any(|l| l.contains("declared a wire_fixed_size above u32::MAX"))
            {
                Err(format!(
                    "a representable size must be silent:\n{}",
                    lines.join("\n")
                ))
            } else {
                Ok(())
            }
        });
    }

    /// `Some(0)` is a REAL answer, not the unknown sentinel: a purely
    /// VARIABLE schema (`tf2_msgs/TFMessage`) has a genuinely zero-byte
    /// fixed section. Collapsing it to `None` would send the recorder back
    /// to the workspace file for exactly the schemas whose layout the file
    /// is least likely to describe.
    #[test]
    fn a_zero_fixed_section_is_a_claim_not_an_absence() {
        let json = r#"{"inputs":[],"outputs":[{"name":"tf","schema_hash":9,"wire_fixed_size":0}]}"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        assert_eq!(
            info.output_meta()[0].wire_fixed_size,
            Some(0),
            "a variable schema's zero-byte fixed section is a declaration, not a miss"
        );
    }

    /// The ABI was deliberately NOT bumped for this key, so a macro cdylib
    /// built before it existed — and every raw-FFI info block — must still
    /// load and simply declare nothing. That is the "or when no producer
    /// metadata exists" arm the recorder falls back on.
    #[test]
    fn an_output_without_the_key_declares_no_size() {
        let full = r#"{"inputs":[],"outputs":[{"name":"o","schema_hash":7,"max_slice_len_default":64,"promise_within_ms":null}]}"#;
        let info = DylibNodeEntry::parse_info_json(full).unwrap();
        assert_eq!(
            info.output_meta()[0].wire_fixed_size,
            None,
            "a Full output object without the key declares no size"
        );
        assert_eq!(
            info.output_meta()[0].schema_hash,
            7,
            "and every other key it does carry is unaffected"
        );

        let bare = r#"{"inputs":[],"outputs":["o"]}"#;
        let info = DylibNodeEntry::parse_info_json(bare).unwrap();
        assert_eq!(
            info.output_meta()[0].wire_fixed_size,
            None,
            "the legacy bare-string output form declares no size either"
        );
    }

    /// No Cerulion frame can carry a fixed section above `u32::MAX`
    /// (`WireHeader::total_size` is a `u32`), so such a declaration is
    /// dropped to "no claim" — never truncated into a confidently wrong
    /// descriptor, and never a parse failure (which, under `untagged`,
    /// would fail the WHOLE port entry rather than one key).
    #[test]
    #[tracing_test::traced_test]
    fn an_unrepresentable_size_is_dropped_not_truncated() {
        let over = u64::from(u32::MAX) + 1;
        let json = format!(
            r#"{{"inputs":[],"outputs":[{{"name":"o","schema_hash":7,"wire_fixed_size":{over}}}]}}"#
        );
        let info = DylibNodeEntry::parse_info_json(&json).unwrap();
        assert_eq!(
            info.output_meta()[0].wire_fixed_size,
            None,
            "an out-of-range size declares nothing; a truncated 0 would be a lie"
        );
        assert_eq!(
            info.output_meta()[0].name,
            "o",
            "and the rest of the port entry still parses"
        );
        // The drop is an ADMISSION, not a vanishing: match the LEVEL TOKEN
        // as well as the message, and read `raw_value` as a whole
        // whitespace token — the message names `u32::MAX` in prose, so a
        // substring match would be satisfied by the wording alone.
        logs_assert(|lines: &[&str]| {
            let warns: Vec<&&str> = lines
                .iter()
                .filter(|l| {
                    l.split_whitespace().any(|t| t == "WARN")
                        && l.contains("declared a wire_fixed_size above u32::MAX")
                })
                .collect();
            match warns.as_slice() {
                [line]
                    if line
                        .split_whitespace()
                        .any(|t| t == format!("raw_value={over}")) =>
                {
                    Ok(())
                }
                [line] => Err(format!("the warn must carry `raw_value={over}`:\n{line}")),
                other => Err(format!(
                    "expected exactly one WARN naming the over-range size, got {}:\n{}",
                    other.len(),
                    lines.join("\n")
                )),
            }
        });
    }

    /// The builder is the construction surface the macro uses, and it
    /// narrows the trait const's `usize` with the same drop-don't-truncate
    /// rule as the JSON arm. `new()` alone declares nothing, which is what
    /// keeps every hand-written `NodeEntry` impl claim-free by default.
    #[test]
    #[tracing_test::traced_test]
    fn the_builder_narrows_usize_and_defaults_to_no_claim() {
        let plain = OutputMeta::new("o".to_string(), 0xABCD, None);
        assert_eq!(plain.wire_fixed_size, None, "`new` declares no size");

        let sized = OutputMeta::new("o".to_string(), 0xABCD, None).with_wire_fixed_size(64);
        assert_eq!(sized.wire_fixed_size, Some(64));

        let variable = OutputMeta::new("o".to_string(), 0xABCD, None).with_wire_fixed_size(0);
        assert_eq!(
            variable.wire_fixed_size,
            Some(0),
            "zero through the builder is a claim too"
        );

        // The BOUNDARY itself, both sides. Jumping straight to `u32::MAX + 1`
        // would leave a `>` / `>=` slip in a hand-rolled bound (replacing
        // `try_from`) invisible.
        let at_ceiling =
            OutputMeta::new("o".to_string(), 0xABCD, None).with_wire_fixed_size(u32::MAX as usize);
        assert_eq!(
            at_ceiling.wire_fixed_size,
            Some(u32::MAX),
            "u32::MAX is representable and must be kept"
        );

        // Everything so far is a VALID declaration, so nothing may have been
        // said yet. Asserted before the over-range call, so this arm cannot be
        // satisfied by a warn that has not happened yet — and it is what kills
        // a builder that reports unconditionally.
        logs_assert(|lines: &[&str]| {
            if lines
                .iter()
                .any(|l| l.contains("declared a wire_fixed_size above u32::MAX"))
            {
                Err(format!(
                    "a representable size must be silent:\n{}",
                    lines.join("\n")
                ))
            } else {
                Ok(())
            }
        });

        if usize::BITS > 32 {
            let over = usize::try_from(u64::from(u32::MAX) + 1).expect("64-bit usize");
            let dropped =
                OutputMeta::new("named".to_string(), 0x1234, None).with_wire_fixed_size(over);
            assert_eq!(
                dropped.wire_fixed_size, None,
                "an unrepresentable size is dropped, never truncated"
            );
            // The drop is an ADMISSION. This is the SHIPPING path — every
            // in-process macro node builds its `OutputMeta` here, while the
            // JSON twin is reachable only from a hand-written info block — so
            // a silent drop here is the one that would matter. Level token +
            // whole-whitespace-token fields, since the message names
            // `u32::MAX` in prose.
            logs_assert(|lines: &[&str]| {
                let warns: Vec<&&str> = lines
                    .iter()
                    .filter(|l| {
                        l.split_whitespace().any(|t| t == "WARN")
                            && l.contains("declared a wire_fixed_size above u32::MAX")
                    })
                    .collect();
                let [line] = warns.as_slice() else {
                    return Err(format!(
                        "expected exactly one WARN for the dropped size, got {}:\n{}",
                        warns.len(),
                        lines.join("\n")
                    ));
                };
                for kv in [
                    format!("raw_value={over}"),
                    "output=named".to_string(),
                    "schema_hash=4660".to_string(),
                ] {
                    if !line.split_whitespace().any(|t| t == kv) {
                        return Err(format!("the warn must carry `{kv}`:\n{line}"));
                    }
                }
                Ok(())
            });
        }
    }

    /// The key is bound by its EXACT name, not by a prefix or a plural: a
    /// near-miss must land in the unknown-key warn's lap (which
    /// `info_json_key_parity_test` pins against `LEGAL_OUTPUT_KEYS`) and
    /// declare nothing, rather than silently supplying the recorder a size
    /// nobody meant. Paired with the positive arm above so a resolver that
    /// accepted everything would fail one of the two.
    #[test]
    fn a_near_miss_key_declares_nothing() {
        let json =
            r#"{"inputs":[],"outputs":[{"name":"o","schema_hash":7,"wire_fixed_sizes":64}]}"#;
        let info =
            DylibNodeEntry::parse_info_json(json).expect("an unknown key never fails a load");
        assert_eq!(
            info.output_meta()[0].wire_fixed_size,
            None,
            "`wire_fixed_sizes` is not `wire_fixed_size`"
        );
    }

    #[test]
    fn test_backpressure_policy_equality() {
        assert_eq!(
            BackpressurePolicy::DropOldest,
            BackpressurePolicy::DropOldest
        );
        assert_eq!(
            BackpressurePolicy::Sample(100),
            BackpressurePolicy::Sample(100)
        );
        assert_ne!(
            BackpressurePolicy::Sample(100),
            BackpressurePolicy::Sample(200)
        );
        assert_ne!(BackpressurePolicy::Block, BackpressurePolicy::DropOldest);
    }

    #[test]
    fn test_input_meta_construction() {
        let meta = InputMeta {
            name: "scan".to_string(),
            schema_hash: 0x1234,
            trigger: true,
            depth: 1,
            backpressure: BackpressurePolicy::DropOldest,
            expect_within_ms: None,
        };
        assert!(meta.trigger);
        assert_eq!(meta.depth, 1);
        assert_eq!(meta.backpressure, BackpressurePolicy::DropOldest);
    }

    #[test]
    fn test_output_meta_construction() {
        let meta = OutputMeta {
            name: "cmd_vel".to_string(),
            schema_hash: 0x5678,
            max_slice_len_default: None,
            promise_within_ms: None,
            wire_fixed_size: None,
        };
        assert_eq!(meta.name, "cmd_vel");
        assert_eq!(meta.schema_hash, 0x5678);
        assert_eq!(meta.max_slice_len_default, None);
    }

    #[test]
    fn test_node_info_from_names() {
        let info = NodeInfo::from_names(vec!["trigger".to_string()], vec!["image".to_string()]);
        assert_eq!(info.input_names, vec!["trigger"]);
        assert_eq!(info.output_names, vec!["image"]);
        assert!(info.input_meta.is_empty());
        assert!(info.output_meta.is_empty());
    }

    /// Construction-time half of the
    /// one-meta-per-port invariant (`GraphTopology::build` re-checks as
    /// defense-in-depth for in-crate literal construction). The panic
    /// message names the offending port.
    #[test]
    #[should_panic(expected = "NodeInfo::with_meta: duplicate input port name 'scan'")]
    fn with_meta_panics_on_duplicate_input_names() {
        let meta = |bp: BackpressurePolicy, depth: usize| InputMeta {
            name: "scan".to_string(),
            schema_hash: 0x1234,
            trigger: false,
            depth,
            backpressure: bp,
            expect_within_ms: None,
        };
        let _ = NodeInfo::with_meta(
            vec![
                meta(BackpressurePolicy::DropOldest, 4),
                meta(BackpressurePolicy::Block, 2), // conflicting duplicate
            ],
            Vec::new(),
        );
    }

    /// False-positive guard for the construction assert: two DISTINCT
    /// input metas construct fine (evaluates the scan predicate at idx 1,
    /// catching an always-true predicate co-located with the assert).
    #[test]
    fn with_meta_accepts_distinct_input_names() {
        let meta = |name: &str| InputMeta {
            name: name.to_string(),
            schema_hash: 0x1234,
            trigger: false,
            depth: 4,
            backpressure: BackpressurePolicy::DropOldest,
            expect_within_ms: None,
        };
        let info = NodeInfo::with_meta(vec![meta("scan"), meta("imu")], Vec::new());
        assert_eq!(info.input_names, vec!["scan", "imu"]);
    }

    /// The output-side twin — duplicate
    /// `OutputMeta` names would silently last-win in
    /// `build_output_meta_lookup` (conflicting schema_hash /
    /// max_slice_len), the same ambiguous-conflict class rejected for
    /// inputs.
    #[test]
    #[should_panic(expected = "NodeInfo::with_meta: duplicate output port name 'cmd_vel'")]
    fn with_meta_panics_on_duplicate_output_names() {
        let meta = |hash: u64| OutputMeta {
            name: "cmd_vel".to_string(),
            schema_hash: hash,
            max_slice_len_default: None,
            promise_within_ms: None,
            wire_fixed_size: None,
        };
        let _ = NodeInfo::with_meta(Vec::new(), vec![meta(0x1), meta(0x2)]);
    }

    /// The declarative-mode constructor enforces
    /// the same invariant (it is the FFI-adjacent constructor —
    /// `parse_info_json` pre-validates so FFI data cannot reach this
    /// panic; direct embedder calls can).
    #[test]
    #[should_panic(
        expected = "NodeInfo::with_input_names_and_output_meta: duplicate output port name 'cmd_vel'"
    )]
    fn with_input_names_and_output_meta_panics_on_duplicate_outputs() {
        let meta = |hash: u64| OutputMeta {
            name: "cmd_vel".to_string(),
            schema_hash: hash,
            max_slice_len_default: None,
            promise_within_ms: None,
            wire_fixed_size: None,
        };
        let _ = NodeInfo::with_input_names_and_output_meta(
            vec!["in".to_string()],
            vec![meta(0x1), meta(0x2)],
        );
    }

    #[test]
    #[should_panic(
        expected = "NodeInfo::with_input_names_and_output_meta: duplicate input port name 'in'"
    )]
    fn with_input_names_and_output_meta_panics_on_duplicate_inputs() {
        let _ = NodeInfo::with_input_names_and_output_meta(
            vec!["in".to_string(), "in".to_string()],
            Vec::new(),
        );
    }

    /// The FFI ingress rejects duplicates with a
    /// structured `Err` (caught by `info()`'s loud fallback) instead of
    /// reaching the constructor panic — a buggy cdylib must not abort the
    /// host. Distinct names still parse (false-positive guard).
    #[test]
    fn parse_info_json_rejects_duplicate_port_names() {
        let dup_inputs = r#"{"inputs": ["a", "a"], "outputs": []}"#;
        let reason = DylibNodeEntry::parse_info_json(dup_inputs)
            .expect_err("duplicate input names from FFI must be rejected")
            .to_string();
        assert!(
            reason.contains("duplicate input port name 'a'"),
            "got: {reason}"
        );
        let dup_outputs = r#"{"inputs": [], "outputs": ["out", "out"]}"#;
        let reason = DylibNodeEntry::parse_info_json(dup_outputs)
            .expect_err("duplicate output names from FFI must be rejected")
            .to_string();
        assert!(
            reason.contains("duplicate output port name 'out'"),
            "got: {reason}"
        );
        let distinct = r#"{"inputs": ["a", "b"], "outputs": ["x", "y"]}"#;
        let info = DylibNodeEntry::parse_info_json(distinct).expect("distinct names parse fine");
        assert_eq!(info.input_names, vec!["a", "b"]);
        assert_eq!(info.output_names, vec!["x", "y"]);
    }

    #[test]
    fn test_node_info_with_declarative_meta() {
        let info = NodeInfo {
            input_names: vec!["scan".to_string()],
            output_names: vec!["cmd_vel".to_string()],
            input_meta: vec![InputMeta {
                name: "scan".to_string(),
                schema_hash: 0x1234,
                trigger: true,
                depth: 1,
                backpressure: BackpressurePolicy::DropOldest,
                expect_within_ms: None,
            }],
            output_meta: vec![OutputMeta {
                name: "cmd_vel".to_string(),
                schema_hash: 0x5678,
                max_slice_len_default: None,
                promise_within_ms: None,
                wire_fixed_size: None,
            }],
            policy: None,
            tick_within_ms: None,
            throttle_ms: None,
        };
        assert_eq!(info.input_meta.len(), 1);
        assert!(info.input_meta[0].trigger);
        assert_eq!(info.output_meta.len(), 1);
    }

    // ---- safe_utf8_prefix oracle tests ----

    /// Multi-byte char straddling the 80-byte boundary — naive
    /// `&s[..80]` would panic; the helper walks back to char boundary.
    /// This test calls the production helper directly (no
    /// re-implementation of the truncation loop in the test body).
    #[test]
    fn safe_utf8_prefix_walks_to_char_boundary() {
        // 26 × 3-byte UTF-8 char (Japanese 'あ' = 0xE3 0x81 0x82) = 78 bytes.
        let mut payload: String = "あ".repeat(26);
        // Push another 3-byte char so bytes 79-81 belong to a single char.
        // Naive `&s[..80]` would panic at byte 80 (mid-codepoint).
        payload.push('い');
        let prefix = safe_utf8_prefix(&payload, 80);
        // Oracle: walks back from byte 80 to the boundary at byte 78
        // (where the 27th char starts), so the prefix is the first 26 'あ's.
        assert_eq!(
            prefix,
            "あ".repeat(26).as_str(),
            "prefix should walk back to char boundary at byte 78"
        );
        assert_eq!(prefix.len(), 78);
    }

    /// Edge: input shorter than max_bytes returns whole input.
    #[test]
    fn safe_utf8_prefix_short_input_returns_whole() {
        assert_eq!(safe_utf8_prefix("hi", 80), "hi");
    }

    /// Edge: empty input returns empty.
    #[test]
    fn safe_utf8_prefix_empty_input() {
        assert_eq!(safe_utf8_prefix("", 80), "");
    }

    /// Edge: max_bytes=0 returns empty.
    #[test]
    fn safe_utf8_prefix_zero_max_returns_empty() {
        assert_eq!(safe_utf8_prefix("anything", 0), "");
    }

    /// Edge: input length exactly equals max_bytes — returns whole.
    #[test]
    fn safe_utf8_prefix_exact_length_match() {
        let s = "あ".repeat(26); // 78 bytes
        assert_eq!(safe_utf8_prefix(&s, 78), s.as_str());
    }

    /// Edge: ASCII-only input — every byte is a char boundary, so
    /// truncation lands exactly at max_bytes.
    #[test]
    fn safe_utf8_prefix_ascii_truncates_at_exact_byte() {
        let s = "a".repeat(200);
        let prefix = safe_utf8_prefix(&s, 80);
        assert_eq!(prefix.len(), 80);
        assert_eq!(prefix, "a".repeat(80).as_str());
    }

    /// Edge: input where bytes 0..max_bytes is a single 4-byte
    /// codepoint cluster — helper must walk back without underflow.
    #[test]
    fn safe_utf8_prefix_starts_with_4byte_codepoint() {
        // U+1F600 'GRINNING FACE' = 0xF0 0x9F 0x98 0x80 (4 bytes)
        let s = "\u{1F600}rest"; // 4 + 4 = 8 bytes
                                 // max_bytes=2 lands mid-emoji at byte 2; walks back to 0.
        assert_eq!(safe_utf8_prefix(s, 2), "");
        // max_bytes=4 lands exactly at the emoji boundary; full emoji.
        assert_eq!(safe_utf8_prefix(s, 4), "\u{1F600}");
        // max_bytes=5 lands at a boundary (byte 5 is between 'r' and 'e').
        assert_eq!(safe_utf8_prefix(s, 5), "\u{1F600}r");
    }

    /// End-to-end: after a `parse_info_json` failure, the production
    /// `info()` path constructs `TransportError::NodeInfoParse { prefix:
    /// safe_utf8_prefix(&json_str, 80).to_string(), .. }` (and emits one
    /// `tracing::error!` carrying the same prefix). This test exercises
    /// the SAME helper that populates that `prefix` field so a
    /// regression in `safe_utf8_prefix` (returning a non-char-boundary
    /// slice) fires here — no tautological re-implementation in the test
    /// body.
    #[test]
    fn safe_utf8_prefix_used_by_info_error_does_not_panic_on_truncated_multibyte() {
        // 27 × 3-byte char + invalid trailing JSON. 81 bytes of multi-byte
        // chars before any ASCII; cutoff at 80 lands mid-codepoint.
        let payload = format!("{}{}", "あ".repeat(27), "invalid");
        // `info()` does
        //   let prefix = safe_utf8_prefix(&json_str, 80).to_string();
        //   ... NodeInfoParse { prefix, .. }
        // which is what we exercise here. If this panics, the production
        // error path panics too — and an operator loses the diagnostic.
        let prefix = safe_utf8_prefix(&payload, 80);
        assert!(prefix.len() <= 80);
        assert!(payload.is_char_boundary(prefix.len()));
        // Also confirm parse_info_json fails on this payload (so the
        // error-path is the one being exercised).
        let parse_result = DylibNodeEntry::parse_info_json(&payload);
        assert!(parse_result.is_err());
    }

    // ---- take_last_error UTF-8 lossy recovery ----

    /// A `take_last_error` built on `.to_str().ok().map(str::to_owned)`
    /// returns `None` on invalid UTF-8 — the actual diagnostic
    /// message vanishes and the upstream caller falls back to a generic
    /// "no detail provided" label. This one uses `from_utf8_lossy`, which
    /// substitutes U+FFFD and returns Some(_).
    #[test]
    fn cstr_to_lossy_string_recovers_invalid_utf8_prefix_bytes() {
        // 0xFF and 0xFE are never valid in any UTF-8 sequence.
        let bytes = [b'b', b'a', b'd', 0xFFu8, 0xFEu8, b'!', 0x00];
        let cstr = CStr::from_bytes_with_nul(&bytes).expect("valid C string");
        let recovered = cstr_to_lossy_string(cstr);
        assert!(
            recovered.contains('\u{FFFD}'),
            "expected U+FFFD substitution, got: {:?}",
            recovered
        );
        assert!(recovered.starts_with("bad"), "got: {:?}", recovered);
        assert!(recovered.ends_with('!'), "got: {:?}", recovered);
    }

    /// Adversarial: lone UTF-8 continuation byte (0x80) — most common
    /// "I corrupted my buffer" failure mode in real C code.
    #[test]
    fn cstr_to_lossy_string_recovers_lone_continuation_byte() {
        let bytes = [b'm', b's', b'g', 0x80u8, b'.', 0x00];
        let cstr = CStr::from_bytes_with_nul(&bytes).expect("valid C string");
        let recovered = cstr_to_lossy_string(cstr);
        assert!(recovered.contains('\u{FFFD}'), "got: {:?}", recovered);
        assert!(recovered.starts_with("msg"), "got: {:?}", recovered);
        assert!(recovered.ends_with('.'), "got: {:?}", recovered);
    }

    /// Adversarial: truncated multi-byte sequence — the cdylib started
    /// to encode a 3-byte char but the buffer ended early.
    /// 0xE3 0x81 starts the 'あ' codepoint (which needs 0xE3 0x81 0x82).
    #[test]
    fn cstr_to_lossy_string_recovers_truncated_multibyte() {
        let bytes = [b'a', 0xE3u8, 0x81u8, b'b', 0x00];
        let cstr = CStr::from_bytes_with_nul(&bytes).expect("valid C string");
        let recovered = cstr_to_lossy_string(cstr);
        assert!(recovered.contains('\u{FFFD}'), "got: {:?}", recovered);
        assert!(recovered.starts_with('a'), "got: {:?}", recovered);
        assert!(recovered.ends_with('b'), "got: {:?}", recovered);
    }

    /// Adversarial: multiple invalid regions in one message —
    /// lossy conversion produces multiple FFFD substitutions.
    #[test]
    fn cstr_to_lossy_string_handles_multiple_invalid_regions() {
        let bytes = [b'x', 0xFFu8, b'y', 0xFEu8, b'z', 0x00];
        let cstr = CStr::from_bytes_with_nul(&bytes).expect("valid C string");
        let recovered = cstr_to_lossy_string(cstr);
        // Each invalid byte in isolation produces one U+FFFD.
        assert_eq!(
            recovered.matches('\u{FFFD}').count(),
            2,
            "expected 2 replacement chars; got: {:?}",
            recovered
        );
        assert!(recovered.starts_with('x'), "got: {:?}", recovered);
        assert!(recovered.ends_with('z'), "got: {:?}", recovered);
    }

    /// Adversarial: all-invalid bytes — no valid char survives, but
    /// the function still returns a non-empty string of replacement chars.
    #[test]
    fn cstr_to_lossy_string_handles_all_invalid_bytes() {
        let bytes = [0xFFu8, 0xFEu8, 0xFDu8, 0x00];
        let cstr = CStr::from_bytes_with_nul(&bytes).expect("valid C string");
        let recovered = cstr_to_lossy_string(cstr);
        assert!(!recovered.is_empty(), "got empty string");
        assert!(
            recovered.chars().all(|c| c == '\u{FFFD}'),
            "expected all replacement chars; got: {:?}",
            recovered
        );
    }

    /// Round-trip: a fully-valid UTF-8 message survives unchanged.
    #[test]
    fn cstr_to_lossy_string_preserves_valid_utf8() {
        let cstr = c"node failed: simulated";
        assert_eq!(cstr_to_lossy_string(cstr), "node failed: simulated");
    }

    /// Round-trip: valid multi-byte UTF-8 survives unchanged.
    #[test]
    fn cstr_to_lossy_string_preserves_valid_multibyte_utf8() {
        let bytes = [
            b'p', b'r', b'e', 0xE3u8, 0x81u8, 0x82u8, // 'あ'
            b's', b'u', b'f', 0x00,
        ];
        let cstr = CStr::from_bytes_with_nul(&bytes).expect("valid C string");
        let recovered = cstr_to_lossy_string(cstr);
        assert_eq!(recovered, "preあsuf");
    }

    /// Empty C string ("\0") yields empty Rust String.
    #[test]
    fn cstr_to_lossy_string_empty() {
        assert_eq!(cstr_to_lossy_string(c""), "");
    }

    // -----------------------------------------------------------------
    // MacroPolicy::DataTrigger parse round-trip + adversarial
    // -----------------------------------------------------------------

    #[test]
    fn test_parse_info_json_data_trigger_round_trip() {
        // Happy path: nested-object shape produces
        // `MacroPolicy::DataTrigger { input_name }`.
        let json = r#"{
            "inputs": ["count"],
            "outputs": [],
            "policy": {"data_trigger": {"input_name": "count"}}
        }"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        match info.policy {
            Some(MacroPolicy::DataTrigger { input_name }) => {
                assert_eq!(input_name, "count");
            }
            other => panic!("expected DataTrigger, got: {other:?}"),
        }
    }

    #[test]
    fn test_parse_info_json_data_trigger_typo_in_payload_is_rejected() {
        // Adversarial: `deny_unknown_fields` on `DataTriggerPayload`
        // makes a typo (e.g. `input` instead of `input_name`)
        // loud-fail at parse time. Without this gate, a typo'd cdylib
        // would silently produce `policy: None` and the runtime would
        // fall into the (None, None) default-Data arm.
        let json = r#"{
            "inputs": ["count"],
            "outputs": [],
            "policy": {"data_trigger": {"input": "count"}}
        }"#;
        let result = DylibNodeEntry::parse_info_json(json);
        assert!(result.is_err(), "typo'd field must reject; got Ok");
    }

    #[test]
    fn test_parse_info_json_data_trigger_missing_input_name_is_rejected() {
        // Adversarial: empty payload (no `input_name`) — required
        // String field, serde rejects.
        let json = r#"{
            "inputs": ["count"],
            "outputs": [],
            "policy": {"data_trigger": {}}
        }"#;
        let result = DylibNodeEntry::parse_info_json(json);
        assert!(result.is_err(), "missing input_name must reject; got Ok");
    }

    #[test]
    fn test_parse_info_json_data_trigger_with_period_loses_to_period() {
        // Adversarial: a malformed cdylib that emitted BOTH
        // `period_ms` AND `data_trigger` would have its policy
        // resolved by match-arm priority. The match arms are listed
        // in (period, deadline, sync, data_trigger, external) order,
        // so period wins. This is defensive: the macro-side
        // validator already rejects `period_ms` + `#[input(trigger)]`
        // at compile time, so a real macro can't produce this combo.
        // If the cdylib is hand-rolled or corrupted, we don't crash
        // — we pick the first variant we see.
        let json = r#"{
            "inputs": ["count"],
            "outputs": [],
            "policy": {"period_ms": 10, "data_trigger": {"input_name": "count"}}
        }"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        assert!(
            matches!(info.policy, Some(MacroPolicy::Period { period_ms: 10 })),
            "period must win when both keys present; got {:?}",
            info.policy
        );
    }

    #[test]
    fn test_parse_info_json_pre_a_legacy_no_data_trigger_field_is_ok() {
        // Forward-compat: pre-(a) cdylib JSON has no `data_trigger`
        // key. `#[serde(default)]` on the field produces `None`,
        // and the four pre-(a) variants still parse normally.
        let json = r#"{
            "inputs": [],
            "outputs": [],
            "policy": {"period_ms": 16}
        }"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        assert!(matches!(
            info.policy,
            Some(MacroPolicy::Period { period_ms: 16 })
        ));
    }

    #[test]
    fn test_parse_info_json_data_trigger_null_falls_through_to_none() {
        // Defensive: a buggy cdylib that emits explicit `null` for the
        // payload (`"data_trigger": null`) parses as
        // `Option<DataTriggerPayload>::None` per serde's standard
        // null-as-None semantics. The post-(a) runtime treats this as
        // "no data-trigger declared", which then falls through to the
        // `(None, None)` default-Data-trigger arm IF there are no
        // other policy keys present. The policy validator catches
        // the silent never-fire shape if `#[input(trigger)]` is also
        // declared, so the worst-case outcome is loud-fail at
        // build-time. No silent runtime failure.
        let json = r#"{
            "inputs": [],
            "outputs": [],
            "policy": {"data_trigger": null}
        }"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        assert!(
            info.policy.is_none(),
            "explicit null payload must produce policy=None, got: {:?}",
            info.policy
        );
    }

    #[test]
    fn test_parse_info_json_top_level_policy_null_is_none() {
        // Same defense for the OUTER `policy: null`. Standard
        // `Option<PolicyJson>` + `#[serde(default)]` semantics → None.
        let json = r#"{
            "inputs": [],
            "outputs": [],
            "policy": null
        }"#;
        let info = DylibNodeEntry::parse_info_json(json).unwrap();
        assert!(info.policy.is_none(), "top-level null policy → None");
    }

    #[test]
    fn test_parse_info_json_rejects_removed_deadline_ms_policy() {
        // The node-level `deadline_ms` trigger was
        // removed, so `PolicyJson` no longer carries a `deadline_ms`
        // field. With `#[serde(deny_unknown_fields)]`, an info-JSON
        // carrying the old `{"policy":{"deadline_ms":N}}` shape is
        // rejected at parse time — pinning that a stale or hand-crafted
        // deadline policy can't silently round-trip back into a
        // (now-nonexistent) trigger. No friendly migration message by
        // design: there are no deployed cdylibs to migrate (pre-prod).
        let json = r#"{
            "inputs": [],
            "outputs": [],
            "policy": { "deadline_ms": 100 }
        }"#;
        let err = DylibNodeEntry::parse_info_json(json).expect_err(
            "removed `deadline_ms` policy field must be rejected by deny_unknown_fields",
        );
        let msg = err.to_string();
        assert!(
            msg.contains("deadline_ms"),
            "rejection should name the unknown `deadline_ms` field; got: {msg}"
        );
    }

    #[test]
    fn test_macro_policy_data_trigger_clone_eq() {
        // `DataTrigger` carries a String, so `Clone` is meaningful
        // (vs the four pre-(a) variants where Clone was a Copy
        // memcpy). Equality is derived; pin both contracts.
        let a = MacroPolicy::DataTrigger {
            input_name: "count".to_string(),
        };
        let b = a.clone();
        assert_eq!(a, b);
        let c = MacroPolicy::DataTrigger {
            input_name: "other".to_string(),
        };
        assert_ne!(a, c);
    }

    #[test]
    fn test_node_info_with_policy_data_trigger() {
        // `NodeInfo::with_policy` stores DataTrigger correctly. (The
        // signature takes `MacroPolicy` by value — moves the variant
        // in.) After the call, `policy()` (which now borrows) should
        // return a reference to the same variant.
        let info = NodeInfo::from_names(vec!["count".to_string()], vec![]).with_policy(
            MacroPolicy::DataTrigger {
                input_name: "count".to_string(),
            },
        );
        match info.policy() {
            Some(MacroPolicy::DataTrigger { input_name }) => {
                assert_eq!(input_name, "count");
            }
            other => panic!("expected DataTrigger via policy(), got: {other:?}"),
        }
    }

    // =============================================================
    // `parse_info_json` narrows
    // `max_slice_len_default: Option<usize>` to `Option<MaxSliceLen>`.
    // These tests pin the four-bucket decision:
    //   n == 0                 → drop silently (interpreted as "unset")
    //   1 <= n < 32            → drop with `tracing::warn!`
    //   32 <= n <= u32::MAX    → accept as `Some(MaxSliceLen(n))`
    //   n > u32::MAX           → drop with `tracing::warn!`
    // Without these, a future flip of `<` to `<=` (or removal of the
    // warn branch) ships silently.
    // =============================================================

    fn parse_info_with_msl(msl: serde_json::Value) -> NodeInfo {
        let json = serde_json::json!({
            "inputs": [],
            "outputs": [
                { "name": "out", "schema_hash": 0u64, "max_slice_len_default": msl }
            ]
        });
        DylibNodeEntry::parse_info_json(&json.to_string()).expect("parse_info_json")
    }

    #[test]
    fn parse_info_json_max_slice_len_default_zero_drops_silently() {
        let info = parse_info_with_msl(serde_json::json!(0u64));
        assert_eq!(info.output_meta[0].max_slice_len_default, None);
    }

    #[test]
    fn parse_info_json_max_slice_len_default_below_floor_drops() {
        // 1, 31: positive but `< WireHeader::SIZE` — dropped with warn.
        for n in [1u64, 16, 31] {
            let info = parse_info_with_msl(serde_json::json!(n));
            assert_eq!(
                info.output_meta[0].max_slice_len_default, None,
                "n={n} should drop to None (below WireHeader::SIZE)"
            );
        }
    }

    #[test]
    fn parse_info_json_max_slice_len_default_at_floor_accepted() {
        // 32 = WireHeader::SIZE — smallest legal MaxSliceLen.
        let info = parse_info_with_msl(serde_json::json!(32u64));
        let msl = info.output_meta[0]
            .max_slice_len_default
            .expect("32 must be accepted");
        assert_eq!(msl.get(), 32);
    }

    #[test]
    fn parse_info_json_max_slice_len_default_at_ceiling_accepted() {
        let info = parse_info_with_msl(serde_json::json!(u32::MAX as u64));
        let msl = info.output_meta[0]
            .max_slice_len_default
            .expect("u32::MAX must be accepted");
        assert_eq!(msl.get(), u32::MAX);
    }

    #[cfg(target_pointer_width = "64")]
    #[test]
    fn parse_info_json_max_slice_len_default_above_u32_max_drops() {
        let info = parse_info_with_msl(serde_json::json!((u32::MAX as u64) + 1));
        assert_eq!(info.output_meta[0].max_slice_len_default, None);
    }
}
