// SPDX-License-Identifier: AGPL-3.0-only
//! Cerulion core runtime: zero-copy robotics middleware.
//!
//! This crate is the runtime a Cerulion node links against. A node author
//! touches a small part of it: the [`prelude`], the three node macros this
//! crate re-exports (`#[cerulion_node]` and `#[cerulion_node_impl]` through
//! the prelude, `#[derive(CerulionState)]` through [`state`]), and the message
//! types in the `native_ros2_messages` crate. The rest of the
//! module tree (`graph`, `transport`, `scheduler`, `clock`, `codegen`, ...) is
//! what the `cerulion` CLI drives on a node's behalf. It is public so the CLI
//! and the framework's own tests can reach it, it is not node-author API, and
//! it can change between releases.
//!
//! # Quick start
//!
//! A Cerulion project is a workspace created by the `cerulion` CLI. Each node
//! type is its own crate under `nodes/<type>/`, and all wiring lives in
//! `graphs/<name>.yaml`. You never write a `main`, build a graph in code, or
//! start the runtime yourself:
//!
//! ```text
//! cerulion workspace create my_robot
//! cd my_robot
//! cerulion node create sensor --policy period_ms=100 -o geometry_msgs/Vector3 reading
//! cerulion node build sensor
//! cerulion graph create perception
//! cerulion node stage sensor -g perception
//! cerulion graph run perception
//! ```
//!
//! `nodes/sensor/src/lib.rs` is the whole node: a struct whose ports are
//! fields, and an impl block with a `tick` method. This is the shape
//! `cerulion node create` writes, with the `tick` body filled in:
//!
//! ```rust
//! use cerulion_core::prelude::*;
//! use native_ros2_messages::geometry_msgs::Vector3;
//!
//! #[cerulion_node(period_ms = 100)]
//! #[derive(Default)]
//! struct SensorNode {
//!     #[output]
//!     reading: Vector3,
//!     tick_count: u32,
//! }
//!
//! #[cerulion_node_impl]
//! impl SensorNode {
//!     fn tick(&mut self) -> Result<(), NodeError> {
//!         self.tick_count += 1;
//!         // A fixed field is written by assignment, straight into the
//!         // loaned shared-memory slot. The frame is published when `tick`
//!         // returns `Ok`; a tick that returns `Err` publishes nothing.
//!         self.reading.x = f64::from(self.tick_count);
//!         Ok(())
//!     }
//! }
//! # fn main() {}
//! ```
//!
//! The trigger policy (`period_ms = 100` here) is part of the node type and is
//! set on the macro. The graph file names instances of the type and wires
//! their ports; it carries no policy and no code.

#![deny(dead_code, unused_imports, unused_variables)]
// Enforce docs on the crate's top-level public surface (items in
// lib.rs + the prelude re-exports). The deep module tree is exempted via
// per-module `#[allow(missing_docs)]` (one attribute above each `pub mod`
// below) — scoped on purpose; documenting every internal pub item is out of
// scope here.
#![deny(missing_docs)]
// Principle #12 (logging): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

// The ABI LAYOUT PIN — a checked-in size/align/offset snapshot of
// every struct that crosses the cdylib `init()` FFI, keyed to
// `CERULION_ABI_VERSION`, plus a compile-time exhaustive field-set pin per
// struct. Test-only: it asserts about production layouts, it does not shape
// them. See the module docs for the covered set and the exclusions.
#[cfg(test)]
mod abi_layout;

// Lock-free, SHM-mappable counting barrier — the rendezvous
// state machine for the cross-process DAG-level barrier. SHM page mapping,
// doorbell wake, and scheduler integration are not wired yet; `pub`
// (mirroring `doorbell`/`monitor_wait`) so the as-yet-unwired surface is
// dead-code-exempt under `dead_code = deny`. See module doc.
#[allow(missing_docs)]
pub mod barrier;
#[allow(missing_docs)]
pub mod clock;
#[allow(missing_docs)]
pub mod codegen;
pub mod dynamic;
// The cross-process `block`-edge CREDIT word — one SHM word
// per split block edge, so a producer and consumer in different process groups
// can share the backpressure gate. `pub` (mirroring
// `barrier`/`doorbell`/`monitor_wait`) because the multi-process supervisor in
// `cerulion_cli_engine` creates and opens the words. See
// module doc.
pub mod credit;
#[allow(missing_docs)]
pub mod dma_lock;
// Per-topic SHM cache-line doorbell (publisher rings; consumer
// monitor-waits). Not wired into the runtime/publisher yet; `pub`
// (mirroring `monitor_wait`) so the as-yet-unwired surface is dead-code-exempt
// under `dead_code = deny`. See module doc.
#[allow(missing_docs)]
pub mod doorbell;
#[allow(missing_docs)]
pub mod error;
#[allow(missing_docs)]
pub mod graph;
#[allow(missing_docs)]
pub mod iceoryx_logger;
// The ONE `CERULION_*` boolean kill-switch GRAMMAR (`0` disables,
// unset/`1` enable, garbage warns and stays on), shared by the barrier's and
// park's os_sync switches and by the credit plane's two. Platform-NEUTRAL: it
// lived in the macOS-only `os_sync` until the credit wake switch (a Linux-first
// feature) became its first cross-platform consumer. Pure; `os_sync` re-exports
// it for its own two switches.
pub(crate) mod kill_switch;
// The env-gated per-stage latency probe shared by the desk daemons
// (`cerulion-netd`'s ingress + `cerulion-vizd`'s poll/render path). Inert unless
// `CERULION_H264_LAT_PROBE` is set.
pub mod lat_probe;
#[allow(missing_docs)]
pub mod message;
// The PURE per-topic rate/liveness watchdog
// engine. It lived in `cerulion_viz` while its only consumer was the desk
// daemon; the Flashback monitors-verdict trigger gave it a SECOND consumer —
// `cerulion_bagd`, robot-side — and `cerulion_viz` cannot be a dependency of a
// robot-side crate (rerun, openh264, ureq, `rust-version = 1.93`, excluded from
// `default-members`), while a `cerulion_core → cerulion_viz` edge would be a
// cycle. It moves here rather than into a new leaf crate because there is no
// cyclic package edge to break — both consumers already depend on this crate —
// and because `flashback::trigger` and `transport::failure_regime_latch` are the
// standing precedent for a pure policy engine living here and serving several
// crates. `cerulion_viz` re-exports it, so its 25 `cerulion_viz::monitor::…`
// references are byte-unchanged.
//
// Nothing here measures anything: it is a pure consumer of `TopicLiveness`,
// which is why one engine can serve a desk sampling taps and a catalog AND a
// recorder sampling its own tap drains, with ONE set of the liveness /
// rate accuracy rules rather than two copies that can drift apart.
pub mod monitor;
// Shallow CPU monitor-wait primitive for the live loop's inter-step
// idle (root-free alternative to the dma_lock C-state cap). Consumed internally
// by the runtime/doorbell; `pub` (like `barrier` / `doorbell`) so the
// primitive stays dead-code-exempt under `dead_code = deny`. See
// module doc.
#[allow(missing_docs)]
pub mod monitor_wait;
// The SHARED Apple os_sync_* dlsym backend (macOS >= 14.4) —
// one resolution, one unusable-latch, one kill-switch grammar — consumed by the
// barrier's wake tiers and the monitor-wait park's degraded-tier nap.
// pub(crate): every consumer is in-crate; the module is macOS-only.
#[cfg(target_os = "macos")]
pub(crate) mod os_sync;
#[allow(missing_docs)]
pub mod prelude;
pub mod read_outcome;
// The rustc that compiled THIS copy of `cerulion_core`, compared across the
// cdylib FFI boundary by `DylibNodeEntry::load` to catch a toolchain-skew
// niche-encoding mismatch the ABI version cannot see. See the module doc.
pub mod rustc_fingerprint;
#[allow(missing_docs)]
pub mod scheduler;
// The POSIX named-SHM map SUBSTRATE shared by `barrier`, `shm_ring`,
// `state_arm`, `credit` and `wedge_page` — the
// `shm_open`/`ftruncate`-once/`mmap`/`shm_unlink`
// mechanics, written once instead of five times. NOT `#[cfg(unix)]` at the
// module: its FNV name fold is portable (the `barrier`/`credit` name recipes
// that use it compile everywhere), and only the POSIX half below it is gated —
// which is also what lets a portable item name it without tripping
// `cfg_audit_test`. `pub(crate)`: internal plumbing with no out-of-crate
// consumer, and every item has a caller, so `dead_code = deny` is satisfied
// without widening the surface.
pub(crate) mod shm_map;
// The generic SPSC POSIX-SHM byte-record ring (`shm_ring`, a
// later `logd` will reuse it) + the trace-specific record/manifest layer
// (`trace_ring`) driving out-of-process record/replay. Real POSIX SHM on
// macOS AND Linux, hence `#[cfg(unix)]`; `pub` so the not-yet-wired producer surface
// (the scheduler hook lands separately) is dead-code-exempt under `dead_code = deny`.
#[cfg(unix)]
#[allow(missing_docs)]
pub mod shm_ring;
#[allow(missing_docs)]
pub mod shm_runtime;
// The node-state capture core (`CerulionState`, the bounded
// sink, the canonical sorted encoder, the blanket inventory). The derive lives
// in `cerulion_macros` and the carrier in `state_carrier`; `pub` (mirroring
// `barrier` / `doorbell` / `monitor_wait`) because the recorder crates consume
// it. See module doc.
pub mod state;
// The checkpoint CARRIER — the bounded inline attempt, the
// `fork(2)` child that encodes what the arena could not hold, and the parent-side
// machinery (breadcrumb, liveness watchdog, panic hook, MADV_DONTFORK sweep) that
// makes a disposable child safe to have. `fork`/`prctl`/`mmap` throughout, hence
// `#[cfg(unix)]`; `pub` so the surface `take_anchor` is still being wired onto is
// dead-code-exempt under `dead_code = deny`.
#[cfg(unix)]
#[allow(missing_docs)]
pub mod state_carrier;
// The node-state CHECKPOINT layer over `shm_ring` (`state_ring`,
// the sibling of `trace_ring`) + the mapped-SHM arm word a mid-run recorder creates
// to arm it (`state_arm`). Real POSIX SHM on macOS AND Linux, hence `#[cfg(unix)]`;
// `pub` because the recorder crates (`cerulion_bag`, `cerulion_bagd`) consume both
// modules.
#[cfg(unix)]
#[allow(missing_docs)]
pub mod state_arm;
#[cfg(unix)]
#[allow(missing_docs)]
pub mod state_ring;
// The ONE always-on capture plane's POLICY: the
// `CERULION_FLASHBACK` kill switch, the anchor cadence, and the arm-time RAM
// projection that refuses to arm a process too big to checkpoint affordably.
// Pure (it takes numbers and returns decisions), but `#[cfg(unix)]` because
// everything it gates is.
#[cfg(unix)]
pub mod flashback;
// The RESTORE side's pure decisions — anchor framing, the
// rendezvous selection, the shape gate, the backlog admission
// and the publisher-sequence seed ladder.
//
// NOT `#[cfg(unix)]`, deliberately: the CAPTURE side needs `fork(2)` and POSIX
// SHM (hence the four gated modules above), but nothing here touches an OS
// facility — it decides whether a RECORDED bag may be replayed, and the desk
// that replays a bag is not necessarily the machine that recorded it. The one
// type it borrowed from the unix-only `state_ring` (`SkipCause`) now lives in
// the portable `state` module and is re-exported from `state_ring`, so no path
// changed for any existing caller.
//
// The gate mattered because `graph/runtime.rs` names this module
// UNCONDITIONALLY (`restore_node_states`), so the module and its callers
// disagreed about which targets they existed on. MEASURED by configuring the
// four unix modules out and building the crate: with the gate, three extra
// errors at `graph/runtime.rs` 9075/9076/9098; without it, none.
//
// That probe ALSO reported 8 errors naming
// `crate::trace_ring` from `scheduler/mod.rs` + `graph/runtime.rs`, and this
// comment used to conclude from them that the trace ring was "named
// unconditionally" and was "the remaining blocker". That conclusion was
// WRONG, and the probe is why: configuring the modules out leaves the `unix`
// predicate itself TRUE, so a reference that is correctly `#[cfg(unix)]`-gated
// stays compiled while the module it names does not — the probe cannot tell a
// gated reference from an ungated one, and every one of those 8 is gated
// (the gates shipped with the `#[cfg(not(unix))] struct TraceRingHook`
// no-op stub). Re-MEASURED by making the predicate false instead
// (`#[cfg(unix)]` -> never, `#[cfg(not(unix))]` -> always, which configures the
// modules out as a consequence): ZERO trace-ring errors.
//
// What the trace ring DID still break on a non-unix target was the DOCS gate,
// which no `cargo check` probe can see: THREE portable items carried intra-doc
// links into a unix-only module — `scheduler::TraceEntry::discarded` and
// `scheduler::ScheduledNode::discard_signal` (both linking `TRACE_DISCARD_BIT`)
// plus `state::skip_cause`'s module docs (linking `crate::state_ring`).
//
// Exactly ONE of the three is REACHED by rustdoc today, and the distinction is
// worth writing down because it is what the walk buys over the docs gate:
// rustdoc lints links only in items it DOCUMENTS, so with `unix` false the gate
// fails on `TraceEntry::discarded` alone (`pub struct TraceEntry`) — MEASURED,
// reverting all three de-links at once yields exactly 1 unresolved link, at
// `handle.rs:827`. The other two hang off PRIVATE items (`struct ScheduledNode`,
// `mod skip_cause`), so they are LATENT: invisible to `cargo doc` until someone
// writes `pub`, at which point CI's Documentation job breaks on a change that
// touched no link. All three are de-linked, and `cfg_audit_test` flags all
// three regardless of visibility (each revert fails it on
// its own), which is precisely the class a docs run cannot cover.
//
// `pub` so the surface the CLI's replay adapter consumes is dead-code-exempt
// under `dead_code = deny`.
pub mod state_restore;
#[cfg(unix)]
#[allow(missing_docs)]
pub mod trace_ring;
// Always-on; codegen references this from
// non-test-helpers callers (e.g. native_ros2_messages's build.rs codegen).
// See module doc for full rationale.
#[allow(missing_docs)]
pub mod spill_fault_injection;
#[cfg(any(test, feature = "test-helpers"))]
#[allow(missing_docs)]
pub mod testing;
#[allow(missing_docs)]
pub mod trace;
#[allow(missing_docs)]
pub mod transport;
pub mod wake;
// The cross-process WEDGE PAGE — the seq-pair transport a supervisor
// watches to learn a worker entered a tick and never came back (`tick_within_ms`
// times only ticks that RETURN, and the barrier that used to notice a
// never-returning one is deleted by this arc). Real POSIX SHM, hence
// `#[cfg(unix)]`; `pub` because the supervisor half lives in `cerulion_cli_engine`.
#[cfg(unix)]
pub mod wedge_page;
#[allow(missing_docs)]
pub mod wire;

/// ABI version for cdylib node compatibility checking.
///
/// Incremented when the cdylib entry point ABI changes. Dynamic libraries
/// must export `cerulion_abi_version() -> u32` returning this exact value.
///
/// History:
/// - v1: original handle-less ABI.
/// - v2: handle-based init/tick/shutdown.
/// - v3: adds `cerulion_take_last_error` / `cerulion_free_error` plus
///   hidden `__cer_rt: CerNodeRuntimeFields` on every node struct.
///   The new symbols are required by the
///   loader; v2 cdylibs missing them fail to load with a clear
///   message rather than silently bumping into UB.
/// - v4: `cerulion_node_info()` JSON shape changes for `outputs` from
///   bare strings (`["name1", ...]`) to per-output objects
///   (`[{name, schema_hash, max_slice_len_default}, ...]`) so the
///   runtime's tier-2 `max_slice_len` resolution can read each
///   schema's `<T as ShmMessage>::MAX_SLICE_LEN`.
///   v4 hosts STILL accept the legacy strings shape
///   via `serde(untagged)` for graceful upgrade in either direction
///   among ABI v4 cdylibs, but the version bump prevents pre-v4
///   hosts from silently ghost-loading a v4 cdylib (which
///   would otherwise fall back to `NodeInfo::default()` with no
///   wiring).
/// - v5: `cerulion_node_info()` JSON gains an optional
///   `policy.data_trigger.{input_name}` nested-object shape.
///   Pre-v5 hosts that load v5 cdylibs would silently
///   drop the new key and the trigger-input-only node would never
///   fire (the exact never-fire bug that now fails loudly). The version bump
///   loud-fails the cross-version load instead. v5 hosts
///   loading pre-v5 cdylibs are unaffected: the new key is just
///   absent from the JSON, parser sees `None`, behavior is identical
///   to pre-v5.
/// - v6: `cerulion_node_info()` JSON carries the four QoS
///   knobs across the cdylib FFI. `inputs` is promoted from
///   an array of bare strings to an array of OBJECTS
///   (`[{"name":"img","expect_within_ms":20}, {"name":"imu"}]`),
///   symmetric with the existing output objects; each input object
///   carries an optional `expect_within_ms`. Each output object gains
///   an always-present `promise_within_ms` (`null` when unset). The
///   top-level JSON gains optional `tick_within_ms` / `throttle_ms`
///   (emitted only when `Some`, like `policy`). The host parser
///   (`DylibNodeEntry::parse_info_json`) accepts BOTH the new input
///   objects AND the legacy bare-string inputs (via `serde(untagged)`),
///   so the parser is backward-compatible; the ABI bump exists so a
///   pre-v6 host loud-fails rather than silently dropping the four QoS
///   knobs a post-v6 cdylib declares. Raw-FFI (CLI-templated) nodes
///   still emit bare-string inputs and no QoS — they parse fine under
///   the new untagged input enum; closing their QoS gap is out of
///   this bump's scope.
/// - v7: cdylibs must export a new required symbol
///   `cerulion_node_pump_history(handle: u64) -> i32`.
///   The in-process `pump_history` path services quiescent
///   late joiners by re-publishing each node's history on every
///   `live_step`, but it cannot reach a cdylib node across the FFI — the
///   host only holds opaque handles. This symbol lets the loader call
///   `node.pump_history()` over the FFI so cdylib nodes ALSO heal silent
///   late joiners. Mirrors `cerulion_node_tick`'s shape (catch_unwind +
///   NODES lock + handle lookup; codes 2 panic / 3 NODES poisoned / 4
///   handle not found; 0 on success). The loader requires the symbol, so
///   a pre-v7 cdylib (missing it) loud-fails at load rather than
///   silently never servicing its late joiners.
/// - v8: `cerulion_node_info()` input objects carry the declared
///   `#[input(depth = N)]` AND the declared backpressure (one
///   bump covers both). Pre-v8 the host HARDCODED
///   `DEFAULT_CONSUMER_DEPTH` + `DropOldest` for every dylib input —
///   `depth = 32` / `backpressure = block` were real in
///   `build_for_test` runs (in-process `InputMeta`) but silently
///   defaulted in production `cerulion graph run` (dylib path): a
///   test/live divergence in the queue-depth property, and for
///   `block` a Principle-#6-adjacent one (the silent `DropOldest`
///   degrade DROPPED DATA live while tests passed). Both keys are
///   emitted only when explicitly declared (absent = the host-side
///   defaults — the same values the in-process path resolves, so
///   parity holds either way; the depth const stays single-sourced
///   host-side). Backpressure wire shape: `"drop_oldest"` / `"block"`
///   / `{"sample":N}`. STRICT bump: a v7 cdylib is REFUSED loudly at
///   load (rebuild it), never tolerated
///   additively.
/// - v9: `cerulion_node_info()` input objects carry the declared
///   `#[input(trigger)]` mark. Pre-v9 the host HARDCODED
///   `trigger: false` for every dylib input and derived a node's
///   trigger set from `MacroPolicy` alone — but `MacroPolicy::Sync` /
///   `UnboundedSync` carry no input identity (they say "align all
///   trigger inputs" without naming which inputs those are), so a
///   sync node's per-input trigger membership was UNKNOWABLE on the
///   FFI path. The mark now rides the wire (`"trigger":true`, emitted
///   only for a trigger input; absent = a non-trigger latest-value
///   read) so the FFI path sees the same per-input truth as the
///   in-process macro path. Same STRICT bump: a v8 cdylib is REFUSED
///   loudly at load (rebuild it).
/// - v10: `NodeContext` carries the HOST's `TransportManager`
///   (the context-carried transport: `transport` field +
///   `set_transport`/`transport()`; the runtime injects it before
///   `init()` moves the context across the FFI). A cdylib node needing
///   the manager (dds_bridge's raw ingress routes) reads it from its
///   context instead of resolving `TransportManager::get()` — which in a
///   cdylib consults the CDYLIB's OWN copy of the `INSTANCE` static (the
///   cross-linkage-unit global trap: never initialized by the host; a
///   `get_or_init` fallback would mint a SECOND manager on the default
///   SHM root and silently split namespaces). `NodeContext` crosses the
///   `init()` FFI as a raw `Box`, so the added field changes the struct
///   layout both sides must agree on — STRICT bump per the house rule: a
///   v9 cdylib is REFUSED loudly at load (rebuild it), never tolerated.
/// - v11: `cerulion_node_info()` input objects carry the port type's
///   layout hash (`"schema_hash"`). Pre-v11 the host
///   HARDCODED `InputMeta.schema_hash = 0` for every dylib input, so the
///   network ingress-hash resolver refused EVERY cdylib-loaded
///   consumer ("no consumer with a declared schema") — the resolver was
///   structurally dead for the production node form. The macro now emits
///   `<T as ShmMessage>::SCHEMA_HASH` on every input object. STRICT bump
///   (also required for this key): a v10 cdylib is REFUSED loudly
///   at load (rebuild it), never tolerated
///   additively — so on a v11 host an ABSENT key can only mean a raw-FFI
///   info block that omits it (the bare-string input form or a
///   hand-written JSON without the key), which serde-defaults to `0`, the
///   "no declared schema" sentinel the resolver fail-opens on.
/// - v12: `CerulionSubscriber` carries the read-outcome stage
///   handle (`read_stage: Option<Arc<ReadOutcomeStage>>`), set at wiring
///   time on every graph-wired subscriber so a recording run can log which
///   frame each input read served (kind-6 trace records). The `Arc` crossing
///   the FFI is the v10 `TransportManager` precedent; a cdylib-side drain
///   records into the host-allocated stage, which the host-side merge
///   drains.
/// - v13: `CerulionSubscriber` ALSO carries the held-head
///   re-offer streak (`held_head_reoffers` + `held_head_warned`) — the only
///   place the system can distinguish a FIFO head that is DEFERRED (a
///   `throttle_ms` / `block` gate, and will be served) from one no tick will
///   ever read (the collapsed read chain), which matters because the
///   latter re-signals an arrival on every boundary and so suppresses that
///   input's `expect_within_ms` liveliness watchdog indefinitely.
///
///   **This is a SEPARATE number from v12 because v12's LAYOUT WAS ALREADY
///   PUBLISHED.** The two field sets landed in different PRs, and by the time
///   the streak change merged `origin/main`, v12 was the shipped `CerulionSubscriber`
///   WITHOUT the streak fields. Folding the streak into v12 would give ONE
///   version number TWO incompatible layouts: a node cdylib built against
///   main-12 loads into a streak-12 host, the version gate PASSES, and the
///   layout mismatch is exactly the FFI struct hazard (the phantom-abort
///   class) the gate exists to catch loudly. The subscribers live inside
///   `NodeContext`, which crosses the `init()` FFI as a raw `Box`, so the
///   added fields change a struct layout both sides must agree on — STRICT
///   bump per the house rule (the v10 precedent): a v12 cdylib is REFUSED
///   loudly at load (rebuild it), never tolerated. Rebuild every
///   `test_fixtures/*_cdylib`.
/// - v14: `CerulionSubscriber` ALSO carries the SERVICE
///   CURSOR (`served_sequence: Option<Arc<AtomicU64>>`) — the wire `sequence`
///   of the last frame that input actually SERVED to a tick, which a mid-run
///   anchor states so a resumed replay stops re-injecting frames the recorded
///   run had already read.
///
///   Same STRICT-bump reasoning as v13, and for the same structural reason:
///   `NodeContext` crosses the `init()` FFI as a raw `Box` and OWNS the
///   subscribers, so adding a field to `CerulionSubscriber` changes a struct
///   layout both sides must agree on — a v13 cdylib handed a v14 host's
///   context would read every subsequent field at the wrong offset. A v13
///   cdylib is REFUSED loudly at load (rebuild it), never tolerated
///   additively. The write is inside the cdylib's OWN compiled `try_view`,
///   so a host-only change could not have delivered the fact at all.
///
///   **OPERATOR COST: every node crate must be rebuilt against this core.**
/// - v15: `CerulionSubscriber` ALSO carries the per-set Sync STAGED
///   NEXT (`next_head: Option<InboundSample>`) — the second slot of a two-slot
///   FIFO buffer in front of the iceoryx2 queue, which is what lets the matcher
///   learn the stamp of the frame BEHIND an input's head without losing it.
///   Without a slot, learning that stamp means popping, and popping means the
///   frame is gone: the alternative designs either re-read a stamp they cannot
///   re-read or serve the tick a frame the matcher never chose.
///
///   Same STRICT-bump reasoning as v13 and v14, and for the same structural
///   reason: `NodeContext` crosses the `init()` FFI as a raw `Box` and OWNS the
///   subscribers, so a v14 cdylib handed a v15 host's context reads every
///   subsequent field at the wrong offset. The field is unconditional and never
///   `#[cfg]`-gated (the SIGSEGV lesson). The slot is READ and WRITTEN
///   inside the cdylib's OWN compiled `try_view` (the R-pop′ promote-serve), so
///   a host-only change could not have delivered the behaviour at all.
///
///   The feature's other FFI surface — the additive, optional
///   `cerulion_node_sync_head_op` symbol — deliberately does NOT bump on its
///   own (the symbol-presence-is-capability precedent); ONE bump covers
///   the whole feature, because the scheduler-side head map, hints and counters
///   are host-side only.
///
///   **OPERATOR COST: every node crate must be rebuilt against this core.**
/// - v16: [`read_outcome::StagedReadOutcome`] carries the
///   READ-SITE ROLE (`role: ReadSiteRole`) — the call site that performed the
///   read, declared as a compile-time constant at every mint, carried through
///   the stage and packed into the recorded record's meta word.
///
///   Same STRICT-bump reasoning as v13, v14 and v15, and for the same
///   structural reason, one indirection further out: `NodeContext` crosses the
///   `init()` FFI as a raw `Box` and OWNS the subscribers, each of which owns
///   its `Arc<ReadOutcomeStage>` (the v12 field) whose buffer is a
///   `Vec<StagedReadOutcome>`. Adding a field to the STAGED RECORD changes a
///   layout both sides write into: a v15 cdylib's `stage_read_outcome` pushes
///   the old, SHORTER struct into a buffer the v16 host drains at the new
///   stride, so every record after the first is read from the wrong offset and
///   the host mints kind-6 records whose input index, kind and role are all
///   garbage — a corrupt read log rather than an absent one. A v15 cdylib is
///   REFUSED loudly at load (rebuild it), never tolerated additively.
///
///   A host-only change could not have delivered it: the role's whole contract
///   is that the CALL SITE names it, and the call sites that matter here —
///   `drain_for_trigger`, the per-set Sync matcher's pops, the node body's own
///   `try_view` — are compiled INSIDE the cdylib. A host that stamped a role
///   after the fact would be inferring the site, which is exactly the silent
///   inversion the declared constant exists to prevent.
///
///   **OPERATOR COST: every node crate must be rebuilt against this core.**
/// - v17: `TransportManager` — reached by CDYLIB-compiled code through
///   `NodeContext.transport: Option<Arc<TransportManager>>` (the
///   design that put the manager IN the context so a cdylib stops resolving
///   its own `INSTANCE` static; a cdylib method indexes the manager's fields
///   at ITS OWN offsets) — changes layout twice
///   over: `dynamic_egress` is retyped from
///   `Mutex<Option<RegistrationChannel>>` to
///   `Mutex<Option<Arc<RegistrationChannel>>>` so a registration can run
///   OUTSIDE the slot lock (a panic in the iceoryx2 send used to poison the
///   slot and leave every later publisher in the process local-only), which
///   changes the field's size and every later field's offset; and the new
///   `dynamic_egress_create: Mutex<()>` serializes channel CREATION so N
///   racing first registrations open exactly one control publisher against
///   the control service's writer cap.
///
///   Same STRICT-bump reasoning as v13-v16: a v16 cdylib handed a v17 host's
///   context dereferences the shared manager at its own, stale field offsets
///   — exactly the UB class the manager's `FieldSetOnly` pin documents as
///   "the bump-per-transport-field cost is the cost of the design" (and a
///   RETYPE is the shape that pin structurally cannot see: the field set is
///   unchanged, so only this version constant carries the change). A v16
///   cdylib is REFUSED loudly at load (rebuild it), never tolerated
///   additively.
///
///   **OPERATOR COST: every node crate must be rebuilt against this core.**
/// - v18: `CerulionSubscriber` carries the EFFECTIVE borrow
///   budget (`max_borrowed_samples: usize`) — the topic's iceoryx2
///   `subscriber_max_borrowed_samples` read from the data service's static
///   config at creation, so the rmw's adopt-take refusal diagnostics report
///   the budget the subscription is REALLY bounded by (an open against a
///   pre-existing smaller service reads ITS value, not any requested
///   create-leg floor).
///
///   Same STRICT-bump reasoning as v13-v17, and for the same structural
///   reason as v15: `NodeContext` crosses the `init()` FFI as a raw `Box`
///   and OWNS the subscribers (`AnySubscriber` values in its `IndexMap`), so
///   the new `usize` — declared between `fault_inject_receive_after` and
///   `max_publishers` — changes the struct's size and every later field's
///   offset, and a v17 cdylib handed a v18 host's context reads
///   `max_publishers`, `drain_scratch`, the frozen slot, the staged next and
///   every field after them from the wrong place, in the cdylib's OWN
///   compiled `try_view`. A v17 cdylib is REFUSED loudly at load (rebuild
///   it), never tolerated additively. The field is unconditional and never
///   `#[cfg]`-gated (the SIGSEGV lesson). It was caught by the
///   layout pin exactly as designed — the exhaustive destructure in
///   `subscriber.rs` refused to compile against the un-pinned field.
///
///   **OPERATOR COST: every node crate must be rebuilt against this core.**
/// - v19: [`read_outcome::ReadOutcomeStage`] carries the DERIVED
///   staging capacity (`capacity: u32`) — each stage is now sized from its own
///   edge's graph facts (declared queue depth, the multi-publisher/External
///   PAIR factor, the per-set `Sync` PEEK+PROMOTE factor, the consumer's fire
///   bound) by one pure function, replacing the deleted global
///   `READ_OUTCOME_STAGE_CAPACITY`.
///
///   Same STRICT-bump reasoning as v12 and v16, and by the same indirection:
///   `NodeContext` crosses the `init()` FFI as a raw `Box` and OWNS the
///   subscribers, each of which owns its `Arc<ReadOutcomeStage>`. The cdylib
///   compiles its OWN layout of that struct and calls `record()` on a
///   HOST-allocated stage, so a v18 cdylib reads `armed`, `pending` and
///   `inner` at offsets that predate the new field — a CORRUPT capture (a
///   `Mutex` dereferenced at the wrong address), not an absent one. A v18
///   cdylib is REFUSED loudly at load (rebuild it), never tolerated additively.
///
///   A host-only change could not have delivered it either: the rim the
///   capacity governs is checked inside `record`, which the cdylib's own
///   subscribers call.
///
///   **OPERATOR COST: every node crate must be rebuilt against this core.**
///
///   RENUMBERED 18 -> 19 at the rebase onto `main`. Both this
///   change and `main`'s subscriber field were developed in
///   parallel and each bumped 17 -> 18 for a DIFFERENT struct. They cannot
///   share a number: a cdylib built against `main`'s v18 carries
///   `max_borrowed_samples` but NOT this stage field, so loading it into a host
///   that has both reads every later field at the wrong offset — exactly the
///   corruption the version exists to refuse. The merged host is v19 and
///   refuses both v17 and v18 cdylibs.
/// - v20: the `block` backpressure MIRROR is retyped everywhere it is held —
///   `CerulionSubscriber`'s `BlockProbe.outstanding` and `CerulionPublisher`'s
///   `block_outstanding` go from a bare `Arc<AtomicU64>` to
///   [`credit::CreditWord`], the ONE handle that is either a process-local
///   heap word or a cross-process `MappedCredit` SHM page. That is the change
///   that makes a `block` edge whose producer and consumer land in DIFFERENT
///   process groups representable at all.
///
///   ANY RETYPE of a field on an FFI-crossing struct breaks a v19 cdylib,
///   whatever the sizes do — that is the whole reason to lead with it. Host
///   and cdylib compile these definitions TWICE, and each writes the field
///   according to ITS OWN type: a v19 `try_view` decrements `BlockProbe`'s
///   mirror as an `AtomicU64` at the offset IT computed, while the v20 host
///   put a two-variant enum there. `NodeContext` crosses the `init()` FFI as a
///   raw `Box` and OWNS the subscribers, and that drain runs inside the
///   cdylib's OWN compiled code, so the result is a CORRUPTED credit count —
///   the one number the lossless-backpressure guarantee rests on — rather than
///   merely a lost one. Any pre-v20 cdylib is REFUSED loudly at load (rebuild
///   it), never tolerated additively.
///
///   The size change is the ADDITIONAL consequence, and it is narrower than it
///   first looks. `CreditWord` is a two-variant enum over two non-null `Arc`s,
///   so it is 16 bytes where the `Arc<AtomicU64>` it replaces was 8, and
///   `BlockProbe` grows 72 -> 80 with its own later fields moving (the one
///   `abi_layout` row this bump re-snapshots). It stops there: the enclosing
///   `BackpressureProbe` is sized by its LARGEST variant, which is
///   `DropOldestProbe` at 120, so it is unchanged; and `CerulionSubscriber` is
///   pinned `FieldSetOnly`, so no offset of its is asserted at all. The
///   `CerulionPublisher.block_outstanding` retype moves nothing either — a
///   `Vec` is 24 bytes whatever it holds. So most of this change is invisible
///   to BOTH halves of the `abi_layout` pin, and only this version constant
///   carries it: exactly the shape v17 recorded.
///
///   **OPERATOR COST: every node crate must be rebuilt against this core.**
///
///   RENUMBERED 18 -> 20 at the rebase onto `main`. This change
///   was developed in parallel with `main`'s v18 subscriber field and
///   the v19 stage field; all three bumped from 17 for DIFFERENT structs.
///   The number is not carried forward from a branch — it is read off the
///   fetched `main` at rebase time and incremented, which is what keeps two
///   parallel PRs from both announcing the same layout.
/// - v22: cdylibs must export a new required symbol
///   `cerulion_rustc_fingerprint() -> *const c_char`. Same shape as the v7
///   bump: a NEW REQUIRED SYMBOL, not a struct field change.
///
///   `NodeContext` crosses the cdylib `init()` FFI as a raw `Box`, and every
///   `repr(Rust)` type reachable through it (`CerulionSubscriber.frozen:
///   Option<FrozenSlot>` is the one that breaks) is laid out however THAT
///   PARTICULAR rustc's niche-filling pass decides. rustc 1.97.0 changed the
///   bit pattern `Option<T>::None` writes for a niche-holding 4-variant `T`
///   (`0x4` through rustc 1.94.1, then `0xFFFF_FFFF_FFFF_FFFF` on 1.97.0 and
///   later) WITHOUT moving any struct's size or any field's offset, a
///   divergence every ABI bump before this one (v1 through v21, and the
///   `abi_layout` pin they feed) is structurally BLIND to, because that pin
///   only measures `size_of`/`align_of`/`offset_of!`. A host built by one
///   rustc release loading a v21-or-earlier cdylib built by a DIFFERENT rustc
///   release therefore passed the ABI check clean and then read a live
///   `Some(FrozenSlot::Err(..))` where the writer meant `None`:
///   `drain_for_trigger`'s `drop_glue<Option<FrozenSlot>>` freed uninitialised
///   bytes and `libmalloc` called `abort()`, so `graph run` died on SIGABRT
///   with no panic text and no backtrace.
///
///   The new symbol reports the cdylib's `rustc -vV` release and commit hash
///   ([`rustc_fingerprint::RUSTC_FINGERPRINT`]); `DylibNodeEntry::load`
///   REFUSES a cdylib whose fingerprint disagrees with the host's, naming both
///   compilers and the two ways out (rebuild the node with the host's
///   toolchain, or reinstall the host with the node's). The loader requires
///   the symbol, so a pre-v22 cdylib (missing it) loud-fails at load rather
///   than risking the corrupt-`None` class silently, the same contract the v7
///   `cerulion_node_pump_history` bump established for a missing symbol.
///
///   **OPERATOR COST: every node crate must be rebuilt against this core.**
pub const CERULION_ABI_VERSION: u32 = 22;

// Re-export commonly used types
pub use clock::{real_ns, thread_cpu_ns, Clock, ExternalClock, RealClock, VirtualClock};
pub use error::{InertReason, NodeNameRefusal, TransportError, TransportResult};
#[cfg(any(test, feature = "test-helpers"))]
pub use graph::node::ClosureNodeEntry;
pub use graph::node::{
    AnyPublisher, AnySubscriber, DataTriggerPayload, DylibNodeEntry, ExternalSource, MacroPolicy,
    NodeContext, NodeEntry, NodeInfo, PolicyJson, ShutdownSignal,
};
pub use graph::{
    CreditBinding, CreditRole, CrossProcessWiring, CycleError, GraphConfig, GraphRuntime,
    GraphTopology, Level, Levels, NodeDef, TriggerEdges, BARRIER_BOUNDARY_TIMEOUT,
};
pub use message::ShmMessage;
pub use monitor_wait::MonitorWaitPolicy;
pub use rustc_fingerprint::{rustc_fingerprint_cstr, RUSTC_FINGERPRINT, RUSTC_RELEASE};
pub use scheduler::{
    merge_partition_traces, AlignOutcome, NodeConfig, NodeHandle, ProcessTrace, Scheduler,
    SyncHeadOp, SyncOpAnswer, TraceEntry, TriggerPolicy,
};
pub use transport::bridge::{DemandSignal, TopicBridgeManager};
pub use transport::cerulion_q::{
    CatalogEntry, CatalogProvenance, CatalogReply, RunEntry, RunEntryState, RunsCompleteness,
    RunsReply, SchemaDoc, SchemaEncoding, SchemaReply, UndescribableRun, UnusableRunsAnswer,
};
pub use transport::demand_authorizer::{
    AllowAllAuthorizer, DemandAuthorizer, DemandDecision, DemandSubject,
};
pub use transport::discovery::TopicToken;
pub use transport::events::PubSubEvent;
pub use transport::gateway::{
    GatewayDriveStats, GatewayEgressPolicy, GatewayIngressEntry, GatewayPlan, GatewayRuntime,
    SchemaHashName, SchemaServing, SchemaServingHandles, TopicSchema, GATEWAY_ZERO_DEMAND_IDLE,
};
pub use transport::input_view::InputView;
pub use transport::liveness::{
    DrainObservation, LivenessRecord, LivenessState, LivenessTable, TopicLiveness,
    TopicLivenessObserver, TopicRateEstimate,
};
pub use transport::network::{
    IngressInjector, IngressRejection, IngressStats, NetworkConfig, NetworkManager,
    ReinjectOutcome, ZenohMode,
};
pub use transport::output_proxy::OutputProxy;
pub use transport::publisher::CerulionPublisher;
pub use transport::subscriber::{CerulionSubscriber, ReceivedMessage};
pub use transport::uds_write::{prepare_accepted_stream, write_line_bounded, UdsWriteOutcome};
pub use transport::{
    ix_config_shm_identity_from_json, IceoryxShmIdentity, NetworkPosture, TopicRequirements,
    TransportConfig, TransportManager,
};
pub use wire::{WireError, WireHeader};

// Re-export `tracing` at the crate root so codegen-
// emitted code can use the fully-qualified `::cerulion_core::tracing::warn!`
// path (the codegen-emitted modules don't know which crate they live
// in, so they can't `use tracing` directly).
pub use tracing;

// Re-export `serde` at the crate root, for exactly the
// same reason `tracing` is re-exported one line above — codegen-emitted
// code must name a path that resolves from whichever crate it is compiled
// into, and a scaffolded node crate's `[dependencies]` is exactly two
// lines (`cerulion_core` + `native_ros2_messages`, see
// `cerulion_cli_engine/src/templates.rs`). A generated `::serde::Serialize`
// therefore does NOT resolve in a user node crate.
//
// Generated `<Name>Snapshot` types derive
// `::cerulion_core::serde::{Serialize, Deserialize}` and carry
// `#[serde(crate = "::cerulion_core::serde")]`, which is REQUIRED, not
// cosmetic: serde's derive emits `extern crate serde as _serde;` unless
// told otherwise, and that needs `serde` in the *emitting* crate's extern
// prelude. With the `crate = ...` attribute it emits `use <path> as
// _serde;` instead, so the user's `Cargo.toml` stays byte-unchanged —
// which is the difference between "you write an ordinary Rust struct" and
// "you write an ordinary Rust struct and also add a dependency."
pub use serde;

/// The default `cerulion` tracing directive when `RUST_LOG` is unset.
///
/// The level is chosen by the caller's verb class:
/// - `verbose` (any verb, `-v/--verbose`) → `cerulion=debug`
/// - one-shot verbs (`quiet_default = true`: run-and-exit introspection /
///   management commands like `topic list`, `schema list`, `node info`) →
///   `cerulion=warn`, so lifecycle breadcrumbs ("discovery ladder gathered",
///   the netd first-use spawn line) don't interleave with the command's output.
/// - long-running / runtime verbs (`quiet_default = false`: `graph run`,
///   `node run`, `replay`, `viz`, …) → `cerulion=info` (the legacy default).
///
/// This is the FALLBACK only — an explicit `RUST_LOG` always wins (see
/// [`init_logging`]).
fn default_log_directive(verbose: bool, quiet_default: bool) -> &'static str {
    if verbose {
        "cerulion=debug"
    } else if quiet_default {
        "cerulion=warn"
    } else {
        "cerulion=info"
    }
}

/// Build the tracing `EnvFilter` for a CLI invocation.
///
/// - When `rust_log` is `Some`, it GOVERNS the filter entirely — an explicit
///   `RUST_LOG` always wins over the per-verb default, honored verbatim.
/// - When `rust_log` is `None`, we seed a global `error` directive so that
///   NON-`cerulion` targets (iceoryx2 / zenoh / rustdds — including the
///   iceoryx2 logger bridge's `error!`s) stay ERROR-visible, then layer the
///   per-verb `cerulion=<level>` default on top.
///
/// Why the global `error` seed is REQUIRED on the default path: `EnvFilter`'s
/// builder injects its ERROR *default directive* ONLY when the parsed directive
/// set is EMPTY (`from_directives`: `if !has_dynamics && statics.is_empty()`).
/// `EnvFilter::new("cerulion=warn")` parses a NON-empty static set, so that
/// injection never fires and every non-`cerulion` target would fall to OFF —
/// silently swallowing subsystem `error!`s. Seeding `error` ourselves restores
/// the earlier default-path behavior (a global ERROR floor beneath the
/// per-verb `cerulion` level, exactly what `from_default_env()` produced on an
/// empty `RUST_LOG`). The `Some(spec)` branch is `EnvFilter::new(spec)`
/// verbatim, matching the earlier behavior for a SET `RUST_LOG` (which also
/// carried no global floor beyond the user's own directives — the operator's
/// spec is authoritative).
///
/// This must NOT be the earlier `from_default_env().add_directive(default)`
/// layering: `EnvFilter` dedups directives by TARGET (ignoring level — see
/// `Directive`'s `Ord`), so a `cerulion=<level>` `add_directive` would REPLACE
/// a user's same-target `RUST_LOG=cerulion=…` directive, and a more-specific
/// `cerulion=warn` default would even cap a global `RUST_LOG=info`. Letting
/// `RUST_LOG` govern alone (the `Some` branch) is the only way "explicit
/// `RUST_LOG` always wins" holds for the `cerulion` target.
fn build_env_filter(
    verbose: bool,
    quiet_default: bool,
    rust_log: Option<&str>,
) -> tracing_subscriber::EnvFilter {
    use tracing_subscriber::EnvFilter;

    let mut filter = match rust_log {
        Some(spec) => EnvFilter::new(spec),
        // `error` = a global ERROR floor for unmatched (non-`cerulion`) targets;
        // the per-verb `cerulion=<level>` is layered on top.
        //
        // That floor is also what drops the zenoh orchestrator's
        // `Starting with no listener endpoints!` WARN (target
        // `zenoh::net::runtime::orchestrator`), logged whenever a peer session
        // opens with no listen endpoint: the expected shape of every desk-side
        // session (a `cerulion` process connects out; only a robot listens).
        // No per-target directive is needed here, and none is added: a
        // directive the floor already implies is dead configuration whose
        // deletion no test could detect. `cerulion-netd`'s default is `info`
        // globally, so ITS filter names the target explicitly (`DEFAULT_FILTER`
        // in its `main.rs`). DEFAULT path only: an explicit `RUST_LOG=warn`
        // shows the line again. A filter, never a change to the zenoh session
        // config.
        None => EnvFilter::new("error").add_directive(
            default_log_directive(verbose, quiet_default)
                .parse()
                .unwrap(),
        ),
    };

    // Silence one zenoh-INTERNAL teardown-race error: `zenoh::api::admin`
    // (zenoh-1.8.0/src/api/admin.rs:229) logs "Unable to publish transport
    // event: session closed" at ERROR when the admin space races a session
    // close — which happens on every short-lived discovery session a `topic
    // list` opens and drops. It is not our code, carries no lost data, and is
    // not user-actionable, so we raise its floor to OFF. (Meaningful again on
    // the default path: the global `error` floor above would otherwise surface
    // it.)
    //
    // The escape hatch is REAL: `EnvFilter` dedups directives by TARGET
    // (ignoring level), so appending our `zenoh::api::admin=off` would silently
    // REPLACE a user's same-target `RUST_LOG` directive. We therefore append it
    // ONLY when the user's `RUST_LOG` does NOT already name that target — a user
    // who writes `RUST_LOG=zenoh::api::admin=trace` keeps full control.
    let user_owns_admin_target = rust_log
        .map(|v| v.contains("zenoh::api::admin"))
        .unwrap_or(false);
    if !user_owns_admin_target {
        filter = filter.add_directive("zenoh::api::admin=off".parse().unwrap());
    }
    filter
}

/// Initialize logging with sensible defaults.
///
/// When `verbose` is false, uses a compact format suitable for end-users.
/// When `verbose` is true, uses a detailed developer format with file paths
/// and thread IDs.
///
/// Uses `tracing` with env-filter support:
/// - One-shot verbs (`quiet_default = true`): `cerulion=warn` — quiet, so the
///   command's output isn't interleaved with lifecycle breadcrumbs
/// - Long-running / runtime verbs (`quiet_default = false`): `cerulion=info`
/// - Verbose (`-v`, any verb): `cerulion=debug`
/// - An explicit `RUST_LOG` ALWAYS wins over the per-verb default (it governs
///   the filter entirely — so e.g. `RUST_LOG=info cerulion topic list` restores
///   the breadcrumbs a one-shot verb's quiet default suppresses).
///
/// On the default path (no `RUST_LOG`) non-`cerulion` targets keep a global
/// ERROR floor, so subsystem `error!`s (iceoryx2 / zenoh / rustdds) stay
/// visible regardless of verb class — see `build_env_filter`.
///
/// Also suppresses verbose iceoryx2 warnings (stale listener notifications).
/// Override with: `IOX2_LOG_LEVEL=warn cargo run ...`
pub fn init_logging(verbose: bool, quiet_default: bool) {
    use tracing_subscriber::{fmt, prelude::*};

    // Suppress iceoryx2 verbose warnings about stale listeners
    // Uses iceoryx2's programmatic API which is more reliable than env vars
    crate::iceoryx_logger::init_iceoryx_log_level_from_env();

    let rust_log = std::env::var("RUST_LOG")
        .ok()
        .filter(|v| !v.trim().is_empty());
    let filter = build_env_filter(verbose, quiet_default, rust_log.as_deref());

    // Diagnostics go to STDERR, never STDOUT. `fmt::layer()` defaults to
    // stdout, which mixes log lines into a command's DATA output — e.g.
    // `cerulion topic list` would interleave INFO/zenoh lines with the topic
    // names, breaking `topic list > file` and any downstream parsing. stdout is
    // for the command's result; stderr is for logs (standard Unix separation).
    if verbose {
        tracing_subscriber::registry()
            .with(filter)
            .with(
                fmt::layer()
                    .with_writer(std::io::stderr)
                    .with_target(true)
                    .with_thread_ids(true)
                    .with_file(true)
                    .with_line_number(true),
            )
            .init();
    } else {
        tracing_subscriber::registry()
            .with(filter)
            .with(
                fmt::layer()
                    .with_writer(std::io::stderr)
                    .with_target(false)
                    .with_thread_ids(false)
                    .with_file(false)
                    .with_line_number(false)
                    .compact(),
            )
            .init();
    }
}

#[cfg(test)]
mod init_logging_tests {
    use super::{build_env_filter, default_log_directive};
    use std::sync::{Arc, Mutex};
    use tracing::Level;
    use tracing_subscriber::layer::{Context, SubscriberExt};
    use tracing_subscriber::{EnvFilter, Layer};

    /// The per-verb default directive maps the (verbose, quiet) verb
    /// class to the exact `cerulion=<level>` fallback used when `RUST_LOG` is
    /// unset. Hand oracle over the full 2×2 matrix.
    #[test]
    fn default_directive_maps_verb_class_to_level() {
        // One-shot verb, not verbose → quiet `warn`.
        assert_eq!(default_log_directive(false, true), "cerulion=warn");
        // Long-running / runtime verb, not verbose → `info` (legacy default).
        assert_eq!(default_log_directive(false, false), "cerulion=info");
        // `-v` raises EITHER class to `debug`.
        assert_eq!(default_log_directive(true, true), "cerulion=debug");
        assert_eq!(default_log_directive(true, false), "cerulion=debug");
    }

    /// A `tracing` layer that records `(target, level)` of every event that
    /// PASSES its attached filter — the observable proof of what a built
    /// `EnvFilter` enables.
    #[derive(Clone, Default)]
    struct Capture(Arc<Mutex<Vec<(String, Level)>>>);

    impl<S: tracing::Subscriber> Layer<S> for Capture {
        fn on_event(&self, event: &tracing::Event<'_>, _ctx: Context<'_, S>) {
            let m = event.metadata();
            self.0
                .lock()
                .unwrap()
                .push((m.target().to_string(), *m.level()));
        }
    }

    /// Drive `emit` under a thread-scoped subscriber = `filter` + a `Capture`
    /// layer, and return the `(target, level)` of every event the filter let
    /// through. Scoped via `with_default` (not a global `.init()`), so each
    /// test is isolated and its event source lines are distinct callsites.
    fn events_passing(filter: EnvFilter, emit: impl FnOnce()) -> Vec<(String, Level)> {
        let cap = Capture::default();
        let subscriber = tracing_subscriber::registry().with(cap.clone().with_filter(filter));
        tracing::subscriber::with_default(subscriber, emit);
        let events = cap.0.lock().unwrap().clone();
        events
    }

    fn has(events: &[(String, Level)], target: &str, level: Level) -> bool {
        events.iter().any(|(t, l)| t == target && *l == level)
    }

    /// The default path (no `RUST_LOG`) keeps a
    /// global ERROR floor for NON-`cerulion` targets — so subsystem `error!`s
    /// (iceoryx2 / zenoh / rustdds, incl. the iceoryx2 logger bridge) stay
    /// visible on BOTH verb classes, while a non-`cerulion` INFO is still
    /// dropped. Regression guard for `EnvFilter::new("cerulion=warn")` silently
    /// taking every non-`cerulion` target to OFF (the builder injects its ERROR
    /// default only when the parsed set is empty).
    #[test]
    fn default_path_keeps_global_error_floor_for_foreign_targets() {
        // One-shot (quiet=true → cerulion=warn) default.
        let one_shot = events_passing(build_env_filter(false, true, None), || {
            tracing::error!(target: "iceoryx2", "foreign error one-shot");
            tracing::info!(target: "iceoryx2", "foreign info one-shot");
        });
        assert!(
            has(&one_shot, "iceoryx2", Level::ERROR),
            "one-shot default must keep a foreign ERROR visible: {one_shot:?}"
        );
        assert!(
            !has(&one_shot, "iceoryx2", Level::INFO),
            "one-shot default must NOT surface a foreign INFO (global floor is ERROR): {one_shot:?}"
        );

        // Long-running (quiet=false → cerulion=info) default.
        let long_running = events_passing(build_env_filter(false, false, None), || {
            tracing::error!(target: "iceoryx2", "foreign error long-running");
            tracing::info!(target: "iceoryx2", "foreign info long-running");
        });
        assert!(
            has(&long_running, "iceoryx2", Level::ERROR),
            "long-running default must keep a foreign ERROR visible: {long_running:?}"
        );
        assert!(
            !has(&long_running, "iceoryx2", Level::INFO),
            "long-running default must NOT surface a foreign INFO: {long_running:?}"
        );
    }

    /// The per-verb `cerulion` level is applied on the default path —
    /// a one-shot default (`cerulion=warn`) suppresses a `cerulion` INFO while a
    /// long-running default (`cerulion=info`) shows it; both show a `cerulion`
    /// WARN. (The behavioral e2e is `quiet_cli_logging_e2e_test`; this pins the
    /// filter construction directly.)
    #[test]
    fn default_path_applies_cerulion_verb_level() {
        let one_shot = events_passing(build_env_filter(false, true, None), || {
            tracing::info!(target: "cerulion_cli_engine", "cerulion info one-shot");
            tracing::warn!(target: "cerulion_cli_engine", "cerulion warn one-shot");
        });
        assert!(
            !has(&one_shot, "cerulion_cli_engine", Level::INFO),
            "one-shot default (cerulion=warn) must SUPPRESS a cerulion INFO: {one_shot:?}"
        );
        assert!(
            has(&one_shot, "cerulion_cli_engine", Level::WARN),
            "one-shot default must still show a cerulion WARN: {one_shot:?}"
        );

        let long_running = events_passing(build_env_filter(false, false, None), || {
            tracing::info!(target: "cerulion_cli_engine", "cerulion info long-running");
        });
        assert!(
            has(&long_running, "cerulion_cli_engine", Level::INFO),
            "long-running default (cerulion=info) must SHOW a cerulion INFO: {long_running:?}"
        );
    }

    /// An explicit `RUST_LOG` GOVERNS the filter — a global
    /// `RUST_LOG=info` restores a `cerulion` INFO the quiet default would drop,
    /// and a `RUST_LOG=warn` is honored (not force-raised to info). Pins the
    /// `Some(spec)` branch directly (no env mutation).
    #[test]
    fn rust_log_governs_the_filter() {
        let restored = events_passing(build_env_filter(false, true, Some("info")), || {
            tracing::info!(target: "cerulion_cli_engine", "restored");
        });
        assert!(
            has(&restored, "cerulion_cli_engine", Level::INFO),
            "RUST_LOG=info must restore a cerulion INFO over the quiet default: {restored:?}"
        );

        let honored = events_passing(build_env_filter(false, false, Some("warn")), || {
            tracing::info!(target: "cerulion_cli_engine", "should be hidden");
        });
        assert!(
            !has(&honored, "cerulion_cli_engine", Level::INFO),
            "RUST_LOG=warn must be honored — a cerulion INFO stays hidden: {honored:?}"
        );
    }

    /// The zenoh orchestrator's `Starting with no listener endpoints!` WARN
    /// (the expected shape of every desk-side session) is dropped on the
    /// DEFAULT path of both verb classes BY THE GLOBAL `error` FLOOR, its
    /// ERROR still passes, and an explicit `RUST_LOG=warn` shows the WARN
    /// again: the floor is a default, never an override of the user's spec.
    ///
    /// What this pins, stated at its real strength: the CLI's default filter
    /// carries NO per-target orchestrator directive (one was measured
    /// redundant: deleting it left this test green, because the floor already
    /// drops the line). The pin is the FLOOR's behaviour on that target; the
    /// daemon, whose global default is `info`, names the target in its own
    /// `DEFAULT_FILTER` and pins it in its own binary.
    #[test]
    fn default_path_drops_the_zenoh_orchestrator_warn_but_rust_log_restores_it() {
        const ORCHESTRATOR: &str = "zenoh::net::runtime::orchestrator";
        for quiet in [true, false] {
            let events = events_passing(build_env_filter(false, quiet, None), || {
                tracing::warn!(target: ORCHESTRATOR, "Starting with no listener endpoints!");
                tracing::error!(target: ORCHESTRATOR, "orchestrator error");
            });
            assert!(
                !has(&events, ORCHESTRATOR, Level::WARN),
                "default path (quiet={quiet}) must drop the orchestrator WARN: {events:?}"
            );
            assert!(
                has(&events, ORCHESTRATOR, Level::ERROR),
                "default path (quiet={quiet}) must keep the orchestrator ERROR: {events:?}"
            );
        }

        let restored = events_passing(build_env_filter(false, false, Some("warn")), || {
            tracing::warn!(target: ORCHESTRATOR, "Starting with no listener endpoints!");
        });
        assert!(
            has(&restored, ORCHESTRATOR, Level::WARN),
            "an explicit RUST_LOG=warn must show the orchestrator WARN again: {restored:?}"
        );
    }
}
