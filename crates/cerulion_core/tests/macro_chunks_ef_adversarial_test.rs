// SPDX-License-Identifier: AGPL-3.0-only
//! Adversarial runtime tests for the struct macro's
//! auto-`Default` impl, hidden runtime-context fields, and the
//! `now_ns` / `request_shutdown` shim methods.
//!
//! The happy-path test in `macro_shim_methods_test.rs` exercises the full
//! lifecycle through `GraphRuntime`. This file goes after the corner cases
//! that file doesn't reach: pre-init shim observability, runtime overwrite
//! ordering, identity of the cloned `Arc<dyn Clock>`, behaviour with
//! marker-only structs, and direct `Default::default()` construction.
//!
//! Run with:
//! ```bash
//! cargo test -p cerulion_core --test macro_chunks_ef_adversarial_test
//! ```

use cerulion_core::wire::MaxSliceLen;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use cerulion_core::clock::{Clock, RealClock, VirtualClock};
use cerulion_core::graph::node::{AnyPublisher, NodeContext, NodeEntry, ShutdownSignal};
use cerulion_core::prelude::*;
use cerulion_core::testing::TestTransport;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

// Helpers -------------------------------------------------------------------

/// Build an isolated iceoryx2 transport plus a publisher and a subscriber on
/// `topic` (the transport has a buffer size of 4,
/// which every call site needs). The returned `TestTransport` MUST be kept in scope
/// by the caller: it owns the iceoryx2 manager that keeps the publisher and
/// subscriber valid.
fn make_pubr(topic: &str) -> (TestTransport, CerulionPublisher, CerulionSubscriber) {
    let tt = TestTransport::with_buffer_size(4);
    let publisher = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let subscriber = tt.subscriber(topic);
    (tt, publisher, subscriber)
}

/// A `Clock` that counts how many times `now_ns` was called and returns a
/// configurable value. Lets us prove the shim reaches THIS clock and not
/// some other (e.g. the default `RealClock`).
struct CountingClock {
    value: AtomicU64,
    calls: AtomicU64,
}

impl CountingClock {
    fn new(value: u64) -> Arc<Self> {
        Arc::new(Self {
            value: AtomicU64::new(value),
            calls: AtomicU64::new(0),
        })
    }
    fn calls(&self) -> u64 {
        self.calls.load(Ordering::Acquire)
    }
}

impl Clock for CountingClock {
    fn now_ns(&self) -> u64 {
        self.calls.fetch_add(1, Ordering::AcqRel);
        self.value.load(Ordering::Acquire)
    }
    /// CountingClock represents a fake/test
    /// clock so `virt_ns()` returns `Some(value)` (not the trait
    /// default `None` which would force every test using
    /// `self.virt_ns().expect(...)` against this clock to panic).
    fn virt_ns(&self) -> Option<u64> {
        self.calls.fetch_add(1, Ordering::AcqRel);
        Some(self.value.load(Ordering::Acquire))
    }
}

// ===========================================================================
// 1. Default::default() works directly on the user struct (no Entry).
// ===========================================================================
//
// Confirms the macro-synthesised `impl Default` actually compiles and
// produces a value with a usable `RealClock` shim and an un-fired signal.

#[cerulion_node(period_ms = 10)]
struct DirectDefaultNode {
    #[output]
    out: Vector3,
    counter: u32,
}

#[cerulion_node_impl]
impl DirectDefaultNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.counter as f64;
        Ok(())
    }
}

#[test]
fn default_constructs_user_struct_with_uninit_clock() {
    let node = DirectDefaultNode::default();
    // User field defaults preserved.
    assert_eq!(node.counter, 0);
    // The default `__cer_rt.clock` is
    // `UninitClock`, never `RealClock`. Calling
    // `virt_ns()` / `now_ns()` on the shim PANICS — the surfacing of
    // "node not initialized via runtime wrapper" misuse, which a
    // `RealClock` default would silently mask. `real_ns` is a
    // free function (not routed through the clock trait) so it still
    // returns a valid CLOCK_MONOTONIC value.
    let t = node.real_ns();
    assert!(
        t > 0,
        "real_ns is a free function; should return CLOCK_MONOTONIC ns, got {t}"
    );
    // The default ShutdownSignal is un-fired; calling request() on it via
    // the shim flips a private signal not shared with anyone (so it's a
    // no-op from the runtime's POV but must not panic).
    node.request_shutdown();
    // Pre-init `virt_ns()` MUST panic (UninitClock contract). We verify
    // this in a separate `#[should_panic]` test below to keep this
    // happy-path test clean.
}

/// Pre-init `virt_ns()` panics with UninitClock.
/// Locks the contract — a regression to a silent
/// RealClock default fails this test.
#[test]
#[should_panic(expected = "UninitClock::virt_ns called")]
fn default_user_struct_sim_ns_panics_pre_init() {
    let node = DirectDefaultNode::default();
    let _ = node.virt_ns();
}

/// Pre-init `now_ns` (via Clock trait) also panics.
#[test]
#[should_panic(expected = "UninitClock::now_ns called")]
fn default_user_struct_now_ns_panics_pre_init() {
    let node = DirectDefaultNode::default();
    // The shim's `now_ns()` doesn't exist as a public method — the
    // user-facing shim only exposes real_ns and virt_ns. To reach the
    // trait method we go through the hidden __cer_rt field. This is
    // a white-box test of the misuse path. `dyn Clock` resolves
    // `now_ns` via the vtable; no `use Clock` import needed.
    let _ = node.__cer_rt.clock.now_ns();
}

// ===========================================================================
// 2. Pre-init shim sees default RealClock; post-init shim sees runtime clock.
// ===========================================================================
//
// The struct macro's wrapper `init` overwrites the hidden fields BEFORE
// storing the context. So:
//   - Calling `self.virt_ns().expect("test runs under VirtualClock or CountingClock; virt_ns must be Some")` before init → default RealClock value.
//   - Calling `self.virt_ns().expect("test runs under VirtualClock or CountingClock; virt_ns must be Some")` after init  → the runtime's clock value.
// We use a CountingClock that returns a fixed sentinel (`424242`) so we
// can distinguish unambiguously from any RealClock reading.

#[cerulion_node(period_ms = 10)]
struct ClockOverwriteNode {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl ClockOverwriteNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self
            .virt_ns()
            .expect("test runs under VirtualClock or CountingClock; virt_ns must be Some")
            as f64;
        Ok(())
    }
}

#[test]
fn init_overwrites_clock_so_shim_sees_runtime_value() {
    let mut entry = ClockOverwriteNodeEntry::new();
    // The pre-init clock is `UninitClock` and
    // `virt_ns()` panics, so the pre-init clock cannot be probed
    // via `virt_ns()`; only `real_ns()` (free function, not routed
    // through the trait) is safe to call before init.
    let pre_init_wall = entry.inner.real_ns();
    assert!(
        pre_init_wall > 0,
        "pre-init real_ns should be CLOCK_MONOTONIC"
    );

    // Build a context with a sentinel clock.
    let sentinel: Arc<CountingClock> = CountingClock::new(424_242);
    let clock_dyn: Arc<dyn Clock> = sentinel.clone();
    let (_tt, pubr, _sub) = make_pubr("test/cef/clock_overwrite/out");
    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("out".to_string(), AnyPublisher::Ipc(pubr));
    let ctx = NodeContext::with_runtime_env(
        publishers,
        IndexMap::new(),
        clock_dyn,
        ShutdownSignal::new(),
        ::std::sync::Arc::new(::std::collections::HashMap::new()),
    );

    entry.init(ctx).expect("init");

    // Post-init: the shim must observe the runtime clock, not the default.
    let post_init = entry
        .inner
        .virt_ns()
        .expect("test runs under VirtualClock or CountingClock; virt_ns must be Some");
    assert_eq!(
        post_init, 424_242,
        "post-init shim must read the runtime's CountingClock (got {post_init})"
    );
    assert!(
        sentinel.calls() >= 1,
        "shim call should have hit the sentinel clock at least once"
    );
}

// ===========================================================================
// 3. Shim's request_shutdown flips the SAME signal NodeContext exposes.
// ===========================================================================

#[cerulion_node(period_ms = 10)]
struct ShutdownIdentityNode {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl ShutdownIdentityNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 0.0;
        Ok(())
    }
}

#[test]
fn shim_request_shutdown_flips_runtime_signal() {
    let signal = ShutdownSignal::new();
    assert!(!signal.is_requested());

    let mut entry = ShutdownIdentityNodeEntry::new();
    let (_tt, pubr, _sub) = make_pubr("test/cef/shutdown_identity/out");
    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("out".to_string(), AnyPublisher::Ipc(pubr));
    let clock: Arc<dyn Clock> = Arc::new(RealClock);
    let ctx = NodeContext::with_runtime_env(
        publishers,
        IndexMap::new(),
        clock,
        signal.clone(),
        Arc::new(std::collections::HashMap::new()),
    );

    entry.init(ctx).expect("init");

    // Shim flips the inner signal; the runtime's external clone must see it.
    entry.inner.request_shutdown();
    assert!(
        signal.is_requested(),
        "shim must have flipped the same shared signal"
    );
}

// ===========================================================================
// 4. Arc identity: the shim's clock is the SAME Arc the runtime holds.
// ===========================================================================
//
// Stronger than the value-equality check above: if `init` ever copies the
// clock by value (e.g. `*context.clock()`) instead of cloning the Arc, the
// pointer comparison below catches it.

#[cerulion_node(period_ms = 10)]
struct ArcIdentityNode {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl ArcIdentityNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 0.0;
        Ok(())
    }
}

#[test]
fn shim_clock_is_same_arc_as_runtime_clock() {
    let counting: Arc<CountingClock> = CountingClock::new(7);
    let runtime_clock: Arc<dyn Clock> = counting.clone();

    let mut entry = ArcIdentityNodeEntry::new();
    let (_tt, pubr, _sub) = make_pubr("test/cef/arc_identity/out");
    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("out".to_string(), AnyPublisher::Ipc(pubr));
    let ctx = NodeContext::with_runtime_env(
        publishers,
        IndexMap::new(),
        runtime_clock.clone(),
        ShutdownSignal::new(),
        ::std::sync::Arc::new(::std::collections::HashMap::new()),
    );
    entry.init(ctx).expect("init");

    // The hidden field MUST point at the same allocation as `runtime_clock`.
    // Hidden runtime context is bundled in `__cer_rt`;
    // the test reaches in via that single field.
    let shim_ptr = Arc::as_ptr(&entry.inner.__cer_rt.clock) as *const ();
    let runtime_ptr = Arc::as_ptr(&runtime_clock) as *const ();
    assert_eq!(
        shim_ptr, runtime_ptr,
        "init must clone the Arc, not deep-copy the value"
    );
}

// ===========================================================================
// 5. Marker-only struct (port fields only, no user state) — Default works.
// ===========================================================================

#[cerulion_node(period_ms = 10)]
struct MarkerOnlyNode {
    #[output]
    x: Vector3,
}

#[cerulion_node_impl]
impl MarkerOnlyNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.x.x = 1.0;
        Ok(())
    }
}

#[test]
fn marker_only_struct_default_and_shim_compile() {
    let node = MarkerOnlyNode::default();
    // Default = UninitClock; virt_ns panics.
    // real_ns is a free function and still works pre-init.
    let _ = node.real_ns();
    node.request_shutdown();

    // Entry constructible too (uses Default).
    let _ = MarkerOnlyNodeEntry::new();
}

// ===========================================================================
// 6. Multiple instances of the same node type each have INDEPENDENT
// shutdown signals once they get distinct contexts.
// ===========================================================================

#[cerulion_node(period_ms = 10)]
struct IndependentSignalNode {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl IndependentSignalNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 0.0;
        Ok(())
    }
}

#[test]
fn distinct_instances_get_distinct_signals() {
    let sig_a = ShutdownSignal::new();
    let sig_b = ShutdownSignal::new();

    let mut a = IndependentSignalNodeEntry::new();
    let mut b = IndependentSignalNodeEntry::new();

    let clock: Arc<dyn Clock> = Arc::new(RealClock);

    let (_tta, pa, _sa) = make_pubr("test/cef/indep/a");
    let mut publishers_a: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers_a.insert("out".to_string(), AnyPublisher::Ipc(pa));
    let ctx_a = NodeContext::with_runtime_env(
        publishers_a,
        IndexMap::new(),
        clock.clone(),
        sig_a.clone(),
        ::std::sync::Arc::new(::std::collections::HashMap::new()),
    );

    let (_ttb, pb, _sb) = make_pubr("test/cef/indep/b");
    let mut publishers_b: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers_b.insert("out".to_string(), AnyPublisher::Ipc(pb));
    let ctx_b = NodeContext::with_runtime_env(
        publishers_b,
        IndexMap::new(),
        clock,
        sig_b.clone(),
        ::std::sync::Arc::new(::std::collections::HashMap::new()),
    );

    a.init(ctx_a).expect("init a");
    b.init(ctx_b).expect("init b");

    // Trip a only.
    a.inner.request_shutdown();
    assert!(sig_a.is_requested(), "a's signal must be flipped");
    assert!(
        !sig_b.is_requested(),
        "b's signal must remain un-fired (signals are not shared by accident)"
    );
}

// ===========================================================================
// 7. Shim methods callable through inherent and Entry::new() paths.
// ===========================================================================

#[cerulion_node(period_ms = 10)]
struct InlineProbeNode {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl InlineProbeNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 0.0;
        Ok(())
    }
}

#[test]
fn shim_methods_callable_through_inherent_impl_paths() {
    // Static dispatch via inherent method (
    // the shim methods are `real_ns` and `virt_ns`).
    // Pre-init `virt_ns` panics, so we don't call
    // it here — only verify real_ns + request_shutdown work pre-init
    // (both are independent of the runtime clock / signal).
    let n = InlineProbeNode::default();
    let _t: u64 = InlineProbeNode::real_ns(&n);
    InlineProbeNode::request_shutdown(&n);

    // The wrapper Entry::new() path uses Default internally.
    let _e = InlineProbeNodeEntry::new();
}

// ===========================================================================
// 8. Re-init: the second init must pick up the NEW runtime clock, not keep
// the old one. (The wrapper writes the hidden fields each init.)
// ===========================================================================

#[cerulion_node(period_ms = 10)]
struct ReinitClockNode {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl ReinitClockNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self
            .virt_ns()
            .expect("test runs under VirtualClock or CountingClock; virt_ns must be Some")
            as f64;
        Ok(())
    }
}

#[test]
fn reinit_overwrites_clock_with_new_runtime_clock() {
    let mut entry = ReinitClockNodeEntry::new();

    // First init with sentinel A.
    let clock_a: Arc<CountingClock> = CountingClock::new(111);
    let (_tta, pa, _sa) = make_pubr("test/cef/reinit/a");
    let mut pubs_a: IndexMap<String, AnyPublisher> = IndexMap::new();
    pubs_a.insert("out".to_string(), AnyPublisher::Ipc(pa));
    let clock_a_dyn: Arc<dyn Clock> = clock_a.clone();
    let ctx_a = NodeContext::with_runtime_env(
        pubs_a,
        IndexMap::new(),
        clock_a_dyn,
        ShutdownSignal::new(),
        ::std::sync::Arc::new(::std::collections::HashMap::new()),
    );
    entry.init(ctx_a).expect("init a");
    assert_eq!(
        entry
            .inner
            .virt_ns()
            .expect("test runs under VirtualClock or CountingClock; virt_ns must be Some"),
        111
    );

    // Shutdown so a re-init is allowed.
    entry.shutdown().expect("shutdown");

    // Second init with sentinel B.
    let clock_b: Arc<CountingClock> = CountingClock::new(222);
    let (_ttb, pb, _sb) = make_pubr("test/cef/reinit/b");
    let mut pubs_b: IndexMap<String, AnyPublisher> = IndexMap::new();
    pubs_b.insert("out".to_string(), AnyPublisher::Ipc(pb));
    let clock_b_dyn: Arc<dyn Clock> = clock_b.clone();
    let ctx_b = NodeContext::with_runtime_env(
        pubs_b,
        IndexMap::new(),
        clock_b_dyn,
        ShutdownSignal::new(),
        ::std::sync::Arc::new(::std::collections::HashMap::new()),
    );
    entry.init(ctx_b).expect("re-init");
    assert_eq!(
        entry
            .inner
            .virt_ns()
            .expect("test runs under VirtualClock or CountingClock; virt_ns must be Some"),
        222,
        "re-init must overwrite the hidden clock with the NEW runtime clock"
    );
}

// ===========================================================================
// 9. VirtualClock advance is observable through the shim post-init.
// ===========================================================================

#[cerulion_node(period_ms = 10)]
struct SimAdvanceNode {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl SimAdvanceNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self
            .virt_ns()
            .expect("test runs under VirtualClock or CountingClock; virt_ns must be Some")
            as f64;
        Ok(())
    }
}

#[test]
fn shim_observes_simulated_clock_advances() {
    let sim = Arc::new(VirtualClock::new());
    let clock_dyn: Arc<dyn Clock> = sim.clone();
    let mut entry = SimAdvanceNodeEntry::new();
    let (_tt, p, _s) = make_pubr("test/cef/sim_advance/out");
    let mut pubs: IndexMap<String, AnyPublisher> = IndexMap::new();
    pubs.insert("out".to_string(), AnyPublisher::Ipc(p));
    let ctx = NodeContext::with_runtime_env(
        pubs,
        IndexMap::new(),
        clock_dyn,
        ShutdownSignal::new(),
        ::std::sync::Arc::new(::std::collections::HashMap::new()),
    );
    entry.init(ctx).expect("init");

    assert_eq!(
        entry
            .inner
            .virt_ns()
            .expect("test runs under VirtualClock or CountingClock; virt_ns must be Some"),
        0
    );
    sim.advance(5_000);
    assert_eq!(
        entry
            .inner
            .virt_ns()
            .expect("test runs under VirtualClock or CountingClock; virt_ns must be Some"),
        5_000
    );
    sim.advance(1);
    assert_eq!(
        entry
            .inner
            .virt_ns()
            .expect("test runs under VirtualClock or CountingClock; virt_ns must be Some"),
        5_001
    );
}

// ===========================================================================
// 10. with_state preserves the user fields and uses Default for the
// hidden runtime-context fields. (User can't construct a node WITH
// custom hidden fields — the hidden fields are private to the macro.)
// ===========================================================================

#[cerulion_node(period_ms = 10)]
struct WithStateNode {
    #[output]
    out: Vector3,
    seed: u64,
}

#[cerulion_node_impl]
impl WithStateNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.seed as f64;
        Ok(())
    }
}

#[test]
fn with_state_preserves_user_fields_and_neutral_hidden() {
    // Construct via Default (because hidden fields are macro-only) and
    // override one user field via struct-update syntax.
    let node = WithStateNode {
        seed: 9999,
        ..Default::default()
    };
    let entry = WithStateNodeEntry::with_state(node);
    assert_eq!(entry.inner.seed, 9999);
    // The hidden clock is `UninitClock`, never
    // `RealClock`. `virt_ns` panics pre-init; `real_ns`
    // is a free function and still works.
    let t = entry.inner.real_ns();
    assert!(t > 0);
}

// ===========================================================================
// 11. Auto-injected #[allow(dead_code)] on port fields keeps the user's
// crate compiling even though the port marker field is never read directly.
// We can't directly assert "dead_code wasn't fired" from a runtime test,
// but if the macro forgot to inject the allow, this whole test crate would
// fail to compile under the workspace's strict `dead_code = "deny"` lint.
// So the existence of this node + a green build is the assertion.
// ===========================================================================

#[cerulion_node(period_ms = 10)]
struct DeadCodeAllowedNode {
    #[output]
    never_read_directly: Vector3,
}

#[cerulion_node_impl]
impl DeadCodeAllowedNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Touch the port via the rewriter so it isn't trivially dead.
        self.never_read_directly.x = 0.0;
        Ok(())
    }
}

#[test]
fn dead_code_allow_compile_smoke() {
    let _ = DeadCodeAllowedNodeEntry::new();
}

// ===========================================================================
// 12. CountingClock proves the shim path makes EXACTLY one virtual call
// per shim invocation (not, say, two due to a Deref + manual call).
// ===========================================================================

#[cerulion_node(period_ms = 10)]
struct CallCountNode {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl CallCountNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 0.0;
        Ok(())
    }
}

#[test]
fn shim_now_ns_makes_exactly_one_virtual_call() {
    let counting: Arc<CountingClock> = CountingClock::new(42);
    let runtime_clock: Arc<dyn Clock> = counting.clone();

    let mut entry = CallCountNodeEntry::new();
    let (_tt, p, _s) = make_pubr("test/cef/call_count/out");
    let mut pubs: IndexMap<String, AnyPublisher> = IndexMap::new();
    pubs.insert("out".to_string(), AnyPublisher::Ipc(p));
    let ctx = NodeContext::with_runtime_env(
        pubs,
        IndexMap::new(),
        runtime_clock,
        ShutdownSignal::new(),
        ::std::sync::Arc::new(::std::collections::HashMap::new()),
    );
    entry.init(ctx).expect("init");

    let baseline = counting.calls();
    let _ = entry
        .inner
        .virt_ns()
        .expect("test runs under VirtualClock or CountingClock; virt_ns must be Some");
    let _ = entry
        .inner
        .virt_ns()
        .expect("test runs under VirtualClock or CountingClock; virt_ns must be Some");
    let _ = entry
        .inner
        .virt_ns()
        .expect("test runs under VirtualClock or CountingClock; virt_ns must be Some");
    assert_eq!(
        counting.calls() - baseline,
        3,
        "three shim calls must produce exactly three Clock::now_ns dispatches"
    );
}
