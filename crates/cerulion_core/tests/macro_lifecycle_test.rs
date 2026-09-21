// SPDX-License-Identifier: AGPL-3.0-only
//! Verifies the user-defined `init` and `shutdown`
//! lifecycle methods on a `#[cerulion_node_impl]` block are actually
//! invoked by the macro-generated wrapper.
//!
//! Without the lifecycle shims the wrapper's `NodeEntry::init` would only
//! stash the context and the wrapper's `shutdown` only clear it: any user-written
//! `fn init` / `fn shutdown` on the impl block would become unused dead
//! code — silently. The bench's `latency_node` requires both
//! (init reads env vars; shutdown writes the CSV row).
//!
//! The shims: `#[cerulion_node_impl]` renames user-written `init` and
//! `shutdown` to `__cer_user_init` / `__cer_user_shutdown` and appends
//! no-op stubs when the user didn't write them. The struct-macro
//! wrapper always calls those shim names.

use cerulion_core::wire::MaxSliceLen;
use std::sync::atomic::{AtomicUsize, Ordering};

use cerulion_core::graph::node::{NodeContext, NodeEntry};
use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

static INIT_CALLS: AtomicUsize = AtomicUsize::new(0);
static TICK_CALLS: AtomicUsize = AtomicUsize::new(0);
static SHUTDOWN_CALLS: AtomicUsize = AtomicUsize::new(0);
static CAPTURED_ENV: std::sync::OnceLock<String> = std::sync::OnceLock::new();

#[cerulion_node(period_ms = 1)]
struct LifecycleNode {
    #[output]
    out: Vector3,
    captured_value: String,
}

#[cerulion_node_impl]
impl LifecycleNode {
    fn init(&mut self, ctx: &mut NodeContext) -> Result<(), NodeError> {
        // Side effect: bump a counter so the test can assert init ran.
        INIT_CALLS.fetch_add(1, Ordering::Relaxed);
        // Real work: pull from env (`NodeContext::env_str`), so
        // we exercise the same path the bench's latency_node uses.
        self.captured_value = ctx.env_str("TEST_LIFECYCLE_VAR", "default-from-init");
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        TICK_CALLS.fetch_add(1, Ordering::Relaxed);
        self.out.x = 0.0; // exercise the proxy so the macro can't elide it
        Ok(())
    }

    fn shutdown(&mut self) -> Result<(), NodeError> {
        SHUTDOWN_CALLS.fetch_add(1, Ordering::Relaxed);
        // Stash what init captured so the test can assert end-to-end.
        let _ = CAPTURED_ENV.set(self.captured_value.clone());
        Ok(())
    }
}

#[test]
fn user_init_and_shutdown_methods_are_invoked() {
    INIT_CALLS.store(0, Ordering::Relaxed);
    TICK_CALLS.store(0, Ordering::Relaxed);
    SHUTDOWN_CALLS.store(0, Ordering::Relaxed);
    std::env::set_var("TEST_LIFECYCLE_VAR", "value-set-by-test");

    let mut entry = LifecycleNodeEntry::new();

    // Wire one iceoryx2 publisher for `out` so init has a real
    // NodeContext to read env from. We don't drive any data through
    // the channel — just need the publisher to satisfy NodeContext.
    use cerulion_core::clock::RealClock;
    use cerulion_core::graph::node::{AnyPublisher, AnySubscriber};
    use cerulion_core::testing::TestTransport;
    use indexmap::IndexMap;
    let tt = TestTransport::with_buffer_size(4);
    let pubr = tt.publisher("test/lifecycle/out", MaxSliceLen::const_new(256), 0);
    let _sub = tt.subscriber("test/lifecycle/out");
    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("out".to_string(), AnyPublisher::Ipc(pubr));
    let subscribers: IndexMap<String, AnySubscriber> = IndexMap::new();
    // NodeContext never falls back to live
    // std::env::var when env_snapshot is empty. Build an explicit
    // snapshot from the live env so this lifecycle test still exercises
    // the env_str path against the value the test set above.
    use cerulion_core::graph::node::ShutdownSignal;
    use std::sync::Arc;
    let live_env_snapshot: Arc<std::collections::HashMap<String, String>> =
        Arc::new(std::env::vars().collect());
    let runtime_clock: Arc<dyn cerulion_core::clock::Clock> = Arc::new(RealClock);
    let ctx = NodeContext::with_runtime_env(
        publishers,
        subscribers,
        runtime_clock,
        ShutdownSignal::new(),
        live_env_snapshot,
    );

    entry.init(ctx).expect("init");
    assert_eq!(
        INIT_CALLS.load(Ordering::Relaxed),
        1,
        "user-defined init must run exactly once"
    );

    entry.tick().expect("tick");
    entry.tick().expect("tick");
    assert_eq!(
        TICK_CALLS.load(Ordering::Relaxed),
        2,
        "user tick fires per call"
    );

    entry.shutdown().expect("shutdown");
    assert_eq!(
        SHUTDOWN_CALLS.load(Ordering::Relaxed),
        1,
        "user-defined shutdown must run exactly once"
    );

    // End-to-end: the env value the test set is what init captured and
    // shutdown then exposed via the OnceLock. Proves both lifecycle
    // hooks ran AND that the same `self.captured_value` carried state
    // from init through ticks into shutdown.
    let captured = CAPTURED_ENV
        .get()
        .expect("shutdown must have set CAPTURED_ENV");
    assert_eq!(captured, "value-set-by-test");

    std::env::remove_var("TEST_LIFECYCLE_VAR");
}
