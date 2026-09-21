// SPDX-License-Identifier: AGPL-3.0-only
//! Integration tests for macro-generated cdylib nodes.
//!
//! Tests that `#[cerulion_node]` + `#[cerulion_node_impl]` with the
//! `cdylib` feature generates FFI entry points compatible with
//! `DylibNodeEntry::load()` for the **zero-copy AST-rewritten path**, and
//! that the rewritten body actually transfers data through the FFI surface
//! (input subscriber → user tick → output publisher → host subscriber)
//! when wired through real iceoryx2 SHM.
//!
//! # Running
//!
//! ```bash
//! cargo build -p test_node_macro_cdylib
//! cargo test -p cerulion_core --test macro_cdylib_test -- --test-threads=1
//! ```
//!
//! `--test-threads=1` is required because the data-flow assertion uses the
//! iceoryx2 singleton (Principle #8 — one node per process).

use cerulion_core::wire::MaxSliceLen;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, SystemTime};

use cerulion_core::graph::node::{
    AnyPublisher, AnySubscriber, DylibNodeEntry, NodeContext, NodeEntry,
};
use cerulion_core::testing::TestTransport;
use cerulion_core::transport::TransportManager;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

/// Build a unique topic name suffixed with wall-clock nanos + a counter so
/// the iceoryx2 singleton's per-topic services don't collide across runs.
fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/macro_cdylib/{base}/{nanos}/{id}")
}

/// Build a NodeContext wired with iceoryx2 publishers/subscribers for the
/// `velocity_in` input and `cmd_out` output. Returned alongside an upstream
/// publisher (to drive `velocity_in`) and a downstream subscriber
/// (to observe `cmd_out`). Used by the lifecycle tests.
///
/// The owning [`TestTransport`] is returned as the first tuple element: it
/// must outlive every publisher/subscriber/NodeContext it created, so callers
/// bind it to a live local for the duration of the test.
fn make_in_process_context() -> (
    TestTransport,
    NodeContext,
    cerulion_core::transport::publisher::CerulionPublisher,
    cerulion_core::transport::subscriber::CerulionSubscriber,
) {
    // One isolated transport owns both topics (velocity_in + cmd_out). The
    // 24-byte Vector3 fixed payload + 32-byte WireHeader fits in 256.
    let tt = TestTransport::with_buffer_size(8);

    // velocity_in: upstream publisher (host owns), node holds subscriber.
    let upstream_pub = tt.publisher(
        "test/macro_cdylib/velocity_in_local",
        MaxSliceLen::const_new(256),
        0,
    );
    let velocity_sub = tt.subscriber("test/macro_cdylib/velocity_in_local");

    // cmd_out: node holds publisher, downstream subscriber observes.
    let cmd_pub = tt.publisher(
        "test/macro_cdylib/cmd_out_local",
        MaxSliceLen::const_new(256),
        0,
    );
    let cmd_sub = tt.subscriber("test/macro_cdylib/cmd_out_local");

    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("cmd_out".to_string(), AnyPublisher::Ipc(cmd_pub));
    let mut subscribers: IndexMap<String, AnySubscriber> = IndexMap::new();
    subscribers.insert("velocity_in".to_string(), AnySubscriber::Ipc(velocity_sub));

    let ctx = NodeContext::for_tests(publishers, subscribers);
    (tt, ctx, upstream_pub, cmd_sub)
}

// ============================================================
// test_macro_cdylib_loads
// ============================================================

#[test]
fn test_macro_cdylib_loads() {
    let path = find_macro_cdylib();
    let _node = DylibNodeEntry::load(&path).expect("should load macro cdylib");
}

// ============================================================
// test_macro_cdylib_info_matches
// ============================================================

#[test]
fn test_macro_cdylib_info_matches() {
    let path = find_macro_cdylib();
    let node = DylibNodeEntry::load(&path).expect("load");
    let info = node.info().expect("info should parse");

    // Zero-copy fixture: one input + one output port.
    assert_eq!(info.input_names(), vec!["velocity_in"]);
    assert_eq!(info.output_names(), vec!["cmd_out"]);

    // The cdylib FFI carries per-output
    // OutputMeta with the macro-resolved `<T as ShmMessage>` consts.
    // The fixture declares `#[output] cmd_out: Vector3`, so the
    // round-trip should land Vector3's `SCHEMA_HASH` and
    // `MAX_SLICE_LEN` in the loader's NodeInfo.
    use cerulion_core::message::ShmMessage;
    assert_eq!(info.output_meta().len(), 1);
    let cmd_out_meta = &info.output_meta()[0];
    assert_eq!(cmd_out_meta.name, "cmd_out");
    assert_eq!(
        cmd_out_meta.schema_hash,
        <Vector3 as ShmMessage>::SCHEMA_HASH,
        "cdylib must carry Vector3's SCHEMA_HASH through the JSON"
    );
    assert_eq!(
        cmd_out_meta.max_slice_len_default,
        <Vector3 as ShmMessage>::MAX_SLICE_LEN,
        "cdylib must carry Vector3's MAX_SLICE_LEN through the JSON"
    );
    // The port type's fixed wire size, which is what the recorder
    // stamps into a bag channel's `SchemaDescriptor` instead of reading it
    // out of a workspace `schemas/` file that may have drifted.
    //
    // A HAND ORACLE, not `<Vector3 as ShmMessage>::WIRE_FIXED_SIZE`:
    // `geometry_msgs/Vector3` is three `float64`s, so its fixed section is
    // 24 bytes. Reading the const on both sides would compare the emitter
    // against itself and would still pass if the macro emitted the WRONG
    // const — which is precisely the defect this arm exists to catch (the
    // key-set parity gate checks key NAMES, so it cannot see a wrong
    // value). The two asserts above are self-compares for that reason and
    // are left as they are; this one is not.
    // Drift guard FIRST, so a Vector3 layout change fails HERE naming the
    // cause rather than at the bare `Some(24)` below. (It reads the HOST's
    // compiled const while the oracle below came off the cdylib's JSON; the
    // two agree only because both are the same compiled crate, so this is a
    // guard on the ORACLE, not independent evidence about the cdylib.)
    assert_eq!(
        <Vector3 as ShmMessage>::WIRE_FIXED_SIZE,
        24,
        "the hand oracle below assumes geometry_msgs/Vector3 is 3 x float64"
    );
    assert_eq!(
        cmd_out_meta.wire_fixed_size,
        Some(24),
        "cdylib must carry Vector3's WIRE_FIXED_SIZE (3 x float64 = 24) through the JSON"
    );
}

// ============================================================
// test_macro_cdylib_tick_works
// ============================================================
//
// Wires the cdylib up with in-process pub/sub. The zero-copy tick wrapper
// requires both the velocity_in subscriber and the cmd_out publisher to
// exist in the NodeContext (or it returns NodeError "missing publisher" /
// nests a no-op `try_view`). With no upstream publish each tick lands in
// the "no sample available — Ok(None) → no-op" branch of the nested
// `try_view`, which is the documented single/multi-input
// semantics. The tick must still succeed.

#[test]
fn test_macro_cdylib_tick_works() {
    let path = find_macro_cdylib();
    let mut node = DylibNodeEntry::load(&path).expect("load");

    let (_tt, ctx, _upstream_pub, _cmd_sub) = make_in_process_context();
    node.init(ctx).expect("init should succeed");

    for _ in 0..5 {
        node.tick().expect("tick should succeed");
    }

    node.shutdown().expect("shutdown should succeed");
}

// ============================================================
// test_macro_cdylib_multi_instance
// ============================================================

#[test]
fn test_macro_cdylib_multi_instance() {
    let path = find_macro_cdylib();
    let mut node1 = DylibNodeEntry::load(&path).expect("load 1");
    let mut node2 = DylibNodeEntry::load(&path).expect("load 2");

    let (_tt1, ctx1, _up1, _down1) = make_in_process_context();
    let (_tt2, ctx2, _up2, _down2) = make_in_process_context();
    node1.init(ctx1).expect("init node1");
    node2.init(ctx2).expect("init node2");

    // Tick node1 twice, node2 once
    node1.tick().expect("tick node1 (1)");
    node1.tick().expect("tick node1 (2)");
    node2.tick().expect("tick node2 (1)");

    // Shutdown node1 — node2 should still work
    node1.shutdown().expect("shutdown node1");

    node2
        .tick()
        .expect("node2 still works after node1 shutdown");
    node2.shutdown().expect("shutdown node2");
}

// ============================================================
// Data-flow assertion
// ============================================================
//
// The earlier lifecycle tests prove load / info / tick / shutdown work
// across the FFI boundary, but they don't prove the user's `tick` body
// (`self.cmd_out.x = self.velocity_in.x` etc.) actually transfers data.
// This test wires real iceoryx2 publishers/subscribers around the cdylib
// and asserts that a Vector3 published on `velocity_in` lands on
// `cmd_out` with the same field values after one tick — which only works
// if the AST-rewriter on the cdylib side correctly:
//
//   1. Took the velocity_in subscriber off the disjoint subscribers map.
//   2. Opened a `try_view::<Vector3, _>` on it inside the generated
//      tick wrapper so `self.velocity_in.x` resolved to the SHM read.
//   3. Loaned an OutputProxy<Vector3> for cmd_out so
//      `self.cmd_out.x = ...` wrote straight into the iceoryx2 sample's
//      SHM region.
//   4. Dropped the proxy at the end of the tick wrapper to publish.
//
// All four steps run inside the cdylib (compiled as a separate dylib);
// the host process only sees the FFI ABI. A failure in any step would
// either error the tick or yield zeroed/garbage Vector3 fields on the
// downstream subscriber.

#[test]
fn test_macro_cdylib_data_flows_through_ffi() {
    let path = find_macro_cdylib();
    let mut node = DylibNodeEntry::load(&path).expect("load cdylib for data-flow");

    let mgr = TransportManager::get_or_init().expect("init iceoryx2 transport manager");

    // Two unique topics — one feeds velocity_in, one drains cmd_out.
    let velocity_topic = unique_topic("velocity_in");
    let cmd_topic = unique_topic("cmd_out");

    // Host owns the upstream publisher (drives velocity_in) and the
    // downstream subscriber (drains cmd_out). The cdylib owns the inverse
    // pair via the NodeContext we hand it.
    let mut upstream_velocity_pub = mgr
        .create_publisher_simple(&velocity_topic, MaxSliceLen::const_new(256))
        .expect("create velocity_in upstream publisher");
    let mut downstream_cmd_sub = mgr
        .create_subscriber(&cmd_topic)
        .expect("create cmd_out downstream subscriber");

    // Cdylib-side ports.
    let cdylib_velocity_sub = mgr
        .create_subscriber(&velocity_topic)
        .expect("create velocity_in subscriber for cdylib");
    let cdylib_cmd_pub = mgr
        .create_publisher_simple(&cmd_topic, MaxSliceLen::const_new(256))
        .expect("create cmd_out publisher for cdylib");

    // Build the NodeContext the cdylib will consume on init().
    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("cmd_out".to_string(), AnyPublisher::Ipc(cdylib_cmd_pub));
    let mut subscribers: IndexMap<String, AnySubscriber> = IndexMap::new();
    subscribers.insert(
        "velocity_in".to_string(),
        AnySubscriber::Ipc(cdylib_velocity_sub),
    );
    let ctx = NodeContext::for_tests(publishers, subscribers);

    node.init(ctx).expect("init should succeed");

    // Publish a known Vector3 upstream; values chosen so each axis is
    // distinguishable (no zeros / no equal pairs) — that way a memcpy
    // bug in the wrong direction would surface as a value mismatch
    // rather than a coincidental pass.
    {
        let mut proxy = upstream_velocity_pub
            .loan_proxy::<Vector3>()
            .expect("loan velocity_in upstream");
        proxy.x = 1.5;
        proxy.y = -2.25;
        proxy.z = 3.75;
        // Drop sends.
    }

    // Give iceoryx2 a moment to deliver. The transport tests use 50ms.
    std::thread::sleep(Duration::from_millis(50));

    // Tick the cdylib — this is the FFI call that runs the AST-rewritten
    // user body inside the dylib's address space.
    node.tick().expect("cdylib tick should succeed");

    std::thread::sleep(Duration::from_millis(50));

    // Drain cmd_out and assert the values match.
    let observed = downstream_cmd_sub
        .try_view::<Vector3, _>(|view| (view.x, view.y, view.z))
        .expect("try_view cmd_out")
        .expect("downstream subscriber should see one frame on cmd_out");

    assert!(
        (observed.0 - 1.5).abs() < f64::EPSILON,
        "cmd_out.x should equal velocity_in.x = 1.5, got {}",
        observed.0
    );
    assert!(
        (observed.1 - -2.25).abs() < f64::EPSILON,
        "cmd_out.y should equal velocity_in.y = -2.25, got {}",
        observed.1
    );
    assert!(
        (observed.2 - 3.75).abs() < f64::EPSILON,
        "cmd_out.z should equal velocity_in.z = 3.75, got {}",
        observed.2
    );

    node.shutdown().expect("shutdown");
}

/// Find the macro-generated test cdylib in the target directory.
fn find_macro_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_macro_cdylib")
}

/// Find the external-macro test cdylib. It exports the OPTIONAL
/// `cerulion_node_external_source` symbol (the macro emits it for an
/// `#[cerulion_node(external)]` node).
fn find_external_macro_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_macro_external_cdylib")
}

/// Find the raw-FFI test cdylib (`test_node_cdylib`) — a HAND-WRITTEN cdylib that
/// does NOT export `cerulion_node_external_source`. Used for the
/// back-compat contract (missing symbol → `external_source() == None`, no ABI
/// error).
fn find_raw_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_cdylib")
}

// ============================================================
// External_source across the cdylib FFI
// ============================================================

/// The external macro cdylib fixture returns
/// `ExternalSource::HostDriven` (FFI kind 3) through the OPTIONAL
/// `cerulion_node_external_source` symbol — proving the host maps the kind code
/// back to an `ExternalSource`. `external_source()` needs a live handle, so the
/// node is `init`'d first with a context carrying the fixture's single `cmd`
/// output.
#[test]
fn test_external_macro_cdylib_reports_host_driven() {
    use cerulion_core::graph::node::ExternalSource;

    let path = find_external_macro_cdylib();
    let mut node = DylibNodeEntry::load(&path).expect("load external macro cdylib");

    let tt = TestTransport::with_buffer_size(8);
    let cmd_pub = tt.publisher(
        "test/macro_cdylib/ext_cmd_local",
        MaxSliceLen::const_new(256),
        0,
    );
    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("cmd".to_string(), AnyPublisher::Ipc(cmd_pub));
    let ctx = NodeContext::for_tests(publishers, IndexMap::new());
    node.init(ctx).expect("init external cdylib");

    assert!(
        matches!(node.external_source(), Some(ExternalSource::HostDriven)),
        "external macro cdylib must report HostDriven via the cerulion_node_external_source FFI"
    );
    // Kind 3 (HostDriven) is NOT a drained doorbell (that is kind 2).
    assert!(
        !node.external_source_is_drained_doorbell_fd(),
        "HostDriven must not be flagged as a drained doorbell fd"
    );

    node.shutdown().expect("shutdown external cdylib");

    // Keep the transport alive until after shutdown (it owns the SHM the node's
    // publisher used).
    drop(tt);
}

/// Back-compat: a raw-FFI cdylib that does NOT export
/// `cerulion_node_external_source` still LOADS (the symbol is additive — the ABI
/// version is deliberately NOT bumped) and reports `external_source() == None`
/// with no crash. `external_source()` short-circuits on the absent symbol, so no
/// `init` is needed.
#[test]
fn test_raw_cdylib_without_external_source_symbol_is_none() {
    let path = find_raw_cdylib();
    let mut node = DylibNodeEntry::load(&path).expect("raw cdylib must still load (no ABI bump)");
    assert!(
        node.external_source().is_none(),
        "a cdylib lacking cerulion_node_external_source must yield external_source() == None"
    );
    assert!(
        !node.external_source_is_drained_doorbell_fd(),
        "no external source ⇒ not a drained doorbell fd"
    );
}

// ============================================================
// Cdylib PARITY — an Err tick through the FFI publishes NOTHING
// ============================================================

/// Find the failing-tick fixture cdylib (`test_node_failing_cdylib`) — a
/// `#[cerulion_node]` node with a FIXED-ONLY `Vector3` output whose tick
/// writes `self.out.x = 0.0` then returns `Err` in the default
/// `CER_FAIL_MODE=tick_logic` mode: the EXACT Err-tick shape (partial write
/// on a fixed-only schema — nothing trips the variable-field discard).
fn find_failing_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_failing_cdylib")
}

/// Cdylib parity (the no-inert-shipping rule): the discard
/// tail is generated INSIDE `__cer_zero_copy_tick`, which serves both the
/// in-process `<Name>Entry::tick` AND the FFI `cerulion_node_tick` (that
/// entry point calls `node.tick()` on the boxed entry) — so the guarantee must
/// hold through `DylibNodeEntry` over the C ABI too. Without the discard tail, every one of
/// the failed ticks below would ship a zero-init Vector3 frame.
///
/// Relies on the fixture's DEFAULT mode (`CER_FAIL_MODE` unset ⇒
/// `tick_logic`): no test in this binary mutates that env var, so no
/// serialization is needed.
#[test]
fn test_err_ticking_cdylib_publishes_nothing() {
    let path = find_failing_cdylib();
    let mut node = DylibNodeEntry::load(&path).expect("load failing cdylib");

    let tt = TestTransport::with_buffer_size(8);
    let out_pub = tt.publisher(
        "test/macro_cdylib/out_local",
        MaxSliceLen::const_new(256),
        0,
    );
    let out_sub = tt.subscriber("test/macro_cdylib/out_local");

    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("out".to_string(), AnyPublisher::Ipc(out_pub));
    let ctx = NodeContext::for_tests(publishers, IndexMap::new());
    node.init(ctx).expect("init failing cdylib");

    // N failed ticks — each must surface the fixture's USER error through
    // the FFI (a loan-exhaustion error here would mean the discard path
    // leaks iceoryx2 slots instead of releasing them).
    for i in 0..4 {
        let err = node
            .tick()
            .expect_err("fixture tick must Err (tick_logic mode)");
        assert!(
            err.to_string().contains("simulated failure"),
            "tick {i}: the FFI must carry the user error, got: {err}"
        );
    }
    node.shutdown().expect("shutdown failing cdylib");

    let mut count = 0usize;
    let _ = out_sub.try_receive(|_| count += 1);
    assert_eq!(
        count, 0,
        "cdylib parity: an Err tick through the C ABI must publish NOTHING \
         (without the discard tail: 4 zero-init Vector3 frames with valid headers)"
    );

    // Keep the transport alive until after shutdown.
    drop(tt);
}
