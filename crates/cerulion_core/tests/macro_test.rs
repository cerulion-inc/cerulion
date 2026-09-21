// SPDX-License-Identifier: AGPL-3.0-only
//! Integration tests for `#[cerulion_node]` proc macro under the SHM-backed
//! transport API.
//!
//! Tests the generated `{Name}Entry` wrapper, `NodeEntry` impl, and lifecycle
//! (init, tick, shutdown). Legacy mode (`inputs(...)`/`outputs(...)` macro args
//! plus manual `tick(&mut NodeContext)`) was removed;
//! every `#[cerulion_node]` is now declarative.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test macro_test
//! ```

use cerulion_core::wire::MaxSliceLen;

use cerulion_core::graph::node::{AnyPublisher, AnySubscriber, NodeContext, NodeEntry};
use cerulion_core::prelude::*;
use cerulion_core::testing::TestTransport;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

// ============================================================
// Declarative-mode nodes
//
// True-zero-copy is the only declarative-mode dispatch
// path. Field types are the SHM markers (`Vector3`); `#[cerulion_node_impl]`
// rewrites every `self.<port>` access in `tick` to dispatch through a
// per-tick `OutputProxy<'_, Vector3>` / `InputView<'_, Vector3>`.
//
// `type_name` and the legacy `inputs(...)`/`outputs(...)`
// args were removed; ports come from `#[input]`/`#[output]` field attrs only.
// ============================================================

/// Declarative source: outputs a `Vector3` each tick (period-driven).
#[cerulion_node(period_ms = 100)]
struct DeclSource {
    #[output]
    count: Vector3,
    tick_count: u32,
}

#[cerulion_node_impl]
impl DeclSource {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.tick_count += 1;
        self.count.x = self.tick_count as f64;
        self.count.y = 0.0;
        self.count.z = 0.0;
        Ok(())
    }
}

/// Declarative processor: input + output, data-triggered.
#[cerulion_node]
struct DeclProcessor {
    #[input(trigger)]
    value_in: Vector3,
    #[output]
    value_out: Vector3,
}

#[cerulion_node_impl]
impl DeclProcessor {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.value_out.x = self.value_in.x * 2.0;
        self.value_out.y = self.value_in.y * 2.0;
        self.value_out.z = self.value_in.z * 2.0;
        Ok(())
    }
}

/// Declarative sink: input only, data-triggered.
#[cerulion_node]
struct DeclSink {
    #[input(trigger)]
    data: Vector3,
}

#[cerulion_node_impl]
impl DeclSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Touch the input so the rewriter wires the per-tick view in.
        let _ = self.data.x;
        Ok(())
    }
}

/// `depth = 64` (exactly `MAX_CONSUMER_DEPTH`) is the
/// accepted boundary — the trybuild `depth_above_max` case pins 65 as
/// the first rejected value. This node existing at all is the
/// compile-pass half of that boundary.
#[cerulion_node]
struct DeclMaxDepthSink {
    #[input(trigger, depth = 64)]
    data: Vector3,
}

#[cerulion_node_impl]
impl DeclMaxDepthSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.data.x;
        Ok(())
    }
}

#[test]
fn test_declarative_depth_at_max_compiles_and_carries_meta() {
    let info = DeclMaxDepthSinkEntry::new()
        .info()
        .expect("macro node info is infallible");
    let meta = info.input_meta();
    assert_eq!(meta.len(), 1);
    assert_eq!(meta[0].name, "data");
    assert_eq!(
        meta[0].depth,
        cerulion_core::graph::topology::MAX_CONSUMER_DEPTH,
        "depth = 64 (boundary) must flow through InputMeta to the runtime gate"
    );
}

/// Depth-default collapse: with the never-executed queue policy gone,
/// an `#[input]` with NO explicit `depth` uniformly carries
/// `DEFAULT_CONSUMER_DEPTH` in its emitted `InputMeta` (the macro emits the
/// const PATH, not a baked literal, so this cannot drift from the runtime).
#[test]
fn test_declarative_unspecified_depth_defaults_to_consumer_depth() {
    let info = DeclSinkEntry::new()
        .info()
        .expect("macro node info is infallible");
    let meta = info.input_meta();
    assert_eq!(meta.len(), 1);
    assert_eq!(meta[0].name, "data");
    assert_eq!(
        meta[0].depth,
        cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
        "unspecified depth must collapse to DEFAULT_CONSUMER_DEPTH"
    );
}

#[test]
fn test_declarative_source_info() {
    let entry = DeclSourceEntry::new();
    let info = entry.info().expect("info should parse");
    assert!(info.input_names().is_empty());
    assert_eq!(info.output_names(), vec!["count"]);
}

#[test]
fn test_declarative_processor_info() {
    let entry = DeclProcessorEntry::new();
    let info = entry.info().expect("info should parse");
    assert_eq!(info.input_names(), vec!["value_in"]);
    assert_eq!(info.output_names(), vec!["value_out"]);
}

#[test]
fn test_declarative_sink_info() {
    let entry = DeclSinkEntry::new();
    let info = entry.info().expect("info should parse");
    assert_eq!(info.input_names(), vec!["data"]);
    assert!(info.output_names().is_empty());
}

#[test]
fn test_declarative_source_publishes_via_loan_proxy() {
    // Wire a real in-process publisher for `count` so the zero-copy tick
    // can loan into SHM and we can observe the published frame.
    let topic = "test/macro/decl_source/count";
    let tt = TestTransport::with_buffer_size(8);
    let publisher = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut subscriber = tt.subscriber(topic);

    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("count".to_string(), AnyPublisher::Ipc(publisher));
    let ctx = NodeContext::for_tests(publishers, IndexMap::new());

    let mut entry = DeclSourceEntry::new();
    entry.init(ctx).expect("init");

    entry.tick().expect("tick 1");
    entry.tick().expect("tick 2");

    // User logic: tick_count counted up to 2 inside the node.
    assert_eq!(entry.inner.tick_count, 2);

    // Verify the second tick's publish landed on the wire (drain to last).
    let mut last_x: Option<f64> = None;
    while let Ok(Some(())) = subscriber.try_view::<Vector3, _>(|view| {
        last_x = Some(view.x);
    }) {}
    assert_eq!(
        last_x,
        Some(2.0),
        "expected x=2 on the final published frame"
    );
}

#[test]
fn test_declarative_node_is_send() {
    fn assert_send<T: Send>() {}
    assert_send::<DeclSourceEntry>();
    assert_send::<DeclProcessorEntry>();
    assert_send::<DeclSinkEntry>();
}

#[test]
fn test_declarative_node_as_dyn_node_entry() {
    let entry: Box<dyn NodeEntry> = Box::new(DeclProcessorEntry::new());
    let info = entry.info().expect("info should parse");
    assert_eq!(info.input_names(), vec!["value_in"]);
}

/// Declarative node returning `NodeError::Logic` from tick to verify error
/// propagation through the macro-emitted wrapper.
#[cerulion_node(period_ms = 100)]
struct DeclFailing {
    #[output]
    data: Vector3,
}

#[cerulion_node_impl]
impl DeclFailing {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Touch the output so the rewriter wires the proxy in.
        self.data.x = 0.0;
        Err(NodeError::Logic(
            "intentional declarative failure".to_string(),
        ))
    }
}

#[test]
fn test_declarative_error_propagation() {
    // Wire a publisher so the zero-copy tick can loan a proxy before the
    // user body runs (a missing publisher returns NodeError before our
    // intentional NodeError::Logic ever fires).
    let topic = "test/macro/decl_failing/data";
    let tt = TestTransport::new();
    let publisher = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("data".to_string(), AnyPublisher::Ipc(publisher));
    let ctx = NodeContext::for_tests(publishers, IndexMap::new());

    let mut entry = DeclFailingEntry::new();
    entry.init(ctx).expect("init");

    let result = entry.tick();
    let err = result.expect_err("declarative tick error should propagate");
    assert!(
        err.to_string().contains("intentional declarative failure"),
        "error should propagate: {}",
        err
    );
}

#[test]
fn test_declarative_reinit_after_shutdown() {
    let mut entry = DeclSinkEntry::new();

    // Wire a real subscriber so the zero-copy tick has somewhere to take a
    // sample handle from — the empty-channel `try_view` returns Ok(None)
    // (no error), but the AnySubscriber must still be present for the
    // re-init path to exercise the full lifecycle.
    let topic = "test/macro/decl_reinit/data";
    let tt = TestTransport::with_buffer_size(8);
    let _publisher = tt.publisher(topic, MaxSliceLen::const_new(256), 0);

    let sub1 = tt.subscriber(topic);
    let mut subs1: IndexMap<String, AnySubscriber> = IndexMap::new();
    subs1.insert("data".to_string(), AnySubscriber::Ipc(sub1));
    let ctx1 = NodeContext::for_tests(IndexMap::new(), subs1);
    entry.init(ctx1).expect("init");
    entry.tick().expect("tick");
    entry.shutdown().expect("shutdown");

    let sub2 = tt.subscriber(topic);
    let mut subs2: IndexMap<String, AnySubscriber> = IndexMap::new();
    subs2.insert("data".to_string(), AnySubscriber::Ipc(sub2));
    let ctx2 = NodeContext::for_tests(IndexMap::new(), subs2);
    entry.init(ctx2).expect("re-init");
    entry.tick().expect("tick after re-init");
}

#[test]
fn test_declarative_double_init_fails() {
    let mut entry = DeclSourceEntry::new();

    let ctx1 = NodeContext::for_tests(IndexMap::new(), IndexMap::new());
    entry.init(ctx1).expect("init");

    let ctx2 = NodeContext::for_tests(IndexMap::new(), IndexMap::new());
    let result = entry.init(ctx2);
    let err = result.expect_err("double init should fail");
    assert!(
        err.to_string().contains("double init") || err.to_string().contains("already initialized"),
        "error should mention double init: {}",
        err
    );
}

#[test]
fn test_declarative_node_default_node_info_has_no_node_type() {
    // NodeInfo no longer carries node_type. The macro's
    // generated info() returns only port names; the node type comes from
    // the folder at graph-load time.
    let info = DeclProcessorEntry::new().info().expect("info should parse");
    assert_eq!(info.input_names(), vec!["value_in"]);
    assert_eq!(info.output_names(), vec!["value_out"]);
    // input_meta/output_meta are present (no longer Option-wrapped) but
    // empty until we wire the macro to populate them with full meta.
    assert!(info.input_meta().is_empty() || info.input_meta().len() == 1);
    assert!(info.output_meta().is_empty() || info.output_meta().len() == 1);
}

/// The IN-PROCESS emitter half — a `#[cerulion_node]`'s generated
/// `info()` declares each `#[output]` port's fixed wire size, which is what
/// the recorder stamps into a bag channel instead of reading it out of a
/// workspace `schemas/` file that may have drifted.
///
/// Its own test because the cdylib arm cannot reach it: `macro_cdylib_test`
/// exercises the `gen_cdylib` JSON path, and the key-set parity gate slices
/// `gen_cdylib`'s source region only — so the `OutputMeta` builder chain
/// `gen_zero_copy_node_entry_impl` emits is covered by neither. Deleting
/// `.with_wire_fixed_size(..)` from the macro left the whole suite green
/// before this arm existed.
///
/// A HAND ORACLE (`geometry_msgs/Vector3` is three `float64`s ⇒ 24 bytes),
/// not the trait const, so an emitter that passed the WRONG const still
/// fails; the const is asserted separately as a drift guard on the oracle.
#[test]
fn an_in_process_macro_node_declares_its_output_wire_fixed_size() {
    let info = DeclSourceEntry::new().info().expect("info should parse");
    let meta = info
        .output_meta()
        .iter()
        .find(|m| m.name == "count")
        .expect("the declarative source declares one `#[output] count: Vector3`");
    // Drift guard FIRST, so a Vector3 layout change fails HERE naming the
    // cause rather than at the bare `Some(24)` below.
    assert_eq!(
        <Vector3 as ShmMessage>::WIRE_FIXED_SIZE,
        24,
        "the hand oracle below assumes geometry_msgs/Vector3 is 3 x float64"
    );
    assert_eq!(
        meta.wire_fixed_size,
        Some(24),
        "the in-process macro path must declare the port type's WIRE_FIXED_SIZE"
    );
}

// ============================================================
// `#[cerulion_node(external)]` external_source override
// ============================================================

/// A host-driven external ingress node — declares its `ExternalSource` via the
/// (now required) `external_source` method. The macro overrides
/// `NodeEntry::external_source()` to surface it.
#[cerulion_node(external)]
struct DeclExternalHostDriven {
    #[output]
    cmd: Vector3,
}

#[cerulion_node_impl]
impl DeclExternalHostDriven {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.cmd.x = 0.0;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn test_external_node_external_source_returns_host_driven() {
    // The macro-generated wrapper overrides `external_source()` for an external
    // node, surfacing the user's method as `Some(..)`.
    let mut node = DeclExternalHostDrivenEntry::new();
    assert!(
        matches!(node.external_source(), Some(ExternalSource::HostDriven)),
        "external node's external_source() must return Some(HostDriven)"
    );
}

#[test]
fn test_non_external_node_external_source_returns_none() {
    // A non-external node emits NO override and inherits the trait default
    // (`None`) — no `external_source` method is required or allowed on it.
    let mut node = DeclSourceEntry::new();
    assert!(
        node.external_source().is_none(),
        "non-external node must inherit the default external_source() == None"
    );
}
