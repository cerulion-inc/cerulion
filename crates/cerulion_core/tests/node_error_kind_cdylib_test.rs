// SPDX-License-Identifier: AGPL-3.0-only
//! The KIND of a node's error crosses the cdylib boundary as a typed value.
//!
//! The `test_node_error_kind_cdylib` fixture returns each `NodeError` arm
//! from `init`, `tick` and `shutdown`; loaded through `DylibNodeEntry`, every
//! one must arrive as `TransportError::NodeFailed` with the matching
//! `NodeErrorKind` (the optional `cerulion_take_last_error_kind` export). A
//! hand-written cdylib that does not export that symbol still loads and still
//! reports an untyped error. Same hand-written vectors as the macro form in
//! `node_error_kind_boundary_test.rs`.
//!
//! Its own binary because, on macOS, upstream iceoryx2 0.10.0 defect 2034
//! refuses new iceoryx2 resources in a process once a plugin node has run.
//!
//! Prereq: `cargo build -p test_node_error_kind_cdylib -p test_node_cdylib`.

use std::collections::HashMap;
use std::sync::Arc;

use cerulion_core::clock::RealClock;
use cerulion_core::graph::node::{
    AnyPublisher, DylibNodeEntry, NodeContext, NodeEntry, ShutdownSignal,
};
use cerulion_core::prelude::*;
use cerulion_core::testing::TestTransport;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use serial_test::serial;

/// `(CERULION_TEST_ERROR_KIND word, expected kind, expected message)`, hand-written.
const VECTORS: &[(&str, NodeErrorKind, &str)] = &[
    ("logic", NodeErrorKind::Logic, "Logic error: probe logic"),
    (
        "invalid_input",
        NodeErrorKind::InvalidInput,
        "Invalid input 'probe': probe invalid",
    ),
    ("fatal", NodeErrorKind::Fatal, "Fatal error: probe fatal"),
    ("custom", NodeErrorKind::Custom, "probe custom"),
    (
        "transport",
        NodeErrorKind::Transport,
        "Transport error: Transport manager not initialized. Call TransportManager::init() before creating publishers or subscribers.",
    ),
    ("io", NodeErrorKind::Io, "I/O error: probe io"),
];

fn assert_typed(err: &TransportError, kind: NodeErrorKind, message: &str, site: &str) {
    assert_eq!(
        err.node_error_kind(),
        Some(kind),
        "{site}: kind must cross the boundary typed; got {err:?}"
    );
    match err {
        TransportError::NodeFailed { reason, .. } => {
            if site.contains(" init ") {
                // A failed init keeps its phase label (it refuses the build).
                assert!(
                    reason.ends_with(message),
                    "{site}: message must cross intact; want suffix {message:?}, got {reason:?}"
                );
            } else {
                // Tick and shutdown carry the node's own message, exactly
                // the text an in-process macro node reports: the FFI
                // wrapper's phase label is removed on the typed path.
                assert_eq!(reason, message, "{site}: message must cross intact");
            }
        }
        other => panic!("{site}: want NodeFailed, got {other:?}"),
    }
}

// ------------------------------------------------------------------

fn cdylib_context(tt: &TestTransport, topic: &str, kind: &str, at: &str) -> NodeContext {
    let publisher = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("out".to_string(), AnyPublisher::Ipc(publisher));
    let mut env = HashMap::new();
    env.insert("CERULION_TEST_ERROR_KIND".to_string(), kind.to_string());
    env.insert("CERULION_TEST_ERROR_AT".to_string(), at.to_string());
    NodeContext::with_runtime_env(
        publishers,
        IndexMap::new(),
        Arc::new(RealClock),
        ShutdownSignal::new(),
        Arc::new(env),
    )
}

/// One body for every cdylib arm: the fixture is one dlopen'd library with a
/// process-wide `NODES` map, so the phases run in a fixed order in one test.
/// Every context (and so every iceoryx2 publisher) is built BEFORE the first
/// cdylib call: on macOS, upstream iceoryx2 0.10.0 defect 2034 refuses new
/// iceoryx2 resources in a process once a plugin node has run.
#[test]
#[serial]
fn every_arm_crosses_the_cdylib_boundary_typed_from_init_tick_and_shutdown() {
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_error_kind_cdylib");
    let tt = TestTransport::new();
    let mut contexts: Vec<[NodeContext; 3]> = VECTORS
        .iter()
        .enumerate()
        .map(|(i, (word, _, _))| {
            [
                cdylib_context(&tt, &format!("test/kind/cdylib/tick/{i}"), word, "1"),
                cdylib_context(&tt, &format!("test/kind/cdylib/init/{i}"), word, "init"),
                cdylib_context(
                    &tt,
                    &format!("test/kind/cdylib/shutdown/{i}"),
                    word,
                    "shutdown",
                ),
            ]
        })
        .collect();
    for ((word, kind, message), [tick_ctx, init_ctx, shutdown_ctx]) in
        VECTORS.iter().zip(contexts.drain(..))
    {
        // tick
        let mut node = DylibNodeEntry::load(&path).expect("load");
        node.init(tick_ctx).expect("init");
        let err = node.tick().expect_err("tick must fail");
        assert_typed(&err, *kind, message, &format!("cdylib tick {word}"));
        node.shutdown().expect("shutdown");

        // init
        let mut node = DylibNodeEntry::load(&path).expect("load");
        let err = node.init(init_ctx).expect_err("init must fail");
        assert_typed(&err, *kind, message, &format!("cdylib init {word}"));

        // shutdown
        let mut node = DylibNodeEntry::load(&path).expect("load");
        node.init(shutdown_ctx).expect("init");
        node.tick().expect("tick is clean in shutdown mode");
        let err = node.shutdown().expect_err("shutdown must fail");
        assert_typed(&err, *kind, message, &format!("cdylib shutdown {word}"));
    }
}

/// The hand-written `test_node_cdylib` exports no
/// `cerulion_take_last_error_kind`: it is what an older node looks like. It
/// must still load, and a failing tick stays the UNTYPED `NodeError` (so it
/// takes today's non-fatal path).
#[test]
#[serial]
fn a_cdylib_without_the_kind_export_loads_and_reports_an_untyped_error() {
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_cdylib");
    std::env::set_var("CER_TICK_FAIL", "1");
    struct Unset;
    impl Drop for Unset {
        fn drop(&mut self) {
            std::env::remove_var("CER_TICK_FAIL");
        }
    }
    let _unset = Unset;

    let mut node = DylibNodeEntry::load(&path).expect("an older cdylib still loads");
    node.init(NodeContext::for_tests(IndexMap::new(), IndexMap::new()))
        .expect("init");
    let err = node.tick().expect_err("tick fails in CER_TICK_FAIL mode");
    assert_eq!(err.node_error_kind(), None, "no export, no kind: {err:?}");
    assert!(
        matches!(err, TransportError::NodeError { .. }),
        "an older cdylib reports the untyped NodeError; got {err:?}"
    );
}
