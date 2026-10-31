// SPDX-License-Identifier: AGPL-3.0-only
//! The KIND of a node's error crosses the node macro boundary, and the cdylib
//! boundary, as a typed value.
//!
//! A node returns `Result<(), NodeError>`; the runtime receives a
//! `TransportError`. These tests pin that every `NodeError` arm arrives as
//! `TransportError::NodeFailed` with the matching `NodeErrorKind`, from `init`,
//! `tick` and `shutdown`, for a node compiled into the test (the macro form).
//! The cdylib form is `node_error_kind_cdylib_test.rs`, with the same vectors.
//!
//! The expected kinds and messages are hand-written below; nothing is derived
//! from the code under test.

use cerulion_core::graph::node::{AnyPublisher, NodeContext, NodeEntry};
use cerulion_core::prelude::*;
use cerulion_core::testing::TestTransport;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

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

// ------------------------------------------------------------------
// The macro form: the same node shape, compiled into this test.
// ------------------------------------------------------------------

fn error_for(kind: &str) -> Option<NodeError> {
    Some(match kind {
        "logic" => NodeError::Logic("probe logic".to_string()),
        "invalid_input" => NodeError::InvalidInput {
            input: "probe".to_string(),
            reason: "probe invalid".to_string(),
        },
        "fatal" => NodeError::Fatal("probe fatal".to_string()),
        "custom" => NodeError::Custom(Box::new(std::io::Error::other("probe custom"))),
        "transport" => NodeError::Transport(TransportError::NotInitialized),
        "io" => NodeError::Io(std::io::Error::other("probe io")),
        _ => return None,
    })
}

#[cerulion_node(period_ms = 10)]
struct InProcessProbe {
    #[output]
    out: Vector3,
    kind: String,
    at: String,
}

#[cerulion_node_impl]
impl InProcessProbe {
    fn init(&mut self, _ctx: &mut NodeContext) -> Result<(), NodeError> {
        if self.at == "init" {
            if let Some(e) = error_for(&self.kind) {
                return Err(e);
            }
        }
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 1.0;
        if self.at == "tick" {
            if let Some(e) = error_for(&self.kind) {
                return Err(e);
            }
        }
        Ok(())
    }

    fn shutdown(&mut self) -> Result<(), NodeError> {
        if self.at == "shutdown" {
            if let Some(e) = error_for(&self.kind) {
                return Err(e);
            }
        }
        Ok(())
    }
}

fn context_with_out(tt: &TestTransport, topic: &str) -> NodeContext {
    let publisher = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("out".to_string(), AnyPublisher::Ipc(publisher));
    NodeContext::for_tests(publishers, IndexMap::new())
}

fn assert_typed(err: &TransportError, kind: NodeErrorKind, message: &str, site: &str) {
    assert_eq!(
        err.node_error_kind(),
        Some(kind),
        "{site}: kind must cross the boundary typed; got {err:?}"
    );
    match err {
        TransportError::NodeFailed { reason, .. } => {
            // The macro boundary sets the reason to `NodeError::to_string()`
            // and nothing else, so the whole text is pinned, not a suffix.
            assert_eq!(reason, message, "{site}: message must cross intact");
        }
        other => panic!("{site}: want NodeFailed, got {other:?}"),
    }
}

fn probe(kind: &str, at: &str) -> InProcessProbeEntry {
    InProcessProbeEntry::with_state(InProcessProbe {
        kind: kind.to_string(),
        at: at.to_string(),
        ..Default::default()
    })
}

#[test]
fn every_arm_returned_from_tick_crosses_the_macro_boundary_typed() {
    let tt = TestTransport::new();
    for (i, (word, kind, message)) in VECTORS.iter().enumerate() {
        let mut entry = probe(word, "tick");
        entry
            .init(context_with_out(&tt, &format!("test/kind/tick/{i}")))
            .expect("init");
        let err = entry.tick().expect_err("tick must fail");
        assert_typed(&err, *kind, message, &format!("macro tick {word}"));
    }
}

#[test]
fn every_arm_returned_from_init_and_shutdown_crosses_the_macro_boundary_typed() {
    let tt = TestTransport::new();
    for (i, (word, kind, message)) in VECTORS.iter().enumerate() {
        let mut entry = probe(word, "init");
        let err = entry
            .init(context_with_out(&tt, &format!("test/kind/init/{i}")))
            .expect_err("init must fail");
        assert_typed(&err, *kind, message, &format!("macro init {word}"));

        let mut entry = probe(word, "shutdown");
        entry
            .init(context_with_out(&tt, &format!("test/kind/shutdown/{i}")))
            .expect("init");
        let err = entry.shutdown().expect_err("shutdown must fail");
        assert_typed(&err, *kind, message, &format!("macro shutdown {word}"));
    }
}

#[test]
fn a_successful_tick_returns_ok_and_carries_no_kind() {
    // Edge: the boundary changes nothing on the non-error path.
    let tt = TestTransport::new();
    let mut entry = probe("", "tick");
    entry
        .init(context_with_out(&tt, "test/kind/ok"))
        .expect("init");
    entry.tick().expect("a node with no error ticks Ok");
}
