// SPDX-License-Identifier: AGPL-3.0-only
//! Node-local adapters for the immutable C boundary.
//!
//! The host alone creates and drops transport, queues and sessions. This module
//! owns only node-local state, copied callback tables, names and typed accessor
//! borrows over host-held leases. Native context layouts remain unchanged.

mod accumulate;
mod bytes;
mod client;
mod context;
mod exports;
mod input;
mod metadata;
mod output;
mod raw_frame;
mod raw_ingress;
mod registry;
mod state;

pub use accumulate::AccumulatorSet;
pub use client::HostClient;
pub use context::{AnyPublisher, AnySubscriber, NodeContext};
pub use exports::{NodeExports, PluginNode};
pub use input::{Accumulated, InputView};
pub use metadata::{metadata, metadata_with_accumulation, raw_metadata};
pub use output::{LazyOutput, OutputProxy};
pub use raw_frame::{RawFrameLoan, RawFrameRead};
pub use raw_ingress::{RawIngress, RawPublisher};
pub use registry::NodeRegistry;

use crate::error::TransportError;

pub(super) fn status(code: u32) -> crate::node_abi::Status {
    crate::node_abi::Status {
        code,
        ..crate::node_abi::Status::default()
    }
}

// hot-path-alloc-ok-fn: malformed boundary arguments construct a diagnostic only on rejection.
fn argument_error(reason: &str) -> TransportError {
    TransportError::NodeError {
        node_id: "node boundary".into(),
        reason: reason.into(),
    }
}

// hot-path-alloc-ok-fn: malformed spans allocate diagnostic text only after validation refuses them.
fn span_error(topic: &str, error: bytes::SpanError) -> TransportError {
    TransportError::Deserialization {
        topic: topic.into(),
        reason: format!("invalid borrowed byte span: {error:?}"),
    }
}

// hot-path-alloc-ok-fn: corrupt recorded state constructs one diagnostic on the restore cold path.
fn state_error(error: crate::state::StateError) -> TransportError {
    TransportError::GraphError {
        reason: error.to_string(),
    }
}
