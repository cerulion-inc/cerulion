// SPDX-License-Identifier: AGPL-3.0-only
//! The one import a node crate writes: `use cerulion_core::prelude::*;`.
//!
//! # What a node author uses from here
//!
//! - The node macros [`cerulion_node`] and [`cerulion_node_impl`]. (The third
//!   macro, `#[derive(CerulionState)]`, is
//!   [`cerulion_core::state::CerulionState`](crate::state::CerulionState).)
//! - [`NodeError`], the error type `tick`, `init` and `shutdown` return.
//!   `TransportError` and `std::io::Error` convert into it through `?`.
//! - [`NodeContext`], handed to `init`: `ctx.env(..)` and `ctx.env_str(..)`
//!   read the node's configuration.
//! - [`FillFrom`] and [`SliceSource`], for writing a variable-length field in
//!   place with `self.<port>.<field>.fill_from(producer)?`.
//! - [`ExternalSource`], returned by `external_source()` on an
//!   `#[cerulion_node(external)]` node.
//! - The event types an `#[on_event]` handler takes: [`BackpressureEvent`],
//!   [`ExpectWithinEvent`], [`PromiseWithinEvent`], [`LivelinessEvent`].
//! - [`tracing`], for logging. A node is a library the runtime loads: it logs
//!   through `tracing` and never prints.
//!
//! Message types are not here. They come from the `native_ros2_messages`
//! crate, one `use` per type.
//!
//! # What else is re-exported, and is not for node code
//!
//! The prelude also re-exports runtime types: [`TransportManager`],
//! [`TransportConfig`], [`CerulionPublisher`], [`CerulionSubscriber`],
//! [`Scheduler`], [`NodeHandle`], [`NodeConfig`], [`TriggerPolicy`],
//! [`GraphRuntime`], [`GraphConfig`], [`NodeDef`], [`NodeEntry`], [`NodeInfo`],
//! [`AnyPublisher`], [`AnySubscriber`], the clocks, [`NetworkManager`] and
//! their configuration types. The `cerulion` CLI and the framework's own
//! tests drive those. A node never constructs or calls them: the runtime is
//! started by `cerulion graph run`, the graph is `graphs/<name>.yaml`, and a
//! node reads time with `self.now_ns()` rather than through a clock object.
//! They can change between releases without notice. They are visible through
//! this import, but nothing in a node's source should name them.
//!
//! # A data-triggered node
//!
//! One node type per crate, at `nodes/safety_controller/src/lib.rs`. The
//! single `#[input(trigger)]` field is the trigger policy, so the macro takes
//! no argument:
//!
//! ```rust
//! use cerulion_core::prelude::*;
//! use native_ros2_messages::geometry_msgs::Vector3;
//! use native_ros2_messages::sensor_msgs::LaserScan;
//!
//! #[cerulion_node]
//! #[derive(Default)]
//! struct SafetyControllerNode {
//!     #[input(trigger, depth = 1, expect_within_ms = 100)]
//!     scan: LaserScan,
//!     #[output(promise_within_ms = 100)]
//!     linear_velocity: Vector3,
//! }
//!
//! #[cerulion_node_impl]
//! impl SafetyControllerNode {
//!     fn tick(&mut self) -> Result<(), NodeError> {
//!         let ranges = self.scan.ranges();
//!         let stop = ranges.is_empty() || ranges.iter().any(|&r| r.is_nan() || r < 0.5);
//!         self.linear_velocity.x = if stop { 0.0 } else { 0.3 };
//!         Ok(())
//!     }
//! }
//! # fn main() {}
//! ```
//!
//! # Configuration, and an event handler
//!
//! ```rust
//! use cerulion_core::prelude::*;
//! use native_ros2_messages::geometry_msgs::Vector3;
//!
//! #[cerulion_node]
//! #[derive(Default)]
//! struct GainNode {
//!     #[input(trigger, backpressure = sample(10))]
//!     value_in: Vector3,
//!     #[output]
//!     value_out: Vector3,
//!     gain: f64,
//!     decimated: u64,
//! }
//!
//! #[cerulion_node_impl]
//! impl GainNode {
//!     fn init(&mut self, ctx: &mut NodeContext) -> Result<(), NodeError> {
//!         // Read from the snapshot of the environment taken when the graph
//!         // was built, never from live `std::env`, so a replay reads the
//!         // same value.
//!         self.gain = ctx.env("GAIN", 2.0);
//!         Ok(())
//!     }
//!
//!     fn tick(&mut self) -> Result<(), NodeError> {
//!         self.value_out.x = self.value_in.x * self.gain;
//!         Ok(())
//!     }
//!
//!     #[on_event(input = "value_in")]
//!     fn on_value_in_backpressure(&mut self, _event: BackpressureEvent) {
//!         self.decimated += 1;
//!     }
//! }
//! # fn main() {}
//! ```
//!
//! Wiring these two types into a running system is not code. It is
//! `cerulion node stage <type> -g <graph>` and the `inputs:` / `outputs:` of
//! `graphs/<graph>.yaml`.

// Core types
pub use crate::error::{NodeError, NodeResult, TransportError, TransportResult};
pub use crate::message::ShmMessage;
pub use crate::transport::fill_from::{FillFrom, SliceSource};
pub use crate::transport::input_view::InputView;
pub use crate::transport::output_proxy::OutputProxy;
pub use crate::wire::{MaxSliceLen, WireError, WireHeader};

// Runtime: transport layer types (not node-author API, see the module doc)
pub use crate::clock::{real_ns, thread_cpu_ns, Clock, ExternalClock, RealClock};
pub use crate::transport::publisher::CerulionPublisher;
pub use crate::transport::subscriber::{CerulionSubscriber, ReceivedMessage};
pub use crate::transport::{TransportConfig, TransportManager};

// Runtime: scheduler types (not node-author API)
pub use crate::clock::VirtualClock;
pub use crate::scheduler::{NodeConfig, NodeHandle, Scheduler, TriggerPolicy};

// Backpressure event surfaced to `#[on_event(input = "...")]`
// user callbacks (with a `BackpressureEvent` parameter).
// Must be in the prelude so a handler signature
// `fn on_x(&mut self, event: BackpressureEvent)` resolves with
// `use cerulion_core::prelude::*;`.
pub use crate::scheduler::BackpressureEvent;

// Reactable QoS watchdog events drained from a node
// body via `ctx.take_{expect,promise}_within_event(name)`. In the prelude
// so a handler signature naming them resolves with
// `use cerulion_core::prelude::*;` (the `#[on_event]` dispatch references
// these too).
pub use crate::scheduler::{ExpectWithinEvent, PromiseWithinEvent};

// Input-scoped liveliness event surfaced to
// `#[on_event(input = "...")]` user callbacks (with a `LivelinessEvent`
// parameter). In the prelude so a handler signature
// `fn on_x(&mut self, ev: LivelinessEvent)` resolves with
// `use cerulion_core::prelude::*;`.
pub use crate::scheduler::{LivelinessCause, LivelinessEvent, LivelinessState};

// Node-facing (`NodeContext`, `ExternalSource`) and runtime graph/node types
#[cfg(any(test, feature = "test-helpers"))]
pub use crate::graph::node::ClosureNodeEntry;
pub use crate::graph::node::{
    AnyPublisher, AnySubscriber, BackpressurePolicy, ExternalSource, InputMeta, NodeContext,
    NodeEntry, NodeInfo, OutputMeta, ShutdownSignal,
};

// Runtime graph types, then the node macros
pub use crate::graph::{GraphConfig, GraphRuntime, NodeDef};
pub use crate::transport::events::PubSubEvent;
pub use cerulion_macros::{cerulion_node, cerulion_node_impl};

// Runtime: network transport (not node-author API)
pub use crate::transport::network::{NetworkConfig, NetworkManager, ZenohMode};

// Re-export tracing for logging in nodes
pub use tracing;
