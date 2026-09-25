// SPDX-License-Identifier: AGPL-3.0-only

//! Cerulion: zero-copy middleware for real-time robotics.
//!
//! This is the umbrella crate. It carries no logic of its own: it re-exports
//! the three crates a Cerulion project uses, held at one version, so that a
//! project names one dependency instead of three and cannot end up with a
//! combination of them that was never released together.
//!
//! What it re-exports:
//!
//! - `cerulion::prelude` is what a node body imports: the node macros,
//!   `NodeError`, the port view types and the event types.
//! - `cerulion::msgs` is the generated ROS 2 message types, one module per
//!   package, and is where a port's type comes from.
//! - `cerulion::core` is the runtime itself: transport, scheduler and graph
//!   execution.
//! - `cerulion::macros` is the macro crate on its own, for a macro the prelude
//!   does not re-export.
//!
//! ```
//! use cerulion::msgs::geometry_msgs::Vector3;
//!
//! // A message type names a wire layout rather than holding data. A node
//! // reaches its fields through a port, writing straight into the loaned
//! // shared-memory slot, so the type itself carries nothing.
//! let _layout = Vector3::default();
//! ```
//!
//! # A package that writes nodes also names `cerulion_core`
//!
//! The node macros expand to absolute paths into the runtime crate, of the
//! form `::cerulion_core::graph::node::NodeEntry`. Rust resolves the first
//! segment of such a path only against a crate the consuming package names in
//! its own manifest, and a re-export cannot put one there, so a package that
//! uses the macros lists both:
//!
//! ```toml
//! [dependencies]
//! cerulion = "1.0.0"
//! cerulion_core = "1.0.0"
//! ```
//!
//! With those two, a node is a struct whose fields are its ports and whose
//! `tick` runs when the trigger policy says so. Writing a port field writes
//! straight into the loaned shared-memory slot; the frame is published when
//! `tick` returns `Ok`.
//!
//! ```
//! use cerulion::msgs::geometry_msgs::Vector3;
//! use cerulion::prelude::*;
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
//!         self.reading.x = f64::from(self.tick_count);
//!         Ok(())
//!     }
//! }
//! # fn main() {}
//! ```
//!
//! # Depending on the crates directly
//!
//! Naming [`cerulion_core`], [`cerulion_macros`] and [`native_ros2_messages`]
//! separately keeps working, and is what the Cerulion workspace itself does.

#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]
#![deny(missing_docs)]

pub use cerulion_core as core;
pub use cerulion_macros as macros;
pub use native_ros2_messages as msgs;

/// Everything a node body names, in one import.
///
/// This is [`cerulion_core::prelude`] plus the `CerulionState` derive, which
/// lives beside the runtime's state types rather than in its prelude. Import
/// it with a glob: `use cerulion::prelude::*;`.
///
/// Message types are deliberately not here. A port's type comes from
/// `cerulion::msgs`, one `use` per type, so that a node's source says which
/// messages it speaks.
pub mod prelude {
    pub use cerulion_core::prelude::*;
    pub use cerulion_core::state::CerulionState;
}
