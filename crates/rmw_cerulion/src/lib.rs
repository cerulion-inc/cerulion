// SPDX-License-Identifier: AGPL-3.0-only
//! `rmw_cerulion`: a ROS 2 rmw implementation over the Cerulion zero-copy
//! transport.
//!
//! `RMW_IMPLEMENTATION=rmw_cerulion` swaps ROS 2's middleware for Cerulion's
//! iceoryx2-backed shared-memory transport with the native flat wire format: no
//! DDS, and no CDR serialization. You normally do not set that variable by hand:
//! `cerulion ros2 run` and `cerulion ros2 launch` stage it and then run the stock
//! `ros2` command. The crate is not published to crates.io; it builds as the C-ABI
//! shared library ROS 2 loads, against the headers of the installed distribution
//! (see `docs/ros2_compatibility.md` in the repository).
//!
//! - **Fixed-size plain messages** (`Pose`-class, after nested resolution): true
//!   zero-copy through the rmw loaned-message API in BOTH directions. The pointer
//!   rclcpp writes through IS the shared-memory slot, and the pointer a loaned take
//!   reads through IS the received frame's payload, with the iceoryx2 sample held
//!   across the C ABI until the loan returns.
//! - **Variable messages**: flatten (scattered C strings and sequences into one flat
//!   frame buffer) plus one memcpy into the shared-memory loan inside `publish_raw`.
//!   No serialization or CDR step exists at all. When the borrow-window heap hook
//!   (`cerulion_heaphook`) is preloaded, a message with an unbounded primitive
//!   sequence can also be filled directly in the loaned slot.
//! - **Interop**: frames carry the SAME fully qualified schema hashes and layouts as
//!   `native_ros2_messages`, so native Cerulion nodes subscribe to MoveIt topics
//!   zero-copy, and `cerulion topic list`, `echo` and `hz` introspect ROS traffic
//!   natively.
//! - **Services and actions**: carried by the deterministic request and response
//!   layer of `cerulion_core` (actions are services plus topics at the rcl level; no
//!   rmw action primitive exists).
//!
//! # What this is NOT
//!
//! An rmw moves bytes; it does not schedule callbacks. It does not make a ROS 2
//! process deterministic: callback scheduling stays with the ROS 2 executor, and
//! OMPL's sampling-based planners remain probabilistic by design. That boundary is
//! explicit and intentional.
//!
//! Design notes for contributors (FFI invariants, type bridges, test map) live in
//! `docs/internals/rmw.md` in the repository.

// P12 (the logging convention in AGENTS.md): library code never prints. It
// logs through `tracing`. Scoped `not(test)` so unit tests keep printing
// diagnostics, and applied at the crate root rather than in `[workspace.lints]`
// because that table cannot distinguish a lib target from a test binary.
// Pinned by `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]
// A ROS 2 rmw shared object for Linux/macOS: the event-driven `rmw_wait`
// blocks on unix fds (`cerulion_core::wake::{Doorbell, FdWakeSet}`), the
// timer-slack prctl and the iceoryx2 SHM transport are unix, and ROS 2 on
// Windows ships its own rmw packaging. Gating the CRATE (rather than
// stubbing a non-unix legacy-wait arm nothing builds or tests) keeps
// `cargo check` meaningful on a non-unix target: the crate is simply absent.
#![cfg(unix)]

pub mod adopt_take;
pub mod borrow_degrade_latch;
pub mod bridge;
pub mod decode_failure_latch;
pub mod era;
pub mod era_check;
pub mod ffi;
pub mod heaphook;
pub mod loan_refusal_latch;
pub mod publish_reject_latch;
pub mod runtime;
pub mod take_gate;
#[cfg(feature = "test-seams")]
pub mod test_seams;
pub mod type_bridge;
pub mod type_bridge_cpp;

mod api;

pub use api::*;
