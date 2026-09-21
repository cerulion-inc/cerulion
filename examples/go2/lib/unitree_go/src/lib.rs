// SPDX-License-Identifier: AGPL-3.0-only
//! `unitree_go` ROS 2 message types for the Go2 demo.
//!
//! The Go2 publishes types that are NOT part of the stock ROS 2 corpus
//! `native_ros2_messages` vendors. Their `.msg` definitions live in the demo
//! workspace's schema store (`examples/go2/schemas/unitree_go/msg/`), which is
//! also what `graphs/go2.bridge.yaml`'s `msg_dirs:` points the `dds_bridge`
//! generic codec at. This crate compiles that SAME text into real Cerulion
//! message types (via `build.rs`) so a node can declare an `#[input]` /
//! `#[output]` of one — the `#[cerulion_node]` macro needs a type it can read
//! `<T as ShmMessage>::SCHEMA_HASH` off, and a `.msg` file is not one.
//!
//! Because both sides run the same `parse_rosmsg -> resolve_fixed_nested`
//! pipeline over the same file, the hash a port declares here is the hash the
//! bridge stamps into the frames it publishes. Edit the `.msg`, and BOTH move
//! together; there is no second copy of the layout to drift.
//!
//! Currently generated: `Go2FrontVideoData` (the `/frontvideostream` camera
//! sample — see the store file's header for how its layout was established
//! from 60 live captures).

// Nothing hand-written lives here; the whole crate IS the generated module,
// so the crate-level attributes the generated text needs live here (an inner
// attribute inside an `include!`d file is a hard error). No
// `#![forbid(unsafe_code)]`: the SHM-backed accessors codegen emits are
// pointer casts over the loaned slot — the same `unsafe` every
// `native_ros2_messages` type carries.
// P12 (the logging convention in AGENTS.md): library code never prints. It
// logs through `tracing`. Scoped `not(test)` so unit tests keep printing
// diagnostics, and applied at the crate root rather than in `[workspace.lints]`
// because that table cannot distinguish a lib target from a test binary.
// Pinned by `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]
#![allow(dead_code, unused_imports)]

include!(concat!(env!("OUT_DIR"), "/unitree_go.rs"));
