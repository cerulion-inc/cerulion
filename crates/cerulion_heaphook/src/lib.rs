// SPDX-License-Identifier: AGPL-3.0-only
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]
//! `libcerulion_heaphook.so`: the borrow-window heap hook.
//!
//! # What it is
//!
//! An `LD_PRELOAD` payload that interposes the `malloc` family so that, while a
//! thread-local **borrow window** is armed, a stock ROS 2 node's `std::vector` or
//! `std::string` fill lands directly in the tail of a loaned shared-memory slot
//! instead of the private heap. The `cerulion ros2 run` and `cerulion ros2 launch`
//! verbs inject this `.so` automatically (they look for `libcerulion_heaphook.so`
//! beside the `cerulion` binary); `rmw_cerulion` resolves the versioned handshake
//! ([`abi::HeaphookAbi`]) with `dlsym` at init and drives the window. It is not
//! published to crates.io: it is consumed as a preloaded shared object, never as a
//! dependency.
//!
//! # Platform boundary
//!
//! The interposition and FFI export layer is **Linux and GNU only**: dynamically
//! linked **glibc 2.34 or newer** and **libstdc++** with the default
//! `std::allocator`. macOS (SIP and libc++) and any other malloc interposer
//! (jemalloc, tcmalloc, ASan) are out of scope; the handshake degrades to the copy
//! path, loudly, and never fails a publish.
//!
//! ## Gating choice
//!
//! Per-module `cfg`, **not** a whole-crate `#![cfg]`:
//!
//! * The pure state machines ([`recursion`], [`registry`], [`window`], [`classify`],
//!   [`abi`]) carry **no `cfg`** and build and unit-test on EVERY platform, with
//!   oracle-vector tests. They are the whole of the crate's logic.
//! * The interposition and FFI exports (`interpose`, `exports`) are gated to
//!   `cfg(all(target_os = "linux", target_env = "gnu"))`. Elsewhere the crate is the
//!   pure core alone: it compiles, but the `cdylib` exports nothing usable as a
//!   preload.
//!
//! This split is what lets the logic be proven where it is written, while the
//! subprocess end-to-end test (which needs a real Linux and `LD_PRELOAD`) runs on
//! the Linux CI jobs; see `tests/heaphook_e2e_test.rs`.
//!
//! # Soundness in one line
//!
//! The window is an API event (arm and disarm); adoption is an EXACT half-open
//! address test; every escape (growth past the tail, reallocation, wrong thread,
//! foreign allocator) is detected exactly and recorded for the rmw to read. There is
//! no outcome in which a subscriber view is silently corrupted.
//!
//! Design notes for contributors live in `docs/internals/rmw.md` in the repository.

pub mod abi;
pub mod classify;
pub mod recursion;
pub mod registry;
pub mod window;

// The interposition + FFI export layer: Linux + GNU (glibc/libstdc++) only.
#[cfg(all(target_os = "linux", target_env = "gnu"))]
mod exports;
#[cfg(all(target_os = "linux", target_env = "gnu"))]
mod interpose;
#[cfg(all(target_os = "linux", target_env = "gnu"))]
mod state;
