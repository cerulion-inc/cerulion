// SPDX-License-Identifier: AGPL-3.0-only
//! Holds the `#[cerulion_node]` diagnostics suite; the library itself is empty.
//!
//! `tests/compile_fail.rs` runs every `tests/ui/*.rs` fixture through trybuild
//! and compares its compiler output with the `.stderr` beside it. Each
//! `tests/ui/pass/*.rs` program is an example target: `cargo test` builds it
//! and `tests/pass_programs.rs` runs it.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]
