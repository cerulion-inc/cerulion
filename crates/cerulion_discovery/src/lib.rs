// SPDX-License-Identifier: AGPL-3.0-only
//! LAN peer discovery types and helpers, shared by the `cerulion` CLI's discovery
//! ladder and the `cerulion-netd` gateway daemon.
//!
//! - [`ladder`]: [`ladder::DiscoveredPeer`] and [`ladder::DiscoveryRung`] (a robot
//!   found on the network and how it was found), the locator parser and dedupe key,
//!   the bounded TCP reachability filter ([`ladder::probe_reachable_locators`]), and
//!   the pure connect-set planner ([`ladder::plan_connect_set`]).
//! - [`peer_cache`]: the `~/.cerulion/peers.json` format and its reader.
//!
//! This is an internal building block, published because `cerulion_cli_engine` and
//! `cerulion_netd` both depend on it. It opens no zenoh session and runs no mDNS
//! browse; a user reaches it through `cerulion topic list` and through any command
//! that asks `cerulion-netd` for a remote topic.
//!
//! # What is deliberately not here
//!
//! The mDNS, hostname and subnet-sweep rungs, the parallel gather engine and its
//! budgets, and the peer-cache writer stay in `cerulion_cli_engine`. The writer
//! split is a capability boundary, not a packaging convenience: a cache write must
//! be backed by evidence that a gather confirmed a robot live, netd runs no gather,
//! and so netd links a crate that has no writer to call. The dependency graph
//! enforces the invariant rather than a comment.
//!
//! Design notes for contributors live in `docs/internals/network-daemons.md` in the
//! repository (the crate boundaries section).

// Maintainer notes (plain comments, not rendered). Why this is a crate of its own:
// `cerulion_cli_engine` depends on `cerulion_netd` (for `NetdClient`), so netd
// cannot import the ladder from the CLI engine: that edge is cyclic. The repo's
// precedent is `cerulion_wireclient`, extracted from `cerulion_connectd` for exactly
// this reason. So the pieces BOTH sides need live here, once. The writer names that
// stayed behind are `save_peers`, `upsert_confirmed` and `record_confirmed`, gated
// by the CLI ladder's `resolve_write_back`; the gather engine is `gather_rungs`.

// Principle 12 (logging; see the Logging convention in `AGENTS.md`): library
// code never prints. It logs through `tracing`. Scoped `not(test)` so unit
// tests keep printing diagnostics, and applied at the crate root rather than
// in `[workspace.lints]` because that table cannot distinguish a lib target
// from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

pub mod ladder;
pub mod peer_cache;
pub mod robot_state;
