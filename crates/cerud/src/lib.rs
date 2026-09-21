// SPDX-License-Identifier: AGPL-3.0-only
//! `cerud`: the open (AGPL) robot-side Cerulion operations service.
//!
//! `cerud` runs on the robot and exposes a small set of **mechanical** operations
//! verbs over a versioned, framed request and response protocol. It is deliberately
//! dumb: all intelligence (which robot, when, why) lives on the app side and the
//! cloud side. `cerud` only does the mechanical thing it is asked to do, after
//! checking that the caller is authorized to ask, and records every invocation to a
//! tamper-evident audit log.
//!
//! It is not published to crates.io: it is a service an operator deploys, and the
//! robot-side remote-access daemon (`cerulion_remoted`) serves it over the iroh
//! `cerulion/ops/1` protocol.
//!
//! # Modules
//!
//! - [`protocol`]: framed request and response with protocol-version negotiation at
//!   connect, plus a per-verb authorization seam re-checked server-side on every
//!   call.
//! - [`transport`]: a [`transport::OpsListener`] seam over "framed bidirectional
//!   bytes", plus a Unix-domain-socket development transport. `cerulion_remoted`
//!   plugs the iroh transport in behind the same seam.
//! - [`authz`]: the pairing and access-list seam ([`authz::Authorizer`]) with a
//!   deny-by-default implementation and a clearly marked permissive one for
//!   development.
//! - [`receipt`]: an append-only, hash-chained on-robot receipt audit log.
//! - [`verbs`]: the verb framework and the three verbs: `inventory`, `log-tail` and
//!   `restart`.
//! - [`deploy`]: content-addressed deploy-bundle manifest types, digest
//!   verification, and a pure symlink-plan computation. Applying a plan on the robot
//!   is not implemented here.
//! - [`lease`]: a pure control-lease, deadman and e-stop state machine.
//! - [`server`] and [`client`]: the glue that drives a connection through handshake,
//!   authorization, verb dispatch and receipt.
//!
//! Nothing here talks to iceoryx2 or zenoh; the crate is transport-agnostic and its
//! tests run in parallel. Design notes for contributors live in
//! `docs/internals/remote-access.md` in the repository.

// Principle #12 (logging): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

pub mod authz;
pub mod client;
pub mod constants;
pub mod deploy;
pub mod error;
pub mod hash;
pub mod lease;
pub mod protocol;
pub mod receipt;
pub mod server;
pub mod transport;
pub mod verbs;

pub use error::{CerudError, CerudResult};
