// SPDX-License-Identifier: AGPL-3.0-only
//! # cerulion_connectd: the desk-side re-inject client
//!
//! `cerulion_connectd` is the process `cerulion connect <robot>` spawns to make a
//! REMOTE robot's topics appear as LOCAL desk SHM topics ("remote = local"). It
//! owns the desk's iroh half:
//!
//! - **Dial** the robot's `cerulion/wire/1` ALPN by [`EndpointId`](cerulion_link::EndpointId)
//!   (+ optional direct LAN socket addresses) with a persisted or ephemeral desk
//!   device key.
//! - **Speak** the shared `cerulion_q` catalog / demand / schema / status
//!   vocabulary over the bidi control stream ([`protocol`]).
//! - **Read** one one-way (robot to desk) data stream per demanded topic and
//!   **re-inject** each raw wire frame into desk-local iceoryx2 SHM via the
//!   zenoh-free ingress seam ([`worker`]), after which `topic echo`, `viz` or a
//!   subscriber tap the now-local topic with ZERO changes.
//! - **Materialize** each demanded topic's `.msg`/YAML closure into the desk
//!   schema store ([`schema`]) so a schema-less desk decodes a never-seen type.
//!
//! The AGPL/closed boundary is a PROCESS boundary: the closed Studio spawns this
//! AGPL binary exactly as it spawns the gateway / `vizd`.
//!
//! ## Modules
//!
//! - [`config`]: the resolved dial parameters ([`ConnectConfig`]) + the pure
//!   input parsers (eid / addrs / desk key / account).
//! - [`protocol`]: the desk-side MIRROR of the robot's `cerulion/wire/1` control
//!   protocol (a dev-dep parity test pins it against the robot's serve types).
//! - [`schema`]: materialize a robot-served schema closure into the desk store.
//! - [`worker`]: [`run_connect`], the dial, demand and per-topic re-inject engine.
//! - [`pair`]: [`run_pair`], the desk-side CPace pairing ceremony over the
//!   robot's `cerulion/ops/1` plane (the `cerulion pair` verb's engine).
//! - [`error`]: the unified [`ConnectError`] enum.

// Principle #12 (logging): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

// `config`, `error`, and `protocol` were EXTRACTED into the neutral
// `cerulion_wireclient` crate (shared with `cerulion_netd`'s WAN plane so there is
// no `netd → connectd` cyclic package edge). They are re-exported here VERBATIM so
// `cerulion_connectd::{config, error, protocol}` — and every dependent (main.rs,
// the parity/reinject tests) — keeps resolving with no source change.
pub use cerulion_wireclient::{config, error, protocol};

pub mod pair;
pub mod schema;
pub mod worker;

pub use cerulion_wireclient::config::{
    parse_account, parse_addrs, parse_eid, resolve_desk_seed, resolve_or_create_desk_seed,
    ConnectConfig, DemandSet,
};
// The SHARED epoch-cache directory resolution (`cerulion-netd`'s WAN plane
// calls the same function) — one env name, one fallback, one answer for both desk
// paths.
pub use cerulion_wireclient::epoch::{resolve_epoch_dir, resolve_epoch_dir_from_env};
// The session's revocation-epoch push outcome + its operator-loudness class,
// surfaced on `ConnectSummary` (Principle #3) — the SHARED types netd's WAN plane
// records + branches on too.
pub use cerulion_wireclient::epoch::{EpochPushOutcome, EpochPushSeverity};
// The ONE sanitizer for peer-chosen text, re-exported so the BINARY
// (which prints the robot's catalog straight to an interactive STDOUT) renders the
// robot's self-reported name and its topic names through the same policy + bound the
// session's log lines use. No second convention that could forget the escape class.
pub use cerulion_wireclient::epoch::sanitize_peer_text;
pub use cerulion_wireclient::error::{ConnectError, ConnectResult};
pub use pair::{run_pair, PairConfig, PairError, PairOutcome};
pub use worker::{run_connect, ConnectSummary, TopicReinjectStats};
