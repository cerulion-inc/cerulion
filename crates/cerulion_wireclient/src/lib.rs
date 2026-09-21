// SPDX-License-Identifier: AGPL-3.0-only
//! # cerulion_wireclient
//!
//! Desk-side client pieces for the `cerulion/wire/1` protocol: what a computer needs
//! to read a remote robot's topics over an iroh connection. It is the desk
//! counterpart of the robot-side server in `cerulion_remoted`.
//!
//! # Who uses it
//!
//! Two consumers share this one implementation: the `cerulion connect` worker
//! (`cerulion_connectd`) and the internet plane of the `cerulion-netd` gateway
//! daemon. It is an internal building block, published because `cerulion_netd`
//! depends on it. Keeping it in its own crate is also what keeps the package graph
//! acyclic: netd depends on this crate, never on `cerulion_connectd`.
//!
//! ## Modules
//!
//! - [`config`]: the resolved dial parameters ([`ConnectConfig`]) and the pure input
//!   parsers (endpoint id, addresses, desk key, account), including the resolver
//!   from a device certificate to its account ([`resolve_desk_account`]) that the
//!   netd dial config uses to present an account-bound identity.
//! - [`protocol`]: the desk-side mirror of the robot's `cerulion/wire/1` control
//!   protocol ([`WireRequest`], [`WireResponse`], [`StreamPreamble`]). A parity test
//!   in `cerulion_connectd` pins it against the types the robot serves.
//! - [`reader`]: [`run_topic_reader`], the validated per-topic re-inject loop. One
//!   dedicated task owns a one-way stream and re-injects each frame into local
//!   shared memory. The session driver (dial, demand, catalog) stays in
//!   `cerulion_connectd`.
//! - [`epoch`]: the shared revocation-epoch push logic (what to send, what the
//!   robot's answer meant), so netd's internet plane and the `cerulion connect` verb
//!   carry the epoch through one implementation.
//! - [`error`]: the unified [`ConnectError`] enum.
//!
//! Design notes for contributors live in `docs/internals/remote-access.md` in the
//! repository.

// P12 (Logging, see AGENTS.md): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

pub mod config;
pub mod epoch;
pub mod error;
pub mod protocol;
pub mod reader;

pub use epoch::{
    classify_epoch_reply, note_transport_failure, prepare_epoch_push, EpochPushOutcome,
    EpochPushPlan,
};

pub use config::{
    decode_epoch_cache, encode_epoch_cache, epoch_cache_file_name, parse_account, parse_addrs,
    parse_desk_subject_cert, parse_device_cert, parse_eid, resolve_desk_account, resolve_desk_seed,
    resolve_epoch_sync, resolve_or_create_desk_seed, resolve_owner_grant, ConnectConfig, DemandSet,
    OwnerGrantBundle, DEFAULT_ROBOT_TIMEOUT,
};
pub use error::{ConnectError, ConnectResult};
pub use protocol::{
    decode_first_reply, AcceptDecision, StatusReply, StreamPreamble, TopicStatus, WireRequest,
    WireResponse,
};
pub use reader::{run_topic_reader, TopicReinjectCounters, TopicReinjectStats};
