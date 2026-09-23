// SPDX-License-Identifier: AGPL-3.0-only
//! # cerulion_remoted: the robot-side remote-access daemon
//!
//! `cerulion_remoted` is the single robot-side process that owns the robot's ONE
//! iroh endpoint and serves both remote protocols on it. It is not published to
//! crates.io: it is a daemon a robot runs, not a library. The facts that shape it:
//!
//! - **One key, one endpoint, one process.** A device's 32-byte ed25519 secret is
//!   BOTH its iroh [`EndpointId`](cerulion_link::EndpointId) AND its
//!   `cerulion_pairing` `DeviceIdentity` seed, and iroh multiplexes both protocols
//!   on ONE endpoint. So exactly one process owns that endpoint, dispatching
//!   `cerulion/wire/1` and `cerulion/ops/1` at accept.
//! - **Always on from boot.** Operations and pairing must be reachable when NO graph
//!   is running (deploy, restart, inventory and pair at rest), and topic data exists
//!   only while a graph runs, so the endpoint host is a robot-level service, NOT the
//!   per-run gateway (which dies with the graph).
//! - **Deny by default.** Every accept is gated through a [`PairingAuthorizer`] over
//!   the offline [`cerulion_pairing`] trust store; an unpaired peer reaches only the
//!   pairing and claim bootstrap surface, and an unclaimed robot admits only
//!   `claim`.
//!
//! ## The daemon and its gate
//!
//! - [`daemon`]: load the device key, the trust store and the side map, build the
//!   ONE endpoint via [`cerulion_link`], and run the `accept_one` loop dispatching on
//!   the negotiated protocol. Each accepted connection is classified by the
//!   authorizer and handed to the wire plane or the ops plane; a connection that is
//!   refused, or whose plane is not configured, gets the decision reported over a
//!   small control frame instead.
//! - [`authorizer`]: the pure [`PairingAuthorizer`] (an impl of
//!   `cerud::authz::Authorizer`). It maps the TLS-authenticated `remote_id` (device
//!   key) to an account via the [`device_index`] side map, then applies the per-verb
//!   capability table, the e-stop floor and the bootstrap exemption.
//! - [`device_index`]: the persisted map from device key to account. The access row
//!   stores the account, not the device key, so `remoted` owns this index and writes
//!   it at pairing time.
//! - [`config`]: the state-root path conventions, the relay seam, and the
//!   `--network off` / `CERULION_NETWORK=off` kill switch (the same switch the
//!   gateway honors).
//! - [`beacon_facts`]: publishes the PUBLIC endpoint facts (`eid`, `iroh_port`,
//!   `claimable`) the gateway reads to enrich its mDNS TXT record; written at startup
//!   and refreshed after a claim.
//! - [`verbs`]: the verb-name constants the authorizer classifies.
//!
//! ## The wire plane
//!
//! - [`wire`]: the `cerulion/wire/1` control protocol ([`WireRequest`] and
//!   [`WireResponse`]), the connection handler, and the [`WirePlane`] context (robot
//!   identity, `remoted`'s own `network:None` `TransportManager`, and catalog and
//!   schema assembly), reusing the `cerulion_q` vocabulary.
//! - [`tap`]: the demand-driven [`TapManager`]: one listener-less
//!   `DataOnlySubscriber` and one robot-to-desk data stream per demanded topic.
//! - [`forward`]: the bounded, drop-to-live per-topic forward queue and its
//!   once-per-regime flood latch.
//!
//! ## The ops plane
//!
//! - [`trust`]: the LIVE, shared [`SharedTrust`] the accept gate reads and the
//!   pairing bootstrap verbs mutate (a claim or pair takes effect with no restart),
//!   with fail-closed, lock-free-I/O, store-before-index persistence.
//! - [`ops`]: the `cerulion/ops/1` handler. [`OpsServing`] bridges the async QUIC
//!   stream to `cerud::OpsServer::serve_connection` via a `QuicOpsStream` adapter.
//! - [`pairing_verbs`]: the self-gating bootstrap and safety verbs (`claim`, `pair`,
//!   `code-pair-start`, `code-pair-finish`, `engage-estop`).
//! - [`flashback`]: publishes a capture request when the ops-plane e-stop is engaged.
//! - [`clock`]: the robot's TRUSTED clock for pairing verification (never a
//!   client-supplied time).
//!
//! Linking `cerulion_core` (for the SHM tap, the wire format and the `cerulion_q`
//! vocabulary) makes `remoted` AGPL; it is robot-side, so that is correct. Design
//! notes for contributors live in `docs/internals/remote-access.md` in the
//! repository.

// Principle #12 (logging): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

pub mod authorizer;
pub mod beacon_facts;
pub mod clock;
pub mod config;
pub mod daemon;
pub mod device_index;
pub mod error;
pub mod flashback;
pub mod forward;
mod local_schema;
pub mod ops;
pub mod pairing_verbs;
pub mod provision;
pub mod tap;
pub mod trust;
pub mod verbs;
pub mod wire;

pub use authorizer::{AcceptDecision, PairingAuthorizer};
pub use beacon_facts::BeaconFacts;
pub use clock::RemotedClock;
pub use config::{
    classify_network, network_disabled, resolve_network_disabled, resolve_network_disabled_loud,
    NetworkSetting, RemotedConfig,
};
pub use daemon::{
    handle_accepted, handle_accepted_with_wire, run, serve, serve_endpoint, serve_with_wire,
};
pub use device_index::DeviceAccountIndex;
pub use error::RemotedError;
pub use flashback::{EstopCaptureAsk, TransportAsk};
pub use ops::OpsServing;
pub use tap::TapManager;
pub use trust::{KeyAccess, SharedTrust, TrustError};
pub use wire::{
    rebuild_frame, serve_wire_connection, StatusReply, StreamPreamble, TopicStatus, WireError,
    WirePlane, WireRequest, WireResponse,
};
