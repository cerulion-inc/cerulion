// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion-vizd` — the long-lived, schema-generic dynamic viz DAEMON.
//!
//! ONE process, ONE iceoryx2 node (Principle #8). It owns a set of listener-less
//! runtime taps ([`cerulion_viz::tap_manager::TapManager`]) it adds/removes on
//! demand, drains them on a poll thread into the never-block
//! [`VizLogWorker`](cerulion_viz::worker::VizLogWorker) (which owns the Rerun
//! `RecordingStream` off the poll thread), and is driven by MULTIPLE concurrent
//! controllers — Cerulion Studio + the Cerulion agent + the `cerulion viz` verb —
//! over a Unix-domain-socket NDJSON control protocol.
//!
//! The recomposition insight (see the dynamic-sink design record
//! in the design record): the generic
//! decode→archetype→log core and the runtime-tap transport primitive already
//! ship in `cerulion_viz` + `cerulion_core`; this daemon is their standalone
//! host + a control surface, not a new invention.
//!
//! # Layers
//!
//! - [`protocol`] — THE sacred NDJSON contract (serde request/response types +
//!   the one-object-per-line codec). Rerun-free; a future client could lift it.
//! - [`host`] — the Rerun-endpoint resolution: HOST a gRPC
//!   message proxy by default (advertised in the banner every viewer connects
//!   to), or connect as a CLIENT to `$CERULION_RERUN_URL`.
//! - [`net`] — the desk-default zenoh [`NetworkConfig`](cerulion_core::transport::network::NetworkConfig)
//!   the daemon initializes its transport with (remote arm): scouting ON,
//!   lazy session, env-threaded `--connect`/`--listen` locators, and the
//!   `CERULION_VIZD_NETWORK=off` local-only kill switch.
//! - [`monitors`] — MONITORS v1: the daemon-side plane around
//!   [`cerulion_viz::monitor`]'s pure engine (one monotonic origin, the bounded
//!   alert ring, the per-`(row, condition)` flood latches). The engine decides;
//!   this module remembers.
//! - [`hygiene`] — socket-path resolution, the PID lock, stale-socket recovery,
//!   the live-daemon refusal, and cleanup-on-shutdown.
//! - [`daemon`] — the runtime: the tap poll thread, per-topic Hz/schema stats,
//!   the UDS control server, and request dispatch. Dependency-injected (manager +
//!   worker + walker) so it drives in-process over an isolated per-test SHM root.
//!
//! `main.rs` is the thin production entry: it `get_or_init`s the singleton
//! transport, builds the worker over the real gRPC `RecordingStream`, and runs
//! the daemon until SIGINT/SIGTERM.

// Principle #12 (logging): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

pub mod daemon;
pub mod events;
pub mod host;
pub mod hygiene;
pub mod monitors;
pub mod net;
pub mod protocol;
pub mod runs;

pub use daemon::{
    start_on_socket, start_with_demand_plane, start_with_planes, ControlSocket, DemandPlane,
    NetdDemandPlane, RunningDaemon, DEFAULT_POLL_INTERVAL, MAX_REQUEST_LINE_BYTES,
    PROVENANCE_CACHE_TTL,
};
pub use events::{
    CatalogEventPoll, CatalogEventSource, CatalogEventStream, CatalogSnapshot, EventHub,
    NetdCatalogEventSource, NoCatalogEventSource, SubscribeFailure, SubscribeFailureCounts,
};
pub use host::{resolve_stream, StreamResolution};
pub use hygiene::{default_socket_path, SOCKET_ENV};
pub use protocol::{
    CatalogChangedEvent, DiscoveryLabel, Hello, Request, Response, RetryHint,
    SubscribeEventsResponse, CATALOG_CHANGED_EVENT, PROTOCOL_VERSION,
};
