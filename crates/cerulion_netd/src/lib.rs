// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion-netd`: the per-computer network gateway daemon.
//!
//! One `cerulion-netd` runs per computer. It owns the machine's single zenoh
//! session and serves reference-counted topic demands to every local tool: many
//! consumers (`cerulion topic echo`, `cerulion viz`, Studio, the recorder) ask for
//! the same remote `(robot, topic)`, the frames cross the network once, netd
//! re-injects them into local shared memory once, and every reader subscribes to
//! that one mirror. When the last demand for a topic is released its mirror is torn
//! down, and a daemon nobody is using exits on its own.
//!
//! # Who uses it
//!
//! Users install it with `cargo install --locked cerulion_cli cerulion_netd` and
//! normally never start it by hand: the first command that needs a remote topic
//! spawns it detached through [`NetdClient`]. A robot that should be discoverable
//! runs it as a standing service with `CERULION_NETD_LISTEN` set;
//! `cerulion-netd --help` lists every environment variable. Library consumers (the
//! CLI engine, the visualization daemon, the recorder) use [`NetdClient`] to
//! connect, demand a topic and query the catalog.
//!
//! # Modules
//!
//! - [`client`]: [`NetdClient`], the consumer half. It connects to the daemon,
//!   spawning it if none is running, and drives `demand` and `release`.
//! - [`protocol`]: the control protocol, one JSON object per line over a Unix
//!   socket.
//! - [`daemon`]: the runtime: the accept loop, per-connection dispatch and the idle
//!   watch that ends an unused daemon.
//! - [`registry`]: the pure reference-count and idle-lifecycle state machine. A
//!   closed connection releases everything it held.
//! - [`mirror`]: the [`MirrorPlane`] seam between the registry and the network, with
//!   the production [`GatewayMirrorPlane`].
//! - [`egress`]: the seam for serving this machine's own topics to the network.
//! - [`query`]: catalog and schema queries over the one zenoh session.
//! - [`catalog_events`]: pushes catalog changes to the consumers that subscribed.
//! - [`convergence`]: the client-side wait policy for first contact, while
//!   discovery has not settled yet.
//! - [`discovery_fold`]: folds the verified peer cache into the session at boot.
//! - [`health`]: the watch over each mirror's stream.
//! - [`beacon`]: the `_cerulion._tcp` mDNS advertisement.
//! - [`net`]: the zenoh configuration, read from the environment
//!   (`CERULION_NETD_NETWORK=off` keeps the daemon local-only).
//! - [`hygiene`]: the socket path, the single-daemon lock and stale-socket recovery,
//!   shared through `cerulion_hygiene`.
//! - `iroh_plane` and `wan` (feature `wan`, on by default): the iroh plane for
//!   robots reached across the internet. `--no-default-features` builds the LAN-only
//!   daemon.
//!
//! The daemon's network planes are injected (a [`MirrorPlane`]), so the end-to-end
//! tests drive it in-process over a temporary socket without zenoh. `main.rs` is the
//! thin production entry: it builds the real planes and runs the daemon until it
//! goes idle or receives SIGINT or SIGTERM.
//!
//! Design notes for contributors live in `docs/internals/network-daemons.md` in the
//! repository.

// P12 (the repo's logging policy): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

/// The `_cerulion._tcp` mDNS beacon netd raises when it becomes this
/// machine's serving gateway, the advertisement that was lost with
/// the `run-gateway` child.
pub mod beacon;
pub mod catalog_events;
pub mod client;
// The PURE first-contact convergence-wait policy — the client-side decision
// that turns netd's "discovery has not converged" marker into a reason to keep
// asking rather than a verdict to render.
pub mod convergence;
pub mod daemon;
// The boot-time fold of the verified `~/.cerulion/peers.json` cache into
// this daemon's connect endpoints, so first contact works with no
// `CERULION_NETD_CONNECT`.
pub mod discovery_fold;
pub mod egress;
pub mod health;
pub mod hygiene;
pub mod mirror;
pub mod net;
pub mod protocol;
pub mod query;
pub mod registry;

// The iroh WAN plane, gated behind the `wan` feature, now DEFAULT-ON (the
// decision ships the WAN plane by default; `--no-default-features` is the lean
// zenoh-only opt-out). A plain `cargo build` stays iroh-free NOT because `wan` is off
// but because netd is excluded from `default-members` (see `Cargo.toml`). `wan` holds
// the WAN robot registry + the dual-plane picker; `iroh_plane` the folded re-inject
// engine.
#[cfg(feature = "wan")]
pub mod iroh_plane;
#[cfg(feature = "wan")]
pub mod wan;

pub use catalog_events::{
    AnnounceStream, AnnounceView, AnnounceWatchPlane, CatalogDelta, CatalogEventHub,
    ChangeCoalescer, GatewayAnnounceWatchPlane, NoopAnnounceWatchPlane, WatchError,
    CHANGE_COALESCE_WINDOW,
};
pub use client::{
    ClientError, ConvergenceAbort, FirstContactWait, NetdClient, NetdEventClient, NextEvent,
    NETD_BIN_ENV,
};
pub use convergence::{
    worst_case_wall, Converged, ConvergenceWait, WaitDecision, WaitOutcome,
    CONVERGENCE_POLL_INTERVAL, FIRST_CONTACT_CONVERGENCE_CEILING,
};
pub use daemon::{
    start, start_with_egress, start_with_planes, start_with_planes_and_events, NetdConfig,
    RunningNetd, DEFAULT_IDLE_GRACE,
};
pub use discovery_fold::{
    decide_fold, fold_cached_peers, fold_cached_peers_at, trust_explicit_by_fiat, PeerFold,
};
pub use egress::{EgressError, EgressId, EgressPlane, GatewayEgressPlane, NoopEgressPlane};
pub use health::{
    BeltConfig, BeltSchedule, HealthConfig, HealthState, HealthTransition, MirrorHealth,
};
pub use hygiene::{acquire_socket, default_socket_path, SocketGuard, SOCKET_ENV};
pub use mirror::{GatewayMirrorPlane, MirrorError, MirrorPlane, MirrorRelease};
pub use net::network_config_from_env;
pub use protocol::{
    classify_control_line, CatalogChanged, CatalogQueryResponse, ControlLine, DiscoveryState,
    Hello, Request, Response, SchemaQueryResponse, SubscribeCatalogResponse, CATALOG_CHANGED_EVENT,
    PROTOCOL_VERSION,
};
pub use query::{
    CatalogGather, GatewayQueryPlane, NoopQueryPlane, QueryError, QueryPlane, RunsAnswer,
    RunsGather, SchemaGather, COLD_START_DISCOVERY_BUDGET,
};
pub use registry::{
    DemandOutcome, DemandRegistry, DisconnectOutcome, RegisterEgressOutcome, ReleaseOutcome,
    TopicKey,
};

#[cfg(feature = "wan")]
pub use iroh_plane::{EpochPushOutcome, EpochPushSeverity, IrohMirrorPlane};
#[cfg(feature = "wan")]
pub use wan::{
    parse_robots_spec, pick_plane, DeskPathInputs, DualMirrorPlane, Plane, WanRegistry, WanRobot,
};

/// The ONE crate-wide lock serializing every env-mutating
/// **lib** test in `cerulion_netd`.
///
/// libtest runs all `#[cfg(test)] mod tests` of a library in a SINGLE test binary, so
/// the process environment is shared across every module. Before this module, `wan.rs`
/// kept a FILE-LOCAL `env_lock()` mutex covering its 30 env writes — which serialized
/// `wan.rs` against ITSELF and against nothing else — while `hygiene.rs`
/// (`socket_env_override_wins` sets `CERULION_NETD_SOCKET`, and `default_socket_path`
/// also READS `XDG_RUNTIME_DIR`/`HOME`) and `net.rs`
/// (`network_config_from_env_reads_the_kill_switch_and_locators` sets
/// `CERULION_NETD_NETWORK`/`_CONNECT`/`_LISTEN` and `CERULION_ROBOT_IDENTITY`) mutated
/// the same process environment behind RAII restore guards and NO lock at all. That is
/// the exact hole closed earlier in `cerulion_cli_engine`, reproduced verbatim in this
/// crate.
///
/// `net.rs`'s in-file note ("no sibling test reads `CERULION_NETD_*` env, so a single
/// mutate+restore body is race-free even under parallel libtest") reasoned about VAR
/// NAMES. That is not the hazard: concurrent `setenv`/`getenv` from different threads
/// is a data race on the environ block itself regardless of which keys are touched, and
/// `wan.rs` is mutating 30 vars in the same binary. Var-name disjointness also cannot be
/// maintained by review — `hygiene.rs`'s `default_socket_path` already falls back to
/// `HOME`, which the sibling desk-path tests set.
///
/// Every env-mutating (or env-READING) lib test in this crate must take THIS guard
/// (`wan.rs`'s module-level `env_lock()` is a thin delegate), so adding a new one in any
/// module is automatically serialized against all the others.
///
/// **`#[serial_test::serial]` is NOT a substitute** — it takes serial_test's OWN mutex,
/// so a `#[serial]` env test and a `test_env` env test run concurrently in two
/// independent domains. Where both are needed, lock order is always `#[serial]` first
/// (the attribute) then this guard (the body), so the two can never deadlock.
///
/// Scope note: integration tests under `tests/` are separate binaries with their own
/// processes, so they keep their own file-local locks — a crate-wide static could not
/// reach across process boundaries anyway.
///
/// Poison is deliberately absorbed (`into_inner`): a panicking test leaves the env
/// restored by its own RAII guard, and poisoning the shared lock would cascade one
/// failure into every other env test in the crate.
#[cfg(test)]
pub(crate) mod test_env {
    /// Acquires the crate-wide env lock. Hold the returned guard for the whole body of
    /// any test that reads or writes process-global environment state.
    pub(crate) fn env_lock() -> std::sync::MutexGuard<'static, ()> {
        static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
        LOCK.lock().unwrap_or_else(|p| p.into_inner())
    }
}
