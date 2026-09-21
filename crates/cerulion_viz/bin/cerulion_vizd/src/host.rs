// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion-vizd` Rerun-endpoint resolution (the
//! checkbox→scene bar).
//!
//! `cerulion_viz::stream` is a pure gRPC CLIENT: it `connect_grpc`s to
//! `$CERULION_RERUN_URL` (default `rerun+http://127.0.0.1:9876/proxy`). Studio's
//! embedded WASM viewer (`@rerun-io/web-viewer`) is ALSO a client of that URL.
//! **Nobody hosts the gRPC message-proxy server unless a native `rerun` viewer
//! app happens to be running** — so a bare machine renders NOTHING. The daemon
//! must be the host.
//!
//! # Two modes
//!
//! - **HOST (default, automagic):** `$CERULION_RERUN_URL` unset ⇒ the daemon
//!   HOSTS a gRPC message proxy on loopback (`RecordingStreamBuilder::serve_grpc_opts`,
//!   the rerun `server` feature), binds a free port (the well-known
//!   [`rerun::DEFAULT_SERVER_PORT`] if free, else an ephemeral one), logs its viz
//!   INTO that hosted server, and advertises `rerun+http://127.0.0.1:{port}/proxy`
//!   in the [`Hello`](crate::protocol::Hello) banner. Any viewer — a native
//!   `rerun` app OR Studio's WASM viewer — then connects to that ONE endpoint with
//!   ZERO extra config.
//! - **CLIENT (power-user override):** `$CERULION_RERUN_URL` set ⇒ the daemon
//!   connects to THAT external endpoint as a client (the existing
//!   [`cerulion_viz::stream::connect_with_config`] path) and advertises the same
//!   URL. The caller is responsible for hosting the endpoint (a running native
//!   viewer / a separately-started proxy).
//!
//! # Why a pre-probe (not `port = 0`)
//!
//! `serve_grpc_opts` binds `{ip}:{port}` ASYNCHRONOUSLY on its own server thread
//! (`re_grpc_server::serve_from_channel`), so a taken port is a SILENT failure —
//! the sink is constructed Ok but the server thread logs an error and dies. And
//! passing `port = 0` reports back port `0` in the sink's URI, not the real
//! ephemeral port. So we PRE-PROBE a concretely-free port with a `std` bind
//! (drop it, then hand the number to `serve_grpc_opts`). There is an inherent
//! probe→serve TOCTOU (the standard bind-ladder race, same as the network
//! gateway), so we additionally VERIFY the proxy came up (a bounded TCP connect)
//! and log LOUDLY if it did not — never a silently-dead endpoint.

use std::net::{Ipv4Addr, SocketAddr, TcpListener, TcpStream};
use std::time::{Duration, Instant};

use rerun::{RecordingStream, RecordingStreamBuilder};

use cerulion_viz::stream::{build_recording_builder, endpoint_override, StreamConfig};

/// The resolved viz stream + the endpoint every viewer connects to.
pub struct StreamResolution {
    /// The `RecordingStream` the never-block worker logs into (a hosted
    /// `GrpcServerSink`, a client gRPC sink, or a `disabled()` no-op).
    pub rec: RecordingStream,
    /// The endpoint advertised in the [`Hello`](crate::protocol::Hello) banner
    /// (`rerun+http://127.0.0.1:{port}/proxy` when hosting, the override URL when
    /// a client, `None` when viz is disabled).
    pub rerun_url: Option<String>,
}

/// Resolve the daemon's viz stream: HOST a gRPC proxy by default, or connect as
/// a CLIENT to `$CERULION_RERUN_URL` when set. See the [module docs](self).
pub fn resolve_stream() -> StreamResolution {
    match endpoint_override() {
        Some(url) => connect_client(url),
        None => host_endpoint(),
    }
}

/// CLIENT mode: `$CERULION_RERUN_URL` is set → connect to that external endpoint
/// (the existing [`cerulion_viz::stream`] path reads the SAME env internally) and
/// advertise it. A connect misconfiguration disables viz but STILL advertises the
/// URL (that is where the operator pointed a viewer; a viewer started there later
/// works).
fn connect_client(url: String) -> StreamResolution {
    match cerulion_viz::stream::connect_with_config(&StreamConfig::default()) {
        Some(rec) => {
            tracing::info!(rerun_url = %url, "cerulion-vizd: viz stream = CLIENT (connecting to $CERULION_RERUN_URL)");
            StreamResolution {
                rec,
                rerun_url: Some(url),
            }
        }
        None => {
            // Misconfigured client endpoint — viz is a no-op, but advertise the
            // intended URL so a viewer started there later still finds the scene.
            StreamResolution {
                rec: RecordingStream::disabled(),
                rerun_url: Some(url),
            }
        }
    }
}

/// The byte budget for TEMPORAL data sitting in the hosted proxy's LIVE
/// broadcast queue. Past this, a further temporal frame is DROPPED rather than
/// queued behind a viewer that is not keeping up — the decision that
/// there is "no buffer for clouds or images" applied to the live path.
///
/// **8 MiB — sized against the HIGH-RATE stream, not picked.** The budget is
/// compared against the queue's CURRENT occupancy, so ONE message always crosses
/// however large it is; what the number buys is how much MORE may sit behind it.
///
/// The stream that matters is the camera: a decoded picture logged as a raw RGB8
/// [`rerun::Image`] is 2.6 MiB for the 1280x720 rendition the Go2 front camera
/// publishes (`examples/go2/nodes/camera_jpeg/src/h264.rs` `KNOWN_RENDITION_HEIGHTS`).
/// 8 MiB admits `floor(8 MiB / 2.6 MiB) + 1 = 4` of them — **at most ~133 ms of
/// video** in flight at 30 Hz (MEASURED: `tests/live_backlog_test.rs`
/// reports a 4.0-frame peak).
///
/// **That steady state is OVER budget, and that is why the fork carries a
/// small-message floor.** Four resident frames is 10.4 MiB against an 8 MiB
/// budget, and the gate compares the budget against the queue's OCCUPANCY, not
/// against the arriving message — so on the byte axis alone, every temporal
/// message sharing this daemon's ONE `RecordingStream` would be dropped for as
/// long as the camera holds the queue there, at any size: plot samples, dynamic
/// `/tf`, `TextLog` lines, marker `Clear`s. The fork exempts messages under
/// `re_grpc_server::LIVE_SMALL_MESSAGE_FLOOR_BYTES` (8 KiB) from THAT axis, so
/// the claim this daemon can make is precise: a message under 8 KiB is never
/// dropped for the queue being over budget — **by construction, not by
/// probability** — and vizd's control-class traffic MEASURES 1.2-1.6 KB, so it
/// sits well inside that exemption. (What
/// `the_small_message_floor_sits_between_control_and_image_traffic` pins is the
/// SEPARATION, not the range: every control-class shape strictly under the floor
/// and every image-class one at or above it. The 1.2-1.6 KB figures are the
/// readings behind that arm, not an interval it asserts.)
/// Two things it does NOT claim: such a message is still eligible on the MESSAGE
/// axis if ~960 messages are already in flight (which is what keeps the proxy's
/// event loop from wedging), and a point cloud or image of any consequence is
/// above the floor and is dropped exactly as intended.
///
/// Deliberately NOT tighter: at one frame the queue would drop on ordinary
/// scheduling jitter, costing frames a viewer that IS keeping up could have
/// displayed. Deliberately NOT looser: the whole defect is that 128 MiB let
/// ~1.6 s accumulate.
///
/// **It is NOT the largest message this daemon can emit, and that is a deliberate
/// limitation.** An occupancy grid can reach `MAX_OCCUPANCY_PIXELS` (32 Mi cells
/// ⇒ a ~32 MB `Image`) and a point cloud's `Points3D` positions are uncapped
/// (`cerulion_viz::archetype`), so a single such message can exceed the budget on
/// its own. It still crosses — the gate admits any message into a within-budget
/// queue — but while it is resident the budget is exceeded, so OTHER temporal
/// messages are dropped until it drains. Sizing the budget for the 32 MB worst
/// case would re-admit ~12 camera frames of backlog, which is the defect. A
/// per-stream budget is the real answer and is not what this ships.
pub const LIVE_TEMPORAL_BUDGET_BYTES: u64 = 8 * 1024 * 1024;

/// The [`rerun::ServerOptions`] for the hosted proxy — **live-only by default
/// (by design).**
///
/// The proxy's message buffer is a PER-CLIENT REPLAY buffer: on every new
/// connection re_grpc_server streams the retained history to THAT client (then
/// subscribes it to the live broadcast). Stock, it replays the whole buffered
/// backlog before live data — a viewer attaching mid-run sees a fast-forward
/// burst through minutes of accumulated sensor frames. Studio uses rerun as PURE
/// LIVE viz — a plot starts getting data when its checkbox is checked, no
/// backlog, no catch-up.
///
/// We use the Cerulion `re_grpc_server` fork's `drop_temporal_history` mode (the
/// `cerulion-inc/re_grpc_server` sparse fork pinned in the root `Cargo.toml`
/// `[patch.crates-io]`; see that repo's `CERULION-PATCH.md`):
///
/// - `drop_temporal_history = true` ⇒ the history buffer NEVER buffers
///   disposable/temporal frames — only `persistent` (SetStoreInfo + blueprint +
///   BlueprintActivationCommand) and `static_` (`is_static` chunks). So a fresh
///   client's PER-CLIENT connect-history carries the scene skeleton (blueprint,
///   robot model, TF tree, camera pinhole) with **zero temporal replay at any
///   producer bandwidth**, and live temporal data flows from connect-time via
///   the broadcast. No periodic re-send ⇒ no blueprint clobber of existing
///   viewers, no chunk-store accumulation.
/// - `memory_limit = ZERO`: moot under the flag (the fork keeps statics
///   regardless + never GC-evicts them). Set to ZERO to make "no temporal byte
///   buffer" explicit.
/// - `playback_behavior = OldestFirst`: with zero temporal backlog there is
///   nothing to scrub, so the direction is otherwise vestigial — but OldestFirst
///   preserves the persistent queue's log order (`SetStoreInfo` →
///   `BlueprintActivationCommand`) that a late joiner's skeleton delivery depends
///   on. Upstream `re_grpc_server` bug rerun#12721 shows `NewestFirst` REVERSES
///   the persistent queue, which would deliver the activation before its store
///   info — the exact late-joiner path this daemon exists for.
///
/// vizd re-logs the scene skeleton (statics + blueprint) on demand — including a
/// runtime `set_blueprint` when a Studio layout changes. The fork retains the
/// LATEST of each: F1 evicts superseded blueprint STORES so `persistent`
/// does not grow across layout changes, and F2 dedups re-logged statics
/// entity-level — so `persistent`/`static_` stay bounded and each new viewer gets
/// exactly the current skeleton on connect. No heartbeat, no accumulation.
///
/// # The LIVE queue
///
/// `drop_temporal_history` above governs the per-client REPLAY buffer — what a
/// LATE JOINER is sent on connect. There is a SECOND, independent buffer on the
/// same path: the LIVE broadcast queue every ALREADY-CONNECTED viewer reads from.
/// `re_grpc_server` byte-quotas it at 128 MiB and AWAITS space when it is full,
/// so a viewer that renders slower than the robot publishes accumulates frames
/// there and plays through a backlog before it shows the present.
///
/// MEASURED (`tests/live_backlog_test.rs`, a 1280x720 RGB8 frame at 30 Hz
/// — what this daemon logs per decoded H.264 picture now that decoding runs on the
/// desk): 0 bytes against a receiver that keeps up; **27.8 MB — 10.1 frames — in
/// 2.4 s** against one that does not, still climbing, headed for ~47 frames ≈
/// **1.6 s of stale video**.
///
/// The 27.8 MB half of that is what a run WITHOUT the budget produces, only
/// reproducible with the budget off — run the cited file's `measure_live_queue_backlog` with
/// `live_temporal_budget_bytes` set back to `None`. On this branch the same
/// stimulus caps at 11.1 MB, which is what its assertions pin; a reader who
/// follows the citation without that caveat gets a different number and no
/// explanation.
///
/// So [`LIVE_TEMPORAL_BUDGET_BYTES`] bounds it: a temporal frame arriving at an
/// over-budget queue is DROPPED, not queued. See that constant for the sizing
/// argument.
///
/// **A THIRD buffer sits on the same path and this does not bound it.** `re_sdk`'s
/// `GrpcServerSink` builds the SDK→proxy channel with `re_log_channel::log_channel`,
/// whose `max_bytes_on_wire` is a hardcoded 128 MiB. It stays shallow while the
/// proxy's event loop drains promptly — which is exactly what dropping (rather
/// than awaiting) keeps true for temporal traffic — but it is where frames would
/// pool if that loop ever blocked, so the true worst case on the image path is
/// this budget PLUS that channel, not this budget alone. It is not reachable from
/// `ServerOptions` and is not touched here.
///
/// `pub` so the hard-gate test drives a real hosted proxy through the
/// EXACT production options (never a test-local reconstruction).
pub fn server_options() -> rerun::ServerOptions {
    let opts = rerun::ServerOptions {
        // Bound the LIVE queue (see the fn docs). Statics + blueprint
        // are never eligible, so this can never cost a viewer its scene.
        live_temporal_budget_bytes: Some(LIVE_TEMPORAL_BUDGET_BYTES),
        // THE live-only kill: the per-client connect-history carries statics +
        // blueprint only, never temporal (the fork's drop-temporal mode).
        drop_temporal_history: true,
        // Moot under drop-temporal (statics are kept + never evicted); ZERO makes
        // "no temporal byte buffer" explicit.
        memory_limit: rerun::MemoryLimit::ZERO,
        // OldestFirst preserves the persistent queue's log order (SetStoreInfo →
        // BlueprintActivationCommand) for a late joiner's skeleton — NewestFirst
        // reverses it (upstream rerun#12721). See the fn docs.
        playback_behavior: rerun::PlaybackBehavior::OldestFirst,
        ..rerun::ServerOptions::default()
    };
    tracing::info!(
        "cerulion-vizd: hosted gRPC proxy is LIVE-ONLY (drop-temporal history) — a fresh viewer gets the \
         scene skeleton on connect + live data from connect-time, with NO catch-up replay of temporal data"
    );
    opts
}

/// HOST mode (the automagic default): bind a free port and host a gRPC message
/// proxy, logging viz INTO it. Advertise the bound `rerun+http://127.0.0.1:{port}/proxy`.
fn host_endpoint() -> StreamResolution {
    let port = probe_free_port();
    let builder: RecordingStreamBuilder = build_recording_builder(&StreamConfig::default());
    // The hosted proxy is LIVE-ONLY (drop-temporal, see `server_options`): it
    // NEVER buffers temporal history and retains only the scene skeleton
    // (statics + latest blueprint) for late joiners — there is no byte cap, and
    // `memory_limit` is ZERO/moot.
    match builder.serve_grpc_opts("127.0.0.1", port, server_options()) {
        Ok(rec) => {
            let url = format!("rerun+http://127.0.0.1:{port}/proxy");
            // The server binds ASYNC on its own thread; probe that a listener came
            // up (a silent async bind-failure surfaces as a LOUD warn, not a dead
            // endpoint). SCOPE (the probe→serve TOCTOU is inherent): a bare
            // TCP connect confirms only that SOMETHING is listening, NOT that it is
            // THIS proxy — in the narrow race between the free-port probe and the
            // async serve, a foreign process could have taken the port. We keep the
            // window as tight as possible (the probe hands the number straight to
            // serve_grpc_opts) and the log says "a listener responded", not
            // "verified our proxy" — so a stolen-port scene reads accurately.
            if wait_port_listening(port, HOST_VERIFY_BOUND) {
                tracing::info!(
                    rerun_url = %url,
                    "cerulion-vizd: viz stream = HOST (gRPC message proxy) — a listener is accepting on the \
                     bound port; point any viewer here (default). NOTE: a TCP probe confirms a listener, not \
                     that it is THIS proxy (the narrow probe→serve race) — if a connected viewer renders \
                     nothing, restart the daemon. Set CERULION_RERUN_URL to connect to an external endpoint \
                     instead."
                );
            } else {
                tracing::warn!(
                    rerun_url = %url,
                    "cerulion-vizd: hosted gRPC proxy did not come up within the bound (port raced away \
                     after the probe?) — advertising the URL anyway; a viewer connect will retry"
                );
            }
            StreamResolution {
                rec,
                rerun_url: Some(url),
            }
        }
        Err(e) => {
            // A bad bind-IP parse (impossible for the literal 127.0.0.1) — the
            // only synchronous failure `serve_grpc_opts` reports. Fall back to a
            // genuine no-op stream so the CONTROL plane still runs.
            tracing::warn!(error = %e, "cerulion-vizd: could not host the gRPC proxy — running the control plane with viz disabled");
            StreamResolution {
                rec: RecordingStream::disabled(),
                rerun_url: None,
            }
        }
    }
}

/// Bounded verify that a proxy is listening on `127.0.0.1:{port}`.
const HOST_VERIFY_BOUND: Duration = Duration::from_secs(2);

/// Pick a concretely-free loopback port: the well-known
/// [`rerun::DEFAULT_SERVER_PORT`] if it binds (so a bare `rerun` viewer's default
/// connect just works), else an OS-assigned ephemeral one. The probed listener is
/// dropped immediately — the returned port is handed to `serve_grpc_opts` (see the
/// [module docs](self) on the inherent probe→serve TOCTOU + the verify that
/// follows).
fn probe_free_port() -> u16 {
    let loopback = Ipv4Addr::LOCALHOST;
    if let Ok(l) = TcpListener::bind((loopback, rerun::DEFAULT_SERVER_PORT)) {
        // `l` drops at the end of this arm → the port is free for `serve_grpc_opts`.
        drop(l);
        return rerun::DEFAULT_SERVER_PORT;
    }
    match TcpListener::bind((loopback, 0)) {
        Ok(l) => {
            let port = l.local_addr().map(|a| a.port()).unwrap_or(0);
            drop(l);
            port
        }
        // No loopback port at all is a catastrophic host state; fall back to the
        // well-known port and let the async bind / verify surface it.
        Err(_) => rerun::DEFAULT_SERVER_PORT,
    }
}

/// Poll a bounded window for `127.0.0.1:{port}` to accept a TCP connection (the
/// proxy is up). Returns `true` as soon as a connect succeeds. Shared by the
/// production startup verify and the host-mode acceptance test.
pub fn wait_port_listening(port: u16, bound: Duration) -> bool {
    let addr = SocketAddr::from((Ipv4Addr::LOCALHOST, port));
    let deadline = Instant::now() + bound;
    loop {
        if TcpStream::connect_timeout(&addr, Duration::from_millis(100)).is_ok() {
            return true;
        }
        if Instant::now() >= deadline {
            return false;
        }
        std::thread::sleep(Duration::from_millis(25));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn server_options_is_live_only_drop_temporal() {
        // THE kill-shot — the proxy runs the fork's drop-temporal mode, so a
        // fresh viewer's per-client connect-history carries statics + blueprint
        // but ZERO temporal frames. A regression that stops setting the flag (or
        // reverts to a byte-buffered default) fails here. `server_options` reads
        // no environment, so no ambient value can perturb it.
        let opts = server_options();
        assert!(
            opts.drop_temporal_history,
            "the live-only proxy MUST drop temporal history (statics-only connect-history)"
        );
        // OldestFirst (B6): preserves the persistent queue's SetStoreInfo ->
        // BlueprintActivationCommand order for a late joiner's skeleton (NewestFirst
        // reverses it — upstream rerun#12721).
        assert!(
            matches!(opts.playback_behavior, rerun::PlaybackBehavior::OldestFirst),
            "playback is OldestFirst"
        );
        // memory_limit is moot under the flag; ZERO makes "no temporal buffer"
        // explicit (the fork keeps statics regardless).
        assert_eq!(opts.memory_limit, rerun::MemoryLimit::ZERO);
    }
}
