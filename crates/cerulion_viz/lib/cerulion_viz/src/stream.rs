// SPDX-License-Identifier: AGPL-3.0-only
//! Process-shared Rerun `RecordingStream` lifecycle.
//!
//! Principle #8 analog: the Go2 viz uses ONE logical recording. A single shared
//! `OnceLock` cannot unify several sink instances — historically the old
//! `rerun_sink` cdylib could be instantiated more than once per graph and, under
//! the multi-process default, in separate PROCESSES, each with its own
//! statics. (That node has since been deleted; today `cerulion-vizd` is the one
//! consumer, so the multi-instance case is dormant rather than gone — the
//! unification below is what keeps it correct if it returns.) So every sink
//! builds its own `RecordingStream` with the SAME `application_id` +
//! `recording_id` and the Rerun viewer merges them into ONE recording. Within a
//! single process the stream is cached here so repeated `init()`s connect once.
//!
//! # Connection
//!
//! `connect_grpc()` (Rerun 0.34's replacement for the old `connect_tcp`)
//! attaches a gRPC client sink to a local viewer. Establishing the connection
//! is non-blocking (a viewer-absent connect buffers + retries in the
//! background), and a genuine connect *misconfiguration* (bad address) logs a
//! loud warning and disables logging (every `log_*` becomes a safe no-op)
//! rather than aborting the graph.
//!
//! **The LOG path, however, is NOT non-blocking.** When the viewer/server side
//! wedges (a suspended browser tab is enough — the server stops draining the
//! gRPC stream), the SDK's backpressure-only pipeline fills and `rec.log`
//! blocks INDEFINITELY (seen in a live incident). The sink's `tick()` is kept off
//! that blocking path by the [`crate::worker::VizLogWorker`], which owns
//! the stream and does all `rec.log` work on its own thread; the tick only
//! enqueues (non-blocking, dropping when the worker is wedged).
//!
//! # Reconnect
//!
//! The 0.34 gRPC client does NOT auto-reconnect after a server bounce
//! (`ClientConnectionState::Disconnected` is terminal — confirmed in
//! `re_grpc_client::write`), so a bounced server used to require a full graph
//! restart. [`reconnect`] swaps a fresh gRPC sink onto the EXISTING stream and
//! [`rearm_after_reconnect`] re-arms the once-guards so the bounced (empty)
//! server re-receives the scene statics + blueprint + skeleton tree; the worker
//! drives this on a health-probe timer.
//!
//! Target: the default Rerun gRPC address, or `$CERULION_RERUN_URL`
//! (e.g. `rerun+http://127.0.0.1:9876/proxy`) when set. That is the ONE name —
//! the old `$RERUN_VIZ_ADDR` fallback was REMOVED and
//! is read by nothing.

use std::sync::Mutex;

use rerun::RecordingStream;

/// Default application id — the viewer groups everything under this. The Go2
/// default; overridable via [`StreamConfig`].
pub const APP_ID: &str = "go2";
/// Default recording id — sinks (possibly in different processes) merge into
/// ONE recording in the viewer by sharing it. The Go2 default; overridable via
/// [`StreamConfig`].
pub const RECORDING_ID: &str = "go2_live";
/// Env var to override the viewer gRPC endpoint. NOT a Go2 value — the platform
/// endpoint override, identical for every robot.
pub const ADDR_ENV: &str = "CERULION_RERUN_URL";

/// Constant 1 (app/recording identity): the Rerun recording identity
/// the sink connects with. `Default` is the Go2 identity ([`APP_ID`] /
/// [`RECORDING_ID`]) so the demo stays byte-identical; the `cerulion viz` verb
/// (in later work) constructs a per-robot identity.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StreamConfig {
    /// The Rerun `application_id`.
    pub app_id: String,
    /// The Rerun `recording_id` (sinks sharing it merge into one recording).
    pub recording_id: String,
}

impl Default for StreamConfig {
    fn default() -> Self {
        Self {
            app_id: APP_ID.to_string(),
            recording_id: RECORDING_ID.to_string(),
        }
    }
}

struct StreamState {
    stream: Option<RecordingStream>,
    tried: bool,
}

static STREAM: Mutex<StreamState> = Mutex::new(StreamState {
    stream: None,
    tried: false,
});

fn lock() -> std::sync::MutexGuard<'static, StreamState> {
    STREAM
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// The process/cdylib-local recording stream, lazily connecting on first
/// use. Returns a cheap clone (`RecordingStream` is `Arc`-backed:
/// `Clone + Send + Sync`), or `None` if connection was misconfigured (in
/// which case callers no-op).
pub fn recording_stream() -> Option<RecordingStream> {
    let mut st = lock();
    if !st.tried && st.stream.is_none() {
        st.stream = connect_default();
        st.tried = true;
    }
    st.stream.clone()
}

/// Install a caller-provided stream (tests inject a `memory()` sink; an
/// embedding host could inject a pre-configured stream). Wins over the lazy
/// connect and replaces any prior stream.
pub fn set_stream(rec: RecordingStream) {
    let mut st = lock();
    st.stream = Some(rec);
    st.tried = true;
}

/// Reset the shared stream (test hygiene — lets an in-process test install a
/// fresh `memory()` sink). ALSO re-arms the scene-statics guard
/// ([`crate::tf::rearm_viz_statics`]), the blueprint-send guard
/// ([`crate::blueprint::rearm_blueprint`]), AND the skeleton
/// static-tree guard ([`crate::skeleton::rearm_skeleton_statics`]) so
/// ONE call restores the full clean-slate invariant: a later same-process graph
/// test re-receives the statics + blueprint + skeleton tree on its first sink
/// fire, keeping exact chunk-count oracles valid regardless of test ordering.
pub fn reset_for_test() {
    crate::tf::rearm_viz_statics();
    crate::blueprint::rearm_blueprint();
    // Also drop any runtime layout (`set_blueprint`) so a later
    // same-process test starts on the Go2 default, not a leaked layout. Unlike
    // `rearm_after_reconnect` — which PRESERVES the runtime layout so a reconnect
    // re-applies it — the test-hygiene reset clears it.
    crate::blueprint::clear_runtime_blueprint();
    crate::skeleton::rearm_skeleton_statics();
    let mut st = lock();
    st.stream = None;
    st.tried = false;
}

/// The configured viewer endpoint override (`$CERULION_RERUN_URL`), or `None`
/// for the SDK default address. Shared by the initial connect and [`reconnect`]
/// so both target the same viewer.
///
/// The ONE read site, and it reads exactly ONE name: [`ADDR_ENV`]. The old
/// `RERUN_VIZ_ADDR` fallback was REMOVED — a deprecated
/// alias that still works is a trap, since an operator with the old name
/// exported keeps a working setup that no documentation describes and no error
/// mentions, and the whole point of the rename was that ONE name is the knob.
fn resolve_addr() -> Option<String> {
    std::env::var(ADDR_ENV).ok()
}

/// The configured viewer-endpoint OVERRIDE, resolved through the ONE read site
/// (`resolve_addr`): [`ADDR_ENV`] if set, else `None`.
///
/// `Some(url)` means a controller/daemon should connect to THIS external
/// endpoint as a CLIENT (the power-user override); `None` means no override is
/// set — the caller is free to HOST its own endpoint (the `cerulion-vizd`
/// automagic default). Public so the daemon can pick host-vs-
/// client without re-implementing the resolution.
pub fn endpoint_override() -> Option<String> {
    resolve_addr()
}

fn connect_default() -> Option<RecordingStream> {
    connect_with_config(&StreamConfig::default())
}

/// Build the Rerun `RecordingStreamBuilder` carrying a given recording identity
/// ([`StreamConfig`]) — the pure builder-construction step shared by the
/// production connect ([`connect_with_config`]) and by tests, which attach a
/// non-network `memory()` sink to read back the identity that reached the built
/// stream's `StoreInfo`. Both `.connect_grpc()` and `.memory()` seed the store
/// info from this builder identically (via the SDK's builder `into_args`), so
/// exercising this helper pins the config→stream flow-through WITHOUT opening a
/// socket. Exposed so the `cerulion viz` verb / an embedding host can construct
/// a per-robot builder, and so the flow-through is unit-observable.
pub fn build_recording_builder(cfg: &StreamConfig) -> rerun::RecordingStreamBuilder {
    rerun::RecordingStreamBuilder::new(cfg.app_id.clone()).recording_id(cfg.recording_id.clone())
}

/// Build + connect a Rerun gRPC sink with the given recording identity
/// ([`StreamConfig`]). The default-config path is the production
/// `connect_default` — byte-identical to the earlier hardcoded connect —
/// so `recording_stream()` is unchanged for the demo; this is exposed so the
/// `cerulion viz` verb / an embedding host can connect a per-robot identity.
pub fn connect_with_config(cfg: &StreamConfig) -> Option<RecordingStream> {
    let builder = build_recording_builder(cfg);
    let result = match resolve_addr() {
        Some(url) => builder.connect_grpc_opts(url),
        None => builder.connect_grpc(),
    };
    match result {
        Ok(rec) => {
            tracing::info!(
                app_id = %cfg.app_id,
                recording_id = %cfg.recording_id,
                "Rerun viz: connected gRPC sink (viewer-absent logs buffer + retry; the sink tick \
                 never blocks — the viz worker owns the log path)"
            );
            Some(rec)
        }
        Err(e) => {
            tracing::warn!(
                error = %e,
                app_id = %cfg.app_id,
                "Rerun viz: gRPC connect misconfigured — visualization disabled for this \
                 run (logging is a no-op; set {ADDR_ENV} or start a viewer)",
            );
            None
        }
    }
}

/// Swap a FRESH gRPC sink onto the existing `rec` — the reconnect
/// primitive for a bounced server. The 0.34 client does not auto-reconnect
/// (`Disconnected` is terminal), so after a detected disconnect the worker calls
/// this to install a new client (which retries connecting to the new server) on
/// the SAME stream, keeping the process-shared statics guards addressable for
/// [`rearm_after_reconnect`]. Targets the same endpoint as the initial connect.
///
/// `set_sink` (which `connect_grpc*` wraps) flushes the OLD sink first; a truly
/// disconnected gRPC sink fails-fast, so this returns promptly. Returns the
/// error string on a bad-address swap.
pub fn reconnect(rec: &RecordingStream) -> Result<(), String> {
    let result = match resolve_addr() {
        Some(url) => rec.connect_grpc_opts(url),
        None => rec.connect_grpc(),
    };
    result.map_err(|e| e.to_string())
}

/// Re-arm the once-per-recording guards after a live [`reconnect`], so
/// the bounced (empty) server re-receives the scene statics + blueprint +
/// skeleton static tree on the next sink fire (a fresh server has none of them).
/// Shares the guard-rearm primitives with [`reset_for_test`] — the same "restore
/// the clean-slate statics invariant" operation, here on the live path.
pub fn rearm_after_reconnect() {
    crate::tf::rearm_viz_statics();
    crate::blueprint::rearm_blueprint();
    crate::skeleton::rearm_skeleton_statics();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn addr_env_is_the_pinned_literal() {
        // The daemon-side half of the mirror contract: the engine's
        // `cerulion_cli_engine::viz_cmd::RERUN_URL_ENV` MIRRORS this literal but
        // cannot import this crate (it pulls rerun), so both sides pin the SAME
        // literal against their own oracle. A rename here fails this pin; a rename
        // there fails `rerun_url_env_is_the_pinned_literal`.
        assert_eq!(ADDR_ENV, "CERULION_RERUN_URL");
    }
}
