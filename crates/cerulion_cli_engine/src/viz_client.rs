// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion-vizd` control client — how the `cerulion viz` verb drives the
//! long-lived viz daemon.
//!
//! `cerulion viz` is a THIN CLIENT of `cerulion-vizd`: it ensures the daemon is
//! running (spawning it detached if absent), connects over the daemon's Unix
//! domain socket, reads the `Hello` banner (which advertises the
//! `rerun_url` every viewer connects to), and sends one `attach` per requested
//! topic. The daemon owns the taps, the schema-generic dispatch, and the hosted
//! Rerun gRPC proxy — so this verb, like `ros2 attach`, **never links `rerun`**;
//! it speaks the daemon's NDJSON protocol over the socket and nothing more.
//!
//! # Why hand-rolled NDJSON (not a shared type crate)
//!
//! The daemon's protocol types live in the `cerulion_vizd` crate, which pulls the
//! `rerun` SDK (via `cerulion_viz`). Depending on it here would drag `rerun` +
//! its 1.93 MSRV into the lean default-members CLI — exactly the coupling the
//! daemon decoupling exists to avoid. So this module hand-rolls the small,
//! forward-compatible request lines with `serde_json` and reads responses
//! field-by-field. The daemon's `protocol.rs` is the SOURCE OF TRUTH; the request
//! shapes here are pinned by oracle tests that mirror its own.
//!
//! # Unix-only
//!
//! The daemon speaks over a Unix domain socket, so the client half is Unix-only.
//! On a non-Unix host the verb reports that `cerulion viz` needs the daemon (a
//! Unix socket) rather than silently doing nothing.

use std::path::PathBuf;
use std::time::Duration;

use serde::Serialize;

/// The env var overriding the daemon's control-socket path. MIRRORS
/// `cerulion_vizd::hygiene::SOCKET_ENV` (that crate pulls `rerun`, so it is not a
/// dependency of this engine — the same decoupling `RERUN_URL_ENV` uses). Kept in
/// sync; the resolution shape is pinned by the module's
/// `socket_env_override_wins_and_default_ladder` test.
pub const VIZD_SOCKET_ENV: &str = cerulion_hygiene::VIZD.socket_env();

/// The daemon binary name discovered on `$PATH` / alongside the running CLI. The
/// CLI never links `rerun` — it SPAWNS this external binary (the `ros2 attach`
/// precedent), which is kept out of `default-members` so a lean CLI build never
/// pulls the SDK.
pub const VIZD_BINARY: &str = "cerulion-vizd";

/// The env var the daemon reads for CONNECT locators. MIRRORS
/// `cerulion_vizd::net::CONNECT_ENV` (that crate pulls `rerun`, so it is not a
/// dependency here — the same decoupling `VIZD_SOCKET_ENV` uses). The verb sets it
/// on the spawned daemon from `--connect` (comma-joined); the daemon splits it.
pub const VIZD_CONNECT_ENV: &str = "CERULION_VIZD_CONNECT";

/// The env var the daemon reads for LISTEN locators. MIRRORS
/// `cerulion_vizd::net::LISTEN_ENV`. Set on the spawned daemon from `--listen`.
pub const VIZD_LISTEN_ENV: &str = "CERULION_VIZD_LISTEN";

/// Resolve the daemon's control-socket path. The daemon binds
/// [`cerulion_hygiene::VIZD`]'s ladder and this client resolves the SAME
/// constant, so the two cannot drift (never a hand-mirrored copy):
/// [`VIZD_SOCKET_ENV`] if set, else `$XDG_RUNTIME_DIR/cerulion/vizd.sock`, else
/// `$HOME/.cerulion/vizd.sock`, else `/tmp/cerulion-<euid>/vizd.sock`.
pub fn vizd_socket_path() -> PathBuf {
    cerulion_hygiene::VIZD.default_socket_path()
}

/// Find the `cerulion-vizd` binary to spawn: FIRST alongside the currently-running
/// `cerulion` executable (so a co-located release build is used before any stray
/// `$PATH` copy), then alongside its RESOLVED path, then on `$PATH`. `None` ⇒ the
/// daemon binary is not installed ⇒ the verb degrades LOUDLY with an install/build
/// hint.
///
/// # Why the resolved-path rung
///
/// `std::env::current_exe()` is NOT canonicalised on macOS — MEASURED: a binary
/// invoked through a symlink reports the SYMLINK, and only Linux's
/// `/proc/self/exe` hands back the target. So a `cerulion` reached through a
/// symlink looks for the daemon in the symlink's directory, skips the release
/// build sitting beside its REAL self, and falls through to whatever `$PATH`
/// happens to hold — a stale copy, or nothing.
///
/// This is the same hazard `cerulion_netd`'s spawn ladder guards against, where
/// it presents as ENOENT spawn failures and an empty Studio
/// sidebar. It is milder here only because the `$PATH` rung can mask it, and a
/// mask is exactly what makes it worth closing: the failure would present as
/// "wrong daemon version" rather than as a missing file.
pub fn find_vizd_binary() -> Option<PathBuf> {
    find_vizd_binary_for(std::env::current_exe().ok().as_deref())
}

/// The IO shell over ONE known executable path.
///
/// Split out so the `canonicalize` step is REACHABLE FROM A TEST: the pure
/// [`vizd_sibling_candidates`] oracles inject the resolved path, so without this
/// nothing observes whether production ever asks for it — and that is the line
/// that fixes the symlinked-CLI case.
pub fn find_vizd_binary_for(exe: Option<&std::path::Path>) -> Option<PathBuf> {
    // `.ok()` on canonicalize: a failure is the ABSENCE of a rung (a deleted
    // binary, a permission wall on an ancestor), never a reason to stop looking.
    let resolved = exe.and_then(|e| std::fs::canonicalize(e).ok());
    for sibling in vizd_sibling_candidates(exe, resolved.as_deref()) {
        if is_executable_file(&sibling) {
            return Some(sibling);
        }
    }
    let path = std::env::var_os("PATH")?;
    std::env::split_paths(&path)
        .map(|dir| dir.join(exe_name(VIZD_BINARY)))
        .find(|candidate| is_executable_file(candidate))
}

/// The PURE sibling rungs [`find_vizd_binary`] tries, in order, deduped
/// (oracle-tested): the daemon beside `exe`, then beside `resolved`.
///
/// Deduped because the two agree on Linux (`/proc/self/exe` is already resolved)
/// and on any un-symlinked install, so the second rung can only ever ADD a
/// directory the running binary genuinely came from.
pub fn vizd_sibling_candidates(
    exe: Option<&std::path::Path>,
    resolved: Option<&std::path::Path>,
) -> Vec<PathBuf> {
    let mut out: Vec<PathBuf> = Vec::new();
    for candidate in [exe, resolved].into_iter().flatten() {
        if let Some(sibling) = candidate.parent().map(|d| d.join(exe_name(VIZD_BINARY))) {
            if !out.contains(&sibling) {
                out.push(sibling);
            }
        }
    }
    out
}

/// The platform executable filename for `name` (adds `.exe` on Windows).
fn exe_name(name: &str) -> String {
    if cfg!(windows) {
        format!("{name}.exe")
    } else {
        name.to_string()
    }
}

#[cfg(unix)]
fn is_executable_file(path: &std::path::Path) -> bool {
    use std::os::unix::fs::PermissionsExt;
    match std::fs::metadata(path) {
        Ok(m) => m.is_file() && (m.permissions().mode() & 0o111 != 0),
        Err(_) => false,
    }
}

#[cfg(not(unix))]
fn is_executable_file(path: &std::path::Path) -> bool {
    path.is_file()
}

// ── Request-line builders (PURE — mirror the daemon's `protocol::Request`) ────
//
// Built from serde-DERIVED structs (NOT a runtime `serde_json::Map`) so the field
// order is the struct's DECLARATION order — stable regardless of serde_json's
// `preserve_order` feature. `method` is emitted as a plain field, which the
// daemon's internally-tagged `#[serde(tag = "method")]` enum consumes as the tag
// (the daemon parses order-independently anyway; the stable line just keeps the
// oracle tests deterministic).

/// A method-only request (`discover` / `list` / `status`).
#[derive(Serialize)]
struct SimpleReq<'a> {
    id: u64,
    method: &'a str,
}

/// A `detach` request.
#[derive(Serialize)]
struct DetachReq<'a> {
    id: u64,
    method: &'a str,
    topic: &'a str,
}

/// An `attach` request — optional fields OMITTED when `None` (the daemon's
/// `skip_serializing_if` shape).
#[derive(Serialize)]
struct AttachReq<'a> {
    id: u64,
    method: &'a str,
    topic: &'a str,
    #[serde(skip_serializing_if = "Option::is_none")]
    entity: Option<&'a str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    robot: Option<&'a str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    schema: Option<&'a str>,
}

fn line<T: Serialize>(req: &T) -> String {
    serde_json::to_string(req).expect("request serializes")
}

/// Build the `discover` NDJSON request line.
pub fn discover_line(id: u64) -> String {
    line(&SimpleReq {
        id,
        method: "discover",
    })
}

/// Build the `list` NDJSON request line.
pub fn list_line(id: u64) -> String {
    line(&SimpleReq { id, method: "list" })
}

/// Build the `status` NDJSON request line.
pub fn status_line(id: u64) -> String {
    line(&SimpleReq {
        id,
        method: "status",
    })
}

/// What a `schemas` side-load actually achieved at the daemon.
///
/// Deliberately NOT a bare count. The daemon answers `ok: false` for a request
/// it could not act on at all — including an OLDER daemon that does not know the
/// verb, which answers a structured error rather than dropping the connection —
/// and a caller reading only `accepted` sees `0` for that, indistinguishable
/// from "it already had everything". A bag player would then retry silently
/// forever while the scene renders nothing, which is the exact success-shaped
/// failure the schema offer exists to remove.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SchemaPushOutcome {
    /// The daemon acted on the request. `accepted` = how many types became
    /// decodable that were not before; `rejected` names any it could not use (a
    /// built-in redefinition it refuses, a workspace-YAML doc it has no parser
    /// for, a `.msg` that would not parse).
    Applied {
        /// Types that became decodable and were not before.
        accepted: u32,
        /// Types the daemon would not or could not take, by qualified name.
        rejected: Vec<String>,
    },
    /// The daemon refused the request outright (`ok: false`) — most often a
    /// daemon too old to know the verb. Carries its stated reason.
    Refused {
        /// The daemon's own error text, or a stand-in when it gave none.
        reason: String,
    },
}

impl SchemaPushOutcome {
    /// Classify a `schemas` reply. PURE — oracle-tested.
    pub fn from_reply(v: &serde_json::Value) -> Self {
        if v.get("ok").and_then(serde_json::Value::as_bool) != Some(true) {
            return SchemaPushOutcome::Refused {
                reason: v
                    .get("error")
                    .and_then(serde_json::Value::as_str)
                    .unwrap_or("the daemon rejected the request and gave no reason")
                    .to_string(),
            };
        }
        SchemaPushOutcome::Applied {
            accepted: v
                .get("accepted")
                .and_then(serde_json::Value::as_u64)
                .unwrap_or(0) as u32,
            rejected: v
                .get("rejected")
                .and_then(serde_json::Value::as_array)
                .map(|a| {
                    a.iter()
                        .filter_map(|e| e.as_str().map(str::to_string))
                        .collect()
                })
                .unwrap_or_default(),
        }
    }
}

/// Build the `schemas` NDJSON request line — side-load schema
/// definitions into a running daemon's walker, so it can decode a type it never
/// compiled.
pub fn schemas_line(id: u64, docs: &[cerulion_core::SchemaDoc]) -> String {
    serde_json::json!({ "method": "schemas", "id": id, "docs": docs }).to_string()
}

/// Build the `detach` NDJSON request line.
pub fn detach_line(id: u64, topic: &str) -> String {
    line(&DetachReq {
        id,
        method: "detach",
        topic,
    })
}

/// Build an `attach` NDJSON request line. `entity` (render override), `robot`
/// (the remote arm), and `schema` (the ROS type, required with `robot`) are
/// OMITTED when `None`, so a local attach is `{"id":..,"method":"attach","topic":".."}`.
pub fn attach_line(
    id: u64,
    topic: &str,
    entity: Option<&str>,
    robot: Option<&str>,
    schema: Option<&str>,
) -> String {
    line(&AttachReq {
        id,
        method: "attach",
        topic,
        entity,
        robot,
        schema,
    })
}

// ── The Unix-socket connection + ensure-daemon (the live client half) ─────────

#[cfg(unix)]
pub use unix_client::*;

#[cfg(unix)]
mod unix_client {
    use super::*;
    use std::io::{self, BufRead, BufReader, Write};
    use std::os::unix::net::UnixStream;
    use std::path::Path;
    use std::time::Instant;

    /// Per-read/-write timeout on the control connection (banner + every
    /// request/response). The daemon writes the banner SYNCHRONOUSLY on accept — but it
    /// is FATAL, not merely slow, that this exists: without it a
    /// wedged daemon (or a foreign process that accepts the socket but never
    /// writes) blocks the blocking `read_line` FOREVER, and the ctrl-c handler only
    /// flips a flag the blocked read never checks → an un-interruptible hang. The
    /// timeout turns a wedged read/write into an actionable `Err` the verb surfaces
    /// + exits nonzero on (`socket read timed out`), never a hang.
    ///
    /// **This bound is why the discovery wait is CLIENT-side.** It is armed on
    /// EVERY reply read, not just the banner, so any daemon-side wait longer than it
    /// does not slow the verb down — it kills it, on precisely the cold-desk attach a
    /// wait would exist to rescue. `cerulion-vizd` therefore answers a topic-resolution
    /// question in one `cerulion-netd` round trip and hands back an [`AttachRetry`]; the
    /// loop lives in [`VizdConn::attach_waiting_for_discovery`]. Do not raise this to
    /// buy a daemon-side wait — move the wait.
    ///
    /// # What actually bounds a reply
    ///
    /// "One round trip" is the STEADY-STATE bound, not a guarantee. `NetdDemandPlane`
    /// connects LAZILY (`with_client` runs `NetdClient::connect_or_spawn` on its first
    /// use), and with explicit topics the first demand-plane call of a daemon's life IS
    /// an attach's gather — so a first-use attach can additionally pay netd's
    /// spawn-READINESS wait, whose own ceiling (`SPAWN_READY_TIMEOUT`, 10 s, retried
    /// once) exceeds this deadline outright.
    ///
    /// That is a property of the lazy spawn path, independent of the convergence wait,
    /// and the composed worst case was MEASURED on an idle desk at ~1.9 s,
    /// comfortably inside the deadline, because the readiness wait polls and returns as
    /// soon as the socket answers rather than spending its ceiling. The exposure is a
    /// desk where netd genuinely takes seconds to become ready; the failure there is
    /// the loud, actionable `Err` this timeout exists to produce.
    ///
    /// `the_reply_deadline_clears_netds_own_cold_start_grace` pins the one half of that
    /// arithmetic that IS in code: this deadline must exceed
    /// [`COLD_START_DISCOVERY_BUDGET`](cerulion_netd::COLD_START_DISCOVERY_BUDGET), or a
    /// cold first answer could never arrive at all.
    const CONN_IO_TIMEOUT: Duration = Duration::from_secs(5);

    /// The longest [`VizdConn::attach_waiting_for_discovery`] sleeps without
    /// re-checking the caller's cancellation flag.
    ///
    /// `cerulion` replaces the default SIGINT/SIGTERM/SIGHUP disposition before the viz
    /// verb runs, and `std::thread::sleep` resumes across `EINTR`, so an UNSLICED sleep
    /// makes Ctrl-C a no-op for its whole length. The actual bound this buys is ONE
    /// ROUND TRIP PLUS ONE SLICE, not one slice: cancellation is observed at round-trip
    /// boundaries and inside the sleep, and a round trip in flight is uninterruptible
    /// for up to `CONN_IO_TIMEOUT` (`BufRead::read_line` retries `Interrupted`
    /// internally). Same value and same reasoning as `cerulion_netd`'s
    /// `CANCEL_CHECK_SLICE`; 100 ms is below the ~150 ms an interactive user reads as
    /// instant.
    const CANCEL_CHECK_SLICE: Duration = Duration::from_millis(100);

    /// THE production first-contact policy for `cerulion viz`'s attach loop —
    /// the SHIPPED ceiling + cadence, and the only place the verb declares one.
    ///
    /// It is a function rather than a `pub use` so `cerulion_cli` need not depend on
    /// `cerulion_netd` (the CLI binary is a thin clap wrapper; the engine already owns
    /// that edge), and so the verb cannot quietly mint its own numbers — a
    /// vizd-specific ceiling would make Studio and `topic hz` wait different amounts
    /// for the same daemon.
    pub fn first_contact_attach_policy() -> cerulion_netd::ConvergenceWait {
        cerulion_netd::ConvergenceWait::default()
    }

    /// Has the caller asked us to stop? `None` (no flag) is never cancelled.
    fn is_cancelled(running: Option<&std::sync::atomic::AtomicBool>) -> bool {
        running.is_some_and(|r| !r.load(std::sync::atomic::Ordering::Relaxed))
    }

    /// Sleep `total`, re-checking cancellation every [`CANCEL_CHECK_SLICE`]. Returns
    /// `false` if the wait was cancelled part-way through.
    fn sleep_cancellable(total: Duration, running: Option<&std::sync::atomic::AtomicBool>) -> bool {
        let deadline = Instant::now() + total;
        loop {
            if is_cancelled(running) {
                return false;
            }
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return true;
            }
            std::thread::sleep(remaining.min(CANCEL_CHECK_SLICE));
        }
    }

    /// The reply deadline, for the guard that relates it to netd's own budgets.
    /// A function rather than a `pub` const so the value stays one definition, and
    /// `cfg(test)` so the shipped surface gains nothing.
    #[cfg(test)]
    pub(crate) fn conn_io_timeout() -> Duration {
        CONN_IO_TIMEOUT
    }

    /// The cancellation slice, for the bounds pin that guards Ctrl-C
    /// responsiveness. Same shape and same reasoning as [`conn_io_timeout`] — a
    /// `cfg(test)` function rather than a `pub` const, so the value stays ONE
    /// definition and the shipped surface gains nothing.
    #[cfg(test)]
    pub(crate) fn cancel_check_slice() -> Duration {
        CANCEL_CHECK_SLICE
    }

    /// Set the read + write timeouts on `stream` (shared across its `try_clone`d
    /// fds — `SO_RCVTIMEO`/`SO_SNDTIMEO` are socket-level). A failure to set them
    /// makes the socket unusable (an un-timeoutable connection is exactly the hang
    /// this guards against), so it is a hard `Err`.
    fn arm_io_timeouts(stream: &UnixStream) -> io::Result<()> {
        stream.set_read_timeout(Some(CONN_IO_TIMEOUT))?;
        stream.set_write_timeout(Some(CONN_IO_TIMEOUT))?;
        Ok(())
    }

    /// The daemon's connect banner (the subset the verb reads): the protocol
    /// version + the advertised `rerun_url` a viewer connects to.
    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct Banner {
        /// The daemon's [`protocol`](Self::protocol) version.
        pub protocol: u64,
        /// The Rerun endpoint the daemon HOSTS + every viewer connects to
        /// (`None` when viz is disabled on the daemon side).
        pub rerun_url: Option<String>,
    }

    /// A parsed daemon response (the subset the verb reads).
    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct AttachReply {
        /// `true` on success, `false` on a daemon error.
        pub ok: bool,
        /// The daemon's actionable error (surfaced VERBATIM) when `ok == false`.
        pub error: Option<String>,
        /// The resolved schema qualified name, or `None` (silent / unresolved).
        pub schema: Option<String>,
        /// The resolved Rerun archetype family, or `None`.
        pub archetype: Option<String>,
        /// The full Rerun entity path the topic renders under.
        pub entity: Option<String>,
        /// `true` when the daemon reports the topic was ALREADY tapped by an
        /// earlier controller (the daemon taps are GLOBAL shared state). The verb
        /// uses this to detach ONLY the taps IT created on Ctrl+C — tearing down a
        /// tap this run did not open would silently freeze a concurrent Studio /
        /// `--detach` controller's scene (`false` when the daemon omits the field —
        /// a pre-`already_attached` daemon or an error reply).
        pub already_attached: bool,
        /// PRESENT iff the daemon says this failure is a NOT-FOUND-**YET**
        /// that re-asking can change (vizd's `ErrorResponse.retry`). Absent (on a
        /// success, on a terminal failure, and on any older daemon) means DO NOT
        /// retry, which is the older behaviour exactly.
        pub retry: Option<AttachRetry>,
    }

    /// The daemon's structured retry hint on an attach failure.
    ///
    /// `cerulion-vizd` answers the CLOSED-WORLD question ("does this LAN serve the
    /// topic *now*?") in ONE round trip; it deliberately does not block on the
    /// OPEN-WORLD one ("*will* it?"). It cannot: `CONN_IO_TIMEOUT` bounds every reply
    /// read on this very socket, so a daemon-side wait past it would not make the verb
    /// slower — it would make it FAIL, on exactly the cold-desk attach a wait exists to
    /// rescue. So the daemon forwards the facts and the CLIENT keeps asking, under
    /// `cerulion_netd`'s shipped `ConvergenceWait` policy — the SAME policy
    /// `topic echo`/`hz`/`schema info` run, so the two surfaces cannot drift
    /// on when to stop.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct AttachRetry {
        /// How much netd had actually read when it answered. An UNRECOGNISED wire word
        /// decodes as `NotConverged` — UNKNOWN is never a positive claim, and the cost
        /// of the conservative reading is a bounded wait, never a wrong verdict.
        pub discovery: cerulion_netd::protocol::DiscoveryState,
        /// netd's un-settled plane age (`None` = settled, or a daemon too old to report
        /// it — UNKNOWN, which caps nothing: the same rule `ConvergenceWait::decide`
        /// applies, so it holds identically on both sides of the wire).
        pub plane_unsettled: Option<Duration>,
    }

    /// How a first-contact wait ENDED — re-exported from `cerulion_netd` rather
    /// than duplicated, so the CLI's viz surface and its `topic`/`schema` surfaces speak
    /// ONE vocabulary for the same three outcomes.
    pub use cerulion_netd::WaitOutcome;

    /// An attach reply plus **how the wait that produced it ended**.
    ///
    /// The outcome is carried rather than left to be re-derived, for the reason
    /// [`WaitOutcome`]'s own docs give: `Cancelled` means "NO give-up claim may be made
    /// from it". A loop that returned a bare [`AttachReply`] from all
    /// four exits would hand back, for a wait the USER interrupted at 2 s of 10, the same
    /// (empty, not-converged) reply as a wait that ran its full budget — and
    /// `cerulion viz` would print the per-topic UNKNOWN ABSENCE CLAIM and exit
    /// nonzero for a question nobody finished asking. That is precisely the defect
    /// the `topic` verbs refuse, one surface over.
    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct AttachOutcome {
        /// The last reply the daemon gave.
        pub reply: AttachReply,
        /// How the wait ended. `Cancelled` licenses NO claim about the topic.
        pub outcome: WaitOutcome,
    }

    /// A live control connection to the daemon: send a request line, read one
    /// response line back.
    pub struct VizdConn {
        writer: UnixStream,
        reader: BufReader<UnixStream>,
        banner: Banner,
        /// Set while a round trip is IN FLIGHT and left set if it failed —
        /// see [`VizdConn::request`].
        desynced: bool,
        /// Has a retry hint on this connection proved the netd behind it
        /// cannot report a plane age? See
        /// [`VizdConn::attach_waiting_for_discovery`]'s stale-daemon memo.
        unreportable_plane_age_seen: bool,
    }

    impl VizdConn {
        /// Connect to the daemon at `socket`, reading + parsing the Hello banner.
        /// Arms a read/write timeout (`CONN_IO_TIMEOUT`) FIRST, so a daemon that
        /// accepts but never writes the banner (wedged / a foreign listener) times
        /// out with an `Err` instead of hanging the (un-interruptible) blocking
        /// banner read forever.
        pub fn connect(socket: &Path) -> io::Result<Self> {
            let stream = UnixStream::connect(socket)?;
            arm_io_timeouts(&stream)?;
            let writer = stream.try_clone()?;
            let mut reader = BufReader::new(stream);
            let mut banner_line = String::new();
            reader.read_line(&mut banner_line)?;
            let v: serde_json::Value = serde_json::from_str(banner_line.trim()).map_err(|e| {
                io::Error::new(io::ErrorKind::InvalidData, format!("bad vizd banner: {e}"))
            })?;
            let banner = Banner {
                protocol: v
                    .get("protocol")
                    .and_then(serde_json::Value::as_u64)
                    .unwrap_or(0),
                rerun_url: v
                    .get("rerun_url")
                    .and_then(serde_json::Value::as_str)
                    .map(str::to_string),
            };
            Ok(VizdConn {
                writer,
                reader,
                banner,
                desynced: false,
                unreportable_plane_age_seen: false,
            })
        }

        /// The banner the daemon sent on connect (protocol version + `rerun_url`).
        pub fn banner(&self) -> &Banner {
            &self.banner
        }

        /// Send one request line, read one response line, parse to JSON.
        ///
        /// A FAILED round trip POISONS the connection. The protocol is
        /// strictly one reply per request in order, so a read that timed out
        /// (`CONN_IO_TIMEOUT`) leaves the daemon's reply still in flight — the next
        /// `request` on this connection would read the PREVIOUS request's answer and
        /// mis-attribute it. `run_viz_unix` happens to `?`-exit on the first such
        /// error, but nothing structural guaranteed that, and a silently mis-paired
        /// reply is a far worse failure than a loud refusal.
        pub fn request(&mut self, line: &str) -> io::Result<serde_json::Value> {
            if self.desynced {
                return Err(io::Error::other(
                    "the cerulion-vizd control connection is desynced after an earlier \
                     failed round trip (a reply may still be in flight) — reconnect \
                     rather than risk pairing it with this request",
                ));
            }
            self.desynced = true;
            writeln!(self.writer, "{line}")?;
            let mut resp = String::new();
            self.reader.read_line(&mut resp)?;
            let parsed = serde_json::from_str(resp.trim()).map_err(|e| {
                io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("bad vizd response: {e}"),
                )
            })?;
            // A complete, parsed reply means the stream is back in step.
            self.desynced = false;
            Ok(parsed)
        }

        /// Has an earlier round trip left this connection unusable? (Principle #3.)
        pub fn is_desynced(&self) -> bool {
            self.desynced
        }

        /// `attach` a topic (local when `robot`/`schema` are `None`), returning the
        /// parsed reply (the caller surfaces `ok`/`error`/placement).
        pub fn attach(
            &mut self,
            id: u64,
            topic: &str,
            entity: Option<&str>,
            robot: Option<&str>,
            schema: Option<&str>,
        ) -> io::Result<AttachReply> {
            let v = self.request(&attach_line(id, topic, entity, robot, schema))?;
            Ok(parse_attach_reply(&v))
        }

        /// [`Self::attach`], but KEEP ASKING while the daemon says the topic is
        /// not discoverable **yet** — the client-side first-contact wait.
        ///
        /// # Why the loop is here and not in the daemon
        ///
        /// Real-LAN convergence was MEASURED at ~3 to 13 s, so the FIRST attach against
        /// a cold desk fails on a topic that is about to exist. The obvious fix — make
        /// `cerulion-vizd` wait before replying — is not available: `CONN_IO_TIMEOUT`
        /// arms `SO_RCVTIMEO` on this socket, so a reply that takes longer never
        /// arrives, and the verb dies on an errno instead of being a few seconds slower.
        /// Moving the loop client-side fixes that by construction (every round trip
        /// stays inside its own deadline) and buys three things besides: the daemon's one
        /// netd client and this connection stay free between polls, cancellation is the
        /// caller's Ctrl-C flag, and the party that decides how long to wait is the one
        /// that knows a human is watching.
        ///
        /// All POLICY is `cerulion_netd::ConvergenceWait` — the same pure, oracle-tested
        /// state machine `topic echo`/`hz`/`schema info` run — so the surfaces cannot
        /// drift. In particular a `Settled` hint answers on the FIRST round trip: netd's
        /// discovery demonstrably works here, so a typo costs nothing.
        ///
        /// `progress` is called once per additional poll with the elapsed wait (the
        /// caller decides where it goes; the CLI writes stderr). `running` follows the
        /// CLI idiom — `Some(flag)` cancels when it goes FALSE.
        ///
        /// The [`AttachOutcome`] carries HOW the wait ended, and the caller must honour
        /// it: a `Cancelled` reply is an INTERRUPTION, and rendering its (empty,
        /// not-converged) contents as an absence claim is the exact bug that motivated
        /// the type.
        // The argument list mirrors `attach`'s (five identifying parameters the
        // protocol itself requires) plus the three the wait needs — policy, cancel
        // flag, progress sink. Bundling them would hide which caller declares which.
        #[allow(clippy::too_many_arguments)]
        pub fn attach_waiting_for_discovery(
            &mut self,
            id: u64,
            topic: &str,
            entity: Option<&str>,
            robot: Option<&str>,
            schema: Option<&str>,
            policy: cerulion_netd::ConvergenceWait,
            running: Option<&std::sync::atomic::AtomicBool>,
            progress: &mut dyn FnMut(Duration),
        ) -> io::Result<AttachOutcome> {
            let started = Instant::now();
            let done = |reply, outcome| Ok(AttachOutcome { reply, outcome });
            // Read BEFORE the loop: an earlier attach on this connection may already
            // have SPENT a ceiling against a daemon that cannot report a plane age.
            let stale_plane_before_this_attach = self.unreportable_plane_age_seen;
            // Noted per round trip, COMMITTED only if this attach exhausts its budget.
            let mut saw_unreportable_plane_age = false;
            loop {
                let reply = self.attach(id, topic, entity, robot, schema)?;
                // A success, or a TERMINAL failure (no hint): answer now. The hint's
                // PRESENCE is this seam's `answer_empty` — the daemon emits it only
                // when the thing asked for was absent from a discovery answer.
                if reply.retry.is_none() {
                    return done(reply, WaitOutcome::Answered);
                }
                let hint = reply.retry.expect("checked just above");
                // The STALE-DAEMON signature. `NotConverged` with NO plane age
                // is what a daemon that cannot report a plane age looks like: an
                // older netd (no `plane_unsettled_ms` on the wire at all), or a
                // still older one whose genuinely-SETTLED plane the trust gate downgrades
                // to `NotConverged` while `unsettled_for()` returns `None`. Against
                // either, the cap can never fire, so every not-found topic would
                // pay the full ceiling and a twelve-topic invocation costs twelve.
                //
                // Only NOTED here — the memo is ARMED at the give-up exit below.
                if hint.discovery == cerulion_netd::protocol::DiscoveryState::NotConverged
                    && hint.plane_unsettled.is_none()
                {
                    saw_unreportable_plane_age = true;
                }
                let waited = started.elapsed();
                // Cancellation is checked FIRST — before the memo, before `decide`.
                //
                // It sits above the memo's early return deliberately: with the memo
                // armed, a return that skipped this check would make Ctrl-C invisible for
                // every remaining topic, so the verb would print absence paragraphs and
                // exit nonzero on the stale-daemon path — one branch
                // over, exactly the defect `AttachOutcome` exists to prevent.
                if is_cancelled(running) {
                    return done(reply, WaitOutcome::Cancelled);
                }
                if stale_plane_before_this_attach {
                    return done(reply, WaitOutcome::GaveUp);
                }
                match policy.decide(hint.discovery, true, waited, hint.plane_unsettled) {
                    cerulion_netd::WaitDecision::Proceed => {
                        return done(reply, WaitOutcome::Answered)
                    }
                    cerulion_netd::WaitDecision::GiveUpHonestUnknown => {
                        // ARM the memo HERE — the one exit where a ceiling was actually
                        // SPENT. Arming on the mere SIGHTING of a no-age hint is wrong
                        // and measurably so: an attach that sees one such hint and then
                        // SUCCEEDS at 110 ms arms it anyway, so the next topic gets
                        // 16 µs of wait and gives up — zero ceilings paid all run, on the
                        // designed happy path (a cold desk converging after a poll or
                        // two). The memo must only ever suppress a wait that had already
                        // been proven not to pay off.
                        if saw_unreportable_plane_age {
                            self.unreportable_plane_age_seen = true;
                        }
                        return done(reply, WaitOutcome::GaveUp);
                    }
                    cerulion_netd::WaitDecision::KeepWaiting { next_poll_delay } => {
                        progress(waited);
                        if !sleep_cancellable(next_poll_delay, running) {
                            return done(reply, WaitOutcome::Cancelled);
                        }
                    }
                }
            }
        }

        /// `detach` a topic (idempotent — the daemon no-ops an un-tapped one).
        pub fn detach(&mut self, id: u64, topic: &str) -> io::Result<()> {
            let _ = self.request(&detach_line(id, topic))?;
            Ok(())
        }

        /// `schemas`: side-load schema definitions so the daemon can
        /// decode a type it never compiled.
        ///
        /// Idempotent at the daemon, so a caller may re-send the same set freely
        /// — which the bag player does, so that a viewer started mid-playback
        /// still learns the definitions the bag carries.
        pub fn push_schemas(
            &mut self,
            id: u64,
            docs: &[cerulion_core::SchemaDoc],
        ) -> io::Result<SchemaPushOutcome> {
            let v = self.request(&schemas_line(id, docs))?;
            Ok(SchemaPushOutcome::from_reply(&v))
        }

        /// `discover` every attachable local topic, returning `(topic, schema)`
        /// pairs (schema `None` for a silent / undecodable topic).
        pub fn discover(&mut self, id: u64) -> io::Result<Vec<(String, Option<String>)>> {
            let v = self.request(&discover_line(id))?;
            let topics = v
                .get("topics")
                .and_then(serde_json::Value::as_array)
                .map(|arr| {
                    arr.iter()
                        .filter_map(|e| {
                            let t = e.get("topic").and_then(serde_json::Value::as_str)?;
                            let s = e
                                .get("schema")
                                .and_then(serde_json::Value::as_str)
                                .map(str::to_string);
                            Some((t.to_string(), s))
                        })
                        .collect()
                })
                .unwrap_or_default();
            Ok(topics)
        }
    }

    /// Parse a daemon response Value into the [`AttachReply`] subset the verb reads.
    pub fn parse_attach_reply(v: &serde_json::Value) -> AttachReply {
        AttachReply {
            ok: v
                .get("ok")
                .and_then(serde_json::Value::as_bool)
                .unwrap_or(false),
            error: v
                .get("error")
                .and_then(serde_json::Value::as_str)
                .map(str::to_string),
            schema: v
                .get("schema")
                .and_then(serde_json::Value::as_str)
                .map(str::to_string),
            archetype: v
                .get("archetype")
                .and_then(serde_json::Value::as_str)
                .map(str::to_string),
            entity: v
                .get("entity")
                .and_then(serde_json::Value::as_str)
                .map(str::to_string),
            // Missing / non-bool ⇒ `false`: an error reply (no such field) and a
            // pre-`already_attached` daemon both read as "this run created the tap"
            // — the SAFE default (the run detaches only taps it can prove it made,
            // and a genuine repeat attach carries `already_attached:true`).
            already_attached: v
                .get("already_attached")
                .and_then(serde_json::Value::as_bool)
                .unwrap_or(false),
            retry: parse_retry_hint(v.get("retry")),
        }
    }

    /// Decode vizd's `retry` hint. PURE — oracle-tested.
    ///
    /// ABSENT (and a non-object) ⇒ `None` ⇒ TERMINAL, which is the fail-closed reading
    /// AND the older behaviour: a client that cannot understand the hint must not
    /// invent a retry loop. An unrecognised `discovery` word decodes as `NotConverged`
    /// for one reason (an unknown marker is never a positive claim) and costs
    /// at most a bounded wait.
    pub fn parse_retry_hint(v: Option<&serde_json::Value>) -> Option<AttachRetry> {
        use cerulion_netd::protocol::DiscoveryState;
        let v = v?;
        if !v.is_object() {
            return None;
        }
        let discovery = match v.get("discovery").and_then(serde_json::Value::as_str) {
            Some("settled") => DiscoveryState::Settled,
            _ => DiscoveryState::NotConverged,
        };
        Some(AttachRetry {
            discovery,
            plane_unsettled: v
                .get("plane_unsettled_ms")
                .and_then(serde_json::Value::as_u64)
                .map(Duration::from_millis),
        })
    }

    /// Spawn `cerulion-vizd` DETACHED (a new session via `setsid(2)`) so the
    /// long-lived daemon OUTLIVES the verb and a terminal close does not kill it.
    /// The production spawn handed to [`ensure_daemon`]; spawns + drops the child
    /// (std's `Child::drop` neither waits nor kills).
    ///
    /// The daemon's stderr — where `init_tracing` writes EVERY diagnostic (startup
    /// failures, the host-mode "proxy did not come up" warn, …) — is redirected to
    /// `log_path` (created, APPEND) instead of `/dev/null`, so a daemon that dies
    /// before it can bind leaves a RECOVERABLE reason the verb can point at (rather
    /// than a silent 10s connect-retry stall against nothing). stdin/stdout still
    /// go to null. Appends (never truncates) so a prior crash's log survives a
    /// respawn. A log-open failure is NON-fatal — the daemon still spawns with
    /// stderr→null (better a diagnostic-less daemon than no daemon).
    ///
    /// `connect`/`listen` are the verb's `--connect`/`--listen` zenoh locators
    /// (the remote arm): when non-empty they are comma-joined into the daemon's
    /// [`VIZD_CONNECT_ENV`]/[`VIZD_LISTEN_ENV`] on the CHILD's environment, so the
    /// spawned daemon folds them into its network config at boot. They apply ONLY
    /// to a daemon this call starts — a daemon already running is never respawned,
    /// so its locators are unchanged (the verb says so).
    pub fn spawn_vizd_detached(
        binary: &Path,
        log_path: &Path,
        connect: &[String],
        listen: &[String],
    ) -> io::Result<()> {
        use std::os::unix::process::CommandExt;
        let stderr = match std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(log_path)
        {
            Ok(f) => std::process::Stdio::from(f),
            Err(e) => {
                tracing::warn!(
                    log = %log_path.display(),
                    error = %e,
                    "cerulion viz: could not open the cerulion-vizd log file — spawning the daemon with stderr discarded"
                );
                std::process::Stdio::null()
            }
        };
        let mut cmd = std::process::Command::new(binary);
        cmd.stdin(std::process::Stdio::null())
            .stdout(std::process::Stdio::null())
            .stderr(stderr);
        // Thread the locators onto the spawned daemon's environment (comma-joined;
        // the daemon splits on commas/whitespace). Only set when present, so a
        // scouting-only run leaves the env untouched.
        if !connect.is_empty() {
            cmd.env(VIZD_CONNECT_ENV, connect.join(","));
        }
        if !listen.is_empty() {
            cmd.env(VIZD_LISTEN_ENV, listen.join(","));
        }
        // SAFETY: `setsid(2)` is async-signal-safe (the only `pre_exec` requirement
        // between fork and exec); the forked child is never a process-group leader,
        // so `setsid` always succeeds.
        unsafe {
            cmd.pre_exec(|| {
                if libc::setsid() == -1 {
                    return Err(io::Error::last_os_error());
                }
                Ok(())
            });
        }
        cmd.spawn()?;
        Ok(())
    }

    /// The daemon's stderr log path — colocated with the control socket + pidfile
    /// (`vizd.log` next to `vizd.sock`). Used by the spawn ([`spawn_vizd_detached`])
    /// and surfaced in the unreachable-daemon error so a failed bring-up is
    /// recoverable.
    pub fn vizd_log_path(socket: &Path) -> PathBuf {
        socket.with_extension("log")
    }

    /// Ensure a daemon is reachable at `socket`: try connecting; if that fails,
    /// call `spawn` (which launches `cerulion-vizd` detached) and RETRY the
    /// connect within `bound`. Returns `(connected VizdConn, spawned)` — `spawned`
    /// is `true` iff THIS call started the daemon (`false` = an already-running
    /// shared daemon was reused). The verb uses `spawned` to say that
    /// `--connect`/`--listen` locators apply only to a daemon it starts. Idempotent
    /// for a concurrent controller — if the daemon is already up, `spawn` is never
    /// called (the daemon's own `flock` also refuses a redundant second daemon, so
    /// a lost spawn race is harmless).
    pub fn ensure_daemon(
        socket: &Path,
        bound: Duration,
        spawn: impl FnOnce() -> io::Result<()>,
    ) -> io::Result<(VizdConn, bool)> {
        // Fast path: the daemon is already running (shared, long-lived) — reused,
        // not spawned by this call.
        if let Ok(conn) = VizdConn::connect(socket) {
            return Ok((conn, false));
        }
        // Absent → spawn it detached, then retry the connect until it binds.
        spawn()?;
        let deadline = Instant::now() + bound;
        loop {
            match VizdConn::connect(socket) {
                Ok(conn) => return Ok((conn, true)),
                Err(e) if Instant::now() >= deadline => {
                    return Err(io::Error::new(
                        io::ErrorKind::TimedOut,
                        format!(
                            "cerulion-vizd did not become reachable at {} within {bound:?}: {e}",
                            socket.display()
                        ),
                    ))
                }
                Err(_) => std::thread::sleep(Duration::from_millis(50)),
            }
        }
    }
}

#[cfg(test)]
mod vizd_resolution_tests {
    use super::vizd_sibling_candidates;
    use std::path::{Path, PathBuf};

    /// A `cerulion` reached through a SYMLINK must still find the daemon beside
    /// its real self.
    ///
    /// `std::env::current_exe()` is not canonicalised on macOS (measured), so
    /// without the second rung the CLI looks only in the symlink's directory,
    /// skips the release build sitting beside its actual binary, and falls
    /// through to whatever `$PATH` holds — which presents as a WRONG DAEMON
    /// VERSION rather than as a missing file, and is why the masked form of this
    /// bug is worth closing rather than tolerating.
    ///
    /// Dropping `resolved` from the rung list makes this fail
    /// with only the symlink's directory offered.
    #[test]
    fn the_resolved_exes_directory_is_a_rung_so_a_symlinked_cli_finds_the_daemon() {
        let via_symlink = Path::new("/w/somewhere/bin/cerulion");
        let real = Path::new("/w/cerulion/target/release/cerulion");
        assert_eq!(
            vizd_sibling_candidates(Some(via_symlink), Some(real)),
            vec![
                PathBuf::from("/w/somewhere/bin/cerulion-vizd"),
                PathBuf::from("/w/cerulion/target/release/cerulion-vizd"),
            ],
            "the symlink's dir is tried FIRST, then the real binary's"
        );
    }

    /// THE SEAM PIN — the shell really canonicalises, over a REAL symlink.
    ///
    /// The oracles here INJECT the resolved path, so they pin the rung list and
    /// say nothing about whether production asks the OS for it. Same gap, same
    /// class, and same measured consequence as the netd ladder's
    /// (`cerulion_netd::client`): the rungs are right and the seam is unwired.
    ///
    /// Dropping the canonicalize from `find_vizd_binary_for`
    /// fails exactly this test.
    #[test]
    fn the_shell_really_canonicalises_so_a_symlinked_cli_finds_the_daemon() {
        use std::os::unix::fs::PermissionsExt as _;

        let root = std::env::temp_dir().join(format!(
            "vizdseam-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .expect("a clock after 1970")
                .as_nanos()
        ));
        let real_dir = root.join("real");
        let link_dir = root.join("link");
        std::fs::create_dir_all(&real_dir).expect("mkdir real");
        std::fs::create_dir_all(&link_dir).expect("mkdir link");

        let real_cli = real_dir.join("cerulion");
        let real_vizd = real_dir.join(super::VIZD_BINARY);
        for p in [&real_cli, &real_vizd] {
            std::fs::write(p, b"#!/bin/sh\n").expect("write a stand-in binary");
            std::fs::set_permissions(p, std::fs::Permissions::from_mode(0o755)).expect("chmod");
        }
        let linked_cli = link_dir.join("cerulion");
        std::os::unix::fs::symlink(&real_cli, &linked_cli).expect("symlink");
        assert!(
            !link_dir.join(super::VIZD_BINARY).exists(),
            "precondition: the symlink's directory holds no daemon"
        );

        let found = super::find_vizd_binary_for(Some(&linked_cli))
            .expect("the daemon beside the RESOLVED CLI must be found");
        // Canonicalised on both sides — macOS's `/var` is a symlink to
        // `/private/var`, so the resolved answer differs from the literal path by
        // that prefix. Without canonicalisation this line is unreachable: the sibling
        // rung finds nothing and the lookup falls through to `$PATH`, which in a
        // developer's environment may hold a real daemon — hence the explicit
        // "is under our scratch root" assertion below.
        let expected = std::fs::canonicalize(&real_vizd).expect("canonicalise expected");
        assert_eq!(found, expected);
        assert!(
            found.starts_with(std::fs::canonicalize(&root).expect("canonicalise root")),
            "the answer must come from our scratch tree, not from a daemon that \
             happens to be on this developer's PATH: {}",
            found.display()
        );

        let _ = std::fs::remove_dir_all(&root);
    }

    /// An already-resolved exe contributes no duplicate rung (the Linux shape,
    /// and any un-symlinked install).
    #[test]
    fn an_already_resolved_exe_contributes_no_duplicate_rung() {
        let exe = Path::new("/opt/cer/bin/cerulion");
        assert_eq!(
            vizd_sibling_candidates(Some(exe), Some(exe)),
            vec![PathBuf::from("/opt/cer/bin/cerulion-vizd")]
        );
        // A canonicalize FAILURE is the absence of a rung, not a refusal to look.
        assert_eq!(
            vizd_sibling_candidates(Some(exe), None),
            vec![PathBuf::from("/opt/cer/bin/cerulion-vizd")]
        );
        // Nothing to go on at all yields nothing — `find_vizd_binary` then falls
        // through to its `$PATH` rung rather than erroring.
        assert!(vizd_sibling_candidates(None, None).is_empty());
        // A parentless path likewise contributes nothing.
        assert!(vizd_sibling_candidates(Some(Path::new("/")), None).is_empty());
    }
}

#[cfg(test)]
mod schema_push_tests {
    use super::SchemaPushOutcome;
    use serde_json::json;

    /// A REFUSED side-load must never read as "already had everything".
    ///
    /// The daemon answers `ok: false` for a request it could not act on —
    /// including a daemon too OLD to know the verb, which answers a structured
    /// error rather than dropping the connection. A caller reading only
    /// `accepted` sees `0` there, indistinguishable from success-with-nothing-new,
    /// and the bag player would retry in silence forever while the scene renders
    /// nothing. That is the exact success-shaped failure the schema offer exists to remove.
    #[test]
    fn a_refusal_is_never_read_as_nothing_new() {
        // An older daemon's unknown-method error.
        let old =
            json!({"id": 1, "ok": false, "error": "malformed request: unknown variant `schemas`"});
        assert_eq!(
            SchemaPushOutcome::from_reply(&old),
            SchemaPushOutcome::Refused {
                reason: "malformed request: unknown variant `schemas`".into()
            }
        );
        // A refusal with no stated reason still classifies as a refusal, with a
        // stand-in — never as a silent zero.
        let bare = json!({"ok": false});
        assert!(matches!(
            SchemaPushOutcome::from_reply(&bare),
            SchemaPushOutcome::Refused { .. }
        ));
        // A reply with NO `ok` at all is not a success either (fails closed).
        assert!(matches!(
            SchemaPushOutcome::from_reply(&json!({"accepted": 3})),
            SchemaPushOutcome::Refused { .. }
        ));

        // ANTI-TAUTOLOGY: a genuine success is Applied, and a genuine
        // nothing-new success is Applied with 0 — distinct from a refusal.
        assert_eq!(
            SchemaPushOutcome::from_reply(&json!({"ok": true, "accepted": 2, "known": 5})),
            SchemaPushOutcome::Applied {
                accepted: 2,
                rejected: vec![]
            }
        );
        assert_eq!(
            SchemaPushOutcome::from_reply(&json!({"ok": true, "accepted": 0, "known": 5})),
            SchemaPushOutcome::Applied {
                accepted: 0,
                rejected: vec![]
            }
        );
    }

    /// Types the daemon could not use are carried back, so the player can say
    /// those topics render nothing instead of reporting a plain success.
    #[test]
    fn rejected_types_survive_the_round_trip() {
        let v = json!({
            "ok": true,
            "accepted": 1,
            "known": 4,
            "rejected": ["ws/YamlType", "sensor_msgs/Image"],
        });
        assert_eq!(
            SchemaPushOutcome::from_reply(&v),
            SchemaPushOutcome::Applied {
                accepted: 1,
                rejected: vec!["ws/YamlType".into(), "sensor_msgs/Image".into()],
            }
        );
        // An older daemon omits the field entirely — absent means "none named",
        // which is accurate: it is a v1 reply, not a claim that nothing was refused.
        assert_eq!(
            SchemaPushOutcome::from_reply(&json!({"ok": true, "accepted": 1})),
            SchemaPushOutcome::Applied {
                accepted: 1,
                rejected: vec![]
            }
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The reply deadline must CLEAR `cerulion-netd`'s own cold-start grace,
    /// or a cold first answer could never arrive at all.
    ///
    /// The behavioural arms in `tests/viz_attach_convergence_test.rs` pin the deadline
    /// only from ABOVE (a reply past it errs) — shrinking `CONN_IO_TIMEOUT` to
    /// 1 s passes every one of them. This is the half that IS in code: netd spends up to
    /// `COLD_START_DISCOVERY_BUDGET` re-harvesting before it answers a cold query
    /// (`cerulion_netd::query`), and vizd's handler waits for that answer, so a deadline
    /// at or below the budget turns every cold attach into an errno.
    ///
    /// Only the budget is `pub` in netd — the harvest and gather windows that make up
    /// the rest of its ~3.25 s worst case are `pub(crate)` — so the margin is asserted
    /// against the term we can name, generously enough that the real worst case still
    /// fits inside it.
    #[test]
    fn the_reply_deadline_clears_netds_own_cold_start_grace() {
        let budget = cerulion_netd::COLD_START_DISCOVERY_BUDGET;
        assert!(
            unix_client::conn_io_timeout() > budget,
            "the reply deadline ({:?}) must exceed netd's cold-start grace ({budget:?}), \
             or a cold attach can never be answered",
            unix_client::conn_io_timeout()
        );
        // netd's documented worst-case FIRST answer is the budget plus two harvest
        // windows and one gather window (~3.25 s with the default constants). Those three are
        // `pub(crate)`, so pin the margin at 2x the budget — satisfied by the default
        // 5 s, and violated by any shrink that would cut into the real worst case.
        assert!(
            unix_client::conn_io_timeout() >= budget * 2,
            "the deadline needs headroom over the budget for netd's harvest + gather \
             windows too: deadline={:?} budget={budget:?}",
            unix_client::conn_io_timeout()
        );
    }

    /// **The cancellation slice is bounded on BOTH sides, as a CONSTANT.**
    ///
    /// # Why this is a unit pin and not a wall in the e2e arm
    ///
    /// The regression it guards — `CANCEL_CHECK_SLICE` growing from 100 ms to
    /// something a user feels — is a CONSTANT EDIT, and a constant compared
    /// against a constant has no phase, no clock and no load term: it fails on
    /// every run, on every machine. This is the existing precedent
    /// (`cerulion_netd::client` pins its ceiling `>= 5 s` AND `<= 30 s`, its
    /// grace `< ceiling / 4`) applied to the one value a wall measures badly.
    ///
    /// The e2e arm could carry that job as a `return − cancellation` wall, and
    /// the reason it does not is a MARGIN, not an inability —
    /// measured rather than reasoned about:
    ///
    /// * It is NOT the coin toss it looks like. The reading is not uniform over
    ///   `(0, S]`: that arm's watcher flips at a FIXED 300 ms while the loop's
    ///   first sleep starts within a millisecond of the first round trip, so the
    ///   reading is `S − 300 ms` deterministically. MEASURED under a 2 s slice:
    ///   the wall fails **12 of 12** runs, all at ~1.72 s.
    ///
    /// * What is thin is the margin on that side (1.72 s against a 1.5 s bound is
    ///   1.15x), and it is thin in a direction that is measurable on macOS:
    ///   background QoS charges timer slack per WAKEUP, a 150 ms
    ///   nominal sleep coming back in 1100-1696 ms, which lands a HEALTHY 100 ms
    ///   slice inside that failing band. That is a
    ///   documented mechanism rather than a failure seen under `taskpolicy -b`
    ///   plus concurrent build load (20 of 20 green) — but a bound whose
    ///   two sides are 1.15x apart is not the instrument for a value that is
    ///   simply a constant somebody can read.
    ///
    /// Both sides are pinned because both directions are real faults: too COARSE
    /// and Ctrl-C on `cerulion viz` stops feeling instant; too FINE and the loop
    /// becomes a busy-wait on a wait that is normally seconds long.
    #[test]
    fn the_cancellation_slice_stays_inside_its_ux_bounds() {
        // Ctrl-C must stay INSTANT. The module docs put the interactive-instant
        // threshold at ~150 ms; 250 ms is the ceiling with a little room above
        // it, and the default 100 ms sits comfortably inside.
        const UX_CEILING: Duration = Duration::from_millis(250);
        // …and it must stay a SLEEP, not a spin. A slice this small would wake the
        // loop ~1000x per second for a wait whose whole point is to be idle.
        const SPIN_FLOOR: Duration = Duration::from_millis(10);
        let slice = unix_client::cancel_check_slice();
        assert!(
            slice <= UX_CEILING,
            "the cancellation slice ({slice:?}) is the granularity at which `cerulion viz` \
             notices Ctrl-C. Above {UX_CEILING:?} the verb stops feeling instant to a user \
             holding the key down. This is the pin that catches it: the e2e wall that used \
             to sat 1.15x away from a 2 s slice, which is not a margin to gate a constant on"
        );
        assert!(
            slice >= SPIN_FLOOR,
            "the cancellation slice ({slice:?}) must stay a SLEEP: below {SPIN_FLOOR:?} the \
             wait loop wakes hundreds of times a second for a poll interval measured in \
             seconds"
        );
    }

    /// RAII: restore `VIZD_SOCKET_ENV` to its pre-test value on drop, so a panicking
    /// test never leaks the override into a sibling.
    struct SocketEnvGuard(Option<std::ffi::OsString>);
    impl SocketEnvGuard {
        fn set(value: &str) -> Self {
            let prev = std::env::var_os(VIZD_SOCKET_ENV);
            std::env::set_var(VIZD_SOCKET_ENV, value);
            SocketEnvGuard(prev)
        }
        fn clear(&self) {
            std::env::remove_var(VIZD_SOCKET_ENV);
        }
    }
    impl Drop for SocketEnvGuard {
        fn drop(&mut self) {
            match &self.0 {
                Some(v) => std::env::set_var(VIZD_SOCKET_ENV, v),
                None => std::env::remove_var(VIZD_SOCKET_ENV),
            }
        }
    }

    #[test]
    fn socket_env_override_wins_and_default_ladder() {
        // This test both writes `VIZD_SOCKET_ENV` and READS `HOME`
        // (through `vizd_socket_path`'s ladder) — and held NO lock, so it raced the
        // `HOME`-rewriting tests in `connect_cmd` / `account_cmd`. Take the ONE
        // crate-wide env mutex.
        let _env_lk = crate::test_env::env_lock();

        // The env override wins verbatim (mirrors the daemon's SOCKET_ENV).
        let guard = SocketEnvGuard::set("/tmp/cer_viz_client_override.sock");
        assert_eq!(
            vizd_socket_path(),
            PathBuf::from("/tmp/cer_viz_client_override.sock")
        );
        guard.clear();

        // With no override, the filename is vizd.sock under a `cerulion` /
        // `.cerulion` dir (XDG uses `cerulion`, the HOME fallback `.cerulion`) —
        // the daemon default shape, never empty.
        let p = vizd_socket_path();
        assert_eq!(
            p.file_name().and_then(|n| n.to_str()),
            Some("vizd.sock"),
            "the socket filename is vizd.sock, got {}",
            p.display()
        );
        let parent = p
            .parent()
            .and_then(|d| d.file_name())
            .and_then(|n| n.to_str())
            .unwrap_or("");
        assert!(
            parent == "cerulion" || parent == ".cerulion",
            "the socket lives under a cerulion dir, got parent {parent} in {}",
            p.display()
        );
    }

    #[test]
    fn attach_line_local_omits_optional_fields() {
        // A LOCAL attach: no entity/robot/schema → the minimal line the daemon's
        // `parse_attach_minimal` oracle expects.
        assert_eq!(
            attach_line(2, "/utlidar/cloud", None, None, None),
            r#"{"id":2,"method":"attach","topic":"/utlidar/cloud"}"#
        );
    }

    #[test]
    fn attach_line_remote_carries_robot_and_schema() {
        // A REMOTE attach: robot + its required schema, in the daemon's field
        // order (serde_json::Map preserves insertion order under preserve_order,
        // which is the workspace default — matched to the daemon's own oracle).
        assert_eq!(
            attach_line(
                9,
                "/x",
                Some("world/cam"),
                Some("go2"),
                Some("geometry_msgs/Vector3")
            ),
            r#"{"id":9,"method":"attach","topic":"/x","entity":"world/cam","robot":"go2","schema":"geometry_msgs/Vector3"}"#
        );
    }

    #[test]
    fn simple_method_lines_match_the_protocol() {
        assert_eq!(discover_line(1), r#"{"id":1,"method":"discover"}"#);
        assert_eq!(list_line(4), r#"{"id":4,"method":"list"}"#);
        assert_eq!(status_line(5), r#"{"id":5,"method":"status"}"#);
        assert_eq!(
            detach_line(3, "/tf"),
            r#"{"id":3,"method":"detach","topic":"/tf"}"#
        );
    }

    #[cfg(unix)]
    #[test]
    fn parse_attach_reply_reads_ok_and_error_arms() {
        // Success arm.
        let ok = serde_json::json!({
            "id": 2, "ok": true, "topic": "/vel",
            "schema": "geometry_msgs/Vector3", "archetype": "Scalars",
            "entity": "world/vel", "route": "vel", "already_attached": false
        });
        let r = parse_attach_reply(&ok);
        assert!(r.ok);
        assert_eq!(r.schema.as_deref(), Some("geometry_msgs/Vector3"));
        assert_eq!(r.archetype.as_deref(), Some("Scalars"));
        assert_eq!(r.entity.as_deref(), Some("world/vel"));
        assert!(r.error.is_none());
        // A first attach reports already_attached:false → THIS run created the tap.
        assert!(
            !r.already_attached,
            "a fresh attach is not already_attached"
        );

        // A REPEAT attach (the daemon taps are GLOBAL) → already_attached:true, so
        // the verb must NOT detach this topic on Ctrl+C (it did not create it).
        let repeat = serde_json::json!({
            "id": 3, "ok": true, "topic": "/vel",
            "schema": "geometry_msgs/Vector3", "archetype": "Scalars",
            "entity": "world/vel", "route": "vel", "already_attached": true
        });
        assert!(
            parse_attach_reply(&repeat).already_attached,
            "a repeat attach threads already_attached:true through the reply"
        );

        // Error arm — the actionable message surfaces VERBATIM; already_attached
        // defaults false (the field is absent on an error reply).
        let err = serde_json::json!({
            "id": 2, "ok": false, "topic": "/x",
            "error": "topic '/x' does not exist"
        });
        let r = parse_attach_reply(&err);
        assert!(!r.ok);
        assert_eq!(r.error.as_deref(), Some("topic '/x' does not exist"));
        assert!(r.schema.is_none());
        assert!(
            !r.already_attached,
            "an absent already_attached field defaults to false"
        );
    }

    #[cfg(unix)]
    #[test]
    fn connect_times_out_against_an_accept_but_never_write_daemon() {
        // A wedged daemon (or a foreign process) that ACCEPTS the socket
        // but never writes the banner must NOT hang the verb forever. `connect`
        // arms a read timeout, so the blocking banner read returns an actionable
        // Err within a bound instead of blocking un-interruptibly.
        use std::os::unix::net::UnixListener;
        use std::sync::atomic::{AtomicBool, Ordering};
        use std::sync::{mpsc, Arc};
        use std::time::Instant;

        let dir = std::env::temp_dir().join(format!("cer_vizc_wedge_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let socket = dir.join("wedged.sock");

        // A wedged "daemon": accept the connection, then hold it open WITHOUT ever
        // writing the banner, until the test signals stop. A `stop` flag (polled in
        // short slices) + a hard ceiling mean this helper never sleeps
        // un-interruptibly, so the join below is bounded — a wedge surfaces as a
        // bounded error, never a hang.
        let stop = Arc::new(AtomicBool::new(false));
        let sock_l = socket.clone();
        let stop_l = Arc::clone(&stop);
        let listener = std::thread::spawn(move || {
            let l = UnixListener::bind(&sock_l).expect("bind wedged daemon");
            if let Ok((s, _)) = l.accept() {
                // Linger long enough to outlast connect's CONN_IO_TIMEOUT (5s) so
                // its blocking banner read hits the timeout, but cap the linger so a
                // leaked thread never sleeps forever (even if the test panics before
                // signaling stop).
                let ceiling = Instant::now() + Duration::from_secs(15);
                while !stop_l.load(Ordering::Relaxed) && Instant::now() < ceiling {
                    std::thread::sleep(Duration::from_millis(50));
                }
                drop(s);
            }
        });
        for _ in 0..50 {
            if socket.exists() {
                break;
            }
            std::thread::sleep(Duration::from_millis(10));
        }

        // `connect` must return within ~CONN_IO_TIMEOUT (5s), NOT hang. Run it on a
        // worker thread guarded by a channel so a REGRESSION (no read timeout →
        // unbounded blocking read) FAILS cleanly instead of hanging the suite.
        let sock_c = socket.clone();
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let t0 = Instant::now();
            let result = VizdConn::connect(&sock_c);
            let _ = tx.send((result.is_err(), t0.elapsed()));
        });
        match rx.recv_timeout(Duration::from_secs(12)) {
            Ok((is_err, elapsed)) => {
                assert!(
                    is_err,
                    "a never-writing daemon must make connect Err, not hang"
                );
                assert!(
                    elapsed < Duration::from_secs(10),
                    "connect timed out within the bound ({elapsed:?}), not hung"
                );
            }
            Err(_) => panic!(
                "VizdConn::connect did not return within 12s against an accept-but-never-write \
                 daemon — the read timeout regressed (an un-interruptible hang)"
            ),
        }
        // Signal the wedged helper to stop, then join with a BOUND: `is_finished`
        // gates the join so a genuinely wedged helper thread never hangs the suite
        // (the previous unbounded `join()` blocked on the helper's fixed 20s sleep).
        stop.store(true, Ordering::Relaxed);
        let join_deadline = Instant::now() + Duration::from_secs(3);
        while !listener.is_finished() && Instant::now() < join_deadline {
            std::thread::sleep(Duration::from_millis(20));
        }
        if listener.is_finished() {
            let _ = listener.join();
        }
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[cfg(unix)]
    #[test]
    fn spawn_vizd_detached_captures_daemon_stderr_to_the_log_file() {
        // The spawned daemon's stderr must land in a RECOVERABLE log
        // file (next to the socket), NOT /dev/null — otherwise every daemon
        // diagnostic (startup failure, the host TOCTOU warn) vanishes.
        use std::io::Write as _;
        use std::os::unix::fs::PermissionsExt;
        use std::time::Instant;

        let dir = std::env::temp_dir().join(format!("cer_vizc_log_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        // A fake "daemon": a tiny script that writes a marker to STDERR and exits.
        let script = dir.join("fake-vizd.sh");
        let mut f = std::fs::File::create(&script).unwrap();
        writeln!(f, "#!/bin/sh\necho STDERR_MARKER 1>&2\n").unwrap();
        drop(f);
        std::fs::set_permissions(&script, std::fs::Permissions::from_mode(0o755)).unwrap();

        let socket = dir.join("vizd.sock");
        let log = vizd_log_path(&socket);
        assert_eq!(log, dir.join("vizd.log"), "log colocated with the socket");
        spawn_vizd_detached(&script, &log, &[], &[]).expect("spawn the fake daemon");

        // The marker appears in the log within a bound (the child writes + exits).
        // A regression to stderr→/dev/null leaves the file empty → this fails.
        let deadline = Instant::now() + Duration::from_secs(5);
        let captured = loop {
            if let Ok(contents) = std::fs::read_to_string(&log) {
                if contents.contains("STDERR_MARKER") {
                    break true;
                }
            }
            if Instant::now() >= deadline {
                break false;
            }
            std::thread::sleep(Duration::from_millis(25));
        };
        assert!(
            captured,
            "the spawned daemon's stderr must be captured to {} (not /dev/null)",
            log.display()
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[cfg(unix)]
    #[test]
    fn spawn_vizd_detached_threads_connect_and_listen_locators_into_the_child_env() {
        // The remote arm: the verb's --connect/--listen locators must reach the
        // SPAWNED daemon via its env (comma-joined). A fake daemon dumps the two
        // env vars to a file; we assert both carry the joined locators — and that
        // an EMPTY locator set leaves the env unset (a scouting-only run).
        use std::io::Write as _;
        use std::os::unix::fs::PermissionsExt;
        use std::time::Instant;

        let dir = std::env::temp_dir().join(format!("cer_vizc_env_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let out = dir.join("env.out");
        let script = dir.join("fake-vizd-env.sh");
        let mut f = std::fs::File::create(&script).unwrap();
        // Print `connect=<val>` / `listen=<val>` (empty when unset) to the out file.
        writeln!(
            f,
            "#!/bin/sh\nprintf 'connect=%s\\nlisten=%s\\n' \
             \"$CERULION_VIZD_CONNECT\" \"$CERULION_VIZD_LISTEN\" > {}\n",
            out.display()
        )
        .unwrap();
        drop(f);
        std::fs::set_permissions(&script, std::fs::Permissions::from_mode(0o755)).unwrap();

        let log = dir.join("vizd.log");
        spawn_vizd_detached(
            &script,
            &log,
            &[
                "tcp/192.168.1.5:7683".to_string(),
                "tcp/10.0.0.9:7447".to_string(),
            ],
            &["tcp/0.0.0.0:7447".to_string()],
        )
        .expect("spawn the fake env-dumping daemon");

        let deadline = Instant::now() + Duration::from_secs(5);
        let contents = loop {
            if let Ok(c) = std::fs::read_to_string(&out) {
                if c.contains("connect=") {
                    break c;
                }
            }
            if Instant::now() >= deadline {
                panic!("the fake daemon never wrote its env dump");
            }
            std::thread::sleep(Duration::from_millis(25));
        };
        assert!(
            contents.contains("connect=tcp/192.168.1.5:7683,tcp/10.0.0.9:7447"),
            "CERULION_VIZD_CONNECT carries the comma-joined --connect locators: {contents}"
        );
        assert!(
            contents.contains("listen=tcp/0.0.0.0:7447"),
            "CERULION_VIZD_LISTEN carries the --listen locator: {contents}"
        );

        // Empty locator sets leave the env unset (scouting-only run).
        let out2 = dir.join("env2.out");
        let script2 = dir.join("fake-vizd-env2.sh");
        let mut f2 = std::fs::File::create(&script2).unwrap();
        writeln!(
            f2,
            "#!/bin/sh\nprintf 'connect=[%s]\\nlisten=[%s]\\n' \
             \"$CERULION_VIZD_CONNECT\" \"$CERULION_VIZD_LISTEN\" > {}\n",
            out2.display()
        )
        .unwrap();
        drop(f2);
        std::fs::set_permissions(&script2, std::fs::Permissions::from_mode(0o755)).unwrap();
        spawn_vizd_detached(&script2, &log, &[], &[]).expect("spawn (no locators)");
        let deadline = Instant::now() + Duration::from_secs(5);
        let contents2 = loop {
            if let Ok(c) = std::fs::read_to_string(&out2) {
                if c.contains("connect=[") {
                    break c;
                }
            }
            if Instant::now() >= deadline {
                panic!("the fake daemon (no locators) never wrote its env dump");
            }
            std::thread::sleep(Duration::from_millis(25));
        };
        assert!(
            contents2.contains("connect=[]") && contents2.contains("listen=[]"),
            "an empty locator set leaves the daemon env unset: {contents2}"
        );

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[cfg(unix)]
    #[test]
    fn ensure_daemon_reuses_a_live_daemon_and_spawns_an_absent_one() {
        use std::io::Write;
        use std::os::unix::net::UnixListener;
        use std::sync::atomic::{AtomicUsize, Ordering};
        use std::sync::{Arc, Mutex};

        const BANNER: &str = r#"{"vizd":"cerulion-vizd","protocol":1,"rerun_url":"rerun+http://127.0.0.1:9876/proxy"}"#;

        // A fake daemon: bind the socket, accept ONE connection, send the Hello
        // banner, then linger briefly so the client can read it.
        fn fake_daemon(socket: PathBuf) -> std::thread::JoinHandle<()> {
            std::thread::spawn(move || {
                let l = UnixListener::bind(&socket).expect("bind fake daemon");
                if let Ok((mut s, _)) = l.accept() {
                    let _ = writeln!(s, "{BANNER}");
                    std::thread::sleep(Duration::from_millis(300));
                }
            })
        }

        let dir = std::env::temp_dir().join(format!("cer_vizc_ensure_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();

        // (A) already up → spawn NOT called, the banner is read.
        let sock_a = dir.join("up.sock");
        let ha = fake_daemon(sock_a.clone());
        // Let the listener bind.
        for _ in 0..50 {
            if sock_a.exists() {
                break;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        let spawn_calls = Arc::new(AtomicUsize::new(0));
        let sc = Arc::clone(&spawn_calls);
        let (conn, spawned) = ensure_daemon(&sock_a, Duration::from_secs(2), || {
            sc.fetch_add(1, Ordering::SeqCst);
            Ok(())
        })
        .expect("connect to the live daemon");
        assert!(
            !spawned,
            "a LIVE daemon is REUSED, not spawned by this call"
        );
        assert_eq!(conn.banner().protocol, 1);
        assert_eq!(
            conn.banner().rerun_url.as_deref(),
            Some("rerun+http://127.0.0.1:9876/proxy"),
            "the advertised rerun_url is read from the banner"
        );
        assert_eq!(
            spawn_calls.load(Ordering::SeqCst),
            0,
            "a LIVE daemon is reused — spawn is NEVER called"
        );
        drop(conn);
        let _ = ha.join();

        // (B) absent → spawn IS called (it starts the fake daemon), then connects.
        let sock_b = dir.join("absent.sock");
        let spawn_calls = Arc::new(AtomicUsize::new(0));
        let sc = Arc::clone(&spawn_calls);
        let started: Arc<Mutex<Option<std::thread::JoinHandle<()>>>> = Arc::new(Mutex::new(None));
        let sc_started = Arc::clone(&started);
        let sock_b2 = sock_b.clone();
        let (conn, spawned) = ensure_daemon(&sock_b, Duration::from_secs(3), move || {
            sc.fetch_add(1, Ordering::SeqCst);
            *sc_started.lock().unwrap() = Some(fake_daemon(sock_b2.clone()));
            Ok(())
        })
        .expect("spawn + connect to the absent daemon");
        assert!(spawned, "an ABSENT daemon is SPAWNED by this call");
        assert_eq!(conn.banner().protocol, 1);
        assert_eq!(
            spawn_calls.load(Ordering::SeqCst),
            1,
            "an ABSENT daemon is spawned EXACTLY once, then connected"
        );
        drop(conn);
        if let Some(h) = started.lock().unwrap().take() {
            let _ = h.join();
        }
        let _ = std::fs::remove_dir_all(&dir);
    }
}
