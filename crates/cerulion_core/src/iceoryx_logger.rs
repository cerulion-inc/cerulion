// SPDX-License-Identifier: AGPL-3.0-only
//! Bridge iceoryx2's internal logger to the `tracing` crate.
//!
//! iceoryx2 emits diagnostic info (cleanup failures,
//! lock contention, registry inconsistencies, etc.) via its own
//! `iceoryx2_log::trace!` / `warn!` / etc. macros. By default
//! these are printed to stderr by iceoryx2's built-in
//! `console::Logger`, which is opaque to programmatic capture.
//! Installing this bridge as iceoryx2's logger (via
//! `iceoryx2::prelude::set_logger`) routes every emission through
//! `tracing::trace!`/etc. instead, so:
//!
//! 1. iceoryx2 events become subject to standard `RUST_LOG`
//!    filtering and Cerulion's tracing-subscriber pipeline.
//! 2. Callers (e.g. `cerulion_cli_engine::ipc_cleanup`) can
//!    install a thread-local capture via [`capture_iceoryx_logs`]
//!    to collect emissions during a specific operation —
//!    enabling structured per-failure diagnostics in
//!    `cerulion clean` without the user setting env vars.
//!
//! `iceoryx2_log::set_logger` can only be called once per process.
//! Wrap the call in [`install_iceoryx2_tracing_bridge`] which
//! returns `false` if the bridge was already installed.

use core::cell::RefCell;

use iceoryx2_log::{Log, LogLevel};

/// Cerulion's default iceoryx2 log level.
///
/// `Error` — iceoryx2's own `warn!`s are operational chatter on a healthy
/// system (stale listener notifications, "no config file was loaded", failed
/// notifies to a saturated listener), and one of them
/// (`iceoryx2-0.9.1/src/port/notifier.rs:525`, `FailedToDeliverSignal`) is
/// emitted once per publish per stuck listener connection — a measured
/// ~2500 lines/s ≈ 5 MB/s in production. `error!` and `fatal!` still surface.
///
/// This is the SAME value the generated workspace's `.cargo/config.toml`
/// documents (`IOX2_LOG_LEVEL = "error"`), the value
/// `TransportManager::init*` has always passed, and the value the shared
/// [`init_iceoryx_log_level`] entry point applies when `IOX2_LOG_LEVEL` is
/// absent — one constant, no drift.
pub const DEFAULT_IOX2_LOG_LEVEL: LogLevel = LogLevel::Error;

/// The environment variable iceoryx2 documents for its own log filter, and the
/// one Cerulion's generated workspaces set. Named here so every entry point
/// reads the SAME key (`TransportManager::init*`, the binaries, and the
/// macro-generated cdylib `init`).
pub const IOX2_LOG_LEVEL_ENV: &str = "IOX2_LOG_LEVEL";

/// Gates the unparseable-`IOX2_LOG_LEVEL` complaint to ONCE per linked copy —
/// see [`init_iceoryx_log_level`]'s "Loudness" section.
static BAD_LEVEL_WARNED: std::sync::Once = std::sync::Once::new();

/// Parse an `IOX2_LOG_LEVEL` value, case-insensitively.
///
/// Returns `None` for an empty or unrecognized value — callers fall back to
/// [`DEFAULT_IOX2_LOG_LEVEL`] **loudly** (see [`init_iceoryx_log_level`]); a
/// silently-ignored typo is exactly the class of bug this exists to catch.
///
/// Accepts the six names iceoryx2 itself accepts, in any case (`fatal`,
/// `error`, `warn`, `info`, `debug`, `trace`) — the live robot evidence used
/// `FATAL`, so case-insensitivity is part of the contract, not an accident.
#[must_use]
pub fn parse_iox2_log_level(value: &str) -> Option<LogLevel> {
    // `eq_ignore_ascii_case` avoids the `to_lowercase()` allocation, so this is
    // callable from a cdylib `init` without touching the allocator.
    const LEVELS: [(&str, LogLevel); 6] = [
        ("trace", LogLevel::Trace),
        ("debug", LogLevel::Debug),
        ("info", LogLevel::Info),
        ("warn", LogLevel::Warn),
        ("error", LogLevel::Error),
        ("fatal", LogLevel::Fatal),
    ];
    let trimmed = value.trim();
    LEVELS
        .iter()
        .find(|(name, _)| trimmed.eq_ignore_ascii_case(name))
        .map(|(_, level)| *level)
}

/// Apply `IOX2_LOG_LEVEL` (or [`DEFAULT_IOX2_LOG_LEVEL`]) to **this linked
/// copy** of `iceoryx2-log`.
///
/// # Why this exists
///
/// iceoryx2's log level is a `static AtomicU8` **inside the `iceoryx2-log`
/// crate** (`iceoryx2-log-0.9.1/src/lib.rs`'s `LOG_LEVEL`), read by
/// `__internal_print_log_msg` before every emission. A cdylib node statically
/// links its OWN copy of `cerulion_core` → `iceoryx2` → `iceoryx2-log`, so it
/// has its OWN `LOG_LEVEL` static (and its OWN `LOGGER`, which is why
/// [`install_cdylib_stderr_tracing`](crate::graph::node::install_cdylib_stderr_tracing)
/// exists for `tracing`). The host binary calling `set_log_level_from_env_or`
/// initializes only the HOST's static — the cdylib's stays at
/// `iceoryx2-log`'s crate default (`Info`), so a cdylib-emitted `warn!` prints
/// no matter what `IOX2_LOG_LEVEL` says. That is precisely how a documented
/// knob became inert: in production, both `IOX2_LOG_LEVEL=error` and
/// `IOX2_LOG_LEVEL=FATAL` still produced >100 MB of iceoryx2 warnings per 30 s,
/// because the publisher doing the notifying lived in the `dds_bridge` cdylib.
///
/// Call this at EVERY entry point that can reach iceoryx2 code in a given
/// linked copy: each binary's `main`, and the macro-generated
/// `cerulion_node_init` for cdylibs.
///
/// # Loudness
///
/// An unparseable value falls back to [`DEFAULT_IOX2_LOG_LEVEL`] and emits ONE
/// `eprintln!` naming the bad value and the effective level. `eprintln!` (not
/// `tracing`) is deliberate and matches
/// [`install_cdylib_stderr_tracing`](crate::graph::node::install_cdylib_stderr_tracing):
/// this runs before a subscriber is guaranteed to exist in this linked copy, so
/// a `tracing::warn!` would dispatch to a no-op and the typo would be silent.
///
/// ONE means once per linked copy, not once per call: this entry point is
/// deliberately called from several sites in a process (`main`, each
/// `TransportManager::init*`, the `topic list` reader node, …), and a typo'd
/// `IOX2_LOG_LEVEL` would otherwise print the same complaint 4-5 times per
/// invocation — the same repeat-noise class this module exists to remove. A
/// process-wide `Once` gates the message; the FALLBACK itself (and the
/// resulting level) is applied on every call regardless.
///
/// # Determinism
///
/// `spec` is passed IN rather than read from `std::env` here so a cdylib can
/// hand its frozen [`NodeContext`](crate::graph::node::NodeContext) env
/// snapshot (replay determinism — see
/// [`init_iceoryx_log_level_from_env`] for the live-env binary entry point).
/// `None` or an empty spec ⇒ the default.
// P12 exception, per this module's docs above: the complaint about an
// unparseable IOX2_LOG_LEVEL is emitted on the PRE-TRACING bootstrap path (this
// runs from a cdylib's `cerulion_node_init` before any subscriber exists in
// that linked copy), so stderr is the only channel it has. Function-scoped
// because a `#[allow]` on a macro STATEMENT does not reach the lint emitted
// inside the expansion.
#[allow(clippy::print_stderr)]
pub fn init_iceoryx_log_level(spec: Option<&str>) {
    let level = match spec.map(str::trim).filter(|s| !s.is_empty()) {
        None => DEFAULT_IOX2_LOG_LEVEL,
        Some(raw) => match parse_iox2_log_level(raw) {
            Some(level) => level,
            None => {
                // Once per linked copy: `init_iceoryx_log_level` is called from
                // several entry points in one process by design, and repeating
                // the same complaint per call is the repeat-noise class this module
                // exists to remove. The fallback below still applies on EVERY call.
                BAD_LEVEL_WARNED.call_once(|| {
                    eprintln!(
                        "cerulion: {IOX2_LOG_LEVEL_ENV}={raw:?} is not a valid iceoryx2 log level \
                         (expected one of: fatal, error, warn, info, debug, trace); using \
                         {DEFAULT_IOX2_LOG_LEVEL:?}"
                    );
                });
                DEFAULT_IOX2_LOG_LEVEL
            }
        },
    };
    // `set_log_level` is a plain relaxed `AtomicU8` store — last-writer-wins,
    // NOT init-once — so calling this from several entry points in one linked
    // copy is safe and idempotent for a fixed environment.
    iceoryx2::prelude::set_log_level(level);
}

/// The iceoryx2 log level currently in effect **in this linked copy**, as the
/// raw `u8` discriminant `iceoryx2-log` compares against
/// (`Trace = 0 … Fatal = 5`; a message at level `L` prints iff
/// `current_iox2_log_level() <= L as u8`).
///
/// Principle #3 observability for a value that is otherwise invisible AND
/// per-linked-copy: a cdylib node's level is a DIFFERENT static from its host's,
/// which is exactly how the inert-level defect hid. A cdylib can report its own effective level
/// through this accessor, letting a test prove the cdylib — not just the host —
/// was initialized.
#[must_use]
pub fn current_iox2_log_level() -> u8 {
    iceoryx2_log::get_log_level()
}

/// [`init_iceoryx_log_level`] reading the LIVE process environment.
///
/// The entry point for BINARIES (`cerulion`, `cerulion-netd`, `bagd`,
/// `cerulion-vizd`, …), which have no frozen env snapshot and want the level
/// applied as early as possible in `main` — before any iceoryx2 call can emit.
/// It is also what every in-process re-application (each
/// `TransportManager::init*`, `graph run` / `run-worker` / `run-gateway` /
/// `graph profile`, the `topic list` reader node) calls, so `IOX2_LOG_LEVEL`
/// WINS everywhere rather than being clobbered by a hardcoded level.
///
/// Cdylibs must use [`init_iceoryx_log_level`] with their `NodeContext`
/// snapshot instead (Principle #7).
pub fn init_iceoryx_log_level_from_env() {
    // Read once; an absent var and an empty var both mean "use the default".
    let raw = std::env::var(IOX2_LOG_LEVEL_ENV).ok();
    init_iceoryx_log_level(raw.as_deref());
}

/// Captured trace event from iceoryx2. Stored in the thread-local
/// capture buffer when [`capture_iceoryx_logs`] is active.
#[derive(Debug, Clone)]
pub struct CapturedLog {
    pub level: LogLevel,
    pub origin: String,
    pub message: String,
}

thread_local! {
    /// Per-thread capture buffer. `Some(buf)` between
    /// `capture_iceoryx_logs`'s setup/teardown; `None` otherwise.
    /// Using thread-local rather than a global Mutex avoids
    /// cross-test contention when tests run in parallel.
    static CAPTURE: RefCell<Option<Vec<CapturedLog>>> = const { RefCell::new(None) };
}

/// The bridge logger: implements iceoryx2's `Log` trait,
/// forwards to `tracing::*` macros, and tees to the thread-local
/// capture buffer when active.
pub struct IceoryxTracingBridge;

impl IceoryxTracingBridge {
    pub const fn new() -> Self {
        Self
    }
}

impl Default for IceoryxTracingBridge {
    fn default() -> Self {
        Self::new()
    }
}

impl Log for IceoryxTracingBridge {
    fn log(&self, level: LogLevel, origin: core::fmt::Arguments, message: core::fmt::Arguments) {
        // Always forward to tracing.
        match level {
            LogLevel::Trace => {
                tracing::trace!(target: "iceoryx2", iox_origin = %origin, "{}", message)
            }
            LogLevel::Debug => {
                tracing::debug!(target: "iceoryx2", iox_origin = %origin, "{}", message)
            }
            LogLevel::Info => {
                tracing::info!(target: "iceoryx2", iox_origin = %origin, "{}", message)
            }
            LogLevel::Warn => {
                tracing::warn!(target: "iceoryx2", iox_origin = %origin, "{}", message)
            }
            LogLevel::Error => {
                tracing::error!(target: "iceoryx2", iox_origin = %origin, "{}", message)
            }
            LogLevel::Fatal => {
                tracing::error!(target: "iceoryx2", iox_origin = %origin, "{}", message)
            }
        }
        // Tee to capture buffer if active on this thread.
        CAPTURE.with(|c| {
            if let Some(buf) = c.borrow_mut().as_mut() {
                buf.push(CapturedLog {
                    level,
                    origin: origin.to_string(),
                    message: message.to_string(),
                });
            }
        });
    }
}

/// Static instance — required by `iceoryx2_log::set_logger`'s
/// `&'static dyn Log` signature.
static BRIDGE: IceoryxTracingBridge = IceoryxTracingBridge::new();

/// Install the bridge as iceoryx2's logger. Returns `true` if
/// installed, `false` if iceoryx2 already has a logger
/// (set_logger is once-per-process). Idempotent on the bridge:
/// calling it twice from the same process is safe; the second
/// call just no-ops.
pub fn install_iceoryx2_tracing_bridge() -> bool {
    iceoryx2::prelude::set_logger(&BRIDGE)
}

/// Run `f` with a thread-local capture buffer collecting every
/// iceoryx2 log emission. Returns `(f's result, captured logs)`.
/// The buffer is taken at end-of-call so nested invocations on
/// the same thread don't leak across.
///
/// The bridge must be installed (via
/// [`install_iceoryx2_tracing_bridge`]) for the capture to see
/// anything; if iceoryx2 is using its default console logger,
/// emissions go to stderr and the buffer stays empty.
///
/// Note: iceoryx2's `LogLevel` is a process-global setting, so
/// the caller must also raise the level (e.g. via
/// `iceoryx2::prelude::set_log_level(LogLevel::Trace)`) for
/// `trace!`-level events to fire. The capture only stores events
/// that iceoryx2 actually emits; level filtering happens before
/// the bridge sees the event.
pub fn capture_iceoryx_logs<F: FnOnce() -> R, R>(f: F) -> (R, Vec<CapturedLog>) {
    /// RAII guard: clears the thread-local capture buffer on
    /// drop, even on panic unwind. Without this, a panic inside
    /// `f` would leave the buffer set to `Some(Vec)` for the
    /// thread's lifetime — and on a thread-pool worker (e.g.
    /// Tokio blocking pool, mandated by Principle #9),
    /// every subsequent iceoryx2 emission on that worker would
    /// silently accumulate into an orphaned buffer without bound.
    struct CaptureGuard;
    impl Drop for CaptureGuard {
        fn drop(&mut self) {
            CAPTURE.with(|c| *c.borrow_mut() = None);
        }
    }

    CAPTURE.with(|c| *c.borrow_mut() = Some(Vec::new()));
    let _guard = CaptureGuard;
    let result = f();
    // Read on the happy path before the guard drops; the guard
    // sets the slot to None on both happy + panic paths. Idempotent
    // re-clearing is fine.
    let captured = CAPTURE.with(|c| c.borrow_mut().take()).unwrap_or_default();
    (result, captured)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serial_test::serial;

    /// Oracle-vector pin for the `IOX2_LOG_LEVEL` parser. Hand-written
    /// expectations, never a self-compare. The `FATAL` row is the LIVE evidence
    /// value (the Go2 operator set `IOX2_LOG_LEVEL=FATAL` and it did nothing) —
    /// case-insensitivity is a contract, not an accident.
    #[test]
    fn parse_iox2_log_level_oracle() {
        let cases: &[(&str, Option<LogLevel>)] = &[
            ("trace", Some(LogLevel::Trace)),
            ("debug", Some(LogLevel::Debug)),
            ("info", Some(LogLevel::Info)),
            ("warn", Some(LogLevel::Warn)),
            ("error", Some(LogLevel::Error)),
            ("fatal", Some(LogLevel::Fatal)),
            // Case-insensitive (the live evidence used `FATAL`).
            ("FATAL", Some(LogLevel::Fatal)),
            ("Error", Some(LogLevel::Error)),
            ("WaRn", Some(LogLevel::Warn)),
            // Surrounding whitespace tolerated (shell/env-file reality).
            ("  error  ", Some(LogLevel::Error)),
            // Rejected — the caller falls back LOUDLY.
            ("", None),
            ("   ", None),
            ("warning", None),
            ("err", None),
            ("notalevel", None),
            ("0", None),
        ];
        for (input, expected) in cases {
            assert_eq!(
                parse_iox2_log_level(input),
                *expected,
                "IOX2_LOG_LEVEL={input:?} parsed wrong"
            );
        }
    }

    /// The default is `Error` and it is the SAME value the generated
    /// workspace `.cargo/config.toml` documents. A drift-guard: if someone
    /// changes the const, this fails and they must update the workspace
    /// template + docs together.
    #[test]
    fn default_iox2_log_level_is_error() {
        assert_eq!(DEFAULT_IOX2_LOG_LEVEL, LogLevel::Error);
        assert_eq!(IOX2_LOG_LEVEL_ENV, "IOX2_LOG_LEVEL");
    }

    /// `init_iceoryx_log_level` actually MOVES this linked copy's
    /// process-global level, for every input class (absent / valid / invalid).
    /// `get_log_level()` returns the raw `u8`, so the oracle is the `as u8`
    /// discriminant — the exact value `__internal_print_log_msg` compares.
    ///
    /// `#[serial]`: the level is a process-global static shared with the
    /// capture tests below (which force `Trace`).
    #[test]
    #[serial]
    fn init_iceoryx_log_level_applies_every_input_class() {
        // Valid, non-default value → applied verbatim.
        init_iceoryx_log_level(Some("trace"));
        assert_eq!(iceoryx2_log::get_log_level(), LogLevel::Trace as u8);

        // Absent → the Cerulion default (Error), NOT iceoryx2's crate default
        // (Info). Asserted from a Trace baseline so a no-op implementation
        // fails here.
        init_iceoryx_log_level(None);
        assert_eq!(iceoryx2_log::get_log_level(), LogLevel::Error as u8);

        // Empty behaves exactly like absent.
        init_iceoryx_log_level(Some("debug"));
        init_iceoryx_log_level(Some("   "));
        assert_eq!(iceoryx2_log::get_log_level(), LogLevel::Error as u8);

        // Unparseable → falls back to the default (loudly; the `eprintln!` is
        // pinned end-to-end by the subprocess test `iox2_log_level_test.rs`).
        init_iceoryx_log_level(Some("trace"));
        init_iceoryx_log_level(Some("notalevel"));
        assert_eq!(iceoryx2_log::get_log_level(), LogLevel::Error as u8);

        // Case-insensitive through the full entry point, not just the parser.
        init_iceoryx_log_level(Some("FATAL"));
        assert_eq!(iceoryx2_log::get_log_level(), LogLevel::Fatal as u8);

        // Restore the suite-wide default so sibling tests are unaffected.
        init_iceoryx_log_level(None);
    }

    #[test]
    fn capture_returns_empty_when_no_logs_emitted() {
        // Install bridge if not already (best-effort; another
        // test in the same process may have done it first).
        let _ = install_iceoryx2_tracing_bridge();
        let (result, captured) = capture_iceoryx_logs(|| 42);
        assert_eq!(result, 42);
        // Outside of any iceoryx2 call we don't expect emissions;
        // capture buffer is empty.
        assert!(captured.is_empty());
    }

    // `capture_collects_emissions_during_call`
    // and `capture_isolates_to_thread` mutate iceoryx2's process-
    // global `set_log_level`. Without serialization, one's
    // `set_log_level(Error)` restore can fire while the other's
    // capture closure is running, suppressing the trace emission
    // and spuriously failing the `captured.len() == 1` assertion.
    // Each affected test is `#[serial]` (from the `serial_test`
    // dev-dep, visible to crate-internal `#[cfg(test)]` mods) so
    // they run one at a time.

    #[test]
    #[serial]
    fn capture_collects_emissions_during_call() {
        // Synthesize a log emission via iceoryx2_log's macros.
        // This proves the bridge captures structured info.
        let _ = install_iceoryx2_tracing_bridge();
        // Set log level so trace! fires.
        iceoryx2::prelude::set_log_level(iceoryx2::prelude::LogLevel::Trace);
        let (_, captured) = capture_iceoryx_logs(|| {
            iceoryx2_log::trace!(from "test_origin", "synthetic trace message");
        });
        // Restore default.
        iceoryx2::prelude::set_log_level(iceoryx2::prelude::LogLevel::Error);
        assert_eq!(captured.len(), 1);
        assert_eq!(captured[0].message, "synthetic trace message");
        assert!(captured[0].origin.contains("test_origin"));
    }

    /// Returns the current state of the thread-local `CAPTURE`
    /// buffer for the calling thread. Test-only — exposes
    /// internal state so panic-safety can be asserted.
    #[cfg(test)]
    pub(super) fn thread_local_capture_state() -> Option<Vec<CapturedLog>> {
        CAPTURE.with(|c| c.borrow().clone())
    }

    #[test]
    fn capture_clears_thread_local_after_panic() {
        // The closure passed to `capture_iceoryx_logs` panics. The
        // `CaptureGuard` RAII must clear the thread-local on
        // unwind so it doesn't leak across calls on a reused
        // thread-pool worker. Spawn a dedicated thread so the
        // panicked state cannot leak into other tests.
        let _ = install_iceoryx2_tracing_bridge();
        let handle = std::thread::spawn(|| {
            // Pre-condition: thread-local starts at None on a
            // fresh thread.
            assert!(thread_local_capture_state().is_none());
            // 1. Panicking closure.
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                capture_iceoryx_logs(|| {
                    panic!("synthetic panic for test");
                });
            }));
            assert!(result.is_err(), "panic must propagate out of capture");
            // 2. Thread-local must be cleared by the RAII guard,
            //    regardless of whether the panic landed before or
            //    after any emission was buffered.
            assert!(
                thread_local_capture_state().is_none(),
                "CAPTURE thread-local must be cleared after panic; \
                 found leaked state: {:?}",
                thread_local_capture_state()
            );
        });
        handle.join().expect("test thread should not panic");
    }

    #[test]
    #[serial]
    fn capture_isolates_to_thread() {
        // Two threads each capture independently — no cross-talk.
        let _ = install_iceoryx2_tracing_bridge();
        iceoryx2::prelude::set_log_level(iceoryx2::prelude::LogLevel::Trace);
        let handle = std::thread::spawn(|| {
            let (_, captured) = capture_iceoryx_logs(|| {
                iceoryx2_log::trace!(from "thread_b", "from thread B");
            });
            captured
        });
        let (_, captured_a) = capture_iceoryx_logs(|| {
            iceoryx2_log::trace!(from "thread_a", "from thread A");
        });
        let captured_b = handle.join().unwrap();
        iceoryx2::prelude::set_log_level(iceoryx2::prelude::LogLevel::Error);
        // Each thread saw only its own emission.
        assert_eq!(captured_a.len(), 1);
        assert!(captured_a[0].message.contains("from thread A"));
        assert_eq!(captured_b.len(), 1);
        assert!(captured_b[0].message.contains("from thread B"));
    }
}
