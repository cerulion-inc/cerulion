// SPDX-License-Identifier: AGPL-3.0-only
//! Hardware `crossterm` terminal pump for the keyboard teleop node.
//!
//! This is the HARDWARE half of the injectable event-source seam. It is
//! compiled on every platform (crossterm is portable — macOS + Linux) but is
//! only ever CONSTRUCTED on the live path: the node's `external_source()` calls
//! [`spawn_crossterm_blocking`], which the deterministic polled/replay path
//! never invokes (Principle #7). Every CI test drives the node with scripted
//! [`KeyOp`]s pushed directly into the shared inbox, so this module never runs
//! without a terminal.
//!
//! # Threading + ring policy
//!
//! [`spawn_crossterm_blocking`] returns an [`ExternalSource::Blocking`]
//! closure the runtime drives on a helper thread; each `true` rings the
//! doorbell (⇒ one tick ⇒ one publish). The helper owns ALL
//! wall-clock pacing:
//!
//! - **startup**: rings exactly once (terminal available or not) so the node
//!   publishes its mandatory startup zero;
//! - **key events**: a mapped keypress pushes a [`KeyOp`] and rings;
//! - **Ctrl-C**: translated by the pump itself (see below) — zero + restore,
//!   then SIGINT on the NEXT invocation;
//! - **keepalive**: while a non-zero command is latched, rings at ~10 Hz so
//!   the downstream mux keeps seeing a fresh command;
//! - **auto-zero**: rings ONCE when the [`crate::keymap::AUTO_ZERO_NS`] silence
//!   window elapses (the node's tick re-derives the zero from its own clock);
//! - **idle**: returns `false` (no ring) — a latched ZERO (after `space`/`Esc`
//!   or auto-zero) publishes nothing further (the no-idle-publish guarantee).
//!
//! The keepalive / auto-zero cadence checks run on EVERY loop iteration —
//! not only on the poll-timeout arm — so a terminal streaming unmapped
//! events (mouse/focus/resize traffic, unbound keys) can never starve them
//! (the same rule the joystick pump applies).
//!
//! A private [`KeyState`] MIRROR (driven by the helper's wall clock) decides
//! the ring cadence + renders the status line. It is NOT authoritative: the
//! node re-folds the pushed [`KeyOp`]s against ITS clock, so the wire output
//! is a pure function of the node's own inputs (the mirror only affects when
//! the doorbell rings + the UI).
//!
//! # Ctrl-C
//!
//! Raw mode disables ISIG, so the terminal driver NEVER turns Ctrl-C into
//! SIGINT — without help, the graph's Ctrl-C shutdown path would simply never
//! run while this node holds the terminal. The pump therefore translates the
//! Ctrl-C KeyEvent explicitly, split across TWO closure invocations to
//! maximize the chance the zero lands before shutdown begins:
//!
//! 1. On the Ctrl-C keypress: push [`KeyOp::Stop`], drop the raw-mode guard
//!    (terminal restored), arm `pending_sigint`, and return `true` — the ring
//!    that publishes the zero Twist.
//! 2. On the NEXT invocation (one poll-timeout later, after the runtime has
//!    had a chance to process that ring): `libc::raise(SIGINT)` so the
//!    graph's EXISTING shutdown machinery runs as if the terminal had
//!    delivered the signal.
//!
//! This ordering makes the zero-publish tick overwhelmingly likely to land
//! before shutdown, but it is BEST-EFFORT — NOT a guarantee (the signal still
//! races the live loop's shutdown flag). The actual robot-stop guarantee on
//! Ctrl-C is the mux's staleness gate: once this keyboard goes
//! silent for > 750 ms its arbitration slot is dropped, and with both sources
//! stale the mux emits sustained safety zeros.
//!
//! # Raw-mode lifecycle (three-layer restore)
//!
//! A wedged terminal is the classic teleop failure, so restore is layered:
//!
//! 1. **Node `shutdown()`** calls [`restore_terminal_best_effort`] — the
//!    deterministic clean-shutdown path. (The guard in layer 2 lives inside
//!    the DETACHED helper thread's closure, whose `Drop` races process exit
//!    on clean shutdown — this layer removes that race.)
//! 2. **[`RawModeGuard`] `Drop`** when the runtime drops the Blocking closure
//!    (and on Ctrl-C, where the pump drops it explicitly before raising).
//! 3. **A process-wide panic hook** disables raw mode before the default hook
//!    runs, covering panics on any thread.
//!
//! All three are idempotent best-effort `disable_raw_mode` calls — running
//! any subset in any order is safe.
//!
//! If raw mode cannot be entered (no TTY — e.g. piped stdin) the pump logs a
//! warn and runs "terminal-less": it still publishes the startup zero, then
//! stays quiet.

use std::io::Write;
use std::sync::{Arc, Mutex, Once};
use std::time::{Duration, Instant};

use cerulion_core::prelude::ExternalSource;
use crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode};

use crate::keymap::{key_to_op, status_line, KeyInput, KeyOp, KeyState};

/// Keepalive ring period while a non-zero command is latched (~10 Hz).
const KEEPALIVE: Duration = Duration::from_millis(100);
/// Bounded event-poll timeout so the helper re-evaluates keepalive / auto-zero
/// and can observe an eventual runtime-drop shutdown between calls.
const POLL_TIMEOUT: Duration = Duration::from_millis(20);

/// RAII raw-mode guard: enters raw mode on construction, restores it on `Drop`
/// and on panic. Layer 2 of the three-layer restore (see the module docs).
pub struct RawModeGuard {
    active: bool,
}

impl RawModeGuard {
    /// Enter raw mode and arm the panic-restore hook. Returns `Err` if there is
    /// no TTY (the caller runs terminal-less).
    pub fn new() -> std::io::Result<Self> {
        enable_raw_mode()?;
        install_panic_restore_hook();
        Ok(Self { active: true })
    }
}

impl Drop for RawModeGuard {
    fn drop(&mut self) {
        if self.active {
            // Best-effort restore; nothing useful to do on error at teardown.
            let _ = disable_raw_mode();
            // Leave the cursor on a fresh line so the shell prompt is clean.
            let _ = write!(std::io::stderr(), "\r\n");
        }
    }
}

/// Best-effort terminal restore for the node's `shutdown()` path — layer 1 of
/// the three-layer restore. Idempotent: `disable_raw_mode`
/// on an already-cooked terminal is a harmless termios re-set, so this
/// composes with the guard's `Drop` and the panic hook in any order.
pub(crate) fn restore_terminal_best_effort() {
    let _ = disable_raw_mode();
    let _ = write!(std::io::stderr(), "\r\n");
}

/// Install (once, process-wide) a panic hook that disables raw mode before the
/// previous hook runs — layer 3: a panic anywhere (including on the pump's
/// helper thread, where the `RawModeGuard` may not be dropped by unwinding)
/// never leaves the operator with a wedged terminal.
fn install_panic_restore_hook() {
    static HOOK: Once = Once::new();
    HOOK.call_once(|| {
        let prev = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| {
            let _ = disable_raw_mode();
            let _ = write!(std::io::stderr(), "\r\n");
            prev(info);
        }));
    });
}

/// Build the [`ExternalSource::Blocking`] terminal pump feeding `inbox`.
pub fn spawn_crossterm_blocking(inbox: Arc<Mutex<Vec<KeyOp>>>) -> ExternalSource {
    // Per-closure state (owned by the helper thread for its whole life).
    // `_guard` is held ONLY for its Drop (raw-mode restore when the runtime
    // drops the closure) — never read, except the Ctrl-C arm which sets it to
    // `None` to restore EARLY. Underscore-PREFIXED (not a bare `_`, which
    // would drop the value immediately): a `_`-prefixed binding lives to
    // scope end while staying clean under deny(unused_variables) and the
    // unused_assignments lint (both skip `_`-prefixed bindings).
    let mut _guard: Option<RawModeGuard> = None;
    let mut terminal_ok = true;
    let mut started = false;
    let base = Instant::now();
    let mut mirror = KeyState::default();
    let mut last_ring = Instant::now();
    let mut prev_moving = false;
    // Armed by the Ctrl-C arm; the raise happens on the NEXT invocation so
    // the zero-publish ring gets processed first (see the module docs —
    // best-effort, not a guarantee).
    let mut pending_sigint = false;

    ExternalSource::Blocking(Box::new(move || -> bool {
        // ---- Deferred Ctrl-C SIGINT (zero first, signal second): the previous
        // invocation pushed the Stop and rang the zero publish; by now the
        // runtime has had a poll-timeout's worth of time to process that
        // ring. Raise the signal the raw-mode terminal never delivered.
        // Best-effort sequencing — the signal still races the live loop —
        // the hard robot-stop backstop is the mux's 750 ms staleness gate.
        if pending_sigint {
            pending_sigint = false;
            #[cfg(unix)]
            // SAFETY: `raise(3)` sends SIGINT to this process; a plain,
            // reentrant-safe libc call with no pointer arguments. The graph
            // installed its Ctrl-C handler at `graph run` startup — exactly
            // the consumer this signal targets.
            unsafe {
                libc::raise(libc::SIGINT);
            }
            #[cfg(not(unix))]
            tracing::warn!(
                "Ctrl-C translated without unix signals (non-unix host): zero \
                 published + terminal restored; stop the graph by its own means"
            );
            return false;
        }

        // ---- Startup: ring exactly once (terminal available or not) so the
        // node publishes its mandatory startup zero, then enter raw mode.
        if !started {
            started = true;
            match RawModeGuard::new() {
                Ok(g) => _guard = Some(g),
                Err(e) => {
                    terminal_ok = false;
                    tracing::warn!(
                        error = %e,
                        "raw mode unavailable (no TTY?) — keyboard node running \
                         terminal-less (startup zero still published; stays quiet)"
                    );
                }
            }
            last_ring = Instant::now();
            return true;
        }

        // ---- Terminal-less: stay quiet (no fabricated keys — Principle #13).
        if !terminal_ok {
            std::thread::sleep(POLL_TIMEOUT);
            return false;
        }

        // ---- One bounded observation. A mapped keypress pushes + rings
        // immediately; EVERYTHING else (unmapped keys, non-key events, poll
        // timeout, even a poll error) falls through to the SHARED cadence
        // checks below, so unmapped-event traffic can never starve the
        // keepalive or the auto-zero ring (as in the joystick pump).
        match event::poll(POLL_TIMEOUT) {
            Ok(true) => {
                if let Ok(Event::Key(k)) = event::read() {
                    // Terminal key RELEASE is unreliable — act on Press/Repeat.
                    if matches!(k.kind, KeyEventKind::Press | KeyEventKind::Repeat) {
                        // Ctrl-C never reaches us as SIGINT under raw
                        // mode (ISIG is off) — translate it ourselves. The
                        // raise itself is DEFERRED to the next invocation via
                        // `pending_sigint` (armed here) so this ring's zero
                        // publish gets processed first.
                        if k.modifiers.contains(KeyModifiers::CONTROL)
                            && matches!(k.code, KeyCode::Char('c') | KeyCode::Char('C'))
                        {
                            pending_sigint = true;
                            return ctrl_c_stop(
                                &inbox,
                                &mut mirror,
                                &mut _guard,
                                &mut prev_moving,
                                base,
                            );
                        }
                        if let Some(op) = to_op(k.code) {
                            let now = base.elapsed().as_nanos() as u64;
                            mirror.apply(op, now);
                            // POISON RECOVERY (applies to every inbox lock
                            // in this crate): the inbox is a Vec of plain
                            // POD key-ops — a panic while it was held can
                            // only leave a valid Vec (an element is either
                            // fully pushed or not; nothing is ever torn), so
                            // recovering the guard is sound. For a SAFETY
                            // inbox, delivering Stop after a panic strictly
                            // beats dropping it: an `if let Ok` would
                            // silently drop every post-poison key while the
                            // doorbell kept ringing on stale state.
                            inbox
                                .lock()
                                .unwrap_or_else(std::sync::PoisonError::into_inner)
                                .push(op);
                            let cmd = mirror.resolve(now);
                            prev_moving = !cmd.is_zero();
                            render_status(cmd);
                            last_ring = Instant::now();
                            return true;
                        }
                    }
                }
                // Unmapped key / non-key event: fall through to cadence.
            }
            Ok(false) => {
                // Poll timeout: fall through to cadence.
            }
            Err(_) => {
                // A poll error (rare) — pace, then still run the cadence
                // checks (an erroring terminal must not freeze the auto-zero).
                std::thread::sleep(POLL_TIMEOUT);
            }
        }

        // ---- SHARED cadence checks, EVERY non-ringing iteration: the
        // auto-zero transition ring + the 10 Hz keepalive. The node
        // re-derives the actual auto-zero from ITS clock; the mirror only
        // paces the ring here.
        let now = base.elapsed().as_nanos() as u64;
        let cmd = mirror.resolve(now); // may auto-zero the mirror
        let moving = !cmd.is_zero();

        if prev_moving && !moving {
            // Auto-zero transition: ring once, then go quiet.
            prev_moving = false;
            render_status(cmd);
            last_ring = Instant::now();
            return true;
        }
        if moving && last_ring.elapsed() >= KEEPALIVE {
            render_status(cmd);
            last_ring = Instant::now();
            return true;
        }
        false
    }))
}

/// First half of the Ctrl-C handling: handle a Ctrl-C KeyEvent. Pushes [`KeyOp::Stop`] (the
/// returned ring publishes one zero Twist) and restores the terminal by
/// dropping the raw-mode guard. The SIGINT raise is NOT here — the caller
/// arms `pending_sigint` and the closure raises on its NEXT invocation, after
/// the runtime has had a chance to process this ring. That sequencing makes
/// the zero-publish tick overwhelmingly likely to land before shutdown, but
/// it is best-effort, NOT a guarantee; the hard robot-stop backstop is the
/// mux's staleness gate (keyboard silent > 750 ms ⇒ slot dropped; both
/// sources stale ⇒ sustained safety zeros). Returns `true` (the ring).
fn ctrl_c_stop(
    inbox: &Arc<Mutex<Vec<KeyOp>>>,
    mirror: &mut KeyState,
    guard: &mut Option<RawModeGuard>,
    prev_moving: &mut bool,
    base: Instant,
) -> bool {
    let now = base.elapsed().as_nanos() as u64;
    mirror.apply(KeyOp::Stop, now);
    *prev_moving = false;
    // Poison-recovering lock (the Ctrl-C Stop is exactly the safety event
    // that must never be dropped) — rationale at the key-event push site.
    inbox
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .push(KeyOp::Stop);
    tracing::info!(
        "Ctrl-C — publishing a zero command and restoring the terminal; \
         SIGINT for the graph's shutdown path follows on the next pump cycle"
    );
    // Restore the terminal NOW so the upcoming shutdown output (and the
    // shell prompt) land on a cooked terminal. Dropping the guard runs
    // disable_raw_mode + a fresh line.
    *guard = None;
    true // the ring — publishes the pushed Stop as a zero Twist
}

/// Map a raw crossterm [`KeyCode`] to a [`KeyOp`] via the pure [`key_to_op`]
/// (only `Char`/`Esc` reach it; everything else is ignored).
fn to_op(code: KeyCode) -> Option<KeyOp> {
    let key = match code {
        KeyCode::Char(c) => KeyInput::Char(c),
        KeyCode::Esc => KeyInput::Esc,
        _ => return None,
    };
    key_to_op(key)
}

/// Render the minimal one-line status to stderr, raw-mode-aware (`\r` returns
/// to column 0; no trailing newline so it overwrites in place). This is the
/// one sanctioned plain-print UI (see AGENTS.md) — a teleop status line is the
/// node's explicit job; it writes to stderr so it never pollutes any piped
/// stdout and does not affect the (SHM) wire output.
fn render_status(cmd: crate::keymap::Cmd) {
    // `\x1b[2K` clears the whole line so a shorter line doesn't leave stale
    // characters; `\r` returns to column 0.
    let _ = write!(std::io::stderr(), "\r\x1b[2K{}", status_line(cmd));
    let _ = std::io::stderr().flush();
}
