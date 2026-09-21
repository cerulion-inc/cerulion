// SPDX-License-Identifier: AGPL-3.0-only
//! Clock abstraction for deterministic timestamps.
//!
//! Provides a `Clock` trait with three implementations:
//! - `RealClock`: uses `CLOCK_MONOTONIC` for real-time operation
//! - `VirtualClock`: uses explicit `advance()`/`set()` for deterministic replay
//! - `ExternalClock`: latched time pushed by an external time master
//!
//! # Why CLOCK_MONOTONIC?
//!
//! `std::time::Instant` uses a per-process epoch — two processes calling `now()` at
//! the same wall-clock moment get different values. `CLOCK_MONOTONIC` uses the kernel's
//! boot-time counter, shared across all processes on the machine. This is critical for:
//! - Cross-process sensor fusion (camera + IMU from different nodes)
//! - Deterministic replay (Principle #7: Replay = Live)

use std::sync::atomic::{AtomicU64, Ordering};

/// Clock trait for obtaining timestamps.
///
/// All timestamps are in nanoseconds from a stable epoch.
/// Implementations must be `Send + Sync` for use across threads.
///
/// # Methods
///
/// - `now_ns()` — runtime-internal "what time is it according to this
///   clock". For `RealClock` this is `CLOCK_MONOTONIC` ns; for
///   `VirtualClock` it's the explicitly-advanced counter; for
///   `ExternalClock` it's the latched external master time. This is
///   the ACTIVE-source read the scheduler uses to evaluate trigger
///   policies, and the user-facing macro shim exposes it directly as
///   `self.now_ns()` — the primary determinism-safe read for node
///   code. The source-specific reads (`real_ns()` / `virt_ns()` /
///   `ext_ns()`) name a SPECIFIC source regardless of which clock is
///   active.
/// - `ext_ns()` — Some(t) only for `ExternalClock` (an external time
///   master, e.g. a physics sim's published `/clock`). None for every
///   other source. (The external source dimension.)
/// - `virt_ns()` — Some(t) only for a virtual/controlled clock
///   (`VirtualClock`). None for real or external sources. (The sole
///   controlled-time read; replaces the former `sim_ns`.)
pub trait Clock: Send + Sync {
    /// Returns the current time in nanoseconds, in whatever epoch the
    /// implementation defines (CLOCK_MONOTONIC for `RealClock`,
    /// explicit counter for `VirtualClock`, latched external time for
    /// `ExternalClock`).
    ///
    /// This is the ACTIVE-source read used by the scheduler and by the
    /// macro shim's `self.now_ns()` — it returns whatever the active
    /// clock's time is. Use `real_ns()` / `virt_ns()` / `ext_ns()` on
    /// the macro shim when you need to name a SPECIFIC source.
    fn now_ns(&self) -> u64;

    /// Returns `Some(t)` only when this clock is an
    /// EXTERNAL-source clock (`ExternalClock`) being driven by an
    /// external time master — e.g. a physics sim that publishes its own
    /// `/clock`. `t` is the latched external time. Default impl returns
    /// `None` for every clock that is not externally driven, so callers
    /// can ask "is this clock externally driven? if so, what's its
    /// current value?" without downcasting.
    fn ext_ns(&self) -> Option<u64> {
        None
    }

    /// Returns `Some(t)` only when this clock is a VIRTUAL /
    /// controlled clock (`VirtualClock`), where `t` is the
    /// explicitly-advanced counter. Default impl returns `None` for
    /// real and external sources.
    ///
    /// This is the sole controlled-time read (the former `sim_ns` was
    /// renamed to `virt_ns`, and the warn-on-Real behavior was
    /// consolidated here). Test/replay code uses `Some(_)` to assert "we're
    /// definitely running under VirtualClock".
    fn virt_ns(&self) -> Option<u64> {
        None
    }
}

/// Platform-agnostic wall-clock nanoseconds
/// since a stable monotonic epoch.
///
/// Resolves to:
/// - **macOS / iOS**: `clock_gettime_nsec_np(CLOCK_UPTIME_RAW)` —
///   ns-direct, suspend-excluding (matches Linux `CLOCK_MONOTONIC` +
///   `std::Instant`); see below
/// - **Linux / *BSD**: `clock_gettime(CLOCK_MONOTONIC)` — kernel
///   boot-time counter, immune to NTP / wall-clock drift, shared
///   across processes on the host.
/// - **Windows**: `QueryPerformanceCounter` divided to ns precision —
///   high-resolution monotonic counter equivalent to CLOCK_MONOTONIC
///   in spirit. (Not currently exercised; Cerulion targets Unix-only
///   in practice but the cfg branch documents intent.)
///
/// # ⚠ Epoch differs across platforms
///
/// `real_ns()` returns nanoseconds since *some* stable monotonic
/// epoch on a SINGLE machine. The epoch IS NOT portable across
/// platforms:
///
/// - On Linux, `CLOCK_MONOTONIC` is "time since boot" and on standard
///   kernels does NOT count system suspend — it PAUSES across
///   sleep/hibernate (`CLOCK_BOOTTIME` is the suspend-inclusive Linux
///   variant, deliberately NOT used here).
/// - On macOS, `CLOCK_UPTIME_RAW` is likewise SUSPEND-EXCLUDING — it
///   does not advance while the system is asleep. So both platforms,
///   and Rust's `std::time::Instant` on each, agree: `real_ns()` is
///   monotonic, suspend-EXCLUDING wall time. (For cross-platform parity,
///   this supersedes the earlier `mach_continuous_time` pick,
///   which was suspend-INCLUDING and diverged from Linux + Instant.)
///
/// A host suspend is therefore INVISIBLE to `real_ns()` on both
/// platforms — by design. Stale-after-suspend is surfaced at the DATA
/// layer (a sensor's wire `timestamp_ns` reads old → `expect_within` /
/// liveliness fires), not by a monotonic-clock jump (which would inject
/// a spurious huge delta and a scheduler catch-up storm).
///
/// Within a single process on a single machine both clocks are
/// monotonic and drift-free, which is what Cerulion needs for
/// scheduler timing and bench measurements. But:
///
/// - **Do NOT compare `real_ns()` values across machines.** Use
///   `SystemTime::now()` or a network-time-synchronized clock for
///   cross-machine timestamps (this clock does not provide one for
///   zenoh-network sensor fusion).
/// - **Do NOT serialize `real_ns()` to a replay log expecting
///   cross-platform replay equivalence.** A trace recorded on macOS
///   that includes a multi-hour sleep will replay differently on
///   Linux because the macOS trace's `real_ns()` "skipped" the sleep
///   while Linux's would have advanced through it.
/// - For deterministic replay, use the runtime's `Clock`-trait
///   timestamps (`VirtualClock::virt_ns()`) — those are platform-
///   agnostic by construction.
///
/// Public free function (rather than a method on `Clock`) because the
/// real clock is INDEPENDENT of any runtime clock injection — the
/// macro shim's `real_ns()` calls this directly so a node always gets
/// hardware time regardless of whether the runtime was built with
/// `RealClock` or `VirtualClock`. Use this for benchmarks,
/// timeouts, or any latency measurement that must be coupled to
/// real-world durations on a SINGLE machine.
#[inline]
pub fn real_ns() -> u64 {
    // macOS: use `clock_gettime_nsec_np(CLOCK_UPTIME_RAW)` — it returns
    // nanoseconds DIRECTLY (the kernel applies the mach timebase), so
    // there is NO raw-counter / wrong-frequency step and the earlier
    // 42x-class bug (reading `cntvct_el0` and dividing by a bad freq) is
    // structurally impossible. `CLOCK_UPTIME_RAW` is monotonic, ns-
    // resolution (~24 MHz on Apple Silicon), and SUSPEND-EXCLUDING (it
    // does not advance while the system is asleep) — the same semantics
    // as Linux `CLOCK_MONOTONIC` below and as Rust's `std::time::Instant`
    // on macOS, so `real_ns()` means the same thing on both platforms.
    //
    // Prefer this cross-platform
    // suspend-EXCLUDING consistency + std::Instant parity over the
    // earlier `mach_continuous_time` pick (suspend-INCLUDING,
    // which diverged from Linux + Instant). A host suspend is surfaced
    // via data-deadline staleness (expect_within/liveliness on stale
    // wire timestamps), NOT via a monotonic-clock jump.
    #[cfg(target_os = "macos")]
    {
        // libc has the `CLOCK_UPTIME_RAW` constant but not the `_np`
        // accessor that returns ns directly — declare it inline (same
        // style as the other inline externs in this crate). Signature
        // from <time.h>:
        //     uint64_t clock_gettime_nsec_np(clockid_t clock_id);
        unsafe extern "C" {
            fn clock_gettime_nsec_np(clock_id: libc::clockid_t) -> u64;
        }
        // SAFETY: documented in <time.h> on Darwin; returns u64 ns
        // directly (no out-pointer). CLOCK_UPTIME_RAW is the same clock
        // `std::time::Instant` reads on macOS.
        unsafe { clock_gettime_nsec_np(libc::CLOCK_UPTIME_RAW) }
    }
    #[cfg(all(unix, not(target_os = "macos")))]
    {
        let mut ts = libc::timespec {
            tv_sec: 0,
            tv_nsec: 0,
        };
        // SAFETY: ts is a valid mutable pointer to a timespec struct.
        // CLOCK_MONOTONIC is always available on POSIX systems and
        // already has nanosecond resolution on Linux/BSD.
        let ret = unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
        debug_assert_eq!(ret, 0, "clock_gettime(CLOCK_MONOTONIC) failed");
        (ts.tv_sec as u64) * 1_000_000_000 + (ts.tv_nsec as u64)
    }
    #[cfg(windows)]
    {
        // QueryPerformanceCounter is the Windows analog. We compute
        // ns by scaling counts by (1e9 / freq). Both calls are stable
        // since Win XP. Frequency is constant per boot (cached in a
        // OnceLock to avoid the repeated syscall on the hot path).
        use std::sync::OnceLock;
        static FREQ: OnceLock<u64> = OnceLock::new();
        let freq = *FREQ.get_or_init(|| {
            let mut f: i64 = 0;
            // SAFETY: f is a valid mutable pointer; the API never fails on
            // post-XP systems but we treat 0 as a sentinel below.
            let ok = unsafe {
                windows_sys::Win32::System::Performance::QueryPerformanceFrequency(&mut f)
            };
            debug_assert_ne!(ok, 0);
            f as u64
        });
        let mut counter: i64 = 0;
        // SAFETY: counter is a valid mutable pointer.
        let ok = unsafe {
            windows_sys::Win32::System::Performance::QueryPerformanceCounter(&mut counter)
        };
        debug_assert_ne!(ok, 0);
        // Scale to ns. Multiply before divide to preserve precision
        // for typical freq values (~10MHz on modern CPUs → 100ns/tick).
        ((counter as u64).saturating_mul(1_000_000_000)) / freq
    }
    #[cfg(not(any(unix, windows)))]
    {
        compile_error!("real_ns(): unsupported platform — Cerulion targets Unix and Windows");
    }
}

/// Per-thread CPU time, in nanoseconds, consumed by
/// the **current thread** — *on-CPU only*.
///
/// This is the thread-CPU companion to [`real_ns()`]. The two measure
/// fundamentally different quantities and must not be confused:
///
/// | fn               | quantity                | includes off-CPU time? |
/// |------------------|-------------------------|------------------------|
/// | [`real_ns()`]    | WALL / monotonic        | YES (preemption, sleep, blocking I/O all count) |
/// | `thread_cpu_ns()`| THREAD-CPU (this thread)| NO  (excludes time the thread was preempted / descheduled / blocked) |
///
/// So a thread that took 10ms of wall time but only 2ms of CPU was
/// preempted/off-CPU for ~8ms — the delta IS the preemption.
///
/// # ⚠ NON-deterministic — diagnostics only
///
/// Like [`real_ns()`], this reads hardware/kernel time and is **NOT
/// determinism-safe**: it is not reproducible across runs and must
/// never feed scheduler trigger evaluation, replay logs, or any
/// observable runtime state (Principle #7: Replay = Live). It lives in
/// the same lint-flagged territory `real_ns()` will occupy under
/// the determinism-safety lint. Use it ONLY for human-facing
/// diagnostics.
///
/// # Intended consumer
///
/// The `tick_within_ms` miss diagnostic (see
/// `scheduler::Scheduler::fire_node`). `tick_within_ms` measures WALL
/// completion via `Instant::now()`; on a miss, a report that
/// additionally carried the thread-CPU elapsed would let a user distinguish
/// preemption ("10ms wall / 2ms CPU → the OS descheduled me") from
/// genuinely slow code ("10ms wall / 9.8ms CPU → my tick is just
/// expensive"). The clock model ships ONLY this primitive; the dual
/// report is not implemented.
///
/// # Platform sources
///
/// - **Unix (Linux + macOS + *BSD):** `clock_gettime(CLOCK_THREAD_CPUTIME_ID)`.
///   The constant resolves to `libc::CLOCK_THREAD_CPUTIME_ID` on every
///   supported unix (Linux = 3, Darwin = 16; macOS has supported this
///   clock id since 10.12), so a single `#[cfg(unix)]` arm covers both
///   without an inline-extern declaration. The kernel reports
///   nanosecond-resolution per-thread CPU time directly.
/// - **Windows:** `GetThreadTimes(GetCurrentThread())` summing the
///   kernel + user `FILETIME`s (each a 100-ns-tick count). Compile-only,
///   never exercised in CI (no Windows runner) — mirrors the posture of
///   `real_ns()`'s `QueryPerformanceCounter` arm.
#[inline]
pub fn thread_cpu_ns() -> u64 {
    #[cfg(unix)]
    {
        let mut ts = libc::timespec {
            tv_sec: 0,
            tv_nsec: 0,
        };
        // SAFETY: ts is a valid mutable pointer to a timespec struct.
        // CLOCK_THREAD_CPUTIME_ID is available on Linux and on macOS
        // (Darwin 10.12+); the kernel reports nanosecond-resolution
        // per-thread (on-CPU) time directly. A non-zero return indicates
        // failure; we assert on it in debug builds only (this is a
        // diagnostics-only primitive, never on the scheduler/replay path).
        let ret = unsafe { libc::clock_gettime(libc::CLOCK_THREAD_CPUTIME_ID, &mut ts) };
        debug_assert_eq!(ret, 0, "clock_gettime(CLOCK_THREAD_CPUTIME_ID) failed");
        (ts.tv_sec as u64) * 1_000_000_000 + (ts.tv_nsec as u64)
    }
    #[cfg(windows)]
    {
        // GetThreadTimes reports four FILETIMEs for a thread; we want
        // the CPU time it actually consumed = kernel + user. Creation
        // and exit times are wall-clock and irrelevant here. Each
        // FILETIME is a count of 100-ns ticks (low/high u32 halves),
        // so ns = ticks * 100. Both APIs are stable since Win 2000.
        // Compile-only (no Windows CI runner) — mirrors the QPC arm.
        use windows_sys::Win32::Foundation::FILETIME;
        use windows_sys::Win32::System::Threading::{GetCurrentThread, GetThreadTimes};

        let mut creation = FILETIME {
            dwLowDateTime: 0,
            dwHighDateTime: 0,
        };
        let mut exit = FILETIME {
            dwLowDateTime: 0,
            dwHighDateTime: 0,
        };
        let mut kernel = FILETIME {
            dwLowDateTime: 0,
            dwHighDateTime: 0,
        };
        let mut user = FILETIME {
            dwLowDateTime: 0,
            dwHighDateTime: 0,
        };
        // SAFETY: GetCurrentThread returns a pseudo-handle that needs no
        // close; all four FILETIME out-pointers are valid and writable.
        let ok = unsafe {
            GetThreadTimes(
                GetCurrentThread(),
                &mut creation,
                &mut exit,
                &mut kernel,
                &mut user,
            )
        };
        debug_assert_ne!(ok, 0, "GetThreadTimes failed");
        // Reassemble each FILETIME's 64-bit 100-ns-tick count and sum
        // kernel + user, then convert ticks → ns (×100).
        let filetime_ticks = |ft: &FILETIME| -> u64 {
            ((ft.dwHighDateTime as u64) << 32) | (ft.dwLowDateTime as u64)
        };
        (filetime_ticks(&kernel) + filetime_ticks(&user)).saturating_mul(100)
    }
    #[cfg(not(any(unix, windows)))]
    {
        compile_error!("thread_cpu_ns(): unsupported platform — Cerulion targets Unix and Windows");
    }
}

/// Real-time clock using `CLOCK_MONOTONIC`.
///
/// Stateless — uses the kernel's monotonic clock which provides:
/// - Kernel-wide epoch (boot time), shared across all processes
/// - Monotonically increasing (never goes backwards)
/// - Not affected by NTP adjustments or wall-clock changes
#[derive(Debug, Clone, Copy, Default)]
pub struct RealClock;

impl Clock for RealClock {
    #[inline]
    fn now_ns(&self) -> u64 {
        // Delegate to the platform-agnostic free function — single
        // source of truth for "what does real-clock time mean here".
        real_ns()
    }

    fn virt_ns(&self) -> Option<u64> {
        // Under the production RealClock
        // default a node illegitimately relying on virtual/controlled
        // time gets None. Warn ONCE per process at the true source of
        // the None so the contract is loud, not silent. (The macro
        // `virt_ns()` shim is the only production caller of
        // `Clock::virt_ns` on a dyn clock, so this fires once when the
        // first node reads virt_ns() under RealClock.)
        use std::sync::atomic::{AtomicBool, Ordering};
        static WARNED: AtomicBool = AtomicBool::new(false);
        if !WARNED.swap(true, Ordering::Relaxed) {
            tracing::warn!(
                "virt_ns() returned None: the graph is running under RealClock (the \
                 production default), so virtual/controlled-time reads are unavailable \
                 — use now_ns() for the active clock or real_ns() for raw wall-clock \
                 time. This warns once."
            );
        }
        None
    }

    fn ext_ns(&self) -> Option<u64> {
        // Same loud-once contract for external-source reads.
        // Under RealClock there is no external time master, so ext_ns()
        // is None; warn once so a node mistakenly assuming an
        // ExternalClock is connected sees the contract.
        use std::sync::atomic::{AtomicBool, Ordering};
        static WARNED: AtomicBool = AtomicBool::new(false);
        if !WARNED.swap(true, Ordering::Relaxed) {
            tracing::warn!(
                "ext_ns() returned None: the graph is NOT running under an ExternalClock \
                 (no external time master connected), so external-source reads are \
                 unavailable — use now_ns() for the active clock. This warns once."
            );
        }
        None
    }
}

/// Virtual (controlled) clock for deterministic testing and replay.
///
/// Time only advances via explicit `advance()` / `advance_by_recorded()`
/// / `set()` calls. Starts at 0ns — not wall-clock time. This makes
/// assertions trivial: `advance(10_000_000)` → `now_ns() == 10_000_000`.
///
/// `VirtualClock` is the launch name for the
/// explicitly-advanced replay/test clock (renamed from the former
/// `SimulatedClock`).
///
/// Uses `AtomicU64` for lock-free reads on the hot path. Only the
/// scheduler calls `advance()`, so no CAS contention.
#[derive(Debug)]
pub struct VirtualClock {
    time_ns: AtomicU64,
}

impl VirtualClock {
    /// Create a new virtual clock starting at 0ns.
    pub fn new() -> Self {
        Self {
            time_ns: AtomicU64::new(0),
        }
    }

    /// Advance time by `delta_ns` nanoseconds. Returns new time.
    ///
    /// Uses `Release` ordering so that any thread reading via `now_ns()`
    /// (with `Acquire`) sees the updated time and all prior writes.
    pub fn advance(&self, delta_ns: u64) -> u64 {
        self.time_ns.fetch_add(delta_ns, Ordering::Release) + delta_ns
    }

    /// Convenience: advance by milliseconds.
    pub fn advance_ms(&self, ms: u64) -> u64 {
        self.advance(ms * 1_000_000)
    }

    /// Advance the controlled (gating) clock by a
    /// run-INDEPENDENT logical quantum, returning the new time. Mechanically
    /// identical to `advance` (atomic add with `Release` ordering), but named
    /// for the canonical determinism contract — it names the run-independence
    /// contract for BOTH intended feeds (the live-polled feed is wired today;
    /// the replay feed lands with the record-exec-time model):
    ///
    /// - **Live-polled gating (wired today):** the poll loop's FIXED delta (its
    ///   run-independent tick quantum) — the only feed routed here at present.
    /// - **Replay (future record-exec-time wiring — not yet fed):** a
    ///   replay-bag RECORDED execution duration (Basis-Robotics model — logical
    ///   time advances by exactly the work that was done). The method is NAMED
    ///   for this feed; no caller supplies a recorded-duration delta yet.
    ///
    /// Both are run-independent — they do NOT depend on measuring the current
    /// run's wall-clock — which is precisely what makes a replay bit-for-bit
    /// identical to the polled gating run (Principle #7).
    ///
    /// GUARDRAIL: wall-clock / telemetry durations (live elapsed, a
    /// `max(measured peer durations)` agreement value, etc.) MUST NOT be passed
    /// here as a gating input — those are bag/telemetry-only and would break
    /// replay = live. (The one documented exception is the `build_for_test`
    /// virtual-LIVE seam, which advances by wall-clock `elapsed`; that
    /// difference is `fire_time_ns`-EXCLUDED — see `polled_vs_live_iox2_test`.)
    ///
    /// This is wired into `Scheduler::begin_step` via
    /// `ClockInner::advance` (the single per-step gating-clock seam). The
    /// dedicated name keeps the run-independence intent explicit at the call
    /// site (vs. a bare `advance`).
    pub fn advance_by_recorded(&self, duration_ns: u64) -> u64 {
        self.time_ns.fetch_add(duration_ns, Ordering::Release) + duration_ns
    }

    /// Set absolute time (for replay positioning).
    pub fn set(&self, time_ns: u64) {
        self.time_ns.store(time_ns, Ordering::Release);
    }
}

impl Default for VirtualClock {
    fn default() -> Self {
        Self::new()
    }
}

impl Clock for VirtualClock {
    #[inline]
    fn now_ns(&self) -> u64 {
        self.time_ns.load(Ordering::Acquire)
    }

    /// VirtualClock IS a virtual/controlled clock, so
    /// `virt_ns()` returns the current counter (same as `now_ns()`)
    /// rather than the trait default `None`. `ext_ns()` stays at the
    /// default `None` — virtual time is not external time.
    #[inline]
    fn virt_ns(&self) -> Option<u64> {
        Some(self.time_ns.load(Ordering::Acquire))
    }
}

/// External-source clock driven by an external time master.
///
/// Mechanically an `AtomicU64` latched counter: an external master
/// (e.g. a physics simulator publishing its own `/clock`) pushes its
/// current time in via [`ExternalClock::set_external`], and the
/// runtime reads it via `now_ns()`.
///
/// Because this IS the external source, `ext_ns()` returns
/// `Some(now_ns())` (vs. the trait default `None`). When an
/// `ExternalClock` is the active clock, `now_ns()` is the
/// active-source read the scheduler uses to evaluate trigger policies.
/// `virt_ns()` stays at the trait default `None` — external
/// time is not virtual/controlled time.
#[derive(Debug)]
pub struct ExternalClock {
    time_ns: AtomicU64,
}

impl ExternalClock {
    /// Create a new external clock starting at 0ns (before the external
    /// master has pushed any time).
    pub fn new() -> Self {
        Self {
            time_ns: AtomicU64::new(0),
        }
    }

    /// Latch the external master's current time, in nanoseconds.
    ///
    /// Stored with `Release` ordering so any thread reading via
    /// `now_ns()` (with `Acquire`) sees the pushed value and all prior
    /// writes. This is the only mutator — the external master is the
    /// sole writer.
    ///
    /// INVARIANT (the caller's / master's responsibility): feed
    /// monotonically non-decreasing time within a run. The scheduler reads
    /// `now_ns()` for trigger evaluation (Period `next_fire_ns`,
    /// `expect_within`/`promise_within` windows); a rewind would produce
    /// negative/wrapping deltas. Unlike `RealClock` (kernel-monotonic) and
    /// `VirtualClock` (`fetch_add`), this source does NOT self-enforce
    /// monotonicity — the enforcement policy (clamp vs error vs treat a
    /// drop as a sim restart) is deferred to when external `/clock` feeding
    /// is wired. Today `ExternalClock` is inert (no
    /// feeder), so this path is unexercised.
    pub fn set_external(&self, ns: u64) {
        self.time_ns.store(ns, Ordering::Release);
    }
}

impl Default for ExternalClock {
    fn default() -> Self {
        Self::new()
    }
}

impl Clock for ExternalClock {
    #[inline]
    fn now_ns(&self) -> u64 {
        self.time_ns.load(Ordering::Acquire)
    }

    /// ExternalClock IS the external source, so `ext_ns()`
    /// returns the latched time (same as `now_ns()`) rather than the
    /// trait default `None`. `virt_ns()` stays at the default `None`.
    #[inline]
    fn ext_ns(&self) -> Option<u64> {
        Some(self.now_ns())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_real_clock_non_zero() {
        let clock = RealClock;
        let t = clock.now_ns();
        assert!(t > 0, "CLOCK_MONOTONIC should return non-zero after boot");
    }

    #[test]
    fn test_real_clock_monotonic() {
        let clock = RealClock;
        let t1 = clock.now_ns();
        let t2 = clock.now_ns();
        assert!(t2 >= t1, "CLOCK_MONOTONIC must be monotonically increasing");
    }

    #[test]
    fn test_real_clock_cross_thread_consistency() {
        use std::sync::Arc;

        let clock = Arc::new(RealClock);
        let clock2 = Arc::clone(&clock);

        let t_before = clock.now_ns();

        let handle = std::thread::spawn(move || clock2.now_ns());

        let t_thread = handle.join().expect("thread should not panic");
        let t_after = clock.now_ns();

        // Thread timestamp should be between before and after
        assert!(
            t_thread >= t_before,
            "thread timestamp should be >= pre-spawn"
        );
        assert!(
            t_after >= t_thread,
            "post-join timestamp should be >= thread timestamp"
        );
    }

    // Thread-CPU clock primitive smoke test. Bounds are
    // deliberately loose (CI-safe) — we only prove the syscall wiring
    // works (returns > 0 after burning CPU) and is monotonic
    // non-decreasing across two reads with a busy loop between. The
    // thread-CPU-vs-wall-under-preemption comparison is not covered here.
    #[test]
    fn test_thread_cpu_ns_positive_and_monotonic() {
        use std::hint::black_box;

        // Burn real CPU on THIS thread so the on-CPU counter advances.
        let mut acc: u64 = 0;
        for i in 0..2_000_000u64 {
            acc = acc.wrapping_add(black_box(i));
        }
        black_box(acc);

        let t1 = thread_cpu_ns();
        assert!(
            t1 > 0,
            "thread_cpu_ns() must be > 0 after a busy CPU loop, got {t1}"
        );

        // Burn more CPU between the two reads.
        let mut acc2: u64 = 0;
        for i in 0..2_000_000u64 {
            acc2 = acc2.wrapping_add(black_box(i).wrapping_mul(3));
        }
        black_box(acc2);

        let t2 = thread_cpu_ns();
        assert!(
            t2 >= t1,
            "thread_cpu_ns() must be monotonic non-decreasing: t1={t1}, t2={t2}"
        );
    }

    #[test]
    fn test_simulated_clock_starts_at_zero() {
        let clock = VirtualClock::new();
        assert_eq!(clock.now_ns(), 0);
    }

    #[test]
    fn test_simulated_clock_advance_exact() {
        let clock = VirtualClock::new();
        let new_time = clock.advance(10_000_000); // 10ms
        assert_eq!(new_time, 10_000_000);
        assert_eq!(clock.now_ns(), 10_000_000);
    }

    #[test]
    fn test_simulated_clock_advance_cumulative() {
        let clock = VirtualClock::new();
        clock.advance(5_000_000); // 5ms
        clock.advance(5_000_000); // +5ms = 10ms
        assert_eq!(clock.now_ns(), 10_000_000);
    }

    #[test]
    fn test_simulated_clock_set_absolute() {
        let clock = VirtualClock::new();
        clock.advance(100_000_000); // 100ms
        clock.set(42_000_000); // Jump to 42ms
        assert_eq!(clock.now_ns(), 42_000_000);
    }

    #[test]
    fn test_simulated_clock_advance_ms() {
        let clock = VirtualClock::new();
        let new_time = clock.advance_ms(10);
        assert_eq!(new_time, 10_000_000);
        assert_eq!(clock.now_ns(), 10_000_000);
    }

    #[test]
    fn test_simulated_clock_default() {
        let clock = VirtualClock::default();
        assert_eq!(clock.now_ns(), 0);
    }

    // ----- Additive clock-model tests -----

    #[test]
    fn test_external_clock_starts_at_zero() {
        let clock = ExternalClock::new();
        assert_eq!(clock.now_ns(), 0);
        assert_eq!(clock.ext_ns(), Some(0));
    }

    #[test]
    fn test_external_clock_default_starts_at_zero() {
        let clock = ExternalClock::default();
        assert_eq!(clock.now_ns(), 0);
    }

    #[test]
    fn test_external_clock_set_external_reflected_in_now_and_ext() {
        let clock = ExternalClock::new();
        clock.set_external(42_000_000);
        assert_eq!(clock.now_ns(), 42_000_000);
        assert_eq!(clock.ext_ns(), Some(42_000_000));
        // Latched, not accumulated: a second set replaces.
        clock.set_external(7);
        assert_eq!(clock.now_ns(), 7);
        assert_eq!(clock.ext_ns(), Some(7));
    }

    #[test]
    fn test_external_clock_ext_is_some_virt_is_none() {
        // ExternalClock IS the external source but is NOT virtual/
        // controlled time. After latching a time, `ext_ns()` reflects the
        // external source while `virt_ns()` stays `None`. (The now_ns/
        // ext_ns latch+replace behavior is pinned separately in
        // test_external_clock_set_external_reflected_in_now_and_ext; this
        // test's distinct contribution is the ext-Some / virt-None split.)
        let clock = ExternalClock::new();
        clock.set_external(123);
        assert_eq!(clock.ext_ns(), Some(123), "external IS the external source");
        assert_eq!(
            clock.virt_ns(),
            None,
            "external is not virtual/controlled time"
        );
    }

    #[test]
    fn test_external_clock_cross_thread_set_read() {
        use std::sync::Arc;

        let clock = Arc::new(ExternalClock::new());
        let clock2 = Arc::clone(&clock);

        let handle = std::thread::spawn(move || {
            clock2.set_external(999_000_000);
        });
        handle.join().expect("writer thread should not panic");

        // The Release store on the writer thread is observed via the
        // Acquire load here after the join's happens-before edge.
        assert_eq!(clock.now_ns(), 999_000_000);
        assert_eq!(clock.ext_ns(), Some(999_000_000));
    }

    #[test]
    fn test_real_clock_ext_ns_and_virt_ns_are_none() {
        let clock = RealClock;
        assert_eq!(clock.ext_ns(), None);
        assert_eq!(clock.virt_ns(), None);
    }

    #[test]
    fn test_simulated_clock_virt_ns_matches_now() {
        let clock = VirtualClock::new();
        assert_eq!(clock.virt_ns(), Some(0));
        clock.advance(3_000);
        assert_eq!(clock.now_ns(), 3_000);
        assert_eq!(clock.virt_ns(), Some(3_000));
        // virt_ns still works during the additive phase.
        assert_eq!(clock.virt_ns(), Some(3_000));
    }

    #[test]
    fn test_simulated_clock_advance_by_recorded_exact_and_cumulative() {
        let clock = VirtualClock::new();
        let t1 = clock.advance_by_recorded(7_000);
        assert_eq!(t1, 7_000);
        assert_eq!(clock.now_ns(), 7_000);
        // Cumulative, like advance().
        let t2 = clock.advance_by_recorded(3_000);
        assert_eq!(t2, 10_000);
        assert_eq!(clock.now_ns(), 10_000);
        // virt_ns tracks the recorded advances.
        assert_eq!(clock.virt_ns(), Some(10_000));
    }

    #[test]
    fn test_virtual_clock_alias_resolves() {
        // VirtualClock is the launch-name alias for VirtualClock.
        let clock = VirtualClock::new();
        assert_eq!(clock.now_ns(), 0);
        clock.advance_by_recorded(5_000);
        assert_eq!(clock.virt_ns(), Some(5_000));
    }

    // Anti-regression: `clock_gettime_nsec_np(CLOCK_UPTIME_RAW)`
    // returns ns directly, so the original bug (reading raw `cntvct_el0`
    // and dividing by a wrong ~1GHz freq when it ticks at ~24MHz — a
    // ~42x error) cannot occur. This pins that real_ns() advances by
    // REAL nanoseconds: a wrong/raw counter read would make a 10ms
    // thread-sleep read as ~0.24ms or ~420ms instead of ~10ms.
    //
    // We compare `real_ns()`'s delta against `std::time::Instant`'s delta
    // over the SAME sleep rather than against an absolute wall bound. On
    // macOS BOTH read the `CLOCK_UPTIME_RAW` clock family (the whole point
    // of the clock-source decision: `real_ns()` matches `std::Instant`), so
    // their deltas over one interval are near-identical no matter how long
    // the sleep actually took. A 42x scaling bug makes `real_ns`'s delta
    // diverge from `Instant`'s by ~42x and fails the ratio check. A loaded
    // CI runner that oversleeps (e.g. a 20ms sleep landing at 58ms) grows
    // BOTH deltas together, so the ratio stays ~1 — immune to scheduling
    // jitter (the old absolute `<= 50ms` upper bound was unsound and flaked
    // on CI). The relative bound (within 2x) is wide enough for the small
    // read-overhead skew between the two reads while still catching an
    // order-of-magnitude scaling error.
    #[cfg(target_os = "macos")]
    #[test]
    fn test_macos_real_ns_uptime_raw_is_real_time() {
        let t1 = real_ns();
        assert!(t1 > 0, "real_ns() must be non-zero after boot");

        let t2 = real_ns();
        assert!(t2 >= t1, "real_ns() must be monotonic across two reads");

        let inst = std::time::Instant::now();
        let r0 = real_ns();
        std::thread::sleep(std::time::Duration::from_millis(20));
        let r_delta = real_ns() - r0;
        let i_delta = inst.elapsed().as_nanos() as u64;

        // real_ns() and Instant read the same macOS clock family → their
        // deltas agree closely over the same interval; a 42x scaling bug
        // fails this. CI oversleep grows BOTH together so the ratio stays
        // ~1 (immune to scheduling jitter).
        assert!(
            r_delta >= i_delta / 2 && r_delta <= i_delta * 2,
            "real_ns delta {r_delta} must track Instant delta {i_delta} \
             within 2x (anti-regression)"
        );
    }
}
