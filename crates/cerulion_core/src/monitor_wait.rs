// SPDX-License-Identifier: AGPL-3.0-only
//! Shallow CPU monitor-wait primitive for the live loop's inter-step idle.
//!
//! # Why
//!
//! The production live loop (`crate::graph::GraphRuntime::run_live`) blocks on
//! the iceoryx2 WaitSet (an `epoll` on a unix socket) between steps. A blocking
//! `epoll` lets the Linux cpuidle governor put the core into a DEEP C-state
//! (C2/C3), whose exit latency (tens-to-hundreds of µs + a cache flush) shows
//! up as the ~18µs cold-wake on the data/period hops. The
//! pm_qos `/dev/cpu_dma_latency` lock (`crate::dma_lock`) suppresses this but
//! needs root.
//!
//! This module is the ROOT-FREE alternative: replace the blocking wait with a
//! CPU MONITOR-WAIT so the core parks in a SHALLOW optimized state (x86 `C0.1`,
//! aarch64 `WFE`) instead of deep-idling — no cpuidle C-state, so no cold-wake.
//!
//! # Determinism firewall (NON-NEGOTIABLE)
//!
//! This is a WAIT primitive only — it changes WHEN the live loop wakes, NEVER
//! what fires. The caller (`crate::graph::GraphRuntime`) polls listeners
//! RECORD-ONLY around it (notification queue only, never the SHM message queue,
//! never a scheduler mutation or clock advance) — exactly the firewall the
//! reactor + `spin_sources` already obey (see `graph/waitset.rs`). This module
//! itself touches no Cerulion state at all.
//!
//! # Scope (LOCKED)
//!
//! Exactly TWO real backends ship; everything else is a clean fallback:
//!
//! - `linux + x86_64` with WAITPKG enumerated → `WaitpkgUmwait`: CPUID-gated
//!   `UMONITOR`/`UMWAIT`, control `1` (the low-latency `C0.1` substate).
//! - `linux + aarch64` → `ArmWfeEventStream`: the ARMv8 baseline `WFE` +
//!   generic-timer event-stream idiom (`SEVL; WFE; LDXR; EOR; CBNZ; WFE`). One
//!   bounded `WFE` per call.
//! - everything else — other arches, non-Linux (incl. macOS), or x86 without
//!   WAITPKG → `Unavailable`: the public entry points return `false`/no-op so
//!   the caller falls back to the normal blocking WaitSet wait. NOT a
//!   busy-spin and NOT a spin-then-block floor (that measured a no-op).
//!
//! Deliberately not supported: AMD `MWAITX` and ARMv8.7 `WFET`. Those
//! resolve to `Unavailable` so they take the blocking-WaitSet fallback.
//!
//! # The macOS degraded-tier NAP
//!
//! macOS stays `Unavailable` as a CPU monitor-wait BACKEND (no UMWAIT/WFE),
//! but the park's degraded sleep-recheck NAP — the `thread::sleep` pacing the
//! `!performed` arm — now rides `os_sync_wait_on_address_with_timeout` on
//! macOS ≥ 14.4 (`park_nap`, the shared `crate::os_sync` dlsym backend),
//! measured at ~HALF `nanosleep`'s timer-coalescing overshoot on the M3 Max.
//! See the section comment below for the exact shape (a timed-nap
//! replacement — the park's wake sources cannot wake an os_sync waiter) and
//! the fallbacks (< 14.4 / `CERULION_PARK_OS_SYNC=0` / the unusable-latch /
//! every other OS = today's sleep, byte-identical).

use std::sync::OnceLock;
use std::time::{Duration, Instant};

/// The resolved monitor-wait / doorbell policy for a graph's live loop.
/// Produced ONCE by the CLI resolver
/// (`cerulion_cli_engine::graph_cmd::resolve_monitor_wait_policy`) and
/// threaded into [`crate::graph::GraphRuntime`]'s `build_live`: the
/// runtime + publishers read THIS, never raw env, so the gating lives in one
/// place.
///
/// - `monitor_wait`: replace the live loop's blocking WaitSet wait with a
///   SHALLOW CPU monitor-wait park (no deep cpuidle C-state; the timer-deadline
///   path for period nodes).
/// - `doorbell`: also arm `UMONITOR`/`WFE` on the producer-rung SHM doorbell so
///   the park wakes the instant DATA is published (the data-hop path). IMPLIES
///   `monitor_wait` (the ring wakes the same park) — the invariant `doorbell` ⇒
///   `monitor_wait` is now TYPE-ENFORCED by [`MonitorWaitPolicy::new`] (the only
///   constructor that sets the flags coerces it), so the illegal state
///   `(monitor_wait: false, doorbell: true)` is UNREPRESENTABLE.
/// - `ns`: the SHM doorbell namespace ([`crate::doorbell::default_namespace`]),
///   so a producer's owned doorbell and the consumer's registry derive the SAME
///   `/cer_db_<ns>_<hash>` object name and map the same page.
///
/// All-false / empty `ns` ([`MonitorWaitPolicy::off`] / `Default`) is the
/// inert production-off state (the live loop runs the existing blocking-WaitSet
/// path verbatim).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct MonitorWaitPolicy {
    /// Park the inter-step idle in a shallow monitor-wait instead of the
    /// blocking WaitSet (timer-deadline path). PRIVATE — read via
    /// [`MonitorWaitPolicy::monitor_wait`]; set only by [`MonitorWaitPolicy::new`]
    /// (which coerces the `doorbell` ⇒ `monitor_wait` invariant).
    monitor_wait: bool,
    /// Arm the park on the data doorbell line (data-hop path). Implies
    /// `monitor_wait`. PRIVATE — read via [`MonitorWaitPolicy::doorbell`].
    doorbell: bool,
    /// SHM doorbell namespace shared by producer + consumer. PRIVATE — read via
    /// [`MonitorWaitPolicy::ns`].
    ns: String,
}

impl MonitorWaitPolicy {
    /// Construct a resolved policy. COERCES the invariant `doorbell ⇒
    /// monitor_wait` (a doorbell ring wakes the SAME park, so arming the
    /// doorbell without the park is contradictory) — established ONCE here, so it
    /// cannot be violated by any caller.
    ///
    /// `ns` is the SHM doorbell namespace. The sibling invariant `doorbell ⇒
    /// non-empty ns` is NOT coerced here (unlike `doorbell ⇒ monitor_wait`,
    /// which is a pure logical implication): the *correct* namespace is a
    /// CONTEXTUAL value ([`crate::doorbell::default_namespace`], `$USER`-derived)
    /// that the CLI resolver owns — reading the environment inside this
    /// constructor would conflate resolution with construction. So the contract
    /// is: **a caller passing `doorbell = true` MUST pass a non-empty `ns`** (the
    /// resolver passes `default_namespace()`, which is never empty). An empty
    /// `ns` is internally consistent within one process (producer + consumer both
    /// read `ns()`, so they agree) but collapses the cross-tenant SHM-name
    /// partition; the doorbell build sites `debug_assert!` it to catch a resolver
    /// mistake in debug/tests.
    pub fn new(monitor_wait: bool, doorbell: bool, ns: String) -> Self {
        Self {
            monitor_wait: monitor_wait || doorbell,
            doorbell,
            ns,
        }
    }

    /// The inert all-off policy (production default until the CLI resolver turns
    /// it on).
    pub fn off() -> Self {
        Self::default()
    }

    /// Park the inter-step idle in a shallow monitor-wait (timer-deadline path).
    pub fn monitor_wait(&self) -> bool {
        self.monitor_wait
    }

    /// Arm the park on the data doorbell line (data-hop path). Implies
    /// `monitor_wait`.
    pub fn doorbell(&self) -> bool {
        self.doorbell
    }

    /// The SHM doorbell namespace shared by producer + consumer.
    pub fn ns(&self) -> &str {
        &self.ns
    }
}

/// Which shallow monitor-wait primitive this build runs at runtime, resolved
/// ONCE (cached in `current_backend`).
///
/// `Unavailable` is the safe fallback: the public entry points no-op on it so
/// the caller blocks on the normal WaitSet. The two real variants are each
/// constructed only on their own target (`WaitpkgUmwait` on `linux + x86_64`
/// with WAITPKG; `ArmWfeEventStream` on `linux + aarch64`) — hence the
/// per-variant `cfg_attr(allow(dead_code))` for the OTHER targets, mirroring
/// `dma_lock`'s cfg-conditional dead-code handling. (Both real variants are
/// still constructed under `cfg(test)` by the hermetic backend-injection tests.)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MonitorWaitBackend {
    /// x86 WAITPKG `UMONITOR`/`UMWAIT` (C0.1). Constructed only on
    /// `linux + x86_64` with WAITPKG enumerated.
    #[cfg_attr(
        not(all(target_os = "linux", target_arch = "x86_64")),
        allow(dead_code)
    )]
    WaitpkgUmwait,
    /// ARMv8 `WFE` + generic-timer event stream. Constructed only on
    /// `linux + aarch64`.
    #[cfg_attr(
        not(all(target_os = "linux", target_arch = "aarch64")),
        allow(dead_code)
    )]
    ArmWfeEventStream,
    /// No real primitive — the caller falls back to the blocking WaitSet wait.
    /// Constructed on `linux + x86_64` without WAITPKG (the CPUID-clear else) and
    /// on every NON-Linux / other-arch target (the `not(any(...))` arm). It is
    /// NOT constructed on `linux + aarch64` — there `resolve_backend`
    /// unconditionally returns `ArmWfeEventStream` — so it needs a per-variant
    /// dead-code allow for EXACTLY that one cell (mirroring the other two
    /// variants' per-target allows). Without it, `dead_code = deny` hard-fails
    /// the whole-crate build on `aarch64-unknown-linux-gnu` — the Jetson/Orin
    /// robotics target, and the one cfg cell neither aarch64-macOS (which
    /// constructs via `not(any)`) nor x86_64-linux (which constructs via
    /// the else) nor CI (ubuntu-x86 + macos-aarch64) ever compiles.
    #[cfg_attr(all(target_os = "linux", target_arch = "aarch64"), allow(dead_code))]
    Unavailable,
}

/// Resolve the backend for THIS target (uncached — `current_backend` caches it).
///
/// `linux + x86_64`: probe CPUID leaf 7 sub-leaf 0, ECX bit 5 (WAITPKG); set
/// ⇒ `WaitpkgUmwait`, else `Unavailable`. `linux + aarch64`: always
/// `ArmWfeEventStream` (the `WFE` + event-stream idiom is ARMv8 baseline).
/// Every other target ⇒ `Unavailable`.
///
/// **WAITPKG on virtualized x86:** selection is
/// purely the CPUID WAITPKG bit; some hypervisors enumerate WAITPKG yet trap
/// UMWAIT (#UD → SIGILL) or pin an inconsistent CPUID/feature baseline across
/// live-migration. There is no in-process #UD fault guard, so on such a VM the
/// operator must disable the park with `--no-monitor-wait` /
/// `CERULION_MONITOR_WAIT=0`. Bare metal is unaffected (verified on WAITPKG hardware).
///
/// True iff the CPU enumerates WAITPKG (UMWAIT/UMONITOR/TPAUSE): CPUID leaf 7,
/// sub-leaf 0, ECX bit 5. Marked `unsafe fn` only to make `resolve_backend`'s
/// call site's `unsafe` block genuine on both the 1.88 MSRV (where
/// `__cpuid_count` is `unsafe`) and current stable (where it is `safe`) — see
/// the call site. `#[inline]` so it costs nothing vs the inline intrinsic.
///
/// # Safety
/// CPUID is baseline on every x86_64 CPU and leaf 7 / sub-leaf 0 is always a
/// valid query, so the read is sound on any CPU that can run this binary. Callers
/// need not uphold any precondition.
#[cfg(all(target_os = "linux", target_arch = "x86_64"))]
#[inline]
unsafe fn waitpkg_enumerated() -> bool {
    (core::arch::x86_64::__cpuid_count(7, 0).ecx & (1 << 5)) != 0
}

fn resolve_backend() -> MonitorWaitBackend {
    #[cfg(all(target_os = "linux", target_arch = "x86_64"))]
    {
        // SAFETY: `waitpkg_enumerated` only does a baseline CPUID read (see its
        // doc). It is an `unsafe fn` purely so the call's `unsafe` is GENUINE on
        // both toolchains — `core::arch::x86_64::__cpuid_count` is `unsafe` on the
        // declared 1.88 MSRV but `safe` on current stable, so a direct
        // `unsafe { __cpuid_count(..) }` would be `unused_unsafe` (a `-D warnings`
        // fail) on stable, while a bare call is `E0133` on 1.88. Routing through
        // an `unsafe fn` makes the `unsafe` block load-bearing on BOTH (calling an
        // `unsafe fn` always requires `unsafe`), with no `#[allow]` and no
        // `is_x86_feature_detected!` ("waitpkg" is not a recognized feature
        // string on either toolchain). NOTE: the x86_64 path is cfg'd OUT on
        // aarch64-macOS — verify changes here with `cargo +1.88.0 check -p
        // cerulion_core --target x86_64-unknown-linux-gnu`.
        if unsafe { waitpkg_enumerated() } {
            MonitorWaitBackend::WaitpkgUmwait
        } else {
            MonitorWaitBackend::Unavailable
        }
    }
    #[cfg(all(target_os = "linux", target_arch = "aarch64"))]
    {
        MonitorWaitBackend::ArmWfeEventStream
    }
    #[cfg(not(any(
        all(target_os = "linux", target_arch = "x86_64"),
        all(target_os = "linux", target_arch = "aarch64")
    )))]
    {
        MonitorWaitBackend::Unavailable
    }
}

/// The resolved backend for this process, computed once and cached.
fn current_backend() -> MonitorWaitBackend {
    static BACKEND: OnceLock<MonitorWaitBackend> = OnceLock::new();
    *BACKEND.get_or_init(|| {
        let backend = resolve_backend();
        // One-time post-mortem breadcrumb: if a
        // WAITPKG-enumerating VM later #UD/SIGILLs on UMWAIT, this records that
        // the park was armed and on which primitive. Logged once (OnceLock), so
        // it never touches the hot path.
        tracing::debug!(backend = ?backend, "monitor-wait: resolved CPU-park backend");
        backend
    })
}

/// Returns `true` iff a real shallow monitor-wait primitive is available on
/// this target: `linux + x86_64` with WAITPKG enumerated, or `linux + aarch64`
/// (`WFE` + event stream). `false` on macOS / Windows / other arches / x86
/// without WAITPKG — the caller must fall back to the blocking WaitSet wait.
///
/// Cheap + cached: the underlying probe runs once.
/// The clamp ceiling for the live spin knob (`CERULION_LIVE_SPIN_US`),
/// 100 ms in µs — ONE ceiling for every consumer of the knob (the native
/// live loop's spin and the rmw wait's spin both clamp here, by
/// construction rather than by mirrored literals). A spin is a pre-block
/// latency optimization for an IMMINENT wake, never a polling loop; the
/// clamp also bounds `Instant + Duration` arithmetic and the core-pin
/// window a pathological value would open.
pub const SPIN_BUDGET_MAX_US: u64 = 100_000;

/// The HARDWARE park's recheck slice — ONE constant for both consumers of the
/// CPU monitor-wait park (the native live loop's
/// `GraphRuntime::monitor_wait_block` hardware arm and `rmw_cerulion`'s
/// `park_block`), by construction rather than by mirrored literals (the
/// two parks are one story).
///
/// Every slice that ends without a wake must be followed by
/// `std::thread::yield_now()` at the call site: to the OS scheduler a
/// `UMWAIT`/`WFE`-parked thread is RUNNING, so a park that slices without
/// yielding holds its core for the whole idle and a co-located runnable peer
/// gets it only at CFS wakeup granularity — measured p50 **6.997 ms** on a
/// same-core pinned ping-pong (rmw park, x86_64) versus 17 µs on the
/// kernel-sleeping tier. 20 µs is short enough that a co-located peer runs
/// well inside 100 µs (a few slices per same-core hop) while `UMWAIT`/`WFE`
/// still wake instantly on a doorbell store INSIDE a slice; the cost is one
/// ~0.5 µs `sched_yield` per slice, on the idle path only. (The no-primitive
/// DEGRADED sleep tiers are unaffected by this constant — a sleeping thread
/// already releases its core, and their chunk sizes are measured shapes of
/// their own: the sleep-recheck's ~100 µs.)
///
/// **aarch64 reality (measured):** base `WFE` has
/// no timeout operand — a bounded wait rides the generic-timer event
/// stream, whose period is MACHINE-SPECIFIC (kernel target ~100µs,
/// power-of-two divider of CNTFRQ; measured ~131µs on a Jetson Orin). A
/// 20µs-requested slice therefore parks ONE stream period there — the
/// yield fires per REAL slice — and a same-core ROUND-TRIP structurally
/// costs ~2-3 wake handoffs ≈ 2.6 periods (the parked side holds the core
/// to a slice boundary before the driver can publish, then needs it back
/// to reply; measured on that Orin: RTT p50 344µs = 2.63x the 131µs quantum, with the
/// slice counter independently reading 2.59 slices/round; x86_64 p50
/// 47.8µs, where `UMWAIT` honors the TSC deadline) — ~20x under the ~7ms
/// CFS wall this constant exists to fix, with the cross-core park win
/// unaffected. A sub-100µs ARM same-core wake would need the
/// wake-word/futex arm or an event-stream divider knob, both
/// outside this constant's scope.
pub const PARK_RECHECK: Duration = Duration::from_micros(20);

/// PURE: the shared classification of a `"1"`/`"0"` monitor-wait-family
/// env flag (`CERULION_MONITOR_WAIT`, `CERULION_DOORBELL`). Exact match
/// only; the CALLER maps [`EnvFlag::Auto`] to its own default and warns on
/// [`EnvFlag::NearMiss`] with its own context — this fn never logs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EnvFlag {
    /// `"1"` — force on.
    ForceOn,
    /// `"0"` — force off.
    ForceOff,
    /// Unset or empty — the caller's auto default applies.
    Auto,
    /// Present, non-empty, and neither `"1"` nor `"0"` — the caller warns
    /// loudly and applies its auto default (loud over silent).
    NearMiss,
}

/// See [`EnvFlag`].
pub fn classify_env_flag(raw: Option<&str>) -> EnvFlag {
    match raw {
        Some("1") => EnvFlag::ForceOn,
        Some("0") => EnvFlag::ForceOff,
        Some(other) if !other.is_empty() => EnvFlag::NearMiss,
        _ => EnvFlag::Auto,
    }
}

/// PURE: the shared classification of `CERULION_LIVE_SPIN_US` — ONE parse
/// for the native live loop and the rmw wait, so the two cannot drift.
/// The CALLER maps [`LiveSpinSetting::Derived`] to its own default (the
/// native loop's imminence-gated spin; the rmw wait's park-first no-spin)
/// and warns on `Malformed`/`clamped` with its own context — this fn never
/// logs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LiveSpinSetting {
    /// Unset or empty — the caller's derived default.
    Derived,
    /// `"0"` — spin explicitly disabled.
    Disabled,
    /// A positive integer ceiling in µs, clamped to
    /// [`SPIN_BUDGET_MAX_US`]; `clamped` says the raw value exceeded it
    /// (the caller warns once).
    Ceiling { us: u64, clamped: bool },
    /// Present but not a non-negative integer — the caller warns loudly
    /// and applies its derived default.
    Malformed,
}

/// See [`LiveSpinSetting`].
pub fn classify_live_spin_us(raw: Option<&str>) -> LiveSpinSetting {
    let Some(raw) = raw else {
        return LiveSpinSetting::Derived;
    };
    if raw.is_empty() {
        return LiveSpinSetting::Derived;
    }
    match raw.parse::<u64>() {
        Ok(0) => LiveSpinSetting::Disabled,
        Ok(n) if n > SPIN_BUDGET_MAX_US => LiveSpinSetting::Ceiling {
            us: SPIN_BUDGET_MAX_US,
            clamped: true,
        },
        Ok(n) => LiveSpinSetting::Ceiling {
            us: n,
            clamped: false,
        },
        Err(_) => LiveSpinSetting::Malformed,
    }
}

pub fn monitor_wait_available() -> bool {
    !matches!(current_backend(), MonitorWaitBackend::Unavailable)
}

/// Outcome of a doorbell-armed shallow park attempt
/// ([`monitor_wait_until_addr`]). The addr variant can
/// return WITHOUT parking via its lost-wakeup guard (the layer-1 pre-arm
/// early-out and the layer-2 arm-time recheck) when the doorbell moved after
/// the caller's `expected` snapshot; a caller keeping slice telemetry
/// (`park_yields`) must not count such a return as a completed slice, which a
/// plain `bool` could not express. (A park cut short at the DEADLINE boundary
/// still reads `Parked` — pre-existing, bounded at one per park window, and
/// the caller's own loop-top deadline check fires first in practice.)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AddrParkOutcome {
    /// A real monitor-wait executed (`UMWAIT`/`WFE` reached, or the wait was
    /// already at its deadline).
    Parked,
    /// The lost-wakeup guard detected a ring since the caller's snapshot and
    /// skipped the park entirely — a WAKE in flight, not a slice.
    RingPending,
    /// No primitive on this target — the caller takes its sleep fallback.
    Unavailable,
}

impl AddrParkOutcome {
    /// Must the caller SKIP its sleep fallback (the old `bool` this
    /// replaces)? True for everything but [`AddrParkOutcome::Unavailable`].
    pub fn performed(self) -> bool {
        !matches!(self, Self::Unavailable)
    }

    /// Did a real wait complete — a slice for telemetry? True only for
    /// [`AddrParkOutcome::Parked`].
    pub fn parked(self) -> bool {
        matches!(self, Self::Parked)
    }
}

/// Park the core in a SHALLOW idle (no deep cpuidle C-state) until either
/// `recheck` elapses or `deadline` is reached (whichever is sooner), then
/// return. Returns `true` iff a real monitor-wait was actually performed
/// (== `monitor_wait_available`); `false` on a target without the primitive
/// (the no-op fallback), so the caller never busy-spins on it and instead
/// blocks on the WaitSet.
///
/// On x86 a single `UMWAIT` parks for `min(recheck, time-to-deadline)` — a
/// real TSC deadline, so the requested slice is honored. On aarch64 one `WFE`
/// parks for up to one event-stream period regardless of `recheck`; the
/// caller loops until `deadline`. Box-measured: base
/// `WFE` has NO timeout operand, so the event stream is the ONLY bound — its
/// period is MACHINE-SPECIFIC (the kernel targets ~100µs but the divider is a
/// power of two of CNTFRQ, so the real period lands wherever that quantizes;
/// measured ~131µs on a Jetson Orin). A `recheck` below the period is therefore
/// quantized UP to one period: the wake GRANULARITY of a timer-bounded
/// aarch64 wait — and the per-slice yield cadence riding it — is the stream
/// period, not `recheck`.
pub(crate) fn monitor_wait_until(deadline: Instant, recheck: Duration) -> bool {
    monitor_wait_until_with_backend(current_backend(), deadline, recheck)
}

/// Like `monitor_wait_until`, but ARM the monitor on the GIVEN `addr` (a
/// doorbell's `AtomicU64` payload) instead of a private scratch word — so a
/// producer's store to that line wakes `UMWAIT`/`WFE` the instant data is
/// published, not just on the TSC / event-stream timer. Returns an
/// [`AddrParkOutcome`]: `Parked` for a real monitor-wait;
/// `RingPending` when the lost-wakeup guard (either layer) detected a ring
/// since the caller's `expected` snapshot and SKIPPED the park — the caller
/// must not count such a return as a completed park slice; `Unavailable` on
/// a target without the primitive (the no-op fallback — the caller takes its
/// sleep path).
///
/// **Lost-wakeup guard (DPDK final-poll-then-monitor):** the doorbell can be
/// rung by another core in the window between the CALLER's snapshot and the
/// park. The caller passes the value it observed in `expected` (its OWN
/// pre-park snapshot, taken BEFORE the outer record-only poll). The guard has
/// two layers, both comparing against the CALLER's `expected` (NEVER a fresh
/// internal read, which would ABSORB and lose a ring that landed since the
/// snapshot):
///
/// 1. a platform-independent EARLY-OUT here: re-read `*addr`; if it already
///    moved past `expected`, a ring landed — skip the park entirely.
/// 2. the backend's arm-then-recheck:
///    AFTER `UMONITOR`-arming the line and BEFORE `UMWAIT`, re-read
///    `*addr` and compare against `expected`; if it differs, skip `UMWAIT`. (On
///    aarch64 the `ldxr` arming the exclusive monitor plus the `cbnz` compare
///    against `expected` is the equivalent.)
///
/// Layer 1 is an additive fast path; layer 2 closes the arm-window race (a ring
/// landing between layer 1 and the arm). Both are required.
///
/// # Safety
///
/// `addr` must point to a valid, 8-byte-aligned `u64` that stays mapped for the
/// duration of the call (the doorbell's mapping satisfies this).
pub unsafe fn monitor_wait_until_addr(
    addr: *const u64,
    expected: u64,
    deadline: Instant,
    recheck: Duration,
) -> AddrParkOutcome {
    // SAFETY: forwarded to the dispatch seam, which reads `*addr` only after the
    // backend check; the caller's `addr` validity contract (above) is upheld.
    monitor_wait_until_addr_with_backend(current_backend(), addr, expected, deadline, recheck)
}

// --- backend-injection dispatch seams -------------------------------------
//
// These private fns ARE the hermetic test seam: they run the full dispatch
// (incl. the `Unavailable` fallback and the platform-independent lost-wakeup
// early-out) against an INJECTED backend, so those paths are testable on ANY
// OS without the WAITPKG/WFE hardware (mirrors `dma_lock`'s path-injected
// `deepest_cap_from_cpu_root`). The public entry points above feed them the
// real `current_backend()`.

/// Dispatch `monitor_wait_until` against an explicit `backend`.
fn monitor_wait_until_with_backend(
    backend: MonitorWaitBackend,
    deadline: Instant,
    recheck: Duration,
) -> bool {
    match backend {
        MonitorWaitBackend::Unavailable => {
            // No real primitive: no-op so the caller blocks on the WaitSet.
            let _ = (deadline, recheck);
            false
        }
        #[cfg(all(target_os = "linux", target_arch = "x86_64"))]
        MonitorWaitBackend::WaitpkgUmwait => x86::monitor_wait_until(deadline, recheck),
        #[cfg(all(target_os = "linux", target_arch = "aarch64"))]
        MonitorWaitBackend::ArmWfeEventStream => aarch64::monitor_wait_until(deadline, recheck),
        // A real backend variant off its target is never produced by
        // `resolve_backend` (and the hermetic tests inject only `Unavailable`
        // here), so this is genuinely unreachable. `_` (not a named arm) keeps
        // the match exhaustive across every target's cfg without an
        // unreachable-pattern warning.
        _ => {
            let _ = (deadline, recheck);
            unreachable!("a real monitor-wait backend can only run on its own target")
        }
    }
}

/// Dispatch `monitor_wait_until_addr` against an explicit `backend`, including
/// the platform-independent lost-wakeup early-out (layer 1 of the guard).
fn monitor_wait_until_addr_with_backend(
    backend: MonitorWaitBackend,
    addr: *const u64,
    expected: u64,
    deadline: Instant,
    recheck: Duration,
) -> AddrParkOutcome {
    // Unavailable: no real primitive — no-op so the caller blocks on the
    // WaitSet. Return BEFORE touching `addr` (so a dummy addr is fine here).
    if matches!(backend, MonitorWaitBackend::Unavailable) {
        let _ = (addr, expected, deadline, recheck);
        return AddrParkOutcome::Unavailable;
    }
    // Layer 1: platform-independent lost-wakeup EARLY-OUT (pure logic,
    // hermetically testable on any OS). If the doorbell already moved past the
    // caller's snapshot, a ring already landed — skip the park. ADDITIVE: it
    // does NOT replace the backend's arm-then-recheck (layer 2, the verbatim
    // 4f615cc guard), which still closes the arm-window race below.
    // SAFETY: the caller of `monitor_wait_until_addr` guarantees `addr` is a
    // valid, 8-byte-aligned `u64` mapped for the call; tests pass a live local.
    if unsafe { core::ptr::read_volatile(addr) } != expected {
        return AddrParkOutcome::RingPending;
    }
    match backend {
        #[cfg(all(target_os = "linux", target_arch = "x86_64"))]
        MonitorWaitBackend::WaitpkgUmwait => {
            x86::monitor_wait_until_addr(addr, expected, deadline, recheck)
        }
        #[cfg(all(target_os = "linux", target_arch = "aarch64"))]
        MonitorWaitBackend::ArmWfeEventStream => {
            aarch64::monitor_wait_until_addr(addr, expected, deadline, recheck)
        }
        // Unavailable handled above; a real backend off its target is never
        // produced by `resolve_backend`, and the hermetic tests only inject a
        // real backend WITH `addr` pre-moved (which the layer-1 early-out above
        // catches first), so this is unreachable.
        _ => {
            let _ = (addr, expected, deadline, recheck);
            unreachable!("a real monitor-wait backend can only run on its own target")
        }
    }
}

// ===================== The degraded-tier park NAP =====================
//
// On a target with NO real CPU monitor-wait primitive the park's `!performed`
// arm paces its ~100µs recheck loop with a bounded nap. That nap used to be
// `thread::sleep` everywhere; on macOS `nanosleep` pays the timer-coalescing
// leeway (MEASURED on the M3 Max at the park's exact 100µs recheck: overshoot
// p50 +52.3µs / p99 +58.5µs / max +90.2µs idle, and the same +52%-of-requested
// at every size — the fat-tail source the Mac bench rows carry). macOS ≥ 14.4
// replaces the nap with `os_sync_wait_on_address_with_timeout` on a
// process-local scratch word (the SHARED dlsym backend of `crate::os_sync`,
// the same family the barrier wake word blocks on), MEASURED at
// overshoot p50 +26.6µs / p99 +32.0µs / max +35–70µs — roughly HALF the slop
// at every size, idle and under compile load.
//
// **SHAPE (the design fork, decided):** this is the TIMED-NAP
// replacement, NOT a full wait-on-address on a condition word. The park waits
// on FOUR sources and none can wake an os_sync waiter: iceoryx2 listener
// notification queues (AF_UNIX socket state — no address), external raw fds
// (`poll(2)`), doorbell counters (SHM `AtomicU64`s whose producers ring with a
// PLAIN STORE — os_sync, unlike UMONITOR/WFE, wakes only on an
// `os_sync_wake_by_address_*` SYSCALL, so blocking on the doorbell address
// would never be woken by a ring, and adding a ring-side wake syscall is a
// publisher-hot-path + doorbell-ABI change, not made here), and
// barrier arrival — which the wake-word block ALREADY kernel-wakes in
// the same `!performed` arm, before this nap is reached. So the nap has NO
// waker by construction, the recheck CADENCE is unchanged (listener/doorbell/
// fd observation is never later than today's sleep chunk), and the whole win
// is the tighter timeout slop. Record-only (Principle #7): the nap changes
// only WHEN the park re-polls, never what fires.
//
// Fallbacks (all take today's `thread::sleep`, byte-identical): macOS < 14.4
// (the dlsym backend is absent), the `CERULION_PARK_OS_SYNC=0` kill switch,
// the process-wide `crate::os_sync` unusable-latch (an EINVAL/ENOTSUP from
// ANY consumer — the barrier included — means the PRIMITIVE is broken), and
// every non-macOS target (Linux behavior is UNCHANGED — the nap compiles to
// exactly the `thread::sleep` call the runtime used to inline).

/// Which mechanism paced ONE degraded-park nap — the record-only
/// attribution the runtime's `park_recheck_os_sync_naps` counter is keyed on
/// (Principle #3: the resolved tier is observable, not inferred from the OS).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ParkNapMechanism {
    /// The nap kernel-blocked on `os_sync_wait_on_address_with_timeout`
    /// (macOS ≥ 14.4, tier active). A benign early return (`EINTR`, a
    /// spurious rc >= 0) RE-ARMS the wait for the remaining cap, so the nap
    /// fills its cap exactly like `thread::sleep` (which retries `EINTR`
    /// internally) — one nap per cap, signal-proof.
    OsSyncTimedWait,
    /// The nap was a plain `thread::sleep` — every non-macOS target, macOS
    /// < 14.4, the kill switch, the unusable-latch, and the bounded middle
    /// arm of an unexpected os_sync errno.
    Sleep,
}

/// `true` iff the park's degraded-tier nap will ride the os_sync
/// timed wait on THIS host RIGHT NOW — the dlsym backend resolved (macOS ≥
/// 14.4) AND the family has not been latched unusable AND the
/// `CERULION_PARK_OS_SYNC` kill switch is not set. `false` on every other
/// target. Mirrors `crate::barrier::wake_word_block_primitive_available`'s
/// folded-gates semantics; a DIAGNOSTIC/test predicate — the nap path reads
/// the same gates directly.
pub fn park_nap_os_sync_available() -> bool {
    #[cfg(target_os = "macos")]
    {
        crate::os_sync::os_sync_backend().is_some()
            && !crate::os_sync::os_sync_latched()
            && !macos_nap::park_os_sync_kill_switch()
    }
    #[cfg(not(target_os = "macos"))]
    {
        false
    }
}

/// ONE degraded-park nap, bounded by `cap` — the pacing wait of the
/// park's `!performed` arm (no CPU monitor-wait primitive, no barrier
/// wake-word block). macOS ≥ 14.4 kernel-blocks on the os_sync timed wait
/// (half the `nanosleep` coalescing slop — see the section comment); every
/// other target and every fallback is `thread::sleep(cap)`, byte-identical to
/// the earlier inline sleep. NEVER a busy-spin: every NONZERO-cap arm
/// blocks or sleeps for the cap (benign early returns re-arm for the
/// remainder — see `park_nap_with_backend_and_latch`); a ZERO cap returns
/// immediately, so a caller looping on `park_nap(ZERO)` must bring its own
/// loop bound (the runtime's deadline check is that bound). Returns which
/// mechanism ran (the caller's counter attribution).
pub(crate) fn park_nap(cap: Duration) -> ParkNapMechanism {
    #[cfg(target_os = "macos")]
    {
        macos_nap::park_nap(cap)
    }
    #[cfg(not(target_os = "macos"))]
    {
        std::thread::sleep(cap);
        ParkNapMechanism::Sleep
    }
}

#[cfg(target_os = "macos")]
mod macos_nap {
    use super::{Duration, ParkNapMechanism};
    use crate::os_sync::{
        os_sync_backend, os_sync_errno_is_benign, os_sync_errno_is_unrecoverable, os_sync_latched,
        parse_os_sync_kill_switch, OsSyncFns,
    };
    use std::sync::OnceLock;

    /// The `CERULION_PARK_OS_SYNC` kill switch, resolved ONCE per
    /// process (mirrors the barrier's `CERULION_BARRIER_OS_SYNC` cache).
    /// `true` = the park nap's os_sync tier is DISABLED (force the sleep
    /// fallback). Its OWN switch, deliberately not the barrier's: the env
    /// names are operator-facing surface, and one feature's switch silently
    /// disabling another is the misleading-name class this repo rejects. The
    /// `=0`/`=1`/garbage GRAMMAR is the shared `parse_os_sync_kill_switch`,
    /// so the two switches cannot drift in what they accept.
    pub(super) fn park_os_sync_kill_switch() -> bool {
        static KILL_SWITCH: OnceLock<bool> = OnceLock::new();
        *KILL_SWITCH.get_or_init(|| {
            resolve_park_os_sync_disabled(std::env::var("CERULION_PARK_OS_SYNC").ok().as_deref())
        })
    }

    /// Resolve the raw `CERULION_PARK_OS_SYNC` value to "disabled?",
    /// emitting the loud garbage warn the resolution requires. Split out of
    /// [`park_os_sync_kill_switch`]'s `OnceLock` closure so the warn is
    /// `#[traced_test]`-pinnable without env / `OnceLock` games (mirrors the
    /// barrier's `resolve_os_sync_disabled`). `=0` and every in-range value
    /// resolve SILENTLY.
    pub(super) fn resolve_park_os_sync_disabled(raw: Option<&str>) -> bool {
        let (disabled, was_garbage) = parse_os_sync_kill_switch(raw);
        if was_garbage {
            tracing::warn!(
                env = "CERULION_PARK_OS_SYNC",
                got = %raw.unwrap_or(""),
                "CERULION_PARK_OS_SYNC is set but not `0` (disable) or `1`/unset (enable); \
                 keeping the macOS os_sync park-nap tier ON (`0` is the explicit kill switch)"
            );
        }
        disabled
    }

    /// The macOS nap: os_sync timed wait when the tier is active, else the
    /// sleep fallback. See the section comment for the measured why.
    pub(super) fn park_nap(cap: Duration) -> ParkNapMechanism {
        if cap.is_zero() {
            // Nothing to nap for (the caller's remaining window is exhausted);
            // a zero-timeout kernel call risks EINVAL for no benefit — and
            // `thread::sleep(ZERO)` is what the earlier inline sleep did
            // here (a no-op return). Attributed Sleep: no kernel block ran.
            return ParkNapMechanism::Sleep;
        }
        if os_sync_latched() || park_os_sync_kill_switch() {
            std::thread::sleep(cap);
            return ParkNapMechanism::Sleep;
        }
        park_nap_with_backend(os_sync_backend(), cap)
    }

    /// The backend-injection seam (mirrors `monitor_wait_until_with_backend`):
    /// runs the full nap dispatch against an EXPLICIT backend, so the
    /// absent-symbol fallback arm (macOS < 14.4 — unreachable on a ≥ 14.4 dev
    /// box) is hermetically testable by injecting `None`. Production latch =
    /// the ONE process-global family latch (`os_sync::family_latch`), so an
    /// EINVAL/ENOTSUP here degrades the barrier tiers with us.
    pub(super) fn park_nap_with_backend(
        backend: Option<&'static OsSyncFns>,
        cap: Duration,
    ) -> ParkNapMechanism {
        park_nap_with_backend_and_latch(backend, cap, crate::os_sync::family_latch())
    }

    /// The full nap dispatch against an EXPLICIT backend AND an EXPLICIT
    /// family latch (the error arms are unreachable on a
    /// healthy kernel, so their tests drive fake `OsSyncFns` backends — and
    /// the EINVAL/ENOTSUP arm flips the latch it is HANDED, so a test can
    /// pass a LOCAL `AtomicBool` instead of poisoning the process-global one
    /// every sibling os_sync activity gate reads).
    ///
    /// **The nap FILLS its cap (contract parity
    /// with `thread::sleep`, which retries `EINTR` internally and never
    /// returns early):** a benign early return (`EINTR`, a spurious rc >= 0 —
    /// Apple documents spurious wakes) RE-ARMS the timed wait for the
    /// remaining cap instead of returning, so callers that count naps per
    /// window (the `naps_single == 1` single-park pin) and callers
    /// that assert `elapsed >= cap` are signal-proof. The loop is bounded by
    /// the WALL deadline, never by an iteration count: each pass issues a
    /// kernel timed wait for the remainder, so even a pathological
    /// always-spurious kernel degenerates to repeated bounded kernel calls
    /// for at most `cap` — the same wall shape `thread::sleep`'s own EINTR
    /// retry has, never an unbounded spin.
    pub(super) fn park_nap_with_backend_and_latch(
        backend: Option<&'static OsSyncFns>,
        cap: Duration,
        latch: &std::sync::atomic::AtomicBool,
    ) -> ParkNapMechanism {
        let Some(backend) = backend else {
            // macOS < 14.4: the symbols are absent — today's sleep, unchanged.
            std::thread::sleep(cap);
            return ParkNapMechanism::Sleep;
        };
        // The watched word lives on this stack frame and NOTHING wakes it (see
        // the section comment: none of the park's wake sources can issue an
        // os_sync wake) — the wait is a pure timed kernel block whose timeout
        // slop is ~half nanosleep's. The passed `value` equals the word's
        // value, so the kernel compare matches and the wait really blocks.
        let word: u64 = 0;
        let deadline = super::Instant::now() + cap;
        loop {
            let remaining = deadline.saturating_duration_since(super::Instant::now());
            if remaining.is_zero() {
                // The cap is spent (the ordinary exit: the kernel's ETIMEDOUT
                // landed at/past the deadline, or a benign early return's
                // re-arm consumed the remainder).
                return ParkNapMechanism::OsSyncTimedWait;
            }
            // SAFETY: `word` is a live, 8-byte-aligned local alive across the
            // call; `backend.wait` is the dlsym-resolved, signature-checked
            // `os_sync_wait_on_address_with_timeout`. LOCAL flags — the word is
            // process-private (never crosses an address space, unlike the
            // barrier's SHARED wake word). Relative-ns timeout on
            // `OS_CLOCK_MACH_ABSOLUTE_TIME`; `remaining` is a bounded park
            // slice, far inside u64.
            let rc = unsafe {
                (backend.wait)(
                    &word as *const u64 as *mut core::ffi::c_void,
                    0,
                    8,
                    libc::OS_SYNC_WAIT_ON_ADDRESS_NONE,
                    libc::OS_CLOCK_MACH_ABSOLUTE_TIME,
                    remaining.as_nanos() as u64,
                )
            };
            if rc >= 0 {
                // A spurious wake (no waker exists for this word): re-arm for
                // the remainder — the loop-top deadline check bounds it.
                continue;
            }
            let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
            if os_sync_errno_is_unrecoverable(errno) {
                // The primitive is unusable (EINVAL/ENOTSUP): latch it OFF
                // process-wide — ONE latch for the whole os_sync family, so
                // the barrier tiers degrade with us — warn ONCE (the latch
                // reports the flip), and finish THIS nap's FRESH REMAINDER on
                // the sleep fallback so the recheck cadence is preserved —
                // recomputed HERE, not the pre-syscall `remaining`, so the
                // failing call's own duration is not added on top of the nap
                // (saturates to zero if it already ran past the deadline;
                // `sleep(0)` is a no-op).
                if crate::os_sync::latch_flip(latch) {
                    tracing::warn!(
                        errno,
                        "park nap os_sync_wait_on_address returned an unrecoverable errno \
                         (EINVAL/ENOTSUP); disabling the os_sync tier process-wide — the \
                         degraded park falls back to sleep-recheck pacing"
                    );
                }
                std::thread::sleep(deadline.saturating_duration_since(super::Instant::now()));
                return ParkNapMechanism::Sleep;
            }
            if !os_sync_errno_is_benign(errno) {
                // Neither the expected ETIMEDOUT/EINTR nor a hard
                // EINVAL/ENOTSUP. Warn once per DISTINCT errno + sleep the
                // bounded FRESH remainder so a persistently-failing syscall
                // can never busy-spin the recheck loop (mirrors the barrier's
                // middle arm; the remainder is inside the caller's bounded
                // slice, and it is recomputed at the sleep site so the failed
                // call's own duration is not added on top of the nap).
                static LAST_WARNED_ERRNO: std::sync::atomic::AtomicI32 =
                    std::sync::atomic::AtomicI32::new(0);
                if LAST_WARNED_ERRNO.swap(errno, std::sync::atomic::Ordering::Relaxed) != errno {
                    tracing::warn!(
                        errno,
                        "park nap os_sync_wait_on_address returned an unexpected errno; \
                         the degraded park sleeps this recheck chunk instead"
                    );
                }
                std::thread::sleep(deadline.saturating_duration_since(super::Instant::now()));
                return ParkNapMechanism::Sleep;
            }
            // Benign: ETIMEDOUT (the slice ran its course — the loop-top
            // deadline check returns) or EINTR (a signal — re-arm for the
            // remainder, exactly as `thread::sleep` retries internally).
        }
    }
}

#[cfg(all(target_os = "linux", target_arch = "x86_64"))]
mod x86 {
    use super::{Duration, Instant};
    use std::sync::OnceLock;

    /// Invariant-TSC cycles-per-nanosecond, calibrated ONCE against `Instant`
    /// via a short busy-spin window. Used to convert the wait `Duration` into
    /// the absolute TSC deadline `UMWAIT` consumes in `EDX:EAX`.
    fn tsc_per_ns() -> f64 {
        static CAL: OnceLock<f64> = OnceLock::new();
        *CAL.get_or_init(|| {
            let start = Instant::now();
            // SAFETY: `_rdtsc` is sound on any x86_64 CPU.
            let c0 = unsafe { core::arch::x86_64::_rdtsc() };
            while start.elapsed() < Duration::from_millis(2) {
                std::hint::spin_loop();
            }
            let elapsed_ns = start.elapsed().as_nanos() as f64;
            // SAFETY: `_rdtsc` is sound on any x86_64 CPU.
            let c1 = unsafe { core::arch::x86_64::_rdtsc() };
            let cycles = c1.saturating_sub(c0) as f64;
            // Floor the rate so a degenerate calibration (rdtsc not advancing)
            // can't yield 0 cycles/ns and stall the deadline arithmetic.
            (cycles / elapsed_ns).max(0.001)
        })
    }

    /// `UMONITOR` a private scratch word + `UMWAIT` to `min(recheck,
    /// deadline)`. Wakes on the TSC deadline (or any interrupt); no writer is
    /// expected, so there is no lost-wakeup hazard. Only ever called by the
    /// dispatch seam when the backend is `WaitpkgUmwait` (WAITPKG enumerated).
    pub(super) fn monitor_wait_until(deadline: Instant, recheck: Duration) -> bool {
        let now = Instant::now();
        if now >= deadline {
            // Already past the deadline; nothing to park for. The caller
            // re-polls + re-checks the deadline, so report "performed".
            return true;
        }
        let wait = deadline.saturating_duration_since(now).min(recheck);
        let wait_cycles = (wait.as_nanos() as f64 * tsc_per_ns()) as u64;

        // The watched word lives on this stack frame. We do NOT depend on any
        // writer to it — the wake is the TSC deadline — so there is no
        // lost-wakeup hazard, and the cache line being touched by the setup
        // below at worst returns UMWAIT early (the caller just re-polls/loops).
        let scratch: u32 = 0;
        // SAFETY: `_rdtsc` is sound on x86_64.
        let tsc_deadline = unsafe { core::arch::x86_64::_rdtsc() }.saturating_add(wait_cycles);
        let lo = tsc_deadline as u32;
        let hi = (tsc_deadline >> 32) as u32;
        // control bit[0] = 1 → C0.1, the LOW-LATENCY optimized substate
        // (shallower wake than the C0.2 deep substate selected by 0).
        let control: u32 = 1;

        // SAFETY: WAITPKG is enumerated (the backend was resolved to
        // `WaitpkgUmwait`), so UMONITOR/UMWAIT do not #UD. UMONITOR arms the
        // monitor on the valid stack address in `rax`; UMWAIT (control in ecx,
        // TSC deadline in EDX:EAX) parks the core in C0.1 until the line is
        // written, an interrupt arrives, or TSC >= deadline. UMWAIT writes
        // RFLAGS.CF (timeout vs wake) which we ignore — hence no
        // `preserves_flags` on it.
        unsafe {
            // UMONITOR rax  (f3 0f ae /6, ModRM f0 → r/m = rax)
            core::arch::asm!(
                ".byte 0xf3, 0x0f, 0xae, 0xf0",
                in("rax") &scratch as *const u32,
                options(nostack, preserves_flags),
            );
            // UMWAIT ecx    (f2 0f ae /6, ModRM f1 → r/m = ecx = control)
            core::arch::asm!(
                ".byte 0xf2, 0x0f, 0xae, 0xf1",
                in("ecx") control,
                in("eax") lo,
                in("edx") hi,
                options(nostack),
            );
        }
        true
    }

    /// `UMONITOR` the doorbell `addr` (not a scratch word) so a producer's
    /// store wakes `UMWAIT`, with the DPDK final-poll-then-monitor lost-wakeup
    /// guard (layer 2). `expected` is the CALLER's pre-park doorbell snapshot —
    /// the arm-time re-check compares against it (NOT a fresh internal read) so
    /// a ring that lands between the caller's snapshot and the UMONITOR arm is
    /// detected, not absorbed. Only ever called by the dispatch seam when the
    /// backend is `WaitpkgUmwait`.
    pub(super) fn monitor_wait_until_addr(
        addr: *const u64,
        expected: u64,
        deadline: Instant,
        recheck: Duration,
    ) -> super::AddrParkOutcome {
        let now = Instant::now();
        if now >= deadline {
            // Deadline-boundary reads Parked (pre-existing, bounded — see
            // the AddrParkOutcome doc).
            return super::AddrParkOutcome::Parked;
        }
        let wait = deadline.saturating_duration_since(now).min(recheck);
        let wait_cycles = (wait.as_nanos() as f64 * tsc_per_ns()) as u64;

        // SAFETY: `_rdtsc` is sound on x86_64.
        let tsc_deadline = unsafe { core::arch::x86_64::_rdtsc() }.saturating_add(wait_cycles);
        let lo = tsc_deadline as u32;
        let hi = (tsc_deadline >> 32) as u32;
        let control: u32 = 1; // C0.1 low-latency substate.

        // SAFETY: WAITPKG enumerated (backend resolved to `WaitpkgUmwait`).
        // UMONITOR arms the monitor on the DOORBELL line in `rax`; a producer
        // store to it (or the TSC deadline / an interrupt) wakes UMWAIT.
        unsafe {
            // UMONITOR rax = addr (the doorbell line)
            core::arch::asm!(
                ".byte 0xf3, 0x0f, 0xae, 0xf0",
                in("rax") addr,
                options(nostack, preserves_flags),
            );
        }
        // Layer 2 — lost-wakeup re-check AFTER arming, BEFORE UMWAIT: compare
        // the line against the CALLER's snapshot `expected` (NOT a fresh
        // internal read). If it differs, a ring landed at any point since the
        // caller snapshotted — including the race window between that snapshot
        // and this UMONITOR arm — so skip the park. A fresh internal read here
        // would equal the just-armed value and ABSORB (lose) that ring.
        // SAFETY: caller guarantees `addr` is a valid 8-byte-aligned mapped u64.
        let cur = unsafe { core::ptr::read_volatile(addr) };
        if cur != expected {
            // An arm-window ring is a WAKE in flight, not
            // a completed slice — the caller must not count it.
            return super::AddrParkOutcome::RingPending;
        }
        // SAFETY: control in ecx, TSC deadline in EDX:EAX; parks in C0.1 until
        // the line is written, an interrupt arrives, or TSC >= deadline.
        unsafe {
            core::arch::asm!(
                ".byte 0xf2, 0x0f, 0xae, 0xf1",
                in("ecx") control,
                in("eax") lo,
                in("edx") hi,
                options(nostack),
            );
        }
        super::AddrParkOutcome::Parked
    }
}

#[cfg(all(target_os = "linux", target_arch = "aarch64"))]
mod aarch64 {
    //! # Generic-timer event-stream dependency (timer-park cadence)
    //!
    //! Unlike the x86 backend (which programs an absolute TSC deadline into
    //! `UMWAIT`, so it self-wakes at the deadline), base ARMv8 `WFE` takes NO
    //! deadline — a `WFE` returns on a store to the armed line, an interrupt, OR
    //! the generic-timer EVENT STREAM (`CNTKCTL_EL0.EVNTEN`, a periodic local
    //! event the kernel targets at ~100µs — but the divider is a power of two
    //! of CNTFRQ, so the REAL period is machine-specific: measured ~131µs on a
    //! Jetson Orin during calibration). The event stream is what bounds the
    //! `WFE` so the caller's `recheck`/deadline loop runs even with no
    //! producer store — and because base `WFE` has no timeout operand, a
    //! caller-requested `recheck` below the period is quantized UP to one
    //! period: timer-bounded wake latency (and the park's per-slice yield
    //! cadence) floors at the stream period on this backend. It is kernel
    //! default-ON (`CONFIG_ARM_ARCH_TIMER_EVTSTREAM=y`) and is enabled on the
    //! Jetson/Orin target this backend ships for (verified on a Jetson Orin).
    //!
    //! There is intentionally NO userspace probe + fallback: `CNTKCTL_EL0` is not
    //! EL0-readable (an `mrs` from userspace traps), so the event-stream state
    //! cannot be cheaply queried here. If a board runs with the event stream
    //! DISABLED, the `WFE` still wakes on ordinary interrupts (the scheduler
    //! tick, the doorbell store), so the recheck cadence merely DEGRADES from
    //! ~100µs toward the timer-interrupt cadence (~ms) — bounded, never a hang or
    //! a lost fire (the firewall still reads the actual data from the iceoryx2
    //! queue, and a Period node fires at most one tick late). The ARMv8.7 `WFET`
    //! (deferred, see the module doc) is the future per-`WFE` deadline that would
    //! make this independent of the event stream.
    use super::{Duration, Instant};

    /// One `WFE` cycle per call over a private scratch word; the caller loops
    /// to honor `recheck`/deadline. Only ever called by the dispatch seam when
    /// the backend is `ArmWfeEventStream`.
    pub(super) fn monitor_wait_until(deadline: Instant, recheck: Duration) -> bool {
        // One WFE cycle per call; the caller loops to honor `recheck`/deadline.
        let _ = recheck;
        if Instant::now() >= deadline {
            return true;
        }
        // The kernel `__cmpwait` idiom: arm the local exclusive monitor on a
        // watched word and WFE. The generic-timer event stream (~100µs) bounds
        // the WFE so it returns even with no writer; the caller re-polls + loops.
        let scratch: u32 = 0;
        let expected: u32 = 0;
        // SAFETY: pure register/`WFE` sequence over a valid stack address; no
        // memory is written (LDXR is a load) and no stack is used (`nostack`).
        // WFE is bounded by the event stream so this cannot hang.
        unsafe {
            core::arch::asm!(
                "sevl",
                "wfe",
                "ldxr {tmp:w}, [{addr}]",
                "eor  {tmp:w}, {tmp:w}, {val:w}",
                "cbnz {tmp:w}, 2f",
                "wfe",
                "2:",
                addr = in(reg) &scratch as *const u32,
                val  = in(reg) expected,
                tmp  = out(reg) _,
                options(nostack),
            );
        }
        true
    }

    /// Arm the local exclusive monitor on the DOORBELL `addr` (a 64-bit word)
    /// and WFE, so a producer's store to it clears the monitor and wakes the
    /// WFE. The `ldxr`/`cbnz` compare against the CALLER's pre-park snapshot
    /// `expected` (NOT a fresh internal read) is the lost-wakeup guard (layer
    /// 2) — a ring landing between the caller's snapshot and the arm
    /// short-circuits the second WFE; the event stream still bounds the WFE so
    /// it returns even with no writer. Only ever called by the dispatch seam
    /// when the backend is `ArmWfeEventStream`.
    pub(super) fn monitor_wait_until_addr(
        addr: *const u64,
        expected: u64,
        deadline: Instant,
        recheck: Duration,
    ) -> super::AddrParkOutcome {
        let _ = recheck;
        if Instant::now() >= deadline {
            // Deadline-boundary reads Parked (pre-existing, bounded — see
            // the AddrParkOutcome doc).
            return super::AddrParkOutcome::Parked;
        }
        // SAFETY: arm the exclusive monitor on the doorbell and WFE. `ldxr` is a
        // load; no memory is written. The `cbnz` compares the loaded value
        // against the CALLER's snapshot `expected`; if it already differs (a
        // ring landed since the caller snapshotted) the second WFE is skipped —
        // `tmp` (the eor residue) is read back so that
        // layer-2 skip classifies as RingPending (no park happened). A store
        // to the line clears the monitor and wakes the WFE; the event stream
        // bounds it otherwise.
        // SAFETY: caller guarantees `addr` is a valid 8-byte-aligned mapped u64.
        let armed_mismatch: u64;
        unsafe {
            core::arch::asm!(
                "sevl",
                "wfe",
                "ldxr {tmp}, [{addr}]",
                "eor  {tmp}, {tmp}, {val}",
                "cbnz {tmp}, 2f",
                "wfe",
                "2:",
                addr = in(reg) addr,
                val  = in(reg) expected,
                tmp  = out(reg) armed_mismatch,
                options(nostack),
            );
        }
        if armed_mismatch != 0 {
            super::AddrParkOutcome::RingPending
        } else {
            super::AddrParkOutcome::Parked
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Hermetic vs hardware-only coverage:
    // - Every test below is HERMETIC (runs on any OS, incl. macOS):
    //   they exercise the backend RESOLUTION, the `Unavailable` no-op fallback,
    //   and the platform-independent layer-1 lost-wakeup early-out via the
    //   backend-injection seams — all pure logic, no WAITPKG/WFE hardware.
    // - HARDWARE-ONLY (NOT asserted here): the actual `UMWAIT`/`WFE` wake-on-store
    //   latency and shallow-park behaviour. Those need real x86-WAITPKG /
    //   aarch64 silicon and a producer on another core; they are measured on
    //   that hardware, not in this unit suite.

    #[test]
    fn policy_off_is_all_false_empty_ns() {
        let p = MonitorWaitPolicy::off();
        assert!(!p.monitor_wait());
        assert!(!p.doorbell());
        assert!(p.ns().is_empty());
        assert_eq!(p, MonitorWaitPolicy::default());
    }

    #[test]
    fn policy_clone_and_eq_roundtrip() {
        let p = MonitorWaitPolicy::new(true, true, "alice".to_string());
        assert_eq!(p, p.clone());
        assert_ne!(p, MonitorWaitPolicy::off());
    }

    #[test]
    fn policy_new_coerces_doorbell_implies_monitor_wait() {
        // The illegal state (monitor_wait: false, doorbell: true) is
        // UNREPRESENTABLE — `new` coerces `monitor_wait` true whenever `doorbell`
        // is set, so the invariant `doorbell ⇒ monitor_wait` holds by
        // construction for EVERY caller.
        let p = MonitorWaitPolicy::new(false, true, "ns".to_string());
        assert!(p.monitor_wait(), "doorbell must coerce monitor_wait on");
        assert!(p.doorbell());
        assert_eq!(p.ns(), "ns");

        // monitor_wait without doorbell is untouched (the timer-only park).
        let q = MonitorWaitPolicy::new(true, false, String::new());
        assert!(q.monitor_wait());
        assert!(!q.doorbell());
    }

    /// Drift guard: the shared hardware-park slice must keep a
    /// co-located peer's wait well inside 100 µs (a few slices per same-core
    /// hop) and must never be zero (a zero slice would turn the yield loop
    /// into a busy-spin). The rmw twin pin lives in `guard_wait.rs`
    /// (`the_park_policy_is_the_platform_table`'s `< 50µs` assert reads the
    /// SAME constant through its alias).
    #[test]
    fn park_recheck_slice_is_short_enough_for_a_same_core_peer() {
        assert!(PARK_RECHECK > Duration::ZERO);
        assert!(
            PARK_RECHECK < Duration::from_micros(50),
            "a park slice must be short enough that a co-located peer runs \
             well inside 100 µs (two-to-five slices per same-core hop)"
        );
    }

    #[test]
    fn probe_and_available_do_not_panic() {
        // The CPUID/probe path must never panic on any OS.
        let _ = resolve_backend();
        let _ = current_backend();
        let _ = monitor_wait_available();
    }

    #[test]
    fn available_agrees_with_resolved_backend() {
        // `monitor_wait_available()` is true iff the resolved backend is a real
        // one (NOT `Unavailable`).
        let resolved = resolve_backend();
        let expect_available = resolved != MonitorWaitBackend::Unavailable;
        assert_eq!(monitor_wait_available(), expect_available);
        // And it agrees with the cached backend the public API actually uses.
        assert_eq!(
            monitor_wait_available(),
            current_backend() != MonitorWaitBackend::Unavailable
        );
    }

    #[test]
    fn available_matches_until_performed_flag_on_this_target() {
        // On this exact target the per-call "performed" flag from
        // `monitor_wait_until` must agree with `monitor_wait_available()`
        // (both true on a real backend, both false on `Unavailable`). On a
        // real-backend box this also proves an already-elapsed deadline returns
        // promptly without hanging.
        let avail = monitor_wait_available();
        let performed = monitor_wait_until(Instant::now(), Duration::from_micros(50));
        assert_eq!(avail, performed);
    }

    #[test]
    fn injected_unavailable_until_is_noop_false_and_does_not_block() {
        // Inject `Unavailable` (the fallback contract): `monitor_wait_until`
        // must return `false` immediately and NOT park/spin, regardless of how
        // far in the future the deadline is.
        let deadline = Instant::now() + Duration::from_secs(3600);
        let started = Instant::now();
        let performed = monitor_wait_until_with_backend(
            MonitorWaitBackend::Unavailable,
            deadline,
            Duration::from_secs(1),
        );
        assert!(!performed, "Unavailable backend must no-op to false");
        assert!(
            started.elapsed() < Duration::from_millis(500),
            "Unavailable fallback must not block (returned in {:?})",
            started.elapsed()
        );
    }

    #[test]
    fn injected_unavailable_addr_is_noop_false_and_does_not_block() {
        // Same fallback contract for the addr variant. `Unavailable` returns
        // before touching `addr`, so it must no-op to `false` without blocking
        // even though `expected` does NOT match the (irrelevant) word value.
        let word: u64 = 7;
        let deadline = Instant::now() + Duration::from_secs(3600);
        let started = Instant::now();
        let outcome = monitor_wait_until_addr_with_backend(
            MonitorWaitBackend::Unavailable,
            &word as *const u64,
            999, // deliberately != word — must be ignored on the no-op path
            deadline,
            Duration::from_secs(1),
        );
        assert_eq!(
            outcome,
            AddrParkOutcome::Unavailable,
            "Unavailable backend must no-op (the caller takes its sleep path)"
        );
        assert!(!outcome.performed() && !outcome.parked());
        assert!(
            started.elapsed() < Duration::from_millis(500),
            "Unavailable fallback must not block (returned in {:?})",
            started.elapsed()
        );
    }

    #[test]
    fn injected_real_backend_with_premoved_addr_returns_immediately() {
        // The layer-1 lost-wakeup EARLY-OUT (pure logic, no hardware): inject a
        // REAL backend but pre-move the doorbell so the observed word differs
        // from the caller's `expected` snapshot. The dispatch must detect the
        // already-landed ring and return `true` immediately WITHOUT reaching
        // the (target-gated) UMWAIT/WFE asm — so this runs hermetically on any
        // OS, incl. macOS where neither asm backend is compiled.
        let word: u64 = 42; // current doorbell value
        let expected: u64 = 41; // caller's older snapshot → a ring already landed
        let deadline = Instant::now() + Duration::from_secs(3600);
        let started = Instant::now();
        // Use BOTH real backends to pin that the early-out precedes dispatch on
        // either target's variant.
        for backend in [
            MonitorWaitBackend::WaitpkgUmwait,
            MonitorWaitBackend::ArmWfeEventStream,
        ] {
            let outcome = monitor_wait_until_addr_with_backend(
                backend,
                &word as *const u64,
                expected,
                deadline,
                Duration::from_secs(1),
            );
            assert_eq!(
                outcome,
                AddrParkOutcome::RingPending,
                "a pre-moved doorbell must classify as RingPending — the \
                 lost-wakeup fast path performs NO park, and the caller must \
                 not count a slice"
            );
            assert!(outcome.performed() && !outcome.parked());
        }
        assert!(
            started.elapsed() < Duration::from_millis(500),
            "the early-out must not park (returned in {:?})",
            started.elapsed()
        );
    }

    /// The outcome→caller mapping truth table. `performed`
    /// gates the sleep fallback (everything but Unavailable skips it);
    /// `parked` gates slice telemetry (only a real completed wait counts).
    #[test]
    fn addr_park_outcome_mapping_oracle() {
        use AddrParkOutcome::*;
        assert!(Parked.performed() && Parked.parked());
        assert!(RingPending.performed() && !RingPending.parked());
        assert!(!Unavailable.performed() && !Unavailable.parked());
    }

    // ================= The degraded-park NAP tests =================
    // Every wall assertion is a LOWER bound (a timed wait/sleep can only
    // lengthen under load — the load-safe direction) or a bound in SECONDS,
    // never in units of the recheck (the mac timer-coalescing lesson).

    /// Off macOS the nap IS `thread::sleep`: Linux behavior is
    /// byte-unchanged by construction, and this pins the wrapper's bound +
    /// mechanism attribution on those targets.
    #[cfg(not(target_os = "macos"))]
    #[test]
    fn park_nap_off_macos_is_a_bounded_sleep() {
        let cap = Duration::from_micros(500);
        let started = Instant::now();
        let mech = park_nap(cap);
        assert_eq!(
            mech,
            ParkNapMechanism::Sleep,
            "off macOS the nap is a sleep"
        );
        assert!(
            started.elapsed() >= cap,
            "the nap must block at least its cap (got {:?})",
            started.elapsed()
        );
    }

    /// On macOS (hermetic on ANY macOS): the ABSENT-backend fallback
    /// arm — the macOS < 14.4 shape, unreachable on macOS ≥ 14.4 except
    /// through the injection seam — is today's sleep: bounded, attributed
    /// `Sleep`, never a busy-spin.
    #[cfg(target_os = "macos")]
    #[test]
    fn park_nap_injected_absent_backend_sleeps_and_reports_sleep() {
        let cap = Duration::from_micros(500);
        let started = Instant::now();
        let mech = macos_nap::park_nap_with_backend(None, cap);
        assert_eq!(
            mech,
            ParkNapMechanism::Sleep,
            "an absent os_sync backend (macOS < 14.4) must take the sleep fallback"
        );
        assert!(
            started.elapsed() >= cap,
            "the fallback must block at least its cap (got {:?})",
            started.elapsed()
        );
    }

    /// On macOS: WHEN the dlsym backend resolved on this host, the nap
    /// rides the os_sync timed wait — attributed `OsSyncTimedWait` and blocked
    /// at least its cap. Signal-proof: a benign
    /// early return (EINTR / a spurious wake) RE-ARMS for the remainder, so
    /// `elapsed >= cap` holds unconditionally (the re-arm itself is pinned
    /// hermetically by the fake-backend tests below). Injects the REAL backend
    /// through the seam so the pin is independent of the kill-switch/latch
    /// `OnceLock`s. On macOS < 14.4 prints a skip (host truth, not a failure).
    #[cfg(target_os = "macos")]
    #[test]
    fn park_nap_on_the_resolved_backend_reports_os_sync_and_waits_at_least_cap() {
        let Some(backend) = crate::os_sync::os_sync_backend() else {
            eprintln!("skipping: os_sync backend absent on this host (macOS < 14.4)");
            return;
        };
        let cap = Duration::from_micros(500);
        let started = Instant::now();
        let mech = macos_nap::park_nap_with_backend(Some(backend), cap);
        assert_eq!(
            mech,
            ParkNapMechanism::OsSyncTimedWait,
            "with the backend resolved the nap must ride the os_sync timed wait"
        );
        assert!(
            started.elapsed() >= cap,
            "the os_sync nap must block at least its cap (got {:?})",
            started.elapsed()
        );
    }

    /// A ZERO cap returns promptly (no kernel call to EINVAL, no
    /// block) — the deadline-edge shape the runtime call site also skips
    /// counting. Bound in SECONDS (load-safe), never in recheck units.
    #[test]
    fn park_nap_zero_cap_returns_promptly() {
        let started = Instant::now();
        let mech = park_nap(Duration::ZERO);
        assert_eq!(
            mech,
            ParkNapMechanism::Sleep,
            "a zero-cap nap ran no kernel block and attributes Sleep"
        );
        assert!(
            started.elapsed() < Duration::from_secs(1),
            "a zero-cap nap must not block (returned in {:?})",
            started.elapsed()
        );
    }

    /// `park_nap_os_sync_available` agrees with the resolved backend
    /// on this host when nothing has latched or killed the tier (nothing in
    /// this binary can latch the family — the latch is only set on a real
    /// kernel EINVAL/ENOTSUP; the error-arm tests below flip LOCAL latches).
    /// Off macOS it is unconditionally false. An AMBIENT
    /// `CERULION_PARK_OS_SYNC` in the invoking shell would legitimately flip
    /// the availability (the kill switch working as designed, cached in a
    /// `OnceLock` no `EnvVarGuard` can reach), so the macOS arm SKIPS loudly
    /// when the var is exported rather than failing on an unrelated claim
    /// (the CERULION_STATE_ARM_TAG isolation class).
    #[test]
    fn park_nap_availability_agrees_with_the_resolved_backend() {
        #[cfg(target_os = "macos")]
        {
            if std::env::var_os("CERULION_PARK_OS_SYNC").is_some() {
                eprintln!(
                    "skipping: CERULION_PARK_OS_SYNC is exported in this shell — the \
                     availability predicate is then a function of the operator's choice, \
                     not of backend presence"
                );
                return;
            }
            assert_eq!(
                park_nap_os_sync_available(),
                crate::os_sync::os_sync_backend().is_some(),
                "with no kill switch and no latch, availability IS backend presence"
            );
        }
        #[cfg(not(target_os = "macos"))]
        {
            assert!(
                !park_nap_os_sync_available(),
                "the os_sync nap tier exists only on macOS"
            );
        }
    }

    /// `=0` resolves the park-nap kill switch to DISABLED (true)
    /// SILENTLY — an explicit, supported choice, not a mistake. Drives the
    /// extracted resolver directly (mirrors the barrier's
    /// `resolve_os_sync_disabled_zero_disables_silently`).
    #[cfg(target_os = "macos")]
    #[tracing_test::traced_test]
    #[test]
    fn resolve_park_os_sync_disabled_zero_disables_silently() {
        assert!(
            macos_nap::resolve_park_os_sync_disabled(Some("0")),
            "`=0` must disable the park-nap os_sync tier"
        );
        assert!(
            !logs_contain("CERULION_PARK_OS_SYNC"),
            "the kill switch is a supported choice and must resolve silently"
        );
    }

    /// Garbage keeps the tier ON (false) AND emits the loud warn
    /// naming THIS env var (house rule: loud over silent; the warn must name
    /// `CERULION_PARK_OS_SYNC`, never the barrier's switch — the env name is
    /// operator-facing surface).
    #[cfg(target_os = "macos")]
    #[tracing_test::traced_test]
    #[test]
    fn resolve_park_os_sync_disabled_garbage_keeps_on_and_warns() {
        assert!(
            !macos_nap::resolve_park_os_sync_disabled(Some("nope")),
            "a garbage value must keep the os_sync nap tier ON (the default)"
        );
        assert!(
            logs_contain("keeping the macOS os_sync park-nap tier ON"),
            "a garbage value must emit the loud keep-on warn"
        );
        assert!(
            logs_contain("CERULION_PARK_OS_SYNC"),
            "the warn must name the park nap's OWN env var"
        );
    }

    /// Unset (`None`) and the explicit `=1` both resolve to ENABLED
    /// (false) SILENTLY — the default-on happy path emits no warn.
    #[cfg(target_os = "macos")]
    #[tracing_test::traced_test]
    #[test]
    fn resolve_park_os_sync_disabled_unset_and_one_are_enabled_silent() {
        assert!(
            !macos_nap::resolve_park_os_sync_disabled(None),
            "unset keeps the os_sync nap tier ON"
        );
        assert!(
            !macos_nap::resolve_park_os_sync_disabled(Some("1")),
            "`=1` keeps the os_sync nap tier ON"
        );
        assert!(
            !logs_contain("CERULION_PARK_OS_SYNC"),
            "the default-on happy path must resolve silently"
        );
    }

    /// FAKE os_sync backends — DI doubles through the REAL
    /// `park_nap_with_backend_and_latch` seam (the spy-plane pattern, not
    /// fabricated data): each `wait` fn pointer scripts a kernel answer the
    /// real kernel never gives a healthy test process, which is the only way
    /// the benign-re-arm and error arms are reachable hermetically. Each
    /// scripted "first call" flag is used by exactly ONE test, so parallel
    /// libtest cannot interleave them.
    #[cfg(target_os = "macos")]
    mod fake_backends {
        use crate::os_sync::OsSyncFns;
        use core::ffi::{c_int, c_void};
        use std::sync::atomic::{AtomicBool, Ordering};

        pub(super) unsafe extern "C" fn wake_noop(_a: *mut c_void, _s: usize, _f: u32) -> c_int {
            0
        }

        fn set_errno(errno: i32) {
            // SAFETY: `__error()` returns this thread's errno slot on macOS —
            // writing it is exactly what a failing syscall does.
            unsafe { *libc::__error() = errno };
        }

        /// A REAL timed wait's shape: block the requested timeout, then report
        /// ETIMEDOUT. errno is set AFTER the sleep (nanosleep may clobber it).
        fn block_timeout_then_etimedout(timeout_ns: u64) -> c_int {
            std::thread::sleep(std::time::Duration::from_nanos(timeout_ns));
            set_errno(libc::ETIMEDOUT);
            -1
        }

        static SPURIOUS_FIRED: AtomicBool = AtomicBool::new(false);
        unsafe extern "C" fn wait_spurious_once(
            _addr: *mut c_void,
            _value: u64,
            _size: usize,
            _flags: u32,
            _clock: u32,
            timeout_ns: u64,
        ) -> c_int {
            if !SPURIOUS_FIRED.swap(true, Ordering::Relaxed) {
                return 0; // a spurious wake, immediately — Apple documents these
            }
            block_timeout_then_etimedout(timeout_ns)
        }
        /// First call: a SPURIOUS wake (rc 0, instantly). Later calls: a real
        /// timed wait.
        pub(super) static SPURIOUS_ONCE: OsSyncFns = OsSyncFns {
            wait: wait_spurious_once,
            wake: wake_noop,
        };

        static EINTR_FIRED: AtomicBool = AtomicBool::new(false);
        unsafe extern "C" fn wait_eintr_once(
            _addr: *mut c_void,
            _value: u64,
            _size: usize,
            _flags: u32,
            _clock: u32,
            timeout_ns: u64,
        ) -> c_int {
            if !EINTR_FIRED.swap(true, Ordering::Relaxed) {
                set_errno(libc::EINTR);
                return -1; // a signal landed, immediately
            }
            block_timeout_then_etimedout(timeout_ns)
        }
        /// First call: EINTR, instantly. Later calls: a real timed wait.
        pub(super) static EINTR_ONCE: OsSyncFns = OsSyncFns {
            wait: wait_eintr_once,
            wake: wake_noop,
        };

        unsafe extern "C" fn wait_einval(
            _addr: *mut c_void,
            _value: u64,
            _size: usize,
            _flags: u32,
            _clock: u32,
            _timeout_ns: u64,
        ) -> c_int {
            set_errno(libc::EINVAL);
            -1
        }
        /// Every call: EINVAL (the primitive is unusable on this host).
        pub(super) static ALWAYS_EINVAL: OsSyncFns = OsSyncFns {
            wait: wait_einval,
            wake: wake_noop,
        };

        unsafe extern "C" fn wait_unexpected_errnos(
            _addr: *mut c_void,
            _value: u64,
            _size: usize,
            _flags: u32,
            _clock: u32,
            _timeout_ns: u64,
        ) -> c_int {
            // Calls 1+2: EFAULT; call 3+: EPERM — two DISTINCT errnos that are
            // neither benign nor unrecoverable, driving the middle arm's
            // warn-once-per-distinct-errno dedup. (The middle arm returns after
            // ONE failure, so each park_nap call sees exactly one of these.)
            static CALLS: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
            let call = CALLS.fetch_add(1, Ordering::Relaxed);
            set_errno(if call < 2 { libc::EFAULT } else { libc::EPERM });
            -1
        }
        /// Calls 1+2: EFAULT; later calls: EPERM (the middle arm's dedup pair).
        pub(super) static UNEXPECTED_ERRNOS: OsSyncFns = OsSyncFns {
            wait: wait_unexpected_errnos,
            wake: wake_noop,
        };
    }

    /// The re-arm contract, spurious half: a
    /// SPURIOUS wake must not end the nap — the seam re-arms for the
    /// remaining cap, so ONE `park_nap` call fills its whole cap exactly like
    /// `thread::sleep`. A variant that returns on any rc >= 0
    /// comes back instantly (elapsed ~0 « cap) and fails the lower bound.
    /// Lower bound only — load can only lengthen a wait (load-safe).
    #[cfg(target_os = "macos")]
    #[test]
    fn park_nap_rearms_after_a_spurious_wake_and_fills_its_cap() {
        let latch = std::sync::atomic::AtomicBool::new(false);
        let cap = Duration::from_micros(500);
        let started = Instant::now();
        let mech = macos_nap::park_nap_with_backend_and_latch(
            Some(&fake_backends::SPURIOUS_ONCE),
            cap,
            &latch,
        );
        assert_eq!(
            mech,
            ParkNapMechanism::OsSyncTimedWait,
            "a spurious wake is a benign kernel answer — the nap stays on the os_sync tier"
        );
        assert!(
            started.elapsed() >= cap,
            "a spurious wake must be re-armed so the nap fills its cap (got {:?})",
            started.elapsed()
        );
        assert!(
            !latch.load(std::sync::atomic::Ordering::Relaxed),
            "a benign return must never touch the family latch"
        );
    }

    /// The re-arm contract, EINTR half: a signal
    /// landing mid-nap must not end it — `thread::sleep` retries EINTR
    /// internally and the os_sync tier must match, or every nap-counting
    /// oracle (the `naps_single == 1` single-park pin) flakes on any
    /// signal delivery (profiler attach, SIGCHLD from a sibling harness).
    #[cfg(target_os = "macos")]
    #[test]
    fn park_nap_rearms_after_eintr_and_fills_its_cap() {
        let latch = std::sync::atomic::AtomicBool::new(false);
        let cap = Duration::from_micros(500);
        let started = Instant::now();
        let mech = macos_nap::park_nap_with_backend_and_latch(
            Some(&fake_backends::EINTR_ONCE),
            cap,
            &latch,
        );
        assert_eq!(
            mech,
            ParkNapMechanism::OsSyncTimedWait,
            "EINTR is a benign kernel answer — the nap stays on the os_sync tier"
        );
        assert!(
            started.elapsed() >= cap,
            "EINTR must be re-armed so the nap fills its cap (got {:?})",
            started.elapsed()
        );
        assert!(
            !latch.load(std::sync::atomic::Ordering::Relaxed),
            "a benign return must never touch the family latch"
        );
    }

    /// The unrecoverable arm:
    /// an EINVAL/ENOTSUP from the kernel wait LATCHES the family (via the
    /// INJECTED latch — a local `AtomicBool`, so the process-global one every
    /// sibling activity gate reads stays untouched), finishes THIS nap's
    /// remainder on the sleep fallback (bounded — a deleted sleep returns
    /// instantly and fails the lower bound), attributes `Sleep`, and warns
    /// EXACTLY once (the latch flip is the warn-once gate: a second failing
    /// nap on an already-flipped latch degrades silently).
    #[cfg(target_os = "macos")]
    #[tracing_test::traced_test]
    #[test]
    fn an_unrecoverable_errno_latches_finishes_on_sleep_and_warns_once() {
        let latch = std::sync::atomic::AtomicBool::new(false);
        let cap = Duration::from_micros(300);

        let started = Instant::now();
        let mech = macos_nap::park_nap_with_backend_and_latch(
            Some(&fake_backends::ALWAYS_EINVAL),
            cap,
            &latch,
        );
        assert_eq!(
            mech,
            ParkNapMechanism::Sleep,
            "an unusable primitive falls back to sleep"
        );
        assert!(
            started.elapsed() >= cap,
            "the fallback must still fill the nap's cap (got {:?})",
            started.elapsed()
        );
        assert!(
            latch.load(std::sync::atomic::Ordering::Relaxed),
            "EINVAL must latch the (injected) family latch"
        );

        // A SECOND failing nap on the already-flipped latch: still Sleep,
        // still bounded, but SILENT (later sites degrade without re-warning).
        let started = Instant::now();
        let mech = macos_nap::park_nap_with_backend_and_latch(
            Some(&fake_backends::ALWAYS_EINVAL),
            cap,
            &latch,
        );
        assert_eq!(mech, ParkNapMechanism::Sleep);
        assert!(started.elapsed() >= cap);

        logs_assert(|lines: &[&str]| {
            let warns = lines
                .iter()
                .filter(|l| l.contains("unrecoverable errno"))
                .count();
            if warns == 1 {
                Ok(())
            } else {
                Err(format!(
                    "expected exactly 1 unrecoverable-errno warn, got {warns}"
                ))
            }
        });
    }

    /// The middle arm: an
    /// errno that is neither benign nor unrecoverable sleeps the bounded
    /// remainder (the anti-busy-spin defense — a deleted sleep would let a
    /// persistently-failing syscall spin the recheck loop at 100% CPU),
    /// attributes `Sleep`, never touches the latch, and warns once per
    /// DISTINCT errno (EFAULT, EFAULT, EPERM ⇒ 2 warns). ONE test body — the
    /// dedup rides a process-global `LAST_WARNED_ERRNO` static, so splitting
    /// these calls across tests would interleave under parallel libtest.
    #[cfg(target_os = "macos")]
    #[tracing_test::traced_test]
    #[test]
    fn an_unexpected_errno_sleeps_the_remainder_and_warns_once_per_distinct_errno() {
        let latch = std::sync::atomic::AtomicBool::new(false);
        let cap = Duration::from_micros(300);

        for _ in 0..2 {
            // EFAULT twice: the second is deduped (same errno as last warned).
            let started = Instant::now();
            let mech = macos_nap::park_nap_with_backend_and_latch(
                Some(&fake_backends::UNEXPECTED_ERRNOS),
                cap,
                &latch,
            );
            assert_eq!(
                mech,
                ParkNapMechanism::Sleep,
                "the middle arm falls back to sleep"
            );
            assert!(
                started.elapsed() >= cap,
                "the middle arm must sleep the bounded remainder — a return without \
                 the sleep is the busy-spin the arm exists to prevent (got {:?})",
                started.elapsed()
            );
        }
        // EPERM: a DISTINCT unexpected errno warns again.
        let mech = macos_nap::park_nap_with_backend_and_latch(
            Some(&fake_backends::UNEXPECTED_ERRNOS),
            cap,
            &latch,
        );
        assert_eq!(mech, ParkNapMechanism::Sleep);
        assert!(
            !latch.load(std::sync::atomic::Ordering::Relaxed),
            "the middle arm must never latch the family (the errno is not EINVAL/ENOTSUP)"
        );

        logs_assert(|lines: &[&str]| {
            let warns = lines
                .iter()
                .filter(|l| l.contains("unexpected errno"))
                .count();
            if warns == 2 {
                Ok(())
            } else {
                Err(format!(
                    "expected exactly 2 unexpected-errno warns (EFAULT once + EPERM once, \
                     the EFAULT repeat deduped), got {warns}"
                ))
            }
        });
    }

    /// The kill-switch child probe. A NORMAL run (env
    /// unset) is a no-op pass; the parent test below re-invokes THIS binary
    /// with `--exact` + `EXPECT_MECH` and asserts the exit status
    /// (the `shm_ring_test::cross_process_child_entrypoint` pattern). A
    /// SUBPROCESS because the kill switch is cached in a per-process
    /// `OnceLock` at first read, so an in-process env A/B is impossible.
    #[cfg(target_os = "macos")]
    #[test]
    // Logging-rule exemption (Principle 12), scoped to this fn rather than the file: this is the body
    // of a SELF-RE-EXEC CHILD process — a process entrypoint by construction,
    // whose exit code IS the channel the parent reads its verdict from (the
    // `shm_ring_test::cross_process_child_entrypoint` pattern). The ban stays
    // armed for every other line in this binary.
    #[allow(clippy::disallowed_methods)]
    fn subprocess_child_park_nap_probe() {
        let expect = match std::env::var("EXPECT_MECH") {
            Ok(v) => v,
            Err(_) => return, // normal run — not the child invocation
        };
        // The PRODUCTION nap path (`monitor_wait::park_nap`), not the seam:
        // the kill switch is consulted inside `macos_nap::park_nap`, which is
        // exactly the gate removing that guard leaves open.
        let got = format!("{:?}", park_nap(Duration::from_micros(300)));
        eprintln!(
            "[kill-switch child] park_nap -> {got} (expected {expect}; \
             CERULION_PARK_OS_SYNC={:?})",
            std::env::var("CERULION_PARK_OS_SYNC").ok()
        );
        std::process::exit(if got == expect { 0 } else { 2 });
    }

    /// Deleting `|| park_os_sync_kill_switch()` from
    /// `macos_nap::park_nap` compiled (the predicate still references the
    /// fn) and passed every test — `CERULION_PARK_OS_SYNC=0` could ship INERT
    /// on the production nap path while the availability predicate (and the
    /// tier log) claimed it worked: the inert-env-knob class, where
    /// the control is tested at one layer and consulted at another. This pin
    /// drives the env through a REAL subprocess (the switch is a per-process
    /// `OnceLock`): the KILL arm must nap on `Sleep`, and the CONTROL arm
    /// (env removed) must nap on `OsSyncTimedWait` — the anti-tautology half,
    /// which also proves the child apparatus resolves the tier at all.
    /// Runtime-gated on backend presence (macOS < 14.4 has no tier to kill).
    #[cfg(target_os = "macos")]
    #[test]
    fn the_kill_switch_forces_the_sleep_tier_on_the_production_nap_path() {
        if crate::os_sync::os_sync_backend().is_none() {
            eprintln!("skipping: os_sync backend absent on this host (macOS < 14.4)");
            return;
        }
        fn run_child(kill_switch: Option<&str>, expect_mech: &str) {
            let exe = std::env::current_exe().expect("current_exe");
            let mut cmd = std::process::Command::new(exe);
            cmd.args([
                "--exact",
                "monitor_wait::tests::subprocess_child_park_nap_probe",
                "--test-threads=1",
                "--nocapture",
            ])
            .env("EXPECT_MECH", expect_mech)
            .stdout(std::process::Stdio::inherit())
            .stderr(std::process::Stdio::inherit());
            match kill_switch {
                Some(v) => cmd.env("CERULION_PARK_OS_SYNC", v),
                None => cmd.env_remove("CERULION_PARK_OS_SYNC"),
            };
            let mut child = cmd.spawn().expect("spawn kill-switch child");
            // BOUNDED wait (SIGKILL + reap on timeout so a hung child never
            // hangs the suite): the child naps 300µs; 30s is a liveness
            // ceiling in SECONDS, never a wall in recheck units.
            let deadline = Instant::now() + Duration::from_secs(30);
            let status = loop {
                match child.try_wait() {
                    Ok(Some(status)) => break status,
                    Ok(None) if Instant::now() < deadline => {
                        std::thread::sleep(Duration::from_millis(20));
                    }
                    _ => {
                        let _ = child.kill();
                        let _ = child.wait();
                        panic!("kill-switch child hung past its 30s liveness ceiling");
                    }
                }
            };
            assert!(
                status.success(),
                "kill-switch child (CERULION_PARK_OS_SYNC={kill_switch:?}, expecting \
                 {expect_mech}) failed with exit {status:?} — see its stderr above"
            );
        }
        // With the switch set, the production nap must be a
        // plain sleep; reverting the guard leaves the child napping on
        // OsSyncTimedWait and exits 2.
        run_child(Some("0"), "Sleep");
        // Anti-tautology control: with the switch REMOVED the same child on
        // the same host rides the os_sync tier (proves the apparatus can see
        // the tier, so the kill arm's "Sleep" is not vacuous).
        run_child(None, "OsSyncTimedWait");
    }
}

#[cfg(test)]
mod knob_classifier_tests {
    use super::*;

    /// The shared "1"/"0" flag ladder, pinned at every arm — exact match,
    /// no trimming, empty = unset.
    #[test]
    fn classify_env_flag_oracle_vectors() {
        use EnvFlag::*;
        assert_eq!(classify_env_flag(None), Auto);
        assert_eq!(classify_env_flag(Some("")), Auto);
        assert_eq!(classify_env_flag(Some("1")), ForceOn);
        assert_eq!(classify_env_flag(Some("0")), ForceOff);
        for near in ["true", "on", "yes", " 1", "01", "2"] {
            assert_eq!(classify_env_flag(Some(near)), NearMiss, "{near:?}");
        }
    }

    /// The shared live-spin classification, pinned on BOTH sides of the
    /// clamp threshold and at every arm. This is the ONE parse the native
    /// live loop and the rmw wait both read — their drift protection is
    /// this function having exactly one definition.
    #[test]
    fn classify_live_spin_us_oracle_vectors() {
        use LiveSpinSetting::*;
        assert_eq!(classify_live_spin_us(None), Derived);
        assert_eq!(classify_live_spin_us(Some("")), Derived);
        assert_eq!(classify_live_spin_us(Some("0")), Disabled);
        assert_eq!(
            classify_live_spin_us(Some("50")),
            Ceiling {
                us: 50,
                clamped: false
            }
        );
        // The cap itself is admitted (a maximum, not an exclusive bound)…
        assert_eq!(
            classify_live_spin_us(Some(&SPIN_BUDGET_MAX_US.to_string())),
            Ceiling {
                us: SPIN_BUDGET_MAX_US,
                clamped: false
            }
        );
        // …and one past it is clamped AND reported.
        assert_eq!(
            classify_live_spin_us(Some(&(SPIN_BUDGET_MAX_US + 1).to_string())),
            Ceiling {
                us: SPIN_BUDGET_MAX_US,
                clamped: true
            }
        );
        // u64::MAX (the "584,000-year spin" input) clamps too.
        assert_eq!(
            classify_live_spin_us(Some("18446744073709551615")),
            Ceiling {
                us: SPIN_BUDGET_MAX_US,
                clamped: true
            }
        );
        // Malformed: strict integer parse — no trimming, no suffixes, no
        // exponents, negative, overflow.
        for bad in ["abc", "-5", "1e3", "100us", " 100", "18446744073709551616"] {
            assert_eq!(classify_live_spin_us(Some(bad)), Malformed, "{bad:?}");
        }
    }
}
