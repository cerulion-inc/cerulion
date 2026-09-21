// SPDX-License-Identifier: AGPL-3.0-only
//! Fault-injection seams for the conformance tests — compiled ONLY under the
//! `test-seams` feature (turned on by this crate's own dev-dependency
//! self-reference, the repo's `test-helpers`/`test-seams` idiom; in no
//! shipping recipe — a hand-built `--features test-seams` cdylib IS
//! deployable, which is why its C++ bypass warns loudly on the first
//! resolve and at each decade of the running total, see
//! `era.rs`). Integration tests link the non-test rlib and cannot see
//! `cfg(test)` items, hence a feature rather than `cfg(test)`.
//!
//! Every seam is DISARMED by default and armed only while an RAII guard is
//! held, so an armed seam can never leak into a sibling test (the guard
//! disarms on drop, unwind included).

use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};

static PANIC_AFTER_PUBLISHER_TRANSPORT_CREATE: AtomicBool = AtomicBool::new(false);
static PUBLISHER_CREATE_PANICS_FIRED: AtomicU64 = AtomicU64::new(0);

/// The injected panic's payload. A test asserts the seam FIRED (via
/// [`publisher_create_panics_fired`]) so a null handle is attributable to the
/// injection and not to some unrelated create failure.
pub const PUBLISHER_CREATE_PANIC_MSG: &str =
    "test-seam: injected panic after publisher transport creation";

/// RAII arm of the publisher-create seam: while held, `rmw_create_publisher`
/// panics right after its TRANSPORT publisher exists and before any later
/// construction step — the shape "a later step failed after the SHM port was
/// created" that the register-last ordering in `rmw_create_publisher`
/// defends. Dropped (normally or by unwind) ⇒ disarmed.
#[must_use = "the seam is armed only while the guard is held"]
pub struct PublisherCreatePanicGuard(());

impl PublisherCreatePanicGuard {
    /// Arm the seam for the lifetime of the returned guard.
    pub fn arm() -> Self {
        assert!(
            !PANIC_AFTER_PUBLISHER_TRANSPORT_CREATE.swap(true, Ordering::SeqCst),
            "the publisher-create seam is already armed — overlapping guards would disarm each other"
        );
        Self(())
    }
}

impl Drop for PublisherCreatePanicGuard {
    fn drop(&mut self) {
        PANIC_AFTER_PUBLISHER_TRANSPORT_CREATE.store(false, Ordering::SeqCst);
    }
}

/// How many times the publisher-create seam has fired in this process
/// (Principle #3 — the observable a test compares before/after so the pin
/// cannot pass on a create that failed for some OTHER reason).
pub fn publisher_create_panics_fired() -> u64 {
    PUBLISHER_CREATE_PANICS_FIRED.load(Ordering::SeqCst)
}

/// The seam itself — called by `rmw_create_publisher` at the point named in
/// [`PublisherCreatePanicGuard`]. A no-op unless armed.
pub(crate) fn maybe_panic_after_publisher_transport_create() {
    if PANIC_AFTER_PUBLISHER_TRANSPORT_CREATE.load(Ordering::SeqCst) {
        PUBLISHER_CREATE_PANICS_FIRED.fetch_add(1, Ordering::SeqCst);
        panic!("{PUBLISHER_CREATE_PANIC_MSG}");
    }
}

// ---------------------------------------------------------------------
// Guard-race seam (the LOST_WAKE window): fire ONE extra trigger BETWEEN
// the readiness probe and the consuming pass of `rmw_wait`'s fired-only
// path — exactly where a blanket flag-clear would erase it.
// ---------------------------------------------------------------------

static TRIGGER_GUARD_BETWEEN_PROBE_AND_CONSUME: AtomicUsize = AtomicUsize::new(0);
static GUARD_RACE_TRIGGERS_FIRED: AtomicU64 = AtomicU64::new(0);

/// RAII arm of the guard-race seam: while held, the FIRST `rmw_wait`
/// iteration that found readiness fires one extra `trigger()` on the given
/// guard between the probe and the consuming pass (self-clearing — it
/// fires once). The pin then proves the late trigger SURVIVES to the next
/// call (Principle #6): probe-and-clear is one atomic swap and the consume
/// never blanket-clears.
#[must_use = "the seam is armed only while the guard is held"]
pub struct GuardRaceTriggerGuard(());

impl GuardRaceTriggerGuard {
    /// Arm the seam. `state` must outlive the returned guard (the test
    /// owns the guard condition it points at).
    pub fn arm(state: *const crate::runtime::GuardConditionState) -> Self {
        TRIGGER_GUARD_BETWEEN_PROBE_AND_CONSUME.store(state as usize, Ordering::SeqCst);
        Self(())
    }
}

impl Drop for GuardRaceTriggerGuard {
    fn drop(&mut self) {
        TRIGGER_GUARD_BETWEEN_PROBE_AND_CONSUME.store(0, Ordering::SeqCst);
    }
}

/// How many times the guard-race seam has fired in this process — the
/// observable that makes the pin attributable to the injected window.
pub fn guard_race_triggers_fired() -> u64 {
    GUARD_RACE_TRIGGERS_FIRED.load(Ordering::SeqCst)
}

/// The seam itself — called by `rmw_wait` between `probe_ready_into` and
/// `consume_from_mask`. A no-op unless armed; fires ONCE per arming.
pub(crate) fn maybe_trigger_guard_between_probe_and_consume() {
    let p = TRIGGER_GUARD_BETWEEN_PROBE_AND_CONSUME.swap(0, Ordering::SeqCst);
    if p != 0 {
        // SAFETY: the armer guarantees the pointee outlives the RAII guard,
        // and the swap above makes this a once-only fire.
        unsafe { (*(p as *const crate::runtime::GuardConditionState)).trigger() };
        GUARD_RACE_TRIGGERS_FIRED.fetch_add(1, Ordering::SeqCst);
    }
}

// ── The adopt-take forge-window panic ──────────────────

static PANIC_IN_ADOPT_FORGE_WINDOW: AtomicBool = AtomicBool::new(false);
static ADOPT_FORGE_WINDOW_PANICS_FIRED: AtomicU64 = AtomicU64::new(0);

/// The injected panic's payload — a test asserts the seam FIRED (via
/// [`adopt_forge_window_panics_fired`]) so an `RMW_RET_ERROR` is
/// attributable to the injection, not some unrelated failure.
pub const ADOPT_FORGE_WINDOW_PANIC_MSG: &str =
    "test-seam: injected panic inside the adopt-take forge window";

/// RAII arm of the adopt-take forge-window seam: while held, `take_adopted`
/// panics AFTER `unflatten_forged` installed SHM pointers in the caller's
/// message and BEFORE any range is registered — the exact window whose
/// unwind the `ForgedMessageGuard` must leave fini-safe. Dropped (normally
/// or by unwind) ⇒ disarmed.
#[must_use = "the seam is armed only while the guard is held"]
pub struct AdoptForgeWindowPanicGuard(());

impl AdoptForgeWindowPanicGuard {
    /// Arm the seam for the lifetime of the returned guard.
    pub fn arm() -> Self {
        PANIC_IN_ADOPT_FORGE_WINDOW.store(true, Ordering::SeqCst);
        Self(())
    }
}

impl Drop for AdoptForgeWindowPanicGuard {
    fn drop(&mut self) {
        PANIC_IN_ADOPT_FORGE_WINDOW.store(false, Ordering::SeqCst);
    }
}

/// How many times the forge-window seam has fired in this process
/// (Principle #3).
pub fn adopt_forge_window_panics_fired() -> u64 {
    ADOPT_FORGE_WINDOW_PANICS_FIRED.load(Ordering::SeqCst)
}

/// The seam itself — called by `take_adopted` inside the guarded window.
/// A no-op unless armed.
pub(crate) fn maybe_panic_in_adopt_forge_window() {
    if PANIC_IN_ADOPT_FORGE_WINDOW.load(Ordering::SeqCst) {
        ADOPT_FORGE_WINDOW_PANICS_FIRED.fetch_add(1, Ordering::SeqCst);
        panic!("{ADOPT_FORGE_WINDOW_PANIC_MSG}");
    }
}

// ── The adopt-take report-window panic ─────────────────

static PANIC_IN_ADOPT_REPORT_WINDOW: AtomicBool = AtomicBool::new(false);
static ADOPT_REPORT_WINDOW_PANICS_FIRED: AtomicU64 = AtomicU64::new(0);

/// The injected panic's payload — a test asserts the seam FIRED (via
/// [`adopt_report_window_panics_fired`]) so an `RMW_RET_ERROR` is
/// attributable to the injection, not some unrelated failure.
pub const ADOPT_REPORT_WINDOW_PANIC_MSG: &str =
    "test-seam: injected panic inside the adopt-take report window";

/// RAII arm of the adopt-take REPORT-window seam: while held,
/// `take_adopted` panics AFTER the registrations completed and every
/// success-path reporter ran, but BEFORE the `ForgedMessageGuard` stands
/// down — the window the disarm-LAST ordering exists for (a panic in
/// any reporter must still un-forge the caller's message). Dropped
/// (normally or by unwind) ⇒ disarmed.
#[must_use = "the seam is armed only while the guard is held"]
pub struct AdoptReportWindowPanicGuard(());

impl AdoptReportWindowPanicGuard {
    /// Arm the seam for the lifetime of the returned guard.
    pub fn arm() -> Self {
        PANIC_IN_ADOPT_REPORT_WINDOW.store(true, Ordering::SeqCst);
        Self(())
    }
}

impl Drop for AdoptReportWindowPanicGuard {
    fn drop(&mut self) {
        PANIC_IN_ADOPT_REPORT_WINDOW.store(false, Ordering::SeqCst);
    }
}

/// How many times the report-window seam has fired in this process
/// (Principle #3).
pub fn adopt_report_window_panics_fired() -> u64 {
    ADOPT_REPORT_WINDOW_PANICS_FIRED.load(Ordering::SeqCst)
}

/// The seam itself — called by `take_adopted` at the END of the guarded
/// window, right before the guard stands down. A no-op unless armed.
pub(crate) fn maybe_panic_in_adopt_report_window() {
    if PANIC_IN_ADOPT_REPORT_WINDOW.load(Ordering::SeqCst) {
        ADOPT_REPORT_WINDOW_PANICS_FIRED.fetch_add(1, Ordering::SeqCst);
        panic!("{ADOPT_REPORT_WINDOW_PANIC_MSG}");
    }
}

// ── The LOANED take's serve-report panic ─────────────────────────────────

static PANIC_IN_LOANED_SERVE_REPORT: AtomicBool = AtomicBool::new(false);
static LOANED_SERVE_REPORT_PANICS_FIRED: AtomicU64 = AtomicU64::new(0);

/// The injected panic's payload — a test asserts the seam FIRED so an
/// `RMW_RET_ERROR` is attributable to the injection.
pub const LOANED_SERVE_REPORT_PANIC_MSG: &str =
    "test-seam: injected panic inside the loaned take's serve report";

/// RAII arm of the LOANED serve-report seam: while held,
/// `rmw_take_loaned_message` panics at `report_loan_served` — after the loan
/// has been pushed into `pending_takes` but before `*loaned_message` and
/// `*taken` are written.
///
/// That is the window the ADOPTED path's rollback closes and this seam's
/// rollback closes on the loaned one: without the rollback the sample stays borrowed
/// and its slot consumed while the caller is told the take failed and never
/// receives the pointer it would need to return the loan.
#[must_use = "the seam is armed only while the guard is held"]
pub struct LoanedServeReportPanicGuard(());

impl LoanedServeReportPanicGuard {
    /// Arm the seam for the lifetime of the returned guard.
    pub fn arm() -> Self {
        PANIC_IN_LOANED_SERVE_REPORT.store(true, Ordering::SeqCst);
        Self(())
    }
}

impl Drop for LoanedServeReportPanicGuard {
    fn drop(&mut self) {
        PANIC_IN_LOANED_SERVE_REPORT.store(false, Ordering::SeqCst);
    }
}

/// How many times the loaned serve-report seam has fired in this process
/// (Principle #3).
pub fn loaned_serve_report_panics_fired() -> u64 {
    LOANED_SERVE_REPORT_PANICS_FIRED.load(Ordering::SeqCst)
}

/// The seam itself — fired inside the loaned take's serve report. A no-op
/// unless armed.
pub(crate) fn maybe_panic_in_loaned_serve_report() {
    if PANIC_IN_LOANED_SERVE_REPORT.load(Ordering::SeqCst) {
        LOANED_SERVE_REPORT_PANICS_FIRED.fetch_add(1, Ordering::SeqCst);
        panic!("{LOANED_SERVE_REPORT_PANIC_MSG}");
    }
}

// ── The adopt-take REGISTRATION-REPORT panic ────────────────────────────

static PANIC_IN_ADOPT_REGISTRATION_REPORT: AtomicBool = AtomicBool::new(false);
static ADOPT_REGISTRATION_REPORT_PANICS_FIRED: AtomicU64 = AtomicU64::new(0);

/// The injected panic's payload — a test asserts the seam FIRED (via
/// [`adopt_registration_report_panics_fired`]) so an `RMW_RET_ERROR` is
/// attributable to the injection.
pub const ADOPT_REGISTRATION_REPORT_PANIC_MSG: &str =
    "test-seam: injected panic inside the adopt-take registration report";

/// RAII arm of the registration-REPORT seam: while held, the adopting take
/// panics AT the registration-recovery report — the reporter that must not
/// run BEFORE the rollback window, where the registrations are already live and
/// their cookies hold the sample.
///
/// It travels WITH that reporter deliberately: move the report back outside
/// the guarded window and this seam moves with it, so the arm below fails.
/// That is the whole point of the pin.
#[must_use = "the seam is armed only while the guard is held"]
pub struct AdoptRegistrationReportPanicGuard(());

impl AdoptRegistrationReportPanicGuard {
    /// Arm the seam for the lifetime of the returned guard.
    pub fn arm() -> Self {
        PANIC_IN_ADOPT_REGISTRATION_REPORT.store(true, Ordering::SeqCst);
        Self(())
    }
}

impl Drop for AdoptRegistrationReportPanicGuard {
    fn drop(&mut self) {
        PANIC_IN_ADOPT_REGISTRATION_REPORT.store(false, Ordering::SeqCst);
    }
}

/// How many times the registration-report seam has fired in this process
/// (Principle #3).
pub fn adopt_registration_report_panics_fired() -> u64 {
    ADOPT_REGISTRATION_REPORT_PANICS_FIRED.load(Ordering::SeqCst)
}

/// The seam itself — fired beside the registration-recovery report. A no-op
/// unless armed.
pub(crate) fn maybe_panic_in_adopt_registration_report() {
    if PANIC_IN_ADOPT_REGISTRATION_REPORT.load(Ordering::SeqCst) {
        ADOPT_REGISTRATION_REPORT_PANICS_FIRED.fetch_add(1, Ordering::SeqCst);
        panic!("{ADOPT_REGISTRATION_REPORT_PANIC_MSG}");
    }
}

// ── The rmw_init announce-window panic ─────────

static PANIC_IN_INIT_ANNOUNCE: AtomicBool = AtomicBool::new(false);
static INIT_ANNOUNCE_PANICS_FIRED: AtomicU64 = AtomicU64::new(0);

/// The injected panic's payload — a test asserts the seam FIRED (via
/// [`init_announce_panics_fired`]) so an `RMW_RET_ERROR` is attributable
/// to the injection.
pub const INIT_ANNOUNCE_PANIC_MSG: &str =
    "test-seam: injected panic inside the rmw_init announce window";

/// RAII arm of the `rmw_init` ANNOUNCE-window seam: while held, `rmw_init`
/// panics AFTER the context is registered in the adopt-take live set and
/// written, at the point where the post-registration `warn!`/`info!` run.
///
/// It stands in for the real hazard, which no in-process harness can
/// install: those log lines go to the HOST's `tracing` subscriber, and
/// `install_tracing` deliberately lets a host's own subscriber win, so a
/// foreign `on_event` that panics unwinds from exactly here. What the seam
/// reproduces is the CONTROL FLOW — a panic between registration and
/// return — which is what the rollback has to survive.
#[must_use = "the seam is armed only while the guard is held"]
pub struct InitAnnouncePanicGuard(());

impl InitAnnouncePanicGuard {
    /// Arm the seam for the lifetime of the returned guard.
    pub fn arm() -> Self {
        PANIC_IN_INIT_ANNOUNCE.store(true, Ordering::SeqCst);
        Self(())
    }
}

impl Drop for InitAnnouncePanicGuard {
    fn drop(&mut self) {
        PANIC_IN_INIT_ANNOUNCE.store(false, Ordering::SeqCst);
    }
}

/// How many times the init announce-window seam has fired in this process
/// (Principle #3).
pub fn init_announce_panics_fired() -> u64 {
    INIT_ANNOUNCE_PANICS_FIRED.load(Ordering::SeqCst)
}

/// The seam itself — called by `rmw_init` inside its announce window. A
/// no-op unless armed.
pub(crate) fn maybe_panic_in_init_announce() {
    if PANIC_IN_INIT_ANNOUNCE.load(Ordering::SeqCst) {
        INIT_ANNOUNCE_PANICS_FIRED.fetch_add(1, Ordering::SeqCst);
        panic!("{INIT_ANNOUNCE_PANIC_MSG}");
    }
}

// ---------------------------------------------------------------------
// Baked-distro override: lets a test exercise the load-time
// era guard (every guarded init export) against a SPECIFIC pair
// in-process. A dev build bakes "vendored-dev", which already refuses
// any named runtime outside the snapshot's era; the override is how a
// test names the baked side it wants (jazzy vs kilted/foxy).
// ---------------------------------------------------------------------

static BAKED_DISTRO_OVERRIDE: std::sync::Mutex<Option<&'static str>> = std::sync::Mutex::new(None);

/// RAII override of [`crate::era::baked_distro`]: while held, every
/// guarded init export compares the injected distro against the runtime
/// `ROS_DISTRO` instead of the build-baked value. Dropped (normally or
/// by unwind) ⇒ the baked value is back.
#[must_use = "the override applies only while the guard is held"]
pub struct BakedDistroOverrideGuard(());

impl BakedDistroOverrideGuard {
    /// Arm the override for the lifetime of the returned guard.
    pub fn set(distro: &'static str) -> Self {
        let mut slot = BAKED_DISTRO_OVERRIDE
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        // Check BEFORE writing: a refused second arm must not overwrite
        // the live guard's value.
        assert!(
            slot.is_none(),
            "the baked-distro override is already armed ({slot:?}) — overlapping guards would \
             disarm each other"
        );
        *slot = Some(distro);
        drop(slot);
        Self(())
    }
}

impl Drop for BakedDistroOverrideGuard {
    fn drop(&mut self) {
        *BAKED_DISTRO_OVERRIDE
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = None;
    }
}

/// The override, if armed — consulted by [`crate::era::baked_distro`].
pub fn baked_distro_override() -> Option<&'static str> {
    *BAKED_DISTRO_OVERRIDE
        .lock()
        .unwrap_or_else(|e| e.into_inner())
}

// ---------------------------------------------------------------------
// Era-guard panic seam: fire a panic
// INSIDE `refuse_on_era_mismatch` — the region the init-options entry
// points run under `ffi_guard` — so a test can prove a panic there
// degrades to the entry point's failure code instead of unwinding
// through the C ABI (which aborts the host process).
// ---------------------------------------------------------------------

static PANIC_INSIDE_ERA_GUARD: AtomicBool = AtomicBool::new(false);
static ERA_GUARD_PANICS_FIRED: AtomicU64 = AtomicU64::new(0);

/// The injected panic's payload — the `panic=` field `ffi_guard` logs.
pub const ERA_GUARD_PANIC_MSG: &str = "test-seam: injected panic inside the era guard";

/// RAII arm of the era-guard panic seam: while held, every guarded
/// init-options entry point (and `rmw_init`) panics at the top of its
/// era guard, before any caller-memory access. Dropped ⇒ disarmed.
#[must_use = "the seam is armed only while the guard is held"]
pub struct EraGuardPanicGuard(());

impl EraGuardPanicGuard {
    /// Arm the seam for the lifetime of the returned guard.
    pub fn arm() -> Self {
        assert!(
            !PANIC_INSIDE_ERA_GUARD.swap(true, Ordering::SeqCst),
            "the era-guard seam is already armed — overlapping guards would disarm each other"
        );
        Self(())
    }
}

impl Drop for EraGuardPanicGuard {
    fn drop(&mut self) {
        PANIC_INSIDE_ERA_GUARD.store(false, Ordering::SeqCst);
    }
}

/// How many times the era-guard seam has fired in this process — the
/// observable a test compares before/after so a failure code is
/// attributable to the injection, not to some other refusal.
pub fn era_guard_panics_fired() -> u64 {
    ERA_GUARD_PANICS_FIRED.load(Ordering::SeqCst)
}

/// The seam itself — called at the top of `refuse_on_era_mismatch`. A
/// no-op unless armed.
pub(crate) fn maybe_panic_inside_era_guard() {
    if PANIC_INSIDE_ERA_GUARD.load(Ordering::SeqCst) {
        ERA_GUARD_PANICS_FIRED.fetch_add(1, Ordering::SeqCst);
        panic!("{ERA_GUARD_PANIC_MSG}");
    }
}

// ---------------------------------------------------------------------
// Restore-flavored env RAII: the ONE guard for the era-guard
// tests. The `era.rs` lib tests and the era-guard integration binary
// share it rather than carrying a copy each, because a fix such as
// the non-UTF-8-prior handling below would reach a second copy only
// by hand — two copies is how a fix to one silently misses the other.
//
// SCOPE: it is NOT the one guard for every
// test in this crate that sets a process-global variable. FOUR
// hand-rolled copies exist, all of the blanket-`remove_var` shape
// — `tests/rmw_wait_event_test.rs`, `tests/rmw_wait_spin_test.rs`,
// `tests/rmw_slice_ceiling_e2e_test.rs` and
// `tests/rmw_wait_pingpong_discriminator_test.rs`. They never restore a
// prior at all, so the non-UTF-8 class below cannot reach them, but a
// pre-existing value of their own `CERULION_*` variables IS clobbered
// for the rest of the process. This guard is `pub`, so any of
// them can use it instead.
// ---------------------------------------------------------------------

static ENV_GUARD_OWNERSHIP_LOSSES: AtomicU64 = AtomicU64::new(0);

/// How many times an [`EnvVarGuard`] found the variable no longer holding
/// what it installed — unconditional, never reset, and bumped on BOTH
/// arms. The panicking arm reports through `tracing`
/// and libtest DISCARDS captured output on a test that survives the
/// unwind, so without this a variant that keeps the `return` but drops
/// the report is observable by nothing.
pub fn env_guard_ownership_losses() -> u64 {
    ENV_GUARD_OWNERSHIP_LOSSES.load(Ordering::Relaxed)
}

/// Panic-safe env RAII, restore-flavored: the container lanes run these
/// tests with a REAL `ROS_DISTRO`, so drop puts the PRIOR value back
/// rather than blanket-removing it. The prior is snapshotted with
/// `var_os`: `var(..).ok()` would read a pre-existing
/// NON-UTF-8 value as `None`, so drop would REMOVE it — process-global state
/// changed for every later test.
///
/// Guards on one variable are expected to nest LIFO, and drop DETECTS
/// the common violations: it refuses to restore a
/// variable that no longer holds what it installed — panicking normally,
/// and (so an unwind cannot become an abort) reporting through `tracing`
/// while `std::thread::panicking()`, with
/// [`env_guard_ownership_losses`] counting either way. Detection is
/// best-effort, not enforcement: two guards installing the SAME value are
/// indistinguishable. What IS guaranteed is that a detected loss SKIPS
/// the stale write, which is the harm the rule exists to prevent.
#[must_use = "the variable is restored when the guard drops"]
pub struct EnvVarGuard {
    name: &'static str,
    prior: Option<std::ffi::OsString>,
    installed: Option<std::ffi::OsString>,
}

impl EnvVarGuard {
    /// The ONE snapshot-and-apply site (constructors snapshotting on
    /// their own is how a raw-value constructor's snapshot bug could
    /// slip past the nesting pin — one site
    /// regresses once, and the nesting pin covers it by construction).
    fn install(name: &'static str, installed: Option<std::ffi::OsString>) -> Self {
        let prior = std::env::var_os(name);
        match &installed {
            Some(value) => std::env::set_var(name, value),
            None => std::env::remove_var(name),
        }
        Self {
            name,
            prior,
            installed,
        }
    }

    /// Set `name` to `value` — a `&str`, or a raw (possibly non-UTF-8)
    /// `OsString` — until the guard drops.
    pub fn set(name: &'static str, value: impl AsRef<std::ffi::OsStr>) -> Self {
        Self::install(name, Some(value.as_ref().to_owned()))
    }

    /// Remove `name` until the guard drops.
    pub fn unset(name: &'static str) -> Self {
        Self::install(name, None)
    }
}

impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        // OWNERSHIP FIRST: restoring a variable this
        // guard no longer owns writes a stale value into process-global
        // state — the precise harm the check advertises preventing — so
        // the restore is SKIPPED when ownership is lost, on BOTH paths.
        // Asserting and then restoring unconditionally would let
        // an out-of-order drop during an unwind mis-restore.
        if std::env::var_os(self.name) != self.installed {
            ENV_GUARD_OWNERSHIP_LOSSES.fetch_add(1, Ordering::Relaxed);
            // While unwinding a panic must not become an abort, so the
            // loss is REPORTED rather than asserted; the test is already
            // failing. Through `tracing`, not `eprintln!`: this module is
            // compiled into the non-test rlib every integration-test
            // target links, where the crate root's
            // `cfg_attr(not(test), deny(clippy::print_stderr))` applies.
            if std::thread::panicking() {
                tracing::error!(
                    name = %self.name,
                    "EnvVarGuard lost ownership during an unwind — not restoring"
                );
                return;
            }
            panic!(
                "EnvVarGuard for `{}` dropped out of LIFO order, or something outside the \
                 guards wrote the variable — it no longer holds what this guard installed",
                self.name
            );
        }
        match &self.prior {
            Some(value) => std::env::set_var(self.name, value),
            None => std::env::remove_var(self.name),
        }
    }
}

// ── The adopt-take shutdown RACE seam ────────────────────────────────────

static CLEAR_CALLBACK_IN_ADOPT_FORGE_WINDOW: AtomicBool = AtomicBool::new(false);
static ADOPT_FORGE_WINDOW_CLEARS_FIRED: AtomicU64 = AtomicU64::new(0);

/// RAII arm of the adopt-take shutdown-RACE seam: while held, `take_adopted`
/// CLEARS the process-global release callback inside its forge window — after
/// it checked the callback epoch and before its registrations land. That is
/// exactly the interleaving a concurrent `rmw_shutdown` produces, made
/// deterministic: a thread-racing test would be flaky and would prove the
/// property only on the runs where it happened to lose the race.
///
/// The take must notice (its post-registration epoch re-read) and withdraw,
/// serving by copy. Dropped ⇒ disarmed; the callback stays cleared, which is
/// what a real shutdown would also leave behind.
#[must_use = "the seam is armed only while the guard is held"]
pub struct AdoptForgeWindowClearGuard(());

impl AdoptForgeWindowClearGuard {
    /// Arm the seam for the lifetime of the returned guard.
    pub fn arm() -> Self {
        CLEAR_CALLBACK_IN_ADOPT_FORGE_WINDOW.store(true, Ordering::SeqCst);
        Self(())
    }
}

impl Drop for AdoptForgeWindowClearGuard {
    fn drop(&mut self) {
        CLEAR_CALLBACK_IN_ADOPT_FORGE_WINDOW.store(false, Ordering::SeqCst);
    }
}

/// How many times the shutdown-race seam has fired in this process
/// (Principle #3) — a test asserts it FIRED, so the rollback it observes is
/// attributable to the injection rather than to some unrelated refusal.
pub fn adopt_forge_window_clears_fired() -> u64 {
    ADOPT_FORGE_WINDOW_CLEARS_FIRED.load(Ordering::SeqCst)
}

/// The seam itself — called by `take_adopted` inside the forge window, once
/// per armed take. A no-op unless armed.
pub(crate) fn maybe_clear_release_callback_in_adopt_forge_window() {
    if CLEAR_CALLBACK_IN_ADOPT_FORGE_WINDOW.load(Ordering::SeqCst) {
        ADOPT_FORGE_WINDOW_CLEARS_FIRED.fetch_add(1, Ordering::SeqCst);
        crate::adopt_take::clear_release_callback_for_seam();
    }
}

// ── A concurrent CREATE re-installing the callback ───────────────────────

static REINSTALL_IN_ADOPT_FORGE_WINDOW: AtomicBool = AtomicBool::new(false);
static ADOPT_FORGE_WINDOW_REINSTALLS_FIRED: AtomicU64 = AtomicU64::new(0);

/// RAII arm of the re-install seam: while held, `take_adopted` RE-INSTALLS the
/// release callback inside its forge window — what a concurrent
/// `rmw_create_subscription` does, since every create re-installs the same
/// pointer. The take must NOT treat that as a shutdown: rolling back there
/// turns every graph change into a silent zero-copy degrade for whatever is
/// taking at the time.
#[must_use = "the seam is armed only while the guard is held"]
pub struct AdoptForgeWindowReinstallGuard(());

impl AdoptForgeWindowReinstallGuard {
    /// Arm the seam for the lifetime of the returned guard.
    pub fn arm() -> Self {
        REINSTALL_IN_ADOPT_FORGE_WINDOW.store(true, Ordering::SeqCst);
        Self(())
    }
}

impl Drop for AdoptForgeWindowReinstallGuard {
    fn drop(&mut self) {
        REINSTALL_IN_ADOPT_FORGE_WINDOW.store(false, Ordering::SeqCst);
    }
}

/// How many times the re-install seam has fired in this process (Principle #3).
pub fn adopt_forge_window_reinstalls_fired() -> u64 {
    ADOPT_FORGE_WINDOW_REINSTALLS_FIRED.load(Ordering::SeqCst)
}

/// The seam itself — called by `take_adopted` inside the forge window. A
/// no-op unless armed.
pub(crate) fn maybe_reinstall_release_callback_in_adopt_forge_window() {
    if REINSTALL_IN_ADOPT_FORGE_WINDOW.load(Ordering::SeqCst) {
        ADOPT_FORGE_WINDOW_REINSTALLS_FIRED.fetch_add(1, Ordering::SeqCst);
        crate::adopt_take::reinstall_release_callback_for_seam();
    }
}

// ── A clear landing AFTER the take's last epoch check ────────────────────

static CLEAR_IN_ADOPT_REPORT_WINDOW: AtomicBool = AtomicBool::new(false);
static ADOPT_REPORT_WINDOW_CLEARS_FIRED: AtomicU64 = AtomicU64::new(0);

/// RAII arm of the POST-CHECK clear seam: while held, `take_adopted` clears
/// the release callback in its report window — after the post-registration
/// epoch re-read and before it returns.
///
/// This is the residual made reproducible, and it is
/// deliberately the one seam whose take is NOT rolled back: nothing after the
/// last check can notice, because the application takes delivery when
/// `rmw_take` returns. Its sibling
/// [`AdoptForgeWindowClearGuard`] fires one step EARLIER and IS caught — the
/// pair is what makes "the window is shutdown-only" a measured claim rather
/// than a prose one.
#[must_use = "the seam is armed only while the guard is held"]
pub struct AdoptReportWindowClearGuard(());

impl AdoptReportWindowClearGuard {
    /// Arm the seam for the lifetime of the returned guard.
    pub fn arm() -> Self {
        CLEAR_IN_ADOPT_REPORT_WINDOW.store(true, Ordering::SeqCst);
        Self(())
    }
}

impl Drop for AdoptReportWindowClearGuard {
    fn drop(&mut self) {
        CLEAR_IN_ADOPT_REPORT_WINDOW.store(false, Ordering::SeqCst);
    }
}

/// How many times the post-check clear seam has fired (Principle #3).
pub fn adopt_report_window_clears_fired() -> u64 {
    ADOPT_REPORT_WINDOW_CLEARS_FIRED.load(Ordering::SeqCst)
}

/// The seam itself — called by `take_adopted` inside the report window, past
/// every check it makes. A no-op unless armed.
pub(crate) fn maybe_clear_release_callback_in_adopt_report_window() {
    if CLEAR_IN_ADOPT_REPORT_WINDOW.load(Ordering::SeqCst) {
        ADOPT_REPORT_WINDOW_CLEARS_FIRED.fetch_add(1, Ordering::SeqCst);
        crate::adopt_take::clear_release_callback_for_seam();
    }
}

#[cfg(test)]
mod tests {
    use super::EnvVarGuard;
    use serial_test::serial;

    /// Variables no other test reads, so the pins exercise the guard
    /// alone (`#[serial]` regardless: the environment is process-global).
    const PIN_VAR: &str = "CERULION_RMW_TEST_ENV_GUARD_PIN";
    const LIFO_VAR: &str = "CERULION_RMW_TEST_ENV_GUARD_LIFO_PIN";
    const UNWIND_VAR: &str = "CERULION_RMW_TEST_ENV_GUARD_UNWIND_PIN";

    #[test]
    #[serial]
    fn the_env_guard_restores_a_non_utf8_prior_value() {
        // A guard that snapshots with `var(..).ok()`
        // reads a NON-UTF-8 prior as `None` and REMOVES it on drop.
        use std::os::unix::ffi::OsStringExt as _;
        let raw = std::ffi::OsString::from_vec(vec![b'r', 0xfe, b'w']);
        let raw2 = std::ffi::OsString::from_vec(vec![b'x', 0xff]);
        let _outer = EnvVarGuard::set(PIN_VAR, raw.clone());
        // Every constructor and both value shapes are the inner guard
        // once (nesting only `set(&str)` would let the raw
        // constructor's own snapshot bug survive — the snapshot
        // has ONE site, so the nesting covers it by construction).
        {
            let _inner = EnvVarGuard::set(PIN_VAR, "jazzy");
            assert_eq!(std::env::var(PIN_VAR).as_deref(), Ok("jazzy"));
        }
        assert_eq!(
            std::env::var_os(PIN_VAR).as_deref(),
            Some(raw.as_os_str()),
            "after `set`"
        );
        {
            let _inner = EnvVarGuard::unset(PIN_VAR);
            assert!(std::env::var_os(PIN_VAR).is_none());
        }
        assert_eq!(
            std::env::var_os(PIN_VAR).as_deref(),
            Some(raw.as_os_str()),
            "after `unset`"
        );
        {
            let _inner = EnvVarGuard::set(PIN_VAR, raw2.clone());
            assert_eq!(std::env::var_os(PIN_VAR).as_deref(), Some(raw2.as_os_str()));
        }
        assert_eq!(
            std::env::var_os(PIN_VAR).as_deref(),
            Some(raw.as_os_str()),
            "dropping the inner raw-value guard must restore the raw prior bytes"
        );
        drop(_outer);
        assert!(
            std::env::var_os(PIN_VAR).is_none(),
            "the outer guard removes what was absent before it"
        );
    }

    #[test]
    #[serial]
    fn a_guard_dropped_out_of_lifo_order_is_refused_not_silently_misrestored() {
        // LIFO is the contract — an outer guard
        // dropped under a live inner one would put ITS prior back while
        // the inner guard still believes it owns the variable, and the
        // inner drop then restores the outer's value. The refusal fires
        // BEFORE any restore, so the outer guard writes nothing.
        // A `should_panic` arm would prove only the
        // panic text — a drop that restored the stale prior and THEN
        // panicked would pass it. The unwind is caught here so the state the
        // refusal exists to protect is asserted right after it: the
        // variable still holds the INNER guard's value, and the loss is
        // recorded on the unconditional counter.
        let losses_before = super::env_guard_ownership_losses();
        let outer = EnvVarGuard::set(LIFO_VAR, "1");
        let inner = EnvVarGuard::set(LIFO_VAR, "2");
        let unwound = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| drop(outer)));
        let message = match unwound {
            Err(payload) => payload
                .downcast_ref::<String>()
                .cloned()
                .or_else(|| payload.downcast_ref::<&str>().map(|m| m.to_string()))
                .unwrap_or_default(),
            Ok(()) => panic!("the out-of-order drop must be refused"),
        };
        assert!(
            message.contains("dropped out of LIFO order"),
            "the refusal names the contract: {message}"
        );
        assert_eq!(
            std::env::var(LIFO_VAR).as_deref(),
            Ok("2"),
            "the refused outer guard must have written NOTHING — the inner guard's value \
             stands"
        );
        assert_eq!(
            super::env_guard_ownership_losses() - losses_before,
            1,
            "the refusal is counted exactly once"
        );
        // The inner guard still owns the variable and restores the
        // outer's `"1"` cleanly; the outer never ran its restore (its
        // prior was ABSENT), so put that back by hand and leave the
        // process as it was found.
        drop(inner);
        assert_eq!(std::env::var(LIFO_VAR).as_deref(), Ok("1"));
        std::env::remove_var(LIFO_VAR);
    }
    #[test]
    #[serial]
    fn a_guard_that_loses_ownership_while_unwinding_skips_the_restore_and_does_not_abort() {
        // The ownership check has TWO arms, and deleting the
        // panicking one survives the LIFO
        // arm, whose loss happens while NOT unwinding. Here the subject
        // moves the variable itself and then panics, so the guard drops
        // during an unwind having lost ownership: it must REPORT and
        // skip, never restore (a stale write) and never assert (a panic
        // inside a drop during an unwind ABORTS the process — deleting that
        // branch kills this test by killing the binary).
        let losses_before = super::env_guard_ownership_losses();
        let _outer = EnvVarGuard::set(UNWIND_VAR, "prior");
        let unwound = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _inner = EnvVarGuard::set(UNWIND_VAR, "installed");
            std::env::set_var(UNWIND_VAR, "clobbered");
            panic!("deliberate");
        }));
        assert!(unwound.is_err(), "the closure must have panicked");
        assert_eq!(
            std::env::var(UNWIND_VAR).as_deref(),
            Ok("clobbered"),
            "a guard that lost ownership must not write its prior back"
        );
        // The loss is REPORTED, not merely skipped: libtest discards the
        // captured `tracing` output of a test that survives the unwind,
        // so the unconditional counter is the only observable a
        // silent skip cannot pass.
        assert_eq!(
            super::env_guard_ownership_losses() - losses_before,
            1,
            "exactly one ownership loss must be recorded"
        );
        // The OUTER guard still owns nothing either, so its own drop
        // takes the same arm — put the variable back by hand so the
        // process is left as it was found.
        std::env::set_var(UNWIND_VAR, "prior");
    }
}
