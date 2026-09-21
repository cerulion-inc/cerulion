// SPDX-License-Identifier: AGPL-3.0-only
//! Adopt-take (`--adopt-take`): zero-copy PLAIN takes for
//! retaining consumers — the substrate `take_impl`'s adoption branch runs
//! on.
//!
//! # The shape
//!
//! When the gate is armed, a plain `rmw_take` on a forgeable type swaps
//! `try_receive_one` for `try_receive_one_owned`, runs `unflatten_forged`
//! against the **caller-owned** message (fixed fields, strings and nested
//! members copy exactly as on the copying take; each forgeable primitive sequence's
//! container header is aimed at the held sample's bytes with
//! `capacity == size`), registers each forged member's exact byte range
//! with the preloaded heap hook, and returns with the sample **held**. The
//! app's eventual `fini`/destructor frees each forged `data` pointer; the
//! hook classifies each as a Release and fires `adopt_release_callback`,
//! which drops one `Arc` clone; the last drop releases the
//! `OwnedInboundSample` (the SHM borrow + publisher-pool slot).
//!
//! # The gate is a WITNESS, not a flag
//!
//! [`AdoptTakeGrant`] is constructible ONLY from an Active heap-hook
//! handshake with all three take-side symbols resolved (`register_segment`,
//! `unregister_segment`, `set_release_callback` — [`crate::heaphook`]
//! resolves ALL entries or degrades, so `active_hook()` returning `Some`
//! IS that proof), and the adoption branch takes `&AdoptTakeGrant` — no
//! grant, no branch; the compiler is the gate. The env var
//! ([`ADOPT_TAKE_ENV`]) is read at subscription CREATE (it keys the borrow
//! floor, and per-subscription latching avoids mid-stream mode flips) and
//! stored on `SubscriptionData` as an `Option<AdoptTakeState>`. Without
//! the grant no app `free` can ever see an SHM address — never UB by
//! construction; the env-without-preload misconfiguration degrades to the
//! copy path with ONE loud warn (`warn_missing_preload_once`).
//!
//! # Refcount = `Arc` strong count
//!
//! One [`Arc<AdoptedSample>`] clone per registered range
//! (`Arc::into_raw` is the cookie); the hook fires the release callback at
//! most once per registration (classification + registry removal are
//! atomic under its locks) and the cookie-is-identity rule means an
//! address-reused later registration can never release the wrong sample —
//! so double-release is structurally impossible and the sample provably
//! outlives the LAST forged free with no separate counter to get wrong.
//!
//! # Destroy / shutdown policy
//!
//! - **Subscription destroy with adopted ranges outstanding:** warn and
//!   LEAVE — the app legitimately holds forged pointers into the samples;
//!   reclaiming would dangle them. The registrations, the callback and the
//!   Arcs reference nothing subscription-scoped, so frees after destroy
//!   still release correctly.
//! - **`rmw_shutdown` with ranges outstanding:** emit the summary, and —
//!   on the FINAL live context's shutdown ONLY (`rmw_shutdown`
//!   tracks live contexts — by IDENTITY, so a repeat shutdown of
//!   one context is the rmw.h no-op and never a second decrement; the
//!   callback is process-global, so an intermediate shutdown clearing it
//!   would route a SURVIVING context's later frees onto the no-callback
//!   leak path, pinning its samples and borrow slots forever) — CLEAR the
//!   release callback (a post-shutdown
//!   free must never call into a possibly-unmapped module; the hook then
//!   leaks-never-frees, its designed no-callback behavior, counter kind 0)
//!   and deliberately leak the outstanding Arcs — they live as raw cookies
//!   inside the hook's registry, which nothing will ever `from_raw` once
//!   the callback is cleared. The SHM mappings stay valid to process exit
//!   because the rmw runtime ([`crate::runtime::runtime`]) is a
//!   process-lifetime `OnceLock` static that is never dropped. Never block
//!   shutdown on app frees; an intermediate shutdown with samples
//!   outstanding says loudly that the callback is KEPT.

use std::collections::BTreeSet;
use std::os::raw::c_void;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};

use cerulion_core::transport::failure_regime_latch::FailureRegimeLatch;
use cerulion_core::transport::subscriber::OwnedInboundSample;

use crate::heaphook::{self, HookApi, RC_OK};

/// The opt-in gate: `CERULION_RMW_ADOPT_TAKE=1` arms adopt-take for every
/// subscription CREATED while it is set. Set it on a node you launch
/// DIRECTLY, with the heap hook preloaded: the
/// `cerulion ros2 run|launch --adopt-take` flag REFUSES rather than setting
/// it for children, because the `ros2` Python CLI spawns the node as a
/// further subprocess and closes the descriptor the launcher hands the hook
/// over on. `0` / unset disarm; any other value is a loud once-per-process
/// warn and disarms (strict parse, never a silent guess).
pub const ADOPT_TAKE_ENV: &str = "CERULION_RMW_ADOPT_TAKE";

/// Env override for [`RMW_ADOPT_TAKE_BORROW_BUDGET`] (CREATE-leg only, per
/// the existing open-tolerates-smaller rule). Must parse as a positive
/// integer; anything else warns and keeps the default.
pub const ADOPT_TAKE_BUDGET_ENV: &str = "CERULION_RMW_ADOPT_TAKE_BUDGET";

/// The `subscriber_max_borrowed_samples` CREATE-leg floor an adopt-armed
/// subscription provisions its service with. Adopted samples pin borrow
/// units for as long as the APP retains the taken messages — the
/// loaned-take sizing (`RMW_TAKE_LOAN_BORROW_BUDGET` = 4, "3 held + 1
/// transient") is wrong for that shape, so the adopt floor is 16
/// (retention is app-controlled; cost is APPARENT bytes only —
/// each unit is one more demand-paged slot in the publisher pool).
/// Exhaustion is loud, latched, and self-heals when the holder releases
/// ([`crate::loan_refusal_latch::BorrowHolders`] names which of the two
/// borrow-holding paths it is — `taken = false`, never an error).
pub const RMW_ADOPT_TAKE_BORROW_BUDGET: usize = 16;

/// The smallest service borrow ceiling on which adoption
/// can work AT ALL.
///
/// A reusing caller — one message buffer taken into over and over, the
/// pattern `release_forgeable_members` exists to support — needs TWO borrow
/// units at the instant of a take: the one its previously-adopted sample
/// still pins, and the one the incoming receive consumes. The release that
/// frees the first runs INSIDE the take, after the receive, so at a ceiling
/// of 1 that receive is refused with `ExceedsMaxBorrows` and the release
/// never runs: the refusal's documented self-heal ("free an adopted
/// message") is unreachable, because for such a caller the free IS the
/// refused call. That is a permanent wedge, not a transient refusal.
///
/// At 2 there is no wedge for that caller: it holds 1, the receive takes the
/// 2nd, and the in-take release drops it back to 1 — every take thereafter
/// is served.
///
/// SCOPE: this is the SINGLE-BUFFER minimum. The rule generalises to
/// `N + 1` for a caller that rotates N message buffers, and nothing at
/// CREATE time can know N — so a DOUBLE-buffering caller on a ceiling-2
/// foreign service still wedges the same way, and this constant does not
/// save it. The gate does not cover that case; what
/// the gate removes is the case the ceiling ALONE proves impossible
/// (a ceiling of 1 cannot serve even one buffer). Covering the general case
/// needs the runtime to learn N — a disarm driven from a repeated
/// `AdoptedOnly` saturation at the full effective budget — which is
/// not implemented.
pub const MIN_ADOPT_EFFECTIVE_BUDGET: usize = 2;

/// Whether a service can carry the adopt path at all.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdoptViability {
    /// The service's borrow ceiling leaves room for a held sample plus an
    /// incoming receive: arm adoption.
    Viable,
    /// The ceiling is below [`MIN_ADOPT_EFFECTIVE_BUDGET`]. Adoption is NOT
    /// armed and the subscription is served by the copying take, which holds
    /// nothing and therefore cannot wedge.
    BorrowCeilingTooSmall,
}

/// The PURE decision: can adoption be armed on a
/// service with this EFFECTIVE borrow ceiling?
///
/// Why the gate is at CREATE and not at the take: the wedge is structural,
/// not transient, so the only possible outcomes are "never adopt on this
/// service" or "wedge the caller". No fix INSIDE the take
/// closes it — the gate cannot
/// pre-empt an `ExceedsMaxBorrows` on the receive, because the receive is
/// what consumes the borrow; and the only mechanism that could is a
/// transport-level receive that asks for a borrow without taking one, which
/// is a `cerulion_core` change serving one degenerate configuration.
///
/// Reachability, established rather than assumed: the rmw's CREATE leg
/// requests `max(budget, 4)`, but the OPEN leg is requirement-free BY
/// DESIGN, so a subscription can attach to a PRE-EXISTING service whose
/// `subscriber_max_borrowed_samples` is 1 (`cerulion_core`'s own borrow-floor
/// degrade warn documents that as a valid configuration). Nothing this crate
/// creates produces one.
///
/// Total over `usize`, allocation-free, dependent on nothing else.
/// The default budget must not itself be unviable: a shipped default below
/// the minimum would disarm adoption on every service. Const-asserted so the
/// contradiction cannot compile rather than being caught at test time.
const _: () = assert!(RMW_ADOPT_TAKE_BORROW_BUDGET >= MIN_ADOPT_EFFECTIVE_BUDGET);

pub fn adopt_viability(effective_budget: usize) -> AdoptViability {
    if effective_budget >= MIN_ADOPT_EFFECTIVE_BUDGET {
        AdoptViability::Viable
    } else {
        AdoptViability::BorrowCeilingTooSmall
    }
}

// ── Stats (Principle #3 — the bench proof lines read these) ───────────────

/// Per-subscription adopt-take counters, `Arc`-shared with every
/// [`AdoptedSample`] (and a process-global registry for the shutdown
/// aggregate) so they survive subscription destroy. All unconditional,
/// never reset.
#[derive(Debug, Default)]
pub struct AdoptStats {
    /// Successful takes served through the adoption branch (adopted AND
    /// copy-fallback alike — the denominator of the bench citability gate
    /// `adopted == takes`).
    pub takes: AtomicU64,
    /// Takes that ADOPTED (≥ 0 ranges registered, sample held or —
    /// all-empty-entries — released immediately; zero payload copies paid).
    pub adopted_takes: AtomicU64,
    /// Release-callback firings — one per registered range freed by the
    /// app.
    pub releases: AtomicU64,
    /// Adopted samples currently held (incremented at adoption,
    /// decremented when the last `Arc` clone drops).
    pub outstanding: AtomicU64,
    /// Adoption-branch takes served by the COPY path instead (nothing
    /// forgeable this frame, or a registration failure rolled back).
    pub fallbacks: AtomicU64,
    /// Takes refused because the app retains the whole borrow budget
    /// (`ExceedsMaxBorrows` under adopt — `taken = false`, self-heals).
    pub budget_refusals: AtomicU64,
    /// Has the owning subscription been destroyed? Retirement
    /// needs BOTH this and a drained `outstanding`, and either can happen
    /// last, so the two finalize paths test the same pair.
    pub destroyed: std::sync::atomic::AtomicBool,
    /// Has this entry already been folded into the retired total? The
    /// once-only guard, taken under the registry lock.
    pub retired: std::sync::atomic::AtomicBool,
}

// ── The witness ───────────────────────────────────────────────────────────

/// Proof the adoption branch may run: an Active heap-hook handshake with
/// the three take-side symbols resolved AND the release callback installed.
/// The field is private and [`AdoptTakeGrant::try_acquire`] is the only
/// constructor — code cannot reach the forge-instead-of-copy branch without
/// one (the compiler is the gate).
pub struct AdoptTakeGrant {
    api: HookApi,
}

impl AdoptTakeGrant {
    /// Construct the witness iff the hook handshake is Active (which, since
    /// `HookApi` carries every take-side entry, proves ALL of them resolved).
    /// Installs `adopt_release_callback` as the process-global release
    /// callback — once per grant construction; repeated installs store the
    /// same function pointer and also RE-ARM it after a shutdown cleared it
    /// (a re-inited context must release again).
    pub fn try_acquire() -> Option<Self> {
        let api = heaphook::active_hook()?;
        // Lifecycle race: the install is
        // serialized with the final-context clear under the ONE lifecycle
        // lock — a shutdown that already decided `final_context` cannot
        // clear the callback AFTER this grant installed it (and a grant
        // constructed after the clear re-installs, so a fresh context
        // always releases).
        let _lifecycle = lifecycle_lock();
        // SAFETY: `api` came from the resolved (or test-installed) hook;
        // `adopt_release_callback` is a process-lifetime `extern "C" fn`.
        let rc = unsafe { (api.set_release_callback)(Some(adopt_release_callback)) };
        // The epoch moves only on a
        // genuine TRANSITION. `try_acquire` runs at EVERY
        // `rmw_create_subscription` and re-installs the SAME function pointer,
        // so an unconditional bump would make any concurrent create look
        // like a shutdown to an in-flight take: its post-registration epoch
        // re-read would differ, it would withdraw good registrations and
        // serve a copy. Node bring-up creates subscriptions while other
        // subscriptions are taking, so that is not a corner — it is a
        // zero-copy path that silently degrades whenever the graph changes.
        // A no-op re-install must be invisible to the check.
        let already_installed = RELEASE_CALLBACK_EPOCH.load(Ordering::Relaxed) != 0;
        if rc == RC_OK && !already_installed {
            // Memory order: the hook's own
            // callback store happens ABOVE, and this publishes it with
            // RELEASE ordering, so a take that reads a non-zero epoch with
            // ACQUIRE cannot observe the epoch without also observing the
            // callback. A `Relaxed` pair has no such edge: on a
            // weakly-ordered target a take could see "installed" while the
            // hook's `RELEASE_CB` was still null, register ranges, and leave
            // them unreleasable.
            let epoch = NEXT_RELEASE_CALLBACK_EPOCH.fetch_add(1, Ordering::Relaxed) + 1;
            RELEASE_CALLBACK_EPOCH.store(epoch, Ordering::Release);
        }
        if rc != RC_OK {
            tracing::warn!(
                rc,
                "adopt-take: the heap hook refused set_release_callback — \
                 the grant is NOT constructed and every take keeps the copy path"
            );
            return None;
        }
        Some(Self { api })
    }

    /// The resolved hook entries the adoption branch drives.
    pub(crate) fn api(&self) -> &HookApi {
        &self.api
    }
}

// ── The adopted sample ────────────────────────────────────────────────────

/// One HELD sample serving an adopted take. The `Arc` strong count IS the
/// refcount: one clone per registered range (its `Arc::into_raw` pointer is
/// the registration's cookie), and the value drops — releasing the SHM
/// borrow + publisher-pool slot — exactly when the app has freed every
/// forged range (or the rollback reclaimed every clone).
pub struct AdoptedSample {
    sample: OwnedInboundSample,
    stats: Arc<AdoptStats>,
}

impl AdoptedSample {
    /// Wrap a received sample; counts it `outstanding` until the value
    /// drops (a registration-failure rollback nets the counter back to
    /// zero through the same `Drop`).
    pub(crate) fn new(sample: OwnedInboundSample, stats: Arc<AdoptStats>) -> Self {
        stats.outstanding.fetch_add(1, Ordering::Relaxed);
        Self { sample, stats }
    }

    /// The held wire frame (header + body) — the bytes the forged members
    /// alias. `pub` so the zero-copy tests
    /// can deref a registration's cookie back to the HELD sample and assert
    /// every forged range lies inside its payload — the oracle a
    /// copy-to-heap-and-register-the-copy implementation cannot pass.
    pub fn payload(&self) -> &[u8] {
        self.sample.payload()
    }
}

impl Drop for AdoptedSample {
    fn drop(&mut self) {
        // The sample field drops AFTER this body — the counter flips first,
        // then the SHM borrow releases.
        let left = self.stats.outstanding.fetch_sub(1, Ordering::Relaxed) - 1;
        // This is the only moment a
        // subscription destroyed while holding samples can be retired.
        // `retire_stats` refuses to fold a live one — correctly, since its
        // counters are still being updated — and without a retry here the
        // entry would stay in the registry for the life of the process: one
        // leak per destroy/free cycle, and every shutdown scan longer.
        if left == 0 {
            finalize_if_drained(&self.stats);
        }
    }
}

// SAFETY (Sync): `Arc<AdoptedSample>` clones cross threads as raw cookies
// (registration on the take thread, release on whichever app thread calls
// `free`), so the type system's `Arc<T>: Send ⇔ T: Send + Sync` rule must
// hold for real. `Send` is auto-derived (`OwnedInboundSample` is Send —
// compile-time-pinned in cerulion_core — and `Arc<AdoptStats>` is Send).
// `Sync` is asserted here with a BOUNDED claim over this type's whole
// `&self` surface (fields private, this module is the only constructor):
//
// - `stats` is atomics — concurrently readable by construction (two app
//   threads freeing two ranges of one sample race only on `fetch_add`s);
// - `payload()` borrows the iceoryx2 `Sample` (upstream Send-but-!Sync),
//   and is called ONLY on the take thread, only BEFORE `take_adopted`
//   returns — at which point no forged pointer has been handed to the app,
//   so no other thread can hold an interest in this sample yet (a free of
//   an address the app was never given is out of contract);
// - `Drop` (the only other touch of `sample`) runs on exactly one thread —
//   the loser of the last strong-count decrement, `Arc`'s own guarantee.
unsafe impl Sync for AdoptedSample {}

/// The process-global release callback (installed once per grant): consumes
/// exactly the `Arc` clone the freed range's registration owned. The hook
/// fires it at most once per registration and by COOKIE identity, so a
/// double release is structurally impossible; the last clone's drop
/// releases the sample's SHM borrow on the app's `free()` thread — sound
/// because `OwnedInboundSample` is `Send` and iceoryx2's
/// `ipc_threadsafe` policy serializes borrow accounting internally (the
/// same contract that lets rclcpp return a loan from another executor
/// thread).
///
/// # Safety
/// `cookie` must be an `Arc::into_raw(Arc<AdoptedSample>)` value handed to
/// `register_segment` by the adoption branch — which is the only producer
/// of cookies under the grant.
pub(crate) unsafe extern "C" fn adopt_release_callback(_ptr: *mut c_void, cookie: usize) {
    // This is an
    // `extern "C"` entry point — the heap hook, a separately-loaded cdylib,
    // calls it — so the crate invariant applies: "wrap every `extern \"C\"`
    // entry-point in `ffi::ffi_guard`". The inner `catch_unwind` below is
    // not enough on its own. It contains the sample drop, but without the
    // guard the `tracing::error!` that REPORTS the containment would sit
    // outside every guard, and `install_tracing` deliberately lets a host's own
    // subscriber win — so a foreign `on_event` that panics would unwind
    // straight across this frame and abort the host, which is the exact
    // outcome the containment exists to prevent. `ffi_guard` reports under
    // a second `catch_unwind` for that same reason.
    //
    // The specific diagnostic is kept INSIDE the guard rather than replaced
    // by `ffi_guard`'s generic one: "the sample is leaked, its borrow slot
    // stays pinned" is the operator-actionable half, and a panicking
    // subscriber costs the generic line instead of the process.
    crate::ffi::ffi_guard((), || {
        // A panic may not unwind into the hook's extern "C" frame (UB). The
        // only panic-capable step is the sample drop's transport accounting;
        // contain it, log, and leak that sample rather than abort the robot.
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            // SAFETY: see the function contract — one clone per cookie,
            // consumed exactly once.
            let adopted = unsafe { Arc::from_raw(cookie as *const AdoptedSample) };
            adopted.stats.releases.fetch_add(1, Ordering::Relaxed);
            drop(adopted);
        }));
        if result.is_err() {
            tracing::error!(
                cookie,
                "adopt-take release callback panicked while releasing a sample — \
                 the sample is leaked (never UB), its borrow slot stays pinned"
            );
        }
    })
}

// ── Per-subscription state ────────────────────────────────────────────────

/// Everything adopt-take stores on `SubscriptionData` when the gate armed
/// at create: the witness, the shared counters, the resolved borrow budget,
/// and the flood latch for the (should-be-never) registration-failure
/// fallback.
pub struct AdoptTakeState {
    /// The witness — the adoption branch takes `&AdoptTakeGrant`.
    pub grant: AdoptTakeGrant,
    /// Shared counters (also registered process-globally for the shutdown
    /// aggregate).
    pub stats: Arc<AdoptStats>,
    /// The CREATE-leg borrow floor this subscription REQUESTED
    /// ([`RMW_ADOPT_TAKE_BORROW_BUDGET`] or the env override).
    pub budget: usize,
    /// The effective
    /// `subscriber_max_borrowed_samples` of the service this subscription
    /// actually attached to — what `ExceedsMaxBorrows` really binds at,
    /// which is SMALLER than [`Self::budget`] when the service already
    /// existed (the floor is CREATE-leg-only). Filled by
    /// `rmw_create_subscription` from the subscriber's own cached read;
    /// the refusal diagnostics report THIS so the operator's remedy math
    /// is right.
    pub effective_budget: usize,
    /// Flood latch for registration-failure fallbacks — its own latch (its
    /// condition is a hook-side registration refusal, not a retained loan
    /// and not a producer-side placement), reported via
    /// `loan_refusal_latch::report_adopt_registration_fallback`.
    pub registration_failures: Mutex<FailureRegimeLatch>,
    /// The latch for a receive that failed for a
    /// reason that is NOT the borrow budget.
    ///
    /// Its OWN, never the borrow-refusal one. `FailureRegimeLatch` is
    /// kind-agnostic and closes only on a SUCCESS, so a consumer retaining
    /// adopted messages keeps the borrow regime open indefinitely (every
    /// take refuses and returns `RMW_RET_OK`, which never closes it) — and a
    /// genuine transport fault arriving during that window would be
    /// suppressed to `debug!`, invisible under the shipped
    /// `rmw_cerulion=warn` filter, while still failing the call. A
    /// recoverable refusal and a hard fault are different conditions with
    /// different remedies, and the shared latch's rule is that one open regime must
    /// not swallow another's loud head.
    pub receive_failures: Mutex<FailureRegimeLatch>,
}

impl AdoptTakeState {
    /// Unconditional running total of registration-failure fallbacks
    /// (independent of log level, never reset — Principle #3).
    pub fn registration_failure_count(&self) -> u64 {
        cerulion_core::transport::failure_regime_latch::lock_regime_latch(
            &self.registration_failures,
        )
        .total_failures()
    }

    /// Is a registration-failure regime OPEN? (Principle #3 — a test
    /// pin reads it to prove a frame that registered NOTHING cannot close
    /// one. The total alone cannot say: a false recovery leaves it
    /// unchanged and only shows up as the next failure being logged loud
    /// again.)
    pub fn registration_regime_open(&self) -> bool {
        cerulion_core::transport::failure_regime_latch::lock_regime_latch(
            &self.registration_failures,
        )
        .is_failing()
    }
}

/// The CREATE-time decision: `Some` iff the env gate is armed AND the
/// grant is constructible AND the type has at least one forgeable
/// sequence (a fixed / non-forgeable type has nothing to adopt — its
/// plain or loaned path already serves it; not a misconfiguration).
/// Env-armed WITHOUT a grant is the misconfiguration: copy path plus ONE
/// loud process-wide warn naming the env var, the missing preload and the
/// DIRECT-launch remedy (the launcher flag is
/// not a remedy — it refuses).
pub(crate) fn arm_for_create(
    topic: &str,
    bridge: &crate::bridge::AnyBridge,
) -> Option<AdoptTakeState> {
    if !env_armed() {
        return None;
    }
    let Some(grant) = AdoptTakeGrant::try_acquire() else {
        warn_missing_preload_once();
        return None;
    };
    if bridge.forged_sequence_count() == 0 {
        return None;
    }
    // NOT registered in the process-global registry here:
    // a later create failure would drop this state but
    // the registry's Arc would leak forever. `rmw_create_subscription`
    // registers the stats in its registry-mutation-LAST block, which only
    // an already-successful create reaches.
    let stats = Arc::new(AdoptStats::default());
    let budget = budget_from_env();
    // NOT the ARMED line: the service does not exist yet, so `effective_budget`
    // is still provisional and the viability gate can still
    // un-arm this. `rmw_create_subscription` makes the one affirmative claim,
    // after that gate — an ARMED `info!` here followed by a NOT-armed `warn!`
    // would tell an operator grepping for it the opposite of the truth.
    tracing::debug!(
        topic = %crate::era_check::escape_control_chars(topic),
        budget,
        "adopt-take requested for this subscription (pending the service's borrow ceiling)"
    );
    Some(AdoptTakeState {
        grant,
        stats,
        budget,
        // Provisional until the subscriber exists — `rmw_create_subscription`
        // overwrites it with the service's real value right after the
        // transport create succeeds.
        effective_budget: budget,
        registration_failures: Mutex::new(FailureRegimeLatch::new()),
        receive_failures: Mutex::new(FailureRegimeLatch::new()),
    })
}

// ── Env reads (CREATE-time, strict) ───────────────────────────────────────
//
// Both reads go through `env::var_os` + a PURE classifier over the raw OS
// value, never `std::env::var`, whose `Err(_)` arm
// collapses `NotPresent` and `NotUnicode` into one silent "unset" (a
// non-UTF-8 value would be ignored with NO diagnostic, contradicting the
// strict-parse promise). A value that FAILS UTF-8 conversion takes the
// SAME loud invalid-value arm as a parseable-but-wrong one (naming the
// var, that the value is not UTF-8, and the accepted forms); the fail-safe
// direction is the same in both arms (gate off / default budget). The classifiers are
// pure over `Option<&OsStr>` so the non-UTF-8 arm is unit-testable (an
// `OsStr` built from raw bytes — the env seams inject strings and cannot
// carry one).

fn env_armed() -> bool {
    classify_gate_value(std::env::var_os(ADOPT_TAKE_ENV).as_deref())
}

fn classify_gate_value(raw: Option<&std::ffi::OsStr>) -> bool {
    let Some(raw) = raw else {
        return false;
    };
    match raw.to_str() {
        Some("1") => true,
        Some("") | Some("0") => false,
        Some(v) => {
            warn_bad_env_value_once(v, "unrecognized");
            false
        }
        None => {
            warn_bad_env_value_once(&raw.to_string_lossy(), "not valid UTF-8");
            false
        }
    }
}

fn budget_from_env() -> usize {
    classify_budget_value(std::env::var_os(ADOPT_TAKE_BUDGET_ENV).as_deref())
}

fn classify_budget_value(raw: Option<&std::ffi::OsStr>) -> usize {
    let Some(raw) = raw else {
        return RMW_ADOPT_TAKE_BORROW_BUDGET;
    };
    match raw.to_str() {
        Some(v) => match v.parse::<usize>() {
            Ok(n) if n >= 1 => n,
            _ => {
                tracing::warn!(
                    value = %crate::era_check::escape_control_chars(v),
                    default = RMW_ADOPT_TAKE_BORROW_BUDGET,
                    "{ADOPT_TAKE_BUDGET_ENV} is not a positive integer — using the default"
                );
                RMW_ADOPT_TAKE_BORROW_BUDGET
            }
        },
        None => {
            tracing::warn!(
                value = %crate::era_check::escape_control_chars(&raw.to_string_lossy()),
                default = RMW_ADOPT_TAKE_BORROW_BUDGET,
                "{ADOPT_TAKE_BUDGET_ENV} is not valid UTF-8 (must be a positive \
                 integer) — using the default"
            );
            RMW_ADOPT_TAKE_BORROW_BUDGET
        }
    }
}

// ── Once-latched misconfiguration warns (+ observables) ───────────────────

static MISSING_PRELOAD_WARNS: AtomicU64 = AtomicU64::new(0);
static BAD_ENV_VALUE_WARNED: AtomicBool = AtomicBool::new(false);

/// How many times the env-armed-but-no-hook warn FIRED (0 or 1 for the
/// process life) — the log-independent oracle the gate-matrix tests pin
/// the once-latch on (Principle #3).
pub fn missing_preload_warn_count() -> u64 {
    MISSING_PRELOAD_WARNS.load(Ordering::Relaxed)
}

/// The remedy half of [`warn_missing_preload_once`], chosen at COMPILE time
/// by the host it will run on.
///
/// The hook interposes glibc's `malloc`/`free` through `LD_PRELOAD`, so on
/// any other host there is no hook to preload and no adoption to reach —
/// telling a macOS reader to preload one would be an impossible remedy,
/// the same class the launcher's own refusal
/// exists to kill. This arm is reachable there: nothing stops a user from
/// exporting the env var on a desk, and the rmw then warns exactly here.
#[cfg(all(target_os = "linux", target_env = "gnu"))]
const MISSING_PRELOAD_REMEDY: &str = "Preload the hook in THIS process: \
     `LD_PRELOAD=<lib dir>/libcerulion_heaphook.so CERULION_RMW_ADOPT_TAKE=1 \
     <node executable>`, launched DIRECTLY. `cerulion ros2 run|launch --adopt-take` \
     cannot do it for you — it refuses, because the `ros2` Python CLI spawns the node \
     as a further subprocess and closes the descriptor the launcher hands the hook \
     over on";

/// See the Linux/GNU arm: on every other host there is nothing to preload,
/// so the remedy states that adoption is unavailable here at all.
#[cfg(not(all(target_os = "linux", target_env = "gnu")))]
const MISSING_PRELOAD_REMEDY: &str = "There is nothing to preload on THIS host: the \
     hook interposes glibc's malloc/free through LD_PRELOAD, so adopt-take is \
     Linux/GNU-ONLY and the copy path is the only path here. Unset \
     CERULION_RMW_ADOPT_TAKE, or move to a Linux/GNU host and preload the hook on a \
     node you launch DIRECTLY";

fn warn_missing_preload_once() {
    if MISSING_PRELOAD_WARNS
        .compare_exchange(0, 1, Ordering::Relaxed, Ordering::Relaxed)
        .is_ok()
    {
        tracing::warn!(
            "{ADOPT_TAKE_ENV}=1 is set but the Cerulion heap hook is not ACTIVE in \
             this process (libcerulion_heaphook.so not preloaded, or its handshake \
             degraded) — adopt-take stays OFF and every take keeps the copy path. \
             {MISSING_PRELOAD_REMEDY}"
        );
    }
}

/// `problem` names WHY the value is unusable ("unrecognized" for a
/// parseable-but-wrong string, "not valid UTF-8" for a value that failed
/// conversion) — one latch, one message shape, two causes.
/// Has the once-latched bad-gate-value warn fired? (Principle #3 — the
/// gate-matrix pin reads it to assert that an UNUSABLE gate value warns
/// while a legitimate `0` stays quiet, which no other observable can
/// distinguish. Never a hot path.) One-way, like the warn itself.
pub fn bad_env_value_warned() -> bool {
    BAD_ENV_VALUE_WARNED.load(Ordering::Relaxed)
}

fn warn_bad_env_value_once(value: &str, problem: &'static str) {
    if !BAD_ENV_VALUE_WARNED.swap(true, Ordering::Relaxed) {
        tracing::warn!(
            value = %crate::era_check::escape_control_chars(value),
            problem,
            "{ADOPT_TAKE_ENV} carries an unusable value — adopt-take stays OFF \
             (set it to 1 to arm, 0/unset to disarm)"
        );
    }
}

// ── Process-global stats registry + the summary lines ─────────────────────

/// Strong refs so a destroyed subscription's counts still reach the
/// shutdown aggregate; bounded by subscription-create count.
static STATS_REGISTRY: OnceLock<Mutex<Vec<Arc<AdoptStats>>>> = OnceLock::new();

/// Register a subscription's stats for the shutdown aggregate. Called by
/// `rmw_create_subscription` in its registry-mutation-LAST block — only a
/// create that already SUCCEEDED reaches it (registering
/// earlier would leak the `Arc` forever on every failed create).
pub(crate) fn register_stats(stats: Arc<AdoptStats>) {
    STATS_REGISTRY
        .get_or_init(|| Mutex::new(Vec::new()))
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .push(stats);
}

/// The counts of subscriptions that have been DESTROYED, folded in so the
/// shutdown aggregate stays exact while the registry above holds only LIVE
/// entries (strong `Arc`s appended per
/// successful create and never removed would let a long-lived process that
/// cycles subscriptions grow the registry without bound — every entry
/// pinned for the lifetime of the process).
///
/// Folding rather than dropping is what keeps the aggregate exact: it is
/// a SUM, so a retired subscription's six numbers add exactly, and the
/// shutdown line reports the same totals as an unretired registry would.
static RETIRED_STATS: OnceLock<AdoptStats> = OnceLock::new();

/// Retire a destroyed subscription's stats: fold its counts into
/// [`RETIRED_STATS`] and drop its registry entry. Called by
/// `rmw_destroy_subscription` AFTER its own proof line, so the per-
/// subscription report is unaffected.
///
/// A subscription with adopted samples
/// still OUTSTANDING is NOT retired. Its `Arc` is shared with every
/// registered range's release cookie, so folding a SNAPSHOT and dropping
/// the registry entry detaches the counters the app's later frees will
/// update — shutdown would then report a stale `released`/`outstanding`
/// pair and could take the outstanding-sample branch on samples that were
/// already freed. Such an entry stays registered and is aggregated LIVE,
/// which is also why there is no double count: an entry is either folded
/// once and removed, or summed from the registry, never both.
///
/// The registry is therefore bounded by live subscriptions PLUS destroyed
/// ones whose samples the app has not yet freed — which is the correct
/// bound, since those counters must remain reachable for exactly that
/// long.
pub(crate) fn retire_stats(stats: &Arc<AdoptStats>) {
    // The subscription is gone; whether its entry can leave the registry
    // depends on whether its samples have drained.
    stats.destroyed.store(true, Ordering::Release);
    finalize_if_drained(stats);
}

/// Fold a DESTROYED and DRAINED subscription's counts into
/// [`RETIRED_STATS`] and drop its registry entry — EXACTLY ONCE.
///
/// This is reached from two places, because either
/// can be last — `retire_stats` when destroy happens after the samples
/// drained, and `AdoptedSample::drop` when the final sample is released
/// after destroy. With only the first, a subscription
/// destroyed while holding a sample would never be retired at all.
///
/// Everything happens under the REGISTRY lock, which is what makes it safe
/// against the shutdown scan: that scan holds the same lock while it sums
/// the live entries and reads `RETIRED_STATS`, so it can observe this
/// entry as live-and-unfolded or folded-and-gone, never both and never
/// neither. The `retired` flag makes the fold once-only under concurrent
/// frees of two ranges of the same sample.
fn finalize_if_drained(stats: &Arc<AdoptStats>) {
    if !stats.destroyed.load(Ordering::Acquire) {
        return;
    }
    let Some(registry) = STATS_REGISTRY.get() else {
        return;
    };
    let mut list = registry.lock().unwrap_or_else(|e| e.into_inner());
    if stats.outstanding.load(Ordering::Relaxed) > 0 {
        return;
    }
    if stats.retired.swap(true, Ordering::AcqRel) {
        return;
    }
    let retired = RETIRED_STATS.get_or_init(AdoptStats::default);
    let s = snapshot(stats);
    retired.takes.fetch_add(s.0, Ordering::Relaxed);
    retired.adopted_takes.fetch_add(s.1, Ordering::Relaxed);
    retired.releases.fetch_add(s.2, Ordering::Relaxed);
    retired.outstanding.fetch_add(s.3, Ordering::Relaxed);
    retired.fallbacks.fetch_add(s.4, Ordering::Relaxed);
    retired.budget_refusals.fetch_add(s.5, Ordering::Relaxed);
    if let Some(at) = list.iter().position(|held| Arc::ptr_eq(held, stats)) {
        list.swap_remove(at);
    }
}

/// How many stats entries the process-global registry holds (Principle #3
/// — the regression oracle: a FAILED subscription create must not grow
/// it, and a DESTROYED one must shrink it back).
pub fn registered_stats_count() -> usize {
    STATS_REGISTRY
        .get()
        .map(|r| r.lock().unwrap_or_else(|e| e.into_inner()).len())
        .unwrap_or(0)
}

/// One summary snapshot — the six numbers the bench proof line carries.
fn snapshot(stats: &AdoptStats) -> (u64, u64, u64, u64, u64, u64) {
    (
        stats.takes.load(Ordering::Relaxed),
        stats.adopted_takes.load(Ordering::Relaxed),
        stats.releases.load(Ordering::Relaxed),
        stats.outstanding.load(Ordering::Relaxed),
        stats.fallbacks.load(Ordering::Relaxed),
        stats.budget_refusals.load(Ordering::Relaxed),
    )
}

/// The per-subscription proof line at destroy (the exact field
/// spellings `takes=`/`adopted=`/`released=`/`outstanding=`/`fallbacks=`/
/// `budget_refusals=` are the bench parsing contract; keep them stable),
/// plus the warn-and-LEAVE arm when adopted ranges are outstanding.
pub(crate) fn log_destroy_summary(topic: &str, stats: &AdoptStats) {
    let (takes, adopted, released, outstanding, fallbacks, budget_refusals) = snapshot(stats);
    tracing::info!(
        topic = %crate::era_check::escape_control_chars(topic),
        takes,
        adopted,
        released,
        outstanding,
        fallbacks,
        budget_refusals,
        "adopt-take summary"
    );
    if outstanding > 0 {
        tracing::warn!(
            topic = %crate::era_check::escape_control_chars(topic),
            outstanding,
            "subscription destroyed with adopted samples outstanding — their \
             registrations are LEFT IN PLACE (reclaiming would dangle the app's \
             forged pointers); each later free of a forged range still releases \
             its sample"
        );
    }
}

// ── The ONE process-wide lifecycle lock ───────────────────────

/// The LIVE CONTEXT SET — keyed by context IDENTITY — UNDER a mutex (not
/// an atomic): the membership transition, the final-context decision, the
/// destructive callback CLEAR, and every grant's callback INSTALL must be
/// mutually serialized — with an atomic count, a concurrent `rmw_init` +
/// armed create could install the callback between a final shutdown's
/// "nothing else live" read and its clear, and the fresh context's adopted
/// samples would then free onto the hook's no-callback leak path
/// (the lifecycle race).
///
/// A set, not a count: rmw.h makes a repeat
/// `rmw_shutdown` on an already-shut-down context a NO-OP, and an unkeyed
/// count cannot tell that repeat from a genuine shutdown — with two
/// contexts live, shutting the SECOND down twice drives the count to zero
/// and clears the callback the FIRST, still-live context's frees need,
/// so its later frees take the hook's no-callback path and pin its
/// samples and borrow slots. Keyed membership makes the repeat provably a
/// no-op, and `len()` IS the count.
///
/// One lock, four entry points: [`context_started`], [`context_ended`],
/// [`context_is_live`], and `AdoptTakeGrant::try_acquire`'s install.
/// Leaf-ordered: holders may take the stats-registry mutex and the hook's
/// own locks, never the reverse.
static LIFECYCLE: Mutex<BTreeSet<usize>> = Mutex::new(BTreeSet::new());

fn lifecycle_lock() -> std::sync::MutexGuard<'static, BTreeSet<usize>> {
    LIFECYCLE.lock().unwrap_or_else(|e| e.into_inner())
}

/// The identity a context is keyed on: its address. rcl allocates the
/// `rmw_context_t` inside its heap-held context impl and hands that ONE
/// address to every rmw call for the context's lifetime, and
/// `rmw_context_fini` zeroes it in place — so the address is the context.
fn context_key(context: *const crate::ffi::rmw_context_t) -> usize {
    context as usize
}

/// One more live rmw context (called by `rmw_init` BEFORE it writes the
/// context, so a refusal leaves the caller's struct untouched). `false` if
/// that context is ALREADY live — which rmw.h says `rmw_init` refuses with
/// `RMW_RET_INVALID_ARGUMENT` ("if context has been already initialized").
pub(crate) fn context_started(context: *const crate::ffi::rmw_context_t) -> bool {
    lifecycle_lock().insert(context_key(context))
}

/// Undo a [`context_started`] whose init then FAILED — the plain removal,
/// with none of `context_ended`'s shutdown behaviour.
///
/// `rmw_init` registers the context before
/// writing it (so an already-live context is refused with the caller's
/// struct untouched), which leaves a window where later fallible work — the
/// post-registration `warn!`/`info!`, whose subscriber belongs to the HOST
/// and may panic — can fail after the entry exists. `ffi_guard` then returns
/// `RMW_RET_ERROR` while the live set still holds an entry that only a
/// shutdown could remove, and the caller has been told the init failed, so
/// no shutdown is coming.
///
/// Deliberately NOT `context_ended`: that logs the hook counters and a
/// shutdown summary and, on the last context, clears the release callback —
/// reporting the end of a session that never began, and taking a
/// process-global action on behalf of a context that never served a take.
/// A rollback removes exactly what the failed init added.
pub(crate) fn context_start_rolled_back(context: *const crate::ffi::rmw_context_t) {
    let mut live = lifecycle_lock();
    if !live.remove(&context_key(context)) {
        return;
    }
    // A rollback that empties the live set is the
    // last context leaving, and it must clear the release callback for the
    // same reason `context_ended` does.
    //
    // The reachable order is not exotic: an older context shuts down while
    // this one is PROVISIONALLY in the set (it is inserted before the
    // caller's struct is written), so that shutdown sees a non-empty set,
    // correctly decides it is not final, and KEEPS the callback — a
    // surviving context's frees still need it. Then this init fails.
    // Removing only the provisional entry would leave the process
    // with no contexts and a callback still pointing into this library: a
    // later free of a still-registered range calls an address that may
    // since have been unmapped.
    //
    // Under the SAME lock hold as the removal, so a concurrent init cannot
    // observe the empty set between the two and install a callback this
    // clear then wipes.
    if live.is_empty() {
        clear_release_callback();
    }
}

/// Is this context live (initialized and not yet shut down)? The
/// `rmw_context_fini` contract check: fini on a valid context that was
/// never shut down is `RMW_RET_INVALID_ARGUMENT` with the context
/// unchanged — and zeroing it would orphan its live-set entry.
pub(crate) fn context_is_live(context: *const crate::ffi::rmw_context_t) -> bool {
    lifecycle_lock().contains(&context_key(context))
}

/// Clear the process-global release callback (final shutdown only).
///
/// A callback left installed after the last
/// context is gone points into a module the process may unmap, and the hook
/// would call it for any range still registered. Idempotent and sound at any
/// time — the hook simply routes later frees onto its no-callback path.
fn clear_release_callback() {
    // The mirror is lowered FIRST. Both writers hold the lifecycle
    // lock, so the flag cannot end up claiming a callback that is gone, and a
    // take that reads it between this store and the hook call simply takes the
    // copy path one call early — the safe direction.
    RELEASE_CALLBACK_EPOCH.store(0, Ordering::Release);
    if let Some(api) = heaphook::active_hook() {
        // SAFETY: clearing the process-global callback; sound at any time.
        let _ = unsafe { (api.set_release_callback)(None) };
    }
}

/// Is `adopt_release_callback`
/// CURRENTLY the process-global release callback?
///
/// An [`AdoptTakeGrant`] proves the hook was live when the SUBSCRIPTION was
/// created; it does not prove the callback is still installed when a take
/// runs. `context_ended` clears the callback on the FINAL context's
/// `rmw_shutdown`, and nothing in rmw or rcl guarantees every subscription is
/// destroyed first — an executor still spinning, or node destructors running
/// after `rclcpp::shutdown()`, can both take afterwards. Such a take would
/// forge pointers and register ranges with a hook that has no callback to
/// call, so the app's later `free()` takes the hook's no-callback path
/// (`release_without_callback`): the range is LEAKED by design rather than
/// wrongly freed, the `Arc<AdoptedSample>` cookie is never reclaimed,
/// `outstanding` never drains, and the SHM borrow plus its publisher-pool
/// slot are pinned for the life of the process. Enough such takes exhaust
/// that subscription's retention budget permanently.
///
/// The adoption branch reads this before dispatching and falls back to the
/// copying take when it is 0.
///
/// The check-then-register race is CLOSED without a lock. Closing it does
/// not mean holding the lifecycle lock across the take (a mutex on the
/// hot path): the take RE-READS this epoch
/// after its registrations land and, if it changed, withdraws them and serves
/// by copy through the rollback the registration-failure arm already owns.
/// Two relaxed-cost atomic loads and a rollback that only runs when the race
/// is actually lost — no lock, and no window in which a registration survives
/// a shutdown. The withdrawal needs no callback (`unregister_segment` is the
/// caller-side half), and the app cannot have freed anything yet because the
/// take has not returned the message to it.
pub fn release_callback_epoch() -> u64 {
    RELEASE_CALLBACK_EPOCH.load(Ordering::Acquire)
}

/// The epoch of the CURRENTLY installed release callback, or 0 when
/// none is installed.
///
/// A bare boolean could only answer "installed?", which is not the question a
/// take needs answered. The take must know that the callback it checked for is
/// the SAME one still installed when its registrations land — and an
/// install/clear/install pair between those two moments leaves a boolean
/// reading `true` both times while the registrations belong to a generation
/// that is gone. Every transition takes a fresh number, so the take compares
/// identities rather than presence.
///
/// Written only by [`AdoptTakeGrant::try_acquire`] and
/// [`clear_release_callback`], both under the lifecycle lock, and published
/// with RELEASE so an ACQUIRE reader that sees a non-zero epoch also sees the
/// hook's callback store.
static RELEASE_CALLBACK_EPOCH: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Source of epoch ids. Never reset; only ever read under the lifecycle lock.
static NEXT_RELEASE_CALLBACK_EPOCH: std::sync::atomic::AtomicU64 =
    std::sync::atomic::AtomicU64::new(0);

/// Clear the callback from a TEST SEAM, to drive the shutdown race
/// deterministically — the seam fires inside `take_adopted`'s forge window,
/// i.e. after the take checked the epoch and before its registrations land.
/// Re-install the callback from a TEST SEAM, modelling a concurrent
/// `rmw_create_subscription` landing inside a take's forge window — the shape
/// that must be INVISIBLE to the epoch check, because the pointer installed is
/// the one already installed.
#[cfg(feature = "test-seams")]
pub(crate) fn reinstall_release_callback_for_seam() {
    let _ = AdoptTakeGrant::try_acquire();
}

#[cfg(feature = "test-seams")]
pub(crate) fn clear_release_callback_for_seam() {
    let _lifecycle = lifecycle_lock();
    clear_release_callback();
}

/// The number of live rmw contexts (Principle #3 — the lifecycle pins read
/// it; never a hot path).
pub fn live_context_count() -> usize {
    lifecycle_lock().len()
}

/// A context ended (called by `rmw_shutdown`). `false` — having done
/// NOTHING — when the context is not live: a repeat shutdown of an
/// already-shut-down context, the rmw.h no-op (no decrement, no summary,
/// and above all no clear of a callback a still-live context's frees
/// need). On a genuine first removal, the heap hook's diagnostic counters
/// (this rmw is their in-process reader, the hook's documented dependency,
/// logged BEFORE the summary), the final-context decision, the summary,
/// and — final context only — the destructive clear all run under ONE
/// lock scope, so no concurrent init/grant-install can interleave the
/// decision and the clear.
pub(crate) fn context_ended(context: *const crate::ffi::rmw_context_t) -> bool {
    let mut live = lifecycle_lock();
    if !live.remove(&context_key(context)) {
        return false;
    }
    heaphook::log_hook_counters();
    let final_context = live.is_empty();
    // The clear runs on
    // EVERY final shutdown, not only when samples are still outstanding.
    // Inside the summary's `outstanding > 0` arm it would miss a
    // process that adopted and then freed everything — the ordinary,
    // healthy shape — which would shut down leaving a process-global
    // callback pointing into this module. Any later free of a range still registered with
    // the hook would call into it, and nothing about shutdown guarantees
    // the module is still mapped. Clearing is sound at any time and cheap;
    // it is the outstanding-sample LEAK, not the clear, that the summary's
    // arm is about. It also sits HERE rather than in the summary because
    // the summary returns early when nothing was ever registered, and a
    // grant installs the callback independently of whether any stats entry
    // was ever made.
    if final_context {
        clear_release_callback();
    }
    log_shutdown_summary_locked(final_context);
    true
}

/// The process-wide proof line at `rmw_shutdown` (beside
/// `log_hook_counters`) plus — on the FINAL context's shutdown only — the
/// leak-and-count arm: with adopted ranges still outstanding, CLEAR the
/// release callback and deliberately leak the outstanding Arcs (see the
/// module doc's shutdown paragraph). Silent when adopt-take never armed.
///
/// `final_context` comes from the live-context set — empty once this
/// context is removed: the callback is
/// PROCESS-global, so an intermediate
/// context's shutdown must never clear it — a surviving context's adopted
/// samples still release through it on the app's frees. An intermediate
/// shutdown prints the summary and, when samples are outstanding, says
/// loudly that the callback is KEPT. Runs with the [`LIFECYCLE`] lock HELD
/// (the caller is [`context_ended`]), which is what closes the lifecycle race.
fn log_shutdown_summary_locked(final_context: bool) {
    let Some(registry) = STATS_REGISTRY.get() else {
        return;
    };
    let list = registry.lock().unwrap_or_else(|e| e.into_inner());
    if list.is_empty() && RETIRED_STATS.get().is_none() {
        return;
    }
    // The retired subscriptions' folded counts are part of the
    // aggregate — the registry holds only live entries, so leaving
    // them out would silently shrink every number a cycling process
    // reports.
    let mut total = RETIRED_STATS
        .get()
        .map(snapshot)
        .unwrap_or((0, 0, 0, 0, 0, 0));
    for stats in list.iter() {
        let s = snapshot(stats);
        total = (
            total.0 + s.0,
            total.1 + s.1,
            total.2 + s.2,
            total.3 + s.3,
            total.4 + s.4,
            total.5 + s.5,
        );
    }
    let (takes, adopted, released, outstanding, fallbacks, budget_refusals) = total;
    tracing::info!(
        takes,
        adopted,
        released,
        outstanding,
        fallbacks,
        budget_refusals,
        "adopt-take summary"
    );
    if outstanding > 0 {
        if !final_context {
            tracing::warn!(
                outstanding,
                "rmw_shutdown with adopted samples outstanding while OTHER rmw \
                 contexts are still live — the release callback is KEPT (clearing \
                 it here would route the surviving contexts' later frees onto the \
                 hook's no-callback leak path, pinning their samples and borrow \
                 slots); the final context's shutdown runs the clear-and-leak arm"
            );
            return;
        }
        tracing::warn!(
            outstanding,
            "rmw_shutdown with adopted samples outstanding — the release callback \
             is CLEARED (a post-shutdown free must never call into a possibly- \
             unmapped module; the hook now leaks-never-frees those ranges, counter \
             kind 0) and the outstanding samples are deliberately leaked to process \
             exit; the rmw runtime and its SHM mappings are process-lifetime \
             statics, so the app's forged pointers stay readable"
        );
    }
}

#[cfg(test)]
mod tests {
    /// Adoption is viable only where the service's borrow
    /// ceiling leaves room for the message the app still holds AND the
    /// receive that is arriving. Hand-written table; the boundary is pinned
    /// on BOTH sides, because a gate that is one off in either direction is
    /// the whole defect (too low re-opens the wedge, too high disables
    /// adoption on a service that works).
    #[test]
    fn adopt_viability_matches_its_oracle_table() {
        let oracle: &[(usize, AdoptViability, &str)] = &[
            (
                0,
                AdoptViability::BorrowCeilingTooSmall,
                "a ceiling of zero lends nothing at all",
            ),
            (
                1,
                AdoptViability::BorrowCeilingTooSmall,
                "THE wedge: the held sample takes the only unit, so the receive that would \
                 release it is refused",
            ),
            (
                2,
                AdoptViability::Viable,
                "the boundary: one held + one arriving is exactly enough for a caller that \
                 reuses ONE buffer (an N-buffer caller needs N+1 — see the constant's doc)",
            ),
            (3, AdoptViability::Viable, "above the boundary"),
            (
                RMW_ADOPT_TAKE_BORROW_BUDGET,
                AdoptViability::Viable,
                "the shipped default budget",
            ),
            (usize::MAX, AdoptViability::Viable, "saturated"),
        ];
        for &(ceiling, want, why) in oracle {
            assert_eq!(
                adopt_viability(ceiling),
                want,
                "effective_budget={ceiling}: {why}"
            );
        }
    }

    /// The constant the gate is written against, pinned so a change to it is
    /// a deliberate act: at 2 a caller reusing ONE message never wedges.
    /// (That the shipped default budget clears the minimum is const-asserted
    /// beside the constants, so it cannot compile wrong.)
    #[test]
    fn the_adopt_minimum_is_the_held_plus_arriving_pair() {
        assert_eq!(
            MIN_ADOPT_EFFECTIVE_BUDGET, 2,
            "one unit for the message the app still holds, one for the incoming receive"
        );
    }

    use super::*;
    use std::ffi::OsStr;
    use std::os::unix::ffi::OsStrExt;
    use tracing_test::traced_test;

    /// A value no UTF-8 conversion can accept (0xFF is never valid UTF-8).
    /// Built from raw bytes because the env seams inject STRINGS and cannot
    /// carry one — which is exactly why the classifiers are pure over
    /// `Option<&OsStr>`.
    fn non_utf8() -> &'static OsStr {
        OsStr::from_bytes(b"\xff\xfe1")
    }

    #[test]
    fn the_gate_classifier_arms_only_on_the_documented_forms() {
        assert!(!classify_gate_value(None), "unset disarms, silently");
        assert!(classify_gate_value(Some(OsStr::new("1"))), "1 arms");
        assert!(!classify_gate_value(Some(OsStr::new("0"))), "0 disarms");
        assert!(
            !classify_gate_value(Some(OsStr::new(""))),
            "empty disarms silently (the deliberate unset-equivalent)"
        );
    }

    /// A non-UTF-8 gate value must take the
    /// SAME loud invalid-value arm as a parseable-but-wrong one — never the
    /// silent unset path — while failing SAFE (gate off). ONE body drives
    /// both unusable shapes in order because the warn is once-latched
    /// process-wide (`BAD_ENV_VALUE_WARNED`), and it doubles as the pin
    /// that both shapes share that one latch.
    #[test]
    #[traced_test]
    fn a_non_utf8_gate_value_warns_loudly_and_fails_safe() {
        assert!(
            !classify_gate_value(Some(non_utf8())),
            "fails safe: the gate stays OFF"
        );
        logs_assert(|lines: &[&str]| {
            let warns: Vec<&&str> = lines
                .iter()
                .filter(|l| l.contains("WARN") && l.contains("carries an unusable value"))
                .collect();
            if warns.len() != 1 {
                return Err(format!("expected exactly 1 gate warn, got {warns:?}"));
            }
            let line = warns[0];
            for needle in [
                "CERULION_RMW_ADOPT_TAKE carries",
                "not valid UTF-8",
                "set it to 1",
            ] {
                if !line.contains(needle) {
                    return Err(format!("gate warn missing `{needle}`: {line}"));
                }
            }
            Ok(())
        });
        // The parseable-but-wrong shape rides the SAME once-latch: no
        // second line for the rest of the process.
        assert!(!classify_gate_value(Some(OsStr::new("bogus"))));
        logs_assert(|lines: &[&str]| {
            let count = lines
                .iter()
                .filter(|l| l.contains("WARN") && l.contains("carries an unusable value"))
                .count();
            if count == 1 {
                Ok(())
            } else {
                Err(format!(
                    "the invalid-value warn must stay once-latched, got {count}"
                ))
            }
        });
    }

    /// The budget classifier's full decision table — unset is
    /// the silent default, a valid positive integer is honored, and BOTH
    /// unusable shapes (non-numeric, zero, non-UTF-8) warn loudly at WARN
    /// while serving the default (the budget warns are per-call, not
    /// latched — creates are rare).
    #[test]
    #[traced_test]
    fn the_budget_classifier_defaults_loudly_on_unusable_values() {
        assert_eq!(
            classify_budget_value(None),
            RMW_ADOPT_TAKE_BORROW_BUDGET,
            "unset takes the default silently"
        );
        assert_eq!(classify_budget_value(Some(OsStr::new("7"))), 7);
        assert_eq!(
            classify_budget_value(Some(non_utf8())),
            RMW_ADOPT_TAKE_BORROW_BUDGET,
            "non-UTF-8 fails safe to the default"
        );
        assert_eq!(
            classify_budget_value(Some(OsStr::new("abc"))),
            RMW_ADOPT_TAKE_BORROW_BUDGET
        );
        assert_eq!(
            classify_budget_value(Some(OsStr::new("0"))),
            RMW_ADOPT_TAKE_BORROW_BUDGET
        );
        logs_assert(|lines: &[&str]| {
            let utf8_warns = lines
                .iter()
                .filter(|l| {
                    l.contains("WARN")
                        && l.contains("CERULION_RMW_ADOPT_TAKE_BUDGET is not valid UTF-8")
                })
                .count();
            if utf8_warns != 1 {
                return Err(format!(
                    "expected exactly 1 non-UTF-8 budget warn, got {utf8_warns}"
                ));
            }
            let parse_warns = lines
                .iter()
                .filter(|l| {
                    l.contains("WARN")
                        && l.contains("CERULION_RMW_ADOPT_TAKE_BUDGET is not a positive integer")
                })
                .count();
            if parse_warns != 2 {
                return Err(format!(
                    "expected 2 parse warns (abc, 0), got {parse_warns}"
                ));
            }
            Ok(())
        });
    }

    /// Log injection: an environment value
    /// reaches the log through the repo's control-character escaping, so a
    /// value carrying CR/LF cannot forge a second log line — the operator
    /// sees one warn whose `value=` field renders the newline as `\u{a}`.
    ///
    /// The oracle is BOTH halves. That the escaped form is present is not
    /// enough on its own: a reporter that logged the raw value would ALSO
    /// contain the visible characters around it, so the arm additionally
    /// requires the forged fragment never to appear at the start of a line
    /// — which is the whole of the injection.
    #[test]
    #[traced_test]
    fn a_budget_value_carrying_control_characters_is_escaped_in_the_warn() {
        assert_eq!(
            classify_budget_value(Some(OsStr::new("9\nWARN forged: adopt-take disabled"))),
            RMW_ADOPT_TAKE_BORROW_BUDGET,
            "the value is unusable, so the default stands"
        );
        logs_assert(|lines: &[&str]| {
            let escaped = lines
                .iter()
                .filter(|l| l.contains("value=9\\u{a}WARN forged"))
                .count();
            if escaped != 1 {
                return Err(format!(
                    "expected the newline rendered as an escape in exactly 1 line, got \
                     {escaped}; lines: {lines:?}"
                ));
            }
            if lines
                .iter()
                .any(|l| l.trim_start().starts_with("WARN forged"))
            {
                return Err(
                    "the value forged a line of its own — the escaping did not run".to_string(),
                );
            }
            Ok(())
        });
    }
}
