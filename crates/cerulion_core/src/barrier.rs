// SPDX-License-Identifier: AGPL-3.0-only
//! Lock-free, SHM-layout-compatible, count-down sense-reversing cross-process
//! barrier state machine — the counting / rendezvous primitive for p4's
//! cross-process DAG-level barrier.
//!
//! # Purpose
//!
//! In p4 a DAG graph is executed by multiple processes in LOCKSTEP: every
//! process advances level-by-level, and before any process may begin level
//! `L+1` it must wait until EVERY participant has finished level `L`. That
//! rendezvous is a counting barrier. [`BarrierShared`] is the lock-free
//! counter + generation state machine behind it: each process [`arrive`](BarrierShared::arrive)s
//! at a generation, the LAST arriver (the one that counts the participant total
//! down to zero) bumps the generation — which IS the release signal — and the
//! rest observe the bump and proceed.
//!
//! The decided mechanism is **SHM atomic counter + kernel wake**. This module
//! builds the counter / state machine ([`BarrierShared`]) AND the SHM page
//! mapping ([`MappedBarrier`] — mmap'ing a [`BarrierShared`] into a `MAP_SHARED`
//! page shared across processes) AND the blocking [`wait`](BarrierShared::wait)
//! — on Linux a spin-then-block PROCESS-SHARED futex on the `generation` word,
//! kernel-woken by the opener's `FUTEX_WAKE` right after its generation store
//! (the earlier shallow CPU monitor-wait park was measured NOT to
//! observe the cross-process store, costing a ~100µs timer backstop per
//! boundary). On macOS ≥ 14.4 the wake is the SAME shape via Apple's
//! `os_sync_wake_by_address_all` on the `generation` word, resolved at runtime
//! through `dlsym` — never a direct extern, so an OLDER macOS lacking
//! the symbol degrades cleanly to the chunked sleep-recheck ladder instead of
//! aborting at launch. Either way the wait is a wake-assisted spin-then-block
//! (never a busy-spin). There is a SECOND kernel-woken word: the
//! step-start park's `wake_seq` epoch + `parked` rank bitmask — the live
//! loop's `monitor_wait_block` kernel-blocks on `wake_seq` (futex / os_sync,
//! the same tiers) so a peer's ARRIVAL (not just an open) wakes a parked
//! context in µs; arrivers bump the epoch unconditionally and gate only the
//! wake SYSCALL on the parked mask, keeping `arrive` atomics-only when nobody
//! parks. The scheduler integration that drives `arrive` at each level
//! boundary lives in `graph/runtime.rs`. Here we deliver a self-contained,
//! hermetically testable algorithm plus its cross-process mapping and wait.
//!
//! # Determinism firewall (NON-NEGOTIABLE)
//!
//! The barrier is a **WHEN gate on level advance** — it changes only WHEN a
//! process is permitted to proceed to the next level, NEVER WHAT fires. The
//! generation counter is a rendezvous signal, exactly like the
//! [`crate::doorbell`] seq: it is never consumed as data, never feeds a node
//! body, never reorders which nodes fire or with what payload. Replay stays
//! identical to live (Principle #7) because the barrier only synchronizes the
//! boundary, leaving the per-level fire set + data flow untouched. Same firewall
//! as [`crate::doorbell`] and [`crate::monitor_wait`].
//!
//! # SHM-layout compatibility
//!
//! [`BarrierShared`] is `#[repr(C)]` and contains ONLY atomics + fixed-size
//! fields (no `Mutex`, no `Condvar`, no heap pointer, no `std::sync::Barrier` —
//! that one is intra-process and not SHM-mappable). So `MappedBarrier` can place a
//! [`BarrierShared`] directly in an mmap'd `MAP_SHARED` page and have two
//! processes operate on the SAME physical atomics. Every operation is lock-free
//! (atomic load / `fetch_sub` / `fetch_add` / store / `fetch_update`), so a
//! crashed participant can never leave the barrier holding a lock.
//!
//! # Count-down sense-reversing design
//!
//! Two fields carry the state: `generation` (the SENSE) and `remaining` (a
//! single per-generation count-DOWN counter). Waiters poll the SENSE —
//! [`is_open`](BarrierShared::is_open) is `generation > my_gen` — so there is NO
//! accumulator slot for a second actor to wipe. Within a generation, `remaining`
//! starts at `expected` and each [`arrive`](BarrierShared::arrive) does a single
//! atomic `fetch_sub(1)`; the caller whose `fetch_sub` returns `prev == 1`
//! brought it to exactly 0 and is the UNIQUE last arriver. That unique opener
//! re-arms `remaining` from `expected` for the next generation BEFORE publishing
//! the new generation (so a waiter that observes the bump via its `Acquire` load
//! is guaranteed to see the re-armed `remaining` it is about to count in), then
//! stores `generation = my_gen + 1`.
//!
//! Because BOTH [`arrive`](BarrierShared::arrive) and
//! [`drop_participant`](BarrierShared::drop_participant) reach 0 through the SAME
//! atomic `fetch_sub`, exactly one of them ever observes `prev == 1` — so the
//! opener is unique even when a live arrival races a dead-peer drop. There is no
//! duplicate `Opened`, and (unlike a two-slot accumulator design) no second
//! opener that could reset a counter AFTER a fast participant has already
//! advanced into the next generation, which would silently lose that arrival.
//! (The degenerate `expected == 0` path has no `fetch_sub`; it singles out its
//! unique opener with a CAS on `generation` instead, so the one-opener guarantee
//! holds UNIVERSALLY — no path is exempt.)
//!
//! # The one-generation max-skew invariant
//!
//! A SINGLE `remaining` counter is correct ONLY because **no participant is ever
//! more than ONE generation behind any other.** In lockstep DAG execution every
//! participant hits every level-barrier in order (no skipping), so the maximum
//! skew between the fastest and slowest participant is exactly one generation.
//! If a participant were ever TWO generations behind, it would `fetch_sub` the
//! WRONG generation's `remaining` — decrementing the counter the fast cohort is
//! using for a later generation, corrupting its count. The lockstep contract
//! rules that out; this is the load-bearing assumption the whole primitive rests
//! on. (The same [`wait`](BarrierShared::wait) that gates every participant on the
//! SENSE is what enforces the bound: no one enters `G + 1` until `G` opened.)

use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::time::{Duration, Instant};

// The name fold both `barrier_shm_name` variants derive their object name with.
// Imported rather than written out here: `shm_ring`, `state_arm` and `credit`
// carried byte-identical copies of it until the `shm_map` extraction, and four
// spellings of one hash is a drift this repo has already paid for.
use crate::shm_map::fnv1a64;

// The Apple `os_sync_*` FFI scalars used by the macOS barrier-wake
// tier (`c_void` addr). Imported only where that tier compiles so no
// unused-import fires on Linux / other OSes.
#[cfg(target_os = "macos")]
use core::ffi::c_void;
// The SHARED os_sync dlsym backend + errno classifiers +
// kill-switch grammar — extracted to `crate::os_sync` so the monitor-wait
// park's nap tier shares ONE resolution and ONE unusable-latch with
// the barrier instead of a second dlsym copy (the one-copy rule). Semantics
// byte-identical to the pre-extraction in-module definitions.
#[cfg(target_os = "macos")]
use crate::os_sync::{
    os_sync_backend, os_sync_errno_is_benign, os_sync_errno_is_unrecoverable,
    parse_os_sync_kill_switch,
};

/// Outcome of an [`arrive`](BarrierShared::arrive) /
/// [`drop_participant`](BarrierShared::drop_participant) attempt.
///
/// Exactly ONE caller per generation observes [`Opened`](ArriveOutcome::Opened),
/// ALWAYS — the winner is singled out atomically on every path: the `fetch_sub`
/// that counts `remaining` down to 0 on the normal path (unique even when a live
/// arrival races a dead-peer drop), or the generation CAS on the vacuous
/// (`expected == 0`) path. Every other caller of that generation sees
/// [`Pending`](ArriveOutcome::Pending).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArriveOutcome {
    /// This caller was the one that completed the generation (counted `remaining`
    /// to 0). Carries the NEW generation (`my_gen + 1`), now open for everyone.
    Opened(u64),
    /// The generation is not complete yet — other participants have not all
    /// arrived. The caller should wait (poll [`is_open`](BarrierShared::is_open)
    /// / the future doorbell) until the generation opens.
    Pending,
}

/// Outcome of a parking [`BarrierShared::wait`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WaitOutcome {
    /// Generation `my_gen` opened — the caller may proceed to the next level.
    Opened,
    /// The wait `timeout` elapsed before the generation opened. **NOT a
    /// proceed signal** — the caller must treat this as fatal/retry (e.g. a
    /// crashed peer that never arrived), NEVER advance past peers that have not
    /// rendezvoused (a determinism + ordering breach). It is also the only
    /// survivable exit when a peer dies mid-rendezvous: no wake (futex
    /// wake, slice timeout, recheck) can rescue a generation no one will ever
    /// open, so a finite `timeout → TimedOut` is the bounded escape.
    TimedOut,
}

/// Outcome of a [`try_drop_participant`](BarrierShared::try_drop_participant): the
/// UNAMBIGUOUS form of [`drop_participant`](BarrierShared::drop_participant) that a
/// supervisor's retry loop needs.
///
/// [`drop_participant`](BarrierShared::drop_participant) folds two very different
/// cases into one ambiguous [`ArriveOutcome::Pending`]: "the target generation was
/// ALREADY open, so this drop touched NOTHING (a full no-op)" and "the drop APPLIED
/// but did not complete the generation". A retry loop cannot act on that fork — it
/// must know whether to STOP (the drop landed) or RE-READ the current generation
/// and retry (the drop was a stale no-op because the generation advanced between
/// the read and the call). `DropAttempt` splits exactly that fork; an
/// [`expected`](BarrierShared::expected)-delta probe cannot, because it is racy
/// against a concurrent worker self-drop that ALSO moves `expected`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DropAttempt {
    /// The `my_gen` passed was NOT the current generation (it was already open) —
    /// the drop was a FULL no-op: neither `expected` nor `remaining` was touched.
    /// The caller should re-read
    /// [`current_generation`](BarrierShared::current_generation) and retry the drop
    /// at the new generation.
    Stale,
    /// The drop APPLIED: `expected` was decremented (saturating) and the dead peer
    /// was counted out of the current generation with the same `fetch_sub` an
    /// arrival uses. Carries the underlying [`ArriveOutcome`] —
    /// [`Opened`](ArriveOutcome::Opened) if this drop completed the generation,
    /// else [`Pending`](ArriveOutcome::Pending).
    Applied(ArriveOutcome),
}

/// Outcome of an OWNER-side [`drop_dead_peer`](BarrierShared::drop_dead_peer).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeadPeerDrop {
    /// The drop was applied to the cohort. `opened` is `true` if THIS drop was the
    /// one that completed the current generation (counted `remaining` to 0),
    /// releasing the survivors into the next generation.
    Applied {
        /// Whether this drop completed (opened) the current generation.
        opened: bool,
    },
    /// The cohort was already empty (`expected == 0`) — there was no participant
    /// slot to drop. A bookkeeping error at the call site (a peer dropped from a
    /// zero-participant cohort); the caller logs it loudly.
    NoSlots,
    /// The bounded apply loop exhausted its retries (the generation advanced on
    /// every attempt). Should be UNREACHABLE in a single-supervisor deployment
    /// (only concurrent generation churn from many live peers could sustain it);
    /// the caller logs it loudly.
    ExhaustedRetries,
}

/// A lock-free, `#[repr(C)]`, SHM-mappable count-down sense-reversing counting
/// barrier.
///
/// See the [module docs](self) for the purpose, the determinism firewall, the
/// count-down design, and the one-generation max-skew invariant. Construct with
/// [`new`](Self::new); participants [`arrive`](Self::arrive) and then wait until
/// the generation opens.
///
/// `#[repr(C)]` + atomics-only so `MappedBarrier` can mmap this struct directly
/// into a `MAP_SHARED` page shared across processes. All fields are private; the
/// public methods are the entire contract.
#[derive(Debug)]
#[repr(C)]
pub struct BarrierShared {
    /// Current generation `G` (the SENSE). Bumped (`Release`) when the count
    /// reaches 0; that bump IS the release signal — waiters poll for
    /// `generation > my_gen`.
    generation: AtomicU64,
    /// Per-generation count-DOWN counter. Re-armed to `expected` by the unique
    /// opener BEFORE it publishes the next generation; each
    /// [`arrive`](Self::arrive) does one `fetch_sub(1)`, and the caller whose
    /// `fetch_sub` returns `prev == 1` is the unique last arriver. A single
    /// counter suffices only under the one-generation max-skew invariant (see
    /// [module docs](self)).
    remaining: AtomicU32,
    /// Number of participants expected at each generation. Settable
    /// ([`set_expected`](Self::set_expected)); [`drop_participant`](Self::drop_participant)
    /// decrements it for dead-peer handling (the WIRING to liveliness is a later
    /// chunk — here we expose only the mechanism + re-check).
    expected: AtomicU32,
    /// The step-start park's WAKE WORD — a pure barrier-activity
    /// EPOCH, bumped (`Release`) on every barrier state transition (arrive /
    /// open / drop / `set_expected`) and kernel-woken so a context parked in
    /// `monitor_wait_block`'s step-start idle observes a peer's arrival in µs
    /// instead of at the ~100µs sleep-recheck cadence. NEVER read as data, never
    /// gates arrive/open logic — compared only by a parker's kernel block
    /// (`park_wait_activity`), with the caller's `peers_waiting` re-derive
    /// staying the sole correctness (Principle #7 record-only firewall). u32
    /// wrap-around: a false-equal needs EXACTLY 2^32 bumps inside one
    /// snapshot→block window (ns–µs; ≈3.3 days of bumps at the humanoid shape's
    /// ~15k arrivals/s), and the block is slice-bounded regardless — the same
    /// argument as the generation low-32 futex alias.
    wake_seq: AtomicU32,
    /// Rank BITMASK of contexts currently inside the step-start
    /// park (bit `r` = the rank-`r` context holds a `ParkedRankGuard`). Gates
    /// the ARRIVER's wake SYSCALL only (the epoch bump is unconditional), so
    /// `arrive` stays atomics-only when nobody parks. A bitmask (not a count) so
    /// a dead peer's stale bit is EXACTLY sweepable by the supervisor
    /// ([`clear_parked_rank`](Self::clear_parked_rank)) without perturbing live
    /// parkers. Ranks ≥ 32 exceed the mask: those contexts park WITHOUT a bit
    /// (falling back to the slice-timeout cadence — today's behavior) with one
    /// loud warn; deployments beyond 32 barrier ranks lose only the wake-word
    /// latency win, never correctness.
    parked: AtomicU32,
}

// SHM mapping across processes requires a stable, atomics-only `repr(C)` layout.
// Lock the size so a future non-atomic field (padding / bool / pointer) fails the
// build rather than silently breaking the cross-process mapping.
//
// LAYOUT BUMP (16 → 24 bytes, accepted by design): adding the
// `wake_seq` + `parked` words changes the mapped struct, which is safe here
// because there is NO compatibility window in which an old-layout process maps
// a new-layout segment: (a) a deployment's supervisor + workers are always the
// SAME binary (the multi-process split self-spawns `graph run-worker` from one
// executable); (b) `MappedBarrier` names are per-run scoped
// (`cerdep_<graph>_<nonce>` namespaces in production, pid+tag in tests), so a
// segment never outlives its run or meets another binary; (c) bags/recordings
// carry no barrier structs, so replay never deserializes one. `BARRIER_BYTES`
// (64), the mmap length, and the SHM object size are all UNCHANGED — bytes
// 16..24 were zero padding never previously read.
const _: () = assert!(
    core::mem::size_of::<BarrierShared>() == 24,
    "BarrierShared layout changed — SHM cross-process mapping requires the 24-byte atomics-only repr(C)"
);
// Pin the wake-word + parked-mask offsets so the park's kernel wait/wake address
// recipe is compile-locked (the same discipline as the low-32 futex alias below).
const _: () = assert!(
    core::mem::offset_of!(BarrierShared, wake_seq) == 16,
    "wake_seq must sit at offset 16 — the park wake word's kernel wait/wake address recipe"
);
const _: () = assert!(
    core::mem::offset_of!(BarrierShared, parked) == 20,
    "parked must sit at offset 20 — the park rank bitmask"
);
const _: () = assert!(
    core::mem::align_of::<BarrierShared>() >= 8,
    "BarrierShared must be >=8-byte aligned so its `generation` field (offset 0) is a valid 8-byte-aligned monitor-wait target"
);
// The Linux futex tier aliases the LOW 32 bits of the `generation`
// AtomicU64 as the futex word (futexes are 32-bit). On a little-endian target
// the low half sits at the field's base address, so `&generation as *const u32`
// IS the low word — that aliasing is LE-only BY CONSTRUCTION. Both supported
// Linux targets (x86_64, aarch64) are little-endian; a big-endian port would
// need an offset-4 futex address and must fail the build here, not silently
// wait on the HIGH half (which never changes until generation 2^32).
const _: () = assert!(
    cfg!(target_endian = "little"),
    "the barrier futex tier aliases the LOW 32 bits of `generation` at the field's base address — little-endian targets only"
);

impl BarrierShared {
    /// Create a barrier expecting `expected` participants at each generation,
    /// starting at generation 0 with the count-down armed (`remaining ==
    /// expected`).
    ///
    /// `expected == 0` is permitted (a vacuously-complete, zero-participant
    /// barrier — see [`arrive`](Self::arrive)); the count can also be retargeted
    /// later via [`set_expected`](Self::set_expected) (e.g. a supervisor that
    /// pins the count at the gen-0 join boundary once all peers have joined).
    /// `const` so a [`BarrierShared`] can be placed in a `static` /
    /// const-initialized SHM region.
    pub const fn new(expected: u32) -> Self {
        Self {
            generation: AtomicU64::new(0),
            remaining: AtomicU32::new(expected),
            expected: AtomicU32::new(expected),
            wake_seq: AtomicU32::new(0),
            parked: AtomicU32::new(0),
        }
    }

    /// Re-initialize a freshly-mapped segment to generation 0 with the count-down
    /// armed. Used by [`MappedBarrier::create_owned`] after an `O_EXCL` create:
    /// POSIX zero-initializes a new SHM object, so the fresh segment already has
    /// `generation == remaining == expected == 0`; this stores the real
    /// participant count into `remaining`/`expected`. The `generation` store is
    /// therefore redundant for a fresh `O_EXCL` object (it is already 0) — it is
    /// kept as a defensive, intent-explicit reset so the function is also a
    /// correct full re-init if ever called on a non-fresh segment.
    ///
    /// `wake_seq`/`parked` get the same defensive reset. A
    /// stale nonzero `wake_seq` would be harmless (a pure epoch — parkers are
    /// snapshot-relative), but a stale `parked` mask from a crashed prior run
    /// would tax every arriver with a no-op wake syscall forever — the reset
    /// kills both. An orphaned prior run's parkers sit on a DIFFERENT physical
    /// page (unlink-first + fresh `O_EXCL` create), unreachable by this segment's
    /// wake word by construction — so, exactly like `generation`, no wake is
    /// issued here.
    #[cfg_attr(not(target_os = "linux"), allow(dead_code))]
    fn reinit(&self, expected: u32) {
        self.generation.store(0, Ordering::Release);
        self.remaining.store(expected, Ordering::Release);
        self.expected.store(expected, Ordering::Release);
        self.wake_seq.store(0, Ordering::Release);
        self.parked.store(0, Ordering::Release);
    }

    /// Record this participant's arrival at generation `my_gen`. Returns
    /// [`ArriveOutcome::Opened`] to the single caller whose arrival counted
    /// `remaining` down to 0 (the last arriver), and [`ArriveOutcome::Pending`]
    /// to every other arriver.
    ///
    /// The caller MUST pass the generation it currently sits at; in lockstep DAG
    /// execution that is the level index it is leaving. The single-counter design
    /// is correct only under the one-generation max-skew invariant (see
    /// [module docs](self)): never call `arrive` for a generation more than one
    /// ahead of any still-incomplete generation, or you would `fetch_sub` the
    /// wrong generation's `remaining`.
    ///
    /// # Ordering
    ///
    /// `fetch_sub` (`AcqRel`) atomically claims this arrival; the unique caller
    /// that drives `remaining` to 0 re-arms it for the next generation
    /// (`Release`) and then bumps `generation` (`Release`). The bump is the
    /// release point a waiter's `Acquire` load in [`is_open`](Self::is_open)
    /// synchronizes with, and because the re-arm is sequenced-before the
    /// generation store, any waiter that observes the new generation also
    /// observes the re-armed `remaining` it is about to count in.
    ///
    /// # `expected == 0` (degenerate)
    ///
    /// A zero-participant barrier is vacuously complete: `arrive` opens generation
    /// `my_gen + 1` immediately WITHOUT a `fetch_sub` (which would underflow
    /// `remaining`). Because there is no `fetch_sub` to single out a winner here,
    /// the open is published by a CAS on `generation`, so the
    /// "exactly one caller observes [`Opened`](ArriveOutcome::Opened)" guarantee
    /// holds on this path TOO — the CAS winner returns `Opened`, concurrent
    /// vacuous arrivals lose the CAS and return [`Pending`](ArriveOutcome::Pending).
    /// This keeps [`is_open`](Self::is_open) /
    /// [`current_generation`](Self::current_generation) consistent with the
    /// `Opened` return.
    pub fn arrive(&self, my_gen: u64) -> ArriveOutcome {
        let cur = self.generation.load(Ordering::Acquire);
        // Only the CURRENT generation may count down `remaining`. Any non-current
        // arrival is an idempotent no-op — we MUST early-return so it never
        // `fetch_sub`s a non-current generation's counter (which would corrupt the
        // live cohort's count in RELEASE, where the debug asserts below compile out).
        if cur != my_gen {
            if cur < my_gen {
                // `my_gen` AHEAD of the current generation — a >1-gen-skew lockstep
                // breach (must not happen). Loud in RELEASE too, since the
                // `debug_assert!` is gone there and the early return is the only
                // thing preventing a non-current `fetch_sub`.
                tracing::error!(
                    my_gen,
                    cur,
                    "barrier arrive: my_gen AHEAD of current generation — one-gen max-skew invariant breached"
                );
                debug_assert!(false, "barrier arrive: my_gen {my_gen} ahead of cur {cur}");
            } else {
                // `cur > my_gen`: arrival for an already-opened generation.
                // `cur == my_gen + 1` is a benign late arrival (idempotent no-op);
                // `cur > my_gen + 1` is a multi-generation skew that must not happen.
                debug_assert!(
                    cur <= my_gen + 1,
                    "barrier arrive: stale by >1 generation (cur={cur}, my_gen={my_gen}) — multi-gen skew"
                );
            }
            return ArriveOutcome::Pending;
        }

        let exp = self.expected.load(Ordering::Acquire);
        // Degenerate (zero-participant) barrier: open immediately, but pick a UNIQUE
        // winner via CAS so concurrent vacuous arrivals don't ALL return `Opened`
        // (there is no `fetch_sub` to single one out on this path). `remaining` is
        // already 0 (exp==0), so no re-arm is needed — the CAS publishes the
        // generation directly; a later `set_expected` arms `remaining` when real
        // participants join. The loser sees the generation already bumped and waits,
        // exactly like a normal non-opener.
        if exp == 0 {
            return match self.generation.compare_exchange(
                my_gen,
                my_gen + 1,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => {
                    // This CAS is a generation-bump site — kernel-wake
                    // any futex waiters exactly like `open_generation` does.
                    self.wake_waiters();
                    ArriveOutcome::Opened(my_gen + 1)
                }
                Err(_) => ArriveOutcome::Pending,
            };
        }

        let prev = self.remaining.fetch_sub(1, Ordering::AcqRel);
        // Publish this arrival to any step-start parker — the
        // epoch bump is UNCONDITIONAL (atomics-only, on a cache line this
        // arrive's RMW already owns), the wake SYSCALL is parked-gated. Placed
        // immediately after the `fetch_sub` (the state transition parkers
        // observe via `peers_waiting`) and BEFORE the prev-branch, so every
        // outcome — Pending arrival, opener (which bumps again in
        // `open_generation`, harmless for an epoch), even the error-path
        // restore — has already signaled. See `note_activity` for the
        // store-buffer-litmus ordering argument.
        self.note_activity();
        if prev == 1 {
            // I brought `remaining` to exactly 0 — the unique last arriver.
            self.open_generation(my_gen);
            ArriveOutcome::Opened(my_gen + 1)
        } else if prev == 0 {
            // Best-effort restore to bound the wrap — NOT a clean recovery: under a
            // real desync the state is already corrupt and this `fetch_add` can race
            // the opener's re-arm. The actual protection is the lockstep invariant +
            // the stale-generation guard at the top of `arrive`. MUST NOT happen
            // under the one-generation-skew invariant (an extra / late / double
            // arrival), so shout — silent recovery would mask a real desync.
            self.remaining.fetch_add(1, Ordering::AcqRel);
            tracing::error!(
                my_gen,
                expected = self.expected.load(Ordering::Acquire),
                "barrier arrive underflow: remaining already 0 — over/double/late arrival or >1-gen skew (lockstep invariant violated)"
            );
            debug_assert!(
                false,
                "barrier arrive underflow at gen {my_gen} — one-gen max-skew invariant breached"
            );
            ArriveOutcome::Pending
        } else {
            ArriveOutcome::Pending
        }
    }

    /// Open generation `my_gen`: re-arm `remaining` from `expected` for the next
    /// generation, then bump `generation` to `my_gen + 1`.
    ///
    /// The re-arm MUST precede the generation bump: a waiter that observes the
    /// new generation (via [`is_open`](Self::is_open)'s `Acquire` load) and then
    /// counts into `remaining` for generation `my_gen + 1` must see it already
    /// re-armed. This is safe for the UNIQUE opener because no other participant
    /// counts in `remaining` for generation `my_gen + 1` until it observes the
    /// generation bump, which is sequenced AFTER this re-arm. (Reading the
    /// possibly-just-decremented `expected` here is intentional: a
    /// [`drop_participant`](Self::drop_participant) lowers `expected` so the NEXT
    /// generation is sized for the surviving participants.)
    fn open_generation(&self, my_gen: u64) {
        self.remaining
            .store(self.expected.load(Ordering::Acquire), Ordering::Release);
        self.generation.store(my_gen + 1, Ordering::Release);
        // Kernel-wake every futex waiter parked on the generation word
        // (Linux; no-op elsewhere). AFTER the store, so a woken waiter's re-poll
        // observes the new generation.
        self.wake_waiters();
    }

    /// Kernel-WAKE every waiter parked on the `generation`
    /// word — the Linux futex tier AND the macOS os_sync tier of
    /// [`wait`](Self::wait). Called after EVERY site that bumps
    /// `generation`: [`open_generation`](Self::open_generation) (the normal
    /// arrive opener AND `try_drop_participant`'s dead-peer opener route here)
    /// and the vacuous `expected == 0` CAS opener in [`arrive`](Self::arrive).
    /// (`reinit` also stores `generation`, but only on a FRESH `O_EXCL`-created
    /// segment no waiter can be parked on — an orphaned prior run's waiters sit
    /// on a DIFFERENT physical page, unreachable by these address-keyed wakes by
    /// construction — so it deliberately does not wake.)
    ///
    /// **Linux — plain `FUTEX_WAKE`**, deliberately NOT `FUTEX_PRIVATE_FLAG`:
    /// the word lives in `MAP_SHARED` memory crossing processes, and private
    /// futexes key on the virtual address within one mm; shared futexes key on
    /// the physical page, which is what makes the cross-process wake work.
    ///
    /// **macOS ≥ 14.4 — `os_sync_wake_by_address_all`** with
    /// `OS_SYNC_WAKE_BY_ADDRESS_SHARED` on the SAME low-32 alias + size the wait
    /// side blocks on (via `os_sync_word` / `OS_SYNC_WORD_SIZE` — ONE shared
    /// recipe so wake and wait can never diverge; both are macOS-gated, so these
    /// are plain code spans not intra-doc links, keeping the all-OS `wake_waiters`
    /// doc link-clean on Linux). The SHARED flag keys the wake on the shared
    /// page, mirroring the Linux shared futex. Gated on `os_sync_active`: if the
    /// primitive was
    /// never resolved (older macOS), the kill switch is set, or an `EINVAL`/
    /// `ENOTSUP` latched it off, the waiters are on the sleep-recheck fallback
    /// (which polls `is_open`, needing no wake) so this is a correct no-op.
    ///
    /// On BOTH tiers the return value (waiters woken) and any error are
    /// deliberately ignored: the waiter's re-poll of `is_open` is the
    /// correctness, the wake is only latency.
    ///
    /// **Opener side of the step-start park:** every open ALSO
    /// unconditionally bumps the `wake_seq` epoch and rings the wake word
    /// (`wake_word_wake`, with no parked gate here by design: this
    /// path already pays a kernel wake, and the unconditional ring self-heals
    /// any transient parked/epoch skew once per generation). Step-start parkers
    /// blocked on `wake_seq` re-derive `peers_waiting` on wake — record-only,
    /// exactly like the boundary waiters' `is_open` re-poll.
    #[inline]
    fn wake_waiters(&self) {
        // Opener-side wake-word bump + UNCONDITIONAL ring,
        // before the generation-word wakes below (order between the two words
        // is immaterial — both sides re-derive their own predicate on wake).
        self.wake_seq.fetch_add(1, Ordering::Release);
        self.wake_word_wake();
        #[cfg(target_os = "linux")]
        {
            let futex_word = &self.generation as *const AtomicU64 as *const u32;
            // SAFETY: `futex_word` is the 4-byte-aligned (in fact 8-byte-aligned)
            // low half of the live `generation` word (LE — see the const-assert
            // at the layout block); FUTEX_WAKE reads no user memory beyond
            // keying on the address. All other args are the documented "unused"
            // values for FUTEX_WAKE.
            unsafe {
                libc::syscall(
                    libc::SYS_futex,
                    futex_word,
                    libc::FUTEX_WAKE,
                    i32::MAX,
                    std::ptr::null::<libc::timespec>(),
                    std::ptr::null::<u32>(),
                    0u32,
                );
            }
        }
        #[cfg(target_os = "macos")]
        {
            if os_sync_active() {
                if let Some(backend) = os_sync_backend() {
                    // SAFETY: `os_sync_word()` is the 4-byte-aligned (8-byte, in
                    // fact) low half of the live `generation` word (LE — see the
                    // layout const-assert), mapped for this `&self` call;
                    // `os_sync_wake_by_address_all` reads no user memory beyond
                    // keying on that address + size. `backend.wake` is the
                    // dlsym-resolved, signature-checked `os_sync_wake_by_address_all`
                    // fn pointer. The returned waiter count / errno is ignored (the
                    // waiter's `is_open` re-poll is the correctness).
                    unsafe {
                        (backend.wake)(
                            self.os_sync_word(),
                            OS_SYNC_WORD_SIZE,
                            libc::OS_SYNC_WAKE_BY_ADDRESS_SHARED,
                        );
                    }
                }
            }
        }
    }

    /// The macOS os_sync watch/wake TARGET — the LOW 32 bits of
    /// `generation` (the little-endian alias at the field's base address, size
    /// [`OS_SYNC_WORD_SIZE`] = 4). This is the SINGLE recipe shared by BOTH the
    /// wake side ([`wake_waiters`](Self::wake_waiters)) and the wait side
    /// ([`wait_os_sync`](Self::wait_os_sync)), so the address + size the opener
    /// wakes can never drift from the address + size a waiter blocks on. Same
    /// low-32 LE aliasing as the Linux futex word (the layout const-assert pins
    /// little-endian; macOS aarch64 AND x86_64 are both LE). 8-byte alignment
    /// (from the `AtomicU64`) satisfies the 4-byte-size alignment os_sync
    /// requires.
    #[cfg(target_os = "macos")]
    #[inline]
    fn os_sync_word(&self) -> *mut c_void {
        &self.generation as *const AtomicU64 as *mut c_void
    }

    // ============== the step-start park wake word ==============
    // A kernel wake for `monitor_wait_block`'s barrier-participant idle: parkers
    // block on the `wake_seq` epoch; every barrier state transition bumps it
    // (arrivers gate only the SYSCALL on the `parked` mask, so `arrive` stays
    // atomics-only when nobody parks). Record-only WAIT machinery (Principle #7):
    // changes WHEN the park returns, never what fires — the caller's
    // `peers_waiting` re-derive after every wake stays the sole correctness.

    /// The ARRIVER/DROP/`set_expected` side of the wake word —
    /// unconditional epoch bump, then a `parked`-GATED kernel wake.
    ///
    /// # Store-buffer litmus (why the `SeqCst` fence)
    ///
    /// This pairs with [`park_enter`](Self::park_enter): parker = {set `parked`
    /// bit; load predicate}, arriver = {`remaining` RMW + `wake_seq` bump; load
    /// `parked`} — the classic SB shape, and Acquire/Release alone permits the
    /// both-see-stale outcome (arriver skips the syscall AND the parker saw a
    /// stale predicate → blocked). The `SeqCst` fence here (between the bump and
    /// the `parked` load) and its twin in `park_enter` (between the bit-set and
    /// the caller's predicate loads) forbid it. Even absent the fences, the
    /// UNCONDITIONAL bump keeps the failure bounded: the parker's kernel
    /// compare-value would already differ, so the worst case is one bounded
    /// slice, never a lost wake — but the fences make it model-correct, not
    /// merely hardware-likely. (~one `dmb ish` per arrive on aarch64, free on
    /// x86; noise at the humanoid shape's ~15k arrivals/s.)
    #[inline]
    fn note_activity(&self) {
        self.wake_seq.fetch_add(1, Ordering::Release);
        core::sync::atomic::fence(Ordering::SeqCst);
        if self.parked.load(Ordering::Relaxed) != 0 {
            self.wake_word_wake();
        }
    }

    /// The kernel wake SYSCALL on the wake word — wake ALL
    /// parkers (they each re-derive `peers_waiting`; a spurious wake is a
    /// bounded no-op re-park). Linux: shared `FUTEX_WAKE` on `wake_seq` (the
    /// same non-PRIVATE discipline as [`wake_waiters`](Self::wake_waiters)).
    /// macOS ≥ 14.4: `os_sync_wake_by_address_all` (gated on `os_sync_active` —
    /// when inactive, parkers never kernel-block on the word, so skipping the
    /// syscall is correct, not just cheap). Other targets: no-op (no parker can
    /// block there — `park_wait_activity` reports not-performed). Return values
    /// / errnos deliberately ignored (the wake is only latency).
    #[inline]
    fn wake_word_wake(&self) {
        #[cfg(target_os = "linux")]
        {
            let word = &self.wake_seq as *const AtomicU32 as *const u32;
            // SAFETY: `word` is the live 4-byte `wake_seq` (offset-16
            // const-assert-pinned, 4-byte-aligned by layout), mapped for this
            // `&self` call; FUTEX_WAKE reads no user memory beyond keying on the
            // address. Other args are the documented "unused" FUTEX_WAKE values.
            unsafe {
                libc::syscall(
                    libc::SYS_futex,
                    word,
                    libc::FUTEX_WAKE,
                    i32::MAX,
                    std::ptr::null::<libc::timespec>(),
                    std::ptr::null::<u32>(),
                    0u32,
                );
            }
        }
        #[cfg(target_os = "macos")]
        {
            if os_sync_active() {
                if let Some(backend) = os_sync_backend() {
                    // SAFETY: `wake_word_addr()` is the live 4-byte `wake_seq`
                    // (offset-16 pinned), mapped for this `&self` call;
                    // `os_sync_wake_by_address_all` reads no user memory beyond
                    // keying on address + size. `backend.wake` is the
                    // dlsym-resolved, signature-checked fn pointer.
                    unsafe {
                        (backend.wake)(
                            self.wake_word_addr(),
                            OS_SYNC_WORD_SIZE,
                            libc::OS_SYNC_WAKE_BY_ADDRESS_SHARED,
                        );
                    }
                }
            }
        }
    }

    /// The wake word's kernel wait/wake ADDRESS — the whole
    /// 4-byte `wake_seq` (no low-32 aliasing needed: it IS a u32). ONE recipe
    /// shared by the wake side ([`wake_word_wake`](Self::wake_word_wake)) and
    /// the wait side ([`park_wait_activity`](Self::park_wait_activity)); size is
    /// `OS_SYNC_WORD_SIZE` (4) on the os_sync tier. Offset-16 from the 8-aligned
    /// struct base → 4-byte alignment holds by construction (const-assert-pinned).
    #[cfg(target_os = "macos")]
    #[inline]
    fn wake_word_addr(&self) -> *mut c_void {
        &self.wake_seq as *const AtomicU32 as *mut c_void
    }

    /// Snapshot the wake-word epoch (`Acquire`). The parker
    /// takes this BEFORE its final `peers_waiting` re-derive, then hands it to
    /// [`park_wait_activity`](Self::park_wait_activity) as the kernel compare
    /// value — any bump landing after the snapshot fails the compare (no lost
    /// wake); any state change before it is caught by the re-derive.
    ///
    /// Runtime-internal park seam (`pub` for the live loop + its integration
    /// pins; stability-exempt pre-1.0).
    pub fn wake_seq_snapshot(&self) -> u32 {
        self.wake_seq.load(Ordering::Acquire)
    }

    /// Mark rank `rank` as step-start-PARKED (set its `parked`
    /// bit). Returns `false` — no bit set — for `rank >= 32` (beyond the mask:
    /// that context keeps today's slice-timeout cadence; ONE loud process-wide
    /// warn names the degradation). The `SeqCst` fence after the bit-set is the
    /// parker's half of the SB-litmus pair — see `note_activity` (private; plain
    /// code span keeps this pub doc link-clean).
    /// Callers use [`ParkedRankGuard`] so the bit is cleared on EVERY exit path.
    ///
    /// Runtime-internal park seam (`pub` for the live loop; stability-exempt).
    pub fn park_enter(&self, rank: u32) -> bool {
        if rank >= 32 {
            static WARNED: std::sync::atomic::AtomicBool =
                std::sync::atomic::AtomicBool::new(false);
            if !WARNED.swap(true, Ordering::Relaxed) {
                tracing::warn!(
                    rank,
                    "barrier park wake: rank >= 32 exceeds the parked bitmask — this context's \
                     step-start park falls back to slice-timeout cadence (bounded, correct; only \
                     the wake-word latency win is lost). Deployments beyond 32 barrier ranks are \
                     outside the wake-word fast path"
                );
            }
            return false;
        }
        self.parked.fetch_or(1u32 << rank, Ordering::AcqRel);
        // Test-only: count every REAL bit-set so the
        // hardware-park test arm can assert the bit was NEVER set during a run
        // (the no-futile-wake probe). Process-global (all barriers), cfg-gated
        // to zero production cost; record-only.
        #[cfg(any(test, feature = "test-helpers"))]
        PARK_ENTER_CALLS.fetch_add(1, Ordering::Relaxed);
        // SB-litmus fence: order the bit-set BEFORE the caller's subsequent
        // predicate loads (`wake_seq_snapshot` + `peers_waiting`), pairing with
        // the fence in `note_activity`. See that method's litmus writeup.
        core::sync::atomic::fence(Ordering::SeqCst);
        true
    }

    /// Clear rank `rank`'s `parked` bit (park exit). Only ever
    /// called by [`ParkedRankGuard`]'s `Drop` (rank < 32 guaranteed — `enter`
    /// returned `None` otherwise). Runtime-internal park seam.
    pub fn park_exit(&self, rank: u32) {
        debug_assert!(rank < 32, "park_exit: rank {rank} was never bit-mapped");
        self.parked.fetch_and(!(1u32 << rank), Ordering::AcqRel);
    }

    /// SUPERVISOR sweep — clear a DEAD peer's stale `parked`
    /// bit. A worker SIGKILLed inside its park never runs its
    /// [`ParkedRankGuard`] `Drop`, leaving its bit set forever: correctness is
    /// untouched (the bit only gates the arrivers' wake syscall) but every
    /// arrive would pay a no-op kernel wake. The multi-process supervisor calls
    /// this alongside [`drop_dead_peer`](Self::drop_dead_peer) (it knows the
    /// dead worker's rank from its spawn plan), making the leak's window the
    /// death-to-sweep interval instead of the rest of the run. `rank >= 32` is a
    /// no-op (such ranks never set a bit). Idempotent; safe against the dead
    /// peer having NOT been parked (clearing an unset bit is a no-op).
    pub fn clear_parked_rank(&self, rank: u32) {
        if rank >= 32 {
            return;
        }
        self.parked.fetch_and(!(1u32 << rank), Ordering::AcqRel);
    }

    /// Test/diagnostic: the current `parked` bitmask
    /// (`Acquire`). Read by the hermetic bitmask oracles; never a control input.
    pub fn parked_mask(&self) -> u32 {
        self.parked.load(Ordering::Acquire)
    }

    /// The step-start park's bounded KERNEL BLOCK on the wake
    /// word — block until `wake_seq` moves off `snapshot` (a barrier state
    /// transition), `cap` elapses, or a spurious wake. Returns `true` if a real
    /// bounded block (or its internal bounded degrade-sleep) was performed — the
    /// caller skips its own pacing sleep; `false` if NOT performed (no primitive
    /// on this target, the os_sync tier inactive/latched, or a zero `cap`) — the
    /// caller keeps today's sleep-recheck pacing, byte-identical behavior.
    ///
    /// SPURIOUS-TOLERANT BY CONTRACT: the return value is only a pacing hint.
    /// The caller re-derives `peers_waiting` (and its other wake sources) after
    /// EVERY return — a wake with no state change is a bounded no-op re-park,
    /// never a correctness event (Principle #7 record-only).
    ///
    /// Tiers (mirrors [`wait`](Self::wait)'s shape):
    /// - **Linux**: shared `FUTEX_WAIT` on `wake_seq` with compare `snapshot`,
    ///   `cap`-bounded. Benign errnos (`EAGAIN` = word already moved — the
    ///   lost-wakeup guard; `EINTR`; `ETIMEDOUT`) return `true` silently; any
    ///   other errno warns once per distinct errno + sleeps a bounded 100µs
    ///   (capped at `cap`) so a persistently-failing syscall can never
    ///   busy-loop the caller.
    /// - **macOS ≥ 14.4**: `os_sync_wait_on_address_with_timeout` on `wake_seq`
    ///   (size 4, SHARED, `OS_CLOCK_MACH_ABSOLUTE_TIME`, relative `cap` ns). A
    ///   value mismatch returns a NON-NEGATIVE rc immediately (verified on macOS).
    ///   rc < 0: `EINVAL`/`ENOTSUP` latch `OS_SYNC_DISABLED`
    ///   process-wide (both this park AND the boundary `wait` degrade — one
    ///   latch, one primitive family) + warn once → `false`; benign
    ///   (`ETIMEDOUT`/`EINTR`) → `true`; other → warn-once-per-errno + bounded
    ///   sleep → `true`.
    /// - **other targets**: `false` (no primitive — caller sleeps as today).
    ///
    /// Runtime-internal park seam (`pub` for the live loop; stability-exempt).
    pub fn park_wait_activity(&self, snapshot: u32, cap: Duration) -> bool {
        if cap.is_zero() {
            // Nothing to block for (the caller's remaining window is exhausted);
            // a zero-timeout kernel call risks EINVAL for no benefit.
            return false;
        }
        #[cfg(target_os = "linux")]
        {
            let word = &self.wake_seq as *const AtomicU32 as *const u32;
            let ts = libc::timespec {
                tv_sec: cap.as_secs() as libc::time_t,
                tv_nsec: libc::c_long::from(cap.subsec_nanos()),
            };
            // SAFETY: `word` is the live 4-byte `wake_seq`, mapped for this
            // `&self` call; `ts` outlives the syscall. Shared (non-PRIVATE)
            // futex — the word crosses processes. The kernel's atomic
            // compare-and-block against `snapshot` IS the lost-wakeup guard.
            let rc = unsafe {
                libc::syscall(
                    libc::SYS_futex,
                    word,
                    libc::FUTEX_WAIT,
                    snapshot,
                    &ts as *const libc::timespec,
                    std::ptr::null::<u32>(),
                    0u32,
                )
            };
            if rc < 0 {
                let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
                if !futex_wait_errno_is_benign(errno) {
                    static LAST_WARNED_ERRNO: std::sync::atomic::AtomicI32 =
                        std::sync::atomic::AtomicI32::new(0);
                    if LAST_WARNED_ERRNO.swap(errno, Ordering::Relaxed) != errno {
                        tracing::warn!(
                            errno,
                            "park wake word FUTEX_WAIT failed with an unexpected errno; the \
                             step-start park degrades to bounded sleep pacing for this errno"
                        );
                    }
                    std::thread::sleep(Duration::from_micros(100).min(cap));
                }
            }
            true
        }
        #[cfg(target_os = "macos")]
        {
            if !os_sync_active() {
                return false;
            }
            let Some(backend) = os_sync_backend() else {
                return false;
            };
            // Relative-ns timeout; `cap` is a bounded park slice (≤ the caller's
            // recheck chunk), far inside u64.
            let timeout_ns = cap.as_nanos() as u64;
            // SAFETY: `wake_word_addr()` is the live 4-byte `wake_seq`, mapped
            // for this `&self` call; `backend.wait` is the dlsym-resolved,
            // signature-checked `os_sync_wait_on_address_with_timeout`. SHARED
            // keys on the physical page (cross-process); a value mismatch
            // returns rc >= 0 immediately, reading no memory beyond addr+size.
            let rc = unsafe {
                (backend.wait)(
                    self.wake_word_addr(),
                    u64::from(snapshot),
                    OS_SYNC_WORD_SIZE,
                    libc::OS_SYNC_WAIT_ON_ADDRESS_SHARED,
                    libc::OS_CLOCK_MACH_ABSOLUTE_TIME,
                    timeout_ns,
                )
            };
            if rc < 0 {
                let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
                if os_sync_errno_is_unrecoverable(errno) {
                    // One latch for the whole os_sync family (this park, the
                    // boundary wait, AND the monitor-wait nap) — an
                    // EINVAL/ENOTSUP means the primitive is unusable, not one
                    // call shape.
                    if crate::os_sync::latch_os_sync_disabled() {
                        tracing::warn!(
                            errno,
                            "park wake word os_sync_wait_on_address returned an unrecoverable \
                             errno (EINVAL/ENOTSUP); disabling the os_sync tier process-wide — \
                             the step-start park falls back to sleep-recheck pacing"
                        );
                    }
                    return false;
                }
                if !os_sync_errno_is_benign(errno) {
                    static LAST_WARNED_ERRNO: std::sync::atomic::AtomicI32 =
                        std::sync::atomic::AtomicI32::new(0);
                    if LAST_WARNED_ERRNO.swap(errno, Ordering::Relaxed) != errno {
                        tracing::warn!(
                            errno,
                            "park wake word os_sync_wait_on_address returned an unexpected \
                             errno; the step-start park degrades to bounded sleep pacing for \
                             this errno"
                        );
                    }
                    std::thread::sleep(Duration::from_micros(100).min(cap));
                }
            }
            true
        }
        #[cfg(not(any(target_os = "linux", target_os = "macos")))]
        {
            false
        }
    }

    /// `true` once generation `my_gen` has opened — i.e. `generation > my_gen`
    /// (`Acquire` load). This is the poll predicate [`wait`](Self::wait)
    /// re-checks after every wake (futex wake, slice timeout, or the non-Linux
    /// recheck).
    pub fn is_open(&self, my_gen: u64) -> bool {
        self.generation.load(Ordering::Acquire) > my_gen
    }

    /// Block the calling thread/process until generation `my_gen` opens or
    /// `timeout` elapses. Returns [`WaitOutcome::Opened`] when
    /// [`is_open`](Self::is_open) becomes true, or [`WaitOutcome::TimedOut`] on
    /// deadline (see [`WaitOutcome::TimedOut`] — it is NOT a proceed signal).
    ///
    /// **Wake mechanism (Linux):** spin-then-block on a PROCESS-SHARED
    /// futex over the low 32 bits of the barrier's own `generation` word (the
    /// SENSE). The unique opener's `generation.store(my_gen + 1)` (in
    /// `open_generation` / the vacuous CAS) is IMMEDIATELY followed by a
    /// `FUTEX_WAKE` on that word, so a parked waiter gets a real KERNEL wake
    /// within microseconds of the store — the barrier needs no separate
    /// doorbell (the data path needs one only because its payload lives in an
    /// unwatchable iceoryx2 queue; the barrier's SENSE is itself the futex
    /// word). A BOUNDED pre-block spin catches the fast-rendezvous case
    /// (opener already arriving) without a syscall — DEFAULT tier-resolved
    /// time-bounded (20µs on every kernel-wake tier —
    /// Linux futex AND macOS os_sync — 150µs only on the macOS sleep-recheck
    /// fallback; `CERULION_BARRIER_SPIN_US` overrides,
    /// `=0` kills it back to a 50-iteration spin — see `barrier_spin_budget`);
    /// it is spin-then-block, never a busy-spin. (The earlier CPU monitor-wait
    /// park was measured NOT
    /// to observe the cross-process store — aarch64 WFE is event-stream-bound
    /// at ~100µs and x86 measured similarly — so every boundary crossing paid a
    /// ~100µs-class timer backstop; the futex wake replaces it. The
    /// [`crate::monitor_wait`] primitive itself is unchanged and stays in use
    /// on the data-path park.)
    ///
    /// **No lost wake (Principle #6):** `FUTEX_WAIT` atomically re-checks the
    /// futex word against the pre-block snapshot INSIDE the kernel — a store
    /// landing between the snapshot and the block makes the call return
    /// `EAGAIN` immediately — and correctness still rests on re-polling the
    /// REAL [`is_open`](Self::is_open) after EVERY wake (spurious wake, EINTR,
    /// slice timeout), never on the wake itself. Each futex block is
    /// additionally bounded by a generous slice, so even a lost wake only
    /// costs one slice, not the whole deadline.
    ///
    /// **Wrap-around note:** the futex compares only the LOW 32 bits of the
    /// generation. A false-equal (blocking although the generation moved)
    /// would need EXACTLY 2^32 generations to complete between the snapshot
    /// and the block — impossible in practice, and the bounded slice re-polls
    /// the full 64-bit word regardless.
    ///
    /// **Wake mechanism (macOS ≥ 14.4):** the SAME wake-assisted
    /// spin-then-block shape as Linux, over Apple's `os_sync_wait_on_address` /
    /// `os_sync_wake_by_address_all` on the low-32 `generation` alias
    /// (`OS_SYNC_WAIT_ON_ADDRESS_SHARED`, the cross-process page-keyed flag). The
    /// opener's `os_sync_wake_by_address_all` (in `wake_waiters`) delivers a real
    /// kernel wake within µs of the store, so the ~100µs sleep-recheck skew a
    /// poll-based macOS tier pays at every boundary is avoided. The primitive is bound
    /// at runtime via `dlsym` (`os_sync_backend`) — a direct extern to a
    /// 14.4-only symbol would make dyld abort at LAUNCH on an older macOS — and
    /// an ABSENT symbol, the `CERULION_BARRIER_OS_SYNC=0` kill switch, or a
    /// latched `EINVAL`/`ENOTSUP` all route to the recheck fallback instead. The
    /// no-lost-wake / wrap-around / spin-bound contracts below hold identically:
    /// a mismatched value makes `os_sync_wait_on_address` return IMMEDIATELY with
    /// a non-negative rc (the lost-wakeup guard, verified on macOS), and the
    /// `is_open` re-poll after every wake stays the sole correctness. See
    /// `wait_os_sync` (macOS-gated — a plain code span, not an intra-doc link,
    /// since this all-OS `wait` doc is built on Linux too).
    ///
    /// **No busy-spin:** the pre-block spin is hard-bounded (the tier-resolved
    /// default budget — 20µs on the kernel-wake tiers, 150µs on the macOS
    /// sleep-recheck fallback — or a few hundred ns under
    /// the `=0` kill switch); the block is a real kernel sleep (futex on Linux,
    /// os_sync on macOS ≥ 14.4). On an OLDER macOS (or any other non-Linux
    /// target — future Windows) with no kernel-wake primitive available the
    /// ladder is: the SAME time-bounded boundary spin (phase 1, absorbing the
    /// fast lockstep rendezvous), then chunked ~100µs sleep-rechecks — also
    /// never a busy-spin. See `wait_recheck_fallback` (cfg-gated off Linux).
    ///
    /// **Unlink-while-parked safety:** a `wait` parked through a
    /// [`MappedBarrier`] stays VALID even if the OWNER tears the barrier down
    /// mid-park (owner `Drop` = its own `munmap` + `shm_unlink`) or a peer process
    /// crashes: POSIX `shm_unlink` removes only the NAME, and the object persists
    /// until the LAST `munmap` — so this process's mapping, and the futex wait
    /// keyed on its `generation` word (shared futexes key on the PHYSICAL page),
    /// remain valid until this process's OWN unmap. The parked waiter simply runs
    /// to [`WaitOutcome::Opened`] (a surviving peer handle can still open the
    /// generation — the wake path outlives the name) or [`WaitOutcome::TimedOut`];
    /// it can never die by SIGSEGV/SIGBUS. A SIGBUS would require an `ftruncate`
    /// SHRINK of the live object under the mapping, which nothing in this codebase
    /// performs — that is the contract. Box-pinned by
    /// `box_parked_waiter_survives_peer_crash_and_owner_unlink`
    /// and `box_parked_waiter_still_wakes_after_owner_unlink` in `barrier_test.rs`.
    pub fn wait(&self, my_gen: u64, timeout: Duration) -> WaitOutcome {
        let deadline = Instant::now() + timeout;
        #[cfg(target_os = "linux")]
        {
            self.wait_futex(my_gen, deadline)
        }
        #[cfg(target_os = "macos")]
        {
            // Prefer the os_sync kernel-wake tier when the 14.4+
            // primitive resolved AND is neither kill-switched nor latched off by
            // a prior EINVAL/ENOTSUP; otherwise the chunked sleep-recheck
            // fallback (older macOS). The routing is read ONCE here per wait.
            if os_sync_active() {
                self.wait_os_sync(my_gen, deadline)
            } else {
                self.wait_recheck_fallback(my_gen, deadline)
            }
        }
        #[cfg(not(any(target_os = "linux", target_os = "macos")))]
        {
            self.wait_recheck_fallback(my_gen, deadline)
        }
    }

    /// The Linux futex tier of [`wait`](Self::wait) — a process-shared
    /// `FUTEX_WAIT` on the low 32 bits of `generation`, kernel-woken by
    /// `wake_waiters` after every generation bump. See `wait`'s doc for the
    /// wake-mechanism / no-lost-wake / wrap-around contracts.
    ///
    /// **Unexpected-errno degrade:** the EXPECTED `FUTEX_WAIT`
    /// returns — 0 (woken), `EAGAIN` (word already changed), `EINTR`,
    /// `ETIMEDOUT` — re-loop silently and hot (the re-poll of `is_open` IS the
    /// correctness). Any OTHER errno (e.g. a seccomp `ENOSYS`, `EFAULT`) means
    /// the kernel park is unavailable: re-looping IMMEDIATELY would let a
    /// persistently-failing syscall degenerate to a silent ~100% busy loop for
    /// the whole deadline. Instead it `tracing::warn!`s ONCE per process (naming
    /// the errno) and sleeps a bounded 100µs recheck before re-looping — the
    /// barrier degrades to bounded sleep-polling (mirroring the non-Linux
    /// tier's never-busy-spin guarantee), never a busy-spin. The classification
    /// is the pure [`futex_wait_errno_is_benign`] (oracle-tested); the
    /// warn+sleep arm itself has no syscall-injection seam (disproportionate
    /// for this unsafe hot path) and is checked by inspection.
    #[cfg(target_os = "linux")]
    fn wait_futex(&self, my_gen: u64, deadline: Instant) -> WaitOutcome {
        // The barrier-boundary spin budget — DEFAULT-ON (per-OS; see
        // `barrier_spin_budget` for the measured knee + the
        // `CERULION_BARRIER_SPIN_US` kill switch/override + the 100ms clamp).
        // One cached OnceLock read here, resolved OUTSIDE the (delegated) loop
        // and handed to `wait_futex_with_budget` so the `=0` kill-switch arm is
        // exercisable hermetically by test (the budget is a parameter there).
        self.wait_futex_with_budget(my_gen, deadline, barrier_spin_budget())
    }

    /// The budget-parameterized core of
    /// [`wait_futex`](Self::wait_futex). [`wait_futex`](Self::wait_futex) calls
    /// this with the cached [`barrier_spin_budget`]; a `spin_budget` of ZERO
    /// selects the earlier 50-iteration pre-block spin (the `=0` kill
    /// switch), a non-zero budget the TIME-bounded spin — see the phase-1 branch
    /// below. Split out as a PURE refactor (zero behavior change: the only
    /// caller passes the exact same cached budget the old body read inline) so
    /// the `=0` legacy arm is testable hermetically without env/`OnceLock`
    /// games. See `wait`'s doc for the wake-mechanism / no-lost-wake /
    /// wrap-around contracts.
    #[cfg(target_os = "linux")]
    fn wait_futex_with_budget(
        &self,
        my_gen: u64,
        deadline: Instant,
        spin_budget: Duration,
    ) -> WaitOutcome {
        /// Bounded pre-block spin iterations (~a few hundred ns of Acquire
        /// loads): catches the fast-rendezvous case — the opener is arriving
        /// RIGHT NOW — without paying a syscall. Spin-then-block, never a
        /// busy-spin: after this bound the waiter goes to a real kernel sleep.
        const PRE_BLOCK_SPIN: u32 = 50;
        /// Max single futex slice. Generous — a real wake arrives in µs; the
        /// slice only bounds the cost of a hypothetically lost wake (re-poll +
        /// re-block) and keeps each kernel wait finite.
        const FUTEX_SLICE: Duration = Duration::from_millis(10);

        // The futex word: the LOW 32 bits of the generation AtomicU64 (LE-only
        // by construction — see the const-assert at the layout block). Valid
        // for the `&self` borrow, which for a `MappedBarrier` Deref keeps the
        // mapping alive for the whole call.
        let futex_word = &self.generation as *const AtomicU64 as *const u32;
        loop {
            if spin_budget.is_zero() {
                // Spin-then-block phase 1 (`=0` kill switch): a HARD-BOUNDED
                // read loop — no `Instant` reads on this path (the earlier
                // behavior, byte-identical).
                for _ in 0..PRE_BLOCK_SPIN {
                    if self.generation.load(Ordering::Acquire) > my_gen {
                        return WaitOutcome::Opened;
                    }
                    core::hint::spin_loop();
                }
            } else {
                // Spin-then-block phase 1 (the DEFAULT): a TIME-bounded read loop instead, to
                // spin through the lockstep rendezvous rather than paying the
                // futex sleep/wake at every boundary. Capped at the caller's
                // `deadline` so the spin can never outlive the wait; the
                // budget applies PER block attempt (each futex-slice re-loop
                // re-spins), which stays bounded — worst case one budget per
                // FUTEX_SLICE (10ms) of waiting.
                let spin_deadline = (Instant::now() + spin_budget).min(deadline);
                if self.spin_for_budget(my_gen, spin_deadline) {
                    return WaitOutcome::Opened;
                }
            }
            // ONE Acquire load reused as BOTH the is_open check AND the futex
            // expected value, so the snapshot->block window is minimal; a store
            // landing inside that window makes FUTEX_WAIT return EAGAIN at once
            // (the kernel's atomic compare-and-block IS the lost-wakeup guard).
            let gen_now = self.generation.load(Ordering::Acquire);
            if gen_now > my_gen {
                return WaitOutcome::Opened;
            }
            let now = Instant::now();
            if now >= deadline {
                return WaitOutcome::TimedOut;
            }
            let slice = FUTEX_SLICE.min(deadline - now);
            let ts = libc::timespec {
                tv_sec: slice.as_secs() as libc::time_t,
                tv_nsec: libc::c_long::from(slice.subsec_nanos()),
            };
            let expected_low = gen_now as u32;
            // Plain FUTEX_WAIT — deliberately NOT FUTEX_PRIVATE_FLAG: the word
            // lives in MAP_SHARED memory crossing processes (shared futexes key
            // on the physical page; private ones on one mm's virtual address).
            // The timeout is RELATIVE for FUTEX_WAIT.
            // SAFETY: `futex_word` is the 4-byte-aligned (8-byte-aligned, in
            // fact) low half of the live `generation` word, mapped for this
            // `&self` call; `ts` outlives the syscall. Every EXPECTED return —
            // 0 (woken), EAGAIN (word already changed), EINTR, ETIMEDOUT,
            // spurious — is handled identically by re-looping to the is_open
            // re-poll above, which IS the correctness (Principle #6). An
            // UNEXPECTED errno takes the warn-once + bounded-sleep degrade
            // below (see the method doc) so a persistently-failing syscall can
            // never busy-spin.
            let rc = unsafe {
                libc::syscall(
                    libc::SYS_futex,
                    futex_word,
                    libc::FUTEX_WAIT,
                    expected_low,
                    &ts as *const libc::timespec,
                    std::ptr::null::<u32>(),
                    0u32,
                )
            };
            if rc < 0 {
                let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
                if !futex_wait_errno_is_benign(errno) {
                    // Warn once per distinct errno per process (a
                    // plain once-flag would let barrier A's ENOSYS mask a
                    // genuinely different failure class — e.g. barrier B's
                    // EFAULT — forever). The last-warned errno lives in a
                    // process-global AtomicI32 (0 = none; errnos are positive):
                    // zero cost on the happy path, re-warn only when the errno
                    // CHANGES, so a persistently-failing syscall still cannot
                    // flood the log at recheck cadence. Then sleep the bounded
                    // recheck (capped at the time remaining) so the degrade is
                    // sleep-polling, NEVER a busy-spin.
                    static LAST_WARNED_ERRNO: std::sync::atomic::AtomicI32 =
                        std::sync::atomic::AtomicI32::new(0);
                    if LAST_WARNED_ERRNO.swap(errno, Ordering::Relaxed) != errno {
                        tracing::warn!(
                            errno,
                            "FUTEX_WAIT failed with an unexpected errno (kernel park \
                             unavailable — e.g. a seccomp filter); the barrier wait \
                             degrades to bounded 100µs sleep-polling for this process"
                        );
                    }
                    std::thread::sleep(
                        Duration::from_micros(100)
                            .min(deadline.saturating_duration_since(Instant::now())),
                    );
                }
            }
        }
    }

    /// The TIME-bounded pre-block spin — the DEFAULT phase 1 of
    /// [`wait_futex`](Self::wait_futex) ([`barrier_spin_budget`] is
    /// default-on, tier-resolved — 20µs on the kernel-wake tiers (Linux futex,
    /// macOS os_sync), 150µs on the macOS sleep-recheck fallback;
    /// `CERULION_BARRIER_SPIN_US=0` is the kill switch
    /// that restores the 50-iter loop). Busy-polls the generation SENSE
    /// (`Acquire` loads + `spin_loop` hints) until it opens (`true`) or
    /// `spin_deadline` passes (`false` — the caller falls through to the
    /// unchanged `FUTEX_WAIT` block). The hot loop is LOAD-ONLY: the
    /// `Instant::now()` deadline check runs only every 64 iterations, so the
    /// watched cache line sees nothing but reads between opener stores.
    ///
    /// BOUNDED spin-then-block, never unbounded: the caller caps
    /// `spin_deadline` at the wait's own `deadline`. FIREWALL (Principle #7):
    /// record-only progress polling — changes only WHEN the waiter proceeds,
    /// never any barrier state (a pure `&self` read loop).
    ///
    /// No longer Linux-only — the macOS wait tier
    /// (`wait_recheck_fallback`, cfg-gated off Linux) runs the SAME
    /// time-bounded boundary spin as its phase 1 before falling back to the
    /// chunked sleep-recheck.
    fn spin_for_budget(&self, my_gen: u64, spin_deadline: Instant) -> bool {
        let mut iters: u32 = 0;
        loop {
            if self.generation.load(Ordering::Acquire) > my_gen {
                return true;
            }
            iters = iters.wrapping_add(1);
            // Check the wall clock only every 64th iteration — an Instant read
            // is a vDSO call we keep off the hot load loop. 64 Acquire loads
            // are tens of ns, so the budget overshoot is negligible.
            if iters & 63 == 0 && Instant::now() >= spin_deadline {
                return false;
            }
            core::hint::spin_loop();
        }
    }

    /// The non-Linux [`wait`](Self::wait) tier (on macOS this runs
    /// over a REAL cross-process `MAP_SHARED` page — no futex, no UMWAIT/WFE).
    /// Since the os_sync tier landed this is the macOS FALLBACK, not the primary: on macOS ≥ 14.4
    /// [`wait`](Self::wait) prefers `wait_os_sync` (a real kernel wake; a plain
    /// code span since it is macOS-gated but this method compiles on every
    /// non-Linux target), routing here only when the `os_sync_*` symbols are ABSENT
    /// (older macOS), the `CERULION_BARRIER_OS_SYNC=0` kill switch is set, or a
    /// latched `EINVAL`/`ENOTSUP` disabled the primitive — plus every other
    /// non-Linux target (future Windows). The ladder, top to bottom — bounded at
    /// every rung, NEVER an unbounded busy-spin:
    ///
    /// 1. **Boundary spin** — the SAME time-bounded
    ///    `spin_for_budget` phase the kernel-wake
    ///    tiers run (on THIS tier the default resolves to 150µs — the reason:
    ///    without a kernel wake, peers observe a boundary open only at the
    ///    ~100µs recheck cadence, so arrivals carry up to ~one-chunk skew a
    ///    20µs spin almost always misses; 150µs covers the skew + margin —
    ///    M3 A/B: split p50 242µs → 63µs. On the kernel-wake tiers the
    ///    default is the shared 20µs knee — see `default_barrier_spin_us`.
    ///    `CERULION_BARRIER_SPIN_US`
    ///    override/kill-switch, capped at the wait's own deadline). Runs ONCE
    ///    at entry — the boundary crossing — absorbing the fast lockstep
    ///    rendezvous without any sleep/wake tax. Deliberately NOT re-armed
    ///    per recheck iteration: re-arming the budget before every 100µs
    ///    sleep would be a covert busy-spin on a stalled peer (~60% duty
    ///    cycle at the 150µs default). The root-cause fix — a real macOS
    ///    wake primitive rung at the generation store — SHIPPED as the
    ///    os_sync tier (macOS ≥ 14.4); this ladder remains only as
    ///    that tier's fallback.
    /// 2. **Chunked sleep-recheck (the macOS idle shape, validated on Apple Silicon)** —
    ///    try the CPU monitor-wait primitive (Linux-only hardware paths; on
    ///    macOS it reports not-performed), else sleep a ~100µs chunk and
    ///    re-poll, until opened or deadline.
    ///
    /// Do NOT confuse the two `expected`s: the `expected` argument to
    /// `crate::monitor_wait::monitor_wait_until_addr` (a `pub(crate)` fn, so
    /// not an intra-doc link) is the awaited GENERATION VALUE (`gen_now`), NOT
    /// [`BarrierShared::expected`] (the participant COUNT).
    #[cfg(not(target_os = "linux"))]
    fn wait_recheck_fallback(&self, my_gen: u64, deadline: Instant) -> WaitOutcome {
        let recheck = Duration::from_micros(100);
        // Spin-then-block phase 1: the boundary spin — one cached-OnceLock budget read,
        // the spin capped at BOTH the budget and the wait's own deadline
        // (spin-then-block discipline; `=0` kills the spin entirely and drops
        // straight to the sleep-recheck rungs).
        let spin_budget = barrier_spin_budget();
        if !spin_budget.is_zero() {
            let spin_deadline = (Instant::now() + spin_budget).min(deadline);
            if self.spin_for_budget(my_gen, spin_deadline) {
                return WaitOutcome::Opened;
            }
        }
        // The monitor target is the `generation` word itself (the barrier SENSE).
        // `&self.generation` is an `AtomicU64` (always 8-byte aligned), valid for
        // the `&self` borrow — which, for a `MappedBarrier` Deref, keeps the
        // mapping alive for the whole call (it can drop only after the borrow ends).
        let gen_addr = &self.generation as *const AtomicU64 as *const u64;
        loop {
            // ONE Acquire load reused as BOTH the is_open check AND the monitor's
            // `expected` snapshot, so the snapshot↔park window is minimal: if the
            // opener stores my_gen+1 after this load, the primitive's layer-1
            // `read_volatile(addr) != gen_now` early-out catches it (no lost wake).
            let gen_now = self.generation.load(Ordering::Acquire);
            if gen_now > my_gen {
                return WaitOutcome::Opened;
            }
            if Instant::now() >= deadline {
                return WaitOutcome::TimedOut;
            }
            // SAFETY: `gen_addr` is the address of `self.generation`, a valid,
            // 8-byte-aligned `AtomicU64` that stays mapped for this `&self` call
            // (the mapping/Arc cannot drop while `wait(&self)` borrows it). Both
            // contracts of `monitor_wait_until_addr` are upheld.
            let performed = unsafe {
                crate::monitor_wait::monitor_wait_until_addr(gen_addr, gen_now, deadline, recheck)
            }
            .performed();
            if !performed {
                // No real CPU park on this target: sleep the recheck (capped at the
                // time remaining) instead of re-looping — never busy-spin.
                std::thread::sleep(recheck.min(deadline.saturating_duration_since(Instant::now())));
            }
        }
    }

    /// The macOS ≥ 14.4 os_sync tier of [`wait`](Self::wait) — a
    /// wake-assisted spin-then-block on the low-32 `generation` alias via
    /// Apple's `os_sync_wait_on_address_with_timeout`, kernel-woken by
    /// [`wake_waiters`](Self::wake_waiters)'s `os_sync_wake_by_address_all` after
    /// every generation bump. Structurally MIRRORS the Linux
    /// [`wait_futex_with_budget`](Self::wait_futex_with_budget): shared phase-1
    /// [`spin_for_budget`](Self::spin_for_budget) (the [`barrier_spin_budget`]
    /// boundary spin), then a bounded kernel block per 10ms slice, re-polling the
    /// REAL [`is_open`](Self::is_open) after every wake.
    ///
    /// **No lost wake (Principle #6):** ONE `Acquire` load of `generation` is
    /// reused as BOTH the `is_open` check AND the kernel COMPARE VALUE (the low
    /// 32 bits, size [`OS_SYNC_WORD_SIZE`]). A store landing in the
    /// snapshot→block window makes `os_sync_wait_on_address` return IMMEDIATELY
    /// with a NON-NEGATIVE rc (value-mismatch — verified on macOS: rc 0, no errno
    /// set), so the re-loop re-polls `is_open` at once. Each block is
    /// additionally bounded by the 10ms slice, so even a hypothetically-lost wake
    /// costs one slice, not the whole deadline.
    ///
    /// **Errno degrade (mirrors the futex tier):** on rc < 0,
    /// [`os_sync_errno_is_benign`] (`ETIMEDOUT` slice expiry / `EINTR`) re-loops
    /// silently; [`os_sync_errno_is_unrecoverable`] (`EINVAL`/`ENOTSUP` — the
    /// primitive is unusable) LATCHES [`OS_SYNC_DISABLED`] (every future wait
    /// takes the fallback), warns ONCE, and falls through to
    /// [`wait_recheck_fallback`](Self::wait_recheck_fallback) for the REMAINDER
    /// of this wait; any OTHER errno warns once-per-distinct-errno + sleeps a
    /// bounded 100µs recheck — never a busy-spin.
    ///
    /// **Wrap-around:** identical to the futex tier — a false-equal would need
    /// EXACTLY 2^32 generations between the snapshot and the block; the 10ms
    /// slice re-polls the full 64-bit word regardless.
    #[cfg(target_os = "macos")]
    fn wait_os_sync(&self, my_gen: u64, deadline: Instant) -> WaitOutcome {
        /// Max single os_sync block slice — MIRRORS the Linux `FUTEX_SLICE`
        /// (10ms; kept a separate const to leave the Linux hot path untouched).
        /// A real wake arrives in µs; the slice only bounds a hypothetically-lost
        /// wake and keeps each kernel wait finite.
        const OS_SYNC_SLICE: Duration = Duration::from_millis(10);

        // `os_sync_active()` gated the entry here, so the backend is present;
        // resolve defensively and fall to the recheck ladder if it somehow
        // vanished (unreachable — the OnceLock is monotone `Some` once resolved).
        let Some(backend) = os_sync_backend() else {
            return self.wait_recheck_fallback(my_gen, deadline);
        };
        let word = self.os_sync_word();
        loop {
            // Spin-then-block phase 1: the SHARED time-bounded boundary spin (default-on; `=0`
            // kills it). Capped at the wait's own deadline — spin-then-block,
            // never a busy-spin (see `spin_for_budget`).
            let spin_budget = barrier_spin_budget();
            if !spin_budget.is_zero() {
                let spin_deadline = (Instant::now() + spin_budget).min(deadline);
                if self.spin_for_budget(my_gen, spin_deadline) {
                    return WaitOutcome::Opened;
                }
            }
            // ONE Acquire load reused as the is_open check AND the kernel compare
            // value, minimizing the snapshot→block window.
            let gen_now = self.generation.load(Ordering::Acquire);
            if gen_now > my_gen {
                return WaitOutcome::Opened;
            }
            let now = Instant::now();
            if now >= deadline {
                return WaitOutcome::TimedOut;
            }
            let slice = OS_SYNC_SLICE.min(deadline - now);
            // Relative ns timeout (the `_with_timeout` variant; ≤ 10ms fits u64).
            let timeout_ns = slice.as_nanos() as u64;
            // The low 32 bits of the snapshot (size 4) — the LE alias `word`
            // points at. Zero-extended so only the low 4 bytes are compared.
            let expected_low = u64::from(gen_now as u32);
            // SAFETY: `word` is the 8-byte-aligned low-32 alias of the live
            // `generation`, valid for this `&self` call (a `MappedBarrier` Deref
            // keeps the mapping alive for the whole call); `backend.wait` is the
            // dlsym-resolved, signature-checked
            // `os_sync_wait_on_address_with_timeout`. OS_SYNC_WAIT_ON_ADDRESS_SHARED
            // keys on the shared page (cross-process); OS_CLOCK_MACH_ABSOLUTE_TIME
            // is the timeout's clock domain. A value-mismatch returns a
            // non-negative rc at once, reading no memory beyond the address+size.
            let rc = unsafe {
                (backend.wait)(
                    word,
                    expected_low,
                    OS_SYNC_WORD_SIZE,
                    libc::OS_SYNC_WAIT_ON_ADDRESS_SHARED,
                    libc::OS_CLOCK_MACH_ABSOLUTE_TIME,
                    timeout_ns,
                )
            };
            if rc < 0 {
                let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
                if os_sync_errno_is_unrecoverable(errno) {
                    // The kernel park is unusable (EINVAL bad-args / ENOTSUP): latch
                    // it OFF process-wide so every future wait takes the fallback,
                    // warn ONCE, then finish THIS wait on the recheck ladder.
                    if crate::os_sync::latch_os_sync_disabled() {
                        tracing::warn!(
                            errno,
                            "os_sync_wait_on_address returned an unrecoverable errno \
                             (EINVAL/ENOTSUP — the macOS wake primitive is unusable); \
                             disabling it process-wide and degrading the barrier wait \
                             to the chunked sleep-recheck fallback"
                        );
                    }
                    return self.wait_recheck_fallback(my_gen, deadline);
                }
                if !os_sync_errno_is_benign(errno) {
                    // Neither the expected ETIMEDOUT/EINTR nor a hard EINVAL/ENOTSUP.
                    // Warn once per DISTINCT errno (a plain once-flag would let one
                    // errno mask a genuinely different later failure class) + sleep a
                    // bounded 100µs recheck so a persistently-failing syscall can
                    // never busy-spin. Mirrors the Linux futex tier's degrade.
                    static LAST_WARNED_ERRNO: std::sync::atomic::AtomicI32 =
                        std::sync::atomic::AtomicI32::new(0);
                    if LAST_WARNED_ERRNO.swap(errno, Ordering::Relaxed) != errno {
                        tracing::warn!(
                            errno,
                            "os_sync_wait_on_address returned an unexpected errno; the \
                             barrier wait degrades to bounded 100µs sleep-polling for \
                             this errno"
                        );
                    }
                    std::thread::sleep(
                        Duration::from_micros(100)
                            .min(deadline.saturating_duration_since(Instant::now())),
                    );
                }
                // Benign (ETIMEDOUT/EINTR) → silently re-loop to the is_open re-poll.
            }
            // rc >= 0 (woken / value-mismatch / spurious) → re-loop to the is_open
            // re-poll above, which IS the correctness.
        }
    }

    /// Test seam (deflake / A-B): drive a SPECIFIC macOS wait tier
    /// explicitly, bypassing the process-cached [`os_sync_active`] routing, so a
    /// test can A/B the os_sync primitive against the recheck fallback IN ONE
    /// PROCESS (the cached `OnceLock`s can't be flipped mid-run) and force the
    /// kill-switch / fallback route deterministically. Gated behind
    /// `#[cfg(any(test, feature = "test-helpers"))]` — invisible to production;
    /// mirrors [`wait_futex_with_budget`](Self::wait_futex_with_budget)'s
    /// budget-parameterization for the same hermetic-testing reason.
    #[cfg(all(target_os = "macos", any(test, feature = "test-helpers")))]
    pub fn wait_macos_for_test(
        &self,
        my_gen: u64,
        timeout: Duration,
        use_os_sync: bool,
    ) -> WaitOutcome {
        let deadline = Instant::now() + timeout;
        if use_os_sync {
            self.wait_os_sync(my_gen, deadline)
        } else {
            self.wait_recheck_fallback(my_gen, deadline)
        }
    }

    /// The current generation (`Acquire` load). 0 before any generation has
    /// opened; `N` after `N` generations have completed.
    pub fn current_generation(&self) -> u64 {
        self.generation.load(Ordering::Acquire)
    }

    /// The number of participants currently expected at each generation
    /// (`Acquire` load).
    pub fn expected(&self) -> u32 {
        self.expected.load(Ordering::Acquire)
    }

    /// `true` while at least one peer has ARRIVED at the current
    /// generation and it has not yet opened (`remaining < expected`, two
    /// `Acquire` loads) — "someone is waiting on me".
    ///
    /// This is the live loop's step-start wake predicate for its monitor-wait
    /// park (see `GraphRuntime`'s `monitor_wait_block`): in a barrier-lockstep
    /// multi-process split, a context whose only doorbell topics publish at
    /// LATER global levels has no data-side wake source for step START — its
    /// producers cannot fire until THIS context reaches the first barrier, and
    /// its doorbell only rings after they fire (a circular wait that otherwise
    /// only the park's timeout breaks). A peer arriving at the shared barrier is
    /// the true "a step has begun" signal, and it is exactly what this
    /// predicate observes. Because the wake path is fixed structurally, no
    /// launch refusal for such topologies is needed — the never-silently-inert
    /// principle is satisfied by the mechanism itself.
    ///
    /// RECORD-ONLY (Principle #7 firewall): a WHEN-signal that changes when the
    /// parked context re-enters `step()`, never the fire set/order/data.
    ///
    /// This predicate is no longer only POLLED — arrivals also
    /// ring a KERNEL wake (the `wake_seq` epoch the step-start park blocks on,
    /// gated on the `parked` rank bitmask), so the parked reader reaches this
    /// re-derive in µs instead of at the recheck cadence. The predicate stays
    /// the sole ATTRIBUTION/correctness check (the wake is only latency).
    ///
    /// Why ABSOLUTE state (not an epoch/delta like the doorbell ring counters):
    /// the parked reader is never the one decrementing `remaining` (it arrives
    /// only while RUNNING a step, not while parked between steps), and when all
    /// participants are idle between steps the unique opener has already
    /// re-armed `remaining == expected` — so the predicate is `false` exactly
    /// when nobody is waiting, with no baseline snapshot to take or race.
    ///
    /// The no-false-wake-on-DROP half additionally relies on
    /// [`try_drop_participant`](Self::try_drop_participant) decrementing
    /// `expected` BEFORE `remaining` (see the ORDER PIN comment there): a
    /// reader observing the post-drop `remaining` also observes the post-drop
    /// `expected`, so a dead-peer drop never exposes a transient
    /// `remaining < expected` window to a parked survivor.
    pub fn peers_waiting(&self) -> bool {
        self.remaining.load(Ordering::Acquire) < self.expected.load(Ordering::Acquire)
    }

    /// Retarget the expected participant count for the CURRENT and future
    /// generations (`Release` stores to both `remaining` and `expected`).
    ///
    /// Intended for the gen-0 join phase: participants register before the first
    /// barrier and a supervisor pins `N` once they are all present. Because it
    /// overwrites the in-flight `remaining`, it MUST be called at a generation
    /// BOUNDARY — before any participant has arrived at the current generation
    /// (e.g. right after the prior generation opened, or before generation 0's
    /// first `arrive`). Calling it with arrivals already in flight at the current
    /// generation would wipe those decrements; the supervisor sets `N` before
    /// the cohort begins arriving, so the lockstep contract never does that.
    pub fn set_expected(&self, n: u32) {
        // Mid-flight misuse guard: at a generation boundary `remaining == expected`
        // (the opener just re-armed it, or it was never decremented). In-flight
        // arrival decrements would make them differ — calling here is the misuse
        // this method's doc forbids. Debug-only (a `Relaxed` read is enough).
        debug_assert_eq!(
            self.remaining.load(Ordering::Relaxed),
            self.expected.load(Ordering::Relaxed),
            "set_expected called mid-generation: remaining has in-flight decrements (must be called at a generation boundary)"
        );
        // Arm `remaining` first, then publish `expected`. At a boundary (no
        // in-flight arrivals) either order is fine; this order means an observer
        // that reads the new `expected` also sees the matching `remaining`.
        self.remaining.store(n, Ordering::Release);
        self.expected.store(n, Ordering::Release);
        // A cohort retarget is a barrier state transition —
        // publish it to any step-start parker (a gen-0 join-phase edge; cold
        // path, keeps the "every state transition bumps the epoch" invariant
        // total). See `note_activity`.
        self.note_activity();
    }

    /// Drop one participant from the expected count (dead-peer handling) and
    /// count it OUT of the current generation, re-checking completion.
    ///
    /// This is the ambiguous-return convenience wrapper over
    /// [`try_drop_participant`](Self::try_drop_participant): it collapses the
    /// [`DropAttempt::Stale`] no-op back into an [`ArriveOutcome::Pending`], so a
    /// caller that does NOT need to distinguish "stale no-op" from "applied but
    /// incomplete" (e.g. a WORKER self-dropping its OWN, unambiguous, next-arrival
    /// generation — [`crate::graph`]'s `leave_barrier_cohort`) keeps its original
    /// [`ArriveOutcome`] signature. A SUPERVISOR dropping a DEAD peer (whose exact
    /// generation it cannot know) must use [`try_drop_participant`](Self::try_drop_participant)
    /// / [`drop_dead_peer`](Self::drop_dead_peer) instead, which need the Stale/Applied
    /// fork to retry safely.
    pub fn drop_participant(&self, my_gen: u64) -> ArriveOutcome {
        match self.try_drop_participant(my_gen) {
            DropAttempt::Stale => ArriveOutcome::Pending,
            DropAttempt::Applied(outcome) => outcome,
        }
    }

    /// Drop one participant from the expected count (dead-peer handling), reporting
    /// [`DropAttempt::Stale`] (unambiguous no-op) vs [`DropAttempt::Applied`]
    /// (see [`DropAttempt`] for why a supervisor retry loop needs the distinction).
    ///
    /// When a peer is declared dead it can no longer arrive, so a barrier it
    /// never reached would deadlock the survivors. Dropping it (a) decrements
    /// `expected` (SATURATING — never underflows below 0) so FUTURE generations
    /// are sized for the survivors, and (b) counts the dead peer out of the
    /// CURRENT generation with the SAME `remaining.fetch_sub(1)` an arrival uses.
    /// Because arrive and drop both reach 0 through that one atomic, exactly one
    /// caller observes `prev == 1` → a UNIQUE opener. If the drop's `fetch_sub`
    /// is the one that hits 0 it opens `my_gen` (re-arm + bump) and returns
    /// `Applied(Opened(my_gen + 1))`; if the generation is incomplete it returns
    /// `Applied(Pending)`; if `my_gen` was already open it returns `Stale` WITHOUT
    /// touching `expected`/`remaining` (a full no-op — the caller re-reads the
    /// current generation and retries).
    ///
    /// # Race note
    ///
    /// The unique-opener property holds even under a drop/arrive race: arrive and
    /// drop share the single atomic `fetch_sub`, so only one observes `prev == 1`
    /// — NO duplicate `Opened`, and (unlike the prior two-slot accumulator
    /// design) NO second opener that could re-arm `remaining` after a fast
    /// participant has already advanced into the next generation, which would
    /// silently LOSE that arrival. Production wires this via the SINGLE-DROPPER
    /// [`drop_dead_peer`](Self::drop_dead_peer): ONE supervisor serializes every
    /// dead-peer drop and each worker self-drops only its OWN slot, so a drop never
    /// races another drop of the same slot. Two SUPERVISORS concurrently dropping
    /// DIFFERENT peers remains unsupported (out of scope).
    pub fn try_drop_participant(&self, my_gen: u64) -> DropAttempt {
        // Already open? Don't touch the count-down for a completed generation —
        // report Stale (a FULL no-op) so a retry loop re-reads and retries instead
        // of mistaking it for an applied-but-incomplete drop.
        if self.generation.load(Ordering::Acquire) > my_gen {
            return DropAttempt::Stale;
        }

        // Drop the dead peer from the expected count (saturating — never
        // underflow), so the NEXT generation's re-arm is sized for the survivors.
        // Capture the PRE-decrement value: the underflow gate below must judge the
        // desync against the count BEFORE this drop, else a drop bringing
        // `expected` 1→0 would read 0 and silently swallow a real desync.
        let old_expected = self
            .expected
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |v| {
                Some(v.saturating_sub(1))
            })
            // The closure always returns `Some`, so `fetch_update` always returns
            // `Ok(prev)`; extract the value from either arm without `.unwrap()`.
            .unwrap_or_else(|v| v);

        // Count the dead peer OUT of the current generation exactly like an
        // arrival, so the opener stays unique.
        //
        // ORDER PIN: `expected` MUST be decremented (above) BEFORE
        // `remaining` (here). `peers_waiting` (`remaining < expected`, the
        // live-loop park's step-start wake predicate) infers expected-freshness
        // from the `remaining` RMW chain: a reader observing the post-drop
        // `remaining` also observes the post-drop `expected`, so a dead-peer
        // drop at an idle re-armed boundary never exposes a transient
        // `remaining < expected` window (mid-drop reads remaining=n,
        // expected=n-1 ⇒ false). Swapping the two decrements would give every
        // parked survivor a spurious (WHEN-only, bounded) wake per drop and
        // break the documented no-false-wakes contract.
        let prev = self.remaining.fetch_sub(1, Ordering::AcqRel);
        // A drop is a barrier state transition like an arrival —
        // publish it to any step-start parker (unconditional epoch bump,
        // parked-gated syscall; the opener path below bumps again via
        // `open_generation`, harmless for an epoch). See `note_activity`.
        self.note_activity();
        if prev == 1 {
            self.open_generation(my_gen);
            DropAttempt::Applied(ArriveOutcome::Opened(my_gen + 1))
        } else if prev == 0 {
            // `remaining` was already 0 (the generation's arrivals were already
            // met, or `expected == 0`). Restore and report Applied(Pending).
            self.remaining.fetch_add(1, Ordering::AcqRel);
            // A PRE-decrement `expected == 0` is a documented vacuous degenerate
            // (the zero-participant barrier — see `arrive`); only `old_expected != 0`
            // here is a real lockstep desync worth shouting about.
            if old_expected != 0 {
                tracing::error!(
                    my_gen,
                    "barrier drop underflow: drop of an already-met generation (lockstep desync)"
                );
                debug_assert!(
                    false,
                    "barrier drop underflow at gen {my_gen} with expected != 0"
                );
            }
            DropAttempt::Applied(ArriveOutcome::Pending)
        } else {
            DropAttempt::Applied(ArriveOutcome::Pending)
        }
    }

    /// OWNER-side (supervisor) drop of a peer that can no longer arrive — the
    /// robust, generation-agnostic wrapper over
    /// [`try_drop_participant`](Self::try_drop_participant) that a supervisor calls
    /// when it observes a worker DEATH but does NOT know the exact barrier
    /// generation the dead peer sat at (peer-loss=continue).
    ///
    /// # Algorithm
    ///
    /// 1. **NoSlots gate.** If `expected() == 0` there is no slot to drop → return
    ///    [`DeadPeerDrop::NoSlots`] (the caller logs it loudly — a peer was dropped
    ///    from an empty cohort).
    /// 2. **Disambiguation phase.** Snapshot `g0 = current_generation()`, then poll
    ///    (~1 ms steps) until EITHER `current_generation() > g0` (the dead peer's
    ///    at-most-one pending PRE-arrival at `g0` was consumed by the survivors
    ///    completing `g0` — so `g0` is closed and a drop at any gen `> g0` is
    ///    PROVABLY the dead peer's genuinely-missing slot) OR `g0` has stayed stable
    ///    for `stall_grace` (the dead peer never pre-arrived, so drop at `g0`).
    /// 3. **Apply loop (bounded).** Repeatedly
    ///    `try_drop_participant(current_generation())`: on
    ///    [`DropAttempt::Applied`] return [`DeadPeerDrop::Applied`]; on
    ///    [`DropAttempt::Stale`] the generation advanced between the read and the
    ///    call, so retry. After a fixed bound return
    ///    [`DeadPeerDrop::ExhaustedRetries`] (should be unreachable with one
    ///    supervisor).
    ///
    /// # Residual single-boundary ambiguity
    ///
    /// If the dead peer HAD pre-arrived at `g0` AND a live survivor is slower than
    /// `stall_grace`, the disambiguation phase times out and drops at `g0` — opening
    /// that generation ONE arrival early. The slow survivor's later `arrive(g0)` is
    /// then absorbed by [`arrive`](Self::arrive)'s stale-generation guard (a benign
    /// idempotent no-op, since `generation` is already `> g0`), and every SUBSEQUENT
    /// generation is correctly sized (`expected` was decremented). So the worst case
    /// is a SINGLE-boundary gate slip at the fault instant — which is already the
    /// non-replayable instant (a crash is not a replayable event). No
    /// recording of the departure is done here (the replay gap is stated in
    /// the PR, not papered over with fabricated data).
    ///
    /// # Memory ordering (concurrent same-gen drop vs a live arrival)
    ///
    /// `try_drop_participant` decrements `expected` (a `fetch_update`, `AcqRel`)
    /// SEQUENCED-BEFORE its `remaining.fetch_sub` (`AcqRel`). A later opener's re-arm
    /// reads `expected` with an `Acquire` load; because the `remaining` `fetch_sub`
    /// chain is `AcqRel`, any arrival whose `fetch_sub` is ordered AFTER this drop's
    /// `fetch_sub` (it read the `remaining` this drop wrote) also observes this
    /// drop's decremented `expected` — so the next generation is re-armed for the
    /// survivor count, never the stale pre-drop count. This is the same ordering
    /// that makes a worker self-drop safe against a concurrent survivor arrival.
    ///
    /// # Single-dropper contract
    ///
    /// ONE supervisor serializes ALL `drop_dead_peer` calls; workers self-drop only
    /// their OWN slot (via [`drop_participant`](Self::drop_participant)). Concurrent
    /// MULTIPLE `drop_dead_peer` callers (two supervisors dropping different peers
    /// at once) remain unsupported.
    pub fn drop_dead_peer(&self, stall_grace: Duration) -> DeadPeerDrop {
        // Production path: no post-snapshot test hook (a never-taken `None` branch on
        // the cold supervisor dead-peer path — see `drop_dead_peer_inner`).
        self.drop_dead_peer_inner(stall_grace, None)
    }

    /// The shared body of [`drop_dead_peer`](Self::drop_dead_peer), with an OPTIONAL
    /// `post_snapshot_hook` invoked EXACTLY ONCE immediately after the disambiguation
    /// phase snapshots `g0 = current_generation()` and BEFORE the disambiguation poll
    /// loop (and its `stall_grace` deadline clock) begins.
    ///
    /// # Why the hook exists — a specific CI flake, not a feature
    ///
    /// The "dropper OBSERVES a generation advance during the grace window, then
    /// applies the resize at the NEXT boundary" scenario is genuinely concurrent: a
    /// survivor must complete `g0` (advancing `generation`) WHILE this method polls in
    /// disambiguation. A test can only pin that ordering deterministically if it can
    /// establish a happens-before on "`g0` has been snapshotted" — otherwise it must
    /// *bias* the interleave with a sleep. That sleep is exactly the defect that
    /// flaked `barrier_test.rs::drop_dead_peer_observes_gen_advance_then_applies_at_next_gen`
    /// on a Linux CI run: a stalled Ubuntu
    /// runner descheduled the dropper thread past its 20 ms bias sleep, so it
    /// snapshotted a LATER generation than the test assumed, and the assertion
    /// "survivor waits at gen 1" (`Pending`) instead observed the equally-legal
    /// early-apply `Opened(2)` (the survivor, not the drop, winning the shared
    /// `remaining` count-down at gen 1). Both outcomes are valid product behavior; the
    /// bug was pinning ONE of them with a sleep. The hook replaces the bias sleep with
    /// a real happens-before over shared observable state (`snapshotted` set here; the
    /// test then advances the generation and, still inside the hook, parks the dropper
    /// until the survivor has counted `remaining` down), so both the "observed gen
    /// advance" branch AND the survivor-before-drop opener ordering are deterministic.
    ///
    /// # Production cost
    ///
    /// Zero. Production reaches this only through the public
    /// [`drop_dead_peer`](Self::drop_dead_peer), which passes `None`; the single
    /// `if let Some(..)` is a never-taken branch on the COLD supervisor
    /// dead-peer-handling path (one call per worker death), never the transport hot
    /// path. The `Some` arm is only reachable via the test-only
    /// `drop_dead_peer_with_post_snapshot_hook` (gated behind
    /// `#[cfg(any(test, feature = "test-helpers"))]`).
    fn drop_dead_peer_inner(
        &self,
        stall_grace: Duration,
        post_snapshot_hook: Option<&dyn Fn()>,
    ) -> DeadPeerDrop {
        // (1) NoSlots gate — nothing to drop from an empty cohort.
        if self.expected() == 0 {
            return DeadPeerDrop::NoSlots;
        }

        // (2) Disambiguation phase: wait for the dead peer's at-most-one pending
        // pre-arrival to be consumed (gen advances past g0), or for g0 to be stable
        // for `stall_grace` (the peer never pre-arrived).
        let g0 = self.current_generation();

        // TEST SEAM (deflake — see this fn's doc): fire the optional hook the instant
        // `g0` is captured, so a test can establish a happens-before on "the dropper
        // has snapshotted g0" instead of biasing the interleave with a sleep. `None`
        // in production (a never-taken cold-path branch). Placed BEFORE `deadline` is
        // computed so a hook that parks the thread does not eat into the grace window.
        if let Some(hook) = post_snapshot_hook {
            hook();
        }

        let poll = Duration::from_millis(1);
        let deadline = Instant::now() + stall_grace;
        loop {
            if self.current_generation() > g0 {
                // Pre-arrival consumed → a drop at any gen > g0 is the dead peer's
                // genuinely-missing slot.
                break;
            }
            let now = Instant::now();
            if now >= deadline {
                // g0 stable for stall_grace → the peer never pre-arrived; drop at g0
                // (residual single-boundary ambiguity documented above).
                break;
            }
            std::thread::sleep(poll.min(deadline.saturating_duration_since(now)));
        }

        // (3) Apply loop (bounded). A Stale means the generation advanced between
        // the read and the call — re-read and retry. One supervisor + monotonic
        // generations make >1 retry vanishingly unlikely; the bound guarantees we
        // never spin forever.
        const MAX_ATTEMPTS: u32 = 64;
        for _ in 0..MAX_ATTEMPTS {
            match self.try_drop_participant(self.current_generation()) {
                DropAttempt::Applied(outcome) => {
                    return DeadPeerDrop::Applied {
                        opened: matches!(outcome, ArriveOutcome::Opened(_)),
                    };
                }
                DropAttempt::Stale => continue,
            }
        }
        DeadPeerDrop::ExhaustedRetries
    }

    /// Test-only (deflake) variant of [`drop_dead_peer`](Self::drop_dead_peer): runs
    /// the identical body but fires `post_snapshot_hook` exactly once, right after the
    /// internal `g0 = current_generation()` snapshot and before the disambiguation
    /// poll loop. It exists SOLELY to make the "observed gen advance" interleave in
    /// `barrier_test.rs::drop_dead_peer_observes_gen_advance_then_applies_at_next_gen`
    /// deterministic, replacing a flake-prone bias sleep with a real happens-before
    /// (see `drop_dead_peer_inner`'s doc for the full CI-flake rationale — on
    /// Linux CI, "survivor waits at gen 1" observed the
    /// legal early-apply `Opened(2)`).
    ///
    /// Gated behind `#[cfg(any(test, feature = "test-helpers"))]`, so it is invisible
    /// to production and downstream builds; the crate self-references `test-helpers`
    /// in its dev-dependencies to expose it to `tests/*.rs`.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn drop_dead_peer_with_post_snapshot_hook(
        &self,
        stall_grace: Duration,
        post_snapshot_hook: &dyn Fn(),
    ) -> DeadPeerDrop {
        self.drop_dead_peer_inner(stall_grace, Some(post_snapshot_hook))
    }
}

/// RAII holder of a rank's `parked` bit — the ONLY caller of
/// [`BarrierShared::park_exit`], so the bit is cleared on EVERY exit path
/// (wake, timeout, early loop exit, panic unwind) of the wake-word block.
/// `enter` returns `None` for `rank >= 32` (beyond the bitmask — that context
/// keeps today's slice-timeout cadence; `park_enter` warns once) so a `None`
/// guard doubles as the "no wake-word fast path for this context" signal.
///
/// SCOPE: the bit's semantic is "a waiter is blocked on the
/// wake word RIGHT NOW" — the guard is scoped to the `!performed` kernel-block
/// arm of `monitor_wait_block` (set immediately before the
/// snapshot→re-derive→block sequence, dropped right after), NOT the whole
/// park. A hardware CPU park (Linux UMWAIT/WFE — `performed == true`) never
/// reaches that arm, never sets the bit, and therefore never taxes arrivers
/// with a futile wake syscall the hardware park cannot hear (the wake word is
/// not the line the CPU monitor watches). The brief pre-block window where the
/// bit is set but the waiter is not yet in the kernel is required by the
/// SB-litmus protocol (bit-set BEFORE the predicate re-derive) and costs at
/// most one early syscall, absorbed by the block's fresh snapshot.
#[must_use = "the parked bit is cleared on drop — bind the guard for the block's lifetime"]
pub struct ParkedRankGuard<'a> {
    barrier: &'a BarrierShared,
    rank: u32,
}

impl<'a> ParkedRankGuard<'a> {
    /// Set rank `rank`'s parked bit and return the clearing guard, or `None`
    /// for a beyond-mask rank (no bit set — see [`BarrierShared::park_enter`]).
    pub fn enter(barrier: &'a BarrierShared, rank: u32) -> Option<Self> {
        if barrier.park_enter(rank) {
            Some(Self { barrier, rank })
        } else {
            None
        }
    }
}

impl Drop for ParkedRankGuard<'_> {
    fn drop(&mut self) {
        self.barrier.park_exit(self.rank);
    }
}

/// Test-only: process-global count of REAL `park_enter`
/// bit-sets (rank < 32 only — beyond-mask parks never touch the mask and are
/// not counted). The hardware-park test arm snapshots this before/after a run
/// to prove the parked bit was NEVER set while a CPU primitive performed the
/// park (the no-futile-wake pin); the wake-word arm asserts a positive delta
/// (anti-tautology — the counter demonstrably moves). Record-only, cfg-gated
/// to zero production cost.
#[cfg(any(test, feature = "test-helpers"))]
static PARK_ENTER_CALLS: AtomicU64 = AtomicU64::new(0);

/// Test seam: read the process-global `PARK_ENTER_CALLS`
/// counter (plain code span — a private static; keeps the doc gate link-clean).
#[cfg(any(test, feature = "test-helpers"))]
pub fn park_enter_call_count_for_test() -> u64 {
    PARK_ENTER_CALLS.load(Ordering::Relaxed)
}

/// Whether THIS host has a kernel-block PRIMITIVE for the
/// step-start park's wake word — Linux always (shared futex), macOS iff the
/// os_sync tier is active (resolved + not kill-switched + not latched off),
/// other targets never.
///
/// **PRIMITIVE EXISTENCE ONLY — this does NOT imply the wake-word arm is
/// taken** (the name deliberately avoids `barrier_park_wake_available`,
/// which would read as a runtime routing promise): on a Linux machine with a
/// hardware CPU park (WAITPKG/WFE), `monitor_wait_block`'s park is performed
/// by the CPU primitive and the `!performed` wake-word arm — the ONLY
/// consumer of this predicate on the park path — never runs, even though this
/// returns `true`. "Which path does THIS machine's park actually take" is
/// `crate::monitor_wait::monitor_wait_available()` (hardware park) composed
/// with this (wake-word block vs sleep pacing on the `!performed` arm). The
/// hermetic tests that drive [`BarrierShared::park_wait_activity`] DIRECTLY
/// gate on this predicate correctly (they bypass the park routing). Also a
/// CI-log diagnostic alongside [`barrier_wake_tier`].
pub fn wake_word_block_primitive_available() -> bool {
    #[cfg(target_os = "linux")]
    {
        true
    }
    #[cfg(target_os = "macos")]
    {
        os_sync_active()
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos")))]
    {
        false
    }
}

/// The wake tier [`BarrierShared::wait`] selects on THIS host — a
/// DIAGNOSTIC accessor for tests + CI logs, NEVER a control input (the wait path
/// reads the tier machinery directly). One of:
/// - `"linux-futex"` — the Linux process-shared `FUTEX_WAIT`/`FUTEX_WAKE` tier.
/// - `"macos-os_sync"` — the macOS ≥ 14.4 `os_sync_wait_on_address` /
///   `os_sync_wake_by_address_all` tier (the backend resolved + is active).
/// - `"macos-recheck-fallback"` — macOS < 14.4, the `CERULION_BARRIER_OS_SYNC=0`
///   kill switch, or a latched `EINVAL`/`ENOTSUP`: the chunked sleep-recheck.
/// - `"recheck-fallback"` — any other OS (future Windows): sleep-recheck.
pub fn barrier_wake_tier() -> &'static str {
    #[cfg(target_os = "linux")]
    {
        "linux-futex"
    }
    #[cfg(target_os = "macos")]
    {
        if os_sync_active() {
            "macos-os_sync"
        } else {
            "macos-recheck-fallback"
        }
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos")))]
    {
        "recheck-fallback"
    }
}

/// Whether the macOS `os_sync_*` wake backend RESOLVED on this host
/// (the 14.4+ symbols were found via `dlsym`) — INDEPENDENT of the kill switch /
/// the `EINVAL` disable latch, so it is a stable "is the primitive present"
/// signal for CI logs (unlike [`barrier_wake_tier`], which also folds in the
/// runtime gates). Always `false` off macOS.
pub fn barrier_os_sync_backend_available() -> bool {
    #[cfg(target_os = "macos")]
    {
        os_sync_backend().is_some()
    }
    #[cfg(not(target_os = "macos"))]
    {
        false
    }
}

/// Test helper: resolve the CURRENT `CERULION_BARRIER_OS_SYNC` process
/// env (read FRESH, NOT the `OnceLock` cache) to "os_sync disabled?" — so a test
/// can set the env via an RAII guard and prove the kill switch is honored
/// end-to-end (the env var NAME + the `=0` parse), independent of the cached
/// [`os_sync_active`] reader (which caches process-wide on first read). Gated
/// behind `#[cfg(any(test, feature = "test-helpers"))]`; macOS-only.
#[cfg(all(target_os = "macos", any(test, feature = "test-helpers")))]
pub fn os_sync_disabled_from_env_uncached() -> bool {
    resolve_os_sync_disabled(std::env::var("CERULION_BARRIER_OS_SYNC").ok().as_deref())
}

/// The DEFAULT barrier-boundary spin budget (µs) on a KERNEL-WAKE
/// tier — the measured 20µs knee. Since the os_sync tier landed this is no longer a
/// per-OS cfg split: 20µs is the default wherever a real kernel wake delivers
/// the boundary open in µs — Linux (futex, both arches) AND macOS ≥ 14.4 with
/// the os_sync tier active. Only the macOS sleep-recheck FALLBACK needs more
/// (`MACOS_FALLBACK_SPIN_US` — macOS-gated, hence a plain code span keeping
/// this all-OS const doc link-clean on Linux); the per-tier resolution lives
/// in [`default_barrier_spin_us`].
///
/// **Why 20µs (measured on Linux):** sweeping the budget on both
/// benchmark machines showed 20µs already captures essentially the whole win
/// (20µs ≈ 100µs ≈ all-spun on both arches): aarch64 (Jetson) split p50
/// 51.3 → 36.5µs (cross-process tax +19.3 → +4.5µs), x86 6.8 → 6.2µs (tax
/// +1.85 → +1.3µs). Above the knee a larger budget only burns CPU; below it
/// the kernel sleep/wake tax returns — the wake lands in µs, so 20µs suffices
/// to ride through the lockstep rendezvous. The os_sync wake measured the same
/// class on the M3 (os_sync A/B: parked-wake p50 ~13µs vs the fallback's
/// ~84µs), so the kernel-wake knee carries over.
///
/// Override / kill via `CERULION_BARRIER_SPIN_US` (see
/// [`barrier_spin_budget`]) — semantics identical on every OS and tier.
/// Shared by ALL wait tiers — the Linux futex tier's pre-block spin,
/// the macOS os_sync tier's pre-block spin, and the recheck tier's boundary
/// spin.
const DEFAULT_BARRIER_SPIN_US: u64 = 20;

/// The DEFAULT spin budget (µs) on the macOS sleep-recheck
/// FALLBACK tier — 150µs. Without a kernel wake a peer's arrival is observed
/// only by the chunked ~100µs sleep-recheck, so peers arrive at a boundary
/// with up to ~one-chunk (~100µs) skew — which a 20µs spin almost always
/// MISSES, dropping every boundary into the sleep-recheck tier. Measured on
/// the M3 in an A/B run: split p50 242µs → 63µs with the 150µs budget (covers
/// the worst-case chunk skew + margin). Since the os_sync tier landed this applies ONLY when
/// the os_sync wake is NOT active (macOS < 14.4, the
/// `CERULION_BARRIER_OS_SYNC=0` kill switch, or the `EINVAL`/`ENOTSUP` latch)
/// — with the wake active the arrival skew is gone and the default drops back
/// to the shared [`DEFAULT_BARRIER_SPIN_US`] 20µs knee (see
/// [`default_barrier_spin_us`]).
#[cfg(target_os = "macos")]
const MACOS_FALLBACK_SPIN_US: u64 = 150;

/// Resolve the DEFAULT spin budget (µs) for THIS host's active wake
/// tier. Linux (and any non-macOS target): the compile-time
/// [`DEFAULT_BARRIER_SPIN_US`] const, unchanged — no runtime tier query on
/// that path. macOS: runtime two-tier — 20µs when the os_sync kernel wake is
/// active (the wake kills the arrival skew the bigger budget existed to
/// cover), 150µs on the sleep-recheck fallback. An explicit
/// `CERULION_BARRIER_SPIN_US` value still overrides whatever this resolves
/// (see [`barrier_spin_budget`]).
///
/// Resolution-order note (macOS): this reads `os_sync_active` (macOS-gated —
/// plain code span; this fn is all-OS) at
/// [`barrier_spin_budget`]'s one-time `OnceLock` init — i.e. at the FIRST
/// barrier wait. A LATER `EINVAL`/`ENOTSUP` latch (mid-run, after the budget
/// is cached) therefore leaves an already-resolved 20µs budget on the
/// now-degraded fallback tier: a latency nuance in an already-degraded mode,
/// never a correctness issue (the fallback's sleep-recheck is bounded
/// regardless of the spin budget).
fn default_barrier_spin_us() -> u64 {
    #[cfg(target_os = "macos")]
    {
        resolve_default_barrier_spin_us(os_sync_active())
    }
    #[cfg(not(target_os = "macos"))]
    {
        DEFAULT_BARRIER_SPIN_US
    }
}

/// The PURE macOS two-tier default — `os_sync_on` → the shared 20µs
/// kernel-wake knee; fallback → the 150µs skew cover. Split out of
/// [`default_barrier_spin_us`] (which feeds it the LIVE [`os_sync_active`])
/// so BOTH arms are oracle-testable without `OnceLock`/env games (the same
/// seam pattern as [`resolve_os_sync_disabled`]). Emits a `tracing::debug!`
/// breadcrumb naming the selected tier + default — in production this runs
/// EXACTLY ONCE, from [`barrier_spin_budget`]'s `OnceLock` init.
#[cfg(target_os = "macos")]
fn resolve_default_barrier_spin_us(os_sync_on: bool) -> u64 {
    if os_sync_on {
        tracing::debug!(
            tier = "macos-os_sync",
            default_spin_us = DEFAULT_BARRIER_SPIN_US,
            "barrier spin default resolved for the os_sync kernel-wake tier"
        );
        DEFAULT_BARRIER_SPIN_US
    } else {
        tracing::debug!(
            tier = "macos-recheck-fallback",
            default_spin_us = MACOS_FALLBACK_SPIN_US,
            "barrier spin default resolved for the sleep-recheck fallback tier"
        );
        MACOS_FALLBACK_SPIN_US
    }
}

/// The HARD upper cap on the applied barrier-boundary
/// spin budget (µs) — 100ms. `CERULION_BARRIER_SPIN_US` accepts any parseable
/// `u64`, so without this clamp a fat-fingered value (e.g. `5_000_000` µs =
/// 5s) would turn the bounded spin-then-block into a de-facto **busy-spin** for
/// the entire ~5s [`crate::BARRIER_BOUNDARY_TIMEOUT`] window on a stalled peer (the
/// spin's own deadline is capped at the wait's deadline, which for a stalled
/// cohort IS the boundary timeout). 100ms is ~5000× the 20µs Linux knee — so
/// far above any legitimate override that the clamp can only ever catch a
/// mistake (~670× even the 150µs macOS fallback-tier default) — yet an order of magnitude BELOW the boundary timeout, keeping the
/// "never a busy-spin" guarantee unconditional. A request above this cap is
/// clamped in [`resolve_barrier_spin_budget`] with a loud `tracing::warn!`.
const MAX_BARRIER_SPIN_US: u64 = 100_000;

/// Pure parse of the `CERULION_BARRIER_SPIN_US` value →
/// `(applied_us, was_garbage)`. Unset / empty → the caller's `default_us`
/// (the tier-resolved default — [`default_barrier_spin_us`]; a PARAMETER
/// since the os_sync tier landed so both macOS tier defaults are oracle-testable); `"0"` → 0
/// (the explicit KILL SWITCH — restores the pure 50-iter + kernel-block
/// behavior); any other non-negative integer → that number (override);
/// garbage → the default, flagged `was_garbage = true` so the caller warns
/// LOUDLY (house rule: loud over silent). Extracted so the parse contract is
/// oracle-testable without env mutation (the cached reader is
/// [`barrier_spin_budget`]).
fn parse_barrier_spin_us(raw: Option<&str>, default_us: u64) -> (u64, bool) {
    match raw {
        None => (default_us, false),
        Some("") => (default_us, false),
        Some(s) => match s.parse::<u64>() {
            Ok(us) => (us, false),
            Err(_) => (default_us, true),
        },
    }
}

/// The barrier-boundary spin budget — DEFAULT-ON at the TIER-RESOLVED
/// default ([`default_barrier_spin_us`]: 20µs on every kernel-wake tier —
/// Linux futex AND macOS os_sync — and 150µs on the macOS sleep-recheck
/// fallback; a deliberate API decision, re-tiered when the os_sync wake
/// killed the skew the 150 covered). Before each
/// kernel block in [`BarrierShared::wait`]'s futex/os_sync tiers, the waiter
/// busy-polls the generation word for up to this budget — a TIME-bounded
/// spin-then-block that spins THROUGH the lockstep rendezvous instead of
/// paying the kernel sleep/wake at every barrier boundary (measured:
/// aarch64 split p50 51.3 → 36.5µs, x86 6.8 → 6.2µs — see the const doc).
/// The macOS recheck tier reads the SAME budget for its entry
/// boundary spin (one spin per `wait`, then chunked sleep-rechecks).
///
/// `CERULION_BARRIER_SPIN_US` is the kill switch / override: `=0` disables
/// the time-bounded spin entirely (the earlier pure 50-iter + futex
/// behavior); `=N` overrides the budget — CLAMPED to
/// [`MAX_BARRIER_SPIN_US`] (100ms) with a loud `tracing::warn!` above that;
/// unset/empty → the default; garbage → the default with ONE loud
/// `tracing::warn!`. All warns are emitted once per process from
/// [`resolve_barrier_spin_budget`] via this `OnceLock` init closure. Read
/// ONCE per process (mirroring `GraphRuntime::wake_ahead`).
///
/// **Bounded, never a busy-spin — UNCONDITIONALLY:** the spin
/// is capped at the budget AND the wait's own deadline, then falls through to
/// the unchanged kernel block. Since the budget itself is now hard-capped at
/// [`MAX_BARRIER_SPIN_US`] (100ms, ~5000× the knee, ≪ the ~5s boundary
/// timeout), the "never a busy-spin" guarantee holds for ANY env value —
/// garbage, zero, or a hostile `5_000_000` — not just the shipped default:
/// no `CERULION_BARRIER_SPIN_US` can make a stalled-peer boundary busy-spin.
/// Worst-case CPU = boundaries/step × budget × step rate (e.g. 5 levels ×
/// 20µs at 1kHz = 10% of one core on the kernel-wake tiers; the 150µs macOS
/// fallback-tier default bounds at 75%) — but that bound assumes EVERY boundary
/// spins its full budget; in lockstep the peers arrive together, so the spin
/// exits in ns–µs and the measured real burn is ≪ 1%.
///
/// FIREWALL (Principle #7): the spin changes only WHEN the waiter proceeds
/// past the boundary, never the fire set/order (the same contract as the
/// futex tier itself: the `is_open` re-poll stays the sole correctness).
fn barrier_spin_budget() -> Duration {
    static SPIN_BUDGET: std::sync::OnceLock<Duration> = std::sync::OnceLock::new();
    *SPIN_BUDGET.get_or_init(|| {
        let raw = std::env::var("CERULION_BARRIER_SPIN_US").ok();
        // The default is tier-resolved HERE (once) — 20µs on a
        // kernel-wake tier, 150µs on the macOS sleep-recheck fallback; a
        // compile-time const on Linux (no runtime tier query on that path).
        resolve_barrier_spin_budget(raw.as_deref(), default_barrier_spin_us())
    })
}

/// Resolve the raw `CERULION_BARRIER_SPIN_US` env value to
/// the applied spin [`Duration`], emitting every loud `tracing::warn!` the
/// resolution requires. Split out of [`barrier_spin_budget`]'s `OnceLock`
/// closure so the WARN emissions are `#[traced_test]`-pinnable WITHOUT
/// env/`OnceLock` games (the closure only wires the process-wide cache).
///
/// Two warn paths:
/// - **garbage** — a non-parseable value (`parse_barrier_spin_us` flags
///   `was_garbage`): apply the caller's tier-resolved `default_us` and warn
///   (house rule: loud over silent).
/// - **over-cap** — a parseable value above [`MAX_BARRIER_SPIN_US`]: clamp to
///   the cap and warn with structured `requested_us` / `clamped_us`, so a
///   fat-fingered budget can never busy-spin the boundary-timeout window.
///
/// `=0` (the kill switch) and any in-range value resolve SILENTLY.
///
/// `default_us` is the tier-resolved default ([`default_barrier_spin_us`]) —
/// a PARAMETER since the os_sync tier landed so both macOS tier arms are pinnable without
/// `OnceLock`/env games (the production caller threads the live value in).
fn resolve_barrier_spin_budget(raw: Option<&str>, default_us: u64) -> Duration {
    let (parsed_us, was_garbage) = parse_barrier_spin_us(raw, default_us);
    if was_garbage {
        tracing::warn!(
            env = "CERULION_BARRIER_SPIN_US",
            got = %raw.unwrap_or(""),
            applied_default_us = default_us,
            "CERULION_BARRIER_SPIN_US is set but not a non-negative integer \
             (microseconds); applying the default barrier-boundary spin budget \
             (`0` is the explicit kill switch)"
        );
    }
    let clamped_us = parsed_us.min(MAX_BARRIER_SPIN_US);
    if clamped_us != parsed_us {
        tracing::warn!(
            env = "CERULION_BARRIER_SPIN_US",
            requested_us = parsed_us,
            clamped_us,
            max_barrier_spin_us = MAX_BARRIER_SPIN_US,
            "CERULION_BARRIER_SPIN_US exceeds the 100ms hard cap; clamping the \
             barrier-boundary spin budget so it stays a bounded spin-then-block \
             (never a busy-spin) far below the ~5s barrier boundary timeout"
        );
    }
    Duration::from_micros(clamped_us)
}

/// Classify a failing `FUTEX_WAIT`'s errno for
/// [`BarrierShared::wait`]'s Linux futex tier. BENIGN errnos are the EXPECTED
/// wait-loop returns — `EAGAIN` (the word already changed: the kernel's
/// lost-wakeup guard fired), `EINTR` (signal), `ETIMEDOUT` (slice expired) —
/// which re-loop silently and immediately (the `is_open` re-poll is the
/// correctness, and staying hot there is the latency contract). Anything else
/// (a seccomp `ENOSYS`, `EFAULT`, `EINVAL`, …) means the kernel park itself is
/// broken: the caller warns once and degrades to bounded sleep-polling so a
/// persistently-failing syscall can never busy-spin. Pure, so the
/// classification is oracle-testable without a syscall-injection seam.
/// (`EWOULDBLOCK` is the same value as `EAGAIN` on Linux, so it is covered.)
///
/// `pub(crate)` so [`crate::credit`]'s producer park shares
/// the ONE classification. The macOS family below (now extracted into
/// `crate::os_sync`) is shared for a STRONGER reason than tidiness
/// — a second `dlsym` cache and a second `EINVAL`/`ENOTSUP` disable latch
/// would let two words on one host disagree about whether the primitive
/// works.
#[cfg(target_os = "linux")]
pub(crate) fn futex_wait_errno_is_benign(errno: i32) -> bool {
    matches!(errno, libc::EAGAIN | libc::EINTR | libc::ETIMEDOUT)
}

// ===================== macOS os_sync wake tier =====================
// The Apple `os_sync_wait_on_address` / `os_sync_wake_by_address_all` family
// (macOS ≥ 14.4) bound at RUNTIME via `dlsym` so an older macOS degrades to the
// recheck fallback instead of dyld-aborting at launch. All items macOS-gated.
// The dlsym backend + the unusable-latch + the errno
// classifiers + the kill-switch GRAMMAR live in the shared `crate::os_sync` (the
// monitor-wait park's nap tier is the second consumer, and
// `credit.rs` is a third — all three share ONE resolution and ONE
// unusable-latch instead of a dlsym copy each); what stays here is BARRIER
// policy — the operand recipe (`OS_SYNC_WORD_SIZE` + the word aliases), the
// `CERULION_BARRIER_OS_SYNC` kill switch, and `os_sync_active`.
// `OS_SYNC_WORD_SIZE` and `os_sync_active` are `pub(crate)`: `credit.rs`
// reuses this SAME operand recipe and activity gate rather than deriving its
// own.

/// The os_sync watch/wake operand SIZE (bytes) — 4, the low-32
/// `generation` alias. The SINGLE size shared by
/// [`BarrierShared::os_sync_word`] on BOTH the wait and wake sides (mirrors the
/// Linux 32-bit futex word); pairing it with `os_sync_word` in one place is what
/// keeps the wake and wait recipes from diverging.
#[cfg(target_os = "macos")]
pub(crate) const OS_SYNC_WORD_SIZE: usize = 4;

/// The BARRIER's os_sync tier is ACTIVE iff the shared backend
/// resolved AND the family has not been latched off by an unrecoverable errno
/// (`crate::os_sync::os_sync_latched` — one latch for every consumer) AND the
/// `CERULION_BARRIER_OS_SYNC` kill switch is not set. Every gate is a cached
/// atomic load — cheap enough to call per generation-open on the wake side.
#[cfg(target_os = "macos")]
pub(crate) fn os_sync_active() -> bool {
    os_sync_backend().is_some() && !crate::os_sync::os_sync_latched() && !os_sync_kill_switch()
}

/// The `CERULION_BARRIER_OS_SYNC` kill switch, resolved ONCE per
/// process (mirrors [`barrier_spin_budget`]'s cache). `true` = os_sync DISABLED
/// (force the recheck fallback).
#[cfg(target_os = "macos")]
fn os_sync_kill_switch() -> bool {
    static KILL_SWITCH: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *KILL_SWITCH.get_or_init(|| {
        resolve_os_sync_disabled(std::env::var("CERULION_BARRIER_OS_SYNC").ok().as_deref())
    })
}

/// Resolve the raw `CERULION_BARRIER_OS_SYNC` value to "disabled?",
/// emitting the loud garbage warn the resolution requires. Split out of
/// [`os_sync_kill_switch`]'s `OnceLock` closure so the warn is
/// `#[traced_test]`-pinnable without env / `OnceLock` games (mirrors
/// [`resolve_barrier_spin_budget`]). `=0` and every in-range value resolve
/// SILENTLY.
#[cfg(target_os = "macos")]
fn resolve_os_sync_disabled(raw: Option<&str>) -> bool {
    let (disabled, was_garbage) = parse_os_sync_kill_switch(raw);
    if was_garbage {
        tracing::warn!(
            env = "CERULION_BARRIER_OS_SYNC",
            got = %raw.unwrap_or(""),
            "CERULION_BARRIER_OS_SYNC is set but not `0` (disable) or `1`/unset (enable); \
             keeping the macOS os_sync barrier-wake tier ON (`0` is the explicit kill switch)"
        );
    }
    disabled
}

/// Derive the POSIX SHM object name for `(ns, id)`. Pure (no I/O), so it is
/// hermetically testable on every OS. Stable across processes, so the owner
/// ([`MappedBarrier::create_owned`]) and a peer ([`MappedBarrier::open_unowned`])
/// of the same `(ns, id)` derive the same name → map the same page.
///
/// Per-OS shape:
/// - **Linux** — `barrier_shm_name_verbose`: `/cer_bar_<ns>_<fnv1a64(id)>`.
///   The raw `<ns>` in the `/dev/shm` filename is a deliberate debuggability
///   aid (a `ls /dev/shm` names the deployment).
/// - **non-Linux (macOS)** — `barrier_shm_name_compact`:
///   `/cer_bar_<fnv1a64(ns 0x1f id)>` (25 chars). macOS caps POSIX SHM names
///   at 31 chars (`PSHMNAMLEN`, incl. the leading slash — see
///   `shm_ring.rs`, which already lives under the same cap), and real
///   deployment namespaces (`cerdep_<graph>_<nonce>`) blow far past it, so
///   BOTH components are hashed into a fixed-width token.
///
/// Either shape preserves the tenant partition (different `ns` → different
/// name; the compact form separates `ns`/`id` with a 0x1F unit separator so
/// `("ab","c")` and `("a","bc")` can never alias).
fn barrier_shm_name(ns: &str, id: &str) -> String {
    #[cfg(target_os = "linux")]
    {
        barrier_shm_name_verbose(ns, id)
    }
    #[cfg(not(target_os = "linux"))]
    {
        barrier_shm_name_compact(ns, id)
    }
}

/// The Linux name shape: `/cer_bar_<ns>_<fnv1a64(id):016x>` — `ns` verbatim
/// (deployment-visible in `/dev/shm`), `id` hashed. See [`barrier_shm_name`].
#[cfg(any(target_os = "linux", test))]
fn barrier_shm_name_verbose(ns: &str, id: &str) -> String {
    let h = fnv1a64(id.as_bytes());
    format!("/cer_bar_{ns}_{h:016x}")
}

/// The macOS-safe name shape: `/cer_bar_<fnv1a64(ns 0x1f id):016x>`
/// — 25 chars, under the macOS 31-char `PSHMNAMLEN` cap for ANY `(ns, id)`.
/// The 0x1F unit separator keeps the `(ns, id)` split unambiguous. See
/// [`barrier_shm_name`].
#[cfg(any(not(target_os = "linux"), test))]
fn barrier_shm_name_compact(ns: &str, id: &str) -> String {
    // hot-path-alloc-ok: name derivation runs only at create/open (cold path).
    let mut key = Vec::with_capacity(ns.len() + 1 + id.len());
    key.extend_from_slice(ns.as_bytes());
    key.push(0x1f);
    key.extend_from_slice(id.as_bytes());
    let h = fnv1a64(&key);
    format!("/cer_bar_{h:016x}")
}

#[cfg(unix)]
mod imp {
    use super::{barrier_shm_name, BarrierShared};
    use crate::shm_map::{create_exclusive, unlink, unmap, OpenedSegment};
    use std::ffi::CString;
    use std::io;
    use std::os::raw::c_void;

    /// 64 bytes — one cache line, matching the doorbell. Only the leading 16
    /// bytes hold the [`BarrierShared`] atomics; the rest pads to a full line so
    /// the barrier never false-shares with an adjacent object.
    const BARRIER_BYTES: usize = 64;

    /// A process-shared SHM mapping of a [`BarrierShared`] — the cross-process
    /// DAG-level barrier's rendezvous counter living in a `MAP_SHARED` page.
    ///
    /// Construct via [`MappedBarrier::create_owned`] (the supervisor/owner —
    /// `O_EXCL`-creates + sizes + arms the segment, owns the name and
    /// `shm_unlink`s it on drop) or [`MappedBarrier::open_unowned`] (a peer —
    /// STRICT open-existing, maps only). [`Deref`](std::ops::Deref)s to the
    /// [`BarrierShared`], so `arrive`/`is_open`/… reach through into the shared
    /// page.
    #[must_use = "the mapped barrier is unmapped (and, if owned, shm_unlink'd) on drop — bind it to a named local"]
    pub struct MappedBarrier {
        /// Pointer to the mapped [`BarrierShared`] (offset 0 of the `MAP_SHARED`
        /// page).
        ptr: *mut BarrierShared,
        /// The POSIX SHM object name — retained so an OWNED mapping can
        /// `shm_unlink` it on drop.
        name: CString,
        /// `true` for an owner-created mapping that owns the name and must
        /// `shm_unlink` it on drop; `false` for a peer mapping (munmap only).
        owns_name: bool,
    }

    impl MappedBarrier {
        /// The mapped region, as `(base, len)`.
        ///
        /// A capture `fork` child must not inherit the
        /// live cross-process rendezvous word, so the carrier needs the exact bounds.
        pub fn mapping(&self) -> (*mut std::ffi::c_void, usize) {
            (self.ptr as *mut std::ffi::c_void, BARRIER_BYTES)
        }
    }

    // SAFETY: the barrier is a `BarrierShared` (atomics only) in a shared page.
    // All access goes through atomic load/store/fetch ops, so it is sound to
    // send/share the handle across threads (the OS guarantees the MAP_SHARED page
    // is coherent across mappings; the atomics give the intra-process ordering).
    // `name`/`owns_name` are plain Send+Sync data.
    unsafe impl Send for MappedBarrier {}
    unsafe impl Sync for MappedBarrier {}

    impl MappedBarrier {
        /// Create + map + arm the barrier for `(ns, id)` as the OWNER (the
        /// supervisor). `O_EXCL`-creates a FRESH segment, sizes it to one cache
        /// line, arms the count-down to `expected`, and takes ownership of the
        /// name (drop `shm_unlink`s it).
        pub fn create_owned(ns: &str, id: &str, expected: u32) -> io::Result<Self> {
            let name = CString::new(barrier_shm_name(ns, id))
                .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;

            // The whole unlink-first / `O_EXCL` / `ftruncate`-once / `mmap`
            // sequence — and its cleanup on every failure arm — lives in
            // `shm_map::create_exclusive`. What it buys the barrier: the
            // orphan clear (so the create yields a FRESH zero-filled object,
            // generation 0), and the best-effort double-supervisor DETECTOR the
            // `O_EXCL` gives (a racing second supervisor's create returns
            // EEXIST; a NON-racing one still succeeds after clearing the first
            // owner's name, so it is a detector rather than a lock).
            let addr = create_exclusive(&name, BARRIER_BYTES)?;

            let ptr = addr as *mut BarrierShared;
            // SAFETY: `ptr` is a freshly-mapped, exclusively-owned (O_EXCL),
            // page-aligned (mmap returns page alignment, ≥ the 8-byte
            // BarrierShared alignment) valid BarrierShared. The fresh O_EXCL
            // object is zero-filled (generation already 0), so `reinit` just arms
            // `remaining`/`expected` (and re-zeros generation for the
            // orphan-survivor case).
            unsafe {
                (*ptr).reinit(expected);
            }
            // AT BIRTH: a capture `fork` child must not
            // inherit the live cross-process rendezvous word. Never fatal.
            // SAFETY: the region just mapped and exclusively owned by this handle.
            unsafe {
                crate::state_carrier::exclude_at_birth(
                    ptr as *mut std::ffi::c_void,
                    BARRIER_BYTES,
                    crate::state_carrier::ForkExcludedMapping::BarrierPage,
                );
            }
            Ok(Self {
                ptr,
                name,
                owns_name: true,
            })
        }

        /// Open + map an EXISTING barrier for `(ns, id)` as a peer. STRICT
        /// open-existing — `O_RDWR` with NO `O_CREAT`, so a missing object
        /// (ENOENT) is an error: the owner must
        /// [`create_owned`](Self::create_owned) first. (Unlike the doorbell
        /// consumer, which `O_CREAT`s; here the barrier owner is authoritative and
        /// `expected == 0` is the not-ready sentinel for the join.)
        ///
        /// The returned mapping's lifetime is INDEPENDENT of the name's:
        /// after the owner drops (its `Drop` `shm_unlink`s the name), a
        /// FRESH `open_unowned` fails with ENOENT — but THIS handle's mapping, and
        /// any `wait` parked through it, stay valid until this handle's own `Drop`
        /// `munmap`s (POSIX: the object persists until the last unmap). See the
        /// unlink-while-parked contract on `BarrierShared::wait`.
        pub fn open_unowned(ns: &str, id: &str) -> io::Result<Self> {
            let name = CString::new(barrier_shm_name(ns, id))
                .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
            // STRICT open-existing (no `O_CREAT`) — a missing segment is
            // ENOENT → `Err`, never a silent create. No `ftruncate` and NO SIZE
            // CHECK: unlike `state_arm`/`credit`, the barrier reads no size and
            // maps the fixed cache line the owner sized. (Deliberately left as
            // it was — adding a check here would be a behaviour change, not an
            // extraction.)
            let addr = OpenedSegment::open(&name)?.map_shared(BARRIER_BYTES)?;
            // Amendment 10 covers a PEER mapping too — this process forks its own
            // capture children whoever created the segment.
            // SAFETY: the region just mapped and exclusively owned by this handle.
            unsafe {
                crate::state_carrier::exclude_at_birth(
                    addr,
                    BARRIER_BYTES,
                    crate::state_carrier::ForkExcludedMapping::BarrierPage,
                );
            }
            Ok(Self {
                ptr: addr as *mut BarrierShared,
                name,
                owns_name: false,
            })
        }

        /// The mapped [`BarrierShared`]'s virtual address — a test/inspection
        /// accessor (so a test can prove two mappings of the same physical page
        /// have distinct virtual addresses).
        pub fn addr(&self) -> *const BarrierShared {
            self.ptr
        }
    }

    impl std::ops::Deref for MappedBarrier {
        type Target = BarrierShared;
        fn deref(&self) -> &BarrierShared {
            // SAFETY: `ptr` is a valid, page-aligned mapping of a `BarrierShared`
            // that stays mapped for as long as `self` lives (unmapped only in
            // `Drop`).
            unsafe { &*self.ptr }
        }
    }

    /// Teardown = this process's OWN `munmap` + (owner only) `shm_unlink`.
    ///
    /// Unlink-while-parked contract: `shm_unlink` removes only
    /// the NAME; the underlying object persists until the LAST mapping is
    /// unmapped. So an owner dropping mid-run can never invalidate a peer's live
    /// mapping — nor a `BarrierShared::wait` PARKED on it (the monitor/WFE target
    /// stays memory-backed; the waiter returns Opened/TimedOut, never
    /// SIGSEGV/SIGBUS). A SIGBUS would require an `ftruncate` SHRINK of the live
    /// object, which nothing in this codebase performs. Box-pinned by the
    /// `box_parked_waiter_*` tests in `barrier_test.rs`.
    impl Drop for MappedBarrier {
        fn drop(&mut self) {
            // SAFETY: unmap exactly the page create_owned/open_unowned mapped;
            // nothing references it after this.
            unsafe {
                unmap(self.ptr as *mut c_void, BARRIER_BYTES);
            }
            if self.owns_name {
                // Best-effort unlink of the name THIS mapping created and owns.
                // Existing peer mappings keep the object alive until they unmap.
                unlink(&self.name);
            }
        }
    }
}

#[cfg(not(unix))]
mod imp {
    use super::{barrier_shm_name, BarrierShared};
    use std::collections::HashMap;
    use std::io;
    use std::sync::{Arc, Mutex, OnceLock};

    /// Process-global registry mapping SHM name → the shared [`BarrierShared`].
    ///
    /// The non-Unix (future-Windows) stub shares a `BarrierShared` BY NAME through
    /// this registry — NOT a fresh `Arc` per handle — so the behavioral
    /// rendezvous tests are a REAL shared-state test on such a host too (a
    /// per-handle `Arc` would make them a tautology / false-green).
    /// Poison-tolerant (`unwrap_or_else(|e| e.into_inner())`).
    fn registry() -> &'static Mutex<HashMap<String, Arc<BarrierShared>>> {
        static REG: OnceLock<Mutex<HashMap<String, Arc<BarrierShared>>>> = OnceLock::new();
        REG.get_or_init(|| Mutex::new(HashMap::new()))
    }

    /// Non-Unix stub of [`MappedBarrier`] (future Windows target only —
    /// macOS uses the REAL POSIX `shm_open` + `MAP_SHARED` arm above).
    ///
    /// It exists so the code COMPILES off Unix AND the BEHAVIORAL tests
    /// (rendezvous, the different-ns negative control, owner-drop-then-open-
    /// fails) run there. It CANNOT verify the real cross-address-space
    /// `MAP_SHARED` page. State is shared by name via the process-global
    /// registry; `Arc<BarrierShared>` is auto `Send + Sync` (`BarrierShared` is
    /// atomics-only), so no `unsafe impl` is needed.
    #[must_use = "the barrier handle keeps its registry entry alive (and, if owned, removes it on drop) — bind it to a named local"]
    pub struct MappedBarrier {
        shared: Arc<BarrierShared>,
        name: String,
        owns_name: bool,
    }

    impl MappedBarrier {
        /// Owner constructor — inserts a FRESH [`BarrierShared`] into the registry
        /// under `(ns, id)`'s name, REPLACING any orphan (parity with the POSIX
        /// arm's unlink-then-`O_EXCL`).
        pub fn create_owned(ns: &str, id: &str, expected: u32) -> io::Result<Self> {
            let name = barrier_shm_name(ns, id);
            let shared = Arc::new(BarrierShared::new(expected));
            registry()
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .insert(name.clone(), Arc::clone(&shared));
            Ok(Self {
                shared,
                name,
                owns_name: true,
            })
        }

        /// Peer constructor — STRICT lookup of an existing entry (mirrors the
        /// Linux ENOENT): a missing name is `NotFound`, never a silent create.
        pub fn open_unowned(ns: &str, id: &str) -> io::Result<Self> {
            let name = barrier_shm_name(ns, id);
            let shared = registry()
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .get(&name)
                .map(Arc::clone);
            match shared {
                Some(shared) => Ok(Self {
                    shared,
                    name,
                    owns_name: false,
                }),
                None => Err(io::Error::new(
                    io::ErrorKind::NotFound,
                    format!(
                        "barrier segment {name:?} does not exist (owner must create_owned first)"
                    ),
                )),
            }
        }

        /// The shared [`BarrierShared`]'s address — a test/inspection accessor.
        pub fn addr(&self) -> *const BarrierShared {
            Arc::as_ptr(&self.shared)
        }
    }

    impl std::ops::Deref for MappedBarrier {
        type Target = BarrierShared;
        fn deref(&self) -> &BarrierShared {
            &self.shared
        }
    }

    impl Drop for MappedBarrier {
        fn drop(&mut self) {
            if self.owns_name {
                // The "unlink": remove the name ONLY if the stored Arc is still
                // THIS handle's segment (`Arc::ptr_eq`), so a re-created owner's
                // fresh entry isn't clobbered by a stale owner's drop. Open peer
                // handles keep their own Arc clone, so their `Deref` stays valid
                // after the name is removed (mirrors POSIX unlink ≠ unmap).
                let mut reg = registry().lock().unwrap_or_else(|e| e.into_inner());
                let is_mine = reg
                    .get(&self.name)
                    .is_some_and(|stored| Arc::ptr_eq(stored, &self.shared));
                if is_mine {
                    reg.remove(&self.name);
                }
            }
        }
    }
}

pub use imp::MappedBarrier;

#[cfg(test)]
mod tests {
    use super::*;
    // The `resolve_barrier_spin_budget_*` warn pins use
    // `#[traced_test]`. The spin-budget machinery is not OS-gated (the macOS
    // recheck tier now runs the same boundary spin), so the resolver and its
    // warn pins run on every OS.
    use tracing_test::traced_test;

    use crate::testing::logged_at;

    #[test]
    fn name_is_deterministic_and_has_cer_bar_prefix() {
        let n1 = barrier_shm_name("g", "a");
        let n2 = barrier_shm_name("g", "a");
        assert_eq!(n1, n2, "same (ns,id) → identical name");
        // Both per-OS shapes share the /cer_bar_ family prefix (prefix-free vs
        // the doorbell's cer_db_ / the trace ring's cer_rg_).
        assert!(
            n1.starts_with("/cer_bar_"),
            "the name must carry the cer_bar family prefix: {n1}"
        );
    }

    /// The Linux (verbose) shape oracle — `ns` verbatim, `id` FNV-hashed.
    /// Driven through the helper directly so it pins on EVERY OS
    /// (`barrier_shm_name` itself is per-OS).
    #[test]
    fn name_oracle_fixed_fnv_verbose() {
        // FNV-1a-64 oracle (identical algorithm to the doorbell's name helper):
        //   fnv1a64("a")     = 0xaf63dc4c8601ec8c
        //   fnv1a64("b")     = 0xaf63df4c8601f1a5
        //   fnv1a64("topic") = 0x520c8b7d6934ac64
        assert_eq!(
            barrier_shm_name_verbose("g", "a"),
            "/cer_bar_g_af63dc4c8601ec8c"
        );
        assert_eq!(
            barrier_shm_name_verbose("g", "b"),
            "/cer_bar_g_af63df4c8601f1a5"
        );
        assert_eq!(
            barrier_shm_name_verbose("g", "topic"),
            "/cer_bar_g_520c8b7d6934ac64"
        );
    }

    /// The macOS-safe (compact) shape oracle — BOTH components FNV-
    /// hashed over `ns 0x1F id`. Hand-computed oracle values (not derived by
    /// calling `fnv1a64` back — that would be a self-compare).
    #[test]
    fn name_oracle_fixed_fnv_compact() {
        //   fnv1a64("g\x1fa")     = 0xd4c07218fa8dad0e
        //   fnv1a64("g\x1fb")     = 0xd4c07118fa8dab5b
        //   fnv1a64("g\x1ftopic") = 0x5a483ed17ac23d26
        assert_eq!(
            barrier_shm_name_compact("g", "a"),
            "/cer_bar_d4c07218fa8dad0e"
        );
        assert_eq!(
            barrier_shm_name_compact("g", "b"),
            "/cer_bar_d4c07118fa8dab5b"
        );
        assert_eq!(
            barrier_shm_name_compact("g", "topic"),
            "/cer_bar_5a483ed17ac23d26"
        );
    }

    /// The compact shape fits the macOS 31-char `PSHMNAMLEN` cap
    /// (incl. the leading slash) for ANY `(ns, id)` — the fixed shape is
    /// `/cer_bar_` (9) + 16 hex = 25 chars, input-length-independent. A real
    /// multi-process deployment namespace (`cerdep_<graph>_<nonce>`) is the
    /// motivating over-31 input.
    #[test]
    fn name_compact_fits_macos_pshmnamlen_for_any_input() {
        let long_ns = "x".repeat(200);
        let long_id = "y".repeat(200);
        for (ns, id) in [
            ("g", "a"),
            (
                "cerdep_my_long_graph_name_1a2b3c4d",
                "my_long_graph_name_levelgate",
            ),
            ("", ""),
            (long_ns.as_str(), long_id.as_str()),
        ] {
            let n = barrier_shm_name_compact(ns, id);
            assert_eq!(
                n.len(),
                25,
                "compact names are fixed-width 25 chars (≤ 31 PSHMNAMLEN): {n:?}"
            );
        }
    }

    /// The 0x1F unit separator keeps the compact `(ns, id)` split
    /// unambiguous — `("ab","c")` and `("a","bc")` must NOT alias (a plain
    /// concatenation would).
    #[test]
    fn name_compact_separator_prevents_boundary_aliasing() {
        assert_ne!(
            barrier_shm_name_compact("ab", "c"),
            barrier_shm_name_compact("a", "bc"),
            "shifting the ns/id boundary must change the name"
        );
    }

    #[test]
    fn name_different_id_different_name() {
        // The per-OS production shape (whichever this build selects).
        assert_ne!(barrier_shm_name("g", "a"), barrier_shm_name("g", "b"));
        // And both shapes explicitly.
        assert_ne!(
            barrier_shm_name_verbose("g", "a"),
            barrier_shm_name_verbose("g", "b")
        );
        assert_ne!(
            barrier_shm_name_compact("g", "a"),
            barrier_shm_name_compact("g", "b")
        );
    }

    /// The futex errno classification is the load-bearing half of the
    /// unexpected-errno degrade (wrongly classifying `EAGAIN` as unexpected
    /// would put a 100µs sleep on the HOT wake path; wrongly classifying
    /// `ENOSYS` as benign re-opens the silent busy loop). Oracle vector, both
    /// directions.
    #[cfg(target_os = "linux")]
    #[test]
    fn futex_errno_classification_oracle() {
        // Benign: the expected wait-loop returns — silent, immediate re-loop.
        for errno in [libc::EAGAIN, libc::EINTR, libc::ETIMEDOUT] {
            assert!(
                futex_wait_errno_is_benign(errno),
                "errno {errno} is an expected FUTEX_WAIT return and must re-loop hot"
            );
        }
        // Unexpected: the kernel park is broken — warn once + bounded sleep.
        for errno in [libc::ENOSYS, libc::EFAULT, libc::EINVAL, libc::EPERM, 0] {
            assert!(
                !futex_wait_errno_is_benign(errno),
                "errno {errno} must take the warn-once + bounded-sleep degrade"
            );
        }
    }

    /// The `CERULION_BARRIER_SPIN_US` parse contract,
    /// DEFAULT-ON semantics — unset / empty → the caller's tier-resolved
    /// default (quiet); `"0"` → 0 (the explicit kill switch); a plain
    /// non-negative integer → verbatim override; garbage → the default AND
    /// flagged for the loud warn. Since the os_sync tier landed the default is a PARAMETER —
    /// the vector runs under BOTH macOS tier defaults (20 kernel-wake / 150
    /// fallback), proving the default is echoed verbatim whatever the tier
    /// resolves (the same seam pattern as the kill-switch tests: the cached
    /// `barrier_spin_budget` / `os_sync_active` readers are `OnceLock`d, so
    /// only the parameterized helpers make the fallback arm testable). Pure
    /// oracle over `(applied_us, was_garbage)` — no env mutation. The pure
    /// parse is pinned HERE; the loud warn emissions (garbage + over-cap
    /// clamp) by the `#[traced_test]` `resolve_barrier_spin_budget_*` tests;
    /// the tier→default selection itself by `default_spin_two_tier_oracle`.
    #[test]
    fn barrier_spin_us_parse_oracle() {
        // Both tier defaults: 20 (every kernel-wake tier) and 150 (the macOS
        // sleep-recheck fallback). LITERAL values, not consts echoed back.
        for default_us in [20u64, 150] {
            for (raw, want, garbage) in [
                (None, default_us, false),
                (Some(""), default_us, false),
                (Some("0"), 0, false),
                (Some("garbage"), default_us, true),
                (Some("abc"), default_us, true),
                (Some("-5"), default_us, true),
                (Some("2.5"), default_us, true),
                (Some("20"), 20, false),
                (Some("50"), 50, false),
                (Some("1000"), 1000, false),
            ] {
                assert_eq!(
                    parse_barrier_spin_us(raw, default_us),
                    (want, garbage),
                    "parse_barrier_spin_us({raw:?}, {default_us}) must be ({want}, {garbage})"
                );
            }
        }
        // The shared kernel-wake knee is 20µs on EVERY OS (the os_sync tier collapsed
        // the per-OS cfg split — the macOS 150 now lives ONLY on the fallback
        // tier). LITERAL pins, not the const echoed back.
        assert_eq!(DEFAULT_BARRIER_SPIN_US, 20);
        #[cfg(target_os = "macos")]
        assert_eq!(MACOS_FALLBACK_SPIN_US, 150);
    }

    /// The macOS two-tier default oracle — the pure
    /// `resolve_default_barrier_spin_us` seam (fed the LIVE `os_sync_active`
    /// by `default_barrier_spin_us` in production) resolves the os_sync
    /// kernel-wake arm to the 20µs knee and the sleep-recheck fallback arm to
    /// the 150µs skew cover. LITERAL pins on BOTH arms — the seam is exactly
    /// what makes the fallback arm testable on an os_sync-active box (the
    /// cached `os_sync_active`/`OnceLock` path can never take it here).
    #[cfg(target_os = "macos")]
    #[test]
    fn default_spin_two_tier_oracle() {
        assert_eq!(
            resolve_default_barrier_spin_us(true),
            20,
            "os_sync-active tier must resolve the shared 20µs kernel-wake knee"
        );
        assert_eq!(
            resolve_default_barrier_spin_us(false),
            150,
            "the sleep-recheck fallback tier must resolve the 150µs skew cover"
        );
    }

    /// The tier-resolution debug BREADCRUMB — each arm of
    /// `resolve_default_barrier_spin_us` names its tier + default (in
    /// production this fires exactly once, from `barrier_spin_budget`'s
    /// `OnceLock` init). `#[traced_test]` captures both arms' emissions.
    #[cfg(target_os = "macos")]
    #[traced_test]
    #[test]
    fn default_spin_resolution_emits_tier_breadcrumbs() {
        // The breadcrumbs ride `debug!`, which `release_max_level_info` compiles
        // OUT when `debug_assertions` is off — so under `--release` a count of
        // them reads 0 no matter what the resolver did. Routing the expectation
        // through `debug_lines_expected` keeps the debug pin at full strength
        // (it still demands EXACTLY one of each line) and stops a release run
        // reporting a log-level fact as a resolver defect. That helper is the
        // one the DEBUG-count discipline walk requires
        // (`tests/debug_count_discipline_test.rs`): tracing's own static gate,
        // the exact predicate `debug!` applies.
        //
        // NOT the main-red class this branch fixes, and not reachable by it: this
        // test is `#[cfg(target_os = "macos")]`, so the Linux
        // `Latency Threshold` job never compiles it. Found while running that
        // job's exact command locally, and fixed here rather than left to
        // ambush the next person who runs a release suite on a Mac.
        //
        // The RESOLVER itself is pinned level-independently by
        // `default_spin_two_tier_oracle` next door, which asserts the RETURNED
        // budgets (20µs / 150µs) rather than the log — so nothing about the
        // tiering goes unpinned in release.
        let _ = resolve_default_barrier_spin_us(true);
        let _ = resolve_default_barrier_spin_us(false);
        // Level-free twin: a tier breadcrumb must never be LOUD — the half of the contract
        // that survives `release_max_level_info`, where the gated counts below read 0.
        logs_assert(|lines: &[&str]| {
            for level in ["WARN", "INFO", "ERROR"] {
                if lines.iter().any(|l| {
                    // Fully qualified, like its two neighbours below: this fn is
                    // `#[cfg(target_os = "macos")]`, so a module-level `use` for
                    // a helper ONLY used here is an unused import on every other
                    // OS — and `unused_imports` is `deny`, so the
                    // `use` stays local to this arm.
                    crate::testing::line_level(l) == Some(level)
                        && (l.contains("os_sync kernel-wake tier")
                            || l.contains("sleep-recheck fallback tier"))
                }) {
                    return Err(format!("a tier breadcrumb was emitted at {level}"));
                }
            }
            // Each breadcrumb counted AT DEBUG, not by text: one re-emitted at
            // `trace!` still satisfies a `logs_contain`, and the sweep above
            // permits TRACE. The same call pairs the level-free total, so a
            // duplicate at another level cannot pass either.
            //
            // This is EXACTLY-one, not presence: `resolve_default_barrier_spin_us`
            // emits one `debug!` per call and is called once per arm above, so a
            // SECOND breadcrumb from either arm is a real regression — but a
            // future reader should know the count is deliberately exact rather
            // than read a failure here as a flake.
            for (marker, arm) in [
                ("os_sync kernel-wake tier", "the os_sync arm"),
                ("sleep-recheck fallback tier", "the fallback arm"),
            ] {
                let n = crate::testing::count_at_exclusively(lines, "DEBUG", &[marker])?;
                if n != crate::testing::debug_lines_expected(1) {
                    return Err(format!(
                        "{arm} must breadcrumb its tier at DEBUG exactly once, got {n}"
                    ));
                }
            }
            Ok(())
        });
    }

    /// The time-bounded spin OPENS when another thread bumps the
    /// generation mid-spin — the default phase-1 happy path, driven directly
    /// through `spin_for_budget` (no env tricks; `barrier_spin_budget`'s
    /// `OnceLock` caches process-wide, so the helper takes the deadline as a
    /// parameter precisely to be testable). A single arrive on an
    /// `expected == 1` barrier is the unique opener (generation 0 → 1).
    #[test]
    fn spin_for_budget_opens_on_mid_spin_bump() {
        // Fully-qualified std::sync::Arc / std::thread: a mod-level `use`
        // would be unused on non-Linux (these tests are cfg-gated) and trip
        // deny(unused_imports).
        let b = std::sync::Arc::new(BarrierShared::new(1));
        let opener = {
            let b = std::sync::Arc::clone(&b);
            std::thread::spawn(move || {
                std::thread::sleep(Duration::from_millis(2));
                assert_eq!(
                    b.arrive(0),
                    ArriveOutcome::Opened(1),
                    "the lone participant is the unique opener"
                );
            })
        };
        // Generous spin deadline (500ms) so the 2ms bump lands well inside it.
        let opened = b.spin_for_budget(0, Instant::now() + Duration::from_millis(500));
        opener.join().expect("opener panicked");
        assert!(
            opened,
            "a generation bump landing mid-spin must open the spin (true)"
        );
    }

    /// The time-bounded spin RETURNS `false` (not-opened) once the
    /// budget deadline passes on a SILENT barrier — proving the spin is
    /// hard-bounded (the caller then falls through to the unchanged futex
    /// block; an unbounded spin would hang this test forever).
    #[test]
    fn spin_for_budget_times_out_on_silent_barrier() {
        let b = BarrierShared::new(2); // nobody else ever arrives
        let start = Instant::now();
        let opened = b.spin_for_budget(0, start + Duration::from_millis(2));
        assert!(
            !opened,
            "a silent barrier must time the spin out (false) at the budget deadline"
        );
        // Bounded: it returned promptly after the 2ms budget, not seconds later.
        assert!(
            start.elapsed() < Duration::from_millis(500),
            "the spin must return promptly after its deadline — took {:?}",
            start.elapsed()
        );
    }

    /// A GARBAGE `CERULION_BARRIER_SPIN_US` resolves to
    /// the passed tier default AND emits the loud garbage warn. Drives the
    /// extracted [`resolve_barrier_spin_budget`] helper directly (no env /
    /// `OnceLock` games), so the warn EMISSION — not merely the `was_garbage`
    /// flag — is the pinned contract. `#[traced_test]` captures the tracing
    /// events. The default is threaded as a parameter — pinned here
    /// with the 20µs kernel-wake knee.
    #[traced_test]
    #[test]
    fn resolve_barrier_spin_budget_garbage_defaults_and_warns() {
        let got = resolve_barrier_spin_budget(Some("garbage"), DEFAULT_BARRIER_SPIN_US);
        assert_eq!(
            got,
            Duration::from_micros(20),
            "garbage must fall back to the passed (20µs kernel-wake) default budget"
        );
        logs_assert(|lines: &[&str]| {
            logged_at(lines, "WARN", "not a non-negative integer")
                .map_err(|e| format!("the garbage value must emit the loud fallback warn — {e}"))
        });
    }

    /// An OVER-CAP `CERULION_BARRIER_SPIN_US`
    /// (> 100ms) is clamped to [`MAX_BARRIER_SPIN_US`] AND emits the loud clamp
    /// warn — proving the guard-rail that keeps a fat-fingered budget from
    /// busy-spinning the ~5s boundary window. Driven through the helper directly.
    #[traced_test]
    #[test]
    fn resolve_barrier_spin_budget_over_cap_clamps_and_warns() {
        // 5_000_000µs = 5s — the pathological value the clamp exists to catch.
        let got = resolve_barrier_spin_budget(Some("5000000"), DEFAULT_BARRIER_SPIN_US);
        assert_eq!(
            got,
            Duration::from_micros(MAX_BARRIER_SPIN_US),
            "an over-cap budget must clamp to the 100ms hard cap"
        );
        assert_eq!(MAX_BARRIER_SPIN_US, 100_000, "the cap is the fixed 100ms");
        logs_assert(|lines: &[&str]| {
            logged_at(lines, "WARN", "exceeds the 100ms hard cap")
                .map_err(|e| format!("clamping must emit the loud clamp warn — {e}"))
        });
    }

    /// The `=0` kill switch resolves to a ZERO budget
    /// SILENTLY — no warn (it is an explicit, supported choice, not a mistake).
    #[traced_test]
    #[test]
    fn resolve_barrier_spin_budget_zero_is_silent() {
        let got = resolve_barrier_spin_budget(Some("0"), DEFAULT_BARRIER_SPIN_US);
        assert_eq!(got, Duration::ZERO, "`=0` is the explicit kill switch");
        assert!(
            !logs_contain("CERULION_BARRIER_SPIN_US"),
            "the kill switch is a supported choice and must resolve silently"
        );
    }

    /// An UNSET (`None`) value resolves to the passed
    /// tier default SILENTLY — the default-on happy path emits no warn.
    /// BOTH tier defaults driven through (the old per-OS literal pin
    /// moved to `default_spin_two_tier_oracle`, which pins the tier→default
    /// selection itself).
    #[traced_test]
    #[test]
    fn resolve_barrier_spin_budget_unset_is_default_and_silent() {
        let got = resolve_barrier_spin_budget(None, DEFAULT_BARRIER_SPIN_US);
        // LITERAL pin: the shared kernel-wake knee is 20µs on every OS.
        assert_eq!(
            got,
            Duration::from_micros(20),
            "unset must apply the kernel-wake-tier default budget"
        );
        // And the macOS fallback-tier default threads through verbatim.
        #[cfg(target_os = "macos")]
        assert_eq!(
            resolve_barrier_spin_budget(None, MACOS_FALLBACK_SPIN_US),
            Duration::from_micros(150),
            "unset must apply the fallback-tier default budget when that tier resolves"
        );
        assert!(
            !logs_contain("CERULION_BARRIER_SPIN_US"),
            "the default-on happy path must resolve silently"
        );
    }

    /// The `=0` KILL-SWITCH arm of `wait_futex` (the
    /// zero-budget 50-iteration pre-block spin, then the unchanged `FUTEX_WAIT`
    /// block) still OPENS correctly — exercised hermetically through the
    /// budget-parameterized [`BarrierShared::wait_futex_with_budget`] (mirrors
    /// `spin_for_budget_opens_on_mid_spin_bump`'s shape). A lone participant on
    /// an `expected == 1` barrier is the unique opener (generation 0 → 1); its
    /// `arrive` `FUTEX_WAKE`s the parked waiter, which re-polls `is_open` and
    /// returns [`WaitOutcome::Opened`]. Guards the legacy else path
    /// the default (non-zero budget) never takes.
    #[cfg(target_os = "linux")]
    #[test]
    fn wait_futex_zero_budget_kill_switch_still_opens() {
        let b = std::sync::Arc::new(BarrierShared::new(1));
        let opener = {
            let b = std::sync::Arc::clone(&b);
            std::thread::spawn(move || {
                std::thread::sleep(Duration::from_millis(2));
                assert_eq!(
                    b.arrive(0),
                    ArriveOutcome::Opened(1),
                    "the lone participant is the unique opener"
                );
            })
        };
        // ZERO budget selects the earlier 50-iter spin arm; a generous 5s
        // deadline lets the futex block ride to the opener's 2ms wake.
        let outcome =
            b.wait_futex_with_budget(0, Instant::now() + Duration::from_secs(5), Duration::ZERO);
        opener.join().expect("opener panicked");
        assert_eq!(
            outcome,
            WaitOutcome::Opened,
            "the `=0` kill-switch wait must still open on the opener's futex wake"
        );
    }

    #[test]
    fn name_different_ns_different_name() {
        // The tenant partition at the name level: same id, different tenant → no
        // collision.
        assert_ne!(barrier_shm_name("g1", "a"), barrier_shm_name("g2", "a"));
    }

    #[test]
    fn layout_is_24_bytes_8_aligned_offsets_pinned() {
        // Runtime restatement of the compile-time const-asserts above (the
        // wake-word layout bump: 16 → 24 with the wake_seq/parked words; offsets pinned so
        // the kernel wait/wake address recipes are locked).
        assert_eq!(core::mem::size_of::<BarrierShared>(), 24);
        assert!(core::mem::align_of::<BarrierShared>() >= 8);
        assert_eq!(core::mem::offset_of!(BarrierShared, wake_seq), 16);
        assert_eq!(core::mem::offset_of!(BarrierShared, parked), 20);
    }

    // ================= macOS os_sync wake tier tests =================
    // All macOS-gated (the primitive + its helpers are `#[cfg(target_os =
    // "macos")]`). On Apple Silicon (macOS ≥ 14.4) the `wait_os_sync`
    // arm exercises the REAL Apple primitive.

    // The pure kill-switch-GRAMMAR + errno-classifier oracles moved
    // to `crate::os_sync`'s in-module tests WITH the fns they pin (the barrier
    // and the monitor-wait nap now share both). The resolver-warn pins below
    // stay: `resolve_os_sync_disabled` (the CERULION_BARRIER_OS_SYNC wrapper)
    // is barrier policy and lives here.

    /// `=0` resolves to DISABLED (true) SILENTLY — an explicit,
    /// supported choice, not a mistake. Drives the extracted resolver directly.
    #[cfg(target_os = "macos")]
    #[traced_test]
    #[test]
    fn resolve_os_sync_disabled_zero_disables_silently() {
        assert!(
            resolve_os_sync_disabled(Some("0")),
            "`=0` must disable the os_sync tier"
        );
        assert!(
            !logs_contain("CERULION_BARRIER_OS_SYNC"),
            "the kill switch is a supported choice and must resolve silently"
        );
    }

    /// Garbage keeps the tier ON (false) AND emits the loud warn (house
    /// rule: loud over silent). Drives the extracted resolver directly (no env /
    /// `OnceLock`), so the warn EMISSION — not just the `was_garbage` flag — is
    /// the pinned contract. Mirrors `resolve_barrier_spin_budget_garbage_*`.
    #[cfg(target_os = "macos")]
    #[traced_test]
    #[test]
    fn resolve_os_sync_disabled_garbage_keeps_on_and_warns() {
        assert!(
            !resolve_os_sync_disabled(Some("nope")),
            "a garbage value must keep os_sync ON (the default)"
        );
        logs_assert(|lines: &[&str]| {
            logged_at(
                lines,
                "WARN",
                "keeping the macOS os_sync barrier-wake tier ON",
            )
            .map_err(|e| format!("a garbage value must emit the loud keep-on warn — {e}"))
        });
    }

    /// Unset (`None`) and the explicit `=1` both resolve to ENABLED
    /// (false) SILENTLY — the default-on happy path emits no warn.
    #[cfg(target_os = "macos")]
    #[traced_test]
    #[test]
    fn resolve_os_sync_disabled_unset_and_one_are_enabled_silent() {
        assert!(!resolve_os_sync_disabled(None), "unset keeps os_sync ON");
        assert!(
            !resolve_os_sync_disabled(Some("1")),
            "`=1` keeps os_sync ON"
        );
        assert!(
            !logs_contain("CERULION_BARRIER_OS_SYNC"),
            "the default-on happy path must resolve silently"
        );
    }

    /// Print-only backend probe — eprintln whether the os_sync symbols
    /// resolved on THIS host + the selected wake tier, so CI logs show whether
    /// the real primitive ran. Never asserts a value (host-dependent: macOS ≥
    /// 14.4 resolves, < 14.4 does not).
    #[cfg(target_os = "macos")]
    #[test]
    fn os_sync_backend_probe_prints_availability() {
        let present = os_sync_backend().is_some();
        eprintln!("os_sync backend available on this host: {present}");
        eprintln!("barrier wake tier: {}", barrier_wake_tier());
    }

    /// `wait_os_sync` (the REAL macOS ≥ 14.4 primitive on this machine)
    /// OPENS when another thread bumps the generation mid-wait — the headline
    /// happy path, driven directly through the private method (mirrors
    /// `wait_futex_zero_budget_kill_switch_still_opens`'s shape). A lone
    /// participant on an `expected == 1` barrier is the unique opener (gen 0→1);
    /// its `arrive` `os_sync_wake`s the waiter, which re-polls `is_open` and
    /// returns Opened. On an older macOS the method degrades to the fallback
    /// internally, so this still asserts Opened on ANY macOS.
    #[cfg(target_os = "macos")]
    #[test]
    fn wait_os_sync_opens_on_mid_wait_wake() {
        let b = std::sync::Arc::new(BarrierShared::new(1));
        let opener = {
            let b = std::sync::Arc::clone(&b);
            std::thread::spawn(move || {
                std::thread::sleep(Duration::from_millis(5));
                assert_eq!(
                    b.arrive(0),
                    ArriveOutcome::Opened(1),
                    "the lone participant is the unique opener"
                );
            })
        };
        let outcome = b.wait_os_sync(0, Instant::now() + Duration::from_secs(5));
        opener.join().expect("opener panicked");
        assert_eq!(
            outcome,
            WaitOutcome::Opened,
            "wait_os_sync must open on the opener's kernel wake"
        );
    }

    /// The recheck FALLBACK rung (what the kill switch / an older macOS
    /// selects) ALSO opens on a mid-wait bump — proving the degraded path works
    /// independent of os_sync. Driven directly through the private
    /// `wait_recheck_fallback`.
    #[cfg(target_os = "macos")]
    #[test]
    fn wait_recheck_fallback_opens_on_mid_wait() {
        let b = std::sync::Arc::new(BarrierShared::new(1));
        let opener = {
            let b = std::sync::Arc::clone(&b);
            std::thread::spawn(move || {
                std::thread::sleep(Duration::from_millis(5));
                assert_eq!(
                    b.arrive(0),
                    ArriveOutcome::Opened(1),
                    "the lone participant is the unique opener"
                );
            })
        };
        let outcome = b.wait_recheck_fallback(0, Instant::now() + Duration::from_secs(5));
        opener.join().expect("opener panicked");
        assert_eq!(
            outcome,
            WaitOutcome::Opened,
            "the recheck fallback must open on the mid-wait bump"
        );
    }

    // ============ step-start park wake-word tests ============
    // Hermetic (no SHM/iceoryx2); in the lib `mod tests` because they drive the
    // private/seam surface (`note_activity` sites via public ops, `reinit`,
    // `park_wait_activity`). Here the kernel block is the REAL primitive
    // (os_sync on macOS ≥ 14.4, futex on Linux) — the same code both tiers.

    /// Exact epoch-bump oracle per state-transition site —
    /// hand-counted deltas, NOT a self-compare. Pending arrive = 1 (the
    /// `note_activity` at the fetch_sub); opener arrive = 2 (note + the
    /// `wake_waiters` open bump); vacuous CAS open = 1 (wake_waiters only — no
    /// fetch_sub on that path); drop applied-pending = 1; drop-opener = 2;
    /// `set_expected` = 1. A regression that drops any site's bump (silencing
    /// that transition for parkers) fails the exact arithmetic here.
    #[test]
    fn wake_seq_bump_site_oracle() {
        // Pending arrive: +1.
        let b = BarrierShared::new(2);
        assert_eq!(b.wake_seq_snapshot(), 0, "fresh barrier starts at epoch 0");
        assert_eq!(b.arrive(0), ArriveOutcome::Pending);
        assert_eq!(
            b.wake_seq_snapshot(),
            1,
            "a pending arrival bumps exactly once"
        );
        // Opener arrive: +2 (note_activity + wake_waiters).
        assert_eq!(b.arrive(0), ArriveOutcome::Opened(1));
        assert_eq!(
            b.wake_seq_snapshot(),
            3,
            "the opener bumps at fetch_sub AND at open"
        );

        // Vacuous CAS open: +1 (wake_waiters only).
        let v = BarrierShared::new(0);
        assert_eq!(v.arrive(0), ArriveOutcome::Opened(1));
        assert_eq!(
            v.wake_seq_snapshot(),
            1,
            "the vacuous CAS open bumps exactly once"
        );

        // Drop applied-pending: +1.
        let d = BarrierShared::new(3);
        assert_eq!(
            d.try_drop_participant(0),
            DropAttempt::Applied(ArriveOutcome::Pending)
        );
        assert_eq!(
            d.wake_seq_snapshot(),
            1,
            "an incomplete drop bumps exactly once"
        );

        // Drop-opener: +2.
        let o = BarrierShared::new(1);
        assert_eq!(
            o.try_drop_participant(0),
            DropAttempt::Applied(ArriveOutcome::Opened(1))
        );
        assert_eq!(
            o.wake_seq_snapshot(),
            2,
            "an opening drop bumps at fetch_sub AND at open"
        );

        // set_expected: +1 (boundary retarget).
        let s = BarrierShared::new(2);
        s.set_expected(4);
        assert_eq!(
            s.wake_seq_snapshot(),
            1,
            "a cohort retarget bumps exactly once"
        );
    }

    /// The parked rank-bitmask oracle — set/clear per rank,
    /// guard RAII clears on drop, the supervisor sweep clears exactly the dead
    /// rank, and a beyond-mask rank (≥ 32) sets NO bit (returns `false`, warns
    /// once — the documented degrade for deployments beyond 32 barrier ranks).
    #[traced_test]
    #[test]
    fn parked_bitmask_oracle() {
        let b = BarrierShared::new(2);
        assert_eq!(b.parked_mask(), 0);
        assert!(b.park_enter(0));
        assert!(b.park_enter(5));
        assert_eq!(b.parked_mask(), 0b10_0001, "bits 0 and 5 set");
        b.park_exit(0);
        assert_eq!(
            b.parked_mask(),
            0b10_0000,
            "exit clears exactly its own bit"
        );
        // Supervisor sweep: clears exactly the dead rank; idempotent; unset-bit
        // and beyond-mask ranks are no-ops.
        b.clear_parked_rank(5);
        assert_eq!(b.parked_mask(), 0, "sweep clears the dead rank's bit");
        b.clear_parked_rank(5);
        b.clear_parked_rank(31);
        b.clear_parked_rank(40);
        assert_eq!(
            b.parked_mask(),
            0,
            "sweeps of unset/beyond-mask ranks are no-ops"
        );
        // Beyond-mask enter: no bit, false, one loud warn.
        assert!(!b.park_enter(32), "rank 32 exceeds the bitmask — no bit");
        assert!(!b.park_enter(63));
        assert_eq!(b.parked_mask(), 0, "beyond-mask ranks never touch the mask");
        logs_assert(|lines: &[&str]| {
            logged_at(lines, "WARN", "exceeds the parked bitmask")
                .map_err(|e| format!("the beyond-mask degrade must warn loudly (once) — {e}"))
        });
        // RAII guard: bit held for the guard's life, cleared on drop; None
        // beyond the mask.
        {
            let g = ParkedRankGuard::enter(&b, 7).expect("rank 7 is in-mask");
            assert_eq!(b.parked_mask(), 1 << 7);
            drop(g);
        }
        assert_eq!(b.parked_mask(), 0, "guard drop cleared the bit");
        assert!(
            ParkedRankGuard::enter(&b, 40).is_none(),
            "beyond-mask rank yields no guard (caller keeps timeout cadence)"
        );
    }

    /// The headline — a parker kernel-blocked on the wake word
    /// is WOKEN by a peer's ARRIVAL (a pure `Pending` arrival on a barrier(2) —
    /// no open, no generation bump; exactly the step-start signal). The
    /// REAL primitive on this machine (os_sync / futex). Oracle: the block returns
    /// far inside its 5s cap, and the parker's re-derive sees
    /// `peers_waiting == true`.
    ///
    /// The arrival is ORDERED against the parker by an explicit handshake,
    /// never by a sleep's timing: the parker publishes "about to block"
    /// only after it has taken its wake-word snapshot and read the
    /// precondition, so neither can be overtaken on a preempted machine.
    /// The short pause that follows the handshake lets the armed parker
    /// reach the kernel call; it carries no ordering, because an arrival
    /// landing in the window between the signal and the block is exactly
    /// what the snapshot exists for (the compare returns the block at once,
    /// and all three oracles still hold).
    #[test]
    fn parked_waiter_on_wake_word_wakes_on_arrival() {
        if !wake_word_block_primitive_available() {
            eprintln!("no park wake primitive on this host — skipping (fallback tier)");
            return;
        }
        let b = std::sync::Arc::new(BarrierShared::new(2));
        let (armed_tx, armed_rx) = std::sync::mpsc::channel::<()>();
        let parker = {
            let b = std::sync::Arc::clone(&b);
            std::thread::spawn(move || {
                let _g = ParkedRankGuard::enter(&b, 0).expect("rank 0 in-mask");
                let snap = b.wake_seq_snapshot();
                assert!(!b.peers_waiting(), "nobody has arrived yet");
                armed_tx
                    .send(())
                    .expect("the arriver must still be waiting");
                let start = Instant::now();
                let performed = b.park_wait_activity(snap, Duration::from_secs(5));
                (performed, start.elapsed(), b.peers_waiting())
            })
        };
        if armed_rx.recv_timeout(Duration::from_secs(5)).is_err() {
            // The sender is gone: the parker panicked before arming. Surface
            // ITS message (the precondition) rather than a timeout.
            match parker.join() {
                Err(payload) => std::panic::resume_unwind(payload),
                Ok(_) => panic!("the parker must arm within 5s"),
            }
        }
        // ORDERING is the handshake's job; this pause only lets the armed
        // parker reach the kernel call, which is the state the wake path
        // exists to serve. It carries no correctness: an arrival that lands
        // first is caught by the snapshot compare and every oracle below
        // still holds, so a machine that overruns it loses sharpness, not a
        // run. Measured: without it the suppressed-wake check passes about
        // half the time.
        std::thread::sleep(Duration::from_millis(5));
        assert_eq!(
            b.arrive(0),
            ArriveOutcome::Pending,
            "one arrival on a barrier(2)"
        );
        let (performed, elapsed, saw_peers) = parker.join().expect("parker panicked");
        assert!(performed, "the kernel block must report performed");
        assert!(
            elapsed < Duration::from_secs(2),
            "the arrival must wake the parker far inside the 5s cap — took {elapsed:?}"
        );
        assert!(
            saw_peers,
            "the woken parker's re-derive must see peers_waiting"
        );
    }

    /// A SPURIOUS bump (a state transition that does NOT make
    /// `peers_waiting` true — here a boundary `set_expected` retarget) wakes the
    /// parker as a bounded no-op: the block returns promptly and the re-derive
    /// correctly reads `false` (the caller re-parks). Spurious-tolerance is the
    /// contract that lets the wake side over-signal freely.
    ///
    /// Ordered by the same handshake as the arrival twin above, and for a
    /// sharper reason: a bump that lands before the parker's snapshot is
    /// already IN that snapshot, so the block would have nothing to compare
    /// against and would ride its full cap. The signal is sent after the
    /// snapshot, so the bump can never get ahead of it.
    #[test]
    fn spurious_bump_wakes_parker_who_rederives_false() {
        if !wake_word_block_primitive_available() {
            eprintln!("no park wake primitive on this host — skipping (fallback tier)");
            return;
        }
        let b = std::sync::Arc::new(BarrierShared::new(2));
        let (armed_tx, armed_rx) = std::sync::mpsc::channel::<()>();
        let parker = {
            let b = std::sync::Arc::clone(&b);
            std::thread::spawn(move || {
                let _g = ParkedRankGuard::enter(&b, 1).expect("rank 1 in-mask");
                let snap = b.wake_seq_snapshot();
                armed_tx.send(()).expect("the bumper must still be waiting");
                let start = Instant::now();
                let performed = b.park_wait_activity(snap, Duration::from_secs(5));
                (performed, start.elapsed(), b.peers_waiting())
            })
        };
        if armed_rx.recv_timeout(Duration::from_secs(5)).is_err() {
            // The sender is gone: the parker panicked before arming. Surface
            // ITS message rather than a timeout.
            match parker.join() {
                Err(payload) => std::panic::resume_unwind(payload),
                Ok(_) => panic!("the parker must arm within 5s"),
            }
        }
        // ORDERING is the handshake's job; this pause only lets the armed
        // parker reach the kernel call, which is the state the wake path
        // exists to serve. It carries no correctness: an arrival that lands
        // first is caught by the snapshot compare and every oracle below
        // still holds, so a machine that overruns it loses sharpness, not a
        // run. Measured: without it the suppressed-wake check passes about
        // half the time.
        std::thread::sleep(Duration::from_millis(5));
        b.set_expected(2); // a bump with remaining == expected → predicate stays false
        let (performed, elapsed, saw_peers) = parker.join().expect("parker panicked");
        assert!(performed);
        assert!(
            elapsed < Duration::from_secs(2),
            "the spurious bump must still wake the block promptly — took {elapsed:?}"
        );
        assert!(
            !saw_peers,
            "the re-derive after a spurious wake must read false (bounded no-op re-park)"
        );
    }

    /// The SB-litmus hammer — parker and arriver race with NO
    /// bias sleeps, 100 fresh rounds. The bounded-latency property (NOT a tight
    /// number): every round's park returns strictly inside its 1s cap, because
    /// one of the three guards always catches the arrival — the post-flag
    /// predicate re-derive, the kernel compare against the pre-block snapshot,
    /// or the parked-gated wake syscall. A lost wake (the SB both-see-stale
    /// outcome the SeqCst fences forbid) would ride to the full cap and fail
    /// the per-round bound.
    #[test]
    fn sb_hammer_every_round_bounded() {
        const ROUNDS: usize = 100;
        const CAP: Duration = Duration::from_secs(1);
        for round in 0..ROUNDS {
            let b = std::sync::Arc::new(BarrierShared::new(2));
            let parker = {
                let b = std::sync::Arc::clone(&b);
                std::thread::spawn(move || {
                    let _g = ParkedRankGuard::enter(&b, 0).expect("rank 0 in-mask");
                    let snap = b.wake_seq_snapshot();
                    let start = Instant::now();
                    if !b.peers_waiting() {
                        b.park_wait_activity(snap, CAP);
                    }
                    start.elapsed()
                })
            };
            let arriver = {
                let b = std::sync::Arc::clone(&b);
                std::thread::spawn(move || {
                    assert_eq!(b.arrive(0), ArriveOutcome::Pending);
                })
            };
            arriver.join().expect("arriver panicked");
            let elapsed = parker.join().expect("parker panicked");
            assert!(
                elapsed < Duration::from_millis(900),
                "round {round}: park must return inside the cap (no lost wake) — took {elapsed:?}"
            );
        }
    }

    /// Orphan reuse: `reinit` resets the wake word AND the parked
    /// mask — a crashed prior run's stale epoch is harmless but a stale parked
    /// bit would tax every arriver with a no-op syscall forever; the
    /// orphan-reuse reset kills both.
    #[test]
    fn reinit_resets_wake_seq_and_parked() {
        let b = BarrierShared::new(2);
        assert_eq!(b.arrive(0), ArriveOutcome::Pending);
        assert!(b.park_enter(3));
        assert!(b.wake_seq_snapshot() > 0);
        assert_ne!(b.parked_mask(), 0);
        b.reinit(5);
        assert_eq!(b.wake_seq_snapshot(), 0, "reinit re-zeros the epoch");
        assert_eq!(b.parked_mask(), 0, "reinit sweeps the whole parked mask");
        assert_eq!(b.expected(), 5);
        assert_eq!(b.current_generation(), 0);
    }

    /// Print-only park-wake latency A/B — the wake-word kernel
    /// block vs today's 100µs sleep-recheck poll, arrival-to-observation, on
    /// THIS machine's real primitive. Asserts only that every round observed the
    /// arrival (numbers are machine-dependent; the printed p50/p99 are the
    /// before/after evidence).
    #[test]
    fn park_wake_ab_print_only() {
        const ROUNDS: usize = 40;
        fn pct(sorted: &[Duration], p: f64) -> Duration {
            let idx = (((sorted.len() as f64) * p).ceil() as usize).clamp(1, sorted.len()) - 1;
            sorted[idx]
        }
        // Leg A: kernel block on the wake word.
        let mut kernel = Vec::with_capacity(ROUNDS);
        if wake_word_block_primitive_available() {
            for _ in 0..ROUNDS {
                let b = std::sync::Arc::new(BarrierShared::new(2));
                let parker = {
                    let b = std::sync::Arc::clone(&b);
                    std::thread::spawn(move || {
                        let _g = ParkedRankGuard::enter(&b, 0).expect("in-mask");
                        let snap = b.wake_seq_snapshot();
                        if !b.peers_waiting() {
                            b.park_wait_activity(snap, Duration::from_secs(5));
                        }
                        (Instant::now(), b.peers_waiting())
                    })
                };
                std::thread::sleep(Duration::from_millis(2));
                let t_wake = Instant::now();
                assert_eq!(b.arrive(0), ArriveOutcome::Pending);
                let (t_ret, observed) = parker.join().expect("parker panicked");
                assert!(observed, "every kernel-leg round must observe the arrival");
                kernel.push(t_ret.saturating_duration_since(t_wake));
            }
        }
        // Leg B: today's cadence — 100µs sleep-recheck poll on peers_waiting.
        let mut poll = Vec::with_capacity(ROUNDS);
        for _ in 0..ROUNDS {
            let b = std::sync::Arc::new(BarrierShared::new(2));
            let parker = {
                let b = std::sync::Arc::clone(&b);
                std::thread::spawn(move || {
                    let deadline = Instant::now() + Duration::from_secs(5);
                    while !b.peers_waiting() && Instant::now() < deadline {
                        std::thread::sleep(Duration::from_micros(100));
                    }
                    (Instant::now(), b.peers_waiting())
                })
            };
            std::thread::sleep(Duration::from_millis(2));
            let t_wake = Instant::now();
            assert_eq!(b.arrive(0), ArriveOutcome::Pending);
            let (t_ret, observed) = parker.join().expect("parker panicked");
            assert!(observed, "every poll-leg round must observe the arrival");
            poll.push(t_ret.saturating_duration_since(t_wake));
        }
        kernel.sort_unstable();
        poll.sort_unstable();
        eprintln!(
            "park-wake A/B ({ROUNDS} rounds, park_wake_available={}):",
            wake_word_block_primitive_available()
        );
        if kernel.is_empty() {
            eprintln!("  wake-word: (no primitive on this host — leg skipped)");
        } else {
            eprintln!(
                "  wake-word : p50={:?} p99={:?}",
                pct(&kernel, 0.50),
                pct(&kernel, 0.99)
            );
        }
        eprintln!(
            "  100us-poll: p50={:?} p99={:?}",
            pct(&poll, 0.50),
            pct(&poll, 0.99)
        );
    }
}
