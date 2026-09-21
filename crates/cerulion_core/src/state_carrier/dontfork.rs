// SPDX-License-Identifier: AGPL-3.0-only
//! Fork exclusion: the mappings a capture child must NOT inherit, and the
//! per-platform primitive that excludes them.
//!
//! # What the control is for, and what it is NOT for
//!
//! `fork` freezes **private anonymous** memory; `MAP_SHARED` regions are not frozen —
//! they stay live and shared. User state is heap, so it is frozen, but any accidental
//! read of an iceoryx2 SHM segment by the encoder would read live, torn,
//! concurrently-mutated bytes and write them into the bag as if they were a
//! point-in-time image (the silent-corruption class). With the region excluded, such a read
//! is a **SIGSEGV in a disposable child** instead of silent corruption in the recording.
//!
//! It is a CORRECTNESS control, not a cost lever: a measurement on the real L4T
//! kernel showed that 1 GiB of touched `MAP_SHARED` mappings changes fork cost by ~nothing
//! (`vma_needs_copy` skips PTE copy for such regions), so excluding them saves no time.
//!
//! **It is also not the load-bearing argument.** That one is structural and
//! platform-independent: the encoder walks only the user struct's own declared fields,
//! iceoryx2 handles are not in it (ports live in `NodeContext`, reached through the
//! injected `__cer_rt`, which the capture never traverses), so there is no path from a
//! capture to an iceoryx2 mapping at all. This is belt-and-braces over that.
//!
//! # The sweep set, and its exact complement
//!
//! The sweep set goes beyond the iceoryx2 pools to the carrier's own live mappings, and
//! the trace ring is the one that matters most: the child inherits a fully ARMED
//! `ShmRingProducer` over it, and a stray write there is the silent-corruption class
//! pointed straight at the LIVE bag.
//!
//! | Mapping | Swept? | Why |
//! |---|---|---|
//! | iceoryx2 SHM pools | YES | torn reads of live data-plane bytes |
//! | the per-worker TRACE ring | YES | the child inherits an armed producer over the LIVE bag |
//! | the `MappedBarrier` page | YES | a live cross-process rendezvous word |
//! | the `StateArmWord` | YES | claims and reservations peers are reading right now |
//! | the `MappedCredit` page | YES | a live cross-process backpressure word |
//! | **the STATE ring** | **no** | the child's OUTPUT — excluding it emits no anchor |
//! | **the breadcrumb** | **no** | the child's only liveness channel — excluding it means every child is killed at the first stall timeout |
//!
//! zenoh and bag mappings are absent by TOPOLOGY rather than by exclusion:
//! the graph/worker process is network-free and the bag belongs to `bagd`, a
//! separate process, so neither is mapped here to sweep.
//!
//! # The platform split
//!
//! `MADV_DONTFORK` is Linux-specific. macOS does not lose the control:
//! **`minherit(2)` / `VM_INHERIT_NONE` is verified empirically on macOS**
//! (the excluded region is absent in the child, SIGSEGV on
//! touch; a control region is inherited intact), which keeps dev/robot parity for
//! this control. Both constants are read from the system headers for this module:
//! `VM_INHERIT_NONE == 2` from `<mach/vm_inherit.h>` and
//! `int minherit(void *, size_t, int)` from `<sys/mman.h>`.
//!
//! The two differ in GRANULARITY — `minherit` works on whole regions — which is why
//! every mapping the carrier creates is sized as its own page-aligned region rather than
//! packed alongside anything else.
//!
//! Any other Unix reports [`ExclusionOutcome::Unsupported`] rather than silently
//! succeeding: a control that cannot be applied must SAY so, or the structural argument
//! above becomes the only defence without anyone knowing it.
//!
//! NOTE: this module is compiled only on Unix — the `#[cfg(unix)]` gate lives on the
//! `pub mod state_carrier;` declaration in `lib.rs`.

/// `VM_INHERIT_NONE` from `<mach/vm_inherit.h>`.
///
/// MEASURED first-party on macOS rather than transcribed: `VM_INHERIT_SHARE=0`,
/// `VM_INHERIT_COPY=1`, `VM_INHERIT_NONE=2`, `VM_INHERIT_DEFAULT=1`. `libc` does not
/// expose it, so the value is pinned here and asserted by the behavioural arm — a wrong
/// constant would make `minherit` refuse (or, worse, request SHARE) and the exclusion
/// would silently not happen.
#[cfg(target_vendor = "apple")]
const VM_INHERIT_NONE: libc::c_int = 2;

#[cfg(target_vendor = "apple")]
unsafe extern "C" {
    /// `int minherit(void *, size_t, int)` — `<sys/mman.h>`. Not in `libc` 0.2, so it
    /// is declared here.
    fn minherit(addr: *mut libc::c_void, len: libc::size_t, inherit: libc::c_int) -> libc::c_int;
}

/// What happened to one exclusion request.
///
/// A three-way answer rather than a `bool`, because "the platform has no such control"
/// and "the platform has one and it failed" need different operator lines: the first is
/// a documented degrade to the structural argument, the second is a live problem.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExclusionOutcome {
    /// The region will not be inherited by a `fork` child.
    Excluded,
    /// This platform has no primitive for it; the structural argument carries alone.
    Unsupported,
    /// The platform has one and the call failed, with this `errno`.
    Failed(i32),
}

impl ExclusionOutcome {
    /// Whether the child is genuinely prevented from touching the region.
    pub fn is_excluded(self) -> bool {
        matches!(self, Self::Excluded)
    }

    /// The operator-facing noun.
    pub fn label(self) -> &'static str {
        match self {
            Self::Excluded => "excluded",
            Self::Unsupported => "unsupported-on-this-platform",
            Self::Failed(_) => "failed",
        }
    }
}

/// Exclude `[ptr, ptr + len)` from inheritance by any future `fork` child.
///
/// # A sub-page `len` is fine, and that is MEASURED rather than assumed
///
/// Both primitives are page-granular and both accept a length shorter than a page:
/// Linux's `madvise` rounds the length up itself, and `minherit` driven at 64 bytes
/// (the `MappedBarrier` rendezvous word) excludes the page. No rounding helper is
/// needed: the whole sweep-set arm passes without one — a barrier page
/// left unexcluded means a MISSING CALL SITE, not a
/// refused length, which is precisely what the behavioural arm exists to tell apart.
///
/// # Safety
///
/// `ptr`/`len` must describe a live mapping this process owns. Passing a region that is
/// NOT owned here would change inheritance for memory belonging to something else.
///
/// # It is applied at BIRTH, not swept once
///
/// A one-shot sweep at arm time would leave a hole exactly where the
/// recorder is: iceoryx2 mappings appear continuously AFTER arm — `bagd`'s own taps arm
/// after the graph is released, its discovery rescan attaches new taps every
/// 250 ms for the life of the recording, and vizd / `topic echo` demands attach at
/// arbitrary times. Any mapping created after a one-shot sweep would be unmarked, so
/// the guarantee would degrade to silent corruption precisely for the segments the
/// recorder's own activity creates. The birth hook is `shm_guard`'s
/// create-time call site; this function is what it calls.
pub unsafe fn exclude_from_fork(ptr: *mut std::ffi::c_void, len: usize) -> ExclusionOutcome {
    if len == 0 {
        // Nothing to exclude. Reporting `Excluded` for an empty region would be a claim
        // about memory that does not exist.
        return ExclusionOutcome::Unsupported;
    }
    #[cfg(target_os = "linux")]
    {
        // SAFETY: the caller guarantees the region is a live mapping it owns.
        let rc = unsafe { libc::madvise(ptr, len, libc::MADV_DONTFORK) };
        if rc == 0 {
            ExclusionOutcome::Excluded
        } else {
            ExclusionOutcome::Failed(errno())
        }
    }
    #[cfg(target_vendor = "apple")]
    {
        // SAFETY: as above; `minherit` is declared from `<sys/mman.h>`.
        let rc = unsafe { minherit(ptr, len, VM_INHERIT_NONE) };
        if rc == 0 {
            ExclusionOutcome::Excluded
        } else {
            ExclusionOutcome::Failed(errno())
        }
    }
    #[cfg(not(any(target_os = "linux", target_vendor = "apple")))]
    {
        let _ = (ptr, len);
        ExclusionOutcome::Unsupported
    }
}

#[cfg(any(target_os = "linux", target_vendor = "apple"))]
fn errno() -> i32 {
    std::io::Error::last_os_error().raw_os_error().unwrap_or(0)
}

/// A mapping the carrier excludes AT BIRTH, named for the operator line.
///
/// A closed set rather than a string, so a new call site has to be added here — the
/// list IS the sweep set, and it should not be extendable by accident.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ForkExcludedMapping {
    /// A per-worker TRACE ring. The child inherits a fully ARMED producer over it, and
    /// a stray write is the silent-corruption class pointed at the LIVE bag.
    TraceRing,
    /// The cross-process `MappedBarrier` rendezvous page.
    BarrierPage,
    /// The `StateArmWord` — claims and reservations peers are reading right now.
    StateArmWord,
    /// An iceoryx2 SHM pool VMA.
    IceoryxPool,
    /// A cross-process `block`-edge CREDIT word — the producer's
    /// defer gate and the consumer's outstanding count, both live.
    CreditPage,
    /// A rank's flow-mode `WedgePage` — the in-tick seq pairs its supervisor is
    /// reading right now. A capture child can never legitimately write one, so all
    /// inheriting it buys is the CoW cost of a live page.
    WedgePage,
}

impl ForkExcludedMapping {
    /// The operator-facing noun.
    pub fn label(self) -> &'static str {
        match self {
            Self::TraceRing => "trace-ring",
            Self::BarrierPage => "barrier-page",
            Self::StateArmWord => "state-arm-word",
            Self::IceoryxPool => "iceoryx2-pool",
            Self::CreditPage => "credit-page",
            Self::WedgePage => "wedge-page",
        }
    }
}

/// Exclude a mapping AT BIRTH and report the outcome.
///
/// This is the production entry point — [`exclude_from_fork`] is the primitive. It is
/// called from each mapping's own constructor rather than from a sweep, because a
/// one-shot sweep at arm time would leave a hole exactly where the recorder is: a
/// mapping created AFTER a sweep would be unmarked, and the recorder's own activity
/// creates mappings continuously.
///
/// NEVER fatal. The compensating control is structural (the encoder walks only the
/// user struct's declared fields), so a failure to exclude degrades to that argument
/// and says so — it does not refuse to create the ring the robot needs.
///
/// # Safety
///
/// `ptr`/`len` must describe a live mapping this process owns.
pub unsafe fn exclude_at_birth(
    ptr: *mut std::ffi::c_void,
    len: usize,
    what: ForkExcludedMapping,
) -> ExclusionOutcome {
    // SAFETY: forwarded from the caller's own contract.
    let outcome = unsafe { exclude_from_fork(ptr, len) };
    // By design, fold this outcome into the process ledger the arm-time
    // sweep reads (see `sweep_birth_exclusions_at_arm`). Recorded BEFORE the log
    // arms below so a mapping is counted whatever level its outcome logs at.
    record_birth_exclusion(outcome);
    match outcome {
        ExclusionOutcome::Excluded => {
            tracing::debug!(
                mapping = what.label(),
                bytes = len,
                "excluded from fork inheritance"
            );
        }
        ExclusionOutcome::Unsupported => {
            tracing::debug!(
                mapping = what.label(),
                "this platform cannot exclude a mapping from fork inheritance; a \
                 capture child's stray read would be silent rather than a fault"
            );
        }
        ExclusionOutcome::Failed(errno) => {
            tracing::warn!(
                mapping = what.label(),
                bytes = len,
                errno,
                "could NOT exclude a mapping from fork inheritance; a capture child \
                 that reads it would see live, torn bytes instead of faulting"
            );
        }
    }
    outcome
}

/// The PROCESS-WIDE ledger of every [`exclude_at_birth`] ATTEMPT, kept as four
/// atomics so a mapping constructor pays no lock to be counted.
///
/// It exists because the arm-time sweep has no other way to ask: exclusion
/// is a hook in each mapping's own constructor (see [`exclude_at_birth`]),
/// not a one-shot pass over a mapping LIST. The hook closes the hole
/// where a mapping born after the pass goes unmarked — but it also leaves nobody
/// holding the pointers, so "did every exclusion succeed?" can only be answered
/// from what those hooks RECORDED.
///
/// # It counts ATTEMPTS, not live mappings, and the distinction is not academic
///
/// MEASURED, not assumed: `transport::shm_guard::advise_shm_pools_no_hugepage`
/// re-scans the whole of `/proc/self/maps` and calls [`exclude_at_birth`] on
/// EVERY iceoryx2 pool VMA it finds, once per publisher / subscriber / capture
/// tap creation — so one long-lived pool is re-attempted on every port a
/// process opens, and `excluded` can exceed the number of distinct mappings
/// several-fold. A counter incremented there is a count of CALLS by
/// construction, and no naming or lifecycle scheme can turn it into a count of
/// regions without that site first learning which VMAs it has already seen.
///
/// So the ledger is read as attempts throughout — see
/// [`sweep_birth_exclusions_at_arm`] for what that does and does not license
/// an operator to conclude.
static BIRTH_EXCLUDED: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
static BIRTH_UNSUPPORTED: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
static BIRTH_FAILED: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
static BIRTH_LAST_ERRNO: std::sync::atomic::AtomicI32 = std::sync::atomic::AtomicI32::new(0);

/// Fold one birth outcome into the ledger. `Relaxed` throughout: the ledger is a
/// DIAGNOSTIC tally read at arm time, never a synchronisation edge, and a
/// counter must never be able to wedge the mapping constructor it observes.
fn record_birth_exclusion(outcome: ExclusionOutcome) {
    use std::sync::atomic::Ordering::Relaxed;
    match outcome {
        ExclusionOutcome::Excluded => {
            BIRTH_EXCLUDED.fetch_add(1, Relaxed);
        }
        ExclusionOutcome::Unsupported => {
            BIRTH_UNSUPPORTED.fetch_add(1, Relaxed);
        }
        ExclusionOutcome::Failed(errno) => {
            BIRTH_FAILED.fetch_add(1, Relaxed);
            BIRTH_LAST_ERRNO.store(errno, Relaxed);
        }
    }
}

/// Principle #3 (observable state): the arm-time sweep of the fork-exclusion
/// set — what every mapping born in this process so far reported.
///
/// # This is a VERIFICATION pass, not a MARKING pass
///
/// Marking is done by per-constructor birth hooks, not by a one-shot
/// `MADV_DONTFORK` sweep AT ARM TIME, because an arm-time marking
/// pass has a hole exactly where the recorder is (the recorder's own
/// activity creates mappings continuously, so anything born after the pass is
/// unmarked). What arm time still owes the operator is the QUESTION a sweep
/// answers — *is the fork set correct before the first anchor?* —
/// and that is answered by reading the hooks' ledger rather than by
/// re-marking memory nobody holds a pointer to any more.
///
/// # What it measures: ATTEMPTS this process made, not mappings it holds
///
/// A CUMULATIVE tally of [`exclude_at_birth`] CALLS, never reset. Two
/// consequences, both real and both stated here rather than left for an
/// operator to discover from a confusing line:
///
/// 1. **A dropped mapping keeps its outcome.** Nothing deregisters: an
///    exclusion that failed for a mapping since `munmap`ped still counts, so
///    [`ForkExclusionSweep::every_attempt_succeeded`] can be false while every
///    mapping alive RIGHT NOW is excluded.
/// 2. **One mapping can be counted many times.** The iceoryx2 pool site
///    re-attempts every pool VMA on every port creation (see the ledger's
///    docs), so `excluded` is not a population count.
///
/// # Why it is not a live-set registry
///
/// Because two of the four birth sites cannot supply one. The iceoryx2 pool
/// site has no owning handle at all — it walks `/proc/self/maps` and marks
/// VMAs whose lifetime belongs to iceoryx2, and nothing in this crate ever
/// unmaps them — and the trace ring's munmap is `Arc`-deferred behind
/// `shm_ring::Mapping`, so the owner's `Drop` is not the moment its region
/// stops being forkable. Pairing births with deaths at the two sites that CAN
/// (the barrier page, the arm word) while the other two silently could not
/// would produce a number that looks like a live set and is not — strictly
/// worse than a tally that says what it is. Making it a real live set means
/// introducing a mapping registry, which is a
/// design change, not a rename.
///
/// # The residual fails in the SAFE direction, deliberately
///
/// Retaining a dropped mapping's failure can only ever raise a false alarm —
/// it can never silence a true one, because an outcome is only ever folded IN.
/// For a diagnostic whose whole job is to tell an operator that the silent-corruption
/// class is unguarded, warning about a hazard that has since gone
/// away is the survivable error and staying quiet about a live one is not.
/// Callers that want a WINDOW rather than a lifetime take two snapshots.
pub fn sweep_birth_exclusions_at_arm() -> ForkExclusionSweep {
    use std::sync::atomic::Ordering::Relaxed;
    ForkExclusionSweep {
        excluded: BIRTH_EXCLUDED.load(Relaxed),
        unsupported: BIRTH_UNSUPPORTED.load(Relaxed),
        failed: BIRTH_FAILED.load(Relaxed),
        last_errno: BIRTH_LAST_ERRNO.load(Relaxed),
    }
}

/// What a sweep did, so it is observable rather than assumed (Principle #3).
///
/// A sweep that silently excluded nothing looks exactly like one that excluded
/// everything, and the difference is whether the silent-corruption class is guarded.
///
/// Every field counts ATTEMPTS — [`exclude_at_birth`] CALLS — not distinct
/// mappings and not live ones. See [`sweep_birth_exclusions_at_arm`] for why
/// that is what the birth sites can actually supply, and which two conclusions
/// it therefore does not license.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ForkExclusionSweep {
    /// Attempts that genuinely excluded their region.
    pub excluded: u32,
    /// Attempts on a platform that cannot exclude a mapping.
    pub unsupported: u32,
    /// Attempts whose exclusion call failed.
    pub failed: u32,
    /// The last `errno` seen, for the operator line.
    pub last_errno: i32,
}

impl ForkExclusionSweep {
    /// Fold one outcome in.
    pub fn record(&mut self, outcome: ExclusionOutcome) {
        match outcome {
            ExclusionOutcome::Excluded => self.excluded += 1,
            ExclusionOutcome::Unsupported => self.unsupported += 1,
            ExclusionOutcome::Failed(e) => {
                self.failed += 1;
                self.last_errno = e;
            }
        }
    }

    /// Whether every exclusion ATTEMPT folded in here succeeded, and there was
    /// at least one.
    ///
    /// FALSE when anything was unsupported or failed — deliberately not "no failures",
    /// because an unsupported platform is exactly the case where the operator must know
    /// the structural argument is carrying alone.
    ///
    /// Named for ATTEMPTS rather than for the fork set, because that is what it
    /// can see: the tally retains a dropped mapping's outcome and can count one
    /// mapping many times, so `false` means "something in this process's history
    /// did not get excluded", NOT "a live mapping is forkable right now". The
    /// gap is one-directional — see [`sweep_birth_exclusions_at_arm`].
    pub fn every_attempt_succeeded(&self) -> bool {
        self.failed == 0 && self.unsupported == 0 && self.excluded > 0
    }

    /// How many exclusion attempts the ledger has folded in.
    pub fn attempts(&self) -> u32 {
        self.excluded + self.unsupported + self.failed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_zero_length_region_is_never_reported_excluded() {
        // Reporting `Excluded` for an empty region would be a claim about memory that
        // does not exist, and it would make `every_attempt_succeeded()` true for a sweep that
        // protected nothing.
        let outcome = unsafe { exclude_from_fork(std::ptr::null_mut(), 0) };
        assert!(!outcome.is_excluded());
        assert_eq!(outcome, ExclusionOutcome::Unsupported);
    }

    #[test]
    fn the_sweep_tally_separates_unsupported_from_failed_and_from_total() {
        // Hand oracle. `every_attempt_succeeded()` must be false for an UNSUPPORTED platform as well as
        // for a failure: that is the case where the operator most needs to know the
        // structural argument is carrying alone, and folding it into "no failures" is
        // exactly how a control goes quietly missing.
        let mut s = ForkExclusionSweep::default();
        assert!(
            !s.every_attempt_succeeded(),
            "an empty sweep protected nothing"
        );
        assert_eq!(s.attempts(), 0);

        s.record(ExclusionOutcome::Excluded);
        s.record(ExclusionOutcome::Excluded);
        assert!(s.every_attempt_succeeded());
        assert_eq!((s.excluded, s.unsupported, s.failed), (2, 0, 0));
        assert_eq!(s.attempts(), 2);

        s.record(ExclusionOutcome::Unsupported);
        assert!(
            !s.every_attempt_succeeded(),
            "one unsupported region breaks totality"
        );
        assert_eq!((s.excluded, s.unsupported, s.failed), (2, 1, 0));

        s.record(ExclusionOutcome::Failed(libc::EINVAL));
        assert!(!s.every_attempt_succeeded());
        assert_eq!((s.excluded, s.unsupported, s.failed), (2, 1, 1));
        assert_eq!(s.last_errno, libc::EINVAL);
        assert_eq!(s.attempts(), 4);
    }

    /// By design, a birth exclusion is folded into the ledger the
    /// arm-time sweep reads.
    ///
    /// Asserted as a DELTA and with `>=`, deliberately: the ledger is
    /// process-wide and monotone, and sibling tests in this crate construct real
    /// barrier pages / trace rings / SHM pools whose own birth hooks bump it
    /// concurrently. An exact total would be a test that fails when an unrelated
    /// test runs beside it; the EXACT folding arithmetic is pinned by
    /// `the_sweep_tally_separates_unsupported_from_failed_and_from_total` over
    /// the same `record`, so what is left to pin here is only that
    /// `exclude_at_birth` reaches the ledger at all.
    #[test]
    fn a_birth_exclusion_is_recorded_in_the_ledger_the_arm_time_sweep_reads() {
        let before = sweep_birth_exclusions_at_arm();
        // A zero-length region is refused by the primitive and reported
        // `Unsupported` (pinned above), so this drives a KNOWN arm of the fold
        // without needing a real mapping to mark.
        let outcome =
            unsafe { exclude_at_birth(std::ptr::null_mut(), 0, ForkExcludedMapping::TraceRing) };
        assert_eq!(outcome, ExclusionOutcome::Unsupported);
        let after = sweep_birth_exclusions_at_arm();
        assert!(
            after.unsupported > before.unsupported,
            "the birth hook's outcome must reach the ledger: {before:?} -> {after:?}"
        );
        assert!(
            after.attempts() > before.attempts(),
            "the sweep must count the mapping it was told about: {before:?} -> {after:?}"
        );
        // The ledger is CUMULATIVE — nothing may reset it, or the arm-time
        // answer would be smaller than the truth.
        assert!(after.excluded >= before.excluded);
        assert!(after.failed >= before.failed);
    }

    /// By design, one region attempted twice is counted twice: the
    /// ledger is a tally of CALLS, and this is the shape that makes that
    /// unavoidable rather than merely true today.
    ///
    /// `transport::shm_guard::advise_shm_pools_no_hugepage` re-scans the whole
    /// of `/proc/self/maps` and calls [`exclude_at_birth`] on every iceoryx2
    /// pool VMA it finds, once per publisher / subscriber / capture-tap
    /// creation — so a long-lived pool is re-attempted on every port the
    /// process opens. A reader who takes `excluded` for a POPULATION would
    /// over-count such a process several-fold, which is why
    /// [`sweep_birth_exclusions_at_arm`] states attempts and
    /// [`ForkExclusionSweep::every_attempt_succeeded`] is named for them.
    ///
    /// The region is REAL (a `MAP_SHARED` anonymous page), not the degenerate
    /// zero-length one the sibling test uses, because the claim is about the
    /// SUCCESS arm: a zero-length region reports `Unsupported` and would leave
    /// the `Excluded` counter — the one the pool site drives — untouched.
    ///
    /// The delta is asserted with `>=` for the reason the sibling states: the
    /// ledger is process-wide and sibling tests in this binary construct real
    /// barrier pages / arm words whose own birth hooks bump it concurrently.
    /// The MULTIPLICITY claim still holds, because a per-REGION ledger could
    /// contribute at most 1 for the two calls below and both are asserted to
    /// have taken the counted arm.
    #[test]
    fn one_region_attempted_twice_is_counted_twice_because_the_tally_counts_calls() {
        const LEN: usize = 4096;
        // SAFETY: a fresh anonymous shared mapping owned solely by this test.
        let addr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                LEN,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED | libc::MAP_ANON,
                -1,
                0,
            )
        };
        assert_ne!(addr, libc::MAP_FAILED, "could not map the probe region");

        let before = sweep_birth_exclusions_at_arm();
        // SAFETY: `addr`/`LEN` describe the live mapping this test owns; the
        // call is non-destructive advice and is deliberately repeated.
        let first = unsafe { exclude_at_birth(addr, LEN, ForkExcludedMapping::IceoryxPool) };
        // SAFETY: same live mapping, still owned here — this is precisely the
        // re-attempt the pool site performs on every port creation.
        let second = unsafe { exclude_at_birth(addr, LEN, ForkExcludedMapping::IceoryxPool) };
        let after = sweep_birth_exclusions_at_arm();

        // SAFETY: unmap exactly what was mapped above.
        unsafe { libc::munmap(addr, LEN) };

        // Both attempts must land on the SAME arm — otherwise the delta below
        // could be explained by two different counters moving once each.
        assert_eq!(
            first, second,
            "re-attempting one region must report the same outcome both times"
        );
        let counted_twice = match first {
            ExclusionOutcome::Excluded => after.excluded >= before.excluded + 2,
            ExclusionOutcome::Unsupported => after.unsupported >= before.unsupported + 2,
            ExclusionOutcome::Failed(_) => after.failed >= before.failed + 2,
        };
        assert!(
            counted_twice,
            "one region attempted twice must be counted twice ({first:?}): {before:?} -> {after:?}"
        );
    }

    /// By design, the retained-outcome residual is one-directional,
    /// and that direction is the whole reason it is acceptable.
    ///
    /// Nothing deregisters, so an exclusion that failed for a mapping since
    /// `munmap`ped keeps counting — [`ForkExclusionSweep::every_attempt_succeeded`]
    /// can be false while every mapping alive right now is excluded. What must
    /// NEVER happen is the inverse: no run of successes may talk the tally back
    /// into claiming totality, because that is the reading that would tell an
    /// operator the silent-corruption class is guarded when it is not.
    ///
    /// Asserted over the fold rather than the process ledger deliberately: the
    /// property is that an outcome is only ever folded IN, which is a statement
    /// about the type's API surface (there is no `un-record`), and driving it
    /// through the shared ledger would make it race every sibling test.
    #[test]
    fn a_recorded_failure_can_never_be_talked_back_into_totality() {
        let mut s = ForkExclusionSweep::default();
        s.record(ExclusionOutcome::Excluded);
        assert!(s.every_attempt_succeeded(), "the healthy baseline");

        // The mapping this failure belonged to is now (conceptually) gone.
        s.record(ExclusionOutcome::Failed(libc::EINVAL));
        assert!(!s.every_attempt_succeeded());

        // Every later attempt succeeds — a process that healed completely.
        for _ in 0..1000 {
            s.record(ExclusionOutcome::Excluded);
        }
        assert!(
            !s.every_attempt_succeeded(),
            "a retained failure must stay retained: the tally may raise a false alarm, \
             never silence a true one"
        );
        assert_eq!(s.failed, 1);
        assert_eq!(s.last_errno, libc::EINVAL);

        // The same one-directional rule for an UNSUPPORTED platform.
        let mut u = ForkExclusionSweep::default();
        u.record(ExclusionOutcome::Unsupported);
        for _ in 0..1000 {
            u.record(ExclusionOutcome::Excluded);
        }
        assert!(!u.every_attempt_succeeded());
    }

    #[test]
    fn every_outcome_has_a_distinct_operator_noun() {
        // Three conditions, three lines: "excluded" and "unsupported" collapsing to one
        // word is how a platform silently losing the control reads as success.
        let labels = [
            ExclusionOutcome::Excluded.label(),
            ExclusionOutcome::Unsupported.label(),
            ExclusionOutcome::Failed(libc::EINVAL).label(),
        ];
        let unique: std::collections::BTreeSet<_> = labels.iter().collect();
        assert_eq!(
            unique.len(),
            labels.len(),
            "labels must not collide: {labels:?}"
        );
    }

    #[cfg(target_vendor = "apple")]
    #[test]
    fn the_apple_inherit_constant_is_the_one_the_kernel_accepts() {
        // The value is hand-pinned (libc does not expose it), so it is checked against
        // the kernel rather than against a comment: `minherit` rejects an out-of-range
        // inherit value, so a successful call on a real mapping is the constant being
        // in range — and the behavioural fork arm proves it is the NONE one rather than
        // SHARE or COPY.
        let len = 4096;
        let p = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                len,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_PRIVATE | libc::MAP_ANONYMOUS,
                -1,
                0,
            )
        };
        assert_ne!(p, libc::MAP_FAILED);
        let ok = unsafe { minherit(p, len, VM_INHERIT_NONE) };
        let bogus = unsafe { minherit(p, len, 99) };
        unsafe { libc::munmap(p, len) };
        assert_eq!(
            ok,
            0,
            "VM_INHERIT_NONE must be accepted: {}",
            std::io::Error::last_os_error()
        );
        assert_ne!(bogus, 0, "an out-of-range inherit value must be refused");
    }
}
