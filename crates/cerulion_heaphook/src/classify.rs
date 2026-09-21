// SPDX-License-Identifier: AGPL-3.0-only
//! The pointer-classification decision — the pure `#[cfg]`-free heart of the
//! `free`/`realloc` interposers.
//!
//! Given a pointer handed to `free`/`realloc`, decide where it must go:
//!
//! * [`PtrClass::WindowSlot`] — it is inside the armed borrow window's slot. It
//!   is a bump-arena allocation; `free` of it is a no-op (the arena does not
//!   reclaim), and `realloc` of it is an escape (the vector grew past its bump).
//!   NEVER passed to the real allocator (the slot is shared memory, not heap).
//! * [`PtrClass::Segment`] — it is inside a registered shared-memory range (a
//!   forged take-side vector's storage, or a publish-side slot the rmw kept).
//!   Routed to the registered release callback with the range's cookie, NEVER to
//!   glibc `free` (which would poison the tcache with an SHM address — the
//!   earlier residual this closes).
//! * [`PtrClass::Quarantine`] — inside a DISARMED window's slot extent that the
//!   consumer has not yet retired. A window bumps EVERY allocation on its thread
//!   while armed — not just message storage, but any buffer stock user code
//!   allocates incidentally during the fill — and those pointers can be freed
//!   AFTER disarm or on ANOTHER thread, where no window is armed. Sending such an
//!   SHM address to glibc `free` corrupts the heap; so ARMING a window
//!   quarantines the slot's WHOLE `[base, tail_limit)` extent up front (coverage
//!   before exposure — disarm folds nothing further), and a
//!   later `free` of any address in it is a NO-OP (a bounded leak, never
//!   corruption). `retire_slot` TOMBSTONES the extent rather than deleting
//!   it, so the coverage — and the no-op — PERSIST past retire (an incidental
//!   in-slot pointer can outlive the sample); a re-armed/recycled slot revives
//!   the entry. This is the same "bounded leak over corruption" direction the
//!   bootstrap arena and the missing-callback path take.
//! * [`PtrClass::RealHeap`] — anything else: an ordinary private-heap pointer,
//!   passed straight to the real allocator.
//!
//! Order: window (armed) → registry (release) → quarantine (no-op) → real heap.
//! While a window is armed its slot bytes are governed by bump-arena semantics;
//! a range the consumer registered for release wins over a stale quarantine
//! entry; only after all three miss is a pointer treated as private heap.

use crate::registry::SegmentRegistry;
use crate::window::BorrowWindow;

/// Where a `free`/`realloc` pointer must be routed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PtrClass {
    /// An ordinary private-heap pointer → the real allocator.
    RealHeap,
    /// Inside the armed window slot → bump-arena semantics (free no-op, realloc
    /// escape). Never the real allocator.
    WindowSlot,
    /// Inside a registered SHM range → the release callback. Carries the range's
    /// opaque cookie.
    Segment {
        /// The caller cookie (sample/slot handle) for the release callback.
        cookie: usize,
    },
    /// Inside a quarantined slot extent — disarmed-but-not-yet-retired, OR
    /// retired-and-tombstoned: free is a NO-OP (bounded leak), realloc
    /// moves the bytes out. Never glibc, never the release callback. (The
    /// live-vs-tombstone distinction is the free path's — a diagnostic-counter
    /// concern — not a classification one: both are the same no-op coverage.)
    Quarantine,
}

/// Classify `ptr` against the (optionally armed) window, the segment registry,
/// and the quarantine set of disarmed slot extents. A null pointer is
/// [`PtrClass::RealHeap`] (`free(NULL)` is a standard no-op the real allocator
/// handles; `realloc(NULL, n)` is a `malloc`).
pub fn classify_ptr(
    ptr: usize,
    window: Option<&BorrowWindow>,
    registry: &SegmentRegistry,
    quarantine: &SegmentRegistry,
) -> PtrClass {
    if ptr == 0 {
        return PtrClass::RealHeap;
    }
    if let Some(w) = window {
        if w.owns(ptr) {
            return PtrClass::WindowSlot;
        }
    }
    if let Some(seg) = registry.classify(ptr) {
        return PtrClass::Segment {
            cookie: seg.cookie(),
        };
    }
    if quarantine.classify(ptr).is_some() {
        return PtrClass::Quarantine;
    }
    PtrClass::RealHeap
}

// ── The atfork prepare-interval decision ───────────────────────

/// What the REENTRANT arm must do with a pointer during the `atfork`
/// prepare-lock interval (the SHM_FREE_BYPASS fix). `Unmanaged` = not ours —
/// the ordinary reentrant real-allocator action applies. For every managed
/// variant: free is a NO-OP or a leak (counted per class by the caller),
/// realloc fails safe, usable-size is conservative 0 — never glibc on a
/// possibly-SHM address.
///
/// Lives in this `#[cfg]`-free module (not `state`) so the decision table is
/// oracle-testable on every platform; `state::atfork_interval_classify` is the
/// Linux wiring that feeds it the thread-local phase + table lookups.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntervalPtrAction {
    /// This thread's armed (un-poisoned) window slot: free is a bump-arena
    /// no-op.
    NoopWindow,
    /// A live quarantine extent (counter kind 3).
    NoopQuarantine,
    /// A tombstoned extent (counter kind 4).
    NoopTombstone,
    /// A REGISTERED take-side range: LEAK-AND-COUNT (kind 5). The registration
    /// is KEPT (coverage intact — a later normal free still releases properly)
    /// and the release callback is NOT fired: firing a consumer callback from
    /// inside a foreign prepare handler, while this thread holds both spin
    /// locks, invites every callback-side deadlock at the worst possible
    /// moment, and an allocation-free deferred-release queue cannot be given
    /// a true bound. A leaked sample during a prepare interval is bounded, counted
    /// and safe; a corrupted allocator is neither.
    LeakRegistered,
    /// A pointer freed inside the WIDENED gate boundary: the
    /// interval gate is armed but this thread does not (provably) hold both
    /// locks — the global tables CANNOT be read (another thread may be
    /// mutating them under the locks), so the pointer cannot be proven
    /// unmanaged. The safe direction is to LEAK-AND-COUNT (kind 5): a leaked
    /// allocation in a few-instruction, signal-only window is bounded and
    /// counted; a glibc `free` of what might be an SHM address corrupts the
    /// heap. Never reached outside the boundary spans of the `atfork`
    /// handlers (`atfork_lock`/`atfork_unlock`, and the child reset's
    /// force-open→clear tail).
    LeakUnclassified,
    /// Not hook-managed: the ordinary reentrant real-allocator action.
    Unmanaged,
}

/// Where this thread stands relative to the `atfork` prepare-lock interval, as
/// published by `state::atfork_lock`/`atfork_unlock` in their ordering: the
/// interval GATE is armed BEFORE the first lock acquisition and cleared AFTER
/// the last release, while the locks-held flag is set only between the actual
/// acquire and release. A signal handler landing between the two observes
/// [`AtforkIntervalPhase::ArmedLocksNotHeld`] — the widened boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AtforkIntervalPhase {
    /// Not in (or around) the interval: interval classification does not
    /// apply — the ordinary reentrant action.
    Outside,
    /// The gate is armed but both locks are not (provably) held by this
    /// thread: the prepare-side pre-acquisition span, the parent-side
    /// post-release span, and the tiny flag-update gaps beside them. The
    /// tables are UNREADABLE here.
    ArmedLocksNotHeld,
    /// The interval proper: this thread holds BOTH locks, so lock-free table
    /// reads are race-free.
    ArmedLocksHeld,
}

/// What the (lock-free) table lookup answered — produced only under
/// [`AtforkIntervalPhase::ArmedLocksHeld`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntervalTableVerdict {
    /// In the segment registry.
    Registered,
    /// In a LIVE quarantine extent.
    QuarantineLive,
    /// In a TOMBSTONED quarantine extent.
    QuarantineTombstoned,
    /// In neither table.
    NotFound,
}

/// The pure `atfork`-interval decision. `window_hit` is whether
/// this thread's (un-poisoned) armed window owns the pointer — thread-local
/// state, safe to read in EVERY armed phase. `tables` is the global-table
/// lookup and is consulted ONLY under [`AtforkIntervalPhase::ArmedLocksHeld`]:
/// in the widened boundary the tables may be mid-mutation under another
/// thread's lock, so reading them would be the exact race the phase split
/// exists to avoid — the boundary answer is the conservative
/// [`IntervalPtrAction::LeakUnclassified`], never a fall-through to glibc and
/// never a table read.
pub fn interval_ptr_action(
    phase: AtforkIntervalPhase,
    window_hit: bool,
    tables: impl FnOnce() -> IntervalTableVerdict,
) -> Option<IntervalPtrAction> {
    match phase {
        AtforkIntervalPhase::Outside => None,
        _ if window_hit => Some(IntervalPtrAction::NoopWindow),
        AtforkIntervalPhase::ArmedLocksNotHeld => Some(IntervalPtrAction::LeakUnclassified),
        AtforkIntervalPhase::ArmedLocksHeld => Some(match tables() {
            IntervalTableVerdict::Registered => IntervalPtrAction::LeakRegistered,
            IntervalTableVerdict::QuarantineLive => IntervalPtrAction::NoopQuarantine,
            IntervalTableVerdict::QuarantineTombstoned => IntervalPtrAction::NoopTombstone,
            IntervalTableVerdict::NotFound => IntervalPtrAction::Unmanaged,
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn armed_window() -> BorrowWindow {
        // slot [10_000, 20_000); bump 100 bytes so cursor -> 10_100.
        let mut w = BorrowWindow::new(10_000, 20_000);
        let _ = w.bump(100, 1);
        w
    }

    fn empty() -> SegmentRegistry {
        SegmentRegistry::new()
    }

    #[test]
    fn a_null_pointer_is_real_heap() {
        let reg = empty();
        let q = empty();
        assert_eq!(classify_ptr(0, None, &reg, &q), PtrClass::RealHeap);
        let w = armed_window();
        assert_eq!(classify_ptr(0, Some(&w), &reg, &q), PtrClass::RealHeap);
    }

    #[test]
    fn an_unrelated_pointer_is_real_heap() {
        let reg = empty();
        let q = empty();
        let w = armed_window();
        assert_eq!(classify_ptr(0x5000, Some(&w), &reg, &q), PtrClass::RealHeap);
        assert_eq!(classify_ptr(0x5000, None, &reg, &q), PtrClass::RealHeap);
    }

    #[test]
    fn a_pointer_in_the_armed_window_slot_is_window_slot() {
        let reg = empty();
        let q = empty();
        let w = armed_window();
        // Anywhere in the SLOT bounds [10_000, 20_000), not merely the bumped
        // region — a free of any slot byte must be kept away from glibc.
        assert_eq!(
            classify_ptr(10_000, Some(&w), &reg, &q),
            PtrClass::WindowSlot
        );
        assert_eq!(
            classify_ptr(15_000, Some(&w), &reg, &q),
            PtrClass::WindowSlot
        );
        assert_eq!(
            classify_ptr(19_999, Some(&w), &reg, &q),
            PtrClass::WindowSlot
        );
        assert_eq!(
            classify_ptr(20_000, Some(&w), &reg, &q),
            PtrClass::RealHeap,
            "tail_limit is not in the slot (half-open)"
        );
    }

    #[test]
    fn a_registered_segment_pointer_routes_to_release_with_its_cookie() {
        let mut reg = empty();
        let q = empty();
        reg.register(30_000, 4096, 0xF00D).expect("register");
        assert_eq!(
            classify_ptr(30_000, None, &reg, &q),
            PtrClass::Segment { cookie: 0xF00D }
        );
        assert_eq!(
            classify_ptr(30_000 + 4095, None, &reg, &q),
            PtrClass::Segment { cookie: 0xF00D }
        );
        assert_eq!(
            classify_ptr(30_000 + 4096, None, &reg, &q),
            PtrClass::RealHeap,
            "the byte past a segment is real heap"
        );
    }

    #[test]
    fn a_quarantined_pointer_is_a_no_op_after_the_window_and_registry_miss() {
        let reg = empty();
        let mut q = empty();
        // A slot extent quarantined at ARM time (whole [base, tail_limit)).
        q.register(10_000, 100, 0).expect("quarantine");
        assert_eq!(classify_ptr(10_000, None, &reg, &q), PtrClass::Quarantine);
        assert_eq!(classify_ptr(10_050, None, &reg, &q), PtrClass::Quarantine);
        assert_eq!(
            classify_ptr(10_100, None, &reg, &q),
            PtrClass::RealHeap,
            "the byte past a quarantine extent is real heap"
        );
    }

    #[test]
    fn a_registered_segment_wins_over_a_quarantine_covering_the_same_range() {
        // A slot the consumer registered for release outranks a stale quarantine
        // entry for the same bytes.
        let mut reg = empty();
        let mut q = empty();
        reg.register(10_000, 100, 0xBEEF).expect("register");
        q.register(10_000, 100, 0).expect("quarantine");
        assert_eq!(
            classify_ptr(10_050, None, &reg, &q),
            PtrClass::Segment { cookie: 0xBEEF }
        );
    }

    #[test]
    fn the_window_wins_over_a_registry_that_also_covers_the_slot() {
        // A degenerate overlap (the rmw registered the slot while armed): window
        // semantics dominate while armed.
        let mut reg = empty();
        let q = empty();
        reg.register(10_000, 10_000, 0xBEEF).expect("register slot");
        let w = armed_window();
        assert_eq!(
            classify_ptr(12_000, Some(&w), &reg, &q),
            PtrClass::WindowSlot,
            "an armed window is checked before the registry"
        );
        // With no window armed, the same address routes to the segment.
        assert_eq!(
            classify_ptr(12_000, None, &reg, &q),
            PtrClass::Segment { cookie: 0xBEEF }
        );
    }

    #[test]
    fn without_an_armed_window_slot_addresses_fall_through_to_registry_or_heap() {
        let reg = empty();
        let q = empty();
        // No window, no registration → real heap even for slot-looking addresses.
        assert_eq!(classify_ptr(10_000, None, &reg, &q), PtrClass::RealHeap);
    }

    // ── interval_ptr_action: the pure decision table ─────────

    /// A `tables` closure that must NOT be consulted — reading the global
    /// tables outside `ArmedLocksHeld` is exactly the race the phase split
    /// forbids, so consulting it here is a red assert, not just a wrong
    /// answer.
    fn tables_unreadable() -> impl FnOnce() -> IntervalTableVerdict {
        || panic!("the global tables were read outside ArmedLocksHeld")
    }

    #[test]
    fn outside_the_interval_no_action_applies() {
        assert_eq!(
            interval_ptr_action(AtforkIntervalPhase::Outside, false, tables_unreadable()),
            None
        );
        // Even a window hit is not the interval's business outside it (the
        // ordinary classify_ptr path owns that case).
        assert_eq!(
            interval_ptr_action(AtforkIntervalPhase::Outside, true, tables_unreadable()),
            None
        );
    }

    // THE SAFE-ARM PIN: a pointer freed while the gate is armed but the
    // locks are NOT held — the signal-window shape between publishing the gate
    // and acquiring the locks (and its mirror after release) — must take the
    // conservative leak-and-count arm. `None` here is the CORRUPT direction
    // (the caller falls through to glibc free of a possibly-SHM address), and
    // a table read here is the RACE direction (the closure panics).
    // `ArmedLocksNotHeld => None` (a fall-through to glibc) goes red on the
    // first assert; `ArmedLocksNotHeld` routed through `tables()` panics.
    #[test]
    fn armed_before_the_locks_are_held_takes_the_safe_leak_arm_without_a_table_read() {
        assert_eq!(
            interval_ptr_action(
                AtforkIntervalPhase::ArmedLocksNotHeld,
                false,
                tables_unreadable()
            ),
            Some(IntervalPtrAction::LeakUnclassified),
            "the widened boundary must leak-and-count, never fall through to glibc"
        );
    }

    #[test]
    fn a_window_hit_is_a_noop_in_both_armed_phases_without_a_table_read() {
        // The window is thread-local — safe to consult in every armed phase,
        // and it wins before any table question is asked.
        for phase in [
            AtforkIntervalPhase::ArmedLocksNotHeld,
            AtforkIntervalPhase::ArmedLocksHeld,
        ] {
            assert_eq!(
                interval_ptr_action(phase, true, tables_unreadable()),
                Some(IntervalPtrAction::NoopWindow),
                "phase {phase:?}"
            );
        }
    }

    #[test]
    fn armed_with_locks_held_classifies_precisely_from_the_tables() {
        for (verdict, want) in [
            (
                IntervalTableVerdict::Registered,
                IntervalPtrAction::LeakRegistered,
            ),
            (
                IntervalTableVerdict::QuarantineLive,
                IntervalPtrAction::NoopQuarantine,
            ),
            (
                IntervalTableVerdict::QuarantineTombstoned,
                IntervalPtrAction::NoopTombstone,
            ),
            (IntervalTableVerdict::NotFound, IntervalPtrAction::Unmanaged),
        ] {
            assert_eq!(
                interval_ptr_action(AtforkIntervalPhase::ArmedLocksHeld, false, || verdict),
                Some(want),
                "verdict {verdict:?}"
            );
        }
    }
}
