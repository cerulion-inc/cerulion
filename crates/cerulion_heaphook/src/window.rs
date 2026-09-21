// SPDX-License-Identifier: AGPL-3.0-only
//! The thread-local borrow WINDOW — a pure `#[cfg]`-free bump arena over a loan
//! slot's tail, with a per-window allocation ledger and EXACT escape detection.
//!
//! # The mechanism
//!
//! The rmw borrow consumer arms a window over `[base, tail_limit)` — the tail of a loan
//! slot it obtained through the stock `borrow_loaned_message` contract — on the
//! thread that will fill the message. While armed, the interposed `malloc`
//! family bump-allocates the message's `std::vector`/`std::string` storage into
//! the slot instead of the private heap, so the fill lands zero-copy in shared
//! memory. At publish the rmw reads the ledger back, range-tests each field's
//! data pointer against the window, and writes offset-table entries pointing at
//! the in-place bytes.
//!
//! # Why every outcome is sound
//!
//! Nothing is inferred from timing, size, or call stacks. The window is an API
//! event (arm/disarm). Adoption is an EXACT half-open address test. Every
//! escape is detected exactly and recorded for the caller to read — there is no
//! outcome in which a subscriber view is silently corrupted:
//!
//! * **Growth past the tail** — a bump that does not fit `[base, tail_limit)` is
//!   refused ([`BumpOutcome::Overflow`]); the interposer falls back to the real
//!   allocator (so the program never faults) and the window latches
//!   [`EscapeKind::GrowthPastTail`], so the rmw copies loudly at publish.
//! * **Reallocation** — a `realloc` of a window pointer means a vector grew past
//!   what the single bump gave it; the window latches [`EscapeKind::Reallocated`]
//!   and the interposer moves the data to the real heap → escape → copy.
//! * **Wrong thread** — the window is thread-local, so a fill that resizes on
//!   another thread never bumps into it; that field's storage lands on the real
//!   heap, and at publish the rmw's [`BorrowWindow::range_adopted`] test fails
//!   for it EXACTLY (its pointer is not in the slot) → copy. No thread-id
//!   comparison is needed — the address test is the detection.
//! * **Foreign allocator** — if some other interposer won `malloc`, our bump
//!   never runs; the ledger stays empty and the range test fails at publish.
//!
//! The first escape latches (sticky, first-wins) exactly like the repo's flood
//! latches — a later benign event cannot un-escape a window.

/// Why a window can no longer be adopted in place. Sticky: the first one latches
/// and later events never clear it. (Cross-thread fills need no variant here —
/// they are caught EXACTLY by [`BorrowWindow::range_adopted`] failing at
/// publish, since the storage is not in the slot.)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EscapeKind {
    /// A bump allocation did not fit the slot tail — the fill wanted more bytes
    /// than the loan reserved. The interposer served it from the real heap.
    GrowthPastTail,
    /// A window pointer was `realloc`'d — a vector grew past its single bump.
    Reallocated,
}

impl EscapeKind {
    /// A stable non-zero code for the FFI (`0` means "no escape").
    pub fn as_code(self) -> i32 {
        match self {
            EscapeKind::GrowthPastTail => 1,
            EscapeKind::Reallocated => 2,
        }
    }
}

/// The outcome of a bump request while a window is armed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BumpOutcome {
    /// The allocation fits the slot; use this in-slot address.
    InSlot(usize),
    /// The allocation does not fit `[base, tail_limit)`; the interposer must
    /// fall back to the real allocator. The window has latched
    /// [`EscapeKind::GrowthPastTail`].
    Overflow,
}

/// One recorded in-slot allocation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LedgerEntry {
    /// The in-slot address handed back.
    pub addr: usize,
    /// The requested size in bytes.
    pub size: usize,
}

/// A thread-local bump arena over a loan slot's tail.
#[derive(Debug)]
pub struct BorrowWindow {
    base: usize,
    tail_limit: usize,
    cursor: usize,
    escape: Option<EscapeKind>,
    ledger: Vec<LedgerEntry>,
}

impl BorrowWindow {
    /// Arm a window over `[base, tail_limit)`.
    ///
    /// A degenerate window with `tail_limit <= base` is a zero-capacity arena:
    /// every bump overflows (and escapes), which is the sound behaviour if the
    /// rmw reserved nothing.
    pub fn new(base: usize, tail_limit: usize) -> Self {
        Self {
            base,
            tail_limit: tail_limit.max(base),
            cursor: base,
            escape: None,
            ledger: Vec::new(),
        }
    }

    /// The window's slot bounds.
    #[inline]
    pub fn base(&self) -> usize {
        self.base
    }
    /// The exclusive tail bound.
    #[inline]
    pub fn tail_limit(&self) -> usize {
        self.tail_limit
    }
    /// The next free in-slot address.
    #[inline]
    pub fn cursor(&self) -> usize {
        self.cursor
    }
    /// Bytes bump-allocated so far (`cursor - base`).
    #[inline]
    pub fn bytes_used(&self) -> usize {
        self.cursor - self.base
    }
    /// The latched escape, if any.
    #[inline]
    pub fn escape(&self) -> Option<EscapeKind> {
        self.escape
    }
    /// The per-window allocation ledger, in bump order.
    #[inline]
    pub fn ledger(&self) -> &[LedgerEntry] {
        &self.ledger
    }

    /// The requested size of the bump that returned `addr`, if any. Used by
    /// `malloc_usable_size` to report the EXACT allocation size for a window
    /// pointer (never larger — a larger answer would let the caller overwrite
    /// the slot past its allocation). The most recent matching bump wins (a
    /// freed-then-rebumped address reads its live size).
    #[inline]
    pub fn ledger_size(&self, addr: usize) -> Option<usize> {
        self.ledger
            .iter()
            .rev()
            .find(|e| e.addr == addr)
            .map(|e| e.size)
    }

    /// Latch an escape (sticky — first-wins).
    fn set_escape(&mut self, kind: EscapeKind) {
        if self.escape.is_none() {
            self.escape = Some(kind);
        }
    }

    /// Bump-allocate `size` bytes at alignment `align` into the slot tail.
    ///
    /// On success advances the cursor, records the allocation, and returns the
    /// in-slot address. On overflow latches [`EscapeKind::GrowthPastTail`] and
    /// returns [`BumpOutcome::Overflow`] — the interposer then serves the bytes
    /// from the real allocator, so the fill still succeeds (just off-slot).
    ///
    /// A `size == 0` request still reserves ONE byte of cursor so two
    /// zero-size allocations get DISTINCT addresses (C requires it) and a
    /// zero-size loop cannot grow the ledger unbounded — but the recorded
    /// ledger size stays the true `0`, so [`Self::ledger_size`] never
    /// over-reports.
    pub fn bump(&mut self, size: usize, align: usize) -> BumpOutcome {
        let aligned = match align_up(self.cursor, align) {
            Some(a) => a,
            None => {
                self.set_escape(EscapeKind::GrowthPastTail);
                return BumpOutcome::Overflow;
            }
        };
        // Reserve at least one byte of cursor so distinct allocations get
        // distinct addresses; the recorded size is still the true request.
        let reserve = size.max(1);
        let end = match aligned.checked_add(reserve) {
            Some(e) => e,
            None => {
                self.set_escape(EscapeKind::GrowthPastTail);
                return BumpOutcome::Overflow;
            }
        };
        if end > self.tail_limit {
            self.set_escape(EscapeKind::GrowthPastTail);
            return BumpOutcome::Overflow;
        }
        self.cursor = end;
        self.ledger.push(LedgerEntry {
            addr: aligned,
            size,
        });
        BumpOutcome::InSlot(aligned)
    }

    /// Whether `addr` lies anywhere in the window's slot bounds
    /// `[base, tail_limit)` — the classification `free`/`realloc` use to route a
    /// window pointer away from the real allocator. Half-open on both ends
    /// (off-by-one mutation target).
    #[inline]
    pub fn owns(&self, addr: usize) -> bool {
        addr >= self.base && addr < self.tail_limit
    }

    /// Whether the whole buffer `[ptr, ptr + len)` is inside the BUMP-ALLOCATED
    /// region `[base, cursor)` — the exact test the rmw runs at publish to
    /// decide a field's storage really landed in the slot (adoption). `len == 0`
    /// is accepted at any in-slot `ptr` (an empty vector is trivially in place);
    /// a `ptr + len` that wraps `usize` is rejected.
    #[inline]
    pub fn range_adopted(&self, ptr: usize, len: usize) -> bool {
        if ptr < self.base {
            return false;
        }
        match ptr.checked_add(len) {
            Some(end) => end <= self.cursor,
            None => false,
        }
    }

    /// Record that a window pointer was handed to `realloc`: a vector grew past
    /// its single bump. Latches [`EscapeKind::Reallocated`]. The interposer then
    /// serves the new size from the real allocator and copies the old bytes, so
    /// the data leaves the slot and the rmw copies at publish.
    pub fn note_window_realloc(&mut self) {
        self.set_escape(EscapeKind::Reallocated);
    }
}

/// Round `addr` up to a multiple of `align`, overflow-checked. Works for any
/// non-power-of-two `align` too (`posix_memalign`/`aligned_alloc` guarantee a
/// power of two, but the checked ceil form is sound regardless). `align == 0`
/// is treated as `1`.
#[inline]
pub fn align_up(addr: usize, align: usize) -> Option<usize> {
    let align = align.max(1);
    let rem = addr % align;
    if rem == 0 {
        Some(addr)
    } else {
        addr.checked_add(align - rem)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── align_up ─────────────────────────────────────────────────────────

    #[test]
    fn align_up_rounds_and_guards_overflow() {
        assert_eq!(align_up(0, 8), Some(0));
        assert_eq!(align_up(1, 8), Some(8));
        assert_eq!(align_up(8, 8), Some(8));
        assert_eq!(align_up(9, 8), Some(16));
        assert_eq!(align_up(100, 1), Some(100), "align 1 is identity");
        assert_eq!(align_up(100, 0), Some(100), "align 0 treated as 1");
        // A non-power-of-two align still ceils correctly.
        assert_eq!(align_up(10, 6), Some(12));
        // Near the top, rounding up overflows → None (never a wrap).
        assert_eq!(align_up(usize::MAX, 8), None);
    }

    // ── bump: the fit / overflow boundary ────────────────────────────────

    #[test]
    fn bump_serves_in_slot_and_advances_the_cursor() {
        let mut w = BorrowWindow::new(1000, 1000 + 256);
        assert_eq!(w.bump(64, 8), BumpOutcome::InSlot(1000));
        assert_eq!(w.cursor(), 1064);
        assert_eq!(w.bytes_used(), 64);
        // The next bump aligns then advances.
        assert_eq!(w.bump(10, 16), BumpOutcome::InSlot(1072));
        assert_eq!(w.cursor(), 1082);
        assert_eq!(
            w.ledger(),
            &[
                LedgerEntry {
                    addr: 1000,
                    size: 64
                },
                LedgerEntry {
                    addr: 1072,
                    size: 10
                },
            ]
        );
        assert_eq!(w.escape(), None, "in-slot bumps never escape");
    }

    #[test]
    fn a_bump_that_exactly_fills_the_tail_is_in_slot() {
        let mut w = BorrowWindow::new(0, 128);
        assert_eq!(
            w.bump(128, 1),
            BumpOutcome::InSlot(0),
            "end == tail_limit fits"
        );
        assert_eq!(w.cursor(), 128);
        assert_eq!(w.escape(), None);
    }

    #[test]
    fn a_bump_one_byte_past_the_tail_overflows_and_latches() {
        let mut w = BorrowWindow::new(0, 128);
        assert_eq!(w.bump(129, 1), BumpOutcome::Overflow, "129 > 128 overflows");
        assert_eq!(
            w.escape(),
            Some(EscapeKind::GrowthPastTail),
            "an overflow latches GrowthPastTail"
        );
        assert!(w.ledger().is_empty(), "an overflowing bump records nothing");
        assert_eq!(w.cursor(), 0, "an overflowing bump does not advance");
    }

    #[test]
    fn a_second_bump_can_overflow_after_a_first_succeeds() {
        let mut w = BorrowWindow::new(0, 100);
        assert_eq!(w.bump(80, 1), BumpOutcome::InSlot(0));
        assert_eq!(w.bump(40, 1), BumpOutcome::Overflow, "80 + 40 > 100");
        assert_eq!(w.escape(), Some(EscapeKind::GrowthPastTail));
    }

    #[test]
    fn alignment_padding_can_push_a_fitting_size_over_the_tail() {
        // size alone fits, but aligning the cursor first does not.
        let mut w = BorrowWindow::new(0, 64);
        assert_eq!(w.bump(60, 1), BumpOutcome::InSlot(0)); // cursor -> 60
        assert_eq!(
            w.bump(1, 8),
            BumpOutcome::Overflow,
            "align 60->64 then +1 = 65 > 64"
        );
        assert_eq!(w.escape(), Some(EscapeKind::GrowthPastTail));
    }

    #[test]
    fn a_zero_capacity_window_overflows_every_bump() {
        let mut w = BorrowWindow::new(500, 500); // tail == base
        assert_eq!(w.tail_limit(), 500);
        assert_eq!(w.bump(1, 1), BumpOutcome::Overflow);
        assert_eq!(w.escape(), Some(EscapeKind::GrowthPastTail));
    }

    #[test]
    fn a_degenerate_tail_below_base_is_clamped_to_zero_capacity() {
        let w = BorrowWindow::new(1000, 900);
        assert_eq!(w.tail_limit(), 1000, "tail below base clamps to base");
    }

    #[test]
    fn two_zero_size_bumps_get_distinct_addresses_but_record_size_zero() {
        let mut w = BorrowWindow::new(0, 64);
        // Each zero-size bump reserves one byte of cursor so the addresses
        // differ (C requires distinct pointers), while the recorded size stays 0
        // so `ledger_size` cannot over-report.
        assert_eq!(w.bump(0, 1), BumpOutcome::InSlot(0));
        assert_eq!(w.cursor(), 1, "a zero-size bump still reserves one byte");
        assert_eq!(w.bump(0, 1), BumpOutcome::InSlot(1), "distinct address");
        assert_eq!(
            w.ledger(),
            &[
                LedgerEntry { addr: 0, size: 0 },
                LedgerEntry { addr: 1, size: 0 },
            ]
        );
        assert_eq!(w.ledger_size(0), Some(0), "size recorded as the true 0");
        assert_eq!(w.ledger_size(1), Some(0));
    }

    // ── owns / range_adopted: the address tests ──────────────────────────

    #[test]
    fn owns_is_half_open_over_the_slot_bounds() {
        let w = BorrowWindow::new(1000, 1100); // [1000, 1100)
        assert!(!w.owns(999));
        assert!(w.owns(1000), "base is owned");
        assert!(w.owns(1099), "last byte is owned");
        assert!(!w.owns(1100), "tail_limit is NOT owned (half-open)");
    }

    // The ZERO-CAPACITY window (`tail_limit == base`) — the
    // rmw's minimum-slice-ceiling shape, deliberately admitted by its
    // `windowed_borrow_geometry` ("a zero-capacity window is sound — every
    // fill escapes ⇒ copies") and accepted by the
    // `arm_window` export. Sound precisely because it can never mint an
    // in-slot pointer: `bump` reserves at least ONE byte (the
    // distinct-address rule), so at zero capacity EVERY request — a
    // zero-size one included — overflows to the real allocator with
    // GrowthPastTail latched; `owns` covers nothing; only the empty buffer
    // at `base` is trivially adopted. Weakening the reserve to the
    // bare request size lets `bump(0, 1)` mint `InSlot(base)` — an
    // "in-slot" pointer OUTSIDE the empty window that a later free would
    // hand to glibc — and fails the second bump assert.
    #[test]
    fn a_zero_capacity_window_owns_nothing_and_every_bump_overflows() {
        let mut w = BorrowWindow::new(1000, 1000); // [1000, 1000) — empty
        assert!(!w.owns(999));
        assert!(
            !w.owns(1000),
            "half-open: even base is outside [base, base)"
        );
        assert_eq!(w.bump(16, 8), BumpOutcome::Overflow);
        assert_eq!(
            w.bump(0, 1),
            BumpOutcome::Overflow,
            "the at-least-one-byte reserve forces even a zero-size request off-slot"
        );
        assert_eq!(w.escape(), Some(EscapeKind::GrowthPastTail));
        assert_eq!(w.cursor(), 1000, "no bump ever advanced the cursor");
        assert!(w.ledger().is_empty(), "nothing was ledgered");
        // The zero-length adopt test at base still answers true (an empty
        // forgeable member is trivially in place — the consumer's cursor
        // bisection recovers cursor == base from exactly this); any nonzero
        // length is not adopted.
        assert!(w.range_adopted(1000, 0));
        assert!(!w.range_adopted(1000, 1));
    }

    #[test]
    fn range_adopted_requires_the_whole_buffer_inside_the_bumped_region() {
        let mut w = BorrowWindow::new(1000, 2000);
        assert_eq!(w.bump(100, 1), BumpOutcome::InSlot(1000)); // cursor -> 1100
        assert!(w.range_adopted(1000, 100), "exactly the bump is adopted");
        assert!(
            w.range_adopted(1000, 0),
            "an empty buffer at base is adopted"
        );
        assert!(
            w.range_adopted(1050, 50),
            "a sub-range inside the bump is adopted"
        );
        assert!(
            !w.range_adopted(1000, 101),
            "one byte past the cursor is NOT adopted"
        );
        assert!(
            !w.range_adopted(1100, 1),
            "storage starting at the cursor (not yet bumped) is NOT adopted"
        );
        assert!(
            !w.range_adopted(999, 10),
            "storage starting before base is NOT adopted"
        );
    }

    #[test]
    fn range_adopted_rejects_a_wrapping_length() {
        let mut w = BorrowWindow::new(1000, usize::MAX);
        let _ = w.bump(10, 1);
        assert!(
            !w.range_adopted(1000, usize::MAX),
            "ptr + len wraps → not adopted"
        );
    }

    // ── escapes: sticky, first-wins ──────────────────────────────────────

    #[test]
    fn realloc_of_a_window_pointer_latches_reallocated() {
        let mut w = BorrowWindow::new(0, 128);
        let _ = w.bump(32, 1);
        w.note_window_realloc();
        assert_eq!(w.escape(), Some(EscapeKind::Reallocated));
    }

    #[test]
    fn the_first_escape_wins_and_later_events_do_not_overwrite_it() {
        let mut w = BorrowWindow::new(0, 8);
        assert_eq!(w.bump(16, 1), BumpOutcome::Overflow); // GrowthPastTail latched
        w.note_window_realloc(); // must NOT overwrite
        assert_eq!(
            w.escape(),
            Some(EscapeKind::GrowthPastTail),
            "first escape is sticky"
        );
    }

    #[test]
    fn escape_codes_are_stable_and_nonzero() {
        assert_eq!(EscapeKind::GrowthPastTail.as_code(), 1);
        assert_eq!(EscapeKind::Reallocated.as_code(), 2);
    }

    #[test]
    fn ledger_size_reports_the_exact_bump_size_or_none() {
        let mut w = BorrowWindow::new(0, 256);
        let BumpOutcome::InSlot(a) = w.bump(40, 8) else {
            panic!("first bump in slot");
        };
        let BumpOutcome::InSlot(b) = w.bump(24, 8) else {
            panic!("second bump in slot");
        };
        assert_eq!(w.ledger_size(a), Some(40));
        assert_eq!(w.ledger_size(b), Some(24));
        assert_eq!(
            w.ledger_size(0xDEAD),
            None,
            "an unknown address has no size"
        );
    }

    #[test]
    fn ledger_size_reads_the_most_recent_bump_at_a_reused_address() {
        // A bump arena never reclaims, so a re-bump lands at a NEW address in
        // normal use; this pins the rev()-scan tie-break for the pathological
        // case of two ledger entries at one address (defensive).
        let mut w = BorrowWindow::new(0, 256);
        w.ledger.push(LedgerEntry { addr: 100, size: 8 });
        w.ledger.push(LedgerEntry {
            addr: 100,
            size: 32,
        });
        assert_eq!(w.ledger_size(100), Some(32), "the most recent entry wins");
    }
}
